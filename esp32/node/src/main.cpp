/*
 * main.cpp — Dirt32 (distributed seismic perimeter sensor): node firmware.
 * Heltec WiFi LoRa 32 V4 (ESP32-S3 + SX1262), spec §5.
 *
 * Flow (spec §5.2):
 *   ADXL355 profile:  deep sleep -> INT1 motion wake -> Ve on -> sample+detect
 *                     -> ALERT (encrypted, retx+ACK) -> re-arm -> sleep.
 *   Geophone profile: continuous listen loop (no hardware wake).
 *   Heartbeat: RTC-timer wake on schedule -> GPS fix -> HEARTBEAT -> sleep.
 *
 * Serial CLI (115200): type `help`. Includes a `taptest` command so the full
 * encrypt->TX->ACK path can be bench-verified with two bare boards and no
 * sensor attached.
 */
#include <Arduino.h>
#include <SPI.h>
#include <driver/gpio.h>   /* gpio_get_level — used by gpsraw */
#include <esp_idf_version.h>
#include <soc/gpio_struct.h>  /* GPIO matrix registers — sensorid pin dump */
#include <soc/gpio_reg.h>     /* GPIO_ENABLE_REG/GPIO_ENABLE1_REG */
#include "sps_proto.h"
#include "config.h"
#include "frontend/FrontEnd.h"
#include "frontend/Adxl355FrontEnd.h"
#include "frontend/GeophoneFrontEnd.h"
#include "detector/StaLta.h"
#include "radio/LoRaLink.h"
#include "display/DebugScreen.h"
#include "gps/GpsUart.h"
#include "csi/CsiSensor.h"
#include <time.h>
#include <Preferences.h>

/* ---------- Persistent (RTC / NVS) state ---------- */
RTC_DATA_ATTR static uint32_t rtc_seq = 0;          /* survives deep sleep */
RTC_DATA_ATTR static uint16_t rtc_reset_count = 0;
RTC_DATA_ATTR static bool     rtc_tamper = false;
/* Self-test result persists across deep-sleep wakes: run automatically at
 * cold boot, re-run any time via the `selftest` CLI command. Without this,
 * armed nodes heartbeat "self-test failed" forever and sit yellow on the
 * gateway map. */
RTC_DATA_ATTR static bool     rtc_selftest_ok = false;

static NodeConfig cfg;
static LoRaLink link_;
static FrontEnd *frontEnd = nullptr;
static StaLta detector;
static bool radioOk = false, sensorOk = false;
static DebugScreen dbgScreen;
#if SPS_CSI_ENABLE
static CsiSensor csi;
static bool csiOk = false;
#endif

/* ---------- SEQ management ----------
 * SEQ is the replay key and nonce base — it must NEVER repeat under the same
 * key, across ANY reset cause (brownout, WDT, power cycle). Scheme:
 *   - NVS holds `seq_hw`, an upper bound on every SEQ ever used.
 *   - Before a SEQ >= seq_hw is issued, seq_hw is advanced by SEQ_CKPT and
 *     written to NVS FIRST. If the write fails, the frame is not sent.
 *   - On every boot, rtc_seq is fast-forwarded to seq_hw (RTC memory may be
 *     stale or lost; NVS bound is authoritative).
 *   - At SPS_SEQ_MAX the node halts TX: 24-bit SEQ must never wrap under a
 *     given key. Rotate the PSK (`keygen` + gateway re-registration) and
 *     `seqreset` to continue.
 */
static const uint32_t SEQ_CKPT = 64;
static uint32_t seq_reserved = 0;   /* RAM copy of NVS seq_hw */

static bool seqReserve(uint32_t upto) {
    Preferences p;
    if (!p.begin("sps", false)) return false;
    bool ok = p.putULong("seq_hw", upto) > 0;
    p.end();
    if (ok) seq_reserved = upto;
    return ok;
}

/* Returns next SEQ, or 0 on failure (exhausted or NVS write failed).
   0 is never a valid SEQ because boot always fast-forwards past seq_hw>=1. */
static uint32_t nextSeq() {
    if (rtc_seq >= SPS_SEQ_MAX) {
        Serial.println("[seq] EXHAUSTED — rotate PSK (keygen) then `seqreset`. TX halted.");
        return 0;
    }
    uint32_t candidate = rtc_seq + 1;
    if (candidate >= seq_reserved) {
        if (!seqReserve(candidate + SEQ_CKPT)) {
            Serial.println("[seq] NVS reserve FAILED — refusing to send (nonce safety)");
            return 0;
        }
    }
    rtc_seq = candidate;
    return rtc_seq;
}

/* Called on EVERY boot, regardless of reset cause. */
static void seqRestore() {
    Preferences p;
    uint32_t hw = 0;
    if (p.begin("sps", true)) { hw = p.getULong("seq_hw", 0); p.end(); }
    if (rtc_seq < hw) rtc_seq = hw;   /* NVS bound is authoritative */
    seq_reserved = hw;                /* force a fresh reservation on first send */
    if (hw == 0) seqReserve(SEQ_CKPT); /* first boot: reserve before any SEQ use */
}

/* ---------- Ve sensor power rail (spec §5.2 — verify polarity!) ---------- */
static SPIClass sensorSPI(HSPI);

static void vePower(bool on);

/* Cycle the sensor Ve rail with all bus lines driven LOW (so nothing
 * back-feeds the unpowered chip through its protection diodes), then
 * re-attach the SPI bus. Recovers an ADXL355 latched at measurement entry. */
static void sensorRailCycle(uint32_t off_ms) {
    sensorSPI.end();
    pinMode(PIN_SENS_SCK, OUTPUT);  digitalWrite(PIN_SENS_SCK, LOW);
    pinMode(PIN_SENS_MOSI, OUTPUT); digitalWrite(PIN_SENS_MOSI, LOW);
    pinMode(PIN_ADXL_CS, OUTPUT);   digitalWrite(PIN_ADXL_CS, LOW);
    pinMode(PIN_ADS_CS, OUTPUT);    digitalWrite(PIN_ADS_CS, LOW);
    pinMode(PIN_SENS_MISO, INPUT);  /* no pullup */
    vePower(false);
    delay(off_ms);
    vePower(true);
    delay(50);
    sensorSPI.begin(PIN_SENS_SCK, PIN_SENS_MISO, PIN_SENS_MOSI, /*SS*/-1);
    digitalWrite(PIN_ADXL_CS, HIGH);
    digitalWrite(PIN_ADS_CS, HIGH);
}

static void vePower(bool on) {
    pinMode(PIN_VE, OUTPUT);
    /* Vext (GPIO36) is ACTIVE LOW — verified empirically on this board:
       HIGH kills the sensor from the first ID read, LOW powers it. */
#ifdef VE_ACTIVE_HIGH
    digitalWrite(PIN_VE, on ? HIGH : LOW);
#else
    digitalWrite(PIN_VE, on ? LOW : HIGH);
#endif
    if (on) delay(15);
}

/* ---------- Battery ---------- */
static uint16_t readBatteryMv() {
    /* Heltec V4 battery divider ~ 4.9:1 via ADC ctrl; adjust factor after
       measuring a known cell voltage. */
    uint32_t raw = analogReadMilliVolts(PIN_VBAT);
    return (uint16_t)(raw * 49 / 10);
}

/* ---------- Solar sense (optional) ----------
 * Charging is pure hardware: the V4's charge-management IC charges the
 * battery whenever 5 V is present (USB or solar) — nothing to enable here.
 * This only *reports* it: if solar_sense_gpio is set, a divided-down panel
 * rail is read and the ON_SOLAR health flag is set in heartbeats. */
static bool onSolar() {
    int8_t p = cfg.solar_sense_gpio;
    if (p < 0) return false;
    /* Runtime guard for stale saved configs: never touch a pin owned by
     * the sensor bus, radio, GPS, or power rails — pinMode(INPUT) here
     * silently killed the sensor SPI when old configs had gpio 48. */
    if (p == PIN_SENS_SCK || p == PIN_SENS_MISO || p == PIN_SENS_MOSI ||
        p == PIN_ADXL_CS || p == PIN_ADXL_INT1 ||
        p == PIN_ADS_CS  || p == PIN_ADS_DRDY ||
        p == LORA_NSS || p == LORA_DIO1 || p == LORA_RST || p == LORA_BUSY ||
        p == PIN_GPS_RX || p == PIN_GPS_TX || p == PIN_GPS_EN || p == PIN_GPS_RESET ||
        p == PIN_VE || p == PIN_VBAT || p == PIN_BUTTON) {
        return false;
    }
    pinMode((uint8_t)p, INPUT);
    return digitalRead((uint8_t)p) == HIGH;
}

/* ---------- GPS (Heltec L76K GNSS plug-in module) ---------- */
struct GpsFix { bool valid = false; int32_t lat_e7 = 0, lon_e7 = 0; uint32_t unix_time = 0; };
static GpsUart gps;

static GpsFix acquireGps(uint16_t timeout_s) {
    gps.powerOn();
    GpsFixResult r = gps.acquire(timeout_s);
    gps.powerOff();   /* module fully unpowered between fixes */
    GpsFix f;
    f.valid = r.valid;
    f.lat_e7 = r.lat_e7;
    f.lon_e7 = r.lon_e7;
    f.unix_time = r.unix_time;
    if (r.valid)
        Serial.printf("[gps] fix: %.7f, %.7f sats=%u hdop=%.1f\n",
                      r.lat_e7 / 1e7, r.lon_e7 / 1e7, r.sats, r.hdop_x10 / 10.0);
    return f;
}

static uint32_t nowUnix(const GpsFix &fix) {
    if (fix.valid && fix.unix_time) return fix.unix_time;
    /* System clock is synced to UTC on any prior GPS fix (settimeofday). */
    time_t t = time(nullptr);
    if (t > 1700000000) return (uint32_t)t;
    /* Fallback: relative time; gateway stamps receipt time (spec §9). */
    return (uint32_t)(millis() / 1000);
}

/* ---------- TX paths ---------- */
static void sendAlert(uint8_t event_class, uint8_t confidence, uint16_t peak) {
#if SPS_CSI_ENABLE
    /* Coexistence: hold our ESP-NOW pings while the LoRa alert burst +
     * ACK window is in flight so both transmitters never key up together. */
    csi.pauseTraffic(true);
#endif
    sps_alert_t a = {
        .timestamp = nowUnix(GpsFix{}),
        .event_class = event_class,
        .confidence = confidence,
        .peak_amp = peak,
        .battery_mv = readBatteryMv(),
    };
    uint8_t frame[SPS_MAX_FRAME];
    uint32_t seq = nextSeq();
    if (seq == 0) goto out;   /* SEQ exhausted or NVS failure — never reuse a nonce */
    {
        int n = sps_seal_alert(cfg.psk, cfg.net_id, cfg.node_id, seq, &a,
                               frame, sizeof(frame));
        if (n <= 0) { Serial.printf("[alert] seal err %d\n", n); goto out; }
        TxOutcome outc = link_.sendReliable(frame, (size_t)n, seq);
        Serial.printf("[alert] seq=%lu class=%u conf=%u peak=%u -> %s\n",
                      (unsigned long)seq, event_class, confidence, peak,
                      outc == TxOutcome::ACKED ? "ACKED" :
                      outc == TxOutcome::SENT_NO_ACK ? "sent (no ack)" : "RADIO ERROR");
    }
out:
#if SPS_CSI_ENABLE
    csi.pauseTraffic(false);
#else
    ;
#endif
}

static void sendHeartbeat(bool deployFlag = false) {
    GpsFix fix = cfg.gps_enable ? acquireGps(cfg.gps_fix_timeout_s) : GpsFix{};
    sps_heartbeat_t hb = {};
    hb.timestamp = nowUnix(fix);
    hb.battery_mv = readBatteryMv();
    hb.lat_e7 = fix.valid ? fix.lat_e7 : cfg.fallback_lat_e7;
    hb.lon_e7 = fix.valid ? fix.lon_e7 : cfg.fallback_lon_e7;
    hb.health_flags = (sensorOk ? SPS_HF_SENSOR_OK : 0) |
                      (fix.valid ? SPS_HF_GPS_FIX : 0) |
                      (rtc_selftest_ok ? SPS_HF_SELFTEST : 0) |
                      (rtc_tamper ? SPS_HF_TAMPER : 0) |
                      (onSolar() ? SPS_HF_ON_SOLAR : 0) |
                      (deployFlag ? SPS_HF_DEPLOY : 0);
    hb.noise_floor = detector.noiseFloor();
    hb.fw_version = SPS_FW_VERSION;
    hb.reset_count = rtc_reset_count;
#if SPS_CSI_ENABLE
    if (csiOk && csi.running()) {
        hb.health_flags |= SPS_HF_CSI_ON;
        if (csi.calibrating()) hb.health_flags |= SPS_HF_CSI_CALIB;
        hb.csi_noise = csi.noiseX100();
    }
#endif

    uint8_t frame[SPS_MAX_FRAME];
    uint32_t seq = nextSeq();
    if (seq == 0) return;   /* SEQ exhausted or NVS failure */
    int n = sps_seal_heartbeat(cfg.psk, cfg.net_id, cfg.node_id, seq, &hb,
                               frame, sizeof(frame));
    if (n <= 0) { Serial.printf("[hb] seal err %d\n", n); return; }
    /* Heartbeats are single-shot TX — no retransmit burst, no ACK window. */
    bool ok = link_.sendOnce(frame, (size_t)n);
    rtc_tamper = false;
    Serial.printf("[hb] seq=%lu batt=%umV%s lat=%.6f lon=%.6f gps=%s -> %s\n",
                  (unsigned long)seq, hb.battery_mv,
                  deployFlag ? " DEPLOY" : "",
                  hb.lat_e7 / 1e7, hb.lon_e7 / 1e7,
                  (hb.health_flags & SPS_HF_GPS_FIX) ? "FIX" : "fallback",
                  ok ? "sent" : "RADIO ERROR");
}

/* ---------- Sampling / detection loop ---------- */
static void runDetection(uint32_t max_ms) {
    int16_t buf[64];
    uint32_t t0 = millis();
    while (millis() - t0 < max_ms) {
        size_t n = frontEnd ? frontEnd->read(buf, 64) : 0;
        for (size_t i = 0; i < n; i++) {
            DetectionResult r = detector.update(buf[i]);
            if (r.triggered) {
                Serial.printf("[detect] ratio=%.1f class=%u\n", detector.ratio(), r.event_class);
                sendAlert(r.event_class, r.confidence, r.peak_amp);
                t0 = millis(); /* extend listen window after an event */
            }
        }
        if (n == 0) delay(2);
    }
}

/* ---------- Sleep ---------- */
static void goToSleep() {
    uint64_t hb_interval_us =
        86400ULL * 1000000ULL / (cfg.heartbeat_per_day ? cfg.heartbeat_per_day : 1);
    esp_sleep_enable_timer_wakeup(hb_interval_us);

    if (cfg.front_end == FE_ADXL355 && cfg.motion_wake_enable && frontEnd && sensorOk) {
        int wakePin = frontEnd->armMotionWake(cfg.motion_threshold_g);
        if (wakePin >= 0)
            esp_sleep_enable_ext0_wakeup((gpio_num_t)wakePin, 1);
        /* Ve stays ON: the accel's activity engine must run in sleep.
           (ADXL355 low-power mode ~ tens of µA.) */
    } else {
        if (frontEnd) frontEnd->powerDown();
        vePower(false);
    }
    Serial.println("[sleep] entering deep sleep");
    Serial.flush();
    esp_deep_sleep_start();
}

/* ---------- CLI ---------- */
static void printHelp() {
    Serial.println(
        "Commands:\n"
        "  show                 print config (PSK redacted)\n"
        "  set <key> <value>    set a parameter (see spec §5.3 keys)\n"
        "  save                 persist config to flash\n"
        "  keygen               generate a random PSK (prints hex ONCE — record it)\n"
        "  taptest [class]      send a synthetic ALERT (default class=footstep)\n"
        "  hb                   send a heartbeat now\n"
        "  detector <seconds>   stream STA/LTA ratio for tuning\n"
        "  csi [seconds]        WiFi-radar status; with seconds, stream the CSI metric for tuning\n"
        "  selftest             run front-end self test\n"
        "  seqreset             reset SEQ to 0 (ONLY after PSK rotation)\n"
        "  screen               show link-config page on the OLED\n"
        "                       PRG tap=deploy hb, hold=config, 8s=sleep\n"
        "  gpstest [secs]       stream raw NMEA from the L76K (default 30s)\n"
        "  gpsfix               acquire+print a parsed GPS fix\n"
        "  gpsdiag              low-level GPS wiring/power diagnostic\n"
        "  gpsraw               bare-minimum factory-style UART dump (no powerOff first)\n"
        "  sensorid [secs]      probe sensor chip ID every 1s (default 30) — catches a bus dying after boot\n"
        "  adxlprobe [secs]     hold ADXL in standby (countdown, default 15s), then enter measurement — for DMM on 1.8V caps\n"
        "  sleep                enter deep sleep now\n"
        "  reboot");
}

static void handleCli(String line) {
    line.trim();
    if (line.isEmpty()) return;
    int sp1 = line.indexOf(' ');
    String cmd = sp1 < 0 ? line : line.substring(0, sp1);
    String rest = sp1 < 0 ? "" : line.substring(sp1 + 1);

    if (cmd == "help") printHelp();
    else if (cmd == "show") configPrint(cfg, Serial);
    else if (cmd == "set") {
        int sp2 = rest.indexOf(' ');
        if (sp2 < 0) { Serial.println("usage: set <key> <value>"); return; }
        String k = rest.substring(0, sp2), v = rest.substring(sp2 + 1);
        Serial.println(configSet(cfg, k, v) ? "ok (unsaved — run `save`)" : "bad key/value");
    }
    else if (cmd == "save") Serial.println(configSave(cfg) ? "saved" : "save FAILED");
    else if (cmd == "keygen") {
        for (size_t i = 0; i < SPS_KEY_LEN; i++) cfg.psk[i] = (uint8_t)esp_random();
        Serial.print("psk ");
        for (size_t i = 0; i < SPS_KEY_LEN; i++) Serial.printf("%02x", cfg.psk[i]);
        /* Ready-to-paste entry for the Pi's /etc/dirt32/keys.json */
        Serial.printf("\n\nkeys.json entry (paste inside the { }):\n  \"%u\": \"",
                      cfg.node_id);
        for (size_t i = 0; i < SPS_KEY_LEN; i++) Serial.printf("%02x", cfg.psk[i]);
        Serial.println("\"");
        Serial.println("Register at the gateway, then `save` here.");
    }
    else if (cmd == "taptest") {
        uint8_t cls = rest.isEmpty() ? SPS_EV_FOOTSTEP : (uint8_t)rest.toInt();
        Serial.println("[taptest] sending synthetic ALERT");
        sendAlert(cls, 200, 1234);
    }
    else if (cmd == "hb") sendHeartbeat();
    else if (cmd == "detector") {
        uint32_t secs = rest.isEmpty() ? 10 : (uint32_t)rest.toInt();
        Serial.printf("[detector] streaming ratio for %lus (trigger at %.1f)\n",
                      (unsigned long)secs, cfg.trigger_ratio);
        int16_t buf[64];
        uint32_t t0 = millis(), lastPrint = 0;
        while (millis() - t0 < secs * 1000) {
            size_t n = frontEnd ? frontEnd->read(buf, 64) : 0;
            for (size_t i = 0; i < n; i++) detector.update(buf[i]);
            if (millis() - lastPrint > 250) {
                Serial.printf("ratio=%.2f noise=%u\n", detector.ratio(), detector.noiseFloor());
                lastPrint = millis();
            }
            if (n == 0) delay(2);
        }
    }
    else if (cmd == "csi") {
#if SPS_CSI_ENABLE
        if (!cfg.csi_enable) {
            Serial.println("[csi] disabled — `set csi_enable 1` then `save` + `reboot`");
            return;
        }
        if (!csiOk) { Serial.println("[csi] init failed at boot"); return; }
        Serial.printf("[csi] running=%d calibrating=%d frames=%lu noise=%u (x100) "
                      "threshold=%.1f\n",
                      (int)csi.running(), (int)csi.calibrating(),
                      (unsigned long)csi.frameCount(), csi.noiseX100(),
                      cfg.csi_threshold);
        uint32_t secs = rest.isEmpty() ? 0 : (uint32_t)rest.toInt();
        if (secs > 0) {
            Serial.printf("[csi] streaming metric for %lus (trigger at %.1f)\n",
                          (unsigned long)secs, cfg.csi_threshold);
            uint32_t t0 = millis(), lastPrint = 0;
            while (millis() - t0 < secs * 1000) {
                csi.service();
                csi.poll();
                if (millis() - lastPrint > 250) {
                    Serial.printf("metric=%.2f frames=%lu%s\n", csi.metric(),
                                  (unsigned long)csi.frameCount(),
                                  csi.calibrating() ? " (calibrating)" : "");
                    lastPrint = millis();
                }
                delay(5);
            }
        }
#else
        Serial.println("[csi] not in this build (SPS_CSI_ENABLE=0)");
#endif
    }
    else if (cmd == "adxlprobe") {
        uint32_t secs = rest.length() ? (uint32_t)rest.toInt() : 15;
        if (secs < 3) secs = 3;
        if (cfg.front_end == FE_ADXL355 && frontEnd) {
            /* Boot init has usually already latched the chip — revive it
             * with a rail cycle before probing. */
            Serial.println("[adxlprobe] cycling Ve rail first (2s off) to revive a latched chip...");
            sensorRailCycle(2000);
            ((Adxl355FrontEnd *)frontEnd)->measureEntryProbe(secs);
        } else {
            Serial.println("[adxlprobe] ADXL355 front-end not active");
        }
    }
    else if (cmd == "sensorid") {
        /* Watch for a sensor bus that dies some time after boot: probe the
         * chip ID once a second and timestamp when (if) it stops answering. */
        uint32_t secs = rest.isEmpty() ? 30 : (uint32_t)rest.toInt();
        if (secs == 0) secs = 30;
        Serial.printf("[sensorid] probing every 1s for %lus (Ctrl+C not needed — it stops itself)\n",
                      (unsigned long)secs);
        if (!frontEnd) { Serial.println("no front-end"); return; }
        uint32_t t0 = millis();
        bool wasOk = true;
        while (millis() - t0 < secs * 1000) {
            bool ok = frontEnd->probe();
            if (!ok && wasOk) {
                /* First failure: bisect firmware vs hardware.
                 * Dump each sensor pin's GPIO-matrix routing straight from
                 * the registers (portable across core versions):
                 *   out_sel 256 = plain GPIO; other values = a peripheral
                 *   signal owns the pin. oe = output enable. */
                Serial.println("[sensorid] sensor-pin routing (out_sel 256 = plain GPIO):");
                static const int probePins[] = { PIN_SENS_SCK, PIN_SENS_MISO,
                                                 PIN_SENS_MOSI, PIN_ADXL_CS,
                                                 PIN_ADXL_INT1 };
                for (int p : probePins) {
                    uint32_t outSel = GPIO.func_out_sel_cfg[p].func_sel;
                    bool oe = (p < 32) ? ((REG_READ(GPIO_ENABLE_REG) >> p) & 1)
                                       : ((REG_READ(GPIO_ENABLE1_REG) >> (p - 32)) & 1);
                    Serial.printf("  GPIO%-2d level=%d out_sel=%lu oe=%d\n",
                                  p, gpio_get_level((gpio_num_t)p),
                                  (unsigned long)outSel, (int)oe);
                }
                Serial.println("[sensorid] attempting live driver re-init...");
                bool r = frontEnd->begin(cfg.sample_rate_hz);
                Serial.printf("[sensorid] re-init %s\n",
                              r ? "RECOVERED -> firmware is disturbing the bus/pins"
                                : "FAILED -> sensor lost power or a wire (hardware side)");
                ok = r;
            }
            wasOk = ok;
            delay(1000);
        }
        Serial.println("[sensorid] done");
    }
    else if (cmd == "selftest") {
        rtc_selftest_ok = frontEnd && frontEnd->selfTest();
        Serial.println(rtc_selftest_ok ? "self-test PASS" : "self-test FAIL");
    }
    else if (cmd == "seqreset") {
        /* ONLY after rotating the PSK — resets the nonce space. */
        rtc_seq = 0;
        seqReserve(SEQ_CKPT);
        Serial.println("SEQ reset. Only valid after a PSK rotation (keygen+save) "
                       "and re-registration at the gateway.");
    }
    else if (cmd == "screen") dbgScreen.showConfig(cfg, radioOk, rtc_seq);
    else if (cmd == "gpstest") {
        uint16_t secs = rest.isEmpty() ? 30 : (uint16_t)rest.toInt();
        if (secs == 0) secs = 30;
        Serial.printf("Powering L76K, streaming raw NMEA for %us "
                      "(look for $GxRMC with status 'A')...\n", secs);
        gps.powerOn();
        gps.passthrough(Serial, secs);
        gps.powerOff();
        Serial.println("\n[gps] done. `gpsfix` attempts a parsed fix.");
    }
    else if (cmd == "gpsfix") {
        Serial.printf("Acquiring fix (timeout %us)...\n", cfg.gps_fix_timeout_s);
        GpsFix f = acquireGps(cfg.gps_fix_timeout_s);
        if (!f.valid) Serial.println("[gps] NO FIX (needs sky view; cold start can take 30-60s)");
    }
    else if (cmd == "gpsdiag") {
        /* Targeted UART test only — no pin scanning (broad scanning corrupts
         * the IO matrix and poisons subsequent gpstest runs).
         * Heltec official V4 example method: EN(34)=LOW (active LOW),
         * RESET(42)=HIGH, Serial1 on RX=39/TX=38 at 9600.
         * No standby pin, no reset pulse.
         * Reboots at the end so the UART state is guaranteed clean. */
        const uint8_t EN_PIN = 34, RST_PIN = 42;

        Serial.println("[gpsdiag] Closing GpsUart...");
        gps.powerOff();
        delay(200);

        Serial.println("[gpsdiag] EN=LOW (VGNSS_Ctrl active LOW), RESET=HIGH, then 10 s read on RX=GPIO39 (Serial1)...");
        pinMode(EN_PIN,  OUTPUT); digitalWrite(EN_PIN, LOW);
        pinMode(RST_PIN, OUTPUT); digitalWrite(RST_PIN, HIGH);
        delay(1000);  /* L76K ~1 s to first NMEA */

        Serial1.begin(9600, SERIAL_8N1, /*RX*/39, /*TX*/38);  /* per Heltec V4 example */
        delay(50);

        uint32_t t0 = millis(), count = 0, printable = 0;
        uint8_t preview[64]; uint32_t pIdx = 0;
        while (millis() - t0 < 10000) {
            while (Serial1.available()) {
                uint8_t b = (uint8_t)Serial1.read();
                count++;
                if (b >= 0x20 && b < 0x7F) printable++;
                if (pIdx < sizeof(preview)) preview[pIdx++] = b;
            }
            delay(2);
        }
        Serial1.end();

        Serial.printf("[gpsdiag] bytes=%-5lu  printable=%lu%%\n",
                      (unsigned long)count,
                      count ? (unsigned long)(printable * 100 / count) : 0UL);
        if (pIdx > 0) {
            Serial.print("[gpsdiag] preview: ");
            uint32_t show = pIdx < 48 ? pIdx : 48;
            for (uint32_t i = 0; i < show; i++) {
                uint8_t b = preview[i];
                if (b >= 0x20 && b < 0x7F) Serial.printf("%c", b);
                else                        Serial.printf("\\x%02X", b);
            }
            Serial.println();
        }
        Serial.println("[gpsdiag] ---");
        if (count == 0) {
            Serial.println("[gpsdiag] RESULT: zero bytes on GPIO38.");
            Serial.println("  1. Flip the JST cable 180 deg and rerun gpsdiag.");
            Serial.println("  2. Measure the GPS module VCC pad with a multimeter (EN=LOW should give ~3.3 V).");
        } else if (count > 10 && printable * 100 / count > 80) {
            Serial.println("[gpsdiag] RESULT: NMEA flowing — run `gpstest 120` outdoors.");
        } else {
            Serial.println("[gpsdiag] RESULT: some bytes but not clean NMEA.");
            Serial.println("  Try flipping the JST cable 180 deg.");
        }
        Serial.println("[gpsdiag] Rebooting for clean UART state...");
        delay(500);
        ESP.restart();
    }
    else if (cmd == "gpsraw") {
        /* Low-level UART dump with SPI teardown and GPIO state logging.
         * Canonical pin map (Heltec official V4 example): RX=39, TX=38,
         * EN=34 ACTIVE LOW. Sub-tests run in order, stopping at first
         * SUSTAINED (>100 byte) success:
         *   A) canonical RX=39 — Serial1 (no SPI.end)
         *   B) after SPI.end   — Serial1 RX=39 (catches SPI pin clamping)
         *   C) after SPI.end   — Serial2 RX=39 (catches UART peripheral issue)
         *   D) swapped RX=38   — Serial2 (catches reversed TX/RX orientation)
         *   E) EN on GPIO42 LOW — Serial2 RX=39 (catches V4.3 R8 pin map)
         *   F) EN=34 HIGH       — Serial2 RX=39 (catches inverted-EN module) */
        uint16_t secs = rest.isEmpty() ? 10 : (uint16_t)rest.toInt();
        if (secs == 0) secs = 10;

        auto readGPIO38 = []() -> const char * {
            /* Read raw digital level without changing pin mode */
            return gpio_get_level((gpio_num_t)38) ? "HIGH" : "LOW";
        };
        auto drain = [&](HardwareSerial &ser) -> uint32_t {
            uint32_t t0 = millis(), count = 0;
            uint8_t preview[64]; uint32_t pIdx = 0;
            while (millis() - t0 < (uint32_t)secs * 1000) {
                while (ser.available()) {
                    uint8_t b = (uint8_t)ser.read();
                    count++;
                    if (pIdx < sizeof(preview)) preview[pIdx++] = b;
                }
                delay(2);
            }
            Serial.printf(" bytes=%lu  ", (unsigned long)count);
            if (pIdx == 0) {
                Serial.print("(nothing)");
            } else {
                uint32_t show = pIdx < 40 ? pIdx : 40;
                for (uint32_t i = 0; i < show; i++) {
                    uint8_t b = preview[i];
                    if (b >= 0x20 && b < 0x7F) Serial.printf("%c", b);
                    else                        Serial.printf("\\x%02X", b);
                }
            }
            Serial.println();
            return count;
        };

        /* EN=LOW once, held for all sub-tests — VGNSS_Ctrl is ACTIVE LOW
         * (Meshtastic heltec_v4: GPS_EN_ACTIVE LOW). */
        pinMode(34, OUTPUT); digitalWrite(34, LOW);
        delay(1500);   /* L76K cold boot after power-up */

        auto readGPIO39 = []() -> const char * {
            return gpio_get_level((gpio_num_t)39) ? "HIGH" : "LOW";
        };

        /* Sub-test A: canonical Serial1 RX=39/TX=38 */
        Serial.printf("[gpsraw-A] GPIO39=%s before open; Serial1 RX=39, no SPI.end()...",
                      readGPIO39());
        Serial1.begin(9600, SERIAL_8N1, 39, 38);
        uint32_t cA = drain(Serial1);
        Serial1.end();
        if (cA > 100) { Serial.println("[gpsraw] DONE — canonical pin map works (RX=39/TX=38, EN=34 LOW)."); return; }

        /* Sub-test B: SPI.end() then Serial1 */
        SPI.end();
        delay(10);
        Serial.printf("[gpsraw-B] GPIO39=%s after SPI.end(); Serial1 RX=39...", readGPIO39());
        Serial1.begin(9600, SERIAL_8N1, 39, 38);
        uint32_t cB = drain(Serial1);
        Serial1.end();
        if (cB > 100) { Serial.println("[gpsraw] DONE — SPI was clamping a GPS pin. Check SPI.begin() uses explicit pins (9,11,10)."); return; }

        /* Sub-test C: Serial2 (different UART peripheral) */
        Serial.printf("[gpsraw-C] GPIO39=%s; Serial2 RX=39...", readGPIO39());
        Serial2.begin(9600, SERIAL_8N1, 39, 38);
        uint32_t cC = drain(Serial2);
        Serial2.end();
        /* A handful of bytes is a pin-matrix glitch, not NMEA (9600 baud NMEA
         * is ~400+ bytes/s). Require sustained traffic before concluding. */
        if (cC > 100) { Serial.println("[gpsraw] DONE — sustained data on Serial2 but not Serial1 (unexpected; verify)."); return; }
        if (cC > 0)   Serial.printf("[gpsraw-C] %lu stray byte(s) — likely glitch, not NMEA. Continuing.\n", (unsigned long)cC);

        /* Sub-test D: reversed orientation — module TX on GPIO38 instead */
        Serial.printf("[gpsraw-D] reversed pins — Serial2 RX=38 TX=39...");
        Serial2.begin(9600, SERIAL_8N1, 38, 39);
        uint32_t cD = drain(Serial2);
        Serial2.end();
        if (cD > 100) { Serial.println("[gpsraw] DONE — TX/RX are REVERSED from canonical: module TX is on GPIO38. Set PIN_GPS_RX=38, PIN_GPS_TX=39."); return; }

        /* Sub-test E: V4.3 R8 revision — GNSS enable moves to GPIO42
         * (also ACTIVE LOW per Meshtastic heltec_v4_r8). */
        pinMode(42, OUTPUT); digitalWrite(42, LOW);
        delay(1500);   /* module cold boot after possible power cycling */
        Serial.printf("[gpsraw-E] R8 pin map — EN on GPIO42 held LOW, RX=39...");
        Serial2.begin(9600, SERIAL_8N1, 39, 38);
        uint32_t cE = drain(Serial2);
        Serial2.end();
        digitalWrite(42, HIGH);
        if (cE > 100) { Serial.println("[gpsraw] DONE — board behaves like V4.3 R8: GNSS EN is GPIO42 (active LOW). Treat 42 as EN, not RESET."); return; }

        /* Sub-test F: inverted EN — in case this third-party module really is
         * active HIGH (some generic modules invert Heltec's polarity). */
        digitalWrite(34, HIGH);
        delay(1500);
        Serial.printf("[gpsraw-F] EN=34 HIGH (inverted polarity), RX=39...");
        Serial2.begin(9600, SERIAL_8N1, 39, 38);
        uint32_t cF = drain(Serial2);
        Serial2.end();
        digitalWrite(34, LOW);
        if (cF > 100) { Serial.println("[gpsraw] DONE — this module's EN is ACTIVE HIGH. Flip the EN writes in GpsUart::powerOn/powerOff."); return; }

        Serial.println("[gpsraw] All sub-tests returned no sustained data.");
        Serial.printf("[gpsraw] Final levels: GPIO39=%s GPIO38=%s\n", readGPIO39(), readGPIO38());
        Serial.println("  RX pin LOW  → pin is being driven LOW by the board (peripheral clamp or short).");
        Serial.println("  RX pin HIGH → pin floating/pulled up; module TX may be disconnected (check cable seating).");
    }
    else if (cmd == "sleep") goToSleep();
    else if (cmd == "reboot") ESP.restart();
    else Serial.println("unknown command — try `help`");
}

/* ---------- Boot ---------- */
void setup() {
    Serial.begin(115200);
    /* Native USB-CDC: give the host a moment to enumerate so early boot
       prints aren't lost. Don't block forever — field nodes have no host. */
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 2000) delay(10);

    esp_reset_reason_t rr = esp_reset_reason();
    if (rr == ESP_RST_BROWNOUT || rr == ESP_RST_WDT || rr == ESP_RST_TASK_WDT)
        rtc_reset_count++;
    if (rr == ESP_RST_POWERON) rtc_reset_count = 0;
    seqRestore();   /* every boot — RTC memory may be stale after any reset */

    configLoad(cfg);

    esp_sleep_wakeup_cause_t wake = esp_sleep_get_wakeup_cause();
    bool motionWake = (wake == ESP_SLEEP_WAKEUP_EXT0);
    bool timerWake = (wake == ESP_SLEEP_WAKEUP_TIMER);
    bool coldBoot = !motionWake && !timerWake;

    if (coldBoot) {
        delay(1500); /* give USB serial a moment on the bench */
        Serial.println("\n=== Dirt32 node ===");
        Serial.printf("node_id=%u net_id=%u fw=%d reset_reason=%d\n",
                      cfg.node_id, cfg.net_id, SPS_FW_VERSION, (int)rr);
    }

    /* Radio first — it is needed on every path. */
    Serial.println("[boot] radio init...");
    radioOk = link_.begin(cfg);
    Serial.println(radioOk ? "[boot] radio OK" : "[radio] SX1262 init FAILED");

    /* Front-end */
    Serial.println("[boot] sensor rail + SPI...");
    /* Deselect BOTH sensor chip-selects before anything touches the shared
     * bus. Critical: PIN_ADS_CS is GPIO46, an ESP32-S3 strapping pin with a
     * default pull-DOWN — if left unconfigured while the ADXL front-end is
     * active, a connected ADS1220 is permanently selected: it clocks in
     * garbage from every ADXL transaction and can drive the shared MISO. */
    pinMode(PIN_ADXL_CS, OUTPUT); digitalWrite(PIN_ADXL_CS, HIGH);
    pinMode(PIN_ADS_CS, OUTPUT);  digitalWrite(PIN_ADS_CS, HIGH);
    vePower(true);
    /* Explicit pins prevent SPI from claiming GPIO38 (FSPIWP/GNSS_RX).
     * Without this, SPI.begin() drives GPIO38 LOW and UART RX never works. */
    SPI.begin(/*SCK*/9, /*MISO*/11, /*MOSI*/10, /*SS*/-1);
    /* Sensors get a DEDICATED SPI bus on header-exposed pins — on the V4,
     * the radio's SCK9/MOSI10/MISO11 nets are internal-only (no header pads),
     * so the old shared-bus wiring is physically impossible. */
    sensorSPI.begin(PIN_SENS_SCK, PIN_SENS_MISO, PIN_SENS_MOSI, /*SS*/-1);
    static Adxl355FrontEnd adxl(sensorSPI, PIN_ADXL_CS, PIN_ADXL_INT1, PIN_SENS_MISO);
    static GeophoneFrontEnd geo(sensorSPI, PIN_ADS_CS, PIN_ADS_DRDY);
    frontEnd = (cfg.front_end == FE_ADXL355)
                   ? (FrontEnd *)&adxl : (FrontEnd *)&geo;
    Serial.println("[boot] sensor front-end init...");
    sensorOk = frontEnd->begin(cfg.sample_rate_hz);
    if (!sensorOk) {
        /* Chip observed latching up on measurement-mode entry, dead until
         * real power removal. Ve is firmware-switched — cycle it and retry.
         * If this recovers the chip, its VDD is genuinely on the Ve rail and
         * the fault is a supply latch-up; if it does NOT recover but a USB
         * replug does, VDD is wired to always-on 3V3 instead of Ve. */
        Serial.println("[sensor] init FAILED — cycling Ve rail (2s off) and retrying...");
        frontEnd->powerDown();      /* best effort; chip may not hear it */
        sensorRailCycle(2000);      /* lines low + rail off = full drain */
        /* Retry the ENTIRE init dead-slow: if 100 kHz survives measurement
         * entry where 5 MHz latched the chip, the fault is signal integrity
         * (edge ringing on jumper wires injecting current past the protection
         * diodes), not the chip or the supply. */
        if (cfg.front_end == FE_ADXL355) {
            Serial.println("[sensor] retrying full init at 100 kHz SPI (signal-integrity test)...");
            Adxl355FrontEnd::setSpiHz(100000);
        }
        sensorOk = frontEnd->begin(cfg.sample_rate_hz);
        Serial.println(sensorOk
            ? "[sensor] RECOVERED after Ve cycle -> if clock is 100kHz, 5MHz edges were latching the chip"
            : "[sensor] still dead after Ve cycle -> VDD likely NOT on the Ve rail (check for 3V3 pin), or chip damaged");
    }
    if (!sensorOk) Serial.println("[sensor] front-end init FAILED (taptest still works)");
    Serial.println("[boot] detector init...");

    DetectorConfig dc = { cfg.sample_rate_hz, cfg.hpf_hz, cfg.sta_ms, cfg.lta_ms,
                          cfg.trigger_ratio, cfg.footstep_lo_hz, cfg.footstep_hi_hz,
                          cfg.vehicle_lo_hz, cfg.vehicle_hi_hz };
    detector.begin(dc);

    if (motionWake) {
        /* Detection path (spec §5.2): confirm + classify, alert, back to sleep. */
        rtc_tamper = true; /* motion since last heartbeat */
        if (sensorOk) runDetection(8000);
        goToSleep();
    }
    if (timerWake) {
        sendHeartbeat();
        goToSleep();
    }
    /* Cold boot: run the sensor self-test automatically so heartbeats carry
     * a truthful HF_SELFTEST bit without anyone typing `selftest`. The
     * result persists in RTC memory across all deep-sleep wakes. */
    if (sensorOk && frontEnd) {
        rtc_selftest_ok = frontEnd->selfTest();
        Serial.println(rtc_selftest_ok ? "[boot] self-test PASS"
                                       : "[boot] self-test FAIL");
    } else {
        rtc_selftest_ok = false;
    }

    /* Cold boot: stay awake for provisioning/bench work.
       Geophone profile also lives here: continuous listen in loop(). */
    dbgScreen.begin(PIN_BUTTON);
    Serial.println("Bench mode — CLI active. `help` for commands.");
    Serial.println("Press PRG button (or `screen`) for the link-debug display.");
    if (cfg.auto_arm_s > 0 && cfg.front_end == FE_ADXL355) {
        if (sensorOk && radioOk)
            Serial.printf("AUTO-ARM: entering armed sleep cycle in %us — any keystroke cancels (set auto_arm_s 0 to disable).\n",
                          (unsigned)cfg.auto_arm_s);
        else
            Serial.printf("AUTO-ARM: skipped — %s failed init; staying in bench mode.\n",
                          sensorOk ? "radio" : "sensor");
    }
    if (cfg.front_end == FE_GEOPHONE)
        Serial.println("Geophone profile: continuous detection running.");

#if SPS_CSI_ENABLE
    /* WiFi radar (CSI): only in continuous-listen operation — the WiFi
     * radio must stay in RX, so CSI is incompatible with the deep-sleep
     * ADXL profile (auto-arm is suppressed below when CSI is on). */
    if (cfg.csi_enable) {
        Serial.println("[csi] WiFi radar init (STA, not associated)...");
        csiOk = csi.begin(cfg);
        if (csiOk)
            Serial.printf("[csi] up: ch=%u role=%s ping=%uHz thresh=%.1f "
                          "window=%u calib=%us — node stays awake while CSI is on\n",
                          cfg.csi_wifi_channel,
                          cfg.csi_role == 0 ? "rx" : cfg.csi_role == 1 ? "tx" : "both",
                          cfg.csi_ping_hz, cfg.csi_threshold,
                          cfg.csi_window_frames, cfg.csi_calib_s);
        else
            Serial.println("[csi] init FAILED — WiFi/ESP-NOW error; seismic path unaffected");
    }
#endif
}

void loop() {
    /* Sensor liveness watchdog: for the first 30 s after boot, silently
     * probe the front-end every 250 ms and print ONLY when it changes
     * state — pinpoints the exact moment the bus dies without the user
     * having to type anything. */
    static uint32_t wdLast = 0;
    static bool wdState = true, wdDone = false;
    if (!wdDone && sensorOk && frontEnd && millis() - wdLast >= 250) {
        wdLast = millis();
        bool ok = frontEnd->alive();
        if (ok != wdState) {
            Serial.printf("[sensor-watchdog %8lums] sensor %s\n",
                          (unsigned long)millis(), ok ? "RECOVERED" : "DIED");
            wdState = ok;
        }
        if (millis() > 30000) {
            wdDone = true;
            Serial.printf("[sensor-watchdog] 30s done — sensor %s\n",
                          wdState ? "still alive" : "dead");
        }
    }

    static String lineBuf;
    static bool autoArmCancelled = false;
    /* Absolute arm deadline; cold-boot default from config, moved closer by
     * the deploy TAP. 0 = not scheduled. */
    static uint32_t armAtMs =
        (cfg.auto_arm_s > 0) ? (uint32_t)cfg.auto_arm_s * 1000UL : 0;
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (!autoArmCancelled && cfg.auto_arm_s > 0) {
            autoArmCancelled = true;
            Serial.println("[auto-arm] cancelled — bench mode until `sleep` or reboot.");
        }
        if (c == '\n' || c == '\r') {
            handleCli(lineBuf);
            lineBuf = "";
        } else lineBuf += c;
    }

    /* Deploy-and-forget: on a cold boot with healthy sensor+radio and an
     * untouched CLI, arm automatically after cfg.auto_arm_s seconds. Any
     * keystroke or button press cancels for this session.
     * CSI mode suppresses auto-arm: WiFi radar needs the radio awake. */
    bool csiHoldsAwake = false;
#if SPS_CSI_ENABLE
    csiHoldsAwake = cfg.csi_enable && csiOk;
#endif
    if (!autoArmCancelled && !csiHoldsAwake && armAtMs > 0 && sensorOk && radioOk &&
        cfg.front_end == FE_ADXL355) {
        static uint32_t lastCountdown = 0;
        uint32_t left = (millis() < armAtMs) ? armAtMs - millis() : 0;
        if (left == 0) {
            Serial.println("[auto-arm] arming now — motion-wake sleep cycle.");
            dbgScreen.showMessage("AUTO-ARM", "sleeping...", nullptr);
            delay(500);
            goToSleep();
        } else if (left <= 30000 && millis() - lastCountdown >= 10000) {
            lastCountdown = millis();
            Serial.printf("[auto-arm] arming in %lus — any keystroke cancels.\n",
                          (unsigned long)(left / 1000));
        }
    }

    /* PRG button — three tiers:
     *   TAP   (< 800 ms)  → GPS fix + deploy heartbeat; node appears on map
     *   HOLD  (800 ms–8s) → show link-config OLED page
     *   SLEEP (8 s held)  → deep sleep (soft power-down) */
    switch (dbgScreen.poll()) {
        case BtnEvent::TAP: {
            Serial.println("[deploy] TAP — sending deploy heartbeat...");
            dbgScreen.showMessage("DEPLOYING...", "GPS fix...", nullptr);
            sendHeartbeat(/*deployFlag=*/true);
            /* After TX, show config page so user can verify params on screen */
            dbgScreen.showConfig(cfg, radioOk, rtc_seq);
            /* Deploy means DEPLOY: (re)schedule arming shortly after the
             * heartbeat, even if a keystroke had cancelled auto-arm. Gives
             * the user 30 s to set the node down and step away. */
            if (sensorOk && radioOk && cfg.front_end == FE_ADXL355) {
                autoArmCancelled = false;
                armAtMs = millis() + 30000UL;
                Serial.println("[deploy] arming in 30s — any keystroke cancels.");
            } else {
                Serial.println("[deploy] NOT arming (sensor/radio init failed or geophone profile).");
            }
            break;
        }
        case BtnEvent::HOLD:
            autoArmCancelled = true;   /* user is interacting — hold bench */
            dbgScreen.showConfig(cfg, radioOk, rtc_seq);
            break;
        case BtnEvent::SLEEP:
            Serial.println("[btn] 8s hold — entering deep sleep.");
            dbgScreen.showMessage("SLEEPING...", nullptr, nullptr);
            delay(800);   /* let the user read it */
            goToSleep();
            break;
        case BtnEvent::NONE:
        default:
            dbgScreen.refreshIfVisible(cfg, radioOk, rtc_seq);
            break;
    }

    /* Continuous-listen seismic detection:
     *   - geophone profile (always continuous by design), and
     *   - ADXL355 nodes held awake by CSI mode — they never enter the armed
     *     deep-sleep cycle, so without this branch they'd have no seismic
     *     detection at all. */
    bool seismicListen = sensorOk &&
        (cfg.front_end == FE_GEOPHONE || (csiHoldsAwake && cfg.front_end == FE_ADXL355));
    if (seismicListen) {
        int16_t buf[64];
        size_t n = frontEnd->read(buf, 64);
        for (size_t i = 0; i < n; i++) {
            DetectionResult r = detector.update(buf[i]);
            if (r.triggered) sendAlert(r.event_class, r.confidence, r.peak_amp);
        }
    }

#if SPS_CSI_ENABLE
    /* WiFi radar: ping traffic + detector. Coexists with seismic detection
     * above — a person can raise both a seismic and a wifi_presence alert. */
    if (csiOk) {
        csi.service();
        static bool wasCalib = true;
        bool calib = csi.calibrating();
        if (wasCalib && !calib)
            Serial.printf("[csi] calibration done — baseline noise=%u (x100)\n",
                          csi.noiseX100());
        wasCalib = calib;
        CsiEvent e = csi.poll();
        if (e.triggered) {
            Serial.printf("[csi] MOTION metric=%.2f conf=%u\n",
                          e.metric_x100 / 100.0f, e.confidence);
            sendAlert(SPS_EV_WIFI_PRESENCE, e.confidence, e.metric_x100);
        }
    }
#endif
    delay(1);
}
