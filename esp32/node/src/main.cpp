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
#include "sps_proto.h"
#include "config.h"
#include "frontend/FrontEnd.h"
#include "frontend/Adxl355FrontEnd.h"
#include "frontend/GeophoneFrontEnd.h"
#include "detector/StaLta.h"
#include "radio/LoRaLink.h"
#include "display/DebugScreen.h"
#include "gps/GpsUart.h"
#include <time.h>
#include <Preferences.h>

/* ---------- Persistent (RTC / NVS) state ---------- */
RTC_DATA_ATTR static uint32_t rtc_seq = 0;          /* survives deep sleep */
RTC_DATA_ATTR static uint16_t rtc_reset_count = 0;
RTC_DATA_ATTR static bool     rtc_tamper = false;

static NodeConfig cfg;
static LoRaLink link_;
static FrontEnd *frontEnd = nullptr;
static StaLta detector;
static bool radioOk = false, sensorOk = false, selfTestOk = false;
static DebugScreen dbgScreen;

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
static void vePower(bool on) {
    pinMode(PIN_VE, OUTPUT);
    /* Vext (GPIO36) is ACTIVE-LOW on the Heltec V4 (confirmed against
       Meshtastic's V4 board support). Define VE_ACTIVE_HIGH to flip. */
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
    if (cfg.solar_sense_gpio < 0) return false;
    pinMode((uint8_t)cfg.solar_sense_gpio, INPUT);
    return digitalRead((uint8_t)cfg.solar_sense_gpio) == HIGH;
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
    sps_alert_t a = {
        .timestamp = nowUnix(GpsFix{}),
        .event_class = event_class,
        .confidence = confidence,
        .peak_amp = peak,
        .battery_mv = readBatteryMv(),
    };
    uint8_t frame[SPS_MAX_FRAME];
    uint32_t seq = nextSeq();
    if (seq == 0) return;   /* SEQ exhausted or NVS failure — never reuse a nonce */
    int n = sps_seal_alert(cfg.psk, cfg.net_id, cfg.node_id, seq, &a,
                           frame, sizeof(frame));
    if (n <= 0) { Serial.printf("[alert] seal err %d\n", n); return; }
    TxOutcome out = link_.sendReliable(frame, (size_t)n, seq);
    Serial.printf("[alert] seq=%lu class=%u conf=%u peak=%u -> %s\n",
                  (unsigned long)seq, event_class, confidence, peak,
                  out == TxOutcome::ACKED ? "ACKED" :
                  out == TxOutcome::SENT_NO_ACK ? "sent (no ack)" : "RADIO ERROR");
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
                      (selfTestOk ? SPS_HF_SELFTEST : 0) |
                      (rtc_tamper ? SPS_HF_TAMPER : 0) |
                      (onSolar() ? SPS_HF_ON_SOLAR : 0) |
                      (deployFlag ? SPS_HF_DEPLOY : 0);
    hb.noise_floor = detector.noiseFloor();
    hb.fw_version = SPS_FW_VERSION;
    hb.reset_count = rtc_reset_count;

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
        "  selftest             run front-end self test\n"
        "  seqreset             reset SEQ to 0 (ONLY after PSK rotation)\n"
        "  screen               show link-config page on the OLED\n"
        "                       PRG tap=deploy hb, hold=config, 8s=sleep\n"
        "  gpstest [secs]       stream raw NMEA from the L76K (default 30s)\n"
        "  gpsfix               acquire+print a parsed GPS fix\n"
        "  gpsdiag              low-level GPS wiring/power diagnostic\n"
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
    else if (cmd == "selftest") {
        selfTestOk = frontEnd && frontEnd->selfTest();
        Serial.println(selfTestOk ? "self-test PASS" : "self-test FAIL");
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
        /* Low-level wiring diagnostic — bypasses GpsUart entirely.
           Tries three EN states × 5 s each and reports raw byte counts + hex.
           Pins are the same as GpsUart.h; any mismatch shows up here. */
        const uint8_t RX_PIN = 38, TX_PIN = 39, EN_PIN = 34,
                      SBY_PIN = 40, RST_PIN = 42;
        Serial.printf("[gpsdiag] RX=GPIO%u  TX=GPIO%u  EN=GPIO%u\n",
                      RX_PIN, TX_PIN, EN_PIN);

        /* Helper lambda: open Serial1, drain for ms_window, close, return count */
        auto probe = [&](const char *label, bool enState, bool enFloat) -> uint32_t {
            /* Set EN + standby + reset */
            if (!enFloat) {
                pinMode(EN_PIN,  OUTPUT);
                digitalWrite(EN_PIN, enState ? HIGH : LOW);
            } else {
                pinMode(EN_PIN, INPUT);   /* floating — don't drive it */
            }
            pinMode(SBY_PIN, OUTPUT); digitalWrite(SBY_PIN, HIGH);
            pinMode(RST_PIN, OUTPUT); digitalWrite(RST_PIN, HIGH);
            delay(200);   /* rail settle */

            Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
            delay(50);

            uint32_t t0 = millis(), count = 0;
            uint8_t preview[32]; uint32_t pIdx = 0;
            while (millis() - t0 < 5000) {
                while (Serial2.available()) {
                    uint8_t b = (uint8_t)Serial2.read();
                    count++;
                    if (pIdx < sizeof(preview)) preview[pIdx++] = b;
                }
                delay(2);
            }
            Serial2.end();

            Serial.printf("[gpsdiag] %-22s  bytes=%-5lu  ", label, (unsigned long)count);
            if (count == 0) {
                Serial.print("(nothing)\n");
            } else {
                uint32_t show = pIdx < 16 ? pIdx : 16;
                for (uint32_t i = 0; i < show; i++) {
                    if (preview[i] >= 0x20 && preview[i] < 0x7F)
                        Serial.printf("%c", preview[i]);
                    else
                        Serial.printf("\\x%02X", preview[i]);
                }
                Serial.println();
            }
            return count;
        };

        /* Test EN=HIGH first — LOW cuts power and recovery takes >200 ms,
         * so always probe the expected-working state before the kill state. */
        Serial.println("[gpsdiag] Phase 1/3: EN=HIGH (Heltec V4 expected: HIGH=on)");
        uint32_t c1 = probe("EN=HIGH",  true,  false);
        Serial.println("[gpsdiag] Phase 2/3: EN floating");
        uint32_t c2 = probe("EN=float", false, /*float*/true);
        Serial.println("[gpsdiag] Phase 3/3: EN=LOW (should kill power)");
        uint32_t c3 = probe("EN=LOW",   false, false);

        Serial.println("[gpsdiag] ---");
        if (c1 == 0 && c2 == 0 && c3 == 0) {
            Serial.println("[gpsdiag] RESULT: zero bytes in all phases — GPS module not present or GPIO38 unwired.");
            return;
        }
        if (c1 > 0)
            Serial.println("[gpsdiag] RESULT: EN=HIGH powers module (correct for Heltec V4).");
        else if (c2 > 0)
            Serial.println("[gpsdiag] RESULT: data only when EN floats — EN may be pulled HIGH externally.");
        else
            Serial.println("[gpsdiag] RESULT: EN=LOW powers module (unexpected — double-check wiring).");

        /* Phase 2: baud rate scan (EN=HIGH, 10 s each) — find the one that
         * gives printable ASCII so we know what to use in GpsUart */
        Serial.println("[gpsdiag] Baud scan (EN=HIGH, 10s each) — looking for printable NMEA...");
        const uint32_t bauds[] = { 4800, 9600, 19200, 38400, 57600, 115200 };
        for (uint32_t b : bauds) {
            pinMode(EN_PIN,  OUTPUT); digitalWrite(EN_PIN, HIGH);
            pinMode(SBY_PIN, OUTPUT); digitalWrite(SBY_PIN, HIGH);
            pinMode(RST_PIN, OUTPUT); digitalWrite(RST_PIN, HIGH);
            delay(1000);   /* L76K needs ~1 s after power-on before first NMEA */
            Serial2.begin(b, SERIAL_8N1, RX_PIN, TX_PIN);
            delay(50);
            uint32_t t0 = millis(), total = 0, printable = 0;
            uint8_t preview[48]; uint32_t pIdx = 0;
            while (millis() - t0 < 10000) {
                while (Serial2.available()) {
                    uint8_t byt = (uint8_t)Serial2.read();
                    total++;
                    if (byt >= 0x20 && byt < 0x7F) printable++;
                    if (pIdx < sizeof(preview)) preview[pIdx++] = byt;
                }
                delay(2);
            }
            Serial2.end();
            Serial.printf("[gpsdiag] %6lu baud: bytes=%-5lu  printable=%lu%%  ",
                          b, (unsigned long)total,
                          total ? (unsigned long)(printable * 100 / total) : 0UL);
            uint32_t show = pIdx < 24 ? pIdx : 24;
            for (uint32_t i = 0; i < show; i++) {
                if (preview[i] >= 0x20 && preview[i] < 0x7F)
                    Serial.printf("%c", preview[i]);
                else
                    Serial.printf("\\x%02X", preview[i]);
            }
            Serial.println();
        }
        Serial.println("[gpsdiag] done. The baud rate with ~100% printable and '$' in preview is correct.");
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
    vePower(true);
    SPI.begin();
    static Adxl355FrontEnd adxl(SPI, PIN_ADXL_CS, PIN_ADXL_INT1);
    static GeophoneFrontEnd geo(SPI, PIN_ADS_CS, PIN_ADS_DRDY);
    frontEnd = (cfg.front_end == FE_ADXL355)
                   ? (FrontEnd *)&adxl : (FrontEnd *)&geo;
    Serial.println("[boot] sensor front-end init...");
    sensorOk = frontEnd->begin(cfg.sample_rate_hz);
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
    /* Cold boot: stay awake for provisioning/bench work.
       Geophone profile also lives here: continuous listen in loop(). */
    dbgScreen.begin(PIN_BUTTON);
    Serial.println("Bench mode — CLI active. `help` for commands.");
    Serial.println("Press PRG button (or `screen`) for the link-debug display.");
    if (cfg.front_end == FE_GEOPHONE)
        Serial.println("Geophone profile: continuous detection running.");
}

void loop() {
    static String lineBuf;
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            handleCli(lineBuf);
            lineBuf = "";
        } else lineBuf += c;
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
            break;
        }
        case BtnEvent::HOLD:
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

    /* Geophone continuous-listen profile */
    if (cfg.front_end == FE_GEOPHONE && sensorOk) {
        int16_t buf[64];
        size_t n = frontEnd->read(buf, 64);
        for (size_t i = 0; i < n; i++) {
            DetectionResult r = detector.update(buf[i]);
            if (r.triggered) sendAlert(r.event_class, r.confidence, r.peak_amp);
        }
    }
    delay(1);
}
