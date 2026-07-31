/*
 * Dirt32 gateway radio bridge — Heltec WiFi LoRa 32 V4.
 *
 * Dumb radio ↔ serial bridge: NO keys, NO crypto.
 * Receives LoRa frames and forwards them to the Pi; transmits frames handed
 * to it (gateway-sealed ACKs).  All trust decisions happen on the Linux side.
 *
 * Serial protocol: COBS/CRC-16 binary framing (lora_link.h).
 * Replaces the old "RX <hex> <rssi> <snr>" text output that garbled on
 * USB-CDC packet boundaries.
 */
#include <Arduino.h>
#include <RadioLib.h>
#include <oled_raw.h>
#include <lora_link.h>

static Module mod(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
static SX1262 radio(&mod);
static bool   radioUp = false;

/* DIO1 interrupt flag — set by ISR, cleared after each packet is consumed.
 * Using interrupt-based detection prevents spurious reads from the
 * GET_RX_BUFFER_STATUS register, which is NOT cleared by readData on SX126x
 * and would cause the bridge to fire twice for every received packet.     */
static volatile bool rxDone = false;
static void ARDUINO_ISR_ATTR onRxDone() { rxDone = true; }

/* ---------- status screen (onboard OLED) ----------
 * The bridge is USB-powered on the Pi, so the screen stays on.
 * PRG button toggles it (some prefer it dark in the field).              */
static OledRaw  oled;
static bool     scrOn    = true;
static uint32_t rxCount  = 0, txCount = 0;
static float    lastRssi = 0, lastSnr = 0;
static uint32_t lastRxAt = 0, lastHostAt = 0;  /* millis(); 0 = never */
static float    cfgFreq  = 0;
static int      cfgSf    = 0, cfgNet = 0;

#ifndef PIN_BUTTON
#define PIN_BUTTON 0
#endif

static void scrRender()
{
    char line[32];
    uint32_t now = millis();
    oled.clear();

    snprintf(line, sizeof(line), "DIRT32 BRIDGE  %s",
             radioUp ? "RX-ON" : "NO RF");
    oled.text(0, 0, line);

    if (cfgFreq > 0)
        snprintf(line, sizeof(line), "NET %d  %.1fMHz SF%d", cfgNet, cfgFreq, cfgSf);
    else
        snprintf(line, sizeof(line), "AWAITING CFG (defaults)");
    oled.text(0, 1, line);

    snprintf(line, sizeof(line), "RX %lu   TX %lu",
             (unsigned long)rxCount, (unsigned long)txCount);
    oled.text(0, 3, line);

    if (rxCount > 0) {
        snprintf(line, sizeof(line), "LAST %.0fdBm %.1fdB", lastRssi, lastSnr);
        oled.text(0, 4, line);
        uint32_t ago = (now - lastRxAt) / 1000;
        snprintf(line, sizeof(line), "%lus AGO", (unsigned long)ago);
        oled.text(0, 5, line);
    } else {
        oled.text(0, 4, "NO FRAMES YET");
    }

    if (lastHostAt == 0)
        snprintf(line, sizeof(line), "HOST: NEVER SEEN");
    else if (now - lastHostAt < 15000)
        snprintf(line, sizeof(line), "HOST: OK");
    else
        snprintf(line, sizeof(line), "HOST: SILENT %lus",
                 (unsigned long)((now - lastHostAt) / 1000));
    oled.text(0, 7, line);

    oled.flush();
}

static void scrPoll()
{
    static bool     rawLast = false, btnLast = false;
    static uint32_t changedAt = 0, lastDraw = 0;
    bool     raw = (digitalRead(PIN_BUTTON) == LOW);
    uint32_t now = millis();
    if (raw != rawLast) { rawLast = raw; changedAt = now; }
    if ((now - changedAt) > 40 && raw != btnLast) {
        btnLast = raw;
        if (raw) {
            scrOn = !scrOn;
            if (scrOn) { if (oled.begin()) scrRender(); }
            else oled.sleep();
            return;
        }
    }
    if (scrOn && oled.ready() && now - lastDraw > 1000) {
        scrRender();
        lastDraw = now;
    }
}

static bool applyCfg(float f, int sf, float bw, int cr, int net, int pwr)
{
    uint8_t sync = (uint8_t)(0x12u ^ (uint8_t)net);
    if (sync == 0x34u) sync ^= 0x01u;
    int st = radio.begin(f, bw, sf, cr, sync, pwr, 8);
    if (st != RADIOLIB_ERR_NONE) return false;
    radio.setCRC(true);
    radio.explicitHeader();
    radio.setDio1Action(onRxDone);
    radio.startReceive();
    rxDone = false;   /* clear any IRQ that fired during radio.begin() */
    return true;
}

/* ---------- Pi→Bridge frame handler (called from ll_rx_feed) ----------- */
static LLRx llRx;

static void onPiFrame(uint8_t type, const uint8_t *payload, uint8_t len,
                      int16_t /*rssi_unused*/, int8_t /*snr_unused*/)
{
    lastHostAt = millis();   /* any valid frame = Pi is alive */

    /* ---- TYPE_CFG: apply new radio config ---- */
    if (type == LL_TYPE_CFG) {
        if (len != sizeof(LLCfg)) {
            ll_send(LL_TYPE_ERR);
            return;
        }
        LLCfg cfg;
        memcpy(&cfg, payload, sizeof(cfg));
        float freq_mhz = (float)cfg.freq_hz / 1e6f;
        float bw_khz   = (float)cfg.bw_hz   / 1e3f;
        radioUp = applyCfg(freq_mhz, cfg.sf, bw_khz, cfg.cr,
                           cfg.net_id, cfg.txpwr_dbm);
        if (radioUp) {
            cfgFreq = freq_mhz;
            cfgSf   = cfg.sf;
            cfgNet  = cfg.net_id;
            ll_send(LL_TYPE_RDY);   /* signal Pi that bridge is back in RX */
        } else {
            ll_send(LL_TYPE_ERR);
        }
        return;
    }

    /* ---- TYPE_PING: keepalive ---- */
    if (type == LL_TYPE_PING) {
        ll_send(LL_TYPE_OK);
        return;
    }

    /* ---- TYPE_TX: transmit a frame via radio (gateway-sealed ACK) ---- */
    if (type == LL_TYPE_TX) {
        if (!radioUp) {
            ll_send(LL_TYPE_ERR);
            return;
        }
        int st = radio.transmit((uint8_t *)payload, (size_t)len);
        /* Clear TX_DONE IRQ that fired during transmit so rxDone doesn't
         * trip on the next loop iteration with stale FIFO data.          */
        rxDone = false;
        radio.startReceive();
        if (st == RADIOLIB_ERR_NONE) { txCount++; ll_send(LL_TYPE_OK); }
        else                         { ll_send(LL_TYPE_ERR); }
        return;
    }

    /* Unknown type: ignore silently */
}

// ===========================================================================
void setup()
{
    Serial.begin(115200);
    /* Native USB-CDC: wait briefly for the host to enumerate. */
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 2000) delay(10);
    delay(300);

    ll_rx_init(&llRx);

    /* Default config so the bridge hears something before the daemon
       connects; the daemon always sends CFG on open.                     */
    radioUp = applyCfg(903.0f, 10, 125.0f, 5, 1, 10);
    ll_send(radioUp ? LL_TYPE_RDY : LL_TYPE_ERR);

    pinMode(PIN_BUTTON, INPUT_PULLUP);
    if (oled.begin()) scrRender();
}

void loop()
{
    /* Feed all available serial bytes into the lora_link RX state machine.
     * ll_rx_feed calls onPiFrame synchronously for every validated frame. */
    while (Serial.available()) {
        uint8_t b = (uint8_t)Serial.read();
        ll_rx_feed(&llRx, &b, 1, onPiFrame);
    }

    /* Radio RX — only enter when DIO1 actually fired (RX_DONE IRQ).
     * Polling getPacketLength() was unreliable: GET_RX_BUFFER_STATUS on
     * SX126x is not cleared after readData, causing every packet to fire
     * twice and contaminating the second read with stale FIFO bytes.     */
    if (radioUp && rxDone) {
        rxDone = false;
        int len = radio.getPacketLength();   /* fresh read right after IRQ */
        if (len > 0 && len <= 64) {
            uint8_t buf[64] = {};
            int st = radio.readData(buf, len);
            if (st == RADIOLIB_ERR_NONE) {
                float rssi = radio.getRSSI(), snr = radio.getSNR();
                /* Send as a single atomic COBS frame — no text splitting,
                 * no USB-CDC boundary issues.                             */
                ll_send(LL_TYPE_ALERT, buf, (uint8_t)len,
                        (int16_t)rssi, (int8_t)snr);
                rxCount++;
                lastRssi = rssi; lastSnr = snr; lastRxAt = millis();
            }
        }
        radio.startReceive();
    }

    scrPoll();
    delay(2);
}
