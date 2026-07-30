/*
 * Dirt32 gateway radio bridge.
 *
 * Line protocol on USB serial (115200, \n-terminated):
 *   Bridge -> Pi:
 *     RDY                          on boot / after CFG
 *     RX <hex> <rssi_dbm> <snr_db> received frame (raw, unverified)
 *     OK <what> / ERR <detail>     command results
 *   Pi -> Bridge:
 *     CFG <freq_mhz> <sf> <bw_khz> <cr> <net_id> <txpower_dbm>
 *     TX <hex>                     transmit a frame (gateway-sealed ACK)
 *     PING                         liveness -> OK PING
 *
 * The sync word is derived from net_id exactly like the node firmware:
 * 0x12 ^ net_id, avoiding 0x34 (public LoRaWAN).
 */
#include <Arduino.h>
#include <RadioLib.h>
#include <oled_raw.h>

static Module mod(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
static SX1262 radio(&mod);
static bool radioUp = false;

/* DIO1 interrupt flag — set by ISR, cleared after each packet is consumed.
 * Using interrupt-based detection prevents spurious reads from the
 * GET_RX_BUFFER_STATUS register, which is NOT cleared by readData on SX126x
 * and would cause the bridge to fire twice for every received packet.     */
static volatile bool rxDone = false;
static void ARDUINO_ISR_ATTR onRxDone() { rxDone = true; }

/* ---------- status screen (onboard OLED) ----------
 * The bridge is USB-powered on the Pi, so the screen stays on.
 * PRG button toggles it (some prefer it dark in the field).      */
static OledRaw oled;
static bool     scrOn = true;
static uint32_t rxCount = 0, txCount = 0;
static float    lastRssi = 0, lastSnr = 0;
static uint32_t lastRxAt = 0, lastHostAt = 0;   /* millis(); 0 = never */
static float    cfgFreq = 0; static int cfgSf = 0, cfgNet = 0;

#ifndef PIN_BUTTON
#define PIN_BUTTON 0
#endif

static void scrRender() {
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

    /* host = Pi daemon; it PINGs/CFGs over USB serial */
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

static void scrPoll() {
    static bool rawLast = false, btnLast = false;
    static uint32_t changedAt = 0, lastDraw = 0;
    bool raw = (digitalRead(PIN_BUTTON) == LOW);
    uint32_t now = millis();
    if (raw != rawLast) { rawLast = raw; changedAt = now; }
    if ((now - changedAt) > 40 && raw != btnLast) {
        btnLast = raw;
        if (raw) {                       /* press toggles the screen */
            scrOn = !scrOn;
            if (scrOn) { if (oled.begin()) scrRender(); }
            else oled.sleep();
            return;                      /* start clean next tick */
        }
    }
    if (scrOn && oled.ready() && now - lastDraw > 1000) {
        scrRender();
        lastDraw = now;
    }
}

static int hexVal(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static bool applyCfg(float f, int sf, float bw, int cr, int net, int pwr) {
    uint8_t sync = (uint8_t)(0x12 ^ (uint8_t)net);
    if (sync == 0x34) sync ^= 0x01;
    int st = radio.begin(f, bw, sf, cr, sync, pwr, 8);
    if (st != RADIOLIB_ERR_NONE) return false;
    radio.setCRC(true);
    radio.explicitHeader();
    radio.setDio1Action(onRxDone);
    radio.startReceive();
    rxDone = false;   /* clear any IRQ that fired during radio.begin() */
    return true;
}

static void handleLine(String &line) {
    line.trim();
    if (line.length() == 0) return;

    lastHostAt = millis();               /* any line = the Pi is alive */

    if (line == "PING") { Serial.println("OK PING"); return; }

    if (line.startsWith("CFG ")) {
        float f, bw; int sf, cr, net, pwr;
        if (sscanf(line.c_str() + 4, "%f %d %f %d %d %d",
                   &f, &sf, &bw, &cr, &net, &pwr) != 6) {
            Serial.println("ERR CFG parse");
            return;
        }
        radioUp = applyCfg(f, sf, bw, cr, net, pwr);
        if (radioUp) { cfgFreq = f; cfgSf = sf; cfgNet = net; }
        Serial.println(radioUp ? "OK CFG" : "ERR CFG radio");
        if (radioUp) Serial.println("RDY");
        return;
    }

    if (line.startsWith("TX ")) {
        if (!radioUp) { Serial.println("ERR TX no-cfg"); return; }
        uint8_t buf[64];
        size_t n = 0;
        const char *p = line.c_str() + 3;
        while (p[0] && p[1] && n < sizeof(buf)) {
            int hi = hexVal(p[0]), lo = hexVal(p[1]);
            if (hi < 0 || lo < 0) { Serial.println("ERR TX hex"); return; }
            buf[n++] = (uint8_t)((hi << 4) | lo);
            p += 2;
        }
        int st = radio.transmit(buf, n);
        /* Clear the TX_DONE IRQ that fired during transmit — without this,
         * rxDone would be true on the next loop tick and we'd try to read
         * a "packet" that doesn't exist, causing getPacketLength to return
         * stale FIFO data.                                               */
        rxDone = false;
        radio.startReceive();               /* back to listening immediately */
        if (st == RADIOLIB_ERR_NONE) txCount++;
        Serial.println(st == RADIOLIB_ERR_NONE ? "OK TX" : "ERR TX radio");
        return;
    }

    Serial.println("ERR unknown");
}

void setup() {
    Serial.begin(115200);
    /* Native USB-CDC: wait briefly for the host to enumerate. */
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 2000) delay(10);
    delay(300);
    /* Default config so the bridge hears something before the daemon
       connects; the daemon always sends CFG on open. */
    radioUp = applyCfg(903.0f, 10, 125.0f, 5, 1, 10);
    Serial.println(radioUp ? "RDY" : "ERR boot radio");

    pinMode(PIN_BUTTON, INPUT_PULLUP);
    if (oled.begin()) scrRender();       /* USB-powered: screen on by default */
}

void loop() {
    static String lineBuf;
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (lineBuf.length()) handleLine(lineBuf);
            lineBuf = "";
        } else if (lineBuf.length() < 200) lineBuf += c;
    }

    /* Only enter when DIO1 actually fired (RX_DONE IRQ).
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
                char hex[132];
                memset(hex, 0, sizeof(hex));
                for (int i = 0; i < len; i++)
                    sprintf(hex + i * 2, "%02x", buf[i]);
                /* Build the full line first, then write in one call.
                 * Multiple Serial.print() calls on USB CDC can be split
                 * across USB packets; if the Pi reads between them it
                 * accumulates a partial line that corrupts the next one. */
                char rxLine[160];
                snprintf(rxLine, sizeof(rxLine), "RX %s %d %d\n",
                         hex, (int)rssi, (int)(snr * 10));
                Serial.print(rxLine);
                rxCount++; lastRssi = rssi; lastSnr = snr; lastRxAt = millis();
            }
        }
        radio.startReceive();
    }

    scrPoll();
    delay(2);
}
