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

static Module mod(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
static SX1262 radio(&mod);
static bool radioUp = false;

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
    radio.startReceive();
    return true;
}

static void handleLine(String &line) {
    line.trim();
    if (line.length() == 0) return;

    if (line == "PING") { Serial.println("OK PING"); return; }

    if (line.startsWith("CFG ")) {
        float f, bw; int sf, cr, net, pwr;
        if (sscanf(line.c_str() + 4, "%f %d %f %d %d %d",
                   &f, &sf, &bw, &cr, &net, &pwr) != 6) {
            Serial.println("ERR CFG parse");
            return;
        }
        radioUp = applyCfg(f, sf, bw, cr, net, pwr);
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
        radio.startReceive();               /* back to listening immediately */
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

    if (radioUp) {
        int len = radio.getPacketLength(false);
        if (len > 0 && len <= 64) {
            uint8_t buf[64];
            int st = radio.readData(buf, len);
            if (st == RADIOLIB_ERR_NONE) {
                float rssi = radio.getRSSI(), snr = radio.getSNR();
                char hex[132];
                for (int i = 0; i < len; i++)
                    sprintf(hex + i * 2, "%02x", buf[i]);
                Serial.printf("RX %s %.0f %.1f\n", hex, rssi, snr);
            }
            radio.startReceive();
        }
    }
    delay(2);
}
