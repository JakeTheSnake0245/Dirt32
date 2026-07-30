#include "DebugScreen.h"
#include <Wire.h>
#include "chacha20poly1305.h"

/* Heltec WiFi LoRa 32 V4 onboard SSD1315 (SSD1306-compatible):
   I2C addr 0x3C, SDA=17 SCL=18, RST=21, powered from Vext (GPIO36,
   ACTIVE LOW). Driven RAW over Wire — no display library. This mirrors
   Heltec's own factory-test init and gives us an error code for every
   single I2C transaction. */
#ifndef OLED_SDA
#define OLED_SDA 17
#endif
#ifndef OLED_SCL
#define OLED_SCL 18
#endif
#ifndef OLED_RST
#define OLED_RST 21
#endif
#define OLED_ADDR 0x3C

static bool oledInit = false;
static uint8_t fb[128 * 8];          /* 128x64 framebuffer, 8 pages */

/* ---------- tiny raw SSD1306 driver ---------- */

static uint8_t oledCmd(uint8_t c) {
    Wire.beginTransmission(OLED_ADDR);
    Wire.write(0x00);                /* Co=0 D/C=0: command */
    Wire.write(c);
    return Wire.endTransmission();
}

static bool oledCmds(const uint8_t *cmds, size_t n, const char *what) {
    for (size_t i = 0; i < n; i++) {
        uint8_t err = oledCmd(cmds[i]);
        if (err != 0) {
            Serial.printf("[oled] cmd 0x%02X (%s) FAILED i2c err=%u\n",
                          cmds[i], what, err);
            return false;
        }
    }
    return true;
}

static bool oledFlush() {
    /* set full window, then stream the framebuffer in 16-byte chunks */
    static const uint8_t win[] = { 0x21, 0, 127,    /* column 0..127 */
                                   0x22, 0, 7 };    /* page 0..7 */
    if (!oledCmds(win, sizeof(win), "window")) return false;
    for (int i = 0; i < 128 * 8; i += 16) {
        Wire.beginTransmission(OLED_ADDR);
        Wire.write(0x40);            /* Co=0 D/C=1: data */
        Wire.write(&fb[i], 16);
        uint8_t err = Wire.endTransmission();
        if (err != 0) {
            Serial.printf("[oled] data chunk @%d FAILED i2c err=%u\n", i, err);
            return false;
        }
    }
    return true;
}

static bool oledPanelInit() {
    /* Standard SSD1306/SSD1315 128x64 init (same values Heltec ships). */
    static const uint8_t seq[] = {
        0xAE,             /* display off */
        0xD5, 0x80,       /* clock divide */
        0xA8, 0x3F,       /* multiplex 64 */
        0xD3, 0x00,       /* display offset 0 */
        0x40,             /* start line 0 */
        0x8D, 0x14,       /* charge pump ON */
        0x20, 0x00,       /* horizontal addressing mode */
        0xA1,             /* segment remap (flip X) */
        0xC8,             /* COM scan dec (flip Y) */
        0xDA, 0x12,       /* COM pins: alt, no remap */
        0x81, 0xCF,       /* contrast */
        0xD9, 0xF1,       /* precharge */
        0xDB, 0x40,       /* VCOMH */
        0xA4,             /* resume from RAM */
        0xA6,             /* normal (not inverted) */
        0xAF,             /* display ON */
    };
    return oledCmds(seq, sizeof(seq), "init");
}

/* ---------- 5x7 font (ASCII 32..127), classic public-domain table ---------- */
static const uint8_t font5x7[] PROGMEM = {
    0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x5F,0x00,0x00, 0x00,0x07,0x00,0x07,0x00,
    0x14,0x7F,0x14,0x7F,0x14, 0x24,0x2A,0x7F,0x2A,0x12, 0x23,0x13,0x08,0x64,0x62,
    0x36,0x49,0x55,0x22,0x50, 0x00,0x05,0x03,0x00,0x00, 0x00,0x1C,0x22,0x41,0x00,
    0x00,0x41,0x22,0x1C,0x00, 0x14,0x08,0x3E,0x08,0x14, 0x08,0x08,0x3E,0x08,0x08,
    0x00,0x50,0x30,0x00,0x00, 0x08,0x08,0x08,0x08,0x08, 0x00,0x60,0x60,0x00,0x00,
    0x20,0x10,0x08,0x04,0x02, 0x3E,0x51,0x49,0x45,0x3E, 0x00,0x42,0x7F,0x40,0x00,
    0x42,0x61,0x51,0x49,0x46, 0x21,0x41,0x45,0x4B,0x31, 0x18,0x14,0x12,0x7F,0x10,
    0x27,0x45,0x45,0x45,0x39, 0x3C,0x4A,0x49,0x49,0x30, 0x01,0x71,0x09,0x05,0x03,
    0x36,0x49,0x49,0x49,0x36, 0x06,0x49,0x49,0x29,0x1E, 0x00,0x36,0x36,0x00,0x00,
    0x00,0x56,0x36,0x00,0x00, 0x08,0x14,0x22,0x41,0x00, 0x14,0x14,0x14,0x14,0x14,
    0x00,0x41,0x22,0x14,0x08, 0x02,0x01,0x51,0x09,0x06, 0x32,0x49,0x79,0x41,0x3E,
    0x7E,0x11,0x11,0x11,0x7E, 0x7F,0x49,0x49,0x49,0x36, 0x3E,0x41,0x41,0x41,0x22,
    0x7F,0x41,0x41,0x22,0x1C, 0x7F,0x49,0x49,0x49,0x41, 0x7F,0x09,0x09,0x09,0x01,
    0x3E,0x41,0x49,0x49,0x7A, 0x7F,0x08,0x08,0x08,0x7F, 0x00,0x41,0x7F,0x41,0x00,
    0x20,0x40,0x41,0x3F,0x01, 0x7F,0x08,0x14,0x22,0x41, 0x7F,0x40,0x40,0x40,0x40,
    0x7F,0x02,0x0C,0x02,0x7F, 0x7F,0x04,0x08,0x10,0x7F, 0x3E,0x41,0x41,0x41,0x3E,
    0x7F,0x09,0x09,0x09,0x06, 0x3E,0x41,0x51,0x21,0x5E, 0x7F,0x09,0x19,0x29,0x46,
    0x46,0x49,0x49,0x49,0x31, 0x01,0x01,0x7F,0x01,0x01, 0x3F,0x40,0x40,0x40,0x3F,
    0x1F,0x20,0x40,0x20,0x1F, 0x3F,0x40,0x38,0x40,0x3F, 0x63,0x14,0x08,0x14,0x63,
    0x07,0x08,0x70,0x08,0x07, 0x61,0x51,0x49,0x45,0x43, 0x00,0x7F,0x41,0x41,0x00,
    0x02,0x04,0x08,0x10,0x20, 0x00,0x41,0x41,0x7F,0x00, 0x04,0x02,0x01,0x02,0x04,
    0x40,0x40,0x40,0x40,0x40, 0x00,0x01,0x02,0x04,0x00, 0x20,0x54,0x54,0x54,0x78,
    0x7F,0x48,0x44,0x44,0x38, 0x38,0x44,0x44,0x44,0x20, 0x38,0x44,0x44,0x48,0x7F,
    0x38,0x54,0x54,0x54,0x18, 0x08,0x7E,0x09,0x01,0x02, 0x0C,0x52,0x52,0x52,0x3E,
    0x7F,0x08,0x04,0x04,0x78, 0x00,0x44,0x7D,0x40,0x00, 0x20,0x40,0x44,0x3D,0x00,
    0x7F,0x10,0x28,0x44,0x00, 0x00,0x41,0x7F,0x40,0x00, 0x7C,0x04,0x18,0x04,0x78,
    0x7C,0x08,0x04,0x04,0x78, 0x38,0x44,0x44,0x44,0x38, 0x7C,0x14,0x14,0x14,0x08,
    0x08,0x14,0x14,0x18,0x7C, 0x7C,0x08,0x04,0x04,0x08, 0x48,0x54,0x54,0x54,0x20,
    0x04,0x3F,0x44,0x40,0x20, 0x3C,0x40,0x40,0x20,0x7C, 0x1C,0x20,0x40,0x20,0x1C,
    0x3C,0x40,0x30,0x40,0x3C, 0x44,0x28,0x10,0x28,0x44, 0x0C,0x50,0x50,0x50,0x3C,
    0x44,0x64,0x54,0x4C,0x44, 0x00,0x08,0x36,0x41,0x00, 0x00,0x00,0x7F,0x00,0x00,
    0x00,0x41,0x36,0x08,0x00, 0x08,0x08,0x2A,0x1C,0x08, 0x08,0x1C,0x2A,0x08,0x08,
};

static void fbClear() { memset(fb, 0, sizeof(fb)); }

/* Draw a string at column x (pixels), page row (0..7). 6 px advance. */
static void fbText(int x, int page, const char *s) {
    if (page < 0 || page > 7) return;
    for (; *s && x <= 122; s++, x += 6) {
        char c = *s;
        if (c < 32 || c > 126) c = '?';
        const uint8_t *g = &font5x7[(c - 32) * 5];
        for (int i = 0; i < 5; i++)
            fb[page * 128 + x + i] = pgm_read_byte(&g[i]);
    }
}

uint16_t pskFingerprint(const uint8_t psk[32]) {
    /* Wire nonces are NODE_ID(2)|TYPE(1)|SEQ(3)|000000 — the last 6 bytes are
       always zero. A nonce with a non-zero tail is therefore guaranteed
       disjoint from every nonce ever used to protect real traffic. */
    static const uint8_t fpNonce[12] =
        { 'S','P','S','-','F','P','R','\xA5','\xA5','\xA5','\xA5','\xA5' };
    const uint8_t zero[2] = {0, 0};
    uint8_t ct[2], tag[16];
    cc20p1305_encrypt(psk, fpNonce, NULL, 0, zero, sizeof(zero), ct, tag);
    return ((uint16_t)ct[0] << 8) | ct[1];
}

void DebugScreen::begin(uint8_t buttonPin) {
    _btn = buttonPin;
    pinMode(_btn, INPUT_PULLUP);
}

static void oledPower(bool on) {
    /* Vext is active-LOW on Heltec (powers OLED + external rail). */
    pinMode(PIN_VE, OUTPUT);
    digitalWrite(PIN_VE, on ? LOW : HIGH);
    if (on) delay(20);
}

void DebugScreen::show(const NodeConfig &cfg, bool radioOk, uint32_t seq) {
    Serial.println("[oled] show() called");
    oledPower(true);
    if (!oledInit) {
        delay(100);                        /* Vext + charge pump settle */
        Wire.begin(OLED_SDA, OLED_SCL);
        Wire.setClock(400000);
        pinMode(OLED_RST, OUTPUT);
        digitalWrite(OLED_RST, LOW);  delay(10);
        digitalWrite(OLED_RST, HIGH); delay(50);
        Wire.beginTransmission(OLED_ADDR);
        Serial.printf("[oled] probe 0x3C -> %s\n",
                      Wire.endTransmission() == 0 ? "ACK" : "no response");
        if (!oledPanelInit()) return;
        Serial.println("[oled] panel init OK");
        /* test pattern: all pixels ON for 1 s */
        memset(fb, 0xFF, sizeof(fb));
        if (!oledFlush()) return;
        Serial.println("[oled] test pattern (all pixels ON) sent OK");
        delay(1000);
        oledInit = true;
    } else {
        /* Re-run the full init on every wake: the SSD1315's charge pump
           must be re-enabled after a display-off, or the panel stays dark
           while still ACKing every write. Init is idempotent and takes
           only a few ms. */
        if (!oledPanelInit()) return;
        Serial.println("[oled] re-init on wake OK");
    }
    render(cfg, radioOk, seq);
    _visible = true;
    _shownAt = millis();
}

void DebugScreen::off() {
    if (oledInit) oledCmd(0xAE);           /* display OFF (sleep) */
    Serial.println("[oled] off (sleep)");
    _visible = false;
    /* Leave Vext as-is: the sensor front-end shares the rail; main.cpp
       owns overall Ve power policy. */
}

void DebugScreen::render(const NodeConfig &cfg, bool radioOk, uint32_t seq) {
    char line[32];
    fbClear();

    /* Everything on this page except NODE/SEQ/radio must MATCH on both
       boards for them to communicate. */
    snprintf(line, sizeof(line), "NET %u   NODE %u", cfg.net_id, cfg.node_id);
    fbText(0, 0, line);

    snprintf(line, sizeof(line), "FREQ %.1f MHz", cfg.f_in_mhz);
    fbText(0, 1, line);

    snprintf(line, sizeof(line), "SF%u BW%.0fk CR4/%u", cfg.sf, cfg.bw_khz, cfg.cr);
    fbText(0, 2, line);

    snprintf(line, sizeof(line), "KEY %04X   ACK %s",
             pskFingerprint(cfg.psk), cfg.ack_enable ? "ON" : "OFF");
    fbText(0, 4, line);

    snprintf(line, sizeof(line), "HB %u/day  TX %ddBm", cfg.heartbeat_per_day,
             (int)cfg.tx_power_dbm);
    fbText(0, 5, line);

    snprintf(line, sizeof(line), "SEQ %lu  RADIO %s", (unsigned long)seq,
             radioOk ? "OK" : "FAIL");
    fbText(0, 7, line);

    if (!oledFlush()) Serial.println("[oled] render flush failed");
}

void DebugScreen::poll(const NodeConfig &cfg, bool radioOk, uint32_t seq) {
    bool raw = (digitalRead(_btn) == LOW);
    uint32_t now = millis();

    /* Debounce: raw state must be stable 40 ms before we accept it. */
    if (raw != _rawLast) { _rawLast = raw; _btnChangedAt = now; }
    if ((now - _btnChangedAt) > 40 && raw != _lastBtn) {
        _lastBtn = raw;
        if (raw) {                        /* accepted press edge */
            if (_visible) off();          /* second press hides it */
            else show(cfg, radioOk, seq);
        }
    }

    if (_visible) {
        if (now - _shownAt > DISPLAY_TIMEOUT_MS) off();
        else if (now - _lastDraw > 1000) {    /* refresh SEQ once a second */
            render(cfg, radioOk, seq);
            _lastDraw = now;
        }
    }
}
