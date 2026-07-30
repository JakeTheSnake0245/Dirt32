#include "DebugScreen.h"
#include <U8g2lib.h>
#include <Wire.h>
#include "chacha20poly1305.h"

/* Heltec WiFi LoRa 32 V3/V4 onboard SSD1306: I2C SDA=17 SCL=18 RST=21.
   The OLED is powered from Vext (GPIO36, ACTIVE LOW on Heltec boards) —
   the same rail as PIN_VE. We drive it low while the screen is on. */
#ifndef OLED_SDA
#define OLED_SDA 17
#endif
#ifndef OLED_SCL
#define OLED_SCL 18
#endif
#ifndef OLED_RST
#define OLED_RST 21
#endif

static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, OLED_RST, OLED_SCL, OLED_SDA);
static bool oledInit = false;

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
    Serial.println("[oled] button/cmd accepted — powering Vext + init");
    oledPower(true);
    delay(50);                     /* let the panel's charge pump settle */
    if (!oledInit) {
        /* Probe the panel at the two common SSD1306 addresses. */
        Wire.begin(OLED_SDA, OLED_SCL);
        for (uint8_t a : {0x3C, 0x3D}) {
            Wire.beginTransmission(a);
            uint8_t err = Wire.endTransmission();
            Serial.printf("[oled] i2c probe 0x%02X -> %s\n", a,
                          err == 0 ? "ACK" : "no response");
        }
        u8g2.begin();
        oledInit = true;
        Serial.println("[oled] u8g2 init done");
    }
    u8g2.setPowerSave(0);
    render(cfg, radioOk, seq);
    _visible = true;
    _shownAt = millis();
}

void DebugScreen::off() {
    if (oledInit) u8g2.setPowerSave(1);
    _visible = false;
    /* Leave Vext as-is: the sensor front-end shares the rail; main.cpp
       owns overall Ve power policy. Screen power-save alone drops the
       OLED to negligible draw. */
}

void DebugScreen::render(const NodeConfig &cfg, bool radioOk, uint32_t seq) {
    char line[32];
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);

    /* Everything on this page except NODE/SEQ/radio must MATCH on both
       boards for them to communicate. */
    snprintf(line, sizeof(line), "NET %u   NODE %u", cfg.net_id, cfg.node_id);
    u8g2.drawStr(0, 9, line);

    snprintf(line, sizeof(line), "FREQ %.1f MHz", cfg.f_in_mhz);
    u8g2.drawStr(0, 20, line);

    snprintf(line, sizeof(line), "SF%u BW%.0fk CR4/%u", cfg.sf, cfg.bw_khz, cfg.cr);
    u8g2.drawStr(0, 31, line);

    snprintf(line, sizeof(line), "KEY %04X   ACK %s",
             pskFingerprint(cfg.psk), cfg.ack_enable ? "ON" : "OFF");
    u8g2.drawStr(0, 42, line);

    snprintf(line, sizeof(line), "HB %u/day  TX %ddBm", cfg.heartbeat_per_day,
             (int)cfg.tx_power_dbm);
    u8g2.drawStr(0, 53, line);

    snprintf(line, sizeof(line), "SEQ %lu  RADIO %s", (unsigned long)seq,
             radioOk ? "OK" : "FAIL");
    u8g2.drawStr(0, 64, line);

    u8g2.sendBuffer();
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
