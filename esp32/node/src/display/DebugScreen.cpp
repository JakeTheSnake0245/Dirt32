#include "DebugScreen.h"
#include <oled_raw.h>
#include "chacha20poly1305.h"

static OledRaw oled;

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

void DebugScreen::show(const NodeConfig &cfg, bool radioOk, uint32_t seq) {
    if (!oled.begin()) return;         /* powers Vext, full re-init on wake */
    render(cfg, radioOk, seq);
    _visible = true;
    _shownAt = millis();
}

void DebugScreen::off() {
    oled.sleep();
    _visible = false;
    /* Leave Vext as-is: the sensor front-end shares the rail; main.cpp
       owns overall Ve power policy. */
}

void DebugScreen::render(const NodeConfig &cfg, bool radioOk, uint32_t seq) {
    char line[32];
    oled.clear();

    /* Everything on this page except NODE/SEQ/radio must MATCH on both
       boards for them to communicate. */
    snprintf(line, sizeof(line), "NET %u   NODE %u", cfg.net_id, cfg.node_id);
    oled.text(0, 0, line);

    snprintf(line, sizeof(line), "FREQ %.1f MHz", cfg.f_in_mhz);
    oled.text(0, 1, line);

    snprintf(line, sizeof(line), "SF%u BW%.0fk CR4/%u", cfg.sf, cfg.bw_khz, cfg.cr);
    oled.text(0, 2, line);

    snprintf(line, sizeof(line), "KEY %04X   ACK %s",
             pskFingerprint(cfg.psk), cfg.ack_enable ? "ON" : "OFF");
    oled.text(0, 4, line);

    snprintf(line, sizeof(line), "HB %u/day  TX %ddBm", cfg.heartbeat_per_day,
             (int)cfg.tx_power_dbm);
    oled.text(0, 5, line);

    snprintf(line, sizeof(line), "SEQ %lu  RADIO %s", (unsigned long)seq,
             radioOk ? "OK" : "FAIL");
    oled.text(0, 7, line);

    if (!oled.flush()) Serial.println("[oled] render flush failed");
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
            /* show() blocks long enough that `now` is stale — using it
               against the fresh _shownAt underflows the unsigned math and
               instantly re-blanks the screen. Start clean next tick. */
            return;
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
