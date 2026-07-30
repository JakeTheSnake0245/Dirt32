#include "DebugScreen.h"
#include <oled_raw.h>
#include "chacha20poly1305.h"

static OledRaw oled;

/* ---------- PSK fingerprint (safe to display) ---------- */
uint16_t pskFingerprint(const uint8_t psk[32]) {
    static const uint8_t fpNonce[12] =
        { 'S','P','S','-','F','P','R','\xA5','\xA5','\xA5','\xA5','\xA5' };
    const uint8_t zero[2] = {0, 0};
    uint8_t ct[2], tag[16];
    cc20p1305_encrypt(psk, fpNonce, NULL, 0, zero, sizeof(zero), ct, tag);
    return ((uint16_t)ct[0] << 8) | ct[1];
}

/* ---------- Internals ---------- */
void DebugScreen::ensureInit() {
    if (!oled.ready()) oled.begin();
}

void DebugScreen::off() {
    oled.sleep();
    _visible = false;
    _showingConfig = false;
}

/* ---------- Public: show pages ---------- */
void DebugScreen::showConfig(const NodeConfig &cfg, bool radioOk, uint32_t seq) {
    ensureInit();
    _cfgPtr = &cfg; _radioOk = radioOk; _seq = seq;
    renderConfig(cfg, radioOk, seq);
    _visible = true;
    _showingConfig = true;
    _shownAt = millis();
    _lastDraw = millis();
}

void DebugScreen::showMessage(const char *line1, const char *line2, const char *line3) {
    ensureInit();
    oled.clear();
    if (line1) oled.text(0, 2, line1);
    if (line2) oled.text(0, 4, line2);
    if (line3) oled.text(0, 6, line3);
    oled.flush();
    _visible = true;
    _showingConfig = false;
    _shownAt = millis();
    _lastDraw = millis();
}

void DebugScreen::refreshIfVisible(const NodeConfig &cfg, bool radioOk, uint32_t seq) {
    if (!_visible || !_showingConfig) return;
    uint32_t now = millis();
    if (now - _shownAt > DISPLAY_TIMEOUT_MS) { off(); return; }
    if (now - _lastDraw > 1000) {
        renderConfig(cfg, radioOk, seq);
        _lastDraw = now;
    }
}

/* ---------- Config page render ---------- */
void DebugScreen::renderConfig(const NodeConfig &cfg, bool radioOk, uint32_t seq) {
    char line[32];
    oled.clear();

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

    snprintf(line, sizeof(line), "SEQ %lu  %s", (unsigned long)seq,
             radioOk ? "RADIO OK" : "RADIO FAIL");
    oled.text(0, 7, line);

    if (!oled.flush()) Serial.println("[oled] render flush failed");
}

/* ---------- Button poll — three tiers ---------- */
BtnEvent DebugScreen::poll() {
    bool raw = (digitalRead(_btn) == LOW);
    uint32_t now = millis();
    BtnEvent ev = BtnEvent::NONE;

    if (raw && !_pressed) {
        /* Press start */
        _pressed = true;
        _pressedAt = now;
        _sleepFired = false;
    } else if (_pressed && raw) {
        /* Held down — fire SLEEP at threshold without waiting for release */
        if (!_sleepFired && (now - _pressedAt) >= SLEEP_THRESH_MS) {
            _sleepFired = true;
            ev = BtnEvent::SLEEP;
        }
    } else if (_pressed && !raw) {
        /* Released */
        uint32_t held = now - _pressedAt;
        _pressed = false;
        if (!_sleepFired) {
            if (held < HOLD_THRESH_MS)
                ev = BtnEvent::TAP;
            else
                ev = BtnEvent::HOLD;
        }
        /* If SLEEP already fired, discard the release — return NONE */
    }

    return ev;
}

void DebugScreen::begin(uint8_t buttonPin) {
    _btn = buttonPin;
    pinMode(_btn, INPUT_PULLUP);
}
