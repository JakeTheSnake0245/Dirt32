/*
 * DebugScreen — bench/debug OLED page for the Heltec V4 onboard SSD1306.
 *
 * Purpose: field-check that two boards can talk. Pressing the PRG (USER)
 * button shows every parameter that must MATCH between boards for the radio
 * link to work, plus a 4-hex-digit key fingerprint so you can confirm both
 * boards hold the same PSK without ever displaying the key itself.
 *
 * The screen auto-blanks after DISPLAY_TIMEOUT_MS to save power; the OLED is
 * never initialized until the first button press (it is unpowered in normal
 * operation and irrelevant to buried deployment).
 */
#pragma once
#include <Arduino.h>
#include "../config.h"

class DebugScreen {
public:
    /* buttonPin: PRG/USER button, active-low with internal pullup. */
    void begin(uint8_t buttonPin);

    /* Call from loop(). Handles debounce, shows the page on press,
       refreshes while visible, blanks after timeout. */
    void poll(const NodeConfig &cfg, bool radioOk, uint32_t seq);

    /* Force the page on (used by the `screen` CLI command). */
    void show(const NodeConfig &cfg, bool radioOk, uint32_t seq);

private:
    static const uint32_t DISPLAY_TIMEOUT_MS = 20000;
    uint8_t  _btn = 0;
    bool     _visible = false;
    uint32_t _shownAt = 0;
    uint32_t _lastDraw = 0;
    bool     _lastBtn = false;     /* debounced pressed-state (true = pressed) */
    bool     _rawLast = false;     /* last raw pressed-state sample */
    uint32_t _btnChangedAt = 0;

    void render(const NodeConfig &cfg, bool radioOk, uint32_t seq);
    void off();
};

/* 16-bit PSK fingerprint (safe to display; derived via the AEAD under a
   nonce that can never occur on the wire). Two boards showing the same
   4 hex digits hold the same key. */
uint16_t pskFingerprint(const uint8_t psk[32]);
