/*
 * DebugScreen — OLED status/config page for the Heltec V4.
 *
 * Button tiers (PRG / USER button, GPIO0, active-low):
 *   TAP    (< 800 ms)    → caller sends deploy heartbeat + shows deploy page
 *   HOLD   (800 ms–8 s)  → released: caller shows config page
 *   SLEEP  (≥ 8 s)       → fires immediately (no release needed): caller sleeps
 *
 * Screen auto-blanks after DISPLAY_TIMEOUT_MS. Always-off until first event.
 */
#pragma once
#include <Arduino.h>
#include "../config.h"

enum class BtnEvent : uint8_t { NONE, TAP, HOLD, SLEEP };

class DebugScreen {
public:
    void begin(uint8_t buttonPin);

    /* Call every loop(). Returns the event if one fired this tick. */
    BtnEvent poll();

    /* Show the link-config page (radio params, key fingerprint, SEQ). */
    void showConfig(const NodeConfig &cfg, bool radioOk, uint32_t seq);

    /* Show a full-screen status message (e.g. "DEPLOYING...", "SLEEPING"). */
    void showMessage(const char *line1, const char *line2 = nullptr,
                     const char *line3 = nullptr);

    /* Blank the display. */
    void off();

    bool visible() const { return _visible; }

    /* Refresh config page if already showing (call from loop when seq changes). */
    void refreshIfVisible(const NodeConfig &cfg, bool radioOk, uint32_t seq);

private:
    static const uint32_t DISPLAY_TIMEOUT_MS = 20000;
    static const uint32_t HOLD_THRESH_MS     =   800;
    static const uint32_t SLEEP_THRESH_MS    =  8000;

    uint8_t  _btn = 0;
    bool     _visible = false;
    uint32_t _shownAt = 0;
    uint32_t _lastDraw = 0;
    bool     _lastRaw = false;
    uint32_t _pressedAt = 0;      /* millis() when current press started */
    bool     _pressed = false;    /* button is currently down */
    bool     _sleepFired = false; /* SLEEP event already emitted this press */

    /* Cached for refresh */
    const NodeConfig *_cfgPtr = nullptr;
    bool     _radioOk = false;
    uint32_t _seq = 0;

    bool     _showingConfig = false;

    void renderConfig(const NodeConfig &cfg, bool radioOk, uint32_t seq);
    void ensureInit();
};

uint16_t pskFingerprint(const uint8_t psk[32]);
