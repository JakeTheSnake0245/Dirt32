/*
 * oled_raw — minimal raw I2C driver for the Heltec V3/V4 onboard SSD1315
 * (SSD1306-compatible) 128x64 OLED. No display library: every I2C
 * transaction is error-checked, and init is re-runnable (the SSD1315
 * drops its charge pump on display-off and needs a full re-init on wake).
 *
 * Proven on real Heltec WiFi LoRa 32 V4 hardware (see node DebugScreen).
 *
 * Defaults (override with -D at build time if a board revision differs):
 *   SDA=17 SCL=18 RST=21, Vext=GPIO36 ACTIVE LOW, addr 0x3C.
 */
#pragma once
#include <Arduino.h>

class OledRaw {
public:
    /* Power Vext, reset the panel, init the controller. Returns false if
       any I2C step fails (each failure is logged on Serial). Safe to call
       again to wake from sleep(). */
    bool begin();

    void sleep();                      /* display off (RAM retained) */

    void clear();                      /* clear framebuffer */
    /* 5x7 font, 6 px advance. x in pixels 0..122, page (text row) 0..7. */
    void text(int x, int page, const char *s);
    bool flush();                      /* push framebuffer to the panel */

    bool ready() const { return _init; }

private:
    bool panelInit();
    uint8_t cmd(uint8_t c);
    bool cmds(const uint8_t *c, size_t n, const char *what);
    uint8_t _fb[128 * 8] = {0};
    bool _init = false;
};
