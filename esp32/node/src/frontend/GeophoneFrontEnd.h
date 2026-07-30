/*
 * GeophoneFrontEnd.h — SM-24 geophone + ADS1220 24-bit ADC (spec §5.1 option B).
 * Best weak-signal sensitivity; must convert continuously (no hardware wake).
 */
#pragma once
#include "FrontEnd.h"
#include <SPI.h>

class GeophoneFrontEnd : public FrontEnd {
public:
    GeophoneFrontEnd(SPIClass &spi, int cs_pin, int drdy_pin)
        : _spi(spi), _cs(cs_pin), _drdy(drdy_pin) {}

    bool begin(uint16_t sample_rate_hz) override;
    size_t read(int16_t *out, size_t max) override;
    bool supportsMotionWake() const override { return false; }
    int armMotionWake(float) override { return -1; }
    bool selfTest() override;
    void powerDown() override;

private:
    void cmd(uint8_t c);
    void writeRegs(const uint8_t regs[4]);

    SPIClass &_spi;
    int _cs, _drdy;
};
