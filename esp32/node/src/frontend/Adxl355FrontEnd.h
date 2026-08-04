/*
 * Adxl355FrontEnd.h — ADXL355 SPI driver (spec §5.1 option A).
 * Low-noise MEMS accelerometer, ~25 µg/√Hz, hardware activity-wake on INT1.
 */
#pragma once
#include "FrontEnd.h"
#include <SPI.h>

class Adxl355FrontEnd : public FrontEnd {
public:
    Adxl355FrontEnd(SPIClass &spi, int cs_pin, int int1_pin, int miso_pin = -1)
        : _spi(spi), _cs(cs_pin), _int1(int1_pin), _miso(miso_pin) {}

    bool begin(uint16_t sample_rate_hz) override;
    size_t read(int16_t *out, size_t max) override;
    bool supportsMotionWake() const override { return true; }
    int armMotionWake(float threshold_g) override;
    bool selfTest() override;
    bool probe() override;
    bool alive() override;
    void powerDown() override;
    static void setSpiHz(uint32_t hz);   /* diagnostic: retry init dead-slow */
    void measureEntryProbe(uint32_t countdown_s); /* DMM-on-1V8-caps test */

private:
    uint8_t  reg(uint8_t addr);
    uint8_t  regAt(uint8_t addr, uint32_t hz);
    void     regWrite(uint8_t addr, uint8_t val);
    void     burstRead(uint8_t addr, uint8_t *buf, size_t n);
    void     postMortem();   /* after a bus death: who is driving MISO? */

    SPIClass &_spi;
    int _cs, _int1, _miso;
};
