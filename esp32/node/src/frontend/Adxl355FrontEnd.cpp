#include <Arduino.h>
#include "Adxl355FrontEnd.h"

/* ADXL355 register map (subset) */
static const uint8_t REG_DEVID_AD   = 0x00;  /* expect 0xAD */
static const uint8_t REG_PARTID     = 0x02;  /* expect 0xED */
static const uint8_t REG_FIFO_ENTRIES = 0x05;
static const uint8_t REG_ZDATA3     = 0x0E;
static const uint8_t REG_FIFO_DATA  = 0x11;
static const uint8_t REG_ACT_EN     = 0x24;
static const uint8_t REG_ACT_THRESH_H = 0x25;
static const uint8_t REG_ACT_COUNT  = 0x27;
static const uint8_t REG_FILTER     = 0x28;
static const uint8_t REG_INT_MAP    = 0x2A;
static const uint8_t REG_RANGE      = 0x2C;
static const uint8_t REG_POWER_CTL  = 0x2D;
static const uint8_t REG_SELFTEST   = 0x2E;

static const uint32_t SPI_HZ = 5000000;

uint8_t Adxl355FrontEnd::reg(uint8_t addr) {
    _spi.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    _spi.transfer((addr << 1) | 0x01);   /* read */
    uint8_t v = _spi.transfer(0x00);
    digitalWrite(_cs, HIGH);
    _spi.endTransaction();
    return v;
}

void Adxl355FrontEnd::regWrite(uint8_t addr, uint8_t val) {
    _spi.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    _spi.transfer(addr << 1);            /* write */
    _spi.transfer(val);
    digitalWrite(_cs, HIGH);
    _spi.endTransaction();
}

void Adxl355FrontEnd::burstRead(uint8_t addr, uint8_t *buf, size_t n) {
    _spi.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    _spi.transfer((addr << 1) | 0x01);
    for (size_t i = 0; i < n; i++) buf[i] = _spi.transfer(0x00);
    digitalWrite(_cs, HIGH);
    _spi.endTransaction();
}

bool Adxl355FrontEnd::begin(uint16_t sample_rate_hz) {
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
    pinMode(_int1, INPUT);
    delay(10); /* Ve rail settle */

    uint8_t devid = reg(REG_DEVID_AD);
    uint8_t partid = reg(REG_PARTID);
    if (devid != 0xAD || partid != 0xED) {
        /* Print raw bytes — they tell you WHERE the wiring is broken:
         *   0x00/0x00 → MISO line dead, sensor unpowered, or CS not reaching it
         *   0xFF/0xFF → MISO floating (sensor not driving the bus: CS or SCLK wrong)
         *   other     → data garbled (SCLK/MOSI swapped, bad contact, wrong mode) */
        Serial.printf("[adxl355] ID check FAILED: DEVID_AD=0x%02X (want 0xAD) "
                      "PARTID=0x%02X (want 0xED)\n", devid, partid);
        return false;
    }
    Serial.println("[adxl355] ID OK (0xAD/0xED)");

    regWrite(REG_POWER_CTL, 0x01);          /* standby for config */
    regWrite(REG_RANGE, 0x01);              /* ±2 g — max sensitivity */

    /* ODR: 0=4kHz .. 5=125Hz, 4=250Hz, 3=500Hz. Pick nearest >= requested. */
    uint8_t odr = (sample_rate_hz > 250) ? 0x03 /*500*/ : 0x04 /*250*/;
    regWrite(REG_FILTER, odr);              /* no HPF here — done in DSP */

    regWrite(REG_POWER_CTL, 0x00);          /* measurement mode */
    delay(20);
    return true;
}

size_t Adxl355FrontEnd::read(int16_t *out, size_t max) {
    /* Drain FIFO: 3 bytes per axis-sample, sequence X,Y,Z (marker bits in
       byte3 bit0-1). We keep only Z (vertical) per spec §5.2. */
    size_t got = 0;
    uint8_t entries = reg(REG_FIFO_ENTRIES);   /* axis-samples pending */
    while (entries >= 3 && got < max) {
        uint8_t raw[9];
        burstRead(REG_FIFO_DATA, raw, 9);      /* one X,Y,Z triplet */
        /* Z is the third 3-byte word; 20-bit left-justified two's complement */
        int32_t z = ((int32_t)raw[6] << 12) | ((int32_t)raw[7] << 4) | (raw[8] >> 4);
        if (z & 0x80000) z -= 0x100000;        /* sign-extend 20-bit */
        /* normalize 20-bit -> 16-bit */
        out[got++] = (int16_t)(z >> 4);
        entries -= 3;
    }
    return got;
}

int Adxl355FrontEnd::armMotionWake(float threshold_g) {
    /* ACT_THRESH is in units of 62.5 µg/LSB at ±2 g range (threshold/0.0000625),
       registers hold value/8 in 16-bit split across H/L. */
    uint32_t counts = (uint32_t)(threshold_g / 0.0000625f) >> 3;
    if (counts > 0xFFFF) counts = 0xFFFF;
    regWrite(REG_POWER_CTL, 0x01);                    /* standby */
    regWrite(REG_ACT_THRESH_H, (counts >> 8) & 0xFF);
    regWrite(REG_ACT_THRESH_H + 1, counts & 0xFF);
    regWrite(REG_ACT_COUNT, 2);                       /* 2 consecutive over-threshold */
    regWrite(REG_ACT_EN, 0x04);                       /* Z axis only */
    regWrite(REG_INT_MAP, 0x08);                      /* ACT -> INT1 */
    /* low-power measurement stays on so activity engine runs */
    regWrite(REG_POWER_CTL, 0x00);
    return _int1;
}

bool Adxl355FrontEnd::selfTest() {
    /* First: re-probe the device IDs so a wiring fault is reported here,
     * not just in the boot log.
     *   0x00/0x00 → MISO dead, sensor unpowered, or CS not connected
     *   0xFF/0xFF → sensor not driving the bus (CS or SCLK on wrong pin)
     *   other     → data garbled (SCLK/MOSI swapped, bad contact)      */
    uint8_t devid = reg(REG_DEVID_AD);
    uint8_t partid = reg(REG_PARTID);
    if (devid != 0xAD || partid != 0xED) {
        Serial.printf("[adxl355] ID check FAILED: DEVID_AD=0x%02X (want 0xAD) "
                      "PARTID=0x%02X (want 0xED) — wiring fault, see README\n",
                      devid, partid);
        return false;
    }
    Serial.println("[adxl355] ID OK (0xAD/0xED) — wiring good, testing actuation...");

    /* Force self-test actuation, expect Z shift of roughly 0.1-0.6 g. */
    int16_t buf[8];
    delay(50); (void)read(buf, 8);
    int32_t before = 0; size_t n = read(buf, 8);
    for (size_t i = 0; i < n; i++) before += buf[i];
    if (n) before /= (int32_t)n;

    regWrite(REG_SELFTEST, 0x03);  /* ST1|ST2 */
    delay(100); (void)read(buf, 8); delay(50);
    int32_t after = 0; n = read(buf, 8);
    for (size_t i = 0; i < n; i++) after += buf[i];
    if (n) after /= (int32_t)n;
    regWrite(REG_SELFTEST, 0x00);

    int32_t delta = after - before;
    if (delta < 0) delta = -delta;
    /* ±2 g over 16-bit => ~0.1 g ≈ 1638 counts; accept a broad window */
    return delta > 400;
}

bool Adxl355FrontEnd::probe() {
    uint8_t devid = reg(REG_DEVID_AD);
    uint8_t partid = reg(REG_PARTID);
    bool ok = (devid == 0xAD && partid == 0xED);
    Serial.printf("[probe %8lums] DEVID_AD=0x%02X PARTID=0x%02X %s\n",
                  (unsigned long)millis(), devid, partid,
                  ok ? "OK" : "FAIL");
    return ok;
}

void Adxl355FrontEnd::powerDown() {
    regWrite(REG_POWER_CTL, 0x01); /* standby; Ve will be cut anyway */
}
