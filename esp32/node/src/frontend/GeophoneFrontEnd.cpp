#include <Arduino.h>
#include "GeophoneFrontEnd.h"

/* ADS1220 commands */
static const uint8_t CMD_RESET     = 0x06;
static const uint8_t CMD_START     = 0x08;
static const uint8_t CMD_POWERDOWN = 0x02;
static const uint8_t CMD_RDATA     = 0x10;
static const uint8_t CMD_WREG      = 0x40;

static const uint32_t SPI_HZ = 2000000;

void GeophoneFrontEnd::cmd(uint8_t c) {
    _spi.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE1));
    digitalWrite(_cs, LOW);
    _spi.transfer(c);
    digitalWrite(_cs, HIGH);
    _spi.endTransaction();
}

void GeophoneFrontEnd::writeRegs(const uint8_t regs[4]) {
    _spi.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE1));
    digitalWrite(_cs, LOW);
    _spi.transfer(CMD_WREG | 0x03);  /* start reg0, write 4 */
    for (int i = 0; i < 4; i++) _spi.transfer(regs[i]);
    digitalWrite(_cs, HIGH);
    _spi.endTransaction();
}

bool GeophoneFrontEnd::begin(uint16_t sample_rate_hz) {
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
    pinMode(_drdy, INPUT);
    delay(5);

    cmd(CMD_RESET);
    delay(1);

    /* reg0: AIN0-AIN1 differential (geophone across inputs), PGA gain 32
       reg1: data rate — normal mode: 330 SPS (0xA0) or 600 (0xC0); pick per cfg
       reg2: internal 2.048 V ref, no rejection filter, no IDAC
       reg3: default */
    uint8_t dr = (sample_rate_hz > 330) ? 0xC0 : 0xA0;
    uint8_t regs[4] = {
        (uint8_t)(0x00 /* MUX AIN0-AIN1 */ | (0x05 << 1) /* gain 32 */),
        (uint8_t)(dr | 0x04 /* continuous conversion */),
        0x00,
        0x00,
    };
    writeRegs(regs);
    cmd(CMD_START);
    delay(5);
    /* Sanity: DRDY should assert within ~2 conversion periods */
    uint32_t t0 = millis();
    while (digitalRead(_drdy) == HIGH) {
        if (millis() - t0 > 50) return false;
    }
    return true;
}

size_t GeophoneFrontEnd::read(int16_t *out, size_t max) {
    size_t got = 0;
    while (got < max && digitalRead(_drdy) == LOW) {
        _spi.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE1));
        digitalWrite(_cs, LOW);
        _spi.transfer(CMD_RDATA);
        int32_t v = ((int32_t)_spi.transfer(0) << 16) |
                    ((int32_t)_spi.transfer(0) << 8) |
                    _spi.transfer(0);
        digitalWrite(_cs, HIGH);
        _spi.endTransaction();
        if (v & 0x800000) v -= 0x1000000;     /* sign-extend 24-bit */
        out[got++] = (int16_t)(v >> 8);       /* normalize 24 -> 16 bit */
        /* DRDY deasserts after read; loop exits until next conversion */
    }
    return got;
}

bool GeophoneFrontEnd::selfTest() {
    /* Basic liveness: conversions flowing and not railed. */
    int16_t buf[16];
    delay(60);
    size_t n = read(buf, 16);
    if (n == 0) return false;
    for (size_t i = 0; i < n; i++)
        if (buf[i] > 32000 || buf[i] < -32000) return false; /* railed => open/short */
    return true;
}

bool GeophoneFrontEnd::probe() {
    /* ADS1220 has no ID register — liveness = conversions flowing. */
    int16_t buf[4];
    delay(10);
    size_t n = read(buf, 4);
    Serial.printf("[probe %8lums] ADS1220 conversions=%u %s\n",
                  (unsigned long)millis(), (unsigned)n, n ? "OK" : "FAIL");
    return n > 0;
}

void GeophoneFrontEnd::powerDown() {
    cmd(CMD_POWERDOWN);
}
