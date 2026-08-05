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
static const uint8_t REG_STATUS     = 0x04;  /* bit4 = NVM_BUSY */
static const uint8_t REG_RESET      = 0x2F;  /* write 0x52 = soft reset */

/* Bus clock is adjustable at runtime: chip observed latching at measurement
 * entry; if the whole init sequence at 100 kHz survives, the root cause is
 * signal integrity (edge ringing/overshoot on jumper wires) at 5 MHz. */
static uint32_t SPI_HZ = 5000000;

void Adxl355FrontEnd::setSpiHz(uint32_t hz) {
    SPI_HZ = hz;
    Serial.printf("[adxl355] SPI clock set to %lu Hz\n", (unsigned long)hz);
}

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

    /* Soft-reset the chip (reg 0x2F <- 0x52) and wait for its internal
     * NVM/fuse reload (STATUS bit4 NVM_BUSY) to finish before configuring.
     * The datasheet warns that improper power cycling can leave the part in
     * a corrupt state causing "lost communications" — a soft reset is the
     * only way to clear that without a full supply drain. */
    regWrite(REG_RESET, 0x52);
    delay(30);
    uint8_t d2 = reg(REG_DEVID_AD);
    if (d2 != 0xAD) {
        Serial.printf("[adxl355] BUS DIED after soft RESET (DEVID now 0x%02X)\n", d2);
        postMortem();
        return false;
    }
    uint32_t t0 = millis();
    uint8_t st;
    while (((st = reg(REG_STATUS)) & 0x10) && millis() - t0 < 200) delay(2);
    if (st & 0x10) Serial.println("[adxl355] WARNING: NVM_BUSY stuck after reset");

    /* Verify liveness after each config step; quiet on success, full
     * postmortem (MISO pull-test, blind-revive, toggle recovery) on death.
     * Kept permanently: a MISO clamp from an external short is invisible
     * otherwise (lesson: PMDZ pin 3/9 breadboard bridge, Aug 2026). */
    auto still = [this](const char *after) {
        uint8_t d = reg(REG_DEVID_AD);
        if (d != 0xAD) {
            Serial.printf("[adxl355] BUS DIED after %s (DEVID now 0x%02X)\n",
                          after, d);
            postMortem();
            return false;
        }
        return true;
    };

    regWrite(REG_POWER_CTL, 0x01);          /* standby for config */
    if (!still("POWER_CTL=standby")) return false;
    regWrite(REG_RANGE, 0x01);              /* ±2 g — max sensitivity */
    if (!still("RANGE=2g")) return false;
    {   /* readback-verify: proves MOSI/write path integrity */
        uint8_t rr = reg(REG_RANGE);
        if (rr != 0x01) {
            Serial.printf("[adxl355] RANGE readback 0x%02X (want 0x01) <- WRITE PATH BROKEN\n", rr);
            return false;
        }
    }

    /* ODR: 0=4kHz .. 5=125Hz, 4=250Hz, 3=500Hz. Pick nearest >= requested. */
    uint8_t odr = (sample_rate_hz > 250) ? 0x03 /*500*/ : 0x04 /*250*/;
    regWrite(REG_FILTER, odr);              /* no HPF here — done in DSP */
    if (!still("FILTER/ODR")) return false;

    /* Measurement mode with DRDY_OFF (bit2): we poll the FIFO, so the DRDY
     * pin is unused — and if the PMDZ's DRDY pad is shorted (solder bridge /
     * breadboard contact), its driver fighting the short at measurement
     * entry is exactly the observed latch-up-until-power-cycle. */
    /* 0x06 = TEMP_OFF | DRDY_OFF, measure. Every known-working example
     * (ADI no-OS, gpvidal, plasmapper) enters measurement with the temp
     * sensor DISABLED; we previously used 0x04 (temp on) — turning off the
     * temp ADC removes one analog block from the turn-on load. */
    regWrite(REG_POWER_CTL, 0x06);          /* measure, TEMP_OFF + DRDY_OFF */
    delay(20);
    if (!still("POWER_CTL=measure+DRDY_OFF (+20ms)")) {
        /* Postmortem showed a blind standby write revives the "dead" bus:
         * MISO is being CLAMPED low by a pin that drives only in measurement
         * mode. DRDY_OFF (bit2) *forces the DRDY pin LOW* in measurement —
         * so if DRDY is bridged to MISO, this is exactly the signature.
         * Discriminator: revive via standby, re-enter measurement with DRDY
         * ENABLED (0x02 = TEMP_OFF only). If reads now work, the DRDY pad
         * is shorted to MISO — a wiring fault, not a chip fault. */
        regWrite(REG_POWER_CTL, 0x01);  delay(10);
        if (reg(REG_DEVID_AD) == 0xAD) {
            /* EngineerZone "ADXL355 output stuck at zero": a known,
             * ADI-unexplained failure class where the part reads zeros until
             * a standby->measurement TOGGLE clears it — sometimes several
             * attempts. Toggle persistently, cycling POWER_CTL variants:
             * 0x06 (TEMP_OFF|DRDY_OFF), 0x02 (TEMP_OFF), 0x00 (plain). */
            Serial.println("[adxl355] revived by standby write — chip alive, reads clamp only in measurement.");
            static const uint8_t variants[] = { 0x06, 0x02, 0x00 };
            for (int attempt = 0; attempt < 9; attempt++) {
                uint8_t v = variants[attempt % 3];
                regWrite(REG_POWER_CTL, 0x01);  delay(20);   /* standby */
                regWrite(REG_POWER_CTL, v);     delay(30);   /* measure */
                uint8_t d2 = reg(REG_DEVID_AD);
                Serial.printf("[adxl355] toggle %d: measure=0x%02X -> DEVID=0x%02X %s\n",
                              attempt + 1, v, d2, d2 == 0xAD ? "ALIVE" : "clamped");
                if (d2 == 0xAD) {
                    delay(200);
                    if (still("measure survived toggle +200ms")) {
                        Serial.printf("[adxl355] >>> RECOVERED via standby/measure toggling (attempt %d, POWER_CTL=0x%02X) <<<\n",
                                      attempt + 1, v);
                        return true;
                    }
                }
            }
            Serial.println("[adxl355] 9 toggles, still clamped in measurement every time.");
        }
        return false;
    }
    delay(200);
    if (!still("measure +200ms")) return false;
    Serial.println("[adxl355] init OK: ID 0xAD/0xED, ±2g, measuring");
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
    /* low-power measurement stays on so activity engine runs (DRDY_OFF) */
    regWrite(REG_POWER_CTL, 0x06);  /* measure, TEMP_OFF+DRDY_OFF */
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

/* After a bus death: classify the failure by looking at who drives MISO.
 *  - MISO follows the pull (high w/ pullup, low w/ pulldown) with CS HIGH
 *    and stays 0x00 on reads -> chip has RELEASED the bus = internal
 *    brownout/latch (interface unpowered).
 *  - MISO pinned LOW regardless of pull -> chip (or a short) is actively
 *    driving the line = interface alive but insane, or hardware short. */
void Adxl355FrontEnd::postMortem() {
    uint8_t d100 = regAt(REG_DEVID_AD, 100000);
    uint8_t s100 = regAt(REG_STATUS, 100000);
    Serial.printf("[postmortem] @100kHz DEVID=0x%02X STATUS=0x%02X\n", d100, s100);
    if (_miso >= 0) {
        digitalWrite(_cs, HIGH);
        pinMode(_miso, INPUT_PULLUP);  delayMicroseconds(50);
        int up = digitalRead(_miso);
        pinMode(_miso, INPUT_PULLDOWN); delayMicroseconds(50);
        int dn = digitalRead(_miso);
        pinMode(_miso, INPUT);
        Serial.printf("[postmortem] MISO idle (CS high): pullup->%d pulldown->%d  %s\n",
                      up, dn,
                      (up == 1 && dn == 0) ? "= chip RELEASED bus (tri-state; interface browned out)"
                    : (up == 0 && dn == 0) ? "= line PINNED LOW (chip driving, or short to GND)"
                    : (up == 1 && dn == 1) ? "= line PINNED HIGH (short to a supply?)"
                                           : "= inconsistent");
    }
    int i1 = digitalRead(_int1);
    Serial.printf("[postmortem] INT1 level: %d\n", i1);

    /* Decisive test: the WRITE path (SCK/MOSI/CS) is unaffected by whatever
     * is clamping MISO. Blindly command standby; if reads come back, the
     * chip was never dead — a measurement-mode output (DRDY drives constant
     * LOW in measurement, esp. with DRDY_OFF) is clamping the MISO line.
     * On the PMDZ header, DRDY (pin 9) sits directly under MISO (pin 3). */
    regWrite(REG_POWER_CTL, 0x01);   /* blind: back to standby */
    delay(10);
    uint8_t d = reg(REG_DEVID_AD);
    if (d == 0xAD) {
        Serial.println("[postmortem] BLIND STANDBY WRITE REVIVED IT -> chip was never dead!");
        Serial.println("[postmortem] MISO is clamped by a measurement-mode output.");
        Serial.println("[postmortem] CHECK: short between PMDZ pin 3 (MISO) and pin 9 (DRDY)");
        Serial.println("[postmortem]        (also pins 7 INT1 / 8 INT2 / adjacent breadboard rows)");
    } else {
        Serial.printf("[postmortem] blind standby write did not revive (DEVID=0x%02X) — chip genuinely latched\n", d);
    }
}

/* ID read at an arbitrary SPI clock — used to distinguish a marginal
 * signal (works slow, fails fast) from a genuinely dead chip. */
uint8_t Adxl355FrontEnd::regAt(uint8_t addr, uint32_t hz) {
    _spi.beginTransaction(SPISettings(hz, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    _spi.transfer((addr << 1) | 0x01);
    uint8_t v = _spi.transfer(0x00);
    digitalWrite(_cs, HIGH);
    _spi.endTransaction();
    return v;
}

bool Adxl355FrontEnd::probe() {
    uint8_t devid = reg(REG_DEVID_AD);
    uint8_t partid = reg(REG_PARTID);
    bool ok = (devid == 0xAD && partid == 0xED);
    Serial.printf("[probe %8lums] DEVID_AD=0x%02X PARTID=0x%02X %s\n",
                  (unsigned long)millis(), devid, partid,
                  ok ? "OK" : "FAIL");
    if (!ok) {
        /* Retry at slower clocks: recovery here = signal-integrity problem
         * at 5 MHz (long jumpers/breadboard), not a dead chip. */
        static const uint32_t slow[] = { 1000000, 100000 };
        for (uint32_t hz : slow) {
            uint8_t d = regAt(REG_DEVID_AD, hz);
            uint8_t p = regAt(REG_PARTID, hz);
            Serial.printf("         retry @%lukHz: 0x%02X/0x%02X %s\n",
                          (unsigned long)(hz / 1000), d, p,
                          (d == 0xAD && p == 0xED) ? "OK <- marginal signal at 5MHz!"
                                                   : "FAIL");
            if (d == 0xAD && p == 0xED) return false;
        }
    }
    return ok;
}

/* Silent liveness check — no serial output. Used by the boot watchdog. */
bool Adxl355FrontEnd::alive() {
    return reg(REG_DEVID_AD) == 0xAD && reg(REG_PARTID) == 0xED;
}

/* Hold the chip in standby with a countdown (get DMM probes on the PMDZ's
 * 1.8 V decoupling caps: V1P8ANA / V1P8DIG should read ~1.8 V), then enter
 * measurement mode and report liveness every 500 ms. If a 1.8 V rail sags
 * or collapses at entry, the eval board's LDO decoupling is the fault. */
void Adxl355FrontEnd::measureEntryProbe(uint32_t countdown_s) {
    uint8_t d = reg(REG_DEVID_AD);
    if (d != 0xAD) {
        Serial.printf("[probe] chip not answering (DEVID=0x%02X) — power cycle first\n", d);
        return;
    }
    regWrite(REG_RESET, 0x52); delay(30);
    regWrite(REG_POWER_CTL, 0x01);   /* standby */
    Serial.println("[probe] chip in STANDBY. Put DMM on a 1.8V decoupling cap (V1P8ANA/V1P8DIG).");
    Serial.println("[probe] Expect ~1.8 V now. Watch it as measurement mode starts.");
    for (uint32_t s = countdown_s; s > 0; s--) {
        Serial.printf("[probe] entering measurement in %lus... (DEVID=0x%02X)\n",
                      (unsigned long)s, reg(REG_DEVID_AD));
        delay(1000);
    }
    Serial.println("[probe] >>> POWER_CTL = measure NOW <<<");
    regWrite(REG_POWER_CTL, 0x06);  /* measure, TEMP_OFF+DRDY_OFF */
    for (int i = 0; i < 20; i++) {
        delay(500);
        uint8_t dd = reg(REG_DEVID_AD);
        Serial.printf("[probe] +%4dms DEVID=0x%02X %s\n", (i + 1) * 500, dd,
                      dd == 0xAD ? "alive" : "DEAD");
    }
    regWrite(REG_POWER_CTL, 0x01);   /* try to return to standby */
}

void Adxl355FrontEnd::powerDown() {
    regWrite(REG_POWER_CTL, 0x01); /* standby; Ve will be cut anyway */
}
