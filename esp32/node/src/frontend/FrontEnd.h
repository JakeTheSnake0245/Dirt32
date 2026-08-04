/*
 * FrontEnd.h — seismic front-end abstraction (spec §5.1 option A/B).
 * Both drivers deliver vertical-axis samples in a common normalized unit
 * so the STA/LTA detector is front-end agnostic.
 */
#pragma once
#include <stdint.h>
#include <stddef.h>

class FrontEnd {
public:
    virtual ~FrontEnd() {}

    /* Power up (Ve rail must already be on) and configure for sampling at
       the given rate. Returns false on comms failure (chip ID mismatch etc). */
    virtual bool begin(uint16_t sample_rate_hz) = 0;

    /* Read up to `max` pending vertical-axis samples into `out` (normalized
       to signed 16-bit full scale). Returns number of samples read. */
    virtual size_t read(int16_t *out, size_t max) = 0;

    /* True if this front-end supports hardware motion wake (deep-sleep path). */
    virtual bool supportsMotionWake() const = 0;

    /* Arm the low-power motion interrupt before deep sleep. threshold_g in g.
       Returns the GPIO to use as an ext wake source, or -1 if unsupported. */
    virtual int armMotionWake(float threshold_g) = 0;

    /* Run self-test / sanity check; used for HEALTH_FLAGS bit2. */
    virtual bool selfTest() = 0;

    /* Lightweight comms probe: read chip ID(s), print a one-line verdict.
       Used by the `sensorid` CLI diagnostic to watch for a bus that dies
       some time after boot. Returns true if the chip answered correctly. */
    virtual bool probe() = 0;

    /* Quiesce the chip before Ve is cut. */
    virtual void powerDown() = 0;
};
