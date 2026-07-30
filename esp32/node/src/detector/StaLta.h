/*
 * StaLta.h — STA/LTA trigger + band-split classifier (spec §5.2).
 * Pipeline: high-pass -> rectified running averages (STA, LTA) -> ratio
 * trigger -> footstep/vehicle band-energy classification.
 * Pure C++ with no hardware deps so it can be unit-tested on host.
 */
#pragma once
#include <stdint.h>
#include <stddef.h>

struct DetectorConfig {
    uint16_t sample_rate_hz;
    float    hpf_hz;
    uint16_t sta_ms;
    uint16_t lta_ms;
    float    trigger_ratio;
    float    footstep_lo_hz, footstep_hi_hz;   /* ~20-80 Hz */
    float    vehicle_lo_hz,  vehicle_hi_hz;    /* ~2-20 Hz  */
};

struct DetectionResult {
    bool     triggered;
    uint8_t  event_class;     /* SPS_EV_* */
    uint8_t  confidence;      /* STA/LTA ratio scaled 0-255 */
    uint16_t peak_amp;        /* peak band-passed |amplitude| during event */
};

class StaLta {
public:
    void begin(const DetectorConfig &cfg);

    /* Feed one sample; returns a result with triggered=true exactly once per
       event (on the rising edge of the STA/LTA ratio crossing). */
    DetectionResult update(int16_t sample);

    /* Ambient baseline for HEARTBEAT NOISE_FLOOR — long-term average of the
       rectified high-passed signal. */
    uint16_t noiseFloor() const;

    /* Current ratio, for the CLI `detector` debug stream. */
    float ratio() const { return _lta > 1e-6f ? _sta / _lta : 0.0f; }

private:
    /* one-pole filters */
    struct OnePoleHP { float a = 0, y = 0, xprev = 0; float run(float x); void set(float fc, float fs); };
    struct OnePoleLP { float a = 0, y = 0;            float run(float x); void set(float fc, float fs); };
    /* band-pass = HP(lo) then LP(hi) */
    struct Band {
        OnePoleHP hp; OnePoleLP lp;
        float energy = 0, eAlpha = 0;
        float peak = 0;
        void set(float lo, float hi, float fs, float energyWindowS);
        void run(float x);
        void resetPeak() { peak = 0; }
    };

    DetectorConfig _cfg{};
    OnePoleHP _hpf;
    float _sta = 0, _lta = 0;
    float _staAlpha = 0, _ltaAlpha = 0;
    Band _foot, _veh;
    bool _inEvent = false;
    uint32_t _holdoff = 0;      /* samples remaining before LTA updates resume */
};
