#include "StaLta.h"
#include <math.h>

/* SPS event class values duplicated here to keep the detector dependency-free
   for host testing; must match sps_proto.h. */
static const uint8_t EV_UNKNOWN = 0, EV_FOOTSTEP = 1, EV_VEHICLE = 2, EV_MULTIPLE = 3;

static float alphaFor(float fc, float fs) {
    /* one-pole coefficient for cutoff fc at sample rate fs */
    float x = 2.0f * 3.14159265f * fc / fs;
    return x / (x + 1.0f);
}

float StaLta::OnePoleHP::run(float x) {
    y = (1.0f - a) * (y + x - xprev);
    xprev = x;
    return y;
}
void StaLta::OnePoleHP::set(float fc, float fs) { a = alphaFor(fc, fs); y = 0; xprev = 0; }

float StaLta::OnePoleLP::run(float x) {
    y += a * (x - y);
    return y;
}
void StaLta::OnePoleLP::set(float fc, float fs) { a = alphaFor(fc, fs); y = 0; }

void StaLta::Band::set(float lo, float hi, float fs, float energyWindowS) {
    hp.set(lo, fs);
    lp.set(hi, fs);
    eAlpha = 1.0f / (energyWindowS * fs);
    energy = 0; peak = 0;
}
void StaLta::Band::run(float x) {
    float b = lp.run(hp.run(x));
    float r = fabsf(b);
    energy += eAlpha * (r - energy);
    if (r > peak) peak = r;
}

void StaLta::begin(const DetectorConfig &cfg) {
    _cfg = cfg;
    float fs = (float)cfg.sample_rate_hz;
    _hpf.set(cfg.hpf_hz, fs);
    _staAlpha = 1000.0f / ((float)cfg.sta_ms * fs);
    _ltaAlpha = 1000.0f / ((float)cfg.lta_ms * fs);
    _sta = _lta = 0;
    _foot.set(cfg.footstep_lo_hz, cfg.footstep_hi_hz, fs, 0.5f);
    _veh.set(cfg.vehicle_lo_hz, cfg.vehicle_hi_hz, fs, 0.5f);
    _inEvent = false;
    _holdoff = 0;
}

DetectionResult StaLta::update(int16_t sample) {
    DetectionResult res = { false, EV_UNKNOWN, 0, 0 };

    float x = _hpf.run((float)sample);
    float r = fabsf(x);

    _sta += _staAlpha * (r - _sta);
    /* Freeze LTA while an event is in progress so the event doesn't
       raise its own baseline and self-cancel. */
    if (!_inEvent && _holdoff == 0)
        _lta += _ltaAlpha * (r - _lta);
    if (_holdoff > 0) _holdoff--;

    _foot.run((float)sample);
    _veh.run((float)sample);

    float ratio = (_lta > 1e-3f) ? (_sta / _lta) : 0.0f;

    if (!_inEvent && ratio >= _cfg.trigger_ratio && _lta > 1e-3f) {
        _inEvent = true;
        res.triggered = true;

        /* Classify by band energy dominance (spec: footstep 20-80, vehicle 2-20) */
        float fe = _foot.energy, ve = _veh.energy;
        if (fe > 2.0f * ve)       res.event_class = EV_FOOTSTEP;
        else if (ve > 2.0f * fe)  res.event_class = EV_VEHICLE;
        else if (fe > 0 || ve > 0) res.event_class = EV_MULTIPLE;
        else                       res.event_class = EV_UNKNOWN;

        /* Confidence: ratio scaled so trigger_ratio -> ~64, 4x trigger -> 255 */
        float scaled = (ratio / _cfg.trigger_ratio) * 64.0f;
        res.confidence = (uint8_t)(scaled > 255.0f ? 255.0f : scaled);

        float pk = _foot.peak > _veh.peak ? _foot.peak : _veh.peak;
        res.peak_amp = (uint16_t)(pk > 65535.0f ? 65535 : pk);
        _foot.resetPeak();
        _veh.resetPeak();
    } else if (_inEvent && ratio < _cfg.trigger_ratio * 0.5f) {
        _inEvent = false;
        /* Hold LTA frozen briefly after the event tail */
        _holdoff = _cfg.sample_rate_hz; /* 1 s */
    }
    return res;
}

uint16_t StaLta::noiseFloor() const {
    float v = _lta;
    return (uint16_t)(v > 65535.0f ? 65535 : v);
}
