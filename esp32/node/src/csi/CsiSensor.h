/*
 * CsiSensor.h — WiFi radar (CSI) sensing front-end (task: WiFi Radar).
 *
 * The ESP32-S3's WiFi radio captures Channel State Information (CSI) from
 * received WiFi frames; motion between/around TX and RX perturbs the
 * per-subcarrier amplitudes. An espectre-style detector (baseline
 * calibration + moving variance + hysteresis/hold-off) turns that into
 * presence events, complementary to the seismic channel.
 *
 * Traffic source: CSI needs frames to measure. A low-rate ESP-NOW
 * broadcast ping (csi_ping_hz) lets any pair of Dirt32 nodes form a
 * TX->RX sensing link with no router around; ambient AP traffic is also
 * captured when present (the CSI callback sees every frame the radio
 * decodes on the configured channel).
 *
 * Compile-time gate: SPS_CSI_ENABLE (platformio.ini). Runtime gate:
 * cfg.csi_enable. CSI keeps the WiFi radio in RX continuously, so it is
 * incompatible with the deep-sleep ADXL profile — enabling it holds the
 * node in continuous-listen mode (like the geophone profile).
 */
#pragma once
#include <stdint.h>
#include "../config.h"

#if SPS_CSI_ENABLE

struct CsiEvent {
    bool     triggered = false;
    uint8_t  confidence = 0;    /* metric/threshold scaled, 0-255 */
    uint16_t metric_x100 = 0;   /* motion metric x100 for ALERT peak_amp */
};

class CsiSensor {
public:
    /* Bring up WiFi (STA, not associated), register the CSI callback and
       start ESP-NOW pings per cfg.csi_role. Returns false on esp-idf error. */
    bool begin(const NodeConfig &cfg);
    void end();

    /* Call from loop(): sends due ESP-NOW pings. */
    void service();

    /* Run the detector over frames received since the last call.
       Hysteresis + hold-off: one clean event per passing person. */
    CsiEvent poll();

    /* Heartbeat/health accessors */
    bool     running() const     { return _running; }
    bool     calibrating() const { return _calibSamples < _calibNeeded; }
    uint16_t noiseX100() const;      /* quiescent baseline metric x100 */
    float    metric() const      { return _lastMetric; }  /* for CLI tuning */
    uint32_t frameCount() const  { return _frames; }
    uint32_t dropCount() const   { return _drops; }   /* ring overruns */

    /* Suspend/resume ping TX around LoRa activity (coexistence: keeps the
       radio quiet while an alert burst + ACK window is in flight). */
    void pauseTraffic(bool paused) { _paused = paused; }

    /* Restart baseline calibration (gateway CSI_RECAL command or local
       re-baseline). Keep the area clear for csi_calib_s afterwards. */
    void recalibrate() {
        _baseline = 0; _calibSamples = 0;
        _winLen = 0; _winPos = 0;
        _inEvent = false; _holdoffUntil = 0;
        _lastMetric = 0;
    }

    /* Called from the CSI RX callback (WiFi task context) — one per-frame
       spatial-turbulence value into the ring. Not for application use. */
    void ingestTurbulence(float t) {
        _frames = _frames + 1;
        uint16_t h = _head;
        uint16_t next = (uint16_t)((h + 1) % RING);
        if (next == _tail) {          /* ring full: drop-newest, count it */
            _drops = _drops + 1;
            return;
        }
        _ring[h] = t;
        _head = next;
    }

private:
    /* config (copied at begin) */
    uint8_t  _role = 2;          /* 0=rx-only 1=tx-only 2=both */
    float    _threshold = 2.0f;  /* trigger at metric >= threshold */
    uint16_t _window = 64;       /* moving-variance window, frames */
    uint16_t _calibNeeded = 0;   /* frames of baseline calibration */
    uint32_t _holdoffMs = 5000;
    uint32_t _pingIntervalMs = 100;

    /* state */
    bool     _running = false;
    bool     _paused = false;
    uint32_t _lastPingMs = 0;
    volatile uint32_t _frames = 0;
    volatile uint32_t _drops = 0;   /* ring-full drops (calibration validity) */

    /* ring of per-frame spatial turbulence values (written from WiFi task,
       read from loop; single-writer single-reader, index race is benign) */
    static const uint16_t RING = 256;
    volatile float    _ring[RING];
    volatile uint16_t _head = 0;
    uint16_t _tail = 0;

    /* detector */
    float    _baseline = 0;      /* EMA of quiescent moving variance */
    uint32_t _calibSamples = 0;
    float    _winBuf[128];       /* window <= 128 */
    uint16_t _winLen = 0, _winPos = 0;
    float    _lastMetric = 0;
    bool     _inEvent = false;
    uint32_t _holdoffUntil = 0;
};

#endif /* SPS_CSI_ENABLE */
