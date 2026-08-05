/*
 * CsiSensor.cpp — WiFi CSI capture + ESP-NOW ping traffic + espectre-style
 * motion detector. See CsiSensor.h for the design overview.
 *
 * Pipeline (per received WiFi frame, in the WiFi task):
 *   CSI callback -> per-subcarrier amplitudes (LLTF) ->
 *   spatial turbulence = std(amp)/mean(amp) over mid subcarriers ->
 *   ring buffer
 * and (in loop(), via poll()):
 *   moving variance of turbulence over csi_window frames ->
 *   metric = variance / calibrated baseline ->
 *   trigger at csi_threshold, release at 0.5x, hold-off csi_holdoff_s.
 *
 * Coexistence: LoRa (SX1262 on SPI) and WiFi are separate radios — the
 * only shared resources are the CPU, the 3V3 rail, and physical antenna
 * proximity. pauseTraffic() stops our own ESP-NOW pings during LoRa
 * TX/ACK windows so the two transmitters never key up together (mostly a
 * peak-current precaution on battery/solar nodes).
 */
#include "CsiSensor.h"

#if SPS_CSI_ENABLE

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <math.h>
#include <string.h>

static CsiSensor *s_instance = nullptr;
static const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/* CSI RX callback — WiFi task context. Keep it short: amplitude math on
 * ~64 subcarriers and one ring write. */
static void IRAM_ATTR csi_rx_cb(void *ctx, wifi_csi_info_t *info) {
    (void)ctx;
    if (!s_instance || !info || !info->buf || info->len < 8) return;

    /* info->buf is interleaved int8 imag/real pairs. Use the middle
     * subcarriers (skip guard/DC region at both ends). */
    const int8_t *b = info->buf;
    int pairs = info->len / 2;
    int lo = pairs / 8, hi = pairs - pairs / 8;
    if (hi - lo < 8) { lo = 0; hi = pairs; }

    float sum = 0, sum2 = 0;
    int n = 0;
    for (int i = lo; i < hi; i++) {
        float im = (float)b[2 * i], re = (float)b[2 * i + 1];
        float amp = sqrtf(re * re + im * im);
        sum += amp; sum2 += amp * amp; n++;
    }
    if (n < 4) return;
    float mean = sum / n;
    if (mean < 1e-3f) return;
    float var = sum2 / n - mean * mean;
    if (var < 0) var = 0;
    /* Spatial turbulence: coefficient of variation across subcarriers —
     * scale-free, so RSSI/AGC changes largely cancel out. */
    s_instance->ingestTurbulence(sqrtf(var) / mean);
}

bool CsiSensor::begin(const NodeConfig &cfg) {
    _role = cfg.csi_role;
    _threshold = cfg.csi_threshold;
    _window = cfg.csi_window_frames;
    if (_window > 128) _window = 128;
    if (_window < 8) _window = 8;
    _holdoffMs = (uint32_t)cfg.csi_holdoff_s * 1000UL;
    _pingIntervalMs = cfg.csi_ping_hz ? (1000UL / cfg.csi_ping_hz) : 100;
    /* calibration: csi_calib_s worth of frames at the expected ping rate,
       min 4 windows so the baseline is a real quiescent estimate */
    _calibNeeded = (uint32_t)cfg.csi_calib_s * (cfg.csi_ping_hz ? cfg.csi_ping_hz : 10);
    if (_calibNeeded < (uint32_t)_window * 4) _calibNeeded = (uint32_t)_window * 4;
    _baseline = 0; _calibSamples = 0;
    _winLen = 0; _winPos = 0; _inEvent = false; _holdoffUntil = 0;
    _head = 0; _tail = 0; _frames = 0;

    WiFi.mode(WIFI_STA);              /* up, but never associates */
    esp_wifi_set_ps(WIFI_PS_NONE);    /* CSI RX needs the radio always on */
    /* CRITICAL: an unassociated STA does not pass received frames to the
     * CSI hook — without promiscuous RX the callback never fires and
     * frames stays at 0 forever (bench-verified). Espressif's esp-csi
     * examples enable promiscuous mode for exactly this reason. */
    esp_wifi_set_promiscuous(true);
    wifi_promiscuous_filter_t filt = {};
    filt.filter_mask = WIFI_PROMIS_FILTER_MASK_DATA | WIFI_PROMIS_FILTER_MASK_MGMT;
    esp_wifi_set_promiscuous_filter(&filt);
    esp_wifi_set_channel(cfg.csi_wifi_channel, WIFI_SECOND_CHAN_NONE);

    /* CSI capture: match esp-csi example config — all LTF sources on,
     * merged, no channel filter. ESP-NOW pings arrive as legacy frames;
     * ambient 11n traffic contributes HT-LTF vectors. */
    wifi_csi_config_t csi_cfg = {};
    csi_cfg.lltf_en = true;
    csi_cfg.htltf_en = true;
    csi_cfg.stbc_htltf2_en = true;
    csi_cfg.ltf_merge_en = true;
    csi_cfg.channel_filter_en = false;
    csi_cfg.manu_scale = false;
    s_instance = this;
    if (esp_wifi_set_csi_config(&csi_cfg) != ESP_OK) return false;
    if (esp_wifi_set_csi_rx_cb(&csi_rx_cb, nullptr) != ESP_OK) return false;
    if (esp_wifi_set_csi(true) != ESP_OK) return false;

    /* ESP-NOW ping TX (roles tx/both). RX-only nodes still measure every
     * frame the radio decodes — pings from peer nodes or ambient AP traffic. */
    if (_role != 0) {
        if (esp_now_init() != ESP_OK) return false;
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, BCAST, 6);
        peer.channel = cfg.csi_wifi_channel;
        peer.ifidx = WIFI_IF_STA;
        peer.encrypt = false;
        esp_now_add_peer(&peer);
    }

    _running = true;
    return true;
}

void CsiSensor::end() {
    if (!_running) return;
    esp_wifi_set_csi(false);
    esp_wifi_set_csi_rx_cb(nullptr, nullptr);
    if (_role != 0) esp_now_deinit();
    s_instance = nullptr;
    WiFi.mode(WIFI_OFF);
    _running = false;
}

void CsiSensor::service() {
    if (!_running || _paused || _role == 0) return;
    uint32_t now = millis();
    if (now - _lastPingMs < _pingIntervalMs) return;
    _lastPingMs = now;
    /* Tiny payload — the frame itself is the measurement, content is
     * irrelevant. 8 bytes keeps airtime ~ a few hundred µs. */
    static uint32_t n = 0;
    uint8_t buf[8] = {'D', '3', '2', 'c'};
    memcpy(buf + 4, &n, 4);
    n++;
    esp_now_send(BCAST, buf, sizeof(buf));
}

uint16_t CsiSensor::noiseX100() const {
    float v = _baseline * 100.0f;
    return (uint16_t)(v > 65535.0f ? 65535 : (v < 0 ? 0 : v));
}

CsiEvent CsiSensor::poll() {
    CsiEvent ev;
    if (!_running) return ev;

    /* Drain the ring into the moving window. */
    uint16_t head = _head;
    while (_tail != head) {
        float t = _ring[_tail];
        _tail = (uint16_t)((_tail + 1) % RING);
        _winBuf[_winPos] = t;
        _winPos = (uint16_t)((_winPos + 1) % _window);
        if (_winLen < _window) { _winLen++; continue; }

        /* Full window: moving variance of turbulence. */
        float s = 0, s2 = 0;
        for (uint16_t i = 0; i < _window; i++) { s += _winBuf[i]; s2 += _winBuf[i] * _winBuf[i]; }
        float mean = s / _window;
        float var = s2 / _window - mean * mean;
        if (var < 0) var = 0;

        if (_calibSamples < _calibNeeded) {
            /* Baseline calibration: EMA of quiescent variance. */
            _baseline = (_calibSamples == 0)
                            ? var : _baseline + 0.02f * (var - _baseline);
            _calibSamples++;
            continue;
        }

        float metric = (_baseline > 1e-9f) ? var / _baseline : 0.0f;
        _lastMetric = metric;

        uint32_t now = millis();
        if (!_inEvent && now >= _holdoffUntil && metric >= _threshold) {
            _inEvent = true;
            ev.triggered = true;
            float scaled = (metric / _threshold) * 64.0f;
            ev.confidence = (uint8_t)(scaled > 255.0f ? 255.0f : scaled);
            float m100 = metric * 100.0f;
            ev.metric_x100 = (uint16_t)(m100 > 65535.0f ? 65535 : m100);
        } else if (_inEvent && metric < _threshold * 0.5f) {
            /* Hysteresis release + hold-off: one event per passing person,
               and the disturbance never re-triggers off its own tail. */
            _inEvent = false;
            _holdoffUntil = now + _holdoffMs;
        }

        /* Track slow environment drift while quiet (very slow EMA so a
           standing person doesn't get absorbed quickly). */
        if (!_inEvent && metric < 1.2f)
            _baseline += 0.001f * (var - _baseline);
    }
    return ev;
}

#endif /* SPS_CSI_ENABLE */
