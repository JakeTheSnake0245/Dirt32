#include "config.h"
#include <Preferences.h>
#include <ArduinoJson.h>

static const char *NVS_NS = "sps";
static const char *NVS_KEY = "cfg";

static bool hexToBytes(const String &hex, uint8_t *out, size_t len) {
    if (hex.length() != len * 2) return false;
    for (size_t i = 0; i < len; i++) {
        char buf[3] = { hex[2 * i], hex[2 * i + 1], 0 };
        char *end;
        long v = strtol(buf, &end, 16);
        if (*end != 0) return false;
        out[i] = (uint8_t)v;
    }
    return true;
}

void configLoad(NodeConfig &cfg) {
    Preferences p;
    if (!p.begin(NVS_NS, true)) return;
    String json = p.getString(NVS_KEY, "");
    p.end();
    if (json.isEmpty()) return;

    JsonDocument doc;
    if (deserializeJson(doc, json) != DeserializationError::Ok) return;

    cfg.node_id = doc["node_id"] | cfg.node_id;
    cfg.net_id = doc["net_id"] | cfg.net_id;
    if (doc["psk"].is<const char*>())
        hexToBytes(doc["psk"].as<String>(), cfg.psk, SPS_KEY_LEN);
    cfg.f_in_mhz = doc["f_in"] | cfg.f_in_mhz;
    cfg.sf = doc["sf"] | cfg.sf;
    cfg.bw_khz = doc["bw"] | cfg.bw_khz;
    cfg.cr = doc["cr"] | cfg.cr;
    cfg.tx_power_dbm = doc["tx_power"] | cfg.tx_power_dbm;
    cfg.heartbeat_per_day = doc["heartbeat_per_day"] | cfg.heartbeat_per_day;
    cfg.sample_rate_hz = doc["sample_rate_hz"] | cfg.sample_rate_hz;
    cfg.hpf_hz = doc["hpf_hz"] | cfg.hpf_hz;
    cfg.sta_ms = doc["sta_ms"] | cfg.sta_ms;
    cfg.lta_ms = doc["lta_ms"] | cfg.lta_ms;
    cfg.trigger_ratio = doc["trigger_ratio"] | cfg.trigger_ratio;
    cfg.footstep_lo_hz = doc["footstep_lo"] | cfg.footstep_lo_hz;
    cfg.footstep_hi_hz = doc["footstep_hi"] | cfg.footstep_hi_hz;
    cfg.vehicle_lo_hz = doc["vehicle_lo"] | cfg.vehicle_lo_hz;
    cfg.vehicle_hi_hz = doc["vehicle_hi"] | cfg.vehicle_hi_hz;
    cfg.retx_count = doc["retx_count"] | cfg.retx_count;
    cfg.retx_jitter_min_ms = doc["retx_jitter_min"] | cfg.retx_jitter_min_ms;
    cfg.retx_jitter_max_ms = doc["retx_jitter_max"] | cfg.retx_jitter_max_ms;
    cfg.ack_enable = doc["ack_enable"] | cfg.ack_enable;
    cfg.ack_window_ms = doc["ack_window_ms"] | cfg.ack_window_ms;
    cfg.front_end = (FrontEndType)(int)(doc["front_end"] | (int)cfg.front_end);
    cfg.motion_wake_enable = doc["motion_wake_enable"] | cfg.motion_wake_enable;
    cfg.motion_threshold_g = doc["motion_threshold"] | cfg.motion_threshold_g;
    cfg.gps_enable = doc["gps_enable"] | cfg.gps_enable;
    cfg.gps_fix_timeout_s = doc["gps_fix_timeout_s"] | cfg.gps_fix_timeout_s;
    cfg.fallback_lat_e7 = doc["fallback_lat_e7"] | cfg.fallback_lat_e7;
    cfg.fallback_lon_e7 = doc["fallback_lon_e7"] | cfg.fallback_lon_e7;
}

static void toJson(const NodeConfig &cfg, JsonDocument &doc, bool includePsk) {
    doc["node_id"] = cfg.node_id;
    doc["net_id"] = cfg.net_id;
    if (includePsk) {
        char hex[SPS_KEY_LEN * 2 + 1];
        for (size_t i = 0; i < SPS_KEY_LEN; i++) sprintf(hex + 2 * i, "%02x", cfg.psk[i]);
        doc["psk"] = hex;
    } else {
        bool set = false;
        for (size_t i = 0; i < SPS_KEY_LEN; i++) if (cfg.psk[i]) set = true;
        doc["psk"] = set ? "(set, redacted)" : "(NOT SET)";
    }
    doc["f_in"] = cfg.f_in_mhz;
    doc["sf"] = cfg.sf;
    doc["bw"] = cfg.bw_khz;
    doc["cr"] = cfg.cr;
    doc["tx_power"] = cfg.tx_power_dbm;
    doc["heartbeat_per_day"] = cfg.heartbeat_per_day;
    doc["sample_rate_hz"] = cfg.sample_rate_hz;
    doc["hpf_hz"] = cfg.hpf_hz;
    doc["sta_ms"] = cfg.sta_ms;
    doc["lta_ms"] = cfg.lta_ms;
    doc["trigger_ratio"] = cfg.trigger_ratio;
    doc["footstep_lo"] = cfg.footstep_lo_hz;
    doc["footstep_hi"] = cfg.footstep_hi_hz;
    doc["vehicle_lo"] = cfg.vehicle_lo_hz;
    doc["vehicle_hi"] = cfg.vehicle_hi_hz;
    doc["retx_count"] = cfg.retx_count;
    doc["retx_jitter_min"] = cfg.retx_jitter_min_ms;
    doc["retx_jitter_max"] = cfg.retx_jitter_max_ms;
    doc["ack_enable"] = cfg.ack_enable;
    doc["ack_window_ms"] = cfg.ack_window_ms;
    doc["front_end"] = (int)cfg.front_end;
    doc["motion_wake_enable"] = cfg.motion_wake_enable;
    doc["motion_threshold"] = cfg.motion_threshold_g;
    doc["gps_enable"] = cfg.gps_enable;
    doc["gps_fix_timeout_s"] = cfg.gps_fix_timeout_s;
    doc["fallback_lat_e7"] = cfg.fallback_lat_e7;
    doc["fallback_lon_e7"] = cfg.fallback_lon_e7;
}

bool configSave(const NodeConfig &cfg) {
    JsonDocument doc;
    toJson(cfg, doc, true);
    String json;
    serializeJson(doc, json);
    Preferences p;
    if (!p.begin(NVS_NS, false)) return false;
    bool ok = p.putString(NVS_KEY, json) > 0;
    p.end();
    return ok;
}

void configPrint(const NodeConfig &cfg, Stream &out) {
    JsonDocument doc;
    toJson(cfg, doc, false);
    serializeJsonPretty(doc, out);
    out.println();
}

bool configSet(NodeConfig &cfg, const String &key, const String &value) {
    if (key == "psk") return hexToBytes(value, cfg.psk, SPS_KEY_LEN);
    if (key == "node_id") { cfg.node_id = value.toInt(); return true; }
    if (key == "net_id") { cfg.net_id = value.toInt(); return true; }
    if (key == "f_in") { cfg.f_in_mhz = value.toFloat(); return true; }
    if (key == "sf") { int v = value.toInt(); if (v < 7 || v > 12) return false; cfg.sf = v; return true; }
    if (key == "bw") { cfg.bw_khz = value.toFloat(); return true; }
    if (key == "cr") { int v = value.toInt(); if (v < 5 || v > 8) return false; cfg.cr = v; return true; }
    if (key == "tx_power") { int v = value.toInt(); if (v < -9 || v > 22) return false; cfg.tx_power_dbm = v; return true; }
    if (key == "heartbeat_per_day") { cfg.heartbeat_per_day = value.toInt(); return true; }
    if (key == "sample_rate_hz") { cfg.sample_rate_hz = value.toInt(); return true; }
    if (key == "hpf_hz") { cfg.hpf_hz = value.toFloat(); return true; }
    if (key == "sta_ms") { cfg.sta_ms = value.toInt(); return true; }
    if (key == "lta_ms") { cfg.lta_ms = value.toInt(); return true; }
    if (key == "trigger_ratio") { cfg.trigger_ratio = value.toFloat(); return true; }
    if (key == "footstep_lo") { cfg.footstep_lo_hz = value.toFloat(); return true; }
    if (key == "footstep_hi") { cfg.footstep_hi_hz = value.toFloat(); return true; }
    if (key == "vehicle_lo") { cfg.vehicle_lo_hz = value.toFloat(); return true; }
    if (key == "vehicle_hi") { cfg.vehicle_hi_hz = value.toFloat(); return true; }
    if (key == "retx_count") { cfg.retx_count = value.toInt(); return true; }
    if (key == "retx_jitter_min") { cfg.retx_jitter_min_ms = value.toInt(); return true; }
    if (key == "retx_jitter_max") { cfg.retx_jitter_max_ms = value.toInt(); return true; }
    if (key == "ack_enable") { cfg.ack_enable = value.toInt() != 0; return true; }
    if (key == "ack_window_ms") { cfg.ack_window_ms = value.toInt(); return true; }
    if (key == "front_end") {
        if (value == "adxl355" || value == "0") { cfg.front_end = FE_ADXL355; return true; }
        if (value == "geophone" || value == "1") { cfg.front_end = FE_GEOPHONE; return true; }
        return false;
    }
    if (key == "motion_wake_enable") { cfg.motion_wake_enable = value.toInt() != 0; return true; }
    if (key == "motion_threshold") { cfg.motion_threshold_g = value.toFloat(); return true; }
    if (key == "gps_enable") { cfg.gps_enable = value.toInt() != 0; return true; }
    if (key == "gps_fix_timeout_s") { cfg.gps_fix_timeout_s = value.toInt(); return true; }
    if (key == "fallback_lat_e7") { cfg.fallback_lat_e7 = value.toInt(); return true; }
    if (key == "fallback_lon_e7") { cfg.fallback_lon_e7 = value.toInt(); return true; }
    return false;
}
