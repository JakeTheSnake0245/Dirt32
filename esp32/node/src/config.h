/*
 * config.h — runtime configuration (spec §5.3 / §11).
 * Stored as JSON in NVS ("sps" namespace), editable over USB-serial CLI.
 * Every parameter is field-programmable without reflashing.
 */
#pragma once
#include <Arduino.h>
#include <stdint.h>
#include "sps_proto.h"

enum FrontEndType : uint8_t { FE_ADXL355 = 0, FE_GEOPHONE = 1 };

struct NodeConfig {
    /* Identity & security */
    uint16_t node_id         = 1;
    uint8_t  net_id          = 0x01;
    uint8_t  psk[SPS_KEY_LEN] = {0};   /* set via CLI: psk <64 hex chars> */

    /* RF (spec §3) */
    float    f_in_mhz        = 903.0f;
    uint8_t  sf              = 10;     /* SF7..SF12 */
    float    bw_khz          = 125.0f;
    uint8_t  cr              = 5;      /* 4/5 */
    int8_t   tx_power_dbm    = 20;     /* max 22; buried nodes need margin */

    /* Schedule */
    uint8_t  heartbeat_per_day = 24;   /* hourly liveness for a security perimeter */

    /* Detection (spec §5.2) */
    uint16_t sample_rate_hz  = 250;    /* 250-500 */
    float    hpf_hz          = 2.0f;   /* 1-8 */
    uint16_t sta_ms          = 200;
    uint16_t lta_ms          = 5000;
    float    trigger_ratio   = 4.0f;
    float    footstep_lo_hz  = 20.0f, footstep_hi_hz = 80.0f;
    float    vehicle_lo_hz   = 2.0f,  vehicle_hi_hz  = 20.0f;

    /* Reliability (spec §4.6) */
    uint8_t  retx_count      = 3;
    uint16_t retx_jitter_min_ms = 200;
    uint16_t retx_jitter_max_ms = 800;
    bool     ack_enable      = true;   /* v2 closed-loop */
    uint16_t ack_window_ms   = 1500;   /* SF10/125k ACK airtime ≈ 400 ms + turnaround */

    /* Front-end */
    FrontEndType front_end   = FE_ADXL355;
    bool     motion_wake_enable = true;
    float    motion_threshold_g = 0.005f;

    /* GPS */
    bool     gps_enable      = true;
    /* Solar sense: GPIO reading the panel/5V rail through a divider
       (e.g. 100k/100k). -1 = disabled. When high, the heartbeat sets the
       ON_SOLAR health flag. Charging itself is pure hardware — the V4's
       charge IC needs no firmware involvement. */
    int8_t   solar_sense_gpio = -1;
    uint16_t gps_fix_timeout_s = 150;  /* L76K cold-start up to ~2 min */

    /* Fallback position (used when no GPS fix; provisioning record §11) */
    int32_t  fallback_lat_e7 = 0;
    int32_t  fallback_lon_e7 = 0;
};

/* Load from NVS; returns defaults for anything unset. */
void configLoad(NodeConfig &cfg);
/* Persist to NVS as JSON. */
bool configSave(const NodeConfig &cfg);
/* Print (PSK redacted) as JSON to a stream. */
void configPrint(const NodeConfig &cfg, Stream &out);
/* Apply "set <key> <value>" from the CLI. Returns false on unknown key/bad value. */
bool configSet(NodeConfig &cfg, const String &key, const String &value);
