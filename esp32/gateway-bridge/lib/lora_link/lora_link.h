/*
 * lora_link.h — COBS/CRC-16 binary framing for the LoRa gateway serial link.
 *
 * Arduino adaptation of lora_link.c (uses Serial.write instead of
 * uart_write_bytes; otherwise byte-for-byte identical wire format).
 *
 * WIRE FORMAT (before COBS), all little-endian:
 *   LEN     u8      payload length (0..255)
 *   TYPE    u8      message type (see LL_TYPE_*)
 *   PAYLOAD LEN     opaque bytes
 *   RSSI    i16 LE  dBm  (0 for Pi→Bridge frames)
 *   SNR     i8      dB   (0 for Pi→Bridge frames)
 *   CRC16   u16 LE  CRC-16/CCITT-FALSE over LEN..SNR
 * On the wire: COBS(frame) + 0x00
 * A 0x00 byte is the ONLY frame boundary; COBS removes all 0x00 from the body.
 */
#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

// ---- Message types -------------------------------------------------------
#define LL_TYPE_ALERT      0x01u  // Bridge→Pi: received LoRa frame (any SPS type)
#define LL_TYPE_HEARTBEAT  0x02u  // Bridge→Pi: received LoRa heartbeat (alias)
#define LL_TYPE_TX         0x03u  // Pi→Bridge: transmit this payload via radio
#define LL_TYPE_CFG        0x10u  // Pi→Bridge: radio config (sizeof(LLCfg)==12)
#define LL_TYPE_PING       0x11u  // Pi→Bridge: keepalive, no payload
#define LL_TYPE_RDY        0x20u  // Bridge→Pi: bridge ready / rebooted
#define LL_TYPE_OK         0x21u  // Bridge→Pi: last command succeeded
#define LL_TYPE_ERR        0x22u  // Bridge→Pi: error

// ---- CFG payload (12 bytes, packed little-endian) ----------------------
// Must match serial_framer.py's _CFG_FMT = '<IBIBBb'
struct __attribute__((packed)) LLCfg {
    uint32_t freq_hz;       // e.g. 903000000
    uint8_t  sf;            // 7–12
    uint32_t bw_hz;         // e.g. 125000
    uint8_t  cr;            // 5–8
    uint8_t  net_id;
    int8_t   txpwr_dbm;
};
static_assert(sizeof(LLCfg) == 12, "LLCfg must be 12 bytes — check packing");

// ---- TX (Bridge→Pi or Pi→Bridge) ----------------------------------------
/**
 * Build and write one COBS-framed message to Serial.
 * rssi / snr are only meaningful for Bridge→Pi data frames; pass 0 otherwise.
 */
void ll_send(uint8_t type,
             const uint8_t *payload = nullptr, uint8_t len = 0,
             int16_t rssi = 0, int8_t snr = 0);

// ---- RX state machine (Pi→Bridge direction) ----------------------------
#define LL_RAW_MAX   262u   /* 1+1+255+2+1+2 */
#define LL_COBS_MAX  272u   /* RAW_MAX + COBS overhead + delimiter */

struct LLRx {
    uint8_t buf[LL_COBS_MAX];
    size_t  n;
    bool    overflow;
};

/** Called for every successfully decoded, CRC-validated frame. */
typedef void (*ll_frame_cb)(uint8_t type,
                            const uint8_t *payload, uint8_t len,
                            int16_t rssi, int8_t snr);

void ll_rx_init(LLRx *rx);

/**
 * Feed raw bytes from Serial into the RX state machine.
 * cb is invoked synchronously for each valid frame (never from an ISR).
 */
void ll_rx_feed(LLRx *rx, const uint8_t *data, size_t len, ll_frame_cb cb);
