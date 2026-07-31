/*
 * lora_link.cpp — COBS/CRC-16 framing implementation (Arduino).
 * See lora_link.h for the wire format spec.
 */
#include "lora_link.h"

// ---- CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflect, no xorout) ----
static uint16_t crc16(const uint8_t *p, size_t n)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)p[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
    }
    return crc;
}

// ---- COBS encode; returns encoded byte count (delimiter NOT appended) ----
static size_t cobs_encode(const uint8_t *in, size_t len, uint8_t *out)
{
    uint8_t *enc    = out;
    uint8_t *code_p = enc++;   /* reserve first code byte */
    uint8_t  code   = 1;
    for (size_t i = 0; i < len; i++) {
        uint8_t c = in[i];
        if (c) { *enc++ = c; code++; }
        if (!c || code == 0xFFu) {
            *code_p = code;
            code = 1;
            code_p = enc++;
        }
    }
    *code_p = code;
    return (size_t)(enc - out);
}

// ---- COBS decode; returns decoded byte count or -1 on malformed input ----
static int cobs_decode(const uint8_t *in, size_t len,
                       uint8_t *out, size_t out_cap)
{
    size_t r = 0, w = 0;
    while (r < len) {
        uint8_t code = in[r++];
        if (code == 0) return -1;                  /* stray delimiter in body */
        for (uint8_t i = 1; i < code; i++) {
            if (r >= len || w >= out_cap) return -1;
            out[w++] = in[r++];
        }
        if (code < 0xFFu && r < len) {
            if (w >= out_cap) return -1;
            out[w++] = 0;
        }
    }
    return (int)w;
}

// ===========================================================================
// ll_send — build frame + COBS encode + write to Serial in one shot
// ===========================================================================
void ll_send(uint8_t type,
             const uint8_t *payload, uint8_t len,
             int16_t rssi, int8_t snr)
{
    // Build raw frame: LEN TYPE [PAYLOAD] RSSI_lo RSSI_hi SNR CRC_lo CRC_hi
    uint8_t raw[LL_RAW_MAX];
    size_t  n = 0;
    raw[n++] = len;
    raw[n++] = type;
    if (len && payload) { memcpy(&raw[n], payload, len); n += len; }
    raw[n++] = (uint8_t)( rssi        & 0xFF);
    raw[n++] = (uint8_t)((rssi >> 8)  & 0xFF);
    raw[n++] = (uint8_t)  snr;
    uint16_t crc  = crc16(raw, n);
    raw[n++] = (uint8_t)(crc & 0xFF);
    raw[n++] = (uint8_t)(crc >> 8);

    // COBS encode + append 0x00 delimiter, then write atomically
    uint8_t enc[LL_COBS_MAX];
    size_t  elen = cobs_encode(raw, n, enc);
    enc[elen++]  = 0x00;
    Serial.write(enc, elen);     /* single write → no inter-byte gaps on USB CDC */
}

// ===========================================================================
// ll_rx_init / ll_rx_feed
// ===========================================================================
void ll_rx_init(LLRx *rx)
{
    rx->n        = 0;
    rx->overflow = false;
}

void ll_rx_feed(LLRx *rx, const uint8_t *data, size_t len, ll_frame_cb cb)
{
    for (size_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        if (b == 0x00) {                           /* end of frame */
            if (!rx->overflow && rx->n > 0) {
                uint8_t raw[LL_RAW_MAX];
                int rn = cobs_decode(rx->buf, rx->n, raw, sizeof raw);
                if (rn >= 7) {                     /* min: LEN+TYPE+RSSI+SNR+CRC */
                    uint8_t plen = raw[0];
                    if ((size_t)rn == (size_t)(1u + 1u + plen + 2u + 1u + 2u)) {
                        uint16_t rx_crc = (uint16_t)raw[rn - 2] |
                                          ((uint16_t)raw[rn - 1] << 8);
                        if (crc16(raw, (size_t)(rn - 2)) == rx_crc) {
                            uint8_t mtype = raw[1];
                            int16_t frssi = (int16_t)((uint16_t)raw[2 + plen] |
                                            ((uint16_t)raw[3 + plen] << 8));
                            int8_t  fsnr  = (int8_t)raw[4 + plen];
                            cb(mtype, &raw[2], plen, frssi, fsnr);
                        }
                        /* CRC mismatch: silently discard — corrupt frame */
                    }
                    /* length mismatch: discard */
                }
                /* rn < 7 or cobs_decode error: discard */
            }
            rx->n        = 0;       /* resync unconditionally on every 0x00 */
            rx->overflow = false;
        } else {
            if (rx->n < sizeof(rx->buf)) rx->buf[rx->n++] = b;
            else rx->overflow = true;   /* oversize: drop until next delimiter */
        }
    }
}
