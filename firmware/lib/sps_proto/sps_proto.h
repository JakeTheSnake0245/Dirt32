/*
 * sps_proto.h — Distributed Seismic Perimeter Sensor system
 * Shared wire-protocol + crypto library. Pure C99, no dependencies.
 *
 * Compiles identically on:
 *   - ESP32-S3 node firmware (Arduino/PlatformIO)
 *   - Heltec V3 relay firmware
 *   - Raspberry Pi gateway ingest (Linux)
 *   - Host test suite (gcc/clang)
 *
 * Wire format (spec §4):
 *   Frame on air: HEADER(8) || CIPHERTEXT(n) || TAG(16)
 *   All multi-byte integers little-endian.
 *
 *   Header (plaintext, used as AAD):
 *     [0]   MAGIC    0xA5
 *     [1]   NET_ID
 *     [2]   MSG_TYPE 0x01 ALERT, 0x02 HEARTBEAT, 0x03 ACK
 *     [3:5] NODE_ID  u16  (source; dest for ACK)
 *     [5:8] SEQ      u24  monotonic per-node counter, nonce base, replay key
 *
 *   AEAD: ChaCha20-Poly1305 (RFC 8439), 16-byte tag, 256-bit per-node PSK.
 *   Nonce (12B): NODE_ID(2 LE) || MSG_TYPE(1) || SEQ(3 LE) || 0x00*6
 */
#ifndef SPS_PROTO_H
#define SPS_PROTO_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- Constants ---------- */

#define SPS_MAGIC            0xA5u
#define SPS_HDR_LEN          8u
#define SPS_TAG_LEN          16u
#define SPS_KEY_LEN          32u
#define SPS_NONCE_LEN        12u

#define SPS_MSG_ALERT        0x01u
#define SPS_MSG_HEARTBEAT    0x02u
#define SPS_MSG_ACK          0x03u

#define SPS_ALERT_PLEN       10u  /* TIMESTAMP4 CLASS1 CONF1 PEAK2 BATT2 */
#define SPS_HEARTBEAT_PLEN   20u  /* TS4 BATT2 LAT4 LON4 FLAGS1 NOISE2 FW1 RST2 */
#define SPS_ACK_PLEN         4u   /* ACK_SEQ3 STATUS1 */

#define SPS_MAX_PLEN         SPS_HEARTBEAT_PLEN
#define SPS_MAX_FRAME        (SPS_HDR_LEN + SPS_MAX_PLEN + SPS_TAG_LEN)

/* Event classes */
#define SPS_EV_UNKNOWN   0u
#define SPS_EV_FOOTSTEP  1u
#define SPS_EV_VEHICLE   2u
#define SPS_EV_MULTIPLE  3u

/* Health flags (heartbeat) */
#define SPS_HF_SENSOR_OK   (1u << 0)
#define SPS_HF_GPS_FIX     (1u << 1)
#define SPS_HF_SELFTEST    (1u << 2)
#define SPS_HF_TAMPER      (1u << 3)
#define SPS_HF_ON_SOLAR    (1u << 4)

/* ACK status */
#define SPS_ACK_OK         0u
#define SPS_ACK_DUP        1u

/* SEQ is 24-bit and MUST NOT wrap under a given key (nonce reuse).
   Senders halt at SPS_SEQ_MAX and require a PSK rotation, which resets
   SEQ to 0 on the node and replay state at the gateway. */
#define SPS_SEQ_MAX        0xFFFFF0u

/* Error codes */
typedef enum {
    SPS_OK = 0,
    SPS_ERR_BAD_MAGIC   = -1,
    SPS_ERR_BAD_NET     = -2,
    SPS_ERR_BAD_TYPE    = -3,
    SPS_ERR_BAD_LEN     = -4,
    SPS_ERR_AUTH        = -5,   /* tag verify failed — forged/corrupt */
    SPS_ERR_REPLAY      = -6,   /* SEQ rejected by replay window */
    SPS_ERR_BUF         = -7,   /* output buffer too small */
} sps_err_t;

/* ---------- Structures ---------- */

typedef struct {
    uint8_t  net_id;
    uint8_t  msg_type;
    uint16_t node_id;
    uint32_t seq;        /* 24-bit used */
} sps_header_t;

typedef struct {
    uint32_t timestamp;
    uint8_t  event_class;
    uint8_t  confidence;   /* STA/LTA ratio scaled 0-255 */
    uint16_t peak_amp;
    uint16_t battery_mv;
} sps_alert_t;

typedef struct {
    uint32_t timestamp;
    uint16_t battery_mv;
    int32_t  lat_e7;       /* degrees × 1e7 */
    int32_t  lon_e7;
    uint8_t  health_flags;
    uint16_t noise_floor;
    uint8_t  fw_version;
    uint16_t reset_count;
} sps_heartbeat_t;

typedef struct {
    uint32_t ack_seq;      /* 24-bit */
    uint8_t  status;
} sps_ack_t;

/* ---------- Header ---------- */

void sps_header_write(const sps_header_t *h, uint8_t out[SPS_HDR_LEN]);
/* Parses and validates MAGIC. Does not check NET_ID (caller policy). */
sps_err_t sps_header_read(const uint8_t *buf, size_t len, sps_header_t *out);

/* Expected payload length for a message type, or 0 if unknown type. */
size_t sps_payload_len(uint8_t msg_type);

/* ---------- Payload serialization (plaintext side) ---------- */

void sps_alert_write(const sps_alert_t *a, uint8_t out[SPS_ALERT_PLEN]);
void sps_alert_read(const uint8_t in[SPS_ALERT_PLEN], sps_alert_t *a);
void sps_heartbeat_write(const sps_heartbeat_t *hb, uint8_t out[SPS_HEARTBEAT_PLEN]);
void sps_heartbeat_read(const uint8_t in[SPS_HEARTBEAT_PLEN], sps_heartbeat_t *hb);
void sps_ack_write(const sps_ack_t *a, uint8_t out[SPS_ACK_PLEN]);
void sps_ack_read(const uint8_t in[SPS_ACK_PLEN], sps_ack_t *a);

/* ---------- Frame seal / open (AEAD) ---------- */

/*
 * Seal: header + plaintext payload -> full on-air frame.
 * out must hold SPS_HDR_LEN + plen + SPS_TAG_LEN bytes.
 * Returns frame length or negative sps_err_t.
 */
int sps_frame_seal(const uint8_t key[SPS_KEY_LEN],
                   const sps_header_t *hdr,
                   const uint8_t *payload, size_t plen,
                   uint8_t *out, size_t out_cap);

/*
 * Open: full on-air frame -> header + decrypted payload.
 * Verifies MAGIC, expected payload length for MSG_TYPE, and auth tag.
 * payload_out must hold SPS_MAX_PLEN bytes; *plen_out receives length.
 * Returns SPS_OK or negative error. On SPS_ERR_AUTH the payload buffer
 * contents are undefined and must be discarded.
 */
sps_err_t sps_frame_open(const uint8_t key[SPS_KEY_LEN],
                         const uint8_t *frame, size_t frame_len,
                         sps_header_t *hdr_out,
                         uint8_t *payload_out, size_t *plen_out);

/* Convenience: seal an ALERT / HEARTBEAT / ACK in one call. */
int sps_seal_alert(const uint8_t key[SPS_KEY_LEN], uint8_t net_id,
                   uint16_t node_id, uint32_t seq, const sps_alert_t *a,
                   uint8_t *out, size_t out_cap);
int sps_seal_heartbeat(const uint8_t key[SPS_KEY_LEN], uint8_t net_id,
                       uint16_t node_id, uint32_t seq, const sps_heartbeat_t *hb,
                       uint8_t *out, size_t out_cap);
int sps_seal_ack(const uint8_t key[SPS_KEY_LEN], uint8_t net_id,
                 uint16_t dest_node_id, uint32_t seq, const sps_ack_t *a,
                 uint8_t *out, size_t out_cap);

/* ---------- Replay protection (gateway / ACK-consumer side) ---------- */

/*
 * Sliding-window replay filter per node. Accepts monotonically increasing
 * SEQ; tolerates reordering within `window` (spec default 8); rejects
 * duplicates and stale sequence numbers.
 *
 * SEQ never wraps by design: senders MUST stop transmitting at
 * SPS_SEQ_MAX and require a key rotation (which resets SEQ to 0 and the
 * receiver's replay state for that node). See SPS_SEQ_MAX.
 */
typedef struct {
    uint32_t last_seq;    /* highest accepted */
    uint32_t bitmap;      /* bit i => (last_seq - 1 - i) seen; window <= 32 */
    uint8_t  window;
    bool     seeded;      /* false until first frame accepted */
} sps_replay_t;

void sps_replay_init(sps_replay_t *r, uint8_t window /* 1..32 */);
/* Returns SPS_OK and records seq, or SPS_ERR_REPLAY. */
sps_err_t sps_replay_check(sps_replay_t *r, uint32_t seq);

/* ---------- Relay dedupe (opaque frames, no keys) ---------- */

/*
 * Small ring of recently seen (NODE_ID, SEQ) pairs. Relay-side: forward a
 * frame only if sps_dedupe_seen() returns false.
 */
#define SPS_DEDUPE_SLOTS 32
typedef struct {
    uint64_t slot[SPS_DEDUPE_SLOTS];  /* exact (node_id << 24) | seq; empty = UINT64_MAX */
    uint8_t  head;
} sps_dedupe_t;

void sps_dedupe_init(sps_dedupe_t *d);
/* Records the pair; returns true if it was already present. */
bool sps_dedupe_seen(sps_dedupe_t *d, uint16_t node_id, uint32_t seq);

/* ---------- Nonce construction (exposed for tests) ---------- */
void sps_build_nonce(uint16_t node_id, uint8_t msg_type, uint32_t seq,
                     uint8_t out[SPS_NONCE_LEN]);

#ifdef __cplusplus
}
#endif
#endif /* SPS_PROTO_H */
