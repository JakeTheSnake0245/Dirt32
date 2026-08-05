/* sps_proto.c — frame pack/parse, AEAD seal/open, replay + dedupe. */
#include "sps_proto.h"
#include "chacha20poly1305.h"
#include <string.h>

/* ---------- LE helpers ---------- */
static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put24(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); }
static void put32(uint8_t *p, uint32_t v) { put16(p, (uint16_t)v); put16(p + 2, (uint16_t)(v >> 16)); }
static uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t get24(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16); }
static uint32_t get32(const uint8_t *p) { return (uint32_t)get16(p) | ((uint32_t)get16(p + 2) << 16); }

/* ---------- Header ---------- */

void sps_header_write(const sps_header_t *h, uint8_t out[SPS_HDR_LEN]) {
    out[0] = SPS_MAGIC;
    out[1] = h->net_id;
    out[2] = h->msg_type;
    put16(out + 3, h->node_id);
    put24(out + 5, h->seq & 0xFFFFFFu);
}

sps_err_t sps_header_read(const uint8_t *buf, size_t len, sps_header_t *out) {
    if (len < SPS_HDR_LEN) return SPS_ERR_BAD_LEN;
    if (buf[0] != SPS_MAGIC) return SPS_ERR_BAD_MAGIC;
    out->net_id   = buf[1];
    out->msg_type = buf[2];
    out->node_id  = get16(buf + 3);
    out->seq      = get24(buf + 5);
    if (sps_payload_len(out->msg_type) == 0) return SPS_ERR_BAD_TYPE;
    return SPS_OK;
}

size_t sps_payload_len(uint8_t msg_type) {
    switch (msg_type) {
        case SPS_MSG_ALERT:     return SPS_ALERT_PLEN;
        case SPS_MSG_HEARTBEAT: return SPS_HEARTBEAT_PLEN;
        case SPS_MSG_ACK:       return SPS_ACK_PLEN;
        case SPS_MSG_CMD:       return SPS_CMD_PLEN;
        default:                return 0;
    }
}

/* ---------- Payloads ---------- */

void sps_alert_write(const sps_alert_t *a, uint8_t out[SPS_ALERT_PLEN]) {
    put32(out, a->timestamp);
    out[4] = a->event_class;
    out[5] = a->confidence;
    put16(out + 6, a->peak_amp);
    put16(out + 8, a->battery_mv);
}
void sps_alert_read(const uint8_t in[SPS_ALERT_PLEN], sps_alert_t *a) {
    a->timestamp   = get32(in);
    a->event_class = in[4];
    a->confidence  = in[5];
    a->peak_amp    = get16(in + 6);
    a->battery_mv  = get16(in + 8);
}

void sps_heartbeat_write(const sps_heartbeat_t *hb, uint8_t out[SPS_HEARTBEAT_PLEN]) {
    put32(out, hb->timestamp);
    put16(out + 4, hb->battery_mv);
    put32(out + 6,  (uint32_t)hb->lat_e7);
    put32(out + 10, (uint32_t)hb->lon_e7);
    out[14] = hb->health_flags;
    put16(out + 15, hb->noise_floor);
    out[17] = hb->fw_version;
    put16(out + 18, hb->reset_count);
    put16(out + 20, hb->csi_noise);
}
void sps_heartbeat_read(const uint8_t in[SPS_HEARTBEAT_PLEN], sps_heartbeat_t *hb) {
    hb->timestamp    = get32(in);
    hb->battery_mv   = get16(in + 4);
    hb->lat_e7       = (int32_t)get32(in + 6);
    hb->lon_e7       = (int32_t)get32(in + 10);
    hb->health_flags = in[14];
    hb->noise_floor  = get16(in + 15);
    hb->fw_version   = in[17];
    hb->reset_count  = get16(in + 18);
    hb->csi_noise    = get16(in + 20);
}

void sps_ack_write(const sps_ack_t *a, uint8_t out[SPS_ACK_PLEN]) {
    put24(out, a->ack_seq & 0xFFFFFFu);
    out[3] = a->status;
}
void sps_ack_read(const uint8_t in[SPS_ACK_PLEN], sps_ack_t *a) {
    a->ack_seq = get24(in);
    a->status  = in[3];
}

void sps_cmd_write(const sps_cmd_t *c, uint8_t out[SPS_CMD_PLEN]) {
    out[0] = c->cmd;
    out[1] = c->param;
    put16(out + 2, c->value);
}
void sps_cmd_read(const uint8_t in[SPS_CMD_PLEN], sps_cmd_t *c) {
    c->cmd   = in[0];
    c->param = in[1];
    c->value = get16(in + 2);
}

/* ---------- Nonce ---------- */

void sps_build_nonce(uint16_t node_id, uint8_t msg_type, uint32_t seq,
                     uint8_t out[SPS_NONCE_LEN]) {
    memset(out, 0, SPS_NONCE_LEN);
    put16(out, node_id);
    out[2] = msg_type;
    put24(out + 3, seq & 0xFFFFFFu);
}

/* ---------- Frame seal / open ---------- */

int sps_frame_seal(const uint8_t key[SPS_KEY_LEN],
                   const sps_header_t *hdr,
                   const uint8_t *payload, size_t plen,
                   uint8_t *out, size_t out_cap) {
    size_t need = SPS_HDR_LEN + plen + SPS_TAG_LEN;
    uint8_t nonce[SPS_NONCE_LEN];
    if (out_cap < need) return SPS_ERR_BUF;
    if (sps_payload_len(hdr->msg_type) != plen) return SPS_ERR_BAD_LEN;

    sps_header_write(hdr, out);
    sps_build_nonce(hdr->node_id, hdr->msg_type, hdr->seq, nonce);
    cc20p1305_encrypt(key, nonce, out /* AAD = header */, SPS_HDR_LEN,
                      payload, plen,
                      out + SPS_HDR_LEN, out + SPS_HDR_LEN + plen);
    return (int)need;
}

sps_err_t sps_frame_open(const uint8_t key[SPS_KEY_LEN],
                         const uint8_t *frame, size_t frame_len,
                         sps_header_t *hdr_out,
                         uint8_t *payload_out, size_t *plen_out) {
    sps_err_t err = sps_header_read(frame, frame_len, hdr_out);
    if (err != SPS_OK) return err;

    size_t plen = sps_payload_len(hdr_out->msg_type);
    if (frame_len != SPS_HDR_LEN + plen + SPS_TAG_LEN) return SPS_ERR_BAD_LEN;

    uint8_t nonce[SPS_NONCE_LEN];
    sps_build_nonce(hdr_out->node_id, hdr_out->msg_type, hdr_out->seq, nonce);
    if (cc20p1305_decrypt(key, nonce, frame, SPS_HDR_LEN,
                          frame + SPS_HDR_LEN, plen,
                          frame + SPS_HDR_LEN + plen, payload_out) != 0)
        return SPS_ERR_AUTH;
    *plen_out = plen;
    return SPS_OK;
}

/* ---------- Convenience seals ---------- */

int sps_seal_alert(const uint8_t key[SPS_KEY_LEN], uint8_t net_id,
                   uint16_t node_id, uint32_t seq, const sps_alert_t *a,
                   uint8_t *out, size_t out_cap) {
    uint8_t pl[SPS_ALERT_PLEN];
    sps_header_t h = { net_id, SPS_MSG_ALERT, node_id, seq };
    sps_alert_write(a, pl);
    return sps_frame_seal(key, &h, pl, sizeof(pl), out, out_cap);
}

int sps_seal_heartbeat(const uint8_t key[SPS_KEY_LEN], uint8_t net_id,
                       uint16_t node_id, uint32_t seq, const sps_heartbeat_t *hb,
                       uint8_t *out, size_t out_cap) {
    uint8_t pl[SPS_HEARTBEAT_PLEN];
    sps_header_t h = { net_id, SPS_MSG_HEARTBEAT, node_id, seq };
    sps_heartbeat_write(hb, pl);
    return sps_frame_seal(key, &h, pl, sizeof(pl), out, out_cap);
}

int sps_seal_ack(const uint8_t key[SPS_KEY_LEN], uint8_t net_id,
                 uint16_t dest_node_id, uint32_t seq, const sps_ack_t *a,
                 uint8_t *out, size_t out_cap) {
    uint8_t pl[SPS_ACK_PLEN];
    sps_header_t h = { net_id, SPS_MSG_ACK, dest_node_id, seq };
    sps_ack_write(a, pl);
    return sps_frame_seal(key, &h, pl, sizeof(pl), out, out_cap);
}

int sps_seal_cmd(const uint8_t key[SPS_KEY_LEN], uint8_t net_id,
                 uint16_t dest_node_id, uint32_t seq, const sps_cmd_t *c,
                 uint8_t *out, size_t out_cap) {
    uint8_t pl[SPS_CMD_PLEN];
    sps_header_t h = { net_id, SPS_MSG_CMD, dest_node_id, seq };
    sps_cmd_write(c, pl);
    return sps_frame_seal(key, &h, pl, sizeof(pl), out, out_cap);
}

/* ---------- Replay window ---------- */

void sps_replay_init(sps_replay_t *r, uint8_t window) {
    if (window < 1) window = 1;
    if (window > 32) window = 32;
    r->last_seq = 0;
    r->bitmap = 0;
    r->window = window;
    r->seeded = false;
}

sps_err_t sps_replay_check(sps_replay_t *r, uint32_t seq) {
    seq &= 0xFFFFFFu;
    if (!r->seeded) {
        r->seeded = true;
        r->last_seq = seq;
        r->bitmap = 0;
        return SPS_OK;
    }
    if (seq > r->last_seq) {
        uint32_t shift = seq - r->last_seq;
        r->bitmap = (shift >= 32) ? 0 : (r->bitmap << shift) | (1u << (shift - 1));
        r->last_seq = seq;
        return SPS_OK;
    }
    if (seq == r->last_seq) return SPS_ERR_REPLAY;
    uint32_t behind = r->last_seq - seq;           /* >= 1 */
    if (behind > r->window) return SPS_ERR_REPLAY; /* too old */
    uint32_t bit = 1u << (behind - 1);
    if (r->bitmap & bit) return SPS_ERR_REPLAY;    /* duplicate */
    r->bitmap |= bit;
    return SPS_OK;
}

/* ---------- Relay dedupe ---------- */

void sps_dedupe_init(sps_dedupe_t *d) {
    memset(d, 0xFF, sizeof(*d)); /* UINT64_MAX never matches a valid pair */
    d->head = 0;
}

bool sps_dedupe_seen(sps_dedupe_t *d, uint16_t node_id, uint32_t seq) {
    /* Exact (node_id, seq) pair — 16-bit node id above the 24-bit seq. */
    uint64_t tag = ((uint64_t)node_id << 24) | (seq & 0xFFFFFFu);
    for (int i = 0; i < SPS_DEDUPE_SLOTS; i++)
        if (d->slot[i] == tag) return true;
    d->slot[d->head] = tag;
    d->head = (uint8_t)((d->head + 1) % SPS_DEDUPE_SLOTS);
    return false;
}
