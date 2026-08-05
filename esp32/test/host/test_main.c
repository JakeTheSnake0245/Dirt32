/*
 * Host-side test suite for sps_proto.
 * Build: gcc -std=c99 -Wall -Wextra -O2 -I../../lib/sps_proto \
 *        ../../lib/sps_proto/*.c test_main.c -o sps_test && ./sps_test
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sps_proto.h"
#include "chacha20poly1305.h"

static int failures = 0;
static int checks = 0;
#define CHECK(cond, msg) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL: %s (line %d)\n", msg, __LINE__); } \
} while (0)

/* ---------------- RFC 8439 §2.8.2 AEAD test vector ---------------- */
static void test_rfc8439_vector(void) {
    const uint8_t key[32] = {
        0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8a,0x8b,0x8c,0x8d,0x8e,0x8f,
        0x90,0x91,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9a,0x9b,0x9c,0x9d,0x9e,0x9f };
    const uint8_t nonce[12] = { 0x07,0x00,0x00,0x00,0x40,0x41,0x42,0x43,0x44,0x45,0x46,0x47 };
    const uint8_t aad[12] = { 0x50,0x51,0x52,0x53,0xc0,0xc1,0xc2,0xc3,0xc4,0xc5,0xc6,0xc7 };
    const char *pt = "Ladies and Gentlemen of the class of '99: "
                     "If I could offer you only one tip for the future, "
                     "sunscreen would be it.";
    const uint8_t expect_ct_first16[16] = {
        0xd3,0x1a,0x8d,0x34,0x64,0x8e,0x60,0xdb,0x7b,0x86,0xaf,0xbc,0x53,0xef,0x7e,0xc2 };
    const uint8_t expect_tag[16] = {
        0x1a,0xe1,0x0b,0x59,0x4f,0x09,0xe2,0x6a,0x7e,0x90,0x2e,0xcb,0xd0,0x60,0x06,0x91 };

    size_t n = strlen(pt);
    uint8_t ct[128], tag[16], out[128];

    cc20p1305_encrypt(key, nonce, aad, sizeof(aad), (const uint8_t *)pt, n, ct, tag);
    CHECK(memcmp(ct, expect_ct_first16, 16) == 0, "RFC8439 ciphertext");
    CHECK(memcmp(tag, expect_tag, 16) == 0, "RFC8439 tag");

    CHECK(cc20p1305_decrypt(key, nonce, aad, sizeof(aad), ct, n, tag, out) == 0,
          "RFC8439 decrypt ok");
    CHECK(memcmp(out, pt, n) == 0, "RFC8439 roundtrip plaintext");

    /* Tamper checks */
    ct[0] ^= 1;
    CHECK(cc20p1305_decrypt(key, nonce, aad, sizeof(aad), ct, n, tag, out) == -1,
          "tampered ciphertext rejected");
    ct[0] ^= 1;
    uint8_t bad_aad[12]; memcpy(bad_aad, aad, 12); bad_aad[0] ^= 1;
    CHECK(cc20p1305_decrypt(key, nonce, bad_aad, sizeof(bad_aad), ct, n, tag, out) == -1,
          "tampered AAD rejected");
}

/* ---------------- Header + payload serialization ---------------- */
static void test_serialization(void) {
    sps_header_t h = { .net_id = 0x42, .msg_type = SPS_MSG_ALERT,
                       .node_id = 0xBEEF, .seq = 0x123456 };
    uint8_t buf[SPS_HDR_LEN];
    sps_header_write(&h, buf);
    CHECK(buf[0] == 0xA5, "magic byte");
    CHECK(buf[3] == 0xEF && buf[4] == 0xBE, "node_id little-endian");
    CHECK(buf[5] == 0x56 && buf[6] == 0x34 && buf[7] == 0x12, "seq u24 LE");

    sps_header_t h2;
    CHECK(sps_header_read(buf, sizeof(buf), &h2) == SPS_OK, "header parse");
    CHECK(h2.net_id == h.net_id && h2.msg_type == h.msg_type &&
          h2.node_id == h.node_id && h2.seq == h.seq, "header roundtrip");

    buf[0] = 0x00;
    CHECK(sps_header_read(buf, sizeof(buf), &h2) == SPS_ERR_BAD_MAGIC, "bad magic");
    buf[0] = 0xA5; buf[2] = 0x99;
    CHECK(sps_header_read(buf, sizeof(buf), &h2) == SPS_ERR_BAD_TYPE, "bad type");

    sps_alert_t a = { .timestamp = 1785400000u, .event_class = SPS_EV_FOOTSTEP,
                      .confidence = 200, .peak_amp = 12345, .battery_mv = 4012 };
    uint8_t pl[SPS_ALERT_PLEN];
    sps_alert_write(&a, pl);
    sps_alert_t a2; sps_alert_read(pl, &a2);
    CHECK(a2.timestamp == a.timestamp && a2.event_class == a.event_class &&
          a2.confidence == a.confidence && a2.peak_amp == a.peak_amp &&
          a2.battery_mv == a.battery_mv, "alert payload roundtrip");

    sps_heartbeat_t hb = { .timestamp = 1785400123u, .battery_mv = 3987,
                           .lat_e7 = 314159265, .lon_e7 = -1123456789,
                           .health_flags = SPS_HF_SENSOR_OK | SPS_HF_GPS_FIX |
                                           SPS_HF_CSI_ON,
                           .noise_floor = 87, .fw_version = 3, .reset_count = 2,
                           .csi_noise = 142 };
    uint8_t plh[SPS_HEARTBEAT_PLEN];
    sps_heartbeat_write(&hb, plh);
    sps_heartbeat_t hb2; sps_heartbeat_read(plh, &hb2);
    CHECK(hb2.lat_e7 == hb.lat_e7 && hb2.lon_e7 == hb.lon_e7, "negative lon LE roundtrip");
    CHECK(hb2.timestamp == hb.timestamp && hb2.battery_mv == hb.battery_mv &&
          hb2.health_flags == hb.health_flags && hb2.noise_floor == hb.noise_floor &&
          hb2.fw_version == hb.fw_version && hb2.reset_count == hb.reset_count,
          "heartbeat payload roundtrip");
    CHECK(hb2.csi_noise == hb.csi_noise, "heartbeat csi_noise roundtrip");
    CHECK((hb2.health_flags & SPS_HF_CSI_ON) != 0, "CSI_ON flag survives roundtrip");
    CHECK(sps_payload_len(SPS_MSG_HEARTBEAT) == 22, "heartbeat plen is 22 (csi field)");

    /* WiFi-presence alert class roundtrips like any other class */
    sps_alert_t wa = { .timestamp = 1785400200u, .event_class = SPS_EV_WIFI_PRESENCE,
                       .confidence = 150, .peak_amp = 2750 /* metric 27.50 x100 */,
                       .battery_mv = 4050 };
    uint8_t plw[SPS_ALERT_PLEN];
    sps_alert_write(&wa, plw);
    sps_alert_t wa2; sps_alert_read(plw, &wa2);
    CHECK(wa2.event_class == SPS_EV_WIFI_PRESENCE && wa2.peak_amp == wa.peak_amp,
          "wifi-presence alert roundtrip");

    sps_ack_t k = { .ack_seq = 0xABCDEF, .status = SPS_ACK_DUP };
    uint8_t plk[SPS_ACK_PLEN];
    sps_ack_write(&k, plk);
    sps_ack_t k2; sps_ack_read(plk, &k2);
    CHECK(k2.ack_seq == k.ack_seq && k2.status == k.status, "ack roundtrip");
}

/* ---------------- Frame seal/open ---------------- */
static void test_frame(void) {
    uint8_t key[32];
    for (int i = 0; i < 32; i++) key[i] = (uint8_t)(i * 7 + 1);

    sps_alert_t a = { .timestamp = 1785412345u, .event_class = SPS_EV_VEHICLE,
                      .confidence = 180, .peak_amp = 5000, .battery_mv = 3900 };
    uint8_t frame[SPS_MAX_FRAME];
    int n = sps_seal_alert(key, 0x07, 101, 42, &a, frame, sizeof(frame));
    CHECK(n == (int)(SPS_HDR_LEN + SPS_ALERT_PLEN + SPS_TAG_LEN), "alert frame length");

    sps_header_t h;
    uint8_t pl[SPS_MAX_PLEN];
    size_t plen;
    CHECK(sps_frame_open(key, frame, (size_t)n, &h, pl, &plen) == SPS_OK, "frame open");
    CHECK(h.node_id == 101 && h.seq == 42 && h.msg_type == SPS_MSG_ALERT, "frame header");
    sps_alert_t a2; sps_alert_read(pl, &a2);
    CHECK(a2.timestamp == a.timestamp && a2.event_class == a.event_class &&
          a2.confidence == a.confidence && a2.peak_amp == a.peak_amp &&
          a2.battery_mv == a.battery_mv, "frame alert roundtrip");

    /* Wrong key must fail */
    uint8_t key2[32]; memcpy(key2, key, 32); key2[0] ^= 1;
    CHECK(sps_frame_open(key2, frame, (size_t)n, &h, pl, &plen) == SPS_ERR_AUTH,
          "wrong key rejected");

    /* Header tamper (NODE_ID spoof) must fail: header is AAD + drives nonce */
    uint8_t spoof[SPS_MAX_FRAME]; memcpy(spoof, frame, (size_t)n);
    spoof[3] ^= 1;
    CHECK(sps_frame_open(key, spoof, (size_t)n, &h, pl, &plen) == SPS_ERR_AUTH,
          "spoofed node_id rejected");

    /* Truncated frame */
    CHECK(sps_frame_open(key, frame, (size_t)n - 1, &h, pl, &plen) == SPS_ERR_BAD_LEN,
          "truncated frame rejected");

    /* Buffer too small */
    CHECK(sps_seal_alert(key, 7, 1, 1, &a, frame, 10) == SPS_ERR_BUF, "small buffer");

    /* Same (node,type,seq) must produce identical nonce -> identical frame
       (deterministic retransmit); different seq must differ. */
    uint8_t f1[SPS_MAX_FRAME], f2[SPS_MAX_FRAME];
    sps_seal_alert(key, 7, 1, 5, &a, f1, sizeof(f1));
    sps_seal_alert(key, 7, 1, 6, &a, f2, sizeof(f2));
    CHECK(memcmp(f1 + SPS_HDR_LEN, f2 + SPS_HDR_LEN, SPS_ALERT_PLEN) != 0,
          "distinct seq -> distinct ciphertext");
}

/* ---------------- Replay window ---------------- */
static void test_replay(void) {
    sps_replay_t r;
    sps_replay_init(&r, 8);

    CHECK(sps_replay_check(&r, 100) == SPS_OK, "first seq accepted");
    CHECK(sps_replay_check(&r, 100) == SPS_ERR_REPLAY, "exact duplicate rejected");
    CHECK(sps_replay_check(&r, 101) == SPS_OK, "monotonic accepted");
    CHECK(sps_replay_check(&r, 105) == SPS_OK, "gap accepted");
    CHECK(sps_replay_check(&r, 103) == SPS_OK, "in-window reorder accepted");
    CHECK(sps_replay_check(&r, 103) == SPS_ERR_REPLAY, "reordered dup rejected");
    CHECK(sps_replay_check(&r, 96) == SPS_ERR_REPLAY, "outside window rejected");
    CHECK(sps_replay_check(&r, 97) == SPS_OK, "edge of window accepted");
    CHECK(sps_replay_check(&r, 200) == SPS_OK, "big jump accepted");
    CHECK(sps_replay_check(&r, 105) == SPS_ERR_REPLAY, "stale after jump rejected");

    /* retransmit burst pattern: same seq x3 -> one accept, two rejects */
    sps_replay_t r2; sps_replay_init(&r2, 8);
    int ok = 0;
    for (int i = 0; i < 3; i++)
        if (sps_replay_check(&r2, 7) == SPS_OK) ok++;
    CHECK(ok == 1, "blind retransmit dedup: exactly one accept");
}

/* ---------------- Relay dedupe ring ---------------- */
static void test_dedupe(void) {
    sps_dedupe_t d;
    sps_dedupe_init(&d);
    CHECK(!sps_dedupe_seen(&d, 10, 1), "first frame not seen");
    CHECK(sps_dedupe_seen(&d, 10, 1), "duplicate seen");
    CHECK(!sps_dedupe_seen(&d, 11, 1), "same seq other node not seen");
    CHECK(!sps_dedupe_seen(&d, 10, 2), "next seq not seen");
    /* fold-collision regression: node ids that would collide under an 8-bit
       fold (0x0101 and 0x0000 both fold to 0) must stay distinct */
    sps_dedupe_t d2;
    sps_dedupe_init(&d2);
    CHECK(!sps_dedupe_seen(&d2, 0x0101, 42), "node 0x0101 seq 42 not seen");
    CHECK(!sps_dedupe_seen(&d2, 0x0000, 42), "node 0x0000 seq 42 distinct (no fold collision)");
    CHECK(sps_dedupe_seen(&d2, 0x0101, 42), "node 0x0101 seq 42 dup detected");
    /* fill past capacity: oldest evicted */
    for (uint32_t s = 100; s < 100 + SPS_DEDUPE_SLOTS; s++)
        sps_dedupe_seen(&d, 10, s);
    CHECK(!sps_dedupe_seen(&d, 10, 1), "evicted entry forgotten");
}

/* ---------------- ACK end-to-end scenario ---------------- */
static void test_ack_flow(void) {
    uint8_t node_key[32];
    for (int i = 0; i < 32; i++) node_key[i] = (uint8_t)(0xC0 + i);

    /* Node sends ALERT seq=9; gateway/relay ACKs it back to the node. */
    sps_alert_t a = { .timestamp = 1785413000u, .event_class = SPS_EV_FOOTSTEP,
                      .confidence = 240, .peak_amp = 900, .battery_mv = 4100 };
    uint8_t alert_frame[SPS_MAX_FRAME];
    int an = sps_seal_alert(node_key, 0x07, 55, 9, &a, alert_frame, sizeof(alert_frame));
    CHECK(an > 0, "alert sealed");

    /* Gateway opens, then builds ACK addressed to node 55 with its own seq. */
    sps_header_t h; uint8_t pl[SPS_MAX_PLEN]; size_t plen;
    CHECK(sps_frame_open(node_key, alert_frame, (size_t)an, &h, pl, &plen) == SPS_OK,
          "gw opens alert");
    sps_ack_t ack = { .ack_seq = h.seq, .status = SPS_ACK_OK };
    uint8_t ack_frame[SPS_MAX_FRAME];
    int kn = sps_seal_ack(node_key, 0x07, h.node_id, 1 /* gw ack seq */, &ack,
                          ack_frame, sizeof(ack_frame));
    CHECK(kn == (int)(SPS_HDR_LEN + SPS_ACK_PLEN + SPS_TAG_LEN), "ack sealed");

    /* Node validates the ACK: correct key, addressed to it, matching ack_seq. */
    sps_header_t ah; uint8_t apl[SPS_MAX_PLEN]; size_t aplen;
    CHECK(sps_frame_open(node_key, ack_frame, (size_t)kn, &ah, apl, &aplen) == SPS_OK,
          "node opens ack");
    CHECK(ah.msg_type == SPS_MSG_ACK && ah.node_id == 55, "ack addressed to node");
    sps_ack_t ack2; sps_ack_read(apl, &ack2);
    CHECK(ack2.ack_seq == 9 && ack2.status == SPS_ACK_OK, "ack matches alert seq");

    /* A forged ACK under a different key must be rejected by the node. */
    uint8_t other_key[32]; memcpy(other_key, node_key, 32); other_key[31] ^= 0xFF;
    uint8_t forged[SPS_MAX_FRAME];
    int fn = sps_seal_ack(other_key, 0x07, 55, 2, &ack, forged, sizeof(forged));
    CHECK(sps_frame_open(node_key, forged, (size_t)fn, &ah, apl, &aplen) == SPS_ERR_AUTH,
          "forged ack rejected");
}

int main(void) {
    test_rfc8439_vector();
    test_serialization();
    test_frame();
    test_replay();
    test_dedupe();
    test_ack_flow();
    printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
