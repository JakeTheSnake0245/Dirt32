/*
 * make_vectors.c — emit sealed frames as hex so the Linux gateway's Python
 * protocol port can be verified byte-for-byte against the C library.
 *
 * Build:
 *   gcc -std=c99 -O2 -I../../lib/sps_proto \
 *       ../../lib/sps_proto/chacha20poly1305.c ../../lib/sps_proto/sps_proto.c \
 *       make_vectors.c -o make_vectors && ./make_vectors
 */
#include <stdio.h>
#include "sps_proto.h"

static void hexdump(const char *label, const uint8_t *b, int n) {
    printf("%s=", label);
    for (int i = 0; i < n; i++) printf("%02x", b[i]);
    printf("\n");
}

int main(void) {
    uint8_t key[32];
    for (int i = 0; i < 32; i++) key[i] = (uint8_t)(i * 7 + 3);

    uint8_t frame[SPS_MAX_FRAME];

    sps_alert_t a = { .timestamp = 1785400000, .event_class = SPS_EV_FOOTSTEP,
                      .confidence = 187, .peak_amp = 4242, .battery_mv = 3987 };
    int n = sps_seal_alert(key, 7, 0x0102, 0x000A0B, &a, frame, sizeof(frame));
    hexdump("alert", frame, n);

    sps_heartbeat_t hb = { .timestamp = 1785400100, .battery_mv = 3702,
                           .lat_e7 = 407128000, .lon_e7 = -740060000,
                           .health_flags = SPS_HF_SENSOR_OK | SPS_HF_GPS_FIX,
                           .noise_floor = 55, .fw_version = 1, .reset_count = 2 };
    n = sps_seal_heartbeat(key, 7, 0x0102, 0x000A0C, &hb, frame, sizeof(frame));
    hexdump("heartbeat", frame, n);

    sps_ack_t ack = { .ack_seq = 0x000A0B, .status = SPS_ACK_OK };
    n = sps_seal_ack(key, 7, 0x0102, 0x000A0B, &ack, frame, sizeof(frame));
    hexdump("ack", frame, n);
    return 0;
}
