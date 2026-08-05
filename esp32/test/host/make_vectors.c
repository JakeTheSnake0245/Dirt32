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
                           .health_flags = SPS_HF_SENSOR_OK | SPS_HF_GPS_FIX |
                                           SPS_HF_CSI_ON,
                           .noise_floor = 55, .fw_version = 1, .reset_count = 2,
                           .csi_noise = 123 };
    n = sps_seal_heartbeat(key, 7, 0x0102, 0x000A0C, &hb, frame, sizeof(frame));
    hexdump("heartbeat", frame, n);

    /* WiFi radar (CSI) presence alert — new event class 4 */
    sps_alert_t wa = { .timestamp = 1785400150, .event_class = SPS_EV_WIFI_PRESENCE,
                       .confidence = 142, .peak_amp = 2750, .battery_mv = 3950 };
    n = sps_seal_alert(key, 7, 0x0102, 0x000A0D, &wa, frame, sizeof(frame));
    hexdump("wifi_alert", frame, n);

    sps_ack_t ack = { .ack_seq = 0x000A0B, .status = SPS_ACK_OK };
    n = sps_seal_ack(key, 7, 0x0102, 0x000A0B, &ack, frame, sizeof(frame));
    hexdump("ack", frame, n);

    /* Gateway->node command: CSI recalibrate (downlink, msg type 0x04) */
    sps_cmd_t cmd = { .cmd = SPS_CMD_CSI_RECAL, .param = 0, .value = 0 };
    n = sps_seal_cmd(key, 7, 0x0102, 0x000101, &cmd, frame, sizeof(frame));
    hexdump("cmd_recal", frame, n);

    /* Gateway->node command: SET csi_threshold = 2.50 (x100 = 250) */
    sps_cmd_t cset = { .cmd = SPS_CMD_SET, .param = SPS_SET_CSI_THRESHOLD,
                       .value = 250 };
    n = sps_seal_cmd(key, 7, 0x0102, 0x000102, &cset, frame, sizeof(frame));
    hexdump("cmd_set", frame, n);
    return 0;
}
