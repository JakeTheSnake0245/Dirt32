"""Cross-verification of the Python protocol port against the C library.

Vectors in VECTORS were produced by esp32/test/host/make_vectors.c using
the same key schedule (key[i] = i*7+3). If the C wire format ever changes,
regenerate them and update here — this test is the compatibility contract
between node firmware and gateway.
"""
import sys, os, unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from dirt32_gateway import proto  # noqa: E402

KEY = bytes((i * 7 + 3) & 0xFF for i in range(32))

VECTORS = {}  # filled from vectors.txt at import if present, else inline
_here = os.path.dirname(__file__)
_vec = os.path.join(_here, "vectors.txt")
if os.path.exists(_vec):
    for line in open(_vec):
        if "=" in line:
            k, v = line.strip().split("=", 1)
            VECTORS[k] = bytes.fromhex(v)


class CrossVector(unittest.TestCase):
    def test_alert_from_c(self):
        hdr, pt = proto.open_frame(KEY, VECTORS["alert"])
        a = proto.Alert.unpack(pt)
        self.assertEqual((hdr.net_id, hdr.node_id, hdr.seq, hdr.msg_type),
                         (7, 0x0102, 0x000A0B, proto.MSG_ALERT))
        self.assertEqual((a.timestamp, a.event_class, a.confidence,
                          a.peak_amp, a.battery_mv),
                         (1785400000, 1, 187, 4242, 3987))

    def test_heartbeat_from_c(self):
        hdr, pt = proto.open_frame(KEY, VECTORS["heartbeat"])
        hb = proto.Heartbeat.unpack(pt)
        self.assertEqual(hdr.seq, 0x000A0C)
        self.assertEqual((hb.timestamp, hb.battery_mv, hb.lat_e7, hb.lon_e7,
                          hb.health_flags, hb.noise_floor, hb.fw_version,
                          hb.reset_count, hb.csi_noise),
                         (1785400100, 3702, 407128000, -740060000,
                          proto.HF_SENSOR_OK | proto.HF_GPS_FIX | proto.HF_CSI_ON,
                          55, 1, 2, 123))

    def test_wifi_alert_from_c(self):
        """WiFi radar (CSI) presence alert — event class 4, C-sealed."""
        hdr, pt = proto.open_frame(KEY, VECTORS["wifi_alert"])
        a = proto.Alert.unpack(pt)
        self.assertEqual(hdr.seq, 0x000A0D)
        self.assertEqual((a.timestamp, a.event_class, a.confidence,
                          a.peak_amp, a.battery_mv),
                         (1785400150, proto.EV_WIFI_PRESENCE, 142, 2750, 3950))
        self.assertEqual(proto.EV_NAMES[a.event_class], "wifi_presence")
        self.assertEqual(proto.EV_CHANNEL[a.event_class], "rf")

    def test_ack_from_c(self):
        hdr, pt = proto.open_frame(KEY, VECTORS["ack"])
        ack = proto.Ack.unpack(pt)
        self.assertEqual((ack.ack_seq, ack.status), (0x000A0B, proto.ACK_OK))

    def test_python_seal_reopens(self):
        """Python-sealed frame opens in Python (and, by the vectors above,
        matches the C AEAD — same construction both directions)."""
        a = proto.Alert(123456, 2, 99, 1000, 3600)
        hdr = proto.Header(7, proto.MSG_ALERT, 42, 1)
        frame = proto.seal(KEY, hdr, a.pack())
        hdr2, pt = proto.open_frame(KEY, frame)
        self.assertEqual(proto.Alert.unpack(pt), a)

    def test_ack_python_matches_c_bytes(self):
        """Gateway-sealed ACK must be byte-identical to a C-sealed ACK —
        the node firmware verifies it with the C library."""
        ack = proto.Ack(0x000A0B, proto.ACK_OK)
        hdr = proto.Header(7, proto.MSG_ACK, 0x0102, 0x000A0B)
        self.assertEqual(proto.seal(KEY, hdr, ack.pack()), VECTORS["ack"])

    def test_tamper_rejected(self):
        bad = bytearray(VECTORS["alert"])
        bad[10] ^= 0x01
        with self.assertRaises(proto.AuthError):
            proto.open_frame(KEY, bytes(bad))

    def test_spoofed_header_rejected(self):
        bad = bytearray(VECTORS["alert"])
        bad[3] ^= 0x01  # node_id — AAD *and* nonce input
        with self.assertRaises(proto.AuthError):
            proto.open_frame(KEY, bytes(bad))

    def test_wrong_key_rejected(self):
        with self.assertRaises(proto.AuthError):
            proto.open_frame(bytes(32), VECTORS["alert"])


class Replay(unittest.TestCase):
    def test_window(self):
        r = proto.ReplayWindow(8)
        r.check(10)
        r.check(11)
        with self.assertRaises(proto.ReplayError):
            r.check(11)
        r.check(9)   # reorder within window
        with self.assertRaises(proto.ReplayError):
            r.check(9)
        with self.assertRaises(proto.ReplayError):
            r.check(1)  # stale
        r.check(30)
        self.assertEqual(r.last_seq, 30)


if __name__ == "__main__":
    unittest.main(verbosity=2)
