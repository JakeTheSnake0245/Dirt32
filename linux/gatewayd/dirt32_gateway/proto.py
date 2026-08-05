"""
Dirt32 wire protocol — Python port of esp32/lib/sps_proto (spec §4).

Byte-for-byte compatible with the C library: same header layout, same
nonce construction, same ChaCha20-Poly1305 AEAD (RFC 8439), same replay
window semantics. Pure stdlib — no dependencies — so the gateway runs on
a stock Ubuntu LTS Python.

Frame on air:  HEADER(8) || CIPHERTEXT(n) || TAG(16)   (little-endian)
Header (AAD):  MAGIC(0xA5) NET_ID MSG_TYPE NODE_ID(2) SEQ(3)
Nonce (12B):   NODE_ID(2 LE) || MSG_TYPE(1) || SEQ(3 LE) || 0*6
"""
from __future__ import annotations
import struct
from dataclasses import dataclass

MAGIC = 0xA5
HDR_LEN = 8
TAG_LEN = 16
KEY_LEN = 32

MSG_ALERT = 0x01
MSG_HEARTBEAT = 0x02
MSG_ACK = 0x03
MSG_CMD = 0x04          # gateway -> node command (downlink)

ALERT_PLEN = 10
HEARTBEAT_PLEN = 22
ACK_PLEN = 4
CMD_PLEN = 2            # CMD1 ARG1
PAYLOAD_LEN = {MSG_ALERT: ALERT_PLEN, MSG_HEARTBEAT: HEARTBEAT_PLEN,
               MSG_ACK: ACK_PLEN, MSG_CMD: CMD_PLEN}

# Command IDs (MSG_CMD payload). Header NODE_ID is the destination; SEQ is
# the gateway's per-node downlink counter (node keeps its own replay
# window for it). No command ACK — confirmation is observable behavior
# (e.g. next heartbeat's HF_CSI_CALIB after a CSI_RECAL).
CMD_CSI_RECAL = 1       # restart WiFi-radar baseline calibration

EV_NAMES = {0: "unknown", 1: "footstep", 2: "vehicle", 3: "multiple",
            4: "wifi_presence"}
EV_WIFI_PRESENCE = 4
# Sensing channel per event class: seismic (ground coupling) vs rf (WiFi CSI).
EV_CHANNEL = {0: "seismic", 1: "seismic", 2: "seismic", 3: "seismic", 4: "rf"}

HF_SENSOR_OK = 1 << 0
HF_GPS_FIX   = 1 << 1
HF_SELFTEST  = 1 << 2
HF_TAMPER    = 1 << 3
HF_ON_SOLAR  = 1 << 4
HF_DEPLOY    = 1 << 5   # node just placed in field (button-triggered heartbeat)
HF_CSI_ON    = 1 << 6   # WiFi radar (CSI) sensing active
HF_CSI_CALIB = 1 << 7   # CSI detector still calibrating its baseline

ACK_OK = 0
ACK_DUP = 1

SEQ_MAX = 0xFFFFF0


class AuthError(Exception):
    """Tag verification failed — forged or corrupt frame."""


class ReplayError(Exception):
    """SEQ rejected by the replay window."""


class FrameError(Exception):
    """Malformed frame (magic/length/type)."""


# ---------------------------------------------------------------------------
# ChaCha20-Poly1305 (RFC 8439) — pure Python, constant-enough for a gateway
# handling a few frames per minute. ~40 µs/frame on a Pi 4.
# ---------------------------------------------------------------------------

def _rotl32(v: int, n: int) -> int:
    return ((v << n) | (v >> (32 - n))) & 0xFFFFFFFF


def _quarter(s, a, b, c, d):
    s[a] = (s[a] + s[b]) & 0xFFFFFFFF; s[d] = _rotl32(s[d] ^ s[a], 16)
    s[c] = (s[c] + s[d]) & 0xFFFFFFFF; s[b] = _rotl32(s[b] ^ s[c], 12)
    s[a] = (s[a] + s[b]) & 0xFFFFFFFF; s[d] = _rotl32(s[d] ^ s[a], 8)
    s[c] = (s[c] + s[d]) & 0xFFFFFFFF; s[b] = _rotl32(s[b] ^ s[c], 7)


def _chacha20_block(key: bytes, counter: int, nonce: bytes) -> bytes:
    st = list(struct.unpack("<4I", b"expand 32-byte k")) + \
         list(struct.unpack("<8I", key)) + [counter] + \
         list(struct.unpack("<3I", nonce))
    w = st[:]
    for _ in range(10):
        _quarter(w, 0, 4, 8, 12); _quarter(w, 1, 5, 9, 13)
        _quarter(w, 2, 6, 10, 14); _quarter(w, 3, 7, 11, 15)
        _quarter(w, 0, 5, 10, 15); _quarter(w, 1, 6, 11, 12)
        _quarter(w, 2, 7, 8, 13); _quarter(w, 3, 4, 9, 14)
    return struct.pack("<16I", *[(a + b) & 0xFFFFFFFF for a, b in zip(w, st)])


def _chacha20_xor(key: bytes, counter: int, nonce: bytes, data: bytes) -> bytes:
    out = bytearray()
    for i in range(0, len(data), 64):
        block = _chacha20_block(key, counter + i // 64, nonce)
        chunk = data[i:i + 64]
        out += bytes(a ^ b for a, b in zip(chunk, block))
    return bytes(out)


def _poly1305(key32: bytes, msg: bytes) -> bytes:
    r = int.from_bytes(key32[:16], "little") & 0x0FFFFFFC0FFFFFFC0FFFFFFC0FFFFFFF
    s = int.from_bytes(key32[16:], "little")
    p = (1 << 130) - 5
    acc = 0
    for i in range(0, len(msg), 16):
        chunk = msg[i:i + 16]
        n = int.from_bytes(chunk + b"\x01", "little")
        acc = ((acc + n) * r) % p
    return ((acc + s) & ((1 << 128) - 1)).to_bytes(16, "little")


def _pad16(b: bytes) -> bytes:
    return b"\x00" * (-len(b) % 16)


def aead_encrypt(key: bytes, nonce: bytes, aad: bytes, plaintext: bytes):
    otk = _chacha20_block(key, 0, nonce)[:32]
    ct = _chacha20_xor(key, 1, nonce, plaintext)
    mac_data = aad + _pad16(aad) + ct + _pad16(ct) + \
        struct.pack("<QQ", len(aad), len(ct))
    return ct, _poly1305(otk, mac_data)


def aead_decrypt(key: bytes, nonce: bytes, aad: bytes, ct: bytes, tag: bytes) -> bytes:
    otk = _chacha20_block(key, 0, nonce)[:32]
    mac_data = aad + _pad16(aad) + ct + _pad16(ct) + \
        struct.pack("<QQ", len(aad), len(ct))
    expect = _poly1305(otk, mac_data)
    # constant-time compare
    diff = 0
    for a, b in zip(expect, tag):
        diff |= a ^ b
    if diff or len(tag) != TAG_LEN:
        raise AuthError("tag mismatch")
    return _chacha20_xor(key, 1, nonce, ct)


# ---------------------------------------------------------------------------
# Frames
# ---------------------------------------------------------------------------

@dataclass
class Header:
    net_id: int
    msg_type: int
    node_id: int
    seq: int

    def pack(self) -> bytes:
        return struct.pack("<BBBH", MAGIC, self.net_id, self.msg_type,
                           self.node_id) + self.seq.to_bytes(3, "little")

    @staticmethod
    def unpack(buf: bytes) -> "Header":
        if len(buf) < HDR_LEN:
            raise FrameError("short header")
        magic, net, mtype, node = struct.unpack("<BBBH", buf[:5])
        if magic != MAGIC:
            raise FrameError("bad magic")
        return Header(net, mtype, node, int.from_bytes(buf[5:8], "little"))


def build_nonce(node_id: int, msg_type: int, seq: int) -> bytes:
    return struct.pack("<HB", node_id, msg_type) + seq.to_bytes(3, "little") + b"\x00" * 6


@dataclass
class Alert:
    timestamp: int
    event_class: int
    confidence: int
    peak_amp: int
    battery_mv: int

    def pack(self) -> bytes:
        return struct.pack("<IBBHH", self.timestamp, self.event_class,
                           self.confidence, self.peak_amp, self.battery_mv)

    @staticmethod
    def unpack(b: bytes) -> "Alert":
        return Alert(*struct.unpack("<IBBHH", b[:ALERT_PLEN]))


@dataclass
class Heartbeat:
    timestamp: int
    battery_mv: int
    lat_e7: int
    lon_e7: int
    health_flags: int
    noise_floor: int
    fw_version: int
    reset_count: int
    csi_noise: int = 0    # CSI quiescent noise metric x100; 0 = CSI off

    def pack(self) -> bytes:
        return struct.pack("<IHiiBHBHH", self.timestamp, self.battery_mv,
                           self.lat_e7, self.lon_e7, self.health_flags,
                           self.noise_floor, self.fw_version, self.reset_count,
                           self.csi_noise)

    @staticmethod
    def unpack(b: bytes) -> "Heartbeat":
        return Heartbeat(*struct.unpack("<IHiiBHBHH", b[:HEARTBEAT_PLEN]))


@dataclass
class Ack:
    ack_seq: int
    status: int

    def pack(self) -> bytes:
        return self.ack_seq.to_bytes(3, "little") + bytes([self.status])

    @staticmethod
    def unpack(b: bytes) -> "Ack":
        return Ack(int.from_bytes(b[:3], "little"), b[3])


@dataclass
class Cmd:
    cmd: int
    arg: int = 0

    def pack(self) -> bytes:
        return bytes([self.cmd, self.arg])

    @staticmethod
    def unpack(b: bytes) -> "Cmd":
        return Cmd(b[0], b[1])


def seal(key: bytes, hdr: Header, payload: bytes) -> bytes:
    if len(key) != KEY_LEN:
        raise ValueError("key must be 32 bytes")
    aad = hdr.pack()
    nonce = build_nonce(hdr.node_id, hdr.msg_type, hdr.seq)
    ct, tag = aead_encrypt(key, nonce, aad, payload)
    return aad + ct + tag


def open_frame(key: bytes, frame: bytes) -> tuple[Header, bytes]:
    hdr = Header.unpack(frame)
    plen = PAYLOAD_LEN.get(hdr.msg_type)
    if plen is None:
        raise FrameError(f"unknown msg_type {hdr.msg_type:#x}")
    if len(frame) != HDR_LEN + plen + TAG_LEN:
        raise FrameError("bad frame length")
    ct = frame[HDR_LEN:HDR_LEN + plen]
    tag = frame[HDR_LEN + plen:]
    nonce = build_nonce(hdr.node_id, hdr.msg_type, hdr.seq)
    pt = aead_decrypt(key, nonce, frame[:HDR_LEN], ct, tag)
    return hdr, pt


class ReplayWindow:
    """Mirror of sps_replay_t: sliding bitmap window, default 8."""

    def __init__(self, window: int = 8):
        self.window = max(1, min(32, window))
        self.last_seq = 0
        self.bitmap = 0
        self.seeded = False

    def check(self, seq: int) -> None:
        if not self.seeded:
            self.last_seq = seq
            self.bitmap = 0
            self.seeded = True
            return
        if seq > self.last_seq:
            shift = seq - self.last_seq
            self.bitmap = ((self.bitmap << shift) | (1 << (shift - 1))) & 0xFFFFFFFF
            self.last_seq = seq
            return
        if seq == self.last_seq:
            raise ReplayError("duplicate seq")
        behind = self.last_seq - seq
        if behind > self.window:
            raise ReplayError("stale seq")
        bit = 1 << (behind - 1)
        if self.bitmap & bit:
            raise ReplayError("duplicate seq in window")
        self.bitmap |= bit

    def to_state(self) -> dict:
        return {"last_seq": self.last_seq, "bitmap": self.bitmap,
                "window": self.window, "seeded": self.seeded}

    @staticmethod
    def from_state(d: dict) -> "ReplayWindow":
        r = ReplayWindow(d.get("window", 8))
        r.last_seq = d.get("last_seq", 0)
        r.bitmap = d.get("bitmap", 0)
        r.seeded = d.get("seeded", False)
        return r
