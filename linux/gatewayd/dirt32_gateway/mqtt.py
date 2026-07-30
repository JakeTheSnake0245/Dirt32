"""Minimal MQTT 3.1.1 publisher — stdlib only, QoS 0.

The gateway PUBLISHES to a broker (mosquitto on localhost by default);
external systems SUBSCRIBE to the broker, not to us. Keeping the client
dependency-free means the daemon runs on a stock Ubuntu Python while
mosquitto handles fan-out, retained messages, and auth for subscribers.

Topics:
  dirt32/alert/<node_id>       one message per verified alert (JSON)
  dirt32/heartbeat/<node_id>   one per verified heartbeat (JSON)
  dirt32/status/<node_id>      retained: current color + reasons (JSON)
  dirt32/gateway/state         retained: online/offline (LWT)
"""
from __future__ import annotations
import json
import socket
import struct
import threading
import time


def _encode_str(s: bytes) -> bytes:
    return struct.pack("!H", len(s)) + s


def _encode_varlen(n: int) -> bytes:
    out = b""
    while True:
        b = n % 128
        n //= 128
        out += bytes([b | (0x80 if n else 0)])
        if not n:
            return out


class MqttPublisher:
    """Auto-reconnecting fire-and-forget publisher. Never raises into the
    ingest path — MQTT being down must not stop alert storage."""

    def __init__(self, host="127.0.0.1", port=1883, client_id="dirt32-gateway",
                 keepalive=60, enabled=True, log=print):
        self.host, self.port = host, port
        self.client_id = client_id
        self.keepalive = keepalive
        self.enabled = enabled
        self.log = log
        self._sock = None
        self._lock = threading.Lock()
        self._last_attempt = 0.0
        self._last_ping = 0.0

    def _connect(self):
        if time.time() - self._last_attempt < 5:
            return False
        self._last_attempt = time.time()
        try:
            s = socket.create_connection((self.host, self.port), timeout=5)
            # CONNECT with LWT on dirt32/gateway/state
            proto_name = _encode_str(b"MQTT") + bytes([4])   # level 4 = 3.1.1
            flags = 0x02 | 0x04 | 0x20                       # clean, will, will-retain
            var = proto_name + bytes([flags]) + struct.pack("!H", self.keepalive)
            payload = _encode_str(self.client_id.encode())
            payload += _encode_str(b"dirt32/gateway/state")
            payload += _encode_str(b'{"online": false}')
            pkt = bytes([0x10]) + _encode_varlen(len(var + payload)) + var + payload
            s.sendall(pkt)
            resp = s.recv(4)
            if len(resp) < 4 or resp[0] != 0x20 or resp[3] != 0x00:
                s.close()
                return False
            self._sock = s
            self._last_ping = time.time()
            self.log("[mqtt] connected to %s:%d" % (self.host, self.port))
            self._publish_raw("dirt32/gateway/state", b'{"online": true}', retain=True)
            return True
        except OSError:
            self._sock = None
            return False

    def _publish_raw(self, topic: str, payload: bytes, retain=False):
        flags = 0x30 | (0x01 if retain else 0x00)
        var = _encode_str(topic.encode())
        pkt = bytes([flags]) + _encode_varlen(len(var + payload)) + var + payload
        self._sock.sendall(pkt)

    def ping_if_due(self):
        with self._lock:
            if not self._sock:
                return
            if time.time() - self._last_ping > self.keepalive / 2:
                try:
                    self._sock.sendall(b"\xc0\x00")   # PINGREQ
                    self._last_ping = time.time()
                except OSError:
                    self._sock = None

    def publish(self, topic: str, obj, retain=False):
        if not self.enabled:
            return
        payload = json.dumps(obj).encode()
        with self._lock:
            if self._sock is None and not self._connect():
                return
            try:
                self._publish_raw(topic, payload, retain)
            except OSError:
                self._sock = None   # reconnect on next publish
