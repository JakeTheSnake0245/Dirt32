"""Frame ingest — the trust boundary. Every radio frame lands here.

Pipeline:
  parse header → look up node key → AEAD open → replay check →
  ACK immediately (alerts only, before any I/O) →
  store → publish MQTT

ACK timing: the node opens a short RX window (ACK_WAIT_MS ≈ 400 ms) the
moment it finishes transmitting.  Everything between open_frame() and the
ACK write is in-memory bit operations, so the ACK exits the Pi in < 5 ms —
well inside the window.  DB writes, MQTT, and logging happen after.
"""
from __future__ import annotations
import threading
import time

from . import proto
from .db import Db
from .status import classify


class Ingest:
    def __init__(self, db: Db, keys: dict[int, bytes], net_id: int,
                 mqtt=None, ack_sender=None, status_cfg=None, log=print,
                 on_event=None):
        """keys: node_id -> 32-byte PSK.  ack_sender(frame_bytes) transmits.
        on_event(kind, obj): hook for the web layer's live feed."""
        self.db = db
        self.keys = keys
        self.net_id = net_id
        self.mqtt = mqtt
        self.ack_sender = ack_sender
        self.status_cfg = status_cfg or {}
        self.log = log
        self.on_event = on_event or (lambda kind, obj: None)
        self._lock = threading.Lock()
        self._replay: dict[int, proto.ReplayWindow] = {}
        for node_id in keys:
            st = db.load_replay(node_id)
            self._replay[node_id] = (proto.ReplayWindow.from_state(st)
                                     if st else proto.ReplayWindow(8))
        # Cache last emitted status per node so we only push an event when
        # color or reasons actually change (suppresses per-heartbeat flood).
        self._last_status: dict[int, tuple[str, tuple]] = {}

    # ------------------------------------------------------------------

    def handle_frame(self, frame: bytes, rssi=None, snr=None):
        hexf = frame.hex()

        # ---- 1. Header parse (no key needed) ----------------------------
        try:
            hdr = proto.Header.unpack(frame)
        except proto.FrameError:
            self.db.add_raw(hexf, rssi, snr, "malformed")
            return

        if hdr.net_id != self.net_id:
            self.db.add_raw(hexf, rssi, snr, "wrong-net")
            return
        if hdr.msg_type == proto.MSG_ACK:
            return   # our own ACK echoed back by the bridge

        # ---- 2. Key lookup ----------------------------------------------
        key = self.keys.get(hdr.node_id)
        if key is None:
            self.db.add_raw(hexf, rssi, snr, "unknown-node")
            self.log(f"[ingest] frame from unknown node {hdr.node_id}")
            return

        # ---- 3. AEAD open (authenticate + decrypt) ----------------------
        try:
            hdr, payload = proto.open_frame(key, frame)
        except proto.AuthError:
            self.db.add_raw(hexf, rssi, snr, "auth-fail")
            self.log(f"[ingest] AUTH FAIL node={hdr.node_id} seq={hdr.seq} "
                     "(forged or corrupt)")
            return
        except proto.FrameError:
            self.db.add_raw(hexf, rssi, snr, "malformed")
            return

        # ---- 4. Replay check (pure in-memory — fast) --------------------
        with self._lock:
            rw = self._replay.setdefault(hdr.node_id, proto.ReplayWindow(8))
            duplicate = False
            try:
                rw.check(hdr.seq)
            except proto.ReplayError:
                duplicate = True
            replay_state = rw.to_state()   # snapshot while we hold the lock

        # ---- 5. ACK — must happen before any DB/MQTT/log I/O -----------
        # The node's RX window is only ~400 ms.  We are still in-memory here.
        # Always ACK alerts, even duplicates: a dup means the node missed
        # our prior ACK and kept retransmitting.
        # Guarded so a malformed field or proto bug can NEVER skip the ACK.
        if hdr.msg_type == proto.MSG_ALERT:
            self._send_ack(key, hdr,
                           proto.ACK_DUP if duplicate else proto.ACK_OK)

        # ---- 6. Bookkeeping (slow I/O starts here) ---------------------
        self.db.save_replay(hdr.node_id, replay_state)

        if duplicate:
            self.db.add_raw(hexf, rssi, snr, "replay")
            return

        self.db.add_raw(hexf, rssi, snr, "ok")
        if hdr.msg_type == proto.MSG_ALERT:
            try:
                self._on_alert(key, hdr,
                               proto.Alert.unpack(payload), rssi, snr)
            except Exception as e:          # noqa: BLE001
                self.log(f"[ingest] _on_alert error node={hdr.node_id}: {e}")
        elif hdr.msg_type == proto.MSG_HEARTBEAT:
            try:
                self._on_heartbeat(hdr,
                                   proto.Heartbeat.unpack(payload), rssi, snr)
            except Exception as e:          # noqa: BLE001
                self.log(f"[ingest] _on_heartbeat error node={hdr.node_id}: {e}")

    # ------------------------------------------------------------------

    def _send_ack(self, key: bytes, hdr: proto.Header, status: int):
        """Build and transmit a sealed ACK frame.  Never raises — a failure
        here must not prevent the rest of handle_frame from running."""
        if self.ack_sender is None:
            return
        try:
            ack = proto.Ack(hdr.seq, status)
            ack_hdr = proto.Header(self.net_id, proto.MSG_ACK,
                                   hdr.node_id, hdr.seq)
            self.ack_sender(proto.seal(key, ack_hdr, ack.pack()))
        except Exception as e:              # noqa: BLE001
            self.log(f"[ingest] _send_ack failed node={hdr.node_id}: {e}")

    def _on_alert(self, key, hdr, a: proto.Alert, rssi, snr):
        # ACK already sent before this method is called.
        received = self.db.add_alert(hdr.node_id, hdr.seq, a.timestamp,
                                     a.event_class, a.confidence, a.peak_amp,
                                     a.battery_mv, rssi, snr)
        ev = {"node_id": hdr.node_id, "seq": hdr.seq, "received_at": received,
              "event": proto.EV_NAMES.get(a.event_class, "?"),
              "confidence": a.confidence, "peak_amp": a.peak_amp,
              "battery_mv": a.battery_mv, "rssi": rssi, "snr": snr}
        self.log(f"[alert] node={hdr.node_id} {ev['event']} "
                 f"conf={a.confidence} rssi={rssi}")
        if self.mqtt:
            self.mqtt.publish(f"dirt32/alert/{hdr.node_id}", ev)
        self.on_event("alert", ev)
        self.publish_status(hdr.node_id)

    def _on_heartbeat(self, hdr, hb: proto.Heartbeat, rssi, snr):
        received = self.db.add_heartbeat(
            hdr.node_id, hdr.seq, hb.timestamp, hb.battery_mv, hb.lat_e7,
            hb.lon_e7, hb.health_flags, hb.noise_floor, hb.fw_version,
            hb.reset_count, rssi, snr)
        ev = {"node_id": hdr.node_id, "seq": hdr.seq, "received_at": received,
              "battery_mv": hb.battery_mv, "lat_e7": hb.lat_e7,
              "lon_e7": hb.lon_e7, "health_flags": hb.health_flags,
              "noise_floor": hb.noise_floor, "rssi": rssi, "snr": snr}
        deploy = bool(hb.health_flags & proto.HF_DEPLOY)
        gps_fix = bool(hb.health_flags & proto.HF_GPS_FIX)
        pos = (f" lat={hb.lat_e7/1e7:.5f} lon={hb.lon_e7/1e7:.5f}"
               f" ({'gps' if gps_fix else 'fallback'})"
               if (hb.lat_e7 or hb.lon_e7) else " pos=none")
        self.log(f"[{'DEPLOY' if deploy else 'hb'}] node={hdr.node_id} "
                 f"batt={hb.battery_mv}mV flags={hb.health_flags:#04x}{pos}")
        if self.mqtt:
            self.mqtt.publish(f"dirt32/heartbeat/{hdr.node_id}", ev)
        self.on_event("heartbeat", ev)
        self.publish_status(hdr.node_id)

    # ------------------------------------------------------------------

    def publish_status(self, node_id: int):
        node = self.db.get_node(node_id)
        if not node:
            return
        color, reasons = classify(node, self.status_cfg)
        key = (color, tuple(reasons))
        # Only emit when color or reasons change — suppresses per-heartbeat
        # flood when the node's state is stable (e.g. "yellow, low battery"
        # every 30 s with nothing actually different).
        if self._last_status.get(node_id) == key:
            return
        self._last_status[node_id] = key
        obj = {"node_id": node_id, "color": color, "reasons": reasons,
               "at": time.time()}
        if self.mqtt:
            self.mqtt.publish(f"dirt32/status/{node_id}", obj, retain=True)
        self.on_event("status", obj)

    def publish_all_status(self):
        for n in self.db.all_nodes():
            self.publish_status(n["node_id"])
