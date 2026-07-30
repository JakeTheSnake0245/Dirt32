"""Node simulator — feeds sealed frames straight into the ingest pipeline
so the whole gateway (crypto, replay, DB, MQTT, GUI) can be exercised with
no radio hardware. Enabled with `"simulator": true` in the config.

Simulates a small perimeter: several healthy nodes, one with a low battery,
one gone quiet (yellow), one tampered (red), plus random footstep/vehicle
alerts.
"""
from __future__ import annotations
import random
import threading
import time

from . import proto


class Simulator:
    def __init__(self, ingest, keys: dict[int, bytes], net_id: int, db,
                 log=print):
        self.ingest = ingest
        self.keys = keys
        self.net_id = net_id
        self.db = db
        self.log = log
        self.seq = {n: random.randint(100, 500) for n in keys}
        # scatter provisioned positions around a field
        base_lat, base_lon = 40.0176, -105.2797
        self.pos = {}
        for i, n in enumerate(sorted(keys)):
            lat = base_lat + (i % 4) * 0.0009 + random.uniform(-1e-4, 1e-4)
            lon = base_lon + (i // 4) * 0.0012 + random.uniform(-1e-4, 1e-4)
            self.pos[n] = (int(lat * 1e7), int(lon * 1e7))
            db.upsert_node(n, name=f"sim-node-{n}",
                           lat_e7=self.pos[n][0], lon_e7=self.pos[n][1])
        nodes = sorted(keys)
        self.quiet_node = nodes[-1] if len(nodes) > 2 else None
        self.low_batt_node = nodes[1] if len(nodes) > 1 else None
        self.tamper_node = nodes[2] if len(nodes) > 3 else None

    def _next_seq(self, n):
        self.seq[n] += 1
        return self.seq[n]

    def _hb(self, n):
        batt = 3180 if n == self.low_batt_node else random.randint(3800, 4100)
        flags = proto.HF_SENSOR_OK | proto.HF_SELFTEST | proto.HF_GPS_FIX
        if n == self.tamper_node:
            flags |= proto.HF_TAMPER
        lat, lon = self.pos[n]
        hb = proto.Heartbeat(int(time.time()), batt,
                             lat + random.randint(-50, 50),
                             lon + random.randint(-50, 50),
                             flags, random.randint(30, 90), 1, 0)
        hdr = proto.Header(self.net_id, proto.MSG_HEARTBEAT, n, self._next_seq(n))
        frame = proto.seal(self.keys[n], hdr, hb.pack())
        self.ingest.handle_frame(frame, rssi=random.uniform(-120, -70),
                                 snr=random.uniform(-5, 10))

    def _alert(self, n):
        ev = random.choice([1, 1, 2])   # footsteps twice as likely as vehicles
        a = proto.Alert(int(time.time()), ev, random.randint(80, 250),
                        random.randint(500, 9000), random.randint(3700, 4100))
        hdr = proto.Header(self.net_id, proto.MSG_ALERT, n, self._next_seq(n))
        frame = proto.seal(self.keys[n], hdr, a.pack())
        self.ingest.handle_frame(frame, rssi=random.uniform(-115, -75),
                                 snr=random.uniform(-3, 10))

    def run(self):
        self.log("[sim] simulator running — fake perimeter active")
        # initial heartbeat burst so the map populates immediately
        for n in self.keys:
            if n != self.quiet_node:
                self._hb(n)
        # backdate the quiet node so it shows the silence path
        if self.quiet_node:
            self.db.upsert_node(self.quiet_node,
                                last_seen=time.time() - 14 * 3600,
                                last_hb=time.time() - 14 * 3600)
            self.ingest.publish_status(self.quiet_node)
        last_hb = time.time()
        while True:
            time.sleep(random.uniform(8, 25))
            if random.random() < 0.35:
                n = random.choice([x for x in self.keys if x != self.quiet_node])
                self._alert(n)
            if time.time() - last_hb > 60:
                for n in self.keys:
                    if n != self.quiet_node:
                        self._hb(n)
                last_hb = time.time()

    def start(self):
        t = threading.Thread(target=self.run, daemon=True, name="simulator")
        t.start()
        return t
