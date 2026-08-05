"""SQLite storage for the Dirt32 gateway (spec §7.2). Stdlib only."""
from __future__ import annotations
import json
import sqlite3
import threading
import time

SCHEMA = """
CREATE TABLE IF NOT EXISTS nodes (
    node_id     INTEGER PRIMARY KEY,
    name        TEXT,
    lat_e7      INTEGER,            -- provisioned position
    lon_e7      INTEGER,
    last_seen   REAL,               -- unix, gateway receipt time
    last_hb     REAL,
    battery_mv  INTEGER,
    health_flags INTEGER,
    noise_floor INTEGER,
    fw_version  INTEGER,
    reset_count INTEGER,
    rssi        REAL,
    snr         REAL,
    hb_lat_e7   INTEGER,            -- last reported position
    hb_lon_e7   INTEGER,
    replay_state TEXT,              -- persisted ReplayWindow json
    csi_noise   INTEGER             -- WiFi radar quiescent noise x100 (0/NULL = CSI off)
);
CREATE TABLE IF NOT EXISTS alerts (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    node_id     INTEGER NOT NULL,
    seq         INTEGER NOT NULL,
    received_at REAL NOT NULL,
    node_time   INTEGER,
    event_class INTEGER,
    confidence  INTEGER,
    peak_amp    INTEGER,
    battery_mv  INTEGER,
    rssi        REAL,
    snr         REAL
);
CREATE TABLE IF NOT EXISTS heartbeats (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    node_id     INTEGER NOT NULL,
    seq         INTEGER NOT NULL,
    received_at REAL NOT NULL,
    node_time   INTEGER,
    battery_mv  INTEGER,
    lat_e7      INTEGER,
    lon_e7      INTEGER,
    health_flags INTEGER,
    noise_floor INTEGER,
    rssi        REAL,
    snr         REAL,
    csi_noise   INTEGER             -- WiFi radar quiescent noise x100
);
CREATE TABLE IF NOT EXISTS raw_frames (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    received_at REAL NOT NULL,
    frame_hex   TEXT NOT NULL,
    rssi        REAL,
    snr         REAL,
    verdict     TEXT                -- ok / auth-fail / replay / unknown-node / malformed
);
CREATE INDEX IF NOT EXISTS idx_alerts_node ON alerts(node_id, received_at);
CREATE INDEX IF NOT EXISTS idx_hb_node ON heartbeats(node_id, received_at);
"""


class Db:
    def __init__(self, path: str):
        self._lock = threading.Lock()
        self._conn = sqlite3.connect(path, check_same_thread=False)
        self._conn.row_factory = sqlite3.Row
        self._conn.executescript(SCHEMA)
        self._conn.commit()
        # Migration for pre-CSI databases: CREATE TABLE IF NOT EXISTS does
        # not add columns to an existing table.
        for table in ("nodes", "heartbeats"):
            try:
                self._conn.execute(
                    f"ALTER TABLE {table} ADD COLUMN csi_noise INTEGER")
                self._conn.commit()
            except sqlite3.OperationalError:
                pass  # column already present
        # Migration: per-node downlink SEQ counter for gateway->node commands.
        try:
            self._conn.execute("ALTER TABLE nodes ADD COLUMN down_seq INTEGER")
            self._conn.commit()
        except sqlite3.OperationalError:
            pass  # column already present

    def _exec(self, sql, args=()):
        with self._lock:
            cur = self._conn.execute(sql, args)
            self._conn.commit()
            return cur

    def upsert_node(self, node_id: int, **fields):
        self._exec("INSERT OR IGNORE INTO nodes (node_id) VALUES (?)", (node_id,))
        if fields:
            sets = ", ".join(f"{k}=?" for k in fields)
            self._exec(f"UPDATE nodes SET {sets} WHERE node_id=?",
                       (*fields.values(), node_id))

    def get_node(self, node_id: int):
        cur = self._exec("SELECT * FROM nodes WHERE node_id=?", (node_id,))
        row = cur.fetchone()
        return dict(row) if row else None

    def all_nodes(self):
        return [dict(r) for r in self._exec("SELECT * FROM nodes ORDER BY node_id")]

    def load_replay(self, node_id: int):
        n = self.get_node(node_id)
        if n and n.get("replay_state"):
            return json.loads(n["replay_state"])
        return None

    def save_replay(self, node_id: int, state: dict):
        self.upsert_node(node_id, replay_state=json.dumps(state))

    def add_alert(self, node_id, seq, node_time, event_class, confidence,
                  peak_amp, battery_mv, rssi, snr):
        now = time.time()
        self._exec(
            "INSERT INTO alerts (node_id, seq, received_at, node_time, event_class,"
            " confidence, peak_amp, battery_mv, rssi, snr) VALUES (?,?,?,?,?,?,?,?,?,?)",
            (node_id, seq, now, node_time, event_class, confidence, peak_amp,
             battery_mv, rssi, snr))
        self.upsert_node(node_id, last_seen=now, battery_mv=battery_mv,
                         rssi=rssi, snr=snr)
        return now

    def add_heartbeat(self, node_id, seq, node_time, battery_mv, lat_e7, lon_e7,
                      health_flags, noise_floor, fw_version, reset_count, rssi, snr,
                      csi_noise=0):
        now = time.time()
        self._exec(
            "INSERT INTO heartbeats (node_id, seq, received_at, node_time, battery_mv,"
            " lat_e7, lon_e7, health_flags, noise_floor, rssi, snr, csi_noise)"
            " VALUES (?,?,?,?,?,?,?,?,?,?,?,?)",
            (node_id, seq, now, node_time, battery_mv, lat_e7, lon_e7,
             health_flags, noise_floor, rssi, snr, csi_noise))
        self.upsert_node(node_id, last_seen=now, last_hb=now, battery_mv=battery_mv,
                         health_flags=health_flags, noise_floor=noise_floor,
                         fw_version=fw_version, reset_count=reset_count,
                         rssi=rssi, snr=snr, hb_lat_e7=lat_e7, hb_lon_e7=lon_e7,
                         csi_noise=csi_noise)
        return now

    def next_down_seq(self, node_id: int) -> int:
        """Atomically increment and return the gateway->node downlink SEQ.
        Monotonic per node and persisted, so the node's replay window never
        sees a reused sequence number across daemon restarts."""
        with self._lock:
            self._conn.execute(
                "INSERT OR IGNORE INTO nodes (node_id) VALUES (?)", (node_id,))
            self._conn.execute(
                "UPDATE nodes SET down_seq = COALESCE(down_seq, 0) + 1"
                " WHERE node_id=?", (node_id,))
            cur = self._conn.execute(
                "SELECT down_seq FROM nodes WHERE node_id=?", (node_id,))
            seq = cur.fetchone()[0]
            self._conn.commit()
            return int(seq)

    def add_raw(self, frame_hex, rssi, snr, verdict):
        self._exec("INSERT INTO raw_frames (received_at, frame_hex, rssi, snr,"
                   " verdict) VALUES (?,?,?,?,?)",
                   (time.time(), frame_hex, rssi, snr, verdict))

    def recent_alerts(self, limit=100):
        return [dict(r) for r in self._exec(
            "SELECT * FROM alerts ORDER BY received_at DESC LIMIT ?", (limit,))]
