"""Dirt32 gateway daemon entry point.

Usage:  python -m dirt32_gateway.main [config.json]

Config search order: argv[1], /etc/dirt32/gateway.json, ./gateway.json.
See linux/gateway.example.json for all keys.
"""
from __future__ import annotations
import json
import os
import sys
import time

from . import proto
from .db import Db
from .ingest import Ingest
from .mqtt import MqttPublisher
from .serial_link import SerialLink
from .web import EventFeed, serve
from .simulator import Simulator

DEFAULT_CONFIG = {
    "net_id": 1,
    "serial_device": "/dev/ttyUSB0",
    "radio": {"freq_mhz": 903.0, "sf": 10, "bw_khz": 125.0, "cr": 5,
              "net_id": 1, "tx_power_dbm": 20},
    "db_path": "dirt32.sqlite3",
    "keys_path": "keys.json",
    "web": {"host": "0.0.0.0", "port": 9000},
    "mqtt": {"enabled": True, "host": "127.0.0.1", "port": 1883},
    "status": {"yellow_silence_s": 43200, "red_silence_s": 86400,
               "low_battery_mv": 3300, "move_threshold_m": 30.0},
    "simulator": False,
}


def load_config() -> dict:
    candidates = sys.argv[1:2] + ["/etc/dirt32/gateway.json", "gateway.json"]
    cfg = dict(DEFAULT_CONFIG)
    for path in candidates:
        if path and os.path.isfile(path):
            with open(path) as f:
                user = json.load(f)
            for k, v in user.items():
                if isinstance(v, dict) and isinstance(cfg.get(k), dict):
                    cfg[k] = {**cfg[k], **v}
                else:
                    cfg[k] = v
            print(f"[cfg] loaded {path}")
            break
    cfg["radio"]["net_id"] = cfg["net_id"]
    return cfg


def load_keys(path: str) -> dict[int, bytes]:
    """keys.json: { "<node_id>": "<64 hex chars>", ... }"""
    if not os.path.isfile(path):
        print(f"[cfg] WARNING: no key registry at {path} — no nodes trusted")
        return {}
    with open(path) as f:
        raw = json.load(f)
    keys = {}
    for k, v in raw.items():
        b = bytes.fromhex(v)
        if len(b) != 32:
            raise ValueError(f"key for node {k} is not 32 bytes")
        keys[int(k)] = b
    print(f"[cfg] {len(keys)} node key(s) loaded")
    return keys


def main():
    cfg = load_config()
    db = Db(cfg["db_path"])
    feed = EventFeed()

    keys = load_keys(cfg["keys_path"])
    if cfg.get("simulator"):
        # fabricate keys for simulated nodes 1..8
        keys = {i: os.urandom(32) for i in range(1, 9)}

    mqtt = MqttPublisher(enabled=cfg["mqtt"].get("enabled", True),
                         host=cfg["mqtt"].get("host", "127.0.0.1"),
                         port=cfg["mqtt"].get("port", 1883))

    link = None
    ingest = Ingest(db, keys, cfg["net_id"], mqtt=mqtt,
                    ack_sender=lambda fr: link and link.send_frame(fr),
                    status_cfg=cfg["status"],
                    on_event=feed.push)

    if cfg.get("simulator"):
        Simulator(ingest, keys, cfg["net_id"], db).start()
    else:
        link = SerialLink(cfg["serial_device"], cfg["radio"],
                          ingest.handle_frame)
        link.start()

    httpd = serve(db, feed, cfg["status"],
                  host=cfg["web"]["host"], port=int(cfg["web"]["port"]),
                  cmd_sender=lambda nid: ingest.send_cmd(
                      nid, proto.CMD_CSI_RECAL),
                  api_token=cfg["web"].get("api_token"))
    print(f"[web] GUI on http://{cfg['web']['host']}:{cfg['web']['port']}")

    # periodic: status refresh (silence-based transitions) + MQTT keepalive
    # Runs hourly — status events only fire on actual color/reason changes
    # (see Ingest.publish_status), so the interval just sets the worst-case
    # detection lag for a node going silent.
    try:
        while True:
            time.sleep(3600)
            mqtt.ping_if_due()
            ingest.publish_all_status()
    except KeyboardInterrupt:
        httpd.shutdown()


if __name__ == "__main__":
    main()
