"""Web GUI + JSON API. Stdlib http.server — no framework dependency.

Endpoints:
  GET /                     the blackbox map
  GET /gw/nodes            all nodes with computed status color + reasons
  GET /gw/alerts?limit=N   recent alerts
  GET /gw/events?since=T   alert/hb/status events after unix time T (polling feed)
"""
from __future__ import annotations
import collections
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

from . import proto
from .db import Db
from .status import classify

STATIC_DIR = os.path.join(os.path.dirname(__file__), "static")


class EventFeed:
    """Ring buffer of recent events for the GUI's polling feed."""

    def __init__(self, cap=500):
        self._buf = collections.deque(maxlen=cap)
        self._lock = threading.Lock()

    def push(self, kind: str, obj: dict):
        with self._lock:
            self._buf.append({"kind": kind, "at": time.time(), "data": obj})

    def since(self, t: float):
        with self._lock:
            return [e for e in self._buf if e["at"] > t]


def make_handler(db: Db, feed: EventFeed, status_cfg: dict):
    class Handler(BaseHTTPRequestHandler):
        server_version = "dirt32-gateway"

        def log_message(self, *a):   # quiet
            pass

        def _json(self, obj, code=200):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _file(self, name, ctype):
            path = os.path.join(STATIC_DIR, name)
            if not os.path.isfile(path):
                self.send_error(404)
                return
            with open(path, "rb") as f:
                body = f.read()
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            u = urlparse(self.path)
            q = parse_qs(u.query)
            path = u.path.rstrip("/") or "/"

            if path.endswith("/gw/nodes"):
                out = []
                for n in db.all_nodes():
                    color, reasons = classify(n, status_cfg)
                    lat = n.get("hb_lat_e7") or n.get("lat_e7") or 0
                    lon = n.get("hb_lon_e7") or n.get("lon_e7") or 0
                    out.append({
                        "node_id": n["node_id"], "name": n.get("name"),
                        "color": color, "reasons": reasons,
                        "lat": lat / 1e7, "lon": lon / 1e7,
                        "last_seen": n.get("last_seen"),
                        "battery_mv": n.get("battery_mv"),
                        "health_flags": n.get("health_flags"),
                        "tamper": bool((n.get("health_flags") or 0) & proto.HF_TAMPER),
                        "noise_floor": n.get("noise_floor"),
                        "rssi": n.get("rssi"), "snr": n.get("snr"),
                        "fw_version": n.get("fw_version"),
                        "reset_count": n.get("reset_count"),
                    })
                self._json({"nodes": out, "now": time.time()})
            elif path.endswith("/gw/alerts"):
                limit = int(q.get("limit", ["50"])[0])
                self._json({"alerts": db.recent_alerts(min(limit, 500))})
            elif path.endswith("/gw/events"):
                since = float(q.get("since", ["0"])[0])
                self._json({"events": feed.since(since), "now": time.time()})
            elif "/gw/" in path:
                self.send_error(404)
            else:
                # anything else (/, /index.html, or a proxy base path) is the GUI
                self._file("index.html", "text/html; charset=utf-8")

    return Handler


def serve(db: Db, feed: EventFeed, status_cfg: dict, host="0.0.0.0", port=8080):
    httpd = ThreadingHTTPServer((host, port), make_handler(db, feed, status_cfg))
    t = threading.Thread(target=httpd.serve_forever, daemon=True, name="web")
    t.start()
    return httpd
