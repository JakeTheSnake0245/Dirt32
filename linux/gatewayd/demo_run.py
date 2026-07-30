"""Run the Dirt32 gateway in simulator mode for the Replit preview.

Reads PORT from the environment (injected by the managed workflow) and
starts the real daemon — real crypto, ingest, replay, DB — fed by the
simulator instead of a radio bridge. Not used on the Pi.
"""
import json
import os
import sys
import tempfile

port = int(os.environ.get("PORT", "8080"))
cfg = {
    "simulator": True,
    "mqtt": {"enabled": False},
    "web": {"host": "0.0.0.0", "port": port},
    "db_path": os.path.join(tempfile.gettempdir(), "dirt32-demo.sqlite3"),
}
# fresh demo state on every start
try:
    os.remove(cfg["db_path"])
except FileNotFoundError:
    pass

cfg_path = os.path.join(tempfile.gettempdir(), "dirt32-demo.json")
with open(cfg_path, "w") as f:
    json.dump(cfg, f)

sys.argv = [sys.argv[0], cfg_path]
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from dirt32_gateway import main  # noqa: E402
main.main()
