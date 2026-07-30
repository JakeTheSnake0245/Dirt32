"""Node status classification — the map dot colors.

User-specified rules:
  GREEN  — heartbeats arriving as expected.
  YELLOW — no heartbeat for half a day, OR low battery, OR health error
           (sensor fail / self-test fail).
  RED    — tamper flag, moved further than expected from its provisioned
           position, or silent for a whole day.
"""
from __future__ import annotations
import math
import time

from . import proto

DEFAULTS = {
    "yellow_silence_s": 12 * 3600,
    "red_silence_s": 24 * 3600,
    "low_battery_mv": 3300,
    "move_threshold_m": 30.0,
}


def distance_m(lat1_e7, lon1_e7, lat2_e7, lon2_e7) -> float:
    """Haversine over e7 coordinates."""
    lat1, lon1 = lat1_e7 / 1e7, lon1_e7 / 1e7
    lat2, lon2 = lat2_e7 / 1e7, lon2_e7 / 1e7
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def classify(node: dict, cfg: dict | None = None, now: float | None = None):
    """Returns (color, [reasons]). node is a Db nodes row."""
    c = {**DEFAULTS, **(cfg or {})}
    now = now or time.time()
    reasons = []
    color = "green"

    def escalate(level, why):
        nonlocal color
        reasons.append(why)
        order = {"green": 0, "yellow": 1, "red": 2}
        if order[level] > order[color]:
            color = level

    last = node.get("last_seen")
    if last is None:
        return "red", ["never heard from"]

    silence = now - last
    if silence > c["red_silence_s"]:
        escalate("red", f"silent for {silence/3600:.1f} h")
    elif silence > c["yellow_silence_s"]:
        escalate("yellow", f"no heartbeat for {silence/3600:.1f} h")

    flags = node.get("health_flags")
    if flags is not None:
        if flags & proto.HF_TAMPER:
            escalate("red", "tamper flag set")
        if not flags & proto.HF_SENSOR_OK:
            escalate("yellow", "sensor fault")
        if not flags & proto.HF_SELFTEST:
            escalate("yellow", "self-test failed")

    batt = node.get("battery_mv")
    if batt is not None and 0 < batt < c["low_battery_mv"]:
        escalate("yellow", f"low battery {batt} mV")

    # moved: compare last GPS-fixed heartbeat position with provisioned
    if (flags is not None and flags & proto.HF_GPS_FIX
            and node.get("lat_e7") and node.get("hb_lat_e7")):
        d = distance_m(node["lat_e7"], node["lon_e7"],
                       node["hb_lat_e7"], node["hb_lon_e7"])
        if d > c["move_threshold_m"]:
            escalate("red", f"moved {d:.0f} m from provisioned position")

    return color, reasons or ["healthy"]
