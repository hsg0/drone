#!/usr/bin/env python3
"""Draw a lat/lon path in a browser on the Pi, save it, and fly it with no internet.

Plan at home (house Wi-Fi, map tiles load):

    python3 ~/Desktop/scripts/flight_path_planner.py serve

Open http://127.0.0.1:8080 on the Pi, or http://10.0.0.40:8080 from the Mac.
Click the map. Save. The file stays in ~/flight_plans/ on the Pi.

Fly later, including after the Pi has left the house network. This does not
need the browser, Wi-Fi, or map tiles. It will arm and take off:

    python3 ~/Desktop/scripts/flight_path_planner.py fly ~/flight_plans/NAME.json --confirm

What fly does:
  1. Pre-flight checks (same bar as hover_test.py): GPS, EKF home, battery,
     level, safety switch solid, PreArm, transmitter on.
  2. GUIDED, arm, wait 2.2 s, take off to the plan altitude.
  3. Fly each saved point in order at the plan speed, nose toward the next
     point. If the plan says return home, fly back to the launch point.
  4. Land.

Failsafes while the path is healthy:
  - Cross-track farther than the corridor, altitude above the ceiling,
    tilt, battery, or lost telemetry: LAND.
  - Transmitter stick or switch: stop guiding and switch to STABILIZE.
    Throttle and yaw are yours. Yaw left disarms after landing.

If GPS or the compass is bad for 4 seconds:
  - Those 4 seconds are a position hold on the last guided target.
  - Then the copter yaws two full turns (a visual signal) and holds.
  - Move the transmitter to take STABILIZE. It does not auto-land from
    this signal unless the battery sags, the tilt limit trips, or you
    tell it to land.
"""

from __future__ import annotations

import argparse
import csv
import datetime
import errno
import json
import math
import os
import re
import signal
import struct
import sys
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from types import SimpleNamespace
from urllib.parse import urlparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hover_test as ht

PLANS_DIR = os.path.expanduser("~/flight_plans")
LOG_DIR = os.path.expanduser("~/flight_path_logs")
HERE_LOCK = threading.Lock()
EARTH_M = 6378137.0

ALT_FT_MIN, ALT_FT_MAX = 3.0, 30.0
SPEED_FPS_MIN, SPEED_FPS_MAX = 1.0, 12.0
CORRIDOR_FT_MIN, CORRIDOR_FT_MAX = 4.0, 40.0
SUBPLOT_FT_MIN, SUBPLOT_FT_MAX = 3.0, 50.0
SPIN_TURNS = 2.0
SPIN_DEG_PER_S = 40.0
NAV_BAD_HOLD_S = 4.0
RC_RESUME_IDLE_S = ht.RC_RESUME_IDLE_S
WP_ACCEPT_M = 0.60
HOME_ACCEPT_M = 0.35
HOME_SETTLE_S = 2.5
CARROT_M = 1.25
CEILING_EXTRA_FT = 8.0


class MissionManager:
    """Tracks an active or recent flight execution started from the UI or CLI."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.running = False
        self.phase = "IDLE"
        self.plan_name = ""
        self.leg = 0
        self.leg_total = 0
        self.sub_index = 0
        self.sub_total = 0
        self.countdown_s = 0
        self.message = "Idle"
        self.logs: list[str] = []
        self.last_snap: dict | None = None
        self.abort_requested = False
        self.land_requested = False
        self.thread: threading.Thread | None = None

    def add_log(self, text: str) -> None:
        clean = text.strip()
        if not clean:
            return
        stamp = datetime.datetime.now().strftime("%H:%M:%S")
        with self.lock:
            self.message = clean
            self.logs.append(f"[{stamp}] {clean}")
            if len(self.logs) > 40:
                self.logs = self.logs[-40:]

    def snapshot(self) -> dict:
        with self.lock:
            return {
                "running": self.running,
                "phase": self.phase,
                "plan_name": self.plan_name,
                "leg": self.leg,
                "leg_total": self.leg_total,
                "sub_index": self.sub_index,
                "sub_total": self.sub_total,
                "countdown_s": self.countdown_s,
                "message": self.message,
                "logs": list(self.logs[-12:]),
                "drone": self.last_snap,
            }


MISSION = MissionManager()


def log(msg: str) -> None:
    print(msg, flush=True)
    MISSION.add_log(msg)


def plans_dir() -> str:
    os.makedirs(PLANS_DIR, exist_ok=True)
    return PLANS_DIR


def safe_name(name: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9 _-]", "", (name or "").strip())
    cleaned = re.sub(r"\s+", " ", cleaned).strip(" .")
    if not cleaned:
        raise ValueError("Plan name is empty")
    return cleaned[:40]


def plan_path(name: str) -> str:
    return os.path.join(plans_dir(), safe_name(name) + ".json")


def latlon_to_ne(lat: float, lon: float, home_lat: float, home_lon: float) -> tuple[float, float]:
    north = math.radians(lat - home_lat) * EARTH_M
    east = math.radians(lon - home_lon) * EARTH_M * math.cos(math.radians(home_lat))
    return north, east


def cross_track_m(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    dx, dy = bx - ax, by - ay
    length_sq = dx * dx + dy * dy
    if length_sq < 1e-6:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def ne_to_latlon(north_m: float, east_m: float, home_lat: float, home_lon: float) -> tuple[float, float]:
    lat = home_lat + math.degrees(north_m / EARTH_M)
    cos_lat = max(1e-6, math.cos(math.radians(home_lat)))
    lon = home_lon + math.degrees(east_m / (EARTH_M * cos_lat))
    return lat, lon


def generate_subplots(
    waypoints: list[dict],
    step_ft: float,
    return_home: bool,
    start_point: dict | None = None,
) -> list[dict]:
    """Build hidden intermediate lat/lon checkpoints every `step_ft` feet along each leg."""
    if not waypoints or step_ft <= 0:
        return []
    anchors: list[tuple[float, float]] = []
    if start_point and "lat" in start_point and "lon" in start_point:
        anchors.append((float(start_point["lat"]), float(start_point["lon"])))
    for wp in waypoints:
        anchors.append((float(wp["lat"]), float(wp["lon"])))
    if return_home and len(anchors) >= 2:
        anchors.append(anchors[0])
    if len(anchors) < 2:
        return []

    subplots: list[dict] = []
    cum_ft = 0.0
    for leg_idx in range(len(anchors) - 1):
        lat1, lon1 = anchors[leg_idx]
        lat2, lon2 = anchors[leg_idx + 1]
        dn, de = latlon_to_ne(lat2, lon2, lat1, lon1)
        leg_m = math.hypot(dn, de)
        leg_ft = leg_m * ht.M_TO_FT
        bearing_deg = (math.degrees(math.atan2(de, dn)) + 360.0) % 360.0
        steps = max(1, int(math.ceil(leg_ft / step_ft))) if leg_ft > 0.1 else 1
        for k in range(1, steps + 1):
            dist_ft = min(leg_ft, k * step_ft)
            frac = (dist_ft / leg_ft) if leg_ft > 1e-6 else 1.0
            lat = lat1 + (lat2 - lat1) * frac
            lon = lon1 + (lon2 - lon1) * frac
            subplots.append(
                {
                    "leg": leg_idx + 1,
                    "sub_index": k,
                    "sub_count": steps,
                    "lat": round(lat, 7),
                    "lon": round(lon, 7),
                    "dist_leg_ft": round(dist_ft, 1),
                    "leg_total_ft": round(leg_ft, 1),
                    "cum_dist_ft": round(cum_ft + dist_ft, 1),
                    "bearing_deg": round(bearing_deg, 1),
                    "is_master": k == steps,
                }
            )
        cum_ft += leg_ft
    return subplots


def parse_plan(data: dict) -> dict:
    if not isinstance(data, dict):
        raise ValueError("Plan must be a JSON object")
    name = safe_name(str(data.get("name") or "path"))
    try:
        alt_ft = float(data.get("alt_ft", 8))
        speed_fps = float(data.get("speed_fps", 4))
        corridor_ft = float(data.get("corridor_ft", 8))
        pause_s = float(data.get("pause_s", 1))
        subplot_step_ft = float(data.get("subplot_step_ft", 10))
    except (TypeError, ValueError) as exc:
        raise ValueError("Altitude, speed, corridor, pause, and sub-plot step must be numbers") from exc
    if not (ALT_FT_MIN <= alt_ft <= ALT_FT_MAX):
        raise ValueError(f"Altitude must be {ALT_FT_MIN:.0f}–{ALT_FT_MAX:.0f} ft")
    if not (SPEED_FPS_MIN <= speed_fps <= SPEED_FPS_MAX):
        raise ValueError(f"Speed must be {SPEED_FPS_MIN:.0f}–{SPEED_FPS_MAX:.0f} ft/s")
    if not (CORRIDOR_FT_MIN <= corridor_ft <= CORRIDOR_FT_MAX):
        raise ValueError(f"Corridor must be {CORRIDOR_FT_MIN:.0f}–{CORRIDOR_FT_MAX:.0f} ft")
    if not (SUBPLOT_FT_MIN <= subplot_step_ft <= SUBPLOT_FT_MAX):
        raise ValueError(f"Sub-plot spacing must be {SUBPLOT_FT_MIN:.0f}–{SUBPLOT_FT_MAX:.0f} ft")
    pause_s = max(0.0, min(10.0, pause_s))
    use_subplots = bool(data.get("use_subplots", True))
    return_home = bool(data.get("return_home", True))
    raw_points = data.get("waypoints")
    if not isinstance(raw_points, list) or not raw_points:
        raise ValueError("Add at least one waypoint")
    if len(raw_points) > 40:
        raise ValueError("A plan can have at most 40 waypoints")
    points = []
    for item in raw_points:
        if not isinstance(item, dict):
            raise ValueError("Each waypoint needs lat and lon")
        lat = float(item["lat"])
        lon = float(item["lon"])
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            raise ValueError(f"Bad coordinate {lat}, {lon}")
        points.append({"lat": lat, "lon": lon})
    start_point = None
    raw_start = data.get("start_point")
    if isinstance(raw_start, dict) and "lat" in raw_start and "lon" in raw_start:
        try:
            slat = float(raw_start["lat"])
            slon = float(raw_start["lon"])
            if -90.0 <= slat <= 90.0 and -180.0 <= slon <= 180.0 and (abs(slat) > 0.01 or abs(slon) > 0.01):
                start_point = {"lat": round(slat, 7), "lon": round(slon, 7)}
        except (TypeError, ValueError):
            start_point = None
    subplots = (
        generate_subplots(points, subplot_step_ft, return_home, start_point=start_point)
        if use_subplots
        else []
    )
    return {
        "name": name,
        "alt_ft": alt_ft,
        "speed_fps": speed_fps,
        "corridor_ft": corridor_ft,
        "pause_s": pause_s,
        "return_home": return_home,
        "use_subplots": use_subplots,
        "subplot_step_ft": subplot_step_ft,
        "start_point": start_point,
        "waypoints": points,
        "subplots": subplots,
    }


def load_plan(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as handle:
        return parse_plan(json.load(handle))


def save_plan(plan: dict) -> str:
    path = plan_path(plan["name"])
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(plan, handle, indent=2)
        handle.write("\n")
    return path


def list_plans() -> list[dict]:
    found = []
    for name in sorted(os.listdir(plans_dir())):
        if not name.endswith(".json"):
            continue
        path = os.path.join(PLANS_DIR, name)
        try:
            plan = load_plan(path)
        except (OSError, ValueError, json.JSONDecodeError, KeyError):
            continue
        found.append(
            {
                "name": plan["name"],
                "file": path,
                "alt_ft": plan["alt_ft"],
                "speed_fps": plan["speed_fps"],
                "corridor_ft": plan["corridor_ft"],
                "pause_s": plan["pause_s"],
                "return_home": plan["return_home"],
                "use_subplots": plan["use_subplots"],
                "subplot_step_ft": plan["subplot_step_ft"],
                "subplot_count": len(plan.get("subplots") or []),
                "waypoints": plan["waypoints"],
            }
        )
    return found


def describe_plan(plan: dict) -> None:
    sub_str = (
        f"hidden sub-plots every {plan['subplot_step_ft']:.0f} ft ({len(plan.get('subplots') or [])} pts)"
        if plan.get("use_subplots", True)
        else "hidden sub-plots off"
    )
    log(
        f"Plan '{plan['name']}': {len(plan['waypoints'])} points, "
        f"{plan['alt_ft']:.0f} ft, {plan['speed_fps']:.1f} ft/s, "
        f"corridor {plan['corridor_ft']:.0f} ft, pause {plan['pause_s']:.0f} s, "
        f"return home {'yes' if plan['return_home'] else 'no'}, {sub_str}"
    )
    for index, point in enumerate(plan["waypoints"], start=1):
        log(f"  {index:2d}  {point['lat']:.7f}, {point['lon']:.7f}")
    log("Takeoff is wherever the drone is sitting when fly starts, not a point on the map.")


def send_attitude(conn: ht.MavlinkConnection, yaw_rad: float, yaw_rate: float, thrust: float = 0.5) -> None:
    half = yaw_rad * 0.5
    ms = int(time.time() * 1000) & 0xFFFFFFFF
    payload = struct.pack(
        "<I4fffffBBB",
        ms,
        math.cos(half),
        0.0,
        0.0,
        math.sin(half),
        0.0,
        0.0,
        float(yaw_rate),
        float(thrust),
        ht.TGT_SYS,
        ht.TGT_COMP,
        0,
    )
    conn.send_msg(82, payload)


def read_state(conn: ht.MavlinkConnection) -> SimpleNamespace:
    now = time.time()
    with conn.state.lock:
        state = conn.state
        return SimpleNamespace(
            armed=state.armed,
            mode=state.custom_mode,
            x=state.x,
            y=state.y,
            z=state.z,
            vx=state.vx,
            vy=state.vy,
            vz=state.vz,
            rel_alt=state.rel_alt_m,
            roll=math.degrees(state.roll_rad),
            pitch=math.degrees(state.pitch_rad),
            yaw=state.yaw_rad,
            volt=state.volt_v,
            curr=state.curr_a,
            fix=state.gps_fix,
            sats=state.gps_sats,
            hdop=state.gps_hdop,
            ekf=state.ekf_flags,
            compass_var=state.ekf_compass_var,
            rc=state.rc_chans,
            lpos_age=now - state.lpos_time,
            att_age=now - state.att_time,
            gps_age=now - state.gps_time if state.gps_time else 99.0,
            lat=state.lat_deg,
            lon=state.lon_deg,
        )


def nav_problem(state: SimpleNamespace) -> str | None:
    abs_missing = state.ekf != 0 and not bool(state.ekf & ht.EKF_POS_HORIZ_ABS)
    gps_bad = (
        state.fix < 3
        or state.sats < 5
        or state.hdop > 3.0
        or state.gps_age > 3.0
        or bool(state.ekf & ht.EKF_CONST_POS_MODE)
        or abs_missing
    )
    compass_bad = state.compass_var > 1.0
    if gps_bad and compass_bad:
        return (
            f"GPS and compass degraded (fix={state.fix}, sats={state.sats}, "
            f"hdop={state.hdop:.2f}, compass var={state.compass_var:.2f})"
        )
    if gps_bad:
        return f"GPS degraded (fix={state.fix}, sats={state.sats}, hdop={state.hdop:.2f}, ekf=0x{state.ekf:02x})"
    if compass_bad:
        return f"Compass degraded (EKF compass var={state.compass_var:.2f})"
    return None


def hand_to_pilot(conn: ht.MavlinkConnection, reason: str) -> None:
    log(f"\n>>> PILOT HAS THE AIRCRAFT: {reason}")
    log(">>> Mode STABILIZE. Sticks work. Land it yourself, then yaw left to disarm. <<<")
    conn.set_mode(ht.MODE_STABILIZE, timeout=2.0)


parse_rc_nudge = ht.parse_rc_nudge


def altitude_m(state: SimpleNamespace, start_z: float) -> float:
    local_alt = -(state.z - start_z)
    if abs(local_alt - state.rel_alt) < 0.8:
        return local_alt
    return state.rel_alt


def build_leg_subplots_ne(
    from_ne: tuple[float, float],
    to_ne: tuple[float, float],
    step_m: float,
) -> list[tuple[float, float, float]]:
    """Return [(north_m, east_m, dist_along_m), ...] spaced every step_m along a leg."""
    dx = to_ne[0] - from_ne[0]
    dy = to_ne[1] - from_ne[1]
    leg_len = math.hypot(dx, dy)
    if leg_len < 0.05 or step_m <= 0.1:
        return [(to_ne[0], to_ne[1], leg_len)]
    steps = max(1, int(math.ceil(leg_len / step_m)))
    pts: list[tuple[float, float, float]] = []
    for k in range(1, steps + 1):
        d = min(leg_len, k * step_m)
        scale = d / leg_len
        pts.append((from_ne[0] + dx * scale, from_ne[1] + dy * scale, d))
    return pts


def open_mavlink_with_retry(close_qgc: bool = True, timeout: float = 5.0) -> ht.MavlinkConnection:
    ht.ensure_port_free(close_qgc=close_qgc)
    deadline = time.time() + timeout
    last_exc: Exception | None = None
    while time.time() < deadline:
        try:
            return ht.MavlinkConnection()
        except Exception as exc:
            last_exc = exc
            time.sleep(0.25)
    if last_exc:
        raise last_exc
    return ht.MavlinkConnection()


def fly_plan(plan: dict, args) -> int:
    target_alt_m = plan["alt_ft"] * ht.FT_TO_M
    speed_mps = plan["speed_fps"] * ht.FT_TO_M
    corridor_m = plan["corridor_ft"] * ht.FT_TO_M
    ceiling_m = (plan["alt_ft"] + CEILING_EXTRA_FT) * ht.FT_TO_M
    use_subplots = bool(plan.get("use_subplots", True))
    step_m = float(plan.get("subplot_step_ft", 10.0)) * ht.FT_TO_M
    max_tilt = float(getattr(args, "max_tilt_deg", 25.0))
    low_volt = float(getattr(args, "low_volt_cutoff", ht.DEFAULT_LOW_VOLT_CUTOFF))
    min_takeoff_volt = float(getattr(args, "min_takeoff_volt", ht.DEFAULT_MIN_TAKEOFF_VOLT))
    gps_wait = float(getattr(args, "gps_wait", 45.0))
    close_qgc = bool(getattr(args, "close_qgc", True))
    start_delay_s = max(0, int(round(float(getattr(args, "start_delay_s", 0)))))

    with MISSION.lock:
        MISSION.running = True
        MISSION.abort_requested = False
        MISSION.land_requested = False
        MISSION.plan_name = plan["name"]
        MISSION.phase = "COUNTDOWN" if start_delay_s > 0 else "PREFLIGHT"
        MISSION.leg = 0
        MISSION.leg_total = len(plan["waypoints"]) + (1 if plan["return_home"] else 0)
        MISSION.sub_index = 0
        MISSION.sub_total = 0
        MISSION.countdown_s = start_delay_s

    abort_reason = None
    mission_done = False

    def on_signal(signum, _frame):
        nonlocal abort_reason
        abort_reason = f"Received signal {signum}"

    try:
        signal.signal(signal.SIGINT, on_signal)
        signal.signal(signal.SIGTERM, on_signal)
    except ValueError:
        pass

    # Optional pre-arm countdown (so user can walk outside with RC controller)
    if start_delay_s > 0:
        log(f"Starting {start_delay_s} s countdown before pre-flight & takeoff (RC fallback ready)...")
        for rem in range(start_delay_s, 0, -1):
            with MISSION.lock:
                MISSION.countdown_s = rem
                if MISSION.abort_requested or MISSION.land_requested:
                    abort_reason = "Countdown cancelled by user"
                    break
            if abort_reason:
                break
            if rem in (60, 45, 30, 20, 15, 10, 5, 4, 3, 2, 1):
                log(f"Takeoff countdown: {rem} s...")
            time.sleep(1.0)
        with MISSION.lock:
            MISSION.countdown_s = 0
        if abort_reason:
            log(abort_reason)
            with MISSION.lock:
                MISSION.running = False
                MISSION.phase = "ABORTED"
            return 1

    with HERE_LOCK:
        try:
            conn = open_mavlink_with_retry(close_qgc=close_qgc, timeout=5.0)
        except Exception as exc:
            log(f"Could not open /dev/ttyAMA0: {exc}")
            with MISSION.lock:
                MISSION.running = False
                MISSION.phase = "ERROR"
            return 1

    try:
        with MISSION.lock:
            MISSION.phase = "PREFLIGHT"
        log("Configuring telemetry on /dev/ttyAMA0...")
        conn.configure_streams()
        conn.send_command_long(ht.MAV_CMD_RUN_PREARM_CHECKS)
        time.sleep(2.5)

        wait_deadline = time.time() + gps_wait
        while time.time() < wait_deadline and abort_reason is None:
            with MISSION.lock:
                if MISSION.abort_requested or MISSION.land_requested:
                    abort_reason = "Cancelled during pre-flight"
                    break
                MISSION.last_snap = drone_snapshot(conn)
            with conn.state.lock:
                ready, issues = ht.evaluate_readiness(conn.state, min_volt=min_takeoff_volt)
                fix = conn.state.gps_fix
                sats = conn.state.gps_sats
                hdop = conn.state.gps_hdop
                ekf_flags = conn.state.ekf_flags
            if ready:
                break
            log(
                f"  [WAITING] fix={fix} sats={sats} hdop={hdop:.2f} ekf=0x{ekf_flags:02x} | "
                + "; ".join(issues)
            )
            conn.send_command_long(ht.MAV_CMD_RUN_PREARM_CHECKS)
            time.sleep(2.0)

        with conn.state.lock:
            ready, issues = ht.evaluate_readiness(conn.state, min_volt=min_takeoff_volt)
            home_lat = conn.state.lat_deg
            home_lon = conn.state.lon_deg
            baseline_rc = conn.state.rc_chans
        if abort_reason:
            log(abort_reason)
            conn.stop_streams()
            return 1
        if not ready:
            log("\nPre-flight checks did not pass:")
            for item in issues:
                log(f"  - {item}")
            log("Not arming.")
            conn.stop_streams()
            return 1
        if abs(home_lat) < 0.01 and abs(home_lon) < 0.01:
            log("GPS position is still 0,0. Not arming.")
            conn.stop_streams()
            return 1

        log(
            f"RC Remote paired & live: CH1..4={baseline_rc[:4]} "
            f"(touch sticks anytime to steer; release for {RC_RESUME_IDLE_S:.0f}s to auto-resume path)"
        )

        try:
            signal.signal(signal.SIGHUP, signal.SIG_IGN)
        except ValueError:
            pass
        os.makedirs(LOG_DIR, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(LOG_DIR, f"path_{stamp}.csv")
        csv_file = open(csv_path, "w", newline="")
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "timestamp", "phase", "wp", "subplot", "alt_ft", "cross_ft", "north_ft", "east_ft",
                "roll_deg", "pitch_deg", "yaw_deg", "volt_v", "curr_a", "sats", "hdop", "note",
            ]
        )

        route = []
        for point in plan["waypoints"]:
            north, east = latlon_to_ne(point["lat"], point["lon"], home_lat, home_lon)
            route.append((north, east, point["lat"], point["lon"]))
        if plan["return_home"]:
            route.append((0.0, 0.0, home_lat, home_lon))

        log(f"\nLaunch GPS locked: {home_lat:.7f}, {home_lon:.7f}")
        total_m = 0.0
        prev = (0.0, 0.0)
        for index, (north, east, lat, lon) in enumerate(route, start=1):
            leg = math.hypot(north - prev[0], east - prev[1])
            total_m += leg
            sub_cnt = len(build_leg_subplots_ne(prev, (north, east), step_m)) if use_subplots else 1
            sub_info = f", {sub_cnt} sub-plots @ {plan['subplot_step_ft']:.0f}ft" if use_subplots else ""
            log(f"  leg {index}: {lat:.7f}, {lon:.7f}  ({leg * ht.M_TO_FT:.0f} ft{sub_info})")
            prev = (north, east)
        log(f"Path length {total_m * ht.M_TO_FT:.0f} ft. Logging to {csv_path}")
        log("Switching to GUIDED...")
        if not conn.set_mode(ht.MODE_GUIDED, timeout=3.0):
            log("GUIDED was refused. Not arming.")
            conn.stop_streams()
            csv_file.close()
            return 1

        log("Arming...")
        armed_ok, reasons = conn.arm(timeout=4.0)
        if not armed_ok:
            log("Arming refused:")
            for reason in reasons:
                log(f"  - {reason}")
            conn.stop_streams()
            csv_file.close()
            return 1
        log("ARMED. Waiting 2.2 s for the arming delay and home reset...")
        time.sleep(2.2)
        with conn.state.lock:
            start_x = conn.state.x
            start_y = conn.state.y
            start_z = conn.state.z
            start_yaw = conn.state.yaw_rad
            home_lat = conn.state.lat_deg or home_lat
            home_lon = conn.state.lon_deg or home_lon
        route = []
        for point in plan["waypoints"]:
            north, east = latlon_to_ne(point["lat"], point["lon"], home_lat, home_lon)
            route.append((north, east))
        if plan["return_home"]:
            route.append((0.0, 0.0))

        leg_subplots = build_leg_subplots_ne((0.0, 0.0), route[0], step_m) if use_subplots else [(route[0][0], route[0][1], math.hypot(*route[0]))]
        sub_idx = 0
        last_verified_sub = (0.0, 0.0)

        log(f"Takeoff to {plan['alt_ft']:.1f} ft...")
        if not conn.command_takeoff(target_alt_m, timeout=4.5):
            abort_reason = "Takeoff command was rejected"

        phase = "TAKEOFF"
        takeoff_started = time.time()
        leg_index = 0
        leg_from = (0.0, 0.0)
        progress_m = 0.0
        pause_until = 0.0
        leg_deadline = time.time() + 60.0
        bad_since = None
        spin_yaw = start_yaw
        spin_left = 0.0
        hold_n = 0.0
        hold_e = 0.0
        note = ""
        last_send = 0.0
        last_print = 0.0
        low_volt_since = None
        cmd_n = 0.0
        cmd_e = 0.0
        cmd_yaw = start_yaw
        cmd_alt_m = target_alt_m
        trim_n_m = 0.0
        trim_e_m = 0.0
        ki_trim = 0.08
        max_trim_m = 0.40
        home_settle_until = 0.0
        override_n = 0.0
        override_e = 0.0
        override_alt_m = target_alt_m
        override_yaw = start_yaw
        rc_idle_since = None
        thr_centered_once = False

        while abort_reason is None and not mission_done:
            now = time.time()
            state = read_state(conn)
            with MISSION.lock:
                MISSION.phase = phase
                MISSION.leg = leg_index + 1
                MISSION.leg_total = len(route)
                MISSION.sub_index = (sub_idx + 1) if use_subplots else 0
                MISSION.sub_total = len(leg_subplots) if use_subplots else 0
                MISSION.last_snap = drone_snapshot(conn)
                if MISSION.land_requested or MISSION.abort_requested:
                    abort_reason = "User clicked Land Now / Stop in UI"
                    break

            if not state.armed:
                log("\nPixhawk disarmed.")
                break
            if state.mode not in (ht.MODE_GUIDED, ht.MODE_LAND) and phase not in ("SIGNAL_SPIN", "SIGNAL_HOLD"):
                log(f"\nMode changed to {ht.MODE_NAMES.get(state.mode, state.mode)}. Stopping guidance.")
                csv_file.close()
                return 0

            north_m = state.x - start_x
            east_m = state.y - start_y
            alt_m = altitude_m(state, start_z)
            alt_ft = alt_m * ht.M_TO_FT

            sw_flip, sw_reason, norm_fwd, norm_right, norm_yaw, norm_climb, thr_centered_once = parse_rc_nudge(
                baseline_rc, state.rc, thr_centered_once
            )
            if sw_flip and sw_reason:
                abort_reason = f"RC SWITCH LAND: {sw_reason}"
                break
            sticks_active = (norm_fwd != 0.0 or norm_right != 0.0 or norm_yaw != 0.0 or norm_climb != 0.0)
            if sticks_active and (phase != "TAKEOFF" or alt_m >= 0.8):
                if phase != "RC_OVERRIDE":
                    override_n = north_m
                    override_e = east_m
                    override_alt_m = max(alt_m, target_alt_m)
                    override_yaw = state.yaw
                    phase = "RC_OVERRIDE"
                    log(
                        f"\n>>> RC OVERRIDE: Pilot moving sticks — steering from RC. "
                        f"Release sticks for {RC_RESUME_IDLE_S:.0f} s to resume flight path. <<<"
                    )
                rc_idle_since = None

            if abs(state.roll) > max_tilt or abs(state.pitch) > max_tilt:
                abort_reason = f"Tilt exceeded {max_tilt:.0f} deg (roll={state.roll:+.1f}, pitch={state.pitch:+.1f})"
                break
            if alt_m > ceiling_m and phase != "TAKEOFF":
                abort_reason = f"Altitude {alt_ft:.1f} ft exceeded the ceiling {ceiling_m * ht.M_TO_FT:.0f} ft"
                break
            if 1.0 < state.volt < ht.DEFAULT_CRIT_VOLT_CUTOFF:
                abort_reason = f"Battery sag {state.volt:.2f} V"
                break
            if 1.0 < state.volt < low_volt:
                if low_volt_since is None:
                    low_volt_since = now
                elif now - low_volt_since >= 1.5:
                    abort_reason = f"Battery below {low_volt:.2f} V for 1.5 s ({state.volt:.2f} V)"
                    break
            else:
                low_volt_since = None

            problem = nav_problem(state)
            if phase in ("LEG", "PAUSE", "HOME_HOLD") or (phase == "TAKEOFF" and alt_m > 1.0):
                if problem:
                    if bad_since is None:
                        bad_since = now
                        if use_subplots:
                            cmd_n, cmd_e = last_verified_sub
                        log(
                            f"Nav warning, holding at sub-plot {sub_idx + 1}/{len(leg_subplots)} for {NAV_BAD_HOLD_S:.0f} s: {problem}"
                            if use_subplots
                            else f"Nav warning, holding for {NAV_BAD_HOLD_S:.0f} s: {problem}"
                        )
                    elif now - bad_since >= NAV_BAD_HOLD_S and phase != "SIGNAL_SPIN":
                        phase = "SIGNAL_SPIN"
                        spin_yaw = state.yaw
                        spin_left = SPIN_TURNS * 2.0 * math.pi
                        hold_n, hold_e = (last_verified_sub if use_subplots else (cmd_n, cmd_e))
                        note = problem
                        log(f"\n>>> GPS/COMPASS SIGNAL: {problem}")
                        log(">>> Yawing two turns, then holding for the transmitter. <<<")
                else:
                    bad_since = None
            elif phase == "TAKEOFF" and problem:
                abort_reason = problem

            cross_m = 0.0
            if phase == "RC_OVERRIDE":
                if sticks_active:
                    vx_fwd = norm_fwd * 1.8
                    vy_right = norm_right * 1.8
                    vn = vx_fwd * math.cos(state.yaw) - vy_right * math.sin(state.yaw)
                    ve = vx_fwd * math.sin(state.yaw) + vy_right * math.cos(state.yaw)
                    override_n = north_m + vn * 0.85
                    override_e = east_m + ve * 0.85
                    override_yaw = ht.wrap_pi(override_yaw + norm_yaw * math.radians(45.0) * 0.1)
                    override_alt_m = max(1.1, min(ceiling_m - 0.3, override_alt_m + norm_climb * 0.7 * 0.1))
                    cmd_n, cmd_e, cmd_yaw, cmd_alt_m = override_n, override_e, override_yaw, override_alt_m
                    trim_n_m = 0.0
                    trim_e_m = 0.0
                    note = f"rc steer (fwd={norm_fwd:+.1f} rgt={norm_right:+.1f})"
                else:
                    if rc_idle_since is None:
                        rc_idle_since = now
                        override_n = north_m
                        override_e = east_m
                        override_yaw = state.yaw
                        log(
                            f"RC sticks released — holding position. "
                            f"Returning to flight path in {RC_RESUME_IDLE_S:.0f} s..."
                        )
                    idle_s = now - rc_idle_since
                    cmd_n, cmd_e, cmd_yaw, cmd_alt_m = override_n, override_e, override_yaw, override_alt_m
                    note = f"rc idle {max(0.0, RC_RESUME_IDLE_S - idle_s):.1f}s"
                    if idle_s >= RC_RESUME_IDLE_S:
                        leg_from = (north_m, east_m)
                        progress_m = 0.0
                        trim_n_m = 0.0
                        trim_e_m = 0.0
                        cmd_alt_m = target_alt_m
                        leg_subplots = (
                            build_leg_subplots_ne(leg_from, route[leg_index], step_m)
                            if use_subplots
                            else [
                                (
                                    route[leg_index][0],
                                    route[leg_index][1],
                                    math.hypot(route[leg_index][0] - leg_from[0], route[leg_index][1] - leg_from[1]),
                                )
                            ]
                        )
                        sub_idx = 0
                        last_verified_sub = leg_from
                        next_len = math.hypot(route[leg_index][0] - leg_from[0], route[leg_index][1] - leg_from[1])
                        leg_deadline = now + max(25.0, 4.0 * next_len / max(speed_mps, 0.3))
                        phase = "LEG"
                        rc_idle_since = None
                        thr_centered_once = False
                        note = "resumed path after 5s rc idle"
                        log(
                            f"\n>>> RC untouched for {RC_RESUME_IDLE_S:.0f} s — "
                            f"resuming flight path toward point {leg_index + 1}/{len(route)} <<<"
                        )
            elif phase == "TAKEOFF":
                cmd_n, cmd_e, cmd_yaw, cmd_alt_m = 0.0, 0.0, start_yaw, target_alt_m
                if state.lpos_age > 1.5 or state.att_age > 1.5:
                    abort_reason = f"Telemetry timeout during takeoff (lpos {state.lpos_age:.1f}s)"
                elif alt_m >= target_alt_m - 0.25 or (now - takeoff_started >= 8.0 and alt_m >= 0.6):
                    phase = "LEG"
                    progress_m = 0.0
                    leg_subplots = (
                        build_leg_subplots_ne(leg_from, route[0], step_m)
                        if use_subplots
                        else [(route[0][0], route[0][1], math.hypot(*route[0]))]
                    )
                    sub_idx = 0
                    last_verified_sub = leg_from
                    leg_deadline = now + max(25.0, 4.0 * math.hypot(*route[0]) / max(speed_mps, 0.3))
                    log(f"\n>>> At {alt_ft:.1f} ft. Flying the path. <<<")
                elif now - takeoff_started > 15.0:
                    abort_reason = f"Takeoff did not reach altitude ({alt_ft:.1f} ft)"
            elif phase == "LEG":
                cmd_alt_m = target_alt_m
                target_n, target_e = route[leg_index]
                leg_len = math.hypot(target_n - leg_from[0], target_e - leg_from[1])
                if now > leg_deadline:
                    abort_reason = f"Did not reach point {leg_index + 1} in time"
                elif not problem and state.lpos_age > 1.5:
                    abort_reason = f"Position telemetry lost (age {state.lpos_age:.1f} s)"
                elif problem:
                    if use_subplots:
                        cmd_n, cmd_e = last_verified_sub
                    cmd_yaw = math.atan2(target_e - cmd_e, target_n - cmd_n)
                else:
                    cross_m = cross_track_m(north_m, east_m, leg_from[0], leg_from[1], target_n, target_e)
                    if cross_m > corridor_m:
                        abort_reason = (
                            f"Left the corridor ({cross_m * ht.M_TO_FT:.1f} ft > {plan['corridor_ft']:.0f} ft)"
                        )
                    else:
                        along = 0.0
                        if leg_len > 0.05:
                            dx, dy = target_n - leg_from[0], target_e - leg_from[1]
                            along = ((north_m - leg_from[0]) * dx + (east_m - leg_from[1]) * dy) / leg_len
                            along = max(0.0, min(leg_len, along))
                        # Smoothly taper advance speed in the last 1.8m (6ft) of a leg to prevent braking overshoot
                        rem_leg_m = max(0.0, leg_len - along)
                        eff_speed_mps = speed_mps if rem_leg_m >= 1.8 else max(0.35, min(speed_mps, rem_leg_m * 0.65))
                        if use_subplots and leg_subplots:
                            sub_n, sub_e, sub_d = leg_subplots[sub_idx]
                            dist_sub = math.hypot(sub_n - north_m, sub_e - east_m)
                            accept_sub = max(WP_ACCEPT_M, min(1.0, step_m * 0.35))
                            if (dist_sub <= accept_sub or along >= sub_d - 0.15) and sub_idx + 1 < len(leg_subplots):
                                last_verified_sub = (sub_n, sub_e)
                                sub_idx += 1
                                sub_n, sub_e, sub_d = leg_subplots[sub_idx]
                            progress_m = min(sub_d, max(progress_m, along) + eff_speed_mps * 0.1)
                        else:
                            progress_m = min(leg_len, max(progress_m, along) + eff_speed_mps * 0.1)
                        if progress_m - along > CARROT_M:
                            progress_m = along + CARROT_M
                        if leg_len < 0.05:
                            cmd_n, cmd_e = target_n, target_e
                        else:
                            scale = progress_m / leg_len
                            cmd_n = leg_from[0] + (target_n - leg_from[0]) * scale
                            cmd_e = leg_from[1] + (target_e - leg_from[1]) * scale
                            if not use_subplots:
                                last_verified_sub = (cmd_n, cmd_e)
                        # Integral cross-track / wind trim (same ki=0.08 as hover_test.py)
                        err_n = north_m - cmd_n
                        err_e = east_m - cmd_e
                        trim_n_m = max(-max_trim_m, min(max_trim_m, trim_n_m + ki_trim * err_n * 0.1))
                        trim_e_m = max(-max_trim_m, min(max_trim_m, trim_e_m + ki_trim * err_e * 0.1))
                        dist_end = math.hypot(target_n - north_m, target_e - east_m)
                        if dist_end > 0.6:
                            cmd_yaw = math.atan2(target_e - east_m, target_n - north_m)
                        is_final_leg = (leg_index + 1 >= len(route))
                        accept_r = HOME_ACCEPT_M if (is_final_leg and plan["return_home"]) else WP_ACCEPT_M
                        if dist_end <= accept_r and progress_m >= leg_len - 0.15:
                            last_verified_sub = (target_n, target_e)
                            if is_final_leg:
                                if plan["return_home"]:
                                    phase = "HOME_HOLD"
                                    home_settle_until = now + HOME_SETTLE_S
                                    cmd_n, cmd_e = 0.0, 0.0
                                    cmd_yaw = start_yaw
                                    log(
                                        f"\n>>> Back over exact launch spot (offset {dist_end * ht.M_TO_FT:.2f} ft). "
                                        f"Centering for {HOME_SETTLE_S:.1f} s before landing... <<<"
                                    )
                                else:
                                    phase = "DONE"
                                    mission_done = True
                                    note = "path complete"
                                    log("\n>>> Path complete. Landing. <<<")
                            else:
                                phase = "PAUSE"
                                pause_until = now + max(1.2, plan["pause_s"])
                                cmd_n, cmd_e = target_n, target_e
                                log(
                                    f"Point {leg_index + 1} reached (offset {dist_end * ht.M_TO_FT:.2f} ft). "
                                    f"Pausing {max(1.2, plan['pause_s']):.1f} s."
                                )
            elif phase == "PAUSE":
                cmd_alt_m = target_alt_m
                target_n, target_e = route[leg_index]
                cmd_n, cmd_e = target_n, target_e
                cross_m = math.hypot(north_m - target_n, east_m - target_e)
                trim_n_m = max(-max_trim_m, min(max_trim_m, trim_n_m + ki_trim * (north_m - target_n) * 0.1))
                trim_e_m = max(-max_trim_m, min(max_trim_m, trim_e_m + ki_trim * (east_m - target_e) * 0.1))
                # Allow +1.5m (~5 ft) braking buffer at corner pauses so stopping inertia does not abort
                pause_limit_m = max(corridor_m + 1.5, 2.5)
                if not problem and cross_m > pause_limit_m:
                    abort_reason = f"Drifted off the waypoint ({cross_m * ht.M_TO_FT:.1f} ft)"
                elif not problem and now >= pause_until and (cross_m <= WP_ACCEPT_M or now >= pause_until + 1.5):
                    leg_from = (target_n, target_e)
                    leg_index += 1
                    progress_m = 0.0
                    phase = "LEG"
                    leg_subplots = (
                        build_leg_subplots_ne(leg_from, route[leg_index], step_m)
                        if use_subplots
                        else [(route[leg_index][0], route[leg_index][1], math.hypot(route[leg_index][0] - leg_from[0], route[leg_index][1] - leg_from[1]))]
                    )
                    sub_idx = 0
                    last_verified_sub = leg_from
                    next_len = math.hypot(route[leg_index][0] - leg_from[0], route[leg_index][1] - leg_from[1])
                    leg_deadline = now + max(25.0, 4.0 * next_len / max(speed_mps, 0.3))
                    log(f"Flying to point {leg_index + 1} of {len(route)}.")
            elif phase == "HOME_HOLD":
                cmd_alt_m = target_alt_m
                cmd_n, cmd_e = 0.0, 0.0
                cmd_yaw = start_yaw
                cross_m = math.hypot(north_m, east_m)
                horiz_spd = math.hypot(state.vx, state.vy)
                trim_n_m = max(-max_trim_m, min(max_trim_m, trim_n_m + ki_trim * north_m * 0.1))
                trim_e_m = max(-max_trim_m, min(max_trim_m, trim_e_m + ki_trim * east_m * 0.1))
                home_limit_m = max(corridor_m + 1.5, 2.5)
                if not problem and cross_m > home_limit_m:
                    abort_reason = f"Drifted off launch spot ({cross_m * ht.M_TO_FT:.1f} ft)"
                elif (now >= home_settle_until and cross_m <= HOME_ACCEPT_M and horiz_spd <= 0.25) or (now >= home_settle_until + 2.5):
                    phase = "DONE"
                    mission_done = True
                    note = f"landed at start (err={cross_m * ht.M_TO_FT:.2f}ft)"
                    log(f"\n>>> Locked over launch spot (drift {cross_m * ht.M_TO_FT:.2f} ft). Landing now. <<<")
            elif phase == "SIGNAL_SPIN":
                spin_yaw = ht.wrap_pi(spin_yaw + math.radians(SPIN_DEG_PER_S) * 0.1)
                spin_left -= math.radians(SPIN_DEG_PER_S) * 0.1
                cmd_yaw = spin_yaw
                note = "signal spin"
                if spin_left <= 0.0:
                    phase = "SIGNAL_HOLD"
                    log(">>> Two turns done. Holding. Move the transmitter to take over. <<<")
            elif phase == "SIGNAL_HOLD":
                cmd_n, cmd_e, cmd_yaw = hold_n, hold_e, spin_yaw
                note = "signal hold"
            elif phase == "DONE":
                mission_done = True

            if abort_reason:
                break

            if now - last_send >= 0.2:
                if phase == "SIGNAL_SPIN":
                    send_attitude(conn, spin_yaw, math.radians(SPIN_DEG_PER_S))
                elif phase == "SIGNAL_HOLD" and problem:
                    send_attitude(conn, spin_yaw, 0.0)
                elif phase not in ("TAKEOFF", "DONE"):
                    conn.send_position_target_ned(
                        start_x + cmd_n - trim_n_m,
                        start_y + cmd_e - trim_e_m,
                        start_z - cmd_alt_m,
                        cmd_yaw,
                        use_velocity=False,
                    )
                last_send = now

            sub_tag = f"{sub_idx + 1}/{len(leg_subplots)}" if use_subplots else "-"
            writer.writerow(
                [
                    f"{now:.3f}",
                    phase,
                    leg_index + 1,
                    sub_tag,
                    f"{alt_ft:.2f}",
                    f"{cross_m * ht.M_TO_FT:.2f}",
                    f"{north_m * ht.M_TO_FT:.2f}",
                    f"{east_m * ht.M_TO_FT:.2f}",
                    f"{state.roll:.1f}",
                    f"{state.pitch:.1f}",
                    f"{(math.degrees(state.yaw) % 360):.1f}",
                    f"{state.volt:.2f}",
                    f"{state.curr:.2f}",
                    state.sats,
                    f"{state.hdop:.2f}",
                    note,
                ]
            )
            csv_file.flush()
            if now - last_print >= 1.0:
                sub_str = f" sub={sub_tag}" if use_subplots and phase == "LEG" else ""
                log(
                    f"[{phase} {leg_index + 1}/{max(1, len(route))}{sub_str}] alt={alt_ft:4.1f}ft "
                    f"off-path={cross_m * ht.M_TO_FT:4.1f}ft "
                    f"N={north_m * ht.M_TO_FT:+5.0f} E={east_m * ht.M_TO_FT:+5.0f} "
                    f"batt={state.volt:4.2f}V sats={state.sats}"
                )
                last_print = now
            time.sleep(0.1)

        csv_file.close()
        with conn.state.lock:
            still_armed = conn.state.armed
            mode_now = conn.state.custom_mode
        if still_armed and mode_now != ht.MODE_STABILIZE:
            with MISSION.lock:
                MISSION.phase = "LANDING"
            ht.execute_landing(conn, abort_reason or "path complete")
        elif abort_reason:
            log(f"Ended: {abort_reason}")
        with conn.state.lock:
            if not conn.state.armed and conn.state.custom_mode != ht.MODE_STABILIZE:
                need_stabilize = True
            else:
                need_stabilize = False
        if need_stabilize:
            conn.set_mode(ht.MODE_STABILIZE, timeout=1.5)
        log(f"Log: {csv_path}")
        with MISSION.lock:
            MISSION.phase = "DONE" if abort_reason is None else "ENDED"
        return 0 if abort_reason is None else 2
    finally:
        try:
            conn.stop_streams()
        except Exception:
            pass
        conn.close()
        with MISSION.lock:
            MISSION.running = False


def fly_hover_rc_test(args) -> int:
    """Take off to alt_ft (default 8 ft), hover for duration_s (default 60 s) while allowing
    live RC stick steering, return to the exact takeoff spot when sticks are released for 5 s,
    and land at the takeoff spot with guaranteed motor disarm.
    """
    target_alt_ft = float(getattr(args, "alt_ft", 8.0))
    target_alt_m = target_alt_ft * ht.FT_TO_M
    hover_duration_s = float(getattr(args, "duration_s", 60.0))
    max_drift_ft = float(getattr(args, "max_drift_ft", 6.0))
    max_drift_m = max_drift_ft * ht.FT_TO_M
    ceiling_m = (target_alt_ft + CEILING_EXTRA_FT) * ht.FT_TO_M
    max_tilt = float(getattr(args, "max_tilt_deg", 25.0))
    low_volt = float(getattr(args, "low_volt_cutoff", ht.DEFAULT_LOW_VOLT_CUTOFF))
    min_takeoff_volt = float(getattr(args, "min_takeoff_volt", ht.DEFAULT_MIN_TAKEOFF_VOLT))
    gps_wait = float(getattr(args, "gps_wait", 45.0))
    close_qgc = bool(getattr(args, "close_qgc", True))
    start_delay_s = max(0, int(round(float(getattr(args, "start_delay_s", 0)))))

    with MISSION.lock:
        MISSION.running = True
        MISSION.abort_requested = False
        MISSION.land_requested = False
        MISSION.plan_name = f"hover_{target_alt_ft:.0f}ft_{hover_duration_s:.0f}s"
        MISSION.phase = "COUNTDOWN" if start_delay_s > 0 else "PREFLIGHT"
        MISSION.leg = 0
        MISSION.leg_total = 0
        MISSION.sub_index = 0
        MISSION.sub_total = int(round(hover_duration_s))
        MISSION.countdown_s = start_delay_s

    abort_reason = None

    def on_signal(signum, _frame):
        nonlocal abort_reason
        abort_reason = f"Received signal {signum}"

    try:
        signal.signal(signal.SIGINT, on_signal)
        signal.signal(signal.SIGTERM, on_signal)
    except ValueError:
        pass

    if start_delay_s > 0:
        log(f"Starting {start_delay_s} s countdown before {target_alt_ft:.0f}-ft / {hover_duration_s:.0f}-s Hover & RC Test...")
        for rem in range(start_delay_s, 0, -1):
            with MISSION.lock:
                MISSION.countdown_s = rem
                if MISSION.abort_requested or MISSION.land_requested:
                    abort_reason = "Countdown cancelled by user"
                    break
            if abort_reason:
                break
            if rem in (60, 45, 30, 20, 15, 10, 5, 4, 3, 2, 1):
                log(f"Hover takeoff countdown: {rem} s...")
            time.sleep(1.0)
        with MISSION.lock:
            MISSION.countdown_s = 0
        if abort_reason:
            log(abort_reason)
            with MISSION.lock:
                MISSION.running = False
                MISSION.phase = "ABORTED"
            return 1

    with HERE_LOCK:
        try:
            conn = open_mavlink_with_retry(close_qgc=close_qgc, timeout=5.0)
        except Exception as exc:
            log(f"Could not open /dev/ttyAMA0: {exc}")
            with MISSION.lock:
                MISSION.running = False
                MISSION.phase = "ERROR"
            return 1

    try:
        with MISSION.lock:
            MISSION.phase = "PREFLIGHT"
        log("Configuring telemetry on /dev/ttyAMA0 for Hover & RC Test...")
        conn.configure_streams()
        conn.send_command_long(ht.MAV_CMD_RUN_PREARM_CHECKS)
        time.sleep(2.5)

        wait_deadline = time.time() + gps_wait
        while time.time() < wait_deadline and abort_reason is None:
            with MISSION.lock:
                if MISSION.abort_requested or MISSION.land_requested:
                    abort_reason = "Cancelled during pre-flight"
                    break
                MISSION.last_snap = drone_snapshot(conn)
            with conn.state.lock:
                ready, issues = ht.evaluate_readiness(conn.state, min_volt=min_takeoff_volt)
                fix = conn.state.gps_fix
                sats = conn.state.gps_sats
                hdop = conn.state.gps_hdop
                ekf_flags = conn.state.ekf_flags
            if ready:
                break
            log(
                f"  [WAITING] fix={fix} sats={sats} hdop={hdop:.2f} ekf=0x{ekf_flags:02x} | "
                + "; ".join(issues)
            )
            conn.send_command_long(ht.MAV_CMD_RUN_PREARM_CHECKS)
            time.sleep(2.0)

        with conn.state.lock:
            ready, issues = ht.evaluate_readiness(conn.state, min_volt=min_takeoff_volt)
            home_lat = conn.state.lat_deg
            home_lon = conn.state.lon_deg
            baseline_rc = conn.state.rc_chans
        if abort_reason:
            log(abort_reason)
            conn.stop_streams()
            return 1
        if not ready:
            log("\nPre-flight checks did not pass:")
            for item in issues:
                log(f"  - {item}")
            log("Not arming.")
            conn.stop_streams()
            return 1

        log(
            f"RC Remote paired & live: CH1..4={baseline_rc[:4]} "
            f"(move sticks anytime during hover; release {RC_RESUME_IDLE_S:.0f}s to return to launch spot)"
        )

        try:
            signal.signal(signal.SIGHUP, signal.SIG_IGN)
        except ValueError:
            pass
        log_dir = os.path.expanduser("~/hover_test_logs")
        os.makedirs(log_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(log_dir, f"hover_{stamp}.csv")
        csv_file = open(csv_path, "w", newline="")
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "timestamp", "phase", "elapsed_s", "mode", "armed",
                "alt_ft", "target_alt_ft", "drift_ft", "north_err_ft", "east_err_ft",
                "cmd_trim_n_m", "cmd_trim_e_m", "vel_horiz_fps", "vz_fps",
                "roll_deg", "pitch_deg", "yaw_deg", "volt_v", "curr_a", "sats", "hdop",
            ]
        )

        log("Switching to GUIDED...")
        if not conn.set_mode(ht.MODE_GUIDED, timeout=3.0):
            log("GUIDED was refused. Not arming.")
            conn.stop_streams()
            csv_file.close()
            return 1

        log("Arming...")
        armed_ok, reasons = conn.arm(timeout=4.0)
        if not armed_ok:
            log("Arming refused:")
            for r in reasons:
                log(f"  - {r}")
            conn.stop_streams()
            csv_file.close()
            return 1
        log("ARMED. Waiting 2.2 s for the arming delay and home reset...")
        time.sleep(2.2)
        with conn.state.lock:
            start_x = conn.state.x
            start_y = conn.state.y
            start_z = conn.state.z
            start_yaw = conn.state.yaw_rad

        log(f"Takeoff to {target_alt_ft:.1f} ft for {hover_duration_s:.0f} s Hover & RC Test...")
        if not conn.command_takeoff(target_alt_m, timeout=4.5):
            abort_reason = "Takeoff command was rejected"

        phase = "TAKEOFF"
        takeoff_started = time.time()
        hover_started = None
        last_send = 0.0
        last_print = 0.0
        low_volt_since = None
        bad_since = None
        spin_yaw = start_yaw
        spin_left = 0.0
        hold_n = 0.0
        hold_e = 0.0
        trim_n_m = 0.0
        trim_e_m = 0.0
        ki_trim = 0.08
        max_trim_m = 0.40
        override_n = 0.0
        override_e = 0.0
        override_alt_m = target_alt_m
        override_yaw = start_yaw
        rc_idle_since = None
        thr_centered_once = False
        rc_moved_any = False

        while abort_reason is None:
            now = time.time()
            state = read_state(conn)
            elapsed_s = (now - hover_started) if hover_started else 0.0
            with MISSION.lock:
                MISSION.phase = phase
                MISSION.sub_index = min(int(round(elapsed_s)), int(round(hover_duration_s)))
                MISSION.sub_total = int(round(hover_duration_s))
                MISSION.last_snap = drone_snapshot(conn)
                if MISSION.land_requested or MISSION.abort_requested:
                    abort_reason = "User clicked Land Now / Stop in UI"
                    break

            if not state.armed:
                log("\nPixhawk disarmed.")
                break
            if state.mode not in (ht.MODE_GUIDED, ht.MODE_LAND) and phase not in ("SIGNAL_SPIN", "SIGNAL_HOLD"):
                log(f"\nMode changed to {ht.MODE_NAMES.get(state.mode, state.mode)}. Stopping guidance.")
                csv_file.close()
                return 0

            north_m = state.x - start_x
            east_m = state.y - start_y
            drift_m = math.hypot(north_m, east_m)
            drift_ft = drift_m * ht.M_TO_FT
            alt_m = altitude_m(state, start_z)
            alt_ft = alt_m * ht.M_TO_FT
            vel_horiz_ms = math.hypot(state.vx, state.vy)

            sw_flip, sw_reason, norm_fwd, norm_right, norm_yaw, norm_climb, thr_centered_once = parse_rc_nudge(
                baseline_rc, state.rc, thr_centered_once
            )
            if sw_flip and sw_reason:
                abort_reason = f"RC SWITCH LAND: {sw_reason}"
                break
            sticks_active = (norm_fwd != 0.0 or norm_right != 0.0 or norm_yaw != 0.0 or norm_climb != 0.0)
            if sticks_active and (phase != "TAKEOFF" or alt_m >= 0.8):
                if phase != "RC_OVERRIDE":
                    if hover_started is None:
                        hover_started = now
                    override_n = north_m
                    override_e = east_m
                    override_alt_m = max(alt_m, target_alt_m)
                    override_yaw = state.yaw
                    phase = "RC_OVERRIDE"
                    rc_moved_any = True
                    log(
                        f"\n>>> RC OVERRIDE: Pilot moving sticks — steering from RC. "
                        f"Release sticks for {RC_RESUME_IDLE_S:.0f} s to return to takeoff spot. <<<"
                    )
                rc_idle_since = None

            if abs(state.roll) > max_tilt or abs(state.pitch) > max_tilt:
                abort_reason = f"Tilt exceeded {max_tilt:.0f} deg (roll={state.roll:+.1f}, pitch={state.pitch:+.1f})"
                break
            if phase == "HOVER" and not rc_moved_any and drift_m > max_drift_m:
                abort_reason = f"Uncommanded hover drift {drift_ft:.1f} ft exceeded {max_drift_ft:.1f} ft limit"
                break
            if drift_m > max(max_drift_m, 18.0):
                abort_reason = f"Distance from launch ({drift_ft:.1f} ft) exceeded 59 ft safety radius"
                break
            if alt_m > ceiling_m and phase != "TAKEOFF":
                abort_reason = f"Altitude {alt_ft:.1f} ft exceeded ceiling {ceiling_m * ht.M_TO_FT:.0f} ft"
                break
            if 1.0 < state.volt < ht.DEFAULT_CRIT_VOLT_CUTOFF:
                abort_reason = f"Battery sag {state.volt:.2f} V"
                break
            if 1.0 < state.volt < low_volt:
                if low_volt_since is None:
                    low_volt_since = now
                elif now - low_volt_since >= 1.5:
                    abort_reason = f"Battery below {low_volt:.2f} V for 1.5 s ({state.volt:.2f} V)"
                    break
            else:
                low_volt_since = None

            problem = nav_problem(state)
            if phase in ("HOVER", "RETURN_HOME") or (phase == "TAKEOFF" and alt_m > 1.0):
                if problem:
                    if bad_since is None:
                        bad_since = now
                        log(f"Nav warning, holding for {NAV_BAD_HOLD_S:.0f} s: {problem}")
                    elif now - bad_since >= NAV_BAD_HOLD_S and phase != "SIGNAL_SPIN":
                        phase = "SIGNAL_SPIN"
                        spin_yaw = state.yaw
                        spin_left = SPIN_TURNS * 2.0 * math.pi
                        hold_n, hold_e = north_m, east_m
                        log(f"\n>>> GPS/COMPASS SIGNAL: {problem}")
                        log(">>> Yawing two turns, then holding for the transmitter. <<<")
                else:
                    bad_since = None
            elif phase == "TAKEOFF" and problem:
                abort_reason = problem

            if phase == "TAKEOFF":
                if state.lpos_age > 1.5 or state.att_age > 1.5:
                    abort_reason = f"Telemetry timeout during takeoff (lpos {state.lpos_age:.1f}s)"
                elif alt_m >= target_alt_m - 0.25 or (now - takeoff_started >= 8.0 and alt_m >= 0.6):
                    phase = "HOVER"
                    hover_started = now
                    log(
                        f"\n>>> Reached {alt_ft:.1f} ft! Starting {hover_duration_s:.0f}-second Hover & RC Test "
                        f"(move sticks anytime; release {RC_RESUME_IDLE_S:.0f}s to return to launch spot)... <<<"
                    )
                elif now - takeoff_started > 15.0:
                    abort_reason = f"Takeoff did not reach altitude ({alt_ft:.1f} ft)"
            elif phase == "RC_OVERRIDE":
                if sticks_active:
                    vx_fwd = norm_fwd * 1.8
                    vy_right = norm_right * 1.8
                    vn = vx_fwd * math.cos(state.yaw) - vy_right * math.sin(state.yaw)
                    ve = vx_fwd * math.sin(state.yaw) + vy_right * math.cos(state.yaw)
                    override_n = north_m + vn * 0.85
                    override_e = east_m + ve * 0.85
                    override_yaw = ht.wrap_pi(override_yaw + norm_yaw * math.radians(45.0) * 0.1)
                    override_alt_m = max(1.1, min(ceiling_m - 0.3, override_alt_m + norm_climb * 0.7 * 0.1))
                else:
                    if rc_idle_since is None:
                        rc_idle_since = now
                        override_n = north_m
                        override_e = east_m
                        override_yaw = state.yaw
                        log(f"RC sticks released — holding spot. Returning to launch point in {RC_RESUME_IDLE_S:.0f} s...")
                    elif now - rc_idle_since >= RC_RESUME_IDLE_S:
                        phase = "RETURN_HOME"
                        rc_idle_since = None
                        thr_centered_once = False
                        log(f"\n>>> RC untouched for {RC_RESUME_IDLE_S:.0f} s — flying back to exact launch spot... <<<")
                if now - last_send >= 0.2:
                    conn.send_position_target_ned(
                        start_x + override_n,
                        start_y + override_e,
                        start_z - override_alt_m,
                        override_yaw,
                        use_velocity=False,
                    )
                    last_send = now
            elif phase == "RETURN_HOME":
                if drift_m <= HOME_ACCEPT_M and vel_horiz_ms <= 0.25:
                    rc_moved_any = False
                    trim_n_m = 0.0
                    trim_e_m = 0.0
                    if elapsed_s >= hover_duration_s:
                        log(f"\n>>> Locked over launch spot (offset {drift_ft:.2f} ft) and {hover_duration_s:.0f}s complete. Landing! <<<")
                        break
                    phase = "HOVER"
                    log(
                        f"\n>>> Back over launch spot (offset {drift_ft:.2f} ft). "
                        f"Resuming hover ({max(0.0, hover_duration_s - elapsed_s):.0f} s remaining)... <<<"
                    )
                else:
                    step_d = min(drift_m, CARROT_M)
                    scale = max(0.0, (drift_m - step_d) / max(0.01, drift_m))
                    cmd_n = north_m * scale
                    cmd_e = east_m * scale
                    if now - last_send >= 0.2:
                        conn.send_position_target_ned(
                            start_x + cmd_n,
                            start_y + cmd_e,
                            start_z - target_alt_m,
                            start_yaw,
                            use_velocity=False,
                        )
                        last_send = now
            elif phase == "HOVER":
                if elapsed_s >= hover_duration_s:
                    if drift_m > HOME_ACCEPT_M:
                        phase = "RETURN_HOME"
                        log(f"\n>>> {hover_duration_s:.0f}s elapsed — centering over launch spot before landing... <<<")
                    else:
                        log(f"\n>>> Completed {hover_duration_s:.0f}-second Hover & RC Test (offset {drift_ft:.2f} ft). Landing! <<<")
                        break
                else:
                    trim_n_m = max(-max_trim_m, min(max_trim_m, trim_n_m + ki_trim * north_m * 0.1))
                    trim_e_m = max(-max_trim_m, min(max_trim_m, trim_e_m + ki_trim * east_m * 0.1))
                    if now - last_send >= 0.2:
                        conn.send_position_target_ned(
                            start_x - trim_n_m,
                            start_y - trim_e_m,
                            start_z - target_alt_m,
                            start_yaw,
                            use_velocity=False,
                        )
                        last_send = now
            elif phase == "SIGNAL_SPIN":
                spin_yaw = ht.wrap_pi(spin_yaw + math.radians(SPIN_DEG_PER_S) * 0.1)
                spin_left -= math.radians(SPIN_DEG_PER_S) * 0.1
                if spin_left <= 0.0:
                    phase = "SIGNAL_HOLD"
                    log(">>> Two turns done. Holding. Move the transmitter to take over. <<<")
                if now - last_send >= 0.2:
                    send_attitude(conn, spin_yaw, math.radians(SPIN_DEG_PER_S))
                    last_send = now
            elif phase == "SIGNAL_HOLD":
                if now - last_send >= 0.2:
                    if problem:
                        send_attitude(conn, spin_yaw, 0.0)
                    else:
                        conn.send_position_target_ned(
                            start_x + hold_n,
                            start_y + hold_e,
                            start_z - target_alt_m,
                            spin_yaw,
                            use_velocity=False,
                        )
                    last_send = now

            if abort_reason:
                break

            writer.writerow(
                [
                    f"{now:.3f}",
                    phase,
                    f"{elapsed_s:.2f}",
                    ht.MODE_NAMES.get(state.mode, str(state.mode)),
                    int(state.armed),
                    f"{alt_ft:.2f}",
                    f"{target_alt_ft:.2f}",
                    f"{drift_ft:.2f}",
                    f"{north_m * ht.M_TO_FT:+.2f}",
                    f"{east_m * ht.M_TO_FT:+.2f}",
                    f"{trim_n_m:+.3f}",
                    f"{trim_e_m:+.3f}",
                    f"{vel_horiz_ms * ht.M_TO_FT:.2f}",
                    f"{-state.vz * ht.M_TO_FT:+.2f}",
                    f"{state.roll:+.1f}",
                    f"{state.pitch:+.1f}",
                    f"{(math.degrees(state.yaw) % 360):.1f}",
                    f"{state.volt:.2f}",
                    f"{state.curr:.2f}",
                    state.sats,
                    f"{state.hdop:.2f}",
                ]
            )
            csv_file.flush()
            if now - last_print >= 1.0:
                log(
                    f"[{phase} {elapsed_s:4.1f}s/{hover_duration_s:.0f}s] alt={alt_ft:4.1f}ft "
                    f"dist={drift_ft:4.2f}ft (N={north_m * ht.M_TO_FT:+4.1f} E={east_m * ht.M_TO_FT:+4.1f}) "
                    f"batt={state.volt:4.2f}V sats={state.sats}"
                )
                last_print = now
            time.sleep(0.1)

        csv_file.close()
        with conn.state.lock:
            still_armed = conn.state.armed
            mode_now = conn.state.custom_mode
        if still_armed and mode_now != ht.MODE_STABILIZE:
            with MISSION.lock:
                MISSION.phase = "LANDING"
            ht.execute_landing(conn, abort_reason or f"{hover_duration_s:.0f}-s Hover & RC Test complete")
        elif abort_reason:
            log(f"Ended: {abort_reason}")
        with conn.state.lock:
            need_stabilize = not conn.state.armed and conn.state.custom_mode != ht.MODE_STABILIZE
        if need_stabilize:
            conn.set_mode(ht.MODE_STABILIZE, timeout=1.5)
        log(f"Hover log: {csv_path}")
        with MISSION.lock:
            MISSION.phase = "DONE" if abort_reason is None else "ENDED"
        return 0 if abort_reason is None else 2
    finally:
        try:
            conn.stop_streams()
        except Exception:
            pass
        conn.close()
        with MISSION.lock:
            MISSION.running = False


PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Flight1 path</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<style>
  html, body { height: 100%; margin: 0; font: 15px/1.4 system-ui, sans-serif; color: #172018; }
  body { display: grid; grid-template-columns: 380px 1fr; }
  aside { padding: 14px 16px; overflow: auto; background: #f4f6f2; border-right: 1px solid #d5ddd4; }
  h1 { font-size: 18px; margin: 0 0 8px; }
  h2 { font-size: 13px; margin: 14px 0 6px; letter-spacing: .04em; text-transform: uppercase; color: #3d4a40; }
  p { margin: 0 0 10px; }
  label { display: block; margin: 8px 0 2px; font-size: 13px; }
  input[type=text], input[type=number] { width: 100%; box-sizing: border-box; padding: 6px 8px; }
  .row { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
  button { margin-top: 8px; margin-right: 6px; padding: 7px 10px; }
  #map { height: 100%; min-height: 420px; background: #d7e2dc; }
  .warn { background: #fff6d8; padding: 8px; border-radius: 6px; }
  #drone { background: #e7f0ea; padding: 8px 10px; border-radius: 6px; white-space: pre-line; }
  .drone-actions { display: flex; align-items: center; gap: 10px; margin: 6px 0 10px; }
  .drone-actions button { margin: 0; }
  #drone-age { font-size: 13px; color: #3d4a40; }
  #msg { min-height: 1.2em; font-weight: 600; }
  #list { display: flex; flex-direction: column; gap: 6px; }
  .pt { background: #fff; border: 1px solid #d5ddd4; border-radius: 8px; padding: 8px 10px; }
  .pt.start { border: 2px solid #14532d; background: #e7f5ec; }
  .pt-label { font-size: 12px; font-weight: 700; letter-spacing: .04em; text-transform: uppercase; color: #14532d; }
  .pt-coord { font-family: ui-monospace, monospace; font-size: 14px; margin-top: 2px; }
  .empty { color: #526056; margin: 0; }
  .leaflet-div-icon.wp-icon { background: transparent; border: 0; }
  .wp-badge { display: flex; align-items: center; justify-content: center; height: 22px; border-radius: 11px; background: #14532d; color: #fff; font: 700 12px/1 system-ui, sans-serif; box-shadow: 0 1px 2px rgba(0,0,0,.35); }
  .wp-badge.start { width: 52px; cursor: pointer; }
  .wp-badge.num { width: 22px; }
  .sub-box { background: #eef4ef; border: 1px solid #cbd8cd; border-radius: 8px; padding: 8px 10px; margin-top: 8px; }
  .sub-meta { font-size: 12px; color: #2b4c35; margin-top: 4px; font-weight: 600; }
  .exec-box { background: #e8f4ec; border: 2px solid #14532d; border-radius: 8px; padding: 10px 12px; margin: 12px 0; }
  .exec-box h2 { margin-top: 0; color: #14532d; }
  .btn-fly { background: #14532d; color: #fff; font-weight: 700; border: 0; border-radius: 6px; padding: 9px 12px; cursor: pointer; }
  .btn-fly:disabled { background: #8da393; cursor: not-allowed; }
  .btn-hover { background: #0f766e; color: #fff; font-weight: 700; border: 0; border-radius: 6px; padding: 9px 12px; cursor: pointer; }
  .btn-hover:disabled { background: #8da393; cursor: not-allowed; }
  .hover-box { background: #f0fdfa; border: 1px solid #99f6e4; border-radius: 6px; padding: 8px 10px; margin: 8px 0; }
  .btn-danger { background: #b91c1c; color: #fff; font-weight: 700; border: 0; border-radius: 6px; padding: 8px 10px; cursor: pointer; }
  .btn-secondary { background: #fff; color: #172018; border: 1px solid #95a599; border-radius: 6px; padding: 7px 10px; cursor: pointer; }
  #missionStatus { font-family: ui-monospace, monospace; font-size: 12px; background: #fff; border: 1px solid #cbd8cd; border-radius: 6px; padding: 7px 8px; margin-top: 8px; white-space: pre-line; max-height: 150px; overflow: auto; }
  select { padding: 6px 8px; width: 100%; box-sizing: border-box; }
</style>
</head>
<body>
<aside>
  <h1>Flight1 path</h1>
  <p>Start is where the drone is. Click the map for point 2, then point 3. Click the Start button on the map to close the loop back to the drone.</p>
  <div id="drone">Reading the drone…</div>
  <div class="drone-actions">
    <button id="here" type="button">Refresh</button>
    <span id="drone-age">Updates every 5 s</span>
  </div>
  <div id="msg"></div>
  <h2>Path</h2>
  <div id="list"></div>
  <label>Plan name</label>
  <input id="name" type="text" value="yard">
  <div class="row">
    <div><label>Height (ft)</label><input id="alt" type="number" min="3" max="30" step="1" value="8"></div>
    <div><label>Speed (ft/s)</label><input id="speed" type="number" min="1" max="12" step="0.5" value="4"></div>
  </div>
  <div class="row">
    <div><label>Corridor each side (ft)</label><input id="corridor" type="number" min="4" max="40" step="1" value="8"></div>
    <div><label>Pause at point (s)</label><input id="pause" type="number" min="0" max="10" step="1" value="1"></div>
  </div>
  <label><input id="returnHome" type="checkbox"> Closed loop, back to Start, then land</label>
  <p>The orange band on the map is the corridor. Leaving it lands the drone.</p>

  <div class="sub-box">
    <label><input id="useSubplots" type="checkbox" checked> Generate hidden sub-plot checkpoints between master points</label>
    <div class="row">
      <div><label>Sub-plot spacing (ft)</label><input id="subplotStep" type="number" min="3" max="50" step="1" value="10"></div>
      <div><label>&nbsp;</label><label><input id="showSubplots" type="checkbox"> Peek dots on map</label></div>
    </div>
    <div id="subplotInfo" class="sub-meta">Hidden sub-plots: 0 checkpoints (every 10 ft)</div>
  </div>

  <div>
    <button id="undo" type="button">Undo</button>
    <button id="clear" type="button">Clear</button>
    <button id="save" type="button" style="font-weight:700;">Save Plan on Pi</button>
  </div>

  <div class="exec-box" id="execBox">
    <h2>Take Off &amp; Execute Plan (RC Fallback)</h2>
    <div id="execPlanLabel" style="font-size:13px;font-weight:600;margin-bottom:6px;">Save or load a plan to arm &amp; fly</div>
    <label>Takeoff timing</label>
    <select id="startDelay">
      <option value="0">Immediate takeoff (0 s delay)</option>
      <option value="10">10 s countdown (step back with RC)</option>
      <option value="20">20 s countdown (walk outside)</option>
      <option value="30">30 s countdown</option>
      <option value="60">60 s countdown</option>
    </select>
    <label style="margin-top:8px;">
      <input id="confirmRc" type="checkbox">
      RC controller is ON in my hand (move sticks anytime to steer; release 5 s to auto-resume path)
    </label>
    <div>
      <button id="btnPreflight" class="btn-secondary" type="button">Check Readiness</button>
      <button id="btnFly" class="btn-fly" type="button">Arm, Take Off &amp; Fly Plan</button>
    </div>
    <div class="hover-box">
      <div style="font-size:12px;font-weight:700;color:#0f766e;text-transform:uppercase;letter-spacing:.03em;">Initial Hover &amp; RC Stick Test (No Plan Needed)</div>
      <div class="row" style="margin-top:4px;">
        <div><label>Hover Height (ft)</label><input id="hoverAlt" type="number" min="3" max="30" step="1" value="8"></div>
        <div><label>Hover Duration (s)</label><input id="hoverDur" type="number" min="15" max="180" step="5" value="60"></div>
      </div>
      <button id="btnHoverTest" class="btn-hover" type="button" style="width:100%;margin-top:8px;">Start 8-ft / 60-s Hover &amp; RC Test (Auto-Return &amp; Land)</button>
    </div>
    <div>
      <button id="btnLand" class="btn-danger" type="button">Land Now</button>
      <button id="btnAbort" class="btn-secondary" type="button">Cancel Countdown / Stop</button>
    </div>
    <div id="missionStatus">Status: IDLE (RC fallback active during flight)</div>
  </div>

  <h2>Type a point</h2>
  <div class="row">
    <div><label>Latitude</label><input id="lat" type="number" step="0.0000001"></div>
    <div><label>Longitude</label><input id="lon" type="number" step="0.0000001"></div>
  </div>
  <button id="add" type="button">Add typed point</button>

  <div class="warn">
    <p>You can also run the saved plan from the Pi terminal after leaving home Wi-Fi:</p>
    <p id="cmd"><code>python3 ~/Desktop/scripts/flight_path_planner.py fly ~/flight_plans/yard.json --confirm</code></p>
    <p>Map pictures need internet. The saved file and the fly command do not.</p>
  </div>
  <label>Saved plans</label>
  <select id="saved" size="5" style="width:100%"></select>
  <button id="load" type="button">Load selected</button>
</aside>
<div id="map"></div>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
const points = [];
let map, layer, droneFix = null, centeredOnce = false, droneBusy = false, lastDroneAt = 0, savedPlanName = '';
function msg(text) { document.getElementById('msg').textContent = text; }
function planBody() {
  return {
    name: document.getElementById('name').value,
    alt_ft: Number(document.getElementById('alt').value),
    speed_fps: Number(document.getElementById('speed').value),
    corridor_ft: Number(document.getElementById('corridor').value),
    pause_s: Number(document.getElementById('pause').value),
    return_home: document.getElementById('returnHome').checked,
    use_subplots: document.getElementById('useSubplots').checked,
    subplot_step_ft: Number(document.getElementById('subplotStep').value),
    start_point: droneFix ? {lat: droneFix.lat, lon: droneFix.lon} : null,
    waypoints: points.map(p => ({lat: p.lat, lon: p.lng}))
  };
}
function computeSubplotsJs(latlngs, stepFt) {
  if (latlngs.length < 2 || !(stepFt > 0)) return [];
  const stepM = stepFt * 0.3048;
  const out = [];
  for (let i = 0; i < latlngs.length - 1; i++) {
    const lat1 = latlngs[i][0], lon1 = latlngs[i][1];
    const lat2 = latlngs[i + 1][0], lon2 = latlngs[i + 1][1];
    const dxy = toLocal(lat2, lon2, lat1, lon1);
    const legM = Math.hypot(dxy[0], dxy[1]);
    const steps = Math.max(1, Math.ceil(legM / stepM));
    for (let k = 1; k <= steps; k++) {
      const frac = Math.min(1, (k * stepM) / Math.max(0.01, legM));
      out.push({
        leg: i + 1,
        sub_index: k,
        sub_count: steps,
        lat: lat1 + (lat2 - lat1) * frac,
        lon: lon1 + (lon2 - lon1) * frac,
        is_master: k === steps
      });
    }
  }
  return out;
}
function toLocal(lat, lon, lat0, lon0) {
  const mPerDegLat = 111132.92;
  const mPerDegLon = 111412.84 * Math.cos(lat0 * Math.PI / 180);
  return [(lon - lon0) * mPerDegLon, (lat - lat0) * mPerDegLat];
}
function fromLocal(x, y, lat0, lon0) {
  const mPerDegLat = 111132.92;
  const mPerDegLon = 111412.84 * Math.cos(lat0 * Math.PI / 180);
  return [lat0 + y / mPerDegLat, lon0 + x / mPerDegLon];
}
function corridorRing(latlngs, halfMeters) {
  if (latlngs.length < 2 || !(halfMeters > 0)) return null;
  const lat0 = latlngs[0][0];
  const lon0 = latlngs[0][1];
  let local = latlngs.map(ll => toLocal(ll[0], ll[1], lat0, lon0));
  const end = local[local.length - 1];
  const closed = local.length > 2 && Math.hypot(end[0] - local[0][0], end[1] - local[0][1]) < 1.5;
  if (closed) local = local.slice(0, -1);
  const n = local.length;
  const segCount = closed ? n : n - 1;
  const normals = [];
  for (let i = 0; i < segCount; i++) {
    const j = closed ? (i + 1) % n : i + 1;
    const dx = local[j][0] - local[i][0];
    const dy = local[j][1] - local[i][1];
    const len = Math.hypot(dx, dy);
    normals.push(len < 0.05 ? null : [-dy / len, dx / len]);
  }
  const left = [];
  const right = [];
  for (let i = 0; i < n; i++) {
    const prev = closed ? normals[(i - 1 + segCount) % segCount] : (i > 0 ? normals[i - 1] : null);
    const next = closed ? normals[i % segCount] : (i < normals.length ? normals[i] : null);
    let nx = 0;
    let ny = 0;
    let miter = 1;
    if (prev && next) {
      nx = prev[0] + next[0];
      ny = prev[1] + next[1];
      const len = Math.hypot(nx, ny);
      if (len < 1e-6) {
        nx = prev[0];
        ny = prev[1];
      } else {
        nx /= len;
        ny /= len;
        const dot = nx * next[0] + ny * next[1];
        miter = Math.max(-3, Math.min(3, 1 / (dot || 1)));
      }
    } else if (next) {
      nx = next[0];
      ny = next[1];
    } else if (prev) {
      nx = prev[0];
      ny = prev[1];
    } else {
      continue;
    }
    const dist = halfMeters * miter;
    left.push(fromLocal(local[i][0] + nx * dist, local[i][1] + ny * dist, lat0, lon0));
    right.push(fromLocal(local[i][0] - nx * dist, local[i][1] - ny * dist, lat0, lon0));
  }
  if (left.length < 2) return null;
  return left.concat(right.reverse());
}
function pointBadge(label, start) {
  const width = start ? 52 : 22;
  return L.divIcon({
    className: 'wp-icon',
    html: '<div class="wp-badge ' + (start ? 'start' : 'num') + '">' + label + '</div>',
    iconSize: [width, 22],
    iconAnchor: [width / 2, 11]
  });
}
function loopIsClosed() {
  return document.getElementById('returnHome').checked && !!droneFix && points.length >= 2;
}
function routeLatLngs() {
  const clicked = points.map(p => [p.lat, p.lng]);
  if (!droneFix) return clicked;
  const route = [[droneFix.lat, droneFix.lon]].concat(clicked);
  if (loopIsClosed()) route.push([droneFix.lat, droneFix.lon]);
  return route;
}
function closeLoop() {
  if (!droneFix) {
    msg('Start is not on the map yet.');
    return;
  }
  if (points.length < 2) {
    msg('Place two points, then click Start to close the loop.');
    return;
  }
  document.getElementById('returnHome').checked = true;
  redraw();
  msg('Closed loop. The drone flies the points and returns to Start.');
}
function addCoordRow(list, label, coordText, isStart) {
  const row = document.createElement('div');
  row.className = 'pt' + (isStart ? ' start' : '');
  const title = document.createElement('div');
  title.className = 'pt-label';
  title.textContent = label;
  const coord = document.createElement('div');
  coord.className = 'pt-coord';
  coord.textContent = coordText;
  row.appendChild(title);
  row.appendChild(coord);
  list.appendChild(row);
}
function redraw() {
  const list = document.getElementById('list');
  list.replaceChildren();
  if (droneFix) {
    addCoordRow(list, 'Start', droneFix.lat.toFixed(7) + ',  ' + droneFix.lon.toFixed(7), true);
  } else {
    addCoordRow(list, 'Start', 'Waiting for the drone GPS', true);
  }
  if (!points.length) {
    const empty = document.createElement('p');
    empty.className = 'empty';
    empty.textContent = 'Click the map for point 2.';
    list.appendChild(empty);
  }
  points.forEach((p, i) => {
    addCoordRow(list, 'Point ' + (i + 2), p.lat.toFixed(7) + ',  ' + p.lng.toFixed(7), false);
  });
  if (loopIsClosed()) {
    addCoordRow(list, 'Back to start', droneFix.lat.toFixed(7) + ',  ' + droneFix.lon.toFixed(7), true);
  }
  if (layer) layer.remove();
  if (!map || typeof L === 'undefined') return;
  const latlngs = routeLatLngs();
  layer = L.layerGroup().addTo(map);
  const halfMeters = Number(document.getElementById('corridor').value) * 0.3048;
  const ring = corridorRing(latlngs, halfMeters);
  if (ring) {
    L.polygon(ring, {
      stroke: false,
      fillColor: '#ea580c',
      fillOpacity: 0.35,
      interactive: false
    }).addTo(layer);
  }
  if (latlngs.length > 1) L.polyline(latlngs, {color: '#14532d', weight: 4}).addTo(layer);
  if (ring) {
    L.polygon(ring, {
      color: '#c2410c',
      weight: 3,
      fillOpacity: 0,
      interactive: false
    }).addTo(layer);
  }
  if (droneFix) {
    const startMarker = L.marker([droneFix.lat, droneFix.lon], {
      icon: pointBadge('Start', true),
      interactive: true,
      zIndexOffset: 800
    }).addTo(layer);
    startMarker.on('click', (event) => {
      L.DomEvent.stopPropagation(event);
      closeLoop();
    });
  }
  const useSub = document.getElementById('useSubplots').checked;
  const stepFt = Number(document.getElementById('subplotStep').value) || 10;
  const subs = useSub ? computeSubplotsJs(latlngs, stepFt) : [];
  const subInfo = document.getElementById('subplotInfo');
  if (subInfo) {
    subInfo.textContent = useSub
      ? 'Hidden sub-plots: ' + subs.length + ' checkpoints (every ' + stepFt + ' ft)'
      : 'Hidden sub-plots: OFF (direct leg line only)';
  }
  if (useSub && document.getElementById('showSubplots').checked) {
    subs.forEach(s => {
      if (!s.is_master) {
        L.circleMarker([s.lat, s.lon], {
          radius: 3,
          color: '#14532d',
          fillColor: '#22c55e',
          fillOpacity: 0.85,
          interactive: false
        }).addTo(layer);
      }
    });
  }
  points.forEach((p, i) => {
    L.marker([p.lat, p.lng], {icon: pointBadge(String(i + 2), false), interactive: false}).addTo(layer);
  });
  const name = (document.getElementById('name').value || 'yard').replace(/[^A-Za-z0-9 _-]/g, '').trim() || 'yard';
  document.getElementById('cmd').textContent =
    'python3 ~/Desktop/scripts/flight_path_planner.py fly ~/flight_plans/' + name + '.json --confirm';
  const execLbl = document.getElementById('execPlanLabel');
  if (execLbl) {
    if (savedPlanName || points.length > 0) {
      const subTxt = useSub ? (subs.length + ' hidden sub-plots @ ' + stepFt + ' ft') : 'sub-plots off';
      execLbl.textContent = 'Ready plan: ' + name + '.json (' + points.length + ' master pts, ' + subTxt + ')';
    } else {
      execLbl.textContent = 'Save or load a plan to arm & fly';
    }
  }
}
function addPoint(lat, lng) {
  points.push({lat: lat, lng: lng});
  document.getElementById('returnHome').checked = false;
  redraw();
}
function bootMap() {
  if (typeof L === 'undefined') {
    msg('Map tiles did not load. Type lat and lon, then Add typed point. Save still works.');
    return;
  }
  map = L.map('map').setView([20, 0], 2);
  L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 19,
    attribution: '&copy; OpenStreetMap'
  }).addTo(map);
  map.on('click', (event) => addPoint(event.latlng.lat, event.latlng.lng));
}
function formatDrone(body) {
  if (body.drone_name) {
    const h1 = document.querySelector('aside h1');
    if (h1) h1.textContent = body.drone_name + ' path';
  }
  const lines = [];
  if (body.located) lines.push(Number(body.lat).toFixed(6) + ', ' + Number(body.lon).toFixed(6));
  else lines.push('GPS ' + (body.fix_name || 'no fix'));
  lines.push('GPS ' + body.fix_name + '   ' + body.sats + ' sats   HDOP ' + Number(body.hdop).toFixed(2));
  lines.push('Battery ' + Number(body.volt_v).toFixed(2) + ' V   ' + Number(body.curr_a).toFixed(1) + ' A');
  lines.push('Baro ' + Number(body.alt_ft).toFixed(1) + ' ft');
  lines.push(body.mode + (body.armed ? '   armed' : '   disarmed'));
  lines.push((body.safety_solid ? 'Safety solid' : 'Safety blinking') + (body.prearm_ok ? '   PreArm ready' : '   PreArm not ready'));
  lines.push('Roll ' + Number(body.roll_deg).toFixed(1) + '   Pitch ' + Number(body.pitch_deg).toFixed(1) + '   Yaw ' + Number(body.yaw_deg).toFixed(0));
  if (body.net_link) lines.push('Net ' + body.net_link);
  return lines.join('\\n');
}
async function showDrone(quiet) {
  if (droneBusy) return;
  droneBusy = true;
  paintDroneAge();
  if (!quiet) document.getElementById('drone').textContent = 'Reading the drone…';
  try {
    const res = await fetch('/api/here');
    const body = await res.json();
    if (!res.ok) {
      document.getElementById('drone').textContent = body.error || 'No drone link';
      return;
    }
    document.getElementById('drone').textContent = formatDrone(body);
    lastDroneAt = Date.now();
    paintDroneAge();
    if (!body.located) {
      droneFix = null;
      redraw();
      if (!quiet) msg('No GPS fix yet. Start updates when the drone has a position.');
      return;
    }
    droneFix = {lat: Number(body.lat), lon: Number(body.lon)};
    redraw();
    if (map && !centeredOnce) {
      map.setView([droneFix.lat, droneFix.lon], 18);
      centeredOnce = true;
    }
    if (!quiet) msg('Start is the drone. Click the map for point 2.');
  } catch (err) {
    if (!quiet) document.getElementById('drone').textContent = 'Could not read the drone';
  } finally {
    droneBusy = false;
  }
}
document.getElementById('add').onclick = () => {
  const lat = Number(document.getElementById('lat').value);
  const lon = Number(document.getElementById('lon').value);
  if (!Number.isFinite(lat) || !Number.isFinite(lon)) { msg('Enter a latitude and longitude.'); return; }
  addPoint(lat, lon);
  if (map) map.setView([lat, lon], Math.max(map.getZoom(), 16));
};
document.getElementById('undo').onclick = () => { points.pop(); redraw(); };
document.getElementById('clear').onclick = () => {
  points.splice(0, points.length);
  document.getElementById('returnHome').checked = false;
  redraw();
};
document.getElementById('returnHome').onchange = () => {
  if (document.getElementById('returnHome').checked && points.length < 2) {
    document.getElementById('returnHome').checked = false;
    msg('Place two points, then click Start on the map to close the loop.');
    return;
  }
  redraw();
  if (loopIsClosed()) msg('Closed loop. The drone flies the points and returns to Start.');
};
document.getElementById('name').oninput = redraw;
document.getElementById('corridor').oninput = redraw;
document.getElementById('useSubplots').onchange = redraw;
document.getElementById('subplotStep').oninput = redraw;
document.getElementById('showSubplots').onchange = redraw;
document.getElementById('here').onclick = () => { showDrone(false); };
document.getElementById('save').onclick = async () => {
  const res = await fetch('/api/plans', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(planBody())});
  const body = await res.json();
  if (!res.ok) { msg(body.error || 'Save failed'); return; }
  savedPlanName = body.name;
  msg('Saved ' + body.file + ' (' + (body.subplot_count || 0) + ' hidden sub-plots). Ready to take off below.');
  loadList();
  redraw();
};
async function loadList() {
  const res = await fetch('/api/plans');
  const body = await res.json();
  const sel = document.getElementById('saved');
  sel.innerHTML = '';
  body.plans.forEach(plan => {
    const opt = document.createElement('option');
    opt.value = plan.name;
    const subTag = plan.use_subplots ? (', ' + (plan.subplot_count || 0) + ' sub-plots') : '';
    opt.textContent = plan.name + ' (' + plan.waypoints.length + ' pts' + subTag + ')';
    sel.appendChild(opt);
  });
}
document.getElementById('load').onclick = async () => {
  const name = document.getElementById('saved').value;
  if (!name) return;
  const res = await fetch('/api/plans/' + encodeURIComponent(name));
  const plan = await res.json();
  if (!res.ok) { msg(plan.error || 'Load failed'); return; }
  points.splice(0, points.length);
  savedPlanName = plan.name;
  document.getElementById('name').value = plan.name;
  document.getElementById('alt').value = plan.alt_ft;
  document.getElementById('speed').value = plan.speed_fps;
  document.getElementById('corridor').value = plan.corridor_ft;
  document.getElementById('pause').value = plan.pause_s;
  document.getElementById('returnHome').checked = !!plan.return_home;
  document.getElementById('useSubplots').checked = plan.use_subplots !== false;
  document.getElementById('subplotStep').value = plan.subplot_step_ft || 10;
  plan.waypoints.forEach(p => points.push({lat: p.lat, lng: p.lon}));
  redraw();
  if (map && routeLatLngs().length) map.fitBounds(routeLatLngs(), {padding: [24, 24]});
  msg('Loaded ' + plan.name + '. Ready to take off below.');
};
document.getElementById('btnPreflight').onclick = async () => {
  const box = document.getElementById('missionStatus');
  box.textContent = 'Running pre-flight readiness check…';
  try {
    const res = await fetch('/api/preflight');
    const body = await res.json();
    if (!res.ok) {
      box.textContent = 'Pre-flight error: ' + (body.error || 'failed');
      return;
    }
    if (body.ready) {
      box.textContent = 'READY FOR TAKEOFF: All pre-flight checks passed (GPS 3D, EKF home, battery, safety switch, RC).';
    } else {
      box.textContent = 'NOT READY YET:\\n- ' + (body.issues || []).join('\\n- ');
    }
  } catch (err) {
    box.textContent = 'Could not reach Pi server for pre-flight check.';
  }
};
document.getElementById('btnFly').onclick = async () => {
  if (!document.getElementById('confirmRc').checked) {
    msg('Check the RC controller confirmation box before starting takeoff.');
    return;
  }
  if (!points.length && !savedPlanName) {
    msg('Add waypoints or load a saved plan first.');
    return;
  }
  if (points.length > 0) {
    const saveRes = await fetch('/api/plans', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(planBody())});
    const saveBody = await saveRes.json();
    if (!saveRes.ok) {
      msg(saveBody.error || 'Could not save plan before takeoff');
      return;
    }
    savedPlanName = saveBody.name;
    loadList();
  }
  const planName = savedPlanName || document.getElementById('name').value;
  const delaySec = Number(document.getElementById('startDelay').value) || 0;
  const res = await fetch('/api/fly', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({name: planName, confirm: true, start_delay_s: delaySec})
  });
  const body = await res.json();
  if (!res.ok) {
    msg(body.error || 'Could not start flight');
    return;
  }
  msg('Flight execution started for ' + planName + ' (RC fallback active).');
  pollMission();
};
function updateHoverBtnLabel() {
  const hAlt = Number(document.getElementById('hoverAlt').value) || 8;
  const hDur = Number(document.getElementById('hoverDur').value) || 60;
  const btn = document.getElementById('btnHoverTest');
  if (btn) btn.textContent = 'Start ' + hAlt + '-ft / ' + hDur + '-s Hover & RC Test (Auto-Return & Land)';
}
document.getElementById('hoverAlt').oninput = updateHoverBtnLabel;
document.getElementById('hoverDur').oninput = updateHoverBtnLabel;
document.getElementById('btnHoverTest').onclick = async () => {
  if (!document.getElementById('confirmRc').checked) {
    msg('Check the RC controller confirmation box before starting the Hover & RC Test.');
    return;
  }
  const hAlt = Number(document.getElementById('hoverAlt').value) || 8;
  const hDur = Number(document.getElementById('hoverDur').value) || 60;
  const delaySec = Number(document.getElementById('startDelay').value) || 0;
  const res = await fetch('/api/hover', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({alt_ft: hAlt, duration_s: hDur, confirm: true, start_delay_s: delaySec})
  });
  const body = await res.json();
  if (!res.ok) {
    msg(body.error || 'Could not start Hover & RC Test');
    return;
  }
  msg('Started ' + hAlt + '-ft / ' + hDur + '-s Hover & RC Test (move sticks anytime; release 5s to return & land).');
  pollMission();
};
document.getElementById('btnLand').onclick = async () => {
  msg('Commanding LAND mode now…');
  const res = await fetch('/api/land', {method: 'POST'});
  const body = await res.json();
  msg(body.message || body.error || 'LAND sent');
  pollMission();
};
document.getElementById('btnAbort').onclick = async () => {
  const res = await fetch('/api/abort', {method: 'POST'});
  const body = await res.json();
  msg(body.message || 'Stop requested');
  pollMission();
};
async function pollMission() {
  try {
    const res = await fetch('/api/mission');
    if (!res.ok) return;
    const m = await res.json();
    const box = document.getElementById('missionStatus');
    const lines = [];
    let head = 'Status: ' + m.phase;
    if (m.plan_name) head += ' (' + m.plan_name + ')';
    if (m.phase === 'COUNTDOWN') head += ' — ' + m.countdown_s + ' s';
    if (m.leg_total > 0 && m.leg > 0) {
      head += ' | Leg ' + m.leg + '/' + m.leg_total;
      if (m.sub_total > 0 && m.sub_index > 0) head += ' • Sub-plot ' + m.sub_index + '/' + m.sub_total;
    } else if (m.sub_total > 0) {
      head += ' | Hover ' + m.sub_index + 's/' + m.sub_total + 's';
    }
    lines.push(head);
    if (m.logs && m.logs.length) {
      lines.push(...m.logs.slice(-6));
    }
    box.textContent = lines.join('\\n');
    document.getElementById('btnFly').disabled = !!m.running;
    const hBtn = document.getElementById('btnHoverTest');
    if (hBtn) hBtn.disabled = !!m.running;
  } catch (err) {}
}
bootMap();
loadList();
redraw();
function paintDroneAge() {
  const el = document.getElementById('drone-age');
  if (!el) return;
  if (droneBusy) {
    el.textContent = 'Reading…';
    return;
  }
  if (!lastDroneAt) {
    el.textContent = 'Updates every 5 s';
    return;
  }
  const sec = Math.round((Date.now() - lastDroneAt) / 1000);
  el.textContent = sec <= 1 ? 'Updated just now' : 'Updated ' + sec + ' s ago';
}
showDrone(false);
setInterval(() => showDrone(true), 5000);
setInterval(paintDroneAge, 1000);
setInterval(pollMission, 1500);
</script>
</body>
</html>
"""


class PlannerHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt: str, *args) -> None:
        return

    def _send(self, code: int, payload: bytes, content_type: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def _json(self, code: int, obj: dict) -> None:
        self._send(code, json.dumps(obj).encode("utf-8"), "application/json")

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/":
            self._send(200, PAGE.encode("utf-8"), "text/html; charset=utf-8")
            return
        if parsed.path == "/api/plans":
            self._json(200, {"plans": list_plans()})
            return
        if parsed.path.startswith("/api/plans/"):
            name = parsed.path[len("/api/plans/") :]
            try:
                plan = load_plan(plan_path(name))
            except (OSError, ValueError, json.JSONDecodeError) as exc:
                self._json(404, {"error": str(exc)})
                return
            self._json(200, plan)
            return
        if parsed.path == "/api/here":
            self._json(*read_here())
            return
        if parsed.path == "/api/mission":
            self._json(200, MISSION.snapshot())
            return
        if parsed.path == "/api/preflight":
            self._json(*check_preflight())
            return
        self._json(404, {"error": "not found"})

    def do_POST(self) -> None:
        path_url = urlparse(self.path).path
        length = int(self.headers.get("Content-Length") or 0)
        raw_body = self.rfile.read(length).decode("utf-8") if 0 < length <= 65536 else "{}"
        if path_url == "/api/plans":
            if length <= 0 or length > 65536:
                self._json(400, {"error": "Plan body is empty or too large"})
                return
            try:
                plan = parse_plan(json.loads(raw_body))
                path = save_plan(plan)
            except (ValueError, json.JSONDecodeError, KeyError, TypeError) as exc:
                self._json(400, {"error": str(exc)})
                return
            self._json(
                200,
                {
                    "ok": True,
                    "file": path,
                    "name": plan["name"],
                    "subplot_count": len(plan.get("subplots") or []),
                },
            )
            return
        if path_url == "/api/fly":
            try:
                body = json.loads(raw_body) if raw_body else {}
            except json.JSONDecodeError:
                self._json(400, {"error": "Invalid JSON"})
                return
            self._json(*start_mission_from_ui(body))
            return
        if path_url == "/api/hover":
            try:
                body = json.loads(raw_body) if raw_body else {}
            except json.JSONDecodeError:
                self._json(400, {"error": "Invalid JSON"})
                return
            self._json(*start_hover_from_ui(body))
            return
        if path_url == "/api/abort":
            self._json(*abort_mission_from_ui())
            return
        if path_url == "/api/land":
            self._json(*land_from_ui())
            return
        self._json(404, {"error": "not found"})


def drone_snapshot(conn: ht.MavlinkConnection) -> dict:
    state = read_state(conn)
    with conn.state.lock:
        safety = conn.state.safety_switch_solid
        prearm = conn.state.prearm_ok
        mode = conn.state.custom_mode
        hb_age = time.time() - conn.state.hb_time if conn.state.hb_time else 99.0
    fix_names = {0: "no fix", 1: "no fix", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK float", 6: "RTK fixed"}
    located = hb_age < 3.0 and abs(state.lat) > 0.01 and state.fix >= 2
    net = ht.get_network_status()
    return {
        "drone_name": ht.DRONE_NAME,
        "serial_port": ht.PORT,
        "net_link": net.get("active_link", ""),
        "link": hb_age < 3.0,
        "located": located,
        "lat": state.lat,
        "lon": state.lon,
        "fix": state.fix,
        "fix_name": fix_names.get(state.fix, str(state.fix)),
        "sats": state.sats,
        "hdop": round(state.hdop, 2),
        "volt_v": round(state.volt, 2),
        "curr_a": round(state.curr, 2),
        "alt_ft": round(state.rel_alt * ht.M_TO_FT, 1),
        "roll_deg": round(state.roll, 1),
        "pitch_deg": round(state.pitch, 1),
        "yaw_deg": round(math.degrees(state.yaw) % 360.0, 1),
        "mode": ht.MODE_NAMES.get(mode, str(mode)),
        "armed": state.armed,
        "safety_solid": safety,
        "prearm_ok": prearm,
    }


def read_here() -> tuple[int, dict]:
    with MISSION.lock:
        if MISSION.running and MISSION.last_snap is not None:
            return 200, MISSION.last_snap
    with HERE_LOCK:
        busy = subprocess_port_busy()
        if busy:
            return 409, {"error": busy}
        try:
            conn = ht.MavlinkConnection()
        except Exception as exc:
            return 503, {"error": f"Could not open /dev/ttyAMA0 ({exc})"}
        try:
            conn.configure_streams()
            conn.send_command_long(ht.MAV_CMD_RUN_PREARM_CHECKS)
            deadline = time.time() + 4.0
            snap = drone_snapshot(conn)
            while time.time() < deadline:
                snap = drone_snapshot(conn)
                if snap["located"] and snap["volt_v"] > 1.0:
                    break
                time.sleep(0.2)
            if not snap["link"]:
                return 503, {"error": "No heartbeat from the Pixhawk on /dev/ttyAMA0"}
            with MISSION.lock:
                MISSION.last_snap = snap
            return 200, snap
        finally:
            try:
                conn.stop_streams()
            except Exception:
                pass
            conn.close()


def check_preflight() -> tuple[int, dict]:
    with MISSION.lock:
        if MISSION.running:
            return 409, {"error": "A mission is already running"}
    with HERE_LOCK:
        try:
            conn = open_mavlink_with_retry(close_qgc=True, timeout=4.0)
        except Exception as exc:
            return 503, {"error": f"Could not open /dev/ttyAMA0 ({exc})"}
        try:
            conn.configure_streams()
            conn.send_command_long(ht.MAV_CMD_RUN_PREARM_CHECKS)
            time.sleep(2.2)
            with conn.state.lock:
                ready, issues = ht.evaluate_readiness(conn.state, min_volt=ht.DEFAULT_MIN_TAKEOFF_VOLT)
                texts = list(conn.state.texts[-5:])
            snap = drone_snapshot(conn)
            if not ready and texts:
                for t in texts:
                    if t not in issues:
                        issues.append(f"FC: {t}")
            return 200, {"ready": ready, "issues": issues, "drone": snap}
        finally:
            try:
                conn.stop_streams()
            except Exception:
                pass
            conn.close()


def start_mission_from_ui(body: dict) -> tuple[int, dict]:
    if not body.get("confirm"):
        return 400, {"error": "Confirmation is required before starting takeoff"}
    with MISSION.lock:
        if MISSION.running:
            return 409, {"error": f"Mission '{MISSION.plan_name}' is already running"}
    try:
        name = safe_name(str(body.get("name") or ""))
        plan = load_plan(plan_path(name))
    except (OSError, ValueError, json.JSONDecodeError, KeyError) as exc:
        return 400, {"error": f"Could not load plan: {exc}"}
    start_delay_s = max(0, min(120, int(float(body.get("start_delay_s", 0)))))
    args = SimpleNamespace(
        gps_wait=float(body.get("gps_wait", 45.0)),
        min_takeoff_volt=float(body.get("min_takeoff_volt", ht.DEFAULT_MIN_TAKEOFF_VOLT)),
        low_volt_cutoff=float(body.get("low_volt_cutoff", ht.DEFAULT_LOW_VOLT_CUTOFF)),
        max_tilt_deg=float(body.get("max_tilt_deg", 25.0)),
        close_qgc=True,
        start_delay_s=start_delay_s,
    )
    with MISSION.lock:
        MISSION.logs.clear()
        MISSION.running = True
        MISSION.plan_name = plan["name"]
        MISSION.phase = "COUNTDOWN" if start_delay_s > 0 else "STARTING"
    describe_plan(plan)

    def _runner() -> None:
        try:
            fly_plan(plan, args)
        except Exception as exc:
            log(f"Mission error: {exc}")
            with MISSION.lock:
                MISSION.phase = "ERROR"
                MISSION.running = False

    thread = threading.Thread(target=_runner, daemon=True)
    with MISSION.lock:
        MISSION.thread = thread
    thread.start()
    return 200, {"ok": True, "name": plan["name"], "start_delay_s": start_delay_s}


def start_hover_from_ui(body: dict) -> tuple[int, dict]:
    if not body.get("confirm"):
        return 400, {"error": "Confirmation is required before starting Hover & RC Test"}
    with MISSION.lock:
        if MISSION.running:
            return 409, {"error": f"Mission '{MISSION.plan_name}' is already running"}
    try:
        alt_ft = float(body.get("alt_ft", 8.0))
        duration_s = float(body.get("duration_s", 60.0))
    except (TypeError, ValueError):
        return 400, {"error": "Hover height and duration must be numbers"}
    if not (ALT_FT_MIN <= alt_ft <= ALT_FT_MAX):
        return 400, {"error": f"Hover height must be {ALT_FT_MIN:.0f}–{ALT_FT_MAX:.0f} ft"}
    duration_s = max(10.0, min(300.0, duration_s))
    start_delay_s = max(0, min(120, int(float(body.get("start_delay_s", 0)))))
    args = SimpleNamespace(
        alt_ft=alt_ft,
        duration_s=duration_s,
        max_drift_ft=float(body.get("max_drift_ft", 6.0)),
        gps_wait=float(body.get("gps_wait", 45.0)),
        min_takeoff_volt=float(body.get("min_takeoff_volt", ht.DEFAULT_MIN_TAKEOFF_VOLT)),
        low_volt_cutoff=float(body.get("low_volt_cutoff", ht.DEFAULT_LOW_VOLT_CUTOFF)),
        max_tilt_deg=float(body.get("max_tilt_deg", 25.0)),
        close_qgc=True,
        start_delay_s=start_delay_s,
    )
    with MISSION.lock:
        MISSION.logs.clear()
        MISSION.running = True
        MISSION.plan_name = f"hover_{alt_ft:.0f}ft_{duration_s:.0f}s"
        MISSION.phase = "COUNTDOWN" if start_delay_s > 0 else "STARTING"

    def _runner() -> None:
        try:
            fly_hover_rc_test(args)
        except Exception as exc:
            log(f"Hover test error: {exc}")
            with MISSION.lock:
                MISSION.phase = "ERROR"
                MISSION.running = False

    thread = threading.Thread(target=_runner, daemon=True)
    with MISSION.lock:
        MISSION.thread = thread
    thread.start()
    return 200, {"ok": True, "alt_ft": alt_ft, "duration_s": duration_s, "start_delay_s": start_delay_s}


def abort_mission_from_ui() -> tuple[int, dict]:
    with MISSION.lock:
        if MISSION.running:
            MISSION.abort_requested = True
            return 200, {"ok": True, "message": "Stop requested. If airborne, the drone will land."}
    return 200, {"ok": True, "message": "No active mission to stop."}


def land_from_ui() -> tuple[int, dict]:
    with MISSION.lock:
        if MISSION.running:
            MISSION.land_requested = True
            MISSION.abort_requested = True
            return 200, {"ok": True, "message": "Active mission signaled to LAND immediately."}

    def _land_now() -> None:
        with HERE_LOCK:
            try:
                conn = open_mavlink_with_retry(close_qgc=True, timeout=4.0)
            except Exception as exc:
                log(f"Could not open /dev/ttyAMA0 for LAND: {exc}")
                return
            try:
                conn.configure_streams()
                ht.execute_landing(conn, "UI Land Now button")
            finally:
                try:
                    conn.stop_streams()
                except Exception:
                    pass
                conn.close()

    threading.Thread(target=_land_now, daemon=True).start()
    return 200, {"ok": True, "message": "Commanding LAND mode on Pixhawk..."}


def subprocess_port_busy() -> str | None:
    import subprocess

    try:
        result = subprocess.run(["fuser", ht.PORT], capture_output=True, text=True)
    except FileNotFoundError:
        return None
    pids = [part for part in result.stdout.split() if part.isdigit()]
    if pids:
        return f"{ht.PORT} is in use by PID {', '.join(pids)}. Close QGroundControl, then try again."
    return None


def planner_already_up(port: int) -> bool:
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/", timeout=1.5) as resp:
            return b"Flight1 path" in resp.read(8000)
    except Exception:
        return False


def desktop_env() -> dict:
    """Session variables the Pi browser needs. A desktop click already has them."""
    env = os.environ.copy()
    runtime = env.get("XDG_RUNTIME_DIR") or f"/run/user/{os.getuid()}"
    env.setdefault("XDG_RUNTIME_DIR", runtime)
    if not env.get("WAYLAND_DISPLAY") and os.path.exists(os.path.join(runtime, "wayland-0")):
        env["WAYLAND_DISPLAY"] = "wayland-0"
    if not env.get("DISPLAY") and os.path.exists("/tmp/.X11-unix/X0"):
        env["DISPLAY"] = ":0"
    bus = os.path.join(runtime, "bus")
    if "DBUS_SESSION_BUS_ADDRESS" not in env and os.path.exists(bus):
        env["DBUS_SESSION_BUS_ADDRESS"] = f"unix:path={bus}"
    return env


def open_page(url: str) -> None:
    import shutil
    import subprocess

    browser = shutil.which("chromium") or shutil.which("xdg-open")
    if browser is None:
        log(f"No browser found. Open {url} yourself.")
        return
    subprocess.Popen(
        [browser, url],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
        env=desktop_env(),
    )


def serve(port: int, open_browser: bool) -> int:
    plans_dir()
    url = f"http://127.0.0.1:{port}"
    if planner_already_up(port):
        if open_browser:
            open_page(url)
            log(f"Planner is already running. Opened {url}")
        else:
            log(f"Planner is already running at {url}")
        return 0
    try:
        server = ThreadingHTTPServer(("0.0.0.0", port), PlannerHandler)
    except OSError as exc:
        if exc.errno == errno.EADDRINUSE and open_browser:
            open_page(url)
            log(f"Port {port} is in use. Opened {url}")
            return 0
        raise
    net = ht.get_network_status()
    log(f"{ht.DRONE_NAME} Path Planner listening at {url} (serial {ht.PORT} @ {ht.BAUD})")
    for iface, ip_s in net.get("all_ips", []):
        log(f"  Network URL ({iface}): http://{ip_s}:{port}")
    log("Ctrl-C stops the page. It does not fly.")
    if open_browser:
        threading.Timer(0.5, lambda: open_page(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        log("\nPlanner stopped.")
    finally:
        server.server_close()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Flight1 map path planner and offline guided runner")
    sub = parser.add_subparsers(dest="cmd")

    default_web_port = int(ht.DRONE_CONFIG.get("web_port", 8080))
    p_serve = sub.add_parser("serve", help=f"Web map on port {default_web_port}. Saves plans. Does not fly.")
    p_serve.add_argument("--port", type=int, default=default_web_port)
    p_serve.add_argument("--open", action="store_true", help="Open the page in the Pi browser")

    sub.add_parser("plans", help="List saved plans")

    p_fly = sub.add_parser("fly", help="Arm, fly a saved plan, and land")
    p_fly.add_argument("plan", help="Path to a plan JSON file")
    p_fly.add_argument("--confirm", action="store_true", help="Required. Without this, nothing is armed.")
    p_fly.add_argument("--start-delay", dest="start_delay_s", type=float, default=0.0, help="Countdown seconds before pre-flight & takeoff")
    p_fly.add_argument("--gps-wait", type=float, default=45.0)
    p_fly.add_argument("--min-takeoff-volt", type=float, default=ht.DEFAULT_MIN_TAKEOFF_VOLT)
    p_fly.add_argument("--low-volt-cutoff", type=float, default=ht.DEFAULT_LOW_VOLT_CUTOFF)
    p_fly.add_argument("--max-tilt-deg", type=float, default=25.0)
    p_fly.add_argument("--close-qgc", action="store_true", default=True)

    p_hover = sub.add_parser("hover", help="8-ft / 60-s Hover & RC Stick Test (auto-returns and lands at launch spot)")
    p_hover.add_argument("--alt-ft", type=float, default=8.0, help="Hover height in feet (default: 8.0)")
    p_hover.add_argument("--duration", dest="duration_s", type=float, default=60.0, help="Hover duration in seconds (default: 60.0)")
    p_hover.add_argument("--max-drift-ft", type=float, default=6.0)
    p_hover.add_argument("--confirm", action="store_true", help="Required. Without this, nothing is armed.")
    p_hover.add_argument("--start-delay", dest="start_delay_s", type=float, default=0.0)
    p_hover.add_argument("--gps-wait", type=float, default=45.0)
    p_hover.add_argument("--min-takeoff-volt", type=float, default=ht.DEFAULT_MIN_TAKEOFF_VOLT)
    p_hover.add_argument("--low-volt-cutoff", type=float, default=ht.DEFAULT_LOW_VOLT_CUTOFF)
    p_hover.add_argument("--max-tilt-deg", type=float, default=25.0)
    p_hover.add_argument("--close-qgc", action="store_true", default=True)

    p_land = sub.add_parser("land", help="Command LAND now")
    p_land.add_argument("--close-qgc", action="store_true", default=True)

    args = parser.parse_args()
    cmd = args.cmd or "plans"
    if cmd == "serve":
        return serve(args.port, args.open)
    if cmd == "plans":
        rows = list_plans()
        if not rows:
            log(f"No plans in {PLANS_DIR}")
            return 0
        for plan in rows:
            describe_plan(plan)
        return 0
    if cmd == "fly":
        try:
            plan = load_plan(args.plan)
        except (OSError, ValueError, json.JSONDecodeError, KeyError) as exc:
            log(f"Could not read that plan: {exc}")
            return 1
        describe_plan(plan)
        if not args.confirm:
            log("\nNot arming. Re-run the same command with --confirm when the drone is outside and you are holding the transmitter.")
            return 2
        return fly_plan(plan, args)
    if cmd == "hover":
        log(f"Hover & RC Test: {args.alt_ft:.1f} ft for {args.duration_s:.0f} s (move sticks anytime; release 5s to return & land)")
        if not args.confirm:
            log("\nNot arming. Re-run with --confirm when the drone is outside and you are holding the transmitter.")
            return 2
        return fly_hover_rc_test(args)
    if cmd == "land":
        ht.ensure_port_free(close_qgc=args.close_qgc)
        conn = ht.MavlinkConnection()
        try:
            conn.configure_streams()
            ht.execute_landing(conn, "flight_path_planner.py land")
            return 0
        finally:
            conn.stop_streams()
            conn.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
