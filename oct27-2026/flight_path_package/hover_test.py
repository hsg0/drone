#!/usr/bin/env python3
"""Autonomous 5-foot Takeoff, 60-Second Active Position-Hold Hover, & Safe Landing for Flight1.

Runs on the Raspberry Pi 5 over TELEM1 (/dev/ttyAMA0, 57600 baud, MAVLink 2).

Usage on the Pi:
    python3 ~/Desktop/scripts/hover_test.py status
    python3 ~/Desktop/scripts/hover_test.py wifi [auto|G_suite_01|House-Gill-2.4]
    python3 ~/Desktop/scripts/hover_test.py run [--alt-ft 5.0] [--duration 60] [--max-drift-ft 6.0]
    python3 ~/Desktop/scripts/hover_test.py land

What `run` does:
  1. Verifies Wi-Fi (can auto-switch to G_suite_01 near the garage) and ignores SIGHUP
     once armed so a Wi-Fi hiccup never interrupts a flight in progress.
  2. Opens /dev/ttyAMA0 exclusively and streams 10 Hz real-time telemetry:
     LOCAL_POSITION_NED, GLOBAL_POSITION_INT, ATTITUDE, GPS_RAW_INT, EKF_STATUS_REPORT,
     RC_CHANNELS, SYS_STATUS, EXTENDED_SYS_STATE, and HEARTBEAT.
  3. Runs pre-flight safety checks: 3D GPS lock, EKF3 horizontal position lock,
     battery voltage, level attitude, and hardware safety switch solid red.
  4. Records the exact starting position (start_x, start_y, start_z) and heading,
     switches to GUIDED mode, arms, and commands takeoff to 5.0 ft (1.524 m).
  5. Holds a 60-second hover using real-time telemetry: measures North/East drift
     and altitude error at 10 Hz, applies bounded integral drift trim, and sends
     5 Hz SET_POSITION_TARGET_LOCAL_NED commands to keep the drone locked over its
     starting spot at 5.0 ft.
  6. Watches for RC controller input (stick movement or switch flip) or any safety
     failsafe (excessive drift, tilt, altitude error, GPS/EKF glitch, low battery,
     or Ctrl-C) and immediately transitions to LAND mode.
"""

import argparse
import csv
import datetime
import json
import math
import os
import signal
import struct
import subprocess
import sys
import threading
import time

import serial

DEFAULT_DRONE_CONFIG = {
    "drone_name": "Flight1",
    "serial_port": "/dev/ttyAMA0",
    "serial_baud": 57600,
    "mav_sysid": 1,
    "mav_compid": 1,
    "gcs_sysid": 255,
    "gcs_compid": 190,
    "web_port": 8080,
    "battery_cells": 3,
    "min_takeoff_volt": 10.8,
    "low_volt_cutoff": 10.3,
    "crit_volt_cutoff": 9.9,
    "rc_pitch_forward_is_high_pwm": True,
    "rc_roll_right_is_high_pwm": True,
    "rc_yaw_right_is_high_pwm": True,
    "rc_resume_idle_s": 5.0,
    "primary_wifi_ssid": "House-Gill-2.4",
    "secondary_wifi_ssid": "G_suite_01",
    "enable_4g_fallback": True,
    "cellular_interface": "auto",
}


def get_config_paths() -> list[str]:
    paths = []
    env_p = os.environ.get("DRONE_CONFIG")
    if env_p:
        paths.append(os.path.expanduser(env_p))
    script_dir = os.path.dirname(os.path.abspath(__file__))
    paths.append(os.path.join(script_dir, "drone_config.json"))
    paths.append(os.path.expanduser("~/.config/flight_path/drone_config.json"))
    paths.append(os.path.expanduser("~/drone_config.json"))
    return paths


def load_drone_config() -> dict:
    cfg = dict(DEFAULT_DRONE_CONFIG)
    for p in get_config_paths():
        if os.path.isfile(p):
            try:
                with open(p, "r", encoding="utf-8") as f:
                    user_cfg = json.load(f)
                if isinstance(user_cfg, dict):
                    cfg.update(user_cfg)
                    cfg["_config_path"] = p
                    break
            except Exception:
                pass
    return cfg


DRONE_CONFIG = load_drone_config()
DRONE_NAME = str(DRONE_CONFIG.get("drone_name", "Flight1"))
PORT = str(DRONE_CONFIG.get("serial_port", "/dev/ttyAMA0"))
BAUD = int(DRONE_CONFIG.get("serial_baud", 57600))
SRC_SYS = int(DRONE_CONFIG.get("gcs_sysid", 255))
SRC_COMP = int(DRONE_CONFIG.get("gcs_compid", 190))
TGT_SYS = int(DRONE_CONFIG.get("mav_sysid", 1))
TGT_COMP = int(DRONE_CONFIG.get("mav_compid", 1))
DEFAULT_MIN_TAKEOFF_VOLT = float(DRONE_CONFIG.get("min_takeoff_volt", 10.8))
DEFAULT_LOW_VOLT_CUTOFF = float(DRONE_CONFIG.get("low_volt_cutoff", 10.3))
DEFAULT_CRIT_VOLT_CUTOFF = float(DRONE_CONFIG.get("crit_volt_cutoff", 9.9))
RC_PITCH_SIGN = 1.0 if DRONE_CONFIG.get("rc_pitch_forward_is_high_pwm", True) else -1.0
RC_ROLL_SIGN = 1.0 if DRONE_CONFIG.get("rc_roll_right_is_high_pwm", True) else -1.0
RC_YAW_SIGN = 1.0 if DRONE_CONFIG.get("rc_yaw_right_is_high_pwm", True) else -1.0
RC_RESUME_IDLE_S = float(DRONE_CONFIG.get("rc_resume_idle_s", 5.0))

FT_TO_M = 0.3048
M_TO_FT = 3.280839895

# ArduCopter 4.7.2 custom mode numbers
MODE_STABILIZE = 0
MODE_ALT_HOLD = 2
MODE_AUTO = 3
MODE_GUIDED = 4
MODE_LOITER = 5
MODE_RTL = 6
MODE_LAND = 9
MODE_BRAKE = 17

MODE_NAMES = {
    0: "STABILIZE",
    2: "ALT_HOLD",
    3: "AUTO",
    4: "GUIDED",
    5: "LOITER",
    6: "RTL",
    9: "LAND",
    17: "BRAKE",
}

# MAVLink CRC_EXTRA table verified against ArduPilot common.xml / ardupilotmega.xml
CRC_EXTRA = {
    0: 50,     # HEARTBEAT
    1: 124,    # SYS_STATUS
    20: 214,   # PARAM_REQUEST_READ
    22: 220,   # PARAM_VALUE
    23: 168,   # PARAM_SET
    24: 24,    # GPS_RAW_INT
    30: 39,    # ATTITUDE
    32: 185,   # LOCAL_POSITION_NED
    33: 104,   # GLOBAL_POSITION_INT
    65: 118,   # RC_CHANNELS
    66: 148,   # REQUEST_DATA_STREAM
    74: 20,    # VFR_HUD
    76: 152,   # COMMAND_LONG
    77: 143,   # COMMAND_ACK
    82: 49,    # SET_ATTITUDE_TARGET
    84: 143,   # SET_POSITION_TARGET_LOCAL_NED
    193: 71,   # EKF_STATUS_REPORT
    242: 104,  # HOME_POSITION
    245: 130,  # EXTENDED_SYS_STATE
    253: 83,   # STATUSTEXT
}

# MAV_CMD constants
MAV_CMD_NAV_LAND = 21
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_DO_SET_MODE = 176
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_RUN_PREARM_CHECKS = 401
MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAV_CMD_REQUEST_MESSAGE = 512

# SET_POSITION_TARGET_LOCAL_NED bitmasks
MAV_FRAME_LOCAL_NED = 1
TYPEMASK_POS_ONLY_WITH_YAW = 56 | 448 | 2048        # ignore vel (56), acc (448), yaw_rate (2048) -> 2552
TYPEMASK_POS_VEL_WITH_YAW = 448 | 2048              # ignore acc (448), yaw_rate (2048) -> 2496

# EKF_STATUS_REPORT flags
EKF_ATTITUDE = 1 << 0
EKF_VELOCITY_HORIZ = 1 << 1
EKF_VELOCITY_VERT = 1 << 2
EKF_POS_HORIZ_REL = 1 << 3
EKF_POS_HORIZ_ABS = 1 << 4
EKF_CONST_POS_MODE = 1 << 7


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        t = b ^ (crc & 0xFF)
        t = (t ^ (t << 4)) & 0xFF
        crc = ((crc >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)) & 0xFFFF
    return crc


def wrap_pi(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


class TelemetryState:
    """Thread-safe container for the latest real-time telemetry from the Pixhawk."""

    def __init__(self):
        self.lock = threading.Lock()
        # Heartbeat
        self.hb_time = 0.0
        self.armed = False
        self.base_mode = 0
        self.custom_mode = -1
        self.system_status = 0
        # SYS_STATUS
        self.sys_time = 0.0
        self.volt_v = 0.0
        self.curr_a = 0.0
        self.prearm_ok = False
        self.safety_switch_solid = False
        # LOCAL_POSITION_NED
        self.lpos_time = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        # GLOBAL_POSITION_INT
        self.gpos_time = 0.0
        self.lat_deg = 0.0
        self.lon_deg = 0.0
        self.rel_alt_m = 0.0
        # ATTITUDE
        self.att_time = 0.0
        self.roll_rad = 0.0
        self.pitch_rad = 0.0
        self.yaw_rad = 0.0
        # GPS_RAW_INT
        self.gps_time = 0.0
        self.gps_fix = 0
        self.gps_sats = 0
        self.gps_hdop = 99.99
        # EKF_STATUS_REPORT
        self.ekf_time = 0.0
        self.ekf_flags = 0
        self.ekf_vel_var = 0.0
        self.ekf_pos_horiz_var = 0.0
        self.ekf_compass_var = 0.0
        # RC_CHANNELS
        self.rc_time = 0.0
        self.rc_chans = (0,) * 8
        self.rc_count = 0
        self.rc_rssi = 0
        # EXTENDED_SYS_STATE
        self.landed_state = 0
        # Queues
        self.texts = []
        self.acks = []


class MavlinkConnection:
    """MAVLink 2 serial link to the Pixhawk on /dev/ttyAMA0."""

    def __init__(self, port: str = PORT, baud: int = BAUD):
        self.ser = serial.Serial(port, baud, timeout=0.04, exclusive=True)
        self.ser.reset_input_buffer()
        self.state = TelemetryState()
        self.seq = 0
        self.tx_lock = threading.Lock()
        self.running = True
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.hb_thread = threading.Thread(target=self._gcs_hb_loop, daemon=True)
        self.rx_thread.start()
        self.hb_thread.start()

    def close(self):
        self.running = False
        self.rx_thread.join(timeout=1.0)
        self.hb_thread.join(timeout=1.0)
        try:
            self.ser.close()
        except Exception:
            pass

    def send_msg(self, msgid: int, payload: bytes):
        with self.tx_lock:
            head = (
                bytes([len(payload), 0, 0, self.seq & 0xFF, SRC_SYS, SRC_COMP])
                + struct.pack("<I", msgid)[:3]
            )
            crc = crc16(head + payload + bytes([CRC_EXTRA[msgid]]))
            frame = b"\xfd" + head + payload + struct.pack("<H", crc)
            self.ser.write(frame)
            self.seq = (self.seq + 1) & 0xFF

    def send_command_long(self, cmd: int, p1=0.0, p2=0.0, p3=0.0, p4=0.0, p5=0.0, p6=0.0, p7=0.0):
        payload = struct.pack(
            "<fffffffHBBB",
            float(p1), float(p2), float(p3), float(p4), float(p5), float(p6), float(p7),
            int(cmd), TGT_SYS, TGT_COMP, 0,
        )
        self.send_msg(76, payload)

    def set_message_interval(self, msg_id: int, rate_hz: float):
        interval_us = int(1_000_000 / rate_hz) if rate_hz > 0 else -1
        self.send_command_long(MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, interval_us)

    def request_single_message(self, msg_id: int):
        self.send_command_long(MAV_CMD_REQUEST_MESSAGE, msg_id)

    def configure_streams(self):
        # Set explicit MAVLink2 intervals for the real-time control loop
        rates = [
            (32, 10.0),   # LOCAL_POSITION_NED (10 Hz)
            (30, 10.0),   # ATTITUDE (10 Hz)
            (65, 10.0),   # RC_CHANNELS (10 Hz)
            (33, 5.0),    # GLOBAL_POSITION_INT (5 Hz)
            (24, 2.0),    # GPS_RAW_INT (2 Hz)
            (1, 2.0),     # SYS_STATUS (2 Hz)
            (193, 2.0),   # EKF_STATUS_REPORT (2 Hz)
            (245, 2.0),   # EXTENDED_SYS_STATE (2 Hz)
        ]
        for mid, hz in rates:
            self.set_message_interval(mid, hz)
            time.sleep(0.02)
        for mid, _ in rates:
            self.request_single_message(mid)
            time.sleep(0.02)

    def stop_streams(self):
        for mid in (32, 30, 65, 33, 24, 1, 193, 245):
            self.set_message_interval(mid, 0)
            time.sleep(0.01)

    def send_position_target_ned(
        self,
        x_m: float,
        y_m: float,
        z_m: float,
        yaw_rad: float,
        vx_ms: float = 0.0,
        vy_ms: float = 0.0,
        vz_ms: float = 0.0,
        use_velocity: bool = False,
    ):
        type_mask = TYPEMASK_POS_VEL_WITH_YAW if use_velocity else TYPEMASK_POS_ONLY_WITH_YAW
        ms = int((time.time() * 1000) % 0xFFFFFFFF)
        payload = struct.pack(
            "<IfffffffffffHBBB",
            ms,
            float(x_m),
            float(y_m),
            float(z_m),
            float(vx_ms),
            float(vy_ms),
            float(vz_ms),
            0.0, 0.0, 0.0,
            float(yaw_rad),
            0.0,
            type_mask,
            TGT_SYS,
            TGT_COMP,
            MAV_FRAME_LOCAL_NED,
        )
        self.send_msg(84, payload)

    def set_mode(self, mode_num: int, timeout: float = 2.5) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.send_command_long(MAV_CMD_DO_SET_MODE, 1.0, float(mode_num))
            if mode_num == MODE_LAND:
                self.send_command_long(MAV_CMD_NAV_LAND)
            end_wait = time.time() + 0.35
            while time.time() < end_wait:
                with self.state.lock:
                    if self.state.custom_mode == mode_num:
                        return True
                time.sleep(0.03)
        with self.state.lock:
            return self.state.custom_mode == mode_num

    def arm(self, timeout: float = 4.0) -> tuple[bool, list[str]]:
        with self.state.lock:
            self.state.texts.clear()
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
            end_wait = time.time() + 0.45
            while time.time() < end_wait:
                with self.state.lock:
                    if self.state.armed:
                        return True, list(self.state.texts)
                time.sleep(0.04)
        with self.state.lock:
            return self.state.armed, list(self.state.texts)

    def disarm(self, force: bool = False, timeout: float = 3.0) -> bool:
        magic = 21196.0 if force else 0.0
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, 0.0, magic)
            end_wait = time.time() + 0.35
            while time.time() < end_wait:
                with self.state.lock:
                    if not self.state.armed:
                        return True
                time.sleep(0.04)
        with self.state.lock:
            return not self.state.armed

    def command_takeoff(self, alt_m: float, timeout: float = 4.5) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self.state.lock:
                self.state.acks.clear()
            self.send_command_long(MAV_CMD_NAV_TAKEOFF, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, float(alt_m))
            end_wait = time.time() + 0.50
            while time.time() < end_wait:
                with self.state.lock:
                    for cmd, res in self.state.acks:
                        if cmd == MAV_CMD_NAV_TAKEOFF:
                            if res == 0:
                                return True
                            break
                time.sleep(0.04)
            time.sleep(0.25)
        return False

    def _gcs_hb_loop(self):
        payload = struct.pack("<IBBBBB", 0, 6, 8, 0, 0, 3)
        while self.running:
            try:
                self.send_msg(0, payload)
            except Exception:
                pass
            time.sleep(1.0)

    def _rx_loop(self):
        buf = bytearray()
        while self.running:
            try:
                chunk = self.ser.read(512)
            except Exception:
                time.sleep(0.02)
                continue
            if chunk:
                buf.extend(chunk)
            while len(buf) >= 12:
                k = next((n for n, b in enumerate(buf) if b in (0xFD, 0xFE)), None)
                if k is None:
                    buf.clear()
                    break
                if k > 0:
                    del buf[:k]
                if len(buf) < 12:
                    break
                if buf[0] == 0xFD:
                    plen, inc = buf[1], buf[2]
                    if inc & ~1:
                        del buf[:1]
                        continue
                    total = 12 + plen + (13 if inc & 1 else 0)
                    sysid = buf[5]
                    msgid = buf[7] | (buf[8] << 8) | (buf[9] << 16)
                    head = bytes(buf[1:10])
                    start = 10
                else:
                    plen = buf[1]
                    total = 8 + plen
                    sysid = buf[3]
                    msgid = buf[5]
                    head = bytes(buf[1:6])
                    start = 6
                if len(buf) < total:
                    break
                payload = bytes(buf[start : start + plen])
                crc_rx = buf[start + plen] | (buf[start + plen + 1] << 8)
                if msgid in CRC_EXTRA:
                    if crc16(head + payload + bytes([CRC_EXTRA[msgid]])) != crc_rx:
                        del buf[:1]
                        continue
                    if sysid == TGT_SYS:
                        self._handle_msg(msgid, payload)
                del buf[:total]

    def _handle_msg(self, msgid: int, raw: bytes):
        now = time.time()
        p = raw + bytes(64)
        with self.state.lock:
            if msgid == 0:
                custom_mode, mtype, ap, base_mode, sys_status, _ = struct.unpack("<IBBBBB", p[:9])
                if mtype != 6:  # ignore GCS heartbeats
                    self.state.hb_time = now
                    self.state.custom_mode = custom_mode
                    self.state.base_mode = base_mode
                    self.state.armed = bool(base_mode & 128)
                    self.state.system_status = sys_status
            elif msgid == 1:
                present, enabled, health = struct.unpack("<III", p[:12])
                v_mv, c_ca = struct.unpack("<Hh", p[14:18])
                self.state.sys_time = now
                self.state.volt_v = v_mv / 1000.0
                self.state.curr_a = c_ca / 100.0
                self.state.prearm_ok = bool(health & (1 << 28))
                self.state.safety_switch_solid = bool(enabled & (1 << 15))
            elif msgid == 32:
                _, x, y, z, vx, vy, vz = struct.unpack("<Iffffff", p[:28])
                self.state.lpos_time = now
                self.state.x = x
                self.state.y = y
                self.state.z = z
                self.state.vx = vx
                self.state.vy = vy
                self.state.vz = vz
            elif msgid == 33:
                _, lat, lon, _, rel_alt = struct.unpack("<Iiiii", p[:20])
                self.state.gpos_time = now
                self.state.lat_deg = lat / 1e7
                self.state.lon_deg = lon / 1e7
                self.state.rel_alt_m = rel_alt / 1000.0
            elif msgid == 30:
                _, r, pt, y, _, _, _ = struct.unpack("<Iffffff", p[:28])
                self.state.att_time = now
                self.state.roll_rad = r
                self.state.pitch_rad = pt
                self.state.yaw_rad = y
            elif msgid == 24:
                _, _, _, _, eph, _, _, _, fix, sats = struct.unpack("<QiiiHHHHBB", p[:30])
                self.state.gps_time = now
                self.state.gps_fix = fix
                self.state.gps_sats = sats
                self.state.gps_hdop = eph / 100.0 if eph < 65535 else 99.99
            elif msgid == 193:
                vel_v, pos_h_v, _, comp_v, _, flags = struct.unpack("<fffffH", p[:22])
                self.state.ekf_time = now
                self.state.ekf_vel_var = vel_v
                self.state.ekf_pos_horiz_var = pos_h_v
                self.state.ekf_compass_var = comp_v
                self.state.ekf_flags = flags
            elif msgid == 65:
                chans = struct.unpack("<8H", p[4:20])
                self.state.rc_time = now
                self.state.rc_chans = chans
                self.state.rc_count = p[40]
                self.state.rc_rssi = p[41]
            elif msgid == 245:
                _, landed = struct.unpack("<BB", p[:2])
                self.state.landed_state = landed
            elif msgid == 253:
                txt = p[1:51].split(b"\0", 1)[0].decode("ascii", "replace").strip()
                if txt:
                    self.state.texts.append(txt)
                    print(f"  [FC] {txt}", flush=True)
            elif msgid == 77:
                cmd, res = struct.unpack("<HB", p[:3])
                self.state.acks.append((cmd, res))


# ---------------------------------------------------------------------------
# Wi-Fi & 4G/LTE Cellular Fallback Helper
# ---------------------------------------------------------------------------

def get_network_status() -> dict:
    """Return active Wi-Fi SSID, Wi-Fi IP, 4G/LTE interface & IP, and default route link."""
    primary_ssid = str(DRONE_CONFIG.get("primary_wifi_ssid") or "").strip()
    secondary_ssid = str(DRONE_CONFIG.get("secondary_wifi_ssid") or "").strip()
    cell_cfg = str(DRONE_CONFIG.get("cellular_interface") or "auto").strip()
    enable_4g = bool(DRONE_CONFIG.get("enable_4g_fallback", True))

    status = {
        "active_ssid": None,
        "wifi_ip": None,
        "cell_iface": None,
        "cell_ip": None,
        "default_iface": None,
        "active_link": "Offline",
        "all_ips": [],
        "signals": {},
        "enable_4g": enable_4g,
    }

    # 1. Read IPv4 addresses across all up interfaces
    try:
        out = subprocess.check_output(["ip", "-4", "-o", "addr", "show"], text=True, timeout=3)
        for line in out.splitlines():
            parts = line.split()
            if len(parts) < 4:
                continue
            iface = parts[1]
            if iface == "lo":
                continue
            ip_str = parts[3].split("/")[0]
            status["all_ips"].append((iface, ip_str))
            if iface.startswith(("wlan", "wl")):
                status["wifi_ip"] = ip_str
            elif (
                (cell_cfg != "auto" and iface == cell_cfg)
                or iface.startswith(("wwan", "usb", "ppp", "lte", "enx", "eth1", "rmnet"))
            ):
                status["cell_iface"] = iface
                status["cell_ip"] = ip_str
    except Exception:
        pass

    # 2. Check default route
    try:
        rout = subprocess.check_output(["ip", "route", "show", "default"], text=True, timeout=3)
        for line in rout.splitlines():
            parts = line.split()
            if "dev" in parts:
                idx = parts.index("dev")
                if idx + 1 < len(parts):
                    status["default_iface"] = parts[idx + 1]
                    break
    except Exception:
        pass

    # 3. Check Wi-Fi SSID & signals if nmcli is available
    try:
        out = subprocess.check_output(
            ["nmcli", "-t", "-f", "IN-USE,SSID,SIGNAL", "dev", "wifi", "list", "--rescan", "no"],
            text=True,
            timeout=4,
        )
        for line in out.splitlines():
            parts = line.split(":")
            if len(parts) >= 3:
                in_use, ssid, sig_str = parts[0], parts[1], parts[2]
                if not ssid:
                    continue
                sig = int(sig_str) if sig_str.isdigit() else 0
                if in_use == "*":
                    status["active_ssid"] = ssid
                status["signals"][ssid] = max(status["signals"].get(ssid, 0), sig)
    except Exception:
        pass

    def_if = status["default_iface"] or ""
    if status["cell_ip"] and (def_if == status["cell_iface"] or not status["wifi_ip"]):
        status["active_link"] = f"4G/LTE ({status['cell_iface']} {status['cell_ip']})"
    elif status["wifi_ip"] or status["active_ssid"]:
        ssid_lbl = status["active_ssid"] or "Wi-Fi"
        ip_lbl = f" {status['wifi_ip']}" if status["wifi_ip"] else ""
        cell_lbl = f" • 4G standby ({status['cell_iface']})" if status["cell_ip"] else ""
        status["active_link"] = f"Wi-Fi ({ssid_lbl}{ip_lbl}){cell_lbl}"
    elif status["all_ips"]:
        iface, ip_s = status["all_ips"][0]
        status["active_link"] = f"{iface} ({ip_s})"

    status["primary_ssid"] = primary_ssid
    status["secondary_ssid"] = secondary_ssid
    return status


def ensure_4g_fallback() -> bool:
    """If Wi-Fi is unavailable and 4G fallback is enabled, bring up any cellular/GSM/LTE connection."""
    if not DRONE_CONFIG.get("enable_4g_fallback", True):
        return False
    net = get_network_status()
    if net.get("cell_ip"):
        return True
    cell_cfg = str(DRONE_CONFIG.get("cellular_interface") or "auto").strip()
    try:
        out = subprocess.check_output(
            ["nmcli", "-t", "-f", "NAME,TYPE,DEVICE", "connection", "show"],
            text=True,
            timeout=4,
        )
        for line in out.splitlines():
            parts = line.split(":")
            if len(parts) >= 2:
                name, ctype = parts[0], parts[1].lower()
                dev = parts[2] if len(parts) > 2 else ""
                if (
                    "gsm" in ctype
                    or "cdma" in ctype
                    or "cellular" in name.lower()
                    or "4g" in name.lower()
                    or "lte" in name.lower()
                    or (cell_cfg != "auto" and (dev == cell_cfg or name == cell_cfg))
                ):
                    subprocess.run(["nmcli", "connection", "up", name], capture_output=True, timeout=10)
                    return True
    except Exception:
        pass
    return False


def get_wifi_status() -> dict:
    """Return active SSID and visible signal levels."""
    info = {"active_ssid": None, "signals": {}}
    try:
        out = subprocess.check_output(
            ["nmcli", "-t", "-f", "IN-USE,SSID,SIGNAL", "dev", "wifi", "list", "--rescan", "auto"],
            text=True,
            timeout=6,
        )
        for line in out.splitlines():
            parts = line.split(":")
            if len(parts) >= 3:
                in_use, ssid, sig_str = parts[0], parts[1], parts[2]
                if not ssid:
                    continue
                sig = int(sig_str) if sig_str.isdigit() else 0
                if in_use == "*":
                    info["active_ssid"] = ssid
                info["signals"][ssid] = max(info["signals"].get(ssid, 0), sig)
    except Exception as exc:
        info["error"] = str(exc)
    return info


def switch_wifi_connection(profile_name: str) -> bool:
    """Activate a saved NetworkManager Wi-Fi profile (works both on desktop and via SSH)."""
    for candidate in (profile_name, f"netplan-wlan0-{profile_name}"):
        try:
            res = subprocess.run(
                ["nmcli", "connection", "up", candidate],
                capture_output=True,
                text=True,
                timeout=12,
            )
            if res.returncode == 0:
                return True
        except Exception:
            pass

    # Fallback when called from an SSH session: run nmcli inside the active seat0 desktop session
    desktop_path = "/tmp/flight1_wifi_switch.desktop"
    out_path = "/tmp/flight1_wifi_switch.out"
    uid = os.getuid()
    try:
        with open(desktop_path, "w") as f:
            f.write(
                "[Desktop Entry]\n"
                "Type=Application\n"
                "Name=Flight1WiFiSwitch\n"
                f'Exec=/bin/sh -c "nmcli connection up \'{profile_name}\' || nmcli connection up \'netplan-wlan0-{profile_name}\' > {out_path} 2>&1"\n'
                "Terminal=false\n"
            )
        env = os.environ.copy()
        env.update({"XDG_RUNTIME_DIR": f"/run/user/{uid}", "WAYLAND_DISPLAY": "wayland-0", "DISPLAY": ":0"})
        subprocess.run(["pcmanfm", desktop_path], env=env, timeout=5, capture_output=True)
        time.sleep(3.0)
    finally:
        for p in (desktop_path, out_path):
            try:
                os.remove(p)
            except OSError:
                pass
    now_info = get_wifi_status()
    return now_info.get("active_ssid") == profile_name


def cmd_wifi(target: str) -> int:
    primary_ssid = str(DRONE_CONFIG.get("primary_wifi_ssid") or "").strip()
    secondary_ssid = str(DRONE_CONFIG.get("secondary_wifi_ssid") or "").strip()
    info = get_wifi_status()
    net = get_network_status()
    active = info.get("active_ssid")
    sigs = info.get("signals", {})
    s_sig = sigs.get(secondary_ssid, 0) if secondary_ssid else 0
    p_sig = sigs.get(primary_ssid, 0) if primary_ssid else 0
    print(f"Current Network    : {net.get('active_link')}")
    print(f"Current Wi-Fi SSID : {active or 'disconnected'}")
    if secondary_ssid:
        print(f"  {secondary_ssid:<16} : {s_sig}% signal")
    if primary_ssid:
        print(f"  {primary_ssid:<16} : {p_sig}% signal")
    if net.get("cell_ip"):
        print(f"  4G/LTE Interface : {net['cell_iface']} ({net['cell_ip']})")

    if target == "status":
        return 0
    if secondary_ssid and target.lower() in (secondary_ssid.lower(), "secondary", "garage"):
        if active == secondary_ssid:
            print(f"Already connected to {secondary_ssid}.")
            return 0
        print(f"Switching Wi-Fi to {secondary_ssid}...")
        ok = switch_wifi_connection(secondary_ssid)
        print(f"Connected to {secondary_ssid}." if ok else f"Failed to switch to {secondary_ssid}.")
        return 0 if ok else 1
    if primary_ssid and target.lower() in (primary_ssid.lower(), "primary", "house"):
        if active == primary_ssid:
            print(f"Already connected to {primary_ssid}.")
            return 0
        print(f"Switching Wi-Fi to {primary_ssid}...")
        ok = switch_wifi_connection(primary_ssid)
        print(f"Connected to {primary_ssid}." if ok else f"Failed to switch to {primary_ssid}.")
        return 0 if ok else 1
    if target == "auto":
        if active is None and secondary_ssid and s_sig > 0:
            print(f"Wi-Fi disconnected; connecting to {secondary_ssid}...")
            switch_wifi_connection(secondary_ssid)
        elif secondary_ssid and active != secondary_ssid and s_sig >= p_sig + 10 and s_sig > 0:
            print(f"{secondary_ssid} ({s_sig}%) is stronger than {active} ({p_sig}%); switching to {secondary_ssid}...")
            switch_wifi_connection(secondary_ssid)
        elif primary_ssid and active != primary_ssid and p_sig >= s_sig + 15 and p_sig > 0:
            print(f"{primary_ssid} ({p_sig}%) is stronger than {active} ({s_sig}%); switching to {primary_ssid}...")
            switch_wifi_connection(primary_ssid)
        elif active is None and p_sig == 0 and s_sig == 0:
            if ensure_4g_fallback():
                print("No configured Wi-Fi in range; 4G/LTE fallback is active.")
            else:
                print("No configured Wi-Fi in range (operating offline).")
        else:
            print(f"Keeping current connection ({net.get('active_link')}).")
        return 0
    return 0


# ---------------------------------------------------------------------------
# RC Override Detector
# ---------------------------------------------------------------------------

def parse_rc_nudge(
    baseline_chans: tuple[int, ...],
    current_chans: tuple[int, ...],
    thr_centered_once: bool = False,
) -> tuple[bool, str | None, float, float, float, float, bool]:
    """Check RC sticks for in-flight nudge override vs switch flip.

    Returns:
      (switch_flipped, switch_reason, norm_fwd, norm_right, norm_yaw, norm_climb, thr_centered_once)
    """
    if not current_chans or len(current_chans) < 4 or all(c == 0 for c in current_chans[:4]):
        return False, None, 0.0, 0.0, 0.0, 0.0, thr_centered_once

    if baseline_chans and len(baseline_chans) >= 6:
        for idx in (4, 5):
            val = current_chans[idx]
            base = baseline_chans[idx]
            if 850 < val < 2150 and 850 < base < 2150 and abs(val - base) > 150:
                return True, f"Pilot flipped switch CH{idx + 1} ({base} -> {val} us)", 0.0, 0.0, 0.0, 0.0, thr_centered_once

    base_r = baseline_chans[0] if (baseline_chans and 1350 <= baseline_chans[0] <= 1650) else 1500
    base_p = baseline_chans[1] if (baseline_chans and 1350 <= baseline_chans[1] <= 1650) else 1500
    base_y = baseline_chans[3] if (baseline_chans and 1350 <= baseline_chans[3] <= 1650) else 1500

    ch1, ch2, ch3, ch4 = current_chans[0], current_chans[1], current_chans[2], current_chans[3]

    norm_right = 0.0
    if 850 < ch1 < 2150 and abs(ch1 - base_r) > 75:
        norm_right = max(-1.0, min(1.0, ((ch1 - base_r) / 450.0) * RC_ROLL_SIGN))

    norm_fwd = 0.0
    if 850 < ch2 < 2150 and abs(ch2 - base_p) > 75:
        norm_fwd = max(-1.0, min(1.0, ((ch2 - base_p) / 450.0) * RC_PITCH_SIGN))

    norm_yaw = 0.0
    if 850 < ch4 < 2150 and abs(ch4 - base_y) > 85:
        norm_yaw = max(-1.0, min(1.0, ((ch4 - base_y) / 450.0) * RC_YAW_SIGN))

    norm_climb = 0.0
    if 850 < ch3 < 2150:
        if ch3 >= 1450:
            thr_centered_once = True
        if ch3 > 1585:
            norm_climb = max(0.0, min(1.0, (ch3 - 1500) / 450.0))
        elif thr_centered_once and 1150 < ch3 < 1415:
            norm_climb = max(-1.0, min(0.0, (ch3 - 1500) / 450.0))

    return False, None, norm_fwd, norm_right, norm_yaw, norm_climb, thr_centered_once


def check_rc_override(baseline_chans: tuple[int, ...], current_chans: tuple[int, ...]) -> str | None:
    """Return a human-readable reason if the pilot moved sticks or flipped a switch on the RC remote."""
    if not current_chans or all(c == 0 for c in current_chans[:4]):
        return None

    # If the transmitter was OFF at baseline and a valid signal just came in:
    if not baseline_chans or all(c == 0 for c in baseline_chans[:4]):
        if any(900 < c < 2100 for c in current_chans[:4]):
            return f"RC transmitter signal came in ({current_chans[:4]})"
        return None

    # Check Roll (CH1), Pitch (CH2), Yaw (CH4) deflection from 1500 (or baseline)
    names = {0: "Roll (CH1)", 1: "Pitch (CH2)", 3: "Yaw (CH4)"}
    for idx, label in names.items():
        val = current_chans[idx]
        base = baseline_chans[idx] if 1350 <= baseline_chans[idx] <= 1650 else 1500
        if 850 < val < 2150 and abs(val - base) > 85:
            return f"Pilot moved {label} stick ({val} us vs neutral {base} us)"

    # Check Throttle (CH3): if moved up > 120 us from idle
    thr = current_chans[2]
    base_thr = baseline_chans[2]
    if 850 < thr < 2150 and base_thr > 850 and abs(thr - base_thr) > 120:
        return f"Pilot moved Throttle (CH3) stick ({thr} us vs start {base_thr} us)"

    # Check switches CH5 and CH6
    for idx in (4, 5):
        val = current_chans[idx]
        base = baseline_chans[idx]
        if 850 < val < 2150 and 850 < base < 2150 and abs(val - base) > 150:
            return f"Pilot flipped switch CH{idx + 1} ({base} -> {val} us)"

    return None


# ---------------------------------------------------------------------------
# Pre-flight Status & Autonomous Hover Run
# ---------------------------------------------------------------------------

def ensure_port_free(close_qgc: bool = False):
    try:
        res = subprocess.run(["fuser", PORT], capture_output=True, text=True)
        pids = [p for p in res.stdout.strip().split() if p.isdigit()]
        if pids:
            if close_qgc:
                print(f"Closing QGroundControl (PID {' '.join(pids)}) so {PORT} is free...", flush=True)
                subprocess.run(["killall", "QGroundControl"], capture_output=True)
                for step in range(30):
                    time.sleep(0.2)
                    chk = subprocess.run(["fuser", PORT], capture_output=True, text=True)
                    rem = [p for p in chk.stdout.strip().split() if p.isdigit()]
                    if not rem:
                        break
                    if step == 20:
                        subprocess.run(["kill", "-9", *rem], capture_output=True)
            else:
                print(
                    f"ERROR: {PORT} is currently held open by PID(s) {' '.join(pids)} (likely QGroundControl).\n"
                    "Close QGroundControl on the Pi desktop first, or pass --close-qgc."
                )
                sys.exit(1)
    except FileNotFoundError:
        pass


def evaluate_readiness(st: TelemetryState, min_volt: float = DEFAULT_MIN_TAKEOFF_VOLT) -> tuple[bool, list[str]]:
    issues = []
    now = time.time()
    if now - st.hb_time > 3.0:
        issues.append(f"No HEARTBEAT from Pixhawk on {PORT}")
    if st.volt_v < min_volt:
        issues.append(f"Battery voltage too low ({st.volt_v:.2f} V < {min_volt:.2f} V)")
    if not st.safety_switch_solid:
        issues.append("Hardware safety switch is NOT solid red (press and hold button on drone for ~2s)")
    if st.gps_fix < 3 or st.gps_sats < 5:
        issues.append(f"GPS needs 3D lock with >=5 satellites (currently fix={st.gps_fix}, sats={st.gps_sats})")
    if st.gps_hdop > 2.5:
        issues.append(f"GPS HDOP is too high ({st.gps_hdop:.2f} > 2.50)")
    if now - st.lpos_time > 2.0:
        issues.append("EKF3 local position (LOCAL_POSITION_NED) not available yet")
    if st.ekf_time > 0 and (st.ekf_flags & EKF_CONST_POS_MODE):
        issues.append("EKF3 is in constant-position mode (waiting for GPS/EKF position lock)")
    roll_deg = math.degrees(st.roll_rad)
    pitch_deg = math.degrees(st.pitch_rad)
    if abs(roll_deg) > 10.0 or abs(pitch_deg) > 10.0:
        issues.append(f"Drone is tilted on the ground (roll={roll_deg:+.1f} deg, pitch={pitch_deg:+.1f} deg)")
    # Verify RC remote is powered ON, paired, and not in radio failsafe
    rc_valid = (now - st.rc_time <= 2.5) and all(900 < c < 2100 for c in st.rc_chans[:4])
    rc_fs = any("radio failsafe" in t.lower() or "rc not found" in t.lower() for t in st.texts[-4:])
    if not rc_valid or rc_fs:
        issues.append("RC remote is not paired/connected (turn ON handheld RC transmitter)")
    elif st.rc_chans[2] > 1150:
        issues.append(f"RC throttle stick (CH3={st.rc_chans[2]}) is not all the way down (<1150)")
    if not st.prearm_ok:
        issues.append("Pixhawk PreArm check flag is False (see [FC] messages above)")
    return len(issues) == 0, issues


def print_telemetry_snapshot(conn: MavlinkConnection):
    conn.configure_streams()
    conn.send_command_long(MAV_CMD_RUN_PREARM_CHECKS)
    time.sleep(2.2)
    net = get_network_status()
    with conn.state.lock:
        st = conn.state
        mode_str = MODE_NAMES.get(st.custom_mode, f"MODE_{st.custom_mode}")
        roll_d = math.degrees(st.roll_rad)
        pitch_d = math.degrees(st.pitch_rad)
        yaw_d = math.degrees(st.yaw_rad) % 360.0
        ok, issues = evaluate_readiness(st)
        print(f"\n=== {DRONE_NAME} Pre-Flight Telemetry Snapshot ===")
        print(f"  Serial Link     : {PORT} @ {BAUD} baud")
        print(f"  Network Link    : {net.get('active_link')}")
        print(f"  Flight Mode     : {mode_str} (armed={st.armed})")
        print(f"  Battery         : {st.volt_v:.2f} V, {st.curr_a:.2f} A")
        print(f"  Safety Switch   : {'SOLID RED (Ready)' if st.safety_switch_solid else 'BLINKING (Safe / Locked)'}")
        print(f"  PreArm Status   : {'READY' if st.prearm_ok else 'NOT READY'}")
        print(f"  GPS Lock        : fix_type={st.gps_fix}, satellites={st.gps_sats}, HDOP={st.gps_hdop:.2f}")
        print(
            f"  EKF3 Local Pos  : N={st.x:+.2f} m, E={st.y:+.2f} m, D={st.z:+.2f} m "
            f"(age={time.time() - st.lpos_time:.1f}s, flags=0x{st.ekf_flags:02x})"
        )
        print(f"  Attitude        : roll={roll_d:+.1f} deg, pitch={pitch_d:+.1f} deg, yaw={yaw_d:.1f} deg")
        print(f"  RC Channels 1-6 : {st.rc_chans[:6]}")
        if ok:
            print("\nREADY FOR HOVER TEST: All pre-flight checks passed.")
        else:
            print("\nNOT READY FOR TAKEOFF YET:")
            for item in issues:
                print(f"  - {item}")
    conn.stop_streams()
    return 0 if ok else 1


def execute_landing(conn: MavlinkConnection, reason: str):
    """Switch immediately to LAND mode, monitor descent, and guarantee full motor disarm on touchdown."""
    print(f"\n>>> INITIATING LANDING: {reason} <<<", flush=True)
    conn.set_mode(MODE_LAND, timeout=3.0)
    start_land = time.time()
    last_print = 0.0
    ground_still_since = None
    while time.time() - start_land < 35.0:
        with conn.state.lock:
            armed = conn.state.armed
            mode = MODE_NAMES.get(conn.state.custom_mode, str(conn.state.custom_mode))
            alt_m = conn.state.rel_alt_m
            vz = conn.state.vz
            landed = conn.state.landed_state
        if not armed:
            print(">>> TOUCHDOWN & DISARM CONFIRMED. Motors stopped. <<<", flush=True)
            conn.set_mode(MODE_STABILIZE, timeout=1.5)
            return
        now = time.time()
        if now - last_print >= 1.0:
            print(
                f"  [LANDING] mode={mode} | alt={alt_m * M_TO_FT:4.1f} ft ({alt_m:.2f} m) | "
                f"descent_rate={vz * M_TO_FT:+4.1f} ft/s",
                flush=True,
            )
            last_print = now
        # Detect touchdown either via EXTENDED_SYS_STATE landed==1 or near-zero vertical rate near ground
        if alt_m < 0.45 and abs(vz) < 0.18:
            if ground_still_since is None:
                ground_still_since = now
        else:
            ground_still_since = None

        on_ground_settled = (ground_still_since is not None and (now - ground_still_since) >= 1.8)
        if (landed == 1 and now - start_land > 2.5) or (on_ground_settled and now - start_land > 3.5):
            if not conn.disarm(force=False, timeout=1.2):
                print("  [LANDING] Touchdown detected; forcing motor disarm so motors do not idle...", flush=True)
                conn.disarm(force=True, timeout=1.5)
        time.sleep(0.1)

    print("Landing timeout reached; forcing motor disarm...", flush=True)
    if not conn.disarm(force=False, timeout=1.5):
        conn.disarm(force=True, timeout=2.0)
    conn.set_mode(MODE_STABILIZE, timeout=1.5)


def run_hover_test(args) -> int:
    target_alt_m = args.alt_ft * FT_TO_M
    max_drift_m = args.max_drift_ft * FT_TO_M
    max_alt_m = max(target_alt_m + 1.5, 3.0)
    hover_duration_s = float(args.duration)

    if args.wifi:
        cmd_wifi(args.wifi)

    ensure_port_free(close_qgc=args.close_qgc)
    conn = MavlinkConnection()

    abort_reason = None

    def sig_handler(signum, _frame):
        nonlocal abort_reason
        abort_reason = f"Received signal {signum} (Ctrl-C / termination)"

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    try:
        print("Configuring 10 Hz real-time telemetry streams on /dev/ttyAMA0...", flush=True)
        conn.configure_streams()
        conn.send_command_long(MAV_CMD_RUN_PREARM_CHECKS)
        time.sleep(2.5)

        # If EKF3 is still waiting for its 10-second stationary GPS alignment, wait up to --gps-wait seconds
        wait_deadline = time.time() + float(args.gps_wait)
        while time.time() < wait_deadline and abort_reason is None:
            with conn.state.lock:
                ok, issues = evaluate_readiness(conn.state, min_volt=args.min_takeoff_volt)
                sats = conn.state.gps_sats
                hdop = conn.state.gps_hdop
                fix = conn.state.gps_fix
                ekf_flags = conn.state.ekf_flags
                lpos_ok = (time.time() - conn.state.lpos_time) < 2.0
            if ok:
                break
            # Only keep waiting if the remaining issue is GPS/EKF lock or RC transmitter being turned on
            print(
                f"  [WAITING FOR READY] fix={fix} sats={sats} hdop={hdop:.2f} "
                f"ekf=0x{ekf_flags:02x} lpos={lpos_ok} | issues: {'; '.join(issues)}",
                flush=True,
            )
            conn.send_command_long(MAV_CMD_RUN_PREARM_CHECKS)
            time.sleep(2.0)

        with conn.state.lock:
            ok, issues = evaluate_readiness(conn.state, min_volt=args.min_takeoff_volt)
            if not ok and not args.skip_gps_wait:
                print("\nPre-flight checks did not pass:", flush=True)
                for item in issues:
                    print(f"  - {item}", flush=True)
                print("Aborting before arming. Resolve the items above and re-run.", flush=True)
                conn.stop_streams()
                return 1

            start_x = conn.state.x
            start_y = conn.state.y
            start_z = conn.state.z
            start_yaw = conn.state.yaw_rad
            baseline_rc = conn.state.rc_chans

        # Ignore SIGHUP once we begin the flight sequence so a garage Wi-Fi drop never kills the script mid-air
        signal.signal(signal.SIGHUP, signal.SIG_IGN)

        log_dir = os.path.expanduser("~/hover_test_logs")
        os.makedirs(log_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(log_dir, f"hover_{stamp}.csv")
        csv_file = open(csv_path, "w", newline="")
        writer = csv.writer(csv_file)
        writer.writerow([
            "timestamp", "phase", "elapsed_s", "mode", "armed",
            "alt_ft", "target_alt_ft", "drift_ft", "north_err_ft", "east_err_ft",
            "cmd_trim_n_m", "cmd_trim_e_m", "vel_horiz_fps", "vz_fps",
            "roll_deg", "pitch_deg", "yaw_deg", "volt_v", "curr_a", "sats", "hdop",
        ])

        print(
            f"\nStarting point locked: N={start_x:+.2f} m, E={start_y:+.2f} m, D={start_z:+.2f} m, "
            f"Heading={math.degrees(start_yaw) % 360.0:.1f} deg"
        )
        print(
            f"Target hover: {args.alt_ft:.1f} ft ({target_alt_m:.2f} m) for {hover_duration_s:.0f} s | "
            f"Max allowed drift: {args.max_drift_ft:.1f} ft ({max_drift_m:.2f} m)"
        )
        print(f"Logging real-time telemetry to: {csv_path}\n", flush=True)

        # 1. Switch to GUIDED mode
        print("Switching Pixhawk to GUIDED mode...", flush=True)
        if not conn.set_mode(MODE_GUIDED, timeout=3.0):
            print("ERROR: Pixhawk refused GUIDED mode (check 3D GPS lock and EKF position).", flush=True)
            conn.stop_streams()
            return 1

        # 2. Arm motors
        print("Arming motors...", flush=True)
        armed_ok, reasons = conn.arm(timeout=4.0)
        if not armed_ok:
            print("ERROR: Arming refused by Pixhawk:", flush=True)
            for r in reasons:
                print(f"  - {r}", flush=True)
            conn.stop_streams()
            return 1
        print("ARMED. Waiting 2.2s for ArduCopter arming delay & home reset...", flush=True)
        time.sleep(2.2)

        # Re-latch (start_x, start_y, start_z, start_yaw) after ArduCopter's arm-time home/height datum reset
        with conn.state.lock:
            start_x = conn.state.x
            start_y = conn.state.y
            start_z = conn.state.z
            start_yaw = conn.state.yaw_rad

        # 3. Command takeoff to target_alt_m (5 feet = 1.524 m)
        print(f"Commanding takeoff to {args.alt_ft:.1f} ft ({target_alt_m:.2f} m)...", flush=True)
        if not conn.command_takeoff(target_alt_m, timeout=4.5):
            abort_reason = "MAV_CMD_NAV_TAKEOFF was rejected by the flight controller"

        # 4. Climb phase + 60-second active hover loop (10 Hz)
        phase = "TAKEOFF"
        takeoff_started = time.time()
        hover_started = None
        last_target_sent = 0.0
        last_status_print = 0.0
        low_volt_since = None

        # Outer-loop integral trim (m) to cancel steady wind/CG drift from (start_x, start_y)
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

        drift_samples_ft = []
        alt_samples_ft = []
        min_volt_seen = 99.0

        while abort_reason is None:
            loop_now = time.time()
            with conn.state.lock:
                st = conn.state
                armed = st.armed
                cur_mode = st.custom_mode
                x, y, z = st.x, st.y, st.z
                vx, vy, vz = st.vx, st.vy, st.vz
                rel_alt_m = st.rel_alt_m
                roll_deg = math.degrees(st.roll_rad)
                pitch_deg = math.degrees(st.pitch_rad)
                yaw_deg = math.degrees(st.yaw_rad) % 360.0
                yaw_rad = st.yaw_rad
                volt_v = st.volt_v
                curr_a = st.curr_a
                sats = st.gps_sats
                fix = st.gps_fix
                hdop = st.gps_hdop
                ekf_flags = st.ekf_flags
                rc_chans = st.rc_chans
                lpos_age = loop_now - st.lpos_time
                att_age = loop_now - st.att_time

            if not armed:
                print("\nPixhawk disarmed.", flush=True)
                break

            # Compute real-time position & drift relative to takeoff spot
            north_err_m = x - start_x
            east_err_m = y - start_y
            drift_m = math.hypot(north_err_m, east_err_m)
            drift_ft = drift_m * M_TO_FT

            alt_local_m = -(z - start_z)
            alt_m = alt_local_m if abs(alt_local_m - rel_alt_m) < 0.8 else rel_alt_m
            alt_ft = alt_m * M_TO_FT
            vel_horiz_ms = math.hypot(vx, vy)
            min_volt_seen = min(min_volt_seen, volt_v) if volt_v > 1.0 else min_volt_seen

            # --- Check RC Switch / Stick Nudge ---
            sw_flip, sw_reason, norm_fwd, norm_right, norm_yaw, norm_climb, thr_centered_once = parse_rc_nudge(
                baseline_rc, rc_chans, thr_centered_once
            )
            if sw_flip and sw_reason:
                abort_reason = f"RC SWITCH LAND: {sw_reason}"
                break
            sticks_active = (norm_fwd != 0.0 or norm_right != 0.0 or norm_yaw != 0.0 or norm_climb != 0.0)
            if sticks_active and (phase != "TAKEOFF" or alt_m >= 0.8):
                if phase != "RC_OVERRIDE":
                    if hover_started is None:
                        hover_started = loop_now
                    override_n = north_err_m
                    override_e = east_err_m
                    override_alt_m = max(alt_m, target_alt_m)
                    override_yaw = yaw_rad
                    phase = "RC_OVERRIDE"
                    rc_moved_any = True
                    print(
                        "\n>>> RC OVERRIDE: Pilot moving sticks — steering from RC. "
                        "Release sticks for 5 s to return to launch spot & hover/land. <<<",
                        flush=True,
                    )
                rc_idle_since = None

            if cur_mode != MODE_GUIDED:
                mode_name = MODE_NAMES.get(cur_mode, str(cur_mode))
                print(f"\nFlight mode changed externally to {mode_name}; yielding control to pilot.", flush=True)
                return 0

            # --- Check Automatic Safety Failsafes ---
            if lpos_age > 1.5 or att_age > 1.5:
                abort_reason = f"Telemetry timeout (lpos_age={lpos_age:.1f}s, att_age={att_age:.1f}s)"
                break
            if abs(roll_deg) > args.max_tilt_deg or abs(pitch_deg) > args.max_tilt_deg:
                abort_reason = f"Tilt exceeded {args.max_tilt_deg:.0f} deg (roll={roll_deg:+.1f}, pitch={pitch_deg:+.1f})"
                break
            # Only enforce strict stationary drift limit when not actively steered by RC or returning from RC move
            if phase == "HOVER" and not rc_moved_any and drift_m > max_drift_m:
                abort_reason = (
                    f"Horizontal drift {drift_ft:.1f} ft ({drift_m:.2f} m) exceeded limit "
                    f"{args.max_drift_ft:.1f} ft ({max_drift_m:.2f} m)"
                )
                break
            if drift_m > max(max_drift_m, 18.0):
                abort_reason = f"Distance from launch ({drift_ft:.1f} ft) exceeded 59 ft safety radius"
                break
            if alt_m > max_alt_m:
                abort_reason = f"Altitude {alt_ft:.1f} ft ({alt_m:.2f} m) exceeded ceiling {max_alt_m * M_TO_FT:.1f} ft"
                break
            if fix < 3 or sats < 4 or hdop > 3.5 or (ekf_flags & EKF_CONST_POS_MODE):
                abort_reason = f"GPS/EKF degraded (fix={fix}, sats={sats}, hdop={hdop:.2f}, ekf=0x{ekf_flags:02x})"
                break
            if 1.0 < volt_v < DEFAULT_CRIT_VOLT_CUTOFF:
                abort_reason = f"Critical battery voltage sag ({volt_v:.2f} V)"
                break
            if 1.0 < volt_v < args.low_volt_cutoff:
                if low_volt_since is None:
                    low_volt_since = loop_now
                elif loop_now - low_volt_since >= 1.5:
                    abort_reason = f"Battery voltage below {args.low_volt_cutoff:.2f} V for 1.5s ({volt_v:.2f} V)"
                    break
            else:
                low_volt_since = None

            # --- Phase Transition: TAKEOFF -> HOVER ---
            if phase == "TAKEOFF":
                if alt_m >= target_alt_m - 0.25 or (loop_now - takeoff_started >= 8.0 and alt_m >= 0.6):
                    phase = "HOVER"
                    hover_started = loop_now
                    print(
                        f"\n>>> Reached hover altitude ({alt_ft:.1f} ft). "
                        f"Starting {hover_duration_s:.0f}-second hover & RC test (move sticks anytime; release 5s to return home)... <<<\n",
                        flush=True,
                    )
                elif loop_now - takeoff_started > 15.0:
                    abort_reason = f"Takeoff did not reach altitude within 15s (current alt={alt_ft:.1f} ft)"
                    break

            # --- Active Position Control in HOVER / RC_OVERRIDE / RETURN_HOME ---
            elapsed_s = (loop_now - hover_started) if hover_started else (loop_now - takeoff_started)
            if phase == "RC_OVERRIDE":
                if sticks_active:
                    vx_fwd = norm_fwd * 1.8
                    vy_right = norm_right * 1.8
                    vn = vx_fwd * math.cos(yaw_rad) - vy_right * math.sin(yaw_rad)
                    ve = vx_fwd * math.sin(yaw_rad) + vy_right * math.cos(yaw_rad)
                    override_n = north_err_m + vn * 0.85
                    override_e = east_err_m + ve * 0.85
                    override_yaw = wrap_pi(override_yaw + norm_yaw * math.radians(45.0) * 0.1)
                    override_alt_m = max(1.1, min(max_alt_m - 0.3, override_alt_m + norm_climb * 0.7 * 0.1))
                else:
                    if rc_idle_since is None:
                        rc_idle_since = loop_now
                        override_n = north_err_m
                        override_e = east_err_m
                        override_yaw = yaw_rad
                        print(
                            "  [RC IDLE] Sticks released — holding spot. Returning to launch point in 5.0 s...",
                            flush=True,
                        )
                    elif loop_now - rc_idle_since >= 5.0:
                        phase = "RETURN_HOME"
                        rc_idle_since = None
                        thr_centered_once = False
                        print(
                            "\n>>> RC untouched for 5 s — flying back to exact takeoff spot... <<<",
                            flush=True,
                        )
                if loop_now - last_target_sent >= 0.20:
                    conn.send_position_target_ned(
                        start_x + override_n,
                        start_y + override_e,
                        start_z - override_alt_m,
                        override_yaw,
                        use_velocity=False,
                    )
                    last_target_sent = loop_now
            elif phase == "RETURN_HOME":
                # Smoothly step back toward (0, 0) at 4 ft/s (1.22 m/s)
                if drift_m <= 0.35 and vel_horiz_ms <= 0.25:
                    rc_moved_any = False
                    trim_n_m = 0.0
                    trim_e_m = 0.0
                    if elapsed_s >= hover_duration_s:
                        print(
                            f"\n>>> Back over takeoff spot (offset {drift_ft:.2f} ft) and {hover_duration_s:.0f}s elapsed. Landing! <<<",
                            flush=True,
                        )
                        break
                    phase = "HOVER"
                    print(
                        f"\n>>> Back over takeoff spot (offset {drift_ft:.2f} ft). Resuming hover timer ({hover_duration_s - elapsed_s:.0f}s left)... <<<",
                        flush=True,
                    )
                else:
                    step_d = min(drift_m, 1.25)
                    scale = max(0.0, (drift_m - step_d) / max(0.01, drift_m))
                    cmd_n = north_err_m * scale
                    cmd_e = east_err_m * scale
                    if loop_now - last_target_sent >= 0.20:
                        conn.send_position_target_ned(
                            start_x + cmd_n,
                            start_y + cmd_e,
                            start_z - target_alt_m,
                            start_yaw,
                            use_velocity=False,
                        )
                        last_target_sent = loop_now
            elif phase == "HOVER":
                drift_samples_ft.append(drift_ft)
                alt_samples_ft.append(alt_ft)

                if elapsed_s >= hover_duration_s:
                    if drift_m > 0.40:
                        phase = "RETURN_HOME"
                        print(
                            f"\n>>> {hover_duration_s:.0f}s elapsed — centering over exact takeoff spot before landing... <<<",
                            flush=True,
                        )
                    else:
                        print(f"\n>>> Completed {hover_duration_s:.0f}-second hover test! <<<", flush=True)
                        break
                else:
                    # Update integral drift trim (10 Hz, dt = 0.1s) to counter steady drift
                    trim_n_m = max(-max_trim_m, min(max_trim_m, trim_n_m + ki_trim * north_err_m * 0.1))
                    trim_e_m = max(-max_trim_m, min(max_trim_m, trim_e_m + ki_trim * east_err_m * 0.1))

                    # Send 5 Hz SET_POSITION_TARGET_LOCAL_NED commanding drone back to (start_x, start_y, start_z - target_alt_m)
                    if loop_now - last_target_sent >= 0.20:
                        cmd_x = start_x - trim_n_m
                        cmd_y = start_y - trim_e_m
                        cmd_z = start_z - target_alt_m
                        conn.send_position_target_ned(cmd_x, cmd_y, cmd_z, start_yaw, use_velocity=False)
                        last_target_sent = loop_now

            # --- Log & Print Real-Time Telemetry ---
            writer.writerow([
                f"{loop_now:.3f}", phase, f"{elapsed_s:.2f}",
                MODE_NAMES.get(cur_mode, str(cur_mode)), int(armed),
                f"{alt_ft:.2f}", f"{args.alt_ft:.2f}",
                f"{drift_ft:.2f}", f"{north_err_m * M_TO_FT:+.2f}", f"{east_err_m * M_TO_FT:+.2f}",
                f"{trim_n_m:+.3f}", f"{trim_e_m:+.3f}",
                f"{vel_horiz_ms * M_TO_FT:.2f}", f"{-vz * M_TO_FT:+.2f}",
                f"{roll_deg:+.1f}", f"{pitch_deg:+.1f}", f"{yaw_deg:.1f}",
                f"{volt_v:.2f}", f"{curr_a:.2f}", sats, f"{hdop:.2f}",
            ])
            csv_file.flush()

            if loop_now - last_status_print >= 1.0:
                tag = f"HOVER {elapsed_s:4.1f}s/{hover_duration_s:.0f}s" if phase == "HOVER" else f"CLIMB {elapsed_s:4.1f}s"
                print(
                    f"[{tag}] alt={alt_ft:4.1f}ft (tgt {args.alt_ft:.1f}ft) | "
                    f"drift={drift_ft:4.2f}ft (N={north_err_m * M_TO_FT:+4.2f}ft, E={east_err_m * M_TO_FT:+4.2f}ft) | "
                    f"spd={vel_horiz_ms * M_TO_FT:3.1f}ft/s | tilt=({roll_deg:+4.1f},{pitch_deg:+4.1f})deg | "
                    f"batt={volt_v:5.2f}V | sats={sats}",
                    flush=True,
                )
                last_status_print = loop_now

            time.sleep(0.10)

        # 5. Land safely (whether completed 60s or triggered by failsafe / RC override)
        with conn.state.lock:
            still_armed = conn.state.armed
        if still_armed:
            execute_landing(conn, abort_reason or "60-second hover test complete")

        csv_file.close()

        # 6. Print Summary
        print("\n=== Flight1 Hover Test Summary ===")
        if drift_samples_ft:
            mean_drift = sum(drift_samples_ft) / len(drift_samples_ft)
            max_drift = max(drift_samples_ft)
            mean_alt = sum(alt_samples_ft) / len(alt_samples_ft)
            print(f"  Hover samples   : {len(drift_samples_ft)} ({len(drift_samples_ft) * 0.1:.1f} s)")
            print(f"  Mean altitude   : {mean_alt:.2f} ft (target {args.alt_ft:.2f} ft)")
            print(f"  Mean XY drift   : {mean_drift:.2f} ft ({mean_drift * FT_TO_M:.2f} m)")
            print(f"  Max XY drift    : {max_drift:.2f} ft ({max_drift * FT_TO_M:.2f} m)")
            print(f"  Min battery V   : {min_volt_seen:.2f} V")
        print(f"  Telemetry CSV   : {csv_path}")
        if abort_reason:
            print(f"  End reason      : {abort_reason}")
        return 0 if abort_reason is None else 1

    finally:
        try:
            conn.stop_streams()
        except Exception:
            pass
        conn.close()


def main() -> int:
    parser = argparse.ArgumentParser(description="Flight1 5-ft / 60-s Autonomous Hover & Position-Hold Test")
    sub = parser.add_subparsers(dest="cmd")

    p_status = sub.add_parser("status", help="Read-only pre-flight check of GPS, EKF, battery, RC, and safety switch")
    p_status.add_argument("--close-qgc", action="store_true", default=True, help="Close QGroundControl on the Pi if it holds /dev/ttyAMA0 (default: True)")

    p_wifi = sub.add_parser("wifi", help="Check or switch Wi-Fi / 4G fallback")
    p_wifi.add_argument(
        "target",
        nargs="?",
        default="auto",
    )

    p_run = sub.add_parser("run", help="Arm, take off, hold position with telemetry control, and land")
    p_run.add_argument("--alt-ft", type=float, default=5.0, help="Target hover altitude in feet (default: 5.0)")
    p_run.add_argument("--duration", type=float, default=60.0, help="Hover duration in seconds (default: 60.0)")
    p_run.add_argument("--max-drift-ft", type=float, default=6.0, help="Max allowed horizontal drift in feet before auto-land (default: 6.0)")
    p_run.add_argument("--max-tilt-deg", type=float, default=25.0, help="Max allowed roll/pitch tilt in degrees before auto-land (default: 25.0)")
    p_run.add_argument("--min-takeoff-volt", type=float, default=DEFAULT_MIN_TAKEOFF_VOLT, help=f"Minimum battery voltage required to start takeoff (default: {DEFAULT_MIN_TAKEOFF_VOLT:.1f}V)")
    p_run.add_argument("--low-volt-cutoff", type=float, default=DEFAULT_LOW_VOLT_CUTOFF, help=f"Sustained in-flight voltage that triggers auto-land (default: {DEFAULT_LOW_VOLT_CUTOFF:.1f}V)")
    p_run.add_argument("--wifi", default="auto", help="Wi-Fi / 4G check before flight")
    p_run.add_argument("--gps-wait", type=float, default=45.0, help="Seconds to wait for GPS/EKF3 home lock & RC before aborting (default: 45s)")
    p_run.add_argument("--close-qgc", action="store_true", default=True, help="Close QGroundControl if it holds /dev/ttyAMA0 (default: True)")
    p_run.add_argument("--skip-gps-wait", action="store_true", help="Do not abort on pre-flight check warning (flight controller still enforces its own checks)")

    p_land = sub.add_parser("land", help="Immediately command the Pixhawk into LAND mode and wait for disarm")
    p_land.add_argument("--close-qgc", action="store_true", default=True)

    args = parser.parse_args()
    cmd = args.cmd or "status"

    if cmd == "wifi":
        return cmd_wifi(args.target)
    if cmd == "status":
        ensure_port_free(close_qgc=getattr(args, "close_qgc", True))
        conn = MavlinkConnection()
        try:
            return print_telemetry_snapshot(conn)
        finally:
            conn.close()
    if cmd == "land":
        ensure_port_free(close_qgc=args.close_qgc)
        conn = MavlinkConnection()
        try:
            conn.configure_streams()
            execute_landing(conn, "Manual 'hover_test.py land' command")
            return 0
        finally:
            conn.stop_streams()
            conn.close()
    if cmd == "run":
        return run_hover_test(args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
