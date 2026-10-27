#!/usr/bin/env bash
# Interactive Installer for Flight Path Planner & Autonomous Hover/Waypoint Runner
# Works on Raspberry Pi OS / Debian Linux with ArduCopter 4.x over MAVLink 2.

set -e

SCRIPT_SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET_SCRIPTS_DIR="$HOME/Desktop/scripts"
CONFIG_DIR="$HOME/.config/flight_path"
PLANS_DIR="$HOME/flight_plans"

echo "================================================================"
echo "  Autonomous Drone Flight Path Planner & Runner — Installer"
echo "================================================================"
echo ""
echo "Press ENTER at any prompt to accept the default value in [brackets]."
echo ""

read -r -p "1. Drone Name [Flight1]: " IN_NAME
DRONE_NAME="${IN_NAME:-Flight1}"

read -r -p "2. Pixhawk Serial Port (/dev/ttyAMA0, /dev/ttyUSB0, /dev/ttyACM0) [/dev/ttyAMA0]: " IN_PORT
SERIAL_PORT="${IN_PORT:-/dev/ttyAMA0}"

read -r -p "3. Serial Baud Rate [57600]: " IN_BAUD
SERIAL_BAUD="${IN_BAUD:-57600}"

read -r -p "4. Battery Cell Count (3 for 3S 11.1V, 4 for 4S 14.8V) [3]: " IN_CELLS
BATT_CELLS="${IN_CELLS:-3}"

if [ "$BATT_CELLS" = "4" ]; then
    DEF_TAKEOFF_V="14.4"
    DEF_LOW_V="13.8"
    DEF_CRIT_V="13.2"
else
    DEF_TAKEOFF_V="10.8"
    DEF_LOW_V="10.3"
    DEF_CRIT_V="9.9"
fi

read -r -p "5. Minimum Takeoff Voltage (V) [$DEF_TAKEOFF_V]: " IN_MIN_V
MIN_TAKEOFF_V="${IN_MIN_V:-$DEF_TAKEOFF_V}"

read -r -p "6. In-Flight Low-Battery Auto-Land Voltage (V) [$DEF_LOW_V]: " IN_LOW_V
LOW_CUTOFF_V="${IN_LOW_V:-$DEF_LOW_V}"

read -r -p "7. Critical Instant-Land Voltage (V) [$DEF_CRIT_V]: " IN_CRIT_V
CRIT_CUTOFF_V="${IN_CRIT_V:-$DEF_CRIT_V}"

read -r -p "8. Does pushing RC Pitch stick UP (forward) send HIGH PWM (~2000us)? (y/n) [y]: " IN_PITCH
if [[ "${IN_PITCH:-y}" =~ ^[Nn] ]]; then
    PITCH_HIGH="false"
else
    PITCH_HIGH="true"
fi

read -r -p "9. Seconds of RC stick release before auto-resuming path [5.0]: " IN_RESUME
RC_RESUME_S="${IN_RESUME:-5.0}"

read -r -p "10. Primary / Home Wi-Fi SSID (leave blank if none) [House-Gill-2.4]: " IN_WIFI1
WIFI_PRIMARY="${IN_WIFI1-House-Gill-2.4}"

read -r -p "11. Secondary / Field / Garage Wi-Fi SSID (leave blank if none) [G_suite_01]: " IN_WIFI2
WIFI_SECONDARY="${IN_WIFI2-G_suite_01}"

read -r -p "12. Enable 4G / LTE Cellular Fallback when Wi-Fi is out of range? (y/n) [y]: " IN_4G
if [[ "${IN_4G:-y}" =~ ^[Nn] ]]; then
    ENABLE_4G="false"
else
    ENABLE_4G="true"
fi

read -r -p "13. 4G / Cellular Interface Name (auto, wwan0, usb0, ppp0, eth1) [auto]: " IN_CELL_IF
CELL_IFACE="${IN_CELL_IF:-auto}"

read -r -p "14. Web Map Planner HTTP Port [8080]: " IN_WPORT
WEB_PORT="${IN_WPORT:-8080}"

echo ""
echo "Creating directories..."
mkdir -p "$TARGET_SCRIPTS_DIR" "$CONFIG_DIR" "$PLANS_DIR" "$HOME/Desktop"

echo "Checking Python dependencies (python3-serial)..."
if ! python3 -c "import serial" >/dev/null 2>&1; then
    echo "Installing python3-serial..."
    sudo apt-get update && sudo apt-get install -y python3-serial psmisc
fi

echo "Copying flight scripts to $TARGET_SCRIPTS_DIR ..."
if [ "$SCRIPT_SRC_DIR" != "$TARGET_SCRIPTS_DIR" ]; then
    cp -f "$SCRIPT_SRC_DIR/hover_test.py" "$TARGET_SCRIPTS_DIR/hover_test.py"
    cp -f "$SCRIPT_SRC_DIR/flight_path_planner.py" "$TARGET_SCRIPTS_DIR/flight_path_planner.py"
fi
cp -f "$TARGET_SCRIPTS_DIR/hover_test.py" "$HOME/hover_test.py"
cp -f "$TARGET_SCRIPTS_DIR/flight_path_planner.py" "$HOME/flight_path_planner.py"
chmod 644 "$TARGET_SCRIPTS_DIR/hover_test.py" "$TARGET_SCRIPTS_DIR/flight_path_planner.py"

CONFIG_JSON=$(python3 - <<PY
import json
cfg = {
    "drone_name": """$DRONE_NAME""",
    "serial_port": """$SERIAL_PORT""",
    "serial_baud": int("""$SERIAL_BAUD"""),
    "mav_sysid": 1,
    "mav_compid": 1,
    "gcs_sysid": 255,
    "gcs_compid": 190,
    "web_port": int("""$WEB_PORT"""),
    "battery_cells": int("""$BATT_CELLS"""),
    "min_takeoff_volt": float("""$MIN_TAKEOFF_V"""),
    "low_volt_cutoff": float("""$LOW_CUTOFF_V"""),
    "crit_volt_cutoff": float("""$CRIT_CUTOFF_V"""),
    "rc_pitch_forward_is_high_pwm": """$PITCH_HIGH""" == "true",
    "rc_roll_right_is_high_pwm": True,
    "rc_yaw_right_is_high_pwm": True,
    "rc_resume_idle_s": float("""$RC_RESUME_S"""),
    "primary_wifi_ssid": """$WIFI_PRIMARY""",
    "secondary_wifi_ssid": """$WIFI_SECONDARY""",
    "enable_4g_fallback": """$ENABLE_4G""" == "true",
    "cellular_interface": """$CELL_IFACE""",
}
print(json.dumps(cfg, indent=2))
PY
)

echo "$CONFIG_JSON" > "$TARGET_SCRIPTS_DIR/drone_config.json"
echo "$CONFIG_JSON" > "$CONFIG_DIR/drone_config.json"
echo "$CONFIG_JSON" > "$HOME/drone_config.json"

DESKTOP_FILE="$HOME/Desktop/Flight Path.desktop"
cat > "$DESKTOP_FILE" <<EOF
[Desktop Entry]
Type=Application
Name=Flight Path ($DRONE_NAME)
Comment=Open the path map. If the planner is already running, this only opens the browser.
Exec=python3 $TARGET_SCRIPTS_DIR/flight_path_planner.py serve --open
Terminal=false
StartupNotify=false
Categories=Utility;
EOF
chmod 644 "$DESKTOP_FILE"
cp -f "$DESKTOP_FILE" "$TARGET_SCRIPTS_DIR/flight-path.desktop"

echo ""
echo "================================================================"
echo "  Installation Complete for $DRONE_NAME!"
echo "================================================================"
echo "  Config saved to : $TARGET_SCRIPTS_DIR/drone_config.json"
echo "  Desktop icon    : $DESKTOP_FILE"
echo ""
echo "Commands:"
echo "  1. Start Web Planner UI:"
echo "     python3 ~/Desktop/scripts/flight_path_planner.py serve --open"
echo "  2. Pre-flight telemetry & network check:"
echo "     python3 ~/Desktop/scripts/hover_test.py status"
echo "  3. 8-ft / 60-s Hover & RC Stick Test:"
echo "     python3 ~/Desktop/scripts/flight_path_planner.py hover --confirm"
echo "  4. Fly a saved path plan:"
echo "     python3 ~/Desktop/scripts/flight_path_planner.py fly ~/flight_plans/yard.json --confirm"
echo "================================================================"
