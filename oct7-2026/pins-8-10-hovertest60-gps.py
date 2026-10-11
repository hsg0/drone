#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    Hover at about 2 feet for 60 seconds using GPS.
    Commands go out on pins 8 and 10.

WHY:
    The no-GPS hover on these pins holds height with the barometer only.
    Wind can still slide the copter sideways.
    This test asks ArduPilot to take off in GUIDED, then hold the GPS
    position in LOITER for 60 seconds.

HOW:
    Pin 8 transmits. Pin 10 receives.
    USB stays with the screen. This script does not open it.
    The script waits for a 3D GPS fix, arms, climbs to 2 feet, switches
    to LOITER, counts 60 seconds, then commands LAND.

IMPORTANT:
    Props-off first. Sit the craft level, with a clear GPS view.
    Type YES before it arms. This script sends the arm command after YES.
    Compass check is skipped for this flight only, then restored.
    GPS checks stay on. Without a 3D fix the script does not arm.
    Ctrl+C starts LAND.
    Ctrl+C again force-disarms immediately.
"""

import time

from pymavlink import mavutil


# ============================================================
# Pins and flight numbers. Change these to tune the test.
# ============================================================

# Pin 8 is Pi transmit. Pin 10 is Pi receive.
SERIAL_PORT = "/dev/serial0"
BAUD = 57600

# Copter mode numbers. Do not send plane mode numbers.
GUIDED = 4
LOITER = 5
LAND = 9

TARGET_FEET = 2.0
TARGET_METRES = TARGET_FEET * 0.3048
HOLD_SECONDS = 60
TAKEOFF_TIMEOUT_SECONDS = 25
# Loiter begins a little before the exact target so the last centimetres
# of the climb cannot hold the script forever.
TAKEOFF_REACHED_FRACTION = 0.90

# 3D fix. A 2D fix cannot hold a position.
MINIMUM_GPS_FIX_TYPE = 3
MINIMUM_SATELLITE_COUNT = 6
UNKNOWN_SATELLITE_COUNT = 255

# Skip the compass check only. GPS lock stays required.
SKIP_COMPASS = 4
FORCE_DISARM = 21196


def read_new_messages(link, latest):
    """Keep the newest GPS, altitude, and heartbeat."""
    while True:
        message = link.recv_match(blocking=False)
        if message is None:
            return
        latest[message.get_type()] = message
        if message.get_type() != "STATUSTEXT":
            continue
        text = getattr(message, "text", "")
        if isinstance(text, bytes):
            text = text.decode("ascii", "ignore")
        text = str(text).strip("\x00").strip()
        if text:
            print("PIXHAWK: {}".format(text))


def ask_for_gps_and_altitude(link):
    """The command cable does not stream GPS until it is asked."""
    for name in ("GPS_RAW_INT", "GLOBAL_POSITION_INT", "HEARTBEAT", "STATUSTEXT"):
        message_id = getattr(mavutil.mavlink, "MAVLINK_MSG_ID_{}".format(name), None)
        if message_id is None:
            continue
        link.mav.command_long_send(
            link.target_system,
            link.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            250000,
            0,
            0,
            0,
            0,
            0,
        )


def parameter_name(message):
    name = message.param_id
    if isinstance(name, bytes):
        name = name.decode("ascii", "ignore")
    return name.strip("\x00").strip()


def read_parameter(link, name):
    link.mav.param_request_read_send(
        link.target_system,
        link.target_component,
        name.encode("ascii"),
        -1,
    )
    deadline = time.monotonic() + 4
    while time.monotonic() < deadline:
        message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if message is not None and parameter_name(message) == name:
            return float(message.param_value)
    return None


def write_parameter(link, name, value):
    link.mav.param_set_send(
        link.target_system,
        link.target_component,
        name.encode("ascii"),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )


def gps_is_ready(latest):
    gps_reading = latest.get("GPS_RAW_INT")
    if gps_reading is None or gps_reading.fix_type is None:
        return False
    if gps_reading.fix_type < MINIMUM_GPS_FIX_TYPE:
        return False
    satellite_count = gps_reading.satellites_visible
    if satellite_count is None or satellite_count == UNKNOWN_SATELLITE_COUNT:
        return False
    return satellite_count >= MINIMUM_SATELLITE_COUNT


def height_metres(latest):
    """Metres above home from the GPS position message. None until it arrives."""
    position = latest.get("GLOBAL_POSITION_INT")
    if position is None:
        return None
    return position.relative_alt / 1000.0


def set_mode(link, mode_number, mode_name):
    link.mav.set_mode_send(
        link.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_number,
    )
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline:
        heartbeat = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if heartbeat is None:
            continue
        if heartbeat.get_srcSystem() != link.target_system:
            continue
        if heartbeat.custom_mode == mode_number:
            print("Mode is now {}.".format(mode_name))
            return
    raise RuntimeError("Pixhawk did not enter {}.".format(mode_name))


def motors_are_armed(heartbeat):
    return bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def send_arm_or_disarm(link, arm, magic=0):
    # magic 21196 disarms even if the Pixhawk still thinks it is flying.
    link.mav.command_long_send(
        link.target_system,
        link.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        arm,
        magic,
        0,
        0,
        0,
        0,
        0,
    )


def arm_and_confirm(link):
    print("Sending ARM.")
    send_arm_or_disarm(link, 1, 0)
    deadline = time.monotonic() + 8
    result = None
    while time.monotonic() < deadline:
        message = link.recv_match(
            type=["COMMAND_ACK", "STATUSTEXT", "HEARTBEAT"],
            blocking=True,
            timeout=0.5,
        )
        if message is None:
            continue
        kind = message.get_type()
        if kind == "STATUSTEXT":
            text = getattr(message, "text", "")
            if isinstance(text, bytes):
                text = text.decode("ascii", "ignore")
            text = str(text).strip("\x00").strip()
            if text:
                print("PIXHAWK: {}".format(text))
        elif kind == "COMMAND_ACK":
            if message.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                result = message.result
                print("ARM answer = {}".format(result))
        elif kind == "HEARTBEAT" and motors_are_armed(message):
            print("ARMED.")
            return True
    print("ARM FAILED. Answer = {}".format(result))
    return False


def force_disarm(link):
    print("FORCE DISARM")
    for _unused in range(8):
        send_arm_or_disarm(link, 0, FORCE_DISARM)
        time.sleep(0.05)
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        heartbeat = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.4)
        if heartbeat is not None and not motors_are_armed(heartbeat):
            print("Motors disarmed.")
            return True
    print("Disarm was not confirmed. Pull the battery if the motors are still spinning.")
    return False


def wait_for_gps(link, latest):
    print("Waiting for a 3D GPS fix.")
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        read_new_messages(link, latest)
        if gps_is_ready(latest):
            gps_reading = latest["GPS_RAW_INT"]
            print(
                "GPS 3D fix, {} satellites.".format(gps_reading.satellites_visible)
            )
            return True
        gps_reading = latest.get("GPS_RAW_INT")
        if gps_reading is None:
            print("GPS waiting.")
        else:
            print(
                "GPS fix {} | {} satellites.".format(
                    gps_reading.fix_type,
                    gps_reading.satellites_visible,
                )
            )
        time.sleep(1.0)
    return False


def climb_to_target(link, latest):
    """ArduPilot flies the climb in GUIDED. This function watches the height."""
    minimum_metres = TARGET_METRES * TAKEOFF_REACHED_FRACTION
    print("Takeoff to {:.1f} feet.".format(TARGET_FEET))
    link.mav.command_long_send(
        link.target_system,
        link.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        TARGET_METRES,
    )
    started = time.monotonic()
    while time.monotonic() - started < TAKEOFF_TIMEOUT_SECONDS:
        read_new_messages(link, latest)
        if not gps_is_ready(latest):
            print("GPS lost during takeoff.")
            return False
        height = height_metres(latest)
        if height is None:
            print("TAKEOFF | height waiting")
        else:
            print("TAKEOFF | {:.2f} ft".format(height / 0.3048))
            if height >= minimum_metres:
                print("Takeoff height reached.")
                return True
        time.sleep(0.2)
    print("Did not reach {:.1f} feet.".format(TARGET_FEET))
    return False


def hold_loiter(link, latest):
    """Stay in LOITER until 60 seconds pass or GPS is no longer good enough."""
    print("LOITER for {} seconds.".format(HOLD_SECONDS))
    started = time.monotonic()
    while time.monotonic() - started < HOLD_SECONDS:
        read_new_messages(link, latest)
        if not gps_is_ready(latest):
            print("GPS lost during LOITER.")
            return False
        height = height_metres(latest)
        held = time.monotonic() - started
        if height is None:
            print("LOITER | height waiting | held {:.0f}s".format(held))
        else:
            print("LOITER | {:.2f} ft | held {:.0f}s".format(height / 0.3048, held))
        time.sleep(1.0)
    print("60 second hold complete.")
    return True


def main():
    print("60 second GPS hover on pins 8 and 10.")
    print("GUIDED takeoff to 2 feet, LOITER for 60 seconds, then LAND.")
    print("The screen stays on USB. GPS must already have a 3D fix.")
    print("Ctrl+C lands. Ctrl+C again disarms immediately.")
    print("Type YES to continue.")
    if input("> ").strip() != "YES":
        print("Cancelled.")
        return

    link = None
    armed = False
    saved_skip = None
    latest = {}

    try:
        print("Opening {} at {}.".format(SERIAL_PORT, BAUD))
        link = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD, source_system=255)
        heartbeat = link.wait_heartbeat(timeout=10)
        if heartbeat is None:
            raise RuntimeError("No heartbeat on pin 10.")
        if link.target_component == 0:
            link.target_component = 1
        print("Pixhawk answered on the command pins.")

        ask_for_gps_and_altitude(link)
        if not wait_for_gps(link, latest):
            raise RuntimeError("No 3D GPS fix. Not arming.")

        saved_skip = read_parameter(link, "ARMING_SKIPCHK")
        if saved_skip is None:
            raise RuntimeError("Could not read ARMING_SKIPCHK. Not arming.")
        print("ARMING_SKIPCHK = {}".format(int(round(saved_skip))))
        write_parameter(link, "ARMING_SKIPCHK", int(round(saved_skip)) | SKIP_COMPASS)
        time.sleep(0.5)

        set_mode(link, GUIDED, "GUIDED")
        if not arm_and_confirm(link):
            raise RuntimeError("Arm failed. See the PIXHAWK lines above.")
        armed = True

        if not climb_to_target(link, latest):
            set_mode(link, LAND, "LAND")
            return

        set_mode(link, LOITER, "LOITER")
        if not hold_loiter(link, latest):
            print("Hold did not finish. Landing.")
        set_mode(link, LAND, "LAND")
    except KeyboardInterrupt:
        print()
        print("CTRL+C")
        if link is not None and armed:
            try:
                set_mode(link, LAND, "LAND")
            except KeyboardInterrupt:
                print("Second Ctrl+C. Disarming now.")
            except Exception as error:
                print("Landing error: {}".format(error))
    except Exception as error:
        print("STOPPED: {}".format(error))
    finally:
        if link is not None and armed:
            print("Waiting for the motors to stop.")
            deadline = time.monotonic() + 15
            try:
                while time.monotonic() < deadline:
                    heartbeat = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.4)
                    if heartbeat is not None and not motors_are_armed(heartbeat):
                        print("Motors disarmed.")
                        armed = False
                        break
            except KeyboardInterrupt:
                print("Ctrl+C again. Disarming now.")
            if armed:
                force_disarm(link)
        if link is not None and saved_skip is not None:
            try:
                write_parameter(link, "ARMING_SKIPCHK", saved_skip)
                print("ARMING_SKIPCHK restored.")
            except Exception as error:
                print("Could not restore ARMING_SKIPCHK: {}".format(error))
        if link is not None:
            link.close()
        print("Finished.")


if __name__ == "__main__":
    main()
