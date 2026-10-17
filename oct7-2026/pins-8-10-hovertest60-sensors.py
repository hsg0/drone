#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    Hover at about 2 feet for 60 seconds without GPS.
    Height comes from whichever downward sensor is answering.
    Commands go out on pins 8 and 10.

WHY:
    This is the same hover as the October 4 no-GPS test.
    GPS is not used, so a missing GPS fix must not stop the flight.
    A downward rangefinder is a better height than the barometer when it
    is answering. When it is not, the barometer still works.

HOW:
    Pin 8 transmits. Pin 10 receives.
    USB stays with the screen. This script does not open it.
    Each loop picks one height:
        1. Pixhawk downward distance sensor, if a fresh reading arrived.
        2. Pixhawk rangefinder, if a fresh reading arrived.
        3. Barometer height above the pressure taken on the ground.
    The sonic sensor is on the power distribution board and faces forward.
    The time-of-flight sensor is under the drone and faces forward and backward.
    Both are printed when they answer. Neither is used as height.
    Climb level to 1 foot, ease up to 2 feet, hold 60 seconds, then land level.

IMPORTANT:
    Props-off first. Sit the craft level.
    Type YES before it arms. This script sends the arm command after YES.
    Compass and GPS arming checks are skipped for this flight only, then restored.
    GPS is not read and is not required.
    Ctrl+C starts the level landing.
    Ctrl+C again force-disarms immediately.
"""

import json
import math
import time

from pymavlink import mavutil


# ============================================================
# Pins and flight numbers. Change these to tune the test.
# ============================================================

# Pin 8 is Pi transmit. Pin 10 is Pi receive.
SERIAL_PORT = "/dev/serial0"
BAUD = 57600

# Copter mode numbers. Do not send plane mode numbers.
GUIDED_NOGPS = 20
LAND = 9

ONE_FOOT_METRES = 0.3048
TARGET_FEET = 2.0
TARGET_METRES = TARGET_FEET * 0.3048
HOLD_SECONDS = 60
METRES_TO_FEET = 3.28084

# Thrust here is a climb request, not a raw motor percentage.
# 0.50 = hold height.
# Higher than 0.50 = climb. Lower than 0.50 = descend.
# 0.00 = fastest descent. It does not stop the motors.
SHARP_CLIMB_THRUST = 0.72
SLOW_CLIMB_THRUST = 0.58
HOLD_THRUST = 0.50
EASE_DOWN_THRUST = 0.42
LAND_THRUST = 0.38

# Skip compass and GPS checks for this flight only. Restore them at the end.
# 4 = compass. 8 = GPS lock. 4096 = GPS configuration.
SKIP_COMPASS_AND_GPS = 4 | 8 | 4096
FORCE_DISARM = 21196

# MAV_SENSOR_ROTATION_PITCH_270. A sensor aimed at the ground.
DOWNWARD_ORIENTATION = 25
HEIGHT_SENSOR_STALE_SECONDS = 1.0

# Written by bootDisplay.py. Forward range only. Not a height.
FORWARD_RANGE_SNAPSHOT_PATH = "/tmp/redboard-forward-distance.json"
FORWARD_RANGE_STALE_SECONDS = 2.0


def read_new_messages(link, latest, arrived_at):
    """Keep the newest message of each kind, and when it arrived."""
    while True:
        message = link.recv_match(blocking=False)
        if message is None:
            return
        message_type = message.get_type()
        latest[message_type] = message
        arrived_at[message_type] = time.monotonic()
        if message_type != "STATUSTEXT":
            continue
        text = getattr(message, "text", "")
        if isinstance(text, bytes):
            text = text.decode("ascii", "ignore")
        text = str(text).strip("\x00").strip()
        if text:
            print("PIXHAWK: {}".format(text))


def message_is_fresh(arrived_at, message_type):
    arrived = arrived_at.get(message_type)
    if arrived is None:
        return False
    return time.monotonic() - arrived <= HEIGHT_SENSOR_STALE_SECONDS


def ask_for_height_messages(link):
    """Ask the command cable for every height source this test can use."""
    for name in (
        "SCALED_PRESSURE",
        "ATTITUDE",
        "DISTANCE_SENSOR",
        "RANGEFINDER",
        "HEARTBEAT",
        "STATUSTEXT",
    ):
        message_id = getattr(mavutil.mavlink, "MAVLINK_MSG_ID_{}".format(name), None)
        if message_id is None:
            continue
        link.mav.command_long_send(
            link.target_system,
            link.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            100000,
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


def pressure_to_altitude_metres(hectopascals):
    return 44330.0 * (1.0 - (hectopascals / 1013.25) ** 0.1903)


def barometer_height_metres(latest, ground_altitude):
    """Metres above the pressure reading taken on the ground."""
    pressure = latest.get("SCALED_PRESSURE")
    if pressure is None or ground_altitude is None:
        return None
    return pressure_to_altitude_metres(pressure.press_abs) - ground_altitude


def downward_distance_metres(latest, arrived_at):
    """Centimetres from a Pixhawk sensor aimed at the ground."""
    if not message_is_fresh(arrived_at, "DISTANCE_SENSOR"):
        return None
    reading = latest.get("DISTANCE_SENSOR")
    if reading is None:
        return None
    if int(getattr(reading, "orientation", -1)) != DOWNWARD_ORIENTATION:
        return None
    centimetres = getattr(reading, "current_distance", None)
    if centimetres is None or centimetres <= 0:
        return None
    return float(centimetres) / 100.0


def rangefinder_height_metres(latest, arrived_at):
    """Metres from the Pixhawk RANGEFINDER message, when it is fresh."""
    if not message_is_fresh(arrived_at, "RANGEFINDER"):
        return None
    reading = latest.get("RANGEFINDER")
    if reading is None:
        return None
    metres = getattr(reading, "distance", None)
    if metres is None or metres <= 0:
        return None
    return float(metres)


def choose_height_metres(latest, arrived_at, ground_altitude):
    """
    Use the first height source that is actually answering.
    Forward sensors are not in this list.
    """
    downward = downward_distance_metres(latest, arrived_at)
    if downward is not None:
        return downward, "downward sensor"
    rangefinder = rangefinder_height_metres(latest, arrived_at)
    if rangefinder is not None:
        return rangefinder, "rangefinder"
    barometer = barometer_height_metres(latest, ground_altitude)
    if barometer is not None:
        return barometer, "barometer"
    return None, "none"


def forward_range_note():
    """Show the power-board sonic sensor and the bottom time-of-flight sensor. Do not fly on them."""
    try:
        with open(FORWARD_RANGE_SNAPSHOT_PATH, "r", encoding="utf-8") as snapshot_file:
            snapshot = json.load(snapshot_file)
    except (OSError, json.JSONDecodeError):
        return "forward sensors quiet"
    updated_at = snapshot.get("updatedAtSeconds")
    if not isinstance(updated_at, (int, float)):
        return "forward sensors quiet"
    if time.time() - float(updated_at) > FORWARD_RANGE_STALE_SECONDS:
        return "forward sensors quiet"
    sonic = snapshot.get("sonicCentimeters")
    time_of_flight = snapshot.get("tofCentimeters")
    sonic_text = "missing" if not isinstance(sonic, (int, float)) else "{:.0f} cm".format(sonic)
    if not isinstance(time_of_flight, (int, float)):
        flight_text = "missing"
    else:
        flight_text = "{:.0f} cm".format(time_of_flight)
    return "sonic, power board, forward {} | time of flight, bottom, forward and backward {}".format(
        sonic_text,
        flight_text,
    )


def level_quaternion(latest):
    """
    Wings level. Nose stays on the current heading.
    Roll and pitch in this command are zero, so no side is told to rise first.
    """
    attitude = latest.get("ATTITUDE")
    yaw = 0.0 if attitude is None else float(attitude.yaw)
    return [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]


def send_level_command(link, latest, thrust):
    """One command: stay level, and climb or descend by the thrust number."""
    ignore_body_rates = (
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
        | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
        | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
    )
    link.mav.set_attitude_target_send(
        0,
        link.target_system,
        link.target_component,
        ignore_body_rates,
        level_quaternion(latest),
        0,
        0,
        0,
        thrust,
    )


def set_mode(link, mode_number, mode_name):
    link.mav.set_mode_send(
        link.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_number,
    )
    deadline = time.monotonic() + 5
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


def choose_thrust(height_metres):
    """
    Sharp and level until 1 foot, then a smaller climb to 2 feet.
    Every choice uses the same thrust on the whole craft. No side is favored.
    """
    if height_metres is None or height_metres < ONE_FOOT_METRES:
        return SHARP_CLIMB_THRUST, "SHARP"
    if height_metres < TARGET_METRES - 0.12:
        return SLOW_CLIMB_THRUST, "SLOW"
    if height_metres > TARGET_METRES + 0.15:
        return EASE_DOWN_THRUST, "EASE"
    return HOLD_THRUST, "HOLD"


def land_level(link, latest, arrived_at, ground_altitude):
    """Come down while still commanding wings level, then hand off to LAND."""
    print("LANDING. Still commanding level.")
    started = time.monotonic()
    while time.monotonic() - started < 25:
        read_new_messages(link, latest, arrived_at)
        height, source = choose_height_metres(latest, arrived_at, ground_altitude)
        if height is not None and height <= 0.12:
            print("Close to the ground. Height from {}.".format(source))
            break
        send_level_command(link, latest, LAND_THRUST)
        if height is None:
            print("LAND | height waiting | {}".format(forward_range_note()))
        else:
            print("LAND | {:.2f} ft | {} | {}".format(
                height * METRES_TO_FEET,
                source,
                forward_range_note(),
            ))
        time.sleep(0.1)
    set_mode(link, LAND, "LAND")


def main():
    print("60 second hover on pins 8 and 10. No GPS.")
    print("Height uses a downward sensor when one answers, otherwise the barometer.")
    print("Sharp level climb to 1 foot, slow climb to 2 feet, hold 60 seconds, level landing.")
    print("The screen stays on USB.")
    print("Ctrl+C lands level. Ctrl+C again disarms immediately.")
    print("Type YES to continue.")
    if input("> ").strip() != "YES":
        print("Cancelled.")
        return

    link = None
    armed = False
    saved_skip = None
    ground_altitude = None
    latest = {}
    arrived_at = {}

    try:
        print("Opening {} at {}.".format(SERIAL_PORT, BAUD))
        link = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD, source_system=255)
        heartbeat = link.wait_heartbeat(timeout=10)
        if heartbeat is None:
            raise RuntimeError("No heartbeat on pin 10.")
        if link.target_component == 0:
            link.target_component = 1
        print("Pixhawk answered on the command pins.")

        ask_for_height_messages(link)

        guided_options = read_parameter(link, "GUID_OPTIONS")
        if guided_options is not None and int(round(guided_options)) & 8:
            raise RuntimeError("GUID_OPTIONS bit 3 is on. Thrust would be raw throttle.")
        print("Thrust number means climb rate.")

        print("Reading the ground barometer...")
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and ground_altitude is None:
            read_new_messages(link, latest, arrived_at)
            pressure = latest.get("SCALED_PRESSURE")
            if pressure is not None:
                ground_altitude = pressure_to_altitude_metres(pressure.press_abs)
                print("Ground pressure {:.1f} hPa.".format(pressure.press_abs))
            time.sleep(0.1)
        if ground_altitude is None:
            raise RuntimeError("No barometer on the command pins. Not arming.")

        saved_skip = read_parameter(link, "ARMING_SKIPCHK")
        if saved_skip is None:
            raise RuntimeError("Could not read ARMING_SKIPCHK. Not arming.")
        print("ARMING_SKIPCHK = {}".format(int(round(saved_skip))))
        write_parameter(link, "ARMING_SKIPCHK", int(round(saved_skip)) | SKIP_COMPASS_AND_GPS)
        time.sleep(0.5)

        set_mode(link, GUIDED_NOGPS, "GUIDED_NOGPS")
        if not arm_and_confirm(link):
            raise RuntimeError("Arm failed. See the PIXHAWK lines above.")
        armed = True

        print("Climbing. GPS is not used.")
        flight_started = time.monotonic()
        hold_started = None
        while True:
            read_new_messages(link, latest, arrived_at)
            height, source = choose_height_metres(latest, arrived_at, ground_altitude)
            thrust, phase = choose_thrust(height)
            send_level_command(link, latest, thrust)

            if phase == "HOLD":
                if hold_started is None:
                    hold_started = time.monotonic()
                    print("At about 2 feet using {}. Starting the 60 second hold.".format(source))
                elif time.monotonic() - hold_started >= HOLD_SECONDS:
                    print("60 second hold complete.")
                    break
            elif hold_started is not None and height is not None:
                if abs(height - TARGET_METRES) > 0.40:
                    print("Left the 2 foot band. Landing.")
                    break

            if hold_started is None and time.monotonic() - flight_started > 25:
                print("Did not reach 2 feet. Landing.")
                break

            held = 0 if hold_started is None else time.monotonic() - hold_started
            if height is None:
                print("{} | height waiting | {}".format(phase, forward_range_note()))
            else:
                print("{} | {:.2f} ft | {} | thrust {:.2f} | held {:.0f}s | {}".format(
                    phase,
                    height * METRES_TO_FEET,
                    source,
                    thrust,
                    held,
                    forward_range_note(),
                ))
            time.sleep(0.1)

        land_level(link, latest, arrived_at, ground_altitude)
    except KeyboardInterrupt:
        print()
        print("CTRL+C")
        if link is not None and armed:
            try:
                land_level(link, latest, arrived_at, ground_altitude)
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
