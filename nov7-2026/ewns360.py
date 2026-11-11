#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    Take off to 5 feet, hover 60 seconds, fly north 10 feet, turn 360
    clockwise, hover 20 seconds, fly south 10 feet, hover 20 seconds,
    turn 360, fly east 10 feet, hover 20 seconds, turn 360, fly west
    10 feet, hover 20 seconds, turn 360, hover 20 seconds, then land
    until the propellers stop.

WHY:
    This is the Nov 7 pattern. The copter should get into the air and
    fly the legs instead of stopping on extra checks.

HOW:
    1. Log every line in nov7-2026/flightLog/.
    2. Connect on /dev/serial0, pins 8 and 10, at 57600.
    3. A false "battery 1 low voltage" reading does not block arm. Type YES.
    4. Arm when the Pixhawk accepts it. GUIDED takeoff to 5 feet.
    5. LOITER 60 seconds.
    6. North 10 feet, clockwise 360, LOITER 20 seconds.
    7. South 10 feet, LOITER 20 seconds, clockwise 360.
    8. East 10 feet, LOITER 20 seconds, clockwise 360.
    9. West 10 feet, LOITER 20 seconds, clockwise 360, LOITER 20 seconds.
    10. LAND until the motors disarm.

IMPORTANT:
    Run this file on the Pi. Do not put python3 in front of the path.
    Leave USB to bootDisplay.py. This script never force-arms and never
    force-disarms. A slow turn or a short leg does not cancel the rest
    of the flight. The Pixhawk still refuses to arm until its own checks pass.
"""

import math
import os
import sys
import time

from dronekit import VehicleMode
from dronekit import connect
from pymavlink import mavutil as middleMan


# --------------------------------------------------
# THIS COPTER
# --------------------------------------------------

PIXHAWK_PORT = "/dev/serial0"
PIXHAWK_BAUD = 57600
CONNECTION_TIMEOUT_SECONDS = 60
GROUND_STATION_SYSTEM_ID = 255

FLIGHT_LOG_DIRECTORY = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "flightLog",
)

FEET_PER_METER = 3.28084
HOVER_ALTITUDE_FEET = 5.0
HOVER_ALTITUDE_METERS = HOVER_ALTITUDE_FEET / FEET_PER_METER
TAKEOFF_REACHED_FRACTION = 0.95
TAKEOFF_SETTLE_SECONDS = 2.0
FIRST_HOVER_SECONDS = 60.0
LEG_HOVER_SECONDS = 20.0
FINAL_HOVER_SECONDS = 20.0

LEG_DISTANCE_FEET = 10.0
LEG_DISTANCE_METERS = LEG_DISTANCE_FEET / FEET_PER_METER
LEG_SPEED_METERS_PER_SECOND = 0.4
ALTITUDE_HOLD_GAIN = 0.5
MAXIMUM_VERTICAL_SPEED_METERS_PER_SECOND = 0.25
# 10 feet at 0.4 m/s is about 8 seconds. The extra time lets a slow leg finish.
LEG_TIMEOUT_SECONDS = 30.0
SETTLE_AFTER_MOVE_SECONDS = 2.0
METERS_PER_DEGREE_OF_LATITUDE = 111319.5
MINIMUM_VALID_COORDINATE_DEGREES = 0.01

CLOCKWISE_TURN_DEGREES = 360.0
YAW_RATE_DEGREES_PER_SECOND = 20.0
HEADING_TOLERANCE_DEGREES = 15.0
FULL_TURN_TIMEOUT_SECONDS = 25.0

TAKEOFF_TIMEOUT_SECONDS = 40.0
LAND_TIMEOUT_SECONDS = 60.0
MODE_CHANGE_TIMEOUT_SECONDS = 10.0
HEARTBEAT_STALE_AFTER_SECONDS = 3.0

STATUS_CHECK_INTERVAL_SECONDS = 0.2
PRINT_INTERVAL_SECONDS = 1.0
PARAMETER_TOLERANCE = 0.01

TAKEOFF_SLEW_PARAMETER_NAME = "TKOFF_SLEW_TIME"
PREFERRED_TAKEOFF_SLEW_SECONDS = 1.0
GROUND_STATION_FAILSAFE_PARAMETER_NAME = "FS_GCS_ENABLE"
GROUND_STATION_FAILSAFE_TIMEOUT_PARAMETER_NAME = "FS_GCS_TIMEOUT"
FAILSAFE_OPTIONS_PARAMETER_NAME = "FS_OPTIONS"
GROUND_STATION_FAILSAFE_LAND_VALUE = 5
CONTINUE_PILOT_MODE_ON_FAILSAFE_BIT = 16
PREFERRED_FAILSAFE_TIMEOUT_SECONDS = 5.0
FALSE_BATTERY_WARNING = "battery 1 low voltage"
BATTERY_LOW_FAILSAFE_ACTION_PARAMETER = "BATT_FS_LOW_ACT"
BATTERY_CRITICAL_FAILSAFE_ACTION_PARAMETER = "BATT_FS_CRT_ACT"
BATTERY_FAILSAFE_ACTION_NONE = 0

MESSAGE_INTERVAL_SECONDS = 0.1
MICROSECONDS_PER_SECOND = 1000000
FLIGHT_MESSAGE_NAMES = (
    "GPS_RAW_INT",
    "GLOBAL_POSITION_INT",
    "ATTITUDE",
    "STATUSTEXT",
)

# North is +X and east is +Y in the Pixhawk's local North-East-Down frame.
DIRECTION_SPEEDS = {
    "north": (LEG_SPEED_METERS_PER_SECOND, 0.0),
    "south": (-LEG_SPEED_METERS_PER_SECOND, 0.0),
    "east": (0.0, LEG_SPEED_METERS_PER_SECOND),
    "west": (0.0, -LEG_SPEED_METERS_PER_SECOND),
}

VELOCITY_ONLY_TYPE_MASK = (
    middleMan.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
    | middleMan.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
    | middleMan.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
    | middleMan.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | middleMan.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | middleMan.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | middleMan.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | middleMan.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

# CONDITION_YAW direction 1 is clockwise as seen from above.
YAW_CLOCKWISE = 1
YAW_RELATIVE = 1


class FlightLog:
    """Copy every printed line to the terminal and to the flight log file."""

    def __init__(self, logFile, terminal):
        self.logFile = logFile
        self.terminal = terminal

    def write(self, text):
        self.terminal.write(text)
        self.terminal.flush()
        self.logFile.write(text)
        self.logFile.flush()

    def writeLogOnly(self, text):
        self.logFile.write(text)
        self.logFile.flush()

    def flush(self):
        self.terminal.flush()
        self.logFile.flush()


def heartbeatIsFresh(vehicle):
    heartbeatAgeSeconds = vehicle.last_heartbeat
    if heartbeatAgeSeconds is None:
        return False
    return heartbeatAgeSeconds <= HEARTBEAT_STALE_AFTER_SECONDS


def headingDegrees(vehicle):
    if vehicle.attitude is None or vehicle.attitude.yaw is None:
        return None
    return math.degrees(float(vehicle.attitude.yaw))


def smallestHeadingDifferenceDegrees(fromDegrees, toDegrees):
    return (float(toDegrees) - float(fromDegrees) + 180.0) % 360.0 - 180.0


def heightMeters(vehicle):
    location = vehicle.location.global_relative_frame
    if location is None or location.alt is None:
        return None
    altitudeMeters = float(location.alt)
    if not math.isfinite(altitudeMeters):
        return None
    return altitudeMeters


def heightText(altitudeMeters):
    if altitudeMeters is None:
        return "height waiting"
    return f"{altitudeMeters:.2f} m ({altitudeMeters * FEET_PER_METER:.2f} ft)"


def latitudeAndLongitude(vehicle):
    location = vehicle.location.global_relative_frame
    if location is None or location.lat is None or location.lon is None:
        return None
    latitudeDegrees = float(location.lat)
    longitudeDegrees = float(location.lon)
    if not math.isfinite(latitudeDegrees) or not math.isfinite(longitudeDegrees):
        return None
    if (
        abs(latitudeDegrees) < MINIMUM_VALID_COORDINATE_DEGREES
        and abs(longitudeDegrees) < MINIMUM_VALID_COORDINATE_DEGREES
    ):
        return None
    return latitudeDegrees, longitudeDegrees


def northAndEastOffsetMeters(startLatitudeDegrees, startLongitudeDegrees, currentLatitudeDegrees, currentLongitudeDegrees):
    northMeters = (currentLatitudeDegrees - startLatitudeDegrees) * METERS_PER_DEGREE_OF_LATITUDE
    eastMeters = (
        (currentLongitudeDegrees - startLongitudeDegrees)
        * METERS_PER_DEGREE_OF_LATITUDE
        * math.cos(math.radians(startLatitudeDegrees))
    )
    return northMeters, eastMeters


def distanceAlongLegMeters(directionName, northMeters, eastMeters):
    if directionName == "north":
        return northMeters
    if directionName == "south":
        return -northMeters
    if directionName == "east":
        return eastMeters
    if directionName == "west":
        return -eastMeters
    raise RuntimeError(f"Unknown direction {directionName}.")


def setFlightMode(vehicle, flightModeName):
    print(f"Requesting {flightModeName}.")
    vehicle.mode = VehicleMode(flightModeName)
    startTime = time.monotonic()
    while time.monotonic() - startTime < MODE_CHANGE_TIMEOUT_SECONDS:
        if vehicle.mode.name == flightModeName:
            print(f"Mode confirmed: {flightModeName}")
            return
        if not heartbeatIsFresh(vehicle):
            raise RuntimeError("Pixhawk heartbeat was lost.")
        time.sleep(STATUS_CHECK_INTERVAL_SECONDS)
    raise RuntimeError(f"Could not enter {flightModeName}.")


def setParameterAndConfirm(vehicle, parameterName, wantedValue):
    currentValue = vehicle.parameters[parameterName]
    if currentValue is not None and abs(float(currentValue) - float(wantedValue)) < PARAMETER_TOLERANCE:
        return float(currentValue)
    wasWritten = vehicle.parameters.set(parameterName, wantedValue)
    confirmedValue = vehicle.parameters[parameterName]
    if (
        not wasWritten
        or confirmedValue is None
        or abs(float(confirmedValue) - float(wantedValue)) >= PARAMETER_TOLERANCE
    ):
        raise RuntimeError(f"{parameterName} did not stay at {float(wantedValue):.1f}.")
    return float(confirmedValue)


def sendLocalVelocity(vehicle, northMetersPerSecond, eastMetersPerSecond, downMetersPerSecond):
    message = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        vehicle._master.target_system,
        vehicle._master.target_component,
        middleMan.mavlink.MAV_FRAME_LOCAL_NED,
        VELOCITY_ONLY_TYPE_MASK,
        0, 0, 0,
        northMetersPerSecond,
        eastMetersPerSecond,
        downMetersPerSecond,
        0, 0, 0,
        0, 0,
    )
    vehicle.send_mavlink(message)
    vehicle.flush()


def downSpeedToHoldAltitude(altitudeMeters):
    if altitudeMeters is None:
        return 0.0
    altitudeErrorMeters = HOVER_ALTITUDE_METERS - altitudeMeters
    climbRateMetersPerSecond = max(
        -MAXIMUM_VERTICAL_SPEED_METERS_PER_SECOND,
        min(
            MAXIMUM_VERTICAL_SPEED_METERS_PER_SECOND,
            altitudeErrorMeters * ALTITUDE_HOLD_GAIN,
        ),
    )
    return -climbRateMetersPerSecond


def checkFlightHealth(vehicle, flightOutcome):
    if not heartbeatIsFresh(vehicle):
        raise RuntimeError("Pixhawk heartbeat was lost.")
    if not vehicle.armed:
        raise RuntimeError("Copter disarmed unexpectedly in flight.")
    altitudeMeters = heightMeters(vehicle)
    if altitudeMeters is not None:
        previousMaximum = flightOutcome.get("maximumAltitudeMeters")
        if previousMaximum is None or altitudeMeters > previousMaximum:
            flightOutcome["maximumAltitudeMeters"] = altitudeMeters
            flightOutcome["maximumAltitudeReachedText"] = heightText(altitudeMeters)
    return altitudeMeters


def commandYaw(vehicle, angleDegrees):
    message = vehicle.message_factory.command_long_encode(
        vehicle._master.target_system,
        vehicle._master.target_component,
        middleMan.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        float(angleDegrees),
        YAW_RATE_DEGREES_PER_SECOND,
        float(YAW_CLOCKWISE),
        float(YAW_RELATIVE),
        0, 0, 0,
    )
    vehicle.send_mavlink(message)
    vehicle.flush()


def hoverInLoiter(vehicle, flightOutcome, hoverSeconds, hoverName):
    setFlightMode(vehicle, "LOITER")
    print(f"{hoverName} started for {hoverSeconds:.0f} seconds at {HOVER_ALTITUDE_FEET:.0f} ft.")
    startedAtSeconds = time.monotonic()
    lastPrintedAtSeconds = 0.0
    while True:
        elapsedSeconds = time.monotonic() - startedAtSeconds
        if elapsedSeconds >= hoverSeconds:
            break
        if vehicle.mode.name != "LOITER":
            raise RuntimeError(f"LOITER mode lost during {hoverName}.")
        altitudeMeters = checkFlightHealth(vehicle, flightOutcome)
        nowSeconds = time.monotonic()
        if nowSeconds - lastPrintedAtSeconds >= PRINT_INTERVAL_SECONDS:
            remainingSeconds = max(0.0, hoverSeconds - elapsedSeconds)
            print(f"{hoverName} | {heightText(altitudeMeters)} | remaining {remainingSeconds:.0f} s")
            lastPrintedAtSeconds = nowSeconds
        time.sleep(STATUS_CHECK_INTERVAL_SECONDS)
    print(f"{hoverName} completed.")


def turnClockwise360(vehicle, flightOutcome, turnName):
    """Turn in place. A slow turn continues the flight instead of landing."""
    setFlightMode(vehicle, "GUIDED")
    startHeading = headingDegrees(vehicle)
    if startHeading is None:
        print(f"{turnName}: heading was not available. Continuing the flight.")
        return
    print(
        f"{turnName}: {CLOCKWISE_TURN_DEGREES:.0f} degrees clockwise "
        f"at {YAW_RATE_DEGREES_PER_SECOND:.0f} deg/s."
    )
    commandYaw(vehicle, CLOCKWISE_TURN_DEGREES)
    previousHeading = startHeading
    turnedDegrees = 0.0
    startedAtSeconds = time.monotonic()
    lastPrintedAtSeconds = 0.0
    while True:
        altitudeMeters = checkFlightHealth(vehicle, flightOutcome)
        if vehicle.mode.name != "GUIDED":
            raise RuntimeError(f"GUIDED mode lost during {turnName}.")
        sendLocalVelocity(vehicle, 0.0, 0.0, downSpeedToHoldAltitude(altitudeMeters))
        currentHeading = headingDegrees(vehicle)
        if currentHeading is not None:
            stepDegrees = smallestHeadingDifferenceDegrees(previousHeading, currentHeading)
            if stepDegrees > 0:
                turnedDegrees += stepDegrees
            previousHeading = currentHeading
        remainingDegrees = max(30.0, CLOCKWISE_TURN_DEGREES - turnedDegrees)
        commandYaw(vehicle, remainingDegrees)
        nowSeconds = time.monotonic()
        if nowSeconds - lastPrintedAtSeconds >= PRINT_INTERVAL_SECONDS:
            print(f"{turnName} | turned {turnedDegrees:+.0f} deg")
            lastPrintedAtSeconds = nowSeconds
        if turnedDegrees >= CLOCKWISE_TURN_DEGREES - HEADING_TOLERANCE_DEGREES:
            print(f"{turnName} completed.")
            return
        if nowSeconds - startedAtSeconds > FULL_TURN_TIMEOUT_SECONDS:
            print(f"{turnName} time reached at {turnedDegrees:+.0f} deg. Continuing the flight.")
            return
        time.sleep(STATUS_CHECK_INTERVAL_SECONDS)


def flyTenFeet(vehicle, flightOutcome, directionName):
    """Fly one 10-foot leg. A short reading continues the flight instead of landing."""
    northSpeed, eastSpeed = DIRECTION_SPEEDS[directionName]
    setFlightMode(vehicle, "GUIDED")
    startSpot = latitudeAndLongitude(vehicle)
    print(
        f"Flying {directionName} {LEG_DISTANCE_FEET:.0f} ft "
        f"at {LEG_SPEED_METERS_PER_SECOND:.1f} m/s."
    )
    startedAtSeconds = time.monotonic()
    lastPrintedAtSeconds = 0.0
    try:
        while True:
            if vehicle.mode.name != "GUIDED":
                raise RuntimeError(f"GUIDED mode lost during the {directionName} leg.")
            altitudeMeters = checkFlightHealth(vehicle, flightOutcome)
            progressFeet = 0.0
            currentSpot = latitudeAndLongitude(vehicle)
            if startSpot is None:
                startSpot = currentSpot
            if startSpot is not None and currentSpot is not None:
                northMeters, eastMeters = northAndEastOffsetMeters(
                    startSpot[0],
                    startSpot[1],
                    currentSpot[0],
                    currentSpot[1],
                )
                progressMeters = distanceAlongLegMeters(directionName, northMeters, eastMeters)
                progressFeet = progressMeters * FEET_PER_METER
                if progressMeters >= LEG_DISTANCE_METERS:
                    print(f"10 feet {directionName} reached | {progressFeet:.1f} ft | {heightText(altitudeMeters)}")
                    break
            if time.monotonic() - startedAtSeconds > LEG_TIMEOUT_SECONDS:
                print(f"{directionName} leg time reached at {progressFeet:.1f} ft. Continuing the flight.")
                break
            sendLocalVelocity(
                vehicle,
                northSpeed,
                eastSpeed,
                downSpeedToHoldAltitude(altitudeMeters),
            )
            nowSeconds = time.monotonic()
            if nowSeconds - lastPrintedAtSeconds >= PRINT_INTERVAL_SECONDS:
                print(f"Flying {directionName} | {progressFeet:.1f}/{LEG_DISTANCE_FEET:.0f} ft | {heightText(altitudeMeters)}")
                lastPrintedAtSeconds = nowSeconds
            time.sleep(STATUS_CHECK_INTERVAL_SECONDS)

        settleStartedAtSeconds = time.monotonic()
        while time.monotonic() - settleStartedAtSeconds < SETTLE_AFTER_MOVE_SECONDS:
            altitudeMeters = checkFlightHealth(vehicle, flightOutcome)
            sendLocalVelocity(vehicle, 0.0, 0.0, downSpeedToHoldAltitude(altitudeMeters))
            time.sleep(STATUS_CHECK_INTERVAL_SECONDS)
        print(f"{directionName} leg stopped.")
    finally:
        if heartbeatIsFresh(vehicle) and not getattr(vehicle, "commandCableClosed", False):
            try:
                sendLocalVelocity(vehicle, 0.0, 0.0, 0.0)
            except Exception as stopError:
                print(f"Could not send the stop speed: {stopError}")


def landUntilPropellersStop(vehicle, flightOutcome):
    if not vehicle.armed:
        print("Motors already disarmed. Propellers are stopped.")
        flightOutcome["landing"] = "SUCCESS (Motors already disarmed)"
        return
    if not heartbeatIsFresh(vehicle):
        print("Heartbeat is lost. Closing the cable so the Pixhawk lands itself.")
        closeCommandCable(vehicle)
        flightOutcome["landing"] = "ONBOARD FAILSAFE LANDING"
        return
    print("Landing.")
    setFlightMode(vehicle, "LAND")
    startedAtSeconds = time.monotonic()
    while vehicle.armed:
        if not heartbeatIsFresh(vehicle):
            print("Heartbeat lost during landing. Closing the cable so the Pixhawk lands itself.")
            closeCommandCable(vehicle)
            flightOutcome["landing"] = "ONBOARD FAILSAFE DURING LANDING"
            return
        if time.monotonic() - startedAtSeconds > LAND_TIMEOUT_SECONDS:
            raise RuntimeError("Landing not confirmed. The motors are still armed.")
        print(f"Waiting for the propellers to stop. {heightText(heightMeters(vehicle))}")
        time.sleep(PRINT_INTERVAL_SECONDS)
    print("Landed. Motors disarmed and propellers stopped.")
    flightOutcome["landing"] = "SUCCESS (Landed, motors disarmed)"


def closeCommandCable(vehicle):
    if vehicle is None or getattr(vehicle, "commandCableClosed", False):
        return
    vehicle.commandCableClosed = True
    try:
        vehicle.close()
        print("Command cable closed.")
    except Exception as error:
        print(f"Could not close the command cable: {error}")


def pixhawkBatteryVoltageText(vehicle):
    battery = getattr(vehicle, "battery", None)
    voltage = getattr(battery, "voltage", None) if battery is not None else None
    if voltage is None or not math.isfinite(float(voltage)) or float(voltage) < 0:
        return "Pixhawk battery voltage was not reported."
    return f"Pixhawk battery voltage reads {float(voltage):.2f} volts."


def rememberPixhawkText(vehicle):
    """Keep the latest pre-arm sentence so a false battery warning can be cleared."""
    vehicle.preflightNotes = {
        "lastPrearmReason": None,
        "sawFalseBatteryWarning": False,
        "falseBatteryFailsafeIgnored": False,
    }

    @vehicle.on_message("STATUSTEXT")
    def onStatusText(_vehicle, _name, message):
        text = getattr(message, "text", "")
        if isinstance(text, bytes):
            text = text.decode("ascii", errors="ignore")
        text = str(text).strip("\x00").strip()
        if not text:
            return
        print(f"Pixhawk: {text}")
        if "prearm" in text.lower():
            vehicle.preflightNotes["lastPrearmReason"] = text
        if FALSE_BATTERY_WARNING in text.lower():
            vehicle.preflightNotes["sawFalseBatteryWarning"] = True


def releaseFalseBatteryFailsafe(vehicle):
    notes = getattr(vehicle, "preflightNotes", None)
    if notes is None or notes.get("falseBatteryFailsafeIgnored"):
        return
    reasonText = (notes.get("lastPrearmReason") or "").lower()
    if FALSE_BATTERY_WARNING not in reasonText and not notes.get("sawFalseBatteryWarning"):
        return
    print(pixhawkBatteryVoltageText(vehicle))
    print("Battery 1 low voltage failsafe is a false reading. Disabling that failsafe for this flight.")
    setParameterAndConfirm(vehicle, BATTERY_LOW_FAILSAFE_ACTION_PARAMETER, BATTERY_FAILSAFE_ACTION_NONE)
    setParameterAndConfirm(vehicle, BATTERY_CRITICAL_FAILSAFE_ACTION_PARAMETER, BATTERY_FAILSAFE_ACTION_NONE)
    notes["falseBatteryFailsafeIgnored"] = True
    notes["lastPrearmReason"] = None
    notes["sawFalseBatteryWarning"] = False


def requestPixhawkMessages(vehicle):
    intervalMicroseconds = int(MESSAGE_INTERVAL_SECONDS * MICROSECONDS_PER_SECOND)
    for messageName in FLIGHT_MESSAGE_NAMES:
        messageId = getattr(middleMan.mavlink, f"MAVLINK_MSG_ID_{messageName}", None)
        if messageId is None:
            continue
        commandMessage = vehicle.message_factory.command_long_encode(
            vehicle._master.target_system,
            vehicle._master.target_component,
            middleMan.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            messageId,
            intervalMicroseconds,
            0, 0, 0, 0, 0,
        )
        vehicle.send_mavlink(commandMessage)
    vehicle.flush()


def openPixhawk():
    vehicle = connect(
        PIXHAWK_PORT,
        baud=PIXHAWK_BAUD,
        wait_ready=True,
        timeout=CONNECTION_TIMEOUT_SECONDS,
        source_system=GROUND_STATION_SYSTEM_ID,
    )
    if vehicle._master.target_component == 0:
        vehicle._master.target_component = 1
    rememberPixhawkText(vehicle)
    requestPixhawkMessages(vehicle)
    return vehicle


def takeOffToFiveFeet(vehicle, flightOutcome):
    setFlightMode(vehicle, "GUIDED")
    print("Arming motors.")
    print(pixhawkBatteryVoltageText(vehicle))
    lastPrintedAtSeconds = 0.0
    while not vehicle.armed:
        if not heartbeatIsFresh(vehicle):
            raise RuntimeError("Pixhawk heartbeat was lost while arming.")
        releaseFalseBatteryFailsafe(vehicle)
        vehicle.armed = True
        nowSeconds = time.monotonic()
        if nowSeconds - lastPrintedAtSeconds >= PRINT_INTERVAL_SECONDS:
            reason = getattr(vehicle, "preflightNotes", {}).get("lastPrearmReason")
            if reason:
                print(f"Waiting for the Pixhawk to arm. {reason}")
            else:
                print("Waiting for the Pixhawk to arm.")
            lastPrintedAtSeconds = nowSeconds
        time.sleep(STATUS_CHECK_INTERVAL_SECONDS)
    print("Motors armed. Propellers spinning.")
    print(f"Taking off to {HOVER_ALTITUDE_FEET:.0f} feet.")
    vehicle.simple_takeoff(HOVER_ALTITUDE_METERS)
    reachedMeters = HOVER_ALTITUDE_METERS * TAKEOFF_REACHED_FRACTION
    startedAtSeconds = time.monotonic()
    lastPrintedAtSeconds = 0.0
    while True:
        if vehicle.mode.name != "GUIDED":
            raise RuntimeError("GUIDED mode lost during takeoff.")
        altitudeMeters = checkFlightHealth(vehicle, flightOutcome)
        nowSeconds = time.monotonic()
        if nowSeconds - lastPrintedAtSeconds >= PRINT_INTERVAL_SECONDS:
            print(f"Climbing | {heightText(altitudeMeters)}")
            lastPrintedAtSeconds = nowSeconds
        if altitudeMeters is not None and altitudeMeters >= reachedMeters:
            print(f"Reached 5 feet: {heightText(altitudeMeters)}.")
            break
        if nowSeconds - startedAtSeconds > TAKEOFF_TIMEOUT_SECONDS:
            print(f"Takeoff time reached at {heightText(altitudeMeters)}. Continuing the flight.")
            break
        time.sleep(STATUS_CHECK_INTERVAL_SECONDS)
    print(f"Settling at 5 feet for {TAKEOFF_SETTLE_SECONDS:.0f} seconds.")
    settleStartedAtSeconds = time.monotonic()
    while time.monotonic() - settleStartedAtSeconds < TAKEOFF_SETTLE_SECONDS:
        checkFlightHealth(vehicle, flightOutcome)
        time.sleep(STATUS_CHECK_INTERVAL_SECONDS)


def flyThePattern(vehicle, flightOutcome):
    hoverInLoiter(vehicle, flightOutcome, FIRST_HOVER_SECONDS, "First hover")
    flyTenFeet(vehicle, flightOutcome, "north")
    turnClockwise360(vehicle, flightOutcome, "360 after north")
    hoverInLoiter(vehicle, flightOutcome, LEG_HOVER_SECONDS, "Hover after the north turn")
    flyTenFeet(vehicle, flightOutcome, "south")
    hoverInLoiter(vehicle, flightOutcome, LEG_HOVER_SECONDS, "Hover after south")
    turnClockwise360(vehicle, flightOutcome, "360 after south")
    flyTenFeet(vehicle, flightOutcome, "east")
    hoverInLoiter(vehicle, flightOutcome, LEG_HOVER_SECONDS, "Hover after east")
    turnClockwise360(vehicle, flightOutcome, "360 after the east hover")
    flyTenFeet(vehicle, flightOutcome, "west")
    hoverInLoiter(vehicle, flightOutcome, LEG_HOVER_SECONDS, "Hover after west")
    turnClockwise360(vehicle, flightOutcome, "360 after the west hover")
    hoverInLoiter(vehicle, flightOutcome, FINAL_HOVER_SECONDS, "Hover before landing")


def main():
    originalStandardOutput = sys.stdout
    logFile = None
    vehicle = None
    originalTakeoffSlewSeconds = None
    savedFailsafe = None
    copterWasArmed = False
    landingWasStarted = False
    flightOutcome = {
        "overallOutcome": "FAILED",
        "stopReason": None,
        "maximumAltitudeReachedText": "none",
        "landing": "NOT STARTED",
    }

    try:
        os.makedirs(FLIGHT_LOG_DIRECTORY, exist_ok=True)
        logPath = os.path.join(
            FLIGHT_LOG_DIRECTORY,
            f"ewns360-{time.strftime('%Y-%m-%d-%H%M%S')}.log",
        )
        logFile = open(logPath, "w", encoding="utf-8")
        sys.stdout = FlightLog(logFile, sys.stdout)
        print(f"Flight log file : {logPath}")
        print(f"Run started at  : {time.strftime('%Y-%m-%d %H:%M:%S %Z')}")
        print("Take off to 5 ft, hover 60 s, north 10 ft, 360, hover 20 s,")
        print("south 10 ft, hover 20 s, 360, east 10 ft, hover 20 s, 360,")
        print("west 10 ft, hover 20 s, 360, hover 20 s, land.")

        if not os.path.exists(PIXHAWK_PORT):
            raise RuntimeError(f"{PIXHAWK_PORT} is missing. Run this script on the Pi.")
        print(f"Connecting to Pixhawk on {PIXHAWK_PORT} at {PIXHAWK_BAUD} baud.")
        vehicle = openPixhawk()
        print("Pixhawk connected.")

        promptText = (
            "Type YES to take off to 5 ft, hover 60 s, fly north, south, east, "
            "and west with a 360 and a 20 s hover on each part, then land: "
        )
        confirmation = input(promptText)
        if isinstance(sys.stdout, FlightLog):
            sys.stdout.writeLogOnly(f"{promptText}{confirmation}\n")
        if confirmation.strip() != "YES":
            print("Cancelled. The copter was not armed.")
            flightOutcome["overallOutcome"] = "CANCELLED BEFORE ARMING"
            flightOutcome["stopReason"] = "Pilot did not type YES."
            return

        savedSlew = vehicle.parameters[TAKEOFF_SLEW_PARAMETER_NAME]
        if savedSlew is not None:
            savedSlew = float(savedSlew)
            takeoffSlew = min(savedSlew, PREFERRED_TAKEOFF_SLEW_SECONDS)
            if abs(takeoffSlew - savedSlew) >= PARAMETER_TOLERANCE:
                setParameterAndConfirm(vehicle, TAKEOFF_SLEW_PARAMETER_NAME, takeoffSlew)
                originalTakeoffSlewSeconds = savedSlew

        savedEnable = float(vehicle.parameters[GROUND_STATION_FAILSAFE_PARAMETER_NAME])
        savedTimeout = float(vehicle.parameters[GROUND_STATION_FAILSAFE_TIMEOUT_PARAMETER_NAME])
        savedOptions = float(vehicle.parameters[FAILSAFE_OPTIONS_PARAMETER_NAME])
        timeoutSeconds = savedTimeout if 0 < savedTimeout <= PREFERRED_FAILSAFE_TIMEOUT_SECONDS else PREFERRED_FAILSAFE_TIMEOUT_SECONDS
        flightOptions = int(round(savedOptions)) & ~CONTINUE_PILOT_MODE_ON_FAILSAFE_BIT
        savedFailsafe = (savedEnable, savedTimeout, savedOptions)
        setParameterAndConfirm(vehicle, GROUND_STATION_FAILSAFE_PARAMETER_NAME, GROUND_STATION_FAILSAFE_LAND_VALUE)
        setParameterAndConfirm(vehicle, GROUND_STATION_FAILSAFE_TIMEOUT_PARAMETER_NAME, timeoutSeconds)
        setParameterAndConfirm(vehicle, FAILSAFE_OPTIONS_PARAMETER_NAME, flightOptions)
        print("Onboard failsafe is set to Land if the Pi connection is lost.")

        copterWasArmed = True
        takeOffToFiveFeet(vehicle, flightOutcome)
        flyThePattern(vehicle, flightOutcome)

        landingWasStarted = True
        landUntilPropellersStop(vehicle, flightOutcome)
        if not vehicle.armed:
            flightOutcome["overallOutcome"] = "SUCCESS"
            print("Flight complete.")
        else:
            flightOutcome["overallOutcome"] = "ENDED WITH FAILSAFE LANDING"

    except KeyboardInterrupt:
        print()
        print("Flight interrupted by Ctrl+C.")
        flightOutcome["overallOutcome"] = "EXITED MID-WAY (Ctrl+C)"
        flightOutcome["stopReason"] = "User pressed Ctrl+C."
    except Exception as error:
        print(f"Flight stopped: {error}")
        flightOutcome["overallOutcome"] = "FAILED"
        flightOutcome["stopReason"] = str(error)
    finally:
        if vehicle is not None and not getattr(vehicle, "commandCableClosed", False):
            if (copterWasArmed or vehicle.armed) and not landingWasStarted:
                print("Requesting LAND.")
                try:
                    landUntilPropellersStop(vehicle, flightOutcome)
                except KeyboardInterrupt:
                    print("Second Ctrl+C. Closing the cable without force-disarming.")
                    flightOutcome["landing"] = "SECOND Ctrl+C (Land failsafe left active)"
                except Exception as landingError:
                    print(f"Landing unconfirmed: {landingError}")
                    flightOutcome["landing"] = f"UNCONFIRMED ({landingError})"
            if heartbeatIsFresh(vehicle) and not vehicle.armed and savedFailsafe is not None:
                try:
                    setParameterAndConfirm(vehicle, GROUND_STATION_FAILSAFE_PARAMETER_NAME, savedFailsafe[0])
                    setParameterAndConfirm(vehicle, GROUND_STATION_FAILSAFE_TIMEOUT_PARAMETER_NAME, savedFailsafe[1])
                    setParameterAndConfirm(vehicle, FAILSAFE_OPTIONS_PARAMETER_NAME, savedFailsafe[2])
                    print("Ground-station failsafe settings restored.")
                except Exception as restoreError:
                    print(f"Could not restore the failsafe: {restoreError}")
            if heartbeatIsFresh(vehicle) and not vehicle.armed and originalTakeoffSlewSeconds is not None:
                try:
                    setParameterAndConfirm(vehicle, TAKEOFF_SLEW_PARAMETER_NAME, originalTakeoffSlewSeconds)
                    print(f"{TAKEOFF_SLEW_PARAMETER_NAME} restored.")
                except Exception as restoreError:
                    print(f"Could not restore the takeoff ramp: {restoreError}")
            closeCommandCable(vehicle)
        print()
        print("FLIGHT OUTCOME")
        print(f"Overall : {flightOutcome['overallOutcome']}")
        print(f"Highest : {flightOutcome['maximumAltitudeReachedText']}")
        print(f"Landing : {flightOutcome['landing']}")
        if flightOutcome["stopReason"]:
            print(f"Stopped : {flightOutcome['stopReason']}")
        print("Script finished.")
        sys.stdout = originalStandardOutput
        if logFile is not None:
            logFile.close()


if __name__ == "__main__":
    main()
