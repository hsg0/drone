#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    Take off to 5 feet, hover 60 seconds, turn 360 degrees clockwise,
    face north and hover 4 seconds, fly 10 feet north at 5 feet, hover
    4 seconds, turn 180 degrees counter-clockwise to face south, hover
    30 seconds, then land until the propellers stop.

WHY:
    The Oct 26 flights stayed within about 1 foot while LOITER held the
    spot, then flew the 10-foot north leg at 0.4 m/s. This script keeps
    that path. The 60-second first hover is the useful piece from the
    other drone's hover test. This copter has no radio, so a stick
    takeover is not used. A lost heartbeat still closes the cable so the
    Pixhawk lands itself.

HOW:
    1. Log every line in oct27-2026/thisDrone/flightLog/.
    2. Connect on /dev/serial0, pins 8 and 10, at 57600.
    3. Wait for Quad X, a 3D GPS fix, 6 satellites, a trusted position,
       and a level copter. Type YES.
    4. GUIDED takeoff to 5 feet, then sit for 2 seconds before LOITER.
    5. LOITER 60 seconds. A drift of 8 feet starts LAND.
    6. GUIDED: one full clockwise turn, then face true north and LOITER
       4 seconds.
    7. GUIDED: fly north 10 feet at 5 feet, then LOITER 4 seconds.
    8. GUIDED: half turn counter-clockwise until the nose faces south,
       then LOITER 30 seconds.
    9. LAND until the motors disarm.

IMPORTANT:
    Run this file on the Pi. Do not put python3 in front of the path.
    Leave USB to bootDisplay.py. This script never force-arms and never
    force-disarms. The safety ceiling is 8 feet.
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
# The Oct 26 climbs were still moving when LOITER started. Two seconds lets that settle.
TAKEOFF_SETTLE_SECONDS = 2.0

# 5 feet is the target. 8 feet is high enough for the bounce seen in the logs.
SAFETY_CEILING_FEET = 8.0
SAFETY_CEILING_METERS = SAFETY_CEILING_FEET / FEET_PER_METER

FIRST_HOVER_SECONDS = 60.0
SHORT_HOVER_SECONDS = 4.0
SOUTH_HOVER_SECONDS = 30.0
# Oct 19 drifted past 8 feet in a few seconds and was stopped. Keep that stop.
MAXIMUM_HOVER_DRIFT_FEET = 8.0
MAXIMUM_HOVER_DRIFT_METERS = MAXIMUM_HOVER_DRIFT_FEET / FEET_PER_METER

NORTH_DISTANCE_FEET = 10.0
NORTH_DISTANCE_METERS = NORTH_DISTANCE_FEET / FEET_PER_METER
NORTH_SPEED_METERS_PER_SECOND = 0.4
ALTITUDE_HOLD_GAIN = 0.5
MAXIMUM_VERTICAL_SPEED_METERS_PER_SECOND = 0.25
MAXIMUM_OFF_COURSE_FEET = 8.0
MAXIMUM_OFF_COURSE_METERS = MAXIMUM_OFF_COURSE_FEET / FEET_PER_METER
NORTH_MOVE_TIMEOUT_SECONDS = 30.0
SETTLE_AFTER_MOVE_SECONDS = 2.0
METERS_PER_DEGREE_OF_LATITUDE = 111319.5
MINIMUM_VALID_COORDINATE_DEGREES = 0.01

CLOCKWISE_TURN_DEGREES = 360.0
COUNTERCLOCKWISE_TURN_DEGREES = 180.0
YAW_RATE_DEGREES_PER_SECOND = 20.0
HEADING_TOLERANCE_DEGREES = 15.0
NORTH_HEADING_DEGREES = 0.0
SOUTH_HEADING_DEGREES = 180.0
# A little longer than the turn at 20 deg/s, so a slow yaw can finish.
FULL_TURN_TIMEOUT_SECONDS = 25.0
HALF_TURN_TIMEOUT_SECONDS = 15.0

TAKEOFF_TIMEOUT_SECONDS = 40.0
LAND_TIMEOUT_SECONDS = 60.0
MODE_CHANGE_TIMEOUT_SECONDS = 10.0
ARM_TIMEOUT_SECONDS = 15.0
HEARTBEAT_STALE_AFTER_SECONDS = 3.0
MAXIMUM_GROUND_LEAN_DEGREES = 5.0
MAXIMUM_SAVED_TRIM_DEGREES = 2.5

STATUS_CHECK_INTERVAL_SECONDS = 0.2
PRINT_INTERVAL_SECONDS = 1.0
PARAMETER_TOLERANCE = 0.01

QUAD_FRAME_CLASS = 1
QUAD_X_FRAME_TYPE = 1
MINIMUM_GPS_FIX_TYPE = 3
MINIMUM_SATELLITE_COUNT = 6
UNKNOWN_SATELLITE_COUNT = 255
SKIP_COMPASS_OR_GPS_MASK = 4 | 8 | 4096

TAKEOFF_SLEW_PARAMETER_NAME = "TKOFF_SLEW_TIME"
PREFERRED_TAKEOFF_SLEW_SECONDS = 1.0
GROUND_STATION_FAILSAFE_PARAMETER_NAME = "FS_GCS_ENABLE"
GROUND_STATION_FAILSAFE_TIMEOUT_PARAMETER_NAME = "FS_GCS_TIMEOUT"
FAILSAFE_OPTIONS_PARAMETER_NAME = "FS_OPTIONS"
GROUND_STATION_FAILSAFE_LAND_VALUE = 5
CONTINUE_PILOT_MODE_ON_FAILSAFE_BIT = 16
PREFERRED_FAILSAFE_TIMEOUT_SECONDS = 5.0

MESSAGE_INTERVAL_SECONDS = 0.1
MICROSECONDS_PER_SECOND = 1000000
FLIGHT_MESSAGE_NAMES = (
    "GPS_RAW_INT",
    "GLOBAL_POSITION_INT",
    "EKF_STATUS_REPORT",
    "ATTITUDE",
    "STATUSTEXT",
)

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

# CONDITION_YAW: direction 1 is clockwise as seen from above. -1 is counter-clockwise.
YAW_CLOCKWISE = 1
YAW_COUNTERCLOCKWISE = -1
YAW_RELATIVE = 1
YAW_ABSOLUTE = 0


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
    if not vehicle.ekf_ok:
        raise RuntimeError("Position estimate lost in flight.")
    altitudeMeters = heightMeters(vehicle)
    if altitudeMeters is not None:
        previousMaximum = flightOutcome.get("maximumAltitudeMeters")
        if previousMaximum is None or altitudeMeters > previousMaximum:
            flightOutcome["maximumAltitudeMeters"] = altitudeMeters
            flightOutcome["maximumAltitudeReachedText"] = heightText(altitudeMeters)
        if altitudeMeters >= SAFETY_CEILING_METERS:
            raise RuntimeError(
                f"Safety ceiling ({SAFETY_CEILING_FEET:.0f} ft) reached: {heightText(altitudeMeters)}"
            )
    return altitudeMeters


def commandYaw(vehicle, angleDegrees, turnDirection, relativeOrAbsolute):
    message = vehicle.message_factory.command_long_encode(
        vehicle._master.target_system,
        vehicle._master.target_component,
        middleMan.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        float(angleDegrees),
        YAW_RATE_DEGREES_PER_SECOND,
        float(turnDirection),
        float(relativeOrAbsolute),
        0, 0, 0,
    )
    vehicle.send_mavlink(message)
    vehicle.flush()


def holdPositionWhileTurning(vehicle, flightOutcome):
    altitudeMeters = checkFlightHealth(vehicle, flightOutcome)
    if vehicle.mode.name != "GUIDED":
        raise RuntimeError("GUIDED mode lost during the turn.")
    sendLocalVelocity(vehicle, 0.0, 0.0, downSpeedToHoldAltitude(altitudeMeters))
    return altitudeMeters


def turnRelativeDegrees(vehicle, flightOutcome, turnDegrees, turnDirection, turnName):
    """Turn in place. Progress is the yaw change, so a full circle can be seen finish."""
    setFlightMode(vehicle, "GUIDED")
    startHeading = headingDegrees(vehicle)
    if startHeading is None:
        raise RuntimeError(f"Heading was not available for {turnName}.")
    print(
        f"{turnName}: {turnDegrees:.0f} degrees at {YAW_RATE_DEGREES_PER_SECOND:.0f} deg/s. "
        f"Starting heading {startHeading:+.1f} deg."
    )
    commandYaw(vehicle, turnDegrees, turnDirection, YAW_RELATIVE)
    previousHeading = startHeading
    turnedDegrees = 0.0
    startedAtSeconds = time.monotonic()
    lastPrintedAtSeconds = 0.0
    timeoutSeconds = FULL_TURN_TIMEOUT_SECONDS if turnDegrees >= 300.0 else HALF_TURN_TIMEOUT_SECONDS
    targetTurnDegrees = turnDegrees if turnDirection == YAW_CLOCKWISE else -turnDegrees

    while True:
        holdPositionWhileTurning(vehicle, flightOutcome)
        currentHeading = headingDegrees(vehicle)
        if currentHeading is None:
            raise RuntimeError(f"Heading was lost during {turnName}.")
        turnedDegrees += smallestHeadingDifferenceDegrees(previousHeading, currentHeading)
        previousHeading = currentHeading
        nowSeconds = time.monotonic()
        if nowSeconds - lastPrintedAtSeconds >= PRINT_INTERVAL_SECONDS:
            print(f"{turnName} | turned {turnedDegrees:+.0f} deg | heading {currentHeading:+.1f} deg")
            lastPrintedAtSeconds = nowSeconds
        clockwiseDone = turnDirection == YAW_CLOCKWISE and turnedDegrees >= targetTurnDegrees - HEADING_TOLERANCE_DEGREES
        counterClockwiseDone = (
            turnDirection == YAW_COUNTERCLOCKWISE
            and turnedDegrees <= targetTurnDegrees + HEADING_TOLERANCE_DEGREES
        )
        if clockwiseDone or counterClockwiseDone:
            print(f"{turnName} completed at heading {currentHeading:+.1f} deg.")
            return currentHeading
        if nowSeconds - startedAtSeconds > timeoutSeconds:
            raise RuntimeError(
                f"{turnName} timed out at heading {currentHeading:+.1f} deg after turning {turnedDegrees:+.0f} deg."
            )
        time.sleep(STATUS_CHECK_INTERVAL_SECONDS)


def faceAbsoluteHeading(vehicle, flightOutcome, targetHeadingDegrees, headingName):
    setFlightMode(vehicle, "GUIDED")
    currentHeading = headingDegrees(vehicle)
    if currentHeading is None:
        raise RuntimeError(f"Heading was not available to face {headingName}.")
    errorDegrees = smallestHeadingDifferenceDegrees(currentHeading, targetHeadingDegrees)
    if abs(errorDegrees) <= HEADING_TOLERANCE_DEGREES:
        print(f"Nose is already facing {headingName} (heading {currentHeading:+.1f} deg).")
        return
    turnDirection = YAW_CLOCKWISE if errorDegrees > 0 else YAW_COUNTERCLOCKWISE
    print(f"Turning to face {headingName}. Heading now {currentHeading:+.1f} deg.")
    commandYaw(vehicle, targetHeadingDegrees, turnDirection, YAW_ABSOLUTE)
    startedAtSeconds = time.monotonic()
    while True:
        holdPositionWhileTurning(vehicle, flightOutcome)
        currentHeading = headingDegrees(vehicle)
        if currentHeading is None:
            raise RuntimeError(f"Heading was lost while facing {headingName}.")
        if abs(smallestHeadingDifferenceDegrees(currentHeading, targetHeadingDegrees)) <= HEADING_TOLERANCE_DEGREES:
            print(f"Facing {headingName} at heading {currentHeading:+.1f} deg.")
            return
        if time.monotonic() - startedAtSeconds > HALF_TURN_TIMEOUT_SECONDS:
            raise RuntimeError(f"Could not face {headingName}. Heading stayed {currentHeading:+.1f} deg.")
        time.sleep(STATUS_CHECK_INTERVAL_SECONDS)


def hoverInLoiter(vehicle, flightOutcome, hoverSeconds, hoverName):
    setFlightMode(vehicle, "LOITER")
    startSpot = latitudeAndLongitude(vehicle)
    if startSpot is None:
        raise RuntimeError(f"GPS position was not available for {hoverName}.")
    startLatitudeDegrees, startLongitudeDegrees = startSpot
    print(f"{hoverName} started for {hoverSeconds:.0f} seconds at {HOVER_ALTITUDE_FEET:.0f} ft.")
    startedAtSeconds = time.monotonic()
    lastPrintedAtSeconds = 0.0
    maximumDriftFeet = 0.0
    while True:
        elapsedSeconds = time.monotonic() - startedAtSeconds
        if elapsedSeconds >= hoverSeconds:
            break
        if vehicle.mode.name != "LOITER":
            raise RuntimeError(f"LOITER mode lost during {hoverName}.")
        altitudeMeters = checkFlightHealth(vehicle, flightOutcome)
        currentSpot = latitudeAndLongitude(vehicle)
        if currentSpot is None:
            raise RuntimeError(f"GPS position was lost during {hoverName}.")
        northMeters, eastMeters = northAndEastOffsetMeters(
            startLatitudeDegrees,
            startLongitudeDegrees,
            currentSpot[0],
            currentSpot[1],
        )
        driftFeet = math.hypot(northMeters, eastMeters) * FEET_PER_METER
        maximumDriftFeet = max(maximumDriftFeet, driftFeet)
        if driftFeet > MAXIMUM_HOVER_DRIFT_FEET:
            raise RuntimeError(
                f"{hoverName} drifted {driftFeet:.1f} ft (limit {MAXIMUM_HOVER_DRIFT_FEET:.0f} ft)."
            )
        nowSeconds = time.monotonic()
        if nowSeconds - lastPrintedAtSeconds >= PRINT_INTERVAL_SECONDS:
            remainingSeconds = max(0.0, hoverSeconds - elapsedSeconds)
            print(
                f"{hoverName} | {heightText(altitudeMeters)} | "
                f"drift {driftFeet:.1f} ft | remaining {remainingSeconds:.0f} s"
            )
            lastPrintedAtSeconds = nowSeconds
        time.sleep(STATUS_CHECK_INTERVAL_SECONDS)
    print(f"{hoverName} completed. Maximum drift {maximumDriftFeet:.1f} ft.")
    return maximumDriftFeet


def flyNorthTenFeet(vehicle, flightOutcome):
    setFlightMode(vehicle, "GUIDED")
    startSpot = latitudeAndLongitude(vehicle)
    if startSpot is None:
        raise RuntimeError("GPS position was not available for the north move.")
    startLatitudeDegrees, startLongitudeDegrees = startSpot
    print(
        f"Flying north {NORTH_DISTANCE_FEET:.0f} ft at {NORTH_SPEED_METERS_PER_SECOND:.1f} m/s "
        f"while holding {HOVER_ALTITUDE_FEET:.0f} ft."
    )
    startedAtSeconds = time.monotonic()
    lastPrintedAtSeconds = 0.0
    try:
        while True:
            if vehicle.mode.name != "GUIDED":
                raise RuntimeError("GUIDED mode lost during the north move.")
            altitudeMeters = checkFlightHealth(vehicle, flightOutcome)
            currentSpot = latitudeAndLongitude(vehicle)
            if currentSpot is None:
                raise RuntimeError("GPS position was lost during the north move.")
            northMeters, eastMeters = northAndEastOffsetMeters(
                startLatitudeDegrees,
                startLongitudeDegrees,
                currentSpot[0],
                currentSpot[1],
            )
            northFeet = northMeters * FEET_PER_METER
            eastFeet = eastMeters * FEET_PER_METER
            flightOutcome["northDistanceTraveledText"] = f"{northFeet:.1f} ft north ({eastFeet:+.1f} ft east)"
            if northMeters >= NORTH_DISTANCE_METERS:
                print(f"10 feet north reached | {northFeet:.1f} ft north | {eastFeet:+.1f} ft east | {heightText(altitudeMeters)}")
                break
            if abs(eastMeters) > MAXIMUM_OFF_COURSE_METERS or northMeters < -MAXIMUM_OFF_COURSE_METERS:
                raise RuntimeError(f"Left the north line: {northFeet:.1f} ft north, {eastFeet:+.1f} ft east.")
            if time.monotonic() - startedAtSeconds > NORTH_MOVE_TIMEOUT_SECONDS:
                raise RuntimeError(f"Did not reach 10 feet north in time ({northFeet:.1f} ft north).")
            sendLocalVelocity(
                vehicle,
                NORTH_SPEED_METERS_PER_SECOND,
                0.0,
                downSpeedToHoldAltitude(altitudeMeters),
            )
            nowSeconds = time.monotonic()
            if nowSeconds - lastPrintedAtSeconds >= PRINT_INTERVAL_SECONDS:
                print(
                    f"Flying north | {northFeet:.1f}/{NORTH_DISTANCE_FEET:.0f} ft | "
                    f"{eastFeet:+.1f} ft east | {heightText(altitudeMeters)}"
                )
                lastPrintedAtSeconds = nowSeconds
            time.sleep(STATUS_CHECK_INTERVAL_SECONDS)

        settleStartedAtSeconds = time.monotonic()
        while time.monotonic() - settleStartedAtSeconds < SETTLE_AFTER_MOVE_SECONDS:
            altitudeMeters = checkFlightHealth(vehicle, flightOutcome)
            sendLocalVelocity(vehicle, 0.0, 0.0, downSpeedToHoldAltitude(altitudeMeters))
            time.sleep(STATUS_CHECK_INTERVAL_SECONDS)
        print("North move stopped.")
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


def preflightBlocker(vehicle):
    if not heartbeatIsFresh(vehicle):
        return "Pixhawk heartbeat is missing."
    frameClass = vehicle.parameters["FRAME_CLASS"]
    frameType = vehicle.parameters["FRAME_TYPE"]
    if frameClass is None or frameType is None:
        return "FRAME_CLASS or FRAME_TYPE could not be read."
    if (
        abs(float(frameClass) - QUAD_FRAME_CLASS) >= PARAMETER_TOLERANCE
        or abs(float(frameType) - QUAD_X_FRAME_TYPE) >= PARAMETER_TOLERANCE
    ):
        return f"Quad X required. FRAME_CLASS {float(frameClass):.0f}, FRAME_TYPE {float(frameType):.0f}."
    skipCheckValue = vehicle.parameters["ARMING_SKIPCHK"]
    if skipCheckValue is not None and (int(round(float(skipCheckValue))) & SKIP_COMPASS_OR_GPS_MASK) != 0:
        return f"ARMING_SKIPCHK is {int(round(float(skipCheckValue)))}. Compass or GPS checks are skipped."
    trimX = vehicle.parameters["AHRS_TRIM_X"]
    trimY = vehicle.parameters["AHRS_TRIM_Y"]
    if trimX is not None and trimY is not None:
        trimRollDegrees = math.degrees(float(trimX))
        trimPitchDegrees = math.degrees(float(trimY))
        if abs(trimRollDegrees) > MAXIMUM_SAVED_TRIM_DEGREES or abs(trimPitchDegrees) > MAXIMUM_SAVED_TRIM_DEGREES:
            return (
                f"Saved level trim is tilted. Roll {trimRollDegrees:+.2f} deg, "
                f"pitch {trimPitchDegrees:+.2f} deg."
            )
    gpsReading = vehicle.gps_0
    if gpsReading is None or gpsReading.fix_type is None or int(gpsReading.fix_type) < MINIMUM_GPS_FIX_TYPE:
        return "GPS 3D fix required."
    satelliteCount = gpsReading.satellites_visible
    if satelliteCount is None or satelliteCount == UNKNOWN_SATELLITE_COUNT or satelliteCount < MINIMUM_SATELLITE_COUNT:
        return f"At least 6 satellites required. Seeing {satelliteCount}."
    if not vehicle.ekf_ok:
        if vehicle._ekf_constposmode:
            return f"3D fix, {satelliteCount} satellites. EKF3 is still aligning GPS and compass."
        return f"3D fix, {satelliteCount} satellites. Horizontal position is not trusted yet."
    if not vehicle.is_armable:
        return "Pixhawk pre-arm check is not ready."
    if vehicle.attitude is None or vehicle.attitude.roll is None or vehicle.attitude.pitch is None:
        return "Attitude is not available."
    rollDegrees = math.degrees(vehicle.attitude.roll)
    pitchDegrees = math.degrees(vehicle.attitude.pitch)
    if abs(rollDegrees) > MAXIMUM_GROUND_LEAN_DEGREES or abs(pitchDegrees) > MAXIMUM_GROUND_LEAN_DEGREES:
        return f"Copter is not level. Roll {rollDegrees:+.1f} deg, pitch {pitchDegrees:+.1f} deg."
    return None


def waitUntilReady(vehicle):
    while True:
        blocker = preflightBlocker(vehicle)
        if blocker is None:
            heading = headingDegrees(vehicle)
            headingText = "unknown" if heading is None else f"{heading:+.1f} deg"
            print("Preflight checks passed.")
            print(
                f"FRAME_CLASS {float(vehicle.parameters['FRAME_CLASS']):.0f} | "
                f"FRAME_TYPE {float(vehicle.parameters['FRAME_TYPE']):.0f} | "
                f"satellites {vehicle.gps_0.satellites_visible} | heading {headingText}."
            )
            return
        print(f"Waiting. {blocker}")
        time.sleep(PRINT_INTERVAL_SECONDS)


def takeOffToFiveFeet(vehicle, flightOutcome):
    setFlightMode(vehicle, "GUIDED")
    print("Arming motors.")
    vehicle.armed = True
    startedAtSeconds = time.monotonic()
    while not vehicle.armed:
        if time.monotonic() - startedAtSeconds > ARM_TIMEOUT_SECONDS:
            raise RuntimeError("Arming timed out.")
        if not heartbeatIsFresh(vehicle):
            raise RuntimeError("Pixhawk heartbeat was lost while arming.")
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
            raise RuntimeError("Takeoff timed out.")
        time.sleep(STATUS_CHECK_INTERVAL_SECONDS)
    print(f"Settling at 5 feet for {TAKEOFF_SETTLE_SECONDS:.0f} seconds.")
    settleStartedAtSeconds = time.monotonic()
    while time.monotonic() - settleStartedAtSeconds < TAKEOFF_SETTLE_SECONDS:
        checkFlightHealth(vehicle, flightOutcome)
        time.sleep(STATUS_CHECK_INTERVAL_SECONDS)


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
        "northDistanceTraveledText": "0.0 ft",
        "landing": "NOT STARTED",
    }

    try:
        os.makedirs(FLIGHT_LOG_DIRECTORY, exist_ok=True)
        logPath = os.path.join(
            FLIGHT_LOG_DIRECTORY,
            f"hover360FlyNorthLand-{time.strftime('%Y-%m-%d-%H%M%S')}.log",
        )
        logFile = open(logPath, "w", encoding="utf-8")
        sys.stdout = FlightLog(logFile, sys.stdout)
        print(f"Flight log file : {logPath}")
        print(f"Run started at  : {time.strftime('%Y-%m-%d %H:%M:%S %Z')}")
        print("Take off to 5 ft, hover 60 s, turn 360 clockwise,")
        print("face north, fly 10 ft north, turn to face south, hover 30 s, land.")
        print(f"Safety ceiling {SAFETY_CEILING_FEET:.0f} ft. Hover drift limit {MAXIMUM_HOVER_DRIFT_FEET:.0f} ft.")

        if not os.path.exists(PIXHAWK_PORT):
            raise RuntimeError(f"{PIXHAWK_PORT} is missing. Run this script on the Pi.")
        print(f"Connecting to Pixhawk on {PIXHAWK_PORT} at {PIXHAWK_BAUD} baud.")
        vehicle = connect(
            PIXHAWK_PORT,
            baud=PIXHAWK_BAUD,
            wait_ready=True,
            timeout=CONNECTION_TIMEOUT_SECONDS,
            source_system=GROUND_STATION_SYSTEM_ID,
        )
        if vehicle._master.target_component == 0:
            vehicle._master.target_component = 1
        print("Pixhawk connected.")

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

        waitUntilReady(vehicle)
        promptText = (
            "Type YES to take off to 5 ft, hover 60 s, turn 360 clockwise, "
            "fly 10 ft north, face south, hover 30 s, and land: "
        )
        confirmation = input(promptText)
        if isinstance(sys.stdout, FlightLog):
            sys.stdout.writeLogOnly(f"{promptText}{confirmation}\n")
        if confirmation.strip() != "YES":
            print("Cancelled. The copter was not armed.")
            flightOutcome["overallOutcome"] = "CANCELLED BEFORE ARMING"
            flightOutcome["stopReason"] = "Pilot did not type YES."
            return

        waitUntilReady(vehicle)
        savedSlew = vehicle.parameters[TAKEOFF_SLEW_PARAMETER_NAME]
        if savedSlew is not None:
            savedSlew = float(savedSlew)
            takeoffSlew = min(savedSlew, PREFERRED_TAKEOFF_SLEW_SECONDS)
            print(f"{TAKEOFF_SLEW_PARAMETER_NAME}: original {savedSlew:.2f}s | takeoff {takeoffSlew:.2f}s")
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

        blocker = preflightBlocker(vehicle)
        if blocker is not None:
            raise RuntimeError(blocker)

        copterWasArmed = True
        takeOffToFiveFeet(vehicle, flightOutcome)
        hoverInLoiter(vehicle, flightOutcome, FIRST_HOVER_SECONDS, "First hover")

        turnRelativeDegrees(
            vehicle,
            flightOutcome,
            CLOCKWISE_TURN_DEGREES,
            YAW_CLOCKWISE,
            "Clockwise 360",
        )
        faceAbsoluteHeading(vehicle, flightOutcome, NORTH_HEADING_DEGREES, "north")
        hoverInLoiter(vehicle, flightOutcome, SHORT_HOVER_SECONDS, "Hover facing north")

        flyNorthTenFeet(vehicle, flightOutcome)
        hoverInLoiter(vehicle, flightOutcome, SHORT_HOVER_SECONDS, "Hover after the north leg")

        turnRelativeDegrees(
            vehicle,
            flightOutcome,
            COUNTERCLOCKWISE_TURN_DEGREES,
            YAW_COUNTERCLOCKWISE,
            "Counter-clockwise 180",
        )
        faceAbsoluteHeading(vehicle, flightOutcome, SOUTH_HEADING_DEGREES, "south")
        hoverInLoiter(vehicle, flightOutcome, SOUTH_HOVER_SECONDS, "Hover facing south")

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
        print(f"North   : {flightOutcome['northDistanceTraveledText']}")
        print(f"Landing : {flightOutcome['landing']}")
        if flightOutcome["stopReason"]:
            print(f"Stopped : {flightOutcome['stopReason']}")
        print("Script finished.")
        sys.stdout = originalStandardOutput
        if logFile is not None:
            logFile.close()


if __name__ == "__main__":
    main()
