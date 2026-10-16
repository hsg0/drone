#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    1. Take off to 4 feet in height.
    2. Hover stationary at 4 feet for 10 seconds.
    3. Fly forward in the Northern direction for 10 feet at 4 feet in height.
    4. Hover stationary at 4 feet for 10 seconds.
    5. Land safely and wait until the propellers stop spinning (motors disarm).
    6. Save a complete flight log and outcome summary of every run in
       oct16-2026/flightLog/.

WHY:
    GUIDED handles takeoff to 4 feet and the 10-foot North move (with gentle
    vertical velocity correction so the copter stays at 4 feet during the move).
    LOITER holds the stationary GPS spot and 4-foot altitude during both
    10-second hovers. LAND brings the copter down and disarms the motors so
    the propellers stop spinning.

HOW:
    1. Open a timestamped log file in oct16-2026/flightLog/ that captures every
       printed line, pilot input, and the outcome of each step.
    2. Connect to the Pixhawk on UART pins 8 and 10 (/dev/serial0 at 57600).
    3. Wait for Quad X frame, 3D GPS fix, 6 satellites, trusted EKF position,
       and a level airframe.
    4. Ask the pilot to type YES.
    5. Shorten TKOFF_SLEW_TIME if needed and set the onboard ground-station
       failsafe to Land (FS_GCS_ENABLE = 5, FS_OPTIONS bit 4 cleared).
    6. Step 1: Arm in GUIDED and climb to 4 feet (1.22 meters).
    7. Step 2: Hold stationary in LOITER at 4 feet for 10 seconds.
    8. Step 3: Fly North 10 feet in GUIDED at 0.4 m/s while holding 4 feet in
       height, then command zero speed for 2 seconds to stop cleanly.
    9. Step 4: Hold stationary in LOITER at 4 feet for 10 seconds.
   10. Step 5: Switch to LAND and wait until the motors disarm (propellers stop).
   11. Always write a final FLIGHT RUN OUTCOME SUMMARY to oct16-2026/flightLog/
       whether the flight succeeded, failed, or was exited mid-way.

IMPORTANT:
    Run on the Pi with /home/drone/dronekit-venv/bin/python3.
    /dev/serial0 uses pin 8 (TX), pin 10 (RX), and pin 6 (ground) at 57600 baud.
    Leave both USB ports alone. bootDisplay.py owns the Pixhawk USB port.
    Airframe is a DJI F450 configured as Quad X (FRAME_CLASS 1, FRAME_TYPE 1).
    Refuse FRAME_TYPE 13 (DJI X) because its motor order does not match this wiring.
    Target hover height is 4.0 feet (1.22 meters). Reaching the safety ceiling of
    5.0 feet (1.52 meters) starts LAND immediately.
    Drifting more than 8 feet off the hover spot or North line starts LAND immediately.
    Never force-arm and never force-disarm in the air.
    If the heartbeat is lost, close the UART cable so the Pixhawk failsafe lands the copter.
"""

import math
import os
import sys
import time

from dronekit import VehicleMode
from dronekit import connect
from pymavlink import mavutil as middleMan


# --------------------------------------------------
# CONNECTION AND FLIGHT SETTINGS
# --------------------------------------------------

# Pin 8 transmits, pin 10 receives, pin 6 is ground.
PIXHAWK_PORT = "/dev/serial0"
PIXHAWK_BAUD = 57600
CONNECTION_TIMEOUT_SECONDS = 60
GROUND_STATION_SYSTEM_ID = 255

FLIGHT_LOG_DIRECTORY = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "flightLog",
)

FEET_PER_METER = 3.28084
HOVER_ALTITUDE_FEET = 4.0
HOVER_ALTITUDE_METERS = HOVER_ALTITUDE_FEET / FEET_PER_METER
TAKEOFF_REACHED_FRACTION = 0.95

# Because the flight target is 4.0 feet (1.22 m), the hard safety ceiling is set
# 1 foot above target at 5.0 feet (1.52 m) so normal hover does not trip the ceiling.
MAXIMUM_ALTITUDE_CEILING_FEET = 5.0
MAXIMUM_ALTITUDE_CEILING_METERS = MAXIMUM_ALTITUDE_CEILING_FEET / FEET_PER_METER

HOVER_DURATION_SECONDS = 10.0
MAXIMUM_HOVER_DRIFT_FEET = 8.0
MAXIMUM_HOVER_DRIFT_METERS = MAXIMUM_HOVER_DRIFT_FEET / FEET_PER_METER

# True North move in GUIDED mode while actively holding 4 feet in height.
NORTH_DISTANCE_FEET = 10.0
NORTH_DISTANCE_METERS = NORTH_DISTANCE_FEET / FEET_PER_METER
NORTH_SPEED_METERS_PER_SECOND = 0.4
ALTITUDE_HOLD_GAIN_DURING_MOVE = 0.5
MAXIMUM_VERTICAL_CORRECTION_METERS_PER_SECOND = 0.25
MAXIMUM_OFF_COURSE_FEET = 8.0
MAXIMUM_OFF_COURSE_METERS = MAXIMUM_OFF_COURSE_FEET / FEET_PER_METER
NORTH_MOVE_TIMEOUT_SECONDS = 20.0
SETTLE_AFTER_MOVE_SECONDS = 2.0
METERS_PER_DEGREE_OF_LATITUDE = 111319.5
MINIMUM_VALID_COORDINATE_DEGREES = 0.01

TAKEOFF_TIMEOUT_SECONDS = 30.0
LAND_TIMEOUT_SECONDS = 60.0
MODE_CHANGE_TIMEOUT_SECONDS = 10.0
ARM_TIMEOUT_SECONDS = 15.0
HEARTBEAT_STALE_AFTER_SECONDS = 3.0
MAXIMUM_GROUND_LEAN_DEGREES = 5.0

STATUS_CHECK_INTERVAL_SECONDS = 0.2
PRINT_INTERVAL_SECONDS = 1.0
PARAMETER_TOLERANCE = 0.01

# Standard Quad X (not DJI X type 13).
QUAD_FRAME_CLASS = 1
QUAD_X_FRAME_TYPE = 1
MINIMUM_GPS_FIX_TYPE = 3
MINIMUM_SATELLITE_COUNT = 6
UNKNOWN_SATELLITE_COUNT = 255

# A takeoff ramp slower than 1 second can tip the copter while lifting off.
TAKEOFF_SLEW_PARAMETER_NAME = "TKOFF_SLEW_TIME"
PREFERRED_TAKEOFF_SLEW_SECONDS = 1.0

# ArduPilot Copter 4.7: FS_GCS_ENABLE = 5 lands when Pi heartbeats stop.
# FS_OPTIONS bit 4 (16) must be cleared or LOITER ignores the failsafe.
GROUND_STATION_FAILSAFE_PARAMETER_NAME = "FS_GCS_ENABLE"
GROUND_STATION_FAILSAFE_TIMEOUT_PARAMETER_NAME = "FS_GCS_TIMEOUT"
FAILSAFE_OPTIONS_PARAMETER_NAME = "FS_OPTIONS"
GROUND_STATION_FAILSAFE_LAND_VALUE = 5
CONTINUE_PILOT_MODE_ON_FAILSAFE_BIT = 16
PREFERRED_FAILSAFE_TIMEOUT_SECONDS = 5.0

# Telemetry does not stream on /dev/serial0 until requested.
MESSAGE_INTERVAL_SECONDS = 0.1
MICROSECONDS_PER_SECOND = 1000000
FLIGHT_MESSAGE_NAMES = (
    "GPS_RAW_INT",
    "GLOBAL_POSITION_INT",
    "EKF_STATUS_REPORT",
    "ATTITUDE",
    "STATUSTEXT",
)

# Use velocity only; ignore position, acceleration, yaw, and yaw rate.
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


# --------------------------------------------------
# FLIGHT LOGGING HELPERS
# --------------------------------------------------

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
        """Write a line to the log file without repeating it on the terminal."""
        self.logFile.write(text)
        self.logFile.flush()

    def flush(self):
        self.terminal.flush()
        self.logFile.flush()


def openFlightLog():
    """Create oct16-2026/flightLog/ and open a timestamped log file for this flight."""
    os.makedirs(FLIGHT_LOG_DIRECTORY, exist_ok=True)
    scriptName = os.path.splitext(os.path.basename(__file__))[0]
    startedAtText = time.strftime("%Y-%m-%d-%H%M%S")
    logPath = os.path.join(
        FLIGHT_LOG_DIRECTORY,
        f"{scriptName}-{startedAtText}.log",
    )
    logFile = open(logPath, "w", encoding="utf-8")
    flightLogger = FlightLog(logFile, sys.stdout)
    sys.stdout = flightLogger
    print(f"Flight log file : {logPath}")
    print(f"Run started at  : {time.strftime('%Y-%m-%d %H:%M:%S %Z')}")
    return logFile, logPath, flightLogger


def createFlightOutcomeRecord():
    """Create a dictionary tracking each phase of the flight for the final log summary."""
    return {
        "startedAt": time.strftime("%Y-%m-%d %H:%M:%S %Z"),
        "finishedAt": None,
        "overallOutcome": "FAILED",
        "preflightAndFailsafe": "NOT STARTED",
        "step1TakeoffTo4Feet": "NOT STARTED",
        "step2FirstHover10Seconds": "NOT STARTED",
        "step3FlyNorth10Feet": "NOT STARTED",
        "step4SecondHover10Seconds": "NOT STARTED",
        "step5LandAndStopProps": "NOT STARTED",
        "maximumAltitudeReachedText": "none",
        "northDistanceTraveledText": "0.0 ft",
        "stopReason": None,
    }


def printFlightOutcomeSummary(flightOutcome, logPath):
    """Write a complete summary of the flight run to both the terminal and the log file."""
    flightOutcome["finishedAt"] = time.strftime("%Y-%m-%d %H:%M:%S %Z")
    print()
    print("=" * 68)
    print("FLIGHT RUN OUTCOME SUMMARY")
    print("=" * 68)
    print(f"Overall Outcome           : {flightOutcome['overallOutcome']}")
    print(f"Started At                : {flightOutcome['startedAt']}")
    print(f"Finished At               : {flightOutcome['finishedAt']}")
    print(f"Preflight & Land Failsafe : {flightOutcome['preflightAndFailsafe']}")
    print(f"Step 1 (Takeoff 4 ft)     : {flightOutcome['step1TakeoffTo4Feet']}")
    print(f"Step 2 (First Hover 10s)  : {flightOutcome['step2FirstHover10Seconds']}")
    print(f"Step 3 (Fly North 10 ft)  : {flightOutcome['step3FlyNorth10Feet']}")
    print(f"Step 4 (Second Hover 10s) : {flightOutcome['step4SecondHover10Seconds']}")
    print(f"Step 5 (Land & Disarm)    : {flightOutcome['step5LandAndStopProps']}")
    print(f"Max Altitude Recorded     : {flightOutcome['maximumAltitudeReachedText']}")
    print(f"North Distance Recorded   : {flightOutcome['northDistanceTraveledText']}")
    if flightOutcome["stopReason"]:
        print(f"Stop / Failure Reason     : {flightOutcome['stopReason']}")
    if logPath is not None:
        print(f"Saved Flight Log          : {logPath}")
    print("=" * 68)


# --------------------------------------------------
# MAVLINK AND MODE HELPERS
# --------------------------------------------------

def heartbeatIsFresh(vehicle):
    """True when a Pixhawk heartbeat arrived within the last 3 seconds."""
    # DroneKit stores last_heartbeat as elapsed seconds, not a clock timestamp.
    heartbeatAgeSeconds = vehicle.last_heartbeat
    if heartbeatAgeSeconds is None:
        return False
    return heartbeatAgeSeconds <= HEARTBEAT_STALE_AFTER_SECONDS


def requestFlightMessages(vehicle):
    """Request GPS, altitude, attitude, and position-health streams at 10 Hz."""
    intervalMicroseconds = int(MESSAGE_INTERVAL_SECONDS * MICROSECONDS_PER_SECOND)
    targetSystem = vehicle._master.target_system
    targetComponent = vehicle._master.target_component
    for messageName in FLIGHT_MESSAGE_NAMES:
        messageId = getattr(middleMan.mavlink, f"MAVLINK_MSG_ID_{messageName}", None)
        if messageId is None:
            print(f"MAVLink message id for {messageName} was not found.")
            continue
        message = vehicle.message_factory.command_long_encode(
            targetSystem,
            targetComponent,
            middleMan.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            messageId,
            intervalMicroseconds,
            0,
            0,
            0,
            0,
            0,
        )
        vehicle.send_mavlink(message)
    vehicle.flush()
    print("Requested GPS, height, and position health.")


def setFlightMode(vehicle, flightModeName):
    """Command a flight mode and wait until the Pixhawk confirms it."""
    print(f"Requesting {flightModeName}.")
    vehicle.mode = VehicleMode(flightModeName)
    startTime = time.monotonic()
    while time.monotonic() - startTime < MODE_CHANGE_TIMEOUT_SECONDS:
        if vehicle.mode.name == flightModeName:
            print(f"Mode confirmed: {flightModeName}")
            return True
        if not heartbeatIsFresh(vehicle):
            raise RuntimeError("Pixhawk heartbeat was lost.")
        time.sleep(STATUS_CHECK_INTERVAL_SECONDS)
    raise RuntimeError(f"Could not enter {flightModeName}.")


def sendLocalVelocity(vehicle, northMetersPerSecond, eastMetersPerSecond, downMetersPerSecond):
    """Send earth-frame North/East/Down velocity in GUIDED mode."""
    # GUIDED velocity times out if not refreshed every second.
    message = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        vehicle._master.target_system,
        vehicle._master.target_component,
        middleMan.mavlink.MAV_FRAME_LOCAL_NED,
        VELOCITY_ONLY_TYPE_MASK,
        0,
        0,
        0,
        northMetersPerSecond,
        eastMetersPerSecond,
        downMetersPerSecond,
        0,
        0,
        0,
        0,
        0,
    )
    vehicle.send_mavlink(message)
    vehicle.flush()


def closeCommandCable(vehicle):
    """Close the UART cable once so DroneKit stops sending heartbeats."""
    if vehicle is None or getattr(vehicle, "commandCableClosed", False):
        return
    vehicle.commandCableClosed = True
    try:
        vehicle.close()
        print("Command cable closed.")
    except Exception as error:
        print(f"Could not close the command cable: {error}")


# --------------------------------------------------
# ALTITUDE AND GPS POSITION HELPERS
# --------------------------------------------------

def heightMeters(vehicle):
    """Height above the arming/home point in meters."""
    location = vehicle.location.global_relative_frame
    if location is None or location.alt is None:
        return None
    altitudeMeters = float(location.alt)
    if not math.isfinite(altitudeMeters):
        return None
    return altitudeMeters


def heightText(altitudeMeters):
    """Format height in both meters and feet."""
    if altitudeMeters is None:
        return "height waiting"
    altitudeFeet = altitudeMeters * FEET_PER_METER
    return f"{altitudeMeters:.2f} m ({altitudeFeet:.2f} ft)"


def heightReachedCeiling(altitudeMeters):
    """True when height reaches the 5.0-foot (1.52 m) safety ceiling."""
    if altitudeMeters is None:
        return False
    return altitudeMeters >= MAXIMUM_ALTITUDE_CEILING_METERS


def altitudeHoldDownSpeedMetersPerSecond(altitudeMeters):
    """Return a gentle NED down velocity (positive = down, negative = climb) to hold 4 feet."""
    if altitudeMeters is None:
        return 0.0
    altitudeErrorMeters = HOVER_ALTITUDE_METERS - altitudeMeters
    climbRateMetersPerSecond = max(
        -MAXIMUM_VERTICAL_CORRECTION_METERS_PER_SECOND,
        min(
            MAXIMUM_VERTICAL_CORRECTION_METERS_PER_SECOND,
            altitudeErrorMeters * ALTITUDE_HOLD_GAIN_DURING_MOVE,
        ),
    )
    # In NED, positive Z velocity is down, so negate the desired climb rate.
    return -climbRateMetersPerSecond


def latitudeAndLongitude(vehicle):
    """Return (latitudeDegrees, longitudeDegrees) or None if not valid."""
    location = vehicle.location.global_relative_frame
    if location is None or location.lat is None or location.lon is None:
        return None
    latitudeDegrees = float(location.lat)
    longitudeDegrees = float(location.lon)
    if not math.isfinite(latitudeDegrees) or not math.isfinite(longitudeDegrees):
        return None
    # 0,0 means the GPS has not populated a real coordinate yet.
    if (
        abs(latitudeDegrees) < MINIMUM_VALID_COORDINATE_DEGREES
        and abs(longitudeDegrees) < MINIMUM_VALID_COORDINATE_DEGREES
    ):
        return None
    return latitudeDegrees, longitudeDegrees


def northAndEastOffsetMeters(
    startLatitudeDegrees,
    startLongitudeDegrees,
    currentLatitudeDegrees,
    currentLongitudeDegrees,
):
    """Return (northMeters, eastMeters) traveled from the starting coordinate."""
    northMeters = (currentLatitudeDegrees - startLatitudeDegrees) * METERS_PER_DEGREE_OF_LATITUDE
    eastMeters = (
        (currentLongitudeDegrees - startLongitudeDegrees)
        * METERS_PER_DEGREE_OF_LATITUDE
        * math.cos(math.radians(startLatitudeDegrees))
    )
    return northMeters, eastMeters


# --------------------------------------------------
# PREFLIGHT AND FAILSAFE SETUP
# --------------------------------------------------

def preflightProblem(vehicle):
    """Return None when ready to arm, or the reason arming is blocked."""
    if not heartbeatIsFresh(vehicle):
        return "Pixhawk heartbeat is missing."

    frameClass = vehicle.parameters["FRAME_CLASS"]
    frameType = vehicle.parameters["FRAME_TYPE"]
    if frameClass is None or frameType is None:
        return "FRAME_CLASS or FRAME_TYPE could not be read."
    frameIsQuad = abs(float(frameClass) - QUAD_FRAME_CLASS) < PARAMETER_TOLERANCE
    frameIsX = abs(float(frameType) - QUAD_X_FRAME_TYPE) < PARAMETER_TOLERANCE
    if not frameIsQuad or not frameIsX:
        return (
            f"Quad X required. FRAME_CLASS {float(frameClass):.0f}, "
            f"FRAME_TYPE {float(frameType):.0f}."
        )

    gpsReading = vehicle.gps_0
    if gpsReading is None or gpsReading.fix_type is None or int(gpsReading.fix_type) < MINIMUM_GPS_FIX_TYPE:
        return "GPS 3D fix required."
    satelliteCount = gpsReading.satellites_visible
    if (
        satelliteCount is None
        or satelliteCount == UNKNOWN_SATELLITE_COUNT
        or satelliteCount < MINIMUM_SATELLITE_COUNT
    ):
        return f"At least 6 satellites required. Seeing {satelliteCount}."

    # ekf_ok confirms the horizontal position is trusted and not in constant-position mode.
    if not vehicle.ekf_ok:
        if vehicle._ekf_constposmode:
            return (
                f"3D fix, {satelliteCount} satellites. "
                "The EKF is not using GPS for horizontal position."
            )
        return (
            f"3D fix, {satelliteCount} satellites. "
            "Horizontal position is not trusted yet."
        )

    if not vehicle.is_armable:
        return "Pixhawk is not armable yet."

    if vehicle.attitude is None or vehicle.attitude.roll is None or vehicle.attitude.pitch is None:
        return "Attitude is not available."
    rollDegrees = math.degrees(vehicle.attitude.roll)
    pitchDegrees = math.degrees(vehicle.attitude.pitch)
    if (
        abs(rollDegrees) > MAXIMUM_GROUND_LEAN_DEGREES
        or abs(pitchDegrees) > MAXIMUM_GROUND_LEAN_DEGREES
    ):
        return f"Copter is not level. Roll {rollDegrees:+.1f} deg, pitch {pitchDegrees:+.1f} deg."

    return None


def waitForPreflight(vehicle):
    """Wait until all preflight checks pass, printing the current blocker once per second."""
    while True:
        problem = preflightProblem(vehicle)
        if problem is None:
            print("Preflight checks passed.")
            print(
                f"FRAME_CLASS {float(vehicle.parameters['FRAME_CLASS']):.0f} | "
                f"FRAME_TYPE {float(vehicle.parameters['FRAME_TYPE']):.0f} | "
                f"satellites {vehicle.gps_0.satellites_visible}."
            )
            print("Frame is a DJI F450, saved as Quad X.")
            return
        print(f"Waiting. {problem}")
        time.sleep(PRINT_INTERVAL_SECONDS)


def setParameterAndConfirm(vehicle, parameterName, wantedValue):
    """Write a Pixhawk parameter and verify that the new value was stored."""
    currentValue = vehicle.parameters[parameterName]
    if currentValue is not None and abs(float(currentValue) - float(wantedValue)) < PARAMETER_TOLERANCE:
        return float(currentValue)
    # DroneKit's set() returns False on timeout instead of raising an exception.
    wasWritten = vehicle.parameters.set(parameterName, wantedValue)
    confirmedValue = vehicle.parameters[parameterName]
    if (
        not wasWritten
        or confirmedValue is None
        or abs(float(confirmedValue) - float(wantedValue)) >= PARAMETER_TOLERANCE
    ):
        raise RuntimeError(f"{parameterName} did not stay at {float(wantedValue):.1f}.")
    return float(confirmedValue)


def shortenSlowTakeoffRamp(vehicle):
    """Shorten TKOFF_SLEW_TIME to 1.0s if slower, and return the original value."""
    savedTakeoffSlewSeconds = vehicle.parameters[TAKEOFF_SLEW_PARAMETER_NAME]
    if savedTakeoffSlewSeconds is None:
        print(f"{TAKEOFF_SLEW_PARAMETER_NAME} was not available. Keeping existing ramp.")
        return None
    savedTakeoffSlewSeconds = float(savedTakeoffSlewSeconds)
    temporaryTakeoffSlewSeconds = min(savedTakeoffSlewSeconds, PREFERRED_TAKEOFF_SLEW_SECONDS)
    print(
        f"{TAKEOFF_SLEW_PARAMETER_NAME}: "
        f"original {savedTakeoffSlewSeconds:.2f}s | "
        f"takeoff {temporaryTakeoffSlewSeconds:.2f}s"
    )
    if abs(temporaryTakeoffSlewSeconds - savedTakeoffSlewSeconds) < PARAMETER_TOLERANCE:
        return None
    setParameterAndConfirm(vehicle, TAKEOFF_SLEW_PARAMETER_NAME, temporaryTakeoffSlewSeconds)
    return savedTakeoffSlewSeconds


def restoreTakeoffRamp(vehicle, originalTakeoffSlewSeconds):
    """Restore TKOFF_SLEW_TIME after disarm if this script changed it."""
    if vehicle is None or originalTakeoffSlewSeconds is None or not heartbeatIsFresh(vehicle):
        return
    if getattr(vehicle, "commandCableClosed", False):
        return
    print(f"Restoring {TAKEOFF_SLEW_PARAMETER_NAME} to {originalTakeoffSlewSeconds:.2f}s.")
    setParameterAndConfirm(vehicle, TAKEOFF_SLEW_PARAMETER_NAME, originalTakeoffSlewSeconds)
    print(f"{TAKEOFF_SLEW_PARAMETER_NAME} restored.")


def enableLandingFailsafe(vehicle):
    """Configure the Pixhawk to land itself if Pi heartbeats stop."""
    savedEnable = vehicle.parameters[GROUND_STATION_FAILSAFE_PARAMETER_NAME]
    savedTimeout = vehicle.parameters[GROUND_STATION_FAILSAFE_TIMEOUT_PARAMETER_NAME]
    savedOptions = vehicle.parameters[FAILSAFE_OPTIONS_PARAMETER_NAME]
    if savedEnable is None or savedTimeout is None or savedOptions is None:
        raise RuntimeError("Ground-station failsafe settings could not be read.")

    savedEnable = float(savedEnable)
    savedTimeout = float(savedTimeout)
    savedOptions = float(savedOptions)

    timeoutSeconds = savedTimeout
    if timeoutSeconds <= 0 or timeoutSeconds > PREFERRED_FAILSAFE_TIMEOUT_SECONDS:
        timeoutSeconds = PREFERRED_FAILSAFE_TIMEOUT_SECONDS
    savedOptionsInt = int(round(savedOptions))
    flightOptions = savedOptionsInt & ~CONTINUE_PILOT_MODE_ON_FAILSAFE_BIT
    savedFailsafe = (savedEnable, savedTimeout, savedOptions)

    print(
        f"{GROUND_STATION_FAILSAFE_PARAMETER_NAME}: "
        f"original {savedEnable:.0f} | flight {GROUND_STATION_FAILSAFE_LAND_VALUE:.0f} (Land)."
    )
    print(
        f"{GROUND_STATION_FAILSAFE_TIMEOUT_PARAMETER_NAME}: "
        f"original {savedTimeout:.1f}s | flight {timeoutSeconds:.1f}s."
    )
    print(f"{FAILSAFE_OPTIONS_PARAMETER_NAME}: original {savedOptionsInt} | flight {flightOptions}.")

    try:
        setParameterAndConfirm(
            vehicle,
            GROUND_STATION_FAILSAFE_PARAMETER_NAME,
            GROUND_STATION_FAILSAFE_LAND_VALUE,
        )
        setParameterAndConfirm(
            vehicle,
            GROUND_STATION_FAILSAFE_TIMEOUT_PARAMETER_NAME,
            timeoutSeconds,
        )
        setParameterAndConfirm(
            vehicle,
            FAILSAFE_OPTIONS_PARAMETER_NAME,
            flightOptions,
        )
    except BaseException:
        try:
            restoreLandingFailsafe(vehicle, savedFailsafe)
        except Exception as restoreError:
            print(f"Could not restore the ground-station failsafe: {restoreError}")
        raise

    vehicle.groundStationFailsafeTimeoutSeconds = timeoutSeconds
    print("Onboard failsafe is set to Land if the Pi connection is lost.")
    return savedFailsafe


def restoreLandingFailsafe(vehicle, savedFailsafe):
    """Restore original failsafe parameters only after the motors are confirmed disarmed."""
    if vehicle is None or savedFailsafe is None:
        return
    if getattr(vehicle, "commandCableClosed", False) or not heartbeatIsFresh(vehicle) or vehicle.armed:
        print("Leaving Land failsafe active on the Pixhawk.")
        return

    savedEnable, savedTimeout, savedOptions = savedFailsafe
    setParameterAndConfirm(vehicle, GROUND_STATION_FAILSAFE_PARAMETER_NAME, savedEnable)
    setParameterAndConfirm(vehicle, GROUND_STATION_FAILSAFE_TIMEOUT_PARAMETER_NAME, savedTimeout)
    setParameterAndConfirm(vehicle, FAILSAFE_OPTIONS_PARAMETER_NAME, savedOptions)
    print("Ground-station failsafe settings restored.")


def flightStillHealthy(vehicle, flightOutcome):
    """Verify heartbeat, arm state, EKF position, and 5-foot safety ceiling."""
    if not heartbeatIsFresh(vehicle):
        raise RuntimeError("Pixhawk heartbeat was lost.")
    if not vehicle.armed:
        raise RuntimeError("Copter disarmed unexpectedly in flight.")
    if not vehicle.ekf_ok:
        raise RuntimeError("Position estimate lost.")
    altitudeMeters = heightMeters(vehicle)
    if altitudeMeters is not None:
        previousMaximum = getattr(vehicle, "maximumAltitudeMeters", None)
        if previousMaximum is None or altitudeMeters > previousMaximum:
            vehicle.maximumAltitudeMeters = altitudeMeters
            flightOutcome["maximumAltitudeReachedText"] = heightText(altitudeMeters)
    if heightReachedCeiling(altitudeMeters):
        raise RuntimeError(
            f"Safety ceiling ({MAXIMUM_ALTITUDE_CEILING_FEET:.0f} ft) reached: "
            f"{heightText(altitudeMeters)}"
        )
    return altitudeMeters


# --------------------------------------------------
# FLIGHT STEPS
# --------------------------------------------------

def takeOff(vehicle, flightOutcome):
    """Step 1: Enter GUIDED, arm the motors, and climb to 4 feet."""
    flightOutcome["step1TakeoffTo4Feet"] = "IN PROGRESS"
    setFlightMode(vehicle, "GUIDED")

    # Normal arm only; ArduPilot still runs all pre-arm checks.
    print("Arming motors.")
    vehicle.armed = True
    startTime = time.monotonic()
    while not vehicle.armed:
        if time.monotonic() - startTime > ARM_TIMEOUT_SECONDS:
            flightOutcome["step1TakeoffTo4Feet"] = "FAILED (Arming timed out)"
            raise RuntimeError("Arming timed out.")
        if not heartbeatIsFresh(vehicle):
            flightOutcome["step1TakeoffTo4Feet"] = "FAILED (Heartbeat lost while arming)"
            raise RuntimeError("Pixhawk heartbeat was lost while arming.")
        time.sleep(STATUS_CHECK_INTERVAL_SECONDS)
    print("Motors armed. Propellers spinning.")

    print(f"Taking off to {HOVER_ALTITUDE_FEET:.1f} feet ({HOVER_ALTITUDE_METERS:.2f} m).")
    vehicle.simple_takeoff(HOVER_ALTITUDE_METERS)
    minimumAltitudeMeters = HOVER_ALTITUDE_METERS * TAKEOFF_REACHED_FRACTION
    startTime = time.monotonic()
    lastPrintedAtSeconds = 0.0
    while True:
        if vehicle.mode.name != "GUIDED":
            flightOutcome["step1TakeoffTo4Feet"] = "FAILED (GUIDED mode lost)"
            raise RuntimeError("GUIDED mode lost during takeoff.")
        altitudeMeters = flightStillHealthy(vehicle, flightOutcome)

        currentTimeSeconds = time.monotonic()
        if currentTimeSeconds - lastPrintedAtSeconds >= PRINT_INTERVAL_SECONDS:
            print(f"Climbing | {heightText(altitudeMeters)}")
            lastPrintedAtSeconds = currentTimeSeconds

        if altitudeMeters is not None and altitudeMeters >= minimumAltitudeMeters:
            print(f"Reached 4-foot target altitude: {heightText(altitudeMeters)}.")
            flightOutcome["step1TakeoffTo4Feet"] = f"SUCCESS ({heightText(altitudeMeters)})"
            return
        if currentTimeSeconds - startTime > TAKEOFF_TIMEOUT_SECONDS:
            flightOutcome["step1TakeoffTo4Feet"] = "FAILED (Takeoff timed out)"
            raise RuntimeError("Takeoff timed out.")
        time.sleep(STATUS_CHECK_INTERVAL_SECONDS)


def hoverStationary(vehicle, hoverName, outcomeKey, flightOutcome):
    """Step 2 and Step 4: Hold stationary at 4 feet in LOITER for 10 seconds."""
    flightOutcome[outcomeKey] = "IN PROGRESS"
    setFlightMode(vehicle, "LOITER")

    hoverStartSpot = latitudeAndLongitude(vehicle)
    if hoverStartSpot is None:
        flightOutcome[outcomeKey] = "FAILED (No GPS spot at hover start)"
        raise RuntimeError(f"GPS position was not available for {hoverName}.")
    startLatitudeDegrees, startLongitudeDegrees = hoverStartSpot

    print(f"{hoverName} started (holding 4 ft for {HOVER_DURATION_SECONDS:.0f} seconds).")
    startTime = time.monotonic()
    lastPrintedAtSeconds = 0.0
    maxDriftFeet = 0.0
    while True:
        elapsedSeconds = time.monotonic() - startTime
        if elapsedSeconds >= HOVER_DURATION_SECONDS:
            break
        if vehicle.mode.name != "LOITER":
            flightOutcome[outcomeKey] = "FAILED (LOITER mode lost)"
            raise RuntimeError(f"LOITER mode lost during {hoverName}.")
        altitudeMeters = flightStillHealthy(vehicle, flightOutcome)

        currentSpot = latitudeAndLongitude(vehicle)
        if currentSpot is None:
            flightOutcome[outcomeKey] = "FAILED (GPS position lost)"
            raise RuntimeError(f"GPS position was lost during {hoverName}.")
        northMeters, eastMeters = northAndEastOffsetMeters(
            startLatitudeDegrees,
            startLongitudeDegrees,
            currentSpot[0],
            currentSpot[1],
        )
        driftMeters = math.hypot(northMeters, eastMeters)
        driftFeet = driftMeters * FEET_PER_METER
        if driftFeet > maxDriftFeet:
            maxDriftFeet = driftFeet
        if driftMeters > MAXIMUM_HOVER_DRIFT_METERS:
            flightOutcome[outcomeKey] = f"FAILED (Drifted {driftFeet:.1f} ft)"
            raise RuntimeError(
                f"{hoverName} drifted {driftFeet:.1f} ft from its spot "
                f"(limit {MAXIMUM_HOVER_DRIFT_FEET:.0f} ft)."
            )

        currentTimeSeconds = time.monotonic()
        if currentTimeSeconds - lastPrintedAtSeconds >= PRINT_INTERVAL_SECONDS:
            remainingSeconds = max(0.0, HOVER_DURATION_SECONDS - elapsedSeconds)
            print(
                f"{hoverName} | {heightText(altitudeMeters)} | "
                f"drift {driftFeet:.1f} ft | remaining {remainingSeconds:.0f} s"
            )
            lastPrintedAtSeconds = currentTimeSeconds
        time.sleep(STATUS_CHECK_INTERVAL_SECONDS)

    print(f"{hoverName} completed.")
    flightOutcome[outcomeKey] = f"SUCCESS (10s completed, max drift {maxDriftFeet:.1f} ft)"


def flyNorthAtFourFeet(vehicle, flightOutcome):
    """Step 3: Fly 10 feet in the Northern direction in GUIDED while holding 4 feet in height."""
    flightOutcome["step3FlyNorth10Feet"] = "IN PROGRESS"
    setFlightMode(vehicle, "GUIDED")

    startSpot = latitudeAndLongitude(vehicle)
    if startSpot is None:
        flightOutcome["step3FlyNorth10Feet"] = "FAILED (No GPS position at start of North move)"
        raise RuntimeError("GPS position was not available for the North move.")
    startLatitudeDegrees, startLongitudeDegrees = startSpot

    print(
        f"Flying North {NORTH_DISTANCE_FEET:.0f} ft at {NORTH_SPEED_METERS_PER_SECOND:.1f} m/s "
        f"while holding {HOVER_ALTITUDE_FEET:.0f} ft height."
    )
    startedAtSeconds = time.monotonic()
    lastPrintedAtSeconds = 0.0
    try:
        while True:
            if vehicle.mode.name != "GUIDED":
                flightOutcome["step3FlyNorth10Feet"] = "FAILED (GUIDED mode lost)"
                raise RuntimeError("GUIDED mode lost during the North move.")
            altitudeMeters = flightStillHealthy(vehicle, flightOutcome)

            currentSpot = latitudeAndLongitude(vehicle)
            if currentSpot is None:
                flightOutcome["step3FlyNorth10Feet"] = "FAILED (GPS position lost)"
                raise RuntimeError("GPS position was lost during the North move.")
            northMeters, eastMeters = northAndEastOffsetMeters(
                startLatitudeDegrees,
                startLongitudeDegrees,
                currentSpot[0],
                currentSpot[1],
            )
            northFeet = northMeters * FEET_PER_METER
            eastFeet = eastMeters * FEET_PER_METER
            flightOutcome["northDistanceTraveledText"] = f"{northFeet:.1f} ft North ({eastFeet:+.1f} ft East)"

            if northMeters >= NORTH_DISTANCE_METERS:
                print(
                    f"10 feet North reached | {northFeet:.1f} ft North | "
                    f"{eastFeet:+.1f} ft East | {heightText(altitudeMeters)}"
                )
                break

            # Stop if the copter drifts more than 8 feet East/West or South.
            leftTheNorthLine = (
                abs(eastMeters) > MAXIMUM_OFF_COURSE_METERS
                or northMeters < -MAXIMUM_OFF_COURSE_METERS
            )
            if leftTheNorthLine:
                flightOutcome["step3FlyNorth10Feet"] = (
                    f"FAILED (Off course: {northFeet:.1f} ft N, {eastFeet:+.1f} ft E)"
                )
                raise RuntimeError(
                    f"Left the North line: {northFeet:.1f} ft North, {eastFeet:+.1f} ft East."
                )
            if time.monotonic() - startedAtSeconds > NORTH_MOVE_TIMEOUT_SECONDS:
                flightOutcome["step3FlyNorth10Feet"] = (
                    f"FAILED (Timed out at {northFeet:.1f} ft N)"
                )
                raise RuntimeError(
                    f"Did not reach 10 feet North in time ({northFeet:.1f} ft North, {eastFeet:+.1f} ft East)."
                )

            downSpeedMetersPerSecond = altitudeHoldDownSpeedMetersPerSecond(altitudeMeters)
            sendLocalVelocity(vehicle, NORTH_SPEED_METERS_PER_SECOND, 0.0, downSpeedMetersPerSecond)

            currentTimeSeconds = time.monotonic()
            if currentTimeSeconds - lastPrintedAtSeconds >= PRINT_INTERVAL_SECONDS:
                print(
                    f"Flying North | {northFeet:.1f}/{NORTH_DISTANCE_FEET:.0f} ft North | "
                    f"{eastFeet:+.1f} ft East | {heightText(altitudeMeters)}"
                )
                lastPrintedAtSeconds = currentTimeSeconds
            time.sleep(STATUS_CHECK_INTERVAL_SECONDS)

        # Hold zero horizontal speed for 2 seconds so the copter stops cleanly before LOITER.
        settleStartedAtSeconds = time.monotonic()
        while time.monotonic() - settleStartedAtSeconds < SETTLE_AFTER_MOVE_SECONDS:
            if vehicle.mode.name != "GUIDED":
                flightOutcome["step3FlyNorth10Feet"] = "FAILED (GUIDED lost while stopping)"
                raise RuntimeError("GUIDED mode lost while stopping.")
            altitudeMeters = flightStillHealthy(vehicle, flightOutcome)
            downSpeedMetersPerSecond = altitudeHoldDownSpeedMetersPerSecond(altitudeMeters)
            sendLocalVelocity(vehicle, 0.0, 0.0, downSpeedMetersPerSecond)
            time.sleep(STATUS_CHECK_INTERVAL_SECONDS)

        print("North move stopped cleanly.")
        flightOutcome["step3FlyNorth10Feet"] = (
            f"SUCCESS ({flightOutcome['northDistanceTraveledText']})"
        )
    finally:
        # Send zero speed on exit if the link is still up.
        if heartbeatIsFresh(vehicle) and not getattr(vehicle, "commandCableClosed", False):
            try:
                sendLocalVelocity(vehicle, 0.0, 0.0, 0.0)
            except Exception as stopError:
                print(f"Could not send the stop speed: {stopError}")


def landAndStopPropellers(vehicle, flightOutcome):
    """Step 5: Enter LAND mode and wait until the motors disarm and propellers stop."""
    flightOutcome["step5LandAndStopProps"] = "IN PROGRESS"
    if getattr(vehicle, "commandCableClosed", False):
        print("Command cable is already closed. The Pi cannot confirm landing.")
        flightOutcome["step5LandAndStopProps"] = (
            "ONBOARD FAILSAFE LANDING (Cable closed after heartbeat loss)"
        )
        return
    if not vehicle.armed:
        print("Motors already disarmed. Propellers are stopped.")
        flightOutcome["step5LandAndStopProps"] = "SUCCESS (Motors disarmed, propellers stopped)"
        return
    if not heartbeatIsFresh(vehicle):
        failsafeTimeoutSeconds = getattr(
            vehicle,
            "groundStationFailsafeTimeoutSeconds",
            PREFERRED_FAILSAFE_TIMEOUT_SECONDS,
        )
        print("Heartbeat is lost. The Pi cannot confirm that the Pixhawk received LAND.")
        print(
            "Closing the cable so the Pixhawk stops hearing the Pi. "
            f"Its onboard failsafe lands the copter after {failsafeTimeoutSeconds:.0f} seconds."
        )
        closeCommandCable(vehicle)
        flightOutcome["step5LandAndStopProps"] = (
            f"ONBOARD FAILSAFE TRIGGERED (Cable closed; Pixhawk lands after {failsafeTimeoutSeconds:.0f}s)"
        )
        return

    print("Landing.")
    setFlightMode(vehicle, "LAND")
    startTime = time.monotonic()
    while vehicle.armed:
        # If the heartbeat dies during descent, close the cable so the onboard failsafe takes over.
        if not heartbeatIsFresh(vehicle):
            failsafeTimeoutSeconds = getattr(
                vehicle,
                "groundStationFailsafeTimeoutSeconds",
                PREFERRED_FAILSAFE_TIMEOUT_SECONDS,
            )
            print("Heartbeat is lost while waiting for landing. The Pi cannot confirm disarm.")
            print(
                "Closing the cable so the Pixhawk stops hearing the Pi. "
                f"Its onboard failsafe lands the copter after {failsafeTimeoutSeconds:.0f} seconds."
            )
            closeCommandCable(vehicle)
            flightOutcome["step5LandAndStopProps"] = (
                f"ONBOARD FAILSAFE TRIGGERED DURING DESCENT (Lands after {failsafeTimeoutSeconds:.0f}s)"
            )
            return
        if time.monotonic() - startTime > LAND_TIMEOUT_SECONDS:
            flightOutcome["step5LandAndStopProps"] = "FAILED (Timed out waiting for motors to disarm)"
            raise RuntimeError("Landing not confirmed. The motors are still armed.")
        altitudeMeters = heightMeters(vehicle)
        print(f"Waiting for landing and propeller stop. {heightText(altitudeMeters)}")
        time.sleep(PRINT_INTERVAL_SECONDS)

    print("Landed safely. Motors disarmed and propellers stopped.")
    flightOutcome["step5LandAndStopProps"] = "SUCCESS (Landed, motors disarmed, propellers stopped)"


# --------------------------------------------------
# MAIN FLIGHT
# --------------------------------------------------

def main():
    """Take off to 4 ft, hover 10s, fly North 10 ft at 4 ft, hover 10s, land, and log the flight."""
    originalStandardOutput = sys.stdout
    logFile = None
    logPath = None
    flightLogger = None
    vehicle = None
    originalTakeoffSlewSeconds = None
    savedFailsafe = None
    copterWasArmed = False
    landingWasStarted = False
    flightOutcome = createFlightOutcomeRecord()

    try:
        logFile, logPath, flightLogger = openFlightLog()

        print("====================================================================")
        print("TAKE OFF TO 4 FT, HOVER 10S, FLY NORTH 10 FT, HOVER 10S, LAND")
        print("====================================================================")
        print(f"Target flight height : {HOVER_ALTITUDE_FEET:.1f} ft ({HOVER_ALTITUDE_METERS:.2f} m)")
        print(f"Safety ceiling       : {MAXIMUM_ALTITUDE_CEILING_FEET:.1f} ft ({MAXIMUM_ALTITUDE_CEILING_METERS:.2f} m)")
        print(f"Stationary hovers    : {HOVER_DURATION_SECONDS:.0f} seconds, twice")
        print(f"North leg            : {NORTH_DISTANCE_FEET:.0f} ft North at {NORTH_SPEED_METERS_PER_SECOND:.1f} m/s")
        print()

        # Connect to the Pixhawk on UART pins 8 and 10.
        if not os.path.exists(PIXHAWK_PORT):
            raise RuntimeError(f"{PIXHAWK_PORT} is missing. Pins 8 and 10 are on the Pi.")

        print(f"Connecting to Pixhawk on {PIXHAWK_PORT} at {PIXHAWK_BAUD} baud.")
        vehicle = connect(
            PIXHAWK_PORT,
            baud=PIXHAWK_BAUD,
            wait_ready=True,
            timeout=CONNECTION_TIMEOUT_SECONDS,
            source_system=GROUND_STATION_SYSTEM_ID,
        )
        print("Pixhawk connected.")
        requestFlightMessages(vehicle)

        # Wait for Quad X frame, 3D GPS fix, 6 satellites, trusted EKF, and level attitude.
        waitForPreflight(vehicle)

        # Require typed YES before arming, and record the pilot's input in the log file.
        promptText = (
            "Type YES to take off to 4 ft, hover 10s, fly 10 ft North, hover 10s, and land: "
        )
        confirmation = input(promptText)
        if flightLogger is not None:
            flightLogger.writeLogOnly(f"{promptText}{confirmation}\n")
        if confirmation.strip() != "YES":
            print("Flight cancelled by pilot. The copter was not armed.")
            flightOutcome["overallOutcome"] = "CANCELLED BEFORE ARMING"
            flightOutcome["preflightAndFailsafe"] = "PASSED (Pilot cancelled at YES prompt)"
            flightOutcome["stopReason"] = "Pilot did not type YES."
            return

        # Re-verify preflight and configure takeoff ramp and onboard Land failsafe.
        waitForPreflight(vehicle)
        originalTakeoffSlewSeconds = shortenSlowTakeoffRamp(vehicle)
        savedFailsafe = enableLandingFailsafe(vehicle)
        problem = preflightProblem(vehicle)
        if problem is not None:
            raise RuntimeError(problem)
        flightOutcome["preflightAndFailsafe"] = "SUCCESS (Preflight passed, FS_GCS_ENABLE=5 Land set)"

        # Step 1: Arm in GUIDED and climb to 4 feet.
        copterWasArmed = True
        takeOff(vehicle, flightOutcome)

        # Step 2: Hover stationary at 4 feet in LOITER for 10 seconds.
        hoverStationary(vehicle, "First hover (4 ft)", "step2FirstHover10Seconds", flightOutcome)

        # Step 3: Fly in the Northern direction for 10 feet at 4 feet in height.
        flyNorthAtFourFeet(vehicle, flightOutcome)

        # Step 4: Hover stationary at 4 feet in LOITER for 10 seconds.
        hoverStationary(vehicle, "Second hover (4 ft)", "step4SecondHover10Seconds", flightOutcome)

        # Step 5: Land safely and wait until the motors disarm (propellers stop).
        landingWasStarted = True
        landAndStopPropellers(vehicle, flightOutcome)
        copterWasArmed = vehicle.armed

        if not vehicle.armed:
            flightOutcome["overallOutcome"] = "SUCCESS"
            print("Flight complete.")
        else:
            flightOutcome["overallOutcome"] = "ENDED WITH FAILSAFE LANDING"

    except KeyboardInterrupt:
        print()
        print("Flight interrupted by user (Ctrl+C).")
        flightOutcome["overallOutcome"] = "EXITED MID-WAY (Ctrl+C)"
        flightOutcome["stopReason"] = "User pressed Ctrl+C."

    except Exception as error:
        print(f"Flight stopped: {error}")
        flightOutcome["overallOutcome"] = "FAILED"
        flightOutcome["stopReason"] = str(error)

    finally:
        # If the flight stopped before Step 5 while armed, command LAND now.
        if vehicle is not None and not getattr(vehicle, "commandCableClosed", False):
            if (copterWasArmed or vehicle.armed) and not landingWasStarted:
                print("Requesting emergency LAND.")
                try:
                    landAndStopPropellers(vehicle, flightOutcome)
                except KeyboardInterrupt:
                    print("Second Ctrl+C. Closing the cable without force-disarming.")
                    print("The onboard failsafe remains set to Land.")
                    flightOutcome["step5LandAndStopProps"] = (
                        "SECOND Ctrl+C (Cable closed; onboard Land failsafe active)"
                    )
                except Exception as landingError:
                    print(f"Landing unconfirmed: {landingError}")
                    flightOutcome["step5LandAndStopProps"] = f"UNCONFIRMED ({landingError})"
            try:
                restoreLandingFailsafe(vehicle, savedFailsafe)
            except Exception as restoreError:
                print(f"Could not restore the ground-station failsafe: {restoreError}")
            try:
                restoreTakeoffRamp(vehicle, originalTakeoffSlewSeconds)
            except Exception as restoreError:
                print(f"Could not restore {TAKEOFF_SLEW_PARAMETER_NAME}: {restoreError}")
            closeCommandCable(vehicle)
        elif vehicle is not None:
            print("Failsafe settings were left on the Pixhawk.")
        print("Script finished.")

        printFlightOutcomeSummary(flightOutcome, logPath)
        sys.stdout = originalStandardOutput
        if logFile is not None:
            logFile.close()


if __name__ == "__main__":
    main()
