#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    Take off to about 3 feet, hover in one GPS spot for 30 seconds, and land.

WHY:
    LOITER holds the GPS spot. GUIDED climbs to 3 feet. LAND brings the copter
    down and disarms the motors. This stationary hover test verifies that the
    recalibrated compass and EKF hold a single spot without yaw twist or drift.

HOW:
    1. Connect to the Pixhawk on UART pins 8 and 10.
    2. Wait for Quad X frame, 3D GPS fix, 6 satellites, trusted position, and a level frame.
    3. Ask the pilot to type YES.
    4. Set the onboard ground-station failsafe to Land.
    5. Arm in GUIDED and climb to 3 feet.
    6. Hover in LOITER for 30 seconds (checking ceiling and drift at 5 Hz).
    7. Switch to LAND and wait until the motors disarm.

IMPORTANT:
    Run on the Pi with /home/drone/dronekit-venv/bin/python3.
    /dev/serial0 uses pin 8 (TX), pin 10 (RX), and pin 6 (ground) at 57600 baud.
    Leave both USB ports alone. bootDisplay.py owns the Pixhawk USB port.
    This copter is a DJI F450 saved as Quad X (FRAME_CLASS 1, FRAME_TYPE 1).
    DJI X is FRAME_TYPE 13 and uses a different motor order, so it will not arm.
    The hover target is 3 feet. Reaching 4 feet, or 1.3 meters, starts LAND.
    Drifting more than 8 feet from the hover spot also starts LAND.
    Never force-arm and never force-disarm.
    If the heartbeat is lost, close the cable so the Pixhawk failsafe lands the copter.
    Ctrl+C after arming requests LAND. A second Ctrl+C closes the cable.
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
HOVER_ALTITUDE_FEET = 3.0
HOVER_ALTITUDE_METERS = HOVER_ALTITUDE_FEET / FEET_PER_METER
TAKEOFF_REACHED_FRACTION = 0.95

# 4 feet (1.22 m) is reached before 1.3 m; both ceilings are enforced.
MAXIMUM_HOVER_ALTITUDE_FEET = 4.0
MAXIMUM_HOVER_ALTITUDE_METERS = 1.3
FOUR_FOOT_CEILING_METERS = MAXIMUM_HOVER_ALTITUDE_FEET / FEET_PER_METER

HOVER_DURATION_SECONDS = 30.0

# Land immediately if the copter drifts more than 8 feet horizontally during the hover.
MAXIMUM_HOVER_DRIFT_FEET = 8.0
MAXIMUM_HOVER_DRIFT_METERS = MAXIMUM_HOVER_DRIFT_FEET / FEET_PER_METER
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

# A ramp slower than 1 second leaves one side on the ground long enough to tip.
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
    """True when height reaches 4 feet (1.22 m) or 1.3 meters."""
    if altitudeMeters is None:
        return False
    reachedFourFeet = altitudeMeters >= FOUR_FOOT_CEILING_METERS
    reachedMeterCeiling = altitudeMeters >= MAXIMUM_HOVER_ALTITUDE_METERS
    return reachedFourFeet or reachedMeterCeiling


def latitudeAndLongitude(vehicle):
    """Return (latitudeDegrees, longitudeDegrees) or None if not valid."""
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


def horizontalDriftMeters(
    startLatitudeDegrees,
    startLongitudeDegrees,
    currentLatitudeDegrees,
    currentLongitudeDegrees,
):
    """Return horizontal distance in meters from the hover start spot."""
    northMeters = (currentLatitudeDegrees - startLatitudeDegrees) * METERS_PER_DEGREE_OF_LATITUDE
    eastMeters = (
        (currentLongitudeDegrees - startLongitudeDegrees)
        * METERS_PER_DEGREE_OF_LATITUDE
        * math.cos(math.radians(startLatitudeDegrees))
    )
    return math.hypot(northMeters, eastMeters)


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


def flightStillHealthy(vehicle):
    """Verify heartbeat, arm state, EKF position, and altitude ceiling."""
    if not heartbeatIsFresh(vehicle):
        raise RuntimeError("Pixhawk heartbeat was lost.")
    if not vehicle.armed:
        raise RuntimeError("Copter disarmed.")
    if not vehicle.ekf_ok:
        raise RuntimeError("Position estimate lost.")
    altitudeMeters = heightMeters(vehicle)
    if heightReachedCeiling(altitudeMeters):
        raise RuntimeError(f"Altitude ceiling reached. {heightText(altitudeMeters)}")
    return altitudeMeters


# --------------------------------------------------
# FLIGHT STEPS
# --------------------------------------------------

def takeOff(vehicle):
    """Step 1: Enter GUIDED, arm the motors, and climb to 3 feet."""
    setFlightMode(vehicle, "GUIDED")

    # Normal arm only; ArduPilot still runs all pre-arm checks.
    print("Arming motors.")
    vehicle.armed = True
    startTime = time.monotonic()
    while not vehicle.armed:
        if time.monotonic() - startTime > ARM_TIMEOUT_SECONDS:
            raise RuntimeError("Arming timed out.")
        if not heartbeatIsFresh(vehicle):
            raise RuntimeError("Pixhawk heartbeat was lost while arming.")
        time.sleep(STATUS_CHECK_INTERVAL_SECONDS)
    print("Motors armed.")

    print(f"Taking off to {HOVER_ALTITUDE_FEET:.0f} feet.")
    vehicle.simple_takeoff(HOVER_ALTITUDE_METERS)
    minimumAltitudeMeters = HOVER_ALTITUDE_METERS * TAKEOFF_REACHED_FRACTION
    startTime = time.monotonic()
    lastPrintedAtSeconds = 0.0
    while True:
        if vehicle.mode.name != "GUIDED":
            raise RuntimeError("GUIDED mode lost.")
        altitudeMeters = flightStillHealthy(vehicle)

        currentTimeSeconds = time.monotonic()
        if currentTimeSeconds - lastPrintedAtSeconds >= PRINT_INTERVAL_SECONDS:
            print(f"Altitude: {heightText(altitudeMeters)}")
            lastPrintedAtSeconds = currentTimeSeconds

        if altitudeMeters is not None and altitudeMeters >= minimumAltitudeMeters:
            print("Target altitude reached.")
            return
        if currentTimeSeconds - startTime > TAKEOFF_TIMEOUT_SECONDS:
            raise RuntimeError("Takeoff timed out.")
        time.sleep(STATUS_CHECK_INTERVAL_SECONDS)


def hover(vehicle):
    """Step 2: Hold the GPS spot in LOITER for 30 seconds, checking ceiling and drift at 5 Hz."""
    setFlightMode(vehicle, "LOITER")
    hoverStartSpot = latitudeAndLongitude(vehicle)
    if hoverStartSpot is None:
        raise RuntimeError("GPS position was not available when starting hover.")
    startLatitudeDegrees, startLongitudeDegrees = hoverStartSpot

    print("Hover started.")
    startTime = time.monotonic()
    lastPrintedAtSeconds = 0.0
    while True:
        elapsedSeconds = time.monotonic() - startTime
        if elapsedSeconds >= HOVER_DURATION_SECONDS:
            break
        if vehicle.mode.name != "LOITER":
            raise RuntimeError("LOITER mode lost.")
        altitudeMeters = flightStillHealthy(vehicle)

        currentSpot = latitudeAndLongitude(vehicle)
        if currentSpot is None:
            raise RuntimeError("GPS position was lost during hover.")
        driftMeters = horizontalDriftMeters(
            startLatitudeDegrees,
            startLongitudeDegrees,
            currentSpot[0],
            currentSpot[1],
        )
        driftFeet = driftMeters * FEET_PER_METER
        if driftMeters > MAXIMUM_HOVER_DRIFT_METERS:
            raise RuntimeError(
                f"Hover drifted {driftFeet:.1f} ft from the spot (limit {MAXIMUM_HOVER_DRIFT_FEET:.0f} ft)."
            )

        currentTimeSeconds = time.monotonic()
        if currentTimeSeconds - lastPrintedAtSeconds >= PRINT_INTERVAL_SECONDS:
            remainingSeconds = max(0.0, HOVER_DURATION_SECONDS - elapsedSeconds)
            print(
                f"Hovering | {heightText(altitudeMeters)} | "
                f"drift {driftFeet:.1f} ft | remaining {remainingSeconds:.0f} s"
            )
            lastPrintedAtSeconds = currentTimeSeconds
        time.sleep(STATUS_CHECK_INTERVAL_SECONDS)
    print("30-second hover completed.")


def land(vehicle):
    """Step 3: Enter LAND mode and wait until the motors disarm."""
    if getattr(vehicle, "commandCableClosed", False):
        print("Command cable is already closed. The Pi cannot confirm landing.")
        return
    if not vehicle.armed:
        print("Motors already disarmed.")
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
            return
        if time.monotonic() - startTime > LAND_TIMEOUT_SECONDS:
            raise RuntimeError("Landing not confirmed. The motors are still armed.")
        altitudeMeters = heightMeters(vehicle)
        print(f"Waiting for landing. {heightText(altitudeMeters)}")
        time.sleep(PRINT_INTERVAL_SECONDS)
    print("Landed. Motors disarmed.")


# --------------------------------------------------
# MAIN FLIGHT
# --------------------------------------------------

def main():
    """Connect, confirm, climb to 3 ft, hover 30 s in one spot, land, and save a flight log."""
    originalStandardOutput = sys.stdout
    logFile = None
    logPath = None
    flightLogger = None
    vehicle = None
    originalTakeoffSlewSeconds = None
    savedFailsafe = None
    copterWasArmed = False
    landingWasStarted = False
    flightOutcome = "FAILED"
    stopReason = None

    try:
        logFile, logPath, flightLogger = openFlightLog()

        print("STATIONARY HOVER TEST (30 SECONDS)")
        print(f"Hover altitude: {HOVER_ALTITUDE_FEET:.0f} ft")
        print(f"Hover time: {HOVER_DURATION_SECONDS:.0f} seconds")
        print(f"Ceilings: {MAXIMUM_HOVER_ALTITUDE_FEET:.0f} ft and {MAXIMUM_HOVER_ALTITUDE_METERS:.1f} m.")
        print(f"Maximum drift: {MAXIMUM_HOVER_DRIFT_FEET:.0f} ft from the hover spot.")
        print()

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

        promptText = "Type YES to take off, hover 30 seconds, and land: "
        confirmation = input(promptText)
        if flightLogger is not None:
            flightLogger.writeLogOnly(f"{promptText}{confirmation}\n")
        if confirmation.strip() != "YES":
            print("Flight cancelled. The copter was not armed.")
            flightOutcome = "CANCELLED BEFORE ARMING"
            stopReason = "Pilot did not type YES."
            return

        # Re-verify preflight and configure ramp and onboard Land failsafe.
        waitForPreflight(vehicle)
        originalTakeoffSlewSeconds = shortenSlowTakeoffRamp(vehicle)
        savedFailsafe = enableLandingFailsafe(vehicle)
        problem = preflightProblem(vehicle)
        if problem is not None:
            raise RuntimeError(problem)

        # Step 1: Arm in GUIDED and climb to about 3 feet.
        copterWasArmed = True
        takeOff(vehicle)

        # Step 2: Hold one GPS spot in LOITER for 30 seconds.
        hover(vehicle)

        # Step 3: Land and wait for the motors to disarm.
        landingWasStarted = True
        land(vehicle)
        copterWasArmed = vehicle.armed
        flightOutcome = "SUCCESS" if not vehicle.armed else "ENDED WITH FAILSAFE LANDING"
        print("Flight complete.")

    except KeyboardInterrupt:
        print()
        print("Flight interrupted.")
        flightOutcome = "EXITED MID-WAY (Ctrl+C)"
        stopReason = "User pressed Ctrl+C."

    except Exception as error:
        print(f"Flight stopped: {error}")
        flightOutcome = "FAILED"
        stopReason = str(error)

    finally:
        # If the flight stopped before Step 3, command LAND now.
        if vehicle is not None and not getattr(vehicle, "commandCableClosed", False):
            if (copterWasArmed or vehicle.armed) and not landingWasStarted:
                print("Requesting LAND.")
                try:
                    land(vehicle)
                except KeyboardInterrupt:
                    print("Second Ctrl+C. Closing the cable without force-disarming.")
                    print("The onboard failsafe remains set to Land.")
                except Exception as landingError:
                    print(f"Landing unconfirmed: {landingError}")
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

        print()
        print("=" * 68)
        print(f"STATIONARY HOVER OUTCOME : {flightOutcome}")
        if stopReason:
            print(f"Stop / Failure Reason    : {stopReason}")
        if logPath is not None:
            print(f"Saved Flight Log         : {logPath}")
        print("=" * 68)
        sys.stdout = originalStandardOutput
        if logFile is not None:
            logFile.close()


if __name__ == "__main__":
    main()
