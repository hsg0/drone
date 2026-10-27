#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    1. Take off to 4 feet in height, then climb smoothly to a hover at 5 feet
       and settle for 2 seconds.
    2. Hover stationary at 5 feet for 10 seconds.
    3. Face Geographic North and fly North for 10 feet at 5 feet in height.
    4. Hover stationary at 5 feet for 10 seconds.
    5. Land safely and wait until the propellers stop spinning (motors disarm).
    6. Save a complete flight log and outcome summary of every run in
       oct26-2026/flightLog/.

HOW:
    1. Open a timestamped log file in oct26-2026/flightLog/.
    2. Connect to the Pixhawk on /dev/serial0 (pins 8 and 10 at 57600 baud).
    3. Verify Quad X frame, 3D GPS fix with at least 6 satellites, trusted EKF
       horizontal position, Pixhawk PreArm health (SYS_STATUS bit 28), and
       relaxed compass variance tolerance so minor compass variance does not block.
    4. Ask the pilot to type YES, shorten TKOFF_SLEW_TIME to 1.0s if slower, and
       enable the onboard Land failsafe (FS_GCS_ENABLE=5, FS_OPTIONS bit 4 cleared).
    5. Step 1: Arm in GUIDED, take off to 4 feet, then climb to 5 feet and settle
       for 2 seconds in GUIDED before switching to LOITER.
    6. Step 2: Hold stationary in LOITER at 5 feet for 10 seconds.
    7. Step 3: Face Geographic North (0 deg), fly North 10 feet in GUIDED at
       0.4 m/s while holding 5 feet height, then command zero speed for 2 seconds
       to stop cleanly.
    8. Step 4: Hold stationary in LOITER at 5 feet for 10 seconds.
    9. Step 5: Switch to LAND and wait until the motors disarm (propellers stop).

IMPORTANT:
    Run on the Pi with:
        /home/drone/Desktop/scripts/oct26-2026/fly10FeetNorth.py
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

PIXHAWK_PORT = "/dev/serial0"
PIXHAWK_BAUD = 57600
CONNECTION_TIMEOUT_SECONDS = 60
GROUND_STATION_SYSTEM_ID = 255

FLIGHT_LOG_DIRECTORY = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "flightLog",
)

FEET_PER_METER = 3.28084

# Step 1: Take off to 4.0 feet, then come to hover at 5.0 feet
INITIAL_TAKEOFF_ALTITUDE_FEET = 4.0
INITIAL_TAKEOFF_ALTITUDE_METERS = INITIAL_TAKEOFF_ALTITUDE_FEET / FEET_PER_METER
HOVER_ALTITUDE_FEET = 5.0
HOVER_ALTITUDE_METERS = HOVER_ALTITUDE_FEET / FEET_PER_METER
TAKEOFF_REACHED_FRACTION = 0.95
TAKEOFF_SETTLE_SECONDS = 2.0

# Safety ceiling above the 5.0-ft hover target (10.0 feet so normal baro bounce never aborts)
MAXIMUM_ALTITUDE_CEILING_FEET = 10.0
MAXIMUM_ALTITUDE_CEILING_METERS = MAXIMUM_ALTITUDE_CEILING_FEET / FEET_PER_METER

HOVER_DURATION_SECONDS = 10.0
MAXIMUM_HOVER_DRIFT_FEET = 16.0
MAXIMUM_HOVER_DRIFT_METERS = MAXIMUM_HOVER_DRIFT_FEET / FEET_PER_METER

NORTH_DISTANCE_FEET = 10.0
NORTH_DISTANCE_METERS = NORTH_DISTANCE_FEET / FEET_PER_METER
NORTH_SPEED_METERS_PER_SECOND = 0.4
ALTITUDE_HOLD_GAIN_DURING_MOVE = 0.5
MAXIMUM_VERTICAL_CORRECTION_METERS_PER_SECOND = 0.25
MAXIMUM_OFF_COURSE_FEET = 16.0
MAXIMUM_OFF_COURSE_METERS = MAXIMUM_OFF_COURSE_FEET / FEET_PER_METER
NORTH_MOVE_TIMEOUT_SECONDS = 30.0
SETTLE_AFTER_MOVE_SECONDS = 2.0
NORTH_HEADING_DEGREES = 0.0
NORTH_HEADING_TOLERANCE_DEGREES = 15.0
YAW_TURN_RATE_DEGREES_PER_SECOND = 30.0
YAW_ALIGN_TIMEOUT_SECONDS = 15.0
YAW_SETTLE_SECONDS = 1.0
METERS_PER_DEGREE_OF_LATITUDE = 111319.5
MINIMUM_VALID_COORDINATE_DEGREES = 0.01

TAKEOFF_TIMEOUT_SECONDS = 30.0
LAND_TIMEOUT_SECONDS = 60.0
MODE_CHANGE_TIMEOUT_SECONDS = 10.0
ARM_TIMEOUT_SECONDS = 15.0
HEARTBEAT_STALE_AFTER_SECONDS = 3.0
MAXIMUM_GROUND_LEAN_DEGREES = 10.0
MAXIMUM_SAVED_TRIM_DEGREES = 8.0

STATUS_CHECK_INTERVAL_SECONDS = 0.2
PRINT_INTERVAL_SECONDS = 1.0
PARAMETER_TOLERANCE = 0.01

QUAD_FRAME_CLASS = 1
QUAD_X_FRAME_TYPE = 1
MINIMUM_GPS_FIX_TYPE = 3
MINIMUM_SATELLITE_COUNT = 5
UNKNOWN_SATELLITE_COUNT = 255
SKIP_COMPASS_OR_GPS_MASK = 4 | 8 | 4096

# Relaxed compass variance tolerance so minor differences between primary & secondary do not block takeoff:
MAXIMUM_COMPASS_VARIANCE = 0.50

PREARM_CHECK_HEALTH_BIT = getattr(
    middleMan.mavlink,
    "MAV_SYS_STATUS_PREARM_CHECK",
    268435456,
)
RUN_PREARM_CHECKS_COMMAND_ID = getattr(
    middleMan.mavlink,
    "MAV_CMD_RUN_PREARM_CHECKS",
    401,
)
PREARM_POLL_INTERVAL_SECONDS = 3.0

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
    "RAW_IMU",
    "SCALED_IMU2",
    "SYS_STATUS",
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


# --------------------------------------------------
# TERMINAL AND FILE LOGGING TEE
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
        self.logFile.write(text)
        self.logFile.flush()

    def flush(self):
        self.terminal.flush()
        self.logFile.flush()


# --------------------------------------------------
# SMALL CORE PIXHAWK PRIMITIVES
# --------------------------------------------------

def heartbeatIsFresh(vehicle):
    heartbeatAgeSeconds = vehicle.last_heartbeat
    if heartbeatAgeSeconds is None:
        return False
    return heartbeatAgeSeconds <= HEARTBEAT_STALE_AFTER_SECONDS


def setFlightMode(vehicle, flightModeName):
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


def angleFromNorthDegrees(headingDegrees):
    return (float(headingDegrees) - NORTH_HEADING_DEGREES + 180.0) % 360.0 - 180.0


def commandFaceGeographicNorth(vehicle):
    currentYawDeg = math.degrees(float(vehicle.attitude.yaw or 0.0))
    errorFromNorthDeg = angleFromNorthDegrees(currentYawDeg)
    turnDirection = -1 if errorFromNorthDeg > 0 else 1
    yawMessage = vehicle.message_factory.command_long_encode(
        vehicle._master.target_system,
        vehicle._master.target_component,
        middleMan.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        NORTH_HEADING_DEGREES,
        YAW_TURN_RATE_DEGREES_PER_SECOND,
        turnDirection,
        0,
        0, 0, 0,
    )
    vehicle.send_mavlink(yawMessage)
    vehicle.flush()


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


def checkFlightHealthAndCeiling(vehicle, flightOutcome):
    if not heartbeatIsFresh(vehicle):
        raise RuntimeError("Pixhawk heartbeat was lost.")
    if not vehicle.armed:
        raise RuntimeError("Copter disarmed unexpectedly in flight.")
    if not vehicle.ekf_ok:
        raise RuntimeError("Position estimate lost in flight.")
    location = vehicle.location.global_relative_frame
    altitudeMeters = None
    if location is not None and location.alt is not None and math.isfinite(float(location.alt)):
        altitudeMeters = float(location.alt)
        previousMax = getattr(vehicle, "maximumAltitudeMeters", None)
        if previousMax is None or altitudeMeters > previousMax:
            vehicle.maximumAltitudeMeters = altitudeMeters
            flightOutcome["maximumAltitudeReachedText"] = (
                f"{altitudeMeters:.2f} m ({altitudeMeters * FEET_PER_METER:.2f} ft)"
            )
        if altitudeMeters >= MAXIMUM_ALTITUDE_CEILING_METERS:
            raise RuntimeError(
                f"Safety ceiling ({MAXIMUM_ALTITUDE_CEILING_FEET:.1f} ft) reached: "
                f"{altitudeMeters:.2f} m ({altitudeMeters * FEET_PER_METER:.2f} ft)"
            )
    return altitudeMeters


# --------------------------------------------------
# MAIN INLINE FLIGHT WALKTHROUGH
# --------------------------------------------------

def main():
    originalStandardOutput = sys.stdout
    logFile = None
    logPath = None
    flightLogger = None
    vehicle = None
    originalTakeoffSlewSeconds = None
    savedFailsafe = None
    copterWasArmed = False
    landingWasStarted = False

    flightOutcome = {
        "startedAt": time.strftime("%Y-%m-%d %H:%M:%S %Z"),
        "finishedAt": None,
        "overallOutcome": "FAILED",
        "preflightAndFailsafe": "NOT STARTED",
        "step1Takeoff4FtToHover5Ft": "NOT STARTED",
        "step2FirstHover5Ft10Seconds": "NOT STARTED",
        "step3FlyNorth10FeetAt5Ft": "NOT STARTED",
        "step4SecondHover5Ft10Seconds": "NOT STARTED",
        "step5LandAndStopProps": "NOT STARTED",
        "maximumAltitudeReachedText": "none",
        "northDistanceTraveledText": "0.0 ft",
        "stopReason": None,
    }

    try:
        # ==================================================
        # OPEN FLIGHT LOG FILE
        # ==================================================
        os.makedirs(FLIGHT_LOG_DIRECTORY, exist_ok=True)
        scriptName = os.path.splitext(os.path.basename(__file__))[0]
        startedAtText = time.strftime("%Y-%m-%d-%H%M%S")
        logPath = os.path.join(FLIGHT_LOG_DIRECTORY, f"{scriptName}-{startedAtText}.log")
        logFile = open(logPath, "w", encoding="utf-8")
        flightLogger = FlightLog(logFile, sys.stdout)
        sys.stdout = flightLogger

        print(f"Flight log file : {logPath}")
        print(f"Run started at  : {time.strftime('%Y-%m-%d %H:%M:%S %Z')}")
        print("====================================================================")
        print("TAKE OFF TO 4 FT -> HOVER AT 5 FT (10S) -> FLY NORTH 10 FT AT 5 FT -> HOVER 5 FT (10S) -> LAND")
        print("====================================================================")
        print(f"Initial takeoff step : {INITIAL_TAKEOFF_ALTITUDE_FEET:.1f} ft ({INITIAL_TAKEOFF_ALTITUDE_METERS:.2f} m)")
        print(f"Hover & flight height: {HOVER_ALTITUDE_FEET:.1f} ft ({HOVER_ALTITUDE_METERS:.2f} m)")
        print(f"Safety ceiling       : {MAXIMUM_ALTITUDE_CEILING_FEET:.1f} ft ({MAXIMUM_ALTITUDE_CEILING_METERS:.2f} m)")
        print(f"Stationary hovers    : {HOVER_DURATION_SECONDS:.0f} seconds at {HOVER_ALTITUDE_FEET:.0f} ft, twice")
        print(f"North leg            : {NORTH_DISTANCE_FEET:.0f} ft North at {NORTH_SPEED_METERS_PER_SECOND:.1f} m/s ({HOVER_ALTITUDE_FEET:.0f} ft height)")
        print()

        # ==================================================
        # CONNECT TO PIXHAWK AND STREAM TELEMETRY
        # ==================================================
        if not os.path.exists(PIXHAWK_PORT):
            raise RuntimeError(f"{PIXHAWK_PORT} is missing. Run this script on the Raspberry Pi.")

        print(f"Connecting to Pixhawk on {PIXHAWK_PORT} at {PIXHAWK_BAUD} baud (downloading parameters)...")
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

        latestTelemetry = {
            "compass1": None,
            "compass2": None,
            "compassVariance": None,
            "prearmHealthy": None,
            "lastPrearmReason": None,
            "lastPrearmPollTime": 0.0,
        }

        @vehicle.on_message("RAW_IMU")
        def onRawImu(_vehicle, _name, message):
            latestTelemetry["compass1"] = (int(message.xmag), int(message.ymag), int(message.zmag))

        @vehicle.on_message("SCALED_IMU2")
        def onScaledImu2(_vehicle, _name, message):
            latestTelemetry["compass2"] = (int(message.xmag), int(message.ymag), int(message.zmag))

        @vehicle.on_message("EKF_STATUS_REPORT")
        def onEkfStatusReport(_vehicle, _name, message):
            latestTelemetry["compassVariance"] = float(message.compass_variance)

        @vehicle.on_message("SYS_STATUS")
        def onSysStatus(_vehicle, _name, message):
            healthMask = int(getattr(message, "onboard_control_sensors_health", 0))
            isHealthy = bool(healthMask & PREARM_CHECK_HEALTH_BIT)
            latestTelemetry["prearmHealthy"] = isHealthy
            if isHealthy:
                latestTelemetry["lastPrearmReason"] = None

        @vehicle.on_message("STATUSTEXT")
        def onStatusText(_vehicle, _name, message):
            text = getattr(message, "text", "")
            if isinstance(text, bytes):
                text = text.decode("ascii", errors="ignore")
            text = str(text).strip("\x00").strip()
            if text:
                if "prearm" in text.lower():
                    latestTelemetry["lastPrearmReason"] = text
                print(f"  Pixhawk: {text}")

        intervalMicroseconds = int(MESSAGE_INTERVAL_SECONDS * MICROSECONDS_PER_SECOND)
        for messageName in FLIGHT_MESSAGE_NAMES:
            messageId = getattr(middleMan.mavlink, f"MAVLINK_MSG_ID_{messageName}", None)
            if messageId is not None:
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
        print("Requested GPS, height, attitude, and position-health streams.")

        # ==================================================
        # PREFLIGHT CHECKS
        # ==================================================
        print("Checking preflight health (Frame, Compass, GPS, EKF, PreArm, Attitude)...")
        while True:
            blocker = None

            if not heartbeatIsFresh(vehicle):
                blocker = "Pixhawk heartbeat is missing."

            if blocker is None:
                frameClass = vehicle.parameters["FRAME_CLASS"]
                frameType = vehicle.parameters["FRAME_TYPE"]
                if frameClass is None or frameType is None:
                    blocker = "FRAME_CLASS or FRAME_TYPE could not be read."
                elif (
                    abs(float(frameClass) - QUAD_FRAME_CLASS) >= PARAMETER_TOLERANCE
                    or abs(float(frameType) - QUAD_X_FRAME_TYPE) >= PARAMETER_TOLERANCE
                ):
                    blocker = f"Quad X required. Found FRAME_CLASS={frameClass}, FRAME_TYPE={frameType}."

            if blocker is None:
                skipCheckValue = vehicle.parameters["ARMING_SKIPCHK"]
                if skipCheckValue is not None and (int(round(float(skipCheckValue))) & SKIP_COMPASS_OR_GPS_MASK) != 0:
                    raise RuntimeError(
                        f"ARMING_SKIPCHK is {int(round(float(skipCheckValue)))} (skipping Compass/GPS checks). "
                        "Run oct26-2026/newReCalibrationOfCompasses.py first."
                    )

            if blocker is None:
                useCompass1 = int(round(float(vehicle.parameters["COMPASS_USE"] or 0)))
                useCompass2 = int(round(float(vehicle.parameters["COMPASS_USE2"] or 0)))
                if useCompass1 == 0 and useCompass2 == 0:
                    raise RuntimeError("Both COMPASS_USE and COMPASS_USE2 are 0. Run newReCalibrationOfCompasses.py first.")
                if blocker is None and latestTelemetry["compassVariance"] is not None:
                    if latestTelemetry["compassVariance"] > MAXIMUM_COMPASS_VARIANCE:
                        blocker = (
                            f"EKF compass variance is {latestTelemetry['compassVariance']:.3f} "
                            f"(needs <= {MAXIMUM_COMPASS_VARIANCE:.2f})."
                        )

            if blocker is None:
                trimX = vehicle.parameters["AHRS_TRIM_X"]
                trimY = vehicle.parameters["AHRS_TRIM_Y"]
                if trimX is not None and trimY is not None:
                    trimRollDeg = math.degrees(float(trimX))
                    trimPitchDeg = math.degrees(float(trimY))
                    if abs(trimRollDeg) > MAXIMUM_SAVED_TRIM_DEGREES or abs(trimPitchDeg) > MAXIMUM_SAVED_TRIM_DEGREES:
                        raise RuntimeError(
                            f"Saved level trim is tilted (Roll {trimRollDeg:+.2f} deg, Pitch {trimPitchDeg:+.2f} deg; "
                            f"max {MAXIMUM_SAVED_TRIM_DEGREES:.1f} deg)."
                        )

            if blocker is None:
                gpsReading = vehicle.gps_0
                if gpsReading is None or gpsReading.fix_type is None or int(gpsReading.fix_type) < MINIMUM_GPS_FIX_TYPE:
                    blocker = "GPS 3D fix required."
                else:
                    satellites = gpsReading.satellites_visible
                    if satellites is None or satellites == UNKNOWN_SATELLITE_COUNT or satellites < MINIMUM_SATELLITE_COUNT:
                        blocker = f"At least {MINIMUM_SATELLITE_COUNT} satellites required. Seeing {satellites}."

            if blocker is None:
                if not vehicle.ekf_ok:
                    satellites = vehicle.gps_0.satellites_visible
                    if vehicle._ekf_constposmode:
                        blocker = (
                            f"3D fix, {satellites} satellites. "
                            "EKF3 is still aligning GPS and compass (keep waiting ~30-60s)."
                        )
                    else:
                        blocker = f"3D fix, {satellites} satellites. Horizontal position is not trusted yet."

            if blocker is None:
                if not vehicle.is_armable or latestTelemetry["prearmHealthy"] is not True:
                    nowPoll = time.monotonic()
                    if nowPoll - latestTelemetry["lastPrearmPollTime"] >= PREARM_POLL_INTERVAL_SECONDS:
                        prearmCheckMessage = vehicle.message_factory.command_long_encode(
                            vehicle._master.target_system,
                            vehicle._master.target_component,
                            RUN_PREARM_CHECKS_COMMAND_ID,
                            0,
                            0, 0, 0, 0, 0, 0, 0,
                        )
                        vehicle.send_mavlink(prearmCheckMessage)
                        vehicle.flush()
                        latestTelemetry["lastPrearmPollTime"] = nowPoll
                    reasonText = latestTelemetry["lastPrearmReason"] or "waiting for Pixhawk PreArm status..."
                    blocker = f"Pixhawk pre-arm check blocked ({reasonText})."

            if blocker is None:
                if vehicle.attitude is None or vehicle.attitude.roll is None or vehicle.attitude.pitch is None:
                    blocker = "Attitude is not available yet."
                else:
                    rollDegrees = math.degrees(vehicle.attitude.roll)
                    pitchDegrees = math.degrees(vehicle.attitude.pitch)
                    if abs(rollDegrees) > MAXIMUM_GROUND_LEAN_DEGREES or abs(pitchDegrees) > MAXIMUM_GROUND_LEAN_DEGREES:
                        blocker = f"Copter is not sitting level. Roll {rollDegrees:+.1f} deg, pitch {pitchDegrees:+.1f} deg."

            if blocker is None:
                trimRollDeg = math.degrees(float(vehicle.parameters["AHRS_TRIM_X"] or 0.0))
                trimPitchDeg = math.degrees(float(vehicle.parameters["AHRS_TRIM_Y"] or 0.0))
                headingDeg = math.degrees(float(vehicle.attitude.yaw)) if vehicle.attitude.yaw is not None else 0.0
                print("Preflight checks passed.")
                print(
                    f"FRAME_CLASS {float(vehicle.parameters['FRAME_CLASS']):.0f} | "
                    f"FRAME_TYPE {float(vehicle.parameters['FRAME_TYPE']):.0f} | "
                    f"satellites {vehicle.gps_0.satellites_visible} | "
                    f"heading {headingDeg:+.1f} deg | "
                    f"saved trim ({trimRollDeg:+.2f}, {trimPitchDeg:+.2f}) deg."
                )
                break

            print(f"Waiting. {blocker}")
            time.sleep(PRINT_INTERVAL_SECONDS)

        # ==================================================
        # ASK PILOT TO TYPE YES AND CONFIGURE FAILSAFE
        # ==================================================
        promptText = (
            "Type YES to take off to 4 ft, come to hover at 5 ft for 10s, "
            "fly 10 ft North at 5 ft, hover at 5 ft for 10s, and land: "
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

        # Shorten TKOFF_SLEW_TIME to 1.0s if slower so the drone lifts off cleanly.
        savedSlew = vehicle.parameters[TAKEOFF_SLEW_PARAMETER_NAME]
        if savedSlew is not None:
            savedSlew = float(savedSlew)
            takeoffSlew = min(savedSlew, PREFERRED_TAKEOFF_SLEW_SECONDS)
            print(f"{TAKEOFF_SLEW_PARAMETER_NAME}: original {savedSlew:.2f}s | takeoff {takeoffSlew:.2f}s")
            if abs(takeoffSlew - savedSlew) >= PARAMETER_TOLERANCE:
                setParameterAndConfirm(vehicle, TAKEOFF_SLEW_PARAMETER_NAME, takeoffSlew)
                originalTakeoffSlewSeconds = savedSlew

        # Enable onboard Land failsafe (FS_GCS_ENABLE=5, FS_OPTIONS bit 4 cleared).
        savedEnable = float(vehicle.parameters[GROUND_STATION_FAILSAFE_PARAMETER_NAME])
        savedTimeout = float(vehicle.parameters[GROUND_STATION_FAILSAFE_TIMEOUT_PARAMETER_NAME])
        savedOptions = float(vehicle.parameters[FAILSAFE_OPTIONS_PARAMETER_NAME])
        timeoutSeconds = savedTimeout if (0 < savedTimeout <= PREFERRED_FAILSAFE_TIMEOUT_SECONDS) else PREFERRED_FAILSAFE_TIMEOUT_SECONDS
        savedOptionsInt = int(round(savedOptions))
        flightOptions = savedOptionsInt & ~CONTINUE_PILOT_MODE_ON_FAILSAFE_BIT
        savedFailsafe = (savedEnable, savedTimeout, savedOptions)

        setParameterAndConfirm(vehicle, GROUND_STATION_FAILSAFE_PARAMETER_NAME, GROUND_STATION_FAILSAFE_LAND_VALUE)
        setParameterAndConfirm(vehicle, GROUND_STATION_FAILSAFE_TIMEOUT_PARAMETER_NAME, timeoutSeconds)
        setParameterAndConfirm(vehicle, FAILSAFE_OPTIONS_PARAMETER_NAME, flightOptions)
        vehicle.groundStationFailsafeTimeoutSeconds = timeoutSeconds
        print("Onboard failsafe is set to Land if the Pi connection is lost.")

        # Re-verify every preflight gate right after the YES prompt.
        if not heartbeatIsFresh(vehicle):
            raise RuntimeError("Pixhawk heartbeat was lost while waiting at the YES prompt.")
        gpsReadingAfterYes = vehicle.gps_0
        if (
            gpsReadingAfterYes is None
            or gpsReadingAfterYes.fix_type is None
            or int(gpsReadingAfterYes.fix_type) < MINIMUM_GPS_FIX_TYPE
            or gpsReadingAfterYes.satellites_visible is None
            or gpsReadingAfterYes.satellites_visible == UNKNOWN_SATELLITE_COUNT
            or gpsReadingAfterYes.satellites_visible < MINIMUM_SATELLITE_COUNT
        ):
            raise RuntimeError(f"GPS 3D fix or satellite count dropped below {MINIMUM_SATELLITE_COUNT} while waiting at the YES prompt.")
        if not vehicle.ekf_ok or not vehicle.is_armable or latestTelemetry["prearmHealthy"] is not True:
            reasonText = latestTelemetry["lastPrearmReason"] or "EKF or PreArm check not ready"
            raise RuntimeError(f"Preflight check became unready while waiting at the YES prompt ({reasonText}).")
        if vehicle.attitude is None or vehicle.attitude.roll is None or vehicle.attitude.pitch is None:
            raise RuntimeError("Attitude became unavailable while waiting at the YES prompt.")
        rollAfterYes = math.degrees(vehicle.attitude.roll)
        pitchAfterYes = math.degrees(vehicle.attitude.pitch)
        if abs(rollAfterYes) > MAXIMUM_GROUND_LEAN_DEGREES or abs(pitchAfterYes) > MAXIMUM_GROUND_LEAN_DEGREES:
            raise RuntimeError(
                f"Copter is no longer level (Roll {rollAfterYes:+.1f} deg, Pitch {pitchAfterYes:+.1f} deg)."
            )

        flightOutcome["preflightAndFailsafe"] = "SUCCESS (Preflight passed, FS_GCS_ENABLE=5 Land set)"

        # ==================================================
        # STEP 1: ARM IN GUIDED, TAKE OFF TO 4 FEET, THEN CLIMB TO HOVER AT 5 FEET
        # ==================================================
        flightOutcome["step1Takeoff4FtToHover5Ft"] = "IN PROGRESS"
        setFlightMode(vehicle, "GUIDED")

        print("Arming motors.")
        copterWasArmed = True
        vehicle.armed = True
        armStartTime = time.monotonic()
        while not vehicle.armed:
            if time.monotonic() - armStartTime > ARM_TIMEOUT_SECONDS:
                flightOutcome["step1Takeoff4FtToHover5Ft"] = "FAILED (Arming timed out)"
                raise RuntimeError("Arming timed out.")
            if not heartbeatIsFresh(vehicle):
                flightOutcome["step1Takeoff4FtToHover5Ft"] = "FAILED (Heartbeat lost while arming)"
                raise RuntimeError("Pixhawk heartbeat was lost while arming.")
            time.sleep(STATUS_CHECK_INTERVAL_SECONDS)
        print("Motors armed. Propellers spinning.")

        # Phase 1A: Initial takeoff to 4.0 feet (with 5.0-ft target commanded to Simple Takeoff so climb continues smoothly to 5.0 ft)
        print(
            f"Taking off to {INITIAL_TAKEOFF_ALTITUDE_FEET:.1f} feet ({INITIAL_TAKEOFF_ALTITUDE_METERS:.2f} m), "
            f"then coming to hover at {HOVER_ALTITUDE_FEET:.1f} feet ({HOVER_ALTITUDE_METERS:.2f} m)."
        )
        vehicle.simple_takeoff(HOVER_ALTITUDE_METERS)
        minimumFourFootMeters = INITIAL_TAKEOFF_ALTITUDE_METERS * TAKEOFF_REACHED_FRACTION
        minimumFiveFootMeters = HOVER_ALTITUDE_METERS * TAKEOFF_REACHED_FRACTION
        takeoffStartTime = time.monotonic()
        lastPrintTime = 0.0
        passedFourFeet = False

        while True:
            if vehicle.mode.name != "GUIDED":
                flightOutcome["step1Takeoff4FtToHover5Ft"] = "FAILED (GUIDED mode lost)"
                raise RuntimeError("GUIDED mode lost during takeoff.")
            altitudeMeters = checkFlightHealthAndCeiling(vehicle, flightOutcome)

            nowTime = time.monotonic()
            if nowTime - lastPrintTime >= PRINT_INTERVAL_SECONDS:
                heightString = "waiting" if altitudeMeters is None else f"{altitudeMeters:.2f} m ({altitudeMeters * FEET_PER_METER:.2f} ft)"
                stageLabel = "Climbing to 4 ft takeoff point" if not passedFourFeet else "Climbing from 4 ft to 5 ft hover height"
                print(f"{stageLabel} | {heightString}")
                lastPrintTime = nowTime

            if not passedFourFeet and altitudeMeters is not None and altitudeMeters >= minimumFourFootMeters:
                passedFourFeet = True
                print(
                    f"Reached 4-foot takeoff height ({altitudeMeters:.2f} m / {altitudeMeters * FEET_PER_METER:.2f} ft) "
                    f"— rising to 5-foot hover height..."
                )

            if altitudeMeters is not None and altitudeMeters >= minimumFiveFootMeters:
                print(f"Reached 5-foot hover altitude: {altitudeMeters:.2f} m ({altitudeMeters * FEET_PER_METER:.2f} ft).")
                break

            if nowTime - takeoffStartTime > TAKEOFF_TIMEOUT_SECONDS:
                flightOutcome["step1Takeoff4FtToHover5Ft"] = "FAILED (Takeoff timed out)"
                raise RuntimeError("Takeoff timed out.")
            time.sleep(STATUS_CHECK_INTERVAL_SECONDS)

        # Settle for 2 seconds in GUIDED at 5 feet so vertical climb momentum stops before switching to LOITER.
        print(f"Settling at 5 feet in GUIDED for {TAKEOFF_SETTLE_SECONDS:.0f} seconds before switching to LOITER...")
        settleStart = time.monotonic()
        while time.monotonic() - settleStart < TAKEOFF_SETTLE_SECONDS:
            if vehicle.mode.name != "GUIDED":
                flightOutcome["step1Takeoff4FtToHover5Ft"] = "FAILED (GUIDED lost during settle)"
                raise RuntimeError("GUIDED mode lost during takeoff settle.")
            altitudeMeters = checkFlightHealthAndCeiling(vehicle, flightOutcome)
            time.sleep(STATUS_CHECK_INTERVAL_SECONDS)
        flightOutcome["step1Takeoff4FtToHover5Ft"] = (
            f"SUCCESS ({altitudeMeters:.2f} m ({altitudeMeters * FEET_PER_METER:.2f} ft))"
        )

        # ==================================================
        # STEP 2: FIRST STATIONARY HOVER AT 5 FEET FOR 10 SECONDS (LOITER)
        # ==================================================
        flightOutcome["step2FirstHover5Ft10Seconds"] = "IN PROGRESS"
        setFlightMode(vehicle, "LOITER")

        firstHoverLocation = vehicle.location.global_relative_frame
        if (
            firstHoverLocation is None
            or firstHoverLocation.lat is None
            or firstHoverLocation.lon is None
            or (abs(float(firstHoverLocation.lat)) < MINIMUM_VALID_COORDINATE_DEGREES and abs(float(firstHoverLocation.lon)) < MINIMUM_VALID_COORDINATE_DEGREES)
        ):
            flightOutcome["step2FirstHover5Ft10Seconds"] = "FAILED (No GPS spot at hover start)"
            raise RuntimeError("GPS position was not available for First hover.")
        firstHoverStartLat = float(firstHoverLocation.lat)
        firstHoverStartLon = float(firstHoverLocation.lon)

        print(f"First hover (5 ft) started (holding 5 ft for {HOVER_DURATION_SECONDS:.0f} seconds).")
        hoverStartTime = time.monotonic()
        lastPrintTime = 0.0
        firstHoverMaxDriftFeet = 0.0
        while True:
            elapsedHover = time.monotonic() - hoverStartTime
            if elapsedHover >= HOVER_DURATION_SECONDS:
                break
            if vehicle.mode.name != "LOITER":
                flightOutcome["step2FirstHover5Ft10Seconds"] = "FAILED (LOITER mode lost)"
                raise RuntimeError("LOITER mode lost during First hover.")
            altitudeMeters = checkFlightHealthAndCeiling(vehicle, flightOutcome)

            currentLoc = vehicle.location.global_relative_frame
            if currentLoc is None or currentLoc.lat is None or currentLoc.lon is None:
                flightOutcome["step2FirstHover5Ft10Seconds"] = "FAILED (GPS position lost)"
                raise RuntimeError("GPS position was lost during First hover.")
            northOffsetMeters = (float(currentLoc.lat) - firstHoverStartLat) * METERS_PER_DEGREE_OF_LATITUDE
            eastOffsetMeters = (
                (float(currentLoc.lon) - firstHoverStartLon)
                * METERS_PER_DEGREE_OF_LATITUDE
                * math.cos(math.radians(firstHoverStartLat))
            )
            driftMeters = math.hypot(northOffsetMeters, eastOffsetMeters)
            driftFeet = driftMeters * FEET_PER_METER
            if driftFeet > firstHoverMaxDriftFeet:
                firstHoverMaxDriftFeet = driftFeet
            if driftMeters > MAXIMUM_HOVER_DRIFT_METERS:
                flightOutcome["step2FirstHover5Ft10Seconds"] = f"FAILED (Drifted {driftFeet:.1f} ft)"
                raise RuntimeError(
                    f"First hover (5 ft) drifted {driftFeet:.1f} ft from its spot "
                    f"(limit {MAXIMUM_HOVER_DRIFT_FEET:.0f} ft)."
                )

            nowTime = time.monotonic()
            if nowTime - lastPrintTime >= PRINT_INTERVAL_SECONDS:
                remainingSeconds = max(0.0, HOVER_DURATION_SECONDS - elapsedHover)
                print(
                    f"First hover (5 ft) | {altitudeMeters:.2f} m ({altitudeMeters * FEET_PER_METER:.2f} ft) | "
                    f"drift {driftFeet:.1f} ft | remaining {remainingSeconds:.0f} s"
                )
                lastPrintTime = nowTime
            time.sleep(STATUS_CHECK_INTERVAL_SECONDS)

        print("First hover (5 ft) completed.")
        flightOutcome["step2FirstHover5Ft10Seconds"] = f"SUCCESS (10s completed, max drift {firstHoverMaxDriftFeet:.1f} ft)"

        # ==================================================
        # STEP 3: FACE GEOGRAPHIC NORTH AND FLY 10 FEET NORTH AT 5 FEET HEIGHT (GUIDED)
        # ==================================================
        flightOutcome["step3FlyNorth10FeetAt5Ft"] = "IN PROGRESS"
        setFlightMode(vehicle, "GUIDED")

        # Ensure the nose is facing Geographic North (0 deg) before flying 10 feet North.
        currentYawDeg = math.degrees(float(vehicle.attitude.yaw or 0.0))
        yawErrorDeg = angleFromNorthDegrees(currentYawDeg)
        if abs(yawErrorDeg) > NORTH_HEADING_TOLERANCE_DEGREES:
            print(
                f"Turning nose to face Geographic North (0 deg) from current heading {currentYawDeg:+.1f} deg..."
            )
            commandFaceGeographicNorth(vehicle)
            yawStartTime = time.monotonic()
            lastYawPrintTime = 0.0
            while True:
                if vehicle.mode.name != "GUIDED":
                    flightOutcome["step3FlyNorth10FeetAt5Ft"] = "FAILED (GUIDED lost while turning North)"
                    raise RuntimeError("GUIDED mode lost while turning to face North.")
                altitudeMeters = checkFlightHealthAndCeiling(vehicle, flightOutcome)
                downHold = 0.0
                if altitudeMeters is not None:
                    altErr = HOVER_ALTITUDE_METERS - altitudeMeters
                    climb = max(
                        -MAXIMUM_VERTICAL_CORRECTION_METERS_PER_SECOND,
                        min(MAXIMUM_VERTICAL_CORRECTION_METERS_PER_SECOND, altErr * ALTITUDE_HOLD_GAIN_DURING_MOVE),
                    )
                    downHold = -climb
                sendLocalVelocity(vehicle, 0.0, 0.0, downHold)
                commandFaceGeographicNorth(vehicle)

                currentYawDeg = math.degrees(float(vehicle.attitude.yaw or 0.0))
                yawErrorDeg = angleFromNorthDegrees(currentYawDeg)
                if abs(yawErrorDeg) <= NORTH_HEADING_TOLERANCE_DEGREES:
                    print(f"Nose aligned with Geographic North (heading {currentYawDeg:+.1f} deg).")
                    break
                nowYaw = time.monotonic()
                if nowYaw - lastYawPrintTime >= PRINT_INTERVAL_SECONDS:
                    print(f"Turning to North | current heading {currentYawDeg:+.1f} deg | remaining turn {abs(yawErrorDeg):.1f} deg")
                    lastYawPrintTime = nowYaw
                if nowYaw - yawStartTime > YAW_ALIGN_TIMEOUT_SECONDS:
                    print(f"Yaw turn time limit reached at {currentYawDeg:+.1f} deg — continuing North move in Earth frame.")
                    break
                time.sleep(STATUS_CHECK_INTERVAL_SECONDS)

            yawSettleStart = time.monotonic()
            while time.monotonic() - yawSettleStart < YAW_SETTLE_SECONDS:
                altitudeMeters = checkFlightHealthAndCeiling(vehicle, flightOutcome)
                sendLocalVelocity(vehicle, 0.0, 0.0, 0.0)
                time.sleep(STATUS_CHECK_INTERVAL_SECONDS)
        else:
            print(f"Nose is already facing Geographic North (heading {currentYawDeg:+.1f} deg).")

        northStartLocation = vehicle.location.global_relative_frame
        if northStartLocation is None or northStartLocation.lat is None or northStartLocation.lon is None:
            flightOutcome["step3FlyNorth10FeetAt5Ft"] = "FAILED (No GPS at start of North move)"
            raise RuntimeError("GPS position was not available for the North move.")
        northStartLat = float(northStartLocation.lat)
        northStartLon = float(northStartLocation.lon)

        print(
            f"Flying North {NORTH_DISTANCE_FEET:.0f} ft at {NORTH_SPEED_METERS_PER_SECOND:.1f} m/s "
            f"while holding {HOVER_ALTITUDE_FEET:.0f} ft height."
        )
        northMoveStartTime = time.monotonic()
        lastPrintTime = 0.0
        try:
            while True:
                if vehicle.mode.name != "GUIDED":
                    flightOutcome["step3FlyNorth10FeetAt5Ft"] = "FAILED (GUIDED mode lost)"
                    raise RuntimeError("GUIDED mode lost during the North move.")
                altitudeMeters = checkFlightHealthAndCeiling(vehicle, flightOutcome)

                currentLoc = vehicle.location.global_relative_frame
                if currentLoc is None or currentLoc.lat is None or currentLoc.lon is None:
                    flightOutcome["step3FlyNorth10FeetAt5Ft"] = "FAILED (GPS position lost)"
                    raise RuntimeError("GPS position was lost during the North move.")
                northMeters = (float(currentLoc.lat) - northStartLat) * METERS_PER_DEGREE_OF_LATITUDE
                eastMeters = (
                    (float(currentLoc.lon) - northStartLon)
                    * METERS_PER_DEGREE_OF_LATITUDE
                    * math.cos(math.radians(northStartLat))
                )
                northFeet = northMeters * FEET_PER_METER
                eastFeet = eastMeters * FEET_PER_METER
                flightOutcome["northDistanceTraveledText"] = f"{northFeet:.1f} ft North ({eastFeet:+.1f} ft East)"

                if northMeters >= NORTH_DISTANCE_METERS:
                    print(
                        f"10 feet North reached | {northFeet:.1f} ft North | "
                        f"{eastFeet:+.1f} ft East | {altitudeMeters:.2f} m ({altitudeMeters * FEET_PER_METER:.2f} ft)"
                    )
                    break

                if abs(eastMeters) > MAXIMUM_OFF_COURSE_METERS or northMeters < -MAXIMUM_OFF_COURSE_METERS:
                    flightOutcome["step3FlyNorth10FeetAt5Ft"] = f"FAILED (Off course: {northFeet:.1f} ft N, {eastFeet:+.1f} ft E)"
                    raise RuntimeError(f"Left the North line: {northFeet:.1f} ft North, {eastFeet:+.1f} ft East.")

                if time.monotonic() - northMoveStartTime > NORTH_MOVE_TIMEOUT_SECONDS:
                    flightOutcome["step3FlyNorth10FeetAt5Ft"] = f"FAILED (Timed out at {northFeet:.1f} ft N)"
                    raise RuntimeError(
                        f"Did not reach 10 feet North in time ({northFeet:.1f} ft North, {eastFeet:+.1f} ft East)."
                    )

                # Compute gentle vertical speed correction to hold 5.0 feet during the North move.
                downSpeedMetersPerSecond = 0.0
                if altitudeMeters is not None:
                    altitudeErrorMeters = HOVER_ALTITUDE_METERS - altitudeMeters
                    climbRate = max(
                        -MAXIMUM_VERTICAL_CORRECTION_METERS_PER_SECOND,
                        min(
                            MAXIMUM_VERTICAL_CORRECTION_METERS_PER_SECOND,
                            altitudeErrorMeters * ALTITUDE_HOLD_GAIN_DURING_MOVE,
                        ),
                    )
                    downSpeedMetersPerSecond = -climbRate

                sendLocalVelocity(vehicle, NORTH_SPEED_METERS_PER_SECOND, 0.0, downSpeedMetersPerSecond)

                nowTime = time.monotonic()
                if nowTime - lastPrintTime >= PRINT_INTERVAL_SECONDS:
                    print(
                        f"Flying North | {northFeet:.1f}/{NORTH_DISTANCE_FEET:.0f} ft North | "
                        f"{eastFeet:+.1f} ft East | {altitudeMeters:.2f} m ({altitudeMeters * FEET_PER_METER:.2f} ft)"
                    )
                    lastPrintTime = nowTime
                time.sleep(STATUS_CHECK_INTERVAL_SECONDS)

            # Hold zero horizontal speed for 2 seconds at 5.0 feet so the copter stops cleanly before Step 4 LOITER.
            brakeStartTime = time.monotonic()
            while time.monotonic() - brakeStartTime < SETTLE_AFTER_MOVE_SECONDS:
                if vehicle.mode.name != "GUIDED":
                    flightOutcome["step3FlyNorth10FeetAt5Ft"] = "FAILED (GUIDED lost while stopping)"
                    raise RuntimeError("GUIDED mode lost while stopping.")
                altitudeMeters = checkFlightHealthAndCeiling(vehicle, flightOutcome)
                downSpeedMetersPerSecond = 0.0
                if altitudeMeters is not None:
                    altitudeErrorMeters = HOVER_ALTITUDE_METERS - altitudeMeters
                    climbRate = max(
                        -MAXIMUM_VERTICAL_CORRECTION_METERS_PER_SECOND,
                        min(MAXIMUM_VERTICAL_CORRECTION_METERS_PER_SECOND, altitudeErrorMeters * ALTITUDE_HOLD_GAIN_DURING_MOVE),
                    )
                    downSpeedMetersPerSecond = -climbRate
                sendLocalVelocity(vehicle, 0.0, 0.0, downSpeedMetersPerSecond)
                time.sleep(STATUS_CHECK_INTERVAL_SECONDS)

            print("North move stopped cleanly.")
            flightOutcome["step3FlyNorth10FeetAt5Ft"] = f"SUCCESS ({flightOutcome['northDistanceTraveledText']})"
        finally:
            if heartbeatIsFresh(vehicle) and not getattr(vehicle, "commandCableClosed", False):
                try:
                    sendLocalVelocity(vehicle, 0.0, 0.0, 0.0)
                except Exception as stopError:
                    print(f"Could not send the stop speed: {stopError}")

        # ==================================================
        # STEP 4: SECOND STATIONARY HOVER AT 5 FEET FOR 10 SECONDS (LOITER)
        # ==================================================
        flightOutcome["step4SecondHover5Ft10Seconds"] = "IN PROGRESS"
        setFlightMode(vehicle, "LOITER")

        secondHoverLocation = vehicle.location.global_relative_frame
        if secondHoverLocation is None or secondHoverLocation.lat is None or secondHoverLocation.lon is None:
            flightOutcome["step4SecondHover5Ft10Seconds"] = "FAILED (No GPS spot at hover start)"
            raise RuntimeError("GPS position was not available for Second hover.")
        secondHoverStartLat = float(secondHoverLocation.lat)
        secondHoverStartLon = float(secondHoverLocation.lon)

        print(f"Second hover (5 ft) started (holding 5 ft for {HOVER_DURATION_SECONDS:.0f} seconds).")
        hoverStartTime = time.monotonic()
        lastPrintTime = 0.0
        secondHoverMaxDriftFeet = 0.0
        while True:
            elapsedHover = time.monotonic() - hoverStartTime
            if elapsedHover >= HOVER_DURATION_SECONDS:
                break
            if vehicle.mode.name != "LOITER":
                flightOutcome["step4SecondHover5Ft10Seconds"] = "FAILED (LOITER mode lost)"
                raise RuntimeError("LOITER mode lost during Second hover.")
            altitudeMeters = checkFlightHealthAndCeiling(vehicle, flightOutcome)

            currentLoc = vehicle.location.global_relative_frame
            if currentLoc is None or currentLoc.lat is None or currentLoc.lon is None:
                flightOutcome["step4SecondHover5Ft10Seconds"] = "FAILED (GPS position lost)"
                raise RuntimeError("GPS position was lost during Second hover.")
            northOffsetMeters = (float(currentLoc.lat) - secondHoverStartLat) * METERS_PER_DEGREE_OF_LATITUDE
            eastOffsetMeters = (
                (float(currentLoc.lon) - secondHoverStartLon)
                * METERS_PER_DEGREE_OF_LATITUDE
                * math.cos(math.radians(secondHoverStartLat))
            )
            driftMeters = math.hypot(northOffsetMeters, eastOffsetMeters)
            driftFeet = driftMeters * FEET_PER_METER
            if driftFeet > secondHoverMaxDriftFeet:
                secondHoverMaxDriftFeet = driftFeet
            if driftMeters > MAXIMUM_HOVER_DRIFT_METERS:
                flightOutcome["step4SecondHover5Ft10Seconds"] = f"FAILED (Drifted {driftFeet:.1f} ft)"
                raise RuntimeError(
                    f"Second hover (5 ft) drifted {driftFeet:.1f} ft from its spot "
                    f"(limit {MAXIMUM_HOVER_DRIFT_FEET:.0f} ft)."
                )

            nowTime = time.monotonic()
            if nowTime - lastPrintTime >= PRINT_INTERVAL_SECONDS:
                remainingSeconds = max(0.0, HOVER_DURATION_SECONDS - elapsedHover)
                print(
                    f"Second hover (5 ft) | {altitudeMeters:.2f} m ({altitudeMeters * FEET_PER_METER:.2f} ft) | "
                    f"drift {driftFeet:.1f} ft | remaining {remainingSeconds:.0f} s"
                )
                lastPrintTime = nowTime
            time.sleep(STATUS_CHECK_INTERVAL_SECONDS)

        print("Second hover (5 ft) completed.")
        flightOutcome["step4SecondHover5Ft10Seconds"] = f"SUCCESS (10s completed, max drift {secondHoverMaxDriftFeet:.1f} ft)"

        # ==================================================
        # STEP 5: LAND SAFELY AND WAIT UNTIL PROPELLERS STOP
        # ==================================================
        landingWasStarted = True
        flightOutcome["step5LandAndStopProps"] = "IN PROGRESS"
        print("Landing.")
        setFlightMode(vehicle, "LAND")
        landStartTime = time.monotonic()
        while vehicle.armed:
            if not heartbeatIsFresh(vehicle):
                failsafeWait = getattr(vehicle, "groundStationFailsafeTimeoutSeconds", PREFERRED_FAILSAFE_TIMEOUT_SECONDS)
                print("Heartbeat is lost while waiting for landing. Closing cable so onboard failsafe finishes landing.")
                vehicle.commandCableClosed = True
                vehicle.close()
                flightOutcome["step5LandAndStopProps"] = (
                    f"ONBOARD FAILSAFE TRIGGERED DURING DESCENT (Lands after {failsafeWait:.0f}s)"
                )
                break
            if time.monotonic() - landStartTime > LAND_TIMEOUT_SECONDS:
                flightOutcome["step5LandAndStopProps"] = "FAILED (Timed out waiting for motors to disarm)"
                raise RuntimeError("Landing not confirmed. The motors are still armed.")
            loc = vehicle.location.global_relative_frame
            altText = "waiting" if (loc is None or loc.alt is None) else f"{float(loc.alt):.2f} m ({float(loc.alt) * FEET_PER_METER:.2f} ft)"
            print(f"Waiting for landing and propeller stop. {altText}")
            time.sleep(PRINT_INTERVAL_SECONDS)

        copterWasArmed = vehicle.armed
        if not vehicle.armed:
            print("Landed safely. Motors disarmed and propellers stopped.")
            flightOutcome["step5LandAndStopProps"] = "SUCCESS (Landed, motors disarmed, propellers stopped)"
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
        if vehicle is not None and not getattr(vehicle, "commandCableClosed", False):
            if (copterWasArmed or vehicle.armed) and not landingWasStarted:
                print("Requesting emergency LAND.")
                flightOutcome["step5LandAndStopProps"] = "IN PROGRESS (Emergency LAND)"
                try:
                    if not heartbeatIsFresh(vehicle):
                        failsafeWait = getattr(vehicle, "groundStationFailsafeTimeoutSeconds", PREFERRED_FAILSAFE_TIMEOUT_SECONDS)
                        print("Heartbeat is lost. Closing cable so onboard failsafe lands the copter.")
                        vehicle.commandCableClosed = True
                        vehicle.close()
                        flightOutcome["step5LandAndStopProps"] = (
                            f"ONBOARD FAILSAFE TRIGGERED (Cable closed; Pixhawk lands after {failsafeWait:.0f}s)"
                        )
                    else:
                        setFlightMode(vehicle, "LAND")
                        emergencyLandStart = time.monotonic()
                        while vehicle.armed:
                            if not heartbeatIsFresh(vehicle):
                                vehicle.commandCableClosed = True
                                vehicle.close()
                                flightOutcome["step5LandAndStopProps"] = "ONBOARD FAILSAFE TRIGGERED DURING DESCENT"
                                break
                            if time.monotonic() - emergencyLandStart > LAND_TIMEOUT_SECONDS:
                                raise RuntimeError("Timed out waiting for motors to disarm.")
                            loc = vehicle.location.global_relative_frame
                            altText = "waiting" if (loc is None or loc.alt is None) else f"{float(loc.alt):.2f} m ({float(loc.alt) * FEET_PER_METER:.2f} ft)"
                            print(f"Waiting for landing and propeller stop. {altText}")
                            time.sleep(PRINT_INTERVAL_SECONDS)
                        if not vehicle.armed:
                            print("Landed safely. Motors disarmed and propellers stopped.")
                            flightOutcome["step5LandAndStopProps"] = "SUCCESS (Landed, motors disarmed, propellers stopped)"
                except KeyboardInterrupt:
                    print("Second Ctrl+C. Closing the cable without force-disarming.")
                    print("The onboard failsafe remains set to Land.")
                    flightOutcome["step5LandAndStopProps"] = "SECOND Ctrl+C (Cable closed; onboard Land failsafe active)"
                except Exception as landingError:
                    print(f"Landing unconfirmed: {landingError}")
                    flightOutcome["step5LandAndStopProps"] = f"UNCONFIRMED ({landingError})"

            if not getattr(vehicle, "commandCableClosed", False) and heartbeatIsFresh(vehicle) and not vehicle.armed:
                if savedFailsafe is not None:
                    try:
                        setParameterAndConfirm(vehicle, GROUND_STATION_FAILSAFE_PARAMETER_NAME, savedFailsafe[0])
                        setParameterAndConfirm(vehicle, GROUND_STATION_FAILSAFE_TIMEOUT_PARAMETER_NAME, savedFailsafe[1])
                        setParameterAndConfirm(vehicle, FAILSAFE_OPTIONS_PARAMETER_NAME, savedFailsafe[2])
                        print("Ground-station failsafe settings restored.")
                    except Exception as restoreError:
                        print(f"Could not restore the ground-station failsafe: {restoreError}")
                if originalTakeoffSlewSeconds is not None:
                    try:
                        print(f"Restoring {TAKEOFF_SLEW_PARAMETER_NAME} to {originalTakeoffSlewSeconds:.2f}s.")
                        setParameterAndConfirm(vehicle, TAKEOFF_SLEW_PARAMETER_NAME, originalTakeoffSlewSeconds)
                        print(f"{TAKEOFF_SLEW_PARAMETER_NAME} restored.")
                    except Exception as restoreError:
                        print(f"Could not restore {TAKEOFF_SLEW_PARAMETER_NAME}: {restoreError}")

            if not getattr(vehicle, "commandCableClosed", False):
                vehicle.commandCableClosed = True
                try:
                    vehicle.close()
                    print("Command cable closed.")
                except Exception as closeError:
                    print(f"Could not close the command cable: {closeError}")
        elif vehicle is not None:
            print("Failsafe settings were left on the Pixhawk.")
        print("Script finished.")

        flightOutcome["finishedAt"] = time.strftime("%Y-%m-%d %H:%M:%S %Z")
        print()
        print("=" * 68)
        print("FLIGHT RUN OUTCOME SUMMARY")
        print("=" * 68)
        print(f"Overall Outcome                : {flightOutcome['overallOutcome']}")
        print(f"Started At                     : {flightOutcome['startedAt']}")
        print(f"Finished At                    : {flightOutcome['finishedAt']}")
        print(f"Preflight & Land Failsafe      : {flightOutcome['preflightAndFailsafe']}")
        print(f"Step 1 (Takeoff 4ft -> 5ft)    : {flightOutcome['step1Takeoff4FtToHover5Ft']}")
        print(f"Step 2 (First Hover 5ft, 10s)  : {flightOutcome['step2FirstHover5Ft10Seconds']}")
        print(f"Step 3 (Fly North 10ft @ 5ft)  : {flightOutcome['step3FlyNorth10FeetAt5Ft']}")
        print(f"Step 4 (Second Hover 5ft, 10s) : {flightOutcome['step4SecondHover5Ft10Seconds']}")
        print(f"Step 5 (Land & Disarm)         : {flightOutcome['step5LandAndStopProps']}")
        print(f"Max Altitude Recorded          : {flightOutcome['maximumAltitudeReachedText']}")
        print(f"North Distance Recorded        : {flightOutcome['northDistanceTraveledText']}")
        if flightOutcome["stopReason"]:
            print(f"Stop / Failure Reason          : {flightOutcome['stopReason']}")
        if logPath is not None:
            print(f"Saved Flight Log               : {logPath}")
        print("=" * 68)

        sys.stdout = originalStandardOutput
        if logFile is not None:
            logFile.close()


if __name__ == "__main__":
    main()
