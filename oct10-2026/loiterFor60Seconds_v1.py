#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    Spin each motor once, then take off to about 1.5 feet, loiter for
    60 seconds, and land. GPS holds the horizontal position.

WHY:
    A slow throttle ramp leaves one side on the ground long enough to tip.
    ArduPilot flies this climb in GUIDED and keeps the wings level.
    This script does not command roll, pitch, or each motor during the flight.
    The Pixhawk mixes the motors.
    Before that flight, each motor is spun alone so the order can be seen.
    The motor-test command does not use motor numbers. On this Quad X,
    test order 2 spins motor 4, so motor 2 is sent as test order 3.
    The takeoff ramp is made firmer only when the saved ramp is slower
    than 1 second. An already-firm ramp is left alone.

HOW:
    1. Connect on pins 8 and 10. Leave USB with the screen.
    2. Request heartbeat, attitude, GPS, position, EKF status, and status text.
    3. Read FRAME_CLASS and FRAME_TYPE. Stop if this is not a Quad X.
    4. Spin motor 1, then 2, then 3, then 4, at 22 percent for 5 seconds.
    5. Wait 30 seconds.
    6. Type YES to continue.
    7. Wait for a 3D fix, at least 6 satellites, and a healthy position.
    8. Shorten TKOFF_SLEW_TIME if it is slower than 1 second.
    9. Enter GUIDED only if roll and pitch are within 5 degrees of level.
    10. Arm, then command takeoff to 1.5 feet.
    11. Enter LOITER only after a real climb is seen on new height readings.
        Hold for 60 seconds.
    12. Enter LAND.
    13. Put TKOFF_SLEW_TIME back if this script changed it.
    14. Keep every printed line in a flight log named after this file.

IMPORTANT:
    The four spins happen before the hover question. Type YES after them to arm.
    This script sends the arm command. It does not renumber the motors.
    Compass and GPS arming checks stay on.
    Two or three satellites are not enough. Position hold needs a 3D fix,
    at least 6 satellites, and a healthy horizontal position.
    The Pixhawk already has the takeoff throttle cap and the higher
    attitude mix. This script does not change those.
    Ctrl+C during a spin stops that motor. Ctrl+C after arming requests LAND.
    A second Ctrl+C closes the link.
    This script does not force-disarm.
    After the test, read the newest file in the flight-logs folder
    beside this script.
"""

import math
import os
import sys
import time

from pymavlink import mavutil


# Pin 8 transmits. Pin 10 receives. Pin 6 is ground.
PIXHAWK_SERIAL_PORT = "/dev/serial0"
PIXHAWK_SERIAL_BAUD = 57600

CONNECTION_TIMEOUT_SECONDS = 15
HEARTBEAT_STALE_AFTER_SECONDS = 3.0
SENSOR_MESSAGE_STALE_AFTER_SECONDS = 1.0
PARAMETER_TIMEOUT_SECONDS = 4.0
MESSAGE_INTERVAL_MICROSECONDS = 100_000
GPS_LISTEN_SECONDS = 30
STATUS_PRINT_INTERVAL_SECONDS = 1.0

GUIDED_MODE_NUMBER = 4
LOITER_MODE_NUMBER = 5
LAND_MODE_NUMBER = 9

METERS_PER_FOOT = 0.3048
FEET_PER_METER = 3.28084
TARGET_ALTITUDE_FEET = 1.5
TARGET_ALTITUDE_METERS = TARGET_ALTITUDE_FEET * METERS_PER_FOOT
HOVER_TIME_SECONDS = 60
TAKEOFF_REACHED_FRACTION = 0.90
# One old height reading must not be treated as a completed takeoff.
REQUIRED_TAKEOFF_HEIGHT_SAMPLES = 3
MINIMUM_RECORDED_CLIMB_METERS = 0.10
CLIMB_START_TIMEOUT_SECONDS = 8.0
# The copter has to start the climb nearly level. This is not a trim setting.
MAXIMUM_GROUND_ROLL_DEGREES = 5.0
MAXIMUM_GROUND_PITCH_DEGREES = 5.0
TAKEOFF_TIMEOUT_SECONDS = 25
LAND_WAIT_TIMEOUT_SECONDS = 30
MODE_CHANGE_TIMEOUT_SECONDS = 10
ARM_CONFIRMATION_TIMEOUT_SECONDS = 10

GPS_POSITION_HOLD_MINIMUM_FIX_TYPE = 3
GPS_POSITION_HOLD_MINIMUM_SATELLITES = 6
UNKNOWN_SATELLITE_COUNT = 255

EKF_HORIZONTAL_POSITION_ABSOLUTE_BIT = 16
EKF_CONSTANT_POSITION_MODE_BIT = 128
EKF_GPS_GLITCHING_BIT = 32768

TAKEOFF_SLEW_PARAMETER_NAME = "TKOFF_SLEW_TIME"
PREFERRED_TAKEOFF_SLEW_TIME_SECONDS = 1.0
PARAMETER_MATCH_TOLERANCE = 0.01

# Quad X. The motor-test command uses the second number, not the motor number.
# Test order 2 is motor 4, so motor 2 is sent as 3.
QUAD_FRAME_CLASS = 1
QUAD_X_FRAME_TYPE = 1
MOTORS_TO_SPIN = (
    (1, 1, "front-right", "counter-clockwise"),
    (2, 3, "back-left", "counter-clockwise"),
    (3, 4, "front-left", "clockwise"),
    (4, 2, "back-right", "clockwise"),
)
MOTOR_SPIN_THROTTLE_PERCENT = 22
MOTOR_SPIN_SECONDS = 5
PAUSE_BETWEEN_MOTORS_SECONDS = 2
PAUSE_BEFORE_HOVER_QUESTION_SECONDS = 30
MOTOR_TEST_ANSWER_TIMEOUT_SECONDS = 3.0

activeMotorTestSequence = None

FLIGHT_LOG_DIRECTORY = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "flight-logs",
)

REQUIRED_MESSAGE_NAMES = (
    "HEARTBEAT",
    "ATTITUDE",
    "GPS_RAW_INT",
    "GLOBAL_POSITION_INT",
    "EKF_STATUS_REPORT",
    "STATUSTEXT",
)

lastHeartbeatReceivedAtSeconds = None


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

    def flush(self):
        self.terminal.flush()
        self.logFile.flush()


def openFlightLog():
    os.makedirs(FLIGHT_LOG_DIRECTORY, exist_ok=True)
    scriptName = os.path.splitext(os.path.basename(__file__))[0]
    startedAt = time.strftime("%Y-%m-%d-%H%M%S")
    logPath = os.path.join(
        FLIGHT_LOG_DIRECTORY,
        f"{scriptName}-{startedAt}.log",
    )
    logFile = open(logPath, "w", encoding="utf-8")
    sys.stdout = FlightLog(logFile, sys.stdout)
    print(f"Flight log: {logPath}")
    return logFile, logPath


def cleanStatusText(message):
    statusText = getattr(message, "text", "")
    if isinstance(statusText, bytes):
        statusText = statusText.decode("ascii", "ignore")
    return str(statusText).strip("\x00").strip()


def parameterNameText(rawParameterName):
    if isinstance(rawParameterName, bytes):
        rawParameterName = rawParameterName.decode("ascii", "ignore")
    return str(rawParameterName).strip("\x00").strip()


def rememberHeartbeat(pixhawkConnection, message):
    global lastHeartbeatReceivedAtSeconds
    if message.get_srcSystem() != pixhawkConnection.target_system:
        return
    lastHeartbeatReceivedAtSeconds = time.monotonic()


def heartbeatIsFresh():
    if lastHeartbeatReceivedAtSeconds is None:
        return False
    heartbeatAgeSeconds = time.monotonic() - lastHeartbeatReceivedAtSeconds
    return heartbeatAgeSeconds <= HEARTBEAT_STALE_AFTER_SECONDS


def connectToPixhawk():
    print()
    print(f"Opening {PIXHAWK_SERIAL_PORT} at {PIXHAWK_SERIAL_BAUD} baud.")
    pixhawkConnection = mavutil.mavlink_connection(
        PIXHAWK_SERIAL_PORT,
        baud=PIXHAWK_SERIAL_BAUD,
        source_system=255,
        autoreconnect=False,
    )
    heartbeat = pixhawkConnection.wait_heartbeat(timeout=CONNECTION_TIMEOUT_SECONDS)
    if heartbeat is None:
        raise RuntimeError("No Pixhawk heartbeat arrived on pin 10.")
    if pixhawkConnection.target_component == 0:
        pixhawkConnection.target_component = 1
    rememberHeartbeat(pixhawkConnection, heartbeat)
    print(
        f"Connected to Pixhawk system {pixhawkConnection.target_system}, "
        f"component {pixhawkConnection.target_component}."
    )
    return pixhawkConnection


def requestRequiredTelemetry(pixhawkConnection):
    for messageName in REQUIRED_MESSAGE_NAMES:
        messageId = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{messageName}", None)
        if messageId is None:
            print(f"MAVLink message id for {messageName} was not found.")
            continue
        pixhawkConnection.mav.command_long_send(
            pixhawkConnection.target_system,
            pixhawkConnection.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            messageId,
            MESSAGE_INTERVAL_MICROSECONDS,
            0,
            0,
            0,
            0,
            0,
        )
    print("Requested attitude, GPS, and height telemetry.")


def processIncomingMessage(pixhawkConnection, latestMessages, messageArrivedAtSeconds, message):
    if message is None:
        return
    # A broken serial byte must not replace the latest heartbeat or GPS fix.
    if message.get_type() == "BAD_DATA":
        return
    if message.get_srcSystem() not in (0, pixhawkConnection.target_system):
        return
    messageType = message.get_type()
    latestMessages[messageType] = message
    messageArrivedAtSeconds[messageType] = time.monotonic()
    if messageType == "HEARTBEAT":
        rememberHeartbeat(pixhawkConnection, message)
    elif messageType == "STATUSTEXT":
        statusText = cleanStatusText(message)
        if statusText:
            print(f"PIXHAWK: {statusText}")


def readAvailableMessages(pixhawkConnection, latestMessages, messageArrivedAtSeconds):
    while True:
        message = pixhawkConnection.recv_match(blocking=False)
        if message is None:
            return
        processIncomingMessage(
            pixhawkConnection,
            latestMessages,
            messageArrivedAtSeconds,
            message,
        )


def messageIsFresh(messageArrivedAtSeconds, messageType):
    arrivedAtSeconds = messageArrivedAtSeconds.get(messageType)
    if arrivedAtSeconds is None:
        return False
    return time.monotonic() - arrivedAtSeconds <= SENSOR_MESSAGE_STALE_AFTER_SECONDS


def copterIsArmed(latestMessages):
    heartbeat = latestMessages.get("HEARTBEAT")
    if heartbeat is None:
        return None
    return bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def copterIsInMode(latestMessages, requestedModeNumber):
    heartbeat = latestMessages.get("HEARTBEAT")
    if heartbeat is None:
        return False
    return int(heartbeat.custom_mode) == int(requestedModeNumber)


def gpsSatelliteCount(latestMessages, messageArrivedAtSeconds):
    if not messageIsFresh(messageArrivedAtSeconds, "GPS_RAW_INT"):
        return None
    gpsReading = latestMessages.get("GPS_RAW_INT")
    if gpsReading is None:
        return None
    satelliteCount = getattr(gpsReading, "satellites_visible", None)
    if satelliteCount is None or satelliteCount == UNKNOWN_SATELLITE_COUNT:
        return None
    return int(satelliteCount)


def ekfHorizontalPositionIsHealthy(latestMessages, messageArrivedAtSeconds):
    if not messageIsFresh(messageArrivedAtSeconds, "EKF_STATUS_REPORT"):
        return False
    ekfStatus = latestMessages.get("EKF_STATUS_REPORT")
    if ekfStatus is None:
        return False
    ekfFlags = int(ekfStatus.flags)
    hasAbsoluteHorizontalPosition = bool(ekfFlags & EKF_HORIZONTAL_POSITION_ABSOLUTE_BIT)
    constantPositionModeIsActive = bool(ekfFlags & EKF_CONSTANT_POSITION_MODE_BIT)
    gpsGlitchIsActive = bool(ekfFlags & EKF_GPS_GLITCHING_BIT)
    return (
        hasAbsoluteHorizontalPosition
        and not constantPositionModeIsActive
        and not gpsGlitchIsActive
    )


def gpsIsUsableForPositionHold(latestMessages, messageArrivedAtSeconds):
    """LOITER needs a 3D fix, 6 satellites, and a healthy horizontal position."""
    if not messageIsFresh(messageArrivedAtSeconds, "GPS_RAW_INT"):
        return False
    gpsReading = latestMessages.get("GPS_RAW_INT")
    if gpsReading is None or gpsReading.fix_type is None:
        return False
    if int(gpsReading.fix_type) < GPS_POSITION_HOLD_MINIMUM_FIX_TYPE:
        return False
    satelliteCount = gpsSatelliteCount(latestMessages, messageArrivedAtSeconds)
    if satelliteCount is None or satelliteCount < GPS_POSITION_HOLD_MINIMUM_SATELLITES:
        return False
    return ekfHorizontalPositionIsHealthy(latestMessages, messageArrivedAtSeconds)


def attitudeDegrees(latestMessages, messageArrivedAtSeconds):
    """Roll and pitch in degrees. None until a fresh attitude message arrives."""
    if not messageIsFresh(messageArrivedAtSeconds, "ATTITUDE"):
        return None
    attitudeMessage = latestMessages.get("ATTITUDE")
    if attitudeMessage is None:
        return None
    rollDegrees = math.degrees(float(attitudeMessage.roll))
    pitchDegrees = math.degrees(float(attitudeMessage.pitch))
    if not math.isfinite(rollDegrees) or not math.isfinite(pitchDegrees):
        return None
    return rollDegrees, pitchDegrees


def attitudeText(latestMessages, messageArrivedAtSeconds):
    attitude = attitudeDegrees(latestMessages, messageArrivedAtSeconds)
    if attitude is None:
        return "attitude waiting"
    rollDegrees, pitchDegrees = attitude
    return f"roll {rollDegrees:+.1f} deg | pitch {pitchDegrees:+.1f} deg"


def copterIsLevelForTakeoff(latestMessages, messageArrivedAtSeconds):
    """A leaning start is what leaves one side on the ground."""
    attitude = attitudeDegrees(latestMessages, messageArrivedAtSeconds)
    if attitude is None:
        print("Attitude is not available, so takeoff was not sent.")
        return False
    rollDegrees, pitchDegrees = attitude
    print(f"Ground attitude: {attitudeText(latestMessages, messageArrivedAtSeconds)}")
    if abs(rollDegrees) > MAXIMUM_GROUND_ROLL_DEGREES or abs(pitchDegrees) > MAXIMUM_GROUND_PITCH_DEGREES:
        print("The copter is leaning. Level it, then run the script again.")
        return False
    print("Ground attitude is level.")
    return True


def relativeAltitudeMeters(latestMessages, messageArrivedAtSeconds):
    if not messageIsFresh(messageArrivedAtSeconds, "GLOBAL_POSITION_INT"):
        return None
    positionMessage = latestMessages.get("GLOBAL_POSITION_INT")
    if positionMessage is None:
        return None
    relativeAltitudeMillimeters = getattr(positionMessage, "relative_alt", None)
    if relativeAltitudeMillimeters is None:
        return None
    altitudeMeters = float(relativeAltitudeMillimeters) / 1000.0
    if not math.isfinite(altitudeMeters):
        return None
    return altitudeMeters


def readPixhawkParameter(
    pixhawkConnection,
    latestMessages,
    messageArrivedAtSeconds,
    parameterName,
):
    pixhawkConnection.mav.param_request_read_send(
        pixhawkConnection.target_system,
        pixhawkConnection.target_component,
        parameterName.encode("ascii"),
        -1,
    )
    deadlineSeconds = time.monotonic() + PARAMETER_TIMEOUT_SECONDS
    while time.monotonic() < deadlineSeconds:
        message = pixhawkConnection.recv_match(blocking=True, timeout=0.4)
        if message is None:
            continue
        processIncomingMessage(
            pixhawkConnection,
            latestMessages,
            messageArrivedAtSeconds,
            message,
        )
        if message.get_type() != "PARAM_VALUE":
            continue
        if parameterNameText(message.param_id) == parameterName:
            return float(message.param_value)
    return None


def setPixhawkParameterAndConfirm(
    pixhawkConnection,
    latestMessages,
    messageArrivedAtSeconds,
    parameterName,
    requestedValue,
):
    pixhawkConnection.mav.param_set_send(
        pixhawkConnection.target_system,
        pixhawkConnection.target_component,
        parameterName.encode("ascii"),
        float(requestedValue),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )
    deadlineSeconds = time.monotonic() + PARAMETER_TIMEOUT_SECONDS
    while time.monotonic() < deadlineSeconds:
        message = pixhawkConnection.recv_match(blocking=True, timeout=0.4)
        if message is None:
            continue
        processIncomingMessage(
            pixhawkConnection,
            latestMessages,
            messageArrivedAtSeconds,
            message,
        )
        if message.get_type() != "PARAM_VALUE":
            continue
        if parameterNameText(message.param_id) != parameterName:
            continue
        returnedValue = float(message.param_value)
        if abs(returnedValue - requestedValue) < PARAMETER_MATCH_TOLERANCE:
            return True
    return False


def prepareFirmerGuidedTakeoff(pixhawkConnection, latestMessages, messageArrivedAtSeconds):
    """Return the original slew time only when this script changes it."""
    originalTakeoffSlewSeconds = readPixhawkParameter(
        pixhawkConnection,
        latestMessages,
        messageArrivedAtSeconds,
        TAKEOFF_SLEW_PARAMETER_NAME,
    )
    if originalTakeoffSlewSeconds is None:
        print(f"{TAKEOFF_SLEW_PARAMETER_NAME} was not available. The existing ramp will be used.")
        return None
    temporaryTakeoffSlewSeconds = min(
        originalTakeoffSlewSeconds,
        PREFERRED_TAKEOFF_SLEW_TIME_SECONDS,
    )
    print(
        f"{TAKEOFF_SLEW_PARAMETER_NAME}: "
        f"original {originalTakeoffSlewSeconds:.2f}s | "
        f"takeoff {temporaryTakeoffSlewSeconds:.2f}s"
    )
    if abs(temporaryTakeoffSlewSeconds - originalTakeoffSlewSeconds) < PARAMETER_MATCH_TOLERANCE:
        return None
    changed = setPixhawkParameterAndConfirm(
        pixhawkConnection,
        latestMessages,
        messageArrivedAtSeconds,
        TAKEOFF_SLEW_PARAMETER_NAME,
        temporaryTakeoffSlewSeconds,
    )
    if not changed:
        print(f"Could not confirm {TAKEOFF_SLEW_PARAMETER_NAME}. The existing ramp will be used.")
        return None
    return originalTakeoffSlewSeconds


def restoreTakeoffSlew(
    pixhawkConnection,
    latestMessages,
    messageArrivedAtSeconds,
    originalTakeoffSlewSeconds,
):
    print(f"Restoring {TAKEOFF_SLEW_PARAMETER_NAME} to {originalTakeoffSlewSeconds:.2f}s.")
    restored = setPixhawkParameterAndConfirm(
        pixhawkConnection,
        latestMessages,
        messageArrivedAtSeconds,
        TAKEOFF_SLEW_PARAMETER_NAME,
        originalTakeoffSlewSeconds,
    )
    if restored:
        print(f"{TAKEOFF_SLEW_PARAMETER_NAME} restored.")
    else:
        print(f"Could not confirm restoration of {TAKEOFF_SLEW_PARAMETER_NAME}.")


def quadXFrameIsReady(pixhawkConnection, latestMessages, messageArrivedAtSeconds):
    """The spin order below is true only for a Quad X."""
    frameClass = readPixhawkParameter(
        pixhawkConnection,
        latestMessages,
        messageArrivedAtSeconds,
        "FRAME_CLASS",
    )
    frameType = readPixhawkParameter(
        pixhawkConnection,
        latestMessages,
        messageArrivedAtSeconds,
        "FRAME_TYPE",
    )
    print()
    print("Flight motor order, looking down:")
    print("  motor 1  front-right  counter-clockwise")
    print("  motor 2  back-left    counter-clockwise")
    print("  motor 3  front-left   clockwise")
    print("  motor 4  back-right   clockwise")
    print("Motor-test order on this frame is 1, 4, 2, 3.")
    if frameClass is None or frameType is None:
        print("FRAME_CLASS or FRAME_TYPE could not be read.")
        return False
    print(f"FRAME_CLASS {frameClass:.0f} | FRAME_TYPE {frameType:.0f}")
    frameIsQuad = abs(frameClass - QUAD_FRAME_CLASS) < PARAMETER_MATCH_TOLERANCE
    frameIsX = abs(frameType - QUAD_X_FRAME_TYPE) < PARAMETER_MATCH_TOLERANCE
    if not frameIsQuad or not frameIsX:
        print("The Pixhawk is not set to Quad X.")
        return False
    print("Frame is Quad X.")
    return True


def sendMotorTest(pixhawkConnection, testSequence, throttlePercent, runSeconds):
    pixhawkConnection.mav.command_long_send(
        pixhawkConnection.target_system,
        pixhawkConnection.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
        0,
        testSequence,
        0,
        throttlePercent,
        runSeconds,
        0,
        0,
        0,
    )


def waitForMotorTestAnswer(pixhawkConnection, latestMessages, messageArrivedAtSeconds):
    deadlineSeconds = time.monotonic() + MOTOR_TEST_ANSWER_TIMEOUT_SECONDS
    while time.monotonic() < deadlineSeconds:
        message = pixhawkConnection.recv_match(blocking=True, timeout=0.4)
        if message is None:
            continue
        processIncomingMessage(
            pixhawkConnection,
            latestMessages,
            messageArrivedAtSeconds,
            message,
        )
        if message.get_type() != "COMMAND_ACK":
            continue
        if message.command != mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST:
            continue
        return message.result
    return None


def stopActiveMotorTest(pixhawkConnection):
    """Stop the motor that is spinning now. A finished spin has nothing to stop."""
    global activeMotorTestSequence
    if pixhawkConnection is None or activeMotorTestSequence is None:
        return
    sendMotorTest(pixhawkConnection, activeMotorTestSequence, 0, 0)
    print(f"Stopped test order {activeMotorTestSequence}.")
    activeMotorTestSequence = None


def spinMotorsOneByOne(pixhawkConnection, latestMessages, messageArrivedAtSeconds):
    """Spin motor 1, then 2, then 3, then 4. Return False if a spin does not start."""
    global activeMotorTestSequence
    readAvailableMessages(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
    if copterIsArmed(latestMessages) is True:
        print("The copter is armed. The motor spins were not started.")
        return False

    print()
    print("Spinning each motor once before the loiter.")
    print(
        f"Each spin is {MOTOR_SPIN_THROTTLE_PERCENT} percent "
        f"for {MOTOR_SPIN_SECONDS:.0f} seconds."
    )
    for motorNumber, testSequence, motorPlace, motorDirection in MOTORS_TO_SPIN:
        print()
        print(f"Motor {motorNumber}: {motorPlace}, {motorDirection}.")
        activeMotorTestSequence = testSequence
        sendMotorTest(
            pixhawkConnection,
            testSequence,
            MOTOR_SPIN_THROTTLE_PERCENT,
            MOTOR_SPIN_SECONDS,
        )
        motorSpinAnswer = waitForMotorTestAnswer(
            pixhawkConnection,
            latestMessages,
            messageArrivedAtSeconds,
        )
        if motorSpinAnswer is None:
            stopActiveMotorTest(pixhawkConnection)
            print(f"No answer for motor {motorNumber}. The loiter was not started.")
            return False
        print(f"Motor test answer = {motorSpinAnswer}")
        if motorSpinAnswer != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            stopActiveMotorTest(pixhawkConnection)
            print("Motor test was refused. Press the safety switch until the LED is solid, then try again.")
            return False
        spinDeadlineSeconds = time.monotonic() + MOTOR_SPIN_SECONDS
        while time.monotonic() < spinDeadlineSeconds:
            readAvailableMessages(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
            time.sleep(0.1)
        stopActiveMotorTest(pixhawkConnection)
        print(f"Motor {motorNumber} spin finished.")
        pauseDeadlineSeconds = time.monotonic() + PAUSE_BETWEEN_MOTORS_SECONDS
        while time.monotonic() < pauseDeadlineSeconds:
            readAvailableMessages(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
            time.sleep(0.1)

    readAvailableMessages(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
    if copterIsArmed(latestMessages) is True:
        print("The copter is still armed after the motor spins. The loiter was not started.")
        return False
    return True


def waitBeforeHoverQuestion(pixhawkConnection, latestMessages, messageArrivedAtSeconds):
    print()
    print(f"Waiting {PAUSE_BEFORE_HOVER_QUESTION_SECONDS:.0f} seconds.")
    print("The loiter does not start during this wait.")
    waitStartedAtSeconds = time.monotonic()
    lastPrintedWaitSecond = -1
    while time.monotonic() - waitStartedAtSeconds < PAUSE_BEFORE_HOVER_QUESTION_SECONDS:
        readAvailableMessages(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
        elapsedSeconds = int(time.monotonic() - waitStartedAtSeconds)
        if elapsedSeconds != lastPrintedWaitSecond:
            remainingSeconds = PAUSE_BEFORE_HOVER_QUESTION_SECONDS - elapsedSeconds
            print(f"Loiter question in {remainingSeconds:.0f} seconds.")
            lastPrintedWaitSecond = elapsedSeconds
        time.sleep(0.1)
    print("Wait finished.")


def waitForGpsPosition(pixhawkConnection, latestMessages, messageArrivedAtSeconds):
    print()
    print(f"Listening for GPS for {GPS_LISTEN_SECONDS:.0f} seconds.")
    print("Need a 3D fix, at least 6 satellites, and a healthy position.")
    deadlineSeconds = time.monotonic() + GPS_LISTEN_SECONDS
    lastPrintedAtSeconds = 0.0
    while time.monotonic() < deadlineSeconds:
        readAvailableMessages(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
        if not heartbeatIsFresh():
            raise RuntimeError("Pixhawk heartbeat was lost.")
        if gpsIsUsableForPositionHold(latestMessages, messageArrivedAtSeconds):
            gpsReading = latestMessages["GPS_RAW_INT"]
            print(
                f"GPS position ready. Fix {gpsReading.fix_type} | "
                f"{gpsReading.satellites_visible} satellites."
            )
            return True
        currentTimeSeconds = time.monotonic()
        if currentTimeSeconds - lastPrintedAtSeconds >= STATUS_PRINT_INTERVAL_SECONDS:
            gpsReading = latestMessages.get("GPS_RAW_INT")
            satelliteCount = gpsSatelliteCount(latestMessages, messageArrivedAtSeconds)
            if gpsReading is None:
                print("GPS: no message yet.")
            else:
                print(f"GPS fix {gpsReading.fix_type} | satellites {satelliteCount}. Not ready.")
            lastPrintedAtSeconds = currentTimeSeconds
        time.sleep(0.1)
    return False


def setFlightModeAndWait(
    pixhawkConnection,
    latestMessages,
    messageArrivedAtSeconds,
    requestedModeNumber,
    requestedModeName,
):
    print(f"Requesting {requestedModeName}.")
    pixhawkConnection.mav.set_mode_send(
        pixhawkConnection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        requestedModeNumber,
    )
    deadlineSeconds = time.monotonic() + MODE_CHANGE_TIMEOUT_SECONDS
    while time.monotonic() < deadlineSeconds:
        readAvailableMessages(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
        if copterIsInMode(latestMessages, requestedModeNumber):
            print(f"{requestedModeName} confirmed.")
            return True
        if not heartbeatIsFresh():
            print("Heartbeat became stale while waiting for the mode change.")
            return False
        time.sleep(0.1)
    print(f"{requestedModeName} was not confirmed.")
    return False


def armAndWaitForConfirmation(pixhawkConnection, latestMessages, messageArrivedAtSeconds):
    print()
    print("Sending ARM command.")
    # An old arm answer must not be treated as the answer to this command.
    latestMessages.pop("COMMAND_ACK", None)
    pixhawkConnection.mav.command_long_send(
        pixhawkConnection.target_system,
        pixhawkConnection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    deadlineSeconds = time.monotonic() + ARM_CONFIRMATION_TIMEOUT_SECONDS
    armCommandResult = None
    while time.monotonic() < deadlineSeconds:
        message = pixhawkConnection.recv_match(blocking=True, timeout=0.5)
        if message is None:
            continue
        processIncomingMessage(
            pixhawkConnection,
            latestMessages,
            messageArrivedAtSeconds,
            message,
        )
        if copterIsArmed(latestMessages) is True:
            print("ARMED CONFIRMED.")
            return True
        if message.get_type() != "COMMAND_ACK":
            continue
        if message.command != mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            continue
        armCommandResult = message.result
        print(f"ARM answer = {armCommandResult}")
        if armCommandResult not in (
            mavutil.mavlink.MAV_RESULT_ACCEPTED,
            mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
        ):
            print(f"ARM FAILED. Answer = {armCommandResult}")
            return False
    print(f"ARM FAILED. Answer = {armCommandResult}")
    return False


def commandGpsTakeoff(pixhawkConnection):
    print()
    print(f"Taking off to {TARGET_ALTITUDE_FEET:.1f} feet.")
    pixhawkConnection.mav.command_long_send(
        pixhawkConnection.target_system,
        pixhawkConnection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        TARGET_ALTITUDE_METERS,
    )


def waitForTakeoffAltitude(
    pixhawkConnection,
    latestMessages,
    messageArrivedAtSeconds,
    startingAltitudeMeters,
):
    """LOITER starts only after new height readings show a real climb."""
    minimumTakeoffAltitudeMeters = TARGET_ALTITUDE_METERS * TAKEOFF_REACHED_FRACTION
    takeoffStartedAtSeconds = time.monotonic()
    deadlineSeconds = takeoffStartedAtSeconds + TAKEOFF_TIMEOUT_SECONDS
    climbStartDeadlineSeconds = takeoffStartedAtSeconds + CLIMB_START_TIMEOUT_SECONDS
    lastPrintedAtSeconds = 0.0
    lastPositionArrivalSeconds = None
    climbDetected = False
    consecutiveHeightSamples = 0
    while time.monotonic() < deadlineSeconds:
        readAvailableMessages(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
        if not heartbeatIsFresh():
            print("Heartbeat was lost during takeoff.")
            return False
        if copterIsArmed(latestMessages) is not True:
            print("The copter disarmed during takeoff.")
            return False
        if not copterIsInMode(latestMessages, GUIDED_MODE_NUMBER):
            print("GUIDED ended during takeoff.")
            return False
        if not gpsIsUsableForPositionHold(latestMessages, messageArrivedAtSeconds):
            print("GPS position was lost during takeoff.")
            return False
        altitudeMeters = relativeAltitudeMeters(latestMessages, messageArrivedAtSeconds)
        positionArrivalSeconds = messageArrivedAtSeconds.get("GLOBAL_POSITION_INT")
        if altitudeMeters is not None and positionArrivalSeconds != lastPositionArrivalSeconds:
            lastPositionArrivalSeconds = positionArrivalSeconds
            climbedMeters = altitudeMeters - startingAltitudeMeters
            if climbedMeters >= MINIMUM_RECORDED_CLIMB_METERS:
                if not climbDetected:
                    climbDetected = True
                    print("Upward movement detected.")
            if climbDetected and altitudeMeters >= minimumTakeoffAltitudeMeters:
                consecutiveHeightSamples += 1
            else:
                consecutiveHeightSamples = 0
        currentTimeSeconds = time.monotonic()
        if currentTimeSeconds - lastPrintedAtSeconds >= STATUS_PRINT_INTERVAL_SECONDS:
            levelText = attitudeText(latestMessages, messageArrivedAtSeconds)
            if altitudeMeters is None:
                print(f"TAKEOFF | height waiting | {levelText}")
            else:
                print(
                    f"TAKEOFF | {altitudeMeters * FEET_PER_METER:.2f} ft | {levelText}"
                )
            lastPrintedAtSeconds = currentTimeSeconds
        if consecutiveHeightSamples >= REQUIRED_TAKEOFF_HEIGHT_SAMPLES:
            print("Takeoff height confirmed.")
            return True
        if not climbDetected and currentTimeSeconds >= climbStartDeadlineSeconds:
            print("No climb was seen after the takeoff command.")
            return False
        time.sleep(0.1)
    print(f"Did not reach {TARGET_ALTITUDE_FEET:.1f} feet.")
    return False


def runGpsHover(pixhawkConnection, latestMessages, messageArrivedAtSeconds):
    print()
    print(f"LOITER for {HOVER_TIME_SECONDS:.0f} seconds.")
    hoverStartedAtSeconds = time.monotonic()
    lastPrintedAtSeconds = 0.0
    while time.monotonic() - hoverStartedAtSeconds < HOVER_TIME_SECONDS:
        readAvailableMessages(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
        if not heartbeatIsFresh():
            raise RuntimeError("Pixhawk heartbeat was lost.")
        if copterIsArmed(latestMessages) is not True:
            raise RuntimeError("The copter became disarmed.")
        if not copterIsInMode(latestMessages, LOITER_MODE_NUMBER):
            raise RuntimeError("LOITER changed during the hover.")
        if not gpsIsUsableForPositionHold(latestMessages, messageArrivedAtSeconds):
            raise RuntimeError("GPS position was lost during the hover.")
        currentTimeSeconds = time.monotonic()
        if currentTimeSeconds - lastPrintedAtSeconds >= STATUS_PRINT_INTERVAL_SECONDS:
            altitudeMeters = relativeAltitudeMeters(latestMessages, messageArrivedAtSeconds)
            hoverElapsedSeconds = currentTimeSeconds - hoverStartedAtSeconds
            satelliteCount = gpsSatelliteCount(latestMessages, messageArrivedAtSeconds)
            if altitudeMeters is None:
                heightText = "height waiting"
            else:
                heightText = f"{altitudeMeters * FEET_PER_METER:.2f} ft"
            levelText = attitudeText(latestMessages, messageArrivedAtSeconds)
            print(
                f"LOITER | {heightText} | {levelText} | satellites {satelliteCount} | "
                f"hover {hoverElapsedSeconds:.0f}/{HOVER_TIME_SECONDS:.0f}s"
            )
            lastPrintedAtSeconds = currentTimeSeconds
        time.sleep(0.1)
    print()
    print(f"{HOVER_TIME_SECONDS:.0f} second GPS loiter complete.")


def commandLand(pixhawkConnection, latestMessages, messageArrivedAtSeconds):
    if not heartbeatIsFresh():
        print("Heartbeat is stale. LAND was not sent.")
        return False
    print()
    print("LAND")
    return setFlightModeAndWait(
        pixhawkConnection,
        latestMessages,
        messageArrivedAtSeconds,
        LAND_MODE_NUMBER,
        "LAND",
    )


def waitForLandingToFinish(pixhawkConnection, latestMessages, messageArrivedAtSeconds):
    landingStartedAtSeconds = time.monotonic()
    lastPrintedAtSeconds = 0.0
    while time.monotonic() - landingStartedAtSeconds < LAND_WAIT_TIMEOUT_SECONDS:
        readAvailableMessages(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
        if copterIsArmed(latestMessages) is False:
            print("Motors disarmed.")
            return True
        currentTimeSeconds = time.monotonic()
        if currentTimeSeconds - lastPrintedAtSeconds >= STATUS_PRINT_INTERVAL_SECONDS:
            altitudeMeters = relativeAltitudeMeters(latestMessages, messageArrivedAtSeconds)
            levelText = attitudeText(latestMessages, messageArrivedAtSeconds)
            if altitudeMeters is None:
                print(f"LAND | height waiting | {levelText}")
            else:
                print(f"LAND | {altitudeMeters * FEET_PER_METER:.2f} ft | {levelText}")
            lastPrintedAtSeconds = currentTimeSeconds
        time.sleep(0.1)
    print("Landing wait ended before disarm was confirmed.")
    return False


def main():
    pixhawkConnection = None
    latestMessages = {}
    messageArrivedAtSeconds = {}
    aircraftWasArmed = False
    originalTakeoffSlewSeconds = None
    flightLogFile = None
    flightLogPath = None
    originalStandardOutput = sys.stdout

    try:
        flightLogFile, flightLogPath = openFlightLog()
        print()
        print("==============================================")
        print("60 SECOND GPS LOITER")
        print("==============================================")
        print()
        print(f"Target altitude: {TARGET_ALTITUDE_FEET:.1f} ft")
        print(f"Hover duration: {HOVER_TIME_SECONDS:.0f} seconds")
        print("Motor spins first, then GUIDED takeoff, LOITER, and LAND.")
        print()

        pixhawkConnection = connectToPixhawk()
        requestRequiredTelemetry(pixhawkConnection)
        if not quadXFrameIsReady(pixhawkConnection, latestMessages, messageArrivedAtSeconds):
            raise RuntimeError("Quad X frame was not confirmed. Not arming.")
        if not spinMotorsOneByOne(pixhawkConnection, latestMessages, messageArrivedAtSeconds):
            return
        waitBeforeHoverQuestion(pixhawkConnection, latestMessages, messageArrivedAtSeconds)

        print()
        print("Type YES to begin the loiter.")
        if input("> ").strip() != "YES":
            print("Cancelled. The loiter was not started.")
            return

        # The spins and the question can sit for a while. Read the buffer
        # again so the takeoff uses the messages from after YES.
        readAvailableMessages(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
        if not waitForGpsPosition(pixhawkConnection, latestMessages, messageArrivedAtSeconds):
            raise RuntimeError("GPS is not ready for position hold. Not arming.")

        originalTakeoffSlewSeconds = prepareFirmerGuidedTakeoff(
            pixhawkConnection,
            latestMessages,
            messageArrivedAtSeconds,
        )
        if not setFlightModeAndWait(
            pixhawkConnection,
            latestMessages,
            messageArrivedAtSeconds,
            GUIDED_MODE_NUMBER,
            "GUIDED",
        ):
            raise RuntimeError("GUIDED was not confirmed.")
        readAvailableMessages(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
        if not copterIsLevelForTakeoff(latestMessages, messageArrivedAtSeconds):
            return
        if not armAndWaitForConfirmation(
            pixhawkConnection,
            latestMessages,
            messageArrivedAtSeconds,
        ):
            return
        aircraftWasArmed = True

        readAvailableMessages(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
        if not copterIsLevelForTakeoff(latestMessages, messageArrivedAtSeconds):
            commandLand(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
            waitForLandingToFinish(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
            return
        startingAltitudeMeters = relativeAltitudeMeters(latestMessages, messageArrivedAtSeconds)
        if startingAltitudeMeters is None:
            print("Height was not available. Takeoff was not sent.")
            commandLand(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
            waitForLandingToFinish(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
            return
        print(f"Starting height: {startingAltitudeMeters * FEET_PER_METER:.2f} ft")
        commandGpsTakeoff(pixhawkConnection)
        if not waitForTakeoffAltitude(
            pixhawkConnection,
            latestMessages,
            messageArrivedAtSeconds,
            startingAltitudeMeters,
        ):
            commandLand(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
            waitForLandingToFinish(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
            return

        if not setFlightModeAndWait(
            pixhawkConnection,
            latestMessages,
            messageArrivedAtSeconds,
            LOITER_MODE_NUMBER,
            "LOITER",
        ):
            raise RuntimeError("LOITER was not confirmed.")

        runGpsHover(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
        commandLand(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
        waitForLandingToFinish(pixhawkConnection, latestMessages, messageArrivedAtSeconds)

    except KeyboardInterrupt:
        print()
        print("Ctrl+C.")
        stopActiveMotorTest(pixhawkConnection)
        if pixhawkConnection is not None and aircraftWasArmed and heartbeatIsFresh():
            try:
                commandLand(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
                waitForLandingToFinish(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
            except KeyboardInterrupt:
                print("Second Ctrl+C. Closing the link. The copter is not force-disarmed.")

    except Exception as error:
        print()
        print(f"STOPPED: {error}")
        stopActiveMotorTest(pixhawkConnection)
        if pixhawkConnection is not None and aircraftWasArmed and heartbeatIsFresh():
            try:
                commandLand(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
                waitForLandingToFinish(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
            except Exception as landingError:
                print(f"Landing error: {landingError}")

    finally:
        stopActiveMotorTest(pixhawkConnection)
        if (
            pixhawkConnection is not None
            and originalTakeoffSlewSeconds is not None
            and heartbeatIsFresh()
        ):
            try:
                restoreTakeoffSlew(
                    pixhawkConnection,
                    latestMessages,
                    messageArrivedAtSeconds,
                    originalTakeoffSlewSeconds,
                )
            except Exception as restoreError:
                print(f"Could not restore {TAKEOFF_SLEW_PARAMETER_NAME}: {restoreError}")
        if pixhawkConnection is not None:
            try:
                pixhawkConnection.close()
            except Exception:
                pass
            print("Pixhawk connection closed.")
        print("Finished.")
        if flightLogPath is not None:
            print(f"Flight log saved: {flightLogPath}")
        sys.stdout = originalStandardOutput
        if flightLogFile is not None:
            flightLogFile.close()


if __name__ == "__main__":
    main()
