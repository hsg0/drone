"""
WHAT:
    Turn one named hand gesture into one Pixhawk flight command.
    The gesture names are the same ones markGesture already returns.

WHY:
    The camera can see a pose. This file is the only place that pose
    becomes a climb, a descent, a move, a hover, a takeoff, or a landing.
    The Pixhawk still mixes the motors. This file never spins one motor.

HOW:
    1. allowGestureFlight opens /dev/serial0 and permits commands. It does not arm.
    2. flyGesture receives one name: takeoff-land, fly-up, fly-down,
       move-away, move-closer, hover, or not-seen.
    3. takeoff-land arms and climbs only on the first sight of that pose.
       The same pose later, after the arms have come down, commands LAND.
    4. fly-up, fly-down, move-away, and move-closer send one body-frame speed.
       The caller repeats flyGesture while the pose is held.
    5. hover asks for LOITER so GPS holds the spot.
    6. not-seen sends no new move. A speed that was already running is zeroed once.

IMPORTANT:
    Importing this file does not connect, arm, or fly.
    This copy is not connected to a web route. The gesture page cannot call it.
    Gesture flight stays off until allowGestureFlight() is called.
    The wide-V pose is the arm and takeoff from hovering30withgps_v1.py.
    Arms down is LOITER, the same GPS hold that script uses after the climb.
    This file does not spin the motors one by one and does not time a 10 second hover.
    The live camera sends the pose only after gesture flight is allowed.
    Run this on the Pi. /dev/serial0 is pins 8 and 10 at 57600. Pin 6 is ground.
    Leave both USB ports alone. bootDisplay.py owns the Pixhawk USB port.
    Takeoff needs a 3D GPS fix, at least 4 satellites, a healthy position
    estimate, and a saved Quad X frame. A wrong frame would tip on liftoff.
    The arm command is a normal arm. This file never force-arms and never
    force-disarms.
    The camera points forward along the nose. Fly closer is toward that view.
    Fly farther is away from that view.
    Positive body z is down, so a climb is a negative z speed.
    Ctrl+C is not handled here. Call stopGestureFlight() to LAND and close.
"""

import os
import time

from pymavlink import mavutil as middleMan


# ---------------------------------------------------------------------------
# Connection settings
# ---------------------------------------------------------------------------

# Pin 8 transmits. Pin 10 receives. Pin 6 is ground.
PIXHAWK_UART_DEVICE = "/dev/serial0"
PIXHAWK_UART_BAUD = 57600
CONNECTION_TIMEOUT_SECONDS = 30
HEARTBEAT_STALE_AFTER_SECONDS = 3.0
SENSOR_MESSAGE_STALE_AFTER_SECONDS = 1.0
MODE_CHANGE_TIMEOUT_SECONDS = 10
ARM_CONFIRMATION_TIMEOUT_SECONDS = 10
TAKEOFF_TIMEOUT_SECONDS = 20
LAND_WAIT_TIMEOUT_SECONDS = 30
PARAMETER_TIMEOUT_SECONDS = 4.0
POLL_INTERVAL_SECONDS = 0.1

# ArduPilot on this UART does not stream GPS until each message is requested.
MESSAGE_INTERVAL_SECONDS = 0.25
MICROSECONDS_PER_SECOND = 1000000
FLIGHT_MESSAGE_NAMES = (
    "HEARTBEAT",
    "GPS_RAW_INT",
    "GLOBAL_POSITION_INT",
    "EKF_STATUS_REPORT",
    "STATUSTEXT",
)

# Copter custom_mode numbers. These are not plane mode numbers.
GUIDED_MODE_NUMBER = 4
LOITER_MODE_NUMBER = 5
LAND_MODE_NUMBER = 9

# FRAME_CLASS 1 is Quad. FRAME_TYPE 1 is X, not Plus.
QUAD_FRAME_CLASS = 1
QUAD_X_FRAME_TYPE = 1

# EKF_STATUS_REPORT bits. A GPS glitch or a constant-position guess is not a hold.
EKF_POSITION_HORIZONTAL_ABSOLUTE_BIT = 16
EKF_CONSTANT_POSITION_MODE_BIT = 128
EKF_GPS_GLITCHING_BIT = 32768
UNKNOWN_SATELLITE_COUNT = 255
MINIMUM_GPS_FIX_TYPE = 3
MINIMUM_SATELLITE_COUNT = 4
GPS_FIX_TYPE_NAMES = {
    0: "no GPS",
    1: "no fix",
    2: "2D fix",
    3: "3D fix",
    4: "DGPS fix",
    5: "RTK float",
    6: "RTK fixed",
}

# Velocity only, in the copter's own frame: x forward, y right, z down.
# Position, acceleration, and yaw are ignored so this request does not turn the copter.
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


# ---------------------------------------------------------------------------
# Flight settings
# ---------------------------------------------------------------------------

# 2.5 feet. The takeoff gesture climbs to this height, then LOITER holds it.
FEET_PER_METER = 3.28084
TAKEOFF_ALTITUDE_FEET = 2.5
TAKEOFF_ALTITUDE_METERS = TAKEOFF_ALTITUDE_FEET / FEET_PER_METER
TAKEOFF_ALTITUDE_REACHED_FRACTION = 0.90
# Fly-down stops here. LAND is the only way to the ground.
MINIMUM_ALTITUDE_METERS = 1.0
# Fly-up stops here so a held pose cannot climb away.
MAXIMUM_ALTITUDE_METERS = 1.3
CLIMB_RATE_METERS_PER_SECOND = 0.3
DESCEND_RATE_METERS_PER_SECOND = 0.3
HORIZONTAL_SPEED_METERS_PER_SECOND = 0.4
# A slower takeoff ramp leaves one side on the ground long enough to tip.
PREFERRED_TAKEOFF_SLEW_SECONDS = 1.0
TAKEOFF_SLEW_PARAMETER_NAME = "TKOFF_SLEW_TIME"

# Names returned by gestureFromArms in capturePicVidController.py.
GESTURE_TAKEOFF_OR_LAND = "takeoff-land"
GESTURE_FLY_UP = "fly-up"
GESTURE_FLY_DOWN = "fly-down"
GESTURE_FLY_FARTHER = "move-away"
GESTURE_FLY_CLOSER = "move-closer"
GESTURE_HOVER = "hover"
GESTURE_NOT_SEEN = "not-seen"
MOVING_GESTURE_NAMES = (
    GESTURE_FLY_UP,
    GESTURE_FLY_DOWN,
    GESTURE_FLY_FARTHER,
    GESTURE_FLY_CLOSER,
)


# ---------------------------------------------------------------------------
# Link state. Importing this file leaves every one of these idle.
# ---------------------------------------------------------------------------

pixhawkConnection = None
latestMessages = {}
messageArrivedAtSeconds = {}
lastHeartbeatAtSeconds = None
gestureFlightIsAllowed = False
lastGestureName = None
copterIsAirborne = False
originalTakeoffSlewSeconds = None
lastPrintedFlightLine = None
stopWasRequested = False
flightLinkClosedReason = "The command cable did not open."


def flightResult(gestureName, accepted, action):
    """Print one line when the result changes, and hand the same result back."""
    global lastPrintedFlightLine
    if accepted:
        flightLine = f"Gesture {gestureName}: {action}"
    else:
        flightLine = f"Gesture {gestureName} refused. {action}"
    # A held pose calls this many times a second. Print only when the words change.
    if flightLine != lastPrintedFlightLine:
        print(flightLine)
        lastPrintedFlightLine = flightLine
    return {
        "gesture": gestureName,
        "accepted": accepted,
        "action": action,
    }


# ---------------------------------------------------------------------------
# Messages from the Pixhawk
# ---------------------------------------------------------------------------

def acceptMessage(message):
    """Keep the newest message of each kind. Ignore a broken byte."""
    global lastHeartbeatAtSeconds
    if message is None or pixhawkConnection is None:
        return
    if message.get_type() == "BAD_DATA":
        return
    if message.get_srcSystem() not in (0, pixhawkConnection.target_system):
        return
    messageType = message.get_type()
    latestMessages[messageType] = message
    messageArrivedAtSeconds[messageType] = time.monotonic()
    if messageType == "HEARTBEAT" and message.get_srcSystem() == pixhawkConnection.target_system:
        lastHeartbeatAtSeconds = time.monotonic()
    elif messageType == "STATUSTEXT":
        statusText = getattr(message, "text", "")
        if isinstance(statusText, bytes):
            statusText = statusText.decode("ascii", "ignore")
        statusText = str(statusText).strip("\x00").strip()
        if statusText:
            print(f"PIXHAWK: {statusText}")


def drainMessages():
    """Read every message already waiting. Do not pause for a new one."""
    if pixhawkConnection is None:
        return
    while True:
        message = pixhawkConnection.recv_match(blocking=False)
        if message is None:
            return
        acceptMessage(message)


def waitForNextMessage(waitSeconds):
    """Pause until one message arrives, then store it."""
    if pixhawkConnection is None:
        return None
    message = pixhawkConnection.recv_match(blocking=True, timeout=waitSeconds)
    if message is None:
        return None
    acceptMessage(message)
    return message


def heartbeatIsFresh():
    """True when a heartbeat arrived within the last 3 seconds."""
    if lastHeartbeatAtSeconds is None:
        return False
    heartbeatAgeSeconds = time.monotonic() - lastHeartbeatAtSeconds
    return heartbeatAgeSeconds <= HEARTBEAT_STALE_AFTER_SECONDS


def messageIsFresh(messageType):
    """True when this message arrived within the last second."""
    arrivedAtSeconds = messageArrivedAtSeconds.get(messageType)
    if arrivedAtSeconds is None:
        return False
    return time.monotonic() - arrivedAtSeconds <= SENSOR_MESSAGE_STALE_AFTER_SECONDS


def copterIsArmed():
    """None until a heartbeat exists. False means the motors are disarmed."""
    heartbeat = latestMessages.get("HEARTBEAT")
    if heartbeat is None:
        return None
    return bool(heartbeat.base_mode & middleMan.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def copterIsInMode(modeNumber):
    """True when the newest heartbeat says the copter is in this mode."""
    heartbeat = latestMessages.get("HEARTBEAT")
    if heartbeat is None:
        return False
    return int(heartbeat.custom_mode) == int(modeNumber)


def satelliteCount():
    """How many satellites the newest GPS message reports. None if that count is missing."""
    if not messageIsFresh("GPS_RAW_INT"):
        return None
    gpsReading = latestMessages.get("GPS_RAW_INT")
    if gpsReading is None:
        return None
    count = getattr(gpsReading, "satellites_visible", None)
    if count is None or count == UNKNOWN_SATELLITE_COUNT:
        return None
    return int(count)


def gpsHoldIsReady():
    """True for a 3D fix, at least 4 satellites, and a real horizontal position."""
    if not messageIsFresh("GPS_RAW_INT"):
        return False
    gpsReading = latestMessages.get("GPS_RAW_INT")
    if gpsReading is None or gpsReading.fix_type is None:
        return False
    if int(gpsReading.fix_type) < MINIMUM_GPS_FIX_TYPE:
        return False
    count = satelliteCount()
    if count is None or count < MINIMUM_SATELLITE_COUNT:
        return False
    if not messageIsFresh("EKF_STATUS_REPORT"):
        return False
    ekfStatus = latestMessages.get("EKF_STATUS_REPORT")
    if ekfStatus is None:
        return False
    flags = int(ekfStatus.flags)
    hasAbsoluteHorizontalPosition = bool(flags & EKF_POSITION_HORIZONTAL_ABSOLUTE_BIT)
    isInConstantPositionMode = bool(flags & EKF_CONSTANT_POSITION_MODE_BIT)
    gpsGlitchIsActive = bool(flags & EKF_GPS_GLITCHING_BIT)
    return hasAbsoluteHorizontalPosition and not isInConstantPositionMode and not gpsGlitchIsActive


def gpsStatusSentence():
    """The GPS facts the takeoff check just used, so a refused arm says why."""
    if not messageIsFresh("GPS_RAW_INT"):
        return "No fresh GPS reading."
    gpsReading = latestMessages.get("GPS_RAW_INT")
    if gpsReading is None or gpsReading.fix_type is None:
        return "GPS fix type is missing."
    fixType = int(gpsReading.fix_type)
    fixName = GPS_FIX_TYPE_NAMES.get(fixType, f"fix type {fixType}")
    count = satelliteCount()
    if count is None:
        satelliteText = "satellite count missing"
    else:
        satelliteText = f"{count} satellites"
    if not messageIsFresh("EKF_STATUS_REPORT"):
        return f"{fixName}, {satelliteText}. The position estimate is missing."
    ekfStatus = latestMessages.get("EKF_STATUS_REPORT")
    flags = int(ekfStatus.flags) if ekfStatus is not None else 0
    if not (flags & EKF_POSITION_HORIZONTAL_ABSOLUTE_BIT):
        return f"{fixName}, {satelliteText}. Horizontal position is not trusted yet."
    if flags & EKF_CONSTANT_POSITION_MODE_BIT:
        return f"{fixName}, {satelliteText}. The position is a guess, not GPS."
    if flags & EKF_GPS_GLITCHING_BIT:
        return f"{fixName}, {satelliteText}. GPS is glitching."
    return f"{fixName}, {satelliteText}."


def gpsNotReadySentence():
    """Why takeoff will not arm until the GPS hold is real."""
    return (
        "GPS position is not ready. Need a 3D fix and at least 4 satellites. "
        + gpsStatusSentence()
    )


def heightMeters():
    """Height above the spot where the Pixhawk was turned on, in meters."""
    if not messageIsFresh("GLOBAL_POSITION_INT"):
        return None
    positionMessage = latestMessages.get("GLOBAL_POSITION_INT")
    if positionMessage is None:
        return None
    relativeAltitudeMillimeters = getattr(positionMessage, "relative_alt", None)
    if relativeAltitudeMillimeters is None:
        return None
    return float(relativeAltitudeMillimeters) / 1000.0


def heightFeetText():
    """One short height phrase for the log line."""
    altitudeMeters = heightMeters()
    if altitudeMeters is None:
        return "height waiting"
    return f"{altitudeMeters * FEET_PER_METER:.1f} ft"


# ---------------------------------------------------------------------------
# Connection
# ---------------------------------------------------------------------------

def openFlightLink():
    """Open the command cable. This does not arm the copter."""
    global pixhawkConnection
    global lastHeartbeatAtSeconds
    global flightLinkClosedReason
    if pixhawkConnection is not None:
        return pixhawkConnection
    if not os.path.exists(PIXHAWK_UART_DEVICE):
        flightLinkClosedReason = (
            "This computer has no command cable. "
            "Pins 8 and 10 are on the Pi. "
            "Choose Pi (192.168.0.138) at the top, and run the backend on the Pi."
        )
        print(flightLinkClosedReason)
        return None
    print(f"Connecting to Pixhawk on {PIXHAWK_UART_DEVICE} at {PIXHAWK_UART_BAUD} baud.")
    try:
        link = middleMan.mavlink_connection(
            PIXHAWK_UART_DEVICE,
            baud=PIXHAWK_UART_BAUD,
            autoreconnect=False,
        )
    except Exception as error:
        flightLinkClosedReason = f"The command cable did not open. {error}"
        print(flightLinkClosedReason)
        return None
    heartbeat = link.wait_heartbeat(timeout=CONNECTION_TIMEOUT_SECONDS)
    if heartbeat is None:
        flightLinkClosedReason = "No Pixhawk heartbeat on the command cable."
        print(flightLinkClosedReason)
        link.close()
        return None
    # A heartbeat can report component 0. Commands need the autopilot component.
    if link.target_component == 0:
        link.target_component = 1
    pixhawkConnection = link
    lastHeartbeatAtSeconds = time.monotonic()
    acceptMessage(heartbeat)
    print(
        f"Connected. Heartbeat from system {link.target_system}, "
        f"component {link.target_component}."
    )
    requestFlightMessages()
    return link


def closeFlightLink():
    """Close the command cable. Call this only after the motors are disarmed."""
    global pixhawkConnection
    global gestureFlightIsAllowed
    if pixhawkConnection is None:
        gestureFlightIsAllowed = False
        return
    pixhawkConnection.close()
    pixhawkConnection = None
    gestureFlightIsAllowed = False
    print("Command cable closed.")


def requestFlightMessages():
    """Ask the Pixhawk to send heartbeat, GPS, height, and status text."""
    intervalMicroseconds = int(MESSAGE_INTERVAL_SECONDS * MICROSECONDS_PER_SECOND)
    for messageName in FLIGHT_MESSAGE_NAMES:
        messageId = getattr(middleMan.mavlink, f"MAVLINK_MSG_ID_{messageName}", None)
        if messageId is None:
            print(f"MAVLink message id for {messageName} was not found.")
            continue
        pixhawkConnection.mav.command_long_send(
            pixhawkConnection.target_system,
            pixhawkConnection.target_component,
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
    print("Requested GPS and height telemetry.")


def closedCableReason():
    """The last reason the command cable stayed closed."""
    return flightLinkClosedReason


def allowGestureFlight():
    """Open the link and permit gesture commands. The copter stays disarmed."""
    global gestureFlightIsAllowed
    if openFlightLink() is None:
        gestureFlightIsAllowed = False
        print("Gesture flight was not allowed. The command cable is closed.")
        return False
    gestureFlightIsAllowed = True
    print("Gesture flight is allowed. The copter is not armed yet.")
    return True


def requestStopGesture():
    """Ask a takeoff that is already running to stop. The website calls this before it waits."""
    global stopWasRequested
    stopWasRequested = True
    print("Stop requested.")


def takeoffStopResult():
    """Land when stop was pressed during takeoff. None means the climb may continue."""
    if not stopWasRequested:
        return None
    if copterIsArmed() is True:
        commandLand()
    return flightResult(GESTURE_TAKEOFF_OR_LAND, False, "Stop was requested. Takeoff ended.")


def stopGestureFlight():
    """LAND if the motors are armed, then close the command cable."""
    global gestureFlightIsAllowed
    global copterIsAirborne
    global stopWasRequested
    gestureFlightIsAllowed = False
    drainMessages()
    if copterIsArmed() is True:
        print("Stopping gesture flight. Requesting LAND.")
        commandLand()
    copterIsAirborne = False
    closeFlightLink()
    stopWasRequested = False


# ---------------------------------------------------------------------------
# Shared command path
# ---------------------------------------------------------------------------

def requestMode(modeNumber, modeName):
    """Ask for a mode and wait until the heartbeat confirms it."""
    print(f"Requesting {modeName}.")
    pixhawkConnection.mav.set_mode_send(
        pixhawkConnection.target_system,
        middleMan.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        modeNumber,
    )
    deadlineSeconds = time.monotonic() + MODE_CHANGE_TIMEOUT_SECONDS
    while time.monotonic() < deadlineSeconds:
        drainMessages()
        if copterIsInMode(modeNumber):
            print(f"{modeName} confirmed.")
            return True
        if not heartbeatIsFresh():
            print("Heartbeat became stale while waiting for the mode change.")
            return False
        time.sleep(POLL_INTERVAL_SECONDS)
    print(f"{modeName} was not confirmed.")
    return False


def sendBodyVelocity(forwardMetersPerSecond, rightMetersPerSecond, downMetersPerSecond):
    """Ask GUIDED for a body-frame speed. Forward is toward the camera view. Down is positive."""
    pixhawkConnection.mav.set_position_target_local_ned_send(
        0,
        pixhawkConnection.target_system,
        pixhawkConnection.target_component,
        middleMan.mavlink.MAV_FRAME_BODY_NED,
        VELOCITY_ONLY_TYPE_MASK,
        0,
        0,
        0,
        forwardMetersPerSecond,
        rightMetersPerSecond,
        downMetersPerSecond,
        0,
        0,
        0,
        0,
        0,
    )


def waitForGpsHold(waitSeconds):
    """Listen until the position check passes. A parameter read can let the last GPS message go stale."""
    deadlineSeconds = time.monotonic() + waitSeconds
    while time.monotonic() < deadlineSeconds:
        drainMessages()
        if not heartbeatIsFresh():
            return False
        if gpsHoldIsReady():
            return True
        time.sleep(POLL_INTERVAL_SECONDS)
    return False


def movementIsBlocked():
    """A sentence explaining why a move must not be sent. None means the move may go out."""
    if not gestureFlightIsAllowed:
        return "Gesture flight is not allowed."
    if pixhawkConnection is None:
        return "The command cable is closed."
    drainMessages()
    if not heartbeatIsFresh():
        return "The Pixhawk heartbeat is missing or stale."
    if copterIsArmed() is not True:
        return "The copter is not armed. The takeoff pose arms it."
    if not copterIsAirborne:
        return "The copter has not finished takeoff."
    if not gpsHoldIsReady():
        return "GPS position is not ready. Need a 3D fix and at least 4 satellites."
    return None


def ensureGuidedMode():
    """Velocity commands are flown in GUIDED. LOITER ignores this speed request."""
    if copterIsInMode(GUIDED_MODE_NUMBER):
        return True
    return requestMode(GUIDED_MODE_NUMBER, "GUIDED")


def parameterNameText(rawParameterName):
    """Turn a parameter name from the Pixhawk into plain text."""
    if isinstance(rawParameterName, bytes):
        rawParameterName = rawParameterName.decode("ascii", "ignore")
    return str(rawParameterName).strip("\x00").strip()


def readParameter(parameterName):
    """Read one Pixhawk setting. None means it did not answer in time."""
    pixhawkConnection.mav.param_request_read_send(
        pixhawkConnection.target_system,
        pixhawkConnection.target_component,
        parameterName.encode("ascii"),
        -1,
    )
    deadlineSeconds = time.monotonic() + PARAMETER_TIMEOUT_SECONDS
    while time.monotonic() < deadlineSeconds:
        message = waitForNextMessage(0.4)
        if message is None or message.get_type() != "PARAM_VALUE":
            continue
        if parameterNameText(message.param_id) == parameterName:
            return float(message.param_value)
    return None


def writeParameter(parameterName, requestedValue):
    """Write one Pixhawk setting and wait until the Pixhawk repeats that value."""
    parameterMatchTolerance = 0.01
    pixhawkConnection.mav.param_set_send(
        pixhawkConnection.target_system,
        pixhawkConnection.target_component,
        parameterName.encode("ascii"),
        float(requestedValue),
        middleMan.mavlink.MAV_PARAM_TYPE_REAL32,
    )
    deadlineSeconds = time.monotonic() + PARAMETER_TIMEOUT_SECONDS
    while time.monotonic() < deadlineSeconds:
        message = waitForNextMessage(0.4)
        if message is None or message.get_type() != "PARAM_VALUE":
            continue
        if parameterNameText(message.param_id) != parameterName:
            continue
        returnedValue = float(message.param_value)
        if abs(returnedValue - requestedValue) < parameterMatchTolerance:
            return True
    return False


def quadXFrameIsSaved():
    """True only when the Pixhawk is saved as Quad X. A different frame would tip."""
    frameClass = readParameter("FRAME_CLASS")
    frameType = readParameter("FRAME_TYPE")
    if frameClass is None or frameType is None:
        print("FRAME_CLASS or FRAME_TYPE could not be read. Not arming.")
        return False
    print(f"FRAME_CLASS {frameClass:.0f} | FRAME_TYPE {frameType:.0f}")
    frameIsQuad = abs(frameClass - QUAD_FRAME_CLASS) < 0.01
    frameIsX = abs(frameType - QUAD_X_FRAME_TYPE) < 0.01
    if not frameIsQuad or not frameIsX:
        print("The Pixhawk is not set to Quad X. Not arming.")
        return False
    print("Frame is Quad X.")
    return True


def shortenSlowTakeoffRamp():
    """Make the takeoff ramp firmer only when the saved ramp is slower than 1 second."""
    global originalTakeoffSlewSeconds
    savedTakeoffSlewSeconds = readParameter(TAKEOFF_SLEW_PARAMETER_NAME)
    if savedTakeoffSlewSeconds is None:
        print(f"{TAKEOFF_SLEW_PARAMETER_NAME} was not available. The existing ramp will be used.")
        return
    temporaryTakeoffSlewSeconds = min(savedTakeoffSlewSeconds, PREFERRED_TAKEOFF_SLEW_SECONDS)
    print(
        f"{TAKEOFF_SLEW_PARAMETER_NAME}: "
        f"original {savedTakeoffSlewSeconds:.2f}s | "
        f"takeoff {temporaryTakeoffSlewSeconds:.2f}s"
    )
    if abs(temporaryTakeoffSlewSeconds - savedTakeoffSlewSeconds) < 0.01:
        originalTakeoffSlewSeconds = None
        return
    slewChanged = writeParameter(TAKEOFF_SLEW_PARAMETER_NAME, temporaryTakeoffSlewSeconds)
    if slewChanged:
        originalTakeoffSlewSeconds = savedTakeoffSlewSeconds
        return
    print(f"Could not confirm {TAKEOFF_SLEW_PARAMETER_NAME}. The existing ramp will be used.")


def restoreTakeoffRamp():
    """Put the original takeoff ramp back when this file shortened it."""
    global originalTakeoffSlewSeconds
    if originalTakeoffSlewSeconds is None or pixhawkConnection is None:
        return
    restored = writeParameter(TAKEOFF_SLEW_PARAMETER_NAME, originalTakeoffSlewSeconds)
    if restored:
        print(f"{TAKEOFF_SLEW_PARAMETER_NAME} restored.")
        originalTakeoffSlewSeconds = None
    else:
        print(f"Warning: {TAKEOFF_SLEW_PARAMETER_NAME} restore could not be confirmed.")


def armForGestureTakeoff():
    """Send a normal arm and wait until the heartbeat shows the motors are armed."""
    print("Sending ARM command.")
    pixhawkConnection.mav.command_long_send(
        pixhawkConnection.target_system,
        pixhawkConnection.target_component,
        middleMan.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
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
    while time.monotonic() < deadlineSeconds:
        message = waitForNextMessage(0.5)
        if copterIsArmed() is True:
            print("ARMED CONFIRMED.")
            return True
        if message is None or message.get_type() != "COMMAND_ACK":
            continue
        if message.command != middleMan.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            continue
        if message.result not in (
            middleMan.mavlink.MAV_RESULT_ACCEPTED,
            middleMan.mavlink.MAV_RESULT_IN_PROGRESS,
        ):
            print(f"ARM FAILED. Answer = {message.result}")
            return False
    print("ARM FAILED. The motors did not arm in time.")
    return False


# ---------------------------------------------------------------------------
# One function for each gesture
# ---------------------------------------------------------------------------

def commandTakeoff():
    """GUIDED, arm, climb to 2.5 feet, then LOITER."""
    global copterIsAirborne
    stopped = takeoffStopResult()
    if stopped is not None:
        return stopped
    drainMessages()
    if not heartbeatIsFresh():
        return flightResult(GESTURE_TAKEOFF_OR_LAND, False, "The Pixhawk heartbeat is missing or stale.")
    if copterIsArmed() is True:
        return flightResult(GESTURE_TAKEOFF_OR_LAND, False, "The copter is already armed. Not taking off.")
    if not gpsHoldIsReady():
        return flightResult(GESTURE_TAKEOFF_OR_LAND, False, gpsNotReadySentence())
    if not quadXFrameIsSaved():
        return flightResult(GESTURE_TAKEOFF_OR_LAND, False, "The frame is not Quad X.")
    shortenSlowTakeoffRamp()
    if not waitForGpsHold(5):
        return flightResult(
            GESTURE_TAKEOFF_OR_LAND,
            False,
            "GPS position was not ready after the frame check. " + gpsStatusSentence(),
        )
    stopped = takeoffStopResult()
    if stopped is not None:
        return stopped
    if not requestMode(GUIDED_MODE_NUMBER, "GUIDED"):
        return flightResult(GESTURE_TAKEOFF_OR_LAND, False, "GUIDED was not confirmed.")
    stopped = takeoffStopResult()
    if stopped is not None:
        return stopped
    if not armForGestureTakeoff():
        return flightResult(GESTURE_TAKEOFF_OR_LAND, False, "The motors did not arm.")
    print(f"Taking off to {TAKEOFF_ALTITUDE_METERS * FEET_PER_METER:.1f} ft.")
    pixhawkConnection.mav.command_long_send(
        pixhawkConnection.target_system,
        pixhawkConnection.target_component,
        middleMan.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        TAKEOFF_ALTITUDE_METERS,
    )
    minimumTakeoffAltitudeMeters = TAKEOFF_ALTITUDE_METERS * TAKEOFF_ALTITUDE_REACHED_FRACTION
    deadlineSeconds = time.monotonic() + TAKEOFF_TIMEOUT_SECONDS
    lastPrintedAtSeconds = 0.0
    while time.monotonic() < deadlineSeconds:
        stopped = takeoffStopResult()
        if stopped is not None:
            return stopped
        drainMessages()
        if not heartbeatIsFresh():
            commandLand()
            return flightResult(GESTURE_TAKEOFF_OR_LAND, False, "Heartbeat was lost during takeoff.")
        if copterIsArmed() is not True:
            copterIsAirborne = False
            return flightResult(GESTURE_TAKEOFF_OR_LAND, False, "The copter disarmed during takeoff.")
        if not copterIsInMode(GUIDED_MODE_NUMBER):
            commandLand()
            return flightResult(GESTURE_TAKEOFF_OR_LAND, False, "GUIDED ended during takeoff.")
        if not gpsHoldIsReady():
            commandLand()
            return flightResult(GESTURE_TAKEOFF_OR_LAND, False, "GPS position was lost during takeoff.")
        altitudeMeters = heightMeters()
        currentTimeSeconds = time.monotonic()
        if currentTimeSeconds - lastPrintedAtSeconds >= 1.0:
            print(f"TAKEOFF | {heightFeetText()}")
            lastPrintedAtSeconds = currentTimeSeconds
        if altitudeMeters is not None and altitudeMeters >= minimumTakeoffAltitudeMeters:
            print("Takeoff height reached.")
            if not requestMode(LOITER_MODE_NUMBER, "LOITER"):
                commandLand()
                return flightResult(GESTURE_TAKEOFF_OR_LAND, False, "LOITER was not confirmed after takeoff.")
            copterIsAirborne = True
            return flightResult(GESTURE_TAKEOFF_OR_LAND, True, f"took off and holding at {heightFeetText()}")
        time.sleep(POLL_INTERVAL_SECONDS)
    commandLand()
    return flightResult(GESTURE_TAKEOFF_OR_LAND, False, "Did not reach takeoff height.")


def commandLand():
    """Ask for LAND and wait until the motors disarm."""
    global copterIsAirborne
    if pixhawkConnection is None:
        return flightResult(GESTURE_TAKEOFF_OR_LAND, False, "The command cable is closed.")
    if not heartbeatIsFresh():
        print("Heartbeat is stale. LAND was not sent.")
        return flightResult(GESTURE_TAKEOFF_OR_LAND, False, "The Pixhawk heartbeat is missing or stale.")
    print("LAND")
    requestMode(LAND_MODE_NUMBER, "LAND")
    landingStartedAtSeconds = time.monotonic()
    lastPrintedAtSeconds = 0.0
    while time.monotonic() - landingStartedAtSeconds < LAND_WAIT_TIMEOUT_SECONDS:
        drainMessages()
        if copterIsArmed() is False:
            print("Motors disarmed.")
            copterIsAirborne = False
            restoreTakeoffRamp()
            return flightResult(GESTURE_TAKEOFF_OR_LAND, True, "landed and disarmed")
        currentTimeSeconds = time.monotonic()
        if currentTimeSeconds - lastPrintedAtSeconds >= 1.0:
            print(f"LAND | {heightFeetText()}")
            lastPrintedAtSeconds = currentTimeSeconds
        time.sleep(POLL_INTERVAL_SECONDS)
    print("Landing wait ended before disarm was confirmed.")
    return flightResult(GESTURE_TAKEOFF_OR_LAND, False, "LAND was sent, but the motors are still armed.")


def takeOffOrLand():
    """Take off from the ground. Land once this file has finished a takeoff."""
    drainMessages()
    if copterIsArmed() is True and copterIsAirborne:
        return commandLand()
    if copterIsArmed() is True:
        return flightResult(
            GESTURE_TAKEOFF_OR_LAND,
            False,
            "Armed, but takeoff has not finished. Not taking off again.",
        )
    return commandTakeoff()


def flyUp():
    """Climb in GUIDED until the height limit, then hold."""
    blockedReason = movementIsBlocked()
    if blockedReason is not None:
        return flightResult(GESTURE_FLY_UP, False, blockedReason)
    if not ensureGuidedMode():
        return flightResult(GESTURE_FLY_UP, False, "GUIDED was not confirmed.")
    altitudeMeters = heightMeters()
    if altitudeMeters is None:
        return flightResult(GESTURE_FLY_UP, False, "Height is missing. Not climbing.")
    if altitudeMeters >= MAXIMUM_ALTITUDE_METERS:
        sendBodyVelocity(0, 0, 0)
        return flightResult(GESTURE_FLY_UP, True, "height limit, holding")
    # Body z is down, so a climb is a negative speed.
    sendBodyVelocity(0, 0, -CLIMB_RATE_METERS_PER_SECOND)
    return flightResult(GESTURE_FLY_UP, True, "climbing")


def flyDown():
    """Descend in GUIDED until the low limit. This does not land."""
    blockedReason = movementIsBlocked()
    if blockedReason is not None:
        return flightResult(GESTURE_FLY_DOWN, False, blockedReason)
    if not ensureGuidedMode():
        return flightResult(GESTURE_FLY_DOWN, False, "GUIDED was not confirmed.")
    altitudeMeters = heightMeters()
    if altitudeMeters is None:
        return flightResult(GESTURE_FLY_DOWN, False, "Height is missing. Not descending.")
    if altitudeMeters <= MINIMUM_ALTITUDE_METERS:
        sendBodyVelocity(0, 0, 0)
        return flightResult(GESTURE_FLY_DOWN, True, "low limit, holding")
    sendBodyVelocity(0, 0, DESCEND_RATE_METERS_PER_SECOND)
    return flightResult(GESTURE_FLY_DOWN, True, "descending")


def flyFarther():
    """Move backward, away from the person in the forward camera."""
    blockedReason = movementIsBlocked()
    if blockedReason is not None:
        return flightResult(GESTURE_FLY_FARTHER, False, blockedReason)
    if not ensureGuidedMode():
        return flightResult(GESTURE_FLY_FARTHER, False, "GUIDED was not confirmed.")
    sendBodyVelocity(-HORIZONTAL_SPEED_METERS_PER_SECOND, 0, 0)
    return flightResult(GESTURE_FLY_FARTHER, True, "flying farther")


def flyCloser():
    """Move forward, toward the person in the forward camera."""
    blockedReason = movementIsBlocked()
    if blockedReason is not None:
        return flightResult(GESTURE_FLY_CLOSER, False, blockedReason)
    if not ensureGuidedMode():
        return flightResult(GESTURE_FLY_CLOSER, False, "GUIDED was not confirmed.")
    sendBodyVelocity(HORIZONTAL_SPEED_METERS_PER_SECOND, 0, 0)
    return flightResult(GESTURE_FLY_CLOSER, True, "flying closer")


def hoverHere():
    """Stop the body speed and ask LOITER to hold the spot."""
    blockedReason = movementIsBlocked()
    if blockedReason is not None:
        return flightResult(GESTURE_HOVER, False, blockedReason)
    if not copterIsInMode(LOITER_MODE_NUMBER):
        if not requestMode(LOITER_MODE_NUMBER, "LOITER"):
            return flightResult(GESTURE_HOVER, False, "LOITER was not confirmed.")
    sendBodyVelocity(0, 0, 0)
    return flightResult(GESTURE_HOVER, True, "hovering")


def flyGesture(gestureName):
    """Run the flight command for one gesture name. A held takeoff pose does not also land."""
    global lastGestureName
    if not gestureFlightIsAllowed:
        return flightResult(gestureName, False, "Gesture flight is not allowed.")

    if gestureName == GESTURE_NOT_SEEN:
        # A missing body must not land the copter. Zero a speed that was still being sent.
        if (
            lastGestureName in MOVING_GESTURE_NAMES
            and copterIsAirborne
            and pixhawkConnection is not None
            and heartbeatIsFresh()
        ):
            sendBodyVelocity(0, 0, 0)
            print("Gesture not-seen. Speed set to zero.")
        lastGestureName = gestureName
        return {
            "gesture": gestureName,
            "accepted": True,
            "action": "no command",
        }

    if gestureName == GESTURE_TAKEOFF_OR_LAND:
        if gestureName == lastGestureName:
            return {
                "gesture": gestureName,
                "accepted": True,
                "action": "holding the takeoff or land pose",
            }
        lastGestureName = gestureName
        return takeOffOrLand()

    lastGestureName = gestureName
    if gestureName == GESTURE_FLY_UP:
        return flyUp()
    if gestureName == GESTURE_FLY_DOWN:
        return flyDown()
    if gestureName == GESTURE_FLY_FARTHER:
        return flyFarther()
    if gestureName == GESTURE_FLY_CLOSER:
        return flyCloser()
    if gestureName == GESTURE_HOVER:
        return hoverHere()
    return flightResult(gestureName, False, "That gesture has no flight command.")


def gestureFlightStatus():
    """Report the link and the copter. This does not send a flight command."""
    drainMessages()
    return {
        "allowed": gestureFlightIsAllowed,
        "connected": pixhawkConnection is not None,
        "armed": copterIsArmed() is True,
        "airborne": copterIsAirborne,
        "heightMeters": heightMeters(),
        "lastGesture": lastGestureName,
        "gps": gpsStatusSentence(),
    }
