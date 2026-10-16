#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    Take off to 1 foot, then land.
    GPS holds the horizontal position. The Pixhawk mixes the motors.

WHY:
    This is the short flight: connect, wait until the copter can arm,
    enter GUIDED, arm, climb, then LAND.
    A slow throttle ramp leaves one side on the ground long enough to tip,
    so a ramp slower than 1 second is made firmer for this climb only.

HOW:
    1. Connect on pins 8 and 10. Leave USB with the screen.
    2. Request heartbeat, GPS, height, position health, and status text.
    3. Read FRAME_CLASS and FRAME_TYPE. Stop if this is not a Quad X.
    4. Wait until GPS can hold the copter. Print the reason while waiting.
    5. Type YES to continue.
    6. Shorten TKOFF_SLEW_TIME if it is slower than 1 second.
    7. Enter GUIDED, arm, and climb to 1 foot.
    8. Enter LAND and wait until the motors disarm.
    9. Put TKOFF_SLEW_TIME back if this script changed it, then close the cable.

IMPORTANT:
    Run this on the Pi. /dev/serial0 is pins 8 and 10 at 57600. Pin 6 is ground.
    Leave both USB ports alone. bootDisplay.py owns the Pixhawk USB port.
    Type YES only when the copter is clear to fly. Anything else stops before arming.
    This copter is a DJI F450. The Pixhawk must still be saved as Quad X
    (FRAME_CLASS 1, FRAME_TYPE 1). That is the motor order the tests confirmed:
    motor 1 front-right counter-clockwise, motor 2 back-left counter-clockwise,
    motor 3 front-left clockwise, motor 4 back-right clockwise.
    DJI X is FRAME_TYPE 13. It swaps motor 2 and motor 3, so this script
    will not arm that setting. A wrong frame would tip on liftoff.
    Takeoff needs a 3D GPS fix, at least 4 satellites, and a trusted
    horizontal position.
    The arm command is a normal arm. This script never force-arms and never
    force-disarms.
    Compass and GPS arming checks stay on.
    Ctrl+C before arming just closes the cable.
    Ctrl+C after arming requests LAND. A second Ctrl+C closes the cable.
"""

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
PARAMETER_TIMEOUT_SECONDS = 4.0
POLL_INTERVAL_SECONDS = 0.1
STATUS_PRINT_INTERVAL_SECONDS = 1.0

# ArduPilot on this UART does not stream GPS until each message is requested.
MESSAGE_INTERVAL_SECONDS = 0.1
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
LAND_MODE_NUMBER = 9

# The airframe is a DJI F450. FRAME_CLASS 1 is Quad. FRAME_TYPE 1 is X.
# FRAME_TYPE 13 is DJI X, which is a different motor-2 and motor-3 order.
QUAD_FRAME_CLASS = 1
QUAD_X_FRAME_TYPE = 1

# Position-health bits. A guess or a GPS glitch is not a hold.
POSITION_HORIZONTAL_ABSOLUTE_BIT = 16
CONSTANT_POSITION_MODE_BIT = 128
GPS_GLITCHING_BIT = 32768
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


# ---------------------------------------------------------------------------
# Flight settings
# ---------------------------------------------------------------------------

FEET_PER_METER = 3.28084
TAKEOFF_ALTITUDE_FEET = 1.0
TAKEOFF_ALTITUDE_METERS = TAKEOFF_ALTITUDE_FEET / FEET_PER_METER
# The climb counts as done at 95 percent of the target, the same check as the tutorial loop.
TAKEOFF_ALTITUDE_REACHED_FRACTION = 0.95
TAKEOFF_TIMEOUT_SECONDS = 20
LAND_WAIT_TIMEOUT_SECONDS = 30
# A slower takeoff ramp leaves one side on the ground long enough to tip.
PREFERRED_TAKEOFF_SLEW_SECONDS = 1.0
TAKEOFF_SLEW_PARAMETER_NAME = "TKOFF_SLEW_TIME"
GPS_RECHECK_SECONDS = 5


# ---------------------------------------------------------------------------
# Link state. Importing this file does not connect or arm.
# ---------------------------------------------------------------------------

pixhawkConnection = None
latestMessages = {}
messageArrivedAtSeconds = {}
lastHeartbeatAtSeconds = None
copterWasArmed = False
originalTakeoffSlewSeconds = None


# ---------------------------------------------------------------------------
# Messages from the Pixhawk
# ---------------------------------------------------------------------------

def acceptMessage(message):
    """Keep the newest message of each kind. A broken byte is ignored."""
    global lastHeartbeatAtSeconds
    if message is None or message.get_type() == "BAD_DATA":
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
    """Read every message already waiting on the cable."""
    while True:
        message = pixhawkConnection.recv_match(blocking=False)
        if message is None:
            return
        acceptMessage(message)


def waitForNextMessage(waitSeconds):
    """Wait up to the given seconds for one message, then store it."""
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
    positionHealth = latestMessages.get("EKF_STATUS_REPORT")
    if positionHealth is None:
        return False
    flags = int(positionHealth.flags)
    hasAbsoluteHorizontalPosition = bool(flags & POSITION_HORIZONTAL_ABSOLUTE_BIT)
    isInConstantPositionMode = bool(flags & CONSTANT_POSITION_MODE_BIT)
    gpsGlitchIsActive = bool(flags & GPS_GLITCHING_BIT)
    return hasAbsoluteHorizontalPosition and not isInConstantPositionMode and not gpsGlitchIsActive


def gpsStatusSentence():
    """The GPS facts the arm check uses, so the wait says why it is still waiting."""
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
    positionHealth = latestMessages.get("EKF_STATUS_REPORT")
    flags = int(positionHealth.flags) if positionHealth is not None else 0
    if not (flags & POSITION_HORIZONTAL_ABSOLUTE_BIT):
        return f"{fixName}, {satelliteText}. Horizontal position is not trusted yet."
    if flags & CONSTANT_POSITION_MODE_BIT:
        return f"{fixName}, {satelliteText}. The position is a guess, not GPS."
    if flags & GPS_GLITCHING_BIT:
        return f"{fixName}, {satelliteText}. GPS is glitching."
    return f"{fixName}, {satelliteText}."


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
    return f"{altitudeMeters * FEET_PER_METER:.2f} ft"


# ---------------------------------------------------------------------------
# Modes and Pixhawk settings
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
    """True only when this DJI F450 is saved as Quad X. A different frame would tip."""
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
    print("Frame is a DJI F450, saved as Quad X.")
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
    """Put the original takeoff ramp back when this script shortened it."""
    global originalTakeoffSlewSeconds
    if originalTakeoffSlewSeconds is None or pixhawkConnection is None or not heartbeatIsFresh():
        return
    print(f"Restoring {TAKEOFF_SLEW_PARAMETER_NAME} to {originalTakeoffSlewSeconds:.2f}s.")
    restored = writeParameter(TAKEOFF_SLEW_PARAMETER_NAME, originalTakeoffSlewSeconds)
    if restored:
        print(f"{TAKEOFF_SLEW_PARAMETER_NAME} restored.")
        originalTakeoffSlewSeconds = None
    else:
        print(f"Could not confirm restoration of {TAKEOFF_SLEW_PARAMETER_NAME}.")


def waitUntilGpsCanHold():
    """Wait until the position check passes. Ctrl+C is the way out of this wait."""
    print("Waiting until the copter can hold position.")
    print("Need a 3D fix and at least 4 satellites.")
    lastPrintedAtSeconds = 0.0
    while True:
        drainMessages()
        if not heartbeatIsFresh():
            print("Pixhawk heartbeat was lost.")
            return False
        if gpsHoldIsReady():
            print(f"GPS position ready. {gpsStatusSentence()}")
            return True
        currentTimeSeconds = time.monotonic()
        if currentTimeSeconds - lastPrintedAtSeconds >= STATUS_PRINT_INTERVAL_SECONDS:
            print(f"Waiting for GPS. {gpsStatusSentence()}")
            lastPrintedAtSeconds = currentTimeSeconds
        time.sleep(POLL_INTERVAL_SECONDS)


def confirmGpsStillReady():
    """Listen again after a parameter read. That read can let the last GPS message go stale."""
    deadlineSeconds = time.monotonic() + GPS_RECHECK_SECONDS
    while time.monotonic() < deadlineSeconds:
        drainMessages()
        if not heartbeatIsFresh():
            print("Pixhawk heartbeat was lost.")
            return False
        if gpsHoldIsReady():
            return True
        time.sleep(POLL_INTERVAL_SECONDS)
    print(f"GPS position was not ready. {gpsStatusSentence()}")
    return False


# ---------------------------------------------------------------------------
# Arm, climb, and land
# ---------------------------------------------------------------------------

def armForTakeoff():
    """Send a normal arm and wait until the heartbeat shows the motors are armed."""
    global copterWasArmed
    print("Sending ARM command.")
    pixhawkConnection.mav.command_long_send(
        pixhawkConnection.target_system,
        pixhawkConnection.target_component,
        middleMan.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,  # arm
        0,  # a normal arm, not a force arm
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
            copterWasArmed = True
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


def climbToTakeoffHeight(targetAltitudeMeters=None):
    """Command the climb and wait until the copter is close to the target height.

    The 1 foot takeoff uses the saved target. A move script can pass a higher one.
    """
    if targetAltitudeMeters is None:
        targetAltitudeMeters = TAKEOFF_ALTITUDE_METERS
        print(f"Taking off to {TAKEOFF_ALTITUDE_FEET:.0f} foot.")
    else:
        targetAltitudeFeet = targetAltitudeMeters * FEET_PER_METER
        print(f"Taking off to {targetAltitudeFeet:.1f} ft.")
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
        targetAltitudeMeters,  # the last field is the height in meters
    )
    minimumTakeoffAltitudeMeters = targetAltitudeMeters * TAKEOFF_ALTITUDE_REACHED_FRACTION
    deadlineSeconds = time.monotonic() + TAKEOFF_TIMEOUT_SECONDS
    lastPrintedAtSeconds = 0.0
    while time.monotonic() < deadlineSeconds:
        drainMessages()
        if not heartbeatIsFresh():
            print("Heartbeat was lost during takeoff.")
            return False
        if copterIsArmed() is not True:
            print("The copter disarmed during takeoff.")
            return False
        if not copterIsInMode(GUIDED_MODE_NUMBER):
            print("GUIDED ended during takeoff.")
            return False
        if not gpsHoldIsReady():
            print("GPS position was lost during takeoff.")
            return False
        currentTimeSeconds = time.monotonic()
        altitudeMeters = heightMeters()
        if currentTimeSeconds - lastPrintedAtSeconds >= STATUS_PRINT_INTERVAL_SECONDS:
            print(f"Current altitude: {heightFeetText()}")
            lastPrintedAtSeconds = currentTimeSeconds
        if altitudeMeters is not None and altitudeMeters >= minimumTakeoffAltitudeMeters:
            print("Target altitude reached.")
            return True
        time.sleep(POLL_INTERVAL_SECONDS)
    print(f"Did not reach {TAKEOFF_ALTITUDE_FEET:.0f} foot.")
    return False


def landAndWaitForDisarm():
    """Ask for LAND and wait until the motors disarm. This does not force-disarm."""
    if pixhawkConnection is None:
        return False
    drainMessages()
    if not heartbeatIsFresh():
        print("Heartbeat is stale. LAND was not sent.")
    elif copterIsArmed() is False:
        print("Motors are already disarmed.")
        return True
    else:
        print("LAND")
        requestMode(LAND_MODE_NUMBER, "LAND")
    landingStartedAtSeconds = time.monotonic()
    lastPrintedAtSeconds = 0.0
    while time.monotonic() - landingStartedAtSeconds < LAND_WAIT_TIMEOUT_SECONDS:
        drainMessages()
        if copterIsArmed() is False:
            print("Motors disarmed.")
            return True
        currentTimeSeconds = time.monotonic()
        if currentTimeSeconds - lastPrintedAtSeconds >= STATUS_PRINT_INTERVAL_SECONDS:
            print(f"LAND | {heightFeetText()}")
            lastPrintedAtSeconds = currentTimeSeconds
        time.sleep(POLL_INTERVAL_SECONDS)
    print("Landing wait ended before disarm was confirmed.")
    return False


# ---------------------------------------------------------------------------
# The flight
# ---------------------------------------------------------------------------

def openCommandCable():
    """Open pins 8 and 10 and wait for the Pixhawk heartbeat."""
    global pixhawkConnection
    print(f"Opening {PIXHAWK_UART_DEVICE} at {PIXHAWK_UART_BAUD} baud.")
    pixhawkConnection = middleMan.mavlink_connection(
        PIXHAWK_UART_DEVICE,
        baud=PIXHAWK_UART_BAUD,
        source_system=255,
        autoreconnect=False,
    )
    heartbeat = pixhawkConnection.wait_heartbeat(timeout=CONNECTION_TIMEOUT_SECONDS)
    if heartbeat is None:
        raise RuntimeError("No Pixhawk heartbeat arrived on pin 10.")
    if pixhawkConnection.target_component == 0:
        pixhawkConnection.target_component = 1
    acceptMessage(heartbeat)
    print(
        f"Connected to Pixhawk system {pixhawkConnection.target_system}, "
        f"component {pixhawkConnection.target_component}."
    )


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


def closeCommandCable():
    """Release pins 8 and 10. Call this only after the motors are disarmed, or if they never armed."""
    global pixhawkConnection
    if pixhawkConnection is None:
        return
    try:
        pixhawkConnection.close()
    except Exception:
        pass
    pixhawkConnection = None
    print("Command cable closed.")


def pilotTypedYes():
    """True only when the typed word is YES. This is the last check before arming."""
    print()
    print(f"Type YES to take off to {TAKEOFF_ALTITUDE_FEET:.0f} foot and then land.")
    typedWord = input("> ").strip()
    if typedWord != "YES":
        print("Cancelled. The copter was not armed.")
        return False
    return True


def main():
    """Connect, arm, climb to 1 foot, then land."""
    print("TAKE OFF AND LAND")
    print(f"Target altitude: {TAKEOFF_ALTITUDE_FEET:.0f} ft")
    print("GUIDED takeoff, then LAND.")
    print()
    try:
        openCommandCable()
        requestFlightMessages()
        if not quadXFrameIsSaved():
            return
        if not waitUntilGpsCanHold():
            return
        if not pilotTypedYes():
            return
        # The prompt can sit for a while. Read the cable again before arming.
        drainMessages()
        if not confirmGpsStillReady():
            print("Not arming.")
            return
        shortenSlowTakeoffRamp()
        if not confirmGpsStillReady():
            print("Not arming.")
            return
        if not requestMode(GUIDED_MODE_NUMBER, "GUIDED"):
            print("Not arming.")
            return
        if not armForTakeoff():
            return
        climbToTakeoffHeight()
        landAndWaitForDisarm()
    except KeyboardInterrupt:
        print()
        print("Ctrl+C.")
        if copterWasArmed:
            try:
                landAndWaitForDisarm()
            except KeyboardInterrupt:
                print("Second Ctrl+C. Closing the cable. The copter is not force-disarmed.")
    except Exception as error:
        print()
        print(f"STOPPED: {error}")
        if copterWasArmed:
            try:
                landAndWaitForDisarm()
            except Exception as landingError:
                print(f"Landing error: {landingError}")
    finally:
        try:
            restoreTakeoffRamp()
        except Exception as restoreError:
            print(f"Could not restore {TAKEOFF_SLEW_PARAMETER_NAME}: {restoreError}")
        closeCommandCable()
        print("Finished.")


if __name__ == "__main__":
    main()
