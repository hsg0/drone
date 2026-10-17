#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    Keep the copter a chosen distance in front of you.
    The camera finds the brown box with the vertical blue stripe. The sonic
    sensor on the power distribution board measures that forward distance, and
    the saved camera scale is the backup.
    The time-of-flight sensor is under the drone and faces forward and
    backward. It is printed, and it does not set the follow distance.
    Farther than the chosen distance, the copter flies forward slowly.
    Closer, it flies backward slowly. About right, it stops.

WHY:
    The Pixhawk holds height and keeps the copter level. This script only
    decides forward, back, or stop. A marker is a clearer target than a person
    on this camera. The sonic sensor on the power board is the real forward
    range. The camera scale saved by
    forward_distance_calibration.py is the backup when the sonic reading
    drops out.
    A very slow liftoff can leave one side on the ground long enough for the
    frame to tip. Before arming, TKOFF_SLEW_TIME is made firmer only when the
    Pixhawk's current value is slower than 1 second. An already-firm ramp is
    left alone.

HOW:
    1. Connect on /dev/serial0. Leave both USB ports alone.
    2. Ask for heartbeat, GPS, altitude, the position estimate, and status text.
    3. Open the forward camera and read the range snapshot from bootDisplay.py.
    4. Load the saved camera scale. A missing file does not stop the flight.
       The sonic sensor can still measure the box.
    5. Continue only after you type YES, then arm on the transmitter.
    6. Set the firmer takeoff ramp, switch to GUIDED, and take off.
    7. Send a slow forward or back speed from the chosen distance.
    8. Stop at once if the marker, the trusted distance, or the position
       estimate disappears. If that lasts too long, command LAND.
    9. Ctrl+C stops the horizontal speed and commands LAND.
    10. Put the original TKOFF_SLEW_TIME back before exit.

IMPORTANT:
    Run this on the Pi, in a terminal. Raspberry Pi Connect is that terminal
    when the computer is on the hotspot. Type YES in that terminal.
    Props-off first. Walk the marker a few feet in front of the nose and
    check the printed distances before typing YES.

    The camera must point forward, along the nose, at the brown box with the
    blue stripe. Run forward_distance_calibration.py once so the camera backup
    has a saved scale. Follow still flies on the sonic sensor if that file
    is missing.

    This script commands takeoff. You arm the copter. It never sends an arm
    command, and it never force-disarms.

    /dev/serial0 is pins 8 and 10 at 57600. Pin 6 is ground.
    bootDisplay.py owns the Pixhawk USB port and the RedBoard. This script
    reads /tmp/redboard-forward-distance.json and does not open USB.

    The RedBoard may send one time-of-flight number, tofCm. That sensor sits
    under the drone and looks forward and backward, so its number is not the
    forward distance. The speed command stays on the sonic sensor or the camera.

    GUIDED needs a real horizontal position. If GPS or the position estimate
    becomes unhealthy, horizontal speed goes to zero, then LAND if it stays bad.
    The follow distance is 10 feet, about 305 cm, so the sonic sensor is still
    accepted out to 400 cm.
    The recent 2-foot and 4-foot hovers are too low to fly toward a person.
    This flight climbs to FOLLOW_ALTITUDE_FEET.
"""

import json
import math
import os
import select
import signal
import sys
import time

from pymavlink import mavutil

scriptDirectory = os.path.dirname(os.path.abspath(__file__))
if scriptDirectory not in sys.path:
    sys.path.insert(0, scriptDirectory)
import forward_distance_calibration


# ---------------------------------------------------------------------------
# Connection settings
# ---------------------------------------------------------------------------

# Pin 8 transmits. Pin 10 receives. Pin 6 is ground.
# The same 57600 command cable as the other pin 8 and pin 10 scripts.
PIXHAWK_UART_DEVICE = "/dev/serial0"
PIXHAWK_UART_BAUD = 57600

CONNECTION_TIMEOUT_SECONDS = 30
HEARTBEAT_STALE_AFTER_SECONDS = 3
MODE_CHANGE_TIMEOUT_SECONDS = 15
POLL_INTERVAL_SECONDS = 0.2

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
LAND_MODE_NUMBER = 9

# EKF_STATUS_REPORT bits.
# Absolute horizontal position is required so zero speed can hold a place.
# Constant-position mode and a GPS glitch are not that estimate.
EKF_POSITION_HORIZONTAL_ABSOLUTE_BIT = 16
EKF_CONSTANT_POSITION_MODE_BIT = 128
EKF_GPS_GLITCHING_BIT = 32768

NAVIGATION_LOSS_BEFORE_LAND_SECONDS = 2.0

# GPS_RAW_INT uses 255 when the satellite count is unknown.
UNKNOWN_SATELLITE_COUNT = 255
MINIMUM_GPS_FIX_TYPE_FOR_FLIGHT = 2
MINIMUM_SATELLITE_COUNT_FOR_FLIGHT = 2

# Velocity only, in the copter's own frame: x forward, y right, z down.
# Position and acceleration are ignored so the speed request is what moves it.
# Yaw is ignored so this script does not turn the copter.
VELOCITY_ONLY_TYPE_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

lastHeartbeatReceivedAtSeconds = None


# ---------------------------------------------------------------------------
# Flight settings
# ---------------------------------------------------------------------------

CONFIRMATION_WORD = "YES"
METERS_PER_FOOT = 0.3048

FOLLOW_DISTANCE_FEET = 10.0
FOLLOW_DISTANCE_METERS = FOLLOW_DISTANCE_FEET * METERS_PER_FOOT
FOLLOW_DISTANCE_TOLERANCE_METERS = 0.30

FOLLOW_ALTITUDE_FEET = 8.0
FOLLOW_ALTITUDE_METERS = FOLLOW_ALTITUDE_FEET * METERS_PER_FOOT
TAKEOFF_ALTITUDE_REACHED_FRACTION = 0.90
TAKEOFF_TIMEOUT_SECONDS = 25

MAXIMUM_FOLLOW_SPEED_METERS_PER_SECOND = 0.5
FOLLOW_SPEED_GAIN_PER_METER = 0.4
VELOCITY_COMMAND_PERIOD_SECONDS = 0.1
STATUS_PRINT_INTERVAL_SECONDS = 1.0

MANUAL_ARM_TIMEOUT_SECONDS = 30
MARKER_LOST_BEFORE_LAND_SECONDS = 3.0
DISTANCE_LOST_BEFORE_LAND_SECONDS = 3.0
INITIAL_TELEMETRY_WAIT_SECONDS = 4.0

# Lower TKOFF_SLEW_TIME ramps takeoff throttle faster.
# If the Pixhawk value is already below this, it is left alone.
TAKEOFF_SLEW_PARAMETER_NAME = "TKOFF_SLEW_TIME"
TAKEOFF_SLEW_TIME_SECONDS = 1.0
PARAMETER_READ_TIMEOUT_SECONDS = 3.0
PARAMETER_SET_TIMEOUT_SECONDS = 3.0
PARAMETER_MATCH_TOLERANCE = 0.01


# ---------------------------------------------------------------------------
# Camera and marker
# ---------------------------------------------------------------------------

# Printed ArUco 4x4 marker. Change the id if the paper uses another one.
MARKER_DICTIONARY_NAME = "DICT_4X4_50"
MARKER_ID = 0
MARKER_WIDTH_METERS = 0.15

# OV5647 full-width view, used to turn marker size into metres.
CAMERA_FRAME_WIDTH_PIXELS = 640
CAMERA_FRAME_HEIGHT_PIXELS = 480
CAMERA_HORIZONTAL_FIELD_OF_VIEW_DEGREES = 54.0

# A marker this close to the picture edge is leaving view. Stop rather than chase.
MARKER_EDGE_MARGIN_FRACTION = 0.08

CAMERA_SCALE_SAMPLE_COUNT = 5
CAMERA_SCALE_MINIMUM = 0.50
CAMERA_SCALE_MAXIMUM = 2.00
CAMERA_SCALE_SAMPLE_MINIMUM_METERS = 0.30


# ---------------------------------------------------------------------------
# Forward range sensors
# ---------------------------------------------------------------------------

# Written by bootDisplay.py. This script must not open the RedBoard itself.
FORWARD_RANGE_SNAPSHOT_PATH = "/tmp/redboard-forward-distance.json"
FORWARD_RANGE_STALE_AFTER_SECONDS = 1.0

# The follow distance is about 305 cm, so 300 cm would drop the sonic reading
# right at the distance we want to hold.
SONIC_MINIMUM_CENTIMETERS = 15.0
SONIC_MAXIMUM_CENTIMETERS = 400.0
SONIC_SNAPSHOT_FIELD_NAMES = (
    "sonicCentimeters",
    "sonicCm",
    "distanceCm",
)
TIME_OF_FLIGHT_SNAPSHOT_FIELD_NAMES = (
    "tofCentimeters",
    "tofCm",
)
SNAPSHOT_FUTURE_SKEW_ALLOWANCE_SECONDS = 1.0

# A time-of-flight reading outside this span is ignored. It is not required.
TIME_OF_FLIGHT_MINIMUM_CENTIMETERS = 5.0
TIME_OF_FLIGHT_MAXIMUM_CENTIMETERS = 400.0

# Close enough to call the time-of-flight reading a confirmation.
RANGE_AGREEMENT_METERS = 0.40

CENTIMETERS_PER_METER = 100.0


# ---------------------------------------------------------------------------
# Signal handling
# ---------------------------------------------------------------------------

def raiseKeyboardInterrupt(signalNumber, stackFrame):
    """A dropped remote terminal takes the same path as Ctrl+C."""
    raise KeyboardInterrupt


def noteHeartbeat(link, message):
    global lastHeartbeatReceivedAtSeconds
    if message.get_srcSystem() != link.target_system:
        return
    lastHeartbeatReceivedAtSeconds = time.monotonic()


def heartbeatIsFresh():
    if lastHeartbeatReceivedAtSeconds is None:
        return False
    heartbeatAgeSeconds = time.monotonic() - lastHeartbeatReceivedAtSeconds
    return heartbeatAgeSeconds <= HEARTBEAT_STALE_AFTER_SECONDS


# ---------------------------------------------------------------------------
# Pixhawk connection
# ---------------------------------------------------------------------------

def connectToPixhawk():
    if not os.path.exists(PIXHAWK_UART_DEVICE):
        print(f"{PIXHAWK_UART_DEVICE} is not present.")
        print("This is the Pi GPIO UART: pin 8 transmit, pin 10 receive, pin 6 ground.")
        return None

    print(
        f"Connecting to Pixhawk on {PIXHAWK_UART_DEVICE} "
        f"at {PIXHAWK_UART_BAUD} baud."
    )
    link = mavutil.mavlink_connection(
        PIXHAWK_UART_DEVICE,
        baud=PIXHAWK_UART_BAUD,
        autoreconnect=False,
    )
    heartbeat = link.wait_heartbeat(timeout=CONNECTION_TIMEOUT_SECONDS)
    if heartbeat is None:
        print("No Pixhawk heartbeat on the command cable.")
        link.close()
        return None

    # A heartbeat can report component 0. Commands need the autopilot component.
    if link.target_component == 0:
        link.target_component = 1

    global lastHeartbeatReceivedAtSeconds
    lastHeartbeatReceivedAtSeconds = time.monotonic()
    print(
        f"Connected. Heartbeat from system {link.target_system}, "
        f"component {link.target_component}."
    )
    requestFlightMessages(link)
    return link


def requestFlightMessages(link):
    intervalMicroseconds = int(MESSAGE_INTERVAL_SECONDS * MICROSECONDS_PER_SECOND)
    for messageName in FLIGHT_MESSAGE_NAMES:
        messageId = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{messageName}", None)
        if messageId is None:
            print(f"No MAVLink id for {messageName}.")
            continue
        link.mav.command_long_send(
            link.target_system,
            link.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            messageId,
            intervalMicroseconds,
            0,
            0,
            0,
            0,
            0,
        )
    print("Requested heartbeat, GPS, altitude, the position estimate, and status text.")


def statusTextFromMessage(message):
    statusText = getattr(message, "text", "")
    if isinstance(statusText, bytes):
        statusText = statusText.decode("ascii", "ignore")
    return str(statusText).strip("\x00").strip()


def rememberIncomingMessage(link, latestMessages, message):
    """Keep the latest copy, and do not drop a heartbeat while reading a parameter."""
    if message.get_srcSystem() not in (0, link.target_system):
        return
    messageType = message.get_type()
    latestMessages[messageType] = message
    if messageType == "HEARTBEAT":
        noteHeartbeat(link, message)
    elif messageType == "STATUSTEXT":
        statusText = statusTextFromMessage(message)
        if statusText:
            print(f"PIXHAWK: {statusText}")


def drainMessages(link, latestMessages):
    while True:
        message = link.recv_match(blocking=False)
        if message is None:
            return
        rememberIncomingMessage(link, latestMessages, message)


def copterIsArmed(latestMessages):
    heartbeat = latestMessages.get("HEARTBEAT")
    if heartbeat is None:
        return None
    return bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def gpsIsGoodEnough(latestMessages):
    gpsReading = latestMessages.get("GPS_RAW_INT")
    if gpsReading is None or gpsReading.fix_type is None:
        return False
    if gpsReading.fix_type < MINIMUM_GPS_FIX_TYPE_FOR_FLIGHT:
        return False
    satelliteCount = gpsReading.satellites_visible
    if satelliteCount is None or satelliteCount == UNKNOWN_SATELLITE_COUNT:
        return False
    return satelliteCount >= MINIMUM_SATELLITE_COUNT_FOR_FLIGHT


def copterIsInMode(latestMessages, expectedModeNumber):
    heartbeat = latestMessages.get("HEARTBEAT")
    if heartbeat is None:
        return False
    return heartbeat.custom_mode == expectedModeNumber


def positionEstimateIsHealthy(latestMessages):
    """GUIDED can hold still only with a real absolute horizontal position."""
    positionEstimate = latestMessages.get("EKF_STATUS_REPORT")
    if positionEstimate is None:
        return False
    flags = int(positionEstimate.flags)
    hasAbsoluteHorizontalPosition = bool(flags & EKF_POSITION_HORIZONTAL_ABSOLUTE_BIT)
    isInConstantPositionMode = bool(flags & EKF_CONSTANT_POSITION_MODE_BIT)
    gpsIsGlitching = bool(flags & EKF_GPS_GLITCHING_BIT)
    return (
        hasAbsoluteHorizontalPosition
        and not isInConstantPositionMode
        and not gpsIsGlitching
    )


def relativeAltitudeMeters(latestMessages):
    position = latestMessages.get("GLOBAL_POSITION_INT")
    if position is None:
        return None
    # relative_alt is millimetres above home.
    return position.relative_alt / 1000.0


def reasonFlightIsBlocked(latestMessages):
    if not heartbeatIsFresh():
        return "The Pixhawk heartbeat is missing or stale."
    if copterIsArmed(latestMessages) is None:
        return "The script cannot tell whether the copter is armed."
    if not gpsIsGoodEnough(latestMessages):
        gpsReading = latestMessages.get("GPS_RAW_INT")
        if gpsReading is None:
            return "No GPS reading has arrived on the command cable."
        return (
            f"GPS fix {gpsReading.fix_type} with {gpsReading.satellites_visible} satellites. "
            "This flight needs a 3D fix and at least "
            f"{MINIMUM_SATELLITE_COUNT_FOR_FLIGHT} satellites."
        )
    if not positionEstimateIsHealthy(latestMessages):
        return "The position estimate is not healthy. GUIDED was not requested."
    return None


def setCopterModeAndWait(link, latestMessages, modeNumber, modeName):
    print(f"Requesting {modeName}.")
    link.mav.set_mode_send(
        link.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        modeNumber,
    )
    startedAtSeconds = time.monotonic()
    while time.monotonic() - startedAtSeconds < MODE_CHANGE_TIMEOUT_SECONDS:
        drainMessages(link, latestMessages)
        if copterIsInMode(latestMessages, modeNumber):
            print(f"{modeName} confirmed.")
            return True
        if not heartbeatIsFresh():
            print("Heartbeat became stale while waiting for the mode change.")
            return False
        time.sleep(POLL_INTERVAL_SECONDS)
    print(f"{modeName} was not confirmed.")
    return False


def waitForPilotToArm(link, latestMessages):
    """Wait for the transmitter. This function never sends an arm command."""
    if copterIsArmed(latestMessages) is True:
        print("The copter is already armed.")
        return True

    print()
    print("ARM THE COPTER NOW")
    print("Use the transmitter to arm.")
    print(f"Waiting up to {MANUAL_ARM_TIMEOUT_SECONDS:.0f} seconds.")
    startedAtSeconds = time.monotonic()
    while time.monotonic() - startedAtSeconds < MANUAL_ARM_TIMEOUT_SECONDS:
        drainMessages(link, latestMessages)
        if not heartbeatIsFresh():
            print("Heartbeat lost while waiting for arming.")
            return False
        if copterIsArmed(latestMessages) is True:
            print("Armed confirmed.")
            return True
        time.sleep(POLL_INTERVAL_SECONDS)
    print("The copter was not armed before the timeout.")
    return False


def parameterNameText(rawParameterName):
    if isinstance(rawParameterName, bytes):
        rawParameterName = rawParameterName.decode("ascii", "ignore")
    return str(rawParameterName).strip("\x00").strip()


def readPixhawkParameter(link, latestMessages, parameterName):
    """Read one parameter. Heartbeats seen while waiting are still counted."""
    link.mav.param_request_read_send(
        link.target_system,
        link.target_component,
        parameterName.encode("ascii"),
        -1,
    )
    deadlineSeconds = time.monotonic() + PARAMETER_READ_TIMEOUT_SECONDS
    while time.monotonic() < deadlineSeconds:
        message = link.recv_match(blocking=True, timeout=0.4)
        if message is None:
            continue
        rememberIncomingMessage(link, latestMessages, message)
        if message.get_type() != "PARAM_VALUE":
            continue
        if parameterNameText(message.param_id) == parameterName:
            return float(message.param_value)
    return None


def setPixhawkParameterAndConfirm(link, latestMessages, parameterName, requestedValue):
    link.mav.param_set_send(
        link.target_system,
        link.target_component,
        parameterName.encode("ascii"),
        float(requestedValue),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )
    deadlineSeconds = time.monotonic() + PARAMETER_SET_TIMEOUT_SECONDS
    while time.monotonic() < deadlineSeconds:
        message = link.recv_match(blocking=True, timeout=0.4)
        if message is None:
            continue
        rememberIncomingMessage(link, latestMessages, message)
        if message.get_type() != "PARAM_VALUE":
            continue
        if parameterNameText(message.param_id) != parameterName:
            continue
        if abs(float(message.param_value) - requestedValue) < PARAMETER_MATCH_TOLERANCE:
            return True
    return False


def prepareFirmTakeoff(link, latestMessages):
    """
    Make the takeoff ramp firmer only when the current ramp is slower.
    Returns the original value when it must be restored, or None when
    nothing should be written back. The second result says whether takeoff
    may continue.
    """
    originalTakeoffSlewTimeSeconds = readPixhawkParameter(
        link,
        latestMessages,
        TAKEOFF_SLEW_PARAMETER_NAME,
    )
    if originalTakeoffSlewTimeSeconds is None:
        print(
            f"Could not read {TAKEOFF_SLEW_PARAMETER_NAME}. "
            "Takeoff was not started."
        )
        return None, False

    temporaryTakeoffSlewTimeSeconds = min(
        originalTakeoffSlewTimeSeconds,
        TAKEOFF_SLEW_TIME_SECONDS,
    )
    print(
        f"Original {TAKEOFF_SLEW_PARAMETER_NAME}: "
        f"{originalTakeoffSlewTimeSeconds:.2f} seconds."
    )
    print(
        f"Takeoff {TAKEOFF_SLEW_PARAMETER_NAME}: "
        f"{temporaryTakeoffSlewTimeSeconds:.2f} seconds."
    )
    if abs(temporaryTakeoffSlewTimeSeconds - originalTakeoffSlewTimeSeconds) < PARAMETER_MATCH_TOLERANCE:
        print("The existing takeoff ramp is already as firm or firmer. Leaving it unchanged.")
        return None, True

    parameterChanged = setPixhawkParameterAndConfirm(
        link,
        latestMessages,
        TAKEOFF_SLEW_PARAMETER_NAME,
        temporaryTakeoffSlewTimeSeconds,
    )
    if not parameterChanged:
        print(f"Could not confirm {TAKEOFF_SLEW_PARAMETER_NAME}. Takeoff was not started.")
        print("Putting the original takeoff ramp back.")
        setPixhawkParameterAndConfirm(
            link,
            latestMessages,
            TAKEOFF_SLEW_PARAMETER_NAME,
            originalTakeoffSlewTimeSeconds,
        )
        return originalTakeoffSlewTimeSeconds, False

    return originalTakeoffSlewTimeSeconds, True


def restoreTakeoffSetting(link, latestMessages, originalTakeoffSlewTimeSeconds):
    print(
        f"Restoring {TAKEOFF_SLEW_PARAMETER_NAME} "
        f"to {originalTakeoffSlewTimeSeconds:.2f} seconds."
    )
    restored = setPixhawkParameterAndConfirm(
        link,
        latestMessages,
        TAKEOFF_SLEW_PARAMETER_NAME,
        originalTakeoffSlewTimeSeconds,
    )
    if restored:
        print(f"{TAKEOFF_SLEW_PARAMETER_NAME} restored.")
    else:
        print(f"Warning: {TAKEOFF_SLEW_PARAMETER_NAME} restore could not be confirmed.")


def commandTakeoffAndWait(link, latestMessages):
    minimumAcceptedAltitudeMeters = (
        FOLLOW_ALTITUDE_METERS * TAKEOFF_ALTITUDE_REACHED_FRACTION
    )
    print()
    print("TAKEOFF")
    print(f"Target height: {FOLLOW_ALTITUDE_FEET:.1f} feet.")
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
        FOLLOW_ALTITUDE_METERS,
    )
    startedAtSeconds = time.monotonic()
    lastPrintedAtSeconds = 0
    while time.monotonic() - startedAtSeconds < TAKEOFF_TIMEOUT_SECONDS:
        drainMessages(link, latestMessages)
        if not heartbeatIsFresh():
            print("Heartbeat lost during takeoff.")
            return False
        if copterIsArmed(latestMessages) is not True:
            print("The copter disarmed during takeoff.")
            return False
        if not copterIsInMode(latestMessages, GUIDED_MODE_NUMBER):
            print("GUIDED mode changed during takeoff.")
            return False
        if not gpsIsGoodEnough(latestMessages):
            print("GPS became unhealthy during takeoff.")
            return False
        if not positionEstimateIsHealthy(latestMessages):
            print("The position estimate became unhealthy during takeoff.")
            return False
        altitudeMeters = relativeAltitudeMeters(latestMessages)
        nowSeconds = time.monotonic()
        if altitudeMeters is not None and nowSeconds - lastPrintedAtSeconds >= STATUS_PRINT_INTERVAL_SECONDS:
            print(
                f"Takeoff | {altitudeMeters:.2f} m | "
                f"{altitudeMeters / METERS_PER_FOOT:.1f} ft"
            )
            lastPrintedAtSeconds = nowSeconds
        if altitudeMeters is not None and altitudeMeters >= minimumAcceptedAltitudeMeters:
            print("Takeoff height reached.")
            return True
        time.sleep(POLL_INTERVAL_SECONDS)
    print("The copter did not reach the takeoff height in time.")
    return False


def sendForwardSpeed(link, forwardMetersPerSecond):
    """Ask GUIDED for a body-frame speed. Positive is forward. Zero hovers."""
    link.mav.set_position_target_local_ned_send(
        0,
        link.target_system,
        link.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        VELOCITY_ONLY_TYPE_MASK,
        0,
        0,
        0,
        forwardMetersPerSecond,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def commandLand(link, latestMessages):
    if not heartbeatIsFresh():
        print("Heartbeat is stale. LAND was not sent.")
        return False
    print()
    print("LAND")
    try:
        return setCopterModeAndWait(link, latestMessages, LAND_MODE_NUMBER, "LAND")
    except KeyboardInterrupt:
        print("Ctrl+C during LAND. Closing the link. The copter is not force-disarmed.")
        return False


# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------

def focalLengthPixels():
    halfViewRadians = math.radians(CAMERA_HORIZONTAL_FIELD_OF_VIEW_DEGREES / 2.0)
    return (CAMERA_FRAME_WIDTH_PIXELS / 2.0) / math.tan(halfViewRadians)


def openForwardCamera():
    print("Opening the forward camera.")
    camera = Picamera2()
    configuration = camera.create_preview_configuration(
        main={
            "size": (CAMERA_FRAME_WIDTH_PIXELS, CAMERA_FRAME_HEIGHT_PIXELS),
            "format": "RGB888",
        }
    )
    camera.configure(configuration)
    camera.start()
    # Picamera2 RGB888 arrives in blue-green-red order, which OpenCV uses.
    dictionary = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, MARKER_DICTIONARY_NAME)
    )
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    print(f"Camera open. Looking for marker {MARKER_ID}.")
    return camera, detector


def markerCornersInFrame(detector, frame):
    grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, markerIds, _rejected = detector.detectMarkers(grayFrame)
    if markerIds is None:
        return None
    for index, markerId in enumerate(markerIds.flatten()):
        if int(markerId) == MARKER_ID:
            return corners[index][0]
    return None


def markerWidthPixels(markerCorners):
    topEdgePixels = math.hypot(
        float(markerCorners[0][0]) - float(markerCorners[1][0]),
        float(markerCorners[0][1]) - float(markerCorners[1][1]),
    )
    bottomEdgePixels = math.hypot(
        float(markerCorners[3][0]) - float(markerCorners[2][0]),
        float(markerCorners[3][1]) - float(markerCorners[2][1]),
    )
    return (topEdgePixels + bottomEdgePixels) / 2.0


def cameraDistanceMeters(markerCorners):
    apparentWidthPixels = markerWidthPixels(markerCorners)
    if apparentWidthPixels < 1.0:
        return None
    return (MARKER_WIDTH_METERS * focalLengthPixels()) / apparentWidthPixels


def markerIsNearFrameEdge(markerCorners):
    marginXPixels = CAMERA_FRAME_WIDTH_PIXELS * MARKER_EDGE_MARGIN_FRACTION
    marginYPixels = CAMERA_FRAME_HEIGHT_PIXELS * MARKER_EDGE_MARGIN_FRACTION
    for point in markerCorners:
        xPixels = float(point[0])
        yPixels = float(point[1])
        if xPixels < marginXPixels or yPixels < marginYPixels:
            return True
        if xPixels > CAMERA_FRAME_WIDTH_PIXELS - marginXPixels:
            return True
        if yPixels > CAMERA_FRAME_HEIGHT_PIXELS - marginYPixels:
            return True
    return False


# ---------------------------------------------------------------------------
# Forward distance
# ---------------------------------------------------------------------------

def readForwardRangeSnapshot():
    try:
        with open(FORWARD_RANGE_SNAPSHOT_PATH, "r", encoding="utf-8") as snapshotFile:
            snapshot = json.load(snapshotFile)
    except (OSError, json.JSONDecodeError):
        return None
    updatedAtSeconds = snapshot.get("updatedAtSeconds")
    if not isinstance(updatedAtSeconds, (int, float)):
        return None
    ageSeconds = time.time() - float(updatedAtSeconds)
    if ageSeconds < -SNAPSHOT_FUTURE_SKEW_ALLOWANCE_SECONDS:
        return None
    if ageSeconds > FORWARD_RANGE_STALE_AFTER_SECONDS:
        return None
    return snapshot


def trustedRangeMeters(snapshot, fieldNames, minimumCentimeters, maximumCentimeters):
    if snapshot is None:
        return None
    centimeters = None
    for fieldName in fieldNames:
        candidateValue = snapshot.get(fieldName)
        if isinstance(candidateValue, (int, float)):
            centimeters = float(candidateValue)
            break
    if centimeters is None:
        return None
    if centimeters < minimumCentimeters or centimeters > maximumCentimeters:
        return None
    return centimeters / CENTIMETERS_PER_METER


def sonicDistanceMeters(snapshot):
    return trustedRangeMeters(
        snapshot,
        SONIC_SNAPSHOT_FIELD_NAMES,
        SONIC_MINIMUM_CENTIMETERS,
        SONIC_MAXIMUM_CENTIMETERS,
    )


def timeOfFlightDistanceMeters(snapshot):
    return trustedRangeMeters(
        snapshot,
        TIME_OF_FLIGHT_SNAPSHOT_FIELD_NAMES,
        TIME_OF_FLIGHT_MINIMUM_CENTIMETERS,
        TIME_OF_FLIGHT_MAXIMUM_CENTIMETERS,
    )


def medianOf(values):
    ordered = sorted(values)
    middleIndex = len(ordered) // 2
    if len(ordered) % 2 == 1:
        return ordered[middleIndex]
    return (ordered[middleIndex - 1] + ordered[middleIndex]) / 2.0


def considerCameraScale(cameraScaleSamples, rawCameraMeters, sonicMeters):
    """Learn how many sonic metres one raw camera metre represents."""
    if rawCameraMeters is None or sonicMeters is None:
        return None
    if (
        rawCameraMeters < CAMERA_SCALE_SAMPLE_MINIMUM_METERS
        or sonicMeters < CAMERA_SCALE_SAMPLE_MINIMUM_METERS
    ):
        return None
    scaleSample = sonicMeters / rawCameraMeters
    if scaleSample < CAMERA_SCALE_MINIMUM or scaleSample > CAMERA_SCALE_MAXIMUM:
        return None
    cameraScaleSamples.append(scaleSample)
    if len(cameraScaleSamples) < CAMERA_SCALE_SAMPLE_COUNT:
        return None
    return medianOf(cameraScaleSamples[-CAMERA_SCALE_SAMPLE_COUNT:])


def chooseForwardDistanceMeters(rawCameraMeters, calibration, snapshot):
    """
    The brown box must already be in view.
    The sonic sensor on the power board sets the forward distance.
    The saved camera scale is the backup.
    The time-of-flight sensor under the drone looks forward and backward,
    so it never sets this distance.
    """
    sonicMeters = forward_distance_calibration.sonicDistanceMeters(snapshot)
    scaledCameraMeters = forward_distance_calibration.scaledCameraDistanceMeters(
        rawCameraMeters,
        calibration,
    )

    if sonicMeters is not None:
        return sonicMeters, "sonic on the power board, facing forward"
    if scaledCameraMeters is not None:
        return scaledCameraMeters, "saved camera scale"
    return None, "no trusted distance"


def forwardSpeedMetersPerSecond(distanceMeters):
    distanceErrorMeters = distanceMeters - FOLLOW_DISTANCE_METERS
    if abs(distanceErrorMeters) <= FOLLOW_DISTANCE_TOLERANCE_METERS:
        return 0.0
    requestedSpeed = distanceErrorMeters * FOLLOW_SPEED_GAIN_PER_METER
    if requestedSpeed > MAXIMUM_FOLLOW_SPEED_METERS_PER_SECOND:
        return MAXIMUM_FOLLOW_SPEED_METERS_PER_SECOND
    if requestedSpeed < -MAXIMUM_FOLLOW_SPEED_METERS_PER_SECOND:
        return -MAXIMUM_FOLLOW_SPEED_METERS_PER_SECOND
    return requestedSpeed


def formatMeters(distanceMeters):
    if distanceMeters is None:
        return "missing"
    return f"{distanceMeters:.2f} m"


def calibrationLine(rawCameraMeters, cameraScale, snapshot, distanceMeters, distanceSource):
    sonicMeters = sonicDistanceMeters(snapshot)
    timeOfFlightMeters = timeOfFlightDistanceMeters(snapshot)
    scaledCameraMeters = None
    if rawCameraMeters is not None and cameraScale is not None:
        scaledCameraMeters = rawCameraMeters * cameraScale
    scaleText = "not ready" if cameraScale is None else f"{cameraScale:.2f}"
    return (
        f"Raw camera {formatMeters(rawCameraMeters)} | "
        f"Scaled camera {formatMeters(scaledCameraMeters)} | "
        f"Sonic, power board, forward {formatMeters(sonicMeters)} | "
        f"Time of flight, bottom, forward and backward {formatMeters(timeOfFlightMeters)} | "
        f"Using {formatMeters(distanceMeters)} from {distanceSource} | "
        f"scale {scaleText}"
    )


def typedLineOrNone():
    readyToRead, _unused, _alsoUnused = select.select([sys.stdin], [], [], 0)
    if not readyToRead:
        return None
    return sys.stdin.readline().strip()


def lookAtMarker(camera, detector):
    frame = camera.capture_array()
    markerCorners = markerCornersInFrame(detector, frame)
    if markerCorners is None:
        return None, None, False
    return (
        cameraDistanceMeters(markerCorners),
        markerCorners,
        markerIsNearFrameEdge(markerCorners),
    )


# ---------------------------------------------------------------------------
# Calibration, then follow
# ---------------------------------------------------------------------------

def acceptYesForFlight(latestMessages, cameraScale):
    """YES starts flight only after calibration and the position checks pass."""
    if cameraScale is None:
        print(
            f"Camera calibration is not complete. "
            f"{CAMERA_SCALE_SAMPLE_COUNT} good samples are required. Flight stays blocked."
        )
        return False
    blockedReason = reasonFlightIsBlocked(latestMessages)
    if blockedReason is not None:
        print(blockedReason)
        print(f"Flight stays blocked. Type {CONFIRMATION_WORD} again when this is resolved.")
        return False
    return True


def calibrateUntilYes(link, latestMessages, camera, detector):
    """
    Scale the camera to the sonic sensor.
    The copter does not move, and YES is refused until calibration is complete.
    """
    print()
    print("CALIBRATION")
    print("Stand a few feet in front of the nose with the marker facing the camera.")
    print("The sonic sensor and the camera must see the marker at the same time.")
    print(
        f"The script needs {CAMERA_SCALE_SAMPLE_COUNT} good samples "
        f"before flight. Then type {CONFIRMATION_WORD}."
    )
    print("The copter does not move during calibration.")

    cameraScale = None
    cameraScaleSamples = []
    lastPrintedAtSeconds = 0

    while True:
        drainMessages(link, latestMessages)
        if not heartbeatIsFresh():
            raise RuntimeError("Pixhawk heartbeat was lost during calibration.")

        nowSeconds = time.monotonic()
        typedLine = typedLineOrNone()
        if typedLine not in (None, ""):
            if typedLine != CONFIRMATION_WORD:
                print(f"Type {CONFIRMATION_WORD} only when you are ready.")
            elif acceptYesForFlight(latestMessages, cameraScale):
                return cameraScale

        if nowSeconds - lastPrintedAtSeconds < STATUS_PRINT_INTERVAL_SECONDS:
            time.sleep(POLL_INTERVAL_SECONDS)
            continue

        rawCameraMeters, markerCorners, markerNearEdge = lookAtMarker(camera, detector)
        snapshot = readForwardRangeSnapshot()
        sonicMeters = sonicDistanceMeters(snapshot)

        if markerCorners is None:
            print("Marker missing. No calibration sample added.")
        elif markerNearEdge:
            print("Marker is near the edge of the picture. Move it toward the centre.")
        else:
            learnedScale = considerCameraScale(
                cameraScaleSamples,
                rawCameraMeters,
                sonicMeters,
            )
            if learnedScale is not None:
                cameraScale = learnedScale
            distanceMeters, distanceSource = chooseForwardDistanceMeters(
                rawCameraMeters,
                cameraScale,
                snapshot,
            )
            print(calibrationLine(
                rawCameraMeters,
                cameraScale,
                snapshot,
                distanceMeters,
                distanceSource,
            ))
            sampleCount = min(len(cameraScaleSamples), CAMERA_SCALE_SAMPLE_COUNT)
            print(f"Calibration samples: {sampleCount}/{CAMERA_SCALE_SAMPLE_COUNT}")
            if cameraScale is not None:
                print(f"Camera scale ready: {cameraScale:.3f}. Type {CONFIRMATION_WORD} when ready to fly.")

        lastPrintedAtSeconds = nowSeconds
        time.sleep(POLL_INTERVAL_SECONDS)


def confirmUntilYes(link, latestMessages, camera, calibration):
    """Show the live distances, then wait for YES. This does not learn a new scale."""
    print()
    print("CHECK THE DISTANCES")
    print("Hold the brown box in front of the nose, with the blue stripe running up and down.")
    print("The sonic sensor sets the distance. The saved camera scale is only the backup.")
    print(f"Type {CONFIRMATION_WORD} to arm, take off, and follow. The copter does not move yet.")
    lastPrintedAtSeconds = 0
    while True:
        drainMessages(link, latestMessages)
        if not heartbeatIsFresh():
            raise RuntimeError("Pixhawk heartbeat was lost before takeoff.")
        nowSeconds = time.monotonic()
        typedLine = typedLineOrNone()
        if typedLine not in (None, ""):
            if typedLine != CONFIRMATION_WORD:
                print(f"Type {CONFIRMATION_WORD} when you are ready.")
            else:
                blockedReason = reasonFlightIsBlocked(latestMessages)
                if blockedReason is not None:
                    print(blockedReason)
                    print(f"Flight stays blocked. Type {CONFIRMATION_WORD} again when this is resolved.")
                else:
                    return
        if nowSeconds - lastPrintedAtSeconds < STATUS_PRINT_INTERVAL_SECONDS:
            time.sleep(POLL_INTERVAL_SECONDS)
            continue
        rawCameraMeters, markerCorners, markerNearEdge, sawBlueWithoutBrownBox = (
            forward_distance_calibration.lookAtMarker(camera)
        )
        snapshot = forward_distance_calibration.readForwardRangeSnapshot()
        sonicMeters = forward_distance_calibration.sonicDistanceMeters(snapshot)
        timeOfFlightMeters = forward_distance_calibration.timeOfFlightDistanceMeters(snapshot)
        if markerCorners is None and sawBlueWithoutBrownBox:
            print("Saw a blue stripe, but not on the brown box.")
        elif markerCorners is None:
            print("Brown box is not in the picture.")
        elif markerNearEdge:
            print("Brown box is near the edge of the picture. Move it toward the centre.")
        else:
            distanceMeters, distanceSource = chooseForwardDistanceMeters(
                rawCameraMeters,
                calibration,
                snapshot,
            )
            print(
                f"Sonic {forward_distance_calibration.formatMeters(sonicMeters)} | "
                f"Camera {forward_distance_calibration.formatMeters(forward_distance_calibration.scaledCameraDistanceMeters(rawCameraMeters, calibration))} | "
                f"Time of flight {forward_distance_calibration.formatMeters(timeOfFlightMeters)} | "
                f"Using {forward_distance_calibration.formatMeters(distanceMeters)} from {distanceSource}"
            )
        lastPrintedAtSeconds = nowSeconds
        time.sleep(POLL_INTERVAL_SECONDS)


def stopAndReport(link, reason):
    sendForwardSpeed(link, 0.0)
    return reason


def followUntilItIsTimeToLand(link, latestMessages, camera, calibration):
    print()
    print("FOLLOW")
    print(
        f"Holding about {FOLLOW_DISTANCE_FEET:.0f} feet. "
        f"Speed cap {MAXIMUM_FOLLOW_SPEED_METERS_PER_SECOND:.1f} m/s. "
        "Ctrl+C commands LAND."
    )
    markerLostAtSeconds = None
    distanceLostAtSeconds = None
    navigationLostAtSeconds = None
    lastPrintedAtSeconds = 0
    nextCommandAtSeconds = 0

    while True:
        drainMessages(link, latestMessages)
        nowSeconds = time.monotonic()
        if not heartbeatIsFresh():
            return "HEARTBEAT LOST: no further movement command was sent."
        if copterIsArmed(latestMessages) is not True:
            return "DISARMED: the copter is no longer armed."
        if not copterIsInMode(latestMessages, GUIDED_MODE_NUMBER):
            return "MODE CHANGED: GUIDED is no longer active. Movement commands stopped."

        navigationIsHealthy = (
            gpsIsGoodEnough(latestMessages) and positionEstimateIsHealthy(latestMessages)
        )
        rawCameraMeters, markerCorners, markerNearEdge, _sawBlueWithoutBrownBox = (
            forward_distance_calibration.lookAtMarker(camera)
        )
        snapshot = forward_distance_calibration.readForwardRangeSnapshot()
        forwardMetersPerSecond = 0.0
        statusText = "holding"

        if not navigationIsHealthy:
            if navigationLostAtSeconds is None:
                navigationLostAtSeconds = nowSeconds
                print("GPS or the position estimate became unhealthy. Stopping horizontal movement.")
            statusText = "navigation unhealthy, holding"
            if nowSeconds - navigationLostAtSeconds >= NAVIGATION_LOSS_BEFORE_LAND_SECONDS:
                return stopAndReport(link, "NAVIGATION LOST: GPS or the position estimate stayed unhealthy.")
        elif markerCorners is None or rawCameraMeters is None:
            navigationLostAtSeconds = None
            if markerLostAtSeconds is None:
                markerLostAtSeconds = nowSeconds
                print("Brown box lost. Stopping horizontal movement.")
            statusText = "brown box lost, holding"
            if nowSeconds - markerLostAtSeconds >= MARKER_LOST_BEFORE_LAND_SECONDS:
                return stopAndReport(link, "BOX LOST: the brown box stayed out of view.")
        elif markerNearEdge:
            navigationLostAtSeconds = None
            markerLostAtSeconds = None
            distanceLostAtSeconds = None
            statusText = "brown box near the edge, holding"
        else:
            navigationLostAtSeconds = None
            markerLostAtSeconds = None
            distanceMeters, distanceSource = chooseForwardDistanceMeters(
                rawCameraMeters,
                calibration,
                snapshot,
            )
            if distanceMeters is None:
                if distanceLostAtSeconds is None:
                    distanceLostAtSeconds = nowSeconds
                    print("No trusted forward distance. Stopping horizontal movement.")
                statusText = "distance missing, holding"
                if nowSeconds - distanceLostAtSeconds >= DISTANCE_LOST_BEFORE_LAND_SECONDS:
                    return stopAndReport(link, "DISTANCE LOST: no trusted forward distance remained.")
            else:
                distanceLostAtSeconds = None
                forwardMetersPerSecond = forwardSpeedMetersPerSecond(distanceMeters)
                if forwardMetersPerSecond > 0:
                    directionText = "forward"
                elif forwardMetersPerSecond < 0:
                    directionText = "backward"
                else:
                    directionText = "hold"
                statusText = (
                    f"{directionText} {abs(forwardMetersPerSecond):.2f} m/s | "
                    f"distance {distanceMeters:.2f} m | {distanceSource}"
                )

        if nowSeconds >= nextCommandAtSeconds:
            sendForwardSpeed(link, forwardMetersPerSecond)
            nextCommandAtSeconds = nowSeconds + VELOCITY_COMMAND_PERIOD_SECONDS

        if nowSeconds - lastPrintedAtSeconds >= STATUS_PRINT_INTERVAL_SECONDS:
            altitudeMeters = relativeAltitudeMeters(latestMessages)
            if altitudeMeters is None:
                altitudeText = "height missing"
            else:
                altitudeText = f"height {altitudeMeters / METERS_PER_FOOT:.1f} ft"
            print(f"{statusText} | {altitudeText}")
            lastPrintedAtSeconds = nowSeconds

        time.sleep(POLL_INTERVAL_SECONDS)


def flightHasStarted(guidedModeConfirmed, takeoffWasCommanded, latestMessages):
    return (
        guidedModeConfirmed
        or takeoffWasCommanded
        or copterIsArmed(latestMessages) is True
    )


def stopThenLand(link, latestMessages):
    if heartbeatIsFresh():
        sendForwardSpeed(link, 0.0)
        commandLand(link, latestMessages)


def main():
    signal.signal(signal.SIGHUP, raiseKeyboardInterrupt)

    link = None
    camera = None
    guidedModeConfirmed = False
    takeoffWasCommanded = False
    originalTakeoffSlewTimeSeconds = None
    latestMessages = {}

    try:
        print()
        print("FOLLOW AT A DISTANCE")
        print()
        link = connectToPixhawk()
        if link is None:
            return

        camera = forward_distance_calibration.openForwardCamera()
        telemetryWaitDeadlineSeconds = time.monotonic() + INITIAL_TELEMETRY_WAIT_SECONDS
        while time.monotonic() < telemetryWaitDeadlineSeconds:
            drainMessages(link, latestMessages)
            time.sleep(0.05)

        calibration = forward_distance_calibration.loadForwardDistanceCalibration()
        if calibration is None:
            print(forward_distance_calibration.savedCalibrationRefusalReason())
            print("The saved camera scale is not available. Follow will use the sonic sensor when the brown box is in view.")
        else:
            print(
                f"Using saved camera scale {float(calibration['cameraScale']):.3f}. "
                "The sonic sensor still sets the distance."
            )
        confirmUntilYes(link, latestMessages, camera, calibration)

        originalTakeoffSlewTimeSeconds, takeoffRampIsReady = prepareFirmTakeoff(
            link,
            latestMessages,
        )
        if not takeoffRampIsReady:
            return

        if not waitForPilotToArm(link, latestMessages):
            return

        drainMessages(link, latestMessages)
        blockedReason = reasonFlightIsBlocked(latestMessages)
        if blockedReason is not None:
            print(blockedReason)
            print("Takeoff was not commanded.")
            return

        guidedModeConfirmed = setCopterModeAndWait(
            link,
            latestMessages,
            GUIDED_MODE_NUMBER,
            "GUIDED",
        )
        if not guidedModeConfirmed:
            print("GUIDED was not confirmed. Takeoff was not commanded.")
            return
        if copterIsArmed(latestMessages) is not True:
            print("The copter is no longer armed. Takeoff was not commanded.")
            return

        takeoffWasCommanded = True
        if not commandTakeoffAndWait(link, latestMessages):
            commandLand(link, latestMessages)
            return

        reasonToLand = followUntilItIsTimeToLand(
            link,
            latestMessages,
            camera,
            calibration,
        )
        print(reasonToLand)
        # Another mode means the pilot or the Pixhawk already chose what to do.
        # A missing heartbeat means another command cannot be trusted.
        if reasonToLand.startswith("MODE CHANGED") or reasonToLand.startswith("HEARTBEAT LOST"):
            return
        if copterIsArmed(latestMessages) is True:
            commandLand(link, latestMessages)

    except KeyboardInterrupt:
        print()
        print("Ctrl+C.")
        if link is not None and heartbeatIsFresh() and flightHasStarted(
            guidedModeConfirmed,
            takeoffWasCommanded,
            latestMessages,
        ):
            stopThenLand(link, latestMessages)

    except Exception as error:
        print()
        print(f"Stopped because of an error: {error}")
        if link is not None and heartbeatIsFresh() and flightHasStarted(
            guidedModeConfirmed,
            takeoffWasCommanded,
            latestMessages,
        ):
            stopThenLand(link, latestMessages)

    finally:
        if (
            link is not None
            and originalTakeoffSlewTimeSeconds is not None
            and heartbeatIsFresh()
        ):
            try:
                restoreTakeoffSetting(link, latestMessages, originalTakeoffSlewTimeSeconds)
            except Exception as restoreError:
                print(f"Could not restore the takeoff setting: {restoreError}")
        if camera is not None:
            try:
                camera.stop()
            except Exception:
                pass
            print("Camera closed.")
        if link is not None:
            try:
                link.close()
            except Exception:
                pass
            print("Pixhawk link closed.")
        print("Finished.")


if __name__ == "__main__":
    main()
