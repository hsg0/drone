#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    Take off to about 2 feet, hover for 60 seconds, and land.
    GPS is optional.

    Two or more satellites means GPS is detected. That is not enough to
    hold position. LOITER is used only with a 3D fix, at least 6 satellites,
    and a healthy horizontal position estimate.

    When GPS is strong enough:
        GUIDED, takeoff, LOITER, 60 second hover, LAND.
    When GPS is missing or weak:
        GUIDED_NOGPS, sensor takeoff, sensor hover, LAND.

WHY:
    One hover test should work outdoors and indoors.
    Good GPS lets LOITER hold position. Weak GPS must not be treated as a
    position. The IMU keeps the copter level. Height comes from a downward
    distance sensor, a rangefinder, or the barometer.
    The sensor takeoff stays wings-level and climbs firmly until about one
    foot, so one side is not left on the ground. The height source chosen
    before takeoff stays selected. A mid-climb switch keeps the same height
    number so the copter does not correct as if it jumped.

HOW:
    1. Connect on pins 8 and 10. Leave USB with the screen.
    2. Request the telemetry this test needs, and remember when each message arrived.
    3. Listen for GPS. Report detection at 2 or more satellites.
    4. Use LOITER only when the position-hold checks pass.
    5. Average the ground barometer.
    6. Pick height: downward DISTANCE_SENSOR, then RANGEFINDER, then barometer.
    7. Type YES, choose GUIDED or GUIDED_NOGPS, then arm.
    8. Climb to about 2 feet and stay there for 2 seconds.
    9. Start the 60 second hover, then LAND.
    10. Put TKOFF_SLEW_TIME back if this script changed it.

IMPORTANT:
    Props-off first. Sit the craft level.
    Type YES before it arms. This script sends the arm command after YES.
    Two or three satellites means GPS is detected only. It does not mean LOITER.
    GPS is not used as the height source on either path.
    A forward DISTANCE_SENSOR is ignored. Only a downward orientation counts.
    GUIDED_NOGPS can hold attitude and height. It can still drift sideways.
    This script does not command individual motors.
    Ctrl+C requests LAND. A second Ctrl+C closes the link. There is no force-disarm.
"""

import math
import time

from pymavlink import mavutil


# Pin 8 transmits. Pin 10 receives. Pin 6 is ground.
PIXHAWK_SERIAL_PORT = "/dev/serial0"
PIXHAWK_SERIAL_BAUD = 57600

CONNECTION_TIMEOUT_SECONDS = 15
INITIAL_TELEMETRY_COLLECTION_SECONDS = 5
GPS_LISTEN_SECONDS = 10
HEARTBEAT_STALE_AFTER_SECONDS = 3.0
PARAMETER_TIMEOUT_SECONDS = 4.0
MESSAGE_INTERVAL_MICROSECONDS = 100000

GUIDED_MODE_NUMBER = 4
LOITER_MODE_NUMBER = 5
LAND_MODE_NUMBER = 9
GUIDED_NOGPS_MODE_NUMBER = 20

METERS_PER_FOOT = 0.3048
FEET_PER_METER = 3.28084
TARGET_ALTITUDE_FEET = 2.0
TARGET_ALTITUDE_METERS = TARGET_ALTITUDE_FEET * METERS_PER_FOOT
HOVER_TIME_SECONDS = 60
HOVER_ALTITUDE_TOLERANCE_METERS = 0.15
HOVER_STABLE_BEFORE_TIMER_SECONDS = 2.0
TAKEOFF_TIMEOUT_SECONDS = 30
HOVER_PREPARATION_TIMEOUT_SECONDS = 20
LAND_WAIT_TIMEOUT_SECONDS = 30

CONTROL_PERIOD_SECONDS = 0.1
STATUS_PRINT_INTERVAL_SECONDS = 1.0

# Firm climb until the gear is clear, then a gentler approach to 2 feet.
FIRM_TAKEOFF_CLIMB_RATE_METERS_PER_SECOND = 0.90
FIRM_TAKEOFF_UNTIL_METERS = 1.0 * METERS_PER_FOOT
MAXIMUM_APPROACH_CLIMB_RATE_METERS_PER_SECOND = 0.50
MAXIMUM_DESCENT_RATE_METERS_PER_SECOND = 0.30
ALTITUDE_PROPORTIONAL_GAIN = 0.8

# Two or more satellites means GPS data is arriving. Position hold is stricter.
GPS_DETECTED_MINIMUM_SATELLITES = 2
GPS_POSITION_HOLD_MINIMUM_FIX_TYPE = 3
GPS_POSITION_HOLD_MINIMUM_SATELLITES = 6
UNKNOWN_SATELLITE_COUNT = 255
GPS_POSITION_LOSS_BEFORE_LAND_SECONDS = 3.0

EKF_HORIZONTAL_POSITION_ABSOLUTE_BIT = 16
EKF_CONSTANT_POSITION_MODE_BIT = 128
EKF_GPS_GLITCHING_BIT = 32768

# MAV_SENSOR_ROTATION_PITCH_270. Aimed at the ground.
DOWNWARD_DISTANCE_SENSOR_ORIENTATION = 25
MINIMUM_VALID_RANGE_METERS = 0.02
MAXIMUM_VALID_RANGE_METERS = 10.0
SENSOR_MESSAGE_STALE_AFTER_SECONDS = 1.0

STANDARD_SEA_LEVEL_PRESSURE_HECTOPASCALS = 1013.25
BAROMETER_GROUND_SAMPLE_COUNT = 30
BAROMETER_GROUND_SAMPLE_TIMEOUT_SECONDS = 6.0

REQUIRED_MESSAGE_NAMES = (
    "HEARTBEAT",
    "ATTITUDE",
    "GPS_RAW_INT",
    "GLOBAL_POSITION_INT",
    "SCALED_PRESSURE",
    "DISTANCE_SENSOR",
    "RANGEFINDER",
    "EKF_STATUS_REPORT",
    "STATUSTEXT",
)

# With this bit off, SET_ATTITUDE_TARGET thrust is a climb rate, not motor throttle.
GUID_OPTIONS_RAW_THRUST_BIT = 8

TAKEOFF_SLEW_PARAMETER_NAME = "TKOFF_SLEW_TIME"
PREFERRED_TAKEOFF_SLEW_TIME_SECONDS = 1.0
PARAMETER_MATCH_TOLERANCE = 0.01

lastHeartbeatReceivedAtSeconds = None


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
    print("Requested flight telemetry.")


def processIncomingMessage(
    pixhawkConnection,
    latestMessages,
    messageArrivedAtSeconds,
    distanceSensors,
    message,
):
    """Keep each distance sensor separate so a forward one cannot erase a downward one."""
    if message is None:
        return
    if message.get_srcSystem() not in (0, pixhawkConnection.target_system):
        return
    messageType = message.get_type()
    arrivedAtSeconds = time.monotonic()
    if messageType == "DISTANCE_SENSOR":
        sensorKey = (
            int(getattr(message, "id", 0)),
            int(getattr(message, "orientation", -1)),
        )
        distanceSensors[sensorKey] = {
            "message": message,
            "arrivedAtSeconds": arrivedAtSeconds,
        }
    else:
        latestMessages[messageType] = message
        messageArrivedAtSeconds[messageType] = arrivedAtSeconds
    if messageType == "HEARTBEAT":
        rememberHeartbeat(pixhawkConnection, message)
    elif messageType == "STATUSTEXT":
        statusText = cleanStatusText(message)
        if statusText:
            print(f"PIXHAWK: {statusText}")


def readAvailableMessages(pixhawkConnection, latestMessages, messageArrivedAtSeconds, distanceSensors):
    while True:
        message = pixhawkConnection.recv_match(blocking=False)
        if message is None:
            return
        processIncomingMessage(
            pixhawkConnection,
            latestMessages,
            messageArrivedAtSeconds,
            distanceSensors,
            message,
        )


def messageIsFresh(messageArrivedAtSeconds, messageType, maximumAgeSeconds=SENSOR_MESSAGE_STALE_AFTER_SECONDS):
    arrivedAtSeconds = messageArrivedAtSeconds.get(messageType)
    if arrivedAtSeconds is None:
        return False
    return time.monotonic() - arrivedAtSeconds <= maximumAgeSeconds


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


def gpsIsDetected(latestMessages, messageArrivedAtSeconds):
    """Two or more satellites means GPS data is arriving. Not a position hold."""
    satelliteCount = gpsSatelliteCount(latestMessages, messageArrivedAtSeconds)
    if satelliteCount is None:
        return False
    return satelliteCount >= GPS_DETECTED_MINIMUM_SATELLITES


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


def listenForGps(pixhawkConnection, latestMessages, messageArrivedAtSeconds, distanceSensors):
    """Return True only when GPS is strong enough for LOITER."""
    print()
    print(f"Listening for GPS for {GPS_LISTEN_SECONDS} seconds.")
    print(f"{GPS_DETECTED_MINIMUM_SATELLITES}+ satellites means GPS is detected.")
    print(
        f"{GPS_POSITION_HOLD_MINIMUM_SATELLITES}+ satellites, a 3D fix, "
        "and a healthy position estimate are required for LOITER."
    )
    listenDeadlineSeconds = time.monotonic() + GPS_LISTEN_SECONDS
    lastPrintedAtSeconds = 0.0
    while time.monotonic() < listenDeadlineSeconds:
        readAvailableMessages(pixhawkConnection, latestMessages, messageArrivedAtSeconds, distanceSensors)
        if gpsIsUsableForPositionHold(latestMessages, messageArrivedAtSeconds):
            gpsReading = latestMessages["GPS_RAW_INT"]
            print(
                f"GPS position ready. Fix {gpsReading.fix_type} | "
                f"{gpsReading.satellites_visible} satellites."
            )
            return True
        nowSeconds = time.monotonic()
        if nowSeconds - lastPrintedAtSeconds >= 1.0:
            gpsReading = latestMessages.get("GPS_RAW_INT")
            satelliteCount = gpsSatelliteCount(latestMessages, messageArrivedAtSeconds)
            if gpsReading is None:
                print("GPS: no message yet.")
            elif gpsIsDetected(latestMessages, messageArrivedAtSeconds):
                print(
                    f"GPS detected. Fix {gpsReading.fix_type} | "
                    f"{satelliteCount} satellites. Not yet ready for LOITER."
                )
            else:
                print(f"GPS weak. Fix {gpsReading.fix_type} | satellites {satelliteCount}.")
            lastPrintedAtSeconds = nowSeconds
        time.sleep(0.1)

    print()
    if gpsIsDetected(latestMessages, messageArrivedAtSeconds):
        print("GPS data is coming through, but it is not strong enough for position hold.")
    else:
        print("GPS is not currently available.")
    print("The sensor hover will be used.")
    return False


def pressureToAltitudeMeters(pressureHectopascals):
    return 44330.0 * (
        1.0
        - (pressureHectopascals / STANDARD_SEA_LEVEL_PRESSURE_HECTOPASCALS) ** 0.1903
    )


def collectBarometerGroundReference(pixhawkConnection, latestMessages, messageArrivedAtSeconds, distanceSensors):
    print()
    print("Reading ground barometer reference...")
    pressureSamplesHectopascals = []
    collectionDeadlineSeconds = time.monotonic() + BAROMETER_GROUND_SAMPLE_TIMEOUT_SECONDS
    lastSampleTimestamp = None
    while time.monotonic() < collectionDeadlineSeconds:
        readAvailableMessages(pixhawkConnection, latestMessages, messageArrivedAtSeconds, distanceSensors)
        if not messageIsFresh(messageArrivedAtSeconds, "SCALED_PRESSURE"):
            time.sleep(0.05)
            continue
        pressureMessage = latestMessages.get("SCALED_PRESSURE")
        currentMessageTimestamp = messageArrivedAtSeconds.get("SCALED_PRESSURE")
        if pressureMessage is None or currentMessageTimestamp == lastSampleTimestamp:
            time.sleep(0.05)
            continue
        pressureHectopascals = getattr(pressureMessage, "press_abs", None)
        if isinstance(pressureHectopascals, (int, float)) and pressureHectopascals > 0:
            pressureSamplesHectopascals.append(float(pressureHectopascals))
            lastSampleTimestamp = currentMessageTimestamp
        if len(pressureSamplesHectopascals) >= BAROMETER_GROUND_SAMPLE_COUNT:
            break
        time.sleep(0.05)

    if not pressureSamplesHectopascals:
        print("No usable barometer reference was collected.")
        return None

    averageGroundPressureHectopascals = (
        sum(pressureSamplesHectopascals) / len(pressureSamplesHectopascals)
    )
    print(
        f"Ground barometer: {averageGroundPressureHectopascals:.2f} hPa "
        f"from {len(pressureSamplesHectopascals)} samples."
    )
    return pressureToAltitudeMeters(averageGroundPressureHectopascals)


def barometerAltitudeMeters(latestMessages, messageArrivedAtSeconds, barometerGroundReferenceMeters):
    if barometerGroundReferenceMeters is None:
        return None
    if not messageIsFresh(messageArrivedAtSeconds, "SCALED_PRESSURE"):
        return None
    pressureMessage = latestMessages.get("SCALED_PRESSURE")
    if pressureMessage is None:
        return None
    pressureHectopascals = getattr(pressureMessage, "press_abs", None)
    if not isinstance(pressureHectopascals, (int, float)) or pressureHectopascals <= 0:
        return None
    return pressureToAltitudeMeters(float(pressureHectopascals)) - barometerGroundReferenceMeters


def downwardDistanceSensorAltitudeMeters(distanceSensors):
    """Use only a fresh downward sensor. A forward sensor stays in the list and is ignored."""
    bestDistanceMeters = None
    newestArrivalSeconds = None
    nowSeconds = time.monotonic()
    for sensorData in distanceSensors.values():
        arrivedAtSeconds = sensorData["arrivedAtSeconds"]
        if nowSeconds - arrivedAtSeconds > SENSOR_MESSAGE_STALE_AFTER_SECONDS:
            continue
        message = sensorData["message"]
        if int(getattr(message, "orientation", -1)) != DOWNWARD_DISTANCE_SENSOR_ORIENTATION:
            continue
        distanceCentimeters = getattr(message, "current_distance", None)
        if not isinstance(distanceCentimeters, (int, float)):
            continue
        distanceMeters = float(distanceCentimeters) / 100.0
        if not MINIMUM_VALID_RANGE_METERS <= distanceMeters <= MAXIMUM_VALID_RANGE_METERS:
            continue
        if newestArrivalSeconds is None or arrivedAtSeconds > newestArrivalSeconds:
            newestArrivalSeconds = arrivedAtSeconds
            bestDistanceMeters = distanceMeters
    return bestDistanceMeters


def rangefinderAltitudeMeters(latestMessages, messageArrivedAtSeconds):
    if not messageIsFresh(messageArrivedAtSeconds, "RANGEFINDER"):
        return None
    rangefinderMessage = latestMessages.get("RANGEFINDER")
    if rangefinderMessage is None:
        return None
    distanceMeters = getattr(rangefinderMessage, "distance", None)
    if not isinstance(distanceMeters, (int, float)):
        return None
    distanceMeters = float(distanceMeters)
    if not MINIMUM_VALID_RANGE_METERS <= distanceMeters <= MAXIMUM_VALID_RANGE_METERS:
        return None
    return distanceMeters


def chooseBestAltitudeMeters(latestMessages, messageArrivedAtSeconds, distanceSensors, barometerGroundReferenceMeters):
    """GPS is not a height source. Downward range, then rangefinder, then barometer."""
    downwardAltitudeMeters = downwardDistanceSensorAltitudeMeters(distanceSensors)
    if downwardAltitudeMeters is not None:
        return downwardAltitudeMeters, "downward distance sensor"
    rangefinderMeters = rangefinderAltitudeMeters(latestMessages, messageArrivedAtSeconds)
    if rangefinderMeters is not None:
        return rangefinderMeters, "rangefinder"
    barometerMeters = barometerAltitudeMeters(
        latestMessages,
        messageArrivedAtSeconds,
        barometerGroundReferenceMeters,
    )
    if barometerMeters is not None:
        return barometerMeters, "barometer"
    return None, "none"


def createAltitudeManager(latestMessages, messageArrivedAtSeconds, distanceSensors, barometerGroundReferenceMeters):
    """Pick one height source before takeoff and keep it until it actually stops."""
    altitudeMeters, sourceName = chooseBestAltitudeMeters(
        latestMessages,
        messageArrivedAtSeconds,
        distanceSensors,
        barometerGroundReferenceMeters,
    )
    if altitudeMeters is None:
        return None
    print(f"Selected height source: {sourceName}. It stays selected for the takeoff.")
    return {
        "sourceName": sourceName,
        "sourceOffsetMeters": 0.0,
        "lastAltitudeMeters": altitudeMeters,
    }


def readNamedAltitudeMeters(sourceName, latestMessages, messageArrivedAtSeconds, distanceSensors, barometerGroundReferenceMeters):
    if sourceName == "downward distance sensor":
        return downwardDistanceSensorAltitudeMeters(distanceSensors)
    if sourceName == "rangefinder":
        return rangefinderAltitudeMeters(latestMessages, messageArrivedAtSeconds)
    if sourceName == "barometer":
        return barometerAltitudeMeters(
            latestMessages,
            messageArrivedAtSeconds,
            barometerGroundReferenceMeters,
        )
    return None


def managedAltitudeMeters(
    altitudeManager,
    latestMessages,
    messageArrivedAtSeconds,
    distanceSensors,
    barometerGroundReferenceMeters,
):
    """
    Keep the same height source during liftoff.
    A sudden switch would look like a jump and can tip the copter.
    If the source really disappears, continue from the last height.
    """
    currentSourceName = altitudeManager["sourceName"]
    sourceOffsetMeters = altitudeManager["sourceOffsetMeters"]
    rawAltitudeMeters = readNamedAltitudeMeters(
        currentSourceName,
        latestMessages,
        messageArrivedAtSeconds,
        distanceSensors,
        barometerGroundReferenceMeters,
    )
    if rawAltitudeMeters is not None:
        adjustedAltitudeMeters = rawAltitudeMeters + sourceOffsetMeters
        altitudeManager["lastAltitudeMeters"] = adjustedAltitudeMeters
        return adjustedAltitudeMeters, currentSourceName

    previousAltitudeMeters = altitudeManager.get("lastAltitudeMeters")
    for fallbackSourceName in ("downward distance sensor", "rangefinder", "barometer"):
        if fallbackSourceName == currentSourceName:
            continue
        fallbackRawAltitudeMeters = readNamedAltitudeMeters(
            fallbackSourceName,
            latestMessages,
            messageArrivedAtSeconds,
            distanceSensors,
            barometerGroundReferenceMeters,
        )
        if fallbackRawAltitudeMeters is None:
            continue
        if previousAltitudeMeters is None:
            newOffsetMeters = 0.0
        else:
            newOffsetMeters = previousAltitudeMeters - fallbackRawAltitudeMeters
        altitudeManager["sourceName"] = fallbackSourceName
        altitudeManager["sourceOffsetMeters"] = newOffsetMeters
        adjustedAltitudeMeters = fallbackRawAltitudeMeters + newOffsetMeters
        altitudeManager["lastAltitudeMeters"] = adjustedAltitudeMeters
        print(f"Height source changed from {currentSourceName} to {fallbackSourceName}.")
        print(f"Continuity offset {newOffsetMeters:+.3f} m, so the climb does not jump.")
        return adjustedAltitudeMeters, fallbackSourceName
    return None, "none"


def readPixhawkParameter(pixhawkConnection, latestMessages, messageArrivedAtSeconds, distanceSensors, parameterName):
    """Read one parameter without throwing away heartbeats that arrive meanwhile."""
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
            distanceSensors,
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
    distanceSensors,
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
            distanceSensors,
            message,
        )
        if message.get_type() != "PARAM_VALUE":
            continue
        if parameterNameText(message.param_id) != parameterName:
            continue
        if abs(float(message.param_value) - requestedValue) < PARAMETER_MATCH_TOLERANCE:
            return True
    return False


def convertArduPilotSpeedToMetersPerSecond(parameterValue, fallbackMetersPerSecond):
    """WPNAV speeds are normally centimetres per second."""
    if parameterValue is None or parameterValue <= 0:
        return fallbackMetersPerSecond
    if parameterValue > 20:
        return parameterValue / 100.0
    return float(parameterValue)


def prepareFirmerGuidedTakeoff(pixhawkConnection, latestMessages, messageArrivedAtSeconds, distanceSensors):
    """Never make an already-firm takeoff ramp slower. Return the original value."""
    originalTakeoffSlewSeconds = readPixhawkParameter(
        pixhawkConnection,
        latestMessages,
        messageArrivedAtSeconds,
        distanceSensors,
        TAKEOFF_SLEW_PARAMETER_NAME,
    )
    if originalTakeoffSlewSeconds is None:
        print(f"{TAKEOFF_SLEW_PARAMETER_NAME} was not available. The existing takeoff ramp will be used.")
        return None

    temporaryTakeoffSlewSeconds = min(
        originalTakeoffSlewSeconds,
        PREFERRED_TAKEOFF_SLEW_TIME_SECONDS,
    )
    print(
        f"{TAKEOFF_SLEW_PARAMETER_NAME}: original {originalTakeoffSlewSeconds:.2f} s | "
        f"takeoff {temporaryTakeoffSlewSeconds:.2f} s"
    )
    if abs(temporaryTakeoffSlewSeconds - originalTakeoffSlewSeconds) < PARAMETER_MATCH_TOLERANCE:
        return None

    if not setPixhawkParameterAndConfirm(
        pixhawkConnection,
        latestMessages,
        messageArrivedAtSeconds,
        distanceSensors,
        TAKEOFF_SLEW_PARAMETER_NAME,
        temporaryTakeoffSlewSeconds,
    ):
        print(f"Could not confirm {TAKEOFF_SLEW_PARAMETER_NAME}. Putting the original ramp back.")
        setPixhawkParameterAndConfirm(
            pixhawkConnection,
            latestMessages,
            messageArrivedAtSeconds,
            distanceSensors,
            TAKEOFF_SLEW_PARAMETER_NAME,
            originalTakeoffSlewSeconds,
        )
        return None
    return originalTakeoffSlewSeconds


def restoreTakeoffSlew(pixhawkConnection, latestMessages, messageArrivedAtSeconds, distanceSensors, originalTakeoffSlewSeconds):
    print(f"Restoring {TAKEOFF_SLEW_PARAMETER_NAME} to {originalTakeoffSlewSeconds:.2f} s.")
    if setPixhawkParameterAndConfirm(
        pixhawkConnection,
        latestMessages,
        messageArrivedAtSeconds,
        distanceSensors,
        TAKEOFF_SLEW_PARAMETER_NAME,
        originalTakeoffSlewSeconds,
    ):
        print(f"{TAKEOFF_SLEW_PARAMETER_NAME} restored.")
    else:
        print(f"Could not confirm restoration of {TAKEOFF_SLEW_PARAMETER_NAME}.")


ARMING_SKIP_GPS_LOCK_VALUE = 1 << 3
ARMING_SKIP_GPS_CONFIGURATION_VALUE = 1 << 12
SENSOR_PATH_GPS_SKIP_MASK = ARMING_SKIP_GPS_LOCK_VALUE | ARMING_SKIP_GPS_CONFIGURATION_VALUE
ARMING_SKIP_CHECK_PARAMETER_NAME = "ARMING_SKIPCHK"
ARMING_NEED_LOCATION_PARAMETER_NAME = "ARMING_NEED_LOC"


def prepareSensorOnlyArming(pixhawkConnection, latestMessages, messageArrivedAtSeconds, distanceSensors):
    """Skip only the GPS arming checks. Compass and the other checks stay on."""
    originalArmingSkipChecks = readPixhawkParameter(
        pixhawkConnection,
        latestMessages,
        messageArrivedAtSeconds,
        distanceSensors,
        ARMING_SKIP_CHECK_PARAMETER_NAME,
    )
    originalArmingNeedLocation = readPixhawkParameter(
        pixhawkConnection,
        latestMessages,
        messageArrivedAtSeconds,
        distanceSensors,
        ARMING_NEED_LOCATION_PARAMETER_NAME,
    )
    if originalArmingSkipChecks is None:
        raise RuntimeError("Could not read ARMING_SKIPCHK.")
    if originalArmingNeedLocation is None:
        raise RuntimeError("Could not read ARMING_NEED_LOC.")

    originalArmingSkipChecksInteger = int(round(originalArmingSkipChecks))
    temporaryArmingSkipChecks = originalArmingSkipChecksInteger | SENSOR_PATH_GPS_SKIP_MASK
    print(f"Sensor path ARMING_SKIPCHK: {originalArmingSkipChecksInteger} -> {temporaryArmingSkipChecks}.")
    if temporaryArmingSkipChecks != originalArmingSkipChecksInteger:
        if not setPixhawkParameterAndConfirm(
            pixhawkConnection,
            latestMessages,
            messageArrivedAtSeconds,
            distanceSensors,
            ARMING_SKIP_CHECK_PARAMETER_NAME,
            temporaryArmingSkipChecks,
        ):
            raise RuntimeError("Could not change the GPS arming checks.")
    if int(round(originalArmingNeedLocation)) != 0:
        print(f"ARMING_NEED_LOC: {int(round(originalArmingNeedLocation))} -> 0.")
        if not setPixhawkParameterAndConfirm(
            pixhawkConnection,
            latestMessages,
            messageArrivedAtSeconds,
            distanceSensors,
            ARMING_NEED_LOCATION_PARAMETER_NAME,
            0,
        ):
            raise RuntimeError("Could not turn off the absolute-location arming requirement.")
    return {
        "armingSkipChecks": originalArmingSkipChecks,
        "armingNeedLocation": originalArmingNeedLocation,
    }


def restoreSensorOnlyArming(
    pixhawkConnection,
    latestMessages,
    messageArrivedAtSeconds,
    distanceSensors,
    originalSensorPathParameters,
):
    if originalSensorPathParameters is None:
        return
    print("Restoring the GPS arming checks.")
    setPixhawkParameterAndConfirm(
        pixhawkConnection,
        latestMessages,
        messageArrivedAtSeconds,
        distanceSensors,
        ARMING_SKIP_CHECK_PARAMETER_NAME,
        originalSensorPathParameters["armingSkipChecks"],
    )
    setPixhawkParameterAndConfirm(
        pixhawkConnection,
        latestMessages,
        messageArrivedAtSeconds,
        distanceSensors,
        ARMING_NEED_LOCATION_PARAMETER_NAME,
        originalSensorPathParameters["armingNeedLocation"],
    )
    print("GPS arming checks restored.")


def setFlightModeAndWait(
    pixhawkConnection,
    latestMessages,
    messageArrivedAtSeconds,
    distanceSensors,
    requestedModeNumber,
    requestedModeName,
):
    print(f"Requesting {requestedModeName}.")
    pixhawkConnection.mav.set_mode_send(
        pixhawkConnection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        requestedModeNumber,
    )
    deadlineSeconds = time.monotonic() + 10.0
    while time.monotonic() < deadlineSeconds:
        readAvailableMessages(pixhawkConnection, latestMessages, messageArrivedAtSeconds, distanceSensors)
        if copterIsInMode(latestMessages, requestedModeNumber):
            print(f"{requestedModeName} confirmed.")
            return True
        if not heartbeatIsFresh():
            print("Heartbeat became stale while waiting for the mode change.")
            return False
        time.sleep(0.1)
    print(f"{requestedModeName} was not confirmed.")
    return False


def armAndWaitForConfirmation(pixhawkConnection, latestMessages, messageArrivedAtSeconds, distanceSensors):
    print()
    print("Sending ARM command.")
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
    deadlineSeconds = time.monotonic() + 10.0
    armCommandResult = None
    while time.monotonic() < deadlineSeconds:
        message = pixhawkConnection.recv_match(
            type=["COMMAND_ACK", "STATUSTEXT", "HEARTBEAT"],
            blocking=True,
            timeout=0.5,
        )
        if message is None:
            continue
        processIncomingMessage(
            pixhawkConnection,
            latestMessages,
            messageArrivedAtSeconds,
            distanceSensors,
            message,
        )
        messageType = message.get_type()
        if messageType == "HEARTBEAT" and copterIsArmed(latestMessages):
            print("Armed confirmed.")
            return True
        if messageType == "COMMAND_ACK" and message.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            armCommandResult = message.result
            print(f"ARM answer = {armCommandResult}")
    print(f"ARM FAILED. Answer = {armCommandResult}")
    return False


def createLevelQuaternionFromCurrentYaw(latestMessages, messageArrivedAtSeconds):
    if not messageIsFresh(messageArrivedAtSeconds, "ATTITUDE"):
        return None
    attitudeMessage = latestMessages.get("ATTITUDE")
    if attitudeMessage is None:
        return None
    currentYawRadians = float(attitudeMessage.yaw)
    return [math.cos(currentYawRadians / 2.0), 0.0, 0.0, math.sin(currentYawRadians / 2.0)]


def sendLevelAttitudeCommand(pixhawkConnection, levelYawQuaternion, climbRateField):
    ignoreBodyRatesMask = (
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
        | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
        | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
    )
    pixhawkConnection.mav.set_attitude_target_send(
        0,
        pixhawkConnection.target_system,
        pixhawkConnection.target_component,
        ignoreBodyRatesMask,
        levelYawQuaternion,
        0,
        0,
        0,
        climbRateField,
    )


def calculateRequestedClimbRate(currentAltitudeMeters):
    """Firm climb to about 1 foot, then a gentler correction toward 2 feet."""
    if currentAltitudeMeters < FIRM_TAKEOFF_UNTIL_METERS:
        return FIRM_TAKEOFF_CLIMB_RATE_METERS_PER_SECOND, "FIRM TAKEOFF"
    altitudeErrorMeters = TARGET_ALTITUDE_METERS - currentAltitudeMeters
    requestedClimbRateMetersPerSecond = ALTITUDE_PROPORTIONAL_GAIN * altitudeErrorMeters
    requestedClimbRateMetersPerSecond = max(
        -MAXIMUM_DESCENT_RATE_METERS_PER_SECOND,
        min(MAXIMUM_APPROACH_CLIMB_RATE_METERS_PER_SECOND, requestedClimbRateMetersPerSecond),
    )
    if abs(altitudeErrorMeters) <= HOVER_ALTITUDE_TOLERANCE_METERS:
        flightPhase = "HOLD"
    elif requestedClimbRateMetersPerSecond > 0:
        flightPhase = "APPROACH"
    else:
        flightPhase = "EASE DOWN"
    return requestedClimbRateMetersPerSecond, flightPhase


def climbRateToAttitudeTargetField(
    requestedClimbRateMetersPerSecond,
    maximumClimbSpeedMetersPerSecond,
    maximumDescentSpeedMetersPerSecond,
):
    """
    0.5 means do not climb. 1.0 means climb at the Pixhawk's maximum climb speed.
    0.0 means descend at the Pixhawk's maximum descent speed.
    This is only valid when GUID_OPTIONS bit 3 is off.
    """
    if requestedClimbRateMetersPerSecond >= 0:
        if maximumClimbSpeedMetersPerSecond <= 0:
            return 0.5
        climbRateField = 0.5 + 0.5 * (
            requestedClimbRateMetersPerSecond / maximumClimbSpeedMetersPerSecond
        )
    else:
        if maximumDescentSpeedMetersPerSecond <= 0:
            return 0.5
        climbRateField = 0.5 - 0.5 * (
            abs(requestedClimbRateMetersPerSecond) / maximumDescentSpeedMetersPerSecond
        )
    return max(0.0, min(1.0, climbRateField))


def runSensorOnlyHover(
    pixhawkConnection,
    latestMessages,
    messageArrivedAtSeconds,
    distanceSensors,
    barometerGroundReferenceMeters,
    altitudeManager,
    maximumClimbSpeedMetersPerSecond,
    maximumDescentSpeedMetersPerSecond,
):
    levelYawQuaternion = createLevelQuaternionFromCurrentYaw(latestMessages, messageArrivedAtSeconds)
    if levelYawQuaternion is None:
        raise RuntimeError("No fresh attitude telemetry.")

    print()
    print("SENSOR HOVER")
    print("GPS position hold is not being used.")
    print("Height is coming from the sensors that are answering.")

    flightStartedAtSeconds = time.monotonic()
    stableAltitudeStartedAtSeconds = None
    hoverStartedAtSeconds = None
    lastStatusPrintedAtSeconds = 0.0

    while True:
        loopStartedAtSeconds = time.monotonic()
        readAvailableMessages(pixhawkConnection, latestMessages, messageArrivedAtSeconds, distanceSensors)
        if not heartbeatIsFresh():
            raise RuntimeError("Pixhawk heartbeat was lost.")
        if copterIsArmed(latestMessages) is not True:
            raise RuntimeError("The copter became disarmed.")
        if not copterIsInMode(latestMessages, GUIDED_NOGPS_MODE_NUMBER):
            raise RuntimeError("GUIDED_NOGPS is no longer active.")
        if not messageIsFresh(messageArrivedAtSeconds, "ATTITUDE"):
            raise RuntimeError("Attitude telemetry became stale.")

        currentAltitudeMeters, altitudeSource = managedAltitudeMeters(
            altitudeManager,
            latestMessages,
            messageArrivedAtSeconds,
            distanceSensors,
            barometerGroundReferenceMeters,
        )
        if currentAltitudeMeters is None:
            raise RuntimeError("All height sources were lost.")

        requestedClimbRateMetersPerSecond, flightPhase = calculateRequestedClimbRate(currentAltitudeMeters)
        climbRateField = climbRateToAttitudeTargetField(
            requestedClimbRateMetersPerSecond,
            maximumClimbSpeedMetersPerSecond,
            maximumDescentSpeedMetersPerSecond,
        )
        sendLevelAttitudeCommand(pixhawkConnection, levelYawQuaternion, climbRateField)

        altitudeErrorMeters = abs(TARGET_ALTITUDE_METERS - currentAltitudeMeters)
        currentTimeSeconds = time.monotonic()
        if hoverStartedAtSeconds is None:
            if altitudeErrorMeters <= HOVER_ALTITUDE_TOLERANCE_METERS:
                if stableAltitudeStartedAtSeconds is None:
                    stableAltitudeStartedAtSeconds = currentTimeSeconds
                    print("Target height reached. Checking that it stays there.")
                if currentTimeSeconds - stableAltitudeStartedAtSeconds >= HOVER_STABLE_BEFORE_TIMER_SECONDS:
                    hoverStartedAtSeconds = currentTimeSeconds
                    print(f"Height stayed steady. Starting the {HOVER_TIME_SECONDS:.0f} second hover.")
            else:
                stableAltitudeStartedAtSeconds = None
            hoverElapsedSeconds = 0.0
            if currentTimeSeconds - flightStartedAtSeconds >= TAKEOFF_TIMEOUT_SECONDS:
                raise RuntimeError("The copter did not settle at 2 feet in time.")
        else:
            hoverElapsedSeconds = currentTimeSeconds - hoverStartedAtSeconds
            if hoverElapsedSeconds >= HOVER_TIME_SECONDS:
                print("60 second sensor hover complete.")
                return

        if currentTimeSeconds - lastStatusPrintedAtSeconds >= STATUS_PRINT_INTERVAL_SECONDS:
            print(
                f"{flightPhase} | {currentAltitudeMeters:.2f} m | "
                f"{currentAltitudeMeters * FEET_PER_METER:.2f} ft | {altitudeSource} | "
                f"climb {requestedClimbRateMetersPerSecond:+.2f} m/s | "
                f"hover {hoverElapsedSeconds:.0f}/{HOVER_TIME_SECONDS:.0f} s"
            )
            lastStatusPrintedAtSeconds = currentTimeSeconds

        remainingLoopDelaySeconds = CONTROL_PERIOD_SECONDS - (time.monotonic() - loopStartedAtSeconds)
        if remainingLoopDelaySeconds > 0:
            time.sleep(remainingLoopDelaySeconds)


def waitForTakeoffAltitude(
    pixhawkConnection,
    latestMessages,
    messageArrivedAtSeconds,
    distanceSensors,
    barometerGroundReferenceMeters,
    altitudeManager,
):
    """Height during the GPS takeoff still comes from the sensor chain, not from GPS."""
    minimumTakeoffAltitudeMeters = TARGET_ALTITUDE_METERS * 0.90
    deadlineSeconds = time.monotonic() + TAKEOFF_TIMEOUT_SECONDS
    lastStatusPrintedAtSeconds = 0.0
    while time.monotonic() < deadlineSeconds:
        readAvailableMessages(pixhawkConnection, latestMessages, messageArrivedAtSeconds, distanceSensors)
        if not heartbeatIsFresh():
            return False
        if copterIsArmed(latestMessages) is not True:
            return False
        if not copterIsInMode(latestMessages, GUIDED_MODE_NUMBER):
            return False
        currentAltitudeMeters, altitudeSource = managedAltitudeMeters(
            altitudeManager,
            latestMessages,
            messageArrivedAtSeconds,
            distanceSensors,
            barometerGroundReferenceMeters,
        )
        nowSeconds = time.monotonic()
        if (
            currentAltitudeMeters is not None
            and nowSeconds - lastStatusPrintedAtSeconds >= STATUS_PRINT_INTERVAL_SECONDS
        ):
            print(
                f"TAKEOFF | {currentAltitudeMeters:.2f} m | "
                f"{currentAltitudeMeters * FEET_PER_METER:.2f} ft | {altitudeSource}"
            )
            lastStatusPrintedAtSeconds = nowSeconds
        if currentAltitudeMeters is not None and currentAltitudeMeters >= minimumTakeoffAltitudeMeters:
            print("Takeoff height reached.")
            return True
        time.sleep(0.1)
    return False


def waitForStableAltitude(
    pixhawkConnection,
    latestMessages,
    messageArrivedAtSeconds,
    distanceSensors,
    barometerGroundReferenceMeters,
    altitudeManager,
):
    stableAltitudeStartedAtSeconds = None
    deadlineSeconds = time.monotonic() + HOVER_PREPARATION_TIMEOUT_SECONDS
    print("Waiting for the height to stay near 2 feet.")
    while time.monotonic() < deadlineSeconds:
        readAvailableMessages(pixhawkConnection, latestMessages, messageArrivedAtSeconds, distanceSensors)
        if not heartbeatIsFresh():
            return False
        currentAltitudeMeters, altitudeSource = managedAltitudeMeters(
            altitudeManager,
            latestMessages,
            messageArrivedAtSeconds,
            distanceSensors,
            barometerGroundReferenceMeters,
        )
        if currentAltitudeMeters is None:
            stableAltitudeStartedAtSeconds = None
            time.sleep(0.1)
            continue
        altitudeErrorMeters = abs(TARGET_ALTITUDE_METERS - currentAltitudeMeters)
        if altitudeErrorMeters <= HOVER_ALTITUDE_TOLERANCE_METERS:
            if stableAltitudeStartedAtSeconds is None:
                stableAltitudeStartedAtSeconds = time.monotonic()
            if time.monotonic() - stableAltitudeStartedAtSeconds >= HOVER_STABLE_BEFORE_TIMER_SECONDS:
                print(
                    f"Height is steady at {currentAltitudeMeters * FEET_PER_METER:.2f} ft "
                    f"using {altitudeSource}."
                )
                return True
        else:
            stableAltitudeStartedAtSeconds = None
        time.sleep(0.1)
    return False


def runGpsHover(
    pixhawkConnection,
    latestMessages,
    messageArrivedAtSeconds,
    distanceSensors,
    barometerGroundReferenceMeters,
    altitudeManager,
):
    print()
    print("GPS HOVER")
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
    if not waitForTakeoffAltitude(
        pixhawkConnection,
        latestMessages,
        messageArrivedAtSeconds,
        distanceSensors,
        barometerGroundReferenceMeters,
        altitudeManager,
    ):
        raise RuntimeError("GUIDED takeoff did not reach 2 feet.")
    if not setFlightModeAndWait(
        pixhawkConnection,
        latestMessages,
        messageArrivedAtSeconds,
        distanceSensors,
        LOITER_MODE_NUMBER,
        "LOITER",
    ):
        raise RuntimeError("LOITER was not confirmed.")
    if not waitForStableAltitude(
        pixhawkConnection,
        latestMessages,
        messageArrivedAtSeconds,
        distanceSensors,
        barometerGroundReferenceMeters,
        altitudeManager,
    ):
        raise RuntimeError("The copter did not settle at 2 feet.")

    print()
    print(f"Starting the {HOVER_TIME_SECONDS:.0f} second GPS hover.")
    hoverStartedAtSeconds = time.monotonic()
    gpsLostAtSeconds = None
    lastStatusPrintedAtSeconds = 0.0
    while time.monotonic() - hoverStartedAtSeconds < HOVER_TIME_SECONDS:
        readAvailableMessages(pixhawkConnection, latestMessages, messageArrivedAtSeconds, distanceSensors)
        if not heartbeatIsFresh():
            raise RuntimeError("Pixhawk heartbeat was lost.")
        if copterIsArmed(latestMessages) is not True:
            raise RuntimeError("The copter became disarmed.")
        if not copterIsInMode(latestMessages, LOITER_MODE_NUMBER):
            raise RuntimeError("LOITER changed during the hover.")
        if gpsIsUsableForPositionHold(latestMessages, messageArrivedAtSeconds):
            gpsLostAtSeconds = None
        else:
            if gpsLostAtSeconds is None:
                gpsLostAtSeconds = time.monotonic()
                print("GPS position quality dropped.")
            if time.monotonic() - gpsLostAtSeconds >= GPS_POSITION_LOSS_BEFORE_LAND_SECONDS:
                raise RuntimeError("GPS position quality stayed too poor for LOITER.")

        currentAltitudeMeters, altitudeSource = managedAltitudeMeters(
            altitudeManager,
            latestMessages,
            messageArrivedAtSeconds,
            distanceSensors,
            barometerGroundReferenceMeters,
        )
        nowSeconds = time.monotonic()
        if nowSeconds - lastStatusPrintedAtSeconds >= STATUS_PRINT_INTERVAL_SECONDS:
            hoverElapsedSeconds = nowSeconds - hoverStartedAtSeconds
            satelliteCount = gpsSatelliteCount(latestMessages, messageArrivedAtSeconds)
            if currentAltitudeMeters is None:
                altitudeText = "height missing"
            else:
                altitudeText = (
                    f"{currentAltitudeMeters:.2f} m | "
                    f"{currentAltitudeMeters * FEET_PER_METER:.2f} ft | {altitudeSource}"
                )
            print(
                f"LOITER | {altitudeText} | GPS satellites {satelliteCount} | "
                f"hover {hoverElapsedSeconds:.0f}/{HOVER_TIME_SECONDS:.0f} s"
            )
            lastStatusPrintedAtSeconds = nowSeconds
        time.sleep(0.1)
    print("60 second GPS hover complete.")


def commandLand(pixhawkConnection, latestMessages, messageArrivedAtSeconds, distanceSensors):
    if not heartbeatIsFresh():
        print("Heartbeat is stale. LAND was not sent.")
        return False
    print()
    print("LAND")
    return setFlightModeAndWait(
        pixhawkConnection,
        latestMessages,
        messageArrivedAtSeconds,
        distanceSensors,
        LAND_MODE_NUMBER,
        "LAND",
    )


def waitForLandingToFinish(
    pixhawkConnection,
    latestMessages,
    messageArrivedAtSeconds,
    distanceSensors,
    barometerGroundReferenceMeters,
    altitudeManager,
):
    landingStartedAtSeconds = time.monotonic()
    lastStatusPrintedAtSeconds = 0.0
    while time.monotonic() - landingStartedAtSeconds < LAND_WAIT_TIMEOUT_SECONDS:
        readAvailableMessages(pixhawkConnection, latestMessages, messageArrivedAtSeconds, distanceSensors)
        if copterIsArmed(latestMessages) is False:
            print("Motors disarmed.")
            return True
        nowSeconds = time.monotonic()
        if nowSeconds - lastStatusPrintedAtSeconds >= STATUS_PRINT_INTERVAL_SECONDS:
            currentAltitudeMeters, altitudeSource = chooseBestAltitudeMeters(
                latestMessages,
                messageArrivedAtSeconds,
                barometerGroundReferenceMeters,
            )
            if currentAltitudeMeters is None:
                print("LAND | height unavailable")
            else:
                print(
                    f"LAND | {currentAltitudeMeters:.2f} m | "
                    f"{currentAltitudeMeters * FEET_PER_METER:.2f} ft | {altitudeSource}"
                )
            lastStatusPrintedAtSeconds = nowSeconds
        time.sleep(0.1)
    print("Landing wait ended before disarm was confirmed.")
    return False


def main():
    pixhawkConnection = None
    latestMessages = {}
    messageArrivedAtSeconds = {}
    distanceSensors = {}
    altitudeManager = None
    aircraftWasArmed = False
    barometerGroundReferenceMeters = None
    gpsPositionHoldWillBeUsed = False
    originalTakeoffSlewSeconds = None
    originalSensorPathParameters = None

    try:
        print()
        print("60 SECOND HOVER - OPTIONAL GPS")
        print(f"Target height: {TARGET_ALTITUDE_FEET:.1f} feet.")
        print(f"Hover time: {HOVER_TIME_SECONDS:.0f} seconds.")
        print()

        pixhawkConnection = connectToPixhawk()
        requestRequiredTelemetry(pixhawkConnection)

        print()
        print("Collecting initial telemetry.")
        telemetryDeadlineSeconds = time.monotonic() + INITIAL_TELEMETRY_COLLECTION_SECONDS
        while time.monotonic() < telemetryDeadlineSeconds:
            readAvailableMessages(pixhawkConnection, latestMessages, messageArrivedAtSeconds, distanceSensors)
            time.sleep(0.05)
        if not heartbeatIsFresh():
            raise RuntimeError("No fresh Pixhawk heartbeat.")

        barometerGroundReferenceMeters = collectBarometerGroundReference(
            pixhawkConnection,
            latestMessages,
            messageArrivedAtSeconds,
            distanceSensors,
        )
        altitudeManager = createAltitudeManager(
            latestMessages,
            messageArrivedAtSeconds,
            distanceSensors,
            barometerGroundReferenceMeters,
        )
        if altitudeManager is None:
            raise RuntimeError("No usable height source is available.")

        guidedOptions = readPixhawkParameter(
            pixhawkConnection,
            latestMessages,
            messageArrivedAtSeconds,
            distanceSensors,
            "GUID_OPTIONS",
        )
        if guidedOptions is None:
            raise RuntimeError("Could not read GUID_OPTIONS.")
        guidedOptionsInteger = int(round(guidedOptions))
        print(f"GUID_OPTIONS = {guidedOptionsInteger}.")
        if guidedOptionsInteger & GUID_OPTIONS_RAW_THRUST_BIT:
            raise RuntimeError("GUID_OPTIONS bit 3 is on. The climb request would be raw motor thrust.")

        maximumClimbSpeedMetersPerSecond = convertArduPilotSpeedToMetersPerSecond(
            readPixhawkParameter(
                pixhawkConnection,
                latestMessages,
                messageArrivedAtSeconds,
                distanceSensors,
                "WPNAV_SPEED_UP",
            ),
            2.5,
        )
        maximumDescentSpeedMetersPerSecond = convertArduPilotSpeedToMetersPerSecond(
            readPixhawkParameter(
                pixhawkConnection,
                latestMessages,
                messageArrivedAtSeconds,
                distanceSensors,
                "WPNAV_SPEED_DN",
            ),
            1.5,
        )
        print(f"Maximum climb {maximumClimbSpeedMetersPerSecond:.2f} m/s.")
        print(f"Maximum descent {maximumDescentSpeedMetersPerSecond:.2f} m/s.")

        gpsPositionHoldWillBeUsed = listenForGps(
            pixhawkConnection,
            latestMessages,
            messageArrivedAtSeconds,
            distanceSensors,
        )

        print()
        if gpsPositionHoldWillBeUsed:
            print("Flight path: GUIDED, takeoff, LOITER, LAND.")
        else:
            print("Flight path: GUIDED_NOGPS, sensor hover, LAND.")
        print(f"Target height: {TARGET_ALTITUDE_FEET:.1f} ft.")
        print(
            f"The height must stay near the target for "
            f"{HOVER_STABLE_BEFORE_TIMER_SECONDS:.0f} seconds before the 60 second clock starts."
        )
        print()
        print("Type YES to begin.")
        if input("> ").strip() != "YES":
            print("Cancelled.")
            return

        if gpsPositionHoldWillBeUsed:
            originalTakeoffSlewSeconds = prepareFirmerGuidedTakeoff(
                pixhawkConnection,
                latestMessages,
                messageArrivedAtSeconds,
                distanceSensors,
            )
            initialModeNumber = GUIDED_MODE_NUMBER
            initialModeName = "GUIDED"
        else:
            originalSensorPathParameters = prepareSensorOnlyArming(
                pixhawkConnection,
                latestMessages,
                messageArrivedAtSeconds,
                distanceSensors,
            )
            initialModeNumber = GUIDED_NOGPS_MODE_NUMBER
            initialModeName = "GUIDED_NOGPS"
        if not setFlightModeAndWait(
            pixhawkConnection,
            latestMessages,
            messageArrivedAtSeconds,
            distanceSensors,
            initialModeNumber,
            initialModeName,
        ):
            raise RuntimeError(f"{initialModeName} was not confirmed.")
        if not armAndWaitForConfirmation(
            pixhawkConnection,
            latestMessages,
            messageArrivedAtSeconds,
            distanceSensors,
        ):
            return
        aircraftWasArmed = True

        if gpsPositionHoldWillBeUsed:
            runGpsHover(
                pixhawkConnection,
                latestMessages,
                messageArrivedAtSeconds,
                distanceSensors,
                barometerGroundReferenceMeters,
                altitudeManager,
            )
        else:
            runSensorOnlyHover(
                pixhawkConnection,
                latestMessages,
                messageArrivedAtSeconds,
                distanceSensors,
                barometerGroundReferenceMeters,
                altitudeManager,
                maximumClimbSpeedMetersPerSecond,
                maximumDescentSpeedMetersPerSecond,
            )

        commandLand(pixhawkConnection, latestMessages, messageArrivedAtSeconds, distanceSensors)
        waitForLandingToFinish(
            pixhawkConnection,
            latestMessages,
            messageArrivedAtSeconds,
            distanceSensors,
            barometerGroundReferenceMeters,
            altitudeManager,
        )
    except KeyboardInterrupt:
        print()
        print("Ctrl+C.")
        if pixhawkConnection is not None and aircraftWasArmed and heartbeatIsFresh():
            try:
                commandLand(pixhawkConnection, latestMessages, messageArrivedAtSeconds, distanceSensors)
                if altitudeManager is not None:
                    waitForLandingToFinish(
                        pixhawkConnection,
                        latestMessages,
                        messageArrivedAtSeconds,
                        distanceSensors,
                        barometerGroundReferenceMeters,
                        altitudeManager,
                    )
            except KeyboardInterrupt:
                print("Second Ctrl+C. Closing the link. The copter is not force-disarmed.")
    except Exception as error:
        print()
        print(f"STOPPED: {error}")
        if pixhawkConnection is not None and aircraftWasArmed and heartbeatIsFresh():
            try:
                commandLand(pixhawkConnection, latestMessages, messageArrivedAtSeconds, distanceSensors)
                if altitudeManager is not None:
                    waitForLandingToFinish(
                        pixhawkConnection,
                        latestMessages,
                        messageArrivedAtSeconds,
                        distanceSensors,
                        barometerGroundReferenceMeters,
                        altitudeManager,
                    )
            except Exception as landingError:
                print(f"Landing error: {landingError}")
    finally:
        if (
            pixhawkConnection is not None
            and originalSensorPathParameters is not None
            and heartbeatIsFresh()
        ):
            try:
                restoreSensorOnlyArming(
                    pixhawkConnection,
                    latestMessages,
                    messageArrivedAtSeconds,
                    distanceSensors,
                    originalSensorPathParameters,
                )
            except Exception as restoreError:
                print(f"Could not restore the GPS arming checks: {restoreError}")
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
                    distanceSensors,
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


if __name__ == "__main__":
    main()
