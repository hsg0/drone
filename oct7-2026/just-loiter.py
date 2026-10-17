#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    Connect to the Pixhawk through the Raspberry Pi GPIO UART.

    The pilot arms the copter manually.

    After the copter is armed, this script:
        1. Switches to GUIDED mode.
        2. Commands a firm, controlled takeoff to 4 feet.
        3. Switches to LOITER after the takeoff altitude is reached.
        4. Holds LOITER for 60 seconds.
        5. Commands LAND.

WHY:
    A very slow liftoff can leave the copter touching the ground long enough
    for one side to rise before the other. The frame can tip, and a propeller
    can strike the ground.

    ArduPilot still flies the takeoff. This script only shortens TKOFF_SLEW_TIME
    so the throttle ramp is firmer. It does not send raw motor commands.

HOW:
    1. Open /dev/serial0. Leave the USB Pixhawk port alone.
    2. Ask the UART for GPS, relative altitude, and the position estimate.
    3. Wait until mode, arm state, GPS, and the position estimate have arrived.
    4. Stop if those checks are not healthy.
    5. Wait until the pilot types TAKEOFF.
    6. Remember TKOFF_SLEW_TIME, then set the firmer ramp.
    7. Wait until the pilot arms the copter on the transmitter.
    8. Switch to GUIDED and command takeoff to 4 feet.
    9. Switch to LOITER once the climb is high enough.
    10. Hold LOITER for 60 seconds. GPS holds the position.
    11. If the sensors are answering, print the sonic distance from the power
        board and the time-of-flight distance from under the drone.
        LOITER does not depend on either reading.
    12. Command LAND.
    13. Put the original TKOFF_SLEW_TIME back.

IMPORTANT:
    Run this on the Pi with the dronekit-venv Python named above.
    Props-off testing comes first after a flight-script change.

    This script commands takeoff. The pilot arms the copter. This script
    never sends an arm command.

    /dev/serial0 is the GPIO UART, the same command cable as the other
    pin 8 and pin 10 scripts:
        physical pin 8  transmits to the Pixhawk
        physical pin 10 receives from the Pixhawk
        physical pin 6  is ground
    The baud rate is 57600, matching that cable.

    bootDisplay.py already owns the Pixhawk USB port. This script does not
    open USB.

    GPS and a healthy position estimate are required because the script
    enters LOITER after takeoff. The command UART does not stream those
    messages until they are requested.

    The brown-box camera scale and the sonic reading are optional help.
    If the saved calibration is missing, the camera cannot see the box, or
    the time-of-flight sensor is quiet, LOITER still holds on GPS.

    TAKEOFF_SLEW_TIME_SECONDS is how quickly ArduPilot ramps throttle during
    its own takeoff. Smaller is firmer. 0.75 seconds is the firm starting
    point. The Pixhawk still controls roll, pitch, yaw, and altitude.

    The original TKOFF_SLEW_TIME is restored before the script exits.

    Ctrl+C commands LAND while the heartbeat is still fresh.
    A second Ctrl+C closes the link. This script never force-disarms.
"""

import os
import signal
import sys
import time

from dronekit import VehicleMode, connect
from pymavlink import mavutil

scriptDirectory = os.path.dirname(os.path.abspath(__file__))
if scriptDirectory not in sys.path:
    sys.path.insert(0, scriptDirectory)
import forward_distance_calibration


# ---------------------------------------------------------------------------
# Connection settings
# ---------------------------------------------------------------------------

# GPIO UART. Pin 8 transmits, pin 10 receives, pin 6 is ground.
# USB stays with bootDisplay.py.
PIXHAWK_UART_DEVICE = "/dev/serial0"

# The pin 8/10 scripts on this Pi already use 57600.
PIXHAWK_UART_BAUD = 57600

CONNECTION_TIMEOUT_SECONDS = 30
TELEMETRY_WAIT_TIMEOUT_SECONDS = 20
POLL_INTERVAL_SECONDS = 0.2
STATUS_PRINT_INTERVAL_SECONDS = 1.0

# ArduPilot on this UART ignores DroneKit's old "send everything" request.
# Each message the flight decisions need is requested on its own.
MESSAGE_INTERVAL_SECONDS = 0.25
MICROSECONDS_PER_SECOND = 1000000
FLIGHT_MESSAGE_NAMES = (
    "GPS_RAW_INT",
    "GLOBAL_POSITION_INT",
    "EKF_STATUS_REPORT",
)

# Filled when a heartbeat is observed. DroneKit's last_heartbeat number is
# captured inside the callback, so it does not grow while the link is quiet.
lastHeartbeatReceivedAtSeconds = None


# ---------------------------------------------------------------------------
# Flight settings
# ---------------------------------------------------------------------------

GUIDED_MODE_NAME = "GUIDED"
LOITER_MODE_NAME = "LOITER"
LAND_MODE_NAME = "LAND"

TAKEOFF_CONFIRMATION_WORD = "TAKEOFF"

METERS_PER_FOOT = 0.3048
TAKEOFF_ALTITUDE_FEET = 4.0
TAKEOFF_ALTITUDE_METERS = TAKEOFF_ALTITUDE_FEET * METERS_PER_FOOT

# Loiter begins a little before the exact target so the last few centimeters
# of the climb cannot hold the script forever.
TAKEOFF_ALTITUDE_REACHED_FRACTION = 0.90

TAKEOFF_TIMEOUT_SECONDS = 20
MANUAL_ARM_TIMEOUT_SECONDS = 20
LOITER_HOLD_TIME_SECONDS = 60
MODE_CHANGE_TIMEOUT_SECONDS = 15
HEARTBEAT_STALE_AFTER_SECONDS = 3


# ---------------------------------------------------------------------------
# Takeoff behaviour
# ---------------------------------------------------------------------------

# ArduPilot parameter. Smaller means the takeoff throttle rises faster.
TAKEOFF_SLEW_PARAMETER_NAME = "TKOFF_SLEW_TIME"
TAKEOFF_SLEW_TIME_SECONDS = 0.75

# The command cable is 57600, so a parameter reply can take a few seconds.
PARAMETER_CHANGE_TIMEOUT_SECONDS = 8
PARAMETER_MATCH_TOLERANCE = 0.01


# ---------------------------------------------------------------------------
# GPS requirements
# ---------------------------------------------------------------------------

# MAVLink fix_type: 3 is a 3D fix. 2D is not enough to hold a position.
MINIMUM_GPS_FIX_TYPE_FOR_FLIGHT = 3
MINIMUM_SATELLITE_COUNT_FOR_FLIGHT = 6

# GPS_RAW_INT uses 255 when the receiver does not report a satellite count.
UNKNOWN_SATELLITE_COUNT = 255

# One poor GPS update should not immediately end the hold.
GPS_LOSS_BEFORE_LAND_SECONDS = 3


# ---------------------------------------------------------------------------
# Signal handling
# ---------------------------------------------------------------------------

def raiseKeyboardInterrupt(signalNumber, stackFrame):
    """A dropped SSH session takes the same path as Ctrl+C."""
    raise KeyboardInterrupt


def noteHeartbeatReceived(vehicle, attributeName, heartbeatAgeSeconds):
    global lastHeartbeatReceivedAtSeconds
    lastHeartbeatReceivedAtSeconds = time.monotonic()


# ---------------------------------------------------------------------------
# GPS helpers
# ---------------------------------------------------------------------------

def describeGpsFix(gpsFixType):
    gpsFixNames = {
        0: "no GPS",
        1: "no fix",
        2: "2D fix",
        3: "3D fix",
        4: "DGPS fix",
        5: "RTK float",
        6: "RTK fixed",
    }
    if gpsFixType is None:
        return "missing"
    return gpsFixNames.get(gpsFixType, f"fix type {gpsFixType}")


def gpsIsGoodEnoughForFlight(vehicle):
    gpsReading = vehicle.gps_0
    if gpsReading is None or gpsReading.fix_type is None:
        return False
    if gpsReading.fix_type < MINIMUM_GPS_FIX_TYPE_FOR_FLIGHT:
        return False

    satelliteCount = gpsReading.satellites_visible
    if satelliteCount is None or satelliteCount == UNKNOWN_SATELLITE_COUNT:
        return False
    return satelliteCount >= MINIMUM_SATELLITE_COUNT_FOR_FLIGHT


def metersToFeet(distanceMeters):
    return distanceMeters / METERS_PER_FOOT


# ---------------------------------------------------------------------------
# Heartbeat and status
# ---------------------------------------------------------------------------

def heartbeatIsFresh(vehicle):
    """True when a heartbeat arrived recently enough to trust the copter state."""
    if lastHeartbeatReceivedAtSeconds is not None:
        heartbeatAgeSeconds = time.monotonic() - lastHeartbeatReceivedAtSeconds
        return heartbeatAgeSeconds <= HEARTBEAT_STALE_AFTER_SECONDS

    # The listener has not run yet. Use DroneKit's age from the connect heartbeat.
    heartbeatAgeSeconds = vehicle.last_heartbeat
    if heartbeatAgeSeconds is None:
        return False
    return heartbeatAgeSeconds <= HEARTBEAT_STALE_AFTER_SECONDS


def copterStateLine(vehicle):
    if vehicle.armed is True:
        armedText = "yes"
    elif vehicle.armed is False:
        armedText = "no"
    else:
        armedText = "unknown"

    if vehicle.mode is not None and vehicle.mode.name:
        flightModeName = vehicle.mode.name
    else:
        flightModeName = "unknown"

    gpsReading = vehicle.gps_0
    if gpsReading is None:
        gpsText = "GPS missing"
    else:
        gpsText = (
            f"GPS {describeGpsFix(gpsReading.fix_type)}, "
            f"{gpsReading.satellites_visible} satellites"
        )

    relativeAltitudeMeters = vehicle.location.global_relative_frame.alt
    if relativeAltitudeMeters is None:
        altitudeText = "relative altitude missing"
    else:
        altitudeText = (
            f"relative altitude {relativeAltitudeMeters:.2f} m "
            f"({metersToFeet(relativeAltitudeMeters):.1f} ft)"
        )

    return (
        f"Mode: {flightModeName} | Armed: {armedText} | "
        f"{gpsText} | {altitudeText}"
    )


# ---------------------------------------------------------------------------
# Pixhawk connection
# ---------------------------------------------------------------------------

def pointLinkAtAutopilot(vehicle):
    """Commands need the autopilot component. A heartbeat can report component 0."""
    pixhawkLink = vehicle._master
    if pixhawkLink.target_component == 0:
        pixhawkLink.target_component = 1
    return pixhawkLink


def requestFlightMessages(vehicle):
    """Ask the command UART for the messages this flight actually uses."""
    pixhawkLink = pointLinkAtAutopilot(vehicle)
    intervalMicroseconds = int(MESSAGE_INTERVAL_SECONDS * MICROSECONDS_PER_SECOND)

    for messageName in FLIGHT_MESSAGE_NAMES:
        messageId = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{messageName}", None)
        if messageId is None:
            print(f"No MAVLink id for {messageName}.")
            continue
        pixhawkLink.mav.command_long_send(
            pixhawkLink.target_system,
            pixhawkLink.target_component,
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

    print(
        "Requested GPS, relative altitude, and the position estimate "
        f"every {MESSAGE_INTERVAL_SECONDS:.2f} seconds."
    )


def flightTelemetryIsReady(vehicle):
    flightModeIsKnown = vehicle.mode is not None and bool(vehicle.mode.name)
    if not flightModeIsKnown or vehicle.armed is None:
        return False
    if not gpsIsGoodEnoughForFlight(vehicle):
        return False
    # ekf_ok is DroneKit's summary of ArduPilot's position estimate.
    return vehicle.ekf_ok is True


def waitForFlightTelemetry(vehicle):
    print("Waiting for mode, arm state, GPS, and a healthy position estimate.")
    startedAtSeconds = time.monotonic()
    lastPrintedAtSeconds = 0

    while time.monotonic() - startedAtSeconds < TELEMETRY_WAIT_TIMEOUT_SECONDS:
        if flightTelemetryIsReady(vehicle):
            return True

        nowSeconds = time.monotonic()
        if nowSeconds - lastPrintedAtSeconds >= STATUS_PRINT_INTERVAL_SECONDS:
            print(copterStateLine(vehicle))
            lastPrintedAtSeconds = nowSeconds

        time.sleep(POLL_INTERVAL_SECONDS)

    print("Mode, arm state, GPS, or the position estimate was not ready in time.")
    print(copterStateLine(vehicle))
    return False


def connectToPixhawk():
    if not os.path.exists(PIXHAWK_UART_DEVICE):
        print(f"{PIXHAWK_UART_DEVICE} is not present.")
        print("This is the Pi GPIO UART: pin 8 transmit, pin 10 receive, pin 6 ground.")
        return None

    print(
        f"Connecting to Pixhawk on {PIXHAWK_UART_DEVICE} "
        f"at {PIXHAWK_UART_BAUD} baud."
    )
    vehicle = connect(
        PIXHAWK_UART_DEVICE,
        baud=PIXHAWK_UART_BAUD,
        wait_ready=False,
        timeout=CONNECTION_TIMEOUT_SECONDS,
        heartbeat_timeout=CONNECTION_TIMEOUT_SECONDS,
    )

    global lastHeartbeatReceivedAtSeconds
    lastHeartbeatReceivedAtSeconds = time.monotonic()
    vehicle.add_attribute_listener("last_heartbeat", noteHeartbeatReceived)

    requestFlightMessages(vehicle)

    if not waitForFlightTelemetry(vehicle):
        vehicle.close()
        return None

    print("Connected. Heartbeat, mode, arm state, GPS, and position estimate are ready.")
    return vehicle


# ---------------------------------------------------------------------------
# Flight readiness
# ---------------------------------------------------------------------------

def reasonFlightIsBlocked(vehicle):
    """Return a sentence when takeoff must not start. None means the checks passed."""
    if not heartbeatIsFresh(vehicle):
        return "The Pixhawk heartbeat is missing or stale, so the copter state is not trusted."

    if vehicle.armed is None:
        return "The script cannot tell whether the copter is armed."

    if not gpsIsGoodEnoughForFlight(vehicle):
        gpsReading = vehicle.gps_0
        if gpsReading is None:
            return "No GPS reading has arrived on the command cable."
        return (
            f"GPS fix is {describeGpsFix(gpsReading.fix_type)} "
            f"with {gpsReading.satellites_visible} satellites. "
            "This flight needs a 3D fix and at least "
            f"{MINIMUM_SATELLITE_COUNT_FOR_FLIGHT} satellites."
        )

    if vehicle.ekf_ok is not True:
        return (
            "The position estimate is not healthy. "
            "GUIDED takeoff and LOITER were not requested."
        )

    if vehicle.is_armable is not True and vehicle.armed is not True:
        return "The Pixhawk reports that the copter is not armable yet."

    return None


# ---------------------------------------------------------------------------
# Flight mode
# ---------------------------------------------------------------------------

def setFlightModeAndWaitUntilConfirmed(vehicle, requestedFlightModeName):
    print(f"Requesting {requestedFlightModeName}.")
    vehicle.mode = VehicleMode(requestedFlightModeName)
    startedAtSeconds = time.monotonic()

    while True:
        currentFlightModeName = None
        if vehicle.mode is not None:
            currentFlightModeName = vehicle.mode.name

        if currentFlightModeName == requestedFlightModeName:
            print(f"{requestedFlightModeName} confirmed.")
            return True

        if time.monotonic() - startedAtSeconds > MODE_CHANGE_TIMEOUT_SECONDS:
            print(
                f"Mode stayed {currentFlightModeName}. "
                f"{requestedFlightModeName} was not confirmed."
            )
            return False

        time.sleep(POLL_INTERVAL_SECONDS)


# ---------------------------------------------------------------------------
# Manual arming
# ---------------------------------------------------------------------------

def waitForPilotToArm(vehicle):
    """Wait for the transmitter. This function never sends an arm command."""
    if vehicle.armed is True:
        print("The copter is already armed.")
        return True

    print()
    print("ARM THE COPTER NOW")
    print("Use the transmitter to arm.")
    print(f"Waiting up to {MANUAL_ARM_TIMEOUT_SECONDS:.0f} seconds.")

    startedAtSeconds = time.monotonic()
    while time.monotonic() - startedAtSeconds < MANUAL_ARM_TIMEOUT_SECONDS:
        if not heartbeatIsFresh(vehicle):
            print("Heartbeat lost while waiting for arming.")
            return False
        if vehicle.armed is True:
            print("Armed confirmed.")
            return True
        time.sleep(POLL_INTERVAL_SECONDS)

    print("The copter was not armed before the timeout.")
    return False


# ---------------------------------------------------------------------------
# Takeoff parameter
# ---------------------------------------------------------------------------

def readPixhawkParameter(vehicle, parameterName):
    try:
        parameterValue = vehicle.parameters[parameterName]
    except Exception as error:
        print(f"Could not read {parameterName}: {error}")
        return None

    if parameterValue is None:
        return None
    return float(parameterValue)


def setPixhawkParameterAndConfirm(vehicle, parameterName, requestedValue):
    try:
        vehicle.parameters[parameterName] = requestedValue
    except Exception as error:
        print(f"Could not set {parameterName}: {error}")
        return False

    startedAtSeconds = time.monotonic()
    while time.monotonic() - startedAtSeconds < PARAMETER_CHANGE_TIMEOUT_SECONDS:
        currentValue = readPixhawkParameter(vehicle, parameterName)
        if currentValue is not None and abs(currentValue - requestedValue) < PARAMETER_MATCH_TOLERANCE:
            return True
        time.sleep(POLL_INTERVAL_SECONDS)

    print(f"{parameterName} did not confirm the requested value {requestedValue}.")
    return False


def prepareFirmTakeoff(vehicle):
    """
    Set the firmer takeoff ramp.

    Returns the original TKOFF_SLEW_TIME when it must be restored later.
    Returns None when the parameter was never changed.
    """
    print(
        f"Reading {TAKEOFF_SLEW_PARAMETER_NAME}. "
        "The parameter list comes down the command cable, so this can take a little while."
    )
    originalTakeoffSlewTimeSeconds = readPixhawkParameter(
        vehicle,
        TAKEOFF_SLEW_PARAMETER_NAME,
    )
    if originalTakeoffSlewTimeSeconds is None:
        print(f"{TAKEOFF_SLEW_PARAMETER_NAME} was not available.")
        print("The Pixhawk's existing takeoff ramp will be used.")
        return None

    print(
        f"Original {TAKEOFF_SLEW_PARAMETER_NAME}: "
        f"{originalTakeoffSlewTimeSeconds:.2f} seconds."
    )
    print(
        f"Temporary {TAKEOFF_SLEW_PARAMETER_NAME}: "
        f"{TAKEOFF_SLEW_TIME_SECONDS:.2f} seconds."
    )

    parameterChanged = setPixhawkParameterAndConfirm(
        vehicle,
        TAKEOFF_SLEW_PARAMETER_NAME,
        TAKEOFF_SLEW_TIME_SECONDS,
    )
    if not parameterChanged:
        print("The firmer takeoff setting could not be confirmed.")
        # A write may still have reached the Pixhawk. The caller restores it.
        return originalTakeoffSlewTimeSeconds

    return originalTakeoffSlewTimeSeconds


def restoreOriginalTakeoffSetting(vehicle, originalTakeoffSlewTimeSeconds):
    print(
        f"Restoring {TAKEOFF_SLEW_PARAMETER_NAME} "
        f"to {originalTakeoffSlewTimeSeconds:.2f} seconds."
    )
    restored = setPixhawkParameterAndConfirm(
        vehicle,
        TAKEOFF_SLEW_PARAMETER_NAME,
        originalTakeoffSlewTimeSeconds,
    )
    if restored:
        print(f"{TAKEOFF_SLEW_PARAMETER_NAME} restored.")
    else:
        print(f"Warning: {TAKEOFF_SLEW_PARAMETER_NAME} could not be confirmed as restored.")


# ---------------------------------------------------------------------------
# Takeoff, loiter, and landing
# ---------------------------------------------------------------------------

def commandTakeoffAndWaitForAltitude(vehicle):
    """ArduPilot flies the climb. This function watches the relative altitude."""
    minimumAcceptedAltitudeMeters = (
        TAKEOFF_ALTITUDE_METERS * TAKEOFF_ALTITUDE_REACHED_FRACTION
    )

    print()
    print("TAKEOFF")
    print(
        f"Target altitude: {TAKEOFF_ALTITUDE_METERS:.2f} m "
        f"({TAKEOFF_ALTITUDE_FEET:.1f} ft)."
    )
    print(f"Takeoff ramp: {TAKEOFF_SLEW_TIME_SECONDS:.2f} seconds.")
    print("Sending the ArduPilot takeoff command.")

    vehicle.simple_takeoff(TAKEOFF_ALTITUDE_METERS)

    startedAtSeconds = time.monotonic()
    lastPrintedAtSeconds = 0

    while True:
        nowSeconds = time.monotonic()

        if not heartbeatIsFresh(vehicle):
            print("Heartbeat lost during takeoff.")
            return False

        if vehicle.armed is not True:
            print("The copter disarmed during takeoff.")
            return False

        if not gpsIsGoodEnoughForFlight(vehicle):
            print("GPS became unhealthy during takeoff.")
            return False

        relativeAltitudeMeters = vehicle.location.global_relative_frame.alt
        if relativeAltitudeMeters is not None:
            if nowSeconds - lastPrintedAtSeconds >= STATUS_PRINT_INTERVAL_SECONDS:
                print(
                    f"Takeoff | {relativeAltitudeMeters:.2f} m | "
                    f"{metersToFeet(relativeAltitudeMeters):.1f} ft"
                )
                lastPrintedAtSeconds = nowSeconds

            if relativeAltitudeMeters >= minimumAcceptedAltitudeMeters:
                print("Takeoff altitude reached.")
                return True

        if nowSeconds - startedAtSeconds >= TAKEOFF_TIMEOUT_SECONDS:
            print("The copter did not reach the takeoff altitude in time.")
            return False

        time.sleep(POLL_INTERVAL_SECONDS)


def prepareOptionalForwardSensors():
    """Load the saved camera scale and open the camera when both are available."""
    calibration = forward_distance_calibration.loadForwardDistanceCalibration()
    if calibration is None:
        print(forward_distance_calibration.savedCalibrationRefusalReason())
        print("LOITER will use GPS without the forward-sensor helper.")
        return None, None

    print(
        f"Saved camera scale {float(calibration['cameraScale']):.3f} is available. "
        "It can describe the brown box. LOITER does not require it."
    )
    try:
        camera = forward_distance_calibration.openForwardCamera()
    except Exception as error:
        print(f"The forward camera did not open: {error}")
        print("LOITER will use GPS without the camera helper.")
        return None, calibration
    return camera, calibration


def forwardSensorHelpLine(camera, calibration):
    """
    Describe the power-board sonic sensor and the bottom time-of-flight sensor.
    A missing reading does not change the LOITER decision.
    """
    snapshot = forward_distance_calibration.readForwardRangeSnapshot()
    sonicMeters = forward_distance_calibration.sonicDistanceMeters(snapshot)
    timeOfFlightMeters = forward_distance_calibration.timeOfFlightDistanceMeters(snapshot)
    cameraMeters = None
    if camera is not None and calibration is not None:
        try:
            rawCameraMeters, markerCorners, markerNearEdge, _sawBlueWithoutBrownBox = (
                forward_distance_calibration.lookAtMarker(camera)
            )
            if markerCorners is not None and not markerNearEdge:
                cameraMeters = forward_distance_calibration.scaledCameraDistanceMeters(
                    rawCameraMeters,
                    calibration,
                )
        except Exception:
            cameraMeters = None

    if sonicMeters is None and cameraMeters is None and timeOfFlightMeters is None:
        return "Range sensors quiet. LOITER is holding on GPS."

    parts = []
    if sonicMeters is not None:
        parts.append(f"sonic, power board, forward {sonicMeters:.2f} m")
    if cameraMeters is not None:
        parts.append(f"camera {cameraMeters:.2f} m")
    if timeOfFlightMeters is not None:
        parts.append(f"time of flight, bottom, forward and backward {timeOfFlightMeters:.2f} m")
    return "Range helper: " + " | ".join(parts) + ". GPS still holds LOITER."


def watchLoiterUntilItIsTimeToLand(vehicle, camera, calibration):
    startedAtSeconds = time.monotonic()
    lastPrintedAtSeconds = 0
    gpsWentBadAtSeconds = None

    while True:
        nowSeconds = time.monotonic()

        if not heartbeatIsFresh(vehicle):
            return "Heartbeat stopped. No further command was sent."

        if vehicle.armed is not True:
            return "The copter is no longer armed."

        if gpsIsGoodEnoughForFlight(vehicle):
            gpsWentBadAtSeconds = None
        else:
            if gpsWentBadAtSeconds is None:
                gpsWentBadAtSeconds = nowSeconds
                print("GPS quality dropped. Waiting briefly in case it recovers.")
            if nowSeconds - gpsWentBadAtSeconds >= GPS_LOSS_BEFORE_LAND_SECONDS:
                return "GPS stayed too poor for LOITER."

        if nowSeconds - startedAtSeconds >= LOITER_HOLD_TIME_SECONDS:
            return f"LOITER hold of {LOITER_HOLD_TIME_SECONDS:.0f} seconds is complete."

        if nowSeconds - lastPrintedAtSeconds >= STATUS_PRINT_INTERVAL_SECONDS:
            print(copterStateLine(vehicle))
            print(forwardSensorHelpLine(camera, calibration))
            lastPrintedAtSeconds = nowSeconds

        time.sleep(POLL_INTERVAL_SECONDS)


def commandLandIfConnectionIsHealthy(vehicle):
    if not heartbeatIsFresh(vehicle):
        print("Heartbeat is stale. LAND was not sent.")
        return False

    print()
    print("LAND")
    try:
        return setFlightModeAndWaitUntilConfirmed(vehicle, LAND_MODE_NAME)
    except KeyboardInterrupt:
        print("Ctrl+C during LAND. Closing the link. The copter is not force-disarmed.")
        return False


def landBecauseTheFlightHadStarted(vehicle, guidedModeConfirmed, takeoffWasCommanded):
    if vehicle is None or not heartbeatIsFresh(vehicle):
        print("Heartbeat is stale. LAND was not sent.")
        return
    if vehicle.armed is True or takeoffWasCommanded or guidedModeConfirmed:
        print("Requesting LAND.")
        commandLandIfConnectionIsHealthy(vehicle)


# ---------------------------------------------------------------------------
# Main flight logic
# ---------------------------------------------------------------------------

def main():
    signal.signal(signal.SIGHUP, raiseKeyboardInterrupt)

    vehicle = None
    forwardCamera = None
    originalTakeoffSlewTimeSeconds = None
    firmTakeoffWasAttempted = False
    guidedModeConfirmed = False
    takeoffWasCommanded = False

    try:
        print()
        print("4 FOOT FIRM TAKEOFF + LOITER")
        print()

        vehicle = connectToPixhawk()
        if vehicle is None:
            return

        print(copterStateLine(vehicle))
        forwardCamera, forwardCalibration = prepareOptionalForwardSensors()
        blockedReason = reasonFlightIsBlocked(vehicle)
        if blockedReason is not None:
            print(blockedReason)
            print("Flight mode was left unchanged.")
            return

        print("Basic flight checks passed.")
        print()
        print("This script will command the copter to take off.")
        print(f"It will climb to {TAKEOFF_ALTITUDE_FEET:.1f} feet, loiter, then land.")
        print(f"Type {TAKEOFF_CONFIRMATION_WORD} to continue.")
        pilotConfirmation = input("> ").strip()
        if pilotConfirmation != TAKEOFF_CONFIRMATION_WORD:
            print("Cancelled.")
            return

        originalTakeoffSlewTimeSeconds = prepareFirmTakeoff(vehicle)
        if originalTakeoffSlewTimeSeconds is not None:
            firmTakeoffWasAttempted = True
            currentSlewTimeSeconds = readPixhawkParameter(vehicle, TAKEOFF_SLEW_PARAMETER_NAME)
            slewTimeIsFirm = (
                currentSlewTimeSeconds is not None
                and abs(currentSlewTimeSeconds - TAKEOFF_SLEW_TIME_SECONDS) < PARAMETER_MATCH_TOLERANCE
            )
            if not slewTimeIsFirm:
                print("Stopping before arming. The takeoff ramp was not confirmed.")
                return

        if not waitForPilotToArm(vehicle):
            return

        print("The copter is armed. Switching to GUIDED for the takeoff.")
        guidedModeConfirmed = setFlightModeAndWaitUntilConfirmed(vehicle, GUIDED_MODE_NAME)
        if not guidedModeConfirmed:
            print("GUIDED was not confirmed. Takeoff was not commanded.")
            return

        if vehicle.armed is not True:
            print("The copter is no longer armed. Takeoff was not commanded.")
            return

        if not heartbeatIsFresh(vehicle) or not gpsIsGoodEnoughForFlight(vehicle):
            print("GPS or the heartbeat failed just before takeoff. Takeoff was not commanded.")
            return

        takeoffWasCommanded = True
        takeoffSucceeded = commandTakeoffAndWaitForAltitude(vehicle)
        if not takeoffSucceeded:
            print("Takeoff did not finish normally.")
            commandLandIfConnectionIsHealthy(vehicle)
            return

        print("Switching from GUIDED takeoff to LOITER.")
        loiterModeConfirmed = setFlightModeAndWaitUntilConfirmed(vehicle, LOITER_MODE_NAME)
        if not loiterModeConfirmed:
            print("LOITER was not confirmed. Landing.")
            commandLandIfConnectionIsHealthy(vehicle)
            return

        print()
        print("LOITER")
        print(f"Holding position for {LOITER_HOLD_TIME_SECONDS:.0f} seconds. Ctrl+C commands LAND.")
        reasonToLand = watchLoiterUntilItIsTimeToLand(vehicle, forwardCamera, forwardCalibration)
        print(reasonToLand)

        if reasonToLand.startswith("Heartbeat stopped"):
            return

        commandLandIfConnectionIsHealthy(vehicle)

    except KeyboardInterrupt:
        print()
        print("Ctrl+C.")
        if vehicle is not None:
            commandLandIfConnectionIsHealthy(vehicle)

    except Exception as error:
        print()
        print(f"Stopped because of an error: {error}")
        landBecauseTheFlightHadStarted(vehicle, guidedModeConfirmed, takeoffWasCommanded)

    finally:
        if forwardCamera is not None:
            try:
                forwardCamera.stop()
            except Exception:
                pass
            print("Forward camera closed.")
        if vehicle is not None and firmTakeoffWasAttempted and originalTakeoffSlewTimeSeconds is not None:
            try:
                restoreOriginalTakeoffSetting(vehicle, originalTakeoffSlewTimeSeconds)
            except KeyboardInterrupt:
                print("Ctrl+C during parameter restore. Closing the link.")
            except Exception as error:
                print(f"Could not restore {TAKEOFF_SLEW_PARAMETER_NAME}: {error}")

        if vehicle is not None:
            try:
                vehicle.close()
            except Exception:
                pass
            print("Pixhawk link closed.")

        print("Finished.")


if __name__ == "__main__":
    main()
