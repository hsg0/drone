#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    1. Take off to about 3 feet.
    2. Hover for 10 seconds.
    3. Fly forward in the North direction for 10 feet.
    4. Hover for 10 seconds.
    5. Land safely.

WHY:
    GUIDED handles takeoff and the North velocity move.
    LOITER holds position during both 10-second hovers.
    LAND brings the copter down and disarms the motors.

HOW:
    1. Connect to the Pixhawk on UART pins 8 and 10.
    2. Wait for Quad X frame, 3D GPS fix, 6 satellites, trusted position, and a level frame.
    3. Ask the pilot to type YES.
    4. Set the onboard ground-station failsafe to Land.
    5. Arm in GUIDED and climb to 3 feet.
    6. Hold LOITER for 10 seconds.
    7. Move North 10 feet in GUIDED at 0.4 m/s, then command zero speed.
    8. Hold LOITER for 10 seconds.
    9. Switch to LAND and wait until the motors disarm.

IMPORTANT:
    Run on the Pi with /home/drone/dronekit-venv/bin/python3.
    /dev/serial0 uses pin 8 (TX), pin 10 (RX), and pin 6 (ground) at 57600 baud.
    Leave USB ports untouched for bootDisplay.py.
    Airframe is a DJI F450 configured as Quad X (FRAME_CLASS 1, FRAME_TYPE 1).
    Refuse FRAME_TYPE 13 (DJI X) because its motor order does not match this wiring.
    Reaching 4 feet or 1.3 meters starts LAND immediately.
    Drifting more than 8 feet off the North line starts LAND immediately.
    Never force-arm and never force-disarm.
    If the heartbeat is lost, close the UART cable so the Pixhawk failsafe lands the copter.
"""

import math
import os
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

FEET_PER_METER = 3.28084
HOVER_ALTITUDE_FEET = 3.0
HOVER_ALTITUDE_METERS = HOVER_ALTITUDE_FEET / FEET_PER_METER
TAKEOFF_REACHED_FRACTION = 0.95

# 4 feet (1.22 m) is reached before 1.3 m; both ceilings are enforced.
MAXIMUM_HOVER_ALTITUDE_FEET = 4.0
MAXIMUM_HOVER_ALTITUDE_METERS = 1.3
FOUR_FOOT_CEILING_METERS = MAXIMUM_HOVER_ALTITUDE_FEET / FEET_PER_METER

HOVER_DURATION_SECONDS = 10.0

# True North move in GUIDED mode.
NORTH_DISTANCE_FEET = 10.0
NORTH_DISTANCE_METERS = NORTH_DISTANCE_FEET / FEET_PER_METER
NORTH_SPEED_METERS_PER_SECOND = 0.4
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


def hover(vehicle, hoverName):
    """Step 2 and Step 4: Hold position in LOITER for 10 seconds."""
    setFlightMode(vehicle, "LOITER")
    print(f"{hoverName} started.")
    startTime = time.monotonic()
    lastPrintedAtSeconds = 0.0
    while True:
        elapsedSeconds = time.monotonic() - startTime
        if elapsedSeconds >= HOVER_DURATION_SECONDS:
            break
        if vehicle.mode.name != "LOITER":
            raise RuntimeError("LOITER mode lost.")
        altitudeMeters = flightStillHealthy(vehicle)

        currentTimeSeconds = time.monotonic()
        if currentTimeSeconds - lastPrintedAtSeconds >= PRINT_INTERVAL_SECONDS:
            remainingSeconds = max(0.0, HOVER_DURATION_SECONDS - elapsedSeconds)
            print(f"{hoverName} | {heightText(altitudeMeters)} | remaining {remainingSeconds:.0f} s")
            lastPrintedAtSeconds = currentTimeSeconds
        time.sleep(STATUS_CHECK_INTERVAL_SECONDS)
    print(f"{hoverName} completed.")


def flyNorth(vehicle):
    """Step 3: Move forward in the North direction for 10 feet in GUIDED, then stop."""
    setFlightMode(vehicle, "GUIDED")

    startSpot = latitudeAndLongitude(vehicle)
    if startSpot is None:
        raise RuntimeError("GPS position was not available for the North move.")
    startLatitudeDegrees, startLongitudeDegrees = startSpot

    print(
        f"Flying North {NORTH_DISTANCE_FEET:.0f} ft "
        f"at {NORTH_SPEED_METERS_PER_SECOND:.1f} m/s."
    )
    startedAtSeconds = time.monotonic()
    lastPrintedAtSeconds = 0.0
    try:
        while True:
            if vehicle.mode.name != "GUIDED":
                raise RuntimeError("GUIDED mode lost during the North move.")
            altitudeMeters = flightStillHealthy(vehicle)

            currentSpot = latitudeAndLongitude(vehicle)
            if currentSpot is None:
                raise RuntimeError("GPS position was lost during the North move.")
            northMeters, eastMeters = northAndEastOffsetMeters(
                startLatitudeDegrees,
                startLongitudeDegrees,
                currentSpot[0],
                currentSpot[1],
            )
            if northMeters >= NORTH_DISTANCE_METERS:
                print(
                    f"10 feet North reached | "
                    f"{northMeters * FEET_PER_METER:.1f} ft North | "
                    f"{eastMeters * FEET_PER_METER:.1f} ft East."
                )
                break

            # Stop if the copter drifts more than 8 feet East/West or South.
            leftTheNorthLine = (
                abs(eastMeters) > MAXIMUM_OFF_COURSE_METERS
                or northMeters < -MAXIMUM_OFF_COURSE_METERS
            )
            if leftTheNorthLine:
                raise RuntimeError(
                    "Left the North line. "
                    f"{northMeters * FEET_PER_METER:.1f} ft North, "
                    f"{eastMeters * FEET_PER_METER:.1f} ft East."
                )
            if time.monotonic() - startedAtSeconds > NORTH_MOVE_TIMEOUT_SECONDS:
                raise RuntimeError(
                    "Did not reach 10 feet North in time. "
                    f"{northMeters * FEET_PER_METER:.1f} ft North, "
                    f"{eastMeters * FEET_PER_METER:.1f} ft East."
                )

            sendLocalVelocity(vehicle, NORTH_SPEED_METERS_PER_SECOND, 0, 0)
            currentTimeSeconds = time.monotonic()
            if currentTimeSeconds - lastPrintedAtSeconds >= PRINT_INTERVAL_SECONDS:
                print(
                    f"Flying North | {northMeters * FEET_PER_METER:.1f} ft North | "
                    f"{eastMeters * FEET_PER_METER:.1f} ft East | "
                    f"{heightText(altitudeMeters)}"
                )
                lastPrintedAtSeconds = currentTimeSeconds
            time.sleep(STATUS_CHECK_INTERVAL_SECONDS)

        # Hold zero velocity for 2 seconds so the copter stops before switching to LOITER.
        settleStartedAtSeconds = time.monotonic()
        while time.monotonic() - settleStartedAtSeconds < SETTLE_AFTER_MOVE_SECONDS:
            if vehicle.mode.name != "GUIDED":
                raise RuntimeError("GUIDED mode lost while stopping.")
            flightStillHealthy(vehicle)
            sendLocalVelocity(vehicle, 0, 0, 0)
            time.sleep(STATUS_CHECK_INTERVAL_SECONDS)
        print("North move stopped.")
    finally:
        # Send zero speed on exit if the link is still up.
        if heartbeatIsFresh(vehicle) and not getattr(vehicle, "commandCableClosed", False):
            try:
                sendLocalVelocity(vehicle, 0, 0, 0)
            except Exception as stopError:
                print(f"Could not send the stop speed: {stopError}")


def land(vehicle):
    """Step 5: Enter LAND mode and wait until the motors disarm."""
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
    """Run preflight, take off to 3 ft, hover 10 s, fly North 10 ft, hover 10 s, and land."""
    vehicle = None
    originalTakeoffSlewSeconds = None
    savedFailsafe = None
    copterWasArmed = False
    landingWasStarted = False

    print("TAKE OFF, HOVER, FLY NORTH 10 FT, HOVER, LAND")
    print(f"Hover altitude: {HOVER_ALTITUDE_FEET:.0f} ft")
    print(f"Hover time: {HOVER_DURATION_SECONDS:.0f} seconds, twice")
    print(f"North leg: {NORTH_DISTANCE_FEET:.0f} ft at {NORTH_SPEED_METERS_PER_SECOND:.1f} m/s")
    print(f"Ceilings: {MAXIMUM_HOVER_ALTITUDE_FEET:.0f} ft and {MAXIMUM_HOVER_ALTITUDE_METERS:.1f} m.")
    print()

    try:
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

        # Require typed YES before arming.
        confirmation = input("Type YES to take off, hover 10s, fly 10 ft North, hover 10s, and land: ")
        if confirmation.strip() != "YES":
            print("Flight cancelled. The copter was not armed.")
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

        # Step 2: Hover at about 3 feet in LOITER for 10 seconds.
        hover(vehicle, "First hover")

        # Step 3: Fly forward in the North direction for 10 feet in GUIDED.
        flyNorth(vehicle)

        # Step 4: Hover in LOITER for 10 seconds.
        hover(vehicle, "Second hover")

        # Step 5: Land and wait for the motors to disarm.
        landingWasStarted = True
        land(vehicle)
        copterWasArmed = vehicle.armed
        print("Flight complete.")

    except KeyboardInterrupt:
        print()
        print("Flight interrupted.")

    except Exception as error:
        print(f"Flight stopped: {error}")

    finally:
        # If the flight stopped before Step 5, command LAND now.
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


if __name__ == "__main__":
    main()
