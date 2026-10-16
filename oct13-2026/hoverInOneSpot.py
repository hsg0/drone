#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    Take off to 3 feet, hover in one GPS spot for 30 seconds, and land.

WHY:
    LOITER holds the spot. GUIDED only climbs. LAND is the way back down.
    DroneKit sends those mode changes. The Pixhawk still mixes the motors.

HOW:
    1. Connect to the Pixhawk on pins 8 and 10.
    2. Check the frame, GPS, position estimate, and that the copter is level.
    3. Ask the pilot to type YES.
    4. Take off in GUIDED to 3 feet.
    5. Hover in LOITER for 30 seconds.
    6. Land and wait until the motors disarm.

IMPORTANT:
    Run this on the Pi with /home/drone/dronekit-venv/bin/python3.
    /dev/serial0 is pins 8 and 10 at 57600. Pin 6 is ground.
    Leave both USB ports alone. bootDisplay.py owns the Pixhawk USB port.
    This copter is a DJI F450 saved as Quad X (FRAME_CLASS 1, FRAME_TYPE 1).
    DJI X is FRAME_TYPE 13 and uses a different motor order, so it will not arm.
    The hover target is 3 feet. Reaching 4 feet, or 1.3 meters, starts LAND.
    The arm command is a normal arm. This script never force-arms and never
    force-disarms.
    Type YES only when the copter is clear to fly.
    Ctrl+C after arming requests LAND. A second Ctrl+C closes the cable.
"""

import math
import os
import time

from dronekit import VehicleMode
from dronekit import connect
from pymavlink import mavutil as middleMan


# --------------------------------------------------
# SETTINGS
# --------------------------------------------------

# Pin 8 transmits. Pin 10 receives. Pin 6 is ground.
PIXHAWK_PORT = "/dev/serial0"
PIXHAWK_BAUD = 57600
CONNECTION_TIMEOUT_SECONDS = 60

FEET_PER_METER = 3.28084
HOVER_ALTITUDE_FEET = 3.0
HOVER_ALTITUDE_METERS = HOVER_ALTITUDE_FEET / FEET_PER_METER
# 4 feet is 1.22 meters, so this ceiling is reached before the 1.3 meter ceiling.
MAXIMUM_HOVER_ALTITUDE_FEET = 4.0
MAXIMUM_HOVER_ALTITUDE_METERS = 1.3
FOUR_FOOT_CEILING_METERS = MAXIMUM_HOVER_ALTITUDE_FEET / FEET_PER_METER
HOVER_DURATION_SECONDS = 30
# Close enough to 3 feet to call the climb done.
TAKEOFF_REACHED_FRACTION = 0.95

TAKEOFF_TIMEOUT_SECONDS = 30
LAND_TIMEOUT_SECONDS = 60
MODE_CHANGE_TIMEOUT_SECONDS = 10
ARM_TIMEOUT_SECONDS = 15
# A quiet cable is treated as lost after this long.
HEARTBEAT_STALE_AFTER_SECONDS = 3.0
# A leaning start leaves one side on the ground.
MAXIMUM_GROUND_LEAN_DEGREES = 5.0

# Quad X. The value from the Pixhawk arrives as a decimal, so compare with a little slack.
QUAD_FRAME_CLASS = 1
QUAD_X_FRAME_TYPE = 1
MINIMUM_GPS_FIX_TYPE = 3
MINIMUM_SATELLITE_COUNT = 6
# 255 means the receiver did not report a count.
UNKNOWN_SATELLITE_COUNT = 255

# A ramp slower than 1 second leaves one side on the ground long enough to tip.
TAKEOFF_SLEW_PARAMETER_NAME = "TKOFF_SLEW_TIME"
PREFERRED_TAKEOFF_SLEW_SECONDS = 1.0

# This serial port stays quiet until each message is requested.
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
# FLIGHT HELPERS
# --------------------------------------------------

def heartbeatIsFresh(vehicle):
    """True when a heartbeat arrived in the last 3 seconds.

    DroneKit's last_heartbeat is already that age in seconds.
    """
    heartbeatAgeSeconds = vehicle.last_heartbeat
    if heartbeatAgeSeconds is None:
        return False
    return heartbeatAgeSeconds <= HEARTBEAT_STALE_AFTER_SECONDS


def requestFlightMessages(vehicle):
    """Ask the Pixhawk to send GPS, height, and position health.

    On pins 8 and 10 those messages do not arrive by themselves.
    ekf_ok stays false until a position-health message shows up.
    """
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
    """Change flight mode and wait until the Pixhawk agrees."""
    print(f"Requesting {flightModeName}.")
    vehicle.mode = VehicleMode(flightModeName)
    startTime = time.monotonic()
    while time.monotonic() - startTime < MODE_CHANGE_TIMEOUT_SECONDS:
        if vehicle.mode.name == flightModeName:
            print(f"Mode confirmed: {flightModeName}")
            return True
        if not heartbeatIsFresh(vehicle):
            raise RuntimeError("Pixhawk heartbeat was lost.")
        time.sleep(0.2)
    raise RuntimeError(f"Could not enter {flightModeName}.")


def heightMeters(vehicle):
    """Height above the spot where the Pixhawk was turned on, in meters."""
    location = vehicle.location.global_relative_frame
    if location is None or location.alt is None:
        return None
    altitudeMeters = float(location.alt)
    if not math.isfinite(altitudeMeters):
        return None
    return altitudeMeters


def heightText(altitudeMeters):
    """Meters and feet together, so the 4 foot ceiling is easy to see."""
    if altitudeMeters is None:
        return "height waiting"
    altitudeFeet = altitudeMeters * FEET_PER_METER
    return f"{altitudeMeters:.2f} m ({altitudeFeet:.2f} ft)"


def heightReachedCeiling(altitudeMeters):
    """True at 4 feet or at 1.3 meters. Four feet comes first."""
    if altitudeMeters is None:
        return False
    reachedFourFeet = altitudeMeters >= FOUR_FOOT_CEILING_METERS
    reachedMeterCeiling = altitudeMeters >= MAXIMUM_HOVER_ALTITUDE_METERS
    return reachedFourFeet or reachedMeterCeiling


def preflightProblem(vehicle):
    """None when the copter may arm. Otherwise the one reason it may not."""
    if not heartbeatIsFresh(vehicle):
        return "Pixhawk heartbeat is missing."

    # A wrong frame would tip on liftoff. DJI X (13) is not this copter's order.
    frameClass = vehicle.parameters["FRAME_CLASS"]
    frameType = vehicle.parameters["FRAME_TYPE"]
    if frameClass is None or frameType is None:
        return "FRAME_CLASS or FRAME_TYPE could not be read."
    frameIsQuad = abs(float(frameClass) - QUAD_FRAME_CLASS) < 0.01
    frameIsX = abs(float(frameType) - QUAD_X_FRAME_TYPE) < 0.01
    if not frameIsQuad or not frameIsX:
        return (
            f"Quad X required. FRAME_CLASS {float(frameClass):.0f}, "
            f"FRAME_TYPE {float(frameType):.0f}."
        )

    gpsReading = vehicle.gps_0
    if gpsReading is None or gpsReading.fix_type is None or int(gpsReading.fix_type) < MINIMUM_GPS_FIX_TYPE:
        return "GPS 3D fix required."
    satelliteCount = gpsReading.satellites_visible
    if satelliteCount is None or satelliteCount == UNKNOWN_SATELLITE_COUNT or satelliteCount < MINIMUM_SATELLITE_COUNT:
        return f"At least 6 satellites required. Seeing {satelliteCount}."

    # ekf_ok means the horizontal position is trusted and is not a constant-position guess.
    # Satellite count is not that check. A 3D fix can still leave the EKF in constant-position mode.
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

    # is_armable is the Pixhawk's own pre-arm list: compass, GPS, and the safety switch.
    if not vehicle.is_armable:
        return "Pixhawk is not armable yet."

    if vehicle.attitude is None or vehicle.attitude.roll is None or vehicle.attitude.pitch is None:
        return "Attitude is not available."
    # Attitude arrives in radians. The 5 degree check is easier to read in degrees.
    rollDegrees = math.degrees(vehicle.attitude.roll)
    pitchDegrees = math.degrees(vehicle.attitude.pitch)
    if abs(rollDegrees) > MAXIMUM_GROUND_LEAN_DEGREES or abs(pitchDegrees) > MAXIMUM_GROUND_LEAN_DEGREES:
        return f"Copter is not level. Roll {rollDegrees:+.1f} deg, pitch {pitchDegrees:+.1f} deg."

    return None


def waitForPreflight(vehicle):
    """Keep printing the failing check. GPS usually arrives after the cable opens."""
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
        time.sleep(1)


def shortenSlowTakeoffRamp(vehicle):
    """Return the saved ramp when this flight shortens it. None means leave it alone."""
    savedTakeoffSlewSeconds = vehicle.parameters[TAKEOFF_SLEW_PARAMETER_NAME]
    if savedTakeoffSlewSeconds is None:
        print(f"{TAKEOFF_SLEW_PARAMETER_NAME} was not available. The existing ramp will be used.")
        return None
    savedTakeoffSlewSeconds = float(savedTakeoffSlewSeconds)
    temporaryTakeoffSlewSeconds = min(savedTakeoffSlewSeconds, PREFERRED_TAKEOFF_SLEW_SECONDS)
    print(
        f"{TAKEOFF_SLEW_PARAMETER_NAME}: "
        f"original {savedTakeoffSlewSeconds:.2f}s | "
        f"takeoff {temporaryTakeoffSlewSeconds:.2f}s"
    )
    if abs(temporaryTakeoffSlewSeconds - savedTakeoffSlewSeconds) < 0.01:
        return None
    vehicle.parameters[TAKEOFF_SLEW_PARAMETER_NAME] = temporaryTakeoffSlewSeconds
    return savedTakeoffSlewSeconds


def restoreTakeoffRamp(vehicle, originalTakeoffSlewSeconds):
    """Put the original ramp back only when this flight changed it."""
    if vehicle is None or originalTakeoffSlewSeconds is None or not heartbeatIsFresh(vehicle):
        return
    print(f"Restoring {TAKEOFF_SLEW_PARAMETER_NAME} to {originalTakeoffSlewSeconds:.2f}s.")
    vehicle.parameters[TAKEOFF_SLEW_PARAMETER_NAME] = originalTakeoffSlewSeconds
    print(f"{TAKEOFF_SLEW_PARAMETER_NAME} restored.")


def takeOff(vehicle):
    """Arm and climb to 3 feet. simple_takeoff wants meters above home."""
    print("Entering GUIDED mode.")
    setFlightMode(vehicle, "GUIDED")

    # Setting armed asks for a normal arm. It does not bypass the pre-arm checks.
    print("Arming motors.")
    vehicle.armed = True
    startTime = time.monotonic()
    while not vehicle.armed:
        if time.monotonic() - startTime > ARM_TIMEOUT_SECONDS:
            raise RuntimeError("Arming timed out.")
        if not heartbeatIsFresh(vehicle):
            raise RuntimeError("Pixhawk heartbeat was lost while arming.")
        time.sleep(0.2)
    print("Motors armed.")

    print(f"Taking off to {HOVER_ALTITUDE_FEET:.0f} feet.")
    vehicle.simple_takeoff(HOVER_ALTITUDE_METERS)
    minimumAltitudeMeters = HOVER_ALTITUDE_METERS * TAKEOFF_REACHED_FRACTION
    startTime = time.monotonic()
    lastPrintedAtSeconds = 0.0
    while True:
        if not vehicle.armed:
            raise RuntimeError("Copter disarmed during takeoff.")
        if vehicle.mode.name != "GUIDED":
            raise RuntimeError("GUIDED mode lost.")
        if not vehicle.ekf_ok:
            raise RuntimeError("Position estimate lost.")
        if not heartbeatIsFresh(vehicle):
            raise RuntimeError("Pixhawk heartbeat was lost.")

        altitudeMeters = heightMeters(vehicle)
        if heightReachedCeiling(altitudeMeters):
            raise RuntimeError(f"Altitude ceiling reached. {heightText(altitudeMeters)}")

        currentTimeSeconds = time.monotonic()
        if currentTimeSeconds - lastPrintedAtSeconds >= 1.0:
            print(f"Altitude: {heightText(altitudeMeters)}")
            lastPrintedAtSeconds = currentTimeSeconds

        if altitudeMeters is not None and altitudeMeters >= minimumAltitudeMeters:
            print("Target altitude reached.")
            return
        if currentTimeSeconds - startTime > TAKEOFF_TIMEOUT_SECONDS:
            raise RuntimeError("Takeoff timed out.")
        time.sleep(0.2)


def hover(vehicle):
    """Hold the GPS spot where the copter is when LOITER starts."""
    setFlightMode(vehicle, "LOITER")
    print("Hover started.")
    startTime = time.monotonic()
    while time.monotonic() - startTime < HOVER_DURATION_SECONDS:
        if not heartbeatIsFresh(vehicle):
            raise RuntimeError("Pixhawk heartbeat was lost.")
        if not vehicle.armed:
            raise RuntimeError("Copter disarmed.")
        if vehicle.mode.name != "LOITER":
            raise RuntimeError("LOITER mode lost.")
        if not vehicle.ekf_ok:
            raise RuntimeError("Position estimate lost.")

        altitudeMeters = heightMeters(vehicle)
        if heightReachedCeiling(altitudeMeters):
            raise RuntimeError(f"Altitude ceiling reached. {heightText(altitudeMeters)}")

        elapsedSeconds = time.monotonic() - startTime
        remainingSeconds = max(0.0, HOVER_DURATION_SECONDS - elapsedSeconds)
        print(f"Hovering | {heightText(altitudeMeters)} | remaining {remainingSeconds:.0f} s")
        time.sleep(1)
    print("30-second hover completed.")


def land(vehicle):
    """Request LAND and wait until the motors disarm. This does not force-disarm."""
    if not vehicle.armed:
        print("Motors already disarmed.")
        return
    if not heartbeatIsFresh(vehicle):
        print("Heartbeat is stale. LAND was not sent.")
        return

    print("Landing.")
    setFlightMode(vehicle, "LAND")
    startTime = time.monotonic()
    while vehicle.armed:
        if time.monotonic() - startTime > LAND_TIMEOUT_SECONDS:
            raise RuntimeError("Landing not confirmed. The motors are still armed.")
        altitudeMeters = heightMeters(vehicle)
        print(f"Waiting for landing. {heightText(altitudeMeters)}")
        time.sleep(1)
    print("Landed. Motors disarmed.")


# --------------------------------------------------
# MAIN FLIGHT
# --------------------------------------------------

def main():
    """Connect, confirm, climb, hover, land, and always close the cable."""
    vehicle = None
    originalTakeoffSlewSeconds = None
    copterWasArmed = False

    print("HOVER IN ONE SPOT")
    print(f"Hover altitude: {HOVER_ALTITUDE_FEET:.0f} ft")
    print(f"Hover time: {HOVER_DURATION_SECONDS:.0f} seconds")
    print(f"Ceilings: {MAXIMUM_HOVER_ALTITUDE_FEET:.0f} ft and {MAXIMUM_HOVER_ALTITUDE_METERS:.1f} m.")
    print()

    try:
        if not os.path.exists(PIXHAWK_PORT):
            raise RuntimeError(f"{PIXHAWK_PORT} is missing. Pins 8 and 10 are on the Pi.")

        print(f"Connecting to Pixhawk on {PIXHAWK_PORT} at {PIXHAWK_BAUD} baud.")
        vehicle = connect(
            PIXHAWK_PORT,
            baud=PIXHAWK_BAUD,
            wait_ready=True,
            timeout=CONNECTION_TIMEOUT_SECONDS,
            source_system=255,
        )
        print("Pixhawk connected.")
        requestFlightMessages(vehicle)

        # The first check can sit here until GPS and the level attitude are good.
        waitForPreflight(vehicle)

        confirmation = input("Type YES to take off, hover 30 seconds, and land: ")
        if confirmation.strip() != "YES":
            print("Flight cancelled. The copter was not armed.")
            return

        # The prompt can sit long enough for the GPS reading to go stale.
        waitForPreflight(vehicle)
        originalTakeoffSlewSeconds = shortenSlowTakeoffRamp(vehicle)
        # Reading that setting can pause the GPS stream, so look once more.
        problem = preflightProblem(vehicle)
        if problem is not None:
            raise RuntimeError(problem)

        takeOff(vehicle)
        copterWasArmed = True
        hover(vehicle)
        land(vehicle)
        copterWasArmed = vehicle.armed
        print("Flight complete.")

    except KeyboardInterrupt:
        print()
        print("Flight interrupted.")

    except Exception as error:
        print(f"Flight stopped: {error}")

    finally:
        # An armed copter gets LAND. A second Ctrl+C only closes the cable.
        if vehicle is not None and (copterWasArmed or vehicle.armed):
            print("Requesting LAND.")
            try:
                land(vehicle)
            except KeyboardInterrupt:
                print("Second Ctrl+C. Closing the cable. The copter is not force-disarmed.")
            except Exception as landingError:
                print(f"Landing unconfirmed: {landingError}")
        try:
            restoreTakeoffRamp(vehicle, originalTakeoffSlewSeconds)
        except Exception as restoreError:
            print(f"Could not restore {TAKEOFF_SLEW_PARAMETER_NAME}: {restoreError}")
        if vehicle is not None:
            vehicle.close()
            print("Command cable closed.")
        print("Script finished.")


if __name__ == "__main__":
    main()
