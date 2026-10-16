
#!/home/drone/dronekit-venv/bin/python3

"""
WHAT:
    Take off without requiring GPS, briefly hover,
    then land automatically.

WHY:
    Test basic motor lift and attitude stabilization.

HOW:
    1. Connect to the Pixhawk.
    2. Check frame, attitude, and controller readiness.
    3. Ask the pilot to type YES.
    4. Enter GUIDED_NOGPS and arm.
    5. Climb toward 0.5 meters.
    6. Hold altitude for 1 second.
    7. Switch to LAND and wait for disarming.

IMPORTANT:
    GPS is not required by this script.
    Pixhawk pre-arm checks remain enabled.
    GPS-related pre-arm requirements must already
    be configured correctly in ArduPilot.

    Altitude estimates without GPS can be inaccurate.
    This script cannot guarantee a 3-foot ceiling.

    Keep the RC transmitter ready to take control.
    Test in a clear, controlled area.
    Never force-arm or force-disarm in the air.
"""

import math
import os
import time

from dronekit import VehicleMode, connect


# --------------------------------------------------
# SETTINGS
# --------------------------------------------------

PIXHAWK_PORT = "/dev/serial0"
PIXHAWK_BAUD = 57600

CONNECTION_TIMEOUT_SECONDS = 60
MODE_TIMEOUT_SECONDS = 10
ARM_TIMEOUT_SECONDS = 15

TARGET_ALTITUDE_METERS = 0.50
MAXIMUM_ALTITUDE_METERS = 0.85

HOVER_DURATION_SECONDS = 1.0
TAKEOFF_TIMEOUT_SECONDS = 5.0
LAND_TIMEOUT_SECONDS = 60.0

HEARTBEAT_TIMEOUT_SECONDS = 3.0

# ArduPilot default climb-rate interpretation:
# 0.5 = hold altitude
# Above 0.5 = climb
CLIMB_COMMAND = 0.58
HOVER_COMMAND = 0.50

COMMAND_INTERVAL_SECONDS = 0.10

MAXIMUM_GROUND_LEAN_DEGREES = 5.0

QUAD_FRAME_CLASS = 1
QUAD_X_FRAME_TYPE = 1


# --------------------------------------------------
# HELPER FUNCTIONS
# --------------------------------------------------

def heartbeatIsFresh(vehicle):
    """Check that Pixhawk is still communicating."""

    heartbeatAge = vehicle.last_heartbeat

    return (
        heartbeatAge is not None
        and heartbeatAge <= HEARTBEAT_TIMEOUT_SECONDS
    )


def getAltitudeMeters(vehicle):
    """Read reported relative altitude."""

    location = vehicle.location.global_relative_frame

    if location is None or location.alt is None:
        return None

    altitude = float(location.alt)

    if not math.isfinite(altitude):
        return None

    return altitude


def getAltitudeAboveStart(vehicle, startingAltitude):
    """Calculate altitude relative to takeoff reading."""

    currentAltitude = getAltitudeMeters(vehicle)

    if currentAltitude is None:
        return None

    return currentAltitude - startingAltitude


def checkPreflight(vehicle):
    """Confirm basic flight-controller readiness."""

    if not heartbeatIsFresh(vehicle):
        raise RuntimeError("Pixhawk heartbeat missing.")

    frameClass = vehicle.parameters["FRAME_CLASS"]
    frameType = vehicle.parameters["FRAME_TYPE"]

    if frameClass is None or frameType is None:
        raise RuntimeError("Cannot read frame settings.")

    if int(frameClass) != QUAD_FRAME_CLASS:
        raise RuntimeError("Quad frame required.")

    if int(frameType) != QUAD_X_FRAME_TYPE:
        raise RuntimeError("Quad X motor layout required.")

    guidedOptions = vehicle.parameters["GUID_OPTIONS"]

    if guidedOptions is None:
        raise RuntimeError("Cannot read GUID_OPTIONS.")

    # Bit 3 changes the meaning of thrust commands.
    if int(guidedOptions) & 8:
        raise RuntimeError(
            "GUID_OPTIONS enables direct thrust. "
            "This script requires climb-rate control."
        )

    attitude = vehicle.attitude

    if (
        attitude is None
        or attitude.roll is None
        or attitude.pitch is None
        or attitude.yaw is None
    ):
        raise RuntimeError("Attitude data unavailable.")

    rollDegrees = math.degrees(attitude.roll)
    pitchDegrees = math.degrees(attitude.pitch)

    if (
        abs(rollDegrees) > MAXIMUM_GROUND_LEAN_DEGREES
        or abs(pitchDegrees) > MAXIMUM_GROUND_LEAN_DEGREES
    ):
        raise RuntimeError("Drone must start level.")

    if not vehicle.is_armable:
        raise RuntimeError(
            "Pixhawk is not armable. "
            "Check ArduPilot pre-arm messages."
        )

    if getAltitudeMeters(vehicle) is None:
        raise RuntimeError("Altitude estimate unavailable.")

    print("Preflight passed.")
    print("GPS position check not required by script.")
    print("Quad X configuration confirmed.")


def changeFlightMode(vehicle, modeName):
    """Request a mode and confirm that Pixhawk accepted it."""

    print(f"Requesting {modeName}.")

    vehicle.mode = VehicleMode(modeName)

    startTime = time.monotonic()

    while time.monotonic() - startTime < MODE_TIMEOUT_SECONDS:

        if vehicle.mode.name == modeName:
            print(f"Mode confirmed: {modeName}")
            return

        if not heartbeatIsFresh(vehicle):
            raise RuntimeError("Pixhawk connection lost.")

        time.sleep(0.1)

    raise RuntimeError(f"Could not enter {modeName}.")


def sendAttitudeCommand(vehicle, yawRadians, climbCommand):
    """
    Request level roll and pitch.

    The Pixhawk handles motor mixing and stabilization.

    With GUID_OPTIONS bit 3 disabled:
        0.50 means zero requested climb rate.
        Above 0.50 requests upward movement.
    """

    quaternion = [
        math.cos(yawRadians / 2.0),
        0.0,
        0.0,
        math.sin(yawRadians / 2.0),
    ]

    message = vehicle.message_factory.set_attitude_target_encode(
        0,
        vehicle._master.target_system,
        vehicle._master.target_component,
        0b00000111,
        quaternion,
        0.0,
        0.0,
        0.0,
        climbCommand,
    )

    vehicle.send_mavlink(message)


def armDrone(vehicle):
    """Arm normally without bypassing Pixhawk checks."""

    print("Arming motors.")

    vehicle.armed = True

    startTime = time.monotonic()

    while not vehicle.armed:

        if time.monotonic() - startTime > ARM_TIMEOUT_SECONDS:
            raise RuntimeError("Arming timed out.")

        if not heartbeatIsFresh(vehicle):
            raise RuntimeError("Pixhawk connection lost.")

        time.sleep(0.1)

    print("Motors armed.")


def checkFlightState(vehicle):
    """Stop normal flight commands if flight state changes."""

    if not heartbeatIsFresh(vehicle):
        raise RuntimeError("Pixhawk connection lost.")

    if not vehicle.armed:
        raise RuntimeError("Drone disarmed.")

    if vehicle.mode.name != "GUIDED_NOGPS":
        raise RuntimeError("GUIDED_NOGPS mode lost.")


def takeOff(vehicle, startingAltitude, startingYaw):
    """Request a short climb to the target height."""

    print("Starting GPS-free takeoff.")

    startTime = time.monotonic()
    lastPrintTime = 0.0

    while True:

        checkFlightState(vehicle)

        currentTime = time.monotonic()
        elapsedTime = currentTime - startTime

        altitude = getAltitudeAboveStart(
            vehicle,
            startingAltitude,
        )

        if altitude is None:
            raise RuntimeError("Altitude reading missing.")

        if altitude >= MAXIMUM_ALTITUDE_METERS:
            raise RuntimeError("Altitude limit reached.")

        if altitude >= TARGET_ALTITUDE_METERS:
            print(f"Target reached: {altitude:.2f} meters.")
            return

        if elapsedTime >= TAKEOFF_TIMEOUT_SECONDS:
            raise RuntimeError("Takeoff timed out.")

        sendAttitudeCommand(
            vehicle,
            startingYaw,
            CLIMB_COMMAND,
        )

        if currentTime - lastPrintTime >= 0.5:
            print(f"Climbing: {altitude:.2f} meters.")
            lastPrintTime = currentTime

        time.sleep(COMMAND_INTERVAL_SECONDS)


def hoverOneSecond(vehicle, startingAltitude, startingYaw):
    """Request zero climb for exactly one second."""

    print("Starting 1-second altitude hold.")

    startTime = time.monotonic()

    while time.monotonic() - startTime < HOVER_DURATION_SECONDS:

        checkFlightState(vehicle)

        altitude = getAltitudeAboveStart(
            vehicle,
            startingAltitude,
        )

        if altitude is None:
            raise RuntimeError("Altitude reading missing.")

        if altitude >= MAXIMUM_ALTITUDE_METERS:
            raise RuntimeError("Altitude limit reached.")

        sendAttitudeCommand(
            vehicle,
            startingYaw,
            HOVER_COMMAND,
        )

        time.sleep(COMMAND_INTERVAL_SECONDS)

    print("1-second hover command completed.")


def landDrone(vehicle):
    """Request LAND and wait until motors disarm."""

    if not vehicle.armed:
        print("Motors already disarmed.")
        return

    print("LANDING.")

    changeFlightMode(vehicle, "LAND")

    startTime = time.monotonic()

    while vehicle.armed:

        if not heartbeatIsFresh(vehicle):
            raise RuntimeError(
                "Landing state unknown: connection lost."
            )

        if time.monotonic() - startTime > LAND_TIMEOUT_SECONDS:
            raise RuntimeError(
                "Landing timed out. Motors may still be armed."
            )

        if vehicle.mode.name != "LAND":
            raise RuntimeError("LAND mode was overridden.")

        time.sleep(0.2)

    print("Landed. Motors disarmed.")


# --------------------------------------------------
# MAIN FLIGHT
# --------------------------------------------------

def main():
    """Connect, take off, hover, land, and close."""

    vehicle = None

    print("--------------------------------")
    print("GPS-FREE 1-SECOND HOVER TEST")
    print("--------------------------------")

    print(f"Target: {TARGET_ALTITUDE_METERS:.2f} meters")
    print(f"Hover command: {HOVER_DURATION_SECONDS:.1f} second")
    print("Landing: automatic")
    print("GPS: not required by script")
    print()

    try:

        if not os.path.exists(PIXHAWK_PORT):
            raise RuntimeError("Pixhawk serial port missing.")

        print("Connecting to Pixhawk.")

        vehicle = connect(
            PIXHAWK_PORT,
            baud=PIXHAWK_BAUD,
            wait_ready=True,
            timeout=CONNECTION_TIMEOUT_SECONDS,
            source_system=255,
        )

        print("Pixhawk connected.")

        checkPreflight(vehicle)

        confirmation = input(
            "Type YES to take off, hover, and land: "
        )

        if confirmation.strip() != "YES":
            print("Flight cancelled.")
            return

        checkPreflight(vehicle)

        changeFlightMode(vehicle, "GUIDED_NOGPS")

        # Record original heading before leaving the ground.
        startingYaw = vehicle.attitude.yaw

        # Use the reported ground altitude as the starting point.
        startingAltitude = getAltitudeMeters(vehicle)

        if startingAltitude is None:
            raise RuntimeError("Starting altitude unavailable.")

        armDrone(vehicle)

        takeOff(
            vehicle,
            startingAltitude,
            startingYaw,
        )

        hoverOneSecond(
            vehicle,
            startingAltitude,
            startingYaw,
        )

        landDrone(vehicle)

        print("Flight completed.")

    except KeyboardInterrupt:
        print("\nFlight interrupted.")

    except Exception as error:
        print(f"Flight stopped: {error}")

    finally:

        if vehicle is not None:

            if vehicle.armed:
                print("Requesting emergency LAND sequence.")

                try:
                    landDrone(vehicle)

                except Exception as landingError:
                    print(f"LAND not confirmed: {landingError}")

                except KeyboardInterrupt:
                    print("Landing wait interrupted.")

            vehicle.close()
            print("Pixhawk connection closed.")

        print("Script finished.")


if __name__ == "__main__":
    main()
