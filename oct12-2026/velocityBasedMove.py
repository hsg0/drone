#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    Take off, move by velocity, then land.
    The moves are forward and left along the nose, then true north and true west,
    then up, then down.

WHY:
    A velocity command tells the Pixhawk a speed, not a list of motor spins.
    Body frame speed follows the nose. Local north-east-down speed follows the map.
    The tutorial this is based on uses 5 meters per second and a 10 meter climb.
    Those numbers are too fast and too high for this copter, so this file uses
    the flight limits already set for it.

HOW:
    1. Connect, check Quad X, and wait for a GPS hold, the same way takeOffAndLand.py does.
    2. Type YES.
    3. GUIDED, arm, and climb to the cruise height.
    4. Repeat each speed once a second for 5 seconds. The Pixhawk drops a speed it stops hearing.
    5. Forward along the nose, left along the nose, true north, true west, up, then down.
    6. Up stops at 1.3 meters. Down stops at 1.0 meter. Down does not land.
    7. LAND and wait until the motors disarm.

IMPORTANT:
    Run this on the Pi, from this folder, so it can use takeOffAndLand.py.
    /dev/serial0 is pins 8 and 10 at 57600. Pin 6 is ground.
    Leave both USB ports alone. bootDisplay.py owns the Pixhawk USB port.
    Type YES only when the copter is clear to fly.
    This copter is a DJI F450 saved as Quad X, the same frame check as
    takeOffAndLand.py. DJI X (FRAME_TYPE 13) is a different motor order.
    Takeoff needs a 3D GPS fix, at least 4 satellites, and a trusted
    horizontal position.
    The arm command is a normal arm. This script never force-arms and never
    force-disarms.
    Positive body x is the nose. Positive body y is right. Positive z is down,
    so a climb is a negative z speed.
    Positive local x is true north. Positive local y is true east.
    Ctrl+C after arming requests LAND. A second Ctrl+C closes the cable.
"""

import os
import sys
import time

from pymavlink import mavutil as middleMan

# This file and takeOffAndLand.py live in the same folder.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import takeOffAndLand as flightLink


# ---------------------------------------------------------------------------
# Flight settings
# ---------------------------------------------------------------------------

# Horizontal and vertical speeds already used for gesture flight.
HORIZONTAL_SPEED_METERS_PER_SECOND = 0.4
CLIMB_RATE_METERS_PER_SECOND = 0.3
DESCEND_RATE_METERS_PER_SECOND = 0.3
# Fly-down stops here. LAND is the only way to the ground.
MINIMUM_ALTITUDE_METERS = 1.0
# Fly-up stops here so a held speed cannot climb away.
MAXIMUM_ALTITUDE_METERS = 1.3
# 95 percent of this height is still above the low limit, so the moves start inside the band.
CRUISE_ALTITUDE_METERS = 1.15

# The tutorial repeats one speed five times, one second apart, then pauses two seconds.
MOVE_STEP_COUNT = 5
MOVE_STEP_SECONDS = 1.0
PAUSE_BETWEEN_MOVES_SECONDS = 2.0

# Velocity only. Position, acceleration, and yaw are ignored so the copter does not turn.
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

BODY_FRAME = middleMan.mavlink.MAV_FRAME_BODY_NED
LOCAL_FRAME = middleMan.mavlink.MAV_FRAME_LOCAL_NED


# ---------------------------------------------------------------------------
# Velocity commands
# ---------------------------------------------------------------------------

def sendVelocity(frame, xMetersPerSecond, yMetersPerSecond, downMetersPerSecond):
    """Send one speed. x and y depend on the frame. z is always down."""
    connection = flightLink.pixhawkConnection
    connection.mav.set_position_target_local_ned_send(
        0,
        connection.target_system,
        connection.target_component,
        frame,
        VELOCITY_ONLY_TYPE_MASK,
        0,
        0,
        0,
        xMetersPerSecond,
        yMetersPerSecond,
        downMetersPerSecond,
        0,
        0,
        0,
        0,
        0,
    )


def stopVelocity(frame):
    """Zero the speed in the frame the last move used."""
    sendVelocity(frame, 0, 0, 0)


def moveIsStillAllowed(altitudeMeters, downMetersPerSecond):
    """None means this step may be sent. A string means the leg should stop."""
    if not flightLink.heartbeatIsFresh():
        return "Heartbeat was lost during the move."
    if flightLink.copterIsArmed() is not True:
        return "The copter disarmed during the move."
    if not flightLink.copterIsInMode(flightLink.GUIDED_MODE_NUMBER):
        return "GUIDED ended during the move."
    if not flightLink.gpsHoldIsReady():
        return "GPS position was lost during the move."
    if altitudeMeters is None:
        return None
    if downMetersPerSecond < 0 and altitudeMeters >= MAXIMUM_ALTITUDE_METERS:
        return f"High limit. {flightLink.heightFeetText()}"
    if downMetersPerSecond > 0 and altitudeMeters <= MINIMUM_ALTITUDE_METERS:
        return f"Low limit. {flightLink.heightFeetText()}"
    if downMetersPerSecond == 0 and (
        altitudeMeters < MINIMUM_ALTITUDE_METERS or altitudeMeters > MAXIMUM_ALTITUDE_METERS
    ):
        return f"Height left the move band. {flightLink.heightFeetText()}"
    return None


def holdVelocity(frame, xMetersPerSecond, yMetersPerSecond, downMetersPerSecond, moveName):
    """Repeat one speed for 5 seconds, then zero it. False means the flight should land now."""
    print(moveName)
    for stepNumber in range(1, MOVE_STEP_COUNT + 1):
        flightLink.drainMessages()
        altitudeMeters = flightLink.heightMeters()
        blockedReason = moveIsStillAllowed(altitudeMeters, downMetersPerSecond)
        if blockedReason is not None:
            print(blockedReason)
            stopVelocity(frame)
            # A height limit ends this leg. A lost link or a lost GPS ends the flight.
            heightLimitReached = blockedReason.startswith("High limit") or blockedReason.startswith("Low limit")
            return heightLimitReached
        sendVelocity(frame, xMetersPerSecond, yMetersPerSecond, downMetersPerSecond)
        print(f"{moveName} | step {stepNumber}/{MOVE_STEP_COUNT} | {flightLink.heightFeetText()}")
        time.sleep(MOVE_STEP_SECONDS)
    stopVelocity(frame)
    return True


def pauseBetweenMoves(frame):
    """Hold zero speed between legs so the last speed does not keep running."""
    pauseStartedAtSeconds = time.monotonic()
    while time.monotonic() - pauseStartedAtSeconds < PAUSE_BETWEEN_MOVES_SECONDS:
        flightLink.drainMessages()
        if not flightLink.heartbeatIsFresh() or flightLink.copterIsArmed() is not True:
            return False
        if not flightLink.gpsHoldIsReady():
            print("GPS position was lost between moves.")
            return False
        stopVelocity(frame)
        time.sleep(flightLink.POLL_INTERVAL_SECONDS)
    return True


def flyMovePattern():
    """The tutorial order: nose forward, nose left, true north, true west, up, down."""
    # The pasted tutorial forgets to reset its counter before true north, so that leg never runs.
    moves = (
        (BODY_FRAME, HORIZONTAL_SPEED_METERS_PER_SECOND, 0, 0, "Moving forward, along the nose."),
        (BODY_FRAME, 0, -HORIZONTAL_SPEED_METERS_PER_SECOND, 0, "Moving left, along the nose."),
        (LOCAL_FRAME, HORIZONTAL_SPEED_METERS_PER_SECOND, 0, 0, "Moving true north."),
        (LOCAL_FRAME, 0, -HORIZONTAL_SPEED_METERS_PER_SECOND, 0, "Moving true west."),
        (BODY_FRAME, 0, 0, -CLIMB_RATE_METERS_PER_SECOND, "Moving up."),
        (BODY_FRAME, 0, 0, DESCEND_RATE_METERS_PER_SECOND, "Moving down."),
    )
    for frame, xMetersPerSecond, yMetersPerSecond, downMetersPerSecond, moveName in moves:
        if not flightLink.copterIsInMode(flightLink.GUIDED_MODE_NUMBER):
            if not flightLink.requestMode(flightLink.GUIDED_MODE_NUMBER, "GUIDED"):
                return False
        legFinished = holdVelocity(
            frame,
            xMetersPerSecond,
            yMetersPerSecond,
            downMetersPerSecond,
            moveName,
        )
        if not legFinished:
            return False
        if not pauseBetweenMoves(frame):
            return False
    return True


def pilotTypedYes():
    """True only when the typed word is YES. This is the last check before arming."""
    print()
    print("Type YES to take off, move, and then land.")
    typedWord = input("> ").strip()
    if typedWord != "YES":
        print("Cancelled. The copter was not armed.")
        return False
    return True


def main():
    """Connect, arm, fly the velocity pattern, then land."""
    cruiseAltitudeFeet = CRUISE_ALTITUDE_METERS * flightLink.FEET_PER_METER
    print("VELOCITY BASED MOVE")
    print(f"Cruise altitude: {cruiseAltitudeFeet:.1f} ft")
    print(
        f"Horizontal {HORIZONTAL_SPEED_METERS_PER_SECOND:.1f} m/s | "
        f"climb {CLIMB_RATE_METERS_PER_SECOND:.1f} m/s | "
        f"descend {DESCEND_RATE_METERS_PER_SECOND:.1f} m/s"
    )
    print(
        f"Height stays between {MINIMUM_ALTITUDE_METERS:.1f} m and {MAXIMUM_ALTITUDE_METERS:.1f} m."
    )
    print()
    try:
        flightLink.openCommandCable()
        flightLink.requestFlightMessages()
        if not flightLink.quadXFrameIsSaved():
            return
        if not flightLink.waitUntilGpsCanHold():
            return
        if not pilotTypedYes():
            return
        flightLink.drainMessages()
        if not flightLink.confirmGpsStillReady():
            print("Not arming.")
            return
        flightLink.shortenSlowTakeoffRamp()
        if not flightLink.confirmGpsStillReady():
            print("Not arming.")
            return
        if not flightLink.requestMode(flightLink.GUIDED_MODE_NUMBER, "GUIDED"):
            print("Not arming.")
            return
        if not flightLink.armForTakeoff():
            return
        climbed = flightLink.climbToTakeoffHeight(CRUISE_ALTITUDE_METERS)
        if climbed:
            flyMovePattern()
        flightLink.landAndWaitForDisarm()
    except KeyboardInterrupt:
        print()
        print("Ctrl+C.")
        if flightLink.copterWasArmed:
            try:
                flightLink.landAndWaitForDisarm()
            except KeyboardInterrupt:
                print("Second Ctrl+C. Closing the cable. The copter is not force-disarmed.")
    except Exception as error:
        print()
        print(f"STOPPED: {error}")
        if flightLink.copterWasArmed:
            try:
                flightLink.landAndWaitForDisarm()
            except Exception as landingError:
                print(f"Landing error: {landingError}")
    finally:
        try:
            flightLink.restoreTakeoffRamp()
        except Exception as restoreError:
            print(f"Could not restore {flightLink.TAKEOFF_SLEW_PARAMETER_NAME}: {restoreError}")
        flightLink.closeCommandCable()
        print("Finished.")


if __name__ == "__main__":
    main()
