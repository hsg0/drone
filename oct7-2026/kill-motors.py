#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    Turn the motors off.

    Connect to the Pixhawk and disarm it. If a normal disarm is refused,
    send the force-disarm value so the motors stop anyway.

WHY:
    A separate script can stop the motors without starting a flight test.
    Run this when the copter should be disarmed now.

HOW:
    1. Connect on pins 8 and 10. Leave USB with the screen.
    2. Read the heartbeat.
    3. If the copter is already disarmed, stop.
    4. Send a normal disarm and wait for the heartbeat to show disarmed.
    5. If that is refused, send the force disarm and wait again.
    6. Close the link.

IMPORTANT:
    This script does not ask for YES. Starting it sends the disarm.
    A normal disarm can be refused while ArduPilot still treats the copter
    as flying. The force disarm stops the motors in that case too.
    This script does not change the flight mode and does not arm.
"""

import time

from pymavlink import mavutil


# Pin 8 transmits. Pin 10 receives. Pin 6 is ground.
PIXHAWK_SERIAL_PORT = "/dev/serial0"
PIXHAWK_SERIAL_BAUD = 57600

CONNECTION_TIMEOUT_SECONDS = 15
DISARM_CONFIRMATION_TIMEOUT_SECONDS = 5
HEARTBEAT_STALE_AFTER_SECONDS = 3.0

# ArduPilot can refuse a normal disarm while it still treats the copter as flying.
# This is the value it requires before it will stop the motors anyway.
FORCE_DISARM_MAGIC_NUMBER = 21196

lastHeartbeatReceivedAtSeconds = None


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


def connectToPixhawk(latestMessages):
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
    processIncomingMessage(pixhawkConnection, latestMessages, heartbeat)
    print(
        f"Connected to Pixhawk system {pixhawkConnection.target_system}, "
        f"component {pixhawkConnection.target_component}."
    )
    return pixhawkConnection


def processIncomingMessage(pixhawkConnection, latestMessages, message):
    if message is None:
        return
    if message.get_srcSystem() not in (0, pixhawkConnection.target_system):
        return
    messageType = message.get_type()
    latestMessages[messageType] = message
    if messageType == "HEARTBEAT":
        rememberHeartbeat(pixhawkConnection, message)
    elif messageType == "STATUSTEXT":
        statusText = getattr(message, "text", "")
        if isinstance(statusText, bytes):
            statusText = statusText.decode("ascii", "ignore")
        statusText = str(statusText).strip("\x00").strip()
        if statusText:
            print(f"PIXHAWK: {statusText}")


def copterIsArmed(latestMessages):
    heartbeat = latestMessages.get("HEARTBEAT")
    if heartbeat is None:
        return None
    return bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def sendDisarmCommand(pixhawkConnection, forceDisarm):
    if forceDisarm:
        disarmMagicNumber = FORCE_DISARM_MAGIC_NUMBER
    else:
        disarmMagicNumber = 0
    pixhawkConnection.mav.command_long_send(
        pixhawkConnection.target_system,
        pixhawkConnection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,
        disarmMagicNumber,
        0,
        0,
        0,
        0,
        0,
    )


def waitUntilMotorsAreOff(pixhawkConnection, latestMessages):
    deadlineSeconds = time.monotonic() + DISARM_CONFIRMATION_TIMEOUT_SECONDS
    while time.monotonic() < deadlineSeconds:
        message = pixhawkConnection.recv_match(blocking=True, timeout=0.5)
        if message is None:
            continue
        processIncomingMessage(pixhawkConnection, latestMessages, message)
        if copterIsArmed(latestMessages) is False:
            return True
    return copterIsArmed(latestMessages) is False


def turnMotorsOff(pixhawkConnection, latestMessages):
    if not heartbeatIsFresh():
        raise RuntimeError("Pixhawk heartbeat was lost before the disarm was sent.")

    if copterIsArmed(latestMessages) is False:
        print("Motors are already off.")
        return

    print("Turning the motors off.")
    sendDisarmCommand(pixhawkConnection, forceDisarm=False)
    if waitUntilMotorsAreOff(pixhawkConnection, latestMessages):
        print("Motors off.")
        return

    print("Normal disarm was refused. Forcing the motors off.")
    sendDisarmCommand(pixhawkConnection, forceDisarm=True)
    if waitUntilMotorsAreOff(pixhawkConnection, latestMessages):
        print("Motors off.")
        return

    print("Could not confirm that the motors turned off.")


def main():
    pixhawkConnection = None
    latestMessages = {}

    try:
        print()
        print("==============================================")
        print("KILL MOTORS")
        print("==============================================")

        pixhawkConnection = connectToPixhawk(latestMessages)
        turnMotorsOff(pixhawkConnection, latestMessages)

    except KeyboardInterrupt:
        print()
        print("Ctrl+C during the disarm.")

    except Exception as error:
        print()
        print(f"STOPPED: {error}")

    finally:
        if pixhawkConnection is not None:
            try:
                pixhawkConnection.close()
            except Exception:
                pass
            print("Pixhawk connection closed.")
        print("Finished.")


if __name__ == "__main__":
    main()
