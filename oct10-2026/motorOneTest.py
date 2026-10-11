#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    Spin motor 1 only, for a few seconds, while the copter stays disarmed.

WHY:
    On this Quad X, motor 1 is the front-right motor.
    Looking down on the copter, that motor should turn counter-clockwise.
    A separate script makes it obvious which motor is being asked to spin.

HOW:
    1. Connect on pins 8 and 10. Leave USB with the screen.
    2. Type YES.
    3. Refuse to start if the copter is armed.
    4. Ask the Pixhawk to run motor 1 at 20 percent for 5 seconds.
    5. Wait for that spin to finish, and print any Pixhawk status text.
    6. Tell motor 1 to stop, then close the link.

IMPORTANT:
    Take the propellers off before you run this.
    This script does not arm the Pixhawk.
    The safety switch must be pressed, so the LED is solid, or the
    Pixhawk will refuse the motor test.
    Ctrl+C tells motor 1 to stop. The copter was never armed.
"""

import time

from pymavlink import mavutil


# Pin 8 transmits. Pin 10 receives. Pin 6 is ground.
PIXHAWK_SERIAL_PORT = "/dev/serial0"
PIXHAWK_SERIAL_BAUD = 57600

CONNECTION_TIMEOUT_SECONDS = 15
HEARTBEAT_STALE_AFTER_SECONDS = 3.0
MOTOR_TEST_ANSWER_TIMEOUT_SECONDS = 3.0

# ArduPilot motor 1 on a Quad X is the front-right motor.
MOTOR_NUMBER = 1
MOTOR_THROTTLE_PERCENT = 20
MOTOR_RUN_SECONDS = 5

# The motor-test command does not take the motor number. It takes the Quad X
# test order, and that order is not 1, 2, 3, 4:
#   test order 1 spins motor 1, front-right, counter-clockwise
#   test order 2 spins motor 4, back-right, clockwise
#   test order 3 spins motor 2, back-left, counter-clockwise
#   test order 4 spins motor 3, front-left, clockwise
MOTOR_TEST_SEQUENCE = 1

# Throttle type 0 means the number above is a percent, not a PWM pulse.
MOTOR_TEST_THROTTLE_IS_PERCENT = 0
# Zero means test one motor. Copter turns that into a count of one.
MOTOR_TEST_ONE_MOTOR = 0

lastHeartbeatReceivedAtSeconds = None


# Saves the time of a heartbeat from this Pixhawk, so a lost link can be noticed.
def rememberHeartbeat(pixhawkConnection, message):
    global lastHeartbeatReceivedAtSeconds
    if message.get_srcSystem() != pixhawkConnection.target_system:
        return
    lastHeartbeatReceivedAtSeconds = time.monotonic()


# True when a heartbeat arrived within the last 3 seconds.
def heartbeatIsFresh():
    if lastHeartbeatReceivedAtSeconds is None:
        return False
    heartbeatAgeSeconds = time.monotonic() - lastHeartbeatReceivedAtSeconds
    return heartbeatAgeSeconds <= HEARTBEAT_STALE_AFTER_SECONDS


# Opens pins 8 and 10 and waits until the Pixhawk answers.
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


# Stores one Pixhawk message and prints any status text. Ignores a broken byte.
def processIncomingMessage(pixhawkConnection, latestMessages, message):
    if message is None:
        return
    if message.get_type() == "BAD_DATA":
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


# Reads every message already waiting on the serial port.
def readAvailableMessages(pixhawkConnection, latestMessages):
    while True:
        message = pixhawkConnection.recv_match(blocking=False)
        if message is None:
            return
        processIncomingMessage(pixhawkConnection, latestMessages, message)


# True when the motors are armed. None until a heartbeat has been stored.
def copterIsArmed(latestMessages):
    heartbeat = latestMessages.get("HEARTBEAT")
    if heartbeat is None:
        return None
    return bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


# Tells the Pixhawk to spin motor 1 at the given percent for the given seconds.
def sendMotorTest(pixhawkConnection, throttlePercent, runSeconds):
    pixhawkConnection.mav.command_long_send(
        pixhawkConnection.target_system,
        pixhawkConnection.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
        0,
        MOTOR_TEST_SEQUENCE,
        MOTOR_TEST_THROTTLE_IS_PERCENT,
        throttlePercent,
        runSeconds,
        MOTOR_TEST_ONE_MOTOR,
        0,
        0,
    )


# Waits up to 3 seconds for the Pixhawk to accept or refuse the spin.
def waitForMotorTestAnswer(pixhawkConnection, latestMessages):
    deadlineSeconds = time.monotonic() + MOTOR_TEST_ANSWER_TIMEOUT_SECONDS
    while time.monotonic() < deadlineSeconds:
        message = pixhawkConnection.recv_match(blocking=True, timeout=0.4)
        if message is None:
            continue
        processIncomingMessage(pixhawkConnection, latestMessages, message)
        if message.get_type() != "COMMAND_ACK":
            continue
        if message.command != mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST:
            continue
        return message.result
    return None


# Asks for YES, spins motor 1 for 5 seconds, then closes the link.
def main():
    pixhawkConnection = None
    latestMessages = {}
    motorSpinWasAccepted = False

    print()
    print("==============================================")
    print("MOTOR 1 TEST")
    print("==============================================")
    print()
    print("Motor 1 is the front-right motor on this Quad X.")
    print("Looking down, it should turn counter-clockwise.")
    print(f"It will spin at {MOTOR_THROTTLE_PERCENT} percent for {MOTOR_RUN_SECONDS:.0f} seconds.")
    print("Take the propellers off first.")
    print()
    print("Type YES to start.")
    if input("> ").strip() != "YES":
        print("Cancelled.")
        return

    try:
        pixhawkConnection = connectToPixhawk(latestMessages)
        readAvailableMessages(pixhawkConnection, latestMessages)
        if not heartbeatIsFresh():
            raise RuntimeError("Pixhawk heartbeat was lost.")
        if copterIsArmed(latestMessages) is not False:
            print("The copter is armed, or arm status is unknown. Motor 1 was not started.")
            return

        print()
        print(f"Asking motor {MOTOR_NUMBER} to spin.")
        sendMotorTest(pixhawkConnection, MOTOR_THROTTLE_PERCENT, MOTOR_RUN_SECONDS)
        motorTestAnswer = waitForMotorTestAnswer(pixhawkConnection, latestMessages)
        if motorTestAnswer is None:
            print("No answer from the Pixhawk. Motor 1 was not left running.")
            return
        print(f"Motor test answer = {motorTestAnswer}")
        if motorTestAnswer != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Motor test was refused. Press the safety switch until the LED is solid, then try again.")
            return
        motorSpinWasAccepted = True

        spinDeadlineSeconds = time.monotonic() + MOTOR_RUN_SECONDS
        while time.monotonic() < spinDeadlineSeconds:
            readAvailableMessages(pixhawkConnection, latestMessages)
            time.sleep(0.1)
        print("Motor 1 spin finished.")

    except KeyboardInterrupt:
        print()
        print("Ctrl+C. Stopping motor 1.")

    finally:
        if pixhawkConnection is not None and motorSpinWasAccepted and heartbeatIsFresh():
            sendMotorTest(pixhawkConnection, 0, 0)
            print("Motor 1 stop sent.")
        if pixhawkConnection is not None:
            try:
                pixhawkConnection.close()
            except Exception:
                pass
            print("Pixhawk connection closed.")
        print("Finished.")


if __name__ == "__main__":
    main()
