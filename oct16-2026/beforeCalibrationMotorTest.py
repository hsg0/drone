#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    Spin each Quad X motor by itself for 20 seconds, ramping from 10 percent
    to 25 percent. The copter stays disarmed.

WHY:
    This is a basic check that every motor turns the right way before compass
    calibration. It does not use GPS, the compass, or the EKF.

HOW:
    1. Connect on pins 8 and 10. Leave USB with the screen.
    2. Type YES.
    3. Refuse to start if the copter is armed.
    4. Spin motor 1, then 2, then 3, then 4.
    5. Each spin lasts 20 seconds and climbs from 10 percent to 25 percent.
    6. Stop that motor before the next one starts.
    7. Close the link.

IMPORTANT:
    Take the propellers off before you run this.
    This script does not arm the Pixhawk.
    The safety switch must be pressed, so the LED is solid, or the
    Pixhawk will refuse the motor test.
    Only one motor is commanded at a time.
    Ctrl+C tells the current motor to stop. The copter was never armed.
"""

import time

from pymavlink import mavutil


# Pin 8 transmits. Pin 10 receives. Pin 6 is ground.
PIXHAWK_SERIAL_PORT = "/dev/serial0"
PIXHAWK_SERIAL_BAUD = 57600

CONNECTION_TIMEOUT_SECONDS = 15
HEARTBEAT_STALE_AFTER_SECONDS = 3.0
MOTOR_TEST_ANSWER_TIMEOUT_SECONDS = 3.0

START_THROTTLE_PERCENT = 10.0
END_THROTTLE_PERCENT = 25.0
MOTOR_RUN_SECONDS = 20.0
# A new command this often keeps the ramp moving and stops the motor if the script dies.
MOTOR_COMMAND_INTERVAL_SECONDS = 0.5
MOTOR_COMMAND_TIMEOUT_SECONDS = 1.0
PAUSE_BETWEEN_MOTORS_SECONDS = 2.0
PRINT_INTERVAL_SECONDS = 1.0

# Throttle type 0 means the number is a percent, not a PWM pulse.
MOTOR_TEST_THROTTLE_IS_PERCENT = 0
# Zero means test one motor. Copter turns that into a count of one.
MOTOR_TEST_ONE_MOTOR = 0

# The motor-test command takes the Quad X test order, not the motor number:
#   test order 1 spins motor 1, front-right, counter-clockwise
#   test order 2 spins motor 4, back-right, clockwise
#   test order 3 spins motor 2, back-left, counter-clockwise
#   test order 4 spins motor 3, front-left, clockwise
MOTOR_TEST_PLAN = (
    (1, 1, "front-right", "counter-clockwise"),
    (2, 3, "back-left", "counter-clockwise"),
    (3, 4, "front-left", "clockwise"),
    (4, 2, "back-right", "clockwise"),
)

lastHeartbeatReceivedAtSeconds = None


def rememberHeartbeat(pixhawkConnection, message):
    """Save the time of a heartbeat from this Pixhawk."""
    global lastHeartbeatReceivedAtSeconds
    if message.get_srcSystem() != pixhawkConnection.target_system:
        return
    lastHeartbeatReceivedAtSeconds = time.monotonic()


def heartbeatIsFresh():
    """True when a heartbeat arrived within the last 3 seconds."""
    if lastHeartbeatReceivedAtSeconds is None:
        return False
    heartbeatAgeSeconds = time.monotonic() - lastHeartbeatReceivedAtSeconds
    return heartbeatAgeSeconds <= HEARTBEAT_STALE_AFTER_SECONDS


def connectToPixhawk(latestMessages):
    """Open pins 8 and 10 and wait until the Pixhawk answers."""
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
    """Store one Pixhawk message and print any status text."""
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


def readAvailableMessages(pixhawkConnection, latestMessages):
    """Read every message already waiting on the serial port."""
    while True:
        message = pixhawkConnection.recv_match(blocking=False)
        if message is None:
            return
        processIncomingMessage(pixhawkConnection, latestMessages, message)


def copterIsArmed(latestMessages):
    """True when the motors are armed. None until a heartbeat has been stored."""
    heartbeat = latestMessages.get("HEARTBEAT")
    if heartbeat is None:
        return None
    return bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def throttlePercentAtElapsedSeconds(elapsedSeconds):
    """10 percent at the start of the spin, 25 percent at 20 seconds."""
    if elapsedSeconds <= 0:
        return START_THROTTLE_PERCENT
    if elapsedSeconds >= MOTOR_RUN_SECONDS:
        return END_THROTTLE_PERCENT
    rampFraction = elapsedSeconds / MOTOR_RUN_SECONDS
    throttleRise = END_THROTTLE_PERCENT - START_THROTTLE_PERCENT
    return START_THROTTLE_PERCENT + (rampFraction * throttleRise)


def sendMotorTest(pixhawkConnection, testSequence, throttlePercent, runSeconds):
    """Ask the Pixhawk to spin one motor at a percent for a short time."""
    pixhawkConnection.mav.command_long_send(
        pixhawkConnection.target_system,
        pixhawkConnection.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
        0,
        testSequence,
        MOTOR_TEST_THROTTLE_IS_PERCENT,
        throttlePercent,
        runSeconds,
        MOTOR_TEST_ONE_MOTOR,
        0,
        0,
    )


def waitForMotorTestAnswer(pixhawkConnection, latestMessages):
    """Wait up to 3 seconds for the Pixhawk to accept or refuse the spin."""
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


def stopMotor(pixhawkConnection, testSequence):
    """Send a zero-throttle motor test so the current motor stops."""
    sendMotorTest(pixhawkConnection, testSequence, 0, 0)


def spinOneMotor(pixhawkConnection, latestMessages, motorNumber, testSequence, cornerName, turnDirection):
    """Ramp one motor from 10 percent to 25 percent over 20 seconds."""
    print()
    print(
        f"Motor {motorNumber}, {cornerName}, should turn {turnDirection}."
    )
    print(
        f"Ramping from {START_THROTTLE_PERCENT:.0f} percent "
        f"to {END_THROTTLE_PERCENT:.0f} percent over {MOTOR_RUN_SECONDS:.0f} seconds."
    )

    startingThrottlePercent = throttlePercentAtElapsedSeconds(0)
    sendMotorTest(
        pixhawkConnection,
        testSequence,
        startingThrottlePercent,
        MOTOR_COMMAND_TIMEOUT_SECONDS,
    )
    motorTestAnswer = waitForMotorTestAnswer(pixhawkConnection, latestMessages)
    if motorTestAnswer is None:
        raise RuntimeError(f"No answer for motor {motorNumber}. That motor was not left running.")
    print(f"Motor test answer = {motorTestAnswer}")
    if motorTestAnswer != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        raise RuntimeError(
            "Motor test was refused. Press the safety switch until the LED is solid, then try again."
        )

    spinStartedAtSeconds = time.monotonic()
    lastCommandAtSeconds = spinStartedAtSeconds
    lastPrintedAtSeconds = 0.0
    while True:
        readAvailableMessages(pixhawkConnection, latestMessages)
        if not heartbeatIsFresh():
            raise RuntimeError("Pixhawk heartbeat was lost during the motor test.")
        if copterIsArmed(latestMessages) is True:
            raise RuntimeError("The copter armed during the motor test. Stopping.")

        elapsedSeconds = time.monotonic() - spinStartedAtSeconds
        if elapsedSeconds >= MOTOR_RUN_SECONDS:
            break

        nowSeconds = time.monotonic()
        if nowSeconds - lastCommandAtSeconds >= MOTOR_COMMAND_INTERVAL_SECONDS:
            throttlePercent = throttlePercentAtElapsedSeconds(elapsedSeconds)
            sendMotorTest(
                pixhawkConnection,
                testSequence,
                throttlePercent,
                MOTOR_COMMAND_TIMEOUT_SECONDS,
            )
            lastCommandAtSeconds = nowSeconds
            if nowSeconds - lastPrintedAtSeconds >= PRINT_INTERVAL_SECONDS:
                remainingSeconds = max(0.0, MOTOR_RUN_SECONDS - elapsedSeconds)
                print(
                    f"Motor {motorNumber} | {throttlePercent:.1f} percent | "
                    f"{remainingSeconds:.0f} s left"
                )
                lastPrintedAtSeconds = nowSeconds
        time.sleep(0.05)

    stopMotor(pixhawkConnection, testSequence)
    print(f"Motor {motorNumber} stopped.")


def main():
    """Spin motors 1, 2, 3, and 4 one at a time, then close the link."""
    pixhawkConnection = None
    latestMessages = {}
    activeTestSequence = None

    print()
    print("==============================================")
    print("BEFORE CALIBRATION MOTOR TEST")
    print("==============================================")
    print()
    print("Each motor spins alone for 20 seconds.")
    print("Throttle starts at 10 percent and rises to 25 percent.")
    print("Order: motor 1, motor 2, motor 3, motor 4.")
    print("Take the propellers off first.")
    print("No GPS or compass is required.")
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
            print("The copter is armed, or arm status is unknown. No motor was started.")
            return

        for motorNumber, testSequence, cornerName, turnDirection in MOTOR_TEST_PLAN:
            activeTestSequence = testSequence
            spinOneMotor(
                pixhawkConnection,
                latestMessages,
                motorNumber,
                testSequence,
                cornerName,
                turnDirection,
            )
            activeTestSequence = None
            if motorNumber != MOTOR_TEST_PLAN[-1][0]:
                print(f"Waiting {PAUSE_BETWEEN_MOTORS_SECONDS:.0f} seconds before the next motor.")
                time.sleep(PAUSE_BETWEEN_MOTORS_SECONDS)
        print()
        print("All four motors finished.")

    except KeyboardInterrupt:
        print()
        print("Ctrl+C. Stopping the current motor.")

    except Exception as error:
        print(f"Motor test stopped: {error}")

    finally:
        if pixhawkConnection is not None and activeTestSequence is not None and heartbeatIsFresh():
            stopMotor(pixhawkConnection, activeTestSequence)
            print("Motor stop sent.")
        if pixhawkConnection is not None:
            try:
                pixhawkConnection.close()
            except Exception:
                pass
            print("Pixhawk connection closed.")
        print("Finished.")


if __name__ == "__main__":
    main()
