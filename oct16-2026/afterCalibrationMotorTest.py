#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    Spin each Quad X motor by itself for 20 seconds, ramping from 10 percent
    to 25 percent, and save a performance log for the run.

WHY:
    This is the same basic motor check used before calibration, run again
    after calibration. The log records whether each motor was accepted, how
    the throttle rose, and what the Pixhawk reported for voltage, current,
    and motor output.

HOW:
    1. Open the next numbered log in oct16-2026/flightLog.
    2. Connect on pins 8 and 10. Leave USB with the screen.
    3. Type YES.
    4. Refuse to start if the copter is armed.
    5. Spin motor 1, then 2, then 3, then 4.
    6. Each spin lasts 20 seconds and climbs from 10 percent to 25 percent.
    7. Write one performance block per motor, then a run summary.
    8. Close the link.

IMPORTANT:
    Take the propellers off before you run this.
    This script does not arm the Pixhawk.
    The safety switch must be pressed, so the LED is solid, or the
    Pixhawk will refuse the motor test.
    Only one motor is commanded at a time.
    Ctrl+C tells the current motor to stop. The copter was never armed.
    The log name is afterCalibration-motorTest-#N.log.
"""

import os
import sys
import time

from pymavlink import mavutil


# Pin 8 transmits. Pin 10 receives. Pin 6 is ground.
PIXHAWK_SERIAL_PORT = "/dev/serial0"
PIXHAWK_SERIAL_BAUD = 57600

FLIGHT_LOG_DIRECTORY = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "flightLog",
)
LOG_TITLE_PREFIX = "afterCalibration-motorTest-#"

CONNECTION_TIMEOUT_SECONDS = 15
HEARTBEAT_STALE_AFTER_SECONDS = 3.0
MOTOR_TEST_ANSWER_TIMEOUT_SECONDS = 3.0
MESSAGE_INTERVAL_MICROSECONDS = 200000

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
UNKNOWN_BATTERY_READING = -1

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

PERFORMANCE_MESSAGE_NAMES = (
    "SYS_STATUS",
    "SERVO_OUTPUT_RAW",
    "BATTERY_STATUS",
    "VIBRATION",
    "ESC_TELEMETRY_1_TO_4",
)

lastHeartbeatReceivedAtSeconds = None


class FlightLog:
    """Copy every printed line to the terminal and to the flight log file."""

    def __init__(self, logFile, terminal):
        self.logFile = logFile
        self.terminal = terminal

    def write(self, text):
        self.terminal.write(text)
        self.terminal.flush()
        self.logFile.write(text)
        self.logFile.flush()

    def writeLogOnly(self, text):
        """Write a line to the log file without repeating it on the terminal."""
        self.logFile.write(text)
        self.logFile.flush()

    def flush(self):
        self.terminal.flush()
        self.logFile.flush()


def nextMotorTestLogPath():
    """Return the next afterCalibration-motorTest-#N.log path."""
    os.makedirs(FLIGHT_LOG_DIRECTORY, exist_ok=True)
    nextNumber = 1
    for fileName in os.listdir(FLIGHT_LOG_DIRECTORY):
        if not fileName.startswith(LOG_TITLE_PREFIX) or not fileName.endswith(".log"):
            continue
        numberText = fileName[len(LOG_TITLE_PREFIX):-len(".log")]
        if numberText.isdigit():
            nextNumber = max(nextNumber, int(numberText) + 1)
    return os.path.join(FLIGHT_LOG_DIRECTORY, f"{LOG_TITLE_PREFIX}{nextNumber}.log")


def openFlightLog():
    """Open the next numbered after-calibration motor-test log."""
    logPath = nextMotorTestLogPath()
    logFile = open(logPath, "w", encoding="utf-8")
    sys.stdout = FlightLog(logFile, sys.stdout)
    print(f"Flight log file : {logPath}")
    print(f"Run started at  : {time.strftime('%Y-%m-%d %H:%M:%S %Z')}")
    return logFile, logPath


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
    requestPerformanceMessages(pixhawkConnection)
    print(
        f"Connected to Pixhawk system {pixhawkConnection.target_system}, "
        f"component {pixhawkConnection.target_component}."
    )
    return pixhawkConnection


def requestPerformanceMessages(pixhawkConnection):
    """Ask for battery, motor output, and vibration while a motor is spinning."""
    for messageName in PERFORMANCE_MESSAGE_NAMES:
        messageId = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{messageName}", None)
        if messageId is None:
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


def batteryVoltsAndAmps(latestMessages):
    """Return battery volts and amps, or None when the Pixhawk has not reported them."""
    systemStatus = latestMessages.get("SYS_STATUS")
    if systemStatus is None:
        return None, None
    volts = None
    amps = None
    if systemStatus.voltage_battery != UNKNOWN_BATTERY_READING:
        volts = systemStatus.voltage_battery / 1000.0
    if systemStatus.current_battery != UNKNOWN_BATTERY_READING:
        amps = systemStatus.current_battery / 100.0
    return volts, amps


def motorOutputPwm(latestMessages, motorNumber):
    """Return the PWM the Pixhawk is sending to this motor output, or None."""
    servoOutput = latestMessages.get("SERVO_OUTPUT_RAW")
    if servoOutput is None:
        return None
    return getattr(servoOutput, f"servo{motorNumber}_raw", None)


def recordPerformanceSample(performanceSamples, elapsedSeconds, throttlePercent, latestMessages, motorNumber):
    """Save one row of commanded throttle and the Pixhawk readings at that moment."""
    volts, amps = batteryVoltsAndAmps(latestMessages)
    performanceSamples.append(
        {
            "elapsedSeconds": elapsedSeconds,
            "throttlePercent": throttlePercent,
            "volts": volts,
            "amps": amps,
            "pwm": motorOutputPwm(latestMessages, motorNumber),
        }
    )


def numberSummary(values, unit):
    """Format the low, high, and last value of a reading."""
    usableValues = [value for value in values if value is not None]
    if not usableValues:
        return "not reported"
    return (
        f"low {min(usableValues):.2f} {unit} | "
        f"high {max(usableValues):.2f} {unit} | "
        f"end {usableValues[-1]:.2f} {unit}"
    )


def printMotorPerformance(motorNumber, cornerName, turnDirection, motorResult, performanceSamples):
    """Write the performance block for one motor into the log and the terminal."""
    print()
    print(f"--- Motor {motorNumber} performance ---")
    print(f"Corner: {cornerName}")
    print(f"Expected turn: {turnDirection}")
    print(f"Result: {motorResult}")
    print(f"Samples: {len(performanceSamples)}")
    if not performanceSamples:
        print("No performance samples were recorded.")
        return
    print(
        "Commanded throttle: "
        f"{performanceSamples[0]['throttlePercent']:.1f} percent to "
        f"{performanceSamples[-1]['throttlePercent']:.1f} percent"
    )
    print(f"Battery voltage: {numberSummary([sample['volts'] for sample in performanceSamples], 'V')}")
    print(f"Battery current: {numberSummary([sample['amps'] for sample in performanceSamples], 'A')}")
    pwmValues = [sample["pwm"] for sample in performanceSamples]
    usablePwm = [value for value in pwmValues if value is not None]
    if usablePwm:
        print(f"Motor output PWM: low {min(usablePwm)} | high {max(usablePwm)} | end {usablePwm[-1]}")
    else:
        print("Motor output PWM: not reported")
    print("Time s | percent | volts | amps | pwm")
    for sample in performanceSamples:
        voltsText = "n/a" if sample["volts"] is None else f"{sample['volts']:.2f}"
        ampsText = "n/a" if sample["amps"] is None else f"{sample['amps']:.2f}"
        pwmText = "n/a" if sample["pwm"] is None else str(sample["pwm"])
        print(
            f"{sample['elapsedSeconds']:6.1f} | "
            f"{sample['throttlePercent']:7.1f} | "
            f"{voltsText:>5} | {ampsText:>4} | {pwmText}"
        )


def spinOneMotor(pixhawkConnection, latestMessages, motorNumber, testSequence, cornerName, turnDirection):
    """Ramp one motor from 10 percent to 25 percent and return its performance samples."""
    print()
    print(f"Motor {motorNumber}, {cornerName}, should turn {turnDirection}.")
    print(
        f"Ramping from {START_THROTTLE_PERCENT:.0f} percent "
        f"to {END_THROTTLE_PERCENT:.0f} percent over {MOTOR_RUN_SECONDS:.0f} seconds."
    )

    performanceSamples = []
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
        printMotorPerformance(
            motorNumber,
            cornerName,
            turnDirection,
            "REFUSED",
            performanceSamples,
        )
        raise RuntimeError(
            "Motor test was refused. Press the safety switch until the LED is solid, then try again."
        )

    spinStartedAtSeconds = time.monotonic()
    lastCommandAtSeconds = 0.0
    lastPrintedAtSeconds = 0.0
    recordPerformanceSample(
        performanceSamples,
        0.0,
        startingThrottlePercent,
        latestMessages,
        motorNumber,
    )
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
            recordPerformanceSample(
                performanceSamples,
                elapsedSeconds,
                throttlePercent,
                latestMessages,
                motorNumber,
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
    printMotorPerformance(
        motorNumber,
        cornerName,
        turnDirection,
        "COMPLETED",
        performanceSamples,
    )
    return performanceSamples


def printRunSummary(motorOutcomes, logPath):
    """Write the result of every motor at the end of the log."""
    print()
    print("=" * 68)
    print("AFTER CALIBRATION MOTOR TEST SUMMARY")
    print("=" * 68)
    completedCount = 0
    for motorNumber, cornerName, resultName in motorOutcomes:
        print(f"Motor {motorNumber} ({cornerName}): {resultName}")
        if resultName == "COMPLETED":
            completedCount += 1
    if completedCount == len(MOTOR_TEST_PLAN):
        print("Overall result: SUCCESS")
    elif completedCount == 0:
        print("Overall result: FAILED")
    else:
        print("Overall result: STOPPED PART WAY")
    if logPath is not None:
        print(f"Saved flight log: {logPath}")
    print("=" * 68)


def main():
    """Spin motors 1 through 4, log each motor, then close the link."""
    originalStandardOutput = sys.stdout
    logFile = None
    logPath = None
    pixhawkConnection = None
    latestMessages = {}
    activeTestSequence = None
    activeMotorNumber = None
    activeCornerName = None
    motorOutcomes = []

    try:
        logFile, logPath = openFlightLog()
        print()
        print("==============================================")
        print("AFTER CALIBRATION MOTOR TEST")
        print("==============================================")
        print()
        print("Each motor spins alone for 20 seconds.")
        print("Throttle starts at 10 percent and rises to 25 percent.")
        print("Order: motor 1, motor 2, motor 3, motor 4.")
        print("Take the propellers off first.")
        print("No GPS or compass is required.")
        print()
        promptText = "Type YES to start: "
        confirmation = input("> ")
        if isinstance(sys.stdout, FlightLog):
            sys.stdout.writeLogOnly(f"{promptText}{confirmation}\n")
        if confirmation.strip() != "YES":
            print("Cancelled.")
            motorOutcomes.append((0, "none", "CANCELLED BEFORE START"))
            return

        pixhawkConnection = connectToPixhawk(latestMessages)
        readAvailableMessages(pixhawkConnection, latestMessages)
        if not heartbeatIsFresh():
            raise RuntimeError("Pixhawk heartbeat was lost.")
        if copterIsArmed(latestMessages) is not False:
            print("The copter is armed, or arm status is unknown. No motor was started.")
            motorOutcomes.append((0, "none", "REFUSED BECAUSE ARMED OR UNKNOWN"))
            return

        for motorNumber, testSequence, cornerName, turnDirection in MOTOR_TEST_PLAN:
            activeTestSequence = testSequence
            activeMotorNumber = motorNumber
            activeCornerName = cornerName
            try:
                spinOneMotor(
                    pixhawkConnection,
                    latestMessages,
                    motorNumber,
                    testSequence,
                    cornerName,
                    turnDirection,
                )
                motorOutcomes.append((motorNumber, cornerName, "COMPLETED"))
            except Exception:
                if not any(outcome[0] == motorNumber for outcome in motorOutcomes):
                    motorOutcomes.append((motorNumber, cornerName, "FAILED"))
                raise
            activeTestSequence = None
            activeMotorNumber = None
            activeCornerName = None
            if motorNumber != MOTOR_TEST_PLAN[-1][0]:
                print(f"Waiting {PAUSE_BETWEEN_MOTORS_SECONDS:.0f} seconds before the next motor.")
                time.sleep(PAUSE_BETWEEN_MOTORS_SECONDS)
        print()
        print("All four motors finished.")

    except KeyboardInterrupt:
        print()
        print("Ctrl+C. Stopping the current motor.")
        if activeMotorNumber is not None and not any(
            outcome[0] == activeMotorNumber for outcome in motorOutcomes
        ):
            motorOutcomes.append((activeMotorNumber, activeCornerName, "STOPPED BY CTRL+C"))

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
        printRunSummary(motorOutcomes, logPath)
        sys.stdout = originalStandardOutput
        if logFile is not None:
            logFile.close()


if __name__ == "__main__":
    main()
