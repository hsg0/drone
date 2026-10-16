#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    Spin each Quad X motor by itself for 20 seconds, in the same 10, 15, 20,
    and 25 percent steps used before calibration, and save a performance log.

WHY:
    This is the check to run again after compass calibration. The log records
    whether each motor was accepted, the throttle steps, battery voltage,
    battery current, and the PWM sent to that motor.

HOW:
    1. Open the next afterCalibration-motorTest#N.log in oct16-2026/flightLog.
    2. Connect with DroneKit on pins 8 and 10. Leave USB with the screen.
    3. Type YES.
    4. Refuse to start if the copter is armed to fly.
    5. Spin motor 1, then 2, then 3, then 4. Five seconds at each percent.
    6. Write one performance block per motor, then a run summary.
    7. Close the link. A failed or stopped run is still saved.

IMPORTANT:
    Run this file on the Pi. Do not put python3 in front of the path.
    Take the propellers off before you run this.
    This script does not arm the Pixhawk for flight.
    A motor test sets the armed flag in the heartbeat. That is not a flight arm.
    The safety switch must be pressed, so the LED is solid, or the
    Pixhawk will refuse the motor test.
    Only one motor is commanded at a time.
    Ctrl+C tells the current motor to stop.
    The log name is afterCalibration-motorTest#N.log.
"""

import os
import sys
import time

from dronekit import connect
from pymavlink import mavutil as middleMan


# Pin 8 transmits. Pin 10 receives. Pin 6 is ground.
PIXHAWK_PORT = "/dev/serial0"
PIXHAWK_BAUD = 57600
GROUND_STATION_SYSTEM_ID = 255

CONNECTION_TIMEOUT_SECONDS = 15
HEARTBEAT_STALE_AFTER_SECONDS = 3.0
MOTOR_TEST_ANSWER_TIMEOUT_SECONDS = 3.0
MESSAGE_INTERVAL_MICROSECONDS = 200000
UNKNOWN_BATTERY_READING = -1

FLIGHT_LOG_DIRECTORY = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "flightLog",
)
LOG_TITLE_PREFIX = "afterCalibration-motorTest#"

MOTOR_RUN_SECONDS = 20.0
# Four steady speeds. One command covers each step, so the motor is not restarted every half second.
THROTTLE_STEP_PERCENTS = (10.0, 15.0, 20.0, 25.0)
SECONDS_PER_THROTTLE_STEP = MOTOR_RUN_SECONDS / len(THROTTLE_STEP_PERCENTS)
PAUSE_BETWEEN_MOTORS_SECONDS = 2.0
SAMPLE_INTERVAL_SECONDS = 1.0

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

PERFORMANCE_MESSAGE_NAMES = (
    "SYS_STATUS",
    "SERVO_OUTPUT_RAW",
)


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
    """Return the next afterCalibration-motorTest#N.log path."""
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


def heartbeatIsFresh(vehicle):
    """True when a Pixhawk heartbeat arrived within the last 3 seconds.

    DroneKit's last_heartbeat is already that age in seconds.
    """
    heartbeatAgeSeconds = vehicle.last_heartbeat
    if heartbeatAgeSeconds is None:
        return False
    return heartbeatAgeSeconds <= HEARTBEAT_STALE_AFTER_SECONDS


def connectToPixhawk(latestMessages):
    """Open pins 8 and 10 with DroneKit and wait for a heartbeat."""
    if not os.path.exists(PIXHAWK_PORT):
        raise RuntimeError(
            f"{PIXHAWK_PORT} is not on this computer. "
            "Run this script on the Pi. Pins 8 and 10 are there."
        )
    print()
    print(f"Opening {PIXHAWK_PORT} at {PIXHAWK_BAUD} baud.")
    # wait_ready stays off so this motor test does not download every parameter.
    vehicle = connect(
        PIXHAWK_PORT,
        baud=PIXHAWK_BAUD,
        wait_ready=False,
        timeout=CONNECTION_TIMEOUT_SECONDS,
        source_system=GROUND_STATION_SYSTEM_ID,
        heartbeat_timeout=CONNECTION_TIMEOUT_SECONDS,
    )
    if vehicle._master.target_component == 0:
        vehicle._master.target_component = 1

    @vehicle.on_message("STATUSTEXT")
    def printStatusText(vehicle, name, message):
        statusText = getattr(message, "text", "")
        if isinstance(statusText, bytes):
            statusText = statusText.decode("ascii", "ignore")
        statusText = str(statusText).strip("\x00").strip()
        if statusText:
            print(f"PIXHAWK: {statusText}")

    @vehicle.on_message(["SYS_STATUS", "SERVO_OUTPUT_RAW"])
    def rememberPerformance(vehicle, name, message):
        latestMessages[name] = message

    requestPerformanceMessages(vehicle)
    deadlineSeconds = time.monotonic() + CONNECTION_TIMEOUT_SECONDS
    while vehicle.last_heartbeat is None:
        if time.monotonic() > deadlineSeconds:
            vehicle.close()
            raise RuntimeError("No Pixhawk heartbeat arrived on pin 10.")
        time.sleep(0.1)
    print(
        f"Connected to Pixhawk system {vehicle._master.target_system}, "
        f"component {vehicle._master.target_component}."
    )
    return vehicle


def requestPerformanceMessages(vehicle):
    """Ask for battery and motor-output messages while a motor is spinning."""
    for messageName in PERFORMANCE_MESSAGE_NAMES:
        messageId = getattr(middleMan.mavlink, f"MAVLINK_MSG_ID_{messageName}", None)
        if messageId is None:
            continue
        message = vehicle.message_factory.command_long_encode(
            vehicle._master.target_system,
            vehicle._master.target_component,
            middleMan.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            messageId,
            MESSAGE_INTERVAL_MICROSECONDS,
            0,
            0,
            0,
            0,
            0,
        )
        vehicle.send_mavlink(message)
    vehicle.flush()


def sendMotorTest(vehicle, testSequence, throttlePercent, runSeconds):
    """Ask the Pixhawk to spin one motor at a percent for the given seconds."""
    message = vehicle.message_factory.command_long_encode(
        vehicle._master.target_system,
        vehicle._master.target_component,
        middleMan.mavlink.MAV_CMD_DO_MOTOR_TEST,
        0,
        testSequence,
        MOTOR_TEST_THROTTLE_IS_PERCENT,
        throttlePercent,
        runSeconds,
        MOTOR_TEST_ONE_MOTOR,
        0,
        0,
    )
    vehicle.send_mavlink(message)
    vehicle.flush()


def waitForMotorTestAnswer(vehicle, testSequence, throttlePercent, runSeconds):
    """Send one motor command and wait until the Pixhawk accepts or refuses it."""
    answers = []

    @vehicle.on_message("COMMAND_ACK")
    def rememberAnswer(vehicle, name, message):
        if message.command == middleMan.mavlink.MAV_CMD_DO_MOTOR_TEST:
            answers.append(message.result)

    sendMotorTest(vehicle, testSequence, throttlePercent, runSeconds)
    deadlineSeconds = time.monotonic() + MOTOR_TEST_ANSWER_TIMEOUT_SECONDS
    while time.monotonic() < deadlineSeconds:
        if answers:
            return answers[-1]
        time.sleep(0.1)
    return None


def stopMotor(vehicle, testSequence):
    """Send a zero-throttle motor test so the current motor stops."""
    sendMotorTest(vehicle, testSequence, 0, 0)


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
        f"{performanceSamples[0]['throttlePercent']:.0f} percent to "
        f"{performanceSamples[-1]['throttlePercent']:.0f} percent"
    )
    print(f"Battery voltage: {numberSummary([sample['volts'] for sample in performanceSamples], 'V')}")
    print(f"Battery current: {numberSummary([sample['amps'] for sample in performanceSamples], 'A')}")
    pwmValues = [sample["pwm"] for sample in performanceSamples if sample["pwm"] is not None]
    if pwmValues:
        print(f"Motor output PWM: low {min(pwmValues)} | high {max(pwmValues)} | end {pwmValues[-1]}")
    else:
        print("Motor output PWM: not reported")
    print("Time s | percent | volts | amps | pwm")
    for sample in performanceSamples:
        voltsText = "n/a" if sample["volts"] is None else f"{sample['volts']:.2f}"
        ampsText = "n/a" if sample["amps"] is None else f"{sample['amps']:.2f}"
        pwmText = "n/a" if sample["pwm"] is None else str(sample["pwm"])
        print(
            f"{sample['elapsedSeconds']:6.1f} | "
            f"{sample['throttlePercent']:7.0f} | "
            f"{voltsText:>5} | {ampsText:>4} | {pwmText}"
        )


def spinOneMotor(vehicle, latestMessages, motorNumber, testSequence, cornerName, turnDirection):
    """Run one motor for 20 seconds and return one sample per second."""
    print()
    print(f"Motor {motorNumber}, {cornerName}, should turn {turnDirection}.")
    print("Watch that motor now. The other three stay stopped.")
    performanceSamples = []
    elapsedAtStepStart = 0.0

    for throttlePercent in THROTTLE_STEP_PERCENTS:
        print(
            f"Motor {motorNumber} | {throttlePercent:.0f} percent | "
            f"{SECONDS_PER_THROTTLE_STEP:.0f} seconds"
        )
        motorTestAnswer = waitForMotorTestAnswer(
            vehicle,
            testSequence,
            throttlePercent,
            SECONDS_PER_THROTTLE_STEP,
        )
        if motorTestAnswer is None:
            printMotorPerformance(motorNumber, cornerName, turnDirection, "NO ANSWER", performanceSamples)
            raise RuntimeError(f"No answer for motor {motorNumber}. That motor was not left running.")
        print(f"Motor test answer = {motorTestAnswer}")
        if motorTestAnswer != middleMan.mavlink.MAV_RESULT_ACCEPTED:
            printMotorPerformance(motorNumber, cornerName, turnDirection, "REFUSED", performanceSamples)
            raise RuntimeError(
                "Motor test was refused. Press the safety switch until the LED is solid, then try again."
            )

        stepStartedAtSeconds = time.monotonic()
        nextSampleAtSeconds = 0.0
        while time.monotonic() - stepStartedAtSeconds < SECONDS_PER_THROTTLE_STEP:
            # A motor test sets the armed flag in the heartbeat. That is not a flight arm.
            if not heartbeatIsFresh(vehicle):
                printMotorPerformance(
                    motorNumber,
                    cornerName,
                    turnDirection,
                    "HEARTBEAT LOST",
                    performanceSamples,
                )
                raise RuntimeError("Pixhawk heartbeat was lost during the motor test.")
            secondsIntoStep = time.monotonic() - stepStartedAtSeconds
            if secondsIntoStep >= nextSampleAtSeconds:
                recordPerformanceSample(
                    performanceSamples,
                    elapsedAtStepStart + secondsIntoStep,
                    throttlePercent,
                    latestMessages,
                    motorNumber,
                )
                nextSampleAtSeconds += SAMPLE_INTERVAL_SECONDS
            time.sleep(0.2)
        elapsedAtStepStart += SECONDS_PER_THROTTLE_STEP

    stopMotor(vehicle, testSequence)
    print(f"Motor {motorNumber} stopped.")
    printMotorPerformance(motorNumber, cornerName, turnDirection, "COMPLETED", performanceSamples)
    return performanceSamples


def printRunSummary(motorOutcomes, overallResult, stopReason, logPath):
    """Write the result of every motor, including a failed or stopped run."""
    print()
    print("=" * 68)
    print("AFTER CALIBRATION MOTOR TEST SUMMARY")
    print("=" * 68)
    if not motorOutcomes:
        print("No motor was started.")
    for motorNumber, cornerName, resultName in motorOutcomes:
        print(f"Motor {motorNumber} ({cornerName}): {resultName}")
    print(f"Overall result: {overallResult}")
    if stopReason:
        print(f"Stop reason: {stopReason}")
    if logPath is not None:
        print(f"Saved flight log: {logPath}")
    print("=" * 68)


def main():
    """Spin motors 1 through 4, log each motor's performance, then close the link."""
    originalStandardOutput = sys.stdout
    logFile = None
    logPath = None
    vehicle = None
    latestMessages = {}
    activeTestSequence = None
    activeMotorNumber = None
    activeCornerName = None
    motorOutcomes = []
    overallResult = "FAILED"
    stopReason = None

    try:
        logFile, logPath = openFlightLog()
        print()
        print("==============================================")
        print("AFTER CALIBRATION MOTOR TEST")
        print("==============================================")
        print()
        print("Each motor spins alone for 20 seconds.")
        print("Throttle steps: 10, 15, 20, then 25 percent. Five seconds each.")
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
            overallResult = "CANCELLED"
            stopReason = "Pilot did not type YES."
            return

        vehicle = connectToPixhawk(latestMessages)
        if not heartbeatIsFresh(vehicle):
            raise RuntimeError("Pixhawk heartbeat was lost.")
        if vehicle.armed:
            print("The copter is armed. No motor was started.")
            overallResult = "FAILED"
            stopReason = "The copter was armed."
            return

        for motorNumber, testSequence, cornerName, turnDirection in MOTOR_TEST_PLAN:
            activeTestSequence = testSequence
            activeMotorNumber = motorNumber
            activeCornerName = cornerName
            spinOneMotor(
                vehicle,
                latestMessages,
                motorNumber,
                testSequence,
                cornerName,
                turnDirection,
            )
            motorOutcomes.append((motorNumber, cornerName, "COMPLETED"))
            activeTestSequence = None
            activeMotorNumber = None
            activeCornerName = None
            if motorNumber != MOTOR_TEST_PLAN[-1][0]:
                print(f"Waiting {PAUSE_BETWEEN_MOTORS_SECONDS:.0f} seconds before the next motor.")
                time.sleep(PAUSE_BETWEEN_MOTORS_SECONDS)
        print()
        print("All four motors finished.")
        overallResult = "SUCCESS"

    except KeyboardInterrupt:
        print()
        print("Ctrl+C. Stopping the current motor.")
        overallResult = "STOPPED"
        stopReason = "User pressed Ctrl+C."
        if activeMotorNumber is not None:
            motorOutcomes.append((activeMotorNumber, activeCornerName, "STOPPED BY CTRL+C"))

    except Exception as error:
        print(f"Motor test stopped: {error}")
        overallResult = "FAILED"
        stopReason = str(error)
        if activeMotorNumber is not None and not any(
            outcome[0] == activeMotorNumber for outcome in motorOutcomes
        ):
            motorOutcomes.append((activeMotorNumber, activeCornerName, "FAILED"))

    finally:
        if vehicle is not None and activeTestSequence is not None and heartbeatIsFresh(vehicle):
            stopMotor(vehicle, activeTestSequence)
            print("Motor stop sent.")
        if vehicle is not None:
            try:
                vehicle.close()
            except Exception:
                pass
            print("Pixhawk connection closed.")
        print("Finished.")
        printRunSummary(motorOutcomes, overallResult, stopReason, logPath)
        sys.stdout = originalStandardOutput
        if logFile is not None:
            logFile.close()


if __name__ == "__main__":
    main()
