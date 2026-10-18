#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    Spin each Quad X motor by itself at 20 percent so the propeller direction
    and mounting can be seen. The copter stays disarmed.

WHY:
    The propellers are on for this check. Twenty percent is enough to see
    which way a propeller turns and whether it is seated the right way up.
    It is not a flight throttle. GPS, the compass, and the EKF are not used.

HOW:
    1. Connect with DroneKit on pins 8 and 10. Leave USB with the screen.
    2. Type YES.
    3. Refuse to start if the copter is armed.
    4. Spin motor 1, then 2, then 3, then 4, each at 20 percent.
    5. Stop that motor before the next one starts.
    6. Save the run in oct17-2026/flightLog, whether it finishes or stops.
    7. Close the link.

IMPORTANT:
    Run this file on the Pi. Do not put python3 in front of the path.
    The propellers stay on. Hold the frame down. One propeller at 20 percent
    can still push that arm.
    Look down from above. Air must blow down onto the bench. Air blowing up
    means that propeller is on upside down.
    This script does not arm the Pixhawk.
    The safety switch must be pressed, so the LED is solid, or the
    Pixhawk will refuse the motor test.
    Only one motor is commanded at a time.
    Ctrl+C tells the current motor to stop. The copter was never armed.
    The log name is withPropsOnMotorTest#N.log.
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

FLIGHT_LOG_DIRECTORY = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "flightLog",
)
LOG_TITLE_PREFIX = "withPropsOnMotorTest#"

# Slow enough to watch the blade, and the same percent on every motor.
CHECK_THROTTLE_PERCENT = 20.0
SECONDS_AT_CHECK_THROTTLE = 10.0
PAUSE_BETWEEN_MOTORS_SECONDS = 3.0

# Throttle type 0 means the number is a percent, not a PWM pulse.
MOTOR_TEST_THROTTLE_IS_PERCENT = 0
# Zero means test one motor. Copter turns that into a count of one.
MOTOR_TEST_ONE_MOTOR = 0

# The motor-test command takes the Quad X test order, not the motor number.
# Looking down on the copter:
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
    """Return the next withPropsOnMotorTest#N.log path."""
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
    """Open the next numbered propeller-direction log."""
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


def connectToPixhawk():
    """Open pins 8 and 10 with DroneKit and wait for a heartbeat."""
    if not os.path.exists(PIXHAWK_PORT):
        raise RuntimeError(
            f"{PIXHAWK_PORT} is not on this computer. "
            "Run this script on the Pi. Pins 8 and 10 are there."
        )
    print()
    print(f"Opening {PIXHAWK_PORT} at {PIXHAWK_BAUD} baud.")
    # wait_ready stays off so this check does not download every parameter.
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


def spinOneMotor(vehicle, motorNumber, testSequence, cornerName, turnDirection):
    """Run one propeller at 20 percent long enough to see its direction."""
    print()
    print(f"Motor {motorNumber}, {cornerName}.")
    print(f"Looking down, this propeller should turn {turnDirection}.")
    print("Air should blow down. If it blows up, that propeller is upside down.")
    print("The other three propellers stay stopped.")
    print(
        f"Motor {motorNumber} | {CHECK_THROTTLE_PERCENT:.0f} percent | "
        f"{SECONDS_AT_CHECK_THROTTLE:.0f} seconds"
    )
    motorTestAnswer = waitForMotorTestAnswer(
        vehicle,
        testSequence,
        CHECK_THROTTLE_PERCENT,
        SECONDS_AT_CHECK_THROTTLE,
    )
    if motorTestAnswer is None:
        raise RuntimeError(f"No answer for motor {motorNumber}. That motor was not left running.")
    print(f"Motor test answer = {motorTestAnswer}")
    if motorTestAnswer != middleMan.mavlink.MAV_RESULT_ACCEPTED:
        raise RuntimeError(
            "Motor test was refused. Press the safety switch until the LED is solid, then try again."
        )

    stepStartedAtSeconds = time.monotonic()
    while time.monotonic() - stepStartedAtSeconds < SECONDS_AT_CHECK_THROTTLE:
        # A motor test sets the armed flag in the heartbeat. That is not a flight arm.
        if not heartbeatIsFresh(vehicle):
            raise RuntimeError("Pixhawk heartbeat was lost during the motor test.")
        time.sleep(0.2)

    stopMotor(vehicle, testSequence)
    print(f"Motor {motorNumber} stopped.")


def printRunSummary(motorOutcomes, overallResult, stopReason, logPath):
    """Write the result of every motor, including a failed or stopped run."""
    print()
    print("=" * 68)
    print("PROPELLER DIRECTION CHECK SUMMARY")
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
    """Spin motors 1, 2, 3, and 4 one at a time at 20 percent, then close the link."""
    originalStandardOutput = sys.stdout
    logFile = None
    logPath = None
    vehicle = None
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
        print("PROPELLER DIRECTION CHECK")
        print("==============================================")
        print()
        print("The propellers stay on for this check.")
        print("Hold the frame down. Clear hands and loose straps away from the blades.")
        print(f"Each motor spins alone at {CHECK_THROTTLE_PERCENT:.0f} percent for {SECONDS_AT_CHECK_THROTTLE:.0f} seconds.")
        print("Order: motor 1, motor 2, motor 3, motor 4.")
        print("Looking down from above:")
        print("  motor 1  front-right  counter-clockwise")
        print("  motor 2  back-left    counter-clockwise")
        print("  motor 3  front-left   clockwise")
        print("  motor 4  back-right   clockwise")
        print("Air from each propeller must blow downward.")
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

        vehicle = connectToPixhawk()
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
            spinOneMotor(vehicle, motorNumber, testSequence, cornerName, turnDirection)
            motorOutcomes.append((motorNumber, cornerName, "COMPLETED"))
            activeTestSequence = None
            activeMotorNumber = None
            activeCornerName = None
            if motorNumber != MOTOR_TEST_PLAN[-1][0]:
                print(f"Waiting {PAUSE_BETWEEN_MOTORS_SECONDS:.0f} seconds before the next motor.")
                time.sleep(PAUSE_BETWEEN_MOTORS_SECONDS)
        print()
        print("All four propellers were shown.")
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
