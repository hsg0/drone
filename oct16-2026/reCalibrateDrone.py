#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    Step-by-step terminal guide to recalibrate the drone's level attitude,
    compasses, EKF GPS settings, and pre-arm safety checks, saving a full
    log and outcome summary of every run in oct16-2026/calibrationLogs/.

WHY:
    Flight Log 49 showed that Compass 1 and Compass 2 disagreed by 74 degrees,
    motor current pulled the compass heading off during liftoff, and the
    Compass pre-arm check was skipped in ARMING_SKIPCHK. Saving a log of every
    successful or failed calibration run gives a permanent record of the
    parameters, offsets, fitness scores, and pre-arm results.

HOW:
    1. Open a timestamped log file in oct16-2026/calibrationLogs/ that records
       all terminal output, user choices, and the final outcome of each step.
    2. Connect to the Pixhawk on UART pins 8 and 10 (/dev/serial0 at 57600)
       and refuse to run if the motors are armed.
    3. Step 1: Restore EKF3 GPS sources, ground-station Land failsafe, and
       re-enable Compass and GPS pre-arm checks (keeping only RC skipped).
    4. Step 2: Inspect all compass slots on the Pixhawk and choose whether
       to use all detected compasses (External Primary + Internal Secondary)
       or only the external GPS mast compass(es).
    5. Step 3: Calibrate level horizon trim while the drone sits flat, and
       require a MAV_RESULT_ACCEPTED acknowledgement.
    6. Step 4: Run onboard compass calibration with a live 0% to 100%
       progress bar, retain the latest MAG_CAL_REPORT for each compass, and
       verify that the Pixhawk saved the new report offsets into parameters.
    7. Step 5: Print the new offsets, optionally reboot the Pixhawk, and
       verify pre-arm status using post-ACK SYS_STATUS enabled/health bits
       and STATUSTEXT messages.
    8. Write a complete CALIBRATION RUN OUTCOME SUMMARY to the log file on
       every run (whether successful, failed, or interrupted).

IMPORTANT:
    Run on the Pi with /home/drone/dronekit-venv/bin/python3.
    Keep propellers OFF or stay well clear, even though this script never arms.
    Do Step 4 (Compass Calibration) outside, away from cars, rebar, and metal tables.
    Make sure the arrow on the external GPS puck points straight forward
    in the same direction as the arrow on the Pixhawk.
"""

import math
import os
import struct
import sys
import time

from pymavlink import mavutil as middleMan


# --------------------------------------------------
# SETTINGS AND CONSTANTS
# --------------------------------------------------

PIXHAWK_PORT = "/dev/serial0"
PIXHAWK_BAUD = 57600
GROUND_STATION_SYSTEM_ID = 255

CALIBRATION_LOG_DIRECTORY = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "calibrationLogs",
)

HEARTBEAT_TIMEOUT_SECONDS = 15.0
PARAMETER_TIMEOUT_SECONDS = 5.0
COMMAND_ACK_TIMEOUT_SECONDS = 5.0
COMPASS_CALIBRATION_TIMEOUT_SECONDS = 180.0
DRAIN_OLD_PACKETS_TIMEOUT_SECONDS = 1.5
REBOOT_WAIT_SECONDS = 12.0
MESSAGE_INTERVAL_MICROSECONDS = 100000
EXTRA3_STREAM_RATE_HERTZ = 10
PARAMETER_TOLERANCE = 0.01
OFFSET_MATCH_TOLERANCE_MILLIGAUSS = 1.0
MATRIX_MATCH_TOLERANCE = 0.02

# ARMING_SKIPCHK bit 6 (value 64) skips only RC Channels so the Pi can arm without a radio.
# Clearing bits 2 (Compass = 4), 3 (GPS lock = 8), and 12 (GPS Config = 4096) turns those safety checks back on.
ARMING_SKIP_RC_ONLY_VALUE = 64

# Copter 4.7 EKF3 source values: 3 = GPS, 1 = Barometer, 1 = Compass.
EKF_SOURCE_GPS_VALUE = 3
EKF_SOURCE_BARO_VALUE = 1
EKF_SOURCE_COMPASS_VALUE = 1

# Copter 4.7 GCS failsafe: 5 = Land always.
GROUND_STATION_FAILSAFE_LAND_VALUE = 5
PREFERRED_FAILSAFE_TIMEOUT_SECONDS = 5.0
CONTINUE_PILOT_MODE_ON_FAILSAFE_BIT = 16

# Compass auto-orientation: 2 = Check and fix orientation automatically during calibration.
COMPASS_AUTO_ROTATION_VALUE = 2

# MAV_SYS_STATUS_PREARM_CHECK bit (1 << 28 = 268435456) in SYS_STATUS sensor bitmasks.
PREARM_CHECK_HEALTH_BIT = getattr(
    middleMan.mavlink,
    "MAV_SYS_STATUS_PREARM_CHECK",
    268435456,
)

INTEGER_PARAMETER_TYPES = (
    middleMan.mavlink.MAV_PARAM_TYPE_UINT8,
    middleMan.mavlink.MAV_PARAM_TYPE_INT8,
    middleMan.mavlink.MAV_PARAM_TYPE_UINT16,
    middleMan.mavlink.MAV_PARAM_TYPE_INT16,
    middleMan.mavlink.MAV_PARAM_TYPE_UINT32,
    middleMan.mavlink.MAV_PARAM_TYPE_INT32,
)

COMPASS_SLOT_DEFINITIONS = (
    {
        "compassId": 0,
        "deviceIdParam": "COMPASS_DEV_ID",
        "useParam": "COMPASS_USE",
        "externalParam": "COMPASS_EXTERNAL",
        "orientParam": "COMPASS_ORIENT",
        "offsetParams": ("COMPASS_OFS_X", "COMPASS_OFS_Y", "COMPASS_OFS_Z"),
        "diagParams": ("COMPASS_DIA_X", "COMPASS_DIA_Y", "COMPASS_DIA_Z"),
        "offdiagParams": ("COMPASS_ODI_X", "COMPASS_ODI_Y", "COMPASS_ODI_Z"),
    },
    {
        "compassId": 1,
        "deviceIdParam": "COMPASS_DEV_ID2",
        "useParam": "COMPASS_USE2",
        "externalParam": "COMPASS_EXTERN2",
        "orientParam": "COMPASS_ORIENT2",
        "offsetParams": ("COMPASS_OFS2_X", "COMPASS_OFS2_Y", "COMPASS_OFS2_Z"),
        "diagParams": ("COMPASS_DIA2_X", "COMPASS_DIA2_Y", "COMPASS_DIA2_Z"),
        "offdiagParams": ("COMPASS_ODI2_X", "COMPASS_ODI2_Y", "COMPASS_ODI2_Z"),
    },
    {
        "compassId": 2,
        "deviceIdParam": "COMPASS_DEV_ID3",
        "useParam": "COMPASS_USE3",
        "externalParam": "COMPASS_EXTERN3",
        "orientParam": "COMPASS_ORIENT3",
        "offsetParams": ("COMPASS_OFS3_X", "COMPASS_OFS3_Y", "COMPASS_OFS3_Z"),
        "diagParams": ("COMPASS_DIA3_X", "COMPASS_DIA3_Y", "COMPASS_DIA3_Z"),
        "offdiagParams": ("COMPASS_ODI3_X", "COMPASS_ODI3_Y", "COMPASS_ODI3_Z"),
    },
)

CALIBRATION_STATUS_NAMES = {
    middleMan.mavlink.MAG_CAL_NOT_STARTED: "NOT STARTED",
    middleMan.mavlink.MAG_CAL_WAITING_TO_START: "WAITING TO START",
    middleMan.mavlink.MAG_CAL_RUNNING_STEP_ONE: "STEP 1 (SPHERE FIT)",
    middleMan.mavlink.MAG_CAL_RUNNING_STEP_TWO: "STEP 2 (ELLIPSOID FIT)",
    middleMan.mavlink.MAG_CAL_SUCCESS: "SUCCESS",
    middleMan.mavlink.MAG_CAL_FAILED: "FAILED",
    middleMan.mavlink.MAG_CAL_FAILED_ORIENTATION: "FAILED ORIENTATION",
    middleMan.mavlink.MAG_CAL_FAILED_RADIUS: "FAILED RADIUS",
    middleMan.mavlink.MAG_CAL_FAILED_OFFSETS: "FAILED OFFSETS",
    middleMan.mavlink.MAG_CAL_FAILED_DIAG_SCALING: "FAILED SCALING",
    middleMan.mavlink.MAG_CAL_FAILED_RESIDUALS_HIGH: "FAILED HIGH RESIDUALS",
}


# --------------------------------------------------
# CALIBRATION LOGGING AND TERMINAL HELPERS
# --------------------------------------------------

class CalibrationLog:
    """Copy every printed line to the terminal and to the calibration log file."""

    def __init__(self, logFile, terminal):
        self.logFile = logFile
        self.terminal = terminal

    def write(self, text):
        self.terminal.write(text)
        self.terminal.flush()
        # Skip raw carriage-return progress redraws so the saved log file stays clean.
        if text.startswith("\r") and not text.endswith("\n"):
            return
        self.logFile.write(text)
        self.logFile.flush()

    def writeLogOnly(self, text):
        """Write a line only to the log file without cluttering the live progress bar."""
        self.logFile.write(text)
        self.logFile.flush()

    def flush(self):
        self.terminal.flush()
        self.logFile.flush()


def openCalibrationLog():
    """Create oct16-2026/calibrationLogs/ and open a timestamped log file for this run."""
    os.makedirs(CALIBRATION_LOG_DIRECTORY, exist_ok=True)
    scriptName = os.path.splitext(os.path.basename(__file__))[0]
    startedAtText = time.strftime("%Y-%m-%d-%H%M%S")
    logPath = os.path.join(
        CALIBRATION_LOG_DIRECTORY,
        f"{scriptName}-{startedAtText}.log",
    )
    logFile = open(logPath, "w", encoding="utf-8")
    calibrationLogger = CalibrationLog(logFile, sys.stdout)
    sys.stdout = calibrationLogger
    print(f"Calibration log file : {logPath}")
    print(f"Run started at       : {time.strftime('%Y-%m-%d %H:%M:%S %Z')}")
    return logFile, logPath, calibrationLogger


def createRunOutcomeRecord():
    """Create a dictionary tracking the outcome of each calibration step."""
    return {
        "startedAt": time.strftime("%Y-%m-%d %H:%M:%S %Z"),
        "finishedAt": None,
        "overallOutcome": "FAILED",
        "connection": "NOT STARTED",
        "step1CoreParameters": "NOT STARTED",
        "step2CompassSelection": "NOT STARTED",
        "step3LevelHorizon": "NOT STARTED",
        "step4CompassCalibration": "NOT STARTED",
        "step5PreArmVerification": "NOT STARTED",
        "failureReason": None,
    }


def printRunOutcomeSummary(runOutcome, logPath):
    """Print the final outcome of every step to both the terminal and the log file."""
    runOutcome["finishedAt"] = time.strftime("%Y-%m-%d %H:%M:%S %Z")
    print()
    print("=" * 68)
    print("CALIBRATION RUN OUTCOME SUMMARY")
    print("=" * 68)
    print(f"Overall Outcome         : {runOutcome['overallOutcome']}")
    print(f"Started At              : {runOutcome['startedAt']}")
    print(f"Finished At             : {runOutcome['finishedAt']}")
    print(f"Pixhawk Connection      : {runOutcome['connection']}")
    print(f"Step 1 (EKF & Pre-Arm)  : {runOutcome['step1CoreParameters']}")
    print(f"Step 2 (Compass Setup)  : {runOutcome['step2CompassSelection']}")
    print(f"Step 3 (Level Horizon)  : {runOutcome['step3LevelHorizon']}")
    print(f"Step 4 (Compass Cal)    : {runOutcome['step4CompassCalibration']}")
    print(f"Step 5 (Pre-Arm Verify) : {runOutcome['step5PreArmVerification']}")
    if runOutcome["failureReason"]:
        print(f"Failure / Stop Reason   : {runOutcome['failureReason']}")
    if logPath is not None:
        print(f"Saved Calibration Log   : {logPath}")
    print("=" * 68)


def printStepBanner(stepNumber, stepTitle):
    """Print a clear section banner in the terminal."""
    print()
    print("=" * 68)
    print(f"STEP {stepNumber}: {stepTitle}")
    print("=" * 68)


def askChoice(promptText, validChoices, defaultChoice):
    """Ask the user for a single-word or single-letter choice and log the answer."""
    choiceText = "/".join(validChoices)
    while True:
        fullPrompt = f"{promptText} [{choiceText}] (default {defaultChoice}): "
        answer = input(fullPrompt).strip().upper()
        chosenValue = defaultChoice if answer == "" else answer
        if chosenValue in validChoices:
            if isinstance(sys.stdout, CalibrationLog):
                sys.stdout.writeLogOnly(f"{fullPrompt}{chosenValue}\n")
            return chosenValue
        print(f"Please type one of: {choiceText}")


def progressBarText(percentage):
    """Return a 25-character ASCII progress bar."""
    clampedPercentage = max(0, min(100, int(percentage)))
    filledCount = int(round((clampedPercentage / 100.0) * 25))
    return "[" + ("#" * filledCount) + ("-" * (25 - filledCount)) + f"] {clampedPercentage:3d}%"


# --------------------------------------------------
# MAVLINK CONNECTION AND PARAMETER HELPERS
# --------------------------------------------------

def connectToPixhawk():
    """Open /dev/serial0, wait for a heartbeat, and verify the copter is disarmed."""
    if not os.path.exists(PIXHAWK_PORT):
        raise RuntimeError(f"{PIXHAWK_PORT} was not found. Run this script on the Raspberry Pi.")

    print(f"Connecting to Pixhawk on {PIXHAWK_PORT} at {PIXHAWK_BAUD} baud...")
    connection = middleMan.mavlink_connection(
        PIXHAWK_PORT,
        baud=PIXHAWK_BAUD,
        source_system=GROUND_STATION_SYSTEM_ID,
    )
    try:
        heartbeat = connection.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT_SECONDS)
        if heartbeat is None:
            raise RuntimeError("No heartbeat from the Pixhawk. Check power and pins 8, 10, and 6.")

        isArmed = bool(heartbeat.base_mode & middleMan.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if isArmed:
            raise RuntimeError("The copter is currently ARMED. Disarm before calibrating.")

        print(
            f"Connected to Pixhawk (system {connection.target_system}, "
            f"component {connection.target_component}). Motors are disarmed."
        )
        requestCalibrationStreams(connection)
        return connection
    except BaseException:
        connection.close()
        raise


def requestCalibrationStreams(connection):
    """Enable attitude, IMU, compass, status text, system status, and calibration streams."""
    targetSystem = connection.target_system
    targetComponent = connection.target_component

    # MAG_CAL_PROGRESS and MAG_CAL_REPORT are sent inside the EXTRA3 stream.
    connection.mav.request_data_stream_send(
        targetSystem,
        targetComponent,
        middleMan.mavlink.MAV_DATA_STREAM_EXTRA3,
        EXTRA3_STREAM_RATE_HERTZ,
        1,
    )

    messageNames = (
        "ATTITUDE",
        "RAW_IMU",
        "SCALED_IMU2",
        "EKF_STATUS_REPORT",
        "SYS_STATUS",
        "STATUSTEXT",
        "MAG_CAL_PROGRESS",
        "MAG_CAL_REPORT",
    )
    for messageName in messageNames:
        messageId = getattr(middleMan.mavlink, f"MAVLINK_MSG_ID_{messageName}", None)
        if messageId is None:
            continue
        connection.mav.command_long_send(
            targetSystem,
            targetComponent,
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


def parameterNameText(rawParameterId):
    """Clean a MAVLink param_id into a normal Python string."""
    if isinstance(rawParameterId, bytes):
        return rawParameterId.decode("ascii", errors="ignore").rstrip("\x00")
    return str(rawParameterId).rstrip("\x00")


def readParameterMessage(connection, parameterName):
    """Read one PARAM_VALUE message so both its value and param_type are known."""
    connection.mav.param_request_read_send(
        connection.target_system,
        connection.target_component,
        parameterName.encode("ascii"),
        -1,
    )
    deadline = time.monotonic() + PARAMETER_TIMEOUT_SECONDS
    while time.monotonic() < deadline:
        message = connection.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if message is None:
            continue
        if parameterNameText(message.param_id) == parameterName:
            return message
    raise RuntimeError(f"Timed out reading {parameterName}.")


def readParameter(connection, parameterName):
    """Read one parameter's numeric value from the Pixhawk."""
    message = readParameterMessage(connection, parameterName)
    return float(message.param_value)


def writeParameterAndConfirm(connection, parameterName, wantedValue):
    """Write one parameter using its native MAVLink param_type and verify it saved."""
    existingMessage = readParameterMessage(connection, parameterName)
    parameterType = getattr(
        existingMessage,
        "param_type",
        middleMan.mavlink.MAV_PARAM_TYPE_REAL32,
    )

    # ArduPilot sends integer and float parameter values as IEEE-754 floats on the wire
    # while tagging the parameter's native type in param_type.
    isIntegerParameter = parameterType in INTEGER_PARAMETER_TYPES
    if isIntegerParameter:
        targetInteger = int(round(wantedValue))
        wireValue = float(targetInteger)
    else:
        targetInteger = None
        wireValue = float(struct.unpack("f", struct.pack("f", float(wantedValue)))[0])

    connection.mav.param_set_send(
        connection.target_system,
        connection.target_component,
        parameterName.encode("ascii"),
        wireValue,
        parameterType,
    )
    deadline = time.monotonic() + PARAMETER_TIMEOUT_SECONDS
    while time.monotonic() < deadline:
        message = connection.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if message is None:
            continue
        if parameterNameText(message.param_id) == parameterName:
            confirmedValue = float(message.param_value)
            if isIntegerParameter:
                if int(round(confirmedValue)) == targetInteger:
                    return float(int(round(confirmedValue)))
            elif abs(confirmedValue - wireValue) < PARAMETER_TOLERANCE:
                return confirmedValue
    raise RuntimeError(f"Could not confirm {parameterName} = {wantedValue}.")


def drainMessagesWithTimeout(connection, messageTypes):
    """Discard queued messages of the given types, bounded by a strict timeout."""
    deadline = time.monotonic() + DRAIN_OLD_PACKETS_TIMEOUT_SECONDS
    while time.monotonic() < deadline:
        message = connection.recv_match(type=messageTypes, blocking=False)
        if message is None:
            break


def isExternalCompassFlag(externalFlagValue):
    """True when COMPASS_EXTERNAL / COMPASS_EXTERN* is 1 (External) or 2 (Forced External)."""
    return int(round(externalFlagValue)) in (1, 2)


def compassLocationLabel(externalFlagValue):
    """Return a clear label for COMPASS_EXTERNAL values 0, 1, and 2."""
    flagInteger = int(round(externalFlagValue))
    if flagInteger == 1:
        return "External GPS Mast"
    if flagInteger == 2:
        return "External GPS Mast (Forced)"
    return "Internal Board"


def readCompassSlots(connection):
    """Read hardware ID, enabled flag, external flag, and orientation for all 3 compass slots."""
    slots = []
    for slotDefinition in COMPASS_SLOT_DEFINITIONS:
        deviceId = int(round(readParameter(connection, slotDefinition["deviceIdParam"])))
        useFlag = int(round(readParameter(connection, slotDefinition["useParam"])))
        externalFlag = int(round(readParameter(connection, slotDefinition["externalParam"])))
        orientValue = int(round(readParameter(connection, slotDefinition["orientParam"])))
        slots.append(
            {
                **slotDefinition,
                "deviceId": deviceId,
                "isDetected": deviceId != 0,
                "useFlag": useFlag,
                "externalFlag": externalFlag,
                "isExternal": isExternalCompassFlag(externalFlag),
                "locationLabel": compassLocationLabel(externalFlag),
                "orientValue": orientValue,
            }
        )
    return slots


def readAttitudeAndCompasses(connection):
    """Collect fresh attitude, EKF status, and magnetometer samples from the Pixhawk."""
    attitudeMessage = None
    ekfStatusMessage = None
    rawImuMessage = None
    secondImuMessage = None
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        message = connection.recv_match(
            type=["ATTITUDE", "EKF_STATUS_REPORT", "RAW_IMU", "SCALED_IMU2"],
            blocking=True,
            timeout=0.3,
        )
        if message is None:
            continue
        messageType = message.get_type()
        if messageType == "ATTITUDE":
            attitudeMessage = message
        elif messageType == "EKF_STATUS_REPORT":
            ekfStatusMessage = message
        elif messageType == "RAW_IMU":
            rawImuMessage = message
        elif messageType == "SCALED_IMU2":
            secondImuMessage = message
        if attitudeMessage and ekfStatusMessage and rawImuMessage and secondImuMessage:
            break
    return attitudeMessage, ekfStatusMessage, rawImuMessage, secondImuMessage


def printCurrentSensorSummary(connection):
    """Print live EKF attitude/heading, compass variance, and sensor magnetic field strength."""
    attitudeMessage, ekfStatusMessage, rawImuMessage, secondImuMessage = readAttitudeAndCompasses(
        connection
    )
    externalOne = readParameter(connection, "COMPASS_EXTERNAL")
    externalTwo = readParameter(connection, "COMPASS_EXTERN2")

    if attitudeMessage is not None:
        rollDegrees = math.degrees(attitudeMessage.roll)
        pitchDegrees = math.degrees(attitudeMessage.pitch)
        yawDegrees = math.degrees(attitudeMessage.yaw)
        compassNote = ""
        if ekfStatusMessage is not None:
            compassNote = f" | EKF Compass Variance {ekfStatusMessage.compass_variance:.3f}"
        print(
            f"Live EKF Attitude : Roll {rollDegrees:+.2f} deg | "
            f"Pitch {pitchDegrees:+.2f} deg | Fused Heading {yawDegrees:+.1f} deg{compassNote}"
        )

    if rawImuMessage is not None:
        compassOneStrength = math.hypot(
            rawImuMessage.xmag,
            rawImuMessage.ymag,
            rawImuMessage.zmag,
        )
        labelOne = compassLocationLabel(externalOne)
        print(
            f"Compass 1 ({labelOne:<26}) : X={rawImuMessage.xmag:4d} Y={rawImuMessage.ymag:4d} "
            f"Z={rawImuMessage.zmag:4d} | Field Strength={compassOneStrength:5.0f} mG"
        )

    if secondImuMessage is not None:
        compassTwoStrength = math.hypot(
            secondImuMessage.xmag,
            secondImuMessage.ymag,
            secondImuMessage.zmag,
        )
        labelTwo = compassLocationLabel(externalTwo)
        print(
            f"Compass 2 ({labelTwo:<26}) : X={secondImuMessage.xmag:4d} Y={secondImuMessage.ymag:4d} "
            f"Z={secondImuMessage.zmag:4d} | Field Strength={compassTwoStrength:5.0f} mG"
        )


# --------------------------------------------------
# STEP 1: FIX EKF GPS, FAILSAFE, AND PRE-ARM PARAMETERS
# --------------------------------------------------

def configureCoreParameters(connection, runOutcome):
    """Step 1: Ensure EKF3 uses GPS, Land failsafe is active, and Compass/GPS checks are on."""
    printStepBanner(1, "CHECK AND FIX EKF, FAILSAFE, AND PRE-ARM PARAMETERS")

    currentSkipChecks = int(round(readParameter(connection, "ARMING_SKIPCHK")))
    currentPosXY = int(round(readParameter(connection, "EK3_SRC1_POSXY")))
    currentVelXY = int(round(readParameter(connection, "EK3_SRC1_VELXY")))
    currentPosZ = int(round(readParameter(connection, "EK3_SRC1_POSZ")))
    currentYaw = int(round(readParameter(connection, "EK3_SRC1_YAW")))
    currentGcsFailsafe = int(round(readParameter(connection, "FS_GCS_ENABLE")))
    currentFailsafeOptions = int(round(readParameter(connection, "FS_OPTIONS")))

    print(f"Current ARMING_SKIPCHK : {currentSkipChecks} (64 = skip RC only; 4172 = skipped Compass & GPS)")
    print(
        f"Current EKF3 Sources   : POSXY={currentPosXY} (3=GPS), "
        f"VELXY={currentVelXY} (3=GPS), POSZ={currentPosZ} (1=Baro), YAW={currentYaw} (1=Compass)"
    )
    print(
        f"Current GCS Failsafe   : FS_GCS_ENABLE={currentGcsFailsafe} (5=Land), "
        f"FS_OPTIONS={currentFailsafeOptions}"
    )

    choice = askChoice(
        "Apply recommended EKF GPS, Land failsafe, and Compass/GPS pre-arm settings?",
        ("Y", "N"),
        "Y",
    )
    if choice != "Y":
        print("Skipped Step 1 parameter updates.")
        runOutcome["step1CoreParameters"] = "SKIPPED BY USER"
        return

    writeParameterAndConfirm(connection, "EK3_SRC1_POSXY", EKF_SOURCE_GPS_VALUE)
    writeParameterAndConfirm(connection, "EK3_SRC1_VELXY", EKF_SOURCE_GPS_VALUE)
    writeParameterAndConfirm(connection, "EK3_SRC1_POSZ", EKF_SOURCE_BARO_VALUE)
    writeParameterAndConfirm(connection, "EK3_SRC1_YAW", EKF_SOURCE_COMPASS_VALUE)
    writeParameterAndConfirm(connection, "ARMING_SKIPCHK", ARMING_SKIP_RC_ONLY_VALUE)
    writeParameterAndConfirm(connection, "FS_GCS_ENABLE", GROUND_STATION_FAILSAFE_LAND_VALUE)
    writeParameterAndConfirm(connection, "FS_GCS_TIMEOUT", PREFERRED_FAILSAFE_TIMEOUT_SECONDS)

    cleanedFailsafeOptions = currentFailsafeOptions & ~CONTINUE_PILOT_MODE_ON_FAILSAFE_BIT
    writeParameterAndConfirm(connection, "FS_OPTIONS", cleanedFailsafeOptions)
    writeParameterAndConfirm(connection, "COMPASS_AUTO_ROT", COMPASS_AUTO_ROTATION_VALUE)

    print("Saved: EKF3 is set to GPS + Baro + Compass, GCS failsafe is Land (5),")
    print("and ARMING_SKIPCHK is 64 (Compass and GPS pre-arm checks are now ON).")
    runOutcome["step1CoreParameters"] = (
        "SUCCESS (EK3_SRC1_POSXY=3, FS_GCS_ENABLE=5, ARMING_SKIPCHK=64)"
    )


# --------------------------------------------------
# STEP 2: VERIFY COMPASS ORDER AND CHOOSE SETUP
# --------------------------------------------------

def configureCompassSelection(connection, runOutcome):
    """Step 2: Inspect all compass slots dynamically and return the enabled compass IDs."""
    printStepBanner(2, "VERIFY COMPASS HARDWARE AND SELECT PRIMARY / SECONDARY SETUP")

    slots = readCompassSlots(connection)
    priorityOneId = int(round(readParameter(connection, "COMPASS_PRIO1_ID")))
    priorityTwoId = int(round(readParameter(connection, "COMPASS_PRIO2_ID")))

    detectedSlots = [slot for slot in slots if slot["isDetected"]]
    externalSlots = [slot for slot in detectedSlots if slot["isExternal"]]

    if not detectedSlots:
        runOutcome["step2CompassSelection"] = "FAILED (No compass hardware detected)"
        raise RuntimeError("No hardware compasses were detected on the Pixhawk.")

    print(f"Compass Priority Order: PRIO1_ID={priorityOneId}, PRIO2_ID={priorityTwoId}")
    for slot in detectedSlots:
        compassNumber = slot["compassId"] + 1
        print(
            f"  Compass {compassNumber} (DEV_ID={slot['deviceId']}) : {slot['locationLabel']} | "
            f"{slot['useParam']}={slot['useFlag']}, "
            f"{slot['externalParam']}={slot['externalFlag']}, "
            f"{slot['orientParam']}={slot['orientValue']}"
        )

    print()
    print("How Pixhawk sensors work:")
    print("  - Internal IMU (gyro + accelerometer) and Barometer inside the Pixhawk")
    print("    are ALWAYS your primary motion and altitude sensors.")
    print("  - For the COMPASS, the external GPS mast sensor sits far away from the")
    print("    high-current ESC and battery wires, so ArduPilot uses the External")
    print("    mast compass as Primary #1 and the Internal board compass as Secondary #2.")
    print()
    print("  Option 1 (All Detected - Primary + Secondary): Enable and calibrate all detected compasses.")
    print("  Option 2 (External GPS Mast Only)            : Enable and calibrate only external compass(es).")

    choice = askChoice("Select compass mode (1 = All detected, 2 = External only)", ("1", "2"), "1")
    if choice == "2":
        if not externalSlots:
            runOutcome["step2CompassSelection"] = "FAILED (No external compass slot found)"
            raise RuntimeError(
                "No compass slot is marked external (COMPASS_EXTERNAL / COMPASS_EXTERN* = 1 or 2). "
                "Cannot enable external-only mode."
            )
        selectedIds = []
        for slot in slots:
            shouldEnable = 1 if (slot["isDetected"] and slot["isExternal"]) else 0
            writeParameterAndConfirm(connection, slot["useParam"], shouldEnable)
            if shouldEnable == 1:
                selectedIds.append(slot["compassId"])
        enabledNames = ", ".join(
            f"Compass {cid + 1} ({slots[cid]['locationLabel']})" for cid in selectedIds
        )
        print(f"Saved: Enabled only external compass(es): {enabledNames}.")
        runOutcome["step2CompassSelection"] = f"SUCCESS (External only: {enabledNames})"
        return selectedIds

    selectedIds = []
    for slot in slots:
        shouldEnable = 1 if slot["isDetected"] else 0
        writeParameterAndConfirm(connection, slot["useParam"], shouldEnable)
        if shouldEnable == 1:
            selectedIds.append(slot["compassId"])
    enabledNames = ", ".join(
        f"Compass {cid + 1} ({slots[cid]['locationLabel']})" for cid in selectedIds
    )
    print(f"Saved: Enabled all detected compasses: {enabledNames}.")
    runOutcome["step2CompassSelection"] = f"SUCCESS (All detected: {enabledNames})"
    return selectedIds


# --------------------------------------------------
# STEP 3: LEVEL HORIZON CALIBRATION
# --------------------------------------------------

def calibrateLevelHorizon(connection, runOutcome):
    """Step 3: Calibrate level horizon trim and require MAV_RESULT_ACCEPTED."""
    printStepBanner(3, "LEVEL HORIZON CALIBRATION (IMU TRIM)")

    print("Place the drone on a flat, level surface and do not touch it.")
    print("Check that the arrow on the GPS puck and the arrow on the Pixhawk both point forward.")
    choice = askChoice("Calibrate level horizon now?", ("Y", "S"), "Y")
    if choice != "Y":
        print("Skipped level horizon calibration.")
        runOutcome["step3LevelHorizon"] = "SKIPPED BY USER"
        return True

    drainMessagesWithTimeout(connection, ["COMMAND_ACK", "STATUSTEXT"])

    # param5 = 2 runs board-level trim (AHRS_TRIM_X / AHRS_TRIM_Y) without erasing 3D accel scales.
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        middleMan.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
        0,
        0,
        0,
        0,
        0,
        middleMan.mavlink.PREFLIGHT_CALIBRATION_ACCELEROMETER_TRIM,
        0,
        0,
    )

    accepted = False
    ackResult = None
    deadline = time.monotonic() + COMMAND_ACK_TIMEOUT_SECONDS
    while time.monotonic() < deadline:
        message = connection.recv_match(
            type=["COMMAND_ACK", "STATUSTEXT"],
            blocking=True,
            timeout=0.5,
        )
        if message is None:
            continue
        if message.get_type() == "STATUSTEXT":
            print(f"  Pixhawk: {message.text}")
        elif (
            message.get_type() == "COMMAND_ACK"
            and message.command == middleMan.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION
        ):
            ackResult = int(message.result)
            accepted = ackResult == middleMan.mavlink.MAV_RESULT_ACCEPTED
            break

    if not accepted:
        runOutcome["step3LevelHorizon"] = f"FAILED (ACK result={ackResult})"
        raise RuntimeError(
            f"Level horizon calibration was not accepted by the Pixhawk (ACK result={ackResult})."
        )

    print("Level horizon calibration accepted by the Pixhawk.")
    time.sleep(1.0)
    trimRollDegrees = math.degrees(readParameter(connection, "AHRS_TRIM_X"))
    trimPitchDegrees = math.degrees(readParameter(connection, "AHRS_TRIM_Y"))
    print(
        f"Saved Level Trim  : AHRS_TRIM_X={trimRollDegrees:+.2f} deg, "
        f"AHRS_TRIM_Y={trimPitchDegrees:+.2f} deg"
    )
    printCurrentSensorSummary(connection)
    runOutcome["step3LevelHorizon"] = (
        f"SUCCESS (AHRS_TRIM_X={trimRollDegrees:+.2f} deg, AHRS_TRIM_Y={trimPitchDegrees:+.2f} deg)"
    )
    return True


# --------------------------------------------------
# STEP 4: LIVE ONBOARD COMPASS CALIBRATION
# --------------------------------------------------

def compassReportMatchesPixhawkParameters(connection, compassId, report):
    """Verify that the Pixhawk's stored offset and matrix parameters match the new MAG_CAL_REPORT."""
    slotDefinition = COMPASS_SLOT_DEFINITIONS[compassId]
    storedOffsets = [
        readParameter(connection, paramName)
        for paramName in slotDefinition["offsetParams"]
    ]
    storedDiagonals = [
        readParameter(connection, paramName)
        for paramName in slotDefinition["diagParams"]
    ]
    storedOffDiagonals = [
        readParameter(connection, paramName)
        for paramName in slotDefinition["offdiagParams"]
    ]

    reportOffsets = [float(report.ofs_x), float(report.ofs_y), float(report.ofs_z)]
    reportDiagonals = [float(report.diag_x), float(report.diag_y), float(report.diag_z)]
    reportOffDiagonals = [
        float(report.offdiag_x),
        float(report.offdiag_y),
        float(report.offdiag_z),
    ]

    offsetsMatch = all(
        abs(storedValue - reportValue) <= OFFSET_MATCH_TOLERANCE_MILLIGAUSS
        for storedValue, reportValue in zip(storedOffsets, reportOffsets)
    )
    diagonalsMatch = all(
        abs(storedValue - reportValue) <= MATRIX_MATCH_TOLERANCE
        for storedValue, reportValue in zip(storedDiagonals, reportDiagonals)
    )
    offDiagonalsMatch = all(
        abs(storedValue - reportValue) <= MATRIX_MATCH_TOLERANCE
        for storedValue, reportValue in zip(storedOffDiagonals, reportOffDiagonals)
    )
    return offsetsMatch and diagonalsMatch and offDiagonalsMatch


def acceptAndRefreshCompassReports(connection, compassBitmask, expectedCompassIds, reportsByCompass):
    """Send MAV_CMD_DO_ACCEPT_MAG_CAL and collect updated MAG_CAL_REPORT messages."""
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        middleMan.mavlink.MAV_CMD_DO_ACCEPT_MAG_CAL,
        0,
        compassBitmask,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    deadline = time.monotonic() + COMMAND_ACK_TIMEOUT_SECONDS
    while time.monotonic() < deadline:
        message = connection.recv_match(
            type=["COMMAND_ACK", "MAG_CAL_REPORT"],
            blocking=True,
            timeout=0.5,
        )
        if message is None:
            continue
        if message.get_type() == "MAG_CAL_REPORT":
            compassId = int(message.compass_id)
            if compassId in expectedCompassIds:
                reportsByCompass[compassId] = message
        elif (
            message.get_type() == "COMMAND_ACK"
            and message.command == middleMan.mavlink.MAV_CMD_DO_ACCEPT_MAG_CAL
        ):
            if message.result != middleMan.mavlink.MAV_RESULT_ACCEPTED:
                print(f"  Warning: MAV_CMD_DO_ACCEPT_MAG_CAL returned result={message.result}.")


def calibrateCompasses(connection, expectedCompassIds, runOutcome):
    """Step 4: Run onboard magnetometer sphere calibration and verify all expected compasses."""
    printStepBanner(4, "ONBOARD COMPASS CALIBRATION (360-DEGREE ROTATION)")

    compassBitmask = 0
    for compassId in expectedCompassIds:
        compassBitmask |= 1 << compassId

    compassListText = ", ".join(f"Compass {compassId + 1}" for compassId in expectedCompassIds)
    print(f"Compasses to calibrate: {compassListText} (bitmask={compassBitmask})")
    print("IMPORTANT BEFORE YOU START:")
    print("  1. Stand outside, away from cars, steel tables, and concrete rebar.")
    print("  2. Once calibration starts, pick up the drone and slowly rotate it through:")
    print("       - Flat and level (spin 360 degrees)")
    print("       - Nose pointing down (spin 360 degrees)")
    print("       - Nose pointing up (spin 360 degrees)")
    print("       - Rolled onto its left side (spin 360 degrees)")
    print("       - Rolled onto its right side (spin 360 degrees)")
    print("       - Upside down (spin 360 degrees)")
    print("  3. Keep rotating smoothly until EVERY compass reaches 100%.")
    print()

    choice = askChoice("Ready to start spinning the drone for compass calibration?", ("Y", "S"), "Y")
    if choice != "Y":
        print("Skipped compass calibration.")
        runOutcome["step4CompassCalibration"] = "SKIPPED BY USER"
        return None

    drainMessagesWithTimeout(connection, ["MAG_CAL_PROGRESS", "MAG_CAL_REPORT", "COMMAND_ACK"])

    # param1 = explicit bitmask of expected compasses, param2 = 1 (retry), param3 = 1 (autosave).
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        middleMan.mavlink.MAV_CMD_DO_START_MAG_CAL,
        0,
        compassBitmask,
        1,
        1,
        0,
        0,
        0,
        0,
    )

    print("Starting compass calibration... Begin rotating the drone in all directions.")
    progressByCompass = {compassId: 0 for compassId in expectedCompassIds}
    loggedMilestoneByCompass = {compassId: -1 for compassId in expectedCompassIds}
    reportsByCompass = {}
    printedReportSignatures = {}
    lastLineText = ""
    startedAtSeconds = time.monotonic()

    try:
        while time.monotonic() - startedAtSeconds < COMPASS_CALIBRATION_TIMEOUT_SECONDS:
            message = connection.recv_match(
                type=["MAG_CAL_PROGRESS", "MAG_CAL_REPORT", "STATUSTEXT", "COMMAND_ACK"],
                blocking=True,
                timeout=0.5,
            )
            if message is None:
                continue

            messageType = message.get_type()
            if (
                messageType == "COMMAND_ACK"
                and message.command == middleMan.mavlink.MAV_CMD_DO_START_MAG_CAL
            ):
                if message.result != middleMan.mavlink.MAV_RESULT_ACCEPTED:
                    runOutcome["step4CompassCalibration"] = (
                        f"FAILED (Pixhawk rejected start command, result={message.result})"
                    )
                    raise RuntimeError(
                        f"Pixhawk rejected MAV_CMD_DO_START_MAG_CAL (result={message.result})."
                    )

            elif messageType == "STATUSTEXT":
                sys.stdout.write("\n")
                print(f"  Pixhawk: {message.text}")
                lastLineText = ""

            elif messageType == "MAG_CAL_PROGRESS":
                compassId = int(message.compass_id)
                percentage = int(message.completion_pct)
                progressByCompass[compassId] = percentage
                statusName = CALIBRATION_STATUS_NAMES.get(
                    message.cal_status,
                    f"STATUS {message.cal_status}",
                )
                parts = [
                    f"C{cid + 1}: {progressBarText(progressByCompass.get(cid, 0))}"
                    for cid in expectedCompassIds
                ]
                lineText = "\r  " + " | ".join(parts) + f" | {statusName:<22}"
                if lineText != lastLineText:
                    sys.stdout.write(lineText)
                    sys.stdout.flush()
                    lastLineText = lineText

                # Record 25% progress milestones cleanly into the log file.
                milestoneBucket = (percentage // 25) * 25
                if (
                    compassId in loggedMilestoneByCompass
                    and milestoneBucket > loggedMilestoneByCompass[compassId]
                ):
                    loggedMilestoneByCompass[compassId] = milestoneBucket
                    if isinstance(sys.stdout, CalibrationLog):
                        sys.stdout.writeLogOnly(
                            f"  [Progress Log] Compass {compassId + 1}: "
                            f"{percentage}% ({statusName})\n"
                        )

            elif messageType == "MAG_CAL_REPORT":
                compassId = int(message.compass_id)
                if compassId in expectedCompassIds:
                    # Always store the newest report so updated status or autosaved flags are retained.
                    reportsByCompass[compassId] = message
                    signature = (int(message.cal_status), int(message.autosaved))
                    if printedReportSignatures.get(compassId) != signature:
                        printedReportSignatures[compassId] = signature
                        sys.stdout.write("\n")
                        statusName = CALIBRATION_STATUS_NAMES.get(
                            message.cal_status,
                            f"STATUS {message.cal_status}",
                        )
                        print(
                            f"  Compass {compassId + 1} Report: {statusName} | "
                            f"fitness={message.fitness:.2f} | "
                            f"offsets=({message.ofs_x:.1f}, {message.ofs_y:.1f}, {message.ofs_z:.1f}) | "
                            f"autosaved={bool(message.autosaved)}"
                        )
                        lastLineText = ""

                # Finish once every expected compass has a terminal report.
                if all(cid in reportsByCompass for cid in expectedCompassIds):
                    break

    except KeyboardInterrupt:
        sys.stdout.write("\n")
        print("Cancelling compass calibration...")
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            middleMan.mavlink.MAV_CMD_DO_CANCEL_MAG_CAL,
            0,
            compassBitmask,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        runOutcome["step4CompassCalibration"] = "CANCELLED BY USER (Ctrl+C)"
        return False

    sys.stdout.write("\n")
    missingCompassIds = [cid for cid in expectedCompassIds if cid not in reportsByCompass]
    if missingCompassIds:
        missingNames = ", ".join(f"Compass {cid + 1}" for cid in missingCompassIds)
        print(f"Compass calibration timed out before receiving reports for: {missingNames}.")
        runOutcome["step4CompassCalibration"] = f"FAILED (Timed out waiting for {missingNames})"
        return False

    # If any compass succeeded but did not autosave yet, send MAV_CMD_DO_ACCEPT_MAG_CAL and refresh reports.
    needsExplicitAccept = any(
        report.cal_status == middleMan.mavlink.MAG_CAL_SUCCESS and int(report.autosaved) != 1
        for report in reportsByCompass.values()
    )
    if needsExplicitAccept:
        acceptAndRefreshCompassReports(
            connection,
            compassBitmask,
            expectedCompassIds,
            reportsByCompass,
        )

    perCompassSummaries = []
    for compassId in expectedCompassIds:
        report = reportsByCompass[compassId]
        statusName = CALIBRATION_STATUS_NAMES.get(
            report.cal_status,
            f"STATUS {report.cal_status}",
        )
        if report.cal_status != middleMan.mavlink.MAG_CAL_SUCCESS:
            print(f"Compass {compassId + 1} failed calibration ({statusName}).")
            runOutcome["step4CompassCalibration"] = (
                f"FAILED (Compass {compassId + 1}: {statusName})"
            )
            return False

        parametersMatchReport = compassReportMatchesPixhawkParameters(
            connection,
            compassId,
            report,
        )
        if not parametersMatchReport:
            print(
                f"Compass {compassId + 1} report offsets did not match the parameters "
                "stored on the Pixhawk."
            )
            runOutcome["step4CompassCalibration"] = (
                f"FAILED (Compass {compassId + 1} parameters did not match report)"
            )
            return False

        perCompassSummaries.append(
            f"C{compassId + 1} offsets=({report.ofs_x:+.1f},{report.ofs_y:+.1f},{report.ofs_z:+.1f}) "
            f"fit={report.fitness:.2f}"
        )

    print("All expected compasses succeeded and verified new offsets in Pixhawk parameters.")
    runOutcome["step4CompassCalibration"] = "SUCCESS (" + "; ".join(perCompassSummaries) + ")"
    return True


# --------------------------------------------------
# STEP 5: VERIFY OFFSETS, RUN PRE-ARM CHECKS, AND REBOOT
# --------------------------------------------------

def readPrearmHealthAndMessages(connection):
    """Run MAV_CMD_RUN_PREARM_CHECKS and check post-ACK SYS_STATUS enabled/health bits and STATUSTEXT."""
    drainMessagesWithTimeout(connection, ["STATUSTEXT", "COMMAND_ACK", "SYS_STATUS"])

    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        middleMan.mavlink.MAV_CMD_RUN_PREARM_CHECKS,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )

    commandAccepted = False
    prearmBitEnabled = None
    prearmBitHealthy = None
    prearmMessages = []
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        message = connection.recv_match(
            type=["COMMAND_ACK", "STATUSTEXT", "SYS_STATUS"],
            blocking=True,
            timeout=0.5,
        )
        if message is None:
            continue
        messageType = message.get_type()
        if (
            messageType == "COMMAND_ACK"
            and message.command == middleMan.mavlink.MAV_CMD_RUN_PREARM_CHECKS
        ):
            commandAccepted = message.result == middleMan.mavlink.MAV_RESULT_ACCEPTED
        elif messageType == "STATUSTEXT":
            if message.text not in prearmMessages:
                prearmMessages.append(message.text)
        elif messageType == "SYS_STATUS" and commandAccepted:
            # Only record SYS_STATUS readings that arrive AFTER the pre-arm check command was acknowledged.
            prearmBitEnabled = bool(
                int(message.onboard_control_sensors_enabled) & PREARM_CHECK_HEALTH_BIT
            )
            prearmBitHealthy = bool(
                int(message.onboard_control_sensors_health) & PREARM_CHECK_HEALTH_BIT
            )

    return commandAccepted, prearmBitEnabled, prearmBitHealthy, prearmMessages


def verifyAndOptionallyReboot(connection, compassCalibrated, runOutcome):
    """Step 5: Print stored offsets, optionally reboot Pixhawk, and verify pre-arm status."""
    printStepBanner(5, "FINAL VERIFICATION AND PIXHAWK REBOOT")

    offsetOneX = readParameter(connection, "COMPASS_OFS_X")
    offsetOneY = readParameter(connection, "COMPASS_OFS_Y")
    offsetOneZ = readParameter(connection, "COMPASS_OFS_Z")
    orientOne = int(round(readParameter(connection, "COMPASS_ORIENT")))
    useTwo = int(round(readParameter(connection, "COMPASS_USE2")))
    offsetTwoX = readParameter(connection, "COMPASS_OFS2_X")
    offsetTwoY = readParameter(connection, "COMPASS_OFS2_Y")
    offsetTwoZ = readParameter(connection, "COMPASS_OFS2_Z")
    orientTwo = int(round(readParameter(connection, "COMPASS_ORIENT2")))
    skipChecks = int(round(readParameter(connection, "ARMING_SKIPCHK")))
    posXY = int(round(readParameter(connection, "EK3_SRC1_POSXY")))

    print(
        f"Compass 1 Offsets : X={offsetOneX:+.1f}, Y={offsetOneY:+.1f}, Z={offsetOneZ:+.1f} "
        f"(ORIENT={orientOne})"
    )
    print(
        f"Compass 2 Offsets : X={offsetTwoX:+.1f}, Y={offsetTwoY:+.1f}, Z={offsetTwoZ:+.1f} "
        f"(USE2={useTwo}, ORIENT2={orientTwo})"
    )
    print(f"ARMING_SKIPCHK    : {skipChecks} (64 = Compass & GPS checks enabled)")
    print(f"EK3_SRC1_POSXY    : {posXY} (3 = GPS enabled)")
    printCurrentSensorSummary(connection)

    rebootedText = "Reboot=No"
    if compassCalibrated is True:
        print()
        print("ArduPilot requires a Pixhawk reboot after compass calibration so EKF3")
        print("loads the new compass offsets.")
        rebootChoice = askChoice("Reboot the Pixhawk now?", ("Y", "N"), "Y")
        if rebootChoice == "Y":
            print("Sending reboot command to the Pixhawk...")
            connection.mav.command_long_send(
                connection.target_system,
                connection.target_component,
                middleMan.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                0,
                1,
                0,
                0,
                0,
                0,
                0,
                0,
            )
            connection.close()
            print(f"Waiting {REBOOT_WAIT_SECONDS:.0f} seconds for the Pixhawk to reboot...")
            time.sleep(REBOOT_WAIT_SECONDS)
            connection = connectToPixhawk()
            rebootedText = "Reboot=Yes"

    print()
    print("Running Pixhawk Pre-Arm Checks...")
    (
        commandAccepted,
        prearmBitEnabled,
        prearmBitHealthy,
        prearmMessages,
    ) = readPrearmHealthAndMessages(connection)
    print(f"  Pre-Arm Command Accepted : {commandAccepted}")
    print(f"  SYS_STATUS Pre-Arm Check : Enabled={prearmBitEnabled}, Healthy={prearmBitHealthy}")

    if prearmMessages:
        print("  Current Pre-Arm Notices from Pixhawk:")
        for text in prearmMessages:
            print(f"    - {text}")

    if commandAccepted and prearmBitEnabled is True and prearmBitHealthy is True and not prearmMessages:
        print("  Pre-Arm Check Result: PASSED (Pre-arm check is enabled, healthy, and reported no blockers).")
        runOutcome["step5PreArmVerification"] = f"PASSED ({rebootedText}, Pre-Arm Healthy=True)"
    else:
        blockerSummary = "; ".join(prearmMessages) if prearmMessages else "SYS_STATUS pre-arm bit False"
        print(
            "  Pre-Arm Check Result: NOT READY YET (Wait for outdoor 3D GPS lock and "
            "press the hardware safety switch before arming)."
        )
        runOutcome["step5PreArmVerification"] = (
            f"CHECKED ({rebootedText}, Healthy={prearmBitHealthy}, Notices: {blockerSummary})"
        )

    return connection


# --------------------------------------------------
# MAIN INTERACTIVE FLOW
# --------------------------------------------------

def main():
    """Walk the user step by step through recalibrating the drone and save a full log."""
    originalStandardOutput = sys.stdout
    logFile = None
    logPath = None
    connection = None
    runOutcome = createRunOutcomeRecord()

    try:
        logFile, logPath, _ = openCalibrationLog()

        print("====================================================================")
        print("DRONE RECALIBRATION WIZARD (COMPASS, LEVEL HORIZON, EKF & PRE-ARM)")
        print("====================================================================")

        connection = connectToPixhawk()
        runOutcome["connection"] = (
            f"CONNECTED (system={connection.target_system}, component={connection.target_component})"
        )
        print()
        print("Current Pixhawk Sensor Snapshot:")
        printCurrentSensorSummary(connection)

        # Step 1: Fix EKF3 GPS sources, Land failsafe, and re-enable Compass/GPS pre-arm checks.
        configureCoreParameters(connection, runOutcome)

        # Step 2: Inspect all compass slots dynamically and select All or External-only.
        expectedCompassIds = configureCompassSelection(connection, runOutcome)

        # Step 3: Calibrate level horizon while the copter sits flat.
        calibrateLevelHorizon(connection, runOutcome)

        # Step 4: Live 0% to 100% onboard compass calibration for all expected compasses.
        compassCalibrated = calibrateCompasses(connection, expectedCompassIds, runOutcome)

        # Step 5: Verify saved parameters, optionally reboot Pixhawk, and run pre-arm checks.
        connection = verifyAndOptionallyReboot(connection, compassCalibrated, runOutcome)

        if compassCalibrated is False:
            runOutcome["overallOutcome"] = "FAILED"
            runOutcome["failureReason"] = "Compass calibration did not succeed."
        elif compassCalibrated is None:
            runOutcome["overallOutcome"] = "COMPLETED (COMPASS CALIBRATION SKIPPED)"
        else:
            runOutcome["overallOutcome"] = "SUCCESS"

        print()
        print("Recalibration walkthrough complete.")

    except KeyboardInterrupt:
        print()
        print("Recalibration stopped by user (Ctrl+C).")
        runOutcome["overallOutcome"] = "INTERRUPTED BY USER"
        runOutcome["failureReason"] = "User pressed Ctrl+C."

    except Exception as error:
        print()
        print(f"Recalibration stopped: {error}")
        runOutcome["overallOutcome"] = "FAILED"
        runOutcome["failureReason"] = str(error)

    finally:
        if connection is not None:
            try:
                connection.close()
                print("Command cable closed.")
            except Exception as closeError:
                print(f"Could not close command cable cleanly: {closeError}")

        printRunOutcomeSummary(runOutcome, logPath)
        sys.stdout = originalStandardOutput
        if logFile is not None:
            logFile.close()


if __name__ == "__main__":
    main()
