#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    Step-by-step terminal guide to recalibrate the drone's level horizon,
    compasses, EKF3 GPS settings, and pre-arm safety checks.
    Written as one top-to-bottom walkthrough with only the core Pixhawk
    read/write helpers so you can read the script and follow along easily.
    Every run saves a complete log in oct19-2026/calibrationLogs/.

WHY:
    Earlier calibration runs timed out at 75% on Compass 1 because the
    terminal did not tell you which direction to spin next, and Step 3 was
    calibrated on a tilted surface (+3.6 deg roll, +2.5 deg pitch). That left
    the drone with a 66-degree compass heading error and a tilted horizon,
    causing LOITER to drift past 8 feet on liftoff.
    This walkthrough guides you through each of the 6 physical drone
    positions live as the percentage climbs from 0% to 100%.

HOW:
    1. Open a timestamped log file in oct19-2026/calibrationLogs/.
    2. Connect to the Pixhawk on /dev/serial0 (pins 8 and 10 at 57600 baud)
       and refuse to run if the motors are armed.
    3. Step 1: Set EKF3 to use GPS + Barometer + Compass, set the ground-station
       failsafe to Land (5), and enable Compass and GPS pre-arm checks.
    4. Step 2: Check all compass slots on the Pixhawk and choose whether to
       enable Both Compasses (External GPS Mast + Internal Board) or only one.
    5. Step 3: Place the drone on a flat, level surface and calibrate Level
       Horizon Trim (IMU level), checking that the trim stays near 0 degrees.
    6. Step 4: Run onboard 6-position compass calibration with live prompts
       telling you exactly how to hold and spin the drone at each percentage:
         - Position 1 (  0% to  17%): Flat and level — spin 360 degrees
         - Position 2 ( 17% to  34%): Nose pointing straight DOWN — spin 360 degrees
         - Position 3 ( 34% to  50%): Nose pointing straight UP — spin 360 degrees
         - Position 4 ( 50% to  67%): Rolled onto LEFT side — spin 360 degrees
         - Position 5 ( 67% to  84%): Rolled onto RIGHT side — spin 360 degrees
         - Position 6 ( 84% to 100%): Upside down (belly up) — spin 360 degrees
    7. Step 5: Verify the new compass offsets saved into the Pixhawk, reboot
       the Pixhawk so EKF3 loads them, and run Pre-Arm Checks.

IMPORTANT:
    Run on the Raspberry Pi:
        /home/drone/Desktop/scripts/oct19-2026/reCalibrateDrone.py
    Take the propellers OFF before running this script.
    Do Step 4 outside, at least 10 feet away from cars, steel tables, or concrete rebar.
    Make sure the arrow on the GPS puck points straight forward toward the nose,
    matching the arrow on the Pixhawk.
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
# Four full minutes so you have plenty of time to rotate through all 6 positions without timing out.
COMPASS_CALIBRATION_TIMEOUT_SECONDS = 240.0
DRAIN_OLD_PACKETS_TIMEOUT_SECONDS = 1.5
REBOOT_WAIT_SECONDS = 12.0
MESSAGE_INTERVAL_MICROSECONDS = 100000
EXTRA3_STREAM_RATE_HERTZ = 10
PARAMETER_TOLERANCE = 0.01
OFFSET_MATCH_TOLERANCE_MILLIGAUSS = 1.0
MATRIX_MATCH_TOLERANCE = 0.02

# ARMING_SKIPCHK = 64 skips only RC radio (bit 6) and turns Compass & GPS pre-arm checks ON.
ARMING_SKIP_RC_ONLY_VALUE = 64

# Copter 4.7 EKF3 source values: 3 = GPS, 1 = Barometer, 1 = Compass.
EKF_SOURCE_GPS_VALUE = 3
EKF_SOURCE_BARO_VALUE = 1
EKF_SOURCE_COMPASS_VALUE = 1

# Copter 4.7 GCS failsafe: 5 = Land if Pi heartbeats stop.
GROUND_STATION_FAILSAFE_LAND_VALUE = 5
PREFERRED_FAILSAFE_TIMEOUT_SECONDS = 5.0
CONTINUE_PILOT_MODE_ON_FAILSAFE_BIT = 16

# Compass auto-orientation: 2 = Check and fix compass orientation automatically during calibration.
COMPASS_AUTO_ROTATION_VALUE = 2

# Bit 28 in SYS_STATUS onboard_control_sensors_health means Pre-Arm Checks are passing.
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
# LOGGING TEE AND CORE PIXHAWK READ / WRITE HELPERS
# --------------------------------------------------

class CalibrationLog:
    """Copy every printed line to both the terminal and the saved log file."""

    def __init__(self, logFile, terminal):
        self.logFile = logFile
        self.terminal = terminal

    def write(self, text):
        self.terminal.write(text)
        self.terminal.flush()
        if text.startswith("\r") and not text.endswith("\n"):
            return
        self.logFile.write(text)
        self.logFile.flush()

    def writeLogOnly(self, text):
        self.logFile.write(text)
        self.logFile.flush()

    def flush(self):
        self.terminal.flush()
        self.logFile.flush()


def connectToPixhawk():
    """Open /dev/serial0, verify heartbeat and disarmed state, and start sensor streams."""
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

        if connection.target_component == 0:
            connection.target_component = 1

        isArmed = bool(heartbeat.base_mode & middleMan.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if isArmed:
            raise RuntimeError("The copter is currently ARMED. Disarm before calibrating.")

        print(
            f"Connected to Pixhawk (system {connection.target_system}, "
            f"component {connection.target_component}). Motors are disarmed."
        )

        # Request EXTRA3 stream and individual sensor/calibration messages at 10 Hz.
        connection.mav.request_data_stream_send(
            connection.target_system,
            connection.target_component,
            middleMan.mavlink.MAV_DATA_STREAM_EXTRA3,
            EXTRA3_STREAM_RATE_HERTZ,
            1,
        )
        for messageName in (
            "ATTITUDE",
            "RAW_IMU",
            "SCALED_IMU2",
            "EKF_STATUS_REPORT",
            "SYS_STATUS",
            "STATUSTEXT",
            "MAG_CAL_PROGRESS",
            "MAG_CAL_REPORT",
        ):
            messageId = getattr(middleMan.mavlink, f"MAVLINK_MSG_ID_{messageName}", None)
            if messageId is not None:
                connection.mav.command_long_send(
                    connection.target_system,
                    connection.target_component,
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
        return connection
    except BaseException:
        connection.close()
        raise


def readParameter(connection, parameterName):
    """Read one parameter value from the Pixhawk."""
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
        rawId = message.param_id
        if isinstance(rawId, bytes):
            cleanName = rawId.decode("ascii", errors="ignore").rstrip("\x00")
        else:
            cleanName = str(rawId).rstrip("\x00")
        if cleanName == parameterName:
            return float(message.param_value)
    raise RuntimeError(f"Timed out reading {parameterName}.")


def writeParameterAndConfirm(connection, parameterName, wantedValue):
    """Write one parameter using its native MAVLink param_type and verify it saved."""
    connection.mav.param_request_read_send(
        connection.target_system,
        connection.target_component,
        parameterName.encode("ascii"),
        -1,
    )
    existingMessage = None
    deadline = time.monotonic() + PARAMETER_TIMEOUT_SECONDS
    while time.monotonic() < deadline:
        message = connection.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if message is None:
            continue
        rawId = message.param_id
        if isinstance(rawId, bytes):
            cleanName = rawId.decode("ascii", errors="ignore").rstrip("\x00")
        else:
            cleanName = str(rawId).rstrip("\x00")
        if cleanName == parameterName:
            existingMessage = message
            break
    if existingMessage is None:
        raise RuntimeError(f"Timed out reading {parameterName} before write.")

    parameterType = getattr(
        existingMessage,
        "param_type",
        middleMan.mavlink.MAV_PARAM_TYPE_REAL32,
    )
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
        rawId = message.param_id
        if isinstance(rawId, bytes):
            cleanName = rawId.decode("ascii", errors="ignore").rstrip("\x00")
        else:
            cleanName = str(rawId).rstrip("\x00")
        if cleanName == parameterName:
            confirmedValue = float(message.param_value)
            if isIntegerParameter and int(round(confirmedValue)) == targetInteger:
                return float(int(round(confirmedValue)))
            if not isIntegerParameter and abs(confirmedValue - wireValue) < PARAMETER_TOLERANCE:
                return confirmedValue
    raise RuntimeError(f"Could not confirm {parameterName} = {wantedValue}.")


def askUserChoice(promptText, validChoices, defaultChoice):
    """Ask a simple terminal question and log the answer."""
    choiceText = "/".join(validChoices)
    while True:
        fullPrompt = f"{promptText} [{choiceText}] (Press Enter for {defaultChoice}): "
        answer = input(fullPrompt).strip().upper()
        chosenValue = defaultChoice if answer == "" else answer
        if chosenValue in validChoices:
            if isinstance(sys.stdout, CalibrationLog):
                sys.stdout.writeLogOnly(f"{fullPrompt}{chosenValue}\n")
            return chosenValue
        print(f"Please type one of: {choiceText}")


def printLiveSensorSnapshot(connection):
    """Read and print live roll, pitch, heading, and magnetic field strengths."""
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

    externalOne = int(round(readParameter(connection, "COMPASS_EXTERNAL")))
    externalTwo = int(round(readParameter(connection, "COMPASS_EXTERN2")))
    labelOne = "External GPS Mast" if externalOne in (1, 2) else "Internal Board"
    labelTwo = "External GPS Mast" if externalTwo in (1, 2) else "Internal Board"

    if attitudeMessage is not None:
        rollDegrees = math.degrees(attitudeMessage.roll)
        pitchDegrees = math.degrees(attitudeMessage.pitch)
        yawDegrees = math.degrees(attitudeMessage.yaw)
        compassNote = ""
        if ekfStatusMessage is not None:
            compassNote = f" | EKF Compass Variance {ekfStatusMessage.compass_variance:.3f}"
        print(
            f"  Live EKF Attitude : Roll {rollDegrees:+.2f} deg | "
            f"Pitch {pitchDegrees:+.2f} deg | Fused Heading {yawDegrees:+.1f} deg{compassNote}"
        )

    if rawImuMessage is not None:
        strengthOne = math.hypot(rawImuMessage.xmag, rawImuMessage.ymag, rawImuMessage.zmag)
        print(
            f"  Compass 1 ({labelOne:<17}) : X={rawImuMessage.xmag:4d} Y={rawImuMessage.ymag:4d} "
            f"Z={rawImuMessage.zmag:4d} | Field Strength={strengthOne:5.0f} mG"
        )

    if secondImuMessage is not None:
        strengthTwo = math.hypot(secondImuMessage.xmag, secondImuMessage.ymag, secondImuMessage.zmag)
        print(
            f"  Compass 2 ({labelTwo:<17}) : X={secondImuMessage.xmag:4d} Y={secondImuMessage.ymag:4d} "
            f"Z={secondImuMessage.zmag:4d} | Field Strength={strengthTwo:5.0f} mG"
        )


# --------------------------------------------------
# MAIN TOP-TO-BOTTOM WALKTHROUGH
# --------------------------------------------------

def main():
    """Walk step by step through recalibrating the drone from top to bottom."""
    originalStandardOutput = sys.stdout
    logFile = None
    logPath = None
    connection = None

    runOutcome = {
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

    try:
        # --------------------------------------------------
        # OPEN CALIBRATION LOG FILE
        # --------------------------------------------------
        os.makedirs(CALIBRATION_LOG_DIRECTORY, exist_ok=True)
        scriptName = os.path.splitext(os.path.basename(__file__))[0]
        startedAtText = time.strftime("%Y-%m-%d-%H%M%S")
        logPath = os.path.join(CALIBRATION_LOG_DIRECTORY, f"{scriptName}-{startedAtText}.log")
        logFile = open(logPath, "w", encoding="utf-8")
        sys.stdout = CalibrationLog(logFile, sys.stdout)

        print(f"Calibration log file : {logPath}")
        print(f"Run started at       : {time.strftime('%Y-%m-%d %H:%M:%S %Z')}")
        print("====================================================================")
        print("DRONE RECALIBRATION WALKTHROUGH (STEP BY STEP)")
        print("====================================================================")

        # --------------------------------------------------
        # CONNECT TO PIXHAWK
        # --------------------------------------------------
        connection = connectToPixhawk()
        runOutcome["connection"] = (
            f"CONNECTED (system={connection.target_system}, component={connection.target_component})"
        )
        print()
        print("Current Pixhawk Sensor Readings:")
        printLiveSensorSnapshot(connection)

        # ==================================================
        # STEP 1: EKF3 GPS SOURCES, LAND FAILSAFE, AND PRE-ARM CHECKS
        # ==================================================
        print()
        print("=" * 68)
        print("STEP 1: EKF3 GPS SOURCES, LAND FAILSAFE, AND PRE-ARM CHECKS")
        print("=" * 68)

        currentSkipChecks = int(round(readParameter(connection, "ARMING_SKIPCHK")))
        currentPosXY = int(round(readParameter(connection, "EK3_SRC1_POSXY")))
        currentVelXY = int(round(readParameter(connection, "EK3_SRC1_VELXY")))
        currentPosZ = int(round(readParameter(connection, "EK3_SRC1_POSZ")))
        currentYaw = int(round(readParameter(connection, "EK3_SRC1_YAW")))
        currentGcsFailsafe = int(round(readParameter(connection, "FS_GCS_ENABLE")))
        currentFailsafeOptions = int(round(readParameter(connection, "FS_OPTIONS")))

        print("What this step sets on the Pixhawk:")
        print(f"  - ARMING_SKIPCHK : {currentSkipChecks} -> 64 (skips RC radio only; turns Compass & GPS checks ON)")
        print(f"  - EKF3 Sources   : POSXY={currentPosXY}->3 (GPS), VELXY={currentVelXY}->3 (GPS), POSZ={currentPosZ}->1 (Baro), YAW={currentYaw}->1 (Compass)")
        print(f"  - GCS Failsafe   : FS_GCS_ENABLE={currentGcsFailsafe}->5 (Land), FS_OPTIONS={currentFailsafeOptions}->0")
        print("  - COMPASS_AUTO_ROT -> 2 (automatically checks and fixes compass orientation during spin)")
        print()

        step1Choice = askUserChoice("Save these Step 1 safety settings?", ("Y", "N"), "Y")
        if step1Choice == "Y":
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
            print("Step 1 complete: EKF3, Land failsafe, and Compass/GPS pre-arm checks are saved.")
            runOutcome["step1CoreParameters"] = (
                "SUCCESS (EK3_SRC1_POSXY=3, FS_GCS_ENABLE=5, ARMING_SKIPCHK=64)"
            )
        else:
            print("Skipped Step 1.")
            runOutcome["step1CoreParameters"] = "SKIPPED BY USER"

        # ==================================================
        # STEP 2: CHOOSE WHICH COMPASSES TO ENABLE AND CALIBRATE
        # ==================================================
        print()
        print("=" * 68)
        print("STEP 2: CHECK COMPASS HARDWARE AND CHOOSE COMPASS MODE")
        print("=" * 68)

        slots = []
        for slotDefinition in COMPASS_SLOT_DEFINITIONS:
            deviceId = int(round(readParameter(connection, slotDefinition["deviceIdParam"])))
            useFlag = int(round(readParameter(connection, slotDefinition["useParam"])))
            externalFlag = int(round(readParameter(connection, slotDefinition["externalParam"])))
            orientValue = int(round(readParameter(connection, slotDefinition["orientParam"])))
            isExternal = externalFlag in (1, 2)
            locationLabel = "External GPS Mast" if isExternal else "Internal Pixhawk Board"
            slots.append(
                {
                    **slotDefinition,
                    "deviceId": deviceId,
                    "isDetected": deviceId != 0,
                    "useFlag": useFlag,
                    "externalFlag": externalFlag,
                    "isExternal": isExternal,
                    "locationLabel": locationLabel,
                    "orientValue": orientValue,
                }
            )

        detectedSlots = [slot for slot in slots if slot["isDetected"]]
        externalSlots = [slot for slot in detectedSlots if slot["isExternal"]]
        internalSlots = [slot for slot in detectedSlots if not slot["isExternal"]]
        if not detectedSlots:
            runOutcome["step2CompassSelection"] = "FAILED (No compass hardware detected)"
            raise RuntimeError("No hardware compasses were detected on the Pixhawk.")

        priorityOneId = int(round(readParameter(connection, "COMPASS_PRIO1_ID")))
        priorityTwoId = int(round(readParameter(connection, "COMPASS_PRIO2_ID")))
        print(f"Detected Compasses (Priority 1 ID={priorityOneId}, Priority 2 ID={priorityTwoId}):")
        for slot in detectedSlots:
            compassNumber = slot["compassId"] + 1
            print(
                f"  - Compass {compassNumber} (ID {slot['deviceId']}): {slot['locationLabel']} | "
                f"{slot['useParam']}={slot['useFlag']} | {slot['orientParam']}={slot['orientValue']}"
            )

        print()
        print("Which compass setup do you want to enable and calibrate?")
        print("  1 = Both Compasses (External GPS Mast #1 + Internal Pixhawk Board #2) [Recommended]")
        print("  2 = External GPS Mast Compass Only (Compass 1)")
        print("  3 = Internal Pixhawk Board Compass Only (Compass 2 — useful if external GPS mast is misaligned)")
        compassModeChoice = askUserChoice("Choose compass setup", ("1", "2", "3"), "1")

        expectedCompassIds = []
        if compassModeChoice == "2":
            if not externalSlots:
                runOutcome["step2CompassSelection"] = "FAILED (No external compass found)"
                raise RuntimeError("No external compass was found.")
            for slot in slots:
                shouldEnable = 1 if (slot["isDetected"] and slot["isExternal"]) else 0
                writeParameterAndConfirm(connection, slot["useParam"], shouldEnable)
                if shouldEnable == 1:
                    expectedCompassIds.append(slot["compassId"])
            enabledNames = ", ".join(
                f"Compass {cid + 1} ({slots[cid]['locationLabel']})" for cid in expectedCompassIds
            )
            print(f"Step 2 complete: Enabled External Only -> {enabledNames}.")
            runOutcome["step2CompassSelection"] = f"SUCCESS (External only: {enabledNames})"

        elif compassModeChoice == "3":
            if not internalSlots:
                runOutcome["step2CompassSelection"] = "FAILED (No internal compass found)"
                raise RuntimeError("No internal board compass was found.")
            for slot in slots:
                shouldEnable = 1 if (slot["isDetected"] and not slot["isExternal"]) else 0
                writeParameterAndConfirm(connection, slot["useParam"], shouldEnable)
                if shouldEnable == 1:
                    expectedCompassIds.append(slot["compassId"])
            enabledNames = ", ".join(
                f"Compass {cid + 1} ({slots[cid]['locationLabel']})" for cid in expectedCompassIds
            )
            print(f"Step 2 complete: Enabled Internal Board Only -> {enabledNames}.")
            runOutcome["step2CompassSelection"] = f"SUCCESS (Internal only: {enabledNames})"

        else:
            for slot in slots:
                shouldEnable = 1 if slot["isDetected"] else 0
                writeParameterAndConfirm(connection, slot["useParam"], shouldEnable)
                if shouldEnable == 1:
                    expectedCompassIds.append(slot["compassId"])
            enabledNames = ", ".join(
                f"Compass {cid + 1} ({slots[cid]['locationLabel']})" for cid in expectedCompassIds
            )
            print(f"Step 2 complete: Enabled All Detected Compasses -> {enabledNames}.")
            runOutcome["step2CompassSelection"] = f"SUCCESS (All detected: {enabledNames})"

        # ==================================================
        # STEP 3: LEVEL HORIZON CALIBRATION (FLAT ON THE GROUND)
        # ==================================================
        print()
        print("=" * 68)
        print("STEP 3: LEVEL HORIZON CALIBRATION (FLAT ON A LEVEL SURFACE)")
        print("=" * 68)
        print("DIRECTIONS FOR STEP 3:")
        print("  1. Set the drone down on a flat, level surface (not tilted on grass or cables).")
        print("  2. Check that the arrow on the GPS puck and the arrow on the Pixhawk")
        print("     both point straight forward toward the nose.")
        print("  3. Stand back and do NOT touch or move the drone while it measures level.")
        print()

        levelChoice = askUserChoice("Is the drone sitting flat and still right now? (Y = Calibrate, S = Skip)", ("Y", "S"), "Y")
        if levelChoice != "Y":
            print("Skipped Step 3 level horizon calibration.")
            runOutcome["step3LevelHorizon"] = "SKIPPED BY USER"
        else:
            # Clear old queued ACK or STATUSTEXT messages before sending the command.
            drainDeadline = time.monotonic() + DRAIN_OLD_PACKETS_TIMEOUT_SECONDS
            while time.monotonic() < drainDeadline:
                if connection.recv_match(type=["COMMAND_ACK", "STATUSTEXT"], blocking=False) is None:
                    break

            # param5 = 2 runs board-level trim (AHRS_TRIM_X / AHRS_TRIM_Y).
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

            levelAccepted = False
            levelAckResult = None
            ackDeadline = time.monotonic() + COMMAND_ACK_TIMEOUT_SECONDS
            while time.monotonic() < ackDeadline:
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
                    levelAckResult = int(message.result)
                    levelAccepted = levelAckResult == middleMan.mavlink.MAV_RESULT_ACCEPTED
                    break

            if not levelAccepted:
                runOutcome["step3LevelHorizon"] = f"FAILED (ACK result={levelAckResult})"
                raise RuntimeError(
                    f"Level horizon calibration was rejected by the Pixhawk (result={levelAckResult})."
                )

            time.sleep(1.0)
            trimRollDegrees = math.degrees(readParameter(connection, "AHRS_TRIM_X"))
            trimPitchDegrees = math.degrees(readParameter(connection, "AHRS_TRIM_Y"))
            print("Step 3 complete: Level horizon trim accepted by the Pixhawk.")
            print(
                f"  Saved Level Trim : AHRS_TRIM_X={trimRollDegrees:+.2f} deg, "
                f"AHRS_TRIM_Y={trimPitchDegrees:+.2f} deg"
            )
            if abs(trimRollDegrees) > 2.5 or abs(trimPitchDegrees) > 2.5:
                print(
                    "  WARNING: Trim is more than 2.5 degrees! Make sure the surface was truly flat."
                )
            printLiveSensorSnapshot(connection)
            runOutcome["step3LevelHorizon"] = (
                f"SUCCESS (AHRS_TRIM_X={trimRollDegrees:+.2f} deg, AHRS_TRIM_Y={trimPitchDegrees:+.2f} deg)"
            )

        # ==================================================
        # STEP 4: 6-POSITION COMPASS CALIBRATION (WITH LIVE DIRECTION PROMPTS)
        # ==================================================
        print()
        print("=" * 68)
        print("STEP 4: ONBOARD COMPASS CALIBRATION (6 CLEAR POSITIONS)")
        print("=" * 68)

        compassBitmask = 0
        for compassId in expectedCompassIds:
            compassBitmask |= 1 << compassId
        compassListText = ", ".join(f"Compass {compassId + 1}" for compassId in expectedCompassIds)

        print(f"Compasses to calibrate: {compassListText}")
        print()
        print("WHY COMPASS CALIBRATION TIMED OUT AT 75% LAST TIME:")
        print("  ArduPilot needs samples from ALL 6 sides of an imaginary sphere.")
        print("  If you only spin flat or miss one side (like nose-up or upside-down),")
        print("  the progress bar stops at 75% and waits until it times out.")
        print()
        print("HOW TO DO THIS EASILY (THE TERMINAL WILL TELL YOU EACH MOVE LIVE):")
        print("  Stand outside away from metal. Pick up the drone and do 1 full slow")
        print("  360-degree turn (about 10 to 15 seconds per turn) in each position:")
        print("    Position 1 (  0% -  17%): Hold drone FLAT & LEVEL         -> turn your body 360 deg")
        print("    Position 2 ( 17% -  34%): Point NOSE STRAIGHT DOWN        -> turn your body 360 deg")
        print("    Position 3 ( 34% -  50%): Point NOSE STRAIGHT UP          -> turn your body 360 deg")
        print("    Position 4 ( 50% -  67%): Tilt LEFT WING / SIDE DOWN      -> turn your body 360 deg")
        print("    Position 5 ( 67% -  84%): Tilt RIGHT WING / SIDE DOWN     -> turn your body 360 deg")
        print("    Position 6 ( 84% - 100%): Flip UPSIDE DOWN (belly up)     -> turn your body 360 deg")
        print()

        compassChoice = askUserChoice(
            "Ready to pick up the drone and start spinning? (Y = Start, S = Skip)",
            ("Y", "S"),
            "Y",
        )
        compassCalibrated = None

        if compassChoice != "Y":
            print("Skipped Step 4 compass calibration.")
            runOutcome["step4CompassCalibration"] = "SKIPPED BY USER"
        else:
            # Clear old queued calibration packets.
            drainDeadline = time.monotonic() + DRAIN_OLD_PACKETS_TIMEOUT_SECONDS
            while time.monotonic() < drainDeadline:
                if connection.recv_match(
                    type=["MAG_CAL_PROGRESS", "MAG_CAL_REPORT", "COMMAND_ACK"],
                    blocking=False,
                ) is None:
                    break

            # Start onboard magnetometer calibration: param1=bitmask, param2=1 (retry), param3=1 (autosave).
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

            print()
            print(">>> POSITION 1 OF 6: Hold the drone FLAT AND LEVEL, and slowly turn 360 degrees...")
            progressByCompass = {compassId: 0 for compassId in expectedCompassIds}
            loggedMilestoneByCompass = {compassId: -1 for compassId in expectedCompassIds}
            reportsByCompass = {}
            printedReportSignatures = {}
            lastLineText = ""
            lastPromptedStage = 1
            lastStallHintTime = time.monotonic()
            lastProgressChangeTime = time.monotonic()
            lastLowestPercentage = 0
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
                        # Filter out noisy repeating PreArm messages while you are spinning the drone.
                        text = str(message.text).strip()
                        if "PreArm:" not in text and "EKF variance" not in text:
                            sys.stdout.write("\n")
                            print(f"  Pixhawk: {text}")
                            lastLineText = ""

                    elif messageType == "MAG_CAL_PROGRESS":
                        compassId = int(message.compass_id)
                        percentage = int(message.completion_pct)
                        progressByCompass[compassId] = percentage
                        statusName = CALIBRATION_STATUS_NAMES.get(
                            message.cal_status,
                            f"STATUS {message.cal_status}",
                        )

                        lowestPercentage = min(
                            progressByCompass.get(cid, 0) for cid in expectedCompassIds
                        )
                        if lowestPercentage != lastLowestPercentage:
                            lastLowestPercentage = lowestPercentage
                            lastProgressChangeTime = time.monotonic()

                        # Print a clear new line whenever the user reaches the next drone orientation stage.
                        if lowestPercentage >= 17 and lastPromptedStage < 2:
                            lastPromptedStage = 2
                            sys.stdout.write("\n")
                            print(
                                ">>> POSITION 2 OF 6: Point the NOSE STRAIGHT DOWN toward the ground, "
                                "and slowly turn 360 degrees..."
                            )
                            lastLineText = ""
                        elif lowestPercentage >= 34 and lastPromptedStage < 3:
                            lastPromptedStage = 3
                            sys.stdout.write("\n")
                            print(
                                ">>> POSITION 3 OF 6: Point the NOSE STRAIGHT UP toward the sky, "
                                "and slowly turn 360 degrees..."
                            )
                            lastLineText = ""
                        elif lowestPercentage >= 50 and lastPromptedStage < 4:
                            lastPromptedStage = 4
                            sys.stdout.write("\n")
                            print(
                                ">>> POSITION 4 OF 6: Tilt the LEFT SIDE DOWN (left arm pointing to ground), "
                                "and slowly turn 360 degrees..."
                            )
                            lastLineText = ""
                        elif lowestPercentage >= 67 and lastPromptedStage < 5:
                            lastPromptedStage = 5
                            sys.stdout.write("\n")
                            print(
                                ">>> POSITION 5 OF 6: Tilt the RIGHT SIDE DOWN (right arm pointing to ground), "
                                "and slowly turn 360 degrees..."
                            )
                            lastLineText = ""
                        elif lowestPercentage >= 84 and lastPromptedStage < 6:
                            lastPromptedStage = 6
                            sys.stdout.write("\n")
                            print(
                                ">>> POSITION 6 OF 6: Flip the drone UPSIDE DOWN (belly facing sky), "
                                "and slowly turn 360 degrees until 100%!"
                            )
                            lastLineText = ""

                        # If progress pauses at the same percentage for 12 seconds, remind the user which angles to hit.
                        nowTime = time.monotonic()
                        if (
                            nowTime - lastProgressChangeTime >= 12.0
                            and nowTime - lastStallHintTime >= 12.0
                        ):
                            lastStallHintTime = nowTime
                            sys.stdout.write("\n")
                            print(
                                f"  [TIP at {lowestPercentage}%]: Progress paused — tilt the drone into an angle "
                                "you haven't spun yet (UPSIDE DOWN, NOSE UP, NOSE DOWN, LEFT SIDE, or RIGHT SIDE)."
                            )
                            lastLineText = ""

                        # Build the live ASCII progress bar for each compass.
                        barParts = []
                        for cid in expectedCompassIds:
                            pct = max(0, min(100, int(progressByCompass.get(cid, 0))))
                            filled = int(round((pct / 100.0) * 20))
                            barString = "[" + ("#" * filled) + ("-" * (20 - filled)) + f"] {pct:3d}%"
                            barParts.append(f"Compass {cid + 1}: {barString}")

                        lineText = "\r  " + " | ".join(barParts) + f" | {statusName:<22}"
                        if lineText != lastLineText:
                            sys.stdout.write(lineText)
                            sys.stdout.flush()
                            lastLineText = lineText

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
                compassCalibrated = False

            if compassCalibrated is not False:
                sys.stdout.write("\n")
                missingCompassIds = [cid for cid in expectedCompassIds if cid not in reportsByCompass]
                if missingCompassIds:
                    missingNames = ", ".join(f"Compass {cid + 1}" for cid in missingCompassIds)
                    print(f"Compass calibration timed out before receiving reports for: {missingNames}.")
                    runOutcome["step4CompassCalibration"] = f"FAILED (Timed out waiting for {missingNames})"
                    compassCalibrated = False
                else:
                    # Send explicit accept if any report succeeded without autosaving yet.
                    needsExplicitAccept = any(
                        report.cal_status == middleMan.mavlink.MAG_CAL_SUCCESS and int(report.autosaved) != 1
                        for report in reportsByCompass.values()
                    )
                    if needsExplicitAccept:
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
                        acceptDeadline = time.monotonic() + COMMAND_ACK_TIMEOUT_SECONDS
                        while time.monotonic() < acceptDeadline:
                            message = connection.recv_match(
                                type=["COMMAND_ACK", "MAG_CAL_REPORT"],
                                blocking=True,
                                timeout=0.5,
                            )
                            if message is None:
                                continue
                            if message.get_type() == "MAG_CAL_REPORT":
                                cid = int(message.compass_id)
                                if cid in expectedCompassIds:
                                    reportsByCompass[cid] = message

                    # Verify every expected compass succeeded and stored its new offsets in Pixhawk parameters.
                    perCompassSummaries = []
                    allCompassesVerified = True
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
                            allCompassesVerified = False
                            break

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
                            abs(storedVal - reportVal) <= OFFSET_MATCH_TOLERANCE_MILLIGAUSS
                            for storedVal, reportVal in zip(storedOffsets, reportOffsets)
                        )
                        diagonalsMatch = all(
                            abs(storedVal - reportVal) <= MATRIX_MATCH_TOLERANCE
                            for storedVal, reportVal in zip(storedDiagonals, reportDiagonals)
                        )
                        offDiagonalsMatch = all(
                            abs(storedVal - reportVal) <= MATRIX_MATCH_TOLERANCE
                            for storedVal, reportVal in zip(storedOffDiagonals, reportOffDiagonals)
                        )

                        if not (offsetsMatch and diagonalsMatch and offDiagonalsMatch):
                            print(
                                f"Compass {compassId + 1} report offsets did not match the parameters "
                                "stored on the Pixhawk."
                            )
                            runOutcome["step4CompassCalibration"] = (
                                f"FAILED (Compass {compassId + 1} parameters did not match report)"
                            )
                            allCompassesVerified = False
                            break

                        perCompassSummaries.append(
                            f"C{compassId + 1} offsets=({report.ofs_x:+.1f},{report.ofs_y:+.1f},{report.ofs_z:+.1f}) "
                            f"fit={report.fitness:.2f}"
                        )

                    if allCompassesVerified:
                        print("Step 4 complete: All selected compasses reached 100% and saved their new offsets!")
                        runOutcome["step4CompassCalibration"] = "SUCCESS (" + "; ".join(perCompassSummaries) + ")"
                        compassCalibrated = True
                    else:
                        compassCalibrated = False

        # ==================================================
        # STEP 5: FINAL VERIFICATION, PIXHAWK REBOOT, AND PRE-ARM CHECK
        # ==================================================
        print()
        print("=" * 68)
        print("STEP 5: FINAL VERIFICATION, PIXHAWK REBOOT, AND PRE-ARM CHECK")
        print("=" * 68)

        useOne = int(round(readParameter(connection, "COMPASS_USE")))
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
            f"  Compass 1 Offsets : X={offsetOneX:+.1f}, Y={offsetOneY:+.1f}, Z={offsetOneZ:+.1f} "
            f"(USE={useOne}, ORIENT={orientOne})"
        )
        print(
            f"  Compass 2 Offsets : X={offsetTwoX:+.1f}, Y={offsetTwoY:+.1f}, Z={offsetTwoZ:+.1f} "
            f"(USE2={useTwo}, ORIENT2={orientTwo})"
        )
        print(f"  ARMING_SKIPCHK    : {skipChecks} (64 = Compass & GPS checks enabled)")
        print(f"  EK3_SRC1_POSXY    : {posXY} (3 = GPS enabled)")
        printLiveSensorSnapshot(connection)

        rebootedText = "Reboot=No"
        if compassCalibrated is True:
            print()
            print("Place the drone back down flat and still on the ground.")
            print("ArduPilot requires a Pixhawk reboot so EKF3 loads the new compass offsets.")
            rebootChoice = askUserChoice("Reboot the Pixhawk now?", ("Y", "N"), "Y")
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
                connection = None
                print(f"Waiting {REBOOT_WAIT_SECONDS:.0f} seconds for the Pixhawk to reboot...")
                time.sleep(REBOOT_WAIT_SECONDS)
                connection = connectToPixhawk()
                rebootedText = "Reboot=Yes"
                print("Pixhawk rebooted. Fresh sensor snapshot:")
                printLiveSensorSnapshot(connection)

        print()
        print("Running Pixhawk Pre-Arm Checks...")
        drainDeadline = time.monotonic() + DRAIN_OLD_PACKETS_TIMEOUT_SECONDS
        while time.monotonic() < drainDeadline:
            if connection.recv_match(type=["STATUSTEXT", "COMMAND_ACK", "SYS_STATUS"], blocking=False) is None:
                break

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
        )

        commandAccepted = False
        prearmBitEnabled = None
        prearmBitHealthy = None
        prearmMessages = []
        prearmDeadline = time.monotonic() + 5.0
        while time.monotonic() < prearmDeadline:
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
                prearmBitEnabled = bool(
                    int(message.onboard_control_sensors_enabled) & PREARM_CHECK_HEALTH_BIT
                )
                prearmBitHealthy = bool(
                    int(message.onboard_control_sensors_health) & PREARM_CHECK_HEALTH_BIT
                )

        print(f"  Pre-Arm Command Accepted : {commandAccepted}")
        print(f"  SYS_STATUS Pre-Arm Check : Enabled={prearmBitEnabled}, Healthy={prearmBitHealthy}")
        if prearmMessages:
            print("  Current Pre-Arm Notices from Pixhawk:")
            for text in prearmMessages:
                print(f"    - {text}")

        if commandAccepted and prearmBitEnabled is True and prearmBitHealthy is True and not prearmMessages:
            print("  Pre-Arm Check Result: PASSED (Ready for flight!).")
            runOutcome["step5PreArmVerification"] = f"PASSED ({rebootedText}, Pre-Arm Healthy=True)"
        else:
            blockerSummary = "; ".join(prearmMessages) if prearmMessages else "SYS_STATUS pre-arm bit False"
            print(
                "  Pre-Arm Check Result: NOT READY YET (See notices above; wait for outdoor "
                "3D GPS lock and press the hardware safety switch before arming)."
            )
            runOutcome["step5PreArmVerification"] = (
                f"CHECKED ({rebootedText}, Healthy={prearmBitHealthy}, Notices: {blockerSummary})"
            )

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

        sys.stdout = originalStandardOutput
        if logFile is not None:
            logFile.close()


if __name__ == "__main__":
    main()
