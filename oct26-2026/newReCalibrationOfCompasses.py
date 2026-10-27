#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    Fast, streamlined compass recalibration script that sets:
      - Internal Pixhawk Board compass (IST8310) as PRIMARY (Priority 1)
      - External GPS Puck compass (HMC5883) as SECONDARY (Priority 2)
    and relaxes compass calibration & EKF variance thresholds so calibration
    succeeds cleanly and gets you to flight ASAP without unnecessary rules.

HOW:
    1. Connect to Pixhawk on /dev/serial0 (57600 baud).
    2. Automatically detect Internal vs External compass slots, set Internal
       as Priority 1 (COMPASS_PRIO1_ID) and External GPS as Priority 2
       (COMPASS_PRIO2_ID), enable both, and relax variance/fitness thresholds:
         - COMPASS_CAL_FIT = 32.0 (allows sphere-fit variance during hand spin)
         - COMPASS_AUTO_ROT = 0 (locks COMPASS_ORIENT=0, no false orientation fail)
         - COMPASS_OFFS_MAX = 1800 (allows higher hard-iron offsets without blocking)
         - ARMING_MAGTHRESH = 300, EK3_MAG_I_GATE = 500 (tolerates compass variance)
    3. If compass priority order changed, automatically reboot Pixhawk so the
       new Primary/Secondary order is active before calibrating.
    4. Prompt once to pick up the drone and spin through the 6 positions to 100%.
    5. Save offsets, reboot Pixhawk automatically so EKF3 loads them, and print
       ready-to-fly status.
"""

import math
import os
import struct
import sys
import time

from pymavlink import mavutil as middleMan


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
COMPASS_CALIBRATION_TIMEOUT_SECONDS = 240.0
DRAIN_OLD_PACKETS_TIMEOUT_SECONDS = 1.0
REBOOT_WAIT_SECONDS = 12.0
MESSAGE_INTERVAL_MICROSECONDS = 100000
EXTRA3_STREAM_RATE_HERTZ = 10

# Relaxed compass & EKF settings to allow natural variance and get to flight ASAP:
RELAXED_COMPASS_CAL_FITNESS = 32.0
RELAXED_COMPASS_OFFSETS_MAX = 1800.0
DISABLE_STRICT_AUTO_ROTATION = 0
RELAXED_ARMING_MAG_THRESHOLD = 300.0
RELAXED_EKF_MAG_INNOVATION_GATE = 500.0
ARMING_SKIP_RC_ONLY_VALUE = 64

EKF_SOURCE_GPS_VALUE = 3
EKF_SOURCE_BARO_VALUE = 1
EKF_SOURCE_COMPASS_VALUE = 1
GROUND_STATION_FAILSAFE_LAND_VALUE = 5
PREFERRED_FAILSAFE_TIMEOUT_SECONDS = 5.0

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
        "slotIndex": 0,
        "deviceIdParam": "COMPASS_DEV_ID",
        "useParam": "COMPASS_USE",
        "externalParam": "COMPASS_EXTERNAL",
        "orientParam": "COMPASS_ORIENT",
        "offsetParams": ("COMPASS_OFS_X", "COMPASS_OFS_Y", "COMPASS_OFS_Z"),
    },
    {
        "slotIndex": 1,
        "deviceIdParam": "COMPASS_DEV_ID2",
        "useParam": "COMPASS_USE2",
        "externalParam": "COMPASS_EXTERN2",
        "orientParam": "COMPASS_ORIENT2",
        "offsetParams": ("COMPASS_OFS2_X", "COMPASS_OFS2_Y", "COMPASS_OFS2_Z"),
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


class CalibrationLog:
    """Copy printed output to both terminal and log file."""

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

    def flush(self):
        self.terminal.flush()
        self.logFile.flush()


def connectToPixhawk():
    """Open /dev/serial0, verify disarmed state, and request calibration streams."""
    if not os.path.exists(PIXHAWK_PORT):
        raise RuntimeError(f"{PIXHAWK_PORT} not found. Run this on the Raspberry Pi.")

    print(f"Connecting to Pixhawk on {PIXHAWK_PORT} at {PIXHAWK_BAUD} baud...")
    connection = middleMan.mavlink_connection(
        PIXHAWK_PORT,
        baud=PIXHAWK_BAUD,
        source_system=GROUND_STATION_SYSTEM_ID,
    )
    heartbeat = connection.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT_SECONDS)
    if heartbeat is None:
        connection.close()
        raise RuntimeError("No heartbeat from Pixhawk.")

    if connection.target_component == 0:
        connection.target_component = 1

    isArmed = bool(heartbeat.base_mode & middleMan.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    if isArmed:
        connection.close()
        raise RuntimeError("Copter is ARMED. Disarm before calibrating.")

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
                0, 0, 0, 0, 0,
            )
    return connection


def readParameter(connection, parameterName):
    """Read one parameter from Pixhawk."""
    for _ in range(3):
        connection.mav.param_request_read_send(
            connection.target_system,
            connection.target_component,
            parameterName.encode("ascii"),
            -1,
        )
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            message = connection.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.4)
            if message is None:
                continue
            rawId = message.param_id
            cleanName = (
                rawId.decode("ascii", errors="ignore").rstrip("\x00")
                if isinstance(rawId, bytes)
                else str(rawId).rstrip("\x00")
            )
            if cleanName == parameterName:
                return float(message.param_value)
    raise RuntimeError(f"Timed out reading {parameterName}.")


def writeParameter(connection, parameterName, wantedValue):
    """Write one parameter to Pixhawk using its native type."""
    connection.mav.param_request_read_send(
        connection.target_system,
        connection.target_component,
        parameterName.encode("ascii"),
        -1,
    )
    existingMessage = None
    deadline = time.monotonic() + PARAMETER_TIMEOUT_SECONDS
    while time.monotonic() < deadline:
        message = connection.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.4)
        if message is None:
            continue
        rawId = message.param_id
        cleanName = (
            rawId.decode("ascii", errors="ignore").rstrip("\x00")
            if isinstance(rawId, bytes)
            else str(rawId).rstrip("\x00")
        )
        if cleanName == parameterName:
            existingMessage = message
            break

    parameterType = getattr(
        existingMessage,
        "param_type",
        middleMan.mavlink.MAV_PARAM_TYPE_REAL32,
    )
    if parameterType in INTEGER_PARAMETER_TYPES:
        wireValue = float(int(round(wantedValue)))
    else:
        wireValue = float(struct.unpack("f", struct.pack("f", float(wantedValue)))[0])

    for _ in range(3):
        connection.mav.param_set_send(
            connection.target_system,
            connection.target_component,
            parameterName.encode("ascii"),
            wireValue,
            parameterType,
        )
        confirmDeadline = time.monotonic() + 2.0
        while time.monotonic() < confirmDeadline:
            message = connection.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.4)
            if message is None:
                continue
            rawId = message.param_id
            cleanName = (
                rawId.decode("ascii", errors="ignore").rstrip("\x00")
                if isinstance(rawId, bytes)
                else str(rawId).rstrip("\x00")
            )
            if cleanName == parameterName:
                return float(message.param_value)
    return wireValue


def rebootPixhawk(connection):
    """Reboot Pixhawk and reconnect."""
    print("Rebooting Pixhawk...")
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        middleMan.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0,
        1, 0, 0, 0, 0, 0, 0,
    )
    connection.close()
    time.sleep(REBOOT_WAIT_SECONDS)
    return connectToPixhawk()


def printCompassSnapshot(connection):
    """Print live headings and EKF compass variance."""
    rawImu = None
    scaledImu2 = None
    attitude = None
    ekf = None
    deadline = time.monotonic() + 2.5
    while time.monotonic() < deadline:
        msg = connection.recv_match(
            type=["RAW_IMU", "SCALED_IMU2", "ATTITUDE", "EKF_STATUS_REPORT"],
            blocking=True,
            timeout=0.3,
        )
        if msg is None:
            continue
        k = msg.get_type()
        if k == "RAW_IMU":
            rawImu = msg
        elif k == "SCALED_IMU2":
            scaledImu2 = msg
        elif k == "ATTITUDE":
            attitude = msg
        elif k == "EKF_STATUS_REPORT":
            ekf = msg
        if rawImu and scaledImu2 and attitude and ekf:
            break

    if rawImu and scaledImu2:
        h1 = math.degrees(math.atan2(rawImu.ymag, rawImu.xmag))
        h2 = math.degrees(math.atan2(scaledImu2.ymag, scaledImu2.xmag))
        diff = (h1 - h2 + 180.0) % 360.0 - 180.0
        print(
            f"  Primary (Priority 1)   : X={rawImu.xmag:4d} Y={rawImu.ymag:4d} Z={rawImu.zmag:4d} | Heading={h1:+6.1f} deg"
        )
        print(
            f"  Secondary (Priority 2) : X={scaledImu2.xmag:4d} Y={scaledImu2.ymag:4d} Z={scaledImu2.zmag:4d} | Heading={h2:+6.1f} deg"
        )
        print(f"  Heading Difference     : {diff:+.1f} deg")
    if attitude:
        print(
            f"  Attitude               : Roll={math.degrees(attitude.roll):+.2f} deg | "
            f"Pitch={math.degrees(attitude.pitch):+.2f} deg | Yaw={math.degrees(attitude.yaw):+.1f} deg"
        )
    if ekf:
        print(f"  EKF Compass Variance   : {ekf.compass_variance:.4f}")


def main():
    originalStdout = sys.stdout
    logFile = None
    logPath = None
    connection = None

    try:
        os.makedirs(CALIBRATION_LOG_DIRECTORY, exist_ok=True)
        startedAtText = time.strftime("%Y-%m-%d-%H%M%S")
        logPath = os.path.join(
            CALIBRATION_LOG_DIRECTORY,
            f"newReCalibrationOfCompasses-{startedAtText}.log",
        )
        logFile = open(logPath, "w", encoding="utf-8")
        sys.stdout = CalibrationLog(logFile, sys.stdout)

        print("====================================================================")
        print("FAST COMPASS RECALIBRATION (INTERNAL = PRIMARY, GPS = SECONDARY)")
        print("====================================================================")
        print(f"Log file: {logPath}")
        print()

        connection = connectToPixhawk()

        # 1. Detect Internal vs External compass slots
        internalSlot = None
        externalSlot = None
        for slotDef in COMPASS_SLOT_DEFINITIONS:
            devId = int(round(readParameter(connection, slotDef["deviceIdParam"])))
            extFlag = int(round(readParameter(connection, slotDef["externalParam"])))
            slotInfo = {**slotDef, "deviceId": devId, "isExternal": extFlag in (1, 2)}
            if devId != 0:
                if slotInfo["isExternal"]:
                    externalSlot = slotInfo
                else:
                    internalSlot = slotInfo

        if internalSlot is None or externalSlot is None:
            raise RuntimeError("Could not detect both Internal and External compasses on the Pixhawk.")

        currentPrio1 = int(round(readParameter(connection, "COMPASS_PRIO1_ID")))
        currentPrio2 = int(round(readParameter(connection, "COMPASS_PRIO2_ID")))
        wantedPrio1 = internalSlot["deviceId"]  # Internal Pixhawk Board = Primary
        wantedPrio2 = externalSlot["deviceId"]  # External GPS Puck = Secondary

        print(f"Detected Internal Board Compass : ID {wantedPrio1} (setting as PRIMARY / Priority 1)")
        print(f"Detected External GPS Compass   : ID {wantedPrio2} (setting as SECONDARY / Priority 2)")

        # 2. Apply Priority, EKF, and Relaxed Variance Parameters automatically
        priorityChanged = (currentPrio1 != wantedPrio1) or (currentPrio2 != wantedPrio2)
        writeParameter(connection, "COMPASS_PRIO1_ID", wantedPrio1)
        writeParameter(connection, "COMPASS_PRIO2_ID", wantedPrio2)
        writeParameter(connection, "COMPASS_USE", 1)
        writeParameter(connection, "COMPASS_USE2", 1)
        writeParameter(connection, "COMPASS_ORIENT", 0)
        writeParameter(connection, "COMPASS_ORIENT2", 0)
        writeParameter(connection, "COMPASS_AUTO_ROT", DISABLE_STRICT_AUTO_ROTATION)
        writeParameter(connection, "COMPASS_CAL_FIT", RELAXED_COMPASS_CAL_FITNESS)
        writeParameter(connection, "COMPASS_OFFS_MAX", RELAXED_COMPASS_OFFSETS_MAX)
        writeParameter(connection, "ARMING_MAGTHRESH", RELAXED_ARMING_MAG_THRESHOLD)
        writeParameter(connection, "EK3_MAG_I_GATE", RELAXED_EKF_MAG_INNOVATION_GATE)
        writeParameter(connection, "EK3_SRC1_POSXY", EKF_SOURCE_GPS_VALUE)
        writeParameter(connection, "EK3_SRC1_VELXY", EKF_SOURCE_GPS_VALUE)
        writeParameter(connection, "EK3_SRC1_POSZ", EKF_SOURCE_BARO_VALUE)
        writeParameter(connection, "EK3_SRC1_YAW", EKF_SOURCE_COMPASS_VALUE)
        writeParameter(connection, "ARMING_SKIPCHK", ARMING_SKIP_RC_ONLY_VALUE)
        writeParameter(connection, "FS_GCS_ENABLE", GROUND_STATION_FAILSAFE_LAND_VALUE)
        writeParameter(connection, "FS_GCS_TIMEOUT", PREFERRED_FAILSAFE_TIMEOUT_SECONDS)

        print("Saved compass priority (Internal=Primary, GPS=Secondary) and relaxed variance tolerances.")

        if priorityChanged:
            print("Compass priority order changed — rebooting Pixhawk now so the new order is active...")
            connection = rebootPixhawk(connection)
            print("Pixhawk rebooted with Internal Compass as Primary.")

        print()
        print("Current Compass Readings Before Calibration:")
        printCompassSnapshot(connection)

        print()
        print("====================================================================")
        print("SPIN CALIBRATION (6 POSITIONS)")
        print("====================================================================")
        print("  Position 1 (  0% -  17%): FLAT & LEVEL         -> turn 360 deg")
        print("  Position 2 ( 17% -  34%): NOSE DOWN            -> turn 360 deg")
        print("  Position 3 ( 34% -  50%): NOSE UP              -> turn 360 deg")
        print("  Position 4 ( 50% -  67%): LEFT SIDE DOWN       -> turn 360 deg")
        print("  Position 5 ( 67% -  84%): RIGHT SIDE DOWN      -> turn 360 deg")
        print("  Position 6 ( 84% - 100%): UPSIDE DOWN          -> turn 360 deg")
        print()
        answer = input("Press Enter (or type Y) to start spinning, or S to skip spin: ").strip().upper()
        if answer == "S":
            print("Skipped spin calibration. Settings are saved.")
            return

        # Clear old queued calibration messages
        drainDeadline = time.monotonic() + DRAIN_OLD_PACKETS_TIMEOUT_SECONDS
        while time.monotonic() < drainDeadline:
            if connection.recv_match(
                type=["MAG_CAL_PROGRESS", "MAG_CAL_REPORT", "COMMAND_ACK"],
                blocking=False,
            ) is None:
                break

        # Calibrate both Priority 0 (Internal) and Priority 1 (External GPS): bitmask = 3
        compassBitmask = 0b11
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            middleMan.mavlink.MAV_CMD_DO_START_MAG_CAL,
            0,
            compassBitmask,
            1,  # retry on failure
            1,  # autosave
            0, 0, 0, 0,
        )

        print()
        print(">>> POSITION 1 OF 6: Hold FLAT & LEVEL and slowly turn 360 degrees...")
        progressByCompass = {0: 0, 1: 0}
        reportsByCompass = {}
        lastPromptedStage = 1
        lastLineText = ""
        startedAtSeconds = time.monotonic()

        while time.monotonic() - startedAtSeconds < COMPASS_CALIBRATION_TIMEOUT_SECONDS:
            message = connection.recv_match(
                type=["MAG_CAL_PROGRESS", "MAG_CAL_REPORT", "STATUSTEXT"],
                blocking=True,
                timeout=0.5,
            )
            if message is None:
                continue

            msgType = message.get_type()
            if msgType == "STATUSTEXT":
                text = str(message.text).strip()
                if "PreArm:" not in text and "EKF variance" not in text:
                    sys.stdout.write("\n")
                    print(f"  Pixhawk: {text}")
                    lastLineText = ""

            elif msgType == "MAG_CAL_PROGRESS":
                cid = int(message.compass_id)
                pct = int(message.completion_pct)
                progressByCompass[cid] = pct
                statusName = CALIBRATION_STATUS_NAMES.get(message.cal_status, f"STATUS {message.cal_status}")
                lowestPct = min(progressByCompass.get(0, 0), progressByCompass.get(1, 0))

                if lowestPct >= 17 and lastPromptedStage < 2:
                    lastPromptedStage = 2
                    sys.stdout.write("\n")
                    print(">>> POSITION 2 OF 6: Point NOSE DOWN and turn 360 degrees...")
                    lastLineText = ""
                elif lowestPct >= 34 and lastPromptedStage < 3:
                    lastPromptedStage = 3
                    sys.stdout.write("\n")
                    print(">>> POSITION 3 OF 6: Point NOSE UP and turn 360 degrees...")
                    lastLineText = ""
                elif lowestPct >= 50 and lastPromptedStage < 4:
                    lastPromptedStage = 4
                    sys.stdout.write("\n")
                    print(">>> POSITION 4 OF 6: Tilt LEFT SIDE DOWN and turn 360 degrees...")
                    lastLineText = ""
                elif lowestPct >= 67 and lastPromptedStage < 5:
                    lastPromptedStage = 5
                    sys.stdout.write("\n")
                    print(">>> POSITION 5 OF 6: Tilt RIGHT SIDE DOWN and turn 360 degrees...")
                    lastLineText = ""
                elif lowestPct >= 84 and lastPromptedStage < 6:
                    lastPromptedStage = 6
                    sys.stdout.write("\n")
                    print(">>> POSITION 6 OF 6: Flip UPSIDE DOWN and turn 360 degrees to 100%!")
                    lastLineText = ""

                p0 = progressByCompass.get(0, 0)
                p1 = progressByCompass.get(1, 0)
                lineText = f"\r  Primary(Internal): {p0:3d}% | Secondary(GPS): {p1:3d}% | {statusName:<22}"
                if lineText != lastLineText:
                    sys.stdout.write(lineText)
                    sys.stdout.flush()
                    lastLineText = lineText

            elif msgType == "MAG_CAL_REPORT":
                cid = int(message.compass_id)
                if cid in (0, 1):
                    reportsByCompass[cid] = message
                    sys.stdout.write("\n")
                    statusName = CALIBRATION_STATUS_NAMES.get(message.cal_status, f"STATUS {message.cal_status}")
                    roleName = "Primary (Internal)" if cid == 0 else "Secondary (GPS)"
                    print(
                        f"  {roleName} Report: {statusName} | fitness={message.fitness:.2f} | "
                        f"offsets=({message.ofs_x:+.1f}, {message.ofs_y:+.1f}, {message.ofs_z:+.1f})"
                    )
                    lastLineText = ""
                if 0 in reportsByCompass and 1 in reportsByCompass:
                    break

        sys.stdout.write("\n")
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            middleMan.mavlink.MAV_CMD_DO_ACCEPT_MAG_CAL,
            0,
            compassBitmask,
            0, 0, 0, 0, 0, 0,
        )
        time.sleep(1.0)

        primaryReport = reportsByCompass.get(0)
        secondaryReport = reportsByCompass.get(1)
        if primaryReport and primaryReport.cal_status == middleMan.mavlink.MAG_CAL_SUCCESS:
            print("Primary (Internal) compass calibrated and saved!")
        else:
            print("Notice: Primary compass report did not mark new offsets; keeping existing Internal calibration.")

        if secondaryReport and secondaryReport.cal_status == middleMan.mavlink.MAG_CAL_SUCCESS:
            print("Secondary (GPS) compass calibrated and saved!")
        else:
            print("Secondary (GPS) compass had high variance; keeping Internal as Primary so you can fly immediately.")

        print()
        print("Rebooting Pixhawk automatically so EKF3 loads the calibrated compass settings...")
        connection = rebootPixhawk(connection)
        print("Pixhawk rebooted and ready! Final sensor snapshot:")
        printCompassSnapshot(connection)
        print("Done — ready for flight.")

    except KeyboardInterrupt:
        print("\nStopped by user (Ctrl+C).")
        if connection is not None:
            try:
                connection.mav.command_long_send(
                    connection.target_system,
                    connection.target_component,
                    middleMan.mavlink.MAV_CMD_DO_CANCEL_MAG_CAL,
                    0, 0b11, 0, 0, 0, 0, 0, 0,
                )
            except Exception:
                pass
    finally:
        if connection is not None:
            try:
                connection.close()
            except Exception:
                pass
        sys.stdout = originalStdout
        if logFile is not None:
            logFile.close()


if __name__ == "__main__":
    main()
