# WHAT — compass calibration for the Pixhawk.
# WHY — the Oct 26 recalibration is the sequence that saves both compasses.
#       This function runs that sequence from the backend and stores the result.
# HOW — YES opens /dev/serial0. Internal compass becomes priority 1, the GPS
#       mast becomes priority 2, then both compasses are spun through 6 positions.
#       The Pixhawk report is saved in the flight-log database.
# IMPORTANT — without YES the Pixhawk is not opened. An armed copter is refused.
#             This does not force-disarm. A live progress page is not part of this file.

import asyncio
import json
import math
import os
import struct
import time

from models.flightlogs import saveFlightLog

PIXHAWK_PORT = "/dev/serial0"
PIXHAWK_BAUD = 57600
GROUND_STATION_SYSTEM_ID = 255
HEARTBEAT_TIMEOUT_SECONDS = 15.0
PARAMETER_TIMEOUT_SECONDS = 5.0
COMPASS_CALIBRATION_TIMEOUT_SECONDS = 240.0
DRAIN_OLD_PACKETS_TIMEOUT_SECONDS = 1.0
REBOOT_WAIT_SECONDS = 12.0
MESSAGE_INTERVAL_MICROSECONDS = 100000
EXTRA3_STREAM_RATE_HERTZ = 10
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
BOTH_COMPASSES_BITMASK = 0b11

COMPASS_SLOT_DEFINITIONS = (
    {
        "slotIndex": 0,
        "deviceIdParam": "COMPASS_DEV_ID",
        "externalParam": "COMPASS_EXTERNAL",
    },
    {
        "slotIndex": 1,
        "deviceIdParam": "COMPASS_DEV_ID2",
        "externalParam": "COMPASS_EXTERN2",
    },
)

SPIN_POSITIONS = (
    (17, "POSITION 2 OF 6: Nose straight at the ground, tail at the sky. Keep the nose down. Look down at the tail and turn clockwise one full 360. Do not roll it upright."),
    (34, "POSITION 3 OF 6: Nose straight at the sky, tail at the ground. Keep the nose up. Look down at the belly and turn clockwise one full 360. Do not roll it."),
    (50, "POSITION 4 OF 6: Drone's left side toward the ground. Left is your left when you stand behind the tail and look along the nose. Keep the left side down. Look down at the right side and turn clockwise one full 360."),
    (67, "POSITION 5 OF 6: Drone's right side toward the ground. Right is your right when you stand behind the tail and look along the nose. Keep the right side down. Look down at the left side and turn clockwise one full 360."),
    (84, "POSITION 6 OF 6: Upside down. Belly and legs point at the sky. Keep it upside down. Look down at the belly and turn clockwise one full 360 until both percentages reach 100."),
)

calibrationIsRunning = False
calibrationGuard = asyncio.Lock()
showCalibrationAsRunning = False
progressLoop = None
progressQueues = set()
calibrationProgress = {
    "running": False,
    "outcome": None,
    "instruction": "Press start, then pick up the drone and follow the highlighted position.",
    "primaryPercent": 0,
    "secondaryPercent": 0,
    "position": 0,
    "status": None,
    "lines": [],
    "compassReports": [],
    "stopReason": None,
}


async def calibrateCompasses(request):
    global calibrationIsRunning
    try:
        if await confirmationText(request) != "YES":
            cancelled = {
                "scriptName": "calibrateCompasses",
                "started": False,
                "outcome": "CANCELLED BEFORE START",
                "stopReason": "The request did not say YES. The Pixhawk was not opened.",
            }
            cancelled["id"] = saveFlightLog(cancelled)
            return cancelled

        from controllers.flightController import flightIsRunning

        async with calibrationGuard:
            if calibrationIsRunning:
                return {
                    "scriptName": "calibrateCompasses",
                    "started": False,
                    "outcome": "A CALIBRATION IS ALREADY RUNNING",
                }
            if flightIsRunning:
                return {
                    "scriptName": "calibrateCompasses",
                    "started": False,
                    "outcome": "A FLIGHT IS ALREADY RUNNING",
                }
            from controllers.motorTest import motorTestIsRunning
            if motorTestIsRunning:
                return {
                    "scriptName": "calibrateCompasses",
                    "started": False,
                    "outcome": "A MOTOR TEST IS ALREADY RUNNING",
                }
            from controllers.flightPlan import flightPlanIsRunning, flightPlanVehicle
            if flightPlanIsRunning or flightPlanVehicle is not None:
                return {
                    "scriptName": "calibrateCompasses",
                    "started": False,
                    "outcome": "A FLIGHT PLAN IS ALREADY USING THE COMMAND CABLE",
                }
            calibrationIsRunning = True

        try:
            calibrationResult = await asyncio.to_thread(runCalibrateCompasses)
        finally:
            async with calibrationGuard:
                calibrationIsRunning = False

        calibrationResult["id"] = saveFlightLog(calibrationResult)
        return calibrationResult
    except Exception as error:
        print(f"calibrateCompasses did not finish: {error}")
        failed = {
            "scriptName": "calibrateCompasses",
            "started": False,
            "outcome": "FAILED",
            "stopReason": str(error),
        }
        failed["id"] = saveFlightLog(failed)
        return failed


async def confirmationText(request):
    try:
        rawBody = await request.body()
        if not rawBody:
            return ""
        payload = json.loads(rawBody)
        if isinstance(payload, dict):
            return str(payload.get("confirm", "")).strip()
    except Exception:
        return ""
    return ""


def rememberProgressLoop(loop):
    global progressLoop
    progressLoop = loop


def readCalibrationProgress():
    try:
        current = calibrationProgress
        return {
            "running": current.get("running", False),
            "outcome": current.get("outcome"),
            "instruction": current.get("instruction"),
            "primaryPercent": current.get("primaryPercent", 0),
            "secondaryPercent": current.get("secondaryPercent", 0),
            "position": current.get("position", 0),
            "status": current.get("status"),
            "lines": list(current.get("lines") or []),
            "compassReports": list(current.get("compassReports") or []),
            "stopReason": current.get("stopReason"),
        }
    except Exception as error:
        print(f"Calibration progress was not read: {error}")
        return {
            "running": False,
            "outcome": None,
            "instruction": "Progress could not be read.",
            "primaryPercent": 0,
            "secondaryPercent": 0,
            "position": 0,
            "status": None,
            "lines": [],
            "compassReports": [],
            "stopReason": str(error),
        }


def publishCalibrationProgress(result):
    global calibrationProgress
    progress = result.get("progress") or {}
    lines = list(result.get("lines") or [])
    calibrationProgress = {
        "running": showCalibrationAsRunning,
        "outcome": None if showCalibrationAsRunning else result.get("outcome"),
        "instruction": lines[-1] if lines else "Getting ready.",
        "primaryPercent": progress.get("primaryPercent", 0),
        "secondaryPercent": progress.get("secondaryPercent", 0),
        "position": progress.get("position", 0),
        "status": progress.get("status"),
        "lines": lines[-8:],
        "compassReports": list(result.get("compassReports") or []),
        "stopReason": None if showCalibrationAsRunning else result.get("stopReason"),
    }
    sendProgressToSockets(calibrationProgress)


def sendProgressToSockets(snapshot):
    if progressLoop is None:
        return
    for progressQueue in list(progressQueues):
        try:
            progressLoop.call_soon_threadsafe(progressQueue.put_nowait, snapshot)
        except Exception as error:
            print(f"Calibration progress was not sent: {error}")


def runCalibrateCompasses():
    global showCalibrationAsRunning
    showCalibrationAsRunning = True
    result = {
        "scriptName": "calibrateCompasses",
        "started": True,
        "startedAt": time.strftime("%Y-%m-%d %H:%M:%S %Z"),
        "finishedAt": None,
        "outcome": "FAILED",
        "stopReason": None,
        "lines": [],
        "compassReports": [],
        "progress": {"primaryPercent": 0, "secondaryPercent": 0, "position": 0},
    }
    connection = None
    calibrationStarted = False
    middleMan = None
    try:
        if not os.path.exists(PIXHAWK_PORT):
            raise RuntimeError(f"{PIXHAWK_PORT} is missing. Run the backend on the Raspberry Pi.")
        from pymavlink import mavutil as middleMan

        writeCalibrationLine(result, "FAST COMPASS RECALIBRATION (INTERNAL = PRIMARY, GPS = SECONDARY)")
        connection = connectToPixhawk(middleMan)
        internalSlot, externalSlot = detectCompassSlots(connection)
        result["internalDeviceId"] = internalSlot["deviceId"]
        result["externalDeviceId"] = externalSlot["deviceId"]
        writeCalibrationLine(result, f"Internal board compass id {internalSlot['deviceId']} set as primary.")
        writeCalibrationLine(result, f"External GPS compass id {externalSlot['deviceId']} set as secondary.")

        priorityChanged = applyCompassSettings(connection, middleMan, internalSlot, externalSlot)
        writeCalibrationLine(result, "Saved compass priority and relaxed compass tolerances.")
        if priorityChanged:
            writeCalibrationLine(result, "Compass priority changed. Rebooting so the new order is active.")
            connection = rebootPixhawk(connection, middleMan)
            writeCalibrationLine(result, "Pixhawk rebooted with the internal compass as primary.")

        result["beforeCalibration"] = readCompassSnapshot(connection)
        writeCalibrationLine(result, "Current compass readings recorded.")
        result["progress"]["position"] = 1
        writeCalibrationLine(
            result,
            "POSITION 1 OF 6: Drone upright, belly and legs toward the ground. Keep it upright. Turn the nose clockwise, to the right, one full 360. Do not flip it.",
        )

        drainOldCalibrationMessages(connection)
        calibrationStarted = True
        startMagCalibration(connection, middleMan)
        reports = waitForMagCalibration(connection, middleMan, result)
        acceptMagCalibration(connection, middleMan)
        result["compassReports"] = reports

        primarySucceeded = reportSucceeded(reports, 0, middleMan)
        secondarySucceeded = reportSucceeded(reports, 1, middleMan)
        if primarySucceeded and secondarySucceeded:
            result["outcome"] = "SUCCESS"
            writeCalibrationLine(result, "Both compasses calibrated and saved.")
        elif primarySucceeded:
            result["outcome"] = "PARTIAL"
            writeCalibrationLine(result, "Primary compass saved. Secondary compass did not report success.")
        else:
            result["outcome"] = "FAILED"
            result["stopReason"] = "The Pixhawk did not report success for the primary compass."
            writeCalibrationLine(result, result["stopReason"])

        writeCalibrationLine(result, "Rebooting so the EKF loads the compass calibration.")
        connection = rebootPixhawk(connection, middleMan)
        result["afterCalibration"] = readCompassSnapshot(connection)
        writeCalibrationLine(result, "Pixhawk rebooted. Final compass snapshot recorded.")
    except Exception as error:
        result["outcome"] = "FAILED"
        result["stopReason"] = str(error)
        writeCalibrationLine(result, f"Calibration stopped: {error}")
        if calibrationStarted and connection is not None and middleMan is not None:
            cancelMagCalibration(connection, middleMan)
    finally:
        showCalibrationAsRunning = False
        if connection is not None:
            try:
                connection.close()
            except Exception as closeError:
                writeCalibrationLine(result, f"Could not close the command cable: {closeError}")
        result["finishedAt"] = time.strftime("%Y-%m-%d %H:%M:%S %Z")
        publishCalibrationProgress(result)
    return result


def writeCalibrationLine(result, text):
    print(text)
    result["lines"].append(text)
    publishCalibrationProgress(result)


def connectToPixhawk(middleMan):
    writeCalibrationLineIfPrintOnly(f"Connecting to Pixhawk on {PIXHAWK_PORT} at {PIXHAWK_BAUD} baud.")
    connection = middleMan.mavlink_connection(
        PIXHAWK_PORT,
        baud=PIXHAWK_BAUD,
        source_system=GROUND_STATION_SYSTEM_ID,
    )
    heartbeat = connection.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT_SECONDS)
    if heartbeat is None:
        connection.close()
        raise RuntimeError("No heartbeat from the Pixhawk.")
    if connection.target_component == 0:
        connection.target_component = 1
    isArmed = bool(heartbeat.base_mode & middleMan.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    if isArmed:
        connection.close()
        raise RuntimeError("Copter is armed. Disarm before calibrating.")
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
        if messageId is None:
            continue
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


def writeCalibrationLineIfPrintOnly(text):
    print(text)


def detectCompassSlots(connection):
    internalSlot = None
    externalSlot = None
    for slotDefinition in COMPASS_SLOT_DEFINITIONS:
        deviceId = int(round(readParameter(connection, slotDefinition["deviceIdParam"])))
        externalFlag = int(round(readParameter(connection, slotDefinition["externalParam"])))
        slotInfo = {
            **slotDefinition,
            "deviceId": deviceId,
            "isExternal": externalFlag in (1, 2),
        }
        if deviceId == 0:
            continue
        if slotInfo["isExternal"]:
            externalSlot = slotInfo
        else:
            internalSlot = slotInfo
    if internalSlot is None or externalSlot is None:
        raise RuntimeError("Could not detect both the internal and external compasses.")
    return internalSlot, externalSlot


def applyCompassSettings(connection, middleMan, internalSlot, externalSlot):
    currentPriority1 = int(round(readParameter(connection, "COMPASS_PRIO1_ID")))
    currentPriority2 = int(round(readParameter(connection, "COMPASS_PRIO2_ID")))
    wantedPriority1 = internalSlot["deviceId"]
    wantedPriority2 = externalSlot["deviceId"]
    priorityChanged = currentPriority1 != wantedPriority1 or currentPriority2 != wantedPriority2
    integerTypes = (
        middleMan.mavlink.MAV_PARAM_TYPE_UINT8,
        middleMan.mavlink.MAV_PARAM_TYPE_INT8,
        middleMan.mavlink.MAV_PARAM_TYPE_UINT16,
        middleMan.mavlink.MAV_PARAM_TYPE_INT16,
        middleMan.mavlink.MAV_PARAM_TYPE_UINT32,
        middleMan.mavlink.MAV_PARAM_TYPE_INT32,
    )
    settings = (
        ("COMPASS_PRIO1_ID", wantedPriority1),
        ("COMPASS_PRIO2_ID", wantedPriority2),
        ("COMPASS_USE", 1),
        ("COMPASS_USE2", 1),
        ("COMPASS_ORIENT", 0),
        ("COMPASS_ORIENT2", 0),
        ("COMPASS_AUTO_ROT", DISABLE_STRICT_AUTO_ROTATION),
        ("COMPASS_CAL_FIT", RELAXED_COMPASS_CAL_FITNESS),
        ("COMPASS_OFFS_MAX", RELAXED_COMPASS_OFFSETS_MAX),
        ("ARMING_MAGTHRESH", RELAXED_ARMING_MAG_THRESHOLD),
        ("EK3_MAG_I_GATE", RELAXED_EKF_MAG_INNOVATION_GATE),
        ("EK3_SRC1_POSXY", EKF_SOURCE_GPS_VALUE),
        ("EK3_SRC1_VELXY", EKF_SOURCE_GPS_VALUE),
        ("EK3_SRC1_POSZ", EKF_SOURCE_BARO_VALUE),
        ("EK3_SRC1_YAW", EKF_SOURCE_COMPASS_VALUE),
        ("ARMING_SKIPCHK", ARMING_SKIP_RC_ONLY_VALUE),
        ("FS_GCS_ENABLE", GROUND_STATION_FAILSAFE_LAND_VALUE),
        ("FS_GCS_TIMEOUT", PREFERRED_FAILSAFE_TIMEOUT_SECONDS),
    )
    for parameterName, wantedValue in settings:
        writeParameter(connection, middleMan, integerTypes, parameterName, wantedValue)
    return priorityChanged


def rebootPixhawk(connection, middleMan):
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        middleMan.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0,
        1, 0, 0, 0, 0, 0, 0,
    )
    connection.close()
    time.sleep(REBOOT_WAIT_SECONDS)
    return connectToPixhawk(middleMan)


def readCompassSnapshot(connection):
    rawImu = None
    scaledImu2 = None
    attitude = None
    ekfReport = None
    deadline = time.monotonic() + 2.5
    while time.monotonic() < deadline:
        message = connection.recv_match(
            type=["RAW_IMU", "SCALED_IMU2", "ATTITUDE", "EKF_STATUS_REPORT"],
            blocking=True,
            timeout=0.3,
        )
        if message is None:
            continue
        messageType = message.get_type()
        if messageType == "RAW_IMU":
            rawImu = message
        elif messageType == "SCALED_IMU2":
            scaledImu2 = message
        elif messageType == "ATTITUDE":
            attitude = message
        elif messageType == "EKF_STATUS_REPORT":
            ekfReport = message
        if rawImu and scaledImu2 and attitude and ekfReport:
            break
    snapshot = {}
    if rawImu is not None and scaledImu2 is not None:
        primaryHeading = math.degrees(math.atan2(rawImu.ymag, rawImu.xmag))
        secondaryHeading = math.degrees(math.atan2(scaledImu2.ymag, scaledImu2.xmag))
        snapshot["primaryHeadingDegrees"] = round(primaryHeading, 1)
        snapshot["secondaryHeadingDegrees"] = round(secondaryHeading, 1)
        snapshot["headingDifferenceDegrees"] = round(
            (primaryHeading - secondaryHeading + 180.0) % 360.0 - 180.0,
            1,
        )
    if attitude is not None:
        snapshot["rollDegrees"] = round(math.degrees(attitude.roll), 2)
        snapshot["pitchDegrees"] = round(math.degrees(attitude.pitch), 2)
        snapshot["yawDegrees"] = round(math.degrees(attitude.yaw), 1)
    if ekfReport is not None:
        snapshot["compassVariance"] = round(float(ekfReport.compass_variance), 4)
    return snapshot


def drainOldCalibrationMessages(connection):
    deadline = time.monotonic() + DRAIN_OLD_PACKETS_TIMEOUT_SECONDS
    while time.monotonic() < deadline:
        if connection.recv_match(
            type=["MAG_CAL_PROGRESS", "MAG_CAL_REPORT", "COMMAND_ACK"],
            blocking=False,
        ) is None:
            return


def startMagCalibration(connection, middleMan):
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        middleMan.mavlink.MAV_CMD_DO_START_MAG_CAL,
        0,
        BOTH_COMPASSES_BITMASK,
        1,
        1,
        0, 0, 0, 0,
    )


def waitForMagCalibration(connection, middleMan, result):
    statusNames = {
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
    progressByCompass = {0: 0, 1: 0}
    reportsByCompass = {}
    lastPromptedStage = 1
    lastProgressText = ""
    startedAtSeconds = time.monotonic()
    while time.monotonic() - startedAtSeconds < COMPASS_CALIBRATION_TIMEOUT_SECONDS:
        message = connection.recv_match(
            type=["MAG_CAL_PROGRESS", "MAG_CAL_REPORT", "STATUSTEXT"],
            blocking=True,
            timeout=0.5,
        )
        if message is None:
            continue
        messageType = message.get_type()
        if messageType == "STATUSTEXT":
            text = str(message.text).strip("\x00").strip()
            if text and "PreArm:" not in text and "EKF variance" not in text:
                writeCalibrationLine(result, f"Pixhawk: {text}")
        elif messageType == "MAG_CAL_PROGRESS":
            compassId = int(message.compass_id)
            percent = int(message.completion_pct)
            progressByCompass[compassId] = percent
            lowestPercent = min(progressByCompass.get(0, 0), progressByCompass.get(1, 0))
            for threshold, prompt in SPIN_POSITIONS:
                stageNumber = SPIN_POSITIONS.index((threshold, prompt)) + 2
                if lowestPercent >= threshold and lastPromptedStage < stageNumber:
                    lastPromptedStage = stageNumber
                    writeCalibrationLine(result, prompt)
            statusName = statusNames.get(message.cal_status, f"STATUS {message.cal_status}")
            result["progress"] = {
                "primaryPercent": progressByCompass.get(0, 0),
                "secondaryPercent": progressByCompass.get(1, 0),
                "position": lastPromptedStage,
                "status": statusName,
            }
            progressText = (
                f"Primary {progressByCompass.get(0, 0)}% | "
                f"Secondary {progressByCompass.get(1, 0)}% | {statusName}"
            )
            if progressText != lastProgressText:
                writeCalibrationLine(result, progressText)
                lastProgressText = progressText
        elif messageType == "MAG_CAL_REPORT":
            compassId = int(message.compass_id)
            if compassId not in (0, 1):
                continue
            reportsByCompass[compassId] = compassReport(message, statusNames)
            writeCalibrationLine(
                result,
                (
                    f"Compass {compassId} report: {reportsByCompass[compassId]['status']} | "
                    f"fitness={reportsByCompass[compassId]['fitness']:.2f} | "
                    f"offsets=({reportsByCompass[compassId]['offsetX']:+.1f}, "
                    f"{reportsByCompass[compassId]['offsetY']:+.1f}, "
                    f"{reportsByCompass[compassId]['offsetZ']:+.1f})"
                ),
            )
            if 0 in reportsByCompass and 1 in reportsByCompass:
                break
    return [reportsByCompass[compassId] for compassId in (0, 1) if compassId in reportsByCompass]


def compassReport(message, statusNames):
    return {
        "compassId": int(message.compass_id),
        "status": statusNames.get(message.cal_status, f"STATUS {message.cal_status}"),
        "calStatus": int(message.cal_status),
        "fitness": float(message.fitness),
        "offsetX": float(message.ofs_x),
        "offsetY": float(message.ofs_y),
        "offsetZ": float(message.ofs_z),
    }


def reportSucceeded(reports, compassId, middleMan):
    for report in reports:
        if report.get("compassId") == compassId:
            return report.get("calStatus") == int(middleMan.mavlink.MAG_CAL_SUCCESS)
    return False


def acceptMagCalibration(connection, middleMan):
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        middleMan.mavlink.MAV_CMD_DO_ACCEPT_MAG_CAL,
        0,
        BOTH_COMPASSES_BITMASK,
        0, 0, 0, 0, 0, 0,
    )
    time.sleep(1.0)


def cancelMagCalibration(connection, middleMan):
    try:
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            middleMan.mavlink.MAV_CMD_DO_CANCEL_MAG_CAL,
            0,
            BOTH_COMPASSES_BITMASK,
            0, 0, 0, 0, 0, 0,
        )
    except Exception:
        pass


def parameterNameFromMessage(message):
    rawName = message.param_id
    if isinstance(rawName, bytes):
        return rawName.decode("ascii", errors="ignore").rstrip("\x00")
    return str(rawName).rstrip("\x00")


def readParameter(connection, parameterName):
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
            if parameterNameFromMessage(message) == parameterName:
                return float(message.param_value)
    raise RuntimeError(f"Timed out reading {parameterName}.")


def writeParameter(connection, middleMan, integerTypes, parameterName, wantedValue):
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
        if parameterNameFromMessage(message) == parameterName:
            existingMessage = message
            break
    parameterType = getattr(existingMessage, "param_type", middleMan.mavlink.MAV_PARAM_TYPE_REAL32)
    if parameterType in integerTypes:
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
            if parameterNameFromMessage(message) == parameterName:
                return float(message.param_value)
    return wireValue
