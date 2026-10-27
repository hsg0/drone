# WHAT — motor tests from the page.
# WHY — one motor can be checked on its own, then all four can be checked together.
# HOW — YES spins the named motor at a percent, or ramps all four from 0 to 20
#       percent over 30 seconds. Stop sends 0 percent. The result is saved.
# IMPORTANT — this uses the Pixhawk motor test. It does not arm the copter to fly.
#             Without YES the cable is not opened.

import json
import os
import threading
import time

from models.flightlogs import saveFlightLog

PIXHAWK_PORT = "/dev/serial0"
PIXHAWK_BAUD = 57600
GROUND_STATION_SYSTEM_ID = 255
HEARTBEAT_TIMEOUT_SECONDS = 15.0
MOTOR_TEST_THROTTLE_IS_PERCENT = 0
MOTOR_TEST_ONE_MOTOR = 0
ONE_MOTOR_MINIMUM_PERCENT = 10.0
ONE_MOTOR_MAXIMUM_PERCENT = 50.0
ALL_MOTORS_MAXIMUM_PERCENT = 20.0
ALL_MOTORS_TEST_SECONDS = 30.0
ONE_MOTOR_COMMAND_SECONDS = 3.0
ONE_MOTOR_REFRESH_SECONDS = 2.0
ONE_MOTOR_MAXIMUM_RUN_SECONDS = 30.0
ALL_MOTORS_COMMAND_SECONDS = 2.0
ALL_MOTORS_STEP_SECONDS = 1.0

MOTOR_TEST_ORDER = (
    {"testOrder": 1, "corner": "front-right", "spin": "counter-clockwise"},
    {"testOrder": 2, "corner": "back-right", "spin": "clockwise"},
    {"testOrder": 3, "corner": "back-left", "spin": "counter-clockwise"},
    {"testOrder": 4, "corner": "front-left", "spin": "clockwise"},
)

motorTestIsRunning = False
rampWasStopped = False
motorTestLock = threading.Lock()
motorTestConnection = None
motorTestHasCommanded = False
stopOneMotorRefresh = False
oneMotorStartedAtSeconds = 0.0
motorTestProgress = {
    "running": False,
    "mode": None,
    "testOrder": None,
    "corner": None,
    "spin": None,
    "percent": 0,
    "secondsLeft": None,
    "outcome": None,
    "stopReason": None,
}


async def spinOneMotor(request):
    try:
        if await confirmationText(request) != "YES":
            return cancelledMotorTest("The request did not say YES. No motor was spun.")
        body = await requestJson(request)
        testOrder = int(body.get("testOrder", 0))
        percent = float(body.get("percent", ONE_MOTOR_MINIMUM_PERCENT))
    except Exception:
        return failedMotorTest("The motor test request could not be read.")
    if testOrder not in (1, 2, 3, 4):
        return failedMotorTest("Choose motor test 1, 2, 3, or 4.")
    percent = min(ONE_MOTOR_MAXIMUM_PERCENT, max(ONE_MOTOR_MINIMUM_PERCENT, percent))
    busy = cableIsBusy()
    if busy:
        return failedMotorTest(busy)
    if motorTestIsRunning and motorTestProgress.get("mode") == "all":
        return failedMotorTest("The all-four motor test is already running.")
    try:
        return await runOnThread(lambda: commandOneMotor(testOrder, percent))
    except Exception as error:
        return failedMotorTest(str(error))


async def stopMotorTest(request):
    try:
        return await runOnThread(stopEveryMotor)
    except Exception as error:
        return failedMotorTest(str(error))


async def rampAllMotors(request):
    if await confirmationText(request) != "YES":
        return cancelledMotorTest("The request did not say YES. No motor was spun.")
    busy = cableIsBusy()
    if busy:
        return failedMotorTest(busy)
    with motorTestLock:
        if motorTestIsRunning and motorTestProgress.get("mode") == "all":
            return failedMotorTest("The all-four motor test is already running.")
    try:
        return await runOnThread(runAllMotorsRamp)
    except Exception as error:
        return failedMotorTest(str(error))


def readMotorTestProgress():
    try:
        return dict(motorTestProgress)
    except Exception as error:
        return failedMotorTest(str(error))


async def confirmationText(request):
    try:
        body = await requestJson(request)
        return str(body.get("confirm", "")).strip()
    except Exception:
        return ""


async def requestJson(request):
    rawBody = await request.body()
    if not rawBody:
        return {}
    payload = json.loads(rawBody)
    return payload if isinstance(payload, dict) else {}


async def runOnThread(work):
    import asyncio
    return await asyncio.to_thread(work)


def cableIsBusy():
    try:
        from controllers.flightController import flightIsRunning
        if flightIsRunning:
            return "A flight is already using the command cable."
    except Exception:
        pass
    try:
        from controllers.calibration import calibrationIsRunning
        if calibrationIsRunning:
            return "A compass calibration is already using the command cable."
    except Exception:
        pass
    try:
        from controllers.flightPlan import flightPlanIsRunning, flightPlanVehicle
        if flightPlanIsRunning or flightPlanVehicle is not None:
            return "A flight plan is already using the command cable."
    except Exception:
        pass
    return None


def commandOneMotor(testOrder, percent):
    global stopOneMotorRefresh, oneMotorStartedAtSeconds
    motor = motorByTestOrder(testOrder)
    connection = openMotorTestConnection()
    previousOrder = motorTestProgress.get("testOrder")
    if previousOrder not in (None, testOrder):
        sendMotorPercent(connection, previousOrder, 0, 0)
    sendMotorPercent(connection, testOrder, percent, ONE_MOTOR_COMMAND_SECONDS)
    oneMotorStartedAtSeconds = time.monotonic()
    publishMotorTest({
        "running": True,
        "mode": "one",
        "testOrder": testOrder,
        "corner": motor["corner"],
        "spin": motor["spin"],
        "percent": percent,
        "secondsLeft": None,
        "outcome": "RUNNING",
        "stopReason": None,
    })
    stopOneMotorRefresh = False
    if not getattr(commandOneMotor, "refreshStarted", False):
        commandOneMotor.refreshStarted = True
        threading.Thread(target=refreshOneMotor, daemon=True).start()
    return dict(motorTestProgress)


def refreshOneMotor():
    while True:
        time.sleep(ONE_MOTOR_REFRESH_SECONDS)
        if stopOneMotorRefresh:
            continue
        if not motorTestIsRunning or motorTestProgress.get("mode") != "one":
            continue
        if time.monotonic() - oneMotorStartedAtSeconds >= ONE_MOTOR_MAXIMUM_RUN_SECONDS:
            stopEveryMotor()
            continue
        connection = motorTestConnection
        testOrder = motorTestProgress.get("testOrder")
        percent = motorTestProgress.get("percent")
        if connection is None or testOrder is None:
            continue
        try:
            sendMotorPercent(connection, testOrder, percent, ONE_MOTOR_COMMAND_SECONDS)
        except Exception as error:
            publishMotorTest({
                "running": False,
                "outcome": "FAILED",
                "stopReason": str(error),
            })


def runAllMotorsRamp():
    global rampWasStopped, stopOneMotorRefresh
    rampWasStopped = False
    stopOneMotorRefresh = True
    connection = openMotorTestConnection()
    if motorTestProgress.get("mode") == "one":
        sendMotorPercent(connection, motorTestProgress.get("testOrder") or 1, 0, 0)
    startedAt = time.monotonic()
    publishMotorTest({
        "running": True,
        "mode": "all",
        "testOrder": None,
        "corner": "all four",
        "spin": "together",
        "percent": 0,
        "secondsLeft": ALL_MOTORS_TEST_SECONDS,
        "outcome": "RUNNING",
        "stopReason": None,
    })
    try:
        while True:
            elapsedSeconds = time.monotonic() - startedAt
            if elapsedSeconds >= ALL_MOTORS_TEST_SECONDS or rampWasStopped:
                break
            percent = ALL_MOTORS_MAXIMUM_PERCENT * (elapsedSeconds / ALL_MOTORS_TEST_SECONDS)
            for motor in MOTOR_TEST_ORDER:
                sendMotorPercent(connection, motor["testOrder"], percent, ALL_MOTORS_COMMAND_SECONDS)
            publishMotorTest({
                "running": True,
                "mode": "all",
                "percent": round(percent, 1),
                "secondsLeft": round(ALL_MOTORS_TEST_SECONDS - elapsedSeconds, 1),
                "outcome": "RUNNING",
            })
            time.sleep(ALL_MOTORS_STEP_SECONDS)
        if rampWasStopped:
            return {"scriptName": "all4MotorsTest", "started": True, "outcome": "STOPPED", "percent": 0}
        silenceMotors()
        result = {
            "scriptName": "all4MotorsTest",
            "started": True,
            "outcome": "SUCCESS",
            "percent": ALL_MOTORS_MAXIMUM_PERCENT,
            "seconds": ALL_MOTORS_TEST_SECONDS,
        }
        result["id"] = saveFlightLog(result)
        publishMotorTest({"running": False, "mode": "all", "percent": ALL_MOTORS_MAXIMUM_PERCENT, "secondsLeft": 0, "outcome": "SUCCESS"})
        return result
    except Exception as error:
        silenceMotors()
        failed = failedMotorTest(str(error))
        failed["scriptName"] = "all4MotorsTest"
        return failed


def stopEveryMotor():
    global rampWasStopped
    rampWasStopped = True
    silenceMotors()
    result = {
        "scriptName": "motorTest",
        "started": motorTestIsRunning,
        "outcome": "STOPPED",
        "percent": 0,
    }
    if motorTestIsRunning:
        result["id"] = saveFlightLog(result)
    publishMotorTest({
        "running": False,
        "mode": None,
        "testOrder": None,
        "percent": 0,
        "secondsLeft": 0,
        "outcome": "STOPPED",
        "stopReason": None,
    })
    return result


def silenceMotors():
    global stopOneMotorRefresh
    stopOneMotorRefresh = True
    connection = motorTestConnection
    if connection is not None:
        for motor in MOTOR_TEST_ORDER:
            try:
                sendMotorPercent(connection, motor["testOrder"], 0, 0)
            except Exception:
                pass
    closeMotorTestConnection()


def openMotorTestConnection():
    global motorTestConnection, motorTestHasCommanded
    if motorTestConnection is not None:
        return motorTestConnection
    if not os.path.exists(PIXHAWK_PORT):
        raise RuntimeError(f"{PIXHAWK_PORT} is missing. Run the backend on the Raspberry Pi.")
    from pymavlink import mavutil as middleMan
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
    if isArmed and not motorTestHasCommanded:
        connection.close()
        raise RuntimeError("The copter is armed to fly. Disarm before a motor test.")
    connection.middleMan = middleMan
    motorTestConnection = connection
    return connection


def closeMotorTestConnection():
    global motorTestConnection, motorTestHasCommanded
    connection = motorTestConnection
    motorTestConnection = None
    motorTestHasCommanded = False
    if connection is None:
        return
    try:
        connection.close()
    except Exception:
        pass


def sendMotorPercent(connection, testOrder, percent, runSeconds):
    global motorTestHasCommanded
    middleMan = connection.middleMan
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        middleMan.mavlink.MAV_CMD_DO_MOTOR_TEST,
        0,
        float(testOrder),
        MOTOR_TEST_THROTTLE_IS_PERCENT,
        float(percent),
        float(runSeconds),
        MOTOR_TEST_ONE_MOTOR,
        0,
        0,
    )
    motorTestHasCommanded = True


def publishMotorTest(changes):
    global motorTestIsRunning
    motorTestProgress.update(changes)
    motorTestIsRunning = bool(motorTestProgress.get("running"))


def motorByTestOrder(testOrder):
    for motor in MOTOR_TEST_ORDER:
        if motor["testOrder"] == testOrder:
            return motor
    raise RuntimeError("Choose motor test 1, 2, 3, or 4.")


def cancelledMotorTest(reason):
    result = {
        "scriptName": "motorTest",
        "started": False,
        "outcome": "CANCELLED BEFORE START",
        "stopReason": reason,
        "running": False,
        "percent": 0,
    }
    result["id"] = saveFlightLog(result)
    return result


def failedMotorTest(reason):
    result = {
        "scriptName": "motorTest",
        "started": False,
        "outcome": "FAILED",
        "stopReason": reason,
        "running": False,
        "percent": 0,
    }
    result["id"] = saveFlightLog(result)
    return result
