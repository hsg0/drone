# WHAT — the actions behind the flight routes.
# WHY — the page asks for logs and for the east-west-north-south flight here.
#       A bad request or a failed save must not stop the backend.
# HOW — list, read, and save call the flight log model. fly10FeetEWNSLand runs
#       the Pixhawk flight, then the model stores the outcome.
# IMPORTANT — the flight arms only when the request body says {"confirm": "YES"}.
#             There is no force-arm and no force-disarm.

import asyncio
import json
import math
import os
import threading
import time

from models.flightlogs import readFlightLog, readFlightLogs, recordFromFlightLog, saveFlightLog


def listFlightLogs():
    try:
        flightLogs = [recordFromFlightLog(flightLog) for flightLog in readFlightLogs()]
        return {"flightLogs": flightLogs}
    except Exception as error:
        print(f"Flight logs were not listed: {error}")
        return {"flightLogs": []}


def readOneFlightLog(flightLogId):
    try:
        flightLog = readFlightLog(flightLogId)
        if flightLog is None:
            return {"flightLog": None}
        return {"flightLog": recordFromFlightLog(flightLog)}
    except Exception as error:
        print(f"Flight log {flightLogId} was not read: {error}")
        return {"flightLog": None}


async def saveOneFlightLog(request):
    try:
        rawBody = await request.body()
        if not rawBody:
            payload = None
        else:
            try:
                payload = json.loads(rawBody)
            except Exception:
                payload = rawBody.decode("utf-8", errors="replace")
        flightLogId = saveFlightLog(payload)
        return {"saved": flightLogId is not None, "id": flightLogId}
    except Exception as error:
        print(f"Flight log was not saved from the route: {error}")
        return {"saved": False, "id": None}


# --------------------------------------------------
# EAST WEST NORTH SOUTH FLIGHT
# Same gates as the Oct 26 north flight that stayed inside about 1 foot.
# Takeoff is 5 feet, the height this mission hovers and flies at.
# --------------------------------------------------

PIXHAWK_PORT = "/dev/serial0"
PIXHAWK_BAUD = 57600
CONNECTION_TIMEOUT_SECONDS = 60
GROUND_STATION_SYSTEM_ID = 255

FEET_PER_METER = 3.28084
HOVER_ALTITUDE_FEET = 5.0
HOVER_ALTITUDE_METERS = HOVER_ALTITUDE_FEET / FEET_PER_METER
TAKEOFF_REACHED_FRACTION = 0.95
TAKEOFF_SETTLE_SECONDS = 2.0
MAXIMUM_ALTITUDE_CEILING_FEET = 10.0
MAXIMUM_ALTITUDE_CEILING_METERS = MAXIMUM_ALTITUDE_CEILING_FEET / FEET_PER_METER
HOVER_DURATION_SECONDS = 10.0
MAXIMUM_HOVER_DRIFT_FEET = 16.0
MAXIMUM_HOVER_DRIFT_METERS = MAXIMUM_HOVER_DRIFT_FEET / FEET_PER_METER
LEG_DISTANCE_FEET = 10.0
LEG_DISTANCE_METERS = LEG_DISTANCE_FEET / FEET_PER_METER
LEG_SPEED_METERS_PER_SECOND = 0.4
ALTITUDE_HOLD_GAIN_DURING_MOVE = 0.5
MAXIMUM_VERTICAL_CORRECTION_METERS_PER_SECOND = 0.25
MAXIMUM_OFF_COURSE_FEET = 16.0
MAXIMUM_OFF_COURSE_METERS = MAXIMUM_OFF_COURSE_FEET / FEET_PER_METER
LEG_TIMEOUT_SECONDS = 30.0
SETTLE_AFTER_MOVE_SECONDS = 2.0
NORTH_HEADING_DEGREES = 0.0
NORTH_HEADING_TOLERANCE_DEGREES = 15.0
YAW_TURN_RATE_DEGREES_PER_SECOND = 30.0
YAW_ALIGN_TIMEOUT_SECONDS = 15.0
YAW_SETTLE_SECONDS = 1.0
METERS_PER_DEGREE_OF_LATITUDE = 111319.5
MINIMUM_VALID_COORDINATE_DEGREES = 0.01
TAKEOFF_TIMEOUT_SECONDS = 30.0
LAND_TIMEOUT_SECONDS = 60.0
PREFLIGHT_WAIT_TIMEOUT_SECONDS = 180.0
MODE_CHANGE_TIMEOUT_SECONDS = 10.0
ARM_TIMEOUT_SECONDS = 15.0
HEARTBEAT_STALE_AFTER_SECONDS = 3.0
MAXIMUM_GROUND_LEAN_DEGREES = 10.0
MAXIMUM_SAVED_TRIM_DEGREES = 8.0
STATUS_CHECK_INTERVAL_SECONDS = 0.2
PRINT_INTERVAL_SECONDS = 1.0
PARAMETER_TOLERANCE = 0.01
QUAD_FRAME_CLASS = 1
QUAD_X_FRAME_TYPE = 1
MINIMUM_GPS_FIX_TYPE = 3
MINIMUM_SATELLITE_COUNT = 5
UNKNOWN_SATELLITE_COUNT = 255
FALSE_BATTERY_WARNING = "battery 1 low voltage"
BATTERY_LOW_FAILSAFE_ACTION_PARAMETER = "BATT_FS_LOW_ACT"
BATTERY_CRITICAL_FAILSAFE_ACTION_PARAMETER = "BATT_FS_CRT_ACT"
BATTERY_FAILSAFE_ACTION_NONE = 0
COMPASS_PRIORITY_REBOOT_WAIT_SECONDS = 12.0
COMPASS_AGREEMENT_LIMIT_DEGREES = 11.0
COMPASS_FIELD_MINIMUM = 100.0
COMPASS_SLOT_PARAMETERS = (
    ("COMPASS_DEV_ID", "COMPASS_EXTERNAL", "COMPASS_USE", "SCALED_IMU"),
    ("COMPASS_DEV_ID2", "COMPASS_EXTERN2", "COMPASS_USE2", "SCALED_IMU2"),
)
SKIP_COMPASS_OR_GPS_MASK = 4 | 8 | 4096
MAXIMUM_COMPASS_VARIANCE = 0.50
PREARM_POLL_INTERVAL_SECONDS = 3.0
TAKEOFF_SLEW_PARAMETER_NAME = "TKOFF_SLEW_TIME"
PREFERRED_TAKEOFF_SLEW_SECONDS = 1.0
GROUND_STATION_FAILSAFE_PARAMETER_NAME = "FS_GCS_ENABLE"
GROUND_STATION_FAILSAFE_TIMEOUT_PARAMETER_NAME = "FS_GCS_TIMEOUT"
FAILSAFE_OPTIONS_PARAMETER_NAME = "FS_OPTIONS"
GROUND_STATION_FAILSAFE_LAND_VALUE = 5
CONTINUE_PILOT_MODE_ON_FAILSAFE_BIT = 16
PREFERRED_FAILSAFE_TIMEOUT_SECONDS = 5.0
MESSAGE_INTERVAL_SECONDS = 0.1
MICROSECONDS_PER_SECOND = 1000000
FLIGHT_MESSAGE_NAMES = (
    "GPS_RAW_INT",
    "GLOBAL_POSITION_INT",
    "EKF_STATUS_REPORT",
    "ATTITUDE",
    "SYS_STATUS",
    "STATUSTEXT",
)
# Nose stays north. East, west, north, and south are Earth directions.
# MAVLink frame 1 is local North-East-Down. Positive east speed flies east.
MAV_FRAME_LOCAL_NED = 1
CARDINAL_LEGS = (
    ("east", 0.0, LEG_SPEED_METERS_PER_SECOND),
    ("west", 0.0, -LEG_SPEED_METERS_PER_SECOND),
    ("north", LEG_SPEED_METERS_PER_SECOND, 0.0),
    ("south", -LEG_SPEED_METERS_PER_SECOND, 0.0),
)

flightIsRunning = False
flightGuard = asyncio.Lock()
flightStopRequested = False
flightReturnHomeRequested = False
flightClearRequested = False
activeFlightVehicle = None
activeFlightOutcome = None
flightProgressLock = threading.RLock()


class FlightStopRequested(Exception):
    """The pilot pressed Stop. The mission must land and end."""


class FlightReturnHomeRequested(Exception):
    """The pilot pressed Return to Home. Fly back to the takeoff spot and land."""


class FlightClearRequested(Exception):
    """The pilot pressed Clear flights. End the request before takeoff."""

flightProgressLoop = None
flightProgressQueues = set()
latestFlightProgress = {
    "running": False,
    "scriptName": None,
    "outcome": None,
    "latestLine": None,
    "repeatCount": 0,
    "lines": [],
    "stopReason": None,
    "startedAt": None,
}


async def fly10FeetEWNSLand(request):
    global flightIsRunning
    try:
        confirmationText = await confirmationTextFromRequest(request)
        if confirmationText != "YES":
            cancelledOutcome = {
                "scriptName": "fly10FeetEWNSLand",
                "started": False,
                "outcome": "CANCELLED BEFORE ARMING",
                "stopReason": "The request did not say YES. The copter was not armed.",
            }
            cancelledOutcome["id"] = saveFlightLog(cancelledOutcome)
            return cancelledOutcome

        from controllers.calibration import calibrationIsRunning

        async with flightGuard:
            if flightIsRunning:
                return {
                    "scriptName": "fly10FeetEWNSLand",
                    "started": False,
                    "outcome": "A FLIGHT IS ALREADY RUNNING",
                }
            if calibrationIsRunning:
                return {
                    "scriptName": "fly10FeetEWNSLand",
                    "started": False,
                    "outcome": "A COMPASS CALIBRATION IS ALREADY RUNNING",
                }
            from controllers.motorTest import motorTestIsRunning
            if motorTestIsRunning:
                return {
                    "scriptName": "fly10FeetEWNSLand",
                    "started": False,
                    "outcome": "A MOTOR TEST IS ALREADY RUNNING",
                }
            from controllers.flightPlan import flightPlanIsRunning, flightPlanVehicle
            if flightPlanIsRunning or flightPlanVehicle is not None:
                return {
                    "scriptName": "fly10FeetEWNSLand",
                    "started": False,
                    "outcome": "A FLIGHT PLAN IS ALREADY USING THE COMMAND CABLE",
                }
            resetFlightControlFlags()
            flightIsRunning = True
        beginFlightProgress("fly10FeetEWNSLand")

        try:
            flightOutcome = await asyncio.to_thread(runFly10FeetEastWestNorthSouthLand)
        finally:
            async with flightGuard:
                flightIsRunning = False

        flightOutcome["id"] = saveFlightLog(flightOutcome)
        publishFlightSnapshot(flightOutcome, running=False)
        return flightOutcome
    except Exception as error:
        print(f"fly10FeetEWNSLand did not finish: {error}")
        failedOutcome = {
            "scriptName": "fly10FeetEWNSLand",
            "started": False,
            "outcome": "FAILED",
            "stopReason": str(error),
        }
        failedOutcome["id"] = saveFlightLog(failedOutcome)
        return failedOutcome


async def confirmationTextFromRequest(request):
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


def resetFlightControlFlags():
    global flightStopRequested, flightReturnHomeRequested, flightClearRequested, activeFlightVehicle, activeFlightOutcome
    flightStopRequested = False
    flightReturnHomeRequested = False
    flightClearRequested = False
    activeFlightVehicle = None
    activeFlightOutcome = None


def runFly10FeetEastWestNorthSouthLand():
    global activeFlightVehicle, activeFlightOutcome
    flightOutcome = {
        "scriptName": "fly10FeetEWNSLand",
        "started": True,
        "startedAt": time.strftime("%Y-%m-%d %H:%M:%S %Z"),
        "finishedAt": None,
        "outcome": "FAILED",
        "maximumAltitudeReachedText": "none",
        "stopReason": None,
        "lines": [],
        "steps": {},
    }
    activeFlightOutcome = flightOutcome
    vehicle = None
    originalTakeoffSlewSeconds = None
    savedFailsafe = None
    copterWasArmed = False
    landingWasStarted = False
    droneKitTools = None

    try:
        writeFlightLine(flightOutcome, "TAKE OFF TO 5 FT -> HOVER 10S -> EAST 10 FT -> HOVER 10S -> WEST 10 FT -> HOVER 10S -> NORTH 10 FT -> HOVER 10S -> SOUTH 10 FT -> HOVER 10S -> LAND")
        if not os.path.exists(PIXHAWK_PORT):
            raise RuntimeError(f"{PIXHAWK_PORT} is missing. Run the backend on the Raspberry Pi.")

        from dronekit import VehicleMode
        from dronekit import connect
        from pymavlink import mavutil as middleMan

        droneKitTools = (VehicleMode, connect, middleMan)
        velocityTypeMask = (
            middleMan.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            | middleMan.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | middleMan.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | middleMan.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | middleMan.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | middleMan.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | middleMan.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            | middleMan.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        prearmHealthBit = getattr(middleMan.mavlink, "MAV_SYS_STATUS_PREARM_CHECK", 268435456)
        runPrearmCommandId = getattr(middleMan.mavlink, "MAV_CMD_RUN_PREARM_CHECKS", 401)

        writeFlightLine(flightOutcome, f"Connecting to Pixhawk on {PIXHAWK_PORT} at {PIXHAWK_BAUD} baud.")
        vehicle = connectUntilCleared(connect, flightOutcome)
        if vehicle._master.target_component == 0:
            vehicle._master.target_component = 1
        writeFlightLine(flightOutcome, "Pixhawk connected.")
        activeFlightVehicle = vehicle
        activeFlightOutcome = flightOutcome
        raiseIfFlightStopped()
        vehicle = ensureBoardCompassIsPrimary(
            vehicle,
            lambda currentVehicle: rebootFlightVehicle(currentVehicle, connect, flightOutcome),
            lambda text: writeFlightLine(flightOutcome, text),
            confirmPixhawkParameter,
        )
        activeFlightVehicle = vehicle
        latestTelemetry = listenForPreflightMessages(vehicle, prearmHealthBit, flightOutcome)
        requestFlightStreams(vehicle, middleMan)
        chooseCompassesForThisFlight(vehicle, lambda text: writeFlightLine(flightOutcome, text), confirmPixhawkParameter)
        waitUntilPreflightPasses(vehicle, latestTelemetry, runPrearmCommandId, flightOutcome)

        savedSlew = vehicle.parameters[TAKEOFF_SLEW_PARAMETER_NAME]
        if savedSlew is not None:
            savedSlew = float(savedSlew)
            takeoffSlew = min(savedSlew, PREFERRED_TAKEOFF_SLEW_SECONDS)
            writeFlightLine(flightOutcome, f"{TAKEOFF_SLEW_PARAMETER_NAME}: original {savedSlew:.2f}s | takeoff {takeoffSlew:.2f}s")
            if abs(takeoffSlew - savedSlew) >= PARAMETER_TOLERANCE:
                confirmPixhawkParameter(vehicle, TAKEOFF_SLEW_PARAMETER_NAME, takeoffSlew)
                originalTakeoffSlewSeconds = savedSlew

        savedFailsafe = armLandFailsafe(vehicle, flightOutcome)
        recheckPreflightAfterFailsafe(vehicle, latestTelemetry)
        flightOutcome["steps"]["preflight"] = "SUCCESS"

        commandFlightMode(vehicle, "GUIDED", VehicleMode, flightOutcome)
        copterWasArmed = True
        armAndWait(vehicle, flightOutcome)
        climbToFiveFeet(vehicle, flightOutcome)
        settleAtFiveFeet(vehicle, flightOutcome)
        flightOutcome["steps"]["takeoff"] = "SUCCESS"
        commandFlightMode(vehicle, "LOITER", VehicleMode, flightOutcome)
        holdHoverAtFiveFeet(vehicle, flightOutcome, "after takeoff")

        commandFlightMode(vehicle, "GUIDED", VehicleMode, flightOutcome)
        faceGeographicNorth(vehicle, middleMan, velocityTypeMask, flightOutcome)
        for directionName, northSpeed, eastSpeed in CARDINAL_LEGS:
            if vehicle.mode.name != "GUIDED":
                commandFlightMode(vehicle, "GUIDED", VehicleMode, flightOutcome)
            flyTenFeet(vehicle, flightOutcome, directionName, northSpeed, eastSpeed, velocityTypeMask)
            commandFlightMode(vehicle, "LOITER", VehicleMode, flightOutcome)
            holdHoverAtFiveFeet(vehicle, flightOutcome, f"after {directionName}")

        landingWasStarted = True
        landUntilPropellersStop(vehicle, flightOutcome, VehicleMode)
        copterWasArmed = vehicle.armed
        if not vehicle.armed:
            flightOutcome["outcome"] = "SUCCESS"
            flightOutcome["steps"]["land"] = "SUCCESS"
            writeFlightLine(flightOutcome, "Flight complete.")
    except KeyboardInterrupt:
        writeFlightLine(flightOutcome, "Flight interrupted.")
        flightOutcome["outcome"] = "EXITED MID-WAY"
        flightOutcome["stopReason"] = "Interrupted."
    except FlightClearRequested:
        flightOutcome["outcome"] = "CLEARED BEFORE FLYING"
        flightOutcome["stopReason"] = "Clear flights pressed. The request ended before takeoff."
        writeFlightLine(flightOutcome, "Flight request cleared before takeoff.")
    except FlightReturnHomeRequested:
        flightOutcome["outcome"] = "RETURNED HOME"
        flightOutcome["stopReason"] = "Return to home pressed. The copter was sent back to the takeoff spot to land."
        writeFlightLine(flightOutcome, "Return to home pressed. Flying back to the takeoff spot, then landing.")
        if vehicle is not None and getattr(vehicle, "armed", False) and not landingWasStarted:
            landingWasStarted = True
            try:
                from dronekit import VehicleMode
                returnHomeUntilLanded(vehicle, flightOutcome, VehicleMode)
            except Exception as landError:
                writeFlightLine(flightOutcome, f"Return home did not finish: {landError}. Landing here.")
                try:
                    from dronekit import VehicleMode
                    landUntilPropellersStop(vehicle, flightOutcome, VehicleMode)
                except Exception as secondError:
                    writeFlightLine(flightOutcome, f"Landing after return home did not finish: {secondError}")
    except FlightStopRequested:
        flightOutcome["outcome"] = "STOPPED BY PILOT"
        flightOutcome["stopReason"] = "Stop pressed. The copter was told to land."
        writeFlightLine(flightOutcome, "Stop pressed. Landing now.")
        if vehicle is not None and getattr(vehicle, "armed", False) and not landingWasStarted:
            landingWasStarted = True
            try:
                from dronekit import VehicleMode
                landUntilPropellersStop(vehicle, flightOutcome, VehicleMode)
            except Exception as landError:
                writeFlightLine(flightOutcome, f"Landing after stop did not finish: {landError}")
    except Exception as error:
        writeFlightLine(flightOutcome, f"Flight stopped: {error}")
        flightOutcome["outcome"] = "FAILED"
        flightOutcome["stopReason"] = str(error)
    finally:
        finishFlightConnection(
            vehicle,
            droneKitTools,
            flightOutcome,
            copterWasArmed,
            landingWasStarted,
            savedFailsafe,
            originalTakeoffSlewSeconds,
        )
    return flightOutcome


def writeFlightLine(flightOutcome, text):
    print(text)
    with flightProgressLock:
        lines = flightOutcome.setdefault("lines", [])
        if lines and lines[-1] == text:
            flightOutcome["repeatCount"] = int(flightOutcome.get("repeatCount") or 1) + 1
        else:
            lines.append(text)
            flightOutcome["repeatCount"] = 1
        publishFlightSnapshot(flightOutcome, running=True)


def raiseIfFlightStopped():
    if flightClearRequested:
        raise FlightClearRequested()
    if flightReturnHomeRequested:
        raise FlightReturnHomeRequested()
    if flightStopRequested:
        raise FlightStopRequested()


def connectUntilCleared(connect, flightOutcome):
    """Connect in short attempts so Clear flights can end a request before takeoff."""
    deadline = time.monotonic() + CONNECTION_TIMEOUT_SECONDS
    lastError = None
    while time.monotonic() < deadline:
        raiseIfFlightStopped()
        attemptSeconds = min(3.0, max(0.5, deadline - time.monotonic()))
        try:
            return connect(
                PIXHAWK_PORT,
                baud=PIXHAWK_BAUD,
                wait_ready=True,
                timeout=attemptSeconds,
                source_system=GROUND_STATION_SYSTEM_ID,
            )
        except (FlightStopRequested, FlightReturnHomeRequested, FlightClearRequested):
            raise
        except Exception as error:
            lastError = error
            time.sleep(0.2)
    raise RuntimeError(f"Could not connect to the Pixhawk. {lastError}")


async def stopFly10FeetEWNSLand():
    """Ask the running east-west-north-south flight to land immediately."""
    global flightStopRequested, flightReturnHomeRequested, flightClearRequested
    if not flightIsRunning:
        return {
            "stopped": False,
            "scriptName": "fly10FeetEWNSLand",
            "outcome": "NO EWNS FLIGHT IS RUNNING",
            "message": "No east-west-north-south flight is running.",
        }
    flightReturnHomeRequested = False
    flightClearRequested = False
    flightStopRequested = True
    vehicle = activeFlightVehicle
    outcome = activeFlightOutcome
    if outcome is not None:
        writeFlightLine(outcome, "Stop pressed. Landing now.")
    if vehicle is None or getattr(vehicle, "commandCableClosed", False):
        return {
            "stopped": True,
            "scriptName": "fly10FeetEWNSLand",
            "outcome": "STOP REQUESTED",
            "message": "Stop pressed. The flight will end without taking off.",
        }
    try:
        from dronekit import VehicleMode
        vehicle.mode = VehicleMode("LAND")
    except Exception as error:
        return {
            "stopped": True,
            "scriptName": "fly10FeetEWNSLand",
            "outcome": "STOP REQUESTED",
            "message": f"Stop pressed. LAND was not confirmed: {error}",
        }
    return {
        "stopped": True,
        "scriptName": "fly10FeetEWNSLand",
        "outcome": "LANDING",
        "message": "Stop pressed. LAND sent. The copter is landing now.",
    }


async def clearFly10FeetEWNSLand():
    """Drop an east-west-north-south request that has not taken off."""
    global flightStopRequested, flightReturnHomeRequested, flightClearRequested
    vehicle = activeFlightVehicle
    armed = (
        vehicle is not None
        and getattr(vehicle, "armed", False)
        and not getattr(vehicle, "commandCableClosed", False)
    )
    if armed:
        return {
            "cleared": False,
            "scriptName": "fly10FeetEWNSLand",
            "outcome": "AIRBORNE",
            "message": "The copter is already in the air. Use Stop and land or Return to Home Location.",
        }
    wasRunning = bool(flightIsRunning)
    if wasRunning:
        flightStopRequested = False
        flightReturnHomeRequested = False
        flightClearRequested = True
        if activeFlightOutcome is not None:
            writeFlightLine(activeFlightOutcome, "Clear flights pressed. Ending the request before takeoff.")
        deadline = time.monotonic() + 8.0
        while flightIsRunning and time.monotonic() < deadline:
            await asyncio.sleep(0.2)
    if flightIsRunning:
        return {
            "cleared": False,
            "stillRunning": True,
            "scriptName": "fly10FeetEWNSLand",
            "outcome": "STILL CLOSING",
            "message": "Clear was sent. The open request is still closing. Wait a moment, then press Fly.",
        }
    return {
        "cleared": True,
        "scriptName": "fly10FeetEWNSLand",
        "outcome": "CLEARED",
        "message": "Flight requests cleared. Press Fly when ready." if wasRunning else "No flight request was waiting. Press Fly when ready.",
    }


async def returnFly10FeetEWNSLandHome():
    """Send the running east-west-north-south flight back to the takeoff spot, then land."""
    global flightStopRequested, flightReturnHomeRequested, flightClearRequested
    if not flightIsRunning:
        return {
            "returning": False,
            "scriptName": "fly10FeetEWNSLand",
            "outcome": "NO EWNS FLIGHT IS RUNNING",
            "message": "No east-west-north-south flight is running.",
        }
    flightStopRequested = False
    flightReturnHomeRequested = True
    flightClearRequested = False
    vehicle = activeFlightVehicle
    outcome = activeFlightOutcome
    if outcome is not None:
        writeFlightLine(outcome, "Return to home pressed. Flying back to the takeoff spot, then landing.")
    if vehicle is None or getattr(vehicle, "commandCableClosed", False) or not getattr(vehicle, "armed", False):
        return {
            "returning": True,
            "scriptName": "fly10FeetEWNSLand",
            "outcome": "RETURN REQUESTED",
            "message": "Return to home pressed. The copter has not taken off, so the flight will end.",
        }
    try:
        from dronekit import VehicleMode
        vehicle.mode = VehicleMode("RTL")
    except Exception as error:
        return {
            "returning": True,
            "scriptName": "fly10FeetEWNSLand",
            "outcome": "RETURN REQUESTED",
            "message": f"Return to home pressed. RTL was not confirmed: {error}",
        }
    return {
        "returning": True,
        "scriptName": "fly10FeetEWNSLand",
        "outcome": "RETURNING HOME",
        "message": "Return to home sent. The copter is flying back to the takeoff spot, then landing.",
    }


def holdCurrentAltitudeForReturn(vehicle, flightOutcome):
    """Keep the return at the current height. RTL_ALT is centimeters, and 0 means do not climb first."""
    if getattr(vehicle, "savedReturnAltitudeCentimeters", None) is None:
        currentValue = vehicle.parameters["RTL_ALT"]
        vehicle.savedReturnAltitudeCentimeters = None if currentValue is None else float(currentValue)
    confirmPixhawkParameter(vehicle, "RTL_ALT", 0)
    if flightOutcome is not None:
        writeFlightLine(flightOutcome, "Return altitude held at the current height.")


def returnHomeUntilLanded(vehicle, flightOutcome, vehicleMode):
    holdCurrentAltitudeForReturn(vehicle, flightOutcome)
    writeFlightLine(flightOutcome, "Returning to the takeoff location, then landing.")
    commandFlightMode(vehicle, "RTL", vehicleMode, flightOutcome)
    landStartTime = time.monotonic()
    while vehicle.armed:
        if flightStopRequested:
            flightOutcome["outcome"] = "STOPPED BY PILOT"
            flightOutcome["stopReason"] = "Stop pressed during return home. Landing here."
            writeFlightLine(flightOutcome, "Stop pressed during return home. Landing here.")
            landUntilPropellersStop(vehicle, flightOutcome, vehicleMode)
            return
        if not pixhawkHeartbeatIsFresh(vehicle):
            writeFlightLine(flightOutcome, "Heartbeat lost during return home. Closing the cable so the onboard failsafe finishes the landing.")
            vehicle.commandCableClosed = True
            vehicle.close()
            flightOutcome["steps"]["land"] = "ONBOARD FAILSAFE"
            return
        if time.monotonic() - landStartTime > LAND_TIMEOUT_SECONDS:
            raise RuntimeError("Return home did not finish. The motors are still armed.")
        location = vehicle.location.global_relative_frame
        heightText = "waiting" if location is None or location.alt is None else f"{float(location.alt) * FEET_PER_METER:.2f} ft"
        writeFlightLine(flightOutcome, f"Returning home. {heightText}")
        time.sleep(PRINT_INTERVAL_SECONDS)
    writeFlightLine(flightOutcome, "Back at the takeoff spot. Motors disarmed and propellers stopped.")


def beginFlightProgress(scriptName):
    publishFlightSnapshot(
        {
            "scriptName": scriptName,
            "startedAt": time.strftime("%Y-%m-%d %H:%M:%S %Z"),
            "lines": ["YES received. Opening the Pixhawk."],
            "repeatCount": 1,
            "outcome": None,
            "stopReason": None,
        },
        running=True,
    )


def rememberFlightProgressLoop(loop):
    global flightProgressLoop
    flightProgressLoop = loop


def readFlightProgress():
    return dict(latestFlightProgress)


def publishFlightSnapshot(flightOutcome, running):
    global latestFlightProgress
    lines = list(flightOutcome.get("lines") or [])
    latestFlightProgress = {
        "running": bool(running),
        "scriptName": flightOutcome.get("scriptName"),
        "outcome": None if running else flightOutcome.get("outcome"),
        "latestLine": lines[-1] if lines else None,
        "repeatCount": int(flightOutcome.get("repeatCount") or (1 if lines else 0)),
        "lines": lines[-80:],
        "stopReason": None if running else flightOutcome.get("stopReason"),
        "startedAt": flightOutcome.get("startedAt"),
    }
    sendFlightProgress(latestFlightProgress)


def sendFlightProgress(snapshot):
    if flightProgressLoop is None:
        return
    outgoing = dict(snapshot)
    outgoing["lines"] = list(snapshot.get("lines") or [])
    for progressQueue in list(flightProgressQueues):
        try:
            flightProgressLoop.call_soon_threadsafe(progressQueue.put_nowait, outgoing)
        except Exception as error:
            print(f"Flight progress was not sent: {error}")


def listenForPreflightMessages(vehicle, prearmHealthBit, flightOutcome):
    latestTelemetry = {
        "compassVariance": None,
        "prearmHealthy": None,
        "lastPrearmReason": None,
        "lastPrearmPollTime": 0.0,
    }

    @vehicle.on_message("EKF_STATUS_REPORT")
    def onEkfStatusReport(_vehicle, _name, message):
        latestTelemetry["compassVariance"] = float(message.compass_variance)

    @vehicle.on_message("SYS_STATUS")
    def onSysStatus(_vehicle, _name, message):
        healthMask = int(getattr(message, "onboard_control_sensors_health", 0))
        isHealthy = bool(healthMask & prearmHealthBit)
        latestTelemetry["prearmHealthy"] = isHealthy
        if isHealthy:
            latestTelemetry["lastPrearmReason"] = None

    @vehicle.on_message("STATUSTEXT")
    def onStatusText(_vehicle, _name, message):
        text = getattr(message, "text", "")
        if isinstance(text, bytes):
            text = text.decode("ascii", errors="ignore")
        text = str(text).strip("\x00").strip()
        if text:
            if "prearm" in text.lower():
                latestTelemetry["lastPrearmReason"] = text
            if FALSE_BATTERY_WARNING in text.lower():
                latestTelemetry["sawFalseBatteryWarning"] = True
            writeFlightLine(flightOutcome, f"Pixhawk: {text}")

    return latestTelemetry


def requestFlightStreams(vehicle, middleMan):
    intervalMicroseconds = int(MESSAGE_INTERVAL_SECONDS * MICROSECONDS_PER_SECOND)
    for messageName in FLIGHT_MESSAGE_NAMES:
        messageId = getattr(middleMan.mavlink, f"MAVLINK_MSG_ID_{messageName}", None)
        if messageId is None:
            continue
        commandMessage = vehicle.message_factory.command_long_encode(
            vehicle._master.target_system,
            vehicle._master.target_component,
            middleMan.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            messageId,
            intervalMicroseconds,
            0, 0, 0, 0, 0,
        )
        vehicle.send_mavlink(commandMessage)
    vehicle.flush()


def waitUntilPreflightPasses(vehicle, latestTelemetry, runPrearmCommandId, flightOutcome):
    startedAt = time.monotonic()
    writeFlightLine(flightOutcome, pixhawkBatteryVoltageText(vehicle))
    while True:
        raiseIfFlightStopped()
        releaseFalseBatteryFailsafe(vehicle, latestTelemetry, lambda text: writeFlightLine(flightOutcome, text), confirmPixhawkParameter)
        preferPrimaryCompassWhenTheyDisagree(vehicle, latestTelemetry, lambda text: writeFlightLine(flightOutcome, text), confirmPixhawkParameter)
        if time.monotonic() - startedAt > PREFLIGHT_WAIT_TIMEOUT_SECONDS:
            raise RuntimeError("Preflight did not pass in time. The copter was not armed.")
        blocker = preflightBlocker(vehicle, latestTelemetry, runPrearmCommandId)
        if blocker is None:
            writeFlightLine(flightOutcome, "Preflight checks passed.")
            return
        writeFlightLine(flightOutcome, f"Waiting. {blocker}")
        time.sleep(PRINT_INTERVAL_SECONDS)


def preflightBlocker(vehicle, latestTelemetry, runPrearmCommandId):
    if not pixhawkHeartbeatIsFresh(vehicle):
        return "Pixhawk heartbeat is missing."

    frameClass = vehicle.parameters["FRAME_CLASS"]
    frameType = vehicle.parameters["FRAME_TYPE"]
    if frameClass is None or frameType is None:
        return "FRAME_CLASS or FRAME_TYPE could not be read."
    if (
        abs(float(frameClass) - QUAD_FRAME_CLASS) >= PARAMETER_TOLERANCE
        or abs(float(frameType) - QUAD_X_FRAME_TYPE) >= PARAMETER_TOLERANCE
    ):
        return f"Quad X required. Found FRAME_CLASS={frameClass}, FRAME_TYPE={frameType}."

    skipCheckValue = vehicle.parameters["ARMING_SKIPCHK"]
    if skipCheckValue is not None and (int(round(float(skipCheckValue))) & SKIP_COMPASS_OR_GPS_MASK) != 0:
        raise RuntimeError("ARMING_SKIPCHK is skipping compass or GPS checks.")

    useCompass1 = int(round(float(vehicle.parameters["COMPASS_USE"] or 0)))
    useCompass2 = int(round(float(vehicle.parameters["COMPASS_USE2"] or 0)))
    if useCompass1 == 0 and useCompass2 == 0:
        raise RuntimeError("Both compasses are turned off.")

    compassVariance = latestTelemetry["compassVariance"]
    if compassVariance is not None and compassVariance > MAXIMUM_COMPASS_VARIANCE:
        return f"EKF compass variance is {compassVariance:.3f} (needs <= {MAXIMUM_COMPASS_VARIANCE:.2f})."

    trimX = vehicle.parameters["AHRS_TRIM_X"]
    trimY = vehicle.parameters["AHRS_TRIM_Y"]
    if trimX is not None and trimY is not None:
        trimRollDegrees = math.degrees(float(trimX))
        trimPitchDegrees = math.degrees(float(trimY))
        if abs(trimRollDegrees) > MAXIMUM_SAVED_TRIM_DEGREES or abs(trimPitchDegrees) > MAXIMUM_SAVED_TRIM_DEGREES:
            raise RuntimeError(
                f"Saved level trim is tilted (roll {trimRollDegrees:+.2f} deg, pitch {trimPitchDegrees:+.2f} deg)."
            )

    gpsReading = vehicle.gps_0
    if gpsReading is None or gpsReading.fix_type is None or int(gpsReading.fix_type) < MINIMUM_GPS_FIX_TYPE:
        return "GPS 3D fix required."
    satellites = gpsReading.satellites_visible
    if satellites is None or satellites == UNKNOWN_SATELLITE_COUNT or satellites < MINIMUM_SATELLITE_COUNT:
        return f"At least {MINIMUM_SATELLITE_COUNT} satellites required. Seeing {satellites}."
    if not vehicle.ekf_ok:
        return f"3D fix, {satellites} satellites. Horizontal position is not trusted yet."

    if not vehicle.is_armable or latestTelemetry["prearmHealthy"] is not True:
        nowPoll = time.monotonic()
        if nowPoll - latestTelemetry["lastPrearmPollTime"] >= PREARM_POLL_INTERVAL_SECONDS:
            prearmCheckMessage = vehicle.message_factory.command_long_encode(
                vehicle._master.target_system,
                vehicle._master.target_component,
                runPrearmCommandId,
                0,
                0, 0, 0, 0, 0, 0, 0,
            )
            vehicle.send_mavlink(prearmCheckMessage)
            vehicle.flush()
            latestTelemetry["lastPrearmPollTime"] = nowPoll
        reasonText = latestTelemetry["lastPrearmReason"] or "waiting for Pixhawk PreArm status..."
        return f"Pixhawk pre-arm check blocked ({reasonText})."

    if vehicle.attitude is None or vehicle.attitude.roll is None or vehicle.attitude.pitch is None:
        return "Attitude is not available yet."
    rollDegrees = math.degrees(vehicle.attitude.roll)
    pitchDegrees = math.degrees(vehicle.attitude.pitch)
    if abs(rollDegrees) > MAXIMUM_GROUND_LEAN_DEGREES or abs(pitchDegrees) > MAXIMUM_GROUND_LEAN_DEGREES:
        return f"Copter is not sitting level. Roll {rollDegrees:+.1f} deg, pitch {pitchDegrees:+.1f} deg."
    return None


def armLandFailsafe(vehicle, flightOutcome):
    savedEnable = float(vehicle.parameters[GROUND_STATION_FAILSAFE_PARAMETER_NAME])
    savedTimeout = float(vehicle.parameters[GROUND_STATION_FAILSAFE_TIMEOUT_PARAMETER_NAME])
    savedOptions = float(vehicle.parameters[FAILSAFE_OPTIONS_PARAMETER_NAME])
    timeoutSeconds = savedTimeout if (0 < savedTimeout <= PREFERRED_FAILSAFE_TIMEOUT_SECONDS) else PREFERRED_FAILSAFE_TIMEOUT_SECONDS
    flightOptions = int(round(savedOptions)) & ~CONTINUE_PILOT_MODE_ON_FAILSAFE_BIT
    confirmPixhawkParameter(vehicle, GROUND_STATION_FAILSAFE_PARAMETER_NAME, GROUND_STATION_FAILSAFE_LAND_VALUE)
    confirmPixhawkParameter(vehicle, GROUND_STATION_FAILSAFE_TIMEOUT_PARAMETER_NAME, timeoutSeconds)
    confirmPixhawkParameter(vehicle, FAILSAFE_OPTIONS_PARAMETER_NAME, flightOptions)
    vehicle.groundStationFailsafeTimeoutSeconds = timeoutSeconds
    writeFlightLine(flightOutcome, "Onboard failsafe is set to Land if the Pi connection is lost.")
    return (savedEnable, savedTimeout, savedOptions)


def recheckPreflightAfterFailsafe(vehicle, latestTelemetry):
    if not pixhawkHeartbeatIsFresh(vehicle):
        raise RuntimeError("Pixhawk heartbeat was lost before arming.")
    gpsReading = vehicle.gps_0
    if (
        gpsReading is None
        or gpsReading.fix_type is None
        or int(gpsReading.fix_type) < MINIMUM_GPS_FIX_TYPE
        or gpsReading.satellites_visible is None
        or gpsReading.satellites_visible == UNKNOWN_SATELLITE_COUNT
        or gpsReading.satellites_visible < MINIMUM_SATELLITE_COUNT
    ):
        raise RuntimeError("GPS was not ready before arming.")
    if not vehicle.ekf_ok or not vehicle.is_armable or latestTelemetry["prearmHealthy"] is not True:
        reasonText = latestTelemetry["lastPrearmReason"] or "EKF or PreArm check not ready"
        raise RuntimeError(f"Preflight became unready before arming ({reasonText}).")
    rollDegrees = math.degrees(vehicle.attitude.roll)
    pitchDegrees = math.degrees(vehicle.attitude.pitch)
    if abs(rollDegrees) > MAXIMUM_GROUND_LEAN_DEGREES or abs(pitchDegrees) > MAXIMUM_GROUND_LEAN_DEGREES:
        raise RuntimeError(f"Copter is no longer level (roll {rollDegrees:+.1f} deg, pitch {pitchDegrees:+.1f} deg).")


def armAndWait(vehicle, flightOutcome):
    writeFlightLine(flightOutcome, "Arming motors.")
    vehicle.armed = True
    armStartTime = time.monotonic()
    while not vehicle.armed:
        raiseIfFlightStopped()
        if time.monotonic() - armStartTime > ARM_TIMEOUT_SECONDS:
            raise RuntimeError("Arming timed out.")
        if not pixhawkHeartbeatIsFresh(vehicle):
            raise RuntimeError("Pixhawk heartbeat was lost while arming.")
        time.sleep(STATUS_CHECK_INTERVAL_SECONDS)
    writeFlightLine(flightOutcome, "Motors armed. Propellers spinning.")


def climbToFiveFeet(vehicle, flightOutcome):
    writeFlightLine(flightOutcome, f"Taking off to {HOVER_ALTITUDE_FEET:.0f} feet.")
    vehicle.simple_takeoff(HOVER_ALTITUDE_METERS)
    minimumAltitudeMeters = HOVER_ALTITUDE_METERS * TAKEOFF_REACHED_FRACTION
    takeoffStartTime = time.monotonic()
    lastPrintTime = 0.0
    while True:
        raiseIfFlightStopped()
        if vehicle.mode.name != "GUIDED":
            raiseIfFlightStopped()
            raise RuntimeError("GUIDED mode lost during takeoff.")
        altitudeMeters = checkFlightHealthAndCeiling(vehicle, flightOutcome)
        nowTime = time.monotonic()
        if nowTime - lastPrintTime >= PRINT_INTERVAL_SECONDS:
            heightText = "waiting" if altitudeMeters is None else f"{altitudeMeters * FEET_PER_METER:.2f} ft"
            writeFlightLine(flightOutcome, f"Climbing to 5 ft | {heightText}")
            lastPrintTime = nowTime
        if altitudeMeters is not None and altitudeMeters >= minimumAltitudeMeters:
            writeFlightLine(flightOutcome, f"Reached 5 feet ({altitudeMeters * FEET_PER_METER:.2f} ft).")
            return
        if nowTime - takeoffStartTime > TAKEOFF_TIMEOUT_SECONDS:
            raise RuntimeError("Takeoff timed out.")
        time.sleep(STATUS_CHECK_INTERVAL_SECONDS)


def settleAtFiveFeet(vehicle, flightOutcome):
    writeFlightLine(flightOutcome, f"Settling for {TAKEOFF_SETTLE_SECONDS:.0f} seconds before the hover.")
    settleStart = time.monotonic()
    while time.monotonic() - settleStart < TAKEOFF_SETTLE_SECONDS:
        raiseIfFlightStopped()
        if vehicle.mode.name != "GUIDED":
            raiseIfFlightStopped()
            raise RuntimeError("GUIDED mode lost during the takeoff settle.")
        checkFlightHealthAndCeiling(vehicle, flightOutcome)
        time.sleep(STATUS_CHECK_INTERVAL_SECONDS)


def holdHoverAtFiveFeet(vehicle, flightOutcome, hoverName, durationSeconds=None):
    if durationSeconds is None:
        durationSeconds = HOVER_DURATION_SECONDS
    startLatitude, startLongitude = latitudeAndLongitude(vehicle, f"GPS position was not available for the {hoverName} hover.")
    writeFlightLine(flightOutcome, f"Hover {hoverName} started for {durationSeconds:.0f} seconds.")
    hoverStartTime = time.monotonic()
    lastPrintTime = 0.0
    largestDriftFeet = 0.0
    while True:
        raiseIfFlightStopped()
        elapsedSeconds = time.monotonic() - hoverStartTime
        if elapsedSeconds >= durationSeconds:
            break
        if vehicle.mode.name != "LOITER":
            raiseIfFlightStopped()
            raise RuntimeError(f"LOITER mode lost during the {hoverName} hover.")
        altitudeMeters = checkFlightHealthAndCeiling(vehicle, flightOutcome)
        northMeters, eastMeters = northAndEastFromStart(vehicle, startLatitude, startLongitude)
        driftFeet = math.hypot(northMeters, eastMeters) * FEET_PER_METER
        largestDriftFeet = max(largestDriftFeet, driftFeet)
        if math.hypot(northMeters, eastMeters) > MAXIMUM_HOVER_DRIFT_METERS:
            raise RuntimeError(f"The {hoverName} hover drifted {driftFeet:.1f} ft.")
        nowTime = time.monotonic()
        if nowTime - lastPrintTime >= PRINT_INTERVAL_SECONDS:
            remainingSeconds = max(0.0, durationSeconds - elapsedSeconds)
            heightText = "waiting" if altitudeMeters is None else f"{altitudeMeters * FEET_PER_METER:.2f} ft"
            writeFlightLine(flightOutcome, f"Hover {hoverName} | {heightText} | drift {driftFeet:.1f} ft | remaining {remainingSeconds:.0f} s")
            lastPrintTime = nowTime
        time.sleep(STATUS_CHECK_INTERVAL_SECONDS)
    flightOutcome["steps"][f"hover {hoverName}"] = f"SUCCESS (max drift {largestDriftFeet:.1f} ft)"
    writeFlightLine(flightOutcome, f"Hover {hoverName} completed.")


def faceGeographicNorth(vehicle, middleMan, velocityTypeMask, flightOutcome, altitudeFeet=HOVER_ALTITUDE_FEET):
    targetAltitudeMeters = altitudeFeet / FEET_PER_METER
    currentHeadingDegrees = math.degrees(float(vehicle.attitude.yaw or 0.0))
    headingErrorDegrees = angleFromNorthDegrees(currentHeadingDegrees)
    if abs(headingErrorDegrees) <= NORTH_HEADING_TOLERANCE_DEGREES:
        writeFlightLine(flightOutcome, f"Nose is already facing north ({currentHeadingDegrees:+.1f} deg).")
        return

    writeFlightLine(flightOutcome, f"Turning nose to north from {currentHeadingDegrees:+.1f} deg.")
    yawStartTime = time.monotonic()
    lastPrintTime = 0.0
    while True:
        raiseIfFlightStopped()
        if vehicle.mode.name != "GUIDED":
            raiseIfFlightStopped()
            raise RuntimeError("GUIDED mode lost while turning north.")
        altitudeMeters = checkFlightHealthAndCeiling(vehicle, flightOutcome)
        sendLocalVelocity(vehicle, 0.0, 0.0, downSpeedToHoldFiveFeet(altitudeMeters, targetAltitudeMeters), velocityTypeMask)
        commandFaceGeographicNorth(vehicle, middleMan)
        currentHeadingDegrees = math.degrees(float(vehicle.attitude.yaw or 0.0))
        headingErrorDegrees = angleFromNorthDegrees(currentHeadingDegrees)
        if abs(headingErrorDegrees) <= NORTH_HEADING_TOLERANCE_DEGREES:
            writeFlightLine(flightOutcome, f"Nose is facing north ({currentHeadingDegrees:+.1f} deg).")
            break
        nowTime = time.monotonic()
        if nowTime - lastPrintTime >= PRINT_INTERVAL_SECONDS:
            writeFlightLine(flightOutcome, f"Turning to north | heading {currentHeadingDegrees:+.1f} deg")
            lastPrintTime = nowTime
        if nowTime - yawStartTime > YAW_ALIGN_TIMEOUT_SECONDS:
            writeFlightLine(flightOutcome, "North turn time limit reached. The moves still use Earth directions.")
            break
        time.sleep(STATUS_CHECK_INTERVAL_SECONDS)

    settleStart = time.monotonic()
    while time.monotonic() - settleStart < YAW_SETTLE_SECONDS:
        raiseIfFlightStopped()
        checkFlightHealthAndCeiling(vehicle, flightOutcome)
        sendLocalVelocity(vehicle, 0.0, 0.0, 0.0, velocityTypeMask)
        time.sleep(STATUS_CHECK_INTERVAL_SECONDS)


def flyTenFeet(vehicle, flightOutcome, directionName, northSpeed, eastSpeed, velocityTypeMask, distanceFeet=None, altitudeFeet=None):
    if distanceFeet is None:
        distanceFeet = LEG_DISTANCE_FEET
    if altitudeFeet is None:
        altitudeFeet = HOVER_ALTITUDE_FEET
    distanceMeters = distanceFeet / FEET_PER_METER
    targetAltitudeMeters = altitudeFeet / FEET_PER_METER
    legTimeoutSeconds = max(LEG_TIMEOUT_SECONDS, (distanceMeters / abs(northSpeed or eastSpeed or LEG_SPEED_METERS_PER_SECOND)) * 2.0 + 10.0)
    startLatitude, startLongitude = latitudeAndLongitude(vehicle, f"GPS position was not available for the {directionName} leg.")
    writeFlightLine(flightOutcome, f"Flying {directionName} {distanceFeet:.0f} ft at {altitudeFeet:.0f} ft.")
    legStartTime = time.monotonic()
    lastPrintTime = 0.0
    try:
        while True:
            raiseIfFlightStopped()
            if vehicle.mode.name != "GUIDED":
                raiseIfFlightStopped()
                raise RuntimeError(f"GUIDED mode lost during the {directionName} leg.")
            altitudeMeters = checkFlightHealthAndCeiling(vehicle, flightOutcome)
            northMeters, eastMeters = northAndEastFromStart(vehicle, startLatitude, startLongitude)
            progressMeters, crossTrackMeters = progressAndCrossTrackMeters(directionName, northMeters, eastMeters)
            progressFeet = progressMeters * FEET_PER_METER
            if progressMeters >= distanceMeters:
                writeFlightLine(flightOutcome, f"{distanceFeet:.0f} feet {directionName} reached | {progressFeet:.1f} ft")
                break
            if abs(crossTrackMeters) > MAXIMUM_OFF_COURSE_METERS or progressMeters < -MAXIMUM_OFF_COURSE_METERS:
                raise RuntimeError(f"Left the {directionName} line at {progressFeet:.1f} ft.")
            if time.monotonic() - legStartTime > legTimeoutSeconds:
                raise RuntimeError(f"Did not reach {distanceFeet:.0f} feet {directionName} in time ({progressFeet:.1f} ft).")
            sendLocalVelocity(
                vehicle,
                northSpeed,
                eastSpeed,
                downSpeedToHoldFiveFeet(altitudeMeters, targetAltitudeMeters),
                velocityTypeMask,
            )
            nowTime = time.monotonic()
            if nowTime - lastPrintTime >= PRINT_INTERVAL_SECONDS:
                heightText = "waiting" if altitudeMeters is None else f"{altitudeMeters * FEET_PER_METER:.2f} ft"
                writeFlightLine(flightOutcome, f"Flying {directionName} | {progressFeet:.1f}/{distanceFeet:.0f} ft | {heightText}")
                lastPrintTime = nowTime
            time.sleep(STATUS_CHECK_INTERVAL_SECONDS)

        brakeStartTime = time.monotonic()
        while time.monotonic() - brakeStartTime < SETTLE_AFTER_MOVE_SECONDS:
            raiseIfFlightStopped()
            if vehicle.mode.name != "GUIDED":
                raiseIfFlightStopped()
                raise RuntimeError(f"GUIDED mode lost while stopping the {directionName} leg.")
            altitudeMeters = checkFlightHealthAndCeiling(vehicle, flightOutcome)
            sendLocalVelocity(vehicle, 0.0, 0.0, downSpeedToHoldFiveFeet(altitudeMeters, targetAltitudeMeters), velocityTypeMask)
            time.sleep(STATUS_CHECK_INTERVAL_SECONDS)
        flightOutcome["steps"][f"fly {directionName}"] = "SUCCESS"
        writeFlightLine(flightOutcome, f"{directionName} leg stopped.")
    finally:
        if pixhawkHeartbeatIsFresh(vehicle) and not getattr(vehicle, "commandCableClosed", False) and not flightStopRequested and not flightReturnHomeRequested and not flightClearRequested:
            try:
                sendLocalVelocity(vehicle, 0.0, 0.0, 0.0, velocityTypeMask)
            except Exception as stopError:
                writeFlightLine(flightOutcome, f"Could not send the stop speed: {stopError}")


def landUntilPropellersStop(vehicle, flightOutcome, vehicleMode):
    writeFlightLine(flightOutcome, "Landing.")
    commandFlightMode(vehicle, "LAND", vehicleMode, flightOutcome)
    landStartTime = time.monotonic()
    while vehicle.armed:
        if not pixhawkHeartbeatIsFresh(vehicle):
            writeFlightLine(flightOutcome, "Heartbeat lost during landing. Closing the cable so the onboard failsafe finishes the landing.")
            vehicle.commandCableClosed = True
            vehicle.close()
            flightOutcome["steps"]["land"] = "ONBOARD FAILSAFE"
            return
        if time.monotonic() - landStartTime > LAND_TIMEOUT_SECONDS:
            raise RuntimeError("Landing not confirmed. The motors are still armed.")
        location = vehicle.location.global_relative_frame
        heightText = "waiting" if location is None or location.alt is None else f"{float(location.alt) * FEET_PER_METER:.2f} ft"
        writeFlightLine(flightOutcome, f"Waiting for the propellers to stop. {heightText}")
        time.sleep(PRINT_INTERVAL_SECONDS)
    writeFlightLine(flightOutcome, "Landed. Motors disarmed and propellers stopped.")


def finishFlightConnection(vehicle, droneKitTools, flightOutcome, copterWasArmed, landingWasStarted, savedFailsafe, originalTakeoffSlewSeconds):
    try:
        if vehicle is not None and not getattr(vehicle, "commandCableClosed", False):
            if flightClearRequested and not getattr(vehicle, "armed", False):
                copterWasArmed = False
            if (copterWasArmed or vehicle.armed) and not landingWasStarted:
                writeFlightLine(flightOutcome, "Requesting LAND.")
                vehicleMode = droneKitTools[0] if droneKitTools is not None else None
                if not pixhawkHeartbeatIsFresh(vehicle) or vehicleMode is None:
                    vehicle.commandCableClosed = True
                    vehicle.close()
                    flightOutcome["steps"]["land"] = "ONBOARD FAILSAFE"
                else:
                    commandFlightMode(vehicle, "LAND", vehicleMode, flightOutcome)
                    emergencyLandStart = time.monotonic()
                    while vehicle.armed:
                        if not pixhawkHeartbeatIsFresh(vehicle):
                            vehicle.commandCableClosed = True
                            vehicle.close()
                            flightOutcome["steps"]["land"] = "ONBOARD FAILSAFE"
                            break
                        if time.monotonic() - emergencyLandStart > LAND_TIMEOUT_SECONDS:
                            raise RuntimeError("Timed out waiting for the motors to disarm.")
                        time.sleep(PRINT_INTERVAL_SECONDS)
                    if not vehicle.armed:
                        writeFlightLine(flightOutcome, "Landed. Motors disarmed and propellers stopped.")
                        flightOutcome["steps"]["land"] = "SUCCESS"

            if not getattr(vehicle, "commandCableClosed", False) and pixhawkHeartbeatIsFresh(vehicle) and not vehicle.armed:
                if savedFailsafe is not None:
                    confirmPixhawkParameter(vehicle, GROUND_STATION_FAILSAFE_PARAMETER_NAME, savedFailsafe[0])
                    confirmPixhawkParameter(vehicle, GROUND_STATION_FAILSAFE_TIMEOUT_PARAMETER_NAME, savedFailsafe[1])
                    confirmPixhawkParameter(vehicle, FAILSAFE_OPTIONS_PARAMETER_NAME, savedFailsafe[2])
                    writeFlightLine(flightOutcome, "Ground-station failsafe settings restored.")
                if originalTakeoffSlewSeconds is not None:
                    confirmPixhawkParameter(vehicle, TAKEOFF_SLEW_PARAMETER_NAME, originalTakeoffSlewSeconds)
                    writeFlightLine(flightOutcome, f"{TAKEOFF_SLEW_PARAMETER_NAME} restored.")
                savedReturnAltitude = getattr(vehicle, "savedReturnAltitudeCentimeters", None)
                if savedReturnAltitude is not None:
                    confirmPixhawkParameter(vehicle, "RTL_ALT", savedReturnAltitude)
                    writeFlightLine(flightOutcome, "Return altitude restored.")
    except Exception as closeError:
        writeFlightLine(flightOutcome, f"Could not close the flight cleanly: {closeError}")
    try:
        if vehicle is not None and not getattr(vehicle, "commandCableClosed", False):
            vehicle.commandCableClosed = True
            vehicle.close()
            writeFlightLine(flightOutcome, "Command cable closed.")
    except Exception as closeError:
        writeFlightLine(flightOutcome, f"Could not close the command cable: {closeError}")
    global activeFlightVehicle, activeFlightOutcome
    flightOutcome["finishedAt"] = time.strftime("%Y-%m-%d %H:%M:%S %Z")
    writeFlightLine(flightOutcome, f"Outcome: {flightOutcome['outcome']}")
    activeFlightVehicle = None
    activeFlightOutcome = None


def pixhawkHeartbeatIsFresh(vehicle):
    heartbeatAgeSeconds = vehicle.last_heartbeat
    if heartbeatAgeSeconds is None:
        return False
    return heartbeatAgeSeconds <= HEARTBEAT_STALE_AFTER_SECONDS


def commandFlightMode(vehicle, flightModeName, vehicleMode, flightOutcome):
    if flightClearRequested:
        raise FlightClearRequested()
    if flightReturnHomeRequested and flightModeName != "RTL":
        raise FlightReturnHomeRequested()
    if flightStopRequested and flightModeName != "LAND":
        raise FlightStopRequested()
    writeFlightLine(flightOutcome, f"Requesting {flightModeName}.")
    vehicle.mode = vehicleMode(flightModeName)
    startTime = time.monotonic()
    while time.monotonic() - startTime < MODE_CHANGE_TIMEOUT_SECONDS:
        if flightClearRequested:
            raise FlightClearRequested()
        if flightReturnHomeRequested and flightModeName != "RTL":
            raise FlightReturnHomeRequested()
        if flightStopRequested and flightModeName != "LAND":
            raise FlightStopRequested()
        if vehicle.mode.name == flightModeName:
            return
        if not pixhawkHeartbeatIsFresh(vehicle):
            raise RuntimeError("Pixhawk heartbeat was lost.")
        time.sleep(STATUS_CHECK_INTERVAL_SECONDS)
    raise RuntimeError(f"Could not enter {flightModeName}.")


def confirmPixhawkParameter(vehicle, parameterName, wantedValue):
    currentValue = vehicle.parameters[parameterName]
    if currentValue is not None and abs(float(currentValue) - float(wantedValue)) < PARAMETER_TOLERANCE:
        return
    wasWritten = vehicle.parameters.set(parameterName, wantedValue)
    confirmedValue = vehicle.parameters[parameterName]
    if (
        not wasWritten
        or confirmedValue is None
        or abs(float(confirmedValue) - float(wantedValue)) >= PARAMETER_TOLERANCE
    ):
        raise RuntimeError(f"{parameterName} did not stay at {float(wantedValue):.1f}.")


def pixhawkBatteryVoltageText(vehicle):
    battery = getattr(vehicle, "battery", None)
    voltage = getattr(battery, "voltage", None) if battery is not None else None
    if voltage is None or not math.isfinite(float(voltage)) or float(voltage) < 0:
        return "Pixhawk battery voltage was not reported."
    return f"Pixhawk battery voltage reads {float(voltage):.2f} volts."


def releaseFalseBatteryFailsafe(vehicle, latestTelemetry, writeLine, confirmParameter):
    """A full pack can still be reported low. That one failsafe must not refuse arm."""
    if latestTelemetry.get("falseBatteryFailsafeIgnored"):
        return
    reasonText = (latestTelemetry.get("lastPrearmReason") or "").lower()
    if FALSE_BATTERY_WARNING not in reasonText and not latestTelemetry.get("sawFalseBatteryWarning"):
        return
    writeLine(pixhawkBatteryVoltageText(vehicle))
    writeLine("Battery 1 low voltage failsafe is a false reading. Disabling that failsafe for this flight.")
    confirmParameter(vehicle, BATTERY_LOW_FAILSAFE_ACTION_PARAMETER, BATTERY_FAILSAFE_ACTION_NONE)
    confirmParameter(vehicle, BATTERY_CRITICAL_FAILSAFE_ACTION_PARAMETER, BATTERY_FAILSAFE_ACTION_NONE)
    latestTelemetry["falseBatteryFailsafeIgnored"] = True
    latestTelemetry["lastPrearmReason"] = None
    latestTelemetry["sawFalseBatteryWarning"] = False


def boardAndGpsPuckCompasses(vehicle):
    internalCompass = None
    gpsPuckCompass = None
    for deviceIdParameter, externalParameter, useParameter, messageName in COMPASS_SLOT_PARAMETERS:
        deviceId = int(round(float(vehicle.parameters[deviceIdParameter] or 0)))
        externalFlag = int(round(float(vehicle.parameters[externalParameter] or 0)))
        compassSlot = {
            "deviceId": deviceId,
            "useParameter": useParameter,
            "messageName": messageName,
        }
        if deviceId == 0:
            continue
        if externalFlag in (1, 2):
            gpsPuckCompass = compassSlot
        else:
            internalCompass = compassSlot
    if internalCompass is None or gpsPuckCompass is None:
        raise RuntimeError("Could not detect both the board compass and the GPS puck compass.")
    return internalCompass, gpsPuckCompass


def boardAndGpsPuckCompassIds(vehicle):
    internalCompass, gpsPuckCompass = boardAndGpsPuckCompasses(vehicle)
    return internalCompass["deviceId"], gpsPuckCompass["deviceId"]


def ensureBoardCompassIsPrimary(vehicle, rebootAndReconnect, writeLine, confirmParameter):
    """Board compass is priority 1. The GPS puck is priority 2. Reboot only when that order changes."""
    internalCompassId, gpsPuckCompassId = boardAndGpsPuckCompassIds(vehicle)
    writeLine(f"Board compass id {internalCompassId}. GPS puck compass id {gpsPuckCompassId}.")
    priority1 = int(round(float(vehicle.parameters["COMPASS_PRIO1_ID"] or 0)))
    priority2 = int(round(float(vehicle.parameters["COMPASS_PRIO2_ID"] or 0)))
    if priority1 == internalCompassId and priority2 == gpsPuckCompassId:
        writeLine("Compass order is already the board compass first and the GPS puck second.")
        return vehicle
    confirmParameter(vehicle, "COMPASS_PRIO1_ID", internalCompassId)
    confirmParameter(vehicle, "COMPASS_PRIO2_ID", gpsPuckCompassId)
    confirmParameter(vehicle, "COMPASS_USE", 1)
    confirmParameter(vehicle, "COMPASS_USE2", 1)
    writeLine("Compass priority changed. Rebooting so the board compass is primary and the GPS puck is secondary.")
    return rebootAndReconnect(vehicle)


def rebootFlightVehicle(vehicle, connect, flightOutcome):
    from pymavlink import mavutil as middleMan
    vehicle._master.mav.command_long_send(
        vehicle._master.target_system,
        vehicle._master.target_component,
        middleMan.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0,
        1, 0, 0, 0, 0, 0, 0,
    )
    try:
        vehicle.close()
    except Exception:
        pass
    time.sleep(COMPASS_PRIORITY_REBOOT_WAIT_SECONDS)
    reconnected = connectUntilCleared(connect, flightOutcome)
    if reconnected._master.target_component == 0:
        reconnected._master.target_component = 1
    writeFlightLine(flightOutcome, "Pixhawk rebooted with the board compass as primary.")
    return reconnected


def compassFieldIsUsable(magneticField):
    if magneticField is None:
        return False
    return math.hypot(magneticField[0], magneticField[1], magneticField[2]) >= COMPASS_FIELD_MINIMUM


def headingDifferenceDegrees(firstField, secondField):
    """Smallest difference between the two compass headings, in degrees."""
    firstHeading = math.atan2(firstField[1], firstField[0])
    secondHeading = math.atan2(secondField[1], secondField[0])
    wrappedDifference = math.atan2(
        math.sin(secondHeading - firstHeading),
        math.cos(secondHeading - firstHeading),
    )
    return abs(math.degrees(wrappedDifference))


def sharedCompassHeadingDegrees(firstField, secondField):
    """Circular mean of the two compass headings. This is the shared direction when they agree."""
    firstHeading = math.atan2(firstField[1], firstField[0])
    secondHeading = math.atan2(secondField[1], secondField[0])
    meanHeading = math.atan2(
        math.sin(firstHeading) + math.sin(secondHeading),
        math.cos(firstHeading) + math.cos(secondHeading),
    )
    return math.degrees(meanHeading) % 360.0


def requestCompassFieldMessages(vehicle):
    from pymavlink import mavutil as middleMan
    intervalMicroseconds = int(MESSAGE_INTERVAL_SECONDS * MICROSECONDS_PER_SECOND)
    for messageName in ("SCALED_IMU", "SCALED_IMU2"):
        messageId = getattr(middleMan.mavlink, f"MAVLINK_MSG_ID_{messageName}", None)
        if messageId is None:
            continue
        commandMessage = vehicle.message_factory.command_long_encode(
            vehicle._master.target_system,
            vehicle._master.target_component,
            middleMan.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            messageId,
            intervalMicroseconds,
            0, 0, 0, 0, 0,
        )
        vehicle.send_mavlink(commandMessage)
    vehicle.flush()


def sampleCompassFields(vehicle):
    capturedFields = {}

    def rememberField(_vehicle, messageName, message):
        capturedFields[messageName] = (float(message.xmag), float(message.ymag), float(message.zmag))

    vehicle.add_message_listener("SCALED_IMU", rememberField)
    vehicle.add_message_listener("SCALED_IMU2", rememberField)
    requestCompassFieldMessages(vehicle)
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and ("SCALED_IMU" not in capturedFields or "SCALED_IMU2" not in capturedFields):
        time.sleep(0.05)
    vehicle.remove_message_listener("SCALED_IMU", rememberField)
    vehicle.remove_message_listener("SCALED_IMU2", rememberField)
    return capturedFields


def setCompassEnabled(vehicle, compassSlot, enabled, confirmParameter):
    confirmParameter(vehicle, compassSlot["useParameter"], 1 if enabled else 0)


def useOnlyThisCompass(vehicle, compassToUse, compassToIgnore, confirmParameter, choiceName):
    setCompassEnabled(vehicle, compassToUse, True, confirmParameter)
    setCompassEnabled(vehicle, compassToIgnore, False, confirmParameter)
    vehicle.compassChoice = choiceName


def preferPrimaryCompassWhenTheyDisagree(vehicle, latestTelemetry, writeLine, confirmParameter):
    """The Pixhawk still says the compasses disagree. Keep the board compass for this flight."""
    if getattr(vehicle, "compassChoice", None) == "board":
        return
    reasonText = (latestTelemetry.get("lastPrearmReason") or "").lower()
    if "compasses inconsistent" not in reasonText:
        return
    internalCompass, gpsPuckCompass = boardAndGpsPuckCompasses(vehicle)
    useOnlyThisCompass(vehicle, internalCompass, gpsPuckCompass, confirmParameter, "board")
    latestTelemetry["lastPrearmReason"] = None
    writeLine("Compasses disagree. This flight will use the board compass only.")


def chooseCompassesForThisFlight(vehicle, writeLine, confirmParameter):
    """Agree within 11 degrees: use both. Otherwise use the board compass, or the GPS puck if the board compass is dead."""
    internalCompass, gpsPuckCompass = boardAndGpsPuckCompasses(vehicle)
    compassFields = sampleCompassFields(vehicle)
    boardField = compassFields.get(internalCompass["messageName"])
    puckField = compassFields.get(gpsPuckCompass["messageName"])
    boardIsAvailable = compassFieldIsUsable(boardField)
    puckIsAvailable = compassFieldIsUsable(puckField)
    if not boardIsAvailable and puckIsAvailable:
        useOnlyThisCompass(vehicle, gpsPuckCompass, internalCompass, confirmParameter, "puck")
        writeLine("The board compass is not available. This flight will use the GPS puck compass only.")
        return
    if boardIsAvailable and not puckIsAvailable:
        useOnlyThisCompass(vehicle, internalCompass, gpsPuckCompass, confirmParameter, "board")
        writeLine("The GPS puck compass is not available. This flight will use the board compass only.")
        return
    if not boardIsAvailable and not puckIsAvailable:
        writeLine("Neither compass sent a usable magnetic field yet.")
        return
    differenceDegrees = headingDifferenceDegrees(boardField, puckField)
    if differenceDegrees is None or differenceDegrees > COMPASS_AGREEMENT_LIMIT_DEGREES:
        useOnlyThisCompass(vehicle, internalCompass, gpsPuckCompass, confirmParameter, "board")
        differenceText = "an unknown amount" if differenceDegrees is None else f"{differenceDegrees:.1f} degrees"
        writeLine(f"Compasses differ by {differenceText}. This flight will use the board compass only.")
        return
    setCompassEnabled(vehicle, internalCompass, True, confirmParameter)
    setCompassEnabled(vehicle, gpsPuckCompass, True, confirmParameter)
    vehicle.compassChoice = "both"
    sharedHeading = sharedCompassHeadingDegrees(boardField, puckField)
    writeLine(
        f"Compasses agree within {differenceDegrees:.1f} degrees. Both stay on. "
        f"The shared heading is {sharedHeading:.1f} degrees."
    )


def checkFlightHealthAndCeiling(vehicle, flightOutcome):
    if not pixhawkHeartbeatIsFresh(vehicle):
        raise RuntimeError("Pixhawk heartbeat was lost.")
    if not vehicle.armed:
        raise RuntimeError("Copter disarmed unexpectedly in flight.")
    if not vehicle.ekf_ok:
        raise RuntimeError("Position estimate lost in flight.")
    location = vehicle.location.global_relative_frame
    if location is None or location.alt is None or not math.isfinite(float(location.alt)):
        return None
    altitudeMeters = float(location.alt)
    previousMax = getattr(vehicle, "maximumAltitudeMeters", None)
    if previousMax is None or altitudeMeters > previousMax:
        vehicle.maximumAltitudeMeters = altitudeMeters
        flightOutcome["maximumAltitudeReachedText"] = f"{altitudeMeters * FEET_PER_METER:.2f} ft"
    if altitudeMeters >= getattr(vehicle, "altitudeCeilingMeters", MAXIMUM_ALTITUDE_CEILING_METERS):
        ceilingFeet = getattr(vehicle, "altitudeCeilingMeters", MAXIMUM_ALTITUDE_CEILING_METERS) * FEET_PER_METER
        raise RuntimeError(f"Safety ceiling ({ceilingFeet:.0f} ft) reached.")
    return altitudeMeters


def latitudeAndLongitude(vehicle, missingMessage):
    location = vehicle.location.global_relative_frame
    if (
        location is None
        or location.lat is None
        or location.lon is None
        or (
            abs(float(location.lat)) < MINIMUM_VALID_COORDINATE_DEGREES
            and abs(float(location.lon)) < MINIMUM_VALID_COORDINATE_DEGREES
        )
    ):
        raise RuntimeError(missingMessage)
    return float(location.lat), float(location.lon)


def northAndEastFromStart(vehicle, startLatitude, startLongitude):
    location = vehicle.location.global_relative_frame
    if location is None or location.lat is None or location.lon is None:
        raise RuntimeError("GPS position was lost.")
    northMeters = (float(location.lat) - startLatitude) * METERS_PER_DEGREE_OF_LATITUDE
    eastMeters = (
        (float(location.lon) - startLongitude)
        * METERS_PER_DEGREE_OF_LATITUDE
        * math.cos(math.radians(startLatitude))
    )
    return northMeters, eastMeters


def progressAndCrossTrackMeters(directionName, northMeters, eastMeters):
    if directionName == "east":
        return eastMeters, northMeters
    if directionName == "west":
        return -eastMeters, northMeters
    if directionName == "north":
        return northMeters, eastMeters
    if directionName == "south":
        return -northMeters, eastMeters
    raise RuntimeError(f"Unknown direction {directionName}.")


def downSpeedToHoldFiveFeet(altitudeMeters, targetAltitudeMeters=None):
    if altitudeMeters is None:
        return 0.0
    if targetAltitudeMeters is None:
        targetAltitudeMeters = HOVER_ALTITUDE_METERS
    altitudeErrorMeters = targetAltitudeMeters - altitudeMeters
    climbRate = max(
        -MAXIMUM_VERTICAL_CORRECTION_METERS_PER_SECOND,
        min(MAXIMUM_VERTICAL_CORRECTION_METERS_PER_SECOND, altitudeErrorMeters * ALTITUDE_HOLD_GAIN_DURING_MOVE),
    )
    return -climbRate


def angleFromNorthDegrees(headingDegrees):
    return (float(headingDegrees) - NORTH_HEADING_DEGREES + 180.0) % 360.0 - 180.0


def commandFaceGeographicNorth(vehicle, middleMan):
    currentHeadingDegrees = math.degrees(float(vehicle.attitude.yaw or 0.0))
    headingErrorDegrees = angleFromNorthDegrees(currentHeadingDegrees)
    turnDirection = -1 if headingErrorDegrees > 0 else 1
    yawMessage = vehicle.message_factory.command_long_encode(
        vehicle._master.target_system,
        vehicle._master.target_component,
        middleMan.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        NORTH_HEADING_DEGREES,
        YAW_TURN_RATE_DEGREES_PER_SECOND,
        turnDirection,
        0,
        0, 0, 0,
    )
    vehicle.send_mavlink(yawMessage)
    vehicle.flush()


def sendLocalVelocity(vehicle, northMetersPerSecond, eastMetersPerSecond, downMetersPerSecond, velocityTypeMask):
    raiseIfFlightStopped()
    message = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        vehicle._master.target_system,
        vehicle._master.target_component,
        MAV_FRAME_LOCAL_NED,
        velocityTypeMask,
        0, 0, 0,
        northMetersPerSecond,
        eastMetersPerSecond,
        downMetersPerSecond,
        0, 0, 0,
        0, 0,
    )
    vehicle.send_mavlink(message)
    vehicle.flush()


TURN_MISSION_TAKEOFF_FEET = 6.0
TURN_MISSION_HOVER_FEET = 7.0
TURN_MISSION_CEILING_FEET = 12.0
TURN_MISSION_FIRST_HOVER_SECONDS = 10.0
TURN_MISSION_SHORT_HOVER_SECONDS = 5.0
FULL_TURN_DEGREES = 360.0
FULL_TURN_DONE_DEGREES = 345.0
FULL_TURN_TIMEOUT_SECONDS = 25.0
YAW_RELATIVE = 1
YAW_CLOCKWISE = 1


async def fly10FeetE360W360N360S360Land(request):
    global flightIsRunning
    try:
        confirmationText = await confirmationTextFromRequest(request)
        if confirmationText != "YES":
            cancelledOutcome = {
                "scriptName": "fly10FeetE360W360N360S360Land",
                "started": False,
                "outcome": "CANCELLED BEFORE ARMING",
                "stopReason": "The request did not say YES. The copter was not armed.",
            }
            cancelledOutcome["id"] = saveFlightLog(cancelledOutcome)
            return cancelledOutcome

        from controllers.calibration import calibrationIsRunning
        from controllers.motorTest import motorTestIsRunning
        from controllers.flightPlan import flightPlanIsRunning, flightPlanVehicle

        async with flightGuard:
            if flightIsRunning or calibrationIsRunning or motorTestIsRunning or flightPlanIsRunning or flightPlanVehicle is not None:
                return {
                    "scriptName": "fly10FeetE360W360N360S360Land",
                    "started": False,
                    "outcome": "THE COMMAND CABLE IS ALREADY IN USE",
                }
            resetFlightControlFlags()
            flightIsRunning = True
        beginFlightProgress("fly10FeetE360W360N360S360Land")
        try:
            flightOutcome = await asyncio.to_thread(runTurnMission)
        finally:
            async with flightGuard:
                flightIsRunning = False
        flightOutcome["id"] = saveFlightLog(flightOutcome)
        publishFlightSnapshot(flightOutcome, running=False)
        return flightOutcome
    except Exception as error:
        print(f"fly10FeetE360W360N360S360Land did not finish: {error}")
        failedOutcome = {
            "scriptName": "fly10FeetE360W360N360S360Land",
            "started": False,
            "outcome": "FAILED",
            "stopReason": str(error),
        }
        failedOutcome["id"] = saveFlightLog(failedOutcome)
        return failedOutcome


def runTurnMission():
    global activeFlightVehicle, activeFlightOutcome
    flightOutcome = {
        "scriptName": "fly10FeetE360W360N360S360Land",
        "started": True,
        "startedAt": time.strftime("%Y-%m-%d %H:%M:%S %Z"),
        "finishedAt": None,
        "outcome": "FAILED",
        "maximumAltitudeReachedText": "none",
        "stopReason": None,
        "lines": [],
        "steps": {},
    }
    activeFlightOutcome = flightOutcome
    vehicle = None
    originalTakeoffSlewSeconds = None
    savedFailsafe = None
    copterWasArmed = False
    landingWasStarted = False
    droneKitTools = None
    try:
        writeFlightLine(
            flightOutcome,
            "TAKE OFF TO 6 FT -> HOVER AT 7 FT FOR 10S -> 360 CLOCKWISE -> HOVER 5S -> EAST 10 FT -> HOVER 5S -> 360 CLOCKWISE -> HOVER 5S -> WEST 20 FT -> HOVER 5S -> 360 CLOCKWISE -> HOVER 5S -> EAST 10 FT -> HOVER 5S -> NORTH 10 FT -> HOVER 5S -> 360 CLOCKWISE -> HOVER 5S -> SOUTH 10 FT -> HOVER 5S -> 360 CLOCKWISE -> HOVER 5S -> LAND",
        )
        if not os.path.exists(PIXHAWK_PORT):
            raise RuntimeError(f"{PIXHAWK_PORT} is missing. Run the backend on the Raspberry Pi.")
        from dronekit import VehicleMode
        from dronekit import connect
        from pymavlink import mavutil as middleMan

        droneKitTools = (VehicleMode, connect, middleMan)
        velocityTypeMask = (
            middleMan.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            | middleMan.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | middleMan.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | middleMan.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | middleMan.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | middleMan.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | middleMan.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            | middleMan.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        prearmHealthBit = getattr(middleMan.mavlink, "MAV_SYS_STATUS_PREARM_CHECK", 268435456)
        runPrearmCommandId = getattr(middleMan.mavlink, "MAV_CMD_RUN_PREARM_CHECKS", 401)
        writeFlightLine(flightOutcome, f"Connecting to Pixhawk on {PIXHAWK_PORT} at {PIXHAWK_BAUD} baud.")
        vehicle = connectUntilCleared(connect, flightOutcome)
        if vehicle._master.target_component == 0:
            vehicle._master.target_component = 1
        writeFlightLine(flightOutcome, "Pixhawk connected.")
        activeFlightVehicle = vehicle
        activeFlightOutcome = flightOutcome
        raiseIfFlightStopped()
        vehicle = ensureBoardCompassIsPrimary(
            vehicle,
            lambda currentVehicle: rebootFlightVehicle(currentVehicle, connect, flightOutcome),
            lambda text: writeFlightLine(flightOutcome, text),
            confirmPixhawkParameter,
        )
        vehicle.altitudeCeilingMeters = TURN_MISSION_CEILING_FEET / FEET_PER_METER
        activeFlightVehicle = vehicle
        latestTelemetry = listenForPreflightMessages(vehicle, prearmHealthBit, flightOutcome)
        requestFlightStreams(vehicle, middleMan)
        chooseCompassesForThisFlight(vehicle, lambda text: writeFlightLine(flightOutcome, text), confirmPixhawkParameter)
        waitUntilPreflightPasses(vehicle, latestTelemetry, runPrearmCommandId, flightOutcome)
        savedSlew = vehicle.parameters[TAKEOFF_SLEW_PARAMETER_NAME]
        if savedSlew is not None:
            savedSlew = float(savedSlew)
            takeoffSlew = min(savedSlew, PREFERRED_TAKEOFF_SLEW_SECONDS)
            if abs(takeoffSlew - savedSlew) >= PARAMETER_TOLERANCE:
                confirmPixhawkParameter(vehicle, TAKEOFF_SLEW_PARAMETER_NAME, takeoffSlew)
                originalTakeoffSlewSeconds = savedSlew
        savedFailsafe = armLandFailsafe(vehicle, flightOutcome)
        recheckPreflightAfterFailsafe(vehicle, latestTelemetry)
        commandFlightMode(vehicle, "GUIDED", VehicleMode, flightOutcome)
        copterWasArmed = True
        armAndWait(vehicle, flightOutcome)
        climbFromTakeoffToHover(vehicle, flightOutcome, TURN_MISSION_TAKEOFF_FEET, TURN_MISSION_HOVER_FEET)
        settleAtFiveFeet(vehicle, flightOutcome)
        commandFlightMode(vehicle, "LOITER", VehicleMode, flightOutcome)
        holdHoverAtFiveFeet(vehicle, flightOutcome, "at 7 feet", TURN_MISSION_FIRST_HOVER_SECONDS)

        commandFlightMode(vehicle, "GUIDED", VehicleMode, flightOutcome)
        faceGeographicNorth(vehicle, middleMan, velocityTypeMask, flightOutcome, altitudeFeet=TURN_MISSION_HOVER_FEET)
        turnClockwiseOneFullCircle(vehicle, middleMan, velocityTypeMask, flightOutcome, "after the first hover")
        commandFlightMode(vehicle, "LOITER", VehicleMode, flightOutcome)
        holdHoverAtFiveFeet(vehicle, flightOutcome, "after the first turn", TURN_MISSION_SHORT_HOVER_SECONDS)

        flyCardinalLeg(vehicle, flightOutcome, "east", 10, velocityTypeMask, VehicleMode, middleMan)
        turnClockwiseOneFullCircle(vehicle, middleMan, velocityTypeMask, flightOutcome, "after flying east")
        commandFlightMode(vehicle, "LOITER", VehicleMode, flightOutcome)
        holdHoverAtFiveFeet(vehicle, flightOutcome, "after the east turn", TURN_MISSION_SHORT_HOVER_SECONDS)

        flyCardinalLeg(vehicle, flightOutcome, "west", 20, velocityTypeMask, VehicleMode, middleMan)
        turnClockwiseOneFullCircle(vehicle, middleMan, velocityTypeMask, flightOutcome, "after flying west")
        commandFlightMode(vehicle, "LOITER", VehicleMode, flightOutcome)
        holdHoverAtFiveFeet(vehicle, flightOutcome, "after the west turn", TURN_MISSION_SHORT_HOVER_SECONDS)

        flyCardinalLeg(vehicle, flightOutcome, "east", 10, velocityTypeMask, VehicleMode, middleMan, hoverAfter=True)
        flyCardinalLeg(vehicle, flightOutcome, "north", 10, velocityTypeMask, VehicleMode, middleMan)
        turnClockwiseOneFullCircle(vehicle, middleMan, velocityTypeMask, flightOutcome, "after flying north")
        commandFlightMode(vehicle, "LOITER", VehicleMode, flightOutcome)
        holdHoverAtFiveFeet(vehicle, flightOutcome, "after the north turn", TURN_MISSION_SHORT_HOVER_SECONDS)

        flyCardinalLeg(vehicle, flightOutcome, "south", 10, velocityTypeMask, VehicleMode, middleMan)
        turnClockwiseOneFullCircle(vehicle, middleMan, velocityTypeMask, flightOutcome, "after flying south")
        commandFlightMode(vehicle, "LOITER", VehicleMode, flightOutcome)
        holdHoverAtFiveFeet(vehicle, flightOutcome, "before landing", TURN_MISSION_SHORT_HOVER_SECONDS)

        landingWasStarted = True
        landUntilPropellersStop(vehicle, flightOutcome, VehicleMode)
        copterWasArmed = vehicle.armed
        if not vehicle.armed:
            flightOutcome["outcome"] = "SUCCESS"
            flightOutcome["steps"]["land"] = "SUCCESS"
            writeFlightLine(flightOutcome, "Flight complete.")
    except KeyboardInterrupt:
        writeFlightLine(flightOutcome, "Flight interrupted.")
        flightOutcome["outcome"] = "EXITED MID-WAY"
        flightOutcome["stopReason"] = "Interrupted."
    except FlightClearRequested:
        flightOutcome["outcome"] = "CLEARED BEFORE FLYING"
        flightOutcome["stopReason"] = "Clear flights pressed. The request ended before takeoff."
        writeFlightLine(flightOutcome, "Flight request cleared before takeoff.")
    except FlightReturnHomeRequested:
        flightOutcome["outcome"] = "RETURNED HOME"
        flightOutcome["stopReason"] = "Return to home pressed. The copter was sent back to the takeoff spot to land."
        writeFlightLine(flightOutcome, "Return to home pressed. Flying back to the takeoff spot, then landing.")
        if vehicle is not None and getattr(vehicle, "armed", False) and not landingWasStarted:
            landingWasStarted = True
            try:
                from dronekit import VehicleMode
                returnHomeUntilLanded(vehicle, flightOutcome, VehicleMode)
            except Exception as landError:
                writeFlightLine(flightOutcome, f"Return home did not finish: {landError}. Landing here.")
                try:
                    from dronekit import VehicleMode
                    landUntilPropellersStop(vehicle, flightOutcome, VehicleMode)
                except Exception as secondError:
                    writeFlightLine(flightOutcome, f"Landing after return home did not finish: {secondError}")
    except FlightStopRequested:
        flightOutcome["outcome"] = "STOPPED BY PILOT"
        flightOutcome["stopReason"] = "Stop pressed. The copter was told to land."
        writeFlightLine(flightOutcome, "Stop pressed. Landing now.")
        if vehicle is not None and getattr(vehicle, "armed", False) and not landingWasStarted:
            landingWasStarted = True
            try:
                from dronekit import VehicleMode
                landUntilPropellersStop(vehicle, flightOutcome, VehicleMode)
            except Exception as landError:
                writeFlightLine(flightOutcome, f"Landing after stop did not finish: {landError}")
    except Exception as error:
        writeFlightLine(flightOutcome, f"Flight stopped: {error}")
        flightOutcome["outcome"] = "FAILED"
        flightOutcome["stopReason"] = str(error)
    finally:
        finishFlightConnection(
            vehicle,
            droneKitTools,
            flightOutcome,
            copterWasArmed,
            landingWasStarted,
            savedFailsafe,
            originalTakeoffSlewSeconds,
        )
    return flightOutcome


def climbFromTakeoffToHover(vehicle, flightOutcome, takeoffFeet, hoverFeet):
    hoverMeters = hoverFeet / FEET_PER_METER
    takeoffMeters = takeoffFeet / FEET_PER_METER
    writeFlightLine(flightOutcome, f"Taking off to {takeoffFeet:.0f} feet, then hovering at {hoverFeet:.0f} feet.")
    vehicle.simple_takeoff(hoverMeters)
    passedTakeoff = False
    started = time.monotonic()
    lastPrint = 0.0
    while True:
        raiseIfFlightStopped()
        if vehicle.mode.name != "GUIDED":
            raiseIfFlightStopped()
            raise RuntimeError("GUIDED mode lost during takeoff.")
        altitudeMeters = checkFlightHealthAndCeiling(vehicle, flightOutcome)
        nowTime = time.monotonic()
        if nowTime - lastPrint >= PRINT_INTERVAL_SECONDS:
            heightText = "waiting" if altitudeMeters is None else f"{altitudeMeters * FEET_PER_METER:.2f} ft"
            writeFlightLine(flightOutcome, f"Climbing | {heightText}")
            lastPrint = nowTime
        if altitudeMeters is not None and not passedTakeoff and altitudeMeters >= takeoffMeters * TAKEOFF_REACHED_FRACTION:
            passedTakeoff = True
            writeFlightLine(flightOutcome, f"Reached {takeoffFeet:.0f} feet. Continuing to {hoverFeet:.0f} feet.")
        if altitudeMeters is not None and altitudeMeters >= hoverMeters * TAKEOFF_REACHED_FRACTION:
            writeFlightLine(flightOutcome, f"Reached {hoverFeet:.0f} feet ({altitudeMeters * FEET_PER_METER:.2f} ft).")
            return
        if nowTime - started > TAKEOFF_TIMEOUT_SECONDS:
            raise RuntimeError("Takeoff timed out.")
        time.sleep(STATUS_CHECK_INTERVAL_SECONDS)


def flyCardinalLeg(vehicle, flightOutcome, directionName, distanceFeet, velocityTypeMask, vehicleMode, middleMan, hoverAfter=True):
    speeds = {
        "east": (0.0, LEG_SPEED_METERS_PER_SECOND),
        "west": (0.0, -LEG_SPEED_METERS_PER_SECOND),
        "north": (LEG_SPEED_METERS_PER_SECOND, 0.0),
        "south": (-LEG_SPEED_METERS_PER_SECOND, 0.0),
    }
    northSpeed, eastSpeed = speeds[directionName]
    if vehicle.mode.name != "GUIDED":
        commandFlightMode(vehicle, "GUIDED", vehicleMode, flightOutcome)
    flyTenFeet(
        vehicle,
        flightOutcome,
        directionName,
        northSpeed,
        eastSpeed,
        velocityTypeMask,
        distanceFeet=distanceFeet,
        altitudeFeet=TURN_MISSION_HOVER_FEET,
    )
    if hoverAfter:
        commandFlightMode(vehicle, "LOITER", vehicleMode, flightOutcome)
        holdHoverAtFiveFeet(vehicle, flightOutcome, f"after {directionName} {distanceFeet:.0f} ft", TURN_MISSION_SHORT_HOVER_SECONDS)
        commandFlightMode(vehicle, "GUIDED", vehicleMode, flightOutcome)


def turnClockwiseOneFullCircle(vehicle, middleMan, velocityTypeMask, flightOutcome, turnName):
    raiseIfFlightStopped()
    if vehicle.mode.name != "GUIDED":
        raise RuntimeError("GUIDED mode was lost before the clockwise turn.")
    writeFlightLine(flightOutcome, f"Turning clockwise 360 degrees, {turnName}. Keep this pose. Do not flip the drone.")
    targetAltitudeMeters = TURN_MISSION_HOVER_FEET / FEET_PER_METER
    lastHeading = math.degrees(float(vehicle.attitude.yaw or 0.0))
    turnedDegrees = 0.0
    started = time.monotonic()
    lastPrint = 0.0
    altitudeMeters = checkFlightHealthAndCeiling(vehicle, flightOutcome)
    sendLocalVelocity(vehicle, 0.0, 0.0, downSpeedToHoldFiveFeet(altitudeMeters, targetAltitudeMeters), velocityTypeMask)
    commandClockwiseRelativeTurn(vehicle, middleMan)
    while turnedDegrees < FULL_TURN_DONE_DEGREES:
        raiseIfFlightStopped()
        if vehicle.mode.name != "GUIDED":
            raiseIfFlightStopped()
            raise RuntimeError("GUIDED mode lost during the clockwise turn.")
        altitudeMeters = checkFlightHealthAndCeiling(vehicle, flightOutcome)
        heading = math.degrees(float(vehicle.attitude.yaw or 0.0))
        change = (heading - lastHeading + 540.0) % 360.0 - 180.0
        if change > 0:
            turnedDegrees += change
        lastHeading = heading
        remainingDegrees = max(30.0, FULL_TURN_DEGREES - turnedDegrees)
        sendLocalVelocity(vehicle, 0.0, 0.0, downSpeedToHoldFiveFeet(altitudeMeters, targetAltitudeMeters), velocityTypeMask)
        commandClockwiseRelativeTurn(vehicle, middleMan, remainingDegrees)
        nowTime = time.monotonic()
        if nowTime - lastPrint >= PRINT_INTERVAL_SECONDS:
            writeFlightLine(flightOutcome, f"Clockwise turn | {turnedDegrees:.0f} of 360 degrees")
            lastPrint = nowTime
        if nowTime - started > FULL_TURN_TIMEOUT_SECONDS:
            raise RuntimeError(f"The clockwise 360 did not finish ({turnedDegrees:.0f} degrees).")
        time.sleep(STATUS_CHECK_INTERVAL_SECONDS)
    writeFlightLine(flightOutcome, f"Clockwise 360 complete ({turnedDegrees:.0f} degrees).")


def commandClockwiseRelativeTurn(vehicle, middleMan, turnDegrees=FULL_TURN_DEGREES):
    yawMessage = vehicle.message_factory.command_long_encode(
        vehicle._master.target_system,
        vehicle._master.target_component,
        middleMan.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        float(turnDegrees),
        YAW_TURN_RATE_DEGREES_PER_SECOND,
        YAW_CLOCKWISE,
        YAW_RELATIVE,
        0, 0, 0,
    )
    vehicle.send_mavlink(yawMessage)
    vehicle.flush()

