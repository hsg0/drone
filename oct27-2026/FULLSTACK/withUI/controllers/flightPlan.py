# WHAT — save a map plan, arm, disarm, or fly that plan.
# WHY — the page picks the path first. The copter does not move until Arm,
#       and it does not take off until YES. A perimeter flight stays inside
#       the drawn shape and lands after 5 minutes.
# HOW — Turf distance is checked again here. Arm applies the Oct 26 preflight
#       and onboard Land failsafe gates. The straight plan flies one bearing.
#       The perimeter plan flies about 6 feet inside the drawn corners.
# IMPORTANT — there is no force-arm and no force-disarm. A straight path over
#             1000 feet is refused. A perimeter over 10000 feet is refused.
#             YES does not arm. Arm does not take off.

import asyncio
import json
import math
import os
import threading
import time

import socketio

from models.flightlogs import readFlightLog, recordFromFlightLog, saveFlightLog

PIXHAWK_PORT = "/dev/serial0"
PIXHAWK_BAUD = 57600
GROUND_STATION_SYSTEM_ID = 255
CONNECTION_TIMEOUT_SECONDS = 60
FEET_PER_METER = 3.28084
METERS_PER_DEGREE_OF_LATITUDE = 111319.5
MAXIMUM_DISTANCE_FEET = 1000.0
MAXIMUM_PERIMETER_FEET = 10000.0
DEFAULT_HEIGHT_FEET = 5.0
MAXIMUM_HEIGHT_FEET = 50.0
MINIMUM_HEIGHT_FEET = 3.0
DEFAULT_SPEED_METERS_PER_SECOND = 0.4
MAXIMUM_SPEED_METERS_PER_SECOND = 2.0
MINIMUM_SPEED_METERS_PER_SECOND = 0.2
CEILING_MARGIN_FEET = 10.0
CROSS_TRACK_LIMIT_FEET = 50.0
PERIMETER_FLIGHT_SECONDS = 300.0
INSET_FEET = 6.0
MINIMUM_PERIMETER_CORNERS = 3
MAXIMUM_PERIMETER_CORNERS = 12
CORNER_ARRIVAL_METERS = 0.6
DEFAULT_CORRIDOR_WIDTH_FEET = 20.0
MINIMUM_CORRIDOR_WIDTH_FEET = 6.0
MAXIMUM_CORRIDOR_WIDTH_FEET = 200.0
# A second click on a corner, or a click back on the start, is the same place.
SAME_CORNER_FEET = 8.0
TAKEOFF_REACHED_FRACTION = 0.95
TAKEOFF_SETTLE_SECONDS = 2.0
SETTLE_AFTER_MOVE_SECONDS = 2.0
TAKEOFF_TIMEOUT_SECONDS = 45.0
LAND_TIMEOUT_SECONDS = 60.0
DISARM_TIMEOUT_SECONDS = 15.0
PREFLIGHT_WAIT_TIMEOUT_SECONDS = 30.0
MODE_CHANGE_TIMEOUT_SECONDS = 10.0
ARM_TIMEOUT_SECONDS = 15.0
HEARTBEAT_STALE_AFTER_SECONDS = 3.0
STATUS_CHECK_INTERVAL_SECONDS = 0.2
PRINT_INTERVAL_SECONDS = 1.0
PARAMETER_TOLERANCE = 0.01

QUAD_FRAME_CLASS = 1
QUAD_X_FRAME_TYPE = 1
MINIMUM_GPS_FIX_TYPE = 3
MINIMUM_SATELLITE_COUNT = 5
UNKNOWN_SATELLITE_COUNT = 255
SKIP_COMPASS_OR_GPS_MASK = 4 | 8 | 4096
MAXIMUM_COMPASS_VARIANCE = 0.50
MAXIMUM_GROUND_LEAN_DEGREES = 10.0
MAXIMUM_SAVED_TRIM_DEGREES = 8.0
PREARM_POLL_INTERVAL_SECONDS = 2.0
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
ALTITUDE_HOLD_GAIN_DURING_MOVE = 0.5
MAXIMUM_VERTICAL_CORRECTION_METERS_PER_SECOND = 0.25

flightPlanVehicle = None
flightPlanIsRunning = False
flightPlanGuard = asyncio.Lock()
perimeterProgressLock = threading.Lock()
perimeterProgressLoop = None
perimeterProgress = {
    "phase": "idle",
    "latitude": None,
    "longitude": None,
    "heightFeet": None,
    "lockPoint": None,
    "secondsRemaining": None,
    "insideFence": None,
    "insideCorridor": None,
    "outcome": None,
    "stopReason": None,
}
lastPerimeterProgressAt = 0.0

perimeterSocket = socketio.AsyncServer(async_mode="asgi", cors_allowed_origins="*")


@perimeterSocket.event
async def connect(sid, _environ):
    await perimeterSocket.emit("perimeter-progress", readPerimeterProgress(), to=sid)


def rememberPerimeterLoop(loop):
    global perimeterProgressLoop
    perimeterProgressLoop = loop


def readPerimeterProgress():
    with perimeterProgressLock:
        return dict(perimeterProgress)


def notePerimeterProgress(vehicle=None, force=False, **fields):
    """Remember the latest flight note and push it to the page. The flight thread calls this."""
    global lastPerimeterProgressAt
    latitude = None
    longitude = None
    heightFeet = None
    if vehicle is not None:
        location = getattr(vehicle, "location", None)
        frame = getattr(location, "global_relative_frame", None) if location is not None else None
        if frame is not None and frame.lat is not None and frame.lon is not None:
            latitude = float(frame.lat)
            longitude = float(frame.lon)
            if frame.alt is not None and math.isfinite(float(frame.alt)):
                heightFeet = round(float(frame.alt) * FEET_PER_METER, 1)
    now = time.monotonic()
    with perimeterProgressLock:
        if latitude is not None:
            perimeterProgress["latitude"] = latitude
            perimeterProgress["longitude"] = longitude
            perimeterProgress["heightFeet"] = heightFeet
        perimeterProgress.update(fields)
        snapshot = dict(perimeterProgress)
        if not force and now - lastPerimeterProgressAt < 0.5:
            return
        lastPerimeterProgressAt = now
    if perimeterProgressLoop is None:
        return
    def pushSnapshot(progressSnapshot=snapshot):
        asyncio.create_task(perimeterSocket.emit("perimeter-progress", progressSnapshot))
    perimeterProgressLoop.call_soon_threadsafe(pushSnapshot)


async def saveFlightPlan(request):
    try:
        body = await requestJson(request)
        plan = confirmedPlan(body)
    except Exception as error:
        return failedPlan(str(error))
    if plan["distanceFeet"] > MAXIMUM_DISTANCE_FEET:
        return failedPlan(
            f"The path is {plan['distanceFeet']:.0f} feet. The limit is {MAXIMUM_DISTANCE_FEET:.0f} feet."
        )
    plan["scriptName"] = "flightPlan"
    plan["outcome"] = "SAVED"
    plan["id"] = saveFlightLog(plan)
    return plan


async def armFlightPlan(request):
    global flightPlanIsRunning
    busy = cableIsBusy()
    if busy:
        return failedPlan(busy)
    async with flightPlanGuard:
        if flightPlanIsRunning:
            return failedPlan("Arm is already checking the copter. Wait for that check to finish.")
        flightPlanIsRunning = True
    try:
        result = await asyncio.to_thread(armCopter)
        return result
    except Exception as error:
        return failedPlan(str(error))
    finally:
        async with flightPlanGuard:
            flightPlanIsRunning = False


async def disarmFlightPlan(request):
    global flightPlanIsRunning
    async with flightPlanGuard:
        if flightPlanIsRunning:
            return failedPlan("A flight plan is currently running.")
        flightPlanIsRunning = True
    try:
        result = await asyncio.to_thread(disarmAndCloseConnection)
        return result
    except Exception as error:
        return failedPlan(str(error))
    finally:
        async with flightPlanGuard:
            flightPlanIsRunning = False


async def flyFlightPlan(request):
    global flightPlanIsRunning
    try:
        body = await requestJson(request)
    except Exception as error:
        return failedPlan(str(error))
    if str(body.get("confirm", "")).strip() != "YES":
        return failedPlan("The request did not say YES. The copter was not taken off.")
    plan = savedPlan(body.get("planId"))
    if plan is None:
        return failedPlan("That saved flight plan was not found.")
    if plan.get("distanceFeet", MAXIMUM_DISTANCE_FEET + 1) > MAXIMUM_DISTANCE_FEET:
        return failedPlan("That saved path is longer than 1000 feet.")
    busy = cableIsBusy()
    if busy:
        return failedPlan(busy)
    async with flightPlanGuard:
        if flightPlanIsRunning:
            return failedPlan("A flight plan is already using the command cable.")
        flightPlanIsRunning = True
    try:
        result = await asyncio.to_thread(flySavedPlan, plan)
        result["id"] = saveFlightLog(result)
        return result
    except Exception as error:
        failed = failedPlan(str(error))
        failed["scriptName"] = "flightPlan"
        return failed
    finally:
        async with flightPlanGuard:
            flightPlanIsRunning = False


async def savePerimeterPlan(request):
    try:
        body = await requestJson(request)
        plan = confirmedPerimeter(body)
    except Exception as error:
        return failedPlan(str(error))
    plan["scriptName"] = "perimeterFlight"
    plan["outcome"] = "SAVED"
    plan["id"] = saveFlightLog(plan)
    return plan


async def flyPerimeterPlan(request):
    global flightPlanIsRunning
    try:
        body = await requestJson(request)
    except Exception as error:
        return failedPlan(str(error))
    if str(body.get("confirm", "")).strip() != "YES":
        return failedPlan("The request did not say YES. The copter was not taken off.")
    plan = savedPerimeter(body.get("planId"))
    if plan is None:
        return failedPlan("That saved perimeter was not found.")
    if plan.get("perimeterFeet", MAXIMUM_PERIMETER_FEET + 1) > MAXIMUM_PERIMETER_FEET:
        return failedPlan("That saved perimeter is longer than 10000 feet.")
    busy = cableIsBusy()
    if busy:
        return failedPlan(busy)
    async with flightPlanGuard:
        if flightPlanIsRunning:
            return failedPlan("A flight plan is already using the command cable.")
        flightPlanIsRunning = True
    try:
        result = await asyncio.to_thread(flySavedPerimeter, plan)
        result["id"] = saveFlightLog(result)
        return result
    except Exception as error:
        failed = failedPlan(str(error))
        failed["scriptName"] = "perimeterFlight"
        return failed
    finally:
        async with flightPlanGuard:
            flightPlanIsRunning = False


async def requestJson(request):
    rawBody = await request.body()
    if not rawBody:
        return {}
    payload = json.loads(rawBody)
    if not isinstance(payload, dict):
        raise RuntimeError("The flight plan was not a JSON object.")
    return payload


def confirmedPlan(body):
    pointA = coordinatePair(body.get("pointA"), "A")
    pointB = coordinatePair(body.get("pointB"), "B")
    distanceFeet = haversineFeet(pointA["lat"], pointA["lon"], pointB["lat"], pointB["lon"])
    bearingDegrees = bearingDegreesClockwise(pointA["lat"], pointA["lon"], pointB["lat"], pointB["lon"])
    heightFeet = boundedNumber(
        body.get("heightFeet", DEFAULT_HEIGHT_FEET),
        MINIMUM_HEIGHT_FEET,
        MAXIMUM_HEIGHT_FEET,
        "Height",
    )
    speedMetersPerSecond = boundedNumber(
        body.get("speedMetersPerSecond", DEFAULT_SPEED_METERS_PER_SECOND),
        MINIMUM_SPEED_METERS_PER_SECOND,
        MAXIMUM_SPEED_METERS_PER_SECOND,
        "Speed",
    )
    return {
        "pointA": pointA,
        "pointB": pointB,
        "distanceFeet": round(distanceFeet, 1),
        "bearingDegrees": round(bearingDegrees, 1),
        "heightFeet": round(heightFeet, 1),
        "speedMetersPerSecond": round(speedMetersPerSecond, 2),
    }


def coordinatePair(value, label):
    if not isinstance(value, dict):
        raise RuntimeError(f"Point {label} is missing.")
    latitude = float(value.get("lat"))
    longitude = float(value.get("lon"))
    if not (-90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0):
        raise RuntimeError(f"Point {label} is not a real latitude and longitude.")
    if abs(latitude) < 0.01 and abs(longitude) < 0.01:
        raise RuntimeError(f"Point {label} is empty.")
    return {"lat": latitude, "lon": longitude}


def boundedNumber(value, minimum, maximum, label):
    number = float(value)
    if not math.isfinite(number) or number < minimum or number > maximum:
        raise RuntimeError(f"{label} must be between {minimum} and {maximum}.")
    return number


def savedPlan(planId):
    flightLog = readFlightLog(planId)
    if flightLog is None:
        return None
    record = recordFromFlightLog(flightLog)
    details = record.get("details")
    if not isinstance(details, dict) or details.get("scriptName") != "flightPlan":
        return None
    if "pointA" not in details or "pointB" not in details:
        return None
    return details | {"id": record["id"]}


def savedPerimeter(planId):
    flightLog = readFlightLog(planId)
    if flightLog is None:
        return None
    record = recordFromFlightLog(flightLog)
    details = record.get("details")
    if not isinstance(details, dict) or details.get("scriptName") != "perimeterFlight":
        return None
    if not isinstance(details.get("corners"), list) or not isinstance(details.get("insetCorners"), list):
        return None
    return details | {"id": record["id"]}


def confirmedPerimeter(body):
    rawCorners = body.get("corners")
    if not isinstance(rawCorners, list):
        raise RuntimeError("The perimeter corners are missing.")
    corners = [coordinatePair(corner, index + 1) for index, corner in enumerate(rawCorners)]
    corners = cornersWithoutStackedClicks(corners)
    if len(corners) < MINIMUM_PERIMETER_CORNERS:
        raise RuntimeError("The corners are too close together to make a fence. Click farther apart.")
    if len(corners) > MAXIMUM_PERIMETER_CORNERS:
        raise RuntimeError(f"Draw no more than {MAXIMUM_PERIMETER_CORNERS} corners.")
    perimeterFeet = perimeterLengthFeet(corners)
    if perimeterFeet > MAXIMUM_PERIMETER_FEET:
        raise RuntimeError(
            f"The perimeter is {perimeterFeet:.0f} feet. The limit is {MAXIMUM_PERIMETER_FEET:.0f} feet."
        )
    heightFeet = boundedNumber(
        body.get("heightFeet", DEFAULT_HEIGHT_FEET),
        MINIMUM_HEIGHT_FEET,
        MAXIMUM_HEIGHT_FEET,
        "Height",
    )
    speedMetersPerSecond = boundedNumber(
        body.get("speedMetersPerSecond", DEFAULT_SPEED_METERS_PER_SECOND),
        MINIMUM_SPEED_METERS_PER_SECOND,
        MAXIMUM_SPEED_METERS_PER_SECOND,
        "Speed",
    )
    corridorWidthFeet = boundedNumber(
        body.get("corridorWidthFeet", DEFAULT_CORRIDOR_WIDTH_FEET),
        MINIMUM_CORRIDOR_WIDTH_FEET,
        MAXIMUM_CORRIDOR_WIDTH_FEET,
        "Corridor width",
    )
    insetCorners, insetFeet = cornersInsetTowardMiddle(corners)
    return {
        "corners": corners,
        "insetCorners": insetCorners,
        "insetFeet": insetFeet,
        "perimeterFeet": round(perimeterFeet, 1),
        "heightFeet": round(heightFeet, 1),
        "speedMetersPerSecond": round(speedMetersPerSecond, 2),
        "corridorWidthFeet": round(corridorWidthFeet, 1),
        "flightSeconds": PERIMETER_FLIGHT_SECONDS,
    }


def flySavedPerimeter(plan):
    vehicle = flightPlanVehicle
    landingWasStarted = False
    result = {
        "scriptName": "perimeterFlight",
        "started": True,
        "planId": plan.get("id"),
        "perimeterFeet": plan.get("perimeterFeet"),
        "heightFeet": plan.get("heightFeet"),
        "speedMetersPerSecond": plan.get("speedMetersPerSecond"),
        "corridorWidthFeet": plan.get("corridorWidthFeet", DEFAULT_CORRIDOR_WIDTH_FEET),
        "flightSeconds": PERIMETER_FLIGHT_SECONDS,
        "maximumAltitudeReachedText": "none",
        "outcome": "FAILED",
        "stopReason": None,
    }
    try:
        if vehicle is None or not heartbeatIsFresh(vehicle) or not vehicle.armed:
            raise RuntimeError("The copter is not armed. Press Arm before YES.")
        from dronekit import VehicleMode
        heightFeet = float(plan["heightFeet"])
        heightMeters = heightFeet / FEET_PER_METER
        ceilingMeters = (heightFeet + CEILING_MARGIN_FEET) / FEET_PER_METER
        speed = float(plan["speedMetersPerSecond"])
        corridorWidthFeet = float(plan.get("corridorWidthFeet", DEFAULT_CORRIDOR_WIDTH_FEET))
        corridorHalfMeters = (corridorWidthFeet / 2.0) / FEET_PER_METER
        drawnCorners = plan["corners"]
        insetCorners = plan["insetCorners"]
        notePerimeterProgress(vehicle, force=True, phase="taking-off", lockPoint=None, secondsRemaining=PERIMETER_FLIGHT_SECONDS, outcome=None, stopReason=None)
        if not copterIsInsidePerimeter(vehicle, drawnCorners):
            raise RuntimeError("The copter is outside the drawn perimeter. It was not taken off.")

        setMode(vehicle, "GUIDED", VehicleMode)
        print(f"Taking off to {heightFeet:.0f} feet.")
        vehicle.simple_takeoff(heightMeters)
        takeoffStarted = time.monotonic()
        while True:
            altitudeMeters = checkGuidedFlight(vehicle, ceilingMeters, result)
            notePerimeterProgress(vehicle, phase="taking-off", secondsRemaining=PERIMETER_FLIGHT_SECONDS)
            if altitudeMeters is not None and altitudeMeters >= heightMeters * TAKEOFF_REACHED_FRACTION:
                break
            if time.monotonic() - takeoffStarted > TAKEOFF_TIMEOUT_SECONDS:
                raise RuntimeError("Takeoff timed out.")
            time.sleep(STATUS_CHECK_INTERVAL_SECONDS)

        settleStarted = time.monotonic()
        while time.monotonic() - settleStarted < TAKEOFF_SETTLE_SECONDS:
            checkGuidedFlight(vehicle, ceilingMeters, result)
            notePerimeterProgress(vehicle, phase="taking-off", secondsRemaining=PERIMETER_FLIGHT_SECONDS)
            time.sleep(STATUS_CHECK_INTERVAL_SECONDS)

        print("Flying the inside edge for 5 minutes.")
        patternStarted = time.monotonic()
        cornerIndex = 0
        while time.monotonic() - patternStarted < PERIMETER_FLIGHT_SECONDS:
            checkGuidedFlight(vehicle, ceilingMeters, result)
            insideFence = copterIsInsidePerimeter(vehicle, drawnCorners)
            distanceFromLineMeters = distanceToFlightLineMeters(vehicle, insetCorners)
            insideCorridor = distanceFromLineMeters <= corridorHalfMeters
            secondsRemaining = max(0, int(PERIMETER_FLIGHT_SECONDS - (time.monotonic() - patternStarted)))
            notePerimeterProgress(
                vehicle,
                phase="flying",
                lockPoint=cornerIndex + 1,
                secondsRemaining=secondsRemaining,
                insideFence=insideFence,
                insideCorridor=insideCorridor,
            )
            if not insideFence:
                raise RuntimeError("The copter left the drawn perimeter.")
            if not insideCorridor:
                raise RuntimeError("The copter left the flight corridor.")
            target = insetCorners[cornerIndex]
            northMeters, eastMeters = metersFromCopterTo(vehicle, target["lat"], target["lon"])
            distanceMeters = math.hypot(northMeters, eastMeters)
            if distanceMeters <= CORNER_ARRIVAL_METERS:
                cornerIndex = (cornerIndex + 1) % len(insetCorners)
                continue
            sendVelocity(
                vehicle,
                speed * northMeters / distanceMeters,
                speed * eastMeters / distanceMeters,
                downSpeedForHeight(vehicle, heightMeters),
            )
            time.sleep(STATUS_CHECK_INTERVAL_SECONDS)

        brakeStarted = time.monotonic()
        while time.monotonic() - brakeStarted < SETTLE_AFTER_MOVE_SECONDS:
            checkGuidedFlight(vehicle, ceilingMeters, result)
            sendVelocity(vehicle, 0.0, 0.0, downSpeedForHeight(vehicle, heightMeters))
            time.sleep(STATUS_CHECK_INTERVAL_SECONDS)
        sendVelocity(vehicle, 0.0, 0.0, 0.0)

        notePerimeterProgress(vehicle, force=True, phase="landing", secondsRemaining=0)
        landingWasStarted = True
        landAndWaitForDisarm(vehicle, VehicleMode, result)
        if result["outcome"] != "ONBOARD FAILSAFE":
            result["outcome"] = "SUCCESS"
            result["stopReason"] = "Flew the inside edge for 5 minutes, then landed."
        notePerimeterProgress(
            vehicle,
            force=True,
            phase="finished",
            outcome=result["outcome"],
            stopReason=result["stopReason"],
            secondsRemaining=0,
        )
        return result
    except Exception as error:
        result["stopReason"] = str(error)
        notePerimeterProgress(vehicle, force=True, phase="finished", outcome="FAILED", stopReason=str(error))
        return result
    finally:
        if vehicle is not None:
            if getattr(vehicle, "armed", False) and not landingWasStarted:
                try:
                    from dronekit import VehicleMode
                    landAndWaitForDisarm(vehicle, VehicleMode, result)
                except Exception as landError:
                    print(f"Emergency land did not confirm disarm: {landError}")
            restoreAndClosePlanVehicle(vehicle)


def cornersWithoutStackedClicks(corners):
    """Drop a click that lands on the previous corner, and a click back on the start."""
    if not corners:
        return []
    keptCorners = [corners[0]]
    for corner in corners[1:]:
        previousCorner = keptCorners[-1]
        separationFeet = haversineFeet(previousCorner["lat"], previousCorner["lon"], corner["lat"], corner["lon"])
        if separationFeet >= SAME_CORNER_FEET:
            keptCorners.append(corner)
    if len(keptCorners) >= 2:
        closingFeet = haversineFeet(
            keptCorners[0]["lat"], keptCorners[0]["lon"], keptCorners[-1]["lat"], keptCorners[-1]["lon"]
        )
        if closingFeet < SAME_CORNER_FEET:
            keptCorners = keptCorners[:-1]
    return keptCorners


def samePlace(firstCorner, secondCorner):
    return (
        abs(firstCorner["lat"] - secondCorner["lat"]) < 0.0000001
        and abs(firstCorner["lon"] - secondCorner["lon"]) < 0.0000001
    )


def perimeterLengthFeet(corners):
    totalFeet = 0.0
    for index, corner in enumerate(corners):
        nextCorner = corners[(index + 1) % len(corners)]
        totalFeet += haversineFeet(corner["lat"], corner["lon"], nextCorner["lat"], nextCorner["lon"])
    return totalFeet


def cornersInsetTowardMiddle(corners):
    """Build a flight path inside the drawing. Use 6 feet when it fits, otherwise the largest inset that stays inside."""
    origin = corners[0]
    localCorners = [localNorthEastMeters(corner["lat"], corner["lon"], origin["lat"], origin["lon"]) for corner in corners]
    drawnPairs = [(east, north) for north, east in localCorners]
    for insetFeet in (6, 5, 4, 3, 2, 1):
        insetLocal = insetCornersByFeet(localCorners, drawnPairs, insetFeet)
        if insetLocal is not None and flightPathStaysInside(insetLocal, drawnPairs):
            return latLonCorners(origin, insetLocal), insetFeet
    raise RuntimeError("The perimeter is too small to fly inside it.")


def insetCornersByFeet(localCorners, drawnPairs, insetFeet):
    """Shift each corner inward along the fence angle. A corner that would leave the fence is rejected."""
    insetMeters = insetFeet / FEET_PER_METER
    edgeNormals = [inwardEdgeNormal(localCorners, drawnPairs, index) for index in range(len(localCorners))]
    insetLocal = []
    for index, (north, east) in enumerate(localCorners):
        previousNormal = edgeNormals[(index - 1) % len(edgeNormals)]
        currentNormal = edgeNormals[index]
        bisectorNorth = previousNormal[0] + currentNormal[0]
        bisectorEast = previousNormal[1] + currentNormal[1]
        bisectorLength = math.hypot(bisectorNorth, bisectorEast)
        if bisectorLength < 0.2:
            bisectorNorth, bisectorEast = currentNormal
        else:
            bisectorNorth /= bisectorLength
            bisectorEast /= bisectorLength
        cornerDot = previousNormal[0] * currentNormal[0] + previousNormal[1] * currentNormal[1]
        halfAngleSine = math.sqrt(max(0.0, (1.0 - cornerDot) / 2.0))
        moveMeters = insetMeters / max(halfAngleSine, 0.35)
        movedNorth = north + bisectorNorth * moveMeters
        movedEast = east + bisectorEast * moveMeters
        if not localPointIsInside(movedEast, movedNorth, drawnPairs):
            return None
        insetLocal.append((movedNorth, movedEast))
    return insetLocal


def inwardEdgeNormal(localCorners, drawnPairs, index):
    """Unit step from the middle of one edge toward the inside of the fence."""
    north, east = localCorners[index]
    nextNorth, nextEast = localCorners[(index + 1) % len(localCorners)]
    edgeNorth = nextNorth - north
    edgeEast = nextEast - east
    edgeLength = math.hypot(edgeNorth, edgeEast) or 1.0
    leftNorth = -edgeEast / edgeLength
    leftEast = edgeNorth / edgeLength
    middleNorth = (north + nextNorth) / 2.0
    middleEast = (east + nextEast) / 2.0
    if localPointIsInside(middleEast + leftEast * 2.0, middleNorth + leftNorth * 2.0, drawnPairs, countEdge=False):
        return (leftNorth, leftEast)
    return (-leftNorth, -leftEast)


def flightPathStaysInside(insetLocal, drawnPairs):
    for index, (north, east) in enumerate(insetLocal):
        nextNorth, nextEast = insetLocal[(index + 1) % len(insetLocal)]
        for step in (0.25, 0.5, 0.75):
            sampleEast = east + (nextEast - east) * step
            sampleNorth = north + (nextNorth - north) * step
            if not localPointIsInside(sampleEast, sampleNorth, drawnPairs):
                return False
    return True


def latLonCorners(origin, localCorners):
    return [
        {
            "lat": origin["lat"] + north / METERS_PER_DEGREE_OF_LATITUDE,
            "lon": origin["lon"] + east / (
                METERS_PER_DEGREE_OF_LATITUDE * math.cos(math.radians(origin["lat"]))
            ),
        }
        for north, east in localCorners
    ]


def copterIsInsidePerimeter(vehicle, corners):
    location = vehicle.location.global_relative_frame
    if location is None or location.lat is None or location.lon is None:
        raise RuntimeError("GPS position was lost.")
    origin = corners[0]
    north, east = localNorthEastMeters(float(location.lat), float(location.lon), origin["lat"], origin["lon"])
    drawnPairs = [
        localNorthEastMeters(corner["lat"], corner["lon"], origin["lat"], origin["lon"])
        for corner in corners
    ]
    return localPointIsInside(east, north, [(pointEast, pointNorth) for pointNorth, pointEast in drawnPairs])


def distanceToFlightLineMeters(vehicle, insetCorners):
    """How far the copter is from the nearest part of the inside flight line."""
    location = vehicle.location.global_relative_frame
    if location is None or location.lat is None or location.lon is None:
        raise RuntimeError("GPS position was lost.")
    origin = insetCorners[0]
    north, east = localNorthEastMeters(float(location.lat), float(location.lon), origin["lat"], origin["lon"])
    localCorners = [
        localNorthEastMeters(corner["lat"], corner["lon"], origin["lat"], origin["lon"])
        for corner in insetCorners
    ]
    nearestMeters = None
    for index, (cornerNorth, cornerEast) in enumerate(localCorners):
        nextNorth, nextEast = localCorners[(index + 1) % len(localCorners)]
        edgeMeters = distanceToSegmentMeters(east, north, cornerEast, cornerNorth, nextEast, nextNorth)
        if nearestMeters is None or edgeMeters < nearestMeters:
            nearestMeters = edgeMeters
    return nearestMeters


def metersFromCopterTo(vehicle, latitude, longitude):
    location = vehicle.location.global_relative_frame
    if location is None or location.lat is None or location.lon is None:
        raise RuntimeError("GPS position was lost.")
    return localNorthEastMeters(latitude, longitude, float(location.lat), float(location.lon))


def localNorthEastMeters(latitude, longitude, originLatitude, originLongitude):
    northMeters = (latitude - originLatitude) * METERS_PER_DEGREE_OF_LATITUDE
    eastMeters = (
        (longitude - originLongitude)
        * METERS_PER_DEGREE_OF_LATITUDE
        * math.cos(math.radians(originLatitude))
    )
    return northMeters, eastMeters


def localPointIsInside(east, north, polygonEastNorth, countEdge=True):
    """Ray cast. A point on the drawn line counts as inside so a tiny GPS wobble does not land the copter."""
    if countEdge and pointIsNearPerimeterEdge(east, north, polygonEastNorth):
        return True
    inside = False
    previousIndex = len(polygonEastNorth) - 1
    for index, (edgeEast, edgeNorth) in enumerate(polygonEastNorth):
        previousEast, previousNorth = polygonEastNorth[previousIndex]
        crossesNorth = (edgeNorth > north) != (previousNorth > north)
        if crossesNorth:
            crossedEast = (previousEast - edgeEast) * (north - edgeNorth) / (previousNorth - edgeNorth) + edgeEast
            if east < crossedEast:
                inside = not inside
        previousIndex = index
    return inside


def pointIsNearPerimeterEdge(east, north, polygonEastNorth):
    edgeMarginMeters = 0.4
    previousIndex = len(polygonEastNorth) - 1
    for index, (edgeEast, edgeNorth) in enumerate(polygonEastNorth):
        previousEast, previousNorth = polygonEastNorth[previousIndex]
        if distanceToSegmentMeters(east, north, previousEast, previousNorth, edgeEast, edgeNorth) <= edgeMarginMeters:
            return True
        previousIndex = index
    return False


def distanceToSegmentMeters(east, north, startEast, startNorth, endEast, endNorth):
    segmentEast = endEast - startEast
    segmentNorth = endNorth - startNorth
    lengthSquared = segmentEast * segmentEast + segmentNorth * segmentNorth
    if lengthSquared == 0:
        return math.hypot(east - startEast, north - startNorth)
    share = ((east - startEast) * segmentEast + (north - startNorth) * segmentNorth) / lengthSquared
    share = max(0.0, min(1.0, share))
    closestEast = startEast + share * segmentEast
    closestNorth = startNorth + share * segmentNorth
    return math.hypot(east - closestEast, north - closestNorth)


def armCopter():
    vehicle = None
    try:
        vehicle = connectForPlan()
        if vehicle.armed:
            return {
                "scriptName": "flightPlan",
                "started": False,
                "armed": True,
                "outcome": "ALREADY ARMED",
                "stopReason": "The copter is already armed. It was not taken off.",
            }
        from controllers.flightController import ensureBoardCompassIsPrimary, chooseCompassesForThisFlight
        vehicle = ensureBoardCompassIsPrimary(
            vehicle,
            rebootPlanVehicle,
            print,
            confirmPlanParameter,
        )
        chooseCompassesForThisFlight(vehicle, print, confirmPlanParameter)
        waitUntilPlanPreflightPasses(vehicle)
        configurePlanFailsafeAndSlew(vehicle)
        blocker = preflightBlocker(vehicle)
        if blocker is not None:
            raise RuntimeError(f"Preflight became unready before arming ({blocker}).")

        from dronekit import VehicleMode
        setMode(vehicle, "GUIDED", VehicleMode)
        print("Arming motors.")
        vehicle.armed = True
        started = time.monotonic()
        while not vehicle.armed:
            if time.monotonic() - started > ARM_TIMEOUT_SECONDS:
                raise RuntimeError("Arming timed out. The copter did not take off.")
            if not heartbeatIsFresh(vehicle):
                raise RuntimeError("Pixhawk heartbeat was lost while arming.")
            time.sleep(STATUS_CHECK_INTERVAL_SECONDS)
        print("Motors armed. Waiting for YES before takeoff.")
        return {
            "scriptName": "flightPlan",
            "started": True,
            "armed": True,
            "outcome": "ARMED",
            "stopReason": None,
        }
    except Exception:
        if vehicle is not None:
            restoreAndClosePlanVehicle(vehicle)
        raise


def rebootPlanVehicle(vehicle):
    """Reboot after a compass-order change, then open the command cable again."""
    from pymavlink import mavutil as middleMan
    from controllers.flightController import COMPASS_PRIORITY_REBOOT_WAIT_SECONDS
    vehicle._master.mav.command_long_send(
        vehicle._master.target_system,
        vehicle._master.target_component,
        middleMan.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0,
        1, 0, 0, 0, 0, 0, 0,
    )
    vehicle.commandCableClosed = True
    try:
        vehicle.close()
    except Exception:
        pass
    clearVehicle()
    time.sleep(COMPASS_PRIORITY_REBOOT_WAIT_SECONDS)
    print("Pixhawk rebooted with the board compass as primary.")
    return connectForPlan()


def disarmAndCloseConnection():
    vehicle = flightPlanVehicle
    if vehicle is None:
        return {
            "scriptName": "flightPlan",
            "started": False,
            "armed": False,
            "outcome": "DISARMED",
            "stopReason": "The command cable is already closed.",
        }
    try:
        if vehicle.armed and heartbeatIsFresh(vehicle):
            print("Disarming motors on the ground.")
            vehicle.armed = False
            started = time.monotonic()
            while vehicle.armed and time.monotonic() - started < DISARM_TIMEOUT_SECONDS:
                if not heartbeatIsFresh(vehicle):
                    break
                time.sleep(STATUS_CHECK_INTERVAL_SECONDS)
        if vehicle.armed:
            raise RuntimeError("Motors did not disarm. Keep clear of the propellers.")
        return {
            "scriptName": "flightPlan",
            "started": False,
            "armed": False,
            "outcome": "DISARMED",
            "stopReason": "Motors disarmed and command cable closed.",
        }
    finally:
        if not getattr(vehicle, "armed", False):
            restoreAndClosePlanVehicle(vehicle)


def flySavedPlan(plan):
    vehicle = flightPlanVehicle
    landingWasStarted = False
    result = {
        "scriptName": "flightPlan",
        "started": True,
        "planId": plan.get("id"),
        "distanceFeet": plan.get("distanceFeet"),
        "bearingDegrees": plan.get("bearingDegrees"),
        "heightFeet": plan.get("heightFeet"),
        "speedMetersPerSecond": plan.get("speedMetersPerSecond"),
        "maximumAltitudeReachedText": "none",
        "outcome": "FAILED",
        "stopReason": None,
    }
    try:
        if vehicle is None or not heartbeatIsFresh(vehicle) or not vehicle.armed:
            raise RuntimeError("The copter is not armed. Press Arm before YES.")
        from dronekit import VehicleMode
        heightFeet = float(plan["heightFeet"])
        heightMeters = heightFeet / FEET_PER_METER
        ceilingMeters = (heightFeet + CEILING_MARGIN_FEET) / FEET_PER_METER
        speed = float(plan["speedMetersPerSecond"])
        distanceMeters = float(plan["distanceFeet"]) / FEET_PER_METER
        bearing = float(plan["bearingDegrees"])

        setMode(vehicle, "GUIDED", VehicleMode)
        print(f"Taking off to {heightFeet:.0f} feet.")
        vehicle.simple_takeoff(heightMeters)
        takeoffStarted = time.monotonic()
        while True:
            altitudeMeters = checkGuidedFlight(vehicle, ceilingMeters, result)
            if altitudeMeters is not None and altitudeMeters >= heightMeters * TAKEOFF_REACHED_FRACTION:
                break
            if time.monotonic() - takeoffStarted > TAKEOFF_TIMEOUT_SECONDS:
                raise RuntimeError("Takeoff timed out.")
            time.sleep(STATUS_CHECK_INTERVAL_SECONDS)

        settleStarted = time.monotonic()
        while time.monotonic() - settleStarted < TAKEOFF_SETTLE_SECONDS:
            checkGuidedFlight(vehicle, ceilingMeters, result)
            time.sleep(STATUS_CHECK_INTERVAL_SECONDS)

        start = vehicle.location.global_relative_frame
        if start is None or start.lat is None or start.lon is None:
            raise RuntimeError("GPS position was not available after takeoff.")
        startLatitude = float(start.lat)
        startLongitude = float(start.lon)
        northSpeed = speed * math.cos(math.radians(bearing))
        eastSpeed = speed * math.sin(math.radians(bearing))
        print(f"Flying {plan['distanceFeet']:.0f} feet on bearing {bearing:.0f} degrees.")
        flyStarted = time.monotonic()
        timeoutSeconds = (distanceMeters / speed) * 1.5 + 30.0
        while True:
            checkGuidedFlight(vehicle, ceilingMeters, result)
            northMeters, eastMeters = northAndEastMeters(
                vehicle, startLatitude, startLongitude
            )
            alongMeters = northMeters * math.cos(math.radians(bearing)) + eastMeters * math.sin(math.radians(bearing))
            crossMeters = -northMeters * math.sin(math.radians(bearing)) + eastMeters * math.cos(math.radians(bearing))
            if alongMeters >= distanceMeters:
                break
            if abs(crossMeters) > CROSS_TRACK_LIMIT_FEET / FEET_PER_METER or alongMeters < -(CROSS_TRACK_LIMIT_FEET / FEET_PER_METER):
                raise RuntimeError("The copter left the planned line.")
            if time.monotonic() - flyStarted > timeoutSeconds:
                raise RuntimeError("The planned distance was not reached in time.")
            sendVelocity(vehicle, northSpeed, eastSpeed, downSpeedForHeight(vehicle, heightMeters))
            time.sleep(STATUS_CHECK_INTERVAL_SECONDS)

        brakeStarted = time.monotonic()
        while time.monotonic() - brakeStarted < SETTLE_AFTER_MOVE_SECONDS:
            checkGuidedFlight(vehicle, ceilingMeters, result)
            sendVelocity(vehicle, 0.0, 0.0, downSpeedForHeight(vehicle, heightMeters))
            time.sleep(STATUS_CHECK_INTERVAL_SECONDS)
        sendVelocity(vehicle, 0.0, 0.0, 0.0)

        landingWasStarted = True
        landAndWaitForDisarm(vehicle, VehicleMode, result)
        if result["outcome"] != "ONBOARD FAILSAFE":
            result["outcome"] = "SUCCESS"
        return result
    except Exception as error:
        result["stopReason"] = str(error)
        return result
    finally:
        if vehicle is not None:
            if getattr(vehicle, "armed", False) and not landingWasStarted:
                try:
                    from dronekit import VehicleMode
                    landAndWaitForDisarm(vehicle, VehicleMode, result)
                except Exception as landError:
                    print(f"Emergency land did not confirm disarm: {landError}")
            restoreAndClosePlanVehicle(vehicle)


def landAndWaitForDisarm(vehicle, vehicleMode, result):
    if not heartbeatIsFresh(vehicle):
        vehicle.commandCableClosed = True
        vehicle.close()
        result["outcome"] = "ONBOARD FAILSAFE"
        return
    setMode(vehicle, "LAND", vehicleMode)
    landStarted = time.monotonic()
    while vehicle.armed:
        if not heartbeatIsFresh(vehicle):
            vehicle.commandCableClosed = True
            vehicle.close()
            result["outcome"] = "ONBOARD FAILSAFE"
            return
        if time.monotonic() - landStarted > LAND_TIMEOUT_SECONDS:
            raise RuntimeError("Timed out waiting for motors to disarm after landing.")
        time.sleep(PRINT_INTERVAL_SECONDS)


def connectForPlan():
    global flightPlanVehicle
    if flightPlanVehicle is not None:
        try:
            if heartbeatIsFresh(flightPlanVehicle):
                return flightPlanVehicle
        except Exception:
            flightPlanVehicle = None
    if not os.path.exists(PIXHAWK_PORT):
        raise RuntimeError(f"{PIXHAWK_PORT} is missing. Run the backend on the Raspberry Pi.")
    from dronekit import connect
    from pymavlink import mavutil as middleMan
    vehicle = connect(
        PIXHAWK_PORT,
        baud=PIXHAWK_BAUD,
        wait_ready=True,
        timeout=CONNECTION_TIMEOUT_SECONDS,
        source_system=GROUND_STATION_SYSTEM_ID,
    )
    if vehicle._master.target_component == 0:
        vehicle._master.target_component = 1
    attachPlanListenersAndStreams(vehicle, middleMan)
    flightPlanVehicle = vehicle
    return vehicle


def attachPlanListenersAndStreams(vehicle, middleMan):
    prearmHealthBit = getattr(middleMan.mavlink, "MAV_SYS_STATUS_PREARM_CHECK", 268435456)
    runPrearmCommandId = getattr(middleMan.mavlink, "MAV_CMD_RUN_PREARM_CHECKS", 401)
    latestTelemetry = {
        "compassVariance": None,
        "prearmHealthy": None,
        "lastPrearmReason": None,
        "lastPrearmPollTime": 0.0,
        "runPrearmCommandId": runPrearmCommandId,
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
        if text and "prearm" in text.lower():
            latestTelemetry["lastPrearmReason"] = text
        if text and "battery 1 low voltage" in text.lower():
            latestTelemetry["sawFalseBatteryWarning"] = True

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
    vehicle.planTelemetry = latestTelemetry


def waitUntilPlanPreflightPasses(vehicle):
    from controllers.flightController import pixhawkBatteryVoltageText, releaseFalseBatteryFailsafe
    print(pixhawkBatteryVoltageText(vehicle))
    started = time.monotonic()
    while True:
        releaseFalseBatteryFailsafe(
            vehicle,
            vehicle.planTelemetry,
            print,
            confirmPlanParameter,
        )
        from controllers.flightController import preferPrimaryCompassWhenTheyDisagree
        preferPrimaryCompassWhenTheyDisagree(vehicle, vehicle.planTelemetry, print, confirmPlanParameter)
        blocker = preflightBlocker(vehicle)
        if blocker is None:
            return
        if time.monotonic() - started > PREFLIGHT_WAIT_TIMEOUT_SECONDS:
            raise RuntimeError(f"Preflight not ready: {blocker}")
        time.sleep(PRINT_INTERVAL_SECONDS)


def preflightBlocker(vehicle):
    if not heartbeatIsFresh(vehicle):
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

    latestTelemetry = getattr(vehicle, "planTelemetry", {})
    compassVariance = latestTelemetry.get("compassVariance")
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

    if not vehicle.is_armable or latestTelemetry.get("prearmHealthy") is not True:
        nowPoll = time.monotonic()
        if nowPoll - latestTelemetry.get("lastPrearmPollTime", 0.0) >= PREARM_POLL_INTERVAL_SECONDS:
            prearmCheckMessage = vehicle.message_factory.command_long_encode(
                vehicle._master.target_system,
                vehicle._master.target_component,
                latestTelemetry.get("runPrearmCommandId", 401),
                0,
                0, 0, 0, 0, 0, 0, 0,
            )
            vehicle.send_mavlink(prearmCheckMessage)
            vehicle.flush()
            latestTelemetry["lastPrearmPollTime"] = nowPoll
        reasonText = latestTelemetry.get("lastPrearmReason") or "waiting for Pixhawk PreArm status..."
        return f"Pixhawk pre-arm check blocked ({reasonText})."

    if vehicle.attitude is None or vehicle.attitude.roll is None or vehicle.attitude.pitch is None:
        return "Attitude is not available yet."
    rollDegrees = math.degrees(vehicle.attitude.roll)
    pitchDegrees = math.degrees(vehicle.attitude.pitch)
    if abs(rollDegrees) > MAXIMUM_GROUND_LEAN_DEGREES or abs(pitchDegrees) > MAXIMUM_GROUND_LEAN_DEGREES:
        return f"Copter is not sitting level. Roll {rollDegrees:+.1f} deg, pitch {pitchDegrees:+.1f} deg."
    return None


def configurePlanFailsafeAndSlew(vehicle):
    savedSlew = vehicle.parameters[TAKEOFF_SLEW_PARAMETER_NAME]
    if savedSlew is not None:
        savedSlew = float(savedSlew)
        takeoffSlew = min(savedSlew, PREFERRED_TAKEOFF_SLEW_SECONDS)
        if abs(takeoffSlew - savedSlew) >= PARAMETER_TOLERANCE:
            confirmPlanParameter(vehicle, TAKEOFF_SLEW_PARAMETER_NAME, takeoffSlew)
            vehicle.originalTakeoffSlewSeconds = savedSlew

    savedEnable = float(vehicle.parameters[GROUND_STATION_FAILSAFE_PARAMETER_NAME])
    savedTimeout = float(vehicle.parameters[GROUND_STATION_FAILSAFE_TIMEOUT_PARAMETER_NAME])
    savedOptions = float(vehicle.parameters[FAILSAFE_OPTIONS_PARAMETER_NAME])
    timeoutSeconds = savedTimeout if (0 < savedTimeout <= PREFERRED_FAILSAFE_TIMEOUT_SECONDS) else PREFERRED_FAILSAFE_TIMEOUT_SECONDS
    flightOptions = int(round(savedOptions)) & ~CONTINUE_PILOT_MODE_ON_FAILSAFE_BIT
    vehicle.savedFailsafe = (savedEnable, savedTimeout, savedOptions)
    confirmPlanParameter(vehicle, GROUND_STATION_FAILSAFE_PARAMETER_NAME, GROUND_STATION_FAILSAFE_LAND_VALUE)
    confirmPlanParameter(vehicle, GROUND_STATION_FAILSAFE_TIMEOUT_PARAMETER_NAME, timeoutSeconds)
    confirmPlanParameter(vehicle, FAILSAFE_OPTIONS_PARAMETER_NAME, flightOptions)


def confirmPlanParameter(vehicle, parameterName, wantedValue):
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


def restoreAndClosePlanVehicle(vehicle):
    try:
        if (
            not getattr(vehicle, "commandCableClosed", False)
            and heartbeatIsFresh(vehicle)
            and not getattr(vehicle, "armed", False)
        ):
            savedFailsafe = getattr(vehicle, "savedFailsafe", None)
            if savedFailsafe is not None:
                confirmPlanParameter(vehicle, GROUND_STATION_FAILSAFE_PARAMETER_NAME, savedFailsafe[0])
                confirmPlanParameter(vehicle, GROUND_STATION_FAILSAFE_TIMEOUT_PARAMETER_NAME, savedFailsafe[1])
                confirmPlanParameter(vehicle, FAILSAFE_OPTIONS_PARAMETER_NAME, savedFailsafe[2])
            originalSlew = getattr(vehicle, "originalTakeoffSlewSeconds", None)
            if originalSlew is not None:
                confirmPlanParameter(vehicle, TAKEOFF_SLEW_PARAMETER_NAME, originalSlew)
    except Exception as restoreError:
        print(f"Could not restore plan failsafe settings: {restoreError}")
    try:
        if not getattr(vehicle, "commandCableClosed", False):
            vehicle.commandCableClosed = True
            vehicle.close()
    except Exception:
        pass
    clearVehicle()


def clearVehicle():
    global flightPlanVehicle
    flightPlanVehicle = None


def setMode(vehicle, modeName, vehicleMode):
    vehicle.mode = vehicleMode(modeName)
    started = time.monotonic()
    while time.monotonic() - started < MODE_CHANGE_TIMEOUT_SECONDS:
        if vehicle.mode.name == modeName:
            return
        if not heartbeatIsFresh(vehicle):
            raise RuntimeError("Pixhawk heartbeat was lost.")
        time.sleep(STATUS_CHECK_INTERVAL_SECONDS)
    raise RuntimeError(f"Could not enter {modeName}.")


def checkGuidedFlight(vehicle, ceilingMeters, result):
    if vehicle.mode.name != "GUIDED":
        raise RuntimeError("GUIDED mode was lost.")
    if not heartbeatIsFresh(vehicle):
        raise RuntimeError("Pixhawk heartbeat was lost.")
    if not vehicle.armed:
        raise RuntimeError("The copter disarmed during the flight.")
    if not vehicle.ekf_ok:
        raise RuntimeError("Position estimate was lost during the flight.")
    altitudeMeters = relativeAltitudeMeters(vehicle)
    if altitudeMeters is not None:
        previousMax = getattr(vehicle, "maximumAltitudeMeters", None)
        if previousMax is None or altitudeMeters > previousMax:
            vehicle.maximumAltitudeMeters = altitudeMeters
            result["maximumAltitudeReachedText"] = f"{altitudeMeters * FEET_PER_METER:.2f} ft"
        if altitudeMeters >= ceilingMeters:
            raise RuntimeError("The safety ceiling was reached.")
    return altitudeMeters


def relativeAltitudeMeters(vehicle):
    location = vehicle.location.global_relative_frame
    if location is None or location.alt is None or not math.isfinite(float(location.alt)):
        return None
    return float(location.alt)


def downSpeedForHeight(vehicle, heightMeters):
    altitudeMeters = relativeAltitudeMeters(vehicle)
    if altitudeMeters is None:
        return 0.0
    climb = max(
        -MAXIMUM_VERTICAL_CORRECTION_METERS_PER_SECOND,
        min(
            MAXIMUM_VERTICAL_CORRECTION_METERS_PER_SECOND,
            (heightMeters - altitudeMeters) * ALTITUDE_HOLD_GAIN_DURING_MOVE,
        ),
    )
    return -climb


def northAndEastMeters(vehicle, startLatitude, startLongitude):
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


def sendVelocity(vehicle, northSpeed, eastSpeed, downSpeed):
    from pymavlink import mavutil as middleMan
    typeMask = (
        middleMan.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
        | middleMan.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
        | middleMan.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
        | middleMan.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | middleMan.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | middleMan.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | middleMan.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        | middleMan.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )
    message = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        vehicle._master.target_system,
        vehicle._master.target_component,
        middleMan.mavlink.MAV_FRAME_LOCAL_NED,
        typeMask,
        0, 0, 0,
        northSpeed,
        eastSpeed,
        downSpeed,
        0, 0, 0,
        0, 0,
    )
    vehicle.send_mavlink(message)
    vehicle.flush()


def heartbeatIsFresh(vehicle):
    age = vehicle.last_heartbeat
    return age is not None and age <= HEARTBEAT_STALE_AFTER_SECONDS


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
        from controllers.motorTest import motorTestIsRunning
        if motorTestIsRunning:
            return "A motor test is already using the command cable."
    except Exception:
        pass
    return None


def haversineFeet(lat1, lon1, lat2, lon2):
    earthRadiusMeters = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dPhi = math.radians(lat2 - lat1)
    dLambda = math.radians(lon2 - lon1)
    a = math.sin(dPhi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dLambda / 2) ** 2
    meters = 2 * earthRadiusMeters * math.asin(math.sqrt(a))
    return meters * FEET_PER_METER


def bearingDegreesClockwise(lat1, lon1, lat2, lon2):
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dLambda = math.radians(lon2 - lon1)
    y = math.sin(dLambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dLambda)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def failedPlan(reason):
    return {
        "scriptName": "flightPlan",
        "started": False,
        "armed": False,
        "outcome": "FAILED",
        "stopReason": reason,
    }
