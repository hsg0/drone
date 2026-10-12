import threading

from fastapi import APIRouter
from fastapi import Body
from fastapi import HTTPException

from controllers.capturePicVidController import seenGesture
from controllers.flight import allowGestureFlight
from controllers.flight import closedCableReason
from controllers.flight import flyGesture
from controllers.flight import gestureFlightStatus
from controllers.flight import requestStopGesture
from controllers.flight import stopGestureFlight


flightRoutes = APIRouter()
flightLock = threading.Lock()
lastFlightResult = None
cachedFlightStatus = {
    "allowed": False,
    "connected": False,
    "armed": False,
    "airborne": False,
    "heightMeters": None,
    "lastGesture": None,
}

# The same names gestureFromArms returns. The website sends one of these.
GESTURE_NAMES = (
    "takeoff-land",
    "fly-up",
    "fly-down",
    "move-away",
    "move-closer",
    "hover",
    "not-seen",
)


def statusPayload(busy):
    """The last known copter report, plus whether a command is still running."""
    payload = dict(cachedFlightStatus)
    payload["busy"] = busy
    payload["lastResult"] = lastFlightResult
    return payload


def readStatusWhileHoldingLock():
    """Read the Pixhawk. The caller must already hold flightLock."""
    global cachedFlightStatus
    cachedFlightStatus = gestureFlightStatus()
    return statusPayload(False)


@flightRoutes.get("/flight/seen")
def readSeenGesture():
    """The pose in the live camera. This does not arm or move the copter."""
    return seenGesture()


@flightRoutes.get("/flight/status")
def readFlightStatus():
    """Report the link. This does not arm or move the copter."""
    if not flightLock.acquire(blocking=False):
        return statusPayload(True)
    try:
        return readStatusWhileHoldingLock()
    finally:
        flightLock.release()


@flightRoutes.post("/flight/allow")
def allowFlight():
    """Open the command cable and permit gesture commands. This does not arm."""
    global lastFlightResult
    with flightLock:
        opened = allowGestureFlight()
        if not opened:
            reason = closedCableReason()
            lastFlightResult = {
                "gesture": None,
                "accepted": False,
                "action": reason,
            }
            raise HTTPException(status_code=503, detail=reason)
        lastFlightResult = {
            "gesture": None,
            "accepted": True,
            "action": "Gesture flight is allowed. The copter is not armed.",
        }
        return readStatusWhileHoldingLock()


@flightRoutes.post("/flight/stop")
def stopFlight():
    """LAND if the motors are armed, then close the command cable."""
    global lastFlightResult
    # Set this before waiting, so a takeoff that already holds the lock can land.
    requestStopGesture()
    with flightLock:
        stopGestureFlight()
        lastFlightResult = {
            "gesture": None,
            "accepted": True,
            "action": "Stop closed the command cable.",
        }
        return readStatusWhileHoldingLock()


@flightRoutes.post("/flight/gesture")
def sendFlightGesture(body: dict = Body(...)):
    """Run one gesture command. Takeoff and land wait until they finish."""
    global lastFlightResult
    gestureName = body.get("gesture")
    if gestureName not in GESTURE_NAMES:
        raise HTTPException(status_code=404, detail="Unknown gesture")
    with flightLock:
        result = flyGesture(gestureName)
        lastFlightResult = result
        status = readStatusWhileHoldingLock()
        status["lastResult"] = result
        return status
