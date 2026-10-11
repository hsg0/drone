#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    Mark the Sullivan Park corners the copter is allowed to fly inside.

    Walk the corners in order. Type MARK at each one. Type DONE when the
    corners are finished, then walk to the middle and type RETURN.
    The points are saved next to this file.

    A later flight script imports the saved shape:

        import sys
        sys.path.insert(0, "/home/drone/Desktop/scripts/oct8-2026")
        import sullivanpark

        perimeter = sullivanpark.sullivanParkPremiter()

WHY:
    The park perimeter has to be marked on the ground with a real GPS
    position. The flight script should read those saved corners instead of
    having the latitude and longitude typed in by hand.

HOW:
    1. Connect on pins 8 and 10. Leave USB with the screen.
    2. Wait for a 3D fix, at least 6 satellites, and a healthy position.
    3. MARK saves the current spot as the next corner.
    4. UNDO removes the last corner. DONE closes the corner list.
    5. RETURN saves one spot inside the corners.
    6. Write the perimeter only after the return spot is inside the shape.

IMPORTANT:
    This script does not arm, fly, or change the Pixhawk fence.
    Carry the copter. Stand still at each corner until MARK is accepted.
    Two satellites are not enough. A weak position is not saved.
    Walk the corners in one direction around the grass. Do not criss-cross.
    At least 3 corners are required. The return spot must be inside them.
    The copter must stay inside this shape on a later flight. Mark the line
    a few meters in from trees, paths, and people.
"""

import json
import math
import os
import select
import sys
import time

from pymavlink import mavutil


# Pin 8 transmits. Pin 10 receives. Pin 6 is ground.
PIXHAWK_SERIAL_PORT = "/dev/serial0"
PIXHAWK_SERIAL_BAUD = 57600

CONNECTION_TIMEOUT_SECONDS = 15
HEARTBEAT_STALE_AFTER_SECONDS = 3.0
SENSOR_MESSAGE_STALE_AFTER_SECONDS = 1.0
MESSAGE_INTERVAL_MICROSECONDS = 100_000
STATUS_PRINT_INTERVAL_SECONDS = 1.0

GPS_POSITION_HOLD_MINIMUM_FIX_TYPE = 3
GPS_POSITION_HOLD_MINIMUM_SATELLITES = 6
UNKNOWN_SATELLITE_COUNT = 255

EKF_HORIZONTAL_POSITION_ABSOLUTE_BIT = 16
EKF_CONSTANT_POSITION_MODE_BIT = 128
EKF_GPS_GLITCHING_BIT = 32768

EARTH_RADIUS_METERS = 6371000.0
MINIMUM_CORNER_COUNT = 3
# Standing in the same spot twice must not create a tiny false edge.
MINIMUM_CORNER_SEPARATION_METERS = 2.0

PARK_NAME = "Sullivan Park"
PERIMETER_VERSION = 1
SAVED_PERIMETER_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "sullivan-park-perimeter.json",
)

REQUIRED_MESSAGE_NAMES = (
    "HEARTBEAT",
    "GPS_RAW_INT",
    "GLOBAL_POSITION_INT",
    "EKF_STATUS_REPORT",
    "STATUSTEXT",
)

MARK_COMMAND = "MARK"
UNDO_COMMAND = "UNDO"
DONE_COMMAND = "DONE"
RETURN_COMMAND = "RETURN"

lastHeartbeatReceivedAtSeconds = None
lastPerimeterRefusalReason = None


def cleanStatusText(message):
    statusText = getattr(message, "text", "")
    if isinstance(statusText, bytes):
        statusText = statusText.decode("ascii", "ignore")
    return str(statusText).strip("\x00").strip()


def rememberHeartbeat(pixhawkConnection, message):
    global lastHeartbeatReceivedAtSeconds
    if message.get_srcSystem() != pixhawkConnection.target_system:
        return
    lastHeartbeatReceivedAtSeconds = time.monotonic()


def heartbeatIsFresh():
    if lastHeartbeatReceivedAtSeconds is None:
        return False
    heartbeatAgeSeconds = time.monotonic() - lastHeartbeatReceivedAtSeconds
    return heartbeatAgeSeconds <= HEARTBEAT_STALE_AFTER_SECONDS


def connectToPixhawk():
    print()
    print(f"Opening {PIXHAWK_SERIAL_PORT} at {PIXHAWK_SERIAL_BAUD} baud.")
    pixhawkConnection = mavutil.mavlink_connection(
        PIXHAWK_SERIAL_PORT,
        baud=PIXHAWK_SERIAL_BAUD,
        source_system=255,
        autoreconnect=False,
    )
    heartbeat = pixhawkConnection.wait_heartbeat(timeout=CONNECTION_TIMEOUT_SECONDS)
    if heartbeat is None:
        raise RuntimeError("No Pixhawk heartbeat arrived on pin 10.")
    if pixhawkConnection.target_component == 0:
        pixhawkConnection.target_component = 1
    rememberHeartbeat(pixhawkConnection, heartbeat)
    print(
        f"Connected to Pixhawk system {pixhawkConnection.target_system}, "
        f"component {pixhawkConnection.target_component}."
    )
    return pixhawkConnection


def requestRequiredTelemetry(pixhawkConnection):
    for messageName in REQUIRED_MESSAGE_NAMES:
        messageId = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{messageName}", None)
        if messageId is None:
            print(f"MAVLink message id for {messageName} was not found.")
            continue
        pixhawkConnection.mav.command_long_send(
            pixhawkConnection.target_system,
            pixhawkConnection.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            messageId,
            MESSAGE_INTERVAL_MICROSECONDS,
            0,
            0,
            0,
            0,
            0,
        )
    print("Requested GPS telemetry.")


def processIncomingMessage(pixhawkConnection, latestMessages, messageArrivedAtSeconds, message):
    if message is None:
        return
    if message.get_srcSystem() not in (0, pixhawkConnection.target_system):
        return
    messageType = message.get_type()
    latestMessages[messageType] = message
    messageArrivedAtSeconds[messageType] = time.monotonic()
    if messageType == "HEARTBEAT":
        rememberHeartbeat(pixhawkConnection, message)
    elif messageType == "STATUSTEXT":
        statusText = cleanStatusText(message)
        if statusText:
            print(f"PIXHAWK: {statusText}")


def readAvailableMessages(pixhawkConnection, latestMessages, messageArrivedAtSeconds):
    while True:
        message = pixhawkConnection.recv_match(blocking=False)
        if message is None:
            return
        processIncomingMessage(
            pixhawkConnection,
            latestMessages,
            messageArrivedAtSeconds,
            message,
        )


def messageIsFresh(messageArrivedAtSeconds, messageType):
    arrivedAtSeconds = messageArrivedAtSeconds.get(messageType)
    if arrivedAtSeconds is None:
        return False
    messageAgeSeconds = time.monotonic() - arrivedAtSeconds
    return messageAgeSeconds <= SENSOR_MESSAGE_STALE_AFTER_SECONDS


def gpsSatelliteCount(latestMessages, messageArrivedAtSeconds):
    if not messageIsFresh(messageArrivedAtSeconds, "GPS_RAW_INT"):
        return None
    gpsReading = latestMessages.get("GPS_RAW_INT")
    if gpsReading is None:
        return None
    satelliteCount = getattr(gpsReading, "satellites_visible", None)
    if satelliteCount is None or satelliteCount == UNKNOWN_SATELLITE_COUNT:
        return None
    return int(satelliteCount)


def ekfHorizontalPositionIsHealthy(latestMessages, messageArrivedAtSeconds):
    if not messageIsFresh(messageArrivedAtSeconds, "EKF_STATUS_REPORT"):
        return False
    ekfStatus = latestMessages.get("EKF_STATUS_REPORT")
    if ekfStatus is None:
        return False
    ekfFlags = int(ekfStatus.flags)
    hasAbsoluteHorizontalPosition = bool(ekfFlags & EKF_HORIZONTAL_POSITION_ABSOLUTE_BIT)
    constantPositionModeIsActive = bool(ekfFlags & EKF_CONSTANT_POSITION_MODE_BIT)
    gpsGlitchIsActive = bool(ekfFlags & EKF_GPS_GLITCHING_BIT)
    return (
        hasAbsoluteHorizontalPosition
        and not constantPositionModeIsActive
        and not gpsGlitchIsActive
    )


def gpsIsUsableForPositionHold(latestMessages, messageArrivedAtSeconds):
    """A saved corner needs a 3D fix, 6 satellites, and a healthy position."""
    if not messageIsFresh(messageArrivedAtSeconds, "GPS_RAW_INT"):
        return False
    gpsReading = latestMessages.get("GPS_RAW_INT")
    if gpsReading is None or gpsReading.fix_type is None:
        return False
    if int(gpsReading.fix_type) < GPS_POSITION_HOLD_MINIMUM_FIX_TYPE:
        return False
    satelliteCount = gpsSatelliteCount(latestMessages, messageArrivedAtSeconds)
    if satelliteCount is None or satelliteCount < GPS_POSITION_HOLD_MINIMUM_SATELLITES:
        return False
    return ekfHorizontalPositionIsHealthy(latestMessages, messageArrivedAtSeconds)


def currentPositionDegrees(latestMessages, messageArrivedAtSeconds):
    """Return the fused latitude and longitude, or None when it is not usable."""
    if not gpsIsUsableForPositionHold(latestMessages, messageArrivedAtSeconds):
        return None
    if not messageIsFresh(messageArrivedAtSeconds, "GLOBAL_POSITION_INT"):
        return None
    positionMessage = latestMessages.get("GLOBAL_POSITION_INT")
    if positionMessage is None:
        return None
    latitudeDegrees = float(positionMessage.lat) / 1.0e7
    longitudeDegrees = float(positionMessage.lon) / 1.0e7
    if abs(latitudeDegrees) < 0.000001 and abs(longitudeDegrees) < 0.000001:
        return None
    if not (-90.0 <= latitudeDegrees <= 90.0):
        return None
    if not (-180.0 <= longitudeDegrees <= 180.0):
        return None
    return latitudeDegrees, longitudeDegrees


def typedLineOrNone():
    readyToRead, _unused, _alsoUnused = select.select([sys.stdin], [], [], 0)
    if not readyToRead:
        return None
    return sys.stdin.readline()


def distanceMeters(latitudeDegrees, longitudeDegrees, otherLatitudeDegrees, otherLongitudeDegrees):
    latitudeRadians = math.radians(latitudeDegrees)
    otherLatitudeRadians = math.radians(otherLatitudeDegrees)
    latitudeDifferenceRadians = math.radians(otherLatitudeDegrees - latitudeDegrees)
    longitudeDifferenceRadians = math.radians(otherLongitudeDegrees - longitudeDegrees)
    haversine = (
        math.sin(latitudeDifferenceRadians / 2.0) ** 2
        + math.cos(latitudeRadians)
        * math.cos(otherLatitudeRadians)
        * math.sin(longitudeDifferenceRadians / 2.0) ** 2
    )
    return 2.0 * EARTH_RADIUS_METERS * math.asin(min(1.0, math.sqrt(haversine)))


def pointIsInsideCorners(latitudeDegrees, longitudeDegrees, corners):
    """Ray cast in latitude and longitude. Sullivan Park is small enough for this."""
    inside = False
    previousCorner = corners[-1]
    for corner in corners:
        crossesLatitude = (
            (corner["latitudeDegrees"] > latitudeDegrees)
            != (previousCorner["latitudeDegrees"] > latitudeDegrees)
        )
        if crossesLatitude:
            longitudeAtCrossing = (
                (previousCorner["longitudeDegrees"] - corner["longitudeDegrees"])
                * (latitudeDegrees - corner["latitudeDegrees"])
                / (previousCorner["latitudeDegrees"] - corner["latitudeDegrees"])
                + corner["longitudeDegrees"]
            )
            if longitudeDegrees < longitudeAtCrossing:
                inside = not inside
        previousCorner = corner
    return inside


def positionRecord(latitudeDegrees, longitudeDegrees, latestMessages, messageArrivedAtSeconds):
    gpsReading = latestMessages.get("GPS_RAW_INT")
    fixType = None
    if gpsReading is not None and gpsReading.fix_type is not None:
        fixType = int(gpsReading.fix_type)
    return {
        "latitudeDegrees": latitudeDegrees,
        "longitudeDegrees": longitudeDegrees,
        "fixType": fixType,
        "satellites": gpsSatelliteCount(latestMessages, messageArrivedAtSeconds),
        "markedAt": time.strftime("%Y-%m-%d %H:%M:%S"),
    }


def formatPosition(position):
    return (
        f"{position['latitudeDegrees']:.7f}, "
        f"{position['longitudeDegrees']:.7f}"
    )


def printCornerList(corners):
    if not corners:
        print("No corners yet.")
        return
    print()
    for cornerNumber, corner in enumerate(corners, start=1):
        if cornerNumber == 1:
            separationText = "first corner"
        else:
            separationMeters = distanceMeters(
                corners[cornerNumber - 2]["latitudeDegrees"],
                corners[cornerNumber - 2]["longitudeDegrees"],
                corner["latitudeDegrees"],
                corner["longitudeDegrees"],
            )
            separationText = f"{separationMeters:.1f} m from the previous corner"
        print(f"Corner {cornerNumber}: {formatPosition(corner)} | {separationText}")
    if len(corners) >= 2:
        closingMeters = distanceMeters(
            corners[-1]["latitudeDegrees"],
            corners[-1]["longitudeDegrees"],
            corners[0]["latitudeDegrees"],
            corners[0]["longitudeDegrees"],
        )
        print(f"Closing edge, last corner back to the first: {closingMeters:.1f} m")


def nearestCornerSeparationMeters(latitudeDegrees, longitudeDegrees, corners):
    if not corners:
        return None
    separationsMeters = [
        distanceMeters(
            latitudeDegrees,
            longitudeDegrees,
            corner["latitudeDegrees"],
            corner["longitudeDegrees"],
        )
        for corner in corners
    ]
    return min(separationsMeters)


def rememberRefusal(reason):
    global lastPerimeterRefusalReason
    lastPerimeterRefusalReason = reason
    return None


def sullivanParkPremiter():
    """
    Load the saved Sullivan Park perimeter.

    Returns a dictionary with "corners" and "returnPoint", or None when the
    saved file is missing or incomplete. corners are in the order they were
    walked. The return point is inside those corners.
    """
    try:
        with open(SAVED_PERIMETER_PATH, "r", encoding="utf-8") as perimeterFile:
            savedPerimeter = json.load(perimeterFile)
    except FileNotFoundError:
        return rememberRefusal(f"No saved perimeter at {SAVED_PERIMETER_PATH}.")
    except (OSError, json.JSONDecodeError):
        return rememberRefusal(f"The perimeter at {SAVED_PERIMETER_PATH} could not be read.")

    if not isinstance(savedPerimeter, dict):
        return rememberRefusal("The saved perimeter was not a set of named values.")
    if savedPerimeter.get("perimeterVersion") != PERIMETER_VERSION:
        return rememberRefusal("The saved perimeter is from an older layout.")
    if savedPerimeter.get("parkName") != PARK_NAME:
        return rememberRefusal("The saved perimeter is for a different park.")

    corners = savedPerimeter.get("corners")
    returnPoint = savedPerimeter.get("returnPoint")
    if not isinstance(corners, list) or len(corners) < MINIMUM_CORNER_COUNT:
        return rememberRefusal("The saved perimeter needs at least 3 corners.")
    if not isinstance(returnPoint, dict):
        return rememberRefusal("The saved perimeter has no return point.")

    for spot in list(corners) + [returnPoint]:
        latitudeDegrees = spot.get("latitudeDegrees")
        longitudeDegrees = spot.get("longitudeDegrees")
        if not isinstance(latitudeDegrees, (int, float)) or not isinstance(longitudeDegrees, (int, float)):
            return rememberRefusal("A saved spot has no latitude or longitude.")

    if not pointIsInsideCorners(
        returnPoint["latitudeDegrees"],
        returnPoint["longitudeDegrees"],
        corners,
    ):
        return rememberRefusal("The saved return point is outside the corners.")

    rememberRefusal("")
    return savedPerimeter


def savePerimeter(corners, returnPoint):
    savedPerimeter = {
        "perimeterVersion": PERIMETER_VERSION,
        "parkName": PARK_NAME,
        "corners": corners,
        "returnPoint": returnPoint,
    }
    temporaryPath = SAVED_PERIMETER_PATH + ".tmp"
    with open(temporaryPath, "w", encoding="utf-8") as perimeterFile:
        json.dump(savedPerimeter, perimeterFile, indent=2)
        perimeterFile.write("\n")
    os.replace(temporaryPath, SAVED_PERIMETER_PATH)
    print()
    print(f"Saved {len(corners)} corners to {SAVED_PERIMETER_PATH}")


def waitUntilPositionIsUsable(pixhawkConnection, latestMessages, messageArrivedAtSeconds):
    print()
    print("Waiting for a position that is safe to mark.")
    print("Need a 3D fix, at least 6 satellites, and a healthy position.")
    lastPrintedAtSeconds = 0.0
    while True:
        readAvailableMessages(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
        if not heartbeatIsFresh():
            raise RuntimeError("Pixhawk heartbeat was lost.")
        currentTimeSeconds = time.monotonic()
        if currentPositionDegrees(latestMessages, messageArrivedAtSeconds) is not None:
            latitudeDegrees, longitudeDegrees = currentPositionDegrees(
                latestMessages,
                messageArrivedAtSeconds,
            )
            satelliteCount = gpsSatelliteCount(latestMessages, messageArrivedAtSeconds)
            print(
                f"Position ready. {latitudeDegrees:.7f}, {longitudeDegrees:.7f} | "
                f"{satelliteCount} satellites."
            )
            return
        if currentTimeSeconds - lastPrintedAtSeconds >= STATUS_PRINT_INTERVAL_SECONDS:
            satelliteCount = gpsSatelliteCount(latestMessages, messageArrivedAtSeconds)
            print(f"Position not ready. Satellites {satelliteCount}.")
            lastPrintedAtSeconds = currentTimeSeconds
        time.sleep(0.1)


def markCorners(pixhawkConnection, latestMessages, messageArrivedAtSeconds):
    corners = []
    print()
    print("Walk the corners in order around the grass.")
    print(f"Stand still, then type {MARK_COMMAND}.")
    print(f"{UNDO_COMMAND} removes the last corner. {DONE_COMMAND} finishes the corners.")
    print()

    lastPrintedAtSeconds = 0.0
    while True:
        readAvailableMessages(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
        if not heartbeatIsFresh():
            raise RuntimeError("Pixhawk heartbeat was lost.")

        currentTimeSeconds = time.monotonic()
        if currentTimeSeconds - lastPrintedAtSeconds >= STATUS_PRINT_INTERVAL_SECONDS:
            position = currentPositionDegrees(latestMessages, messageArrivedAtSeconds)
            satelliteCount = gpsSatelliteCount(latestMessages, messageArrivedAtSeconds)
            if position is None:
                print(f"Waiting. Corners {len(corners)}. Satellites {satelliteCount}.")
            else:
                print(
                    f"Here: {position[0]:.7f}, {position[1]:.7f} | "
                    f"corners {len(corners)} | satellites {satelliteCount}"
                )
            lastPrintedAtSeconds = currentTimeSeconds

        typedLine = typedLineOrNone()
        if typedLine is None:
            time.sleep(0.1)
            continue
        command = typedLine.strip().upper()
        if command == "":
            continue

        if command == UNDO_COMMAND:
            if not corners:
                print("There is no corner to remove.")
                continue
            removedCorner = corners.pop()
            print(f"Removed {formatPosition(removedCorner)}")
            printCornerList(corners)
            continue

        if command == DONE_COMMAND:
            if len(corners) < MINIMUM_CORNER_COUNT:
                print(f"Mark at least {MINIMUM_CORNER_COUNT} corners before {DONE_COMMAND}.")
                continue
            printCornerList(corners)
            return corners

        if command != MARK_COMMAND:
            print(f"Type {MARK_COMMAND}, {UNDO_COMMAND}, or {DONE_COMMAND}.")
            continue

        position = currentPositionDegrees(latestMessages, messageArrivedAtSeconds)
        if position is None:
            print("This spot was not marked. The position is not ready.")
            continue
        latitudeDegrees, longitudeDegrees = position
        separationMeters = nearestCornerSeparationMeters(latitudeDegrees, longitudeDegrees, corners)
        if separationMeters is not None and separationMeters < MINIMUM_CORNER_SEPARATION_METERS:
            print(
                f"This spot is only {separationMeters:.1f} m from a corner already marked. "
                "Walk to the next corner."
            )
            continue
        corners.append(
            positionRecord(
                latitudeDegrees,
                longitudeDegrees,
                latestMessages,
                messageArrivedAtSeconds,
            )
        )
        print(f"Corner {len(corners)} marked. {formatPosition(corners[-1])}")
        printCornerList(corners)


def markReturnPoint(pixhawkConnection, latestMessages, messageArrivedAtSeconds, corners):
    print()
    print("Walk to the middle, inside the corners.")
    print(f"Stand still, then type {RETURN_COMMAND}.")
    print()

    lastPrintedAtSeconds = 0.0
    while True:
        readAvailableMessages(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
        if not heartbeatIsFresh():
            raise RuntimeError("Pixhawk heartbeat was lost.")

        currentTimeSeconds = time.monotonic()
        if currentTimeSeconds - lastPrintedAtSeconds >= STATUS_PRINT_INTERVAL_SECONDS:
            position = currentPositionDegrees(latestMessages, messageArrivedAtSeconds)
            if position is None:
                print("Waiting for a position in the middle.")
            else:
                insideText = "inside" if pointIsInsideCorners(position[0], position[1], corners) else "outside"
                print(f"Here: {position[0]:.7f}, {position[1]:.7f} | {insideText} the corners")
            lastPrintedAtSeconds = currentTimeSeconds

        typedLine = typedLineOrNone()
        if typedLine is None:
            time.sleep(0.1)
            continue
        command = typedLine.strip().upper()
        if command == "":
            continue
        if command != RETURN_COMMAND:
            print(f"Type {RETURN_COMMAND} when you are standing in the middle.")
            continue

        position = currentPositionDegrees(latestMessages, messageArrivedAtSeconds)
        if position is None:
            print("The return spot was not marked. The position is not ready.")
            continue
        latitudeDegrees, longitudeDegrees = position
        if not pointIsInsideCorners(latitudeDegrees, longitudeDegrees, corners):
            print("That spot is outside the corners. Walk farther inside and type RETURN again.")
            continue
        returnPoint = positionRecord(
            latitudeDegrees,
            longitudeDegrees,
            latestMessages,
            messageArrivedAtSeconds,
        )
        print(f"Return point marked. {formatPosition(returnPoint)}")
        return returnPoint


def main():
    pixhawkConnection = None
    latestMessages = {}
    messageArrivedAtSeconds = {}

    try:
        print()
        print("==============================================")
        print("SULLIVAN PARK PERIMETER")
        print("==============================================")
        print()
        print("Mark the corners the copter must stay inside.")
        print("This script does not fly.")
        print()

        existingPerimeter = sullivanParkPremiter()
        if existingPerimeter is not None:
            print(f"A perimeter is already saved with {len(existingPerimeter['corners'])} corners.")
            print("Marking a new one replaces it only after RETURN is accepted.")
        else:
            print(lastPerimeterRefusalReason)

        pixhawkConnection = connectToPixhawk()
        requestRequiredTelemetry(pixhawkConnection)
        waitUntilPositionIsUsable(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
        corners = markCorners(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
        returnPoint = markReturnPoint(
            pixhawkConnection,
            latestMessages,
            messageArrivedAtSeconds,
            corners,
        )
        savePerimeter(corners, returnPoint)
        loadedPerimeter = sullivanParkPremiter()
        if loadedPerimeter is None:
            raise RuntimeError(lastPerimeterRefusalReason)
        print("sullivanParkPremiter() can now load this perimeter.")

    except KeyboardInterrupt:
        print()
        print("Ctrl+C. No new perimeter was saved.")

    except Exception as error:
        print()
        print(f"STOPPED: {error}")

    finally:
        if pixhawkConnection is not None:
            try:
                pixhawkConnection.close()
            except Exception:
                pass
            print("Pixhawk connection closed.")
        print("Finished.")


if __name__ == "__main__":
    main()
