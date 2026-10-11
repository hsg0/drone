#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    Calibrate the forward camera against the sonic sensor and save the result.
    The time-of-flight sensor is checked too. When it is answering and close
    to the sonic reading, that agreement is saved as a note. It is not required.
    Any later script can import this file and load the saved numbers.

WHY:
    The sonic sensor already reports centimetres. The camera only sees how
    big the marker looks. The saved camera scale turns that picture size into
    the same metres as the sonic sensor, so every flight script can share one
    calibration instead of learning it again.

HOW:
    1. Open the forward camera. Do not open USB. Do not talk to the Pixhawk.
    2. Read the sonic and time-of-flight snapshot written by bootDisplay.py.
    3. Each second the marker and the sonic sensor agree, keep one scale sample.
    4. After enough samples, type SAVE. The numbers are written to disk.
    5. Another file imports loadForwardDistanceCalibration() and receives them.

IMPORTANT:
    Run this on the Pi with the propellers off. bootDisplay.py must already
    be running, because it owns the RedBoard and writes the range snapshot.
    This script does not open the Pixhawk USB port or the RedBoard USB port.

    The camera must point forward. The target is the brown box with the blue
    tape running from top to bottom. Keep the whole blue stripe in the picture.
    BLUE_STRIPE_LENGTH_METERS is a stand-in length. The sonic sensor sets the
    real scale, so the same stripe must be used every time.

    The saved file remembers the marker and the camera size. A later script
    that asks for a different marker or a different picture size is given no
    calibration, instead of a scale that belongs to another setup.

    A missing time-of-flight reading does not block the save. The sonic sensor
    is the calibration reference. Flight scripts must still read the live
    time-of-flight sensor. The saved note only says whether it was answering
    and agreeing during this calibration.

Import from any script on the Pi. This file lives in the oct7-2026 folder:

    import sys
    sys.path.insert(0, "/home/drone/Desktop/scripts/oct7-2026")
    import forward_distance_calibration

    calibration = forward_distance_calibration.loadForwardDistanceCalibration()
    if calibration is None:
        raise RuntimeError(forward_distance_calibration.savedCalibrationRefusalReason())

    cameraScale = forward_distance_calibration.loadCameraScale()
"""

import json
import math
import os
import select
import sys
import time

import cv2
from picamera2 import Picamera2


# ---------------------------------------------------------------------------
# Saved calibration
# ---------------------------------------------------------------------------

# Outside the date folder, so every script can find the same remembered file.
SAVED_CALIBRATION_PATH = "/home/drone/Desktop/scripts/forward-distance-calibration.json"
CALIBRATION_VERSION = 2
SAVE_WORD = "SAVE"

# ---------------------------------------------------------------------------
# Camera and marker
# ---------------------------------------------------------------------------

# Saved so a later script can tell this target from a different one.
MARKER_DICTIONARY_NAME = "BROWN_BOX_BLUE_STRIPE"
MARKER_ID = 0

# Stand-in length of the blue stripe. The sonic samples correct the camera
# scale, so this does not have to be the tape's exact length.
BLUE_STRIPE_LENGTH_METERS = 0.30
MARKER_WIDTH_METERS = BLUE_STRIPE_LENGTH_METERS

# Measured from brown-box-blue-stripe.jpg, the Pi photo of this box.
# OpenCV hue is 0 to 180. The tape in that photo is a narrow bright-blue
# column. The cardboard beside it is a darker orange-brown.
BLUE_HUE_MINIMUM = 95
BLUE_HUE_MAXIMUM = 125
BLUE_SATURATION_MINIMUM = 140
BLUE_VALUE_MINIMUM = 35
BLUE_VALUE_MAXIMUM = 140
BLUE_STRIPE_MINIMUM_HEIGHT_PIXELS = 40
BLUE_STRIPE_MINIMUM_HEIGHT_TO_WIDTH = 1.5
# The tape fills its outline. The training photo is about 0.43.
# A shadowed post can look faintly blue, but only in scattered pixels.
BLUE_STRIPE_MINIMUM_FILL_FRACTION = 0.25

# Cardboard beside the stripe. A blue line with no brown box is ignored.
BROWN_HUE_MINIMUM = 15
BROWN_HUE_MAXIMUM = 50
BROWN_SATURATION_MINIMUM = 40
BROWN_SATURATION_MAXIMUM = 255
BROWN_VALUE_MINIMUM = 15
BROWN_VALUE_MAXIMUM = 140
BROWN_BESIDE_STRIPE_MINIMUM_FRACTION = 0.12

CAMERA_FRAME_WIDTH_PIXELS = 640
CAMERA_FRAME_HEIGHT_PIXELS = 480
CAMERA_HORIZONTAL_FIELD_OF_VIEW_DEGREES = 54.0
MARKER_EDGE_MARGIN_FRACTION = 0.08

CAMERA_SCALE_SAMPLE_COUNT = 5
CAMERA_SCALE_MINIMUM = 0.50
CAMERA_SCALE_MAXIMUM = 2.00
CAMERA_SCALE_SAMPLE_MINIMUM_METERS = 0.30
# A new look must stay this close to the samples already kept.
CAMERA_SCALE_MATCH_FRACTION = 0.15

STATUS_PRINT_INTERVAL_SECONDS = 1.0
POLL_INTERVAL_SECONDS = 0.2


# ---------------------------------------------------------------------------
# Forward range sensors
# ---------------------------------------------------------------------------

# Written by bootDisplay.py. This script must not open the RedBoard itself.
FORWARD_RANGE_SNAPSHOT_PATH = "/tmp/redboard-forward-distance.json"
# The display refreshes the snapshot often. Two seconds still covers a short gap.
FORWARD_RANGE_STALE_AFTER_SECONDS = 2.0
SNAPSHOT_FUTURE_SKEW_ALLOWANCE_SECONDS = 1.0

SONIC_MINIMUM_CENTIMETERS = 15.0
SONIC_MAXIMUM_CENTIMETERS = 400.0
SONIC_SNAPSHOT_FIELD_NAMES = (
    "sonicCentimeters",
    "sonicCm",
    "distanceCm",
)

TIME_OF_FLIGHT_MINIMUM_CENTIMETERS = 5.0
TIME_OF_FLIGHT_MAXIMUM_CENTIMETERS = 400.0
TIME_OF_FLIGHT_SNAPSHOT_FIELD_NAMES = (
    "tofCentimeters",
    "tofCm",
)

# Close enough to remember that the time-of-flight sensor confirmed the sonic reading.
RANGE_AGREEMENT_METERS = 0.40
CENTIMETERS_PER_METER = 100.0

lastSavedCalibrationRefusalReason = None


# ---------------------------------------------------------------------------
# Live ranges
# ---------------------------------------------------------------------------

def readForwardRangeSnapshot():
    """Return the latest sonic and time-of-flight snapshot, or None when it is stale."""
    try:
        with open(FORWARD_RANGE_SNAPSHOT_PATH, "r", encoding="utf-8") as snapshotFile:
            snapshot = json.load(snapshotFile)
    except (OSError, json.JSONDecodeError):
        return None
    updatedAtSeconds = snapshot.get("updatedAtSeconds")
    if not isinstance(updatedAtSeconds, (int, float)):
        return None
    ageSeconds = time.time() - float(updatedAtSeconds)
    if ageSeconds < -SNAPSHOT_FUTURE_SKEW_ALLOWANCE_SECONDS:
        return None
    if ageSeconds > FORWARD_RANGE_STALE_AFTER_SECONDS:
        return None
    return snapshot


def trustedRangeMeters(snapshot, fieldNames, minimumCentimeters, maximumCentimeters):
    if snapshot is None:
        return None
    centimeters = None
    for fieldName in fieldNames:
        candidateValue = snapshot.get(fieldName)
        if isinstance(candidateValue, (int, float)):
            centimeters = float(candidateValue)
            break
    if centimeters is None:
        return None
    if centimeters < minimumCentimeters or centimeters > maximumCentimeters:
        return None
    return centimeters / CENTIMETERS_PER_METER


def sonicDistanceMeters(snapshot):
    return trustedRangeMeters(
        snapshot,
        SONIC_SNAPSHOT_FIELD_NAMES,
        SONIC_MINIMUM_CENTIMETERS,
        SONIC_MAXIMUM_CENTIMETERS,
    )


def timeOfFlightDistanceMeters(snapshot):
    return trustedRangeMeters(
        snapshot,
        TIME_OF_FLIGHT_SNAPSHOT_FIELD_NAMES,
        TIME_OF_FLIGHT_MINIMUM_CENTIMETERS,
        TIME_OF_FLIGHT_MAXIMUM_CENTIMETERS,
    )


def timeOfFlightAgreesWithSonic(sonicMeters, timeOfFlightMeters):
    """A missing time-of-flight reading is not agreement and not a failure."""
    if sonicMeters is None or timeOfFlightMeters is None:
        return False
    return abs(sonicMeters - timeOfFlightMeters) <= RANGE_AGREEMENT_METERS


# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------

def focalLengthPixels():
    halfViewRadians = math.radians(CAMERA_HORIZONTAL_FIELD_OF_VIEW_DEGREES / 2.0)
    return (CAMERA_FRAME_WIDTH_PIXELS / 2.0) / math.tan(halfViewRadians)


def openForwardCamera():
    print("Opening the forward camera.")
    camera = Picamera2()
    configuration = camera.create_preview_configuration(
        main={
            "size": (CAMERA_FRAME_WIDTH_PIXELS, CAMERA_FRAME_HEIGHT_PIXELS),
            "format": "RGB888",
        }
    )
    camera.configure(configuration)
    camera.start()
    print("Camera open. Looking for the brown box with the vertical blue stripe.")
    return camera


def brownIsBesideStripe(hsvFrame, stripeX, stripeY, stripeWidth, stripeHeight):
    """The blue tape should sit on the brown box, not on a random blue object."""
    searchWidthPixels = max(int(stripeWidth * 3), 24)
    leftX = max(0, stripeX - searchWidthPixels)
    rightX = min(CAMERA_FRAME_WIDTH_PIXELS, stripeX + stripeWidth + searchWidthPixels)
    bottomY = min(CAMERA_FRAME_HEIGHT_PIXELS, stripeY + stripeHeight)
    nearby = hsvFrame[stripeY:bottomY, leftX:rightX]
    if nearby.size == 0:
        return False
    brownMask = cv2.inRange(
        nearby,
        (BROWN_HUE_MINIMUM, BROWN_SATURATION_MINIMUM, BROWN_VALUE_MINIMUM),
        (BROWN_HUE_MAXIMUM, BROWN_SATURATION_MAXIMUM, BROWN_VALUE_MAXIMUM),
    )
    brownPixels = cv2.countNonZero(brownMask)
    return (brownPixels / nearby.shape[0] / nearby.shape[1]) >= BROWN_BESIDE_STRIPE_MINIMUM_FRACTION


def blueStripeCorners(frame):
    """
    Return the four corners of the vertical blue stripe, and whether a blue
    stripe was seen without the brown box.
    """
    hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blueMask = cv2.inRange(
        hsvFrame,
        (BLUE_HUE_MINIMUM, BLUE_SATURATION_MINIMUM, BLUE_VALUE_MINIMUM),
        (BLUE_HUE_MAXIMUM, 255, BLUE_VALUE_MAXIMUM),
    )
    # Close small gaps along the tape so one stripe stays one shape.
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 11))
    blueMask = cv2.morphologyEx(blueMask, cv2.MORPH_CLOSE, kernel)
    contours, _hierarchy = cv2.findContours(blueMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    bestStripe = None
    bestHeightPixels = 0
    sawBlueWithoutBrownBox = False
    for contour in contours:
        stripeX, stripeY, stripeWidth, stripeHeight = cv2.boundingRect(contour)
        if stripeHeight < BLUE_STRIPE_MINIMUM_HEIGHT_PIXELS:
            continue
        if stripeHeight < stripeWidth * BLUE_STRIPE_MINIMUM_HEIGHT_TO_WIDTH:
            continue
        stripeAreaPixels = stripeWidth * stripeHeight
        bluePixelsInsideStripe = cv2.countNonZero(
            blueMask[stripeY:stripeY + stripeHeight, stripeX:stripeX + stripeWidth]
        )
        if stripeAreaPixels <= 0:
            continue
        if (bluePixelsInsideStripe / stripeAreaPixels) < BLUE_STRIPE_MINIMUM_FILL_FRACTION:
            continue
        if not brownIsBesideStripe(hsvFrame, stripeX, stripeY, stripeWidth, stripeHeight):
            sawBlueWithoutBrownBox = True
            continue
        if stripeHeight > bestHeightPixels:
            bestHeightPixels = stripeHeight
            bestStripe = (stripeX, stripeY, stripeWidth, stripeHeight)

    if bestStripe is None:
        return None, sawBlueWithoutBrownBox

    stripeX, stripeY, stripeWidth, stripeHeight = bestStripe
    corners = [
        [stripeX, stripeY],
        [stripeX + stripeWidth, stripeY],
        [stripeX + stripeWidth, stripeY + stripeHeight],
        [stripeX, stripeY + stripeHeight],
    ]
    return corners, False


def stripeLengthPixels(markerCorners):
    """The vertical run of the blue tape. That length is what changes with distance."""
    return abs(float(markerCorners[3][1]) - float(markerCorners[0][1]))


def rawCameraDistanceMeters(markerCorners):
    apparentLengthPixels = stripeLengthPixels(markerCorners)
    if apparentLengthPixels < 1.0:
        return None
    return (BLUE_STRIPE_LENGTH_METERS * focalLengthPixels()) / apparentLengthPixels


def markerIsNearFrameEdge(markerCorners):
    marginXPixels = CAMERA_FRAME_WIDTH_PIXELS * MARKER_EDGE_MARGIN_FRACTION
    marginYPixels = CAMERA_FRAME_HEIGHT_PIXELS * MARKER_EDGE_MARGIN_FRACTION
    for point in markerCorners:
        xPixels = float(point[0])
        yPixels = float(point[1])
        if xPixels < marginXPixels or yPixels < marginYPixels:
            return True
        if xPixels > CAMERA_FRAME_WIDTH_PIXELS - marginXPixels:
            return True
        if yPixels > CAMERA_FRAME_HEIGHT_PIXELS - marginYPixels:
            return True
    return False


def lookAtMarker(camera):
    frame = camera.capture_array()
    markerCorners, sawBlueWithoutBrownBox = blueStripeCorners(frame)
    if markerCorners is None:
        return None, None, False, sawBlueWithoutBrownBox
    return (
        rawCameraDistanceMeters(markerCorners),
        markerCorners,
        markerIsNearFrameEdge(markerCorners),
        False,
    )


def scaledCameraDistanceMeters(rawCameraDistanceMeters, calibration):
    """Apply a loaded camera scale. Returns None when either piece is missing."""
    if rawCameraDistanceMeters is None or calibration is None:
        return None
    cameraScale = calibration.get("cameraScale")
    if not isinstance(cameraScale, (int, float)):
        return None
    return float(rawCameraDistanceMeters) * float(cameraScale)


# ---------------------------------------------------------------------------
# Samples and the saved file
# ---------------------------------------------------------------------------

def medianOf(values):
    ordered = sorted(values)
    middleIndex = len(ordered) // 2
    if len(ordered) % 2 == 1:
        return ordered[middleIndex]
    return (ordered[middleIndex - 1] + ordered[middleIndex]) / 2.0


def addCameraScaleSample(cameraScaleSamples, rawCameraMeters, sonicMeters):
    """Keep one sonic-to-camera sample when it agrees with the samples already kept."""
    if len(cameraScaleSamples) >= CAMERA_SCALE_SAMPLE_COUNT:
        return medianOf(cameraScaleSamples[:CAMERA_SCALE_SAMPLE_COUNT])
    if rawCameraMeters is None or sonicMeters is None:
        return None
    if (
        rawCameraMeters < CAMERA_SCALE_SAMPLE_MINIMUM_METERS
        or sonicMeters < CAMERA_SCALE_SAMPLE_MINIMUM_METERS
    ):
        return None
    scaleSample = sonicMeters / rawCameraMeters
    if scaleSample < CAMERA_SCALE_MINIMUM or scaleSample > CAMERA_SCALE_MAXIMUM:
        return None
    if len(cameraScaleSamples) >= 2:
        currentMedian = medianOf(cameraScaleSamples)
        if currentMedian > 0:
            differenceFraction = abs(scaleSample - currentMedian) / currentMedian
            if differenceFraction > CAMERA_SCALE_MATCH_FRACTION:
                return None
    cameraScaleSamples.append(scaleSample)
    if len(cameraScaleSamples) < CAMERA_SCALE_SAMPLE_COUNT:
        return None
    return medianOf(cameraScaleSamples)


def formatMeters(distanceMeters):
    if distanceMeters is None:
        return "missing"
    return f"{distanceMeters:.2f} m"


def rememberRefusal(reason):
    global lastSavedCalibrationRefusalReason
    lastSavedCalibrationRefusalReason = reason
    return None


def savedCalibrationRefusalReason():
    """Why the last loadForwardDistanceCalibration() call returned None."""
    if not lastSavedCalibrationRefusalReason:
        return "The saved calibration matches this marker and camera."
    return lastSavedCalibrationRefusalReason


def calibrationMatchesThisSetup(savedCalibration, markerDictionaryName, markerId, markerWidthMeters, cameraFrameWidthPixels, cameraFrameHeightPixels):
    if savedCalibration.get("calibrationVersion") != CALIBRATION_VERSION:
        return "The saved calibration is from an older layout."
    if savedCalibration.get("markerDictionaryName") != markerDictionaryName:
        return "The saved calibration used a different ArUco dictionary."
    if savedCalibration.get("markerId") != markerId:
        return "The saved calibration used a different marker id."
    savedWidthMeters = savedCalibration.get("markerWidthMeters")
    if not isinstance(savedWidthMeters, (int, float)):
        return "The saved calibration has no marker width."
    if abs(float(savedWidthMeters) - float(markerWidthMeters)) > 0.001:
        return "The saved calibration used a different printed marker width."
    if savedCalibration.get("cameraFrameWidthPixels") != cameraFrameWidthPixels:
        return "The saved calibration used a different camera width."
    if savedCalibration.get("cameraFrameHeightPixels") != cameraFrameHeightPixels:
        return "The saved calibration used a different camera height."
    cameraScale = savedCalibration.get("cameraScale")
    if not isinstance(cameraScale, (int, float)):
        return "The saved calibration has no camera scale."
    if float(cameraScale) < CAMERA_SCALE_MINIMUM or float(cameraScale) > CAMERA_SCALE_MAXIMUM:
        return "The saved camera scale is outside the accepted range."
    return None


def loadForwardDistanceCalibration(
    markerDictionaryName=MARKER_DICTIONARY_NAME,
    markerId=MARKER_ID,
    markerWidthMeters=MARKER_WIDTH_METERS,
    cameraFrameWidthPixels=CAMERA_FRAME_WIDTH_PIXELS,
    cameraFrameHeightPixels=CAMERA_FRAME_HEIGHT_PIXELS,
):
    """
    Load the remembered calibration for this marker and camera.
    Returns None when the file is missing or belongs to a different setup.
    Call savedCalibrationRefusalReason() for the sentence that explains why.
    """
    try:
        with open(SAVED_CALIBRATION_PATH, "r", encoding="utf-8") as calibrationFile:
            savedCalibration = json.load(calibrationFile)
    except FileNotFoundError:
        return rememberRefusal(f"No saved calibration at {SAVED_CALIBRATION_PATH}.")
    except (OSError, json.JSONDecodeError):
        return rememberRefusal(f"The saved calibration at {SAVED_CALIBRATION_PATH} could not be read.")

    if not isinstance(savedCalibration, dict):
        return rememberRefusal("The saved calibration was not a set of named values.")

    refusalReason = calibrationMatchesThisSetup(
        savedCalibration,
        markerDictionaryName,
        markerId,
        markerWidthMeters,
        cameraFrameWidthPixels,
        cameraFrameHeightPixels,
    )
    if refusalReason is not None:
        return rememberRefusal(refusalReason)

    rememberRefusal("")
    return savedCalibration


def loadCameraScale(
    markerDictionaryName=MARKER_DICTIONARY_NAME,
    markerId=MARKER_ID,
    markerWidthMeters=MARKER_WIDTH_METERS,
    cameraFrameWidthPixels=CAMERA_FRAME_WIDTH_PIXELS,
    cameraFrameHeightPixels=CAMERA_FRAME_HEIGHT_PIXELS,
):
    """Return only the saved camera scale, or None when the file cannot be used."""
    calibration = loadForwardDistanceCalibration(
        markerDictionaryName,
        markerId,
        markerWidthMeters,
        cameraFrameWidthPixels,
        cameraFrameHeightPixels,
    )
    if calibration is None:
        return None
    return float(calibration["cameraScale"])


def saveForwardDistanceCalibration(cameraScale, sampleCount, timeOfFlightReadingCount, timeOfFlightAgreementCount):
    """Write the finished calibration. A temporary file is renamed into place."""
    savedCalibration = {
        "calibrationVersion": CALIBRATION_VERSION,
        "cameraScale": float(cameraScale),
        "sampleCount": int(sampleCount),
        "markerDictionaryName": MARKER_DICTIONARY_NAME,
        "markerId": MARKER_ID,
        "markerWidthMeters": MARKER_WIDTH_METERS,
        "cameraFrameWidthPixels": CAMERA_FRAME_WIDTH_PIXELS,
        "cameraFrameHeightPixels": CAMERA_FRAME_HEIGHT_PIXELS,
        "cameraHorizontalFieldOfViewDegrees": CAMERA_HORIZONTAL_FIELD_OF_VIEW_DEGREES,
        "timeOfFlightWasSeen": timeOfFlightReadingCount > 0,
        "timeOfFlightReadingCount": int(timeOfFlightReadingCount),
        "timeOfFlightAgreementCount": int(timeOfFlightAgreementCount),
        "savedAtSeconds": time.time(),
    }
    os.makedirs(os.path.dirname(SAVED_CALIBRATION_PATH), exist_ok=True)
    temporaryPath = SAVED_CALIBRATION_PATH + ".tmp"
    with open(temporaryPath, "w", encoding="utf-8") as calibrationFile:
        json.dump(savedCalibration, calibrationFile, indent=2)
        calibrationFile.write("\n")
    os.replace(temporaryPath, SAVED_CALIBRATION_PATH)
    return savedCalibration


def typedLineOrNone():
    readyToRead, _unused, _alsoUnused = select.select([sys.stdin], [], [], 0)
    if not readyToRead:
        return None
    return sys.stdin.readline().strip()


def collectSamplesUntilSave(camera):
    """Watch the marker until SAVE is accepted. Nothing here moves the copter."""
    print()
    print("SENSOR CALIBRATION")
    print("Hold the brown box a few feet in front of the nose, with the blue stripe running up and down.")
    print("bootDisplay.py must be running so the sonic snapshot is fresh.")
    print(f"{CAMERA_SCALE_SAMPLE_COUNT} good camera and sonic samples are required.")
    print(f"Type {SAVE_WORD} to remember them. The copter will not move.")

    cameraScale = None
    cameraScaleSamples = []
    timeOfFlightReadingCount = 0
    timeOfFlightAgreementCount = 0
    lastPrintedAtSeconds = 0

    while True:
        nowSeconds = time.monotonic()
        typedLine = typedLineOrNone()
        if typedLine not in (None, ""):
            if typedLine != SAVE_WORD:
                print(f"Type {SAVE_WORD} when the scale is ready.")
            elif cameraScale is None:
                print("Calibration is not complete. Nothing was saved.")
            else:
                return cameraScale, len(cameraScaleSamples), timeOfFlightReadingCount, timeOfFlightAgreementCount

        if nowSeconds - lastPrintedAtSeconds < STATUS_PRINT_INTERVAL_SECONDS:
            time.sleep(POLL_INTERVAL_SECONDS)
            continue

        rawCameraMeters, markerCorners, markerNearEdge, sawBlueWithoutBrownBox = lookAtMarker(camera)
        snapshot = readForwardRangeSnapshot()
        sonicMeters = sonicDistanceMeters(snapshot)
        timeOfFlightMeters = timeOfFlightDistanceMeters(snapshot)
        if timeOfFlightMeters is not None:
            timeOfFlightReadingCount += 1
        if timeOfFlightAgreesWithSonic(sonicMeters, timeOfFlightMeters):
            timeOfFlightAgreementCount += 1

        if snapshot is None:
            print("No fresh forward-range snapshot. Leave bootDisplay.py running.")
        elif markerCorners is None and sawBlueWithoutBrownBox:
            print("Saw a blue stripe, but not on the brown box. No calibration sample added.")
        elif markerCorners is None:
            print("Brown box with the blue stripe is not in the picture. No calibration sample added.")
        elif markerNearEdge:
            print("Marker is near the edge of the picture. Move it toward the centre.")
        elif sonicMeters is None:
            print("Sonic distance is missing or outside the accepted range. No calibration sample added.")
        elif rawCameraMeters is None:
            print("Camera distance could not be calculated. No calibration sample added.")
        elif len(cameraScaleSamples) >= CAMERA_SCALE_SAMPLE_COUNT:
            cameraScale = medianOf(cameraScaleSamples[:CAMERA_SCALE_SAMPLE_COUNT])
            print(f"Sonic {formatMeters(sonicMeters)} | Time of flight {formatMeters(timeOfFlightMeters)}")
            print(
                f"Camera scale locked at {cameraScale:.3f}. "
                f"Type {SAVE_WORD} to remember it."
            )
        else:
            samplesBefore = len(cameraScaleSamples)
            learnedScale = addCameraScaleSample(cameraScaleSamples, rawCameraMeters, sonicMeters)
            if learnedScale is not None:
                cameraScale = learnedScale
            sampleCount = len(cameraScaleSamples)
            if sampleCount == samplesBefore and samplesBefore >= 2:
                print("This look does not match the samples already kept. Hold the box still.")
            scaledMeters = None if cameraScale is None or rawCameraMeters is None else rawCameraMeters * cameraScale
            print(
                f"Raw camera {formatMeters(rawCameraMeters)} | "
                f"Scaled camera {formatMeters(scaledMeters)} | "
                f"Sonic {formatMeters(sonicMeters)} | "
                f"Time of flight {formatMeters(timeOfFlightMeters)}"
            )
            print(f"Calibration samples: {sampleCount}/{CAMERA_SCALE_SAMPLE_COUNT}")
            if cameraScale is not None:
                print(f"Camera scale locked at {cameraScale:.3f}. Type {SAVE_WORD} to remember it.")

        lastPrintedAtSeconds = nowSeconds
        time.sleep(POLL_INTERVAL_SECONDS)


def main():
    camera = None
    try:
        print()
        print("FORWARD DISTANCE CALIBRATION")
        print()
        camera = openForwardCamera()
        (
            cameraScale,
            sampleCount,
            timeOfFlightReadingCount,
            timeOfFlightAgreementCount,
        ) = collectSamplesUntilSave(camera)
        savedCalibration = saveForwardDistanceCalibration(
            cameraScale,
            sampleCount,
            timeOfFlightReadingCount,
            timeOfFlightAgreementCount,
        )
        print()
        print(f"Saved camera scale: {savedCalibration['cameraScale']:.3f}")
        print(f"Saved file: {SAVED_CALIBRATION_PATH}")
        print(f"Samples collected: {savedCalibration['sampleCount']}")
        if savedCalibration["timeOfFlightWasSeen"]:
            print("Time of flight was answering.")
            print(
                f"It agreed with the sonic sensor {timeOfFlightAgreementCount} time"
                f"{'' if timeOfFlightAgreementCount == 1 else 's'}."
            )
        else:
            print("Time of flight was not answering. The camera scale was still saved.")
        print("Other scripts can import this file and reuse the saved calibration.")
    except KeyboardInterrupt:
        print()
        print("Calibration stopped. Nothing new was saved.")
    except Exception as error:
        print()
        print(f"Calibration stopped because of an error: {error}")
    finally:
        if camera is not None:
            try:
                camera.stop()
            except Exception:
                pass
            print("Camera closed.")
        print("Finished.")


if __name__ == "__main__":
    main()
