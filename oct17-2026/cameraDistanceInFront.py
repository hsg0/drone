#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    Calibrate the forward camera with the sonic sensor and the time-of-flight
    sensor, and provide importable functions so any other script can read the
    distance of an object in front of the camera.

WHY:
    The camera only sees how tall the blue stripe on the brown box looks in
    pixels. The sonic sensor on the power distribution board and the
    time-of-flight sensor under the drone measure real distance in centimetres.
    Calibrating the camera against those sensors saves one scale file on disk
    so every other script can import this file and read the distance in front
    while the camera is on.

HOW:
    1. Open the forward camera. Do not open USB. Do not talk to the Pixhawk.
    2. Read the sonic and time-of-flight snapshot written by bootDisplay.py.
    3. Find the brown box with the vertical blue stripe in the camera frame.
    4. Combine the sonic sensor, the time-of-flight sensor, and the calibrated
       camera to determine the distance of the object in front.
    5. When run directly, collect 5 agreeing samples and type SAVE to store
       the calibration in /home/drone/Desktop/scripts/forward-distance-calibration.json.
    6. When imported into another script, call readDistanceInFront(camera) or
       distanceInFrontMeters(camera) to get the live distance data.

IMPORTANT:
    Run this on the Pi with the propellers off. bootDisplay.py must already
    be running, because it owns the RedBoard and writes the range snapshot.
    The camera must point forward, the same way as the sonic sensor.

Import from any other script on the Pi:

    import sys
    sys.path.insert(0, "/home/drone/Desktop/scripts/oct17-2026")
    import cameraDistanceInFront

    camera = cameraDistanceInFront.openForwardCamera()
    calibration = cameraDistanceInFront.loadForwardDistanceCalibration()

    reading = cameraDistanceInFront.readDistanceInFront(camera, calibration)
    print(reading["distanceMeters"], reading["distanceSource"])

    cameraDistanceInFront.closeForwardCamera(camera)
"""

import os
import sys
import time


# ---------------------------------------------------------------------------
# Shared calibration module
# ---------------------------------------------------------------------------

SCRIPT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
CALIBRATION_DIRECTORY_CANDIDATES = (
    os.path.join(os.path.dirname(SCRIPT_DIRECTORY), "oct7-2026"),
    "/home/drone/Desktop/scripts/oct7-2026",
)

for calibrationDirectory in CALIBRATION_DIRECTORY_CANDIDATES:
    if os.path.isdir(calibrationDirectory) and calibrationDirectory not in sys.path:
        sys.path.insert(0, calibrationDirectory)

import forward_distance_calibration


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Both sensors are seeing the same object in front when their readings are this close.
# Farther apart, the bottom time-of-flight sensor is seeing a different direction or range.
SENSORS_SEE_SAME_OBJECT_METERS = 0.40

CENTIMETERS_PER_METER = 100.0
FEET_PER_METER = 3.28084
STATUS_PRINT_INTERVAL_SECONDS = 1.0

SAVED_CALIBRATION_PATH = forward_distance_calibration.SAVED_CALIBRATION_PATH


# ---------------------------------------------------------------------------
# Camera and calibration helpers for other scripts to import
# ---------------------------------------------------------------------------

def openForwardCamera():
    """Open and start the Raspberry Pi forward camera."""
    return forward_distance_calibration.openForwardCamera()


def closeForwardCamera(camera):
    """Stop and close the forward camera cleanly."""
    if camera is None:
        return
    try:
        camera.stop()
    except Exception:
        pass
    try:
        camera.close()
    except Exception:
        pass


def loadForwardDistanceCalibration():
    """Load the saved camera calibration dictionary, or None if not available."""
    return forward_distance_calibration.loadForwardDistanceCalibration()


def loadCameraScale():
    """Load only the saved camera scale number, or None if not available."""
    return forward_distance_calibration.loadCameraScale()


def savedCalibrationRefusalReason():
    """Return the sentence explaining why the saved calibration could not be used."""
    return forward_distance_calibration.savedCalibrationRefusalReason()


def readSensorDistancesMeters():
    """Read the live sonic and time-of-flight distances in meters from bootDisplay.py."""
    snapshot = forward_distance_calibration.readForwardRangeSnapshot()
    sonicMeters = forward_distance_calibration.sonicDistanceMeters(snapshot)
    timeOfFlightMeters = forward_distance_calibration.timeOfFlightDistanceMeters(snapshot)
    return sonicMeters, timeOfFlightMeters, snapshot


# ---------------------------------------------------------------------------
# Distance in front calculation
# ---------------------------------------------------------------------------

def distanceInFrontFromSensors(sonicMeters, timeOfFlightMeters):
    """
    Return the distance of the object in front from the range sensors,
    and a sentence that says which sensor supplied it.
    """
    if sonicMeters is None and timeOfFlightMeters is None:
        return None, "Neither range sensor is answering."

    if sonicMeters is None:
        return timeOfFlightMeters, "Time-of-flight sensor under the drone (sonic sensor is not answering)."

    if timeOfFlightMeters is None:
        return sonicMeters, "Sonic sensor on the power board (time-of-flight sensor is out of range)."

    differenceMeters = abs(sonicMeters - timeOfFlightMeters)
    if differenceMeters <= SENSORS_SEE_SAME_OBJECT_METERS:
        averageMeters = (sonicMeters + timeOfFlightMeters) / 2.0
        return averageMeters, "Sonic sensor and time-of-flight sensor both see the object in front."

    return sonicMeters, "Sonic sensor on the power board (time-of-flight sensor sees a different distance)."


def cameraDistanceMeters(rawCameraMeters, cameraScale):
    """Convert raw camera distance to calibrated meters using cameraScale."""
    if rawCameraMeters is None or cameraScale is None:
        return None
    return float(rawCameraMeters) * float(cameraScale)


def combineCameraAndSensorDistanceMeters(cameraMeters, sonicMeters, timeOfFlightMeters):
    """
    Determine the best distance in front when the camera is on.
    Uses sonic, time-of-flight, and the calibrated camera together.
    """
    sensorDistanceMeters, sensorSentence = distanceInFrontFromSensors(
        sonicMeters,
        timeOfFlightMeters,
    )

    if cameraMeters is not None and sensorDistanceMeters is not None:
        differenceMeters = abs(cameraMeters - sensorDistanceMeters)
        if differenceMeters <= SENSORS_SEE_SAME_OBJECT_METERS:
            if (
                sonicMeters is not None
                and timeOfFlightMeters is not None
                and abs(sonicMeters - timeOfFlightMeters) <= SENSORS_SEE_SAME_OBJECT_METERS
            ):
                combinedMeters = (cameraMeters + sonicMeters + timeOfFlightMeters) / 3.0
                return combinedMeters, "camera, sonic sensor, and time-of-flight sensor"
            combinedMeters = (cameraMeters + sensorDistanceMeters) / 2.0
            return combinedMeters, "camera and range sensor"
        return sensorDistanceMeters, sensorSentence

    if sensorDistanceMeters is not None:
        return sensorDistanceMeters, sensorSentence

    if cameraMeters is not None:
        return cameraMeters, "calibrated camera"

    return None, "no distance available"


def readDistanceInFront(camera, calibration=None):
    """
    Capture one camera frame, read the sonic and time-of-flight sensors,
    and return a dictionary of all distance data for other scripts to use.
    """
    if calibration is None:
        calibration = loadForwardDistanceCalibration()

    rawCameraMeters, markerCorners, markerNearEdge, sawBlueWithoutBrownBox = (
        forward_distance_calibration.lookAtMarker(camera)
    )
    objectSeenInCamera = markerCorners is not None and not markerNearEdge

    cameraMeters = None
    if objectSeenInCamera:
        cameraMeters = forward_distance_calibration.scaledCameraDistanceMeters(
            rawCameraMeters,
            calibration,
        )

    sonicMeters, timeOfFlightMeters, snapshot = readSensorDistancesMeters()
    distanceMeters, distanceSource = combineCameraAndSensorDistanceMeters(
        cameraMeters,
        sonicMeters,
        timeOfFlightMeters,
    )

    distanceCentimeters = (
        None if distanceMeters is None else distanceMeters * CENTIMETERS_PER_METER
    )
    distanceFeet = None if distanceMeters is None else distanceMeters * FEET_PER_METER

    return {
        "distanceMeters": distanceMeters,
        "distanceCentimeters": distanceCentimeters,
        "distanceFeet": distanceFeet,
        "distanceSource": distanceSource,
        "cameraMeters": cameraMeters,
        "rawCameraMeters": rawCameraMeters,
        "sonicMeters": sonicMeters,
        "timeOfFlightMeters": timeOfFlightMeters,
        "objectSeenInCamera": objectSeenInCamera,
        "markerCorners": markerCorners,
        "markerNearEdge": markerNearEdge,
        "sawBlueWithoutBrownBox": sawBlueWithoutBrownBox,
        "snapshotFresh": snapshot is not None,
    }


def distanceInFrontMeters(camera, calibration=None):
    """Return only the distance in front in meters, or None if unavailable."""
    reading = readDistanceInFront(camera, calibration)
    return reading["distanceMeters"]


# ---------------------------------------------------------------------------
# Interactive camera calibration loop
# ---------------------------------------------------------------------------

def runCameraDistance(camera):
    """Watch the object in front and calibrate the camera scale until SAVE is typed."""
    print()
    print("CAMERA DISTANCE IN FRONT")
    print("Hold the brown box in front of the nose, with the blue stripe running up and down.")
    print("The sonic sensor faces forward from the power board.")
    print("The time-of-flight sensor faces forward and backward from the bottom of the drone.")
    print(f"{forward_distance_calibration.CAMERA_SCALE_SAMPLE_COUNT} agreeing looks are required.")
    print(f"Type {forward_distance_calibration.SAVE_WORD} to remember the camera scale.")

    existingCalibration = loadForwardDistanceCalibration()
    cameraScale = None
    if existingCalibration is not None:
        cameraScale = float(existingCalibration["cameraScale"])
        print(f"Loaded existing camera scale: {cameraScale:.3f}")

    cameraScaleSamples = []
    timeOfFlightReadingCount = 0
    lastPrintedAtSeconds = 0.0

    while True:
        nowSeconds = time.monotonic()
        typedLine = forward_distance_calibration.typedLineOrNone()
        if typedLine not in (None, ""):
            if typedLine != forward_distance_calibration.SAVE_WORD:
                print(f"Type {forward_distance_calibration.SAVE_WORD} when the camera distance is ready.")
            elif len(cameraScaleSamples) < forward_distance_calibration.CAMERA_SCALE_SAMPLE_COUNT and cameraScale is None:
                print("The camera scale is not ready. Nothing was saved.")
            else:
                return cameraScale, len(cameraScaleSamples), timeOfFlightReadingCount

        if nowSeconds - lastPrintedAtSeconds < STATUS_PRINT_INTERVAL_SECONDS:
            time.sleep(forward_distance_calibration.POLL_INTERVAL_SECONDS)
            continue

        rawCameraMeters, markerCorners, markerNearEdge, sawBlueWithoutBrownBox = (
            forward_distance_calibration.lookAtMarker(camera)
        )
        sonicMeters, timeOfFlightMeters, snapshot = readSensorDistancesMeters()
        if timeOfFlightMeters is not None:
            timeOfFlightReadingCount += 1
        sensorDistanceMeters, sensorSentence = distanceInFrontFromSensors(
            sonicMeters,
            timeOfFlightMeters,
        )

        if snapshot is None:
            print("No fresh sensor snapshot. Leave bootDisplay.py running.")
        elif markerCorners is None and sawBlueWithoutBrownBox:
            print(
                f"Saw a blue stripe, but not on the brown box | "
                f"Sensor distance in front: {forward_distance_calibration.formatMeters(sensorDistanceMeters)} | "
                f"Sonic: {forward_distance_calibration.formatMeters(sonicMeters)} | "
                f"ToF: {forward_distance_calibration.formatMeters(timeOfFlightMeters)}"
            )
        elif markerCorners is None:
            print(
                f"Brown box not in camera view | "
                f"Sensor distance in front: {forward_distance_calibration.formatMeters(sensorDistanceMeters)} | "
                f"Sonic: {forward_distance_calibration.formatMeters(sonicMeters)} | "
                f"ToF: {forward_distance_calibration.formatMeters(timeOfFlightMeters)}"
            )
        elif markerNearEdge:
            print("The blue stripe is cut off by the edge of the picture. Move it toward the centre.")
        elif sensorDistanceMeters is None:
            print(sensorSentence)
        else:
            samplesBefore = len(cameraScaleSamples)
            learnedScale = forward_distance_calibration.addCameraScaleSample(
                cameraScaleSamples,
                rawCameraMeters,
                sensorDistanceMeters,
            )
            if learnedScale is not None:
                cameraScale = learnedScale
            sampleCount = len(cameraScaleSamples)
            if sampleCount == samplesBefore and samplesBefore >= 2 and len(cameraScaleSamples) < forward_distance_calibration.CAMERA_SCALE_SAMPLE_COUNT:
                print("This look does not match the samples already kept. Hold the box still.")

            scaledCameraMeters = cameraDistanceMeters(rawCameraMeters, cameraScale)
            combinedMeters, combinedSource = combineCameraAndSensorDistanceMeters(
                scaledCameraMeters,
                sonicMeters,
                timeOfFlightMeters,
            )

            print(
                f"Object in front: {forward_distance_calibration.formatMeters(combinedMeters)} "
                f"({combinedSource})"
            )
            print(
                f"  Camera: {forward_distance_calibration.formatMeters(scaledCameraMeters)} "
                f"(raw {forward_distance_calibration.formatMeters(rawCameraMeters)}) | "
                f"Sonic: {forward_distance_calibration.formatMeters(sonicMeters)} | "
                f"ToF: {forward_distance_calibration.formatMeters(timeOfFlightMeters)}"
            )
            print(
                f"  Calibration samples: {sampleCount}/"
                f"{forward_distance_calibration.CAMERA_SCALE_SAMPLE_COUNT}"
            )
            if len(cameraScaleSamples) >= forward_distance_calibration.CAMERA_SCALE_SAMPLE_COUNT:
                print(
                    f"  Camera scale locked at {cameraScale:.3f}. "
                    f"Type {forward_distance_calibration.SAVE_WORD} to save it."
                )

        lastPrintedAtSeconds = nowSeconds
        time.sleep(forward_distance_calibration.POLL_INTERVAL_SECONDS)


def main():
    camera = None
    try:
        print()
        print("The camera will measure and calibrate the distance of the object in front.")
        print("The copter will not move.")
        camera = openForwardCamera()
        cameraScale, sampleCount, timeOfFlightReadingCount = runCameraDistance(camera)
        savedCalibration = forward_distance_calibration.saveForwardDistanceCalibration(
            cameraScale,
            sampleCount,
            timeOfFlightReadingCount,
        )
        print()
        print(f"Saved camera scale: {savedCalibration['cameraScale']:.3f}")
        print(f"Saved file: {SAVED_CALIBRATION_PATH}")
        print("Other scripts can import cameraDistanceInFront and call readDistanceInFront(camera).")
    except KeyboardInterrupt:
        print()
        print("Stopped. Nothing new was saved.")
    except Exception as error:
        print()
        print(f"Stopped because of an error: {error}")
    finally:
        closeForwardCamera(camera)
        print("Camera closed.")


if __name__ == "__main__":
    main()
