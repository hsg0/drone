#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    Measure the distance of the brown box in front of the camera.
    The sonic sensor and the time-of-flight sensor supply the real distance.
    The camera turns the size of the blue stripe into that same distance.

WHY:
    The camera only sees how tall the stripe looks. It does not know metres
    until a real sensor tells it. The sonic sensor is on the power
    distribution board and faces forward, so it measures the object in front.
    The time-of-flight sensor is under the drone and faces forward and
    backward. Its number is included when it matches the sonic sensor, because
    then both sensors are seeing the front. When the two numbers differ, the
    time-of-flight sensor is seeing the other direction and is not mixed in.

HOW:
    1. Open the forward camera. Do not open USB. Do not talk to the Pixhawk.
    2. Read the sonic and time-of-flight snapshot written by bootDisplay.py.
    3. Find the brown box with the vertical blue stripe.
    4. Take the forward distance from both sensors when they agree, or from
       the sonic sensor when the time-of-flight sensor is looking elsewhere.
    5. Learn the camera scale from that distance. Print the camera distance
       of the box once a second.
    6. Type SAVE to remember the scale. Ctrl+C stops without saving.

IMPORTANT:
    Run this on the Pi, with the propellers off. bootDisplay.py must already
    be running, because it owns the RedBoard and writes the range snapshot.
    The camera must point forward, the same way as the sonic sensor.
    The whole blue stripe must stay in the picture. This script measures that
    box. It cannot guess the distance of an unmarked object, because the
    camera needs a known size.
    The time-of-flight sensor does not need a calibration of its own. It
    already reports centimetres.
"""

import os
import sys
import time


SCRIPT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
CALIBRATION_DIRECTORY_CANDIDATES = (
    os.path.join(os.path.dirname(SCRIPT_DIRECTORY), "oct7-2026"),
    "/home/drone/Desktop/scripts/oct7-2026",
)

for calibrationDirectory in CALIBRATION_DIRECTORY_CANDIDATES:
    if os.path.isdir(calibrationDirectory) and calibrationDirectory not in sys.path:
        sys.path.insert(0, calibrationDirectory)

import forward_distance_calibration


# Both sensors are seeing the object in front when their readings are this close.
# Farther apart, the bottom sensor is looking backward and must not be averaged in.
SENSORS_SEE_SAME_OBJECT_METERS = 0.40

STATUS_PRINT_INTERVAL_SECONDS = 1.0


def distanceInFrontFromSensors(sonicMeters, timeOfFlightMeters):
    """
    Return the distance of the object in front, and a sentence that says
    which sensor supplied it.
    """
    if sonicMeters is None and timeOfFlightMeters is None:
        return None, "Neither sensor is answering."

    if sonicMeters is None:
        return None, (
            "The sonic sensor is not answering. "
            "The time-of-flight reading may be behind the drone."
        )

    if timeOfFlightMeters is None:
        return sonicMeters, "Sonic sensor on the power board. Time of flight is not answering."

    differenceMeters = abs(sonicMeters - timeOfFlightMeters)
    if differenceMeters <= SENSORS_SEE_SAME_OBJECT_METERS:
        averageMeters = (sonicMeters + timeOfFlightMeters) / 2.0
        return averageMeters, "Sonic sensor and time-of-flight sensor both see the object in front."

    return sonicMeters, "Sonic sensor. The time-of-flight sensor is seeing a different direction."


def cameraDistanceMeters(rawCameraMeters, cameraScale):
    if rawCameraMeters is None or cameraScale is None:
        return None
    return rawCameraMeters * cameraScale


def runCameraDistance(camera):
    """Watch the box until SAVE is accepted. Nothing here moves the copter."""
    print()
    print("CAMERA DISTANCE IN FRONT")
    print("Hold the brown box in front of the nose, with the blue stripe running up and down.")
    print("The sonic sensor faces forward from the power board.")
    print("The time-of-flight sensor faces forward and backward from the bottom of the drone.")
    print(f"{forward_distance_calibration.CAMERA_SCALE_SAMPLE_COUNT} agreeing looks are required.")
    print(f"Type {forward_distance_calibration.SAVE_WORD} to remember the camera scale.")

    cameraScale = None
    cameraScaleSamples = []
    timeOfFlightReadingCount = 0
    lastPrintedAtSeconds = 0.0

    while True:
        nowSeconds = time.monotonic()
        typedLine = forward_distance_calibration.typedLineOrNone()
        if typedLine not in (None, ""):
            if typedLine != forward_distance_calibration.SAVE_WORD:
                print(f"Type {forward_distance_calibration.SAVE_WORD} when the camera distance is ready.")
            elif cameraScale is None:
                print("The camera scale is not ready. Nothing was saved.")
            else:
                return cameraScale, len(cameraScaleSamples), timeOfFlightReadingCount

        if nowSeconds - lastPrintedAtSeconds < STATUS_PRINT_INTERVAL_SECONDS:
            time.sleep(forward_distance_calibration.POLL_INTERVAL_SECONDS)
            continue

        rawCameraMeters, markerCorners, markerNearEdge, sawBlueWithoutBrownBox = (
            forward_distance_calibration.lookAtMarker(camera)
        )
        snapshot = forward_distance_calibration.readForwardRangeSnapshot()
        sonicMeters = forward_distance_calibration.sonicDistanceMeters(snapshot)
        timeOfFlightMeters = forward_distance_calibration.timeOfFlightDistanceMeters(snapshot)
        if timeOfFlightMeters is not None:
            timeOfFlightReadingCount += 1
        sensorDistanceMeters, sensorSentence = distanceInFrontFromSensors(
            sonicMeters,
            timeOfFlightMeters,
        )

        if snapshot is None:
            print("No fresh sensor snapshot. Leave bootDisplay.py running.")
        elif markerCorners is None and sawBlueWithoutBrownBox:
            print("Saw a blue stripe, but not on the brown box.")
        elif markerCorners is None:
            print("The camera does not see the brown box.")
        elif markerNearEdge:
            print("The blue stripe is cut off by the edge of the picture.")
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
            if sampleCount == samplesBefore and samplesBefore >= 2 and cameraScale is None:
                print("This look does not match the samples already kept. Hold the box still.")

            shownDistanceMeters = cameraDistanceMeters(rawCameraMeters, cameraScale)
            if shownDistanceMeters is None:
                shownDistanceMeters = sensorDistanceMeters
                distanceName = "sensors, camera scale still learning"
            else:
                distanceName = "camera"

            print(
                f"Object in front: {forward_distance_calibration.formatMeters(shownDistanceMeters)} "
                f"from the {distanceName}."
            )
            print(sensorSentence)
            print(
                f"Sonic, {forward_distance_calibration.SONIC_PLACEMENT}: "
                f"{forward_distance_calibration.formatMeters(sonicMeters)}"
            )
            print(
                f"Time of flight, {forward_distance_calibration.TIME_OF_FLIGHT_PLACEMENT}: "
                f"{forward_distance_calibration.formatMeters(timeOfFlightMeters)}"
            )
            print(
                f"Camera scale samples: {sampleCount}/"
                f"{forward_distance_calibration.CAMERA_SCALE_SAMPLE_COUNT}"
            )
            if cameraScale is not None:
                print(
                    f"Camera scale is {cameraScale:.3f}. "
                    f"Type {forward_distance_calibration.SAVE_WORD} to remember it."
                )

        lastPrintedAtSeconds = nowSeconds
        time.sleep(forward_distance_calibration.POLL_INTERVAL_SECONDS)


def main():
    camera = None
    try:
        print()
        print("The camera will learn the distance of the box in front.")
        print("The copter will not move.")
        camera = forward_distance_calibration.openForwardCamera()
        cameraScale, sampleCount, timeOfFlightReadingCount = runCameraDistance(camera)
        savedCalibration = forward_distance_calibration.saveForwardDistanceCalibration(
            cameraScale,
            sampleCount,
            timeOfFlightReadingCount,
        )
        print()
        print(f"Saved camera scale: {savedCalibration['cameraScale']:.3f}")
        print(f"Saved file: {forward_distance_calibration.SAVED_CALIBRATION_PATH}")
        print("Later scripts can use this scale for the distance of the box in front.")
    except KeyboardInterrupt:
        print()
        print("Stopped. Nothing new was saved.")
    except Exception as error:
        print()
        print(f"Stopped because of an error: {error}")
    finally:
        if camera is not None:
            try:
                camera.stop()
            except Exception:
                pass
            try:
                camera.close()
            except Exception:
                pass
            print("Camera closed.")


if __name__ == "__main__":
    main()
