#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    Show the forward camera and the distances the drone is reading.

    The window draws the brown box stripe the camera found. The same
    picture lists the camera distance, the sonic distance, and the
    time-of-flight distance.

WHY:
    This is a check, not a flight. Seeing the stripe and the three
    distances together shows the forward camera, the sonic sensor on the
    power board, and the time-of-flight sensor under the drone.

HOW:
    1. Open the forward camera. Do not open USB. Do not talk to the Pixhawk.
    2. Read the sonic and time-of-flight snapshot written by bootDisplay.py.
    3. Find the brown box with the vertical blue stripe.
    4. Draw that stripe and print every distance on the picture.
    5. Repeat until Q or Ctrl+C. Then close the camera.

IMPORTANT:
    Run this on the Pi desktop so the window can open.
    bootDisplay.py must already be running. It owns the RedBoard and writes
    the sonic and time-of-flight snapshot.
    The target is the same brown box with the blue tape running up and down.
    A missing saved camera scale does not stop the view. The raw camera
    distance and the sensor distances still appear.
    The time-of-flight sensor faces forward and backward from the bottom of
    the drone. A missing reading does not hide the camera or the sonic distance.
    A different time-of-flight number is expected, because it is not looking
    from the same place as the sonic sensor.
"""

import os
import sys
import time

import cv2


SCRIPT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
CALIBRATION_DIRECTORY_CANDIDATES = (
    os.path.join(os.path.dirname(SCRIPT_DIRECTORY), "oct7-2026"),
    "/home/drone/Desktop/scripts/oct7-2026",
)

for calibrationDirectory in CALIBRATION_DIRECTORY_CANDIDATES:
    if os.path.isdir(calibrationDirectory) and calibrationDirectory not in sys.path:
        sys.path.insert(0, calibrationDirectory)

import forward_distance_calibration


CAMERA_WINDOW_NAME = "Camera view"
STATUS_PRINT_INTERVAL_SECONDS = 1.0
QUIT_KEY_LETTER = ord("q")

# One missed frame should not flash "missing". A clipped stripe is not kept.
CAMERA_DISTANCE_HOLD_SECONDS = 0.5
CAMERA_DISTANCE_MEDIAN_COUNT = 5

LABEL_LINE_HEIGHT_PIXELS = 22
LABEL_LEFT_PIXELS = 8


def distanceText(distanceMeters):
    return forward_distance_calibration.formatMeters(distanceMeters)


def drawLabels(frame, lines):
    """Put the words on a black band at the bottom so they do not cover the stripe."""
    bandHeightPixels = (LABEL_LINE_HEIGHT_PIXELS * len(lines)) + 10
    bandTopPixels = frame.shape[0] - bandHeightPixels
    cv2.rectangle(
        frame,
        (0, bandTopPixels),
        (frame.shape[1], frame.shape[0]),
        (0, 0, 0),
        -1,
    )
    for lineNumber, line in enumerate(lines):
        baselinePixels = bandTopPixels + 18 + (lineNumber * LABEL_LINE_HEIGHT_PIXELS)
        cv2.putText(
            frame,
            line,
            (LABEL_LEFT_PIXELS, baselinePixels),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )


def drawStripe(frame, markerCorners):
    stripeX = int(markerCorners[0][0])
    stripeY = int(markerCorners[0][1])
    stripeRight = int(markerCorners[2][0])
    stripeBottom = int(markerCorners[2][1])
    cv2.rectangle(frame, (stripeX, stripeY), (stripeRight, stripeBottom), (0, 255, 0), 2)


def medianDistanceMeters(recentDistancesMeters):
    orderedDistancesMeters = sorted(recentDistancesMeters)
    middleIndex = len(orderedDistancesMeters) // 2
    if len(orderedDistancesMeters) % 2 == 1:
        return orderedDistancesMeters[middleIndex]
    return (
        orderedDistancesMeters[middleIndex - 1]
        + orderedDistancesMeters[middleIndex]
    ) / 2.0


def viewLines(
    rawCameraMeters,
    markerCorners,
    stripeIsNearEdge,
    sawBlueWithoutBrownBox,
    holdingLastDistance,
    calibration,
    snapshot,
):
    scaledCameraMeters = forward_distance_calibration.scaledCameraDistanceMeters(
        rawCameraMeters,
        calibration,
    )
    sonicMeters = forward_distance_calibration.sonicDistanceMeters(snapshot)
    timeOfFlightMeters = forward_distance_calibration.timeOfFlightDistanceMeters(snapshot)

    if stripeIsNearEdge:
        stripeText = "Stripe is cut off by the frame. Distance not used."
    elif holdingLastDistance:
        stripeText = "Holding the last full stripe"
    elif markerCorners is not None:
        stripeText = "Camera sees the brown box"
    elif sawBlueWithoutBrownBox:
        stripeText = "Camera sees blue, not on the brown box"
    else:
        stripeText = "Camera does not see the brown box"

    lines = [
        stripeText,
        f"Camera raw {distanceText(rawCameraMeters)}",
        f"Camera scaled {distanceText(scaledCameraMeters)}",
        f"Sonic, power board, forward {distanceText(sonicMeters)}",
        f"Time of flight, bottom, forward and backward {distanceText(timeOfFlightMeters)}",
    ]

    if timeOfFlightMeters is None:
        lines.append("Time of flight is not answering")
    if sonicMeters is None:
        lines.append("Sonic reading is missing")

    if calibration is None:
        lines.append("No saved camera scale")

    return lines


def showCameraView(camera, calibration):
    cv2.namedWindow(CAMERA_WINDOW_NAME, cv2.WINDOW_NORMAL)
    lastPrintedAtSeconds = 0.0
    recentRawCameraMeters = []
    lastAcceptedRawCameraMeters = None
    lastAcceptedAtSeconds = 0.0

    while True:
        # Copy first. Drawing on the camera buffer makes the next frame see our own text.
        frame = camera.capture_array().copy()
        markerCorners, sawBlueWithoutBrownBox = forward_distance_calibration.blueStripeCorners(frame)
        stripeIsNearEdge = False
        measuredRawCameraMeters = None
        if markerCorners is not None:
            stripeIsNearEdge = forward_distance_calibration.markerIsNearFrameEdge(markerCorners)
            drawStripe(frame, markerCorners)
            if not stripeIsNearEdge:
                measuredRawCameraMeters = forward_distance_calibration.rawCameraDistanceMeters(markerCorners)

        currentTimeSeconds = time.monotonic()
        holdingLastDistance = False
        if measuredRawCameraMeters is not None:
            recentRawCameraMeters.append(measuredRawCameraMeters)
            if len(recentRawCameraMeters) > CAMERA_DISTANCE_MEDIAN_COUNT:
                recentRawCameraMeters.pop(0)
            rawCameraMeters = medianDistanceMeters(recentRawCameraMeters)
            lastAcceptedRawCameraMeters = rawCameraMeters
            lastAcceptedAtSeconds = currentTimeSeconds
        elif stripeIsNearEdge:
            recentRawCameraMeters.clear()
            rawCameraMeters = None
        elif (
            lastAcceptedRawCameraMeters is not None
            and currentTimeSeconds - lastAcceptedAtSeconds <= CAMERA_DISTANCE_HOLD_SECONDS
        ):
            rawCameraMeters = lastAcceptedRawCameraMeters
            holdingLastDistance = True
        else:
            recentRawCameraMeters.clear()
            rawCameraMeters = None

        snapshot = forward_distance_calibration.readForwardRangeSnapshot()
        lines = viewLines(
            rawCameraMeters,
            markerCorners,
            stripeIsNearEdge,
            sawBlueWithoutBrownBox,
            holdingLastDistance,
            calibration,
            snapshot,
        )

        drawLabels(frame, lines)

        cv2.imshow(CAMERA_WINDOW_NAME, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == QUIT_KEY_LETTER or key == 27:
            print("Closing the camera view.")
            return

        currentTimeSeconds = time.monotonic()
        if currentTimeSeconds - lastPrintedAtSeconds >= STATUS_PRINT_INTERVAL_SECONDS:
            print(" | ".join(lines))
            lastPrintedAtSeconds = currentTimeSeconds


def main():
    camera = None

    try:
        print()
        print("==============================================")
        print("CAMERA VIEW")
        print("==============================================")
        print()
        print("The window shows the camera picture and the distances.")
        print("Press Q in the window to close it.")
        print()

        calibration = forward_distance_calibration.loadForwardDistanceCalibration()
        if calibration is None:
            print(forward_distance_calibration.savedCalibrationRefusalReason())
            print("The view will still show the raw camera distance and the sensors.")
        else:
            cameraScale = calibration.get("cameraScale")
            print(f"Saved camera scale: {cameraScale:.3f}")

        camera = forward_distance_calibration.openForwardCamera()
        showCameraView(camera, calibration)

    except KeyboardInterrupt:
        print()
        print("Ctrl+C. Closing the camera view.")

    except Exception as error:
        print()
        print(f"STOPPED: {error}")

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
        cv2.destroyAllWindows()
        print("Finished.")


if __name__ == "__main__":
    main()
