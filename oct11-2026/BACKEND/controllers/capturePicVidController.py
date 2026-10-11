import os
import time

import cv2


MEDIA_FOLDER = os.path.join(os.path.dirname(__file__), "..", "media")
VIDEOS_FOLDER = os.path.join(MEDIA_FOLDER, "videos")
PICTURES_FOLDER = os.path.join(MEDIA_FOLDER, "pictures")
VIDEO_PATH = os.path.join(os.path.dirname(__file__), "..", "test_video.mp4")
CAMERA_WIDTH_PIXELS = 640
CAMERA_HEIGHT_PIXELS = 480
CAMERA_FRAMES_PER_SECOND = 10
GESTURE_RECORD_SECONDS = 3
BLUR_KERNEL_SIZE = 15

# OpenCV hue runs from 0 to 179. Yellow sits in this band.
YELLOW_HUE_MINIMUM = 20
YELLOW_HUE_MAXIMUM = 35
YELLOW_SATURATION_MINIMUM = 80
YELLOW_SATURATION_MAXIMUM = 255
YELLOW_VALUE_MINIMUM = 80
YELLOW_VALUE_MAXIMUM = 255
YELLOW_BOX_MINIMUM_AREA_PIXELS = 500
YELLOW_BOX_COLOR_BGR = (0, 255, 255)
YELLOW_BOX_THICKNESS_PIXELS = 3
YELLOW_BOX_TEXT = "yellow item"
YELLOW_BOX_TEXT_SCALE = 0.6
YELLOW_BOX_TEXT_THICKNESS_PIXELS = 2
YELLOW_BOX_TEXT_PADDING_PIXELS = 4


def grayScale(frame):
    """Turn a color frame into shades of gray."""
    return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


def blur(frame):
    """Soften a frame by averaging nearby pixels."""
    return cv2.blur(frame, (BLUR_KERNEL_SIZE, BLUR_KERNEL_SIZE))


def gaussianBlur(frame):
    """Soften a frame with a stronger, smoother blur."""
    return cv2.GaussianBlur(frame, (BLUR_KERNEL_SIZE, BLUR_KERNEL_SIZE), 0)


def yellowBoxText(frame, itemXPixels, itemYPixels, itemWidthPixels, itemHeightPixels, detectedName=None):
    """Write what was detected at the top right of the yellow box."""
    label = YELLOW_BOX_TEXT if detectedName is None else detectedName
    lineYPixels = itemYPixels + YELLOW_BOX_TEXT_PADDING_PIXELS
    for line in label.split("\n"):
        (textWidthPixels, textHeightPixels), _baseline = cv2.getTextSize(
            line,
            cv2.FONT_HERSHEY_SIMPLEX,
            YELLOW_BOX_TEXT_SCALE,
            YELLOW_BOX_TEXT_THICKNESS_PIXELS,
        )
        textXPixels = itemXPixels + itemWidthPixels - textWidthPixels - YELLOW_BOX_TEXT_PADDING_PIXELS
        textYPixels = lineYPixels + textHeightPixels
        if textXPixels < 0:
            textXPixels = YELLOW_BOX_TEXT_PADDING_PIXELS
        cv2.putText(
            frame,
            line,
            (textXPixels, textYPixels),
            cv2.FONT_HERSHEY_SIMPLEX,
            YELLOW_BOX_TEXT_SCALE,
            YELLOW_BOX_COLOR_BGR,
            YELLOW_BOX_TEXT_THICKNESS_PIXELS,
        )
        lineYPixels = textYPixels + YELLOW_BOX_TEXT_PADDING_PIXELS


def yellowBox(frame):
    """Find the largest yellow item, draw a yellow box around it, and crop to that box."""
    hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    yellowMask = cv2.inRange(
        hsvFrame,
        (YELLOW_HUE_MINIMUM, YELLOW_SATURATION_MINIMUM, YELLOW_VALUE_MINIMUM),
        (YELLOW_HUE_MAXIMUM, YELLOW_SATURATION_MAXIMUM, YELLOW_VALUE_MAXIMUM),
    )
    contours, _hierarchy = cv2.findContours(yellowMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return frame

    largestContour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largestContour) < YELLOW_BOX_MINIMUM_AREA_PIXELS:
        return frame

    itemXPixels, itemYPixels, itemWidthPixels, itemHeightPixels = cv2.boundingRect(largestContour)
    boxedFrame = frame.copy()
    cv2.rectangle(
        boxedFrame,
        (itemXPixels, itemYPixels),
        (itemXPixels + itemWidthPixels, itemYPixels + itemHeightPixels),
        YELLOW_BOX_COLOR_BGR,
        YELLOW_BOX_THICKNESS_PIXELS,
    )
    yellowBoxText(boxedFrame, itemXPixels, itemYPixels, itemWidthPixels, itemHeightPixels)
    return boxedFrame[
        itemYPixels:itemYPixels + itemHeightPixels,
        itemXPixels:itemXPixels + itemWidthPixels,
    ]


def applyEffect(frame, effect):
    if effect == "gray":
        return grayScale(frame)
    if effect == "blur":
        return blur(frame)
    if effect == "gaussian":
        return gaussianBlur(frame)
    if effect == "yellow":
        return yellowBox(frame)
    if effect == "gesture":
        return markGesture(frame)[0]
    return frame


def jpegBytes(frame):
    encoded, jpeg = cv2.imencode(".jpg", frame)
    if not encoded:
        return None
    return jpeg.tobytes()


def videoFrames(effect=None):
    """Read test_video.mp4 one frame at a time and hand each frame back as JPEG bytes."""
    capture = cv2.VideoCapture(VIDEO_PATH)
    if not capture.isOpened():
        capture.release()
        return

    try:
        while True:
            success, frame = capture.read()
            if not success:
                break
            jpeg = jpegBytes(applyEffect(frame, effect))
            if jpeg is None:
                continue
            yield jpeg
    finally:
        capture.release()


def openPiCamera():
    """Open the Raspberry Pi camera. Picamera2 RGB888 frames are BGR for OpenCV."""
    from picamera2 import Picamera2

    camera = Picamera2()
    configuration = camera.create_preview_configuration(
        main={"size": (CAMERA_WIDTH_PIXELS, CAMERA_HEIGHT_PIXELS), "format": "RGB888"}
    )
    camera.configure(configuration)
    camera.start()
    return camera


def cameraFrames(camera, effect=None):
    """Hand back one JPEG at a time, then close the camera."""
    try:
        while True:
            frame = camera.capture_array()
            jpeg = jpegBytes(applyEffect(frame, effect))
            if jpeg is None:
                continue
            yield jpeg
    finally:
        camera.stop()
        camera.close()


# Pose numbers from the body model. Y grows downward, so a smaller Y is higher.
NOSE_LANDMARK = 0
LEFT_SHOULDER_LANDMARK = 11
RIGHT_SHOULDER_LANDMARK = 12
LEFT_WRIST_LANDMARK = 15
RIGHT_WRIST_LANDMARK = 16
POSE_MODEL_PATH = os.path.join(os.path.dirname(__file__), "..", "models", "pose_landmarker_lite.task")
GESTURE_BOX_PADDING_PIXELS = 16
poseLandmarker = None


def grayPicture(frame):
    """Gray picture kept in three channels so the yellow box stays yellow."""
    grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return cv2.cvtColor(grayFrame, cv2.COLOR_GRAY2BGR)


def poseFinder():
    global poseLandmarker
    if poseLandmarker is None:
        import mediapipe as mp
        from mediapipe.tasks.python.core import base_options as base_options_module
        from mediapipe.tasks.python import vision

        poseLandmarker = vision.PoseLandmarker.create_from_options(
            vision.PoseLandmarkerOptions(
                base_options=base_options_module.BaseOptions(model_asset_path=POSE_MODEL_PATH),
                num_poses=1,
            )
        )
    return poseLandmarker


def bodyLandmarks(frame):
    """Find the body. Arms straight up can vanish in gray, so the color frame is the backup."""
    import mediapipe as mp

    grayFrame = grayPicture(frame)
    for picture in (cv2.cvtColor(grayFrame, cv2.COLOR_BGR2RGB), cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)):
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=picture)
        poses = poseFinder().detect(image)
        if poses.pose_landmarks:
            return poses.pose_landmarks[0]
    return None


def gestureFromArms(noseY, shoulderY, wristY, shoulderSpread, wristSpread):
    """Turn wrist height and width into the command these poses mean."""
    wristsAboveHead = wristY < noseY - 0.08
    wristsWide = wristSpread > shoulderSpread * 2.2
    wristsClose = wristSpread < shoulderSpread * 2.0
    if wristsAboveHead and wristsWide:
        return "takeoff-land", "wide arms overhead", "take off or land"
    if wristsAboveHead:
        return "fly-up", "arms up", "fly up"
    if wristY < shoulderY and wristsClose:
        return "move-closer", "hands in", "fly closer"
    if wristY <= shoulderY + 0.06 and wristSpread > shoulderSpread * 2.5:
        return "move-away", "arms out", "fly farther"
    if wristY > shoulderY + 0.15:
        return "hover", "arms down", "hover"
    return "fly-down", "arms coming down", "fly down"


def markGesture(frame):
    """Gray the picture, box the body, and name the gesture and the flight command."""
    grayFrame = grayPicture(frame)
    landmarks = bodyLandmarks(frame)
    if landmarks is None:
        yellowBoxText(grayFrame, 8, 8, grayFrame.shape[1] - 16, 40, "not seen\nno command")
        return grayFrame, "not-seen"

    heightPixels, widthPixels = grayFrame.shape[:2]
    usedLandmarks = (
        NOSE_LANDMARK,
        LEFT_SHOULDER_LANDMARK,
        RIGHT_SHOULDER_LANDMARK,
        LEFT_WRIST_LANDMARK,
        RIGHT_WRIST_LANDMARK,
    )
    xs = [landmarks[index].x * widthPixels for index in usedLandmarks]
    ys = [landmarks[index].y * heightPixels for index in usedLandmarks]
    itemXPixels = max(0, int(min(xs)) - GESTURE_BOX_PADDING_PIXELS)
    itemYPixels = max(0, int(min(ys)) - GESTURE_BOX_PADDING_PIXELS)
    itemRightPixels = min(widthPixels - 1, int(max(xs)) + GESTURE_BOX_PADDING_PIXELS)
    itemBottomPixels = min(heightPixels - 1, int(max(ys)) + GESTURE_BOX_PADDING_PIXELS)
    noseY = landmarks[NOSE_LANDMARK].y
    shoulderY = (landmarks[LEFT_SHOULDER_LANDMARK].y + landmarks[RIGHT_SHOULDER_LANDMARK].y) / 2
    wristY = (landmarks[LEFT_WRIST_LANDMARK].y + landmarks[RIGHT_WRIST_LANDMARK].y) / 2
    shoulderSpread = abs(landmarks[LEFT_SHOULDER_LANDMARK].x - landmarks[RIGHT_SHOULDER_LANDMARK].x)
    wristSpread = abs(landmarks[LEFT_WRIST_LANDMARK].x - landmarks[RIGHT_WRIST_LANDMARK].x)
    gestureName, seenPose, commandName = gestureFromArms(
        noseY, shoulderY, wristY, shoulderSpread, wristSpread
    )
    cv2.rectangle(
        grayFrame,
        (itemXPixels, itemYPixels),
        (itemRightPixels, itemBottomPixels),
        YELLOW_BOX_COLOR_BGR,
        YELLOW_BOX_THICKNESS_PIXELS,
    )
    yellowBoxText(
        grayFrame,
        itemXPixels,
        itemYPixels,
        itemRightPixels - itemXPixels,
        itemBottomPixels - itemYPixels,
        seenPose + "\n" + commandName,
    )
    return grayFrame, gestureName


def recordGestureVideo():
    """Record a short camera clip into media/videos. Ten frames a second keeps each frame different enough to label."""
    os.makedirs(VIDEOS_FOLDER, exist_ok=True)
    gestureName = time.strftime("gesture-%Y-%m-%d-%H%M%S")
    videoPath = os.path.join(VIDEOS_FOLDER, gestureName + ".mp4")
    camera = openPiCamera()
    writer = cv2.VideoWriter(
        videoPath,
        cv2.VideoWriter_fourcc(*"mp4v"),
        CAMERA_FRAMES_PER_SECOND,
        (CAMERA_WIDTH_PIXELS, CAMERA_HEIGHT_PIXELS),
    )
    frameCount = int(GESTURE_RECORD_SECONDS * CAMERA_FRAMES_PER_SECOND)
    try:
        for _frameNumber in range(frameCount):
            frame = camera.capture_array()
            writer.write(frame)
            time.sleep(1 / CAMERA_FRAMES_PER_SECOND)
    finally:
        writer.release()
        camera.stop()
        camera.close()
    return videoPath


def saveVideoFrames(videoPath):
    """Save every frame of that clip into media/pictures, in a folder named after the video."""
    gestureName = os.path.splitext(os.path.basename(videoPath))[0]
    pictureFolder = os.path.join(PICTURES_FOLDER, gestureName)
    os.makedirs(pictureFolder, exist_ok=True)
    capture = cv2.VideoCapture(videoPath)
    frameNumber = 0
    try:
        while True:
            success, frame = capture.read()
            if not success:
                break
            frameNumber += 1
            framePath = os.path.join(pictureFolder, "frame-{:05d}.jpg".format(frameNumber))
            cv2.imwrite(framePath, frame)
    finally:
        capture.release()
    return pictureFolder, frameNumber
