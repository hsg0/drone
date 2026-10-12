import collections
import os
import subprocess
import sys
import threading
import time

import cv2
import numpy


MEDIA_FOLDER = os.path.join(os.path.dirname(__file__), "..", "media")
VIDEOS_FOLDER = os.path.join(MEDIA_FOLDER, "videos")
PICTURES_FOLDER = os.path.join(MEDIA_FOLDER, "pictures")
VIDEO_PATH = os.path.join(os.path.dirname(__file__), "..", "test_video.mp4")
CAMERA_WIDTH_PIXELS = 640
CAMERA_HEIGHT_PIXELS = 480
CAMERA_FRAMES_PER_SECOND = 10
GESTURE_RECORD_SECONDS = 3
BLUR_KERNEL_SIZE = 15
# A stream with no pause uses a whole Pi core, and then /flight/status stops answering.
STREAM_FRAMES_PER_SECOND = 8
STREAM_FRAME_SECONDS = 1.0 / STREAM_FRAMES_PER_SECOND
# Where rpicam-vid writes its reason when the camera will not start.
RPICAM_COMPLAINT_PATH = "/tmp/oct11-camera.log"

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


def applyEffect(frame, effect, rememberGesture=False):
    if effect == "gray":
        return grayScale(frame)
    if effect == "blur":
        return blur(frame)
    if effect == "gaussian":
        return gaussianBlur(frame)
    if effect == "yellow":
        return yellowBox(frame)
    if effect == "gesture":
        return markGesture(frame, rememberGesture)[0]
    return frame


def jpegBytes(frame):
    encoded, jpeg = cv2.imencode(".jpg", frame)
    if not encoded:
        return None
    return jpeg.tobytes()


# Every reload of the website opens another stream. Old streams keep reading
# frames until the browser notices, and several at once starve this Pi.
streamCountLock = threading.Lock()
newestStreamNumber = 0


def claimNewestStream():
    """Take the newest stream number. Older streams stop when they see a newer one."""
    global newestStreamNumber
    with streamCountLock:
        newestStreamNumber += 1
        return newestStreamNumber


def streamIsNewest(streamNumber):
    with streamCountLock:
        return streamNumber == newestStreamNumber


def currentStreamNumber():
    with streamCountLock:
        return newestStreamNumber


def videoFrames(effect=None):
    """Read test_video.mp4 one frame at a time and hand each frame back as JPEG bytes."""
    capture = cv2.VideoCapture(VIDEO_PATH)
    if not capture.isOpened():
        capture.release()
        return

    streamNumber = claimNewestStream()
    try:
        while streamIsNewest(streamNumber):
            frameStartedAtSeconds = time.monotonic()
            success, frame = capture.read()
            if not success:
                break
            jpeg = jpegBytes(applyEffect(frame, effect))
            if jpeg is None:
                continue
            yield jpeg
            restSeconds = STREAM_FRAME_SECONDS - (time.monotonic() - frameStartedAtSeconds)
            if restSeconds > 0:
                time.sleep(restSeconds)  # leave the Pi enough time to answer the website
    finally:
        capture.release()


class RpicamCamera:
    """Camera pictures from the rpicam-vid program.

    picamera2 belongs to the Pi system Python, not to this virtual environment,
    and /dev/video0 is the raw sensor, which gives no usable picture.
    rpicam-vid writes finished JPEG pictures, so it works from here.
    The method names copy picamera2 so the rest of this file does not change.
    """

    def __init__(self):
        self.complaintFile = open(RPICAM_COMPLAINT_PATH, "w+")
        self.process = subprocess.Popen(
            [
                "rpicam-vid",
                "--codec", "mjpeg",
                "--timeout", "0",  # 0 keeps the camera running until this file stops it
                "--nopreview",
                "--width", str(CAMERA_WIDTH_PIXELS),
                "--height", str(CAMERA_HEIGHT_PIXELS),
                "--framerate", str(CAMERA_FRAMES_PER_SECOND),
                "--output", "-",
            ],
            stdout=subprocess.PIPE,
            stderr=self.complaintFile,
        )
        self.pictureBytes = bytearray()
        self.firstPicture = None
        # Ask for one picture now, so a busy camera is reported before the stream starts.
        self.firstPicture = self.capture_array()

    def cameraComplaint(self):
        """The last thing rpicam-vid said. A camera already in use says so here."""
        self.complaintFile.flush()
        self.complaintFile.seek(0)
        lines = [line.strip() for line in self.complaintFile.readlines() if line.strip()]
        return lines[-1] if lines else "no reason given"

    def capture_array(self):
        """One picture in blue-green-red order, the order OpenCV uses."""
        jpegStartMarker = b"\xff\xd8"
        jpegEndMarker = b"\xff\xd9"
        if self.firstPicture is not None:
            picture = self.firstPicture
            self.firstPicture = None
            return picture
        while True:
            startAt = self.pictureBytes.find(jpegStartMarker)
            endAt = self.pictureBytes.find(jpegEndMarker, startAt + 2) if startAt >= 0 else -1
            if startAt >= 0 and endAt >= 0:
                onePicture = bytes(self.pictureBytes[startAt:endAt + 2])
                del self.pictureBytes[:endAt + 2]
                frame = cv2.imdecode(numpy.frombuffer(onePicture, numpy.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    return frame
                continue
            moreBytes = self.process.stdout.read(4096)
            if not moreBytes:
                raise RuntimeError("The camera sent no pictures: {}".format(self.cameraComplaint()))
            self.pictureBytes.extend(moreBytes)

    def stop(self):
        if self.process.poll() is None:
            self.process.terminate()

    def close(self):
        try:
            self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=3)
        if self.process.stdout is not None:
            self.process.stdout.close()
        self.complaintFile.close()


# The Pi has one camera. A browser that walks away leaves its stream asleep,
# and an asleep stream never closes the camera by itself. So the next viewer
# closes it instead.
liveCameraLock = threading.Lock()
liveCamera = None
liveCameraStreamNumber = None


def takeCameraFromOlderViewer():
    """Close the camera an older viewer left open."""
    global liveCamera
    global liveCameraStreamNumber
    with liveCameraLock:
        olderCamera = liveCamera
        liveCamera = None
        liveCameraStreamNumber = None
    if olderCamera is None:
        return
    print("Closing the camera an older viewer left open.")
    olderCamera.stop()
    olderCamera.close()


def openPiCamera():
    """Take the camera from any older viewer, then open it. BGR frames suit OpenCV."""
    global liveCamera
    global liveCameraStreamNumber
    streamNumber = claimNewestStream()  # the older stream sees a newer number and stops
    takeCameraFromOlderViewer()
    try:
        from picamera2 import Picamera2
    except ImportError:
        print("picamera2 is not in this virtual environment. Using rpicam-vid instead.")
        camera = RpicamCamera()
    else:
        camera = Picamera2()
        configuration = camera.create_preview_configuration(
            main={"size": (CAMERA_WIDTH_PIXELS, CAMERA_HEIGHT_PIXELS), "format": "RGB888"}
        )
        camera.configure(configuration)
        camera.start()
    with liveCameraLock:
        liveCamera = camera
        liveCameraStreamNumber = streamNumber
    return camera


def cameraFrames(camera, effect=None):
    """Hand back one JPEG at a time, then close the camera."""
    streamNumber = currentStreamNumber()  # openPiCamera already claimed this number
    try:
        while streamIsNewest(streamNumber):
            frameStartedAtSeconds = time.monotonic()
            frame = camera.capture_array()
            # Only the live camera may update the pose the website sends to the copter.
            jpeg = jpegBytes(applyEffect(frame, effect, rememberGesture=True))
            if jpeg is None:
                continue
            yield jpeg
            restSeconds = STREAM_FRAME_SECONDS - (time.monotonic() - frameStartedAtSeconds)
            if restSeconds > 0:
                time.sleep(restSeconds)  # leave the Pi enough time to answer the website
    finally:
        rememberSeenGesture("not-seen", "not seen", "no command")
        global liveCamera
        global liveCameraStreamNumber
        with liveCameraLock:
            cameraIsStillOurs = liveCameraStreamNumber == streamNumber
            if cameraIsStillOurs:
                liveCamera = None
                liveCameraStreamNumber = None
        if cameraIsStillOurs:  # a newer viewer may already have closed it
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

# YOLO pose is the detector for the Pi 4, where MediaPipe will not run.
# It reports 17 body points. These five are the ones the gestures need.
YOLO_POSE_MODEL_PATH = os.path.join(os.path.dirname(__file__), "..", "models", "yolo11n-pose.onnx")
YOLO_INPUT_PIXELS = 256
YOLO_PERSON_SCORE_MINIMUM = 0.4
YOLO_KEYPOINT_FOR_LANDMARK = {
    NOSE_LANDMARK: 0,
    LEFT_SHOULDER_LANDMARK: 5,
    RIGHT_SHOULDER_LANDMARK: 6,
    LEFT_WRIST_LANDMARK: 9,
    RIGHT_WRIST_LANDMARK: 10,
}

# One body point, as a share of the picture. MediaPipe hands back the same two names.
BodyPoint = collections.namedtuple("BodyPoint", ["x", "y"])

poseLandmarker = None
yoloPoseSession = None
mediapipeWorksOnThisComputer = None
gestureSeenLock = threading.Lock()
latestSeenGesture = {
    "gesture": "not-seen",
    "pose": "not seen",
    "command": "no command",
}


def rememberSeenGesture(gestureName, seenPose, commandName):
    """Keep the newest live-camera pose. The website reads this and may send it to the copter."""
    global latestSeenGesture
    with gestureSeenLock:
        latestSeenGesture = {
            "gesture": gestureName,
            "pose": seenPose,
            "command": commandName,
        }


def seenGesture():
    """The newest live-camera pose. This does not arm or move the copter."""
    with gestureSeenLock:
        return dict(latestSeenGesture)


def mediapipeCanRun():
    """True when MediaPipe survives one frame. On a Pi 4 it stops the whole program, so ask a separate one."""
    global mediapipeWorksOnThisComputer
    if mediapipeWorksOnThisComputer is not None:
        return mediapipeWorksOnThisComputer
    backendFolder = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    probe = subprocess.run(
        [
            sys.executable,
            "-c",
            "import cv2\n"
            "from controllers.capturePicVidController import mediapipeBodyLandmarks\n"
            "capture = cv2.VideoCapture('test_video.mp4')\n"
            "ok, frame = capture.read()\n"
            "capture.release()\n"
            "if ok:\n"
            "    mediapipeBodyLandmarks(frame)\n"
            "print('mediapipe ok')\n",
        ],
        cwd=backendFolder,
        capture_output=True,
        text=True,
        timeout=60,
    )
    mediapipeWorksOnThisComputer = probe.returncode == 0 and "mediapipe ok" in probe.stdout
    if mediapipeWorksOnThisComputer:
        print("Pose detection is using MediaPipe.")
    else:
        print("MediaPipe will not run on this computer. Pose detection is using YOLO.")
    return mediapipeWorksOnThisComputer


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


def mediapipeBodyLandmarks(frame):
    """Find the body with MediaPipe. Arms straight up can vanish in gray, so color is the backup."""
    import mediapipe as mp

    grayFrame = grayPicture(frame)
    for picture in (cv2.cvtColor(grayFrame, cv2.COLOR_BGR2RGB), cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)):
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=picture)
        poses = poseFinder().detect(image)
        if poses.pose_landmarks:
            return poses.pose_landmarks[0]
    return None


def yoloPoseFinder():
    global yoloPoseSession
    if yoloPoseSession is None:
        import onnxruntime

        yoloPoseSession = onnxruntime.InferenceSession(
            YOLO_POSE_MODEL_PATH,
            providers=["CPUExecutionProvider"],
        )
    return yoloPoseSession


def yoloBodyLandmarks(frame):
    """Find the body with YOLO pose. Returns the five points the gestures need, or None."""
    session = yoloPoseFinder()
    squarePicture = cv2.resize(frame, (YOLO_INPUT_PIXELS, YOLO_INPUT_PIXELS))
    colorPicture = cv2.cvtColor(squarePicture, cv2.COLOR_BGR2RGB).astype(numpy.float32) / 255.0
    # YOLO wants one picture, colors first, then height and width.
    askedPicture = numpy.transpose(colorPicture, (2, 0, 1))[numpy.newaxis, :, :, :]
    answer = session.run(None, {session.get_inputs()[0].name: askedPicture})[0][0]
    # Each column is one guess: 4 box numbers, 1 score, then 17 points of x, y, score.
    personScores = answer[4]
    bestGuess = int(numpy.argmax(personScores))
    if float(personScores[bestGuess]) < YOLO_PERSON_SCORE_MINIMUM:
        return None
    landmarks = {}
    for landmarkNumber, keypointNumber in YOLO_KEYPOINT_FOR_LANDMARK.items():
        firstRow = 5 + keypointNumber * 3
        xPixels = float(answer[firstRow][bestGuess])
        yPixels = float(answer[firstRow + 1][bestGuess])
        # Share of the picture, so the gesture rules read the same as MediaPipe's.
        landmarks[landmarkNumber] = BodyPoint(xPixels / YOLO_INPUT_PIXELS, yPixels / YOLO_INPUT_PIXELS)
    return landmarks


def bodyLandmarks(frame):
    """Find the body with whichever detector this computer can run."""
    if mediapipeCanRun():
        return mediapipeBodyLandmarks(frame)
    return yoloBodyLandmarks(frame)


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


def markGesture(frame, remember=False):
    """Gray the picture, box the body, and name the gesture and the flight command."""
    grayFrame = grayPicture(frame)
    try:
        landmarks = bodyLandmarks(frame)
    except Exception as error:  # a broken detector must not stop the stream
        yellowBoxText(grayFrame, 8, 8, grayFrame.shape[1] - 16, 40, "pose off\nno command")
        print("Pose detection failed: {}".format(error))
        if remember:
            rememberSeenGesture("not-seen", "pose off", "no command")
        return grayFrame, "not-seen"
    if landmarks is None:
        yellowBoxText(grayFrame, 8, 8, grayFrame.shape[1] - 16, 40, "not seen\nno command")
        if remember:
            rememberSeenGesture("not-seen", "not seen", "no command")
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
    if remember:
        rememberSeenGesture(gestureName, seenPose, commandName)
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
