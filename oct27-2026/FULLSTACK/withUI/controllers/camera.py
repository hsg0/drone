# WHAT — open the forward camera and name the pose in the picture.
# WHY — the gesture page needs the live picture, and the same six arm poses
#       October already used. YOLO finds the nose, shoulders, and wrists.
#       MediaPipe is not used. This file does not arm or fly.
# HOW — open the Pi camera, or the saved clip when the camera is not here.
#       Draw the pose on each frame and remember the newest name.
#       The gesture page receives those frames over WebRTC.
#       One viewer at a time. A newer viewer closes the camera an older one left open.
# IMPORTANT — the withUI environment often has no Picamera2. rpicam-vid is the
#             fallback, and it exists on the Pi, not on the desktop.
#             Close camera-view.py and follow-me.py before opening this camera.
#             This file does not open /dev/serial0.

import asyncio
import collections
import os
import subprocess
import threading
import time

import cv2
import numpy
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.mediastreams import VIDEO_CLOCK_RATE, VIDEO_TIME_BASE, MediaStreamError
from av import VideoFrame


CAMERA_WIDTH_PIXELS = 640
CAMERA_HEIGHT_PIXELS = 480
CAMERA_FRAMES_PER_SECOND = 10
# A stream with no pause uses a whole Pi core, and then the other pages stop answering.
STREAM_FRAMES_PER_SECOND = 8
STREAM_FRAME_SECONDS = 1.0 / STREAM_FRAMES_PER_SECOND
RPICAM_COMPLAINT_PATH = "/tmp/withui-camera.log"
SAMPLE_VIDEO_PATH = os.path.join(os.path.dirname(__file__), "..", "test_video.mp4")

YELLOW_BOX_COLOR_BGR = (0, 255, 255)
YELLOW_BOX_THICKNESS_PIXELS = 3
YELLOW_BOX_TEXT_SCALE = 0.6
YELLOW_BOX_TEXT_THICKNESS_PIXELS = 2
YELLOW_BOX_TEXT_PADDING_PIXELS = 4

# Pose numbers from the body model. Y grows downward, so a smaller Y is higher.
NOSE_LANDMARK = 0
LEFT_SHOULDER_LANDMARK = 11
RIGHT_SHOULDER_LANDMARK = 12
LEFT_WRIST_LANDMARK = 15
RIGHT_WRIST_LANDMARK = 16
GESTURE_BOX_PADDING_PIXELS = 16

# YOLO pose reports 17 body points. These five are the ones the gestures need.
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

BodyPoint = collections.namedtuple("BodyPoint", ["x", "y"])

yoloPoseSession = None
gestureSeenLock = threading.Lock()
latestSeenGesture = {
    "gesture": "not-seen",
    "pose": "not seen",
    "command": "no command",
}

streamCountLock = threading.Lock()
newestStreamNumber = 0
liveCameraLock = threading.Lock()
liveCamera = None
liveCameraStreamNumber = None


def rememberSeenGesture(gestureName, seenPose, commandName):
    """Keep the newest pose. Reading it does not arm or move the copter."""
    global latestSeenGesture
    with gestureSeenLock:
        latestSeenGesture = {
            "gesture": gestureName,
            "pose": seenPose,
            "command": commandName,
        }


def seenGesture():
    """The newest pose from the camera or the saved clip."""
    with gestureSeenLock:
        return dict(latestSeenGesture)


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


def jpegBytes(frame):
    encoded, jpeg = cv2.imencode(".jpg", frame)
    if not encoded:
        return None
    return jpeg.tobytes()


def writePoseLabel(frame, itemXPixels, itemYPixels, itemWidthPixels, label):
    """Write the pose and its command on the picture."""
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


class RpicamCamera:
    """Pictures from rpicam-vid. Picamera2 is often missing from this virtual environment."""

    def __init__(self):
        self.complaintFile = open(RPICAM_COMPLAINT_PATH, "w+")
        self.process = subprocess.Popen(
            [
                "rpicam-vid",
                "--codec", "mjpeg",
                "--timeout", "0",
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
    """Take the camera from any older viewer, then open it."""
    global liveCamera
    global liveCameraStreamNumber
    streamNumber = claimNewestStream()
    takeCameraFromOlderViewer()
    try:
        from picamera2 import Picamera2
    except ImportError:
        print("Picamera2 is not in this environment. Using rpicam-vid.")
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


def yoloPoseFinder():
    global yoloPoseSession
    if yoloPoseSession is None:
        import onnxruntime

        if not os.path.isfile(YOLO_POSE_MODEL_PATH):
            raise FileNotFoundError("Missing pose model: {}".format(YOLO_POSE_MODEL_PATH))
        yoloPoseSession = onnxruntime.InferenceSession(
            YOLO_POSE_MODEL_PATH,
            providers=["CPUExecutionProvider"],
        )
        print("Pose detection is using YOLO.")
    return yoloPoseSession


def yoloBodyLandmarks(frame):
    """Find the body with YOLO. Returns the five points the gestures need, or None."""
    session = yoloPoseFinder()
    squarePicture = cv2.resize(frame, (YOLO_INPUT_PIXELS, YOLO_INPUT_PIXELS))
    colorPicture = cv2.cvtColor(squarePicture, cv2.COLOR_BGR2RGB).astype(numpy.float32) / 255.0
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
        landmarks[landmarkNumber] = BodyPoint(xPixels / YOLO_INPUT_PIXELS, yPixels / YOLO_INPUT_PIXELS)
    return landmarks


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


def grayPicture(frame):
    """Gray picture kept in three channels so the yellow box stays yellow."""
    grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return cv2.cvtColor(grayFrame, cv2.COLOR_GRAY2BGR)


def markGesture(frame):
    """Gray the picture, box the body, and name the gesture."""
    grayFrame = grayPicture(frame)
    try:
        landmarks = yoloBodyLandmarks(frame)
    except Exception as error:
        writePoseLabel(grayFrame, 8, 8, grayFrame.shape[1] - 16, "pose off\nno command")
        print("Pose detection failed: {}".format(error))
        rememberSeenGesture("not-seen", "pose off", "no command")
        return grayFrame
    if landmarks is None:
        writePoseLabel(grayFrame, 8, 8, grayFrame.shape[1] - 16, "not seen\nno command")
        rememberSeenGesture("not-seen", "not seen", "no command")
        return grayFrame

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
    rememberSeenGesture(gestureName, seenPose, commandName)
    cv2.rectangle(
        grayFrame,
        (itemXPixels, itemYPixels),
        (itemRightPixels, itemBottomPixels),
        YELLOW_BOX_COLOR_BGR,
        YELLOW_BOX_THICKNESS_PIXELS,
    )
    writePoseLabel(
        grayFrame,
        itemXPixels,
        itemYPixels,
        itemRightPixels - itemXPixels,
        seenPose + "\n" + commandName,
    )
    return grayFrame


def pictureForViewer(frame, labelPose):
    if labelPose:
        return markGesture(frame)
    return frame


def cameraFrames(camera, labelPose):
    """Hand back one JPEG at a time, then close the camera."""
    streamNumber = currentStreamNumber()
    try:
        while streamIsNewest(streamNumber):
            frameStartedAtSeconds = time.monotonic()
            frame = camera.capture_array()
            jpeg = jpegBytes(pictureForViewer(frame, labelPose))
            if jpeg is None:
                continue
            yield jpeg
            restSeconds = STREAM_FRAME_SECONDS - (time.monotonic() - frameStartedAtSeconds)
            if restSeconds > 0:
                time.sleep(restSeconds)
    finally:
        rememberSeenGesture("not-seen", "not seen", "no command")
        global liveCamera
        global liveCameraStreamNumber
        with liveCameraLock:
            cameraIsStillOurs = liveCameraStreamNumber == streamNumber
            if cameraIsStillOurs:
                liveCamera = None
                liveCameraStreamNumber = None
        if cameraIsStillOurs:
            camera.stop()
            camera.close()


def openSampleClip():
    """Open the saved clip before the stream starts, so a missing file is reported at once."""
    if not os.path.isfile(SAMPLE_VIDEO_PATH):
        raise FileNotFoundError("Missing saved clip: {}".format(SAMPLE_VIDEO_PATH))
    capture = cv2.VideoCapture(SAMPLE_VIDEO_PATH)
    if not capture.isOpened():
        capture.release()
        raise RuntimeError("The saved clip did not open.")
    return capture


def sampleVideoFrames():
    """Play the saved clip through the same pose labeler. Loop it so the page stays up."""
    capture = openSampleClip()
    streamNumber = claimNewestStream()
    try:
        while streamIsNewest(streamNumber):
            frameStartedAtSeconds = time.monotonic()
            success, frame = capture.read()
            if not success:
                capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
                time.sleep(STREAM_FRAME_SECONDS)
                continue
            jpeg = jpegBytes(markGesture(frame))
            if jpeg is None:
                continue
            yield jpeg
            restSeconds = STREAM_FRAME_SECONDS - (time.monotonic() - frameStartedAtSeconds)
            if restSeconds > 0:
                time.sleep(restSeconds)
    finally:
        rememberSeenGesture("not-seen", "not seen", "no command")
        capture.release()


# ---------------------------------------------------------------------------
# WebRTC. One peer at a time. The page sends an offer and plays the answer.
# ---------------------------------------------------------------------------

cameraPeer = None
cameraPeerLock = asyncio.Lock()
ICE_GATHER_TIMEOUT_SECONDS = 5


def openCameraOrSample():
    """Open the forward camera. The saved clip is the picture when that camera is absent."""
    try:
        camera = openPiCamera()
        print("WebRTC is using the forward camera.")
        return camera, None, "camera"
    except Exception as error:
        print(f"Live camera did not open: {error}")
    capture = openSampleClip()
    print("WebRTC is using the saved clip.")
    return None, capture, "saved clip"


def releaseLiveCamera(camera):
    """Stop this camera only when it is still the one this program opened."""
    global liveCamera
    global liveCameraStreamNumber
    with liveCameraLock:
        if liveCamera is camera:
            liveCamera = None
            liveCameraStreamNumber = None
    try:
        camera.stop()
        camera.close()
    except Exception as error:
        print(f"Camera close failed: {error}")


class GestureCameraTrack(VideoStreamTrack):
    """Frames from the forward camera or the saved clip, with the pose drawn on."""

    def __init__(self):
        super().__init__()
        self.sourceName = None
        self.openError = None
        self.latestLabeledFrame = None
        self.opened = threading.Event()
        self.frameReady = threading.Event()
        self.stopReading = threading.Event()
        self.frameTimestamp = None
        self.frameStartedAt = None
        self.readerThread = threading.Thread(target=self.readLoop, daemon=True)

    def startPicture(self):
        """Open the picture on the reader thread, then return where it came from."""
        self.readerThread.start()
        self.opened.wait(20)
        if self.openError is not None:
            raise self.openError
        return self.sourceName

    def readOnePicture(self, camera, capture):
        if camera is not None:
            return camera.capture_array()
        success, frame = capture.read()
        if not success:
            capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
            time.sleep(STREAM_FRAME_SECONDS)
            success, frame = capture.read()
        if not success or frame is None:
            raise RuntimeError("The saved clip sent no picture.")
        return frame

    def readLoop(self):
        """Open and read on one thread. OpenCV can stall when those happen on different threads."""
        camera = None
        capture = None
        try:
            camera, capture, self.sourceName = openCameraOrSample()
            self.opened.set()
            while not self.stopReading.is_set():
                frame = self.readOnePicture(camera, capture)
                self.latestLabeledFrame = markGesture(frame)
                self.frameReady.set()
        except Exception as error:
            self.openError = error
            self.opened.set()
            print(f"WebRTC picture stopped: {error}")
        finally:
            if camera is not None:
                releaseLiveCamera(camera)
            if capture is not None:
                capture.release()
            rememberSeenGesture("not-seen", "not seen", "no command")

    def waitForFrame(self):
        if not self.frameReady.wait(15):
            raise RuntimeError("No picture arrived.")
        self.frameReady.clear()
        return self.latestLabeledFrame

    async def nextTimestamp(self):
        """Pace the picture at the same 8 frames a second as the JPEG stream."""
        if self.readyState != "live":
            raise MediaStreamError
        if self.frameTimestamp is not None:
            self.frameTimestamp += int(STREAM_FRAME_SECONDS * VIDEO_CLOCK_RATE)
            waitSeconds = self.frameStartedAt + (self.frameTimestamp / VIDEO_CLOCK_RATE) - time.time()
            if waitSeconds > 0:
                await asyncio.sleep(waitSeconds)
        else:
            self.frameStartedAt = time.time()
            self.frameTimestamp = 0
        return self.frameTimestamp, VIDEO_TIME_BASE

    async def recv(self):
        if self.readyState != "live":
            raise MediaStreamError
        pts, timeBase = await self.nextTimestamp()
        labeledFrame = await asyncio.to_thread(self.waitForFrame)
        videoFrame = VideoFrame.from_ndarray(labeledFrame, format="bgr24")
        videoFrame.pts = pts
        videoFrame.time_base = timeBase
        return videoFrame

    def stop(self):
        super().stop()
        self.stopReading.set()
        if self.readerThread.is_alive() and threading.current_thread() is not self.readerThread:
            self.readerThread.join(timeout=5)


async def waitUntilIceGathered(peer):
    """Wait until the answer includes the local addresses the browser can reach."""
    if peer.iceGatheringState == "complete":
        return
    gathered = asyncio.Event()

    @peer.on("icegatheringstatechange")
    def iceGatheringStateChanged():
        if peer.iceGatheringState == "complete":
            gathered.set()

    if peer.iceGatheringState == "complete":
        return
    try:
        await asyncio.wait_for(gathered.wait(), ICE_GATHER_TIMEOUT_SECONDS)
    except asyncio.TimeoutError:
        print("WebRTC ICE gathering did not finish. Sending the candidates gathered so far.")


async def closeCameraPeerUnlocked():
    """Close the current peer. Caller must already hold cameraPeerLock."""
    global cameraPeer
    peer = cameraPeer
    cameraPeer = None
    if peer is None:
        return {"closed": True}
    await peer.close()
    return {"closed": True}


async def closeCameraConnection():
    """Close the WebRTC connection and the camera it opened."""
    async with cameraPeerLock:
        return await closeCameraPeerUnlocked()


async def answerCameraOffer(offerSdp, offerType):
    """Accept one browser offer and answer with the camera track."""
    global cameraPeer
    async with cameraPeerLock:
        await closeCameraPeerUnlocked()
        peer = RTCPeerConnection()
        track = GestureCameraTrack()

        @peer.on("connectionstatechange")
        def connectionStateChanged():
            print(f"WebRTC connection {peer.connectionState}.")
            if peer.connectionState in ("failed", "closed"):
                track.stop()

        try:
            sourceName = await asyncio.to_thread(track.startPicture)
            peer.addTrack(track)
            await peer.setRemoteDescription(RTCSessionDescription(sdp=offerSdp, type=offerType))
            answer = await peer.createAnswer()
            await peer.setLocalDescription(answer)
            await waitUntilIceGathered(peer)
        except Exception:
            track.stop()
            await peer.close()
            raise
        cameraPeer = peer
        return {
            "sdp": peer.localDescription.sdp,
            "type": peer.localDescription.type,
            "source": sourceName,
        }
