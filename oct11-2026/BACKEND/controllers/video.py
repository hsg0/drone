import os

import cv2


VIDEO_PATH = os.path.join(os.path.dirname(__file__), "..", "test_video.mp4")


def video_frames():
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
            encoded, jpeg = cv2.imencode(".jpg", frame)
            if not encoded:
                continue
            yield jpeg.tobytes()
    finally:
        capture.release()
