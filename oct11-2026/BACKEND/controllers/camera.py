import cv2


CAMERA_WIDTH_PIXELS = 640
CAMERA_HEIGHT_PIXELS = 480


def open_pi_camera():
    """Open the Raspberry Pi camera. Picamera2 RGB888 frames are BGR for OpenCV."""
    from picamera2 import Picamera2

    camera = Picamera2()
    configuration = camera.create_preview_configuration(
        main={"size": (CAMERA_WIDTH_PIXELS, CAMERA_HEIGHT_PIXELS), "format": "RGB888"}
    )
    camera.configure(configuration)
    camera.start()
    return camera


def camera_frames(camera):
    """Hand back one JPEG at a time, then close the camera."""
    try:
        while True:
            frame = camera.capture_array()
            encoded, jpeg = cv2.imencode(".jpg", frame)
            if not encoded:
                continue
            yield jpeg.tobytes()
    finally:
        camera.stop()
        camera.close()
