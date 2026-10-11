import os

from fastapi import APIRouter
from fastapi import HTTPException
from fastapi.responses import StreamingResponse

from controllers.camera import camera_frames
from controllers.camera import open_pi_camera
from controllers.video import VIDEO_PATH
from controllers.video import video_frames

web_routes = APIRouter()


def jpeg_parts(frames):
    for frame in frames:
        yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"


@web_routes.get("/video")
def watch_video():
    if not os.path.isfile(VIDEO_PATH):
        raise HTTPException(status_code=404, detail="Missing video file: test_video.mp4")
    return StreamingResponse(
        jpeg_parts(video_frames()),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


@web_routes.get("/camera")
def watch_camera():
    try:
        camera = open_pi_camera()
    except Exception as error:
        raise HTTPException(status_code=503, detail="Camera did not open: {}".format(error))
    return StreamingResponse(
        jpeg_parts(camera_frames(camera)),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )
