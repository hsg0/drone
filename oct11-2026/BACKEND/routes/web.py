import os

from fastapi import APIRouter
from fastapi import HTTPException
from fastapi.responses import StreamingResponse

from controllers.capturePicVidController import VIDEO_PATH
from controllers.capturePicVidController import cameraFrames
from controllers.capturePicVidController import openPiCamera
from controllers.capturePicVidController import recordGestureVideo
from controllers.capturePicVidController import saveVideoFrames
from controllers.capturePicVidController import videoFrames

web_routes = APIRouter()


def jpegParts(frames):
    for frame in frames:
        yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"


@web_routes.post("/camera/record")
def recordAndSaveFrames():
    try:
        videoPath = recordGestureVideo()
        pictureFolder, frameCount = saveVideoFrames(videoPath)
    except Exception as error:
        raise HTTPException(status_code=503, detail="Recording did not finish: {}".format(error))
    return {
        "video": videoPath,
        "pictures": pictureFolder,
        "frameCount": frameCount,
    }


@web_routes.get("/video")
@web_routes.get("/video/{effect}")
def watchVideo(effect=None):
    if effect not in (None, "gray", "blur", "gaussian", "yellow", "gesture"):
        raise HTTPException(status_code=404, detail="Unknown effect")
    if not os.path.isfile(VIDEO_PATH):
        raise HTTPException(status_code=404, detail="Missing video file: test_video.mp4")
    return StreamingResponse(
        jpegParts(videoFrames(effect)),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


@web_routes.get("/camera")
@web_routes.get("/camera/{effect}")
def watchCamera(effect=None):
    if effect not in (None, "gray", "blur", "gaussian", "yellow", "gesture"):
        raise HTTPException(status_code=404, detail="Unknown effect")
    try:
        camera = openPiCamera()
    except Exception as error:
        raise HTTPException(status_code=503, detail="Camera did not open: {}".format(error))
    return StreamingResponse(
        jpegParts(cameraFrames(camera, effect)),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )
