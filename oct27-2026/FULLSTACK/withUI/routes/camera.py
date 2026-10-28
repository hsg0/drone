# WHAT — web addresses for the forward camera.
# WHY — the gesture page shows the picture. These addresses only call the camera controller.
# HOW — GET /camera is the plain picture. GET /camera/gesture draws the pose.
#       GET /camera/sample plays the saved clip with the same pose labels.
#       GET /camera/seen returns the newest pose name and does not open the camera.
#       POST /camera/webrtc/offer answers a WebRTC offer with the camera track.
#       POST /camera/webrtc/close closes that connection.
# IMPORTANT — a missing camera returns an error. It does not arm or fly.

from fastapi import APIRouter, HTTPException, Request
from fastapi.responses import StreamingResponse

from controllers.camera import (
    answerCameraOffer,
    cameraFrames,
    closeCameraConnection,
    openPiCamera,
    openSampleClip,
    sampleVideoFrames,
    seenGesture,
)


cameraRouter = APIRouter()


def jpegParts(frames):
    for frame in frames:
        yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"


@cameraRouter.get("/camera/seen")
def getSeenGesture():
    return seenGesture()


@cameraRouter.get("/camera/sample")
def watchSampleClip():
    try:
        capture = openSampleClip()
    except Exception as error:
        raise HTTPException(status_code=503, detail="Saved clip did not open: {}".format(error))
    capture.release()
    return StreamingResponse(
        jpegParts(sampleVideoFrames()),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


@cameraRouter.get("/camera")
def watchCamera():
    return startCameraStream(False)


@cameraRouter.get("/camera/gesture")
def watchCameraGesture():
    return startCameraStream(True)


@cameraRouter.post("/camera/webrtc/offer")
async def postCameraOffer(request: Request):
    body = await request.json()
    offerSdp = body.get("sdp") if isinstance(body, dict) else None
    offerType = body.get("type") if isinstance(body, dict) else None
    if not offerSdp or not offerType:
        raise HTTPException(status_code=400, detail="Missing WebRTC offer.")
    try:
        return await answerCameraOffer(offerSdp, offerType)
    except Exception as error:
        raise HTTPException(status_code=503, detail="WebRTC did not start: {}".format(error))


@cameraRouter.post("/camera/webrtc/close")
async def postCameraClose():
    return await closeCameraConnection()


def startCameraStream(labelPose):
    try:
        camera = openPiCamera()
    except Exception as error:
        raise HTTPException(status_code=503, detail="Camera did not open: {}".format(error))
    return StreamingResponse(
        jpegParts(cameraFrames(camera, labelPose)),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )
