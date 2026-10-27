# WHAT — web address for compass calibration.
# WHY — the page starts calibration through this address. The address calls
#       the calibration controller and does nothing else.
# HOW — POST /calibration/compasses runs calibrateCompasses.
#       The live socket at /calibration/compasses/live pushes each new position.
# IMPORTANT — do not put Pixhawk or database decisions in this file.

import asyncio

from fastapi import APIRouter, Request, WebSocket, WebSocketDisconnect

from controllers.calibration import calibrateCompasses, progressQueues, readCalibrationProgress, rememberProgressLoop

calibrationRouter = APIRouter()


@calibrationRouter.websocket("/calibration/compasses/live")
async def watchCalibrationProgress(websocket: WebSocket):
    await websocket.accept()
    rememberProgressLoop(asyncio.get_running_loop())
    progressQueue = asyncio.Queue()
    progressQueues.add(progressQueue)
    try:
        await websocket.send_json(readCalibrationProgress())
        while True:
            incoming = asyncio.create_task(websocket.receive())
            outgoing = asyncio.create_task(progressQueue.get())
            finished, stillWaiting = await asyncio.wait(
                {incoming, outgoing},
                return_when=asyncio.FIRST_COMPLETED,
            )
            for waitingTask in stillWaiting:
                waitingTask.cancel()
            if incoming in finished:
                message = incoming.result()
                if message.get("type") == "websocket.disconnect":
                    break
            if outgoing in finished:
                await websocket.send_json(outgoing.result())
    except WebSocketDisconnect:
        return
    finally:
        progressQueues.discard(progressQueue)


@calibrationRouter.post("/calibration/compasses")
async def postCalibrateCompasses(request: Request):
    return await calibrateCompasses(request)
