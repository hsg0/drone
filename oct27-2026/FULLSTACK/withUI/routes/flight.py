# WHAT — web addresses for flight logs.
# WHY — the page asks for logs through these addresses. Each address calls
#       one function in the flight controller and does nothing else.
# HOW — GET /flights lists logs. GET /flights/{id} reads one. POST /flights
#       saves whatever body was sent. POST /flights/fly10FeetEWNSLand calls
#       the east-west-north-south flight. POST /flights/fly10FeetE360W360N360S360Land
#       calls the same legs with a clockwise 360 between them.
# IMPORTANT — do not put database or flight decisions in this file.

import asyncio

from fastapi import APIRouter, Request, WebSocket, WebSocketDisconnect

from controllers.flightController import (
    flightProgressQueues,
    fly10FeetE360W360N360S360Land,
    fly10FeetEWNSLand,
    listFlightLogs,
    clearFly10FeetEWNSLand,
    returnFly10FeetEWNSLandHome,
    stopFly10FeetEWNSLand,
    readFlightProgress,
    readOneFlightLog,
    rememberFlightProgressLoop,
    saveOneFlightLog,
)

flightRouter = APIRouter()


@flightRouter.websocket("/flights/live")
async def watchFlightProgress(websocket: WebSocket):
    await websocket.accept()
    rememberFlightProgressLoop(asyncio.get_running_loop())
    progressQueue = asyncio.Queue()
    flightProgressQueues.add(progressQueue)
    try:
        await websocket.send_json(readFlightProgress())
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
        flightProgressQueues.discard(progressQueue)


@flightRouter.get("/flights")
def getFlights():
    return listFlightLogs()


@flightRouter.get("/flights/{flightLogId}")
def getFlight(flightLogId: str):
    return readOneFlightLog(flightLogId)


@flightRouter.post("/flights")
async def postFlight(request: Request):
    return await saveOneFlightLog(request)


@flightRouter.post("/flights/fly10FeetEWNSLand")
async def postFly10FeetEWNSLand(request: Request):
    return await fly10FeetEWNSLand(request)


@flightRouter.post("/flights/fly10FeetEWNSLand/stop")
async def postStopFly10FeetEWNSLand():
    return await stopFly10FeetEWNSLand()


@flightRouter.post("/flights/fly10FeetEWNSLand/return-home")
async def postReturnFly10FeetEWNSLandHome():
    return await returnFly10FeetEWNSLandHome()


@flightRouter.post("/flights/fly10FeetEWNSLand/clear")
async def postClearFly10FeetEWNSLand():
    return await clearFly10FeetEWNSLand()


@flightRouter.post("/flights/fly10FeetE360W360N360S360Land")
async def postFly10FeetTurnMission(request: Request):
    return await fly10FeetE360W360N360S360Land(request)
