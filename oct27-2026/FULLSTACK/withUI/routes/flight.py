# WHAT — web addresses for flight logs.
# WHY — the page asks for logs through these addresses. Each address calls
#       one function in the flight controller and does nothing else.
# HOW — GET /flights lists logs. GET /flights/{id} reads one. POST /flights
#       saves whatever body was sent. POST /flights/fly10FeetEWNSLand calls
#       the east-west-north-south flight. POST /flights/fly10FeetE360W360N360S360Land
#       calls the same legs with a clockwise 360 between them.
# IMPORTANT — do not put database or flight decisions in this file.

from fastapi import APIRouter, Request

from controllers.flightController import (
    fly10FeetE360W360N360S360Land,
    fly10FeetEWNSLand,
    listFlightLogs,
    readOneFlightLog,
    saveOneFlightLog,
)

flightRouter = APIRouter()


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


@flightRouter.post("/flights/fly10FeetE360W360N360S360Land")
async def postFly10FeetTurnMission(request: Request):
    return await fly10FeetE360W360N360S360Land(request)
