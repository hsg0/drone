# WHAT — web addresses for a saved A-to-B flight plan.
# WHY — the map saves the path first. Arm, disarm, and takeoff are separate addresses.
# HOW — POST /flight-plans saves a straight path. POST /flight-plans/perimeter
#       saves a closed perimeter. POST /flight-plans/arm arms.
#       POST /flight-plans/disarm disarms and closes the cable.
#       POST /flight-plans/fly takes off only when the body says YES.
#       POST /flight-plans/perimeter/fly flies the inside edge for 5 minutes.
#       GET /flight-plans/perimeter/progress returns the latest flight note.
# IMPORTANT — do not put Pixhawk decisions in this file.

from fastapi import APIRouter, Request

from controllers.flightPlan import (
    armFlightPlan,
    disarmFlightPlan,
    flyFlightPlan,
    flyPerimeterPlan,
    readPerimeterProgress,
    saveFlightPlan,
    savePerimeterPlan,
)

flightPlanRouter = APIRouter()


@flightPlanRouter.post("/flight-plans")
async def postFlightPlan(request: Request):
    return await saveFlightPlan(request)


@flightPlanRouter.post("/flight-plans/arm")
async def postArmFlightPlan(request: Request):
    return await armFlightPlan(request)


@flightPlanRouter.post("/flight-plans/disarm")
async def postDisarmFlightPlan(request: Request):
    return await disarmFlightPlan(request)


@flightPlanRouter.post("/flight-plans/fly")
async def postFlyFlightPlan(request: Request):
    return await flyFlightPlan(request)


@flightPlanRouter.post("/flight-plans/perimeter")
async def postPerimeterPlan(request: Request):
    return await savePerimeterPlan(request)


@flightPlanRouter.post("/flight-plans/perimeter/fly")
async def postFlyPerimeterPlan(request: Request):
    return await flyPerimeterPlan(request)


@flightPlanRouter.get("/flight-plans/perimeter/progress")
def getPerimeterProgress():
    return readPerimeterProgress()
