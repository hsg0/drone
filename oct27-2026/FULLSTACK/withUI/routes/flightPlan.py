# WHAT — web addresses for a saved A-to-B flight plan.
# WHY — the map saves the path first. Arm, disarm, and takeoff are separate addresses.
# HOW — POST /flight-plans saves. POST /flight-plans/arm arms.
#       POST /flight-plans/disarm disarms and closes the cable.
#       POST /flight-plans/fly takes off only when the body says YES.
# IMPORTANT — do not put Pixhawk decisions in this file.

from fastapi import APIRouter, Request

from controllers.flightPlan import armFlightPlan, disarmFlightPlan, flyFlightPlan, saveFlightPlan

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
