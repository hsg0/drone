# WHAT — web addresses for the motor tests.
# WHY — the motor pages start and stop a test through these addresses.
# HOW — POST /motortest/one spins one motor. POST /motortest/all ramps all four.
#       POST /motortest/stop sends 0 percent. GET /motortest/progress is the latest percent.
# IMPORTANT — do not put Pixhawk decisions in this file.

from fastapi import APIRouter, Request

from controllers.motorTest import rampAllMotors, readMotorTestProgress, spinOneMotor, stopMotorTest

motorTestRouter = APIRouter()


@motorTestRouter.get("/motortest/progress")
def getMotorTestProgress():
    return readMotorTestProgress()


@motorTestRouter.post("/motortest/one")
async def postOneMotor(request: Request):
    return await spinOneMotor(request)


@motorTestRouter.post("/motortest/all")
async def postAllMotors(request: Request):
    return await rampAllMotors(request)


@motorTestRouter.post("/motortest/stop")
async def postStopMotors(request: Request):
    return await stopMotorTest(request)
