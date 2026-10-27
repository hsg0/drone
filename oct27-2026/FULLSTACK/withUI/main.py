# WHAT — local web backend for the withUI pages.
# WHY — the browser page and this program run on different ports, so the
#       browser blocks the page unless this program explicitly allows it.
# HOW — FastAPI listens on this computer only. A page request goes to the
#       flight routes, then the controller, then the model, then SQLite.
#       On startup the model creates the flight-log file.
#       Start it with: uv run python main.py
# IMPORTANT — this listens on 127.0.0.1 port 4020. Another computer on the
#             network cannot open it. Do not point this at the Pixhawk.

import asyncio
from contextlib import asynccontextmanager

import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from controllers.calibration import rememberProgressLoop
from models.flightlogs import createFlightLogTables, databaseFilePath
from routes.calibration import calibrationRouter
from routes.flight import flightRouter
from routes.flightPlan import flightPlanRouter
from routes.motorTest import motorTestRouter

LISTEN_HOST = "0.0.0.0"
LISTEN_PORT = 4020

# Any page opened on localhost or the local network may call this backend.
LOCAL_PAGE_ORIGIN_PATTERN = (
    r"http://(localhost|127\.0\.0\.1|192\.168\.\d+\.\d+|10\.\d+\.\d+\.\d+|25\.\d+\.\d+\.\d+|[\w-]+\.local)(:\d+)?$"
)

@asynccontextmanager
async def backendLifespan(_app):
    rememberProgressLoop(asyncio.get_running_loop())
    try:
        createFlightLogTables()
        print(f"Flight log database: {databaseFilePath}")
    except Exception as error:
        print(f"Flight log database did not open: {error}")
    yield


app = FastAPI(title="withUI", lifespan=backendLifespan)
app.include_router(flightRouter)
app.include_router(flightPlanRouter)
app.include_router(calibrationRouter)
app.include_router(motorTestRouter)

app.add_middleware(
    CORSMiddleware,
    allow_origin_regex=LOCAL_PAGE_ORIGIN_PATTERN,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/")
def readHealth():
    return {"service": "withUI", "status": "ok"}


def main():
    print(f"withUI backend at http://localhost:{LISTEN_PORT}")
    uvicorn.run(
        "main:app",
        host=LISTEN_HOST,
        port=LISTEN_PORT,
        reload=True,
    )


if __name__ == "__main__":
    main()
