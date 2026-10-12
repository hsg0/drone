from fastapi import FastAPI
from fastapi import Request
from fastapi.responses import HTMLResponse
import uvicorn

from routes.web import web_routes
from routes.api import api_routes
from routes.error import error_routes
from routes.testflight import testflight
from routes.flight import flightRoutes
from middleware.middleware import middleware

app = FastAPI()
middleware(app)
app.include_router(web_routes)
app.include_router(api_routes)
app.include_router(testflight)
app.include_router(flightRoutes)
error_routes(app)


@app.get("/", response_class=HTMLResponse)
def readRoot(request: Request):
    """The flight deck is the website. /video is only the camera stream."""
    flightDeckAddress = "http://{}:3000".format(request.url.hostname)
    return (
        '<p><a href="{}">Open the flight deck</a></p>'
        '<p><a href="/video">Watch the camera stream</a></p>'
    ).format(flightDeckAddress)


if __name__ == "__main__":
    print("http://127.0.0.1:8000")
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)
