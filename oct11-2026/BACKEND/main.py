from fastapi import FastAPI
from fastapi.responses import HTMLResponse
import uvicorn

from routes.web import web_routes
from routes.api import api_routes
from routes.error import error_routes
from routes.testflight import testflight
from middleware.middleware import middleware

app = FastAPI()
middleware(app)
app.include_router(web_routes)
app.include_router(api_routes)
app.include_router(testflight)
error_routes(app)


@app.get("/", response_class=HTMLResponse)
def readRoot():
    return '<a href="/video">Hello, World!</a>'


if __name__ == "__main__":
    print("http://127.0.0.1:8000")
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)
