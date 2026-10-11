from fastapi import Request
from fastapi.responses import JSONResponse


def error_routes(app):
    @app.exception_handler(404)
    async def not_found(request: Request, error):
        return JSONResponse(status_code=404, content={"message": "Not found"})
