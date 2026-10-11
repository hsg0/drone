from fastapi.middleware.cors import CORSMiddleware


def middleware(app):
    app.add_middleware(
        CORSMiddleware,
        allow_origins=[
            "http://127.0.0.1:3000",
            "http://localhost:3000",
        ],
        allow_methods=["GET", "POST"],
        allow_headers=["*"],
    )
    return app
