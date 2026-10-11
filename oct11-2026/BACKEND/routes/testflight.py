from fastapi import APIRouter

testflight = APIRouter()


@testflight.get("/testflight")
def testFlight():
    """List the gesture commands. This route does not arm the copter."""
    return {
        "route": "testflight",
        "armed": False,
        "commands": [
            {"pose": "wide arms overhead, peace sign", "command": "take off or land"},
            {"pose": "arms up", "command": "fly up"},
            {"pose": "arms coming down", "command": "fly down"},
            {"pose": "arms out", "command": "fly farther"},
            {"pose": "hands in", "command": "fly closer"},
            {"pose": "arms down", "command": "hover"},
        ],
    }
