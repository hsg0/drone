# WHAT — the SQLite connection for flight logs.
# WHY — the model needs one local file to save into. This file opens that
#       connection and does not decide what a flight log looks like.
# HOW — the model imports this engine, then reads and writes rows.
# IMPORTANT — the database file is local. It is not the Pixhawk, and it does
#             not arm or fly the copter.

from pathlib import Path

from sqlmodel import create_engine

DATABASE_DIRECTORY = Path(__file__).resolve().parent / "sqlDB"
DATABASE_FILE_PATH = DATABASE_DIRECTORY / "flightLogs.sqlite"

DATABASE_DIRECTORY.mkdir(parents=True, exist_ok=True)
engine = create_engine(
    f"sqlite:///{DATABASE_FILE_PATH}",
    connect_args={"check_same_thread": False},
)
