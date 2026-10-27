# WHAT — the shape of one saved flight log.
# WHY — a flight may send a full story, one printed line, or something we have
#       not thought of yet. This model keeps all of it and refuses nothing.
# HOW — the controller calls this file. buildFlightLog packs any payload into
#       detailsJson, then this file reads and writes that row through the
#       SQLite connection. Known names are not required.
# IMPORTANT — buildFlightLog, save, and read do not raise. This file is the
#             only one that talks to the SQLite connection.

import json
from datetime import datetime
from typing import Any

from pydantic import ConfigDict
from sqlmodel import Field, Session, SQLModel, col, select

from DATABASE.flightLogDatabase import DATABASE_FILE_PATH, engine

databaseFilePath = DATABASE_FILE_PATH


class FlightLog(SQLModel, table=True):
    __tablename__ = "flightLog"
    model_config = ConfigDict(extra="ignore")

    id: int | None = Field(default=None, primary_key=True)
    savedAt: str | None = None
    # The whole log, in whatever shape it arrived. Stored as text so a strange
    # value cannot fail a column type check.
    detailsJson: str | None = None
    rawText: str | None = None


def buildFlightLog(payload: Any = None) -> FlightLog:
    rawText = None
    details = {}
    try:
        if payload is None:
            details = {}
        elif isinstance(payload, str):
            rawText = payload
            details = {"lineText": payload}
        elif isinstance(payload, dict):
            details = payload
            rawText = textOrNone(payload.get("lineText"))
        elif isinstance(payload, tuple):
            details = list(payload)
        else:
            # Lists, numbers, and other values are stored as themselves.
            # default=str inside the JSON step turns an unusual object into text.
            details = payload
            if not isinstance(payload, (list, dict)):
                rawText = textOrNone(payload)
        detailsJson = json.dumps(details, default=str, ensure_ascii=False)
    except Exception:
        rawText = textOrNone(payload)
        detailsJson = json.dumps({"unreadable": True, "rawText": rawText})

    try:
        return FlightLog(
            savedAt=datetime.now().isoformat(timespec="seconds"),
            detailsJson=detailsJson,
            rawText=rawText,
        )
    except Exception:
        return FlightLog()


def createFlightLogTables():
    try:
        FlightLog.metadata.create_all(engine)
    except Exception as error:
        print(f"Flight log tables were not created: {error}")


def saveFlightLog(payload: Any = None) -> int | None:
    try:
        flightLog = buildFlightLog(payload)
        with Session(engine) as session:
            session.add(flightLog)
            session.commit()
            session.refresh(flightLog)
            return flightLog.id
    except Exception as error:
        print(f"Flight log was not saved: {error}")
        return None


def readFlightLogs():
    try:
        with Session(engine) as session:
            statement = select(FlightLog).order_by(col(FlightLog.id).desc())
            flightLogs = list(session.exec(statement).all())
            for flightLog in flightLogs:
                session.expunge(flightLog)
            return flightLogs
    except Exception as error:
        print(f"Flight logs were not read: {error}")
        return []


def readFlightLog(flightLogId) -> FlightLog | None:
    try:
        wantedId = int(flightLogId)
    except Exception:
        return None
    try:
        with Session(engine) as session:
            flightLog = session.get(FlightLog, wantedId)
            if flightLog is not None:
                session.expunge(flightLog)
            return flightLog
    except Exception as error:
        print(f"Flight log {flightLogId} was not read: {error}")
        return None


def recordFromFlightLog(flightLog: FlightLog):
    details = None
    try:
        if flightLog.detailsJson:
            details = json.loads(flightLog.detailsJson)
    except Exception:
        details = flightLog.detailsJson
    return {
        "id": flightLog.id,
        "savedAt": flightLog.savedAt,
        "rawText": flightLog.rawText,
        "details": details,
    }


def textOrNone(value: Any) -> str | None:
    if value is None:
        return None
    try:
        return str(value)
    except Exception:
        try:
            return repr(value)
        except Exception:
            return None
