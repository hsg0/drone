import { useEffect, useRef, useState } from "react"
import { Link } from "react-router-dom"
import {
  clearFly10FeetEastWestNorthSouthLand,
  fly10FeetTurnMission,
  readFlightLogs,
  returnFly10FeetEastWestNorthSouthLandHome,
  sendFly10FeetTurnMissionYes,
  stopFly10FeetEastWestNorthSouthLand,
  watchEwnsTelemetry,
  watchFlightProgress,
} from "../backend-call/callBackend.js"
import { LiveFlightSteps } from "./liveFlightSteps.jsx"

const FLIGHT_STEPS = [
  "Take off to 6 feet, then come up to a hover at 7 feet.",
  "Hover at 7 feet for 10 seconds. Stay in this upright pose.",
  "Turn clockwise 360 degrees. Keep the drone upright. Do not flip it. The nose turns to the right until it points the same way it started.",
  "Hover for 5 seconds.",
  "Fly east 10 feet at 7 feet. This is a straight move, not a flip.",
  "Hover for 5 seconds.",
  "Turn clockwise 360 degrees again. Stay upright.",
  "Hover for 5 seconds.",
  "Fly west 20 feet at 7 feet.",
  "Hover for 5 seconds.",
  "Turn clockwise 360 degrees. Stay upright.",
  "Hover for 5 seconds.",
  "Fly east 10 feet at 7 feet. This brings the copter back over the east-west start.",
  "Hover for 5 seconds.",
  "Fly north 10 feet at 7 feet.",
  "Hover for 5 seconds.",
  "Turn clockwise 360 degrees. Stay upright.",
  "Hover for 5 seconds.",
  "Fly south 10 feet at 7 feet.",
  "Hover for 5 seconds.",
  "Turn clockwise 360 degrees. Stay upright.",
  "Hover for 5 seconds.",
  "Land until the propellers stop.",
]

function Fly10EastWestNorthSouthTurns() {
  const [flightLogs, setFlightLogs] = useState([])
  const [logStatus, setLogStatus] = useState("Loading flight logs...")
  const [flightReply, setFlightReply] = useState("")
  const [flightIsRunning, setFlightIsRunning] = useState(false)
  const [liveProgress, setLiveProgress] = useState(null)
  const [liveConnection, setLiveConnection] = useState("connecting")
  const [telemetry, setTelemetry] = useState(null)
  const [telemetryConnection, setTelemetryConnection] = useState("connecting")
  const flyRequestRef = useRef(null)
  const yesSentRef = useRef(false)

  useEffect(() => {
    let pageIsOpen = true
    readFlightLogs()
      .then((body) => {
        if (!pageIsOpen) return
        const logs = Array.isArray(body?.flightLogs) ? body.flightLogs : []
        setFlightLogs(logs)
        setLogStatus(logs.length === 0 ? "No flight logs yet." : "")
      })
      .catch(() => {
        if (pageIsOpen) setLogStatus("Flight logs did not load.")
      })
    return () => {
      pageIsOpen = false
    }
  }, [])

  useEffect(() => {
    return watchFlightProgress(setLiveProgress, setLiveConnection)
  }, [])

  useEffect(() => {
    return watchEwnsTelemetry(setTelemetry, setTelemetryConnection)
  }, [])

  useEffect(() => {
    if (!flightIsRunning || yesSentRef.current || !liveProgress?.satelliteFixReady) return
    yesSentRef.current = true
    setFlightReply("Sending YES.")
    sendFly10FeetTurnMissionYes().catch((error) => {
      yesSentRef.current = false
      setFlightReply(error?.message || "YES was not sent.")
    })
  }, [flightIsRunning, liveProgress])

  const turnMissionIsRunning =
    flightIsRunning || (liveProgress?.running && liveProgress?.scriptName === "fly10FeetE360W360N360S360Land")

  async function clearFlights() {
    flyRequestRef.current?.abort()
    setFlightReply("Clearing flight requests...")
    try {
      const reply = await clearFly10FeetEastWestNorthSouthLand()
      setFlightReply(reply?.message || "Flight requests cleared.")
      if (reply?.cleared) setFlightIsRunning(false)
    } catch (error) {
      setFlightReply(error?.response?.data?.detail || error?.message || "Clear did not reach the backend.")
    }
  }

  async function returnHome() {
    setFlightReply("Return to home pressed. Flying back to the takeoff spot, then landing.")
    try {
      const reply = await returnFly10FeetEastWestNorthSouthLandHome()
      setFlightReply(reply?.message || reply?.outcome || "Return to home sent.")
    } catch (error) {
      setFlightReply(error?.response?.data?.detail || error?.message || "Return to home did not reach the backend.")
    }
  }

  async function stopFlight() {
    setFlightReply("Stop pressed. Landing now.")
    try {
      const reply = await stopFly10FeetEastWestNorthSouthLand()
      setFlightReply(reply?.message || reply?.outcome || "Stop sent.")
    } catch (error) {
      setFlightReply(error?.response?.data?.detail || error?.message || "Stop did not reach the backend.")
    }
  }

  async function startFlight() {
    const controller = new AbortController()
    flyRequestRef.current = controller
    yesSentRef.current = false
    setFlightIsRunning(true)
    setFlightReply("Connecting. YES is sent, then the copter arms.")
    try {
      const reply = await fly10FeetTurnMission(controller.signal)
      const outcome = reply?.outcome || "No outcome"
      const reason = reply?.stopReason ? ` — ${reply.stopReason}` : ""
      setFlightReply(`${outcome}${reason}`)
      const body = await readFlightLogs()
      setFlightLogs(Array.isArray(body?.flightLogs) ? body.flightLogs : [])
      setLogStatus("")
    } catch (error) {
      if (error?.code === "ERR_CANCELED" || error?.name === "CanceledError") return
      setFlightReply(error?.message || "The backend did not answer.")
    } finally {
      if (flyRequestRef.current === controller) flyRequestRef.current = null
      setFlightIsRunning(false)
    }
  }

  return (
    <main className="screen">
      <h1>fly10feetE360W360N360S360Land</h1>
      <p>
        The drone stays upright. A 360 is one clockwise turn in place. East, west, north, and south are straight moves at 7 feet.
      </p>
      <button className="brightButton clearButton" type="button" onClick={clearFlights}>
        Clear flights before flying
      </button>
      <div className="buttonRow">
        <button
          className="brightButton flightButton"
          type="button"
          onClick={startFlight}
          disabled={turnMissionIsRunning}
        >
          {turnMissionIsRunning ? "Flying..." : "Fly — sends YES"}
        </button>
        <button
          className="brightButton homeButton"
          type="button"
          onClick={returnHome}
          disabled={!turnMissionIsRunning}
        >
          Return to Home Location
        </button>
        <button
          className="brightButton stopButton"
          type="button"
          onClick={stopFlight}
          disabled={!turnMissionIsRunning}
        >
          Stop and land
        </button>
      </div>
      <section className="planFacts">
        <h2>Telemetry</h2>
        <p className="liveLine">Socket.io: {telemetryConnection}</p>
        <p className="liveLine">Updated: {telemetry?.updatedAt || "waiting"}</p>
        <p>Latitude: {formatCoordinate(telemetry?.latitude)}</p>
        <p>Longitude: {formatCoordinate(telemetry?.longitude)}</p>
        <p>Satellites: {formatCount(telemetry?.satelliteCount)}{telemetry?.fixType >= 3 ? ", 3D fix" : ""}</p>
        <p>Board compass direction: {formatCompassDirection(telemetry?.boardCompassDegrees)}</p>
        <p>GPS compass direction: {formatCompassDirection(telemetry?.gpsCompassDegrees)}</p>
      </section>
      {flightReply ? <p>{flightReply}</p> : null}
      <LiveFlightSteps progress={liveProgress} connection={liveConnection} />
      <ol className="positionList">
        {FLIGHT_STEPS.map((step) => (
          <li key={step}>{step}</li>
        ))}
      </ol>
      <section className="logList">
        <h2>Flight logs</h2>
        {logStatus ? <p>{logStatus}</p> : null}
        <ul>
          {flightLogs.map((flightLog) => (
            <li key={flightLog.id}>
              {flightLog.savedAt || "unsaved time"} — {flightLog.details?.scriptName || "log"} — {flightLog.details?.outcome || flightLog.rawText || "saved"}{flightLog.details?.stopReason ? ` — ${flightLog.details.stopReason}` : ""}
            </li>
          ))}
        </ul>
      </section>
      <Link className="brightButton flightButton" to="/">
        Back to dashboard
      </Link>
    </main>
  )
}

function formatCoordinate(value) {
  if (value == null) return "waiting"
  const number = Number(value)
  if (!Number.isFinite(number) || number === 0) return "waiting"
  return number.toFixed(6)
}

function formatCount(value) {
  if (value == null) return "waiting"
  const number = Number(value)
  if (!Number.isFinite(number)) return "waiting"
  return String(number)
}

function formatCompassDirection(value) {
  if (value == null) return "waiting"
  const degrees = Number(value)
  if (!Number.isFinite(degrees)) return "waiting"
  const names = ["north", "northeast", "east", "southeast", "south", "southwest", "west", "northwest"]
  const name = names[Math.round(degrees / 45) % 8]
  return `${degrees.toFixed(1)}° ${name}`
}

export default Fly10EastWestNorthSouthTurns
