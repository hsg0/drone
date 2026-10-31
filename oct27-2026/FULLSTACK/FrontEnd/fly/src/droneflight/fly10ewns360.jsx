import { useEffect, useRef, useState } from "react"
import { Link } from "react-router-dom"
import {
  clearFly10FeetEastWestNorthSouthLand,
  fly10FeetTurnMission,
  readFlightLogs,
  returnFly10FeetEastWestNorthSouthLandHome,
  stopFly10FeetEastWestNorthSouthLand,
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
  const flyRequestRef = useRef(null)

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
    setFlightIsRunning(true)
    setFlightReply("Flight requested. Waiting for the copter to finish.")
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

export default Fly10EastWestNorthSouthTurns
