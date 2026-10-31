import { useEffect, useRef, useState } from "react"
import { Link } from "react-router-dom"
import { clearFly10FeetEastWestNorthSouthLand, fly10FeetEastWestNorthSouthLand, readFlightLogs, returnFly10FeetEastWestNorthSouthLandHome, stopFly10FeetEastWestNorthSouthLand, watchFlightProgress } from "../backend-call/callBackend.js"
import { LiveFlightSteps } from "./liveFlightSteps.jsx"

function Fly10EastWestNorthSouth() {
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

  const ewnsIsRunning = flightIsRunning || (liveProgress?.running && liveProgress?.scriptName === "fly10FeetEWNSLand")

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
      const reply = await fly10FeetEastWestNorthSouthLand(controller.signal)
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
      <h1>fly10FeetEWNSLand</h1>
      <p>Take off to 5 feet, then east, west, north, and south, 10 feet each, and land.</p>
      <button className="brightButton clearButton" type="button" onClick={clearFlights}>
        Clear flights before flying
      </button>
      <div className="buttonRow">
        <button
          className="brightButton flightButton"
          type="button"
          onClick={startFlight}
          disabled={ewnsIsRunning}
        >
          {ewnsIsRunning ? "Flying..." : "Fly — sends YES"}
        </button>
        <button
          className="brightButton homeButton"
          type="button"
          onClick={returnHome}
          disabled={!ewnsIsRunning}
        >
          Return to Home Location
        </button>
        <button
          className="brightButton stopButton"
          type="button"
          onClick={stopFlight}
          disabled={!ewnsIsRunning}
        >
          Stop and land
        </button>
      </div>
      {flightReply ? <p>{flightReply}</p> : null}
      <LiveFlightSteps progress={liveProgress} connection={liveConnection} />
      <section className="logList">
        <h2>Flight logs</h2>
        {logStatus ? <p>{logStatus}</p> : null}
        <ul>
          {flightLogs.map((flightLog) => (
            <li key={flightLog.id}>
              {flightLog.savedAt || "unsaved time"} — {flightLog.details?.outcome || flightLog.rawText || "log"}{flightLog.details?.stopReason ? ` — ${flightLog.details.stopReason}` : ""}
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

export default Fly10EastWestNorthSouth
