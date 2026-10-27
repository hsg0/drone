import { useEffect, useState } from "react"
import { Link } from "react-router-dom"
import { fly10FeetEastWestNorthSouthLand, readFlightLogs } from "../backend-call/callBackend.js"

function Fly10EastWestNorthSouth() {
  const [flightLogs, setFlightLogs] = useState([])
  const [logStatus, setLogStatus] = useState("Loading flight logs...")
  const [flightReply, setFlightReply] = useState("")
  const [flightIsRunning, setFlightIsRunning] = useState(false)

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

  async function startFlight() {
    setFlightIsRunning(true)
    setFlightReply("Flight requested. Waiting for the copter to finish.")
    try {
      const reply = await fly10FeetEastWestNorthSouthLand()
      const outcome = reply?.outcome || "No outcome"
      const reason = reply?.stopReason ? ` — ${reply.stopReason}` : ""
      setFlightReply(`${outcome}${reason}`)
      const body = await readFlightLogs()
      setFlightLogs(Array.isArray(body?.flightLogs) ? body.flightLogs : [])
      setLogStatus("")
    } catch (error) {
      setFlightReply(error?.message || "The backend did not answer.")
    } finally {
      setFlightIsRunning(false)
    }
  }

  return (
    <main className="screen">
      <h1>fly10FeetEWNSLand</h1>
      <p>Take off to 5 feet, then east, west, north, and south, 10 feet each, and land.</p>
      <button
        className="brightButton flightButton"
        type="button"
        onClick={startFlight}
        disabled={flightIsRunning}
      >
        {flightIsRunning ? "Flying..." : "Fly — sends YES"}
      </button>
      {flightReply ? <p>{flightReply}</p> : null}
      <section className="logList">
        <h2>Flight logs</h2>
        {logStatus ? <p>{logStatus}</p> : null}
        <ul>
          {flightLogs.map((flightLog) => (
            <li key={flightLog.id}>
              {flightLog.savedAt || "unsaved time"} — {flightLog.details?.outcome || flightLog.rawText || "log"}
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
