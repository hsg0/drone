import { useEffect, useState } from "react"
import { Link } from "react-router-dom"
import dronePhoto from "./assets/droneDashboard.jpg"
import { readBackendHealth } from "./backend-call/callBackend.js"
import "./App.css"

function App() {
  const [backendStatus, setBackendStatus] = useState("Checking the backend...")

  useEffect(() => {
    let pageIsOpen = true
    readBackendHealth()
      .then((health) => {
        if (!pageIsOpen) return
        if (health && health.status === "ok") {
          setBackendStatus("Backend is up")
          return
        }
        setBackendStatus("Backend answered")
      })
      .catch(() => {
        if (pageIsOpen) setBackendStatus("Backend is not answering")
      })
    return () => {
      pageIsOpen = false
    }
  }, [])

  return (
    <main className="dashboard">
      <img
        className="dashboardPhoto"
        src={dronePhoto}
        alt="Drone hovering over the landing pad"
      />
      <h1>Drone</h1>
      <p className="backendStatus">{backendStatus}</p>

      <section className="buttonGroup">
        <h2>Motor test</h2>
        <div className="buttonRow">
          <Link className="brightButton motorButton" to="/motortest/one">
            One motor at a time
          </Link>
          <Link className="brightButton motorButton" to="/motortest/all">
            All four motors
          </Link>
        </div>
      </section>

      <section className="buttonGroup">
        <h2>Calibration</h2>
        <div className="buttonRow">
          <Link className="brightButton calibrationButton" to="/calibration/compasses">
            Calibrate drone compasses
          </Link>
          <Link className="brightButton calibrationButton" to="/calibration/sensors">
            Calibrate external sensors
          </Link>
        </div>
      </section>

      <section className="buttonGroup">
        <h2>Fly</h2>
        <div className="buttonRow">
          <Link className="brightButton flightButton" to="/fly/fly10ewns">
            fly10FeetEWNSLand
          </Link>
          <Link className="brightButton flightButton" to="/fly/fly10ewns360">
            fly10feetE360W360N360S360Land
          </Link>
          <Link className="brightButton flightButton" to="/geo/flight-plan">
            Flight plan
          </Link>
        </div>
      </section>

      <section className="buttonGroup">
        <h2>Gesture control</h2>
        <div className="buttonRow">
          <Link className="brightButton gestureButton" to="/gesture/human">
            Human gesture control
          </Link>
        </div>
      </section>

      <section className="buttonGroup">
        <h2>Autonomous</h2>
        <div className="buttonRow">
          <Link className="brightButton autonomousButton" to="/autonomous/5-minutes">
            5 minute perimeter
          </Link>
        </div>
      </section>

      <section className="buttonGroup">
        <h2>Object detection autonomous</h2>
        <div className="buttonRow">
          <Link className="brightButton objectDetectionButton" to="/object-detection/autonomous">
            Object detection autonomous
          </Link>
        </div>
      </section>
    </main>
  )
}

export default App
