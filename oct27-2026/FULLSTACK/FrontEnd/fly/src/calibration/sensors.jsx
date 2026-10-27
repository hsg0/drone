import { Link } from "react-router-dom"

function CalibrateSensors() {
  return (
    <main className="screen">
      <h1>Calibrate external sensors</h1>
      <p>Sonic and time-of-flight sensors on this copter.</p>
      <Link className="brightButton calibrationButton" to="/">
        Back to dashboard
      </Link>
    </main>
  )
}

export default CalibrateSensors
