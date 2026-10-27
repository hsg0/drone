import { useEffect, useRef, useState } from "react"
import { Link } from "react-router-dom"
import { spinOneMotor, stopMotorTest } from "../backend-call/callBackend.js"

const MOTORS = [
  { testOrder: 1, corner: "Front-right", spin: "counter-clockwise" },
  { testOrder: 2, corner: "Back-right", spin: "clockwise" },
  { testOrder: 3, corner: "Back-left", spin: "counter-clockwise" },
  { testOrder: 4, corner: "Front-left", spin: "clockwise" },
]

function OneMotorAtATime() {
  const [motorIndex, setMotorIndex] = useState(0)
  const [thrustPercent, setThrustPercent] = useState(10)
  const [motorIsSpinning, setMotorIsSpinning] = useState(false)
  const [statusText, setStatusText] = useState("")
  const spinningRef = useRef(false)
  const currentMotor = MOTORS[motorIndex]

  useEffect(() => {
    return () => {
      if (spinningRef.current) {
        stopMotorTest().catch(() => {})
      }
    }
  }, [])

  async function spinCurrentMotor(percent) {
    setMotorIsSpinning(true)
    spinningRef.current = true
    try {
      const result = await spinOneMotor(currentMotor.testOrder, percent)
      setStatusText(result.stopReason || `${currentMotor.corner} at ${percent} percent.`)
      if (result.outcome === "FAILED" || result.outcome === "CANCELLED BEFORE START") {
        setMotorIsSpinning(false)
        spinningRef.current = false
      }
    } catch (error) {
      setMotorIsSpinning(false)
      spinningRef.current = false
      setStatusText(error?.message || "The backend did not answer.")
    }
  }

  async function changeThrust(nextPercent) {
    const boundedPercent = Math.min(50, Math.max(10, Number(nextPercent)))
    setThrustPercent(boundedPercent)
    if (motorIsSpinning) {
      await spinCurrentMotor(boundedPercent)
    }
  }

  async function stopAndMove(nextIndex) {
    spinningRef.current = false
    try {
      await stopMotorTest()
    } catch (error) {
      setStatusText(error?.message || "The motor did not stop.")
    }
    setMotorIsSpinning(false)
    setMotorIndex(nextIndex)
    setStatusText("Motor stopped. Press spin when you are ready for the next one.")
  }

  return (
    <main className="screen">
      <h1>One motor at a time</h1>
      <p>
        Only the highlighted motor spins. The others stay still. Thrust is a percent, from 10 to 50.
        Props should be off, or hold the frame.
      </p>
      <ol className="motorList">
        {MOTORS.map((motor, index) => (
          <li key={motor.testOrder} className={index === motorIndex ? "currentMotor" : ""}>
            <strong>Test {motor.testOrder}. {motor.corner}</strong>
            <span> Spins {motor.spin}.</span>
          </li>
        ))}
      </ol>
      <label className="thrustControl">
        Thrust {thrustPercent} percent
        <input
          type="range"
          min="10"
          max="50"
          step="1"
          value={thrustPercent}
          onChange={(event) => changeThrust(event.target.value)}
        />
      </label>
      <button className="brightButton motorButton" type="button" onClick={() => spinCurrentMotor(thrustPercent)}>
        Spin this motor — sends YES
      </button>
      <button className="brightButton motorButton" type="button" onClick={() => stopAndMove(motorIndex)}>
        Stop
      </button>
      <button
        className="brightButton motorButton"
        type="button"
        onClick={() => stopAndMove((motorIndex + 1) % MOTORS.length)}
      >
        Next motor
      </button>
      {statusText ? <p>{statusText}</p> : null}
      <Link className="brightButton motorButton" to="/">
        Back to dashboard
      </Link>
    </main>
  )
}

export default OneMotorAtATime
