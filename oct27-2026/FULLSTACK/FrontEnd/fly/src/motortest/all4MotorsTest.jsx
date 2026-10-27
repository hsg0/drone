import { useEffect, useRef, useState } from "react"
import { Link } from "react-router-dom"
import { rampAllMotors, readMotorTestProgress, stopMotorTest } from "../backend-call/callBackend.js"

const TEST_SECONDS = 30
const MAXIMUM_PERCENT = 20

function All4MotorsTest() {
  const [statusText, setStatusText] = useState("")
  const [percent, setPercent] = useState(0)
  const [secondsLeft, setSecondsLeft] = useState(TEST_SECONDS)
  const [testIsRunning, setTestIsRunning] = useState(false)
  const runningRef = useRef(false)

  useEffect(() => {
    return () => {
      if (runningRef.current) {
        stopMotorTest().catch(() => {})
      }
    }
  }, [])

  useEffect(() => {
    if (!testIsRunning) return undefined
    let pageIsOpen = true
    const progressTimer = setInterval(() => {
      readMotorTestProgress()
        .then((progress) => {
          if (!pageIsOpen) return
          setPercent(Number(progress.percent) || 0)
          setSecondsLeft(progress.secondsLeft == null ? TEST_SECONDS : Number(progress.secondsLeft))
          if (!progress.running) setTestIsRunning(false)
        })
        .catch(() => {
          if (pageIsOpen) setStatusText("The motor progress did not load.")
        })
    }, 500)
    return () => {
      pageIsOpen = false
      clearInterval(progressTimer)
    }
  }, [testIsRunning])

  async function startRamp() {
    setTestIsRunning(true)
    runningRef.current = true
    setPercent(0)
    setSecondsLeft(TEST_SECONDS)
    setStatusText("All four motors are ramping from 0 to 20 percent.")
    try {
      const result = await rampAllMotors()
      setStatusText(result.stopReason || result.outcome || "Finished")
      setPercent(Number(result.percent) || 0)
    } catch (error) {
      setStatusText(error?.message || "The backend did not answer.")
    } finally {
      setTestIsRunning(false)
      runningRef.current = false
    }
  }

  async function stopRamp() {
    runningRef.current = false
    try {
      const result = await stopMotorTest()
      setStatusText(result.outcome || "Stopped")
    } catch (error) {
      setStatusText(error?.message || "The motors did not stop.")
    } finally {
      setTestIsRunning(false)
      setPercent(0)
      setSecondsLeft(0)
    }
  }

  return (
    <main className="screen">
      <h1>All four motors</h1>
      <p>
        All four motors spin together for {TEST_SECONDS} seconds. Thrust starts at 0 percent and
        reaches {MAXIMUM_PERCENT} percent at the end. Hold the frame. This can lift the copter.
      </p>
      <p>Time left: {Math.ceil(secondsLeft)} seconds</p>
      <p>Thrust now: {percent} percent, rising to {MAXIMUM_PERCENT} percent</p>
      <div className="percentTrack">
        <div className="percentFill" style={{ width: `${(percent / MAXIMUM_PERCENT) * 100}%` }} />
      </div>
      <button className="brightButton motorButton" type="button" onClick={startRamp} disabled={testIsRunning}>
        {testIsRunning ? "Test running..." : "Start 30 second test — sends YES"}
      </button>
      <button className="brightButton motorButton" type="button" onClick={stopRamp}>
        Stop
      </button>
      {statusText ? <p>{statusText}</p> : null}
      <Link className="brightButton motorButton" to="/">
        Back to dashboard
      </Link>
    </main>
  )
}

export default All4MotorsTest
