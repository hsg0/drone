import { useEffect, useState } from "react"
import { Link } from "react-router-dom"
import { calibrateCompasses, watchCalibrationProgress } from "../backend-call/callBackend.js"
import uprightDoThis from "../assets/compass-upright-do.jpg"
import uprightDoNotThis from "../assets/compass-upright-dont.jpg"
import noseDownDoThis from "../assets/compass-nose-down-do.jpg"
import noseDownDoNotThis from "../assets/compass-nose-down-dont.jpg"
import noseUpDoThis from "../assets/compass-nose-up-do.jpg"
import noseUpDoNotThis from "../assets/compass-nose-up-dont.jpg"
import leftSideDoThis from "../assets/compass-left-side-do.jpg"
import leftSideDoNotThis from "../assets/compass-left-side-dont.jpg"
import rightSideDoThis from "../assets/compass-right-side-do.jpg"
import rightSideDoNotThis from "../assets/compass-right-side-dont.jpg"
import upsideDownDoThis from "../assets/compass-upside-down-do.jpg"
import upsideDownDoNotThis from "../assets/compass-upside-down-dont.jpg"

const SPIN_POSITIONS = [
  {
    position: 1,
    title: "1. Upright",
    direction:
      "The drone is upright, the way it sits on the ground. The belly and the legs point at the ground. The top points at the sky. Keep it upright. Turn the nose clockwise, to the right, until it points the same way it started. That is one 360 turn. Do not flip it.",
    pictures: [
      { src: uprightDoThis, alt: "Drone upright. A cyan arrow shows one clockwise 360 turn of the nose." },
      { src: uprightDoNotThis, alt: "Do not flip or tumble the drone. Keep the belly toward the ground." },
    ],
  },
  {
    position: 2,
    title: "2. Nose at the ground",
    direction:
      "Tip the drone so the nose points straight at the ground and the tail points at the sky. Keep the nose pointing down the whole time. Look down at the tail and turn the drone clockwise, to your right, one full 360 turn. Do not roll it upright while you turn.",
    pictures: [
      { src: noseDownDoThis, alt: "Nose at the ground. Look down at the tail and turn clockwise, to your right." },
      { src: noseDownDoNotThis, alt: "Do not roll the drone upright. Keep the nose at the ground." },
    ],
  },
  {
    position: 3,
    title: "3. Nose at the sky",
    direction:
      "Tip the drone so the nose points straight at the sky and the tail points at the ground. Keep the nose pointing up the whole time. Look down at the belly and turn the drone clockwise, to your right, one full 360 turn. Do not roll it while you turn.",
    pictures: [
      { src: noseUpDoThis, alt: "Nose at the sky. Look down at the belly and turn clockwise, to your right." },
      { src: noseUpDoNotThis, alt: "Do not roll the drone. Keep the nose at the sky." },
    ],
  },
  {
    position: 4,
    title: "4. Left side on the ground",
    direction:
      "Stand behind the tail and look the way the nose points. Your left is the drone's left. Lay that left side toward the ground. Keep the left side down the whole time. Look down at the right side and turn clockwise, to your right, one full 360 turn.",
    pictures: [
      { src: leftSideDoThis, alt: "Left side on the ground. Look down at the right side and turn clockwise." },
      { src: leftSideDoNotThis, alt: "Do not roll the drone flat. Keep the left side on the ground." },
    ],
  },
  {
    position: 5,
    title: "5. Right side on the ground",
    direction:
      "Stand behind the tail and look the way the nose points. Your right is the drone's right. Lay that right side toward the ground. Keep the right side down the whole time. Look down at the left side and turn clockwise, to your right, one full 360 turn.",
    pictures: [
      { src: rightSideDoThis, alt: "Right side on the ground. Look down at the left side and turn clockwise." },
      { src: rightSideDoNotThis, alt: "Do not roll the drone flat. Keep the right side on the ground." },
    ],
  },
  {
    position: 6,
    title: "6. Upside down",
    direction:
      "This is the only upside-down pose. The belly and the legs point at the sky. The top of the drone points at the ground. Keep it upside down. Look down at the belly and turn clockwise, to your right, one full 360 turn. Stay upside down until both percentages reach 100.",
    pictures: [
      { src: upsideDownDoThis, alt: "Upside down. Look down at the belly, legs toward the sky, and turn clockwise." },
      { src: upsideDownDoNotThis, alt: "Do not flip the drone upright. Stay upside down." },
    ],
  },
]

function CalibrateCompasses() {
  const [progress, setProgress] = useState(null)
  const [savedResult, setSavedResult] = useState(null)
  const [calibrationIsRunning, setCalibrationIsRunning] = useState(false)
  const [progressError, setProgressError] = useState("")
  const [liveConnection, setLiveConnection] = useState("connecting")

  useEffect(() => {
    let pageIsOpen = true
    const closeLiveConnection = watchCalibrationProgress(
      (latestProgress) => {
        if (pageIsOpen) setProgress(latestProgress)
      },
      (connectionState) => {
        if (!pageIsOpen) return
        setLiveConnection(connectionState)
        if (connectionState === "error") setProgressError("The live connection did not open.")
      },
    )
    return () => {
      pageIsOpen = false
      closeLiveConnection()
    }
  }, [])

  async function startCalibration() {
    scrollToCalibrationStep(1)
    setCalibrationIsRunning(true)
    setSavedResult(null)
    setProgressError("")
    try {
      const result = await calibrateCompasses()
      setSavedResult(result)
    } catch (error) {
      setProgressError(error?.message || "The backend did not answer.")
    } finally {
      setCalibrationIsRunning(false)
    }
  }

  const currentPosition = progress?.position || 0
  const currentStep = SPIN_POSITIONS.find((step) => step.position === currentPosition)

  useEffect(() => {
    scrollToCalibrationStep(currentPosition)
  }, [currentPosition])
  const boardPercent = clampPercent(progress?.primaryPercent)
  const gpsPercent = clampPercent(progress?.secondaryPercent)
  const reports = savedResult?.compassReports || progress?.compassReports || []

  return (
    <main className="screen">
      <h1>Calibrate drone compasses</h1>
      <p className="liveLine">Live connection: {liveConnection}</p>
      <p>
        Each step is one pose, then one clockwise 360 turn in that pose. Do not tumble the drone during the turn.
        Change pose only when the highlighted step changes. Both percentages have to climb. The result is saved at the end.
      </p>
      <button
        className="brightButton calibrationButton"
        type="button"
        onClick={startCalibration}
        disabled={calibrationIsRunning || progress?.running}
      >
        {calibrationIsRunning || progress?.running ? "Calibration running..." : "Start calibration — sends YES"}
      </button>
      {progressError ? <p>{progressError}</p> : null}

      <section className="currentDirection">
        <h2>{currentStep ? currentStep.title : "Waiting to start"}</h2>
        <p>{currentStep ? currentStep.direction : progress?.instruction}</p>
        {currentStep?.pictures ? <SpinPictures pictures={currentStep.pictures} /> : null}
        {currentStep && progress?.instruction ? <p className="liveLine">{progress.instruction}</p> : null}
      </section>

      <section className="percentGroup">
        <PercentBar label="Board compass" percent={boardPercent} />
        <PercentBar label="GPS compass" percent={gpsPercent} />
      </section>

      <ol className="positionList">
        {SPIN_POSITIONS.map((step) => (
          <li
            key={step.position}
            id={`calibration-step-${step.position}`}
            className={step.position === currentPosition ? "currentPosition" : ""}
          >
            <strong>{step.title}</strong>
            <span>{step.direction}</span>
            {step.pictures ? <SpinPictures pictures={step.pictures} /> : null}
          </li>
        ))}
      </ol>

      {savedResult ? (
        <section className="savedResult">
          <h2>{savedResult.outcome || "Finished"}</h2>
          <p>{savedResult.id ? `Saved in the flight log as ${savedResult.id}.` : "The result was not saved."}</p>
          {savedResult.stopReason ? <p>{savedResult.stopReason}</p> : null}
          <ul>
            {reports.map((report) => (
              <li key={report.compassId}>
                Compass {report.compassId}: {report.status}, fitness {Number(report.fitness).toFixed(1)}, offsets{" "}
                {Number(report.offsetX).toFixed(0)}, {Number(report.offsetY).toFixed(0)}, {Number(report.offsetZ).toFixed(0)}
              </li>
            ))}
          </ul>
        </section>
      ) : null}

      <Link className="brightButton calibrationButton" to="/">
        Back to dashboard
      </Link>
    </main>
  )
}

function scrollToCalibrationStep(position) {
  const stepNumber = Number(position)
  if (stepNumber < 1) return
  window.requestAnimationFrame(() => {
    const stepElement = document.getElementById(`calibration-step-${stepNumber}`)
    if (!stepElement) return
    stepElement.scrollIntoView({ behavior: "smooth", block: "start" })
  })
}

function SpinPictures({ pictures }) {
  return (
    <div className="spinPictures">
      {pictures.map((picture) => (
        <img key={picture.alt} src={picture.src} alt={picture.alt} />
      ))}
    </div>
  )
}

function PercentBar({ label, percent }) {
  return (
    <div className="percentBar">
      <span>{label}</span>
      <div className="percentTrack">
        <div className="percentFill" style={{ width: `${percent}%` }} />
      </div>
      <strong>{percent}%</strong>
    </div>
  )
}

function clampPercent(value) {
  const number = Number(value)
  if (!Number.isFinite(number)) return 0
  return Math.max(0, Math.min(100, Math.round(number)))
}

export default CalibrateCompasses
