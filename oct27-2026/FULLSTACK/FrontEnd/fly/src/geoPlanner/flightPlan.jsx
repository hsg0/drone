import { useEffect, useMemo, useRef, useState } from "react"
import { Link } from "react-router-dom"
import { MapContainer, TileLayer, CircleMarker, Polyline, useMap, useMapEvents } from "react-leaflet"
import { bearing, distance, point } from "@turf/turf"
import { armForFlightPlan, disarmForFlightPlan, flySavedFlightPlan, saveFlightPlan } from "../backend-call/callBackend.js"
import "leaflet/dist/leaflet.css"

const MAXIMUM_DISTANCE_FEET = 1000
const DEFAULT_CENTER = [39.5, -98.35]

function FlightPlan() {
  const [pointA, setPointA] = useState(null)
  const [pointB, setPointB] = useState(null)
  const [heightFeet, setHeightFeet] = useState(5)
  const [speedMetersPerSecond, setSpeedMetersPerSecond] = useState(0.4)
  const [savedPlan, setSavedPlan] = useState(null)
  const [armed, setArmed] = useState(false)
  const [statusText, setStatusText] = useState("Click the start point, then the end point.")
  const armedOnGroundRef = useRef(false)

  useEffect(() => {
    return () => {
      if (armedOnGroundRef.current) {
        disarmForFlightPlan().catch(() => {})
      }
    }
  }, [])

  const measurement = useMemo(() => measurePlan(pointA, pointB), [pointA, pointB])
  const distanceIsAllowed = measurement !== null && measurement.distanceFeet <= MAXIMUM_DISTANCE_FEET
  const valuesAreVisible = measurement !== null
  const canSave = valuesAreVisible && distanceIsAllowed && heightFeet >= 3 && heightFeet <= 50 && speedMetersPerSecond >= 0.2 && speedMetersPerSecond <= 2

  function pickPoint(latitude, longitude) {
    if (armedOnGroundRef.current) {
      armedOnGroundRef.current = false
      disarmForFlightPlan().catch(() => {})
    }
    const nextPoint = { lat: latitude, lon: longitude }
    if (pointA === null) {
      setPointA(nextPoint)
      setStatusText("Click the end point.")
    } else if (pointB === null) {
      setPointB(nextPoint)
      setStatusText("Confirm the distance, height, and speed, then save.")
    }
    setSavedPlan(null)
    setArmed(false)
  }

  function clearPoints() {
    if (armedOnGroundRef.current) {
      armedOnGroundRef.current = false
      disarmForFlightPlan().catch(() => {})
    }
    setPointA(null)
    setPointB(null)
    setSavedPlan(null)
    setArmed(false)
    setStatusText("Click the start point, then the end point.")
  }

  async function savePlan() {
    try {
      const result = await saveFlightPlan({
        pointA,
        pointB,
        distanceFeet: measurement.distanceFeet,
        bearingDegrees: measurement.bearingDegrees,
        heightFeet: Number(heightFeet),
        speedMetersPerSecond: Number(speedMetersPerSecond),
      })
      if (result.outcome === "SAVED" && result.id) {
        setSavedPlan(result)
        setStatusText(`Plan saved as ${result.id}. Arm first. YES takes off.`)
        return
      }
      setStatusText(result.stopReason || "The plan was not saved.")
    } catch (error) {
      setStatusText(error?.message || "The backend did not answer.")
    }
  }

  async function armCopter() {
    try {
      const result = await armForFlightPlan()
      const isNowArmed = Boolean(result.armed)
      setArmed(isNowArmed)
      armedOnGroundRef.current = isNowArmed
      setStatusText(result.stopReason || result.outcome || "Arm finished.")
    } catch (error) {
      setStatusText(error?.message || "The backend did not answer.")
    }
  }

  async function disarmCopter() {
    armedOnGroundRef.current = false
    try {
      const result = await disarmForFlightPlan()
      setArmed(Boolean(result.armed))
      setStatusText(result.stopReason || result.outcome || "Disarmed.")
    } catch (error) {
      setStatusText(error?.message || "The backend did not answer.")
    }
  }

  async function takeOff() {
    armedOnGroundRef.current = false
    try {
      const result = await flySavedFlightPlan(savedPlan.id)
      setStatusText(result.stopReason || result.outcome || "Flight finished.")
      setArmed(false)
    } catch (error) {
      setStatusText(error?.message || "The backend did not answer.")
    }
  }

  return (
    <main className="screen">
      <h1>Flight plan</h1>
      <p>Click A, then B. The line is straight. A path longer than 1000 feet can be seen, but it cannot be saved or flown.</p>
      <MapContainer className="flightMap" center={DEFAULT_CENTER} zoom={4}>
        <TileLayer
          attribution='&copy; OpenStreetMap'
          url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
        />
        <JumpToCurrentLocation />
        <MapClickCatcher onPick={pickPoint} />
        {pointA ? <CircleMarker center={[pointA.lat, pointA.lon]} radius={8} pathOptions={{ color: "#2ee6ff" }} /> : null}
        {pointB ? <CircleMarker center={[pointB.lat, pointB.lon]} radius={8} pathOptions={{ color: "#ffb020" }} /> : null}
        {pointA && pointB ? (
          <Polyline positions={[[pointA.lat, pointA.lon], [pointB.lat, pointB.lon]]} pathOptions={{ color: "#3d7dff", weight: 4 }} />
        ) : null}
      </MapContainer>

      <section className="planFacts">
        <p>A: {formatCoordinate(pointA)}</p>
        <p>B: {formatCoordinate(pointB)}</p>
        <p className={measurement && !distanceIsAllowed ? "planWarning" : ""}>
          Distance: {measurement ? `${measurement.distanceFeet.toFixed(0)} feet` : "waiting for both points"}
        </p>
        <p>Bearing: {measurement ? `${measurement.bearingDegrees.toFixed(0)} degrees clockwise from north` : "waiting for both points"}</p>
        <label>
          Height in feet
          <input
            type="number"
            min="3"
            max="50"
            step="1"
            value={heightFeet}
            onChange={(event) => setHeightFeet(Number(event.target.value))}
          />
        </label>
        <label>
          Speed in meters per second
          <input
            type="number"
            min="0.2"
            max="2"
            step="0.1"
            value={speedMetersPerSecond}
            onChange={(event) => setSpeedMetersPerSecond(Number(event.target.value))}
          />
        </label>
      </section>

      <button className="brightButton flightButton" type="button" onClick={savePlan} disabled={!canSave}>
        Save flight plan
      </button>
      <button className="brightButton flightButton" type="button" onClick={clearPoints}>
        Clear points
      </button>
      <button className="brightButton flightButton" type="button" onClick={armCopter} disabled={!savedPlan || armed}>
        Arm
      </button>
      <button className="brightButton flightButton" type="button" onClick={disarmCopter} disabled={!armed}>
        Disarm
      </button>
      <button className="brightButton flightButton" type="button" onClick={takeOff} disabled={!savedPlan || !armed}>
        YES — take off and fly
      </button>
      {statusText ? <p>{statusText}</p> : null}
      <Link className="brightButton flightButton" to="/">
        Back to dashboard
      </Link>
    </main>
  )
}

function JumpToCurrentLocation() {
  const map = useMap()
  useEffect(() => {
    if (!navigator.geolocation) return undefined
    navigator.geolocation.getCurrentPosition((position) => {
      map.setView([position.coords.latitude, position.coords.longitude], 17)
    })
    return undefined
  }, [map])
  return null
}

function MapClickCatcher({ onPick }) {
  useMapEvents({
    click(event) {
      onPick(event.latlng.lat, event.latlng.lng)
    },
  })
  return null
}

function measurePlan(pointA, pointB) {
  if (!pointA || !pointB) return null
  const start = point([pointA.lon, pointA.lat])
  const end = point([pointB.lon, pointB.lat])
  return {
    distanceFeet: distance(start, end, { units: "feet" }),
    bearingDegrees: (bearing(start, end) + 360) % 360,
  }
}

function formatCoordinate(place) {
  if (!place) return "not chosen"
  return `${place.lat.toFixed(6)}, ${place.lon.toFixed(6)}`
}

export default FlightPlan
