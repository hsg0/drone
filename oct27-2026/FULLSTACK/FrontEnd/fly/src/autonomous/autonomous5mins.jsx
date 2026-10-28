import { useEffect, useMemo, useRef, useState } from "react"
import { Link } from "react-router-dom"
import { CircleMarker, MapContainer, Polygon, Polyline, TileLayer, useMap, useMapEvents } from "react-leaflet"
import { distance, point } from "@turf/turf"
import { io } from "socket.io-client"
import { armForFlightPlan, backendBaseUrl, closeCameraConnection, disarmForFlightPlan, flySavedPerimeter, offerCameraConnection, savePerimeterPlan } from "../backend-call/callBackend.js"
import "leaflet/dist/leaflet.css"

const MAXIMUM_PERIMETER_FEET = 10000
const DEFAULT_CENTER = [39.5, -98.35]
const ICE_WAIT_MILLISECONDS = 3000

function AutonomousFiveMinutes() {
  const videoRef = useRef(null)
  const peerRef = useRef(null)
  const armedOnGroundRef = useRef(false)
  const [corners, setCorners] = useState([])
  const [perimeterIsClosed, setPerimeterIsClosed] = useState(false)
  const [heightFeet, setHeightFeet] = useState(5)
  const [speedMetersPerSecond, setSpeedMetersPerSecond] = useState(0.4)
  const [corridorWidthFeet, setCorridorWidthFeet] = useState(20)
  const [savedPlan, setSavedPlan] = useState(null)
  const [armed, setArmed] = useState(false)
  const [flightIsRunning, setFlightIsRunning] = useState(false)
  const [flightStatus, setFlightStatus] = useState("Click each corner, then close the perimeter.")
  const [pictureSource, setPictureSource] = useState("off")
  const [cameraStatus, setCameraStatus] = useState("The camera is off. Turning it on does not arm the copter.")
  const [liveProgress, setLiveProgress] = useState(null)

  const perimeterFeet = useMemo(() => perimeterLengthFeet(corners, perimeterIsClosed), [corners, perimeterIsClosed])
  const perimeterIsAllowed = perimeterFeet !== null && perimeterFeet <= MAXIMUM_PERIMETER_FEET
  const heightAndSpeedAreAllowed = heightFeet >= 3 && heightFeet <= 50 && speedMetersPerSecond >= 0.2 && speedMetersPerSecond <= 2
  const corridorIsAllowed = corridorWidthFeet >= 6 && corridorWidthFeet <= 200
  const canSave = perimeterIsClosed && perimeterIsAllowed && heightAndSpeedAreAllowed && corridorIsAllowed && !flightIsRunning

  useEffect(() => {
    const progressSocket = io(backendBaseUrl, { transports: ["websocket", "polling"] })
    progressSocket.on("perimeter-progress", (progress) => setLiveProgress(progress))
    return () => {
      progressSocket.close()
    }
  }, [])

  useEffect(() => {
    return () => {
      if (armedOnGroundRef.current) {
        disarmForFlightPlan().catch(() => {})
      }
      const peer = peerRef.current
      peerRef.current = null
      if (peer) peer.close()
      closeCameraConnection().catch(() => {})
    }
  }, [])

  function forgetSavedPlan() {
    setSavedPlan(null)
    setArmed(false)
    if (armedOnGroundRef.current) {
      armedOnGroundRef.current = false
      disarmForFlightPlan().catch(() => {})
    }
  }

  function addCorner(latitude, longitude) {
    if (perimeterIsClosed || flightIsRunning) return
    const nextCorner = { lat: latitude, lon: longitude }
    if (corners.length > 0) {
      const previousCorner = corners[corners.length - 1]
      const feetFromPrevious = distance(
        point([previousCorner.lon, previousCorner.lat]),
        point([nextCorner.lon, nextCorner.lat]),
        { units: "feet" },
      )
      if (feetFromPrevious < 8) {
        setFlightStatus("That click was on the corner you just placed.")
        return
      }
      if (corners.length >= 3) {
        const firstCorner = corners[0]
        const feetFromStart = distance(
          point([firstCorner.lon, firstCorner.lat]),
          point([nextCorner.lon, nextCorner.lat]),
          { units: "feet" },
        )
        if (feetFromStart < 8) {
          closePerimeter()
          return
        }
      }
    }
    forgetSavedPlan()
    setCorners((currentCorners) => [...currentCorners, nextCorner])
    setFlightStatus("Add corners, then close the perimeter. At least three corners are required.")
  }

  function closePerimeter() {
    if (corners.length < 3 || flightIsRunning) return
    forgetSavedPlan()
    setPerimeterIsClosed(true)
    setFlightStatus("Confirm the height and speed, then save. Arm before YES.")
  }

  function clearPerimeter() {
    if (flightIsRunning) return
    forgetSavedPlan()
    setCorners([])
    setPerimeterIsClosed(false)
    setFlightStatus("Click each corner, then close the perimeter.")
  }

  async function savePlan() {
    try {
      const result = await savePerimeterPlan({
        corners,
        heightFeet: Number(heightFeet),
        speedMetersPerSecond: Number(speedMetersPerSecond),
        corridorWidthFeet: Number(corridorWidthFeet),
      })
      if (result.outcome === "SAVED" && result.id) {
        setSavedPlan(result)
        setFlightStatus(`Perimeter saved as ${result.id}. Arm first. YES takes off and flies for 5 minutes.`)
        return
      }
      setFlightStatus(result.stopReason || "The perimeter was not saved.")
    } catch (error) {
      setFlightStatus(error?.message || "The backend did not answer.")
    }
  }

  async function armCopter() {
    try {
      const result = await armForFlightPlan()
      const isNowArmed = Boolean(result.armed)
      setArmed(isNowArmed)
      armedOnGroundRef.current = isNowArmed
      setFlightStatus(result.stopReason || result.outcome || "Arm finished.")
    } catch (error) {
      setFlightStatus(error?.message || "The backend did not answer.")
    }
  }

  async function disarmCopter() {
    armedOnGroundRef.current = false
    try {
      const result = await disarmForFlightPlan()
      setArmed(Boolean(result.armed))
      setFlightStatus(result.stopReason || result.outcome || "Disarmed.")
    } catch (error) {
      setFlightStatus(error?.message || "The backend did not answer.")
    }
  }

  async function takeOff() {
    armedOnGroundRef.current = false
    setFlightIsRunning(true)
    setFlightStatus("Flying the inside edge for 5 minutes. The copter will land when the time is up.")
    if (pictureSource === "off") {
      turnCameraOn()
    }
    try {
      const result = await flySavedPerimeter(savedPlan.id)
      setFlightStatus(result.stopReason || result.outcome || "Flight finished.")
      setArmed(false)
    } catch (error) {
      setFlightStatus(error?.message || "The backend did not answer.")
    } finally {
      setFlightIsRunning(false)
    }
  }

  async function turnCameraOff() {
    const peer = peerRef.current
    peerRef.current = null
    if (videoRef.current) videoRef.current.srcObject = null
    if (peer) peer.close()
    try {
      await closeCameraConnection()
    } catch {
      // The backend may already have closed this connection.
    }
    setPictureSource("off")
    setCameraStatus("The camera is off. Turning it on does not arm the copter.")
  }

  async function turnCameraOn() {
    setCameraStatus("Opening the WebRTC connection.")
    setPictureSource("connecting")
    const peer = new RTCPeerConnection()
    peerRef.current = peer
    peer.addTransceiver("video", { direction: "recvonly" })
    peer.ontrack = (event) => {
      const stream = event.streams[0]
      if (videoRef.current && stream) videoRef.current.srcObject = stream
    }
    peer.onconnectionstatechange = () => {
      if (peer.connectionState === "failed") {
        setCameraStatus("The WebRTC connection failed.")
        setPictureSource("off")
      }
    }
    try {
      const offer = await peer.createOffer()
      await peer.setLocalDescription(offer)
      await waitForIce(peer)
      if (peerRef.current !== peer) return
      const answer = await offerCameraConnection({
        sdp: peer.localDescription.sdp,
        type: peer.localDescription.type,
      })
      if (peerRef.current !== peer) {
        peer.close()
        return
      }
      await peer.setRemoteDescription({ sdp: answer.sdp, type: answer.type })
      setPictureSource(answer.source === "camera" ? "camera" : "saved clip")
      setCameraStatus(
        answer.source === "camera"
          ? "WebRTC is connected to the forward camera."
          : "The live camera did not open. Showing the saved clip. The live camera is on the Pi."
      )
    } catch (error) {
      if (peerRef.current === peer) peerRef.current = null
      peer.close()
      closeCameraConnection().catch(() => {})
      setPictureSource("off")
      setCameraStatus(error?.response?.data?.detail || error?.message || "The WebRTC connection did not start.")
    }
  }

  const drawnPositions = corners.map((corner) => [corner.lat, corner.lon])
  const insetPositions = Array.isArray(savedPlan?.insetCorners)
    ? savedPlan.insetCorners.map((corner) => [corner.lat, corner.lon])
    : []
  const corridorHalfFeet = (savedPlan?.corridorWidthFeet || corridorWidthFeet) / 2
  const corridorInside = offsetClosedPath(savedPlan?.insetCorners, -corridorHalfFeet)
  const corridorOutside = offsetClosedPath(savedPlan?.insetCorners, corridorHalfFeet)
  const speedFeetEachSecond = speedMetersPerSecond * 3.28084

  return (
    <main className="screen">
      <h1>5 minute perimeter</h1>
      <p>
        Draw the fence, then close it. After Arm and YES, the copter flies inside that line for 5 minutes and then lands. Leaving the fence also lands it. The camera does not move the copter.
      </p>
      <MapContainer className="flightMap" center={DEFAULT_CENTER} zoom={4}>
        <TileLayer attribution="&copy; OpenStreetMap" url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png" />
        <JumpToCurrentLocation />
        <MapClickCatcher onPick={addCorner} />
        {corners.map((corner, index) => (
          <CircleMarker
            key={`${corner.lat}-${corner.lon}-${index}`}
            center={[corner.lat, corner.lon]}
            radius={7}
            pathOptions={{ color: "#d58bff" }}
          />
        ))}
        {perimeterIsClosed ? (
          <Polygon positions={drawnPositions} pathOptions={{ color: "#d58bff", weight: 3 }} />
        ) : (
          <Polyline positions={drawnPositions} pathOptions={{ color: "#d58bff", weight: 3 }} />
        )}
        {insetPositions.length >= 3 ? (
          <Polygon positions={insetPositions} pathOptions={{ color: "#3d7dff", weight: 3, dashArray: "6 6" }} />
        ) : null}
        {corridorInside.length >= 3 ? (
          <Polyline positions={corridorInside} pathOptions={{ color: "#ff3b3b", weight: 2 }} />
        ) : null}
        {corridorOutside.length >= 3 ? (
          <Polyline positions={corridorOutside} pathOptions={{ color: "#ff3b3b", weight: 2 }} />
        ) : null}
      </MapContainer>

      <section className="planFacts">
        <p>Corners: {corners.length}{perimeterIsClosed ? ", closed" : ""}</p>
        <p className={perimeterFeet !== null && !perimeterIsAllowed ? "planWarning" : ""}>
          Perimeter: {perimeterFeet === null ? "waiting for a closed shape" : `${perimeterFeet.toFixed(0)} feet`}
        </p>
        <label>
          Height above the ground, in feet
          <input
            type="number"
            min="3"
            max="50"
            step="1"
            value={heightFeet}
            onChange={(event) => {
              forgetSavedPlan()
              setHeightFeet(Number(event.target.value))
            }}
          />
        </label>
        <label>
          Speed along the fence, meters each second
          <input
            type="number"
            min="0.2"
            max="2"
            step="0.1"
            value={speedMetersPerSecond}
            onChange={(event) => {
              forgetSavedPlan()
              setSpeedMetersPerSecond(Number(event.target.value))
            }}
          />
        </label>
        <p>
          This is how fast the copter flies along the fence. {speedMetersPerSecond} meters each second is about {speedFeetEachSecond.toFixed(1)} feet each second. 0.4 is a slow walk.
        </p>
        <label>
          Corridor width, in feet
          <input
            type="number"
            min="6"
            max="200"
            step="1"
            value={corridorWidthFeet}
            onChange={(event) => {
              forgetSavedPlan()
              setCorridorWidthFeet(Number(event.target.value))
            }}
          />
        </label>
        <p>
          The corridor is the band around the flight line. The copter must stay inside this width. The red lines show the two edges of that band after you save.
        </p>
        <h2>Fence corners</h2>
        <ol className="positionList">
          {corners.map((corner, index) => (
            <li key={`${corner.lat}-${corner.lon}-${index}`}>
              <strong>Corner {index + 1}</strong>
              <span>Latitude {corner.lat.toFixed(6)}</span>
              <span>Longitude {corner.lon.toFixed(6)}</span>
            </li>
          ))}
        </ol>
        {Array.isArray(savedPlan?.insetCorners) ? (
          <>
            <h2>Saved lock points</h2>
            <p>The copter flies to these latitude and longitude points, {savedPlan.insetFeet || 6} feet inside the fence.</p>
            <ol className="positionList">
              {savedPlan.insetCorners.map((corner, index) => (
                <li key={`lock-${corner.lat}-${corner.lon}-${index}`}>
                  <strong>Lock point {index + 1}</strong>
                  <span>Latitude {corner.lat.toFixed(6)}</span>
                  <span>Longitude {corner.lon.toFixed(6)}</span>
                </li>
              ))}
            </ol>
          </>
        ) : null}
      </section>

      <div className="buttonRow">
        <button className="brightButton autonomousButton" type="button" onClick={closePerimeter} disabled={perimeterIsClosed || corners.length < 3 || flightIsRunning}>
          Close perimeter
        </button>
        <button className="brightButton autonomousButton" type="button" onClick={clearPerimeter} disabled={flightIsRunning}>
          Clear perimeter
        </button>
        <button className="brightButton autonomousButton" type="button" onClick={savePlan} disabled={!canSave}>
          Save perimeter
        </button>
        <button className="brightButton autonomousButton" type="button" onClick={armCopter} disabled={!savedPlan || armed || flightIsRunning}>
          Arm
        </button>
        <button className="brightButton autonomousButton" type="button" onClick={disarmCopter} disabled={!armed || flightIsRunning}>
          Disarm
        </button>
        <button className="brightButton autonomousButton" type="button" onClick={takeOff} disabled={!savedPlan || !armed || flightIsRunning}>
          {flightIsRunning ? "Flying..." : "YES — fly 5 minutes"}
        </button>
      </div>
      {flightStatus ? <p>{flightStatus}</p> : null}
      {liveProgress && liveProgress.phase && liveProgress.phase !== "idle" ? (
        <section className="planFacts">
          <h2>Live flight</h2>
          <p>Phase: {liveProgress.phase}</p>
          <p>Latitude: {liveProgress.latitude == null ? "waiting" : Number(liveProgress.latitude).toFixed(6)}</p>
          <p>Longitude: {liveProgress.longitude == null ? "waiting" : Number(liveProgress.longitude).toFixed(6)}</p>
          <p>Height: {liveProgress.heightFeet == null ? "waiting" : `${liveProgress.heightFeet} feet`}</p>
          <p>Lock point: {liveProgress.lockPoint == null ? "waiting" : liveProgress.lockPoint}</p>
          <p>Time remaining: {liveProgress.secondsRemaining == null ? "waiting" : `${liveProgress.secondsRemaining} seconds`}</p>
          <p>Inside the fence: {liveProgress.insideFence == null ? "waiting" : liveProgress.insideFence ? "yes" : "no"}</p>
          <p>Inside the corridor: {liveProgress.insideCorridor == null ? "waiting" : liveProgress.insideCorridor ? "yes" : "no"}</p>
        </section>
      ) : null}

      <div className="buttonRow">
        <button className="brightButton gestureButton" type="button" onClick={turnCameraOn} disabled={pictureSource !== "off"}>
          Turn camera on
        </button>
        <button className="brightButton gestureButton" type="button" onClick={turnCameraOff} disabled={pictureSource === "off"}>
          Turn camera off
        </button>
      </div>
      <p>{cameraStatus}</p>
      {pictureSource !== "off" ? <video ref={videoRef} className="cameraPicture" autoPlay playsInline /> : null}

      <Link className="brightButton autonomousButton" to="/">
        Back to dashboard
      </Link>
    </main>
  )
}

function waitForIce(peer) {
  if (peer.iceGatheringState === "complete") return Promise.resolve()
  return new Promise((resolve) => {
    const timer = setTimeout(finish, ICE_WAIT_MILLISECONDS)
    function finish() {
      clearTimeout(timer)
      peer.removeEventListener("icegatheringstatechange", iceGatheringFinished)
      resolve()
    }
    function iceGatheringFinished() {
      if (peer.iceGatheringState === "complete") finish()
    }
    peer.addEventListener("icegatheringstatechange", iceGatheringFinished)
  })
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

function offsetClosedPath(pathCorners, offsetFeet) {
  if (!Array.isArray(pathCorners) || pathCorners.length < 3) return []
  const origin = pathCorners[0]
  const localCorners = pathCorners.map((corner) => localMeters(corner, origin))
  const offsetMeters = offsetFeet / 3.28084
  const movedCorners = localCorners.map((current, index) => {
    const previous = localCorners[(index - 1 + localCorners.length) % localCorners.length]
    const next = localCorners[(index + 1) % localCorners.length]
    const leftNormal = averagedLeftNormal(previous, current, next)
    return {
      north: current.north + leftNormal.north * offsetMeters,
      east: current.east + leftNormal.east * offsetMeters,
    }
  })
  return movedCorners.map((corner) => [
    origin.lat + corner.north / 111319.5,
    origin.lon + corner.east / (111319.5 * Math.cos((origin.lat * Math.PI) / 180)),
  ])
}

function localMeters(corner, origin) {
  return {
    north: (corner.lat - origin.lat) * 111319.5,
    east: (corner.lon - origin.lon) * 111319.5 * Math.cos((origin.lat * Math.PI) / 180),
  }
}

function averagedLeftNormal(previous, current, next) {
  const arriving = leftUnit(current.north - previous.north, current.east - previous.east)
  const leaving = leftUnit(next.north - current.north, next.east - current.east)
  const north = arriving.north + leaving.north
  const east = arriving.east + leaving.east
  const length = Math.hypot(north, east) || 1
  return { north: north / length, east: east / length }
}

function leftUnit(north, east) {
  const length = Math.hypot(north, east) || 1
  return { north: -east / length, east: north / length }
}

function perimeterLengthFeet(corners, perimeterIsClosed) {
  if (!perimeterIsClosed || corners.length < 3) return null
  let totalFeet = 0
  for (let index = 0; index < corners.length; index += 1) {
    const start = corners[index]
    const end = corners[(index + 1) % corners.length]
    totalFeet += distance(point([start.lon, start.lat]), point([end.lon, end.lat]), { units: "feet" })
  }
  return totalFeet
}

export default AutonomousFiveMinutes
