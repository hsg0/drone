import { useEffect, useRef, useState } from "react"
import { Link } from "react-router-dom"
import { io } from "socket.io-client"
import { backendBaseUrl, closeCameraConnection, offerCameraConnection } from "../backend-call/callBackend.js"

const ICE_WAIT_MILLISECONDS = 3000

function ObjectDetectionAutonomous() {
  const videoRef = useRef(null)
  const peerRef = useRef(null)
  const [pictureSource, setPictureSource] = useState("off")
  const [cameraStatus, setCameraStatus] = useState("The camera is off. Turning it on does not arm the copter.")
  const [socketStatus, setSocketStatus] = useState("Connecting to telemetry...")
  const [liveProgress, setLiveProgress] = useState(null)

  useEffect(() => {
    const progressSocket = io(backendBaseUrl, { transports: ["websocket", "polling"] })
    progressSocket.on("connect", () => setSocketStatus("Telemetry socket is connected."))
    progressSocket.on("disconnect", () => setSocketStatus("Telemetry socket closed."))
    progressSocket.on("connect_error", () => setSocketStatus("Telemetry socket did not connect."))
    progressSocket.on("perimeter-progress", (progress) => setLiveProgress(progress))
    return () => {
      progressSocket.close()
    }
  }, [])

  useEffect(() => {
    return () => {
      const peer = peerRef.current
      peerRef.current = null
      if (peer) peer.close()
      closeCameraConnection().catch(() => {})
    }
  }, [])

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
    setCameraStatus("Opening the WebRTC connection. The picture travels over UDP.")
    setPictureSource("connecting")
    const peer = new RTCPeerConnection()
    peerRef.current = peer
    peer.addTransceiver("video", { direction: "recvonly" })
    peer.ontrack = (event) => {
      const stream = event.streams[0]
      if (videoRef.current && stream) videoRef.current.srcObject = stream
    }
    peer.onconnectionstatechange = () => {
      if (peer.connectionState === "connected") {
        setCameraStatus("WebRTC is connected. The live picture is coming over UDP.")
      }
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
      if (answer.source !== "camera") {
        setCameraStatus("The live camera did not open. Showing the saved clip. The live camera is on the Pi.")
      }
    } catch (error) {
      if (peerRef.current === peer) peerRef.current = null
      peer.close()
      closeCameraConnection().catch(() => {})
      setPictureSource("off")
      setCameraStatus(error?.response?.data?.detail || error?.message || "The WebRTC connection did not start.")
    }
  }

  return (
    <main className="screen">
      <h1>Object detection autonomous</h1>
      <p>
        Turn the camera on to watch the forward view. The picture uses WebRTC and travels over UDP. The numbers below come from the telemetry socket. This page does not arm the copter.
      </p>
      <div className="buttonRow">
        <button className="brightButton objectDetectionButton" type="button" onClick={turnCameraOn} disabled={pictureSource !== "off"}>
          Turn camera on
        </button>
        <button className="brightButton objectDetectionButton" type="button" onClick={turnCameraOff} disabled={pictureSource === "off"}>
          Turn camera off
        </button>
      </div>
      <p>{cameraStatus}</p>
      {pictureSource !== "off" ? (
        <video ref={videoRef} className="cameraPicture" autoPlay playsInline />
      ) : null}

      <section className="planFacts">
        <h2>Telemetry</h2>
        <p>{socketStatus}</p>
        <div className="telemetryColumns">
          <div>
            <p>Phase: {liveProgress?.phase || "waiting"}</p>
            <p>Latitude: {liveProgress?.latitude == null ? "waiting" : Number(liveProgress.latitude).toFixed(6)}</p>
            <p>Longitude: {liveProgress?.longitude == null ? "waiting" : Number(liveProgress.longitude).toFixed(6)}</p>
            <p>Height: {liveProgress?.heightFeet == null ? "waiting" : `${liveProgress.heightFeet} feet`}</p>
          </div>
          <div>
            <p>Lock point: {liveProgress?.lockPoint == null ? "waiting" : liveProgress.lockPoint}</p>
            <p>Time remaining: {liveProgress?.secondsRemaining == null ? "waiting" : `${liveProgress.secondsRemaining} seconds`}</p>
            <p>Inside the fence: {liveProgress?.insideFence == null ? "waiting" : liveProgress.insideFence ? "yes" : "no"}</p>
            <p>Inside the corridor: {liveProgress?.insideCorridor == null ? "waiting" : liveProgress.insideCorridor ? "yes" : "no"}</p>
          </div>
        </div>
      </section>

      <Link className="brightButton objectDetectionButton" to="/">
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

export default ObjectDetectionAutonomous
