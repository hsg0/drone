import { useEffect, useRef, useState } from "react"
import { Link } from "react-router-dom"
import { closeCameraConnection, offerCameraConnection, readSeenGesture } from "../backend-call/callBackend.js"

const ARM_POSES = [
  { name: "takeoff-land", pose: "Wide arms overhead in a V", command: "Take off or land" },
  { name: "fly-up", pose: "Arms straight up", command: "Fly up" },
  { name: "fly-down", pose: "Arms coming down", command: "Fly down" },
  { name: "move-away", pose: "Arms out", command: "Fly farther" },
  { name: "move-closer", pose: "Hands in", command: "Fly closer" },
  { name: "hover", pose: "Arms down", command: "Hover" },
]

const ICE_WAIT_MILLISECONDS = 3000

function statusForSource(sourceName) {
  if (sourceName === "camera") return "WebRTC is connected to the forward camera."
  return "The live camera did not open. Showing the saved clip over WebRTC. The live camera is on the Pi. Close any other program using the camera."
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

function HumanGestureControl() {
  const videoRef = useRef(null)
  const peerRef = useRef(null)
  const [pictureSource, setPictureSource] = useState("off")
  const [statusText, setStatusText] = useState("The camera is off. Turning it on does not arm the copter.")
  const [seenGesture, setSeenGesture] = useState(null)

  async function stopConnection() {
    const peer = peerRef.current
    peerRef.current = null
    if (videoRef.current) videoRef.current.srcObject = null
    if (peer) peer.close()
    try {
      await closeCameraConnection()
    } catch {
      // The backend may already have closed this connection.
    }
  }

  useEffect(() => {
    return () => {
      const peer = peerRef.current
      peerRef.current = null
      if (peer) peer.close()
      closeCameraConnection().catch(() => {})
    }
  }, [])

  useEffect(() => {
    if (pictureSource === "off") return undefined
    let pageIsOpen = true
    const timer = setInterval(() => {
      readSeenGesture()
        .then((body) => {
          if (pageIsOpen) setSeenGesture(body)
        })
        .catch(() => {
          if (pageIsOpen) setSeenGesture(null)
        })
    }, 500)
    return () => {
      pageIsOpen = false
      clearInterval(timer)
    }
  }, [pictureSource])

  async function turnCameraOn() {
    setSeenGesture(null)
    setStatusText("Opening the WebRTC connection.")
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
        setStatusText("The WebRTC connection failed.")
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
      setStatusText(statusForSource(answer.source))
    } catch (error) {
      if (peerRef.current === peer) peerRef.current = null
      peer.close()
      closeCameraConnection().catch(() => {})
      setPictureSource("off")
      setStatusText(error?.response?.data?.detail || error?.message || "The WebRTC connection did not start.")
    }
  }

  async function turnCameraOff() {
    await stopConnection()
    setPictureSource("off")
    setSeenGesture(null)
    setStatusText("The camera is off. Turning it on does not arm the copter.")
  }

  return (
    <main className="screen">
      <h1>Human gesture control</h1>
      <p>
        Stand in front of the nose so the camera sees your upper body. The yellow words are the pose YOLO sees. This page does not arm and does not fly.
      </p>
      <div className="buttonRow">
        <button className="brightButton gestureButton" type="button" onClick={turnCameraOn} disabled={pictureSource !== "off"}>
          Turn camera on
        </button>
        <button className="brightButton gestureButton" type="button" onClick={turnCameraOff} disabled={pictureSource === "off"}>
          Turn camera off
        </button>
      </div>
      <p>{statusText}</p>
      {pictureSource !== "off" ? (
        <video ref={videoRef} className="cameraPicture" autoPlay playsInline />
      ) : null}
      {seenGesture ? (
        <p className="currentDirection">
          {seenGesture.pose || "not seen"} — {seenGesture.command || "no command"}
        </p>
      ) : null}
      <ol className="positionList">
        {ARM_POSES.map((armPose) => (
          <li key={armPose.name} className={seenGesture?.gesture === armPose.name ? "currentPosition" : ""}>
            <strong>{armPose.pose}</strong>
            <span>{armPose.command}</span>
          </li>
        ))}
      </ol>
      <Link className="brightButton gestureButton" to="/">
        Back to dashboard
      </Link>
    </main>
  )
}

export default HumanGestureControl
