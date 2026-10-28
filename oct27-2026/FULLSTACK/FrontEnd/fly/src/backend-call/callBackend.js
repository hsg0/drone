import axios from "axios"

const defaultHost = typeof window !== "undefined" && window.location.hostname ? window.location.hostname : "127.0.0.1"
const configuredUrl = import.meta.env.VITE_BACKEND_URL
export const backendBaseUrl =
  configuredUrl && defaultHost !== "127.0.0.1" && defaultHost !== "localhost"
    ? configuredUrl.replace(/127\.0\.0\.1|localhost/, defaultHost)
    : configuredUrl || `http://${defaultHost}:4020`

const backendClient = axios.create({
  baseURL: backendBaseUrl,
  headers: {
    "Content-Type": "application/json",
  },
})

export async function getFromBackend(path) {
  const response = await backendClient.get(path)
  return response.data
}

export async function postToBackend(path, body) {
  const response = await backendClient.post(path, body)
  return response.data
}

export function readBackendHealth() {
  return getFromBackend("/")
}

export function readFlightLogs() {
  return getFromBackend("/flights")
}

export function readOneFlightLog(flightLogId) {
  return getFromBackend(`/flights/${flightLogId}`)
}

export function saveFlightLog(body) {
  return postToBackend("/flights", body)
}

export function fly10FeetEastWestNorthSouthLand() {
  return postToBackend("/flights/fly10FeetEWNSLand", { confirm: "YES" })
}

export function fly10FeetTurnMission() {
  return postToBackend("/flights/fly10FeetE360W360N360S360Land", { confirm: "YES" })
}

export function calibrateCompasses() {
  return postToBackend("/calibration/compasses", { confirm: "YES" })
}

export function saveFlightPlan(plan) {
  return postToBackend("/flight-plans", plan)
}

export function armForFlightPlan() {
  return postToBackend("/flight-plans/arm", {})
}

export function disarmForFlightPlan() {
  return postToBackend("/flight-plans/disarm", {})
}

export function flySavedFlightPlan(planId) {
  return postToBackend("/flight-plans/fly", { confirm: "YES", planId })
}

export function savePerimeterPlan(plan) {
  return postToBackend("/flight-plans/perimeter", plan)
}

export function flySavedPerimeter(planId) {
  return postToBackend("/flight-plans/perimeter/fly", { confirm: "YES", planId })
}

export function spinOneMotor(testOrder, percent) {
  return postToBackend("/motortest/one", { confirm: "YES", testOrder, percent })
}

export function rampAllMotors() {
  return postToBackend("/motortest/all", { confirm: "YES" })
}

export function stopMotorTest() {
  return postToBackend("/motortest/stop", {})
}

export function readMotorTestProgress() {
  return getFromBackend("/motortest/progress")
}

export function readSeenGesture() {
  return getFromBackend("/camera/seen")
}

export function offerCameraConnection(offer) {
  return postToBackend("/camera/webrtc/offer", offer)
}

export function closeCameraConnection() {
  return postToBackend("/camera/webrtc/close", {})
}

export function watchCalibrationProgress(onProgress, onConnection) {
  const liveUrl = backendBaseUrl.replace(/^http/, "ws") + "/calibration/compasses/live"
  const socket = new WebSocket(liveUrl)
  socket.onopen = () => onConnection("open")
  socket.onclose = () => onConnection("closed")
  socket.onerror = () => onConnection("error")
  socket.onmessage = (event) => {
    try {
      onProgress(JSON.parse(event.data))
    } catch {
      onConnection("error")
    }
  }
  return () => socket.close()
}
