import { StrictMode } from "react"
import { createRoot } from "react-dom/client"
import { BrowserRouter, Route, Routes } from "react-router-dom"
import "./index.css"
import "./App.css"
import App from "./App.jsx"
import CalibrateCompasses from "./calibration/compasses.jsx"
import CalibrateSensors from "./calibration/sensors.jsx"
import Fly10EastWestNorthSouth from "./droneflight/fly10ewns.jsx"
import Fly10EastWestNorthSouthTurns from "./droneflight/fly10ewns360.jsx"
import OneMotorAtATime from "./motortest/oneMotorAtATime.jsx"
import All4MotorsTest from "./motortest/all4MotorsTest.jsx"
import FlightPlan from "./geoPlanner/flightPlan.jsx"

createRoot(document.getElementById("root")).render(
  <StrictMode>
    <BrowserRouter>
      <Routes>
        <Route path="/" element={<App />} />
        <Route path="/motortest/one" element={<OneMotorAtATime />} />
        <Route path="/motortest/all" element={<All4MotorsTest />} />
        <Route path="/calibration/compasses" element={<CalibrateCompasses />} />
        <Route path="/calibration/sensors" element={<CalibrateSensors />} />
        <Route path="/fly/fly10ewns" element={<Fly10EastWestNorthSouth />} />
        <Route path="/fly/fly10ewns360" element={<Fly10EastWestNorthSouthTurns />} />
        <Route path="/geo/flight-plan" element={<FlightPlan />} />
      </Routes>
    </BrowserRouter>
  </StrictMode>,
)
