# Oct 16, 2026 — Handoff Notes for Next Agent / Computer

Written on Oct 16, 2026 so the next agent on another computer can pick up immediately with full hardware, flight-log, calibration, and script context.

---

## 1. Hardware, Wiring, and Access Rules (Do Not Change)

- **Airframe:** DJI Flame Wheel F450 quadcopter configured in ArduPilot as **standard Quad X** (`FRAME_CLASS = 1`, `FRAME_TYPE = 1`).
  - **IMPORTANT:** Never switch `FRAME_TYPE` to `13` (DJI X). Motor tests (`oct10-2026/motorOneTest.py` .. `motorFourTest.py`) verified that this copter's physical ESC wiring matches ArduPilot's standard Quad X order (`FRAME_TYPE = 1`).
- **Flight Controller:** Pixhawk 1 running **ArduCopter V4.7.0** (`1511f271`), ChibiOS `4f34e217`, u-blox GPS + external compass on mast.
- **No RC Transmitter:** `RC Protocol: None`. All flights are commanded by the Raspberry Pi over UART.
- **Command Cable (Flight Scripts):**
  - Port: `/dev/serial0` (`ttyAMA0`) at `57600` baud.
  - Wiring: Raspberry Pi GPIO **Pin 8 (TX)**, **Pin 10 (RX)**, **Pin 6 (Ground)**.
  - Telemetry does **not** stream automatically on `/dev/serial0`; scripts must send `MAV_CMD_SET_MESSAGE_INTERVAL` addressed to `target_system` / `target_component`.
- **USB Ports (Leave Alone):**
  - `/dev/ttyACM0` and `/dev/ttyACM1` belong to `/home/drone/Desktop/scripts/oct4-2026/bootDisplay.py` (`boot-display.service`), which drives the RedBoard OLED display and ultrasonic sensor. Never open USB ports from flight scripts and never kill `bootDisplay.py`.
- **Raspberry Pi SSH & Python Environment:**
  - Host: `192.168.0.138` (LAN)
  - User: `drone` | Password: `4663`
  - Flight / calibration Python interpreter on the Pi: `/home/drone/dronekit-venv/bin/python3` (Python 3.13 with `dronekit 2.9.2` and `pymavlink`).
  - Pi scripts root: `/home/drone/Desktop/scripts/`
  - Note on DroneKit `vehicle.last_heartbeat`: In DroneKit 2.9.2, `vehicle.last_heartbeat` is **already the age in seconds** (`monotonic.monotonic() - self._heartbeat_lastreceived`), **not** a Unix timestamp. Never subtract it from `time.time()`.

---

## 2. What Happened in Flight Log 49 (Why We Are Recalibrating)

We downloaded and analyzed Pixhawk DataFlash Log 49 (`oct13-2026/pixhawk-log-49.bin` and compressed `oct13-2026/pixhawk-log-49.bin.7z`) from the Oct 13 outdoor test of `hoverInOneSpot.py`:

1. **The script commands worked:** At `t = 397.8s`, the Pi armed the copter in `GUIDED` and commanded `simple_takeoff(0.914 m)` (3 feet). At `t = 402.0s` it switched to `LOITER`.
2. **What went wrong in the air:**
   - **Two compasses disagreed by ~74°:** Before takeoff, Compass 1 (`COMPASS_DEV_ID = 466441`, external GPS mast) read heading **−15.2°**, while Compass 2 (`COMPASS_DEV_ID2 = 658945`, internal Pixhawk board) read **−88.9°**. Compass 2 had uncalibrated default diagonal/off-diagonal matrices (`COMPASS_DIA2 = 1, 1, 1`).
   - **Compass pre-arm check was skipped:** `ARMING_SKIPCHK` was set to `4172` (`4 + 8 + 64 + 4096`), which skipped Compass (`4`), GPS lock (`8`), RC (`64`), and GPS Config (`4096`). That allowed the Pixhawk to arm despite the 74° compass disagreement.
   - **Motor current twisted the magnetic field on liftoff:** `COMPASS_MOTCT = 0` (motor interference compensation disabled). As the motors spun up at `t = 400.5s`, the magnetic field swung and at `t = 401.4s` EKF3 logged:
     `EKF3 IMU0 MAG0 ground mag anomaly, yaw re-aligned` (`EV 62 EKF_YAW_RESET`), jumping yaw from **3.3° to 64.5°**.
   - **LOITER position runaway & GPS/Compass conflict:** Because `LOITER` rotates North/East GPS corrections into body roll/pitch using the yaw heading, a ~60° heading error caused the copter to lean in the wrong direction, accelerating up to **4.1 m/s** and drifting **~14.7 m South** while climbing to **1.53 m (5.0 ft)** above home (`2.31 m` baro alt).
   - At `t = 411.7s`, the Pixhawk logged `ERR subsys=11 ecode=2` (`GPS Glitch or Compass error`) even though GPS had **11 satellites and HDOP 0.81**.
   - At `t = 412.2s`, the script saw the altitude pass the 4-foot ceiling and commanded `LAND`, but the copter was already pitching/rolling hard and tipped over at `t = 414.0s`.
   - **Onboard GCS failsafe was disabled:** `FS_GCS_ENABLE = 0.0` and `FS_OPTIONS = 16.0` (bit 4: continue in pilot modes on GCS failsafe).

---

## 3. Live Pixhawk Inspection Findings (Oct 16, 2026)

When we inspected the live Pixhawk parameters over `/dev/serial0` on Oct 16:
- **IMU 1 (Accelerometer & Gyroscope):** Calibrated (`INS_ACCOFFS_X = -0.154, Y = -0.294, Z = -0.422`, `INS_ACCSCAL = 0.991, 0.998, 1.003`). Live bench attitude was flat (`Roll = -0.35°`, `Pitch = -2.32°`).
- **EKF3 Sources:** Already set to `EK3_SRC1_POSXY = 3` (GPS), `EK3_SRC1_VELXY = 3` (GPS), `EK3_SRC1_POSZ = 1` (Baro), `EK3_SRC1_YAW = 1` (Compass).
- **Compasses:** Still disagreed on the bench (`Compass 1` vs `Compass 2`), and `ARMING_SKIPCHK` was still `4172`.
- **Pre-Arm Check on the Bench:** Reported `PreArm: Hardware safety switch` and `PreArm: Battery 1 low voltage failsafe` (because only 5V USB was powering the Pixhawk on the bench, not the flight LiPo battery).

---

## 4. What Was Built in `oct16-2026/` (And Synced to the Pi)

All files below exist both on the desktop (`/home/sunny/Desktop/drone/oct16-2026/`) and on the Raspberry Pi (`/home/drone/Desktop/scripts/oct16-2026/`), compiled and marked executable (`chmod 775`):

### A. `oct16-2026/reCalibrateDrone.py`
Interactive step-by-step terminal wizard to recalibrate the drone over SSH without needing Mission Planner:
- **Logs every run automatically** to `oct16-2026/calibrationLogs/reCalibrateDrone-YYYY-MM-DD-HHMMSS.log`, including a full `CALIBRATION RUN OUTCOME SUMMARY` at the end whether it succeeds, fails, or is interrupted.
- **Step 1:** Sets `EK3_SRC1_POSXY = 3` (GPS), `EK3_SRC1_VELXY = 3` (GPS), `EK3_SRC1_POSZ = 1` (Baro), `EK3_SRC1_YAW = 1` (Compass), `FS_GCS_ENABLE = 5` (Land), `FS_GCS_TIMEOUT = 5.0s`, clears `FS_OPTIONS` bit 4 (`16`), sets `COMPASS_AUTO_ROT = 2`, and sets `ARMING_SKIPCHK = 64` (skips **only** RC Channels; **re-enables Compass and GPS pre-arm checks**).
- **Step 2:** Dynamically inspects all 3 compass slots (`COMPASS_DEV_ID*`, `COMPASS_EXTERNAL` / `COMPASS_EXTERN*`) and lets the user choose:
  - **Option 1:** Enable and calibrate all detected compasses (External GPS mast Primary + Internal board Secondary).
  - **Option 2:** Enable and calibrate **only** the external GPS mast compass(es) (`COMPASS_EXTERNAL = 1` or `2`) and disable internal compass(es).
- **Step 3:** Runs level horizon trim calibration (`MAV_CMD_PREFLIGHT_CALIBRATION`, `param5 = 2`) while the drone sits flat, requiring `MAV_RESULT_ACCEPTED`.
- **Step 4:** Runs onboard 360° sphere/ellipsoid compass calibration (`MAV_CMD_DO_START_MAG_CAL`) with a live `0% -> 100%` ASCII progress bar for every expected compass, retains the latest `MAG_CAL_REPORT`, sends `MAV_CMD_DO_ACCEPT_MAG_CAL` if needed, and verifies that the Pixhawk's stored `COMPASS_OFS*`, `COMPASS_DIA*`, and `COMPASS_ODI*` parameters match the new report.
- **Step 5:** Offers to reboot the Pixhawk (`MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`) so EKF3 loads the new compass offsets, then runs `MAV_CMD_RUN_PREARM_CHECKS` and checks post-ACK `SYS_STATUS` (`MAV_SYS_STATUS_PREARM_CHECK` enabled & healthy bits) + `STATUSTEXT`.

### B. `oct16-2026/beforeCalibrationMotorTest.py`
Props-off motor check to run before compass calibration. No GPS, compass, or EKF is required. The copter stays disarmed.
- Spins one motor at a time, in order: motor 1 (front-right, CCW, test order 1), motor 2 (back-left, CCW, test order 3), motor 3 (front-left, CW, test order 4), motor 4 (back-right, CW, test order 2).
- Each motor runs **20 seconds**, ramping from **10 percent to 25 percent**, with a 2-second pause between motors.
- Requires typed `YES`, a solid safety-switch LED, and propellers removed. Ctrl+C stops the motor that is spinning.
- This script does not write a flight log.

### C. `oct16-2026/afterCalibrationMotorTest.py`
The same 20-second, 10-to-25-percent, one-motor-at-a-time test, run again after calibration.
- Saves the next numbered file in `oct16-2026/flightLog/afterCalibration-motorTest-#1.log`, then `#2`, and so on.
- For each motor the log records accept or refuse, commanded percent over the 20 seconds, battery voltage, battery current, and the PWM sent to that motor output, plus a run summary (`SUCCESS`, `FAILED`, or `STOPPED PART WAY`).

### D. `oct16-2026/stationaryHoverTest.py`
Updated version of `oct13-2026/hoverInOneSpot.py`:
- Takes off in `GUIDED` to **3.0 feet** (`0.91 m`), hovers in `LOITER` in one GPS spot for **30 seconds**, and lands (`LAND`).
- Enforces the **4.0-foot** (`1.22 m`) and **1.3 m** ceilings and an **8.0-foot** horizontal drift limit at **5 Hz** (`0.2 s` loop).
- Configures the onboard ground-station failsafe (`FS_GCS_ENABLE = 5` Land, `FS_OPTIONS` bit 4 cleared) before arming.
- If the Pixhawk heartbeat is lost, closes `/dev/serial0` so DroneKit stops sending GCS heartbeats and the Pixhawk lands itself after 5 seconds.
- Saves a log at `oct16-2026/flightLog/stationaryHoverTest-YYYY-MM-DD-HHMMSS.log`.

### E. `oct16-2026/flyForward10FeetLand.py`
Updated version of `oct15-2026/takeOffFlyForwardLand.py` for the 4-foot mission:
- **Logs every flight automatically** (successful, failed, or exited mid-way) to `oct16-2026/flightLog/flyForward10FeetLand-YYYY-MM-DD-HHMMSS.log` with a full `FLIGHT RUN OUTCOME SUMMARY`.
- **Mission sequence:**
  1. Connect, verify Quad X, 3D GPS fix, ≥ 6 satellites, trusted EKF position, level within 5°, and ask for typed `YES`.
  2. Configure `TKOFF_SLEW_TIME` (≤ 1.0s) and onboard `FS_GCS_ENABLE = 5` (Land) failsafe.
  3. **Step 1:** Arm in `GUIDED` and climb to **4.0 feet** (`1.22 m`). Safety ceiling is **5.0 feet** (`1.52 m`).
  4. **Step 2:** Hover stationary at **4.0 feet** in `LOITER` for **10 seconds** (monitoring 5.0 ft ceiling and 8.0 ft drift limit at 5 Hz).
  5. **Step 3:** Switch to `GUIDED` and fly **10.0 feet North** at **0.4 m/s** (`MAV_FRAME_LOCAL_NED`) while actively holding **4.0 feet** altitude (`altitudeHoldDownSpeedMetersPerSecond`), then command zero velocity for **2 seconds** to stop cleanly.
  6. **Step 4:** Hover stationary at **4.0 feet** in `LOITER` for **10 seconds**.
  7. **Step 5:** Switch to `LAND` and wait until `vehicle.armed` is `False` (propellers completely stopped).

---

## 5. What Needs to Be Done Next (Step-by-Step Instructions)

### Step 1 — Props-off motor check before calibration
1. Remove the propellers and press the safety switch until the LED is solid.
2. Run:
   ```bash
   /home/drone/dronekit-venv/bin/python3 /home/drone/Desktop/scripts/oct16-2026/beforeCalibrationMotorTest.py
   ```
3. Type `YES`. Watch motor 1, then 2, then 3, then 4. Each should turn the direction printed by the script while the throttle rises from 10 percent to 25 percent.

### Step 2 — Physical compass check, then calibrate outside
1. Verify the **arrow on the external GPS/compass puck** points straight forward in the **exact same direction as the arrow on the Pixhawk**.
2. Ensure the thin GPS/compass cable is routed away from the high-current battery and 4-in-1 ESC power leads.

### Step 3 — Run `reCalibrateDrone.py` Outside
1. Take the drone outside away from cars, metal tables, and concrete rebar.
2. SSH into the Pi (`ssh drone@192.168.0.138`, password `4663`) and run:
   ```bash
   /home/drone/dronekit-venv/bin/python3 /home/drone/Desktop/scripts/oct16-2026/reCalibrateDrone.py
   ```
3. Follow the prompts:
   - **Step 1:** Press `Y` to apply EKF GPS, Land failsafe (`FS_GCS_ENABLE = 5`), and re-enable Compass/GPS pre-arm checks (`ARMING_SKIPCHK = 64`).
   - **Step 2:** Choose **Option 1** (calibrate both external + internal compasses) or **Option 2** (use only the external GPS mast compass if the internal compass fails or has too much magnetic interference).
   - **Step 3:** Place the drone flat and level and press `Y` to calibrate level horizon trim.
   - **Step 4:** Press `Y`, pick up the drone, and slowly rotate it 360° through all 6 orientations (flat, nose down, nose up, left side, right side, upside down) until the progress bar reaches `100%` and reports `SUCCESS`.
   - **Step 5:** Press `Y` to reboot the Pixhawk and review the Pre-Arm check output and saved log in `/home/drone/Desktop/scripts/oct16-2026/calibrationLogs/`.
4. **Sanity check:** Point the drone's nose North and verify `Fused Heading` is near `0°`.

### Step 4 — Props-off motor check after calibration
1. Propellers stay off. Safety switch LED solid.
2. Run:
   ```bash
   /home/drone/dronekit-venv/bin/python3 /home/drone/Desktop/scripts/oct16-2026/afterCalibrationMotorTest.py
   ```
3. Type `YES`. Compare the new `flightLog/afterCalibration-motorTest-#N.log` with what was seen before calibration. Each motor block should say `COMPLETED`.

### Step 5 — Run the 30-Second Stationary Hover Test (`stationaryHoverTest.py`)
1. Connect a charged flight battery, place the drone on level ground in a clear outdoor area, and press the hardware safety switch.
2. Run:
   ```bash
   /home/drone/dronekit-venv/bin/python3 /home/drone/Desktop/scripts/oct16-2026/stationaryHoverTest.py
   ```
3. When preflight checks pass, type `YES`.
4. Confirm it climbs to 3 feet, holds one spot cleanly in `LOITER` for 30 seconds without yaw twist or drift, and lands. The log is in `flightLog/stationaryHoverTest-YYYY-MM-DD-HHMMSS.log`.

### Step 6 — Run the 4-Foot North Mission (`flyForward10FeetLand.py`)
1. Once stationary hover is solid, run:
   ```bash
   /home/drone/dronekit-venv/bin/python3 /home/drone/Desktop/scripts/oct16-2026/flyForward10FeetLand.py
   ```
2. When preflight checks pass, type `YES`.
3. It will climb to 4 ft, hover 10s, fly 10 ft North at 4 ft, hover 10s, land until propellers stop, and save its full log in `/home/drone/Desktop/scripts/oct16-2026/flightLog/`.

### Step 7 — Copy Logs Back & Check Pixhawk DataFlash Log
1. Copy the new logs from `/home/drone/Desktop/scripts/oct16-2026/calibrationLogs/` and `/home/drone/Desktop/scripts/oct16-2026/flightLog/` back to the desktop repo.
2. If needed, download the newest Pixhawk DataFlash `.bin` log (Log 50+) over `/dev/serial0` and verify there are no `ground mag anomaly` or `GPS Glitch or Compass error` events.
