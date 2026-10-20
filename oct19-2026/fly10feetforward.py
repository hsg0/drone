#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    1. Take off to 4 feet in height and settle for 2 seconds.
    2. Hover stationary at 4 feet for 10 seconds.
    3. Fly forward in the Northern direction for 10 feet at 4 feet in height.
    4. Hover stationary at 4 feet for 10 seconds.
    5. Land safely and wait until the propellers stop spinning (motors disarm).
    6. Save a complete flight log and outcome summary of every run in
       oct19-2026/flightLog/.

WHY:
    Earlier runs drifted past 8 feet right after takeoff because:
      - Compass 1 was uncalibrated and ARMING_SKIPCHK or compass variance was not
        checked before takeoff.
      - AHRS_TRIM_X / AHRS_TRIM_Y had a 3.6-degree tilt saved from an uneven
        calibration surface, making the drone lean on liftoff.
      - Step 1 switched from GUIDED to LOITER the instant altitude crossed 3.8 ft
        while the drone was still climbing at 2 ft/s, carrying momentum into hover.
    This file fixes all three mistakes and runs the entire flight inline in main()
    so you can read the sequence from top to bottom without jumping between helpers.

HOW:
    1. Open a timestamped log file in oct19-2026/flightLog/.
    2. Connect to the Pixhawk on /dev/serial0 (pins 8 and 10 at 57600 baud).
    3. Verify Quad X frame, Compass/GPS pre-arm checks enabled (ARMING_SKIPCHK=64),
       saved level trim within 2.5 degrees, live tilt within 5 degrees, 3D GPS fix
       with at least 6 satellites, and trusted EKF horizontal position.
    4. Ask the pilot to type YES, shorten TKOFF_SLEW_TIME to 1.0s if slower, and
       enable the onboard Land failsafe (FS_GCS_ENABLE=5, FS_OPTIONS bit 4 cleared).
    5. Step 1: Arm in GUIDED, climb to 4 feet, and settle for 2 seconds in GUIDED
       before switching to LOITER.
    6. Step 2: Hold stationary in LOITER at 4 feet for 10 seconds.
    7. Step 3: Fly North 10 feet in GUIDED at 0.4 m/s while holding 4 feet height,
       then command zero speed for 2 seconds to stop cleanly.
    8. Step 4: Hold stationary in LOITER at 4 feet for 10 seconds.
    9. Step 5: Switch to LAND and wait until the motors disarm (propellers stop).

IMPORTANT:
    Run on the Pi with:
        /home/drone/Desktop/scripts/oct19-2026/fly10feetforward.py
    Run /home/drone/Desktop/scripts/oct19-2026/reCalibrateDrone.py first if level
    trim or compass calibration is out of date.
"""

import math  # math helpers for degrees,trig, and distance
import os  # file and folder paths
import sys  # terminal output stream
import time  # timers and timestamps

from dronekit import VehicleMode  # Pixhawk flight mode object
from dronekit import connect  # connect to Pixhawk over serial
from pymavlink import mavutil as middleMan  # MAVLink constants and messages


# --------------------------------------------------
# CONNECTION AND FLIGHT SETTINGS
# --------------------------------------------------

PIXHAWK_PORT = "/dev/serial0"  # UART on Pi pins 8 (TX) and 10 (RX)
PIXHAWK_BAUD = 57600  # UART baud rate
CONNECTION_TIMEOUT_SECONDS = 60  # max wait to connect
GROUND_STATION_SYSTEM_ID = 255  # GCS MAVLink system ID

FLIGHT_LOG_DIRECTORY = os.path.join(  # folder for flight logs
    os.path.dirname(os.path.abspath(__file__)),  # current script folder
    "flightLog",  # subfolder name
)  # end of log folder path

FEET_PER_METER = 3.28084  # feet in 1 meter
HOVER_ALTITUDE_FEET = 4.0  # target altitude in feet
HOVER_ALTITUDE_METERS = HOVER_ALTITUDE_FEET / FEET_PER_METER  # target altitude in meters
TAKEOFF_REACHED_FRACTION = 0.95  # 95% of 4 ft counts as reached
TAKEOFF_SETTLE_SECONDS = 2.0  # pause at 4 ft in GUIDED so climb momentum stops before LOITER

MAXIMUM_ALTITUDE_CEILING_FEET = 5.0  # hard ceiling 1 ft above 4 ft target
MAXIMUM_ALTITUDE_CEILING_METERS = MAXIMUM_ALTITUDE_CEILING_FEET / FEET_PER_METER  # ceiling in meters

HOVER_DURATION_SECONDS = 10.0  # hold each hover for 10 seconds
MAXIMUM_HOVER_DRIFT_FEET = 8.0  # abort hover if drift exceeds 8 feet
MAXIMUM_HOVER_DRIFT_METERS = MAXIMUM_HOVER_DRIFT_FEET / FEET_PER_METER  # max drift in meters

NORTH_DISTANCE_FEET = 10.0  # fly North 10 feet
NORTH_DISTANCE_METERS = NORTH_DISTANCE_FEET / FEET_PER_METER  # 10 feet in meters
NORTH_SPEED_METERS_PER_SECOND = 0.4  # forward North speed in m/s
ALTITUDE_HOLD_GAIN_DURING_MOVE = 0.5  # vertical speed gain to hold 4 ft
MAXIMUM_VERTICAL_CORRECTION_METERS_PER_SECOND = 0.25  # max vertical correction speed
MAXIMUM_OFF_COURSE_FEET = 8.0  # max East/West or South drift during North leg
MAXIMUM_OFF_COURSE_METERS = MAXIMUM_OFF_COURSE_FEET / FEET_PER_METER  # off-course limit in meters
NORTH_MOVE_TIMEOUT_SECONDS = 20.0  # max seconds allowed for the 10-ft North move
SETTLE_AFTER_MOVE_SECONDS = 2.0  # brake at zero speed for 2s after North move
METERS_PER_DEGREE_OF_LATITUDE = 111319.5  # meters per degree of latitude
MINIMUM_VALID_COORDINATE_DEGREES = 0.01  # reject 0,0 uninitialized GPS coordinates

TAKEOFF_TIMEOUT_SECONDS = 30.0  # max seconds allowed for takeoff
LAND_TIMEOUT_SECONDS = 60.0  # max seconds allowed for landing disarm
MODE_CHANGE_TIMEOUT_SECONDS = 10.0  # max seconds allowed for a mode switch
ARM_TIMEOUT_SECONDS = 15.0  # max seconds allowed for arming
HEARTBEAT_STALE_AFTER_SECONDS = 3.0  # heartbeat age limit in seconds
MAXIMUM_GROUND_LEAN_DEGREES = 5.0  # max live roll/pitch tilt before takeoff
MAXIMUM_SAVED_TRIM_DEGREES = 2.5  # reject flight if saved AHRS_TRIM exceeds 2.5 deg (tilted cal)

STATUS_CHECK_INTERVAL_SECONDS = 0.2  # loop check interval (5 Hz)
PRINT_INTERVAL_SECONDS = 1.0  # terminal print interval (1 Hz)
PARAMETER_TOLERANCE = 0.01  # float comparison tolerance for parameters

QUAD_FRAME_CLASS = 1  # ArduPilot Quad frame class
QUAD_X_FRAME_TYPE = 1  # ArduPilot Quad X frame type (not DJI X 13)
MINIMUM_GPS_FIX_TYPE = 3  # 3D GPS fix required
MINIMUM_SATELLITE_COUNT = 6  # minimum visible satellites required
UNKNOWN_SATELLITE_COUNT = 255  # MAVLink unknown satellite value
SKIP_COMPASS_OR_GPS_MASK = 4 | 8 | 4096  # ARMING_SKIPCHK bits that must NOT be set

TAKEOFF_SLEW_PARAMETER_NAME = "TKOFF_SLEW_TIME"  # takeoff throttle ramp parameter
PREFERRED_TAKEOFF_SLEW_SECONDS = 1.0  # 1.0s ramp prevents slow tip-over on liftoff

GROUND_STATION_FAILSAFE_PARAMETER_NAME = "FS_GCS_ENABLE"  # GCS failsafe enable parameter
GROUND_STATION_FAILSAFE_TIMEOUT_PARAMETER_NAME = "FS_GCS_TIMEOUT"  # GCS failsafe timeout parameter
FAILSAFE_OPTIONS_PARAMETER_NAME = "FS_OPTIONS"  # failsafe options bitmask parameter
GROUND_STATION_FAILSAFE_LAND_VALUE = 5  # 5 = Always Land on GCS failsafe
CONTINUE_PILOT_MODE_ON_FAILSAFE_BIT = 16  # bit 4 (16) must be cleared so LOITER lands on failsafe
PREFERRED_FAILSAFE_TIMEOUT_SECONDS = 5.0  # 5-second failsafe timeout

MESSAGE_INTERVAL_SECONDS = 0.1  # request 10 Hz telemetry streams
MICROSECONDS_PER_SECOND = 1000000  # microseconds per second
FLIGHT_MESSAGE_NAMES = (  # MAVLink messages to stream
    "GPS_RAW_INT",  # GPS fix and satellite count
    "GLOBAL_POSITION_INT",  # relative altitude and coordinates
    "EKF_STATUS_REPORT",  # EKF health flags
    "ATTITUDE",  # roll, pitch, yaw
    "STATUSTEXT",  # Pixhawk status and PreArm text
)  # end of message list

VELOCITY_ONLY_TYPE_MASK = (  # bitmask to use velocity only in SET_POSITION_TARGET_LOCAL_NED
    middleMan.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE  # ignore North position
    | middleMan.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE  # ignore East position
    | middleMan.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE  # ignore Down position
    | middleMan.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE  # ignore North acceleration
    | middleMan.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE  # ignore East acceleration
    | middleMan.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE  # ignore Down acceleration
    | middleMan.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE  # ignore yaw angle
    | middleMan.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE  # ignore yaw rate
)  # end of velocity mask


# --------------------------------------------------
# TERMINAL AND FILE LOGGING TEE
# --------------------------------------------------

class FlightLog:  # tee output to both terminal and log file
    """Copy every printed line to the terminal and to the flight log file."""

    def __init__(self, logFile, terminal):  # store file and terminal handles
        self.logFile = logFile  # opened log file
        self.terminal = terminal  # original sys.stdout

    def write(self, text):  # write to both outputs
        self.terminal.write(text)  # print to screen
        self.terminal.flush()  # flush screen
        self.logFile.write(text)  # write to file
        self.logFile.flush()  # flush file

    def writeLogOnly(self, text):  # write input echo to log file only
        self.logFile.write(text)  # write to file
        self.logFile.flush()  # flush file

    def flush(self):  # flush both streams
        self.terminal.flush()  # flush screen
        self.logFile.flush()  # flush file


# --------------------------------------------------
# SMALL CORE PIXHAWK PRIMITIVES
# --------------------------------------------------

def heartbeatIsFresh(vehicle):  # check if Pixhawk heartbeat arrived within 3s
    heartbeatAgeSeconds = vehicle.last_heartbeat  # age in seconds from DroneKit
    if heartbeatAgeSeconds is None:  # no heartbeat yet
        return False  # not fresh
    return heartbeatAgeSeconds <= HEARTBEAT_STALE_AFTER_SECONDS  # True when <= 3.0s


def setFlightMode(vehicle, flightModeName):  # switch mode and confirm it
    print(f"Requesting {flightModeName}.")  # log requested mode
    vehicle.mode = VehicleMode(flightModeName)  # send mode command
    startTime = time.monotonic()  # start mode timeout timer
    while time.monotonic() - startTime < MODE_CHANGE_TIMEOUT_SECONDS:  # wait up to 10s
        if vehicle.mode.name == flightModeName:  # Pixhawk confirmed mode
            print(f"Mode confirmed: {flightModeName}")  # log confirmation
            return True  # done
        if not heartbeatIsFresh(vehicle):  # heartbeat lost during wait
            raise RuntimeError("Pixhawk heartbeat was lost.")  # abort
        time.sleep(STATUS_CHECK_INTERVAL_SECONDS)  # wait 0.2s
    raise RuntimeError(f"Could not enter {flightModeName}.")  # timed out


def setParameterAndConfirm(vehicle, parameterName, wantedValue):  # write parameter and verify
    currentValue = vehicle.parameters[parameterName]  # read current value
    if currentValue is not None and abs(float(currentValue) - float(wantedValue)) < PARAMETER_TOLERANCE:  # already set
        return float(currentValue)  # return existing value
    wasWritten = vehicle.parameters.set(parameterName, wantedValue)  # send parameter write
    confirmedValue = vehicle.parameters[parameterName]  # read back stored value
    if (  # check if write failed or value did not stick
        not wasWritten  # DroneKit timed out
        or confirmedValue is None  # missing readback
        or abs(float(confirmedValue) - float(wantedValue)) >= PARAMETER_TOLERANCE  # mismatch
    ):  # end of parameter verification condition
        raise RuntimeError(f"{parameterName} did not stay at {float(wantedValue):.1f}.")  # abort
    return float(confirmedValue)  # return confirmed value


def sendLocalVelocity(vehicle, northMetersPerSecond, eastMetersPerSecond, downMetersPerSecond):  # send NED speed
    message = vehicle.message_factory.set_position_target_local_ned_encode(  # build MAVLink velocity packet
        0,  # time_boot_ms unused
        vehicle._master.target_system,  # Pixhawk system ID
        vehicle._master.target_component,  # Pixhawk component ID
        middleMan.mavlink.MAV_FRAME_LOCAL_NED,  # Earth North-East-Down frame
        VELOCITY_ONLY_TYPE_MASK,  # use velocities only
        0,  # North position ignored
        0,  # East position ignored
        0,  # Down position ignored
        northMetersPerSecond,  # North speed (m/s)
        eastMetersPerSecond,  # East speed (m/s)
        downMetersPerSecond,  # Down speed (m/s, negative climbs)
        0,  # North accel ignored
        0,  # East accel ignored
        0,  # Down accel ignored
        0,  # yaw ignored
        0,  # yaw rate ignored
    )  # end of velocity packet
    vehicle.send_mavlink(message)  # queue packet
    vehicle.flush()  # send immediately


def checkFlightHealthAndCeiling(vehicle, flightOutcome):  # check link, arm, EKF, and 5-ft ceiling
    if not heartbeatIsFresh(vehicle):  # link lost
        raise RuntimeError("Pixhawk heartbeat was lost.")  # abort to land
    if not vehicle.armed:  # motors disarmed unexpectedly
        raise RuntimeError("Copter disarmed unexpectedly in flight.")  # abort
    if not vehicle.ekf_ok:  # EKF lost horizontal trust
        raise RuntimeError("Position estimate lost in flight.")  # abort to land
    location = vehicle.location.global_relative_frame  # read relative frame
    altitudeMeters = None  # default when missing
    if location is not None and location.alt is not None and math.isfinite(float(location.alt)):  # valid number
        altitudeMeters = float(location.alt)  # store height in meters
        previousMax = getattr(vehicle, "maximumAltitudeMeters", None)  # previous max height
        if previousMax is None or altitudeMeters > previousMax:  # new peak height
            vehicle.maximumAltitudeMeters = altitudeMeters  # update peak height
            flightOutcome["maximumAltitudeReachedText"] = (  # save formatted peak height
                f"{altitudeMeters:.2f} m ({altitudeMeters * FEET_PER_METER:.2f} ft)"  # meters and feet
            )  # end of peak height string
        if altitudeMeters >= MAXIMUM_ALTITUDE_CEILING_METERS:  # hit 5.0-ft safety ceiling
            raise RuntimeError(  # abort and land immediately
                f"Safety ceiling ({MAXIMUM_ALTITUDE_CEILING_FEET:.0f} ft) reached: "  # error prefix
                f"{altitudeMeters:.2f} m ({altitudeMeters * FEET_PER_METER:.2f} ft)"  # height reached
            )  # end of ceiling error
    return altitudeMeters  # return current height in meters


# --------------------------------------------------
# MAIN INLINE FLIGHT WALKTHROUGH
# --------------------------------------------------

def main():  # run the full flight sequence from top to bottom
    originalStandardOutput = sys.stdout  # save original terminal stream
    logFile = None  # log file handle
    logPath = None  # log file path
    flightLogger = None  # tee logger instance
    vehicle = None  # DroneKit vehicle handle
    originalTakeoffSlewSeconds = None  # original TKOFF_SLEW_TIME if modified
    savedFailsafe = None  # original (FS_GCS_ENABLE, FS_GCS_TIMEOUT, FS_OPTIONS)
    copterWasArmed = False  # True once arming begins
    landingWasStarted = False  # True once Step 5 landing begins

    flightOutcome = {  # outcome record for the final summary
        "startedAt": time.strftime("%Y-%m-%d %H:%M:%S %Z"),  # start timestamp
        "finishedAt": None,  # end timestamp
        "overallOutcome": "FAILED",  # default until completed
        "preflightAndFailsafe": "NOT STARTED",  # preflight status
        "step1TakeoffTo4Feet": "NOT STARTED",  # Step 1 status
        "step2FirstHover10Seconds": "NOT STARTED",  # Step 2 status
        "step3FlyNorth10Feet": "NOT STARTED",  # Step 3 status
        "step4SecondHover10Seconds": "NOT STARTED",  # Step 4 status
        "step5LandAndStopProps": "NOT STARTED",  # Step 5 status
        "maximumAltitudeReachedText": "none",  # highest altitude recorded
        "northDistanceTraveledText": "0.0 ft",  # North distance recorded
        "stopReason": None,  # reason if stopped early
    }  # end of outcome dictionary

    try:  # main flight try block
        # ==================================================
        # OPEN FLIGHT LOG FILE
        # ==================================================
        os.makedirs(FLIGHT_LOG_DIRECTORY, exist_ok=True)  # ensure flightLog folder exists
        scriptName = os.path.splitext(os.path.basename(__file__))[0]  # script name without .py
        startedAtText = time.strftime("%Y-%m-%d-%H%M%S")  # timestamp for file name
        logPath = os.path.join(FLIGHT_LOG_DIRECTORY, f"{scriptName}-{startedAtText}.log")  # full log path
        logFile = open(logPath, "w", encoding="utf-8")  # open log file for writing
        flightLogger = FlightLog(logFile, sys.stdout)  # wrap stdout in FlightLog
        sys.stdout = flightLogger  # redirect all print() calls to terminal + file

        print(f"Flight log file : {logPath}")  # print log path
        print(f"Run started at  : {time.strftime('%Y-%m-%d %H:%M:%S %Z')}")  # print start time
        print("====================================================================")  # banner top
        print("TAKE OFF TO 4 FT, HOVER 10S, FLY NORTH 10 FT, HOVER 10S, LAND")  # banner title
        print("====================================================================")  # banner bottom
        print(f"Target flight height : {HOVER_ALTITUDE_FEET:.1f} ft ({HOVER_ALTITUDE_METERS:.2f} m)")  # target height
        print(f"Safety ceiling       : {MAXIMUM_ALTITUDE_CEILING_FEET:.1f} ft ({MAXIMUM_ALTITUDE_CEILING_METERS:.2f} m)")  # ceiling
        print(f"Stationary hovers    : {HOVER_DURATION_SECONDS:.0f} seconds, twice")  # hover duration
        print(f"North leg            : {NORTH_DISTANCE_FEET:.0f} ft North at {NORTH_SPEED_METERS_PER_SECOND:.1f} m/s")  # north leg
        print()  # blank line

        # ==================================================
        # CONNECT TO PIXHAWK AND STREAM TELEMETRY
        # ==================================================
        if not os.path.exists(PIXHAWK_PORT):  # check UART exists on this machine
            raise RuntimeError(f"{PIXHAWK_PORT} is missing. Run this script on the Raspberry Pi.")  # stop if not on Pi

        print(f"Connecting to Pixhawk on {PIXHAWK_PORT} at {PIXHAWK_BAUD} baud (downloading parameters)...")  # status
        vehicle = connect(  # open DroneKit connection
            PIXHAWK_PORT,  # /dev/serial0
            baud=PIXHAWK_BAUD,  # 57600
            wait_ready=True,  # wait for parameters to download
            timeout=CONNECTION_TIMEOUT_SECONDS,  # 60s timeout
            source_system=GROUND_STATION_SYSTEM_ID,  # GCS ID 255
        )  # end of connect call
        if vehicle._master.target_component == 0:  # ensure target component is valid
            vehicle._master.target_component = 1  # default autopilot component 1
        print("Pixhawk connected.")  # log connection

        @vehicle.on_message("STATUSTEXT")  # listen for Pixhawk PreArm/status text
        def onStatusText(_vehicle, _name, message):  # callback for STATUSTEXT
            text = getattr(message, "text", "")  # read text field
            if isinstance(text, bytes):  # decode bytes if needed
                text = text.decode("ascii", errors="ignore")  # convert to string
            text = str(text).strip("\x00").strip()  # clean nulls and spaces
            if text:  # print non-empty status text
                print(f"  Pixhawk: {text}")  # show Pixhawk message in terminal and log

        intervalMicroseconds = int(MESSAGE_INTERVAL_SECONDS * MICROSECONDS_PER_SECOND)  # 10 Hz in microseconds
        for messageName in FLIGHT_MESSAGE_NAMES:  # request each telemetry stream
            messageId = getattr(middleMan.mavlink, f"MAVLINK_MSG_ID_{messageName}", None)  # look up msg ID
            if messageId is not None:  # valid message ID
                commandMessage = vehicle.message_factory.command_long_encode(  # build interval command
                    vehicle._master.target_system,  # Pixhawk system ID
                    vehicle._master.target_component,  # Pixhawk component ID
                    middleMan.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # set interval command
                    0,  # confirmation
                    messageId,  # message ID
                    intervalMicroseconds,  # 100000 us (10 Hz)
                    0,  # unused
                    0,  # unused
                    0,  # unused
                    0,  # unused
                    0,  # unused
                )  # end of interval command
                vehicle.send_mavlink(commandMessage)  # send command
        vehicle.flush()  # push requests out
        print("Requested GPS, height, attitude, and position-health streams.")  # log stream request

        # ==================================================
        # PREFLIGHT CHECKS (WITH NEW TRIM & COMPASS GATES FROM LOG ANALYSIS)
        # ==================================================
        print("Checking preflight health (Frame, Compass/GPS checks, Level Trim, GPS, EKF, Attitude)...")  # status
        while True:  # wait until every preflight gate passes
            blocker = None  # reason arming is blocked, or None if ready

            if not heartbeatIsFresh(vehicle):  # check heartbeat
                blocker = "Pixhawk heartbeat is missing."  # link dead

            if blocker is None:  # check Quad X frame
                frameClass = vehicle.parameters["FRAME_CLASS"]  # read frame class
                frameType = vehicle.parameters["FRAME_TYPE"]  # read frame type
                if frameClass is None or frameType is None:  # missing frame params
                    blocker = "FRAME_CLASS or FRAME_TYPE could not be read."  # wait
                elif (  # verify class=1 (Quad) and type=1 (X)
                    abs(float(frameClass) - QUAD_FRAME_CLASS) >= PARAMETER_TOLERANCE  # not Quad
                    or abs(float(frameType) - QUAD_X_FRAME_TYPE) >= PARAMETER_TOLERANCE  # not Quad X
                ):  # end of frame check
                    blocker = f"Quad X required. Found FRAME_CLASS={frameClass}, FRAME_TYPE={frameType}."  # wrong frame

            if blocker is None:  # check that Compass and GPS pre-arm checks are NOT skipped
                skipCheckValue = vehicle.parameters["ARMING_SKIPCHK"]  # read skip bitmask
                if skipCheckValue is not None and (int(round(float(skipCheckValue))) & SKIP_COMPASS_OR_GPS_MASK) != 0:  # skipped
                    raise RuntimeError(  # hard stop: must not fly with compass/GPS checks skipped
                        f"ARMING_SKIPCHK is {int(round(float(skipCheckValue)))} (skipping Compass/GPS checks). "  # message
                        "Run oct19-2026/reCalibrateDrone.py first."  # fix instruction
                    )  # end of skipchk error

            if blocker is None:  # check that at least one compass is enabled
                useCompass1 = int(round(float(vehicle.parameters["COMPASS_USE"] or 0)))  # compass 1 enabled
                useCompass2 = int(round(float(vehicle.parameters["COMPASS_USE2"] or 0)))  # compass 2 enabled
                if useCompass1 == 0 and useCompass2 == 0:  # no compass enabled
                    raise RuntimeError("Both COMPASS_USE and COMPASS_USE2 are 0. Run reCalibrateDrone.py first.")  # stop

            if blocker is None:  # check saved level horizon trim (caught the +3.61 deg tilted trim from Oct 16)
                trimX = vehicle.parameters["AHRS_TRIM_X"]  # roll trim in radians
                trimY = vehicle.parameters["AHRS_TRIM_Y"]  # pitch trim in radians
                if trimX is not None and trimY is not None:  # trim values present
                    trimRollDeg = math.degrees(float(trimX))  # roll trim in degrees
                    trimPitchDeg = math.degrees(float(trimY))  # pitch trim in degrees
                    if abs(trimRollDeg) > MAXIMUM_SAVED_TRIM_DEGREES or abs(trimPitchDeg) > MAXIMUM_SAVED_TRIM_DEGREES:  # tilted cal
                        raise RuntimeError(  # hard stop: tilted trim causes immediate liftoff drift
                            f"Saved level trim is tilted (Roll {trimRollDeg:+.2f} deg, Pitch {trimPitchDeg:+.2f} deg; "  # values
                            f"max {MAXIMUM_SAVED_TRIM_DEGREES:.1f} deg). Re-run Step 3 in oct19-2026/reCalibrateDrone.py on a flat surface."  # fix
                        )  # end of trim error

            if blocker is None:  # check GPS 3D fix and satellite count
                gpsReading = vehicle.gps_0  # GPS status object
                if gpsReading is None or gpsReading.fix_type is None or int(gpsReading.fix_type) < MINIMUM_GPS_FIX_TYPE:  # no 3D fix
                    blocker = "GPS 3D fix required."  # wait for 3D fix
                else:  # 3D fix present, check satellite count
                    satellites = gpsReading.satellites_visible  # visible satellites
                    if satellites is None or satellites == UNKNOWN_SATELLITE_COUNT or satellites < MINIMUM_SATELLITE_COUNT:  # < 6 sats
                        blocker = f"At least 6 satellites required. Seeing {satellites}."  # wait for satellites

            if blocker is None:  # check EKF horizontal position trust
                if not vehicle.ekf_ok:  # EKF not ready yet
                    satellites = vehicle.gps_0.satellites_visible  # current satellites
                    if vehicle._ekf_constposmode:  # EKF still in constant-position mode
                        blocker = (  # explain that EKF is still aligning GPS + compass
                            f"3D fix, {satellites} satellites. "  # GPS info
                            "EKF3 is still aligning GPS and compass (keep waiting ~30-60s)."  # clear wait note
                        )  # end of blocker text
                    else:  # other EKF wait state
                        blocker = f"3D fix, {satellites} satellites. Horizontal position is not trusted yet."  # wait

            if blocker is None:  # check Pixhawk overall armability (includes compass pre-arm check)
                if not vehicle.is_armable:  # Pixhawk pre-arm check still failing
                    blocker = "Pixhawk pre-arm check is not ready yet (check Pixhawk messages above)."  # wait

            if blocker is None:  # check live roll and pitch on the ground
                if vehicle.attitude is None or vehicle.attitude.roll is None or vehicle.attitude.pitch is None:  # no attitude
                    blocker = "Attitude is not available yet."  # wait
                else:  # attitude present
                    rollDegrees = math.degrees(vehicle.attitude.roll)  # live roll in degrees
                    pitchDegrees = math.degrees(vehicle.attitude.pitch)  # live pitch in degrees
                    if abs(rollDegrees) > MAXIMUM_GROUND_LEAN_DEGREES or abs(pitchDegrees) > MAXIMUM_GROUND_LEAN_DEGREES:  # tilted
                        blocker = f"Copter is not sitting level. Roll {rollDegrees:+.1f} deg, pitch {pitchDegrees:+.1f} deg."  # wait

            if blocker is None:  # all preflight checks passed!
                trimRollDeg = math.degrees(float(vehicle.parameters["AHRS_TRIM_X"] or 0.0))  # saved roll trim
                trimPitchDeg = math.degrees(float(vehicle.parameters["AHRS_TRIM_Y"] or 0.0))  # saved pitch trim
                headingDeg = math.degrees(float(vehicle.attitude.yaw)) if vehicle.attitude.yaw is not None else 0.0  # nose heading
                print("Preflight checks passed.")  # log pass
                print(  # print preflight snapshot
                    f"FRAME_CLASS {float(vehicle.parameters['FRAME_CLASS']):.0f} | "  # frame class
                    f"FRAME_TYPE {float(vehicle.parameters['FRAME_TYPE']):.0f} | "  # frame type
                    f"satellites {vehicle.gps_0.satellites_visible} | "  # satellite count
                    f"heading {headingDeg:+.1f} deg | "  # current nose heading
                    f"saved trim ({trimRollDeg:+.2f}, {trimPitchDeg:+.2f}) deg."  # level trim
                )  # end of preflight print
                break  # exit preflight wait loop

            print(f"Waiting. {blocker}")  # print current blocker once per second
            time.sleep(PRINT_INTERVAL_SECONDS)  # sleep 1 second

        # ==================================================
        # ASK PILOT TO TYPE YES AND CONFIGURE FAILSAFE
        # ==================================================
        promptText = "Type YES to take off to 4 ft, hover 10s, fly 10 ft North, hover 10s, and land: "  # prompt
        confirmation = input(promptText)  # read user input
        if flightLogger is not None:  # echo input into log file
            flightLogger.writeLogOnly(f"{promptText}{confirmation}\n")  # write to file
        if confirmation.strip() != "YES":  # pilot did not type YES
            print("Flight cancelled by pilot. The copter was not armed.")  # log cancel
            flightOutcome["overallOutcome"] = "CANCELLED BEFORE ARMING"  # outcome
            flightOutcome["preflightAndFailsafe"] = "PASSED (Pilot cancelled at YES prompt)"  # preflight note
            flightOutcome["stopReason"] = "Pilot did not type YES."  # stop reason
            return  # exit cleanly

        # Shorten TKOFF_SLEW_TIME to 1.0s if slower so the drone lifts off cleanly.
        savedSlew = vehicle.parameters[TAKEOFF_SLEW_PARAMETER_NAME]  # read TKOFF_SLEW_TIME
        if savedSlew is not None:  # parameter exists
            savedSlew = float(savedSlew)  # convert to float
            takeoffSlew = min(savedSlew, PREFERRED_TAKEOFF_SLEW_SECONDS)  # cap at 1.0s
            print(f"{TAKEOFF_SLEW_PARAMETER_NAME}: original {savedSlew:.2f}s | takeoff {takeoffSlew:.2f}s")  # log slew
            if abs(takeoffSlew - savedSlew) >= PARAMETER_TOLERANCE:  # needs change
                setParameterAndConfirm(vehicle, TAKEOFF_SLEW_PARAMETER_NAME, takeoffSlew)  # write 1.0s
                originalTakeoffSlewSeconds = savedSlew  # remember original for restore

        # Enable onboard Land failsafe (FS_GCS_ENABLE=5, FS_OPTIONS bit 4 cleared).
        savedEnable = float(vehicle.parameters[GROUND_STATION_FAILSAFE_PARAMETER_NAME])  # read FS_GCS_ENABLE
        savedTimeout = float(vehicle.parameters[GROUND_STATION_FAILSAFE_TIMEOUT_PARAMETER_NAME])  # read FS_GCS_TIMEOUT
        savedOptions = float(vehicle.parameters[FAILSAFE_OPTIONS_PARAMETER_NAME])  # read FS_OPTIONS
        timeoutSeconds = savedTimeout if (0 < savedTimeout <= PREFERRED_FAILSAFE_TIMEOUT_SECONDS) else PREFERRED_FAILSAFE_TIMEOUT_SECONDS  # 5.0s
        savedOptionsInt = int(round(savedOptions))  # integer bitmask
        flightOptions = savedOptionsInt & ~CONTINUE_PILOT_MODE_ON_FAILSAFE_BIT  # clear bit 4 (16)
        savedFailsafe = (savedEnable, savedTimeout, savedOptions)  # save tuple for restore

        setParameterAndConfirm(vehicle, GROUND_STATION_FAILSAFE_PARAMETER_NAME, GROUND_STATION_FAILSAFE_LAND_VALUE)  # set Land (5)
        setParameterAndConfirm(vehicle, GROUND_STATION_FAILSAFE_TIMEOUT_PARAMETER_NAME, timeoutSeconds)  # set 5.0s timeout
        setParameterAndConfirm(vehicle, FAILSAFE_OPTIONS_PARAMETER_NAME, flightOptions)  # set cleaned options
        vehicle.groundStationFailsafeTimeoutSeconds = timeoutSeconds  # store timeout on vehicle
        print("Onboard failsafe is set to Land if the Pi connection is lost.")  # log failsafe ready

        if not vehicle.ekf_ok or not vehicle.is_armable:  # final sanity check right before arming
            raise RuntimeError("EKF or pre-arm check became unready while waiting at the YES prompt.")  # abort
        flightOutcome["preflightAndFailsafe"] = "SUCCESS (Preflight passed, FS_GCS_ENABLE=5 Land set)"  # log success

        # ==================================================
        # STEP 1: ARM IN GUIDED, CLIMB TO 4 FEET, AND SETTLE 2 SECONDS
        # ==================================================
        copterWasArmed = True  # mark armed so finally block will land on any error
        flightOutcome["step1TakeoffTo4Feet"] = "IN PROGRESS"  # update outcome
        setFlightMode(vehicle, "GUIDED")  # switch to GUIDED

        print("Arming motors.")  # log arming
        vehicle.armed = True  # command normal arm
        armStartTime = time.monotonic()  # start arm timer
        while not vehicle.armed:  # wait for Pixhawk to arm
            if time.monotonic() - armStartTime > ARM_TIMEOUT_SECONDS:  # arm timeout
                flightOutcome["step1TakeoffTo4Feet"] = "FAILED (Arming timed out)"  # record failure
                raise RuntimeError("Arming timed out.")  # abort
            if not heartbeatIsFresh(vehicle):  # lost heartbeat
                flightOutcome["step1TakeoffTo4Feet"] = "FAILED (Heartbeat lost while arming)"  # record failure
                raise RuntimeError("Pixhawk heartbeat was lost while arming.")  # abort
            time.sleep(STATUS_CHECK_INTERVAL_SECONDS)  # wait 0.2s
        print("Motors armed. Propellers spinning.")  # log armed

        print(f"Taking off to {HOVER_ALTITUDE_FEET:.1f} feet ({HOVER_ALTITUDE_METERS:.2f} m).")  # log takeoff
        vehicle.simple_takeoff(HOVER_ALTITUDE_METERS)  # send GUIDED takeoff command
        minimumTakeoffMeters = HOVER_ALTITUDE_METERS * TAKEOFF_REACHED_FRACTION  # 95% of 4 ft (1.16 m)
        takeoffStartTime = time.monotonic()  # start climb timer
        lastPrintTime = 0.0  # print timer
        while True:  # climb loop
            if vehicle.mode.name != "GUIDED":  # mode changed unexpectedly
                flightOutcome["step1TakeoffTo4Feet"] = "FAILED (GUIDED mode lost)"  # record failure
                raise RuntimeError("GUIDED mode lost during takeoff.")  # abort
            altitudeMeters = checkFlightHealthAndCeiling(vehicle, flightOutcome)  # check health and 5-ft ceiling

            nowTime = time.monotonic()  # current time
            if nowTime - lastPrintTime >= PRINT_INTERVAL_SECONDS:  # print every 1s
                heightString = "waiting" if altitudeMeters is None else f"{altitudeMeters:.2f} m ({altitudeMeters * FEET_PER_METER:.2f} ft)"  # format
                print(f"Climbing | {heightString}")  # print climb height
                lastPrintTime = nowTime  # update print timer

            if altitudeMeters is not None and altitudeMeters >= minimumTakeoffMeters:  # reached 4 ft target
                print(f"Reached 4-foot target altitude: {altitudeMeters:.2f} m ({altitudeMeters * FEET_PER_METER:.2f} ft).")  # log reached
                break  # exit climb loop
            if nowTime - takeoffStartTime > TAKEOFF_TIMEOUT_SECONDS:  # climb timed out
                flightOutcome["step1TakeoffTo4Feet"] = "FAILED (Takeoff timed out)"  # record failure
                raise RuntimeError("Takeoff timed out.")  # abort
            time.sleep(STATUS_CHECK_INTERVAL_SECONDS)  # wait 0.2s

        # Pause 2 seconds in GUIDED at 4 feet so vertical climb momentum stops before switching to LOITER.
        print(f"Settling at 4 feet in GUIDED for {TAKEOFF_SETTLE_SECONDS:.0f} seconds before switching to LOITER...")  # log settle
        settleStart = time.monotonic()  # start settle timer
        while time.monotonic() - settleStart < TAKEOFF_SETTLE_SECONDS:  # 2-second settle loop
            if vehicle.mode.name != "GUIDED":  # verify still in GUIDED
                flightOutcome["step1TakeoffTo4Feet"] = "FAILED (GUIDED lost during settle)"  # record failure
                raise RuntimeError("GUIDED mode lost during takeoff settle.")  # abort
            altitudeMeters = checkFlightHealthAndCeiling(vehicle, flightOutcome)  # verify ceiling & EKF
            time.sleep(STATUS_CHECK_INTERVAL_SECONDS)  # wait 0.2s
        flightOutcome["step1TakeoffTo4Feet"] = (  # record Step 1 success
            f"SUCCESS ({altitudeMeters:.2f} m ({altitudeMeters * FEET_PER_METER:.2f} ft))"  # height text
        )  # end of Step 1 outcome

        # ==================================================
        # STEP 2: FIRST STATIONARY HOVER AT 4 FEET FOR 10 SECONDS (LOITER)
        # ==================================================
        flightOutcome["step2FirstHover10Seconds"] = "IN PROGRESS"  # mark Step 2 started
        setFlightMode(vehicle, "LOITER")  # switch to LOITER

        firstHoverLocation = vehicle.location.global_relative_frame  # read GPS position at hover start
        if (  # verify valid GPS coordinate
            firstHoverLocation is None  # no location
            or firstHoverLocation.lat is None  # no latitude
            or firstHoverLocation.lon is None  # no longitude
            or (abs(float(firstHoverLocation.lat)) < MINIMUM_VALID_COORDINATE_DEGREES and abs(float(firstHoverLocation.lon)) < MINIMUM_VALID_COORDINATE_DEGREES)  # 0,0
        ):  # end of GPS check
            flightOutcome["step2FirstHover10Seconds"] = "FAILED (No GPS spot at hover start)"  # record failure
            raise RuntimeError("GPS position was not available for First hover.")  # abort
        firstHoverStartLat = float(firstHoverLocation.lat)  # lock hover start latitude
        firstHoverStartLon = float(firstHoverLocation.lon)  # lock hover start longitude

        print(f"First hover (4 ft) started (holding 4 ft for {HOVER_DURATION_SECONDS:.0f} seconds).")  # log hover start
        hoverStartTime = time.monotonic()  # start 10s timer
        lastPrintTime = 0.0  # reset print timer
        firstHoverMaxDriftFeet = 0.0  # track max drift in feet
        while True:  # 10-second hover loop
            elapsedHover = time.monotonic() - hoverStartTime  # seconds elapsed
            if elapsedHover >= HOVER_DURATION_SECONDS:  # 10 seconds finished
                break  # exit first hover loop
            if vehicle.mode.name != "LOITER":  # lost LOITER mode
                flightOutcome["step2FirstHover10Seconds"] = "FAILED (LOITER mode lost)"  # record failure
                raise RuntimeError("LOITER mode lost during First hover.")  # abort
            altitudeMeters = checkFlightHealthAndCeiling(vehicle, flightOutcome)  # check health & ceiling

            currentLoc = vehicle.location.global_relative_frame  # read current GPS coordinate
            if currentLoc is None or currentLoc.lat is None or currentLoc.lon is None:  # GPS missing
                flightOutcome["step2FirstHover10Seconds"] = "FAILED (GPS position lost)"  # record failure
                raise RuntimeError("GPS position was lost during First hover.")  # abort
            northOffsetMeters = (float(currentLoc.lat) - firstHoverStartLat) * METERS_PER_DEGREE_OF_LATITUDE  # North drift (m)
            eastOffsetMeters = (  # East drift (m)
                (float(currentLoc.lon) - firstHoverStartLon)  # longitude delta
                * METERS_PER_DEGREE_OF_LATITUDE  # meters per degree
                * math.cos(math.radians(firstHoverStartLat))  # latitude cosine scale
            )  # end of East drift calculation
            driftMeters = math.hypot(northOffsetMeters, eastOffsetMeters)  # total horizontal drift (m)
            driftFeet = driftMeters * FEET_PER_METER  # drift in feet
            if driftFeet > firstHoverMaxDriftFeet:  # new max drift
                firstHoverMaxDriftFeet = driftFeet  # update max drift
            if driftMeters > MAXIMUM_HOVER_DRIFT_METERS:  # exceeded 8-ft limit
                flightOutcome["step2FirstHover10Seconds"] = f"FAILED (Drifted {driftFeet:.1f} ft)"  # record failure
                raise RuntimeError(  # abort and land
                    f"First hover (4 ft) drifted {driftFeet:.1f} ft from its spot "  # drift distance
                    f"(limit {MAXIMUM_HOVER_DRIFT_FEET:.0f} ft)."  # limit
                )  # end of drift error

            nowTime = time.monotonic()  # current time
            if nowTime - lastPrintTime >= PRINT_INTERVAL_SECONDS:  # print every 1s
                remainingSeconds = max(0.0, HOVER_DURATION_SECONDS - elapsedHover)  # seconds left
                print(  # print hover status
                    f"First hover (4 ft) | {altitudeMeters:.2f} m ({altitudeMeters * FEET_PER_METER:.2f} ft) | "  # height
                    f"drift {driftFeet:.1f} ft | remaining {remainingSeconds:.0f} s"  # drift and countdown
                )  # end of hover print
                lastPrintTime = nowTime  # update print time
            time.sleep(STATUS_CHECK_INTERVAL_SECONDS)  # wait 0.2s

        print("First hover (4 ft) completed.")  # log completion
        flightOutcome["step2FirstHover10Seconds"] = f"SUCCESS (10s completed, max drift {firstHoverMaxDriftFeet:.1f} ft)"  # record pass

        # ==================================================
        # STEP 3: FLY FORWARD (NORTH) 10 FEET AT 4 FEET HEIGHT (GUIDED)
        # ==================================================
        flightOutcome["step3FlyNorth10Feet"] = "IN PROGRESS"  # mark Step 3 started
        setFlightMode(vehicle, "GUIDED")  # switch to GUIDED for velocity control

        northStartLocation = vehicle.location.global_relative_frame  # read GPS start of North leg
        if northStartLocation is None or northStartLocation.lat is None or northStartLocation.lon is None:  # missing GPS
            flightOutcome["step3FlyNorth10Feet"] = "FAILED (No GPS at start of North move)"  # record failure
            raise RuntimeError("GPS position was not available for the North move.")  # abort
        northStartLat = float(northStartLocation.lat)  # lock start latitude
        northStartLon = float(northStartLocation.lon)  # lock start longitude

        print(  # announce North move
            f"Flying North {NORTH_DISTANCE_FEET:.0f} ft at {NORTH_SPEED_METERS_PER_SECOND:.1f} m/s "  # distance & speed
            f"while holding {HOVER_ALTITUDE_FEET:.0f} ft height."  # altitude hold
        )  # end of announcement
        northMoveStartTime = time.monotonic()  # start North move timer
        lastPrintTime = 0.0  # reset print timer
        try:  # ensure we send zero velocity when leaving the North move
            while True:  # North move loop
                if vehicle.mode.name != "GUIDED":  # lost GUIDED mode
                    flightOutcome["step3FlyNorth10Feet"] = "FAILED (GUIDED mode lost)"  # record failure
                    raise RuntimeError("GUIDED mode lost during the North move.")  # abort
                altitudeMeters = checkFlightHealthAndCeiling(vehicle, flightOutcome)  # check health & ceiling

                currentLoc = vehicle.location.global_relative_frame  # read current GPS position
                if currentLoc is None or currentLoc.lat is None or currentLoc.lon is None:  # lost GPS
                    flightOutcome["step3FlyNorth10Feet"] = "FAILED (GPS position lost)"  # record failure
                    raise RuntimeError("GPS position was lost during the North move.")  # abort
                northMeters = (float(currentLoc.lat) - northStartLat) * METERS_PER_DEGREE_OF_LATITUDE  # North traveled (m)
                eastMeters = (  # East offset (m)
                    (float(currentLoc.lon) - northStartLon)  # longitude delta
                    * METERS_PER_DEGREE_OF_LATITUDE  # meters per degree
                    * math.cos(math.radians(northStartLat))  # latitude cosine scale
                )  # end of East offset
                northFeet = northMeters * FEET_PER_METER  # North traveled in feet
                eastFeet = eastMeters * FEET_PER_METER  # East offset in feet
                flightOutcome["northDistanceTraveledText"] = f"{northFeet:.1f} ft North ({eastFeet:+.1f} ft East)"  # update summary

                if northMeters >= NORTH_DISTANCE_METERS:  # reached 10 feet North!
                    print(  # log arrival
                        f"10 feet North reached | {northFeet:.1f} ft North | "  # North distance
                        f"{eastFeet:+.1f} ft East | {altitudeMeters:.2f} m ({altitudeMeters * FEET_PER_METER:.2f} ft)"  # East & height
                    )  # end of arrival print
                    break  # exit North move loop

                if abs(eastMeters) > MAXIMUM_OFF_COURSE_METERS or northMeters < -MAXIMUM_OFF_COURSE_METERS:  # off course > 8 ft
                    flightOutcome["step3FlyNorth10Feet"] = f"FAILED (Off course: {northFeet:.1f} ft N, {eastFeet:+.1f} ft E)"  # record
                    raise RuntimeError(f"Left the North line: {northFeet:.1f} ft North, {eastFeet:+.1f} ft East.")  # abort

                if time.monotonic() - northMoveStartTime > NORTH_MOVE_TIMEOUT_SECONDS:  # move timed out (> 20s)
                    flightOutcome["step3FlyNorth10Feet"] = f"FAILED (Timed out at {northFeet:.1f} ft N)"  # record failure
                    raise RuntimeError(  # abort
                        f"Did not reach 10 feet North in time ({northFeet:.1f} ft North, {eastFeet:+.1f} ft East)."  # timeout msg
                    )  # end of timeout error

                # Compute gentle vertical speed correction to hold 4.0 feet during the North move.
                downSpeedMetersPerSecond = 0.0  # default zero vertical speed
                if altitudeMeters is not None:  # valid altitude
                    altitudeErrorMeters = HOVER_ALTITUDE_METERS - altitudeMeters  # positive if below 4 ft
                    climbRate = max(  # clamp vertical speed to +-0.25 m/s
                        -MAXIMUM_VERTICAL_CORRECTION_METERS_PER_SECOND,  # max descent rate
                        min(  # clamp upper climb rate
                            MAXIMUM_VERTICAL_CORRECTION_METERS_PER_SECOND,  # max climb rate
                            altitudeErrorMeters * ALTITUDE_HOLD_GAIN_DURING_MOVE,  # proportional climb
                        ),  # end of upper clamp
                    )  # end of vertical clamp
                    downSpeedMetersPerSecond = -climbRate  # NED down is negative climb

                sendLocalVelocity(vehicle, NORTH_SPEED_METERS_PER_SECOND, 0.0, downSpeedMetersPerSecond)  # command North velocity

                nowTime = time.monotonic()  # current time
                if nowTime - lastPrintTime >= PRINT_INTERVAL_SECONDS:  # print progress every 1s
                    print(  # print North distance and height
                        f"Flying North | {northFeet:.1f}/{NORTH_DISTANCE_FEET:.0f} ft North | "  # North progress
                        f"{eastFeet:+.1f} ft East | {altitudeMeters:.2f} m ({altitudeMeters * FEET_PER_METER:.2f} ft)"  # East & height
                    )  # end of progress print
                    lastPrintTime = nowTime  # update print time
                time.sleep(STATUS_CHECK_INTERVAL_SECONDS)  # wait 0.2s

            # Hold zero horizontal speed for 2 seconds so the copter stops cleanly before Step 4 LOITER.
            brakeStartTime = time.monotonic()  # start 2s brake timer
            while time.monotonic() - brakeStartTime < SETTLE_AFTER_MOVE_SECONDS:  # 2-second braking loop
                if vehicle.mode.name != "GUIDED":  # lost GUIDED while braking
                    flightOutcome["step3FlyNorth10Feet"] = "FAILED (GUIDED lost while stopping)"  # record failure
                    raise RuntimeError("GUIDED mode lost while stopping.")  # abort
                altitudeMeters = checkFlightHealthAndCeiling(vehicle, flightOutcome)  # check health & ceiling
                downSpeedMetersPerSecond = 0.0  # default zero vertical speed
                if altitudeMeters is not None:  # keep holding 4 ft while braking
                    altitudeErrorMeters = HOVER_ALTITUDE_METERS - altitudeMeters  # height error
                    climbRate = max(  # clamp vertical rate
                        -MAXIMUM_VERTICAL_CORRECTION_METERS_PER_SECOND,  # lower limit
                        min(MAXIMUM_VERTICAL_CORRECTION_METERS_PER_SECOND, altitudeErrorMeters * ALTITUDE_HOLD_GAIN_DURING_MOVE),  # upper
                    )  # end of clamp
                    downSpeedMetersPerSecond = -climbRate  # NED down velocity
                sendLocalVelocity(vehicle, 0.0, 0.0, downSpeedMetersPerSecond)  # command zero horizontal speed
                time.sleep(STATUS_CHECK_INTERVAL_SECONDS)  # wait 0.2s

            print("North move stopped cleanly.")  # log clean stop
            flightOutcome["step3FlyNorth10Feet"] = f"SUCCESS ({flightOutcome['northDistanceTraveledText']})"  # record pass
        finally:  # always send zero speed when exiting North move block
            if heartbeatIsFresh(vehicle) and not getattr(vehicle, "commandCableClosed", False):  # link still active
                try:  # try sending zero velocity
                    sendLocalVelocity(vehicle, 0.0, 0.0, 0.0)  # zero all speeds
                except Exception as stopError:  # ignore send error on exit
                    print(f"Could not send the stop speed: {stopError}")  # log warning

        # ==================================================
        # STEP 4: SECOND STATIONARY HOVER AT 4 FEET FOR 10 SECONDS (LOITER)
        # ==================================================
        flightOutcome["step4SecondHover10Seconds"] = "IN PROGRESS"  # mark Step 4 started
        setFlightMode(vehicle, "LOITER")  # switch to LOITER

        secondHoverLocation = vehicle.location.global_relative_frame  # read GPS spot at start of second hover
        if secondHoverLocation is None or secondHoverLocation.lat is None or secondHoverLocation.lon is None:  # missing GPS
            flightOutcome["step4SecondHover10Seconds"] = "FAILED (No GPS spot at hover start)"  # record failure
            raise RuntimeError("GPS position was not available for Second hover.")  # abort
        secondHoverStartLat = float(secondHoverLocation.lat)  # lock second hover latitude
        secondHoverStartLon = float(secondHoverLocation.lon)  # lock second hover longitude

        print(f"Second hover (4 ft) started (holding 4 ft for {HOVER_DURATION_SECONDS:.0f} seconds).")  # log start
        hoverStartTime = time.monotonic()  # start 10s timer
        lastPrintTime = 0.0  # reset print timer
        secondHoverMaxDriftFeet = 0.0  # track max drift
        while True:  # 10-second second hover loop
            elapsedHover = time.monotonic() - hoverStartTime  # elapsed time
            if elapsedHover >= HOVER_DURATION_SECONDS:  # 10 seconds completed
                break  # exit second hover loop
            if vehicle.mode.name != "LOITER":  # lost LOITER mode
                flightOutcome["step4SecondHover10Seconds"] = "FAILED (LOITER mode lost)"  # record failure
                raise RuntimeError("LOITER mode lost during Second hover.")  # abort
            altitudeMeters = checkFlightHealthAndCeiling(vehicle, flightOutcome)  # check health & ceiling

            currentLoc = vehicle.location.global_relative_frame  # read current GPS position
            if currentLoc is None or currentLoc.lat is None or currentLoc.lon is None:  # GPS missing
                flightOutcome["step4SecondHover10Seconds"] = "FAILED (GPS position lost)"  # record failure
                raise RuntimeError("GPS position was lost during Second hover.")  # abort
            northOffsetMeters = (float(currentLoc.lat) - secondHoverStartLat) * METERS_PER_DEGREE_OF_LATITUDE  # North drift
            eastOffsetMeters = (  # East drift
                (float(currentLoc.lon) - secondHoverStartLon)  # lon delta
                * METERS_PER_DEGREE_OF_LATITUDE  # meters per degree
                * math.cos(math.radians(secondHoverStartLat))  # cosine scale
            )  # end of East drift
            driftMeters = math.hypot(northOffsetMeters, eastOffsetMeters)  # total drift in meters
            driftFeet = driftMeters * FEET_PER_METER  # total drift in feet
            if driftFeet > secondHoverMaxDriftFeet:  # new peak drift
                secondHoverMaxDriftFeet = driftFeet  # save peak drift
            if driftMeters > MAXIMUM_HOVER_DRIFT_METERS:  # drifted > 8 feet
                flightOutcome["step4SecondHover10Seconds"] = f"FAILED (Drifted {driftFeet:.1f} ft)"  # record failure
                raise RuntimeError(  # abort and land
                    f"Second hover (4 ft) drifted {driftFeet:.1f} ft from its spot "  # drift distance
                    f"(limit {MAXIMUM_HOVER_DRIFT_FEET:.0f} ft)."  # limit
                )  # end of drift error

            nowTime = time.monotonic()  # current time
            if nowTime - lastPrintTime >= PRINT_INTERVAL_SECONDS:  # print every 1s
                remainingSeconds = max(0.0, HOVER_DURATION_SECONDS - elapsedHover)  # seconds left
                print(  # print hover status
                    f"Second hover (4 ft) | {altitudeMeters:.2f} m ({altitudeMeters * FEET_PER_METER:.2f} ft) | "  # height
                    f"drift {driftFeet:.1f} ft | remaining {remainingSeconds:.0f} s"  # drift and countdown
                )  # end of status print
                lastPrintTime = nowTime  # update print timer
            time.sleep(STATUS_CHECK_INTERVAL_SECONDS)  # wait 0.2s

        print("Second hover (4 ft) completed.")  # log completion
        flightOutcome["step4SecondHover10Seconds"] = f"SUCCESS (10s completed, max drift {secondHoverMaxDriftFeet:.1f} ft)"  # record pass

        # ==================================================
        # STEP 5: LAND SAFELY AND WAIT UNTIL PROPELLERS STOP
        # ==================================================
        landingWasStarted = True  # mark Step 5 started
        flightOutcome["step5LandAndStopProps"] = "IN PROGRESS"  # update outcome
        print("Landing.")  # log landing start
        setFlightMode(vehicle, "LAND")  # command LAND mode
        landStartTime = time.monotonic()  # start landing timer
        while vehicle.armed:  # wait until motors disarm
            if not heartbeatIsFresh(vehicle):  # heartbeat lost during descent
                failsafeWait = getattr(vehicle, "groundStationFailsafeTimeoutSeconds", PREFERRED_FAILSAFE_TIMEOUT_SECONDS)  # 5s
                print("Heartbeat is lost while waiting for landing. Closing cable so onboard failsafe finishes landing.")  # log
                vehicle.commandCableClosed = True  # mark cable closed
                vehicle.close()  # close serial port to stop heartbeats
                flightOutcome["step5LandAndStopProps"] = (  # record failsafe landing
                    f"ONBOARD FAILSAFE TRIGGERED DURING DESCENT (Lands after {failsafeWait:.0f}s)"  # outcome text
                )  # end of failsafe outcome
                break  # exit disarm wait loop
            if time.monotonic() - landStartTime > LAND_TIMEOUT_SECONDS:  # > 60s timeout
                flightOutcome["step5LandAndStopProps"] = "FAILED (Timed out waiting for motors to disarm)"  # record failure
                raise RuntimeError("Landing not confirmed. The motors are still armed.")  # abort
            loc = vehicle.location.global_relative_frame  # read height during descent
            altText = "waiting" if (loc is None or loc.alt is None) else f"{float(loc.alt):.2f} m ({float(loc.alt) * FEET_PER_METER:.2f} ft)"  # format
            print(f"Waiting for landing and propeller stop. {altText}")  # print descent height
            time.sleep(PRINT_INTERVAL_SECONDS)  # wait 1s

        copterWasArmed = vehicle.armed  # update armed flag
        if not vehicle.armed:  # motors disarmed cleanly
            print("Landed safely. Motors disarmed and propellers stopped.")  # log disarmed
            flightOutcome["step5LandAndStopProps"] = "SUCCESS (Landed, motors disarmed, propellers stopped)"  # record pass
            flightOutcome["overallOutcome"] = "SUCCESS"  # overall mission succeeded
            print("Flight complete.")  # log mission complete
        else:  # ended via onboard failsafe
            flightOutcome["overallOutcome"] = "ENDED WITH FAILSAFE LANDING"  # record failsafe finish

    except KeyboardInterrupt:  # user pressed Ctrl+C
        print()  # newline
        print("Flight interrupted by user (Ctrl+C).")  # log interrupt
        flightOutcome["overallOutcome"] = "EXITED MID-WAY (Ctrl+C)"  # record interrupt
        flightOutcome["stopReason"] = "User pressed Ctrl+C."  # save reason

    except Exception as error:  # any flight error or safety trigger
        print(f"Flight stopped: {error}")  # log error message
        flightOutcome["overallOutcome"] = "FAILED"  # mark failed
        flightOutcome["stopReason"] = str(error)  # save error text

    finally:  # always execute emergency land, parameter restore, and summary log
        if vehicle is not None and not getattr(vehicle, "commandCableClosed", False):  # connection still open
            if (copterWasArmed or vehicle.armed) and not landingWasStarted:  # stopped mid-flight while armed
                print("Requesting emergency LAND.")  # log emergency land
                flightOutcome["step5LandAndStopProps"] = "IN PROGRESS (Emergency LAND)"  # update status
                try:  # attempt emergency LAND and wait for disarm
                    if not heartbeatIsFresh(vehicle):  # heartbeat lost
                        failsafeWait = getattr(vehicle, "groundStationFailsafeTimeoutSeconds", PREFERRED_FAILSAFE_TIMEOUT_SECONDS)  # 5s
                        print("Heartbeat is lost. Closing cable so onboard failsafe lands the copter.")  # log
                        vehicle.commandCableClosed = True  # mark closed
                        vehicle.close()  # close serial cable
                        flightOutcome["step5LandAndStopProps"] = (  # record failsafe trigger
                            f"ONBOARD FAILSAFE TRIGGERED (Cable closed; Pixhawk lands after {failsafeWait:.0f}s)"  # text
                        )  # end of failsafe text
                    else:  # link is alive, command LAND
                        setFlightMode(vehicle, "LAND")  # switch to LAND
                        emergencyLandStart = time.monotonic()  # start 60s timer
                        while vehicle.armed:  # wait for disarm
                            if not heartbeatIsFresh(vehicle):  # link lost during emergency descent
                                vehicle.commandCableClosed = True  # mark closed
                                vehicle.close()  # close serial cable
                                flightOutcome["step5LandAndStopProps"] = "ONBOARD FAILSAFE TRIGGERED DURING DESCENT"  # record
                                break  # exit loop
                            if time.monotonic() - emergencyLandStart > LAND_TIMEOUT_SECONDS:  # timeout
                                raise RuntimeError("Timed out waiting for motors to disarm.")  # raise timeout
                            loc = vehicle.location.global_relative_frame  # read height
                            altText = "waiting" if (loc is None or loc.alt is None) else f"{float(loc.alt):.2f} m ({float(loc.alt) * FEET_PER_METER:.2f} ft)"  # format
                            print(f"Waiting for landing and propeller stop. {altText}")  # print height
                            time.sleep(PRINT_INTERVAL_SECONDS)  # wait 1s
                        if not vehicle.armed:  # disarmed cleanly
                            print("Landed safely. Motors disarmed and propellers stopped.")  # log disarmed
                            flightOutcome["step5LandAndStopProps"] = "SUCCESS (Landed, motors disarmed, propellers stopped)"  # record
                except KeyboardInterrupt:  # second Ctrl+C during emergency landing
                    print("Second Ctrl+C. Closing the cable without force-disarming.")  # log second Ctrl+C
                    print("The onboard failsafe remains set to Land.")  # explain failsafe
                    flightOutcome["step5LandAndStopProps"] = "SECOND Ctrl+C (Cable closed; onboard Land failsafe active)"  # record
                except Exception as landingError:  # emergency landing error
                    print(f"Landing unconfirmed: {landingError}")  # log error
                    flightOutcome["step5LandAndStopProps"] = f"UNCONFIRMED ({landingError})"  # record

            # Restore failsafe and takeoff ramp parameters only if disarmed and link is still open.
            if not getattr(vehicle, "commandCableClosed", False) and heartbeatIsFresh(vehicle) and not vehicle.armed:  # safe to restore
                if savedFailsafe is not None:  # restore failsafe tuple
                    try:  # attempt restore
                        setParameterAndConfirm(vehicle, GROUND_STATION_FAILSAFE_PARAMETER_NAME, savedFailsafe[0])  # FS_GCS_ENABLE
                        setParameterAndConfirm(vehicle, GROUND_STATION_FAILSAFE_TIMEOUT_PARAMETER_NAME, savedFailsafe[1])  # FS_GCS_TIMEOUT
                        setParameterAndConfirm(vehicle, FAILSAFE_OPTIONS_PARAMETER_NAME, savedFailsafe[2])  # FS_OPTIONS
                        print("Ground-station failsafe settings restored.")  # log restored
                    except Exception as restoreError:  # restore failed
                        print(f"Could not restore the ground-station failsafe: {restoreError}")  # log warning
                if originalTakeoffSlewSeconds is not None:  # restore TKOFF_SLEW_TIME
                    try:  # attempt restore
                        print(f"Restoring {TAKEOFF_SLEW_PARAMETER_NAME} to {originalTakeoffSlewSeconds:.2f}s.")  # log
                        setParameterAndConfirm(vehicle, TAKEOFF_SLEW_PARAMETER_NAME, originalTakeoffSlewSeconds)  # write original
                        print(f"{TAKEOFF_SLEW_PARAMETER_NAME} restored.")  # log restored
                    except Exception as restoreError:  # restore failed
                        print(f"Could not restore {TAKEOFF_SLEW_PARAMETER_NAME}: {restoreError}")  # log warning

            if not getattr(vehicle, "commandCableClosed", False):  # close UART if still open
                vehicle.commandCableClosed = True  # mark closed
                try:  # close port
                    vehicle.close()  # close DroneKit connection
                    print("Command cable closed.")  # log closed
                except Exception as closeError:  # close error
                    print(f"Could not close the command cable: {closeError}")  # log warning
        elif vehicle is not None:  # cable was already closed
            print("Failsafe settings were left on the Pixhawk.")  # log note
        print("Script finished.")  # log script finished

        flightOutcome["finishedAt"] = time.strftime("%Y-%m-%d %H:%M:%S %Z")  # record finish time
        print()  # blank line
        print("=" * 68)  # summary top border
        print("FLIGHT RUN OUTCOME SUMMARY")  # summary title
        print("=" * 68)  # summary divider
        print(f"Overall Outcome           : {flightOutcome['overallOutcome']}")  # overall result
        print(f"Started At                : {flightOutcome['startedAt']}")  # start time
        print(f"Finished At               : {flightOutcome['finishedAt']}")  # finish time
        print(f"Preflight & Land Failsafe : {flightOutcome['preflightAndFailsafe']}")  # preflight result
        print(f"Step 1 (Takeoff 4 ft)     : {flightOutcome['step1TakeoffTo4Feet']}")  # Step 1 result
        print(f"Step 2 (First Hover 10s)  : {flightOutcome['step2FirstHover10Seconds']}")  # Step 2 result
        print(f"Step 3 (Fly North 10 ft)  : {flightOutcome['step3FlyNorth10Feet']}")  # Step 3 result
        print(f"Step 4 (Second Hover 10s) : {flightOutcome['step4SecondHover10Seconds']}")  # Step 4 result
        print(f"Step 5 (Land & Disarm)    : {flightOutcome['step5LandAndStopProps']}")  # Step 5 result
        print(f"Max Altitude Recorded     : {flightOutcome['maximumAltitudeReachedText']}")  # max height
        print(f"North Distance Recorded   : {flightOutcome['northDistanceTraveledText']}")  # North distance
        if flightOutcome["stopReason"]:  # print stop reason if present
            print(f"Stop / Failure Reason     : {flightOutcome['stopReason']}")  # stop reason
        if logPath is not None:  # print saved log path
            print(f"Saved Flight Log          : {logPath}")  # log file path
        print("=" * 68)  # summary bottom border

        sys.stdout = originalStandardOutput  # restore standard output
        if logFile is not None:  # close log file
            logFile.close()  # close file handle


if __name__ == "__main__":  # entry point check
    main()  # run main flight
