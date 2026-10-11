#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    Take off to about 1 foot, hover for 30 seconds, and land.
    GPS holds the horizontal position.
    Arm only when the Pixhawk frame is Quad X, which is the motor
    order confirmed by the motor tests.

WHY:
    A slow throttle ramp leaves one side on the ground long enough to tip.
    ArduPilot flies this climb in GUIDED, then holds the spot in LOITER.
    The script does not command each motor. The Pixhawk mixes them.
    The motor-test command uses a different order from the flight motors.
    On this Quad X that test order is 1, then 4, then 2, then 3.
    Flight still uses the motor numbers:
    motor 1 front-right, motor 2 back-left, motor 3 front-left, motor 4 back-right.
    The takeoff ramp is made firmer only when the saved ramp is slower
    than 1 second. An already-firm ramp is left alone.

HOW:
    1. Connect on pins 8 and 10. Leave USB with the screen.
    2. Request heartbeat, GPS, position, EKF status, and status text.
    3. Read FRAME_CLASS and FRAME_TYPE. Stop if this is not a Quad X.
    4. Wait for a 3D fix, at least 6 satellites, and a healthy position.
    5. Type YES. Shorten TKOFF_SLEW_TIME if it is slower than 1 second.
    6. Enter GUIDED, arm, and command takeoff to 1 foot.
    7. Enter LOITER once the height is reached. Hold for 30 seconds.
    8. Enter LAND.
    9. Put TKOFF_SLEW_TIME back if this script changed it.
    10. Keep every printed line in a flight log named after this file.

IMPORTANT:
    Type YES before it arms. This script sends the arm command.
    This script does not renumber the motors. The tests showed the
    Pixhawk is already a Quad X. A different frame would tip on liftoff.
    Compass and GPS arming checks stay on.
    Two or three satellites are not enough. Position hold needs a 3D fix,
    at least 6 satellites, and a healthy horizontal position.
    The Pixhawk already has the takeoff throttle cap and the higher
    attitude mix. This script does not change those.
    Ctrl+C requests LAND. A second Ctrl+C closes the link.
    This script does not force-disarm.
    After the test, read the newest file in the flight-logs folder
    beside this script.

    middleMan is pymavlink's mavutil. The flight steps below are the story.
    The small helpers inside main() are only the checks that takeoff, hover,
    and landing all share, so those checks stay one copy.
"""

import os as importedOS
import sys as importedSystem
import time as importedTime

from pymavlink import mavutil as middleMan


class FlightLog:
    """Copy every printed line to the terminal and to the flight log file."""

    def __init__(self, logFile, terminal):
        self.logFile = logFile
        self.terminal = terminal

    def write(self, text):
        self.terminal.write(text)
        self.terminal.flush()
        self.logFile.write(text)
        self.logFile.flush()

    def flush(self):
        self.terminal.flush()
        self.logFile.flush()


def main():
    pixhawkConnection = None
    latestMessages = {}
    messageArrivedAtSeconds = {}
    lastHeartbeatAtSeconds = None
    aircraftWasArmed = False
    originalTakeoffSlewSeconds = None
    takeoffSlewParameterName = "TKOFF_SLEW_TIME"
    flightLogFile = None
    flightLogPath = None
    originalStandardOutput = importedSystem.stdout

    def acceptMessage(message):
        """Keep a real Pixhawk message. Ignore a broken serial byte."""
        nonlocal lastHeartbeatAtSeconds
        if message is None:
            return
        if message.get_type() == "BAD_DATA":
            return
        if message.get_srcSystem() not in (0, pixhawkConnection.target_system):
            return
        messageType = message.get_type()
        latestMessages[messageType] = message
        messageArrivedAtSeconds[messageType] = importedTime.monotonic()
        if messageType == "HEARTBEAT" and message.get_srcSystem() == pixhawkConnection.target_system:
            lastHeartbeatAtSeconds = importedTime.monotonic()
        elif messageType == "STATUSTEXT":
            statusText = getattr(message, "text", "")
            if isinstance(statusText, bytes):
                statusText = statusText.decode("ascii", "ignore")
            statusText = str(statusText).strip("\x00").strip()
            if statusText:
                print(f"PIXHAWK: {statusText}")

    def drainMessages():
        while True:
            message = pixhawkConnection.recv_match(blocking=False)
            if message is None:
                return
            acceptMessage(message)

    def waitForNextMessage(waitSeconds):
        message = pixhawkConnection.recv_match(blocking=True, timeout=waitSeconds)
        if message is None:
            return None
        acceptMessage(message)
        return message

    def heartbeatIsFresh():
        # Three seconds without a heartbeat means the link is gone.
        heartbeatStaleAfterSeconds = 3.0
        if lastHeartbeatAtSeconds is None:
            return False
        heartbeatAgeSeconds = importedTime.monotonic() - lastHeartbeatAtSeconds
        return heartbeatAgeSeconds <= heartbeatStaleAfterSeconds

    def messageIsFresh(messageType):
        # GPS, EKF, and height older than this are not used for a decision.
        sensorMessageStaleAfterSeconds = 1.0
        arrivedAtSeconds = messageArrivedAtSeconds.get(messageType)
        if arrivedAtSeconds is None:
            return False
        return importedTime.monotonic() - arrivedAtSeconds <= sensorMessageStaleAfterSeconds

    def armedState():
        """None until a heartbeat exists. False means the motors are disarmed."""
        heartbeat = latestMessages.get("HEARTBEAT")
        if heartbeat is None:
            return None
        return bool(heartbeat.base_mode & middleMan.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def modeIs(modeNumber):
        heartbeat = latestMessages.get("HEARTBEAT")
        if heartbeat is None:
            return False
        return int(heartbeat.custom_mode) == int(modeNumber)

    def satelliteCount():
        # 255 means the receiver did not report a count.
        unknownSatelliteCount = 255
        if not messageIsFresh("GPS_RAW_INT"):
            return None
        gpsReading = latestMessages.get("GPS_RAW_INT")
        if gpsReading is None:
            return None
        count = getattr(gpsReading, "satellites_visible", None)
        if count is None or count == unknownSatelliteCount:
            return None
        return int(count)

    def gpsHoldIsReady():
        """LOITER needs a 3D fix, 6 satellites, and a healthy horizontal position."""
        minimumFixType = 3
        minimumSatellites = 6
        # EKF flags: absolute horizontal position, constant-position mode, GPS glitch.
        absoluteHorizontalPositionBit = 16
        constantPositionModeBit = 128
        gpsGlitchingBit = 32768
        if not messageIsFresh("GPS_RAW_INT"):
            return False
        gpsReading = latestMessages.get("GPS_RAW_INT")
        if gpsReading is None or gpsReading.fix_type is None:
            return False
        if int(gpsReading.fix_type) < minimumFixType:
            return False
        count = satelliteCount()
        if count is None or count < minimumSatellites:
            return False
        if not messageIsFresh("EKF_STATUS_REPORT"):
            return False
        ekfStatus = latestMessages.get("EKF_STATUS_REPORT")
        if ekfStatus is None:
            return False
        ekfFlags = int(ekfStatus.flags)
        hasAbsoluteHorizontalPosition = bool(ekfFlags & absoluteHorizontalPositionBit)
        constantPositionModeIsActive = bool(ekfFlags & constantPositionModeBit)
        gpsGlitchIsActive = bool(ekfFlags & gpsGlitchingBit)
        return (
            hasAbsoluteHorizontalPosition
            and not constantPositionModeIsActive
            and not gpsGlitchIsActive
        )

    def heightMeters():
        if not messageIsFresh("GLOBAL_POSITION_INT"):
            return None
        positionMessage = latestMessages.get("GLOBAL_POSITION_INT")
        if positionMessage is None:
            return None
        relativeAltitudeMillimeters = getattr(positionMessage, "relative_alt", None)
        if relativeAltitudeMillimeters is None:
            return None
        return float(relativeAltitudeMillimeters) / 1000.0

    def requestMode(modeNumber, modeName):
        modeChangeTimeoutSeconds = 10
        print(f"Requesting {modeName}.")
        pixhawkConnection.mav.set_mode_send(
            pixhawkConnection.target_system,
            middleMan.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            modeNumber,
        )
        deadlineSeconds = importedTime.monotonic() + modeChangeTimeoutSeconds
        while importedTime.monotonic() < deadlineSeconds:
            drainMessages()
            if modeIs(modeNumber):
                print(f"{modeName} confirmed.")
                return True
            if not heartbeatIsFresh():
                print("Heartbeat became stale while waiting for the mode change.")
                return False
            importedTime.sleep(0.1)
        print(f"{modeName} was not confirmed.")
        return False

    def parameterNameText(rawParameterName):
        if isinstance(rawParameterName, bytes):
            rawParameterName = rawParameterName.decode("ascii", "ignore")
        return str(rawParameterName).strip("\x00").strip()

    def readParameter(parameterName):
        parameterTimeoutSeconds = 4.0
        pixhawkConnection.mav.param_request_read_send(
            pixhawkConnection.target_system,
            pixhawkConnection.target_component,
            parameterName.encode("ascii"),
            -1,
        )
        deadlineSeconds = importedTime.monotonic() + parameterTimeoutSeconds
        while importedTime.monotonic() < deadlineSeconds:
            message = waitForNextMessage(0.4)
            if message is None or message.get_type() != "PARAM_VALUE":
                continue
            if parameterNameText(message.param_id) == parameterName:
                return float(message.param_value)
        return None

    def writeParameter(parameterName, requestedValue):
        parameterTimeoutSeconds = 4.0
        parameterMatchTolerance = 0.01
        pixhawkConnection.mav.param_set_send(
            pixhawkConnection.target_system,
            pixhawkConnection.target_component,
            parameterName.encode("ascii"),
            float(requestedValue),
            middleMan.mavlink.MAV_PARAM_TYPE_REAL32,
        )
        deadlineSeconds = importedTime.monotonic() + parameterTimeoutSeconds
        while importedTime.monotonic() < deadlineSeconds:
            message = waitForNextMessage(0.4)
            if message is None or message.get_type() != "PARAM_VALUE":
                continue
            if parameterNameText(message.param_id) != parameterName:
                continue
            returnedValue = float(message.param_value)
            if abs(returnedValue - requestedValue) < parameterMatchTolerance:
                return True
        return False

    def landAndWaitForDisarm():
        landModeNumber = 9
        landWaitTimeoutSeconds = 30
        statusPrintIntervalSeconds = 1.0
        feetPerMeter = 3.28084
        if not heartbeatIsFresh():
            print("Heartbeat is stale. LAND was not sent.")
        else:
            print()
            print("LAND")
            requestMode(landModeNumber, "LAND")
        landingStartedAtSeconds = importedTime.monotonic()
        lastPrintedAtSeconds = 0.0
        while importedTime.monotonic() - landingStartedAtSeconds < landWaitTimeoutSeconds:
            drainMessages()
            if armedState() is False:
                print("Motors disarmed.")
                return True
            currentTimeSeconds = importedTime.monotonic()
            if currentTimeSeconds - lastPrintedAtSeconds >= statusPrintIntervalSeconds:
                altitudeMeters = heightMeters()
                if altitudeMeters is None:
                    print("LAND | height waiting")
                else:
                    print(f"LAND | {altitudeMeters * feetPerMeter:.2f} ft")
                lastPrintedAtSeconds = currentTimeSeconds
            importedTime.sleep(0.1)
        print("Landing wait ended before disarm was confirmed.")
        return False

    try:
        flightLogDirectory = importedOS.path.join(
            importedOS.path.dirname(importedOS.path.abspath(__file__)),
            "flight-logs",
        )
        importedOS.makedirs(flightLogDirectory, exist_ok=True)
        startedAt = importedTime.strftime("%Y-%m-%d-%H%M%S")
        scriptName = importedOS.path.splitext(importedOS.path.basename(__file__))[0]
        flightLogPath = importedOS.path.join(
            flightLogDirectory,
            f"{scriptName}-{startedAt}.log",
        )
        flightLogFile = open(flightLogPath, "w", encoding="utf-8")
        importedSystem.stdout = FlightLog(flightLogFile, importedSystem.stdout)
        print(f"Flight log: {flightLogPath}")
        print()
        print("==============================================")
        print("30 SECOND GPS HOVER, QUAD X")
        print("==============================================")
        print()
        targetAltitudeFeet = 1.0
        metersPerFoot = 0.3048
        feetPerMeter = 3.28084
        targetAltitudeMeters = targetAltitudeFeet * metersPerFoot
        hoverTimeSeconds = 30
        print(f"Target altitude: {targetAltitudeFeet:.0f} ft")
        print(f"Hover duration: {hoverTimeSeconds:.0f} seconds")
        print("GUIDED takeoff, LOITER, then LAND.")
        print()

        # Pin 8 transmits. Pin 10 receives. Pin 6 is ground. USB stays with the screen.
        pixhawkSerialPort = "/dev/serial0"
        pixhawkSerialBaud = 57600
        connectionTimeoutSeconds = 15
        print(f"Opening {pixhawkSerialPort} at {pixhawkSerialBaud} baud.")
        pixhawkConnection = middleMan.mavlink_connection(
            pixhawkSerialPort,
            baud=pixhawkSerialBaud,
            source_system=255,
            autoreconnect=False,
        )
        heartbeat = pixhawkConnection.wait_heartbeat(timeout=connectionTimeoutSeconds)
        if heartbeat is None:
            raise RuntimeError("No Pixhawk heartbeat arrived on pin 10.")
        if pixhawkConnection.target_component == 0:
            pixhawkConnection.target_component = 1
        acceptMessage(heartbeat)
        print(
            f"Connected to Pixhawk system {pixhawkConnection.target_system}, "
            f"component {pixhawkConnection.target_component}."
        )

        # One message every 0.1 seconds. 100_000 microseconds is that interval.
        messageIntervalMicroseconds = 100_000
        requiredMessageNames = (
            "HEARTBEAT",
            "GPS_RAW_INT",
            "GLOBAL_POSITION_INT",
            "EKF_STATUS_REPORT",
            "STATUSTEXT",
        )
        for messageName in requiredMessageNames:
            messageId = getattr(middleMan.mavlink, f"MAVLINK_MSG_ID_{messageName}", None)
            if messageId is None:
                print(f"MAVLink message id for {messageName} was not found.")
                continue
            pixhawkConnection.mav.command_long_send(
                pixhawkConnection.target_system,
                pixhawkConnection.target_component,
                middleMan.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                messageId,
                messageIntervalMicroseconds,
                0,
                0,
                0,
                0,
                0,
            )
        print("Requested GPS and height telemetry.")

        # Quad X flight order. This is not the motor-test order.
        # Test order 2 spins motor 4. Flight motor 2 is still the back-left motor.
        quadFrameClass = 1
        quadXFrameType = 1
        frameClass = readParameter("FRAME_CLASS")
        frameType = readParameter("FRAME_TYPE")
        print()
        print("Flight motor order, looking down:")
        print("  motor 1  front-right  counter-clockwise")
        print("  motor 2  back-left    counter-clockwise")
        print("  motor 3  front-left   clockwise")
        print("  motor 4  back-right   clockwise")
        print("Motor-test order on this frame is 1, 4, 2, 3. This flight does not use it.")
        if frameClass is None or frameType is None:
            raise RuntimeError("FRAME_CLASS or FRAME_TYPE could not be read. Not arming.")
        print(f"FRAME_CLASS {frameClass:.0f} | FRAME_TYPE {frameType:.0f}")
        frameIsQuad = abs(frameClass - quadFrameClass) < 0.01
        frameIsX = abs(frameType - quadXFrameType) < 0.01
        if not frameIsQuad or not frameIsX:
            raise RuntimeError("The Pixhawk is not set to Quad X. Not arming.")
        print("Frame is Quad X. The motor order matches the motor tests.")

        gpsListenSeconds = 20
        statusPrintIntervalSeconds = 1.0
        print()
        print(f"Listening for GPS for {gpsListenSeconds:.0f} seconds.")
        print("Need a 3D fix, at least 6 satellites, and a healthy position.")
        gpsDeadlineSeconds = importedTime.monotonic() + gpsListenSeconds
        lastPrintedAtSeconds = 0.0
        gpsIsReady = False
        while importedTime.monotonic() < gpsDeadlineSeconds:
            drainMessages()
            if not heartbeatIsFresh():
                raise RuntimeError("Pixhawk heartbeat was lost.")
            if gpsHoldIsReady():
                gpsReading = latestMessages["GPS_RAW_INT"]
                print(
                    f"GPS position ready. Fix {gpsReading.fix_type} | "
                    f"{gpsReading.satellites_visible} satellites."
                )
                gpsIsReady = True
                break
            currentTimeSeconds = importedTime.monotonic()
            if currentTimeSeconds - lastPrintedAtSeconds >= statusPrintIntervalSeconds:
                gpsReading = latestMessages.get("GPS_RAW_INT")
                if gpsReading is None:
                    print("GPS: no message yet.")
                else:
                    print(
                        f"GPS fix {gpsReading.fix_type} | "
                        f"satellites {satelliteCount()}. Not ready."
                    )
                lastPrintedAtSeconds = currentTimeSeconds
            importedTime.sleep(0.1)
        if not gpsIsReady:
            raise RuntimeError("GPS is not ready for position hold. Not arming.")

        print()
        print("Type YES to begin.")
        if input("> ").strip() != "YES":
            print("Cancelled.")
            return

        # The prompt can sit open for a while. Read the buffer again so the
        # takeoff uses the GPS fix from after YES, not the one from before it.
        drainMessages()

        # A slower ramp leaves one side on the ground. Only shorten a slower value.
        preferredTakeoffSlewTimeSeconds = 1.0
        parameterMatchTolerance = 0.01
        savedTakeoffSlewSeconds = readParameter(takeoffSlewParameterName)
        if savedTakeoffSlewSeconds is None:
            print(f"{takeoffSlewParameterName} was not available. The existing ramp will be used.")
        else:
            temporaryTakeoffSlewSeconds = min(
                savedTakeoffSlewSeconds,
                preferredTakeoffSlewTimeSeconds,
            )
            print(
                f"{takeoffSlewParameterName}: "
                f"original {savedTakeoffSlewSeconds:.2f}s | "
                f"takeoff {temporaryTakeoffSlewSeconds:.2f}s"
            )
            slewAlreadyFirm = (
                abs(temporaryTakeoffSlewSeconds - savedTakeoffSlewSeconds) < parameterMatchTolerance
            )
            if slewAlreadyFirm:
                originalTakeoffSlewSeconds = None
            else:
                slewChanged = writeParameter(takeoffSlewParameterName, temporaryTakeoffSlewSeconds)
                if slewChanged:
                    originalTakeoffSlewSeconds = savedTakeoffSlewSeconds
                else:
                    print(
                        f"Could not confirm {takeoffSlewParameterName}. "
                        "The existing ramp will be used."
                    )

        guidedModeNumber = 4
        if not requestMode(guidedModeNumber, "GUIDED"):
            raise RuntimeError("GUIDED was not confirmed.")

        print()
        print("Sending ARM command.")
        armConfirmationTimeoutSeconds = 10
        pixhawkConnection.mav.command_long_send(
            pixhawkConnection.target_system,
            pixhawkConnection.target_component,
            middleMan.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        armDeadlineSeconds = importedTime.monotonic() + armConfirmationTimeoutSeconds
        armCommandResult = None
        armed = False
        while importedTime.monotonic() < armDeadlineSeconds:
            message = waitForNextMessage(0.5)
            if message is None:
                continue
            if armedState() is True:
                print("ARMED CONFIRMED.")
                armed = True
                break
            if message.get_type() != "COMMAND_ACK":
                continue
            if message.command != middleMan.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                continue
            armCommandResult = message.result
            print(f"ARM answer = {armCommandResult}")
            if armCommandResult not in (
                middleMan.mavlink.MAV_RESULT_ACCEPTED,
                middleMan.mavlink.MAV_RESULT_IN_PROGRESS,
            ):
                print(f"ARM FAILED. Answer = {armCommandResult}")
                break
        if not armed:
            if armCommandResult is None or armCommandResult in (
                middleMan.mavlink.MAV_RESULT_ACCEPTED,
                middleMan.mavlink.MAV_RESULT_IN_PROGRESS,
            ):
                print(f"ARM FAILED. Answer = {armCommandResult}")
            return
        aircraftWasArmed = True

        print()
        print(f"Taking off to {targetAltitudeFeet:.0f} foot.")
        pixhawkConnection.mav.command_long_send(
            pixhawkConnection.target_system,
            pixhawkConnection.target_component,
            middleMan.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            targetAltitudeMeters,
        )

        takeoffReachedFraction = 0.90
        takeoffTimeoutSeconds = 20
        minimumTakeoffAltitudeMeters = targetAltitudeMeters * takeoffReachedFraction
        takeoffDeadlineSeconds = importedTime.monotonic() + takeoffTimeoutSeconds
        lastPrintedAtSeconds = 0.0
        takeoffReached = False
        takeoffStoppedEarly = False
        while importedTime.monotonic() < takeoffDeadlineSeconds:
            drainMessages()
            if not heartbeatIsFresh():
                print("Heartbeat was lost during takeoff.")
                takeoffStoppedEarly = True
                break
            if armedState() is not True:
                print("The copter disarmed during takeoff.")
                takeoffStoppedEarly = True
                break
            if not modeIs(guidedModeNumber):
                print("GUIDED ended during takeoff.")
                takeoffStoppedEarly = True
                break
            if not gpsHoldIsReady():
                print("GPS position was lost during takeoff.")
                takeoffStoppedEarly = True
                break
            altitudeMeters = heightMeters()
            currentTimeSeconds = importedTime.monotonic()
            if currentTimeSeconds - lastPrintedAtSeconds >= statusPrintIntervalSeconds:
                if altitudeMeters is None:
                    print("TAKEOFF | height waiting")
                else:
                    print(f"TAKEOFF | {altitudeMeters * feetPerMeter:.2f} ft")
                lastPrintedAtSeconds = currentTimeSeconds
            if altitudeMeters is not None and altitudeMeters >= minimumTakeoffAltitudeMeters:
                print("Takeoff height reached.")
                takeoffReached = True
                break
            importedTime.sleep(0.1)
        if not takeoffReached:
            if not takeoffStoppedEarly:
                print(f"Did not reach {targetAltitudeFeet:.0f} foot.")
            landAndWaitForDisarm()
            return

        loiterModeNumber = 5
        if not requestMode(loiterModeNumber, "LOITER"):
            raise RuntimeError("LOITER was not confirmed.")

        print()
        print(f"LOITER for {hoverTimeSeconds:.0f} seconds.")
        hoverStartedAtSeconds = importedTime.monotonic()
        lastPrintedAtSeconds = 0.0
        while importedTime.monotonic() - hoverStartedAtSeconds < hoverTimeSeconds:
            drainMessages()
            if not heartbeatIsFresh():
                raise RuntimeError("Pixhawk heartbeat was lost.")
            if armedState() is not True:
                raise RuntimeError("The copter became disarmed.")
            if not modeIs(loiterModeNumber):
                raise RuntimeError("LOITER changed during the hover.")
            if not gpsHoldIsReady():
                raise RuntimeError("GPS position was lost during the hover.")
            currentTimeSeconds = importedTime.monotonic()
            if currentTimeSeconds - lastPrintedAtSeconds >= statusPrintIntervalSeconds:
                altitudeMeters = heightMeters()
                hoverElapsedSeconds = currentTimeSeconds - hoverStartedAtSeconds
                if altitudeMeters is None:
                    heightText = "height waiting"
                else:
                    heightText = f"{altitudeMeters * feetPerMeter:.2f} ft"
                print(
                    f"LOITER | {heightText} | satellites {satelliteCount()} | "
                    f"hover {hoverElapsedSeconds:.0f}/{hoverTimeSeconds:.0f}s"
                )
                lastPrintedAtSeconds = currentTimeSeconds
            importedTime.sleep(0.1)
        print()
        print("30 second GPS hover complete.")

        landAndWaitForDisarm()

    except KeyboardInterrupt:
        print()
        print("Ctrl+C.")
        if pixhawkConnection is not None and aircraftWasArmed and heartbeatIsFresh():
            try:
                landAndWaitForDisarm()
            except KeyboardInterrupt:
                print("Second Ctrl+C. Closing the link. The copter is not force-disarmed.")

    except Exception as error:
        print()
        print(f"STOPPED: {error}")
        if pixhawkConnection is not None and aircraftWasArmed and heartbeatIsFresh():
            try:
                landAndWaitForDisarm()
            except Exception as landingError:
                print(f"Landing error: {landingError}")

    finally:
        if (
            pixhawkConnection is not None
            and originalTakeoffSlewSeconds is not None
            and heartbeatIsFresh()
        ):
            try:
                print(f"Restoring {takeoffSlewParameterName} to {originalTakeoffSlewSeconds:.2f}s.")
                restored = writeParameter(takeoffSlewParameterName, originalTakeoffSlewSeconds)
                if restored:
                    print(f"{takeoffSlewParameterName} restored.")
                else:
                    print(f"Could not confirm restoration of {takeoffSlewParameterName}.")
            except Exception as restoreError:
                print(f"Could not restore {takeoffSlewParameterName}: {restoreError}")
        if pixhawkConnection is not None:
            try:
                pixhawkConnection.close()
            except Exception:
                pass
            print("Pixhawk connection closed.")
        print("Finished.")
        if flightLogPath is not None:
            print(f"Flight log saved: {flightLogPath}")
        importedSystem.stdout = originalStandardOutput
        if flightLogFile is not None:
            flightLogFile.close()


if __name__ == "__main__":
    main()
