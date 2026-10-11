#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    Take off to about 1 foot, hover for 10 seconds, and land.
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
    4. Spin motor 1, then 2, then 3, then 4. Each spin is 20 percent for 5 seconds.
    5. Wait 30 seconds.
    6. Type YES to continue.
    7. Wait for a 3D fix and at least 4 satellites.
    8. Shorten TKOFF_SLEW_TIME if it is slower than 1 second.
    9. Enter GUIDED, arm, and command takeoff to 1 foot.
    10. Enter LOITER once the height is reached. Hold for 10 seconds.
    11. Enter LAND.
    12. Put TKOFF_SLEW_TIME back if this script changed it.
    13. Keep every printed line in a flight log named after this file.

IMPORTANT:
    The four spins happen before the hover question. Type YES after them to arm.
    This script does not renumber the motors. The tests showed the
    Pixhawk is already a Quad X. A different frame would tip on liftoff.
    Compass and GPS arming checks stay on.
    Two or three satellites are not enough. A 3D fix and 4 satellites
    are enough to start. The position can wander more than it would with 6.
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

import os as importedOS  # file paths for the flight log
import sys as importedSystem  # the terminal, so the log can share its output
import time as importedTime  # clock for timeouts and the waits

from pymavlink import mavutil as middleMan  # library that speaks to the Pixhawk


class FlightLog:
    """Copy every printed line to the terminal and to the flight log file."""

    def __init__(self, logFile, terminal):
        self.logFile = logFile  # the file beside this script
        self.terminal = terminal  # the screen you are watching

    def write(self, text):
        self.terminal.write(text)  # show the line now
        self.terminal.flush()  # do not hold it in a buffer
        self.logFile.write(text)  # keep the same line in the log
        self.logFile.flush()  # write it to disk immediately

    def flush(self):
        self.terminal.flush()  # finish any pending terminal text
        self.logFile.flush()  # finish any pending log text


def main():
    pixhawkConnection = None  # no link yet
    latestMessages = {}  # newest message of each kind
    messageArrivedAtSeconds = {}  # when each kind last arrived
    lastHeartbeatAtSeconds = None  # no heartbeat saved yet
    aircraftWasArmed = False  # the hover arm has not happened
    activeMotorTestSequence = None  # no motor is spinning yet
    originalTakeoffSlewSeconds = None  # set only if this script changes the ramp
    takeoffSlewParameterName = "TKOFF_SLEW_TIME"  # the Pixhawk takeoff-ramp setting
    flightLogFile = None  # opened after the log folder exists
    flightLogPath = None  # filled in when the log file is created
    originalStandardOutput = importedSystem.stdout  # put the terminal back at the end

    # Stores one real Pixhawk message. Ignores a broken byte.
    def acceptMessage(message):
        nonlocal lastHeartbeatAtSeconds  # share the heartbeat time with the freshness check
        if message is None:
            return  # nothing arrived
        if message.get_type() == "BAD_DATA":
            return  # a garbled byte must not replace the last good message
        if message.get_srcSystem() not in (0, pixhawkConnection.target_system):
            return  # ignore a message from another system
        messageType = message.get_type()  # HEARTBEAT, GPS, height, and so on
        latestMessages[messageType] = message  # keep only the newest of each kind
        messageArrivedAtSeconds[messageType] = importedTime.monotonic()  # mark when it arrived
        if messageType == "HEARTBEAT" and message.get_srcSystem() == pixhawkConnection.target_system:
            lastHeartbeatAtSeconds = importedTime.monotonic()  # this heartbeat is from our Pixhawk
        elif messageType == "STATUSTEXT":
            statusText = getattr(message, "text", "")  # the words the Pixhawk sent
            if isinstance(statusText, bytes):
                statusText = statusText.decode("ascii", "ignore")  # turn those bytes into text
            statusText = str(statusText).strip("\x00").strip()  # drop the blank padding
            if statusText:
                print(f"PIXHAWK: {statusText}")  # show the Pixhawk's own words

    # Reads every message already waiting on the serial port.
    def drainMessages():
        while True:
            message = pixhawkConnection.recv_match(blocking=False)  # take one if one is waiting
            if message is None:
                return  # the port is quiet, so stop looking
            acceptMessage(message)  # store that message

    # Waits up to the given seconds for the next message, then stores it.
    def waitForNextMessage(waitSeconds):
        message = pixhawkConnection.recv_match(blocking=True, timeout=waitSeconds)  # pause for a message
        if message is None:
            return None  # nothing arrived in time
        acceptMessage(message)  # store it before the caller looks at it
        return message  # hand the same message back

    # True when a heartbeat arrived within the last 3 seconds.
    def heartbeatIsFresh():
        heartbeatStaleAfterSeconds = 3.0  # a quiet link is treated as lost after this
        if lastHeartbeatAtSeconds is None:
            return False  # nothing has arrived yet
        heartbeatAgeSeconds = importedTime.monotonic() - lastHeartbeatAtSeconds  # age of the last heartbeat
        return heartbeatAgeSeconds <= heartbeatStaleAfterSeconds  # still recent enough to trust

    # True when this message arrived within the last second.
    def messageIsFresh(messageType):
        sensorMessageStaleAfterSeconds = 1.0  # older GPS, EKF, or height is not used
        arrivedAtSeconds = messageArrivedAtSeconds.get(messageType)  # when this kind last arrived
        if arrivedAtSeconds is None:
            return False  # this kind has never arrived
        return importedTime.monotonic() - arrivedAtSeconds <= sensorMessageStaleAfterSeconds  # still recent

    # None until a heartbeat exists. False means the motors are disarmed.
    def armedState():
        heartbeat = latestMessages.get("HEARTBEAT")  # the newest heartbeat
        if heartbeat is None:
            return None  # arm state is still unknown
        return bool(heartbeat.base_mode & middleMan.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)  # that bit means armed

    # True when the newest heartbeat says the copter is in this mode.
    def modeIs(modeNumber):
        heartbeat = latestMessages.get("HEARTBEAT")  # the newest heartbeat
        if heartbeat is None:
            return False  # mode is unknown, so do not treat it as a match
        return int(heartbeat.custom_mode) == int(modeNumber)  # custom mode is GUIDED, LOITER, or LAND

    # How many satellites the newest GPS message reports. None if that count is missing.
    def satelliteCount():
        unknownSatelliteCount = 255  # 255 means the receiver did not report a count
        if not messageIsFresh("GPS_RAW_INT"):
            return None  # the GPS message is missing or old
        gpsReading = latestMessages.get("GPS_RAW_INT")  # the newest GPS message
        if gpsReading is None:
            return None  # no GPS message is stored
        count = getattr(gpsReading, "satellites_visible", None)  # the satellite field
        if count is None or count == unknownSatelliteCount:
            return None  # the receiver did not give a real count
        return int(count)  # a usable satellite count

    # True for a 3D fix and at least 4 satellites. A GPS glitch still blocks the hover.
    def gpsHoldIsReady():
        minimumFixType = 3  # 3 means a 3D fix
        minimumSatellites = 4  # 4 is the smallest count this hover will accept
        gpsGlitchingBit = 32768  # EKF has rejected the GPS
        if not messageIsFresh("GPS_RAW_INT"):
            return False  # GPS is missing or old
        gpsReading = latestMessages.get("GPS_RAW_INT")  # the newest GPS message
        if gpsReading is None or gpsReading.fix_type is None:
            return False  # no fix type to judge
        if int(gpsReading.fix_type) < minimumFixType:
            return False  # not a 3D fix yet
        count = satelliteCount()  # satellites in that same GPS message
        if count is None or count < minimumSatellites:
            return False  # fewer than 4 satellites
        if not messageIsFresh("EKF_STATUS_REPORT"):
            return False  # the position-health message is missing or old
        ekfStatus = latestMessages.get("EKF_STATUS_REPORT")  # the newest EKF report
        if ekfStatus is None:
            return False  # no EKF report is stored
        gpsGlitchIsActive = bool(int(ekfStatus.flags) & gpsGlitchingBit)  # GPS was rejected
        return not gpsGlitchIsActive  # a reported glitch still means do not hover

    # Height above the spot where the Pixhawk was turned on, in meters.
    def heightMeters():
        if not messageIsFresh("GLOBAL_POSITION_INT"):
            return None  # the height message is missing or old
        positionMessage = latestMessages.get("GLOBAL_POSITION_INT")  # the newest position
        if positionMessage is None:
            return None  # no position is stored
        relativeAltitudeMillimeters = getattr(positionMessage, "relative_alt", None)  # height in millimeters
        if relativeAltitudeMillimeters is None:
            return None  # this message has no height
        return float(relativeAltitudeMillimeters) / 1000.0  # convert millimeters to meters

    # Asks for a mode and waits until the heartbeat confirms it.
    def requestMode(modeNumber, modeName):
        modeChangeTimeoutSeconds = 10  # how long to wait for the mode to change
        print(f"Requesting {modeName}.")
        pixhawkConnection.mav.set_mode_send(  # ask for GUIDED, LOITER, or LAND
            pixhawkConnection.target_system,
            middleMan.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # use the copter mode number
            modeNumber,
        )
        deadlineSeconds = importedTime.monotonic() + modeChangeTimeoutSeconds  # give up after 10 seconds
        while importedTime.monotonic() < deadlineSeconds:
            drainMessages()  # look for the heartbeat that shows the new mode
            if modeIs(modeNumber):
                print(f"{modeName} confirmed.")
                return True  # the Pixhawk is in the mode we asked for
            if not heartbeatIsFresh():
                print("Heartbeat became stale while waiting for the mode change.")
                return False  # the link died during the wait
            importedTime.sleep(0.1)  # check about 10 times a second
        print(f"{modeName} was not confirmed.")
        return False  # the mode never arrived

    # Turns a parameter name from the Pixhawk into plain text.
    def parameterNameText(rawParameterName):
        if isinstance(rawParameterName, bytes):
            rawParameterName = rawParameterName.decode("ascii", "ignore")  # names arrive as bytes
        return str(rawParameterName).strip("\x00").strip()  # drop the blank padding

    # Reads one Pixhawk setting. Returns None if it does not answer in 4 seconds.
    def readParameter(parameterName):
        parameterTimeoutSeconds = 4.0  # how long to wait for this one setting
        pixhawkConnection.mav.param_request_read_send(  # ask for this setting by name
            pixhawkConnection.target_system,
            pixhawkConnection.target_component,
            parameterName.encode("ascii"),
            -1,  # -1 means look up the name, not a numbered slot
        )
        deadlineSeconds = importedTime.monotonic() + parameterTimeoutSeconds  # give up after 4 seconds
        while importedTime.monotonic() < deadlineSeconds:
            message = waitForNextMessage(0.4)  # wait briefly for the next message
            if message is None or message.get_type() != "PARAM_VALUE":
                continue  # not a setting reply
            if parameterNameText(message.param_id) == parameterName:
                return float(message.param_value)  # the value of the setting we asked for
        return None  # no matching reply arrived

    # Writes one Pixhawk setting and waits until the Pixhawk repeats that value.
    def writeParameter(parameterName, requestedValue):
        parameterTimeoutSeconds = 4.0  # how long to wait for the confirmation
        parameterMatchTolerance = 0.01  # close enough to call the value the same
        pixhawkConnection.mav.param_set_send(  # send the new value
            pixhawkConnection.target_system,
            pixhawkConnection.target_component,
            parameterName.encode("ascii"),
            float(requestedValue),
            middleMan.mavlink.MAV_PARAM_TYPE_REAL32,  # these settings are decimal numbers
        )
        deadlineSeconds = importedTime.monotonic() + parameterTimeoutSeconds  # give up after 4 seconds
        while importedTime.monotonic() < deadlineSeconds:
            message = waitForNextMessage(0.4)  # wait briefly for the reply
            if message is None or message.get_type() != "PARAM_VALUE":
                continue  # not a setting reply
            if parameterNameText(message.param_id) != parameterName:
                continue  # a reply for some other setting
            returnedValue = float(message.param_value)  # the value the Pixhawk stored
            if abs(returnedValue - requestedValue) < parameterMatchTolerance:
                return True  # the Pixhawk confirmed the new value
        return False  # the new value was not confirmed

    # Tells the Pixhawk to spin one motor by Quad X test order, not by motor number.
    def sendMotorTest(testSequence, throttlePercent, runSeconds):
        pixhawkConnection.mav.command_long_send(  # one command with seven number fields
            pixhawkConnection.target_system,  # which vehicle
            pixhawkConnection.target_component,  # which autopilot
            middleMan.mavlink.MAV_CMD_DO_MOTOR_TEST,  # the motor-test command
            0,  # not a confirmation resend
            testSequence,  # Quad X test order, not the motor number
            0,  # the next number is a percent
            throttlePercent,  # how hard to spin
            runSeconds,  # how long to spin; 0 means stop
            0,  # this motor only
            0,  # unused
            0,  # unused
        )

    # Waits up to 3 seconds for the Pixhawk to accept or refuse the spin.
    def motorTestAnswer():
        answerTimeoutSeconds = 3.0  # how long to wait for the answer
        deadlineSeconds = importedTime.monotonic() + answerTimeoutSeconds  # give up after 3 seconds
        while importedTime.monotonic() < deadlineSeconds:
            message = waitForNextMessage(0.4)  # wait briefly for the next message
            if message is None or message.get_type() != "COMMAND_ACK":
                continue  # not an answer to a command
            if message.command != middleMan.mavlink.MAV_CMD_DO_MOTOR_TEST:
                continue  # an answer to some other command
            return message.result  # accepted, refused, or another result code
        return None  # no answer arrived in time

    # Stops the motor that is spinning now. A finished spin has nothing to stop.
    def stopActiveMotorTest():
        nonlocal activeMotorTestSequence  # clear the spinning motor when the stop is sent
        if pixhawkConnection is None or activeMotorTestSequence is None:
            return  # no link, or no spin is in progress
        sendMotorTest(activeMotorTestSequence, 0, 0)  # throttle 0 and time 0 stop that test order
        print(f"Stopped test order {activeMotorTestSequence}.")
        activeMotorTestSequence = None  # do not send the same stop again

    # Asks for LAND and waits until the motors disarm.
    def landAndWaitForDisarm():
        landModeNumber = 9  # ArduPilot LAND
        landWaitTimeoutSeconds = 30  # how long to wait for the motors to stop
        statusPrintIntervalSeconds = 1.0  # print height about once a second
        feetPerMeter = 3.28084  # turn the meter height into feet for the print
        if not heartbeatIsFresh():
            print("Heartbeat is stale. LAND was not sent.")  # do not command a dead link
        else:
            print()
            print("LAND")
            requestMode(landModeNumber, "LAND")  # ask for LAND and wait for it to confirm
        landingStartedAtSeconds = importedTime.monotonic()  # start of the disarm wait
        lastPrintedAtSeconds = 0.0  # no height line printed yet
        while importedTime.monotonic() - landingStartedAtSeconds < landWaitTimeoutSeconds:
            drainMessages()  # watch for the disarm heartbeat
            if armedState() is False:
                print("Motors disarmed.")
                return True  # the motors are off
            currentTimeSeconds = importedTime.monotonic()
            if currentTimeSeconds - lastPrintedAtSeconds >= statusPrintIntervalSeconds:
                altitudeMeters = heightMeters()  # current height, or None if it is old
                if altitudeMeters is None:
                    print("LAND | height waiting")
                else:
                    print(f"LAND | {altitudeMeters * feetPerMeter:.2f} ft")  # height during the descent
                lastPrintedAtSeconds = currentTimeSeconds  # do not print again for 1 second
            importedTime.sleep(0.1)  # check about 10 times a second
        print("Landing wait ended before disarm was confirmed.")
        return False  # still armed when the wait ran out

    try:
        flightLogDirectory = importedOS.path.join(  # flight-logs folder beside this script
            importedOS.path.dirname(importedOS.path.abspath(__file__)),
            "flight-logs",
        )
        importedOS.makedirs(flightLogDirectory, exist_ok=True)  # create that folder if it is missing
        startedAt = importedTime.strftime("%Y-%m-%d-%H%M%S")  # timestamp for this run
        scriptName = importedOS.path.splitext(importedOS.path.basename(__file__))[0]  # name without .py
        flightLogPath = importedOS.path.join(  # one log file per run
            flightLogDirectory,
            f"{scriptName}-{startedAt}.log",
        )
        flightLogFile = open(flightLogPath, "w", encoding="utf-8")  # start a new log
        importedSystem.stdout = FlightLog(flightLogFile, importedSystem.stdout)  # copy every print into the log
        print(f"Flight log: {flightLogPath}")
        print()
        print("==============================================")
        print("10 SECOND GPS HOVER, QUAD X")  # title
        print("==============================================")
        print()
        targetAltitudeFeet = 1.0  # the hover height
        metersPerFoot = 0.3048  # the takeoff command uses meters
        feetPerMeter = 3.28084  # the printed height uses feet
        targetAltitudeMeters = targetAltitudeFeet * metersPerFoot  # 1 foot in meters
        hoverTimeSeconds = 10  # how long to hold after the climb
        print(f"Target altitude: {targetAltitudeFeet:.0f} ft")
        print(f"Hover duration: {hoverTimeSeconds:.0f} seconds")
        print("GUIDED takeoff, LOITER, then LAND.")
        print()

        # Pin 8 transmits. Pin 10 receives. Pin 6 is ground. USB stays with the screen.
        pixhawkSerialPort = "/dev/serial0"  # GPIO serial
        pixhawkSerialBaud = 57600  # speed of that link
        connectionTimeoutSeconds = 15  # how long to wait for the first heartbeat
        print(f"Opening {pixhawkSerialPort} at {pixhawkSerialBaud} baud.")
        pixhawkConnection = middleMan.mavlink_connection(  # open pins 8 and 10
            pixhawkSerialPort,
            baud=pixhawkSerialBaud,
            source_system=255,  # this computer is a ground station
            autoreconnect=False,  # do not quietly reopen a dead link
        )
        heartbeat = pixhawkConnection.wait_heartbeat(timeout=connectionTimeoutSeconds)  # wait for the Pixhawk to speak
        if heartbeat is None:
            raise RuntimeError("No Pixhawk heartbeat arrived on pin 10.")  # stop if pin 10 stays silent
        if pixhawkConnection.target_component == 0:
            pixhawkConnection.target_component = 1  # talk to the autopilot itself
        acceptMessage(heartbeat)  # keep that first heartbeat
        print(  # show which Pixhawk answered
            f"Connected to Pixhawk system {pixhawkConnection.target_system}, "
            f"component {pixhawkConnection.target_component}."
        )

        # One message every 0.1 seconds. 100_000 microseconds is that interval.
        messageIntervalMicroseconds = 100_000
        requiredMessageNames = (  # the messages this flight reads
            "HEARTBEAT",
            "GPS_RAW_INT",
            "GLOBAL_POSITION_INT",
            "EKF_STATUS_REPORT",
            "STATUSTEXT",
        )
        for messageName in requiredMessageNames:
            messageId = getattr(middleMan.mavlink, f"MAVLINK_MSG_ID_{messageName}", None)  # number for this message
            if messageId is None:
                print(f"MAVLink message id for {messageName} was not found.")
                continue  # skip a name this library does not know
            pixhawkConnection.mav.command_long_send(  # ask the Pixhawk to send this message steadily
                pixhawkConnection.target_system,
                pixhawkConnection.target_component,
                middleMan.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,  # not a confirmation resend
                messageId,  # which message
                messageIntervalMicroseconds,  # how often, in microseconds
                0,
                0,
                0,
                0,
                0,
            )
        print("Requested GPS and height telemetry.")

        # Quad X flight order. Test order 2 spins motor 4. Flight motor 2 is still the back-left motor.
        quadFrameClass = 1  # Quad
        quadXFrameType = 1  # X, not Plus
        frameClass = readParameter("FRAME_CLASS")  # what kind of frame the Pixhawk has saved
        frameType = readParameter("FRAME_TYPE")  # which layout of that frame
        print()
        print("Flight motor order, looking down:")  # the order the hover uses
        print("  motor 1  front-right  counter-clockwise")
        print("  motor 2  back-left    counter-clockwise")
        print("  motor 3  front-left   clockwise")
        print("  motor 4  back-right   clockwise")
        print("Motor-test order on this frame is 1, 4, 2, 3.")  # the order the spins use
        print("The spins below use that order. The hover still uses the motor numbers above.")
        if frameClass is None or frameType is None:
            raise RuntimeError("FRAME_CLASS or FRAME_TYPE could not be read. Not arming.")  # do not guess the frame
        print(f"FRAME_CLASS {frameClass:.0f} | FRAME_TYPE {frameType:.0f}")
        frameIsQuad = abs(frameClass - quadFrameClass) < 0.01  # saved class is Quad
        frameIsX = abs(frameType - quadXFrameType) < 0.01  # saved layout is X
        if not frameIsQuad or not frameIsX:
            raise RuntimeError("The Pixhawk is not set to Quad X. Not arming.")  # a wrong frame would tip
        print("Frame is Quad X. The motor order matches the motor tests.")

        # Motor number, then the Quad X test order that actually spins that motor.
        motorsToSpin = (
            (1, 1, "front-right", "counter-clockwise"),  # test order 1 is motor 1
            (2, 3, "back-left", "counter-clockwise"),  # test order 3 is motor 2
            (3, 4, "front-left", "clockwise"),  # test order 4 is motor 3
            (4, 2, "back-right", "clockwise"),  # test order 2 is motor 4
        )
        motorSpinThrottlePercent = 20  # light spin before the hover
        motorSpinSeconds = 5  # how long each motor spins
        pauseBetweenMotorsSeconds = 2  # gap so each motor can be seen alone
        pauseBeforeHoverQuestionSeconds = 30  # wait after the last spin
        drainMessages()  # use the newest heartbeat for the armed check
        if armedState() is True:
            print("The copter is armed. The motor spins were not started.")
            return  # do not spin motors that are already armed

        print()
        print("Spinning each motor once before the hover.")
        print(f"Each spin is {motorSpinThrottlePercent} percent for {motorSpinSeconds:.0f} seconds.")
        for motorNumber, testSequence, motorPlace, motorDirection in motorsToSpin:
            print()
            print(f"Motor {motorNumber}: {motorPlace}, {motorDirection}.")  # which motor to watch
            activeMotorTestSequence = testSequence  # remember which test order to stop
            sendMotorTest(testSequence, motorSpinThrottlePercent, motorSpinSeconds)  # start this spin
            motorSpinAnswer = motorTestAnswer()  # wait for accept or refuse
            if motorSpinAnswer is None:
                stopActiveMotorTest()  # do not leave a spin running without an answer
                print(f"No answer for motor {motorNumber}. The hover was not started.")
                return
            print(f"Motor test answer = {motorSpinAnswer}")  # show the result code
            if motorSpinAnswer != middleMan.mavlink.MAV_RESULT_ACCEPTED:
                stopActiveMotorTest()  # the spin was refused
                print("Motor test was refused. Press the safety switch until the LED is solid, then try again.")
                return
            spinDeadlineSeconds = importedTime.monotonic() + motorSpinSeconds  # when this spin is over
            while importedTime.monotonic() < spinDeadlineSeconds:
                drainMessages()  # keep printing status text during the spin
                importedTime.sleep(0.1)  # check about 10 times a second
            stopActiveMotorTest()  # end this spin before the next motor
            print(f"Motor {motorNumber} spin finished.")
            pauseDeadlineSeconds = importedTime.monotonic() + pauseBetweenMotorsSeconds  # short gap
            while importedTime.monotonic() < pauseDeadlineSeconds:
                drainMessages()  # keep the link alive between spins
                importedTime.sleep(0.1)

        drainMessages()  # see whether the motor test left the copter armed
        if armedState() is True:
            print("The copter is still armed after the motor spins. The hover was not started.")
            return  # do not start the hover on top of an unfinished test

        print()
        print(f"Waiting {pauseBeforeHoverQuestionSeconds:.0f} seconds.")
        print("The hover does not start during this wait.")
        waitStartedAtSeconds = importedTime.monotonic()  # start of the 30 second wait
        lastPrintedWaitSecond = -1  # print the countdown once per second
        while importedTime.monotonic() - waitStartedAtSeconds < pauseBeforeHoverQuestionSeconds:
            drainMessages()  # keep reading during the quiet wait
            elapsedSeconds = int(importedTime.monotonic() - waitStartedAtSeconds)  # whole seconds so far
            if elapsedSeconds != lastPrintedWaitSecond:
                remainingSeconds = pauseBeforeHoverQuestionSeconds - elapsedSeconds  # seconds left
                print(f"Hover question in {remainingSeconds:.0f} seconds.")
                lastPrintedWaitSecond = elapsedSeconds  # do not print this second again
            importedTime.sleep(0.1)
        print("Wait finished.")

        print()
        print("Type YES to begin the hover.")
        if input("> ").strip() != "YES":
            print("Cancelled. The hover was not started.")
            return  # anything except YES stops here

        # The prompt and the motor spins can sit for a while. Read the buffer
        # again so the hover uses the messages from after YES.
        drainMessages()

        gpsListenSeconds = 20  # how long to wait for a usable position
        statusPrintIntervalSeconds = 1.0  # print progress about once a second
        print()
        print(f"Listening for GPS for {gpsListenSeconds:.0f} seconds.")
        print("Need a 3D fix and at least 4 satellites.")
        gpsDeadlineSeconds = importedTime.monotonic() + gpsListenSeconds  # give up after 20 seconds
        lastPrintedAtSeconds = 0.0  # no GPS progress line yet
        gpsIsReady = False  # not ready until the check below passes
        while importedTime.monotonic() < gpsDeadlineSeconds:
            drainMessages()  # take the newest GPS and EKF messages
            if not heartbeatIsFresh():
                raise RuntimeError("Pixhawk heartbeat was lost.")  # the link died
            if gpsHoldIsReady():
                gpsReading = latestMessages["GPS_RAW_INT"]  # the fix that passed the check
                print(
                    f"GPS position ready. Fix {gpsReading.fix_type} | "
                    f"{gpsReading.satellites_visible} satellites."
                )
                gpsIsReady = True
                break  # position hold is allowed
            currentTimeSeconds = importedTime.monotonic()
            if currentTimeSeconds - lastPrintedAtSeconds >= statusPrintIntervalSeconds:
                gpsReading = latestMessages.get("GPS_RAW_INT")  # whatever GPS has arrived, if any
                if gpsReading is None:
                    print("GPS: no message yet.")
                else:
                    print(  # show why it is not ready yet
                        f"GPS fix {gpsReading.fix_type} | "
                        f"satellites {satelliteCount()}. Not ready."
                    )
                lastPrintedAtSeconds = currentTimeSeconds
            importedTime.sleep(0.1)  # check about 10 times a second
        if not gpsIsReady:
            raise RuntimeError("GPS is not ready for position hold. Not arming.")  # do not take off

        # A slower ramp leaves one side on the ground. Only shorten a slower value.
        preferredTakeoffSlewTimeSeconds = 1.0  # the firm ramp this flight wants
        parameterMatchTolerance = 0.01  # close enough to leave the saved ramp alone
        savedTakeoffSlewSeconds = readParameter(takeoffSlewParameterName)  # the ramp saved on the Pixhawk
        if savedTakeoffSlewSeconds is None:
            print(f"{takeoffSlewParameterName} was not available. The existing ramp will be used.")
        else:
            temporaryTakeoffSlewSeconds = min(  # never make the ramp slower than it already is
                savedTakeoffSlewSeconds,
                preferredTakeoffSlewTimeSeconds,
            )
            print(  # show the saved ramp and the ramp this flight will use
                f"{takeoffSlewParameterName}: "
                f"original {savedTakeoffSlewSeconds:.2f}s | "
                f"takeoff {temporaryTakeoffSlewSeconds:.2f}s"
            )
            slewAlreadyFirm = (
                abs(temporaryTakeoffSlewSeconds - savedTakeoffSlewSeconds) < parameterMatchTolerance
            )  # true when no change is needed
            if slewAlreadyFirm:
                originalTakeoffSlewSeconds = None  # nothing to put back later
            else:
                slewChanged = writeParameter(takeoffSlewParameterName, temporaryTakeoffSlewSeconds)  # shorten the ramp
                if slewChanged:
                    originalTakeoffSlewSeconds = savedTakeoffSlewSeconds  # put this back after the flight
                else:
                    print(  # the Pixhawk did not confirm the change
                        f"Could not confirm {takeoffSlewParameterName}. "
                        "The existing ramp will be used."
                    )

        guidedModeNumber = 4  # ArduPilot GUIDED
        if not requestMode(guidedModeNumber, "GUIDED"):
            raise RuntimeError("GUIDED was not confirmed.")  # do not arm in the wrong mode

        print()
        print("Sending ARM command.")
        armConfirmationTimeoutSeconds = 10  # how long to wait for the motors to arm
        pixhawkConnection.mav.command_long_send(  # ask the Pixhawk to arm
            pixhawkConnection.target_system,
            pixhawkConnection.target_component,
            middleMan.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # not a confirmation resend
            1,  # 1 means arm
            0,  # a normal arm, not a force arm
            0,
            0,
            0,
            0,
            0,
        )
        armDeadlineSeconds = importedTime.monotonic() + armConfirmationTimeoutSeconds  # give up after 10 seconds
        armCommandResult = None  # no arm answer yet
        armed = False  # not armed until the heartbeat says so
        while importedTime.monotonic() < armDeadlineSeconds:
            message = waitForNextMessage(0.5)  # wait briefly for the next message
            if message is None:
                continue  # nothing yet
            if armedState() is True:
                print("ARMED CONFIRMED.")
                armed = True
                break  # the heartbeat shows the motors are armed
            if message.get_type() != "COMMAND_ACK":
                continue  # not an answer to the arm command
            if message.command != middleMan.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                continue  # an answer to some other command
            armCommandResult = message.result  # accepted, refused, or still in progress
            print(f"ARM answer = {armCommandResult}")
            if armCommandResult not in (
                middleMan.mavlink.MAV_RESULT_ACCEPTED,
                middleMan.mavlink.MAV_RESULT_IN_PROGRESS,
            ):
                print(f"ARM FAILED. Answer = {armCommandResult}")
                break  # the Pixhawk refused to arm
        if not armed:
            if armCommandResult is None or armCommandResult in (
                middleMan.mavlink.MAV_RESULT_ACCEPTED,
                middleMan.mavlink.MAV_RESULT_IN_PROGRESS,
            ):
                print(f"ARM FAILED. Answer = {armCommandResult}")  # timed out, or accepted but never armed
            return  # do not take off
        aircraftWasArmed = True  # from here, Ctrl+C should request LAND

        print()
        print(f"Taking off to {targetAltitudeFeet:.0f} foot.")
        pixhawkConnection.mav.command_long_send(  # ask GUIDED to climb to 1 foot
            pixhawkConnection.target_system,
            pixhawkConnection.target_component,
            middleMan.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,  # not a confirmation resend
            0,
            0,
            0,
            0,
            0,
            0,
            targetAltitudeMeters,  # the last field is the height in meters
        )

        takeoffReachedFraction = 0.90  # close enough to call the climb done
        takeoffTimeoutSeconds = 20  # how long the climb may take
        minimumTakeoffAltitudeMeters = targetAltitudeMeters * takeoffReachedFraction  # 90 percent of 1 foot
        takeoffDeadlineSeconds = importedTime.monotonic() + takeoffTimeoutSeconds  # give up after 20 seconds
        lastPrintedAtSeconds = 0.0  # no climb line printed yet
        takeoffReached = False  # not high enough yet
        takeoffStoppedEarly = False  # true if the climb is abandoned before the timeout
        while importedTime.monotonic() < takeoffDeadlineSeconds:
            drainMessages()  # watch height, mode, arm state, and GPS
            if not heartbeatIsFresh():
                print("Heartbeat was lost during takeoff.")
                takeoffStoppedEarly = True
                break  # the link died
            if armedState() is not True:
                print("The copter disarmed during takeoff.")
                takeoffStoppedEarly = True
                break  # the motors stopped on their own
            if not modeIs(guidedModeNumber):
                print("GUIDED ended during takeoff.")
                takeoffStoppedEarly = True
                break  # the climb mode was left
            if not gpsHoldIsReady():
                print("GPS position was lost during takeoff.")
                takeoffStoppedEarly = True
                break  # position hold is no longer valid
            altitudeMeters = heightMeters()  # current height, or None if it is old
            currentTimeSeconds = importedTime.monotonic()
            if currentTimeSeconds - lastPrintedAtSeconds >= statusPrintIntervalSeconds:
                if altitudeMeters is None:
                    print("TAKEOFF | height waiting")
                else:
                    print(f"TAKEOFF | {altitudeMeters * feetPerMeter:.2f} ft")  # climb progress
                lastPrintedAtSeconds = currentTimeSeconds
            if altitudeMeters is not None and altitudeMeters >= minimumTakeoffAltitudeMeters:
                print("Takeoff height reached.")
                takeoffReached = True
                break  # high enough to start the hover
            importedTime.sleep(0.1)  # check about 10 times a second
        if not takeoffReached:
            if not takeoffStoppedEarly:
                print(f"Did not reach {targetAltitudeFeet:.0f} foot.")  # the 20 seconds ran out
            landAndWaitForDisarm()  # come back down instead of hovering low
            return

        loiterModeNumber = 5  # ArduPilot LOITER
        if not requestMode(loiterModeNumber, "LOITER"):
            raise RuntimeError("LOITER was not confirmed.")  # the land path below handles an armed copter

        print()
        print(f"LOITER for {hoverTimeSeconds:.0f} seconds.")
        hoverStartedAtSeconds = importedTime.monotonic()  # start of the 10 second hold
        lastPrintedAtSeconds = 0.0  # no hover line printed yet
        while importedTime.monotonic() - hoverStartedAtSeconds < hoverTimeSeconds:
            drainMessages()  # watch the link, the mode, and the GPS
            if not heartbeatIsFresh():
                raise RuntimeError("Pixhawk heartbeat was lost.")  # the link died during the hold
            if armedState() is not True:
                raise RuntimeError("The copter became disarmed.")  # the motors stopped during the hold
            if not modeIs(loiterModeNumber):
                raise RuntimeError("LOITER changed during the hover.")  # the hold mode was left
            if not gpsHoldIsReady():
                raise RuntimeError("GPS position was lost during the hover.")  # position is no longer healthy
            currentTimeSeconds = importedTime.monotonic()
            if currentTimeSeconds - lastPrintedAtSeconds >= statusPrintIntervalSeconds:
                altitudeMeters = heightMeters()  # current height, or None if it is old
                hoverElapsedSeconds = currentTimeSeconds - hoverStartedAtSeconds  # seconds already held
                if altitudeMeters is None:
                    heightText = "height waiting"
                else:
                    heightText = f"{altitudeMeters * feetPerMeter:.2f} ft"  # height in feet
                print(  # one progress line per second
                    f"LOITER | {heightText} | satellites {satelliteCount()} | "
                    f"hover {hoverElapsedSeconds:.0f}/{hoverTimeSeconds:.0f}s"
                )
                lastPrintedAtSeconds = currentTimeSeconds
            importedTime.sleep(0.1)  # check about 10 times a second
        print()
        print("10 second GPS hover complete.")

        landAndWaitForDisarm()  # the planned landing

    except KeyboardInterrupt:
        print()
        print("Ctrl+C.")
        stopActiveMotorTest()  # stop a spin if one is still running
        if pixhawkConnection is not None and aircraftWasArmed and heartbeatIsFresh():
            try:
                landAndWaitForDisarm()  # the hover had armed, so come down
            except KeyboardInterrupt:
                print("Second Ctrl+C. Closing the link. The copter is not force-disarmed.")  # leave the link only

    except Exception as error:
        print()
        print(f"STOPPED: {error}")  # say why the flight stopped
        stopActiveMotorTest()  # stop a spin if one is still running
        if pixhawkConnection is not None and aircraftWasArmed and heartbeatIsFresh():
            try:
                landAndWaitForDisarm()  # come down if the hover had armed
            except Exception as landingError:
                print(f"Landing error: {landingError}")  # the landing itself failed

    finally:
        stopActiveMotorTest()  # one more stop, in case an exit skipped the earlier one
        if (
            pixhawkConnection is not None
            and originalTakeoffSlewSeconds is not None
            and heartbeatIsFresh()
        ):
            try:
                print(f"Restoring {takeoffSlewParameterName} to {originalTakeoffSlewSeconds:.2f}s.")
                restored = writeParameter(takeoffSlewParameterName, originalTakeoffSlewSeconds)  # put the old ramp back
                if restored:
                    print(f"{takeoffSlewParameterName} restored.")
                else:
                    print(f"Could not confirm restoration of {takeoffSlewParameterName}.")
            except Exception as restoreError:
                print(f"Could not restore {takeoffSlewParameterName}: {restoreError}")
        if pixhawkConnection is not None:
            try:
                pixhawkConnection.close()  # release pins 8 and 10
            except Exception:
                pass  # a dead link should not hide the result
            print("Pixhawk connection closed.")
        print("Finished.")
        if flightLogPath is not None:
            print(f"Flight log saved: {flightLogPath}")  # where to read this run
        importedSystem.stdout = originalStandardOutput  # stop copying prints into the log
        if flightLogFile is not None:
            flightLogFile.close()  # close the log file


if __name__ == "__main__":
    main()  # run only when this file is started, not when it is imported
