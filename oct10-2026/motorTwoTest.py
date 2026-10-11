#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    Spin motor 2 only, for a few seconds, while the copter stays disarmed.

WHY:
    On this Quad X, motor 2 is the back-left motor.
    Looking down on the copter, that motor should turn counter-clockwise.
    A separate script makes it obvious which motor is being asked to spin.

HOW:
    1. Connect on pins 8 and 10. Leave USB with the screen.
    2. Type YES.
    3. Refuse to start if the copter is armed.
    4. Ask the Pixhawk to run motor 2 at 20 percent for 5 seconds.
    5. Wait for that spin to finish, and print any Pixhawk status text.
    6. Tell motor 2 to stop, then close the link.

IMPORTANT:
    Take the propellers off before you run this.
    This script does not arm the Pixhawk.
    The safety switch must be pressed, so the LED is solid, or the
    Pixhawk will refuse the motor test.
    Ctrl+C tells motor 2 to stop. The copter was never armed.
"""

import time  # clock for the timeouts and the 5 second wait

from pymavlink import mavutil  # library that speaks to the Pixhawk


# Pin 8 transmits. Pin 10 receives. Pin 6 is ground.
PIXHAWK_SERIAL_PORT = "/dev/serial0"  # GPIO serial, so USB stays with the screen
PIXHAWK_SERIAL_BAUD = 57600  # speed of that serial link

CONNECTION_TIMEOUT_SECONDS = 15  # how long to wait for the first heartbeat
HEARTBEAT_STALE_AFTER_SECONDS = 5.0  # a quiet link is treated as lost after this
MOTOR_TEST_ANSWER_TIMEOUT_SECONDS = 5.0  # how long to wait for accept or refuse

# ArduPilot motor 2 on a Quad X is the back-left motor.
MOTOR_NUMBER = 2  # the motor this file is about
MOTOR_THROTTLE_PERCENT = 20  # light spin, as a percent of full output
MOTOR_RUN_SECONDS = 5  # how long that spin lasts

# The motor-test command does not take the motor number. It takes the Quad X
# test order, and that order is not 1, 2, 3, 4:
#   test order 1 spins motor 1, front-right, counter-clockwise
#   test order 2 spins motor 4, back-right, clockwise
#   test order 3 spins motor 2, back-left, counter-clockwise
#   test order 4 spins motor 3, front-left, clockwise
# Sending 2 is what made this script spin motor 4 clockwise.
MOTOR_TEST_SEQUENCE = 3  # the test order that actually spins motor 2

# Throttle type 0 means the number above is a percent, not a PWM pulse.
MOTOR_TEST_THROTTLE_IS_PERCENT = 0
# Zero means test one motor. Copter turns that into a count of one.
MOTOR_TEST_ONE_MOTOR = 0

lastHeartbeatReceivedAtSeconds = None  # no heartbeat has been saved yet


# Saves the time of a heartbeat from this Pixhawk, so a lost link can be noticed.
def rememberHeartbeat(pixhawkConnection, message):
    global lastHeartbeatReceivedAtSeconds  # share this time with the freshness check
    if message.get_srcSystem() != pixhawkConnection.target_system:
        return  # ignore a heartbeat from some other device
    lastHeartbeatReceivedAtSeconds = time.monotonic()  # mark when this heartbeat arrived


# True when a heartbeat arrived within the last 3 seconds.
def heartbeatIsFresh():
    if lastHeartbeatReceivedAtSeconds is None:
        return False  # nothing has arrived yet
    heartbeatAgeSeconds = time.monotonic() - lastHeartbeatReceivedAtSeconds  # age of the last heartbeat
    return heartbeatAgeSeconds <= HEARTBEAT_STALE_AFTER_SECONDS  # still recent enough to trust


# Opens pins 8 and 10 and waits until the Pixhawk answers.
def connectToPixhawk(latestMessages):
    print()
    print(f"Opening {PIXHAWK_SERIAL_PORT} at {PIXHAWK_SERIAL_BAUD} baud.")  # say which port is opening
    pixhawkConnection = mavutil.mavlink_connection(  # open the serial link
        PIXHAWK_SERIAL_PORT,
        baud=PIXHAWK_SERIAL_BAUD,
        source_system=255,  # this computer is a ground station, not another vehicle
        autoreconnect=False,  # do not quietly reopen a dead link
    )
    heartbeat = pixhawkConnection.wait_heartbeat(timeout=CONNECTION_TIMEOUT_SECONDS)  # wait for the Pixhawk to speak
    if heartbeat is None:
        raise RuntimeError("No Pixhawk heartbeat arrived on pin 10.")  # stop if pin 10 stays silent
    if pixhawkConnection.target_component == 0:
        pixhawkConnection.target_component = 1  # talk to the autopilot itself
    processIncomingMessage(pixhawkConnection, latestMessages, heartbeat)  # keep that first heartbeat
    print(  # show which Pixhawk answered
        f"Connected to Pixhawk system {pixhawkConnection.target_system}, "
        f"component {pixhawkConnection.target_component}."
    )
    return pixhawkConnection  # hand the open link back to main


# Stores one Pixhawk message and prints any status text. Ignores a broken byte.
def processIncomingMessage(pixhawkConnection, latestMessages, message):
    if message is None:
        return  # nothing arrived
    if message.get_type() == "BAD_DATA":
        return  # a garbled byte must not replace the last good message
    if message.get_srcSystem() not in (0, pixhawkConnection.target_system):
        return  # ignore a message from another system
    messageType = message.get_type()  # HEARTBEAT, STATUSTEXT, COMMAND_ACK, and so on
    latestMessages[messageType] = message  # keep only the newest of each kind
    if messageType == "HEARTBEAT":
        rememberHeartbeat(pixhawkConnection, message)  # update the link timer
    elif messageType == "STATUSTEXT":
        statusText = getattr(message, "text", "")  # the words the Pixhawk sent
        if isinstance(statusText, bytes):
            statusText = statusText.decode("ascii", "ignore")  # turn those bytes into text
        statusText = str(statusText).strip("\x00").strip()  # drop the blank padding
        if statusText:
            print(f"PIXHAWK: {statusText}")  # show the Pixhawk's own words


# Reads every message already waiting on the serial port.
def readAvailableMessages(pixhawkConnection, latestMessages):
    while True:
        message = pixhawkConnection.recv_match(blocking=False)  # take one message if one is waiting
        if message is None:
            return  # the port is quiet, so stop looking
        processIncomingMessage(pixhawkConnection, latestMessages, message)  # store that message


# True when the motors are armed. None until a heartbeat has been stored.
def copterIsArmed(latestMessages):
    heartbeat = latestMessages.get("HEARTBEAT")  # the newest heartbeat
    if heartbeat is None:
        return None  # arm state is still unknown
    return bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)  # that bit means armed


# Tells the Pixhawk to spin motor 2, or to stop it when the run time is 0.
def sendMotorTest(pixhawkConnection, throttlePercent, runSeconds):
    pixhawkConnection.mav.command_long_send(  # one command with seven number fields
        pixhawkConnection.target_system,  # which vehicle
        pixhawkConnection.target_component,  # which autopilot
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,  # the motor-test command
        0,  # not a confirmation resend
        MOTOR_TEST_SEQUENCE,  # 3, which spins motor 2 on this Quad X
        MOTOR_TEST_THROTTLE_IS_PERCENT,  # the next number is a percent
        throttlePercent,  # how hard to spin
        runSeconds,  # how long to spin; 0 means stop
        MOTOR_TEST_ONE_MOTOR,  # this motor only
        0,  # unused
        0,  # unused
    )


# Waits up to 3 seconds for the Pixhawk to accept or refuse the spin.
def waitForMotorTestAnswer(pixhawkConnection, latestMessages):
    deadlineSeconds = time.monotonic() + MOTOR_TEST_ANSWER_TIMEOUT_SECONDS  # give up after 3 seconds
    while time.monotonic() < deadlineSeconds:
        message = pixhawkConnection.recv_match(blocking=True, timeout=0.4)  # wait briefly for the next message
        if message is None:
            continue  # nothing yet, so keep waiting
        processIncomingMessage(pixhawkConnection, latestMessages, message)  # store a heartbeat or status line
        if message.get_type() != "COMMAND_ACK":
            continue  # this message is not an answer to a command
        if message.command != mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST:
            continue  # this answer belongs to some other command
        return message.result  # accepted, refused, or another result code
    return None  # no answer arrived in time


# Asks for YES, spins motor 2 for 5 seconds, then closes the link.
def main():
    pixhawkConnection = None  # no link yet
    latestMessages = {}  # newest message of each kind
    motorSpinWasAccepted = False  # do not send a stop unless a spin really started

    print()
    print("==============================================")
    print("MOTOR 2 TEST")  # title
    print("==============================================")
    print()
    print("Motor 2 is the back-left motor on this Quad X.")  # which motor to watch
    print("Looking down, it should turn counter-clockwise.")  # the expected direction
    print(f"It will spin at {MOTOR_THROTTLE_PERCENT} percent for {MOTOR_RUN_SECONDS:.0f} seconds.")
    print("Take the propellers off first.")
    print()
    print("Type YES to start.")
    if input("> ").strip() != "YES":
        print("Cancelled.")
        return  # anything except YES leaves the motors alone

    try:
        pixhawkConnection = connectToPixhawk(latestMessages)  # open pins 8 and 10
        readAvailableMessages(pixhawkConnection, latestMessages)  # catch up on messages already waiting
        if not heartbeatIsFresh():
            raise RuntimeError("Pixhawk heartbeat was lost.")  # the link died before the spin
        if copterIsArmed(latestMessages) is not False:
            print("The copter is armed, or arm status is unknown. Motor 2 was not started.")
            return  # do not spin unless the heartbeat says disarmed

        print()
        print(f"Asking motor {MOTOR_NUMBER} to spin.")
        sendMotorTest(pixhawkConnection, MOTOR_THROTTLE_PERCENT, MOTOR_RUN_SECONDS)  # request the 5 second spin
        motorTestAnswer = waitForMotorTestAnswer(pixhawkConnection, latestMessages)  # wait for accept or refuse
        if motorTestAnswer is None:
            print("No answer from the Pixhawk. Motor 2 was not left running.")
            return  # do not wait as if the motor were spinning
        print(f"Motor test answer = {motorTestAnswer}")  # show the Pixhawk's result code
        if motorTestAnswer != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Motor test was refused. Press the safety switch until the LED is solid, then try again.")
            return  # refused, so do not time a spin that did not start
        motorSpinWasAccepted = True  # a stop command is now worth sending

        spinDeadlineSeconds = time.monotonic() + MOTOR_RUN_SECONDS  # when the 5 seconds are over
        while time.monotonic() < spinDeadlineSeconds:
            readAvailableMessages(pixhawkConnection, latestMessages)  # keep printing status text during the spin
            time.sleep(0.1)  # check the port about 10 times a second
        print("Motor 2 spin finished.")

    except KeyboardInterrupt:
        print()
        print("Ctrl+C. Stopping motor 2.")  # the stop itself is sent below

    finally:
        if pixhawkConnection is not None and motorSpinWasAccepted and heartbeatIsFresh():
            sendMotorTest(pixhawkConnection, 0, 0)  # throttle 0 and time 0 tell motor 2 to stop
            print("Motor 2 stop sent.")
        if pixhawkConnection is not None:
            try:
                pixhawkConnection.close()  # release pins 8 and 10
            except Exception:
                pass  # a dead link should not hide the result
            print("Pixhawk connection closed.")
        print("Finished.")


if __name__ == "__main__":
    main()  # run only when this file is started, not when it is imported
