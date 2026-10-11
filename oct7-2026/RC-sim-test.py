#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    Arm the Pixhawk from this script, then let the transmitter throttle
    fly the copter.

    The script checks the radio, releases any stick override, and sends
    one arm command. After that it only prints the arm state, mode, and
    stick positions.

WHY:
    This is a ground-to-hover check that the arm command and the
    transmitter throttle work together. The sticks stay in control, so
    the pilot raises and lowers the throttle.

HOW:
    1. Connect on pins 8 and 10. Leave USB with the screen.
    2. Request heartbeat, RC channels, radio status, and status text.
    3. Require fresh roll, pitch, throttle, and yaw channels.
    4. Require a mode in which the sticks fly the copter.
    5. Release any stick override.
    6. Type YES. The throttle stick must be down.
    7. Send the arm command and wait until the Pixhawk reports armed.
    8. Print once per second. Raise the throttle on the transmitter.
    9. Ctrl+C disarms the Pixhawk so the motors stop, then closes the link.

IMPORTANT:
    Throttle down before typing YES. In STABILIZE the throttle stick is
    the throttle: down is idle, and raising it adds thrust.
    This copter's flight-mode switch is STABILIZE in every position, so
    the sticks are the controls.
    GUIDED, GUIDED_NOGPS, AUTO, RTL, and LAND ignore the throttle stick.
    The script will not arm in those modes.
    Ctrl+C turns the motors off. A normal disarm is sent first. If the
    Pixhawk refuses that while the copter is still flying, the script
    sends the force-disarm value so the motors still stop.
    The transmitter can also disarm: throttle down and yaw left.
"""

import time

from pymavlink import mavutil


# Pin 8 transmits. Pin 10 receives. Pin 6 is ground.
PIXHAWK_SERIAL_PORT = "/dev/serial0"
PIXHAWK_SERIAL_BAUD = 57600

CONNECTION_TIMEOUT_SECONDS = 15
INITIAL_TELEMETRY_COLLECTION_SECONDS = 5
ARM_CONFIRMATION_TIMEOUT_SECONDS = 10
DISARM_CONFIRMATION_TIMEOUT_SECONDS = 5
THROTTLE_DOWN_WAIT_SECONDS = 20

# ArduPilot can refuse a normal disarm while it still treats the copter as flying.
# This is the value it requires before it will stop the motors anyway.
FORCE_DISARM_MAGIC_NUMBER = 21196
HEARTBEAT_STALE_AFTER_SECONDS = 3.0
RC_STALE_AFTER_SECONDS = 1.0
MESSAGE_INTERVAL_MICROSECONDS = 100_000
STATUS_PRINT_INTERVAL_SECONDS = 1.0

# A value of 0 in RC_CHANNELS_OVERRIDE releases that channel back to the radio.
RC_CHANNEL_RELEASE_VALUE = 0

# Normal transmitter pulses sit around 1000 to 2000 microseconds.
MINIMUM_RC_PULSE_MICROSECONDS = 800
MAXIMUM_RC_PULSE_MICROSECONDS = 2200

# The arm command is sent only while the throttle stick is still at the bottom.
THROTTLE_IS_DOWN_BELOW_MICROSECONDS = 1300

ROLL_CHANNEL_NUMBER = 1
PITCH_CHANNEL_NUMBER = 2
THROTTLE_CHANNEL_NUMBER = 3
YAW_CHANNEL_NUMBER = 4

REQUIRED_MESSAGE_NAMES = (
    "HEARTBEAT",
    "RC_CHANNELS",
    "RC_CHANNELS_RAW",
    "RADIO_STATUS",
    "STATUSTEXT",
)

# Modes in which the transmitter throttle is the throttle.
RC_STICK_MODE_NUMBERS = (
    0,   # STABILIZE
    1,   # ACRO
    2,   # ALT_HOLD
    5,   # LOITER
    11,  # DRIFT
    13,  # SPORT
    16,  # POSHOLD
    22,  # FLOWHOLD
)

COPTER_MODE_NAMES = {
    0: "STABILIZE",
    1: "ACRO",
    2: "ALT_HOLD",
    3: "AUTO",
    4: "GUIDED",
    5: "LOITER",
    6: "RTL",
    7: "CIRCLE",
    9: "LAND",
    11: "DRIFT",
    13: "SPORT",
    16: "POSHOLD",
    20: "GUIDED_NOGPS",
    21: "SMART_RTL",
    22: "FLOWHOLD",
}

lastHeartbeatReceivedAtSeconds = None


def cleanStatusText(message):
    statusText = getattr(message, "text", "")
    if isinstance(statusText, bytes):
        statusText = statusText.decode("ascii", "ignore")
    return str(statusText).strip("\x00").strip()


def rememberHeartbeat(pixhawkConnection, message):
    global lastHeartbeatReceivedAtSeconds
    if message.get_srcSystem() != pixhawkConnection.target_system:
        return
    lastHeartbeatReceivedAtSeconds = time.monotonic()


def heartbeatIsFresh():
    if lastHeartbeatReceivedAtSeconds is None:
        return False
    heartbeatAgeSeconds = time.monotonic() - lastHeartbeatReceivedAtSeconds
    return heartbeatAgeSeconds <= HEARTBEAT_STALE_AFTER_SECONDS


def connectToPixhawk(latestMessages, messageArrivedAtSeconds):
    print()
    print(f"Opening {PIXHAWK_SERIAL_PORT} at {PIXHAWK_SERIAL_BAUD} baud.")
    pixhawkConnection = mavutil.mavlink_connection(
        PIXHAWK_SERIAL_PORT,
        baud=PIXHAWK_SERIAL_BAUD,
        source_system=255,
        autoreconnect=False,
    )
    heartbeat = pixhawkConnection.wait_heartbeat(timeout=CONNECTION_TIMEOUT_SECONDS)
    if heartbeat is None:
        raise RuntimeError("No Pixhawk heartbeat arrived on pin 10.")
    if pixhawkConnection.target_component == 0:
        pixhawkConnection.target_component = 1
    # wait_heartbeat consumes this message. Keep it, or the mode check can
    # run before the next heartbeat arrives.
    processIncomingMessage(
        pixhawkConnection,
        latestMessages,
        messageArrivedAtSeconds,
        heartbeat,
    )
    print(
        f"Connected to Pixhawk system {pixhawkConnection.target_system}, "
        f"component {pixhawkConnection.target_component}."
    )
    return pixhawkConnection


def requestRequiredTelemetry(pixhawkConnection):
    for messageName in REQUIRED_MESSAGE_NAMES:
        messageId = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{messageName}", None)
        if messageId is None:
            print(f"MAVLink message id for {messageName} was not found.")
            continue
        pixhawkConnection.mav.command_long_send(
            pixhawkConnection.target_system,
            pixhawkConnection.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            messageId,
            MESSAGE_INTERVAL_MICROSECONDS,
            0,
            0,
            0,
            0,
            0,
        )
    print("Requested RC telemetry.")


def processIncomingMessage(pixhawkConnection, latestMessages, messageArrivedAtSeconds, message):
    if message is None:
        return
    if message.get_srcSystem() not in (0, pixhawkConnection.target_system):
        return
    messageType = message.get_type()
    latestMessages[messageType] = message
    messageArrivedAtSeconds[messageType] = time.monotonic()
    if messageType == "HEARTBEAT":
        rememberHeartbeat(pixhawkConnection, message)
    elif messageType == "STATUSTEXT":
        statusText = cleanStatusText(message)
        if statusText:
            print(f"PIXHAWK: {statusText}")


def readAvailableMessages(pixhawkConnection, latestMessages, messageArrivedAtSeconds):
    while True:
        message = pixhawkConnection.recv_match(blocking=False)
        if message is None:
            return
        processIncomingMessage(
            pixhawkConnection,
            latestMessages,
            messageArrivedAtSeconds,
            message,
        )


def messageIsFresh(messageArrivedAtSeconds, messageType, maximumAgeSeconds):
    arrivedAtSeconds = messageArrivedAtSeconds.get(messageType)
    if arrivedAtSeconds is None:
        return False
    return time.monotonic() - arrivedAtSeconds <= maximumAgeSeconds


def copterIsArmed(latestMessages):
    heartbeat = latestMessages.get("HEARTBEAT")
    if heartbeat is None:
        return None
    return bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def currentModeName(latestMessages):
    heartbeat = latestMessages.get("HEARTBEAT")
    if heartbeat is None:
        return None, "no heartbeat"
    modeNumber = int(heartbeat.custom_mode)
    modeName = COPTER_MODE_NAMES.get(modeNumber, f"mode {modeNumber}")
    return modeNumber, modeName


def latestRcMessage(latestMessages, messageArrivedAtSeconds):
    """Prefer the full RC_CHANNELS message. Fall back to the older 8-channel message."""
    if messageIsFresh(messageArrivedAtSeconds, "RC_CHANNELS", RC_STALE_AFTER_SECONDS):
        return latestMessages.get("RC_CHANNELS")
    if messageIsFresh(messageArrivedAtSeconds, "RC_CHANNELS_RAW", RC_STALE_AFTER_SECONDS):
        return latestMessages.get("RC_CHANNELS_RAW")
    return None


def rcChannelMicroseconds(rcMessage, channelNumber):
    if rcMessage is None:
        return None
    channelValue = getattr(rcMessage, f"chan{channelNumber}_raw", None)
    if not isinstance(channelValue, (int, float)):
        return None
    channelMicroseconds = int(channelValue)
    if not (MINIMUM_RC_PULSE_MICROSECONDS <= channelMicroseconds <= MAXIMUM_RC_PULSE_MICROSECONDS):
        return None
    return channelMicroseconds


def transmitterSticksArePresent(latestMessages, messageArrivedAtSeconds):
    rcMessage = latestRcMessage(latestMessages, messageArrivedAtSeconds)
    if rcMessage is None:
        return False
    for channelNumber in (
        ROLL_CHANNEL_NUMBER,
        PITCH_CHANNEL_NUMBER,
        THROTTLE_CHANNEL_NUMBER,
        YAW_CHANNEL_NUMBER,
    ):
        if rcChannelMicroseconds(rcMessage, channelNumber) is None:
            return False
    return True


def throttleIsDown(latestMessages, messageArrivedAtSeconds):
    rcMessage = latestRcMessage(latestMessages, messageArrivedAtSeconds)
    throttleMicroseconds = rcChannelMicroseconds(rcMessage, THROTTLE_CHANNEL_NUMBER)
    if throttleMicroseconds is None:
        return False
    return throttleMicroseconds < THROTTLE_IS_DOWN_BELOW_MICROSECONDS


def releaseRcOverride(pixhawkConnection):
    """Give every overridden channel back to the transmitter."""
    pixhawkConnection.mav.rc_channels_override_send(
        pixhawkConnection.target_system,
        pixhawkConnection.target_component,
        RC_CHANNEL_RELEASE_VALUE,
        RC_CHANNEL_RELEASE_VALUE,
        RC_CHANNEL_RELEASE_VALUE,
        RC_CHANNEL_RELEASE_VALUE,
        RC_CHANNEL_RELEASE_VALUE,
        RC_CHANNEL_RELEASE_VALUE,
        RC_CHANNEL_RELEASE_VALUE,
        RC_CHANNEL_RELEASE_VALUE,
    )
    print("Released stick override. The transmitter is the stick source.")


def stickControlText(modeNumber):
    if modeNumber in RC_STICK_MODE_NUMBERS:
        return "transmitter throttle is the throttle"
    return "move the flight mode switch to STABILIZE before arming"


def printRcStatus(latestMessages, messageArrivedAtSeconds):
    armed = copterIsArmed(latestMessages)
    if armed is True:
        armedText = "ARMED"
    elif armed is False:
        armedText = "DISARMED"
    else:
        armedText = "arm state unknown"

    modeNumber, modeName = currentModeName(latestMessages)
    rcMessage = latestRcMessage(latestMessages, messageArrivedAtSeconds)
    rollMicroseconds = rcChannelMicroseconds(rcMessage, ROLL_CHANNEL_NUMBER)
    pitchMicroseconds = rcChannelMicroseconds(rcMessage, PITCH_CHANNEL_NUMBER)
    throttleMicroseconds = rcChannelMicroseconds(rcMessage, THROTTLE_CHANNEL_NUMBER)
    yawMicroseconds = rcChannelMicroseconds(rcMessage, YAW_CHANNEL_NUMBER)

    if rcMessage is None:
        stickText = "RC channels are stale"
    else:
        stickText = (
            f"roll {rollMicroseconds} "
            f"pitch {pitchMicroseconds} "
            f"throttle {throttleMicroseconds} "
            f"yaw {yawMicroseconds}"
        )

    print(f"{armedText} | {modeName} | {stickControlText(modeNumber)} | {stickText}")


def waitForTransmitter(pixhawkConnection, latestMessages, messageArrivedAtSeconds):
    print()
    print(f"Listening for the transmitter for {INITIAL_TELEMETRY_COLLECTION_SECONDS:.0f} seconds.")
    deadlineSeconds = time.monotonic() + INITIAL_TELEMETRY_COLLECTION_SECONDS
    lastPrintedAtSeconds = 0.0
    while time.monotonic() < deadlineSeconds:
        readAvailableMessages(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
        if not heartbeatIsFresh():
            raise RuntimeError("Pixhawk heartbeat was lost.")
        currentTimeSeconds = time.monotonic()
        if currentTimeSeconds - lastPrintedAtSeconds >= STATUS_PRINT_INTERVAL_SECONDS:
            printRcStatus(latestMessages, messageArrivedAtSeconds)
            lastPrintedAtSeconds = currentTimeSeconds
        if (
            transmitterSticksArePresent(latestMessages, messageArrivedAtSeconds)
            and latestMessages.get("HEARTBEAT") is not None
        ):
            print()
            print("Transmitter channels are arriving.")
            return
        time.sleep(0.1)
    raise RuntimeError("Roll, pitch, throttle, and yaw did not arrive from the transmitter.")


def waitUntilThrottleIsDown(pixhawkConnection, latestMessages, messageArrivedAtSeconds):
    print()
    print("Hold the throttle stick down.")
    deadlineSeconds = time.monotonic() + THROTTLE_DOWN_WAIT_SECONDS
    lastPrintedAtSeconds = 0.0
    while time.monotonic() < deadlineSeconds:
        readAvailableMessages(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
        if not heartbeatIsFresh():
            raise RuntimeError("Pixhawk heartbeat was lost.")
        if not transmitterSticksArePresent(latestMessages, messageArrivedAtSeconds):
            raise RuntimeError("Transmitter channels were lost before arming.")
        if throttleIsDown(latestMessages, messageArrivedAtSeconds):
            print("Throttle stick is down.")
            return
        currentTimeSeconds = time.monotonic()
        if currentTimeSeconds - lastPrintedAtSeconds >= STATUS_PRINT_INTERVAL_SECONDS:
            printRcStatus(latestMessages, messageArrivedAtSeconds)
            lastPrintedAtSeconds = currentTimeSeconds
        time.sleep(0.1)
    raise RuntimeError("The throttle stick was not down, so the arm command was not sent.")


def armAndWaitForConfirmation(pixhawkConnection, latestMessages, messageArrivedAtSeconds):
    print()
    print("Sending ARM command.")
    pixhawkConnection.mav.command_long_send(
        pixhawkConnection.target_system,
        pixhawkConnection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    deadlineSeconds = time.monotonic() + ARM_CONFIRMATION_TIMEOUT_SECONDS
    armCommandResult = None
    while time.monotonic() < deadlineSeconds:
        message = pixhawkConnection.recv_match(blocking=True, timeout=0.5)
        if message is None:
            continue
        processIncomingMessage(
            pixhawkConnection,
            latestMessages,
            messageArrivedAtSeconds,
            message,
        )
        if copterIsArmed(latestMessages) is True:
            print("ARMED CONFIRMED.")
            print("Raise the throttle on the transmitter.")
            return True
        if message.get_type() != "COMMAND_ACK":
            continue
        if message.command != mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            continue
        armCommandResult = message.result
        print(f"ARM answer = {armCommandResult}")
        if armCommandResult not in (
            mavutil.mavlink.MAV_RESULT_ACCEPTED,
            mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
        ):
            print(f"ARM FAILED. Answer = {armCommandResult}")
            return False
    print(f"ARM FAILED. Answer = {armCommandResult}")
    return False


def sendDisarmCommand(pixhawkConnection, forceDisarm):
    if forceDisarm:
        disarmMagicNumber = FORCE_DISARM_MAGIC_NUMBER
    else:
        disarmMagicNumber = 0
    pixhawkConnection.mav.command_long_send(
        pixhawkConnection.target_system,
        pixhawkConnection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,
        disarmMagicNumber,
        0,
        0,
        0,
        0,
        0,
    )


def waitUntilMotorsAreOff(pixhawkConnection, latestMessages, messageArrivedAtSeconds):
    deadlineSeconds = time.monotonic() + DISARM_CONFIRMATION_TIMEOUT_SECONDS
    while time.monotonic() < deadlineSeconds:
        try:
            message = pixhawkConnection.recv_match(blocking=True, timeout=0.5)
        except KeyboardInterrupt:
            return False
        if message is None:
            continue
        processIncomingMessage(
            pixhawkConnection,
            latestMessages,
            messageArrivedAtSeconds,
            message,
        )
        if copterIsArmed(latestMessages) is False:
            return True
    return False


def turnMotorsOff(pixhawkConnection, latestMessages, messageArrivedAtSeconds):
    """Stop the motors. Force the disarm only if a normal disarm does not take."""
    if copterIsArmed(latestMessages) is False:
        print("Motors are already off.")
        return

    print("Turning the motors off.")
    sendDisarmCommand(pixhawkConnection, forceDisarm=False)
    if waitUntilMotorsAreOff(pixhawkConnection, latestMessages, messageArrivedAtSeconds):
        print("Motors off.")
        return

    print("Normal disarm was refused. Forcing the motors off.")
    sendDisarmCommand(pixhawkConnection, forceDisarm=True)
    if waitUntilMotorsAreOff(pixhawkConnection, latestMessages, messageArrivedAtSeconds):
        print("Motors off.")
        return

    print("Could not confirm that the motors turned off.")


def watchTransmitterThrottle(pixhawkConnection, latestMessages, messageArrivedAtSeconds):
    print()
    print("==============================================")
    print("TRANSMITTER THROTTLE")
    print("==============================================")
    print()
    print("Raise the throttle on the transmitter.")
    print("Ctrl+C turns the motors off.")
    print()

    lastPrintedAtSeconds = 0.0
    while True:
        readAvailableMessages(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
        if not heartbeatIsFresh():
            raise RuntimeError("Pixhawk heartbeat was lost.")
        currentTimeSeconds = time.monotonic()
        if currentTimeSeconds - lastPrintedAtSeconds >= STATUS_PRINT_INTERVAL_SECONDS:
            printRcStatus(latestMessages, messageArrivedAtSeconds)
            lastPrintedAtSeconds = currentTimeSeconds
        time.sleep(0.1)


def main():
    pixhawkConnection = None
    latestMessages = {}
    messageArrivedAtSeconds = {}
    aircraftWasArmed = False

    try:
        print()
        print("==============================================")
        print("RC SIM TEST")
        print("==============================================")
        print()
        print("This script arms the Pixhawk.")
        print("The transmitter throttle flies the copter after that.")
        print("Ctrl+C turns the motors off.")
        print()

        pixhawkConnection = connectToPixhawk(latestMessages, messageArrivedAtSeconds)
        requestRequiredTelemetry(pixhawkConnection)
        waitForTransmitter(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
        releaseRcOverride(pixhawkConnection)

        modeNumber, modeName = currentModeName(latestMessages)
        print(f"Current mode: {modeName}.")
        if modeNumber is None:
            raise RuntimeError("The flight mode has not arrived yet.")
        if modeNumber not in RC_STICK_MODE_NUMBERS:
            raise RuntimeError(
                f"{modeName} does not give the throttle stick control. "
                "Move the flight mode switch to STABILIZE."
            )
        print("The transmitter throttle will be the throttle after arming.")
        print()
        print("Throttle down, then type YES to arm.")
        if input("> ").strip() != "YES":
            print("Cancelled.")
            return

        if copterIsArmed(latestMessages) is True:
            aircraftWasArmed = True
            print("Already armed. Use the transmitter throttle.")
        else:
            waitUntilThrottleIsDown(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
            if not armAndWaitForConfirmation(
                pixhawkConnection,
                latestMessages,
                messageArrivedAtSeconds,
            ):
                return
            aircraftWasArmed = True

        watchTransmitterThrottle(pixhawkConnection, latestMessages, messageArrivedAtSeconds)

    except KeyboardInterrupt:
        print()
        print("Ctrl+C.")

    except Exception as error:
        print()
        print(f"STOPPED: {error}")

    finally:
        if pixhawkConnection is not None and aircraftWasArmed and copterIsArmed(latestMessages) is not False:
            try:
                turnMotorsOff(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
            except Exception as disarmError:
                print(f"Could not turn the motors off: {disarmError}")
        if pixhawkConnection is not None:
            try:
                pixhawkConnection.close()
            except Exception:
                pass
            print("Pixhawk connection closed.")
        print("Finished.")


if __name__ == "__main__":
    main()
