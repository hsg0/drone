#!/home/drone/dronekit-venv/bin/python3
"""
WHAT:
    Let the RC transmitter fly the copter.

    This script connects to the Pixhawk, checks that the transmitter
    channels are arriving, and releases any stick override so the radio
    is the only control source.

    The pilot arms with the transmitter and hovers with the sticks.
    The script only prints the mode, arm state, and stick positions.

WHY:
    The other hover scripts send takeoff, attitude, and landing commands.
    Those commands are not used here. A manual hover has to come from the
    transmitter flight-mode switch and the sticks.

HOW:
    1. Connect on pins 8 and 10. Leave USB with the screen.
    2. Request heartbeat, RC channels, radio status, and status text.
    3. Require fresh roll, pitch, throttle, and yaw channels.
    4. Send a channel release so an old override cannot hold the sticks.
    5. Type YES to begin the watch.
    6. Print once per second until Ctrl+C.
    7. Close the link. Do not change the flight mode and do not disarm.

IMPORTANT:
    This script does not arm the copter.
    Arm with the transmitter.
    Select the hover mode on the transmitter. ALT_HOLD and STABILIZE
    give the sticks direct control. LOITER still uses the sticks, and
    it also holds position when GPS is healthy.
    GUIDED, GUIDED_NOGPS, AUTO, RTL, and LAND do not fly from the sticks.
    Move the flight-mode switch if the printed mode is one of those.
    This script does not command motors, attitude, climb rate, or mode.
    Ctrl+C only closes the Pi link. The transmitter still has the copter.
"""

import time

from pymavlink import mavutil


# Pin 8 transmits. Pin 10 receives. Pin 6 is ground.
PIXHAWK_SERIAL_PORT = "/dev/serial0"
PIXHAWK_SERIAL_BAUD = 57600

CONNECTION_TIMEOUT_SECONDS = 15
INITIAL_TELEMETRY_COLLECTION_SECONDS = 5
HEARTBEAT_STALE_AFTER_SECONDS = 3.0
RC_STALE_AFTER_SECONDS = 1.0
MESSAGE_INTERVAL_MICROSECONDS = 100_000
STATUS_PRINT_INTERVAL_SECONDS = 1.0

# A value of 0 in RC_CHANNELS_OVERRIDE releases that channel back to the radio.
# 65535 would leave an existing override untouched.
RC_CHANNEL_RELEASE_VALUE = 0

# Normal transmitter pulses sit around 1000 to 2000 microseconds.
MINIMUM_RC_PULSE_MICROSECONDS = 800
MAXIMUM_RC_PULSE_MICROSECONDS = 2200

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

# Modes in which the transmitter sticks fly the copter.
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


def connectToPixhawk():
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
    rememberHeartbeat(pixhawkConnection, heartbeat)
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


def releaseRcOverride(pixhawkConnection):
    """
    Give every overridden channel back to the transmitter.

    Zero means release. This does not command a stick position.
    """
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
        return "sticks are flying"
    return "move the flight mode switch to ALT_HOLD or STABILIZE"


def radioSignalText(latestMessages, messageArrivedAtSeconds):
    if not messageIsFresh(messageArrivedAtSeconds, "RADIO_STATUS", RC_STALE_AFTER_SECONDS):
        return "radio status unavailable"
    radioStatus = latestMessages.get("RADIO_STATUS")
    if radioStatus is None:
        return "radio status unavailable"
    remoteRssi = getattr(radioStatus, "remrssi", None)
    if remoteRssi is None:
        return "radio status unavailable"
    if int(remoteRssi) <= 0:
        return "transmitter signal is gone"
    return f"radio rssi {int(remoteRssi)}"


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

    print(
        f"{armedText} | {modeName} | {stickControlText(modeNumber)} | "
        f"{stickText} | {radioSignalText(latestMessages, messageArrivedAtSeconds)}"
    )


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
        if transmitterSticksArePresent(latestMessages, messageArrivedAtSeconds):
            print()
            print("Transmitter channels are arriving.")
            return
        time.sleep(0.1)
    raise RuntimeError("Roll, pitch, throttle, and yaw did not arrive from the transmitter.")


def watchRcHover(pixhawkConnection, latestMessages, messageArrivedAtSeconds):
    print()
    print("==============================================")
    print("RC HOVER")
    print("==============================================")
    print()
    print("Arm and fly with the transmitter.")
    print("This window only shows what the copter is doing.")
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

    try:
        print()
        print("==============================================")
        print("RC ARM AND HOVER")
        print("==============================================")
        print()
        print("The transmitter arms and flies.")
        print("This script does not send stick or attitude commands.")
        print()

        pixhawkConnection = connectToPixhawk()
        requestRequiredTelemetry(pixhawkConnection)
        waitForTransmitter(pixhawkConnection, latestMessages, messageArrivedAtSeconds)
        releaseRcOverride(pixhawkConnection)

        modeNumber, modeName = currentModeName(latestMessages)
        print(f"Current mode: {modeName}.")
        print(stickControlText(modeNumber) + ".")
        print()
        print("Type YES to watch the RC hover.")
        if input("> ").strip() != "YES":
            print("Cancelled.")
            return

        watchRcHover(pixhawkConnection, latestMessages, messageArrivedAtSeconds)

    except KeyboardInterrupt:
        print()
        print("Ctrl+C.")
        print("Closing the link. The transmitter still has the copter.")
        print("The flight mode was not changed, and the copter was not disarmed.")

    except Exception as error:
        print()
        print(f"STOPPED: {error}")

    finally:
        if pixhawkConnection is not None:
            try:
                pixhawkConnection.close()
            except Exception:
                pass
            print("Pixhawk connection closed.")
        print("Finished.")


if __name__ == "__main__":
    main()
