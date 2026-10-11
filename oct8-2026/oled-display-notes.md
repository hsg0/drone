# OLED display: what broke, and what fixed it

Written 8 Oct 2026 after the screen came back. Do not treat this as a change request. The screen in the photo is the working one.

## What the photo shows

```
DOING STAB off
SON 92.0cm
LAT 49.17749
LON -122.88793
R-0.4 P-1.8
YAW -53.5
BARO 1001
BAT 11.1 58%
```

That is a live link. `STAB off` is STABILIZE and disarmed. Sonic is the RedBoard's own forward distance. Latitude, longitude, roll, pitch, yaw, and barometer came from the Pixhawk through the Pi. `BAT 11.1 58%` is the 3-cell voltage scale, not the Pixhawk's 99% counter.

Each line is 16 characters. Longer text is cut off.

## Two programs, one USB cable

The panel is a 128x64 SSD1306 on the RedBoard (RP2350, MicroPython `main.py`), I2C address 0x3D.

`bootDisplay.py` on the Pi is `boot-display.service`. It is the only program that may open the USB Pixhawk and the USB RedBoard. Flight scripts stay on GPIO UART `/dev/serial0` and must not open those USB ports.

The Pi sends one JSON line:

```
{"display": {"doing": "STAB off", "latitude": "49.17749", ...}}
```

The RedBoard sends sonic readings back the other way as JSON lines. `bootDisplay.py` saves those to `/tmp/redboard-forward-distance.json`.

## How to read a bad screen

`PI waiting` means the RedBoard is running, but it has not parsed a complete display line for 10 seconds (`STATUS_STALE_MS`). Sonic can still show a number, because that sensor is on the RedBoard. Latitude and the lines under it stay blank on purpose.

`DOING waiting` means the `doing` field never arrived.

`SONIC n/a` in the Pi journal, while the panel still shows a sonic distance, means the Pi has the serial port open but is not receiving the board. Writes can still succeed. Restart `boot-display` after the board is already running. Do not reflash first.

`piLinked: true` inside `/tmp/redboard-forward-distance.json` means the board accepted a display line.

## What actually broke it

The Pi called `send_display()` on every pass of a loop that sleeps 0.05 seconds. That is about 20 status lines a second, each about 245 bytes.

The RedBoard cannot read USB while it is taking the sonic samples. That gap is a few hundred milliseconds. The USB receive buffer overflowed, JSON lines arrived broken, `json.loads` rejected them, and after 10 seconds the panel showed `PI waiting`.

A second trap is in MicroPython on this USB port. `sys.stdin.read(1)` can return a whole packet, including the newline. Code that treats that return value as one character never sees the newline, fills a 512-character buffer, and throws the line away.

## What fixed it

1. Send the display line once per second, in the same place as the once-per-second journal print. Leave `board.poll()` in the fast loop so sonic readings are still read.
2. In `pumpIncomingStatus`, walk every character that `read(1)` returns, and end the line on `\n` or `\r`.

After that, the journal showed `SONIC 92.0 cm`, the snapshot said `piLinked: true`, and the panel matched the photo.

## Battery percent

The Pixhawk `battery_remaining` value is milliamp-hours used since boot. A rested pack at 11.4 V still reads about 99%. Do not show that number.

The pack is 3 cells. 9.0 V is 0%. 12.6 V is 100%.

```
percent = (shown volts - 9.0) / 3.6 * 100
```

The screen rounds the voltage to one decimal first, then computes the percent from that same number, so the two figures agree:

- 12.6 V is 100%
- 11.4 V is 67%
- 11.1 V is 58%
- 10.5 V is 42%
- 9.9 V is 25%
- 9.0 V is 0%

The photo's `BAT 11.1 58%` is that scale at 11.1 V.

## What made an earlier fix worse

The panel was already drawing, and then several extra edits were uploaded together: a different USB reader, a different boot order, skipped time-of-flight startup, a draw during setup, and satellites on the DOING line. The screen went back to `DOING waiting` and dashes.

When the panel is blank or stuck, change one thing. The two changes above are the ones that belonged. Do not retune boot order, the time-of-flight startup, or the sonic timing to fix a stuck status line.

## If the RedBoard code has to be uploaded again

1. `systemctl stop boot-display.service` first. The service owns the USB port.
2. Write `main.py`, compile it on the board, then `machine.reset()`.
3. Wait until the board is up. Then `systemctl start boot-display.service`.
4. Open the port with DTR and RTS held false. A normal open can interrupt MicroPython.
5. Check the journal for a sonic distance, and check `piLinked` in the snapshot, before deciding the panel failed.
