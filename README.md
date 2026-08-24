# 🤖 esp32c3_super_mini_robot-move — b3 on the `30_esp32_c3_move` board

A port of [`esp32c3_super_mini_robot-bit-rxy`](https://github.com/abourdim/esp32c3_super_mini_robot-bit-rxy)
("b3") onto the **`30_esp32_c3_move`** carrier board, plus a panning sonar head
the original does not have. Driven over Bluetooth Low Energy from the
[rxy web app](https://github.com/abourdim/rxy_web) — no install, no account, no
WiFi.

**The robot owns the layout.** The app is a generic renderer: it connects, asks
the robot for its panel, and draws whatever comes back. There is no per-robot
build of the app.

## Faithful to b3

Every module b3 has is still here — buzzer, NeoPixels, battery sense, board
LEDs, OLED, the two NeoPixel demos, all eight panels. Nothing was subtracted.
What changed is the pin map, because the two carrier boards share an MCU and
nothing else.

Three features need **wires**, because the board has no footprint for them.
They are wired in software and will work the moment the hardware is attached;
they simply have no connector to plug into.

| feature | b3 | here | needs |
|---|---|---|---|
| drive servos | GPIO 6 / 3 | **5 / 10** — J5 / J12, level-shifted | — |
| HC-SR04 | 21 / 20 | **6 / 7** — J10, level-shifted | — |
| head servo | *(none)* | **1** — J6 | — |
| green LED | 1 | **0** — J7 | — |
| red LED | 10 | **21** — J8 | — |
| NeoPixel strip | 5 (or 10) | **20** — J9 | — |
| button / OTA | 0 (module BOOT) | **3** — SW1 | — |
| buzzer | 4 | 4 — SW2 pads | 🔧 a buzzer, soldered |
| battery sense | 4 | 4 — SW2 pads | 🔧 a divider, soldered |
| OLED | I²C 8 / 9 | 8 / 9 — module pads | 🔧 4 wires to the module |

GPIO 2, 8 and 9 reach the SuperMini's pads and no connector at all, which is
why the OLED needs wires rather than a plug. b3's own buzzer/battery pin
collision on GPIO 4 is preserved rather than fixed — the Power panel puts a
Buzz button next to the live voltage for exactly that reason.

**The screen is expected here, not optional.** SDA to GPIO 8, SCL to GPIO 9,
and **VCC to 3V3 rather than 5V** — the module's own pull-ups follow whatever
you feed it, and the C3 is not 5V tolerant. Full wiring and the strapping-pin
reasoning are in [`02_hardware/README.md`](02_hardware/README.md#attaching-the-oled).

The firmware degrades gracefully if it is missing — it probes `0x3C` at boot and
re-probes every five seconds, so a bodge wire that seats late or works loose
recovers on its own rather than needing a power cycle. Everything else keeps
running meanwhile. What it will not do is drive a bus with nothing on it, which
is what hung the app at "Checking layout version" the first time out.

Board source and the full netlist-derived pin map: [`02_hardware/`](02_hardware/).

## The 3-pin headers double as FET gates

J6, J7, J8 and J9 carry the same nets as Q1–Q4's gates, which drive the J1–J4
screw terminals and their yellow indicator LEDs. Anything plugged into a 3-pin
header switches its paired screw terminal in sympathy, so **leave J1–J4
unpopulated**. The upside is free activity indicators: D6–D9 light along with
whatever is on the header, which makes the link LEDs visible even with nothing
plugged in.

## Why the button moved off GPIO 0

b3 uses the SuperMini's own BOOT button for the three-second OTA hold, and that
button exists here too. But on this board GPIO 0 is Q2's gate, so the gesture
would switch the J2 output on for the length of the hold — and GPIO 0 is more
useful as the green LED. SW1 is a real button with its own pull-up that drives
nothing else.

## The head, and the radar

The head servo is the one thing here that b3 does not have. It exists to give
the app's **radar** widget a bearing to plot against: the radar carries no
value of its own, it reads the distance gauge and the head slider by id, draws
rings at 10/30/100 cm with a beam on the live angle, and lets detections
persist and fade over five seconds. A sweep builds a picture of the room
instead of flashing one number.

Head angle is **not** persisted. The wheel trims are, because a trim is a
calibration you want back after a power cycle; the head is a live control, and
restoring yesterday's bearing at boot would point the sensor somewhere nobody
asked for. It is echoed to the app on connect instead.

`CONFIG_SERVO_HEAD_MIN` / `_MAX` are the **mechanical** limits of your pan
mount, not 0–180. Narrow them to what the mount clears — a servo told to go
past its stop sits there stalled, drawing full stall current off the same rail
as the C3.

## Panels

All eight of b3's, switched with the **Level** selector; the choice is kept in
NVS. The Distance test panel gains the radar and the head slider.

The six test panels are generated:

```bash
python 01_software/01_app/gen_test_layouts.py
```

Beginner and Expert are **not** — those are arranged in the app and exported,
then spliced in. So the radar is on the Distance panel only; if you want it on
Expert, arrange that panel in the app and export it the usual way.

## Build and flash

```bash
cd 01_software/01_app && pio run -e esp32-c3-devkitm-1 -t upload
```

After the first USB flash, hold **SW1 for 3 seconds** to enter WiFi OTA mode.
Hold to 8 seconds to forget the saved network. The device is reachable as
`wdiy-move`, and the setup AP is `WDIY-Robot-Setup`.

BLE device name is `diy_app_mv`, distinct from b3's `diy_app_b3` so the two are
told apart in the scanner when both are powered on in the same room.

**The image is at 92% of flash.** b3's full feature set on the default
partition table leaves about 100 KB of headroom, which is enough for the
current work but not for another subsystem. Anything substantial from here
wants a bigger app partition.

## Before you build one

Three board facts firmware cannot paper over — the long form is in
[`02_hardware/README.md`](02_hardware/README.md):

- **J1–J4 must stay unpopulated** while anything is on J6–J9. Same nets.
- **J6 has no level shifter.** J5, J12 and J10 run through the TXB0104 and get
  clean 5 V logic; J6 carries the raw 3.3 V pin. Most hobby servos accept that.
- **The servo rail needs 470–1000 µF** at the connectors, fed from the input
  terminal rather than through the SuperMini's 5 V pin. Three drive servos on a
  rail with 100 nF browns the C3 out mid-drive, which reads as a random reboot.

## Known-stale

`01_software/01_app/02_web/` is b3's site and still describes b3's pin map. Its
Pages workflow has been set to manual trigger only, so a push cannot publish a
site that is wrong about this robot. Rewrite it before re-enabling.

Powered by [Workshop-DIY.org](https://workshop-diy.org)
