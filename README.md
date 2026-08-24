# 🤖 esp32c3_super_mini_robot-move — drive servos, a panning sonar head, BLE control

An ESP32-C3 SuperMini robot on the **`30_esp32_c3_move`** carrier board: two
continuous-rotation drive servos, an HC-SR04 on a **servo head that turns**, and
four spare low-side switched outputs. Driven over Bluetooth Low Energy from the
[rxy web app](https://github.com/abourdim/rxy_web) — no install, no account, no
WiFi.

**The robot owns the layout.** The app is a generic renderer: it connects, asks
the robot for its panel, and draws whatever comes back. There is no per-robot
build of the app.

## Lineage

Forked from
[`esp32c3_super_mini_servo_sonar-rxy`](https://github.com/abourdim/esp32c3_super_mini_servo_sonar-rxy)
("S2") with its full history, which was itself forked from
[`esp32c3_super_mini_robot-bit-rxy`](https://github.com/abourdim/esp32c3_super_mini_robot-bit-rxy)
("b3"). Fixes cherry-pick along that chain rather than being hand-copied.

S2 was the right base because it had already subtracted everything this board
cannot feed — buzzer, NeoPixels, battery sense, board LEDs, OLED. What this
board adds back is the head servo, which S2 never had.

| | b3 | S2 | **this board** |
|---|---|---|---|
| drive servos | GPIO 6 / 3 | GPIO 10 / 7 | **GPIO 5 / 10**, level-shifted |
| HC-SR04 | 21 / 20 | 21 / 20 | **6 / 7**, level-shifted |
| head servo | — | — | **GPIO 1** |
| button / OTA | GPIO 0 (module BOOT) | GPIO 0 | **GPIO 3** (SW1 on the board) |
| buzzer | GPIO 4 | — | — |
| battery sense | GPIO 4 | — | — |
| NeoPixel strip | GPIO 5 (or 10) | — | — |
| 2 board LEDs | GPIO 10 / 1 | — | — |
| OLED | I²C 8 / 9 | — | — (pins go nowhere) |

Board source and the full pin map: [`02_hardware/`](02_hardware/).

## Why the button moved off GPIO 0

Both older robots use the SuperMini's own BOOT button for the three-second OTA
hold. On this board GPIO 0 is Q2's gate, so that gesture would switch the J2
screw terminal on for the duration of the hold. SW1 (GPIO 3) is a real button
with its own pull-up that drives nothing else.

## The head, and the radar

The head servo exists to give the app's **radar** widget an angle to plot
against. The radar carries no value of its own — it reads the distance gauge
and the head slider by id, draws rings at 10/30/100 cm with a beam on the live
bearing, and lets detections persist and fade over five seconds. A sweep builds
a picture of the room instead of flashing one number.

Head angle is **not** persisted. The wheel trims are, because a trim is a
calibration you want back after a power cycle; the head is a live control, and
restoring yesterday's bearing at boot would point the sensor somewhere nobody
asked for. It is echoed to the app on connect instead, so the slider and the
radar both start out truthful.

`CONFIG_SERVO_HEAD_MIN` / `_MAX` are the **mechanical** limits of your pan
mount, not 0–180. Narrow them to what the mount actually clears — a servo told
to go past its stop sits there stalled, drawing full stall current off the same
rail as the C3.

## Panels

Four, switched with the **Level** selector; the choice is kept in NVS, so a
power cycle does not drop you back to Beginner.

| panel | what it has |
|---|---|
| **Beginner** | D-pad, distance gauge, obstacle alert |
| **Expert** | D-pad + joystick + speed + STOP + trim, distance gauge/graph/alert, **radar + head**, firmware, uptime, signal, button, telemetry level |
| **Drive** | D-pad, speed, STOP, speed gauge, trim — one subsystem, for bring-up |
| **Distance** | gauge, alert, graph, **radar + head** |

Layouts are generated, not hand-edited:

```bash
python 01_software/01_app/gen_layouts.py
```

It rebuilds all four blobs, splices them into `03_bit-rxy.cpp`, and **fails**
rather than emitting a panel with overlapping zones, a widget escaping its
group, or a widget id that has neither a handler nor a telemetry sender. The
radar is exempt from that last check by way of a `DERIVED` set: it is fully
driven while never being a `SET` or `UPD` target, which is a different thing
from being decoration.

## Build and flash

```bash
cd 01_software/01_app && pio run -e esp32-c3-devkitm-1 -t upload
```

After the first USB flash, hold **SW1 for 3 seconds** to enter WiFi OTA mode.
This board has no NeoPixels and no screen to report OTA progress on, so status
goes to Serial only — which is enough, because the setup AP name is fixed
(`WDIY-Robot-Setup`) and the device is reachable as `wdiy-move`. Hold to 8
seconds to forget the saved network.

BLE device name is `diy_app_mv`, distinct from b3's `diy_app_b3` and S2's
`diy_app_s2` so the three are told apart in the scanner when more than one is
powered on in the same room.

## Before you build one

Three board facts that firmware cannot paper over — the long form is in
[`02_hardware/README.md`](02_hardware/README.md):

- **J1 and J6 share GPIO 1.** A head servo pulses the J1 output at 50 Hz and
  flickers its indicator LED; a load on J1 makes the head twitch. Populate one
  or the other, never both.
- **J6 has no level shifter.** J5, J12 and J10 run through the TXB0104 and get
  clean 5 V logic. J6 carries the raw 3.3 V pin.
- **The servo rail needs 470–1000 µF** at the connectors, fed from the input
  terminal rather than through the SuperMini's 5 V pin. Three drive servos on a
  rail with 100 nF browns the C3 out mid-drive, which reads as a random reboot.

## Known-stale

`01_software/01_app/02_web/` is inherited from b3 and describes **b3's**
hardware — LEDs, strip, OLED, battery — none of which is on this board. Its
Pages workflow has been set to manual trigger only, so a push cannot publish a
site that is wrong about this robot. Rewrite it before re-enabling.

Powered by [Workshop-DIY.org](https://workshop-diy.org)
