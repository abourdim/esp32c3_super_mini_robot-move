# 🤖 esp32c3_super_mini_servo_sonar-rxy — servos + sonar, BLE control

An ESP32-C3 SuperMini robot with **two continuous-rotation drive servos and an
HC-SR04 distance sensor**. Nothing else. Driven over Bluetooth Low Energy from
the [rxy web app](https://github.com/abourdim/rxy_web) — no install, no
account, no WiFi.

**The robot owns the layout.** The app is a generic renderer: it connects, asks
the robot for its panel, and draws whatever comes back. There is no per-robot
build of the app.

## Relationship to b3

This is a deliberately reduced version of
[`esp32c3_super_mini_robot-bit-rxy`](https://github.com/abourdim/esp32c3_super_mini_robot-bit-rxy)
("b3"), forked with its full history so fixes can be cherry-picked between the
two rather than hand-copied.

| | b3 | this board |
|---|---|---|
| MCU | ESP32-C3 SuperMini | same |
| drive | 2 continuous-rotation servos, GPIO 6 / 3 | 2 servos, **GPIO 10 / 7** |
| HC-SR04 | TRIG 21 / ECHO 20 | **identical** |
| buzzer | GPIO 4 | — |
| battery sense | GPIO 4 | — |
| NeoPixel strip | GPIO 5 (or 10) | — |
| 2 board LEDs | GPIO 10 / 1 | — |
| OLED | I²C on 8 / 9 | — |
| BOOT button | GPIO 0 (on the module) | **same** — it is what opens OTA |

GPIO 10 is available for a servo here *only because* the LEDs and the external
strip are gone — on b3 that pin is `CONFIG_PIN_LED_RED` and the strip output.

Schematic: `01_kicad/31_richa_c3_move/34_richa_c3_servo_sonar/v1`.

## Panels

b3 serves eight panels. Five of them drive hardware that is not on this board,
so four remain. Switch between them with the **Level** selector; the choice is
kept in NVS, so a power cycle does not drop you back to Beginner.

| panel | what it has |
|---|---|
| **Beginner** | D-pad, distance gauge, obstacle alert |
| **Expert** | D-pad + joystick + speed + STOP, distance gauge/graph/alert, firmware, uptime, signal, button, telemetry level |
| **Drive** | D-pad, speed, STOP, speed gauge — one subsystem, for bring-up |
| **Distance** | gauge, alert, graph |

Layouts are generated, not hand-edited:

```bash
python 01_software/01_app/gen_layouts.py
```

It rebuilds all four blobs, splices them into `03_bit-rxy.cpp`, and **fails**
rather than emitting a panel with overlapping zones, a widget escaping its
group, or a widget id that has neither a handler nor a telemetry sender. That
last check is the important one: a control that looks live and does nothing is
worse than a missing control.

## Build and flash

```bash
cd 01_software/01_app && pio run -e esp32-c3-devkitm-1 -t upload
```

After the first USB flash, hold the module's **BOOT button for 3 seconds** to
enter WiFi OTA mode. b3 reports OTA progress on its NeoPixels and OLED; this
board has neither, so status goes to Serial only — which is enough, because the
setup AP name is fixed (`WDIY-Robot-Setup`) and the device is reachable as
`wdiy-servo-sonar`. Hold to 8 seconds to forget the saved network.

## Hardware note worth reading before you build a batch

The two servos share `VCC` with the ESP32, and the v1 board has only 100 nF on
that rail. These are the *drive* servos — they run constantly, not
occasionally — and an SG90 pulls ~700 mA stalled with a larger startup surge.
100 nF does nothing at that current: the rail dips and the C3 browns out
mid-drive. Add **470–1000 µF at the servo connectors**, and prefer feeding
servo power straight from the input terminal rather than through the
SuperMini's 5V pin, so servo current does not run through the module.

## Known-stale

`01_software/01_app/02_web/` is inherited from b3 and still describes b3's
hardware — LEDs, strip, OLED, battery. It is what `pages.yml` publishes, so it
was left in place rather than deleted, but it should be rewritten or unpublished
before anyone is pointed at it.

Powered by [Workshop-DIY.org](https://workshop-diy.org)
