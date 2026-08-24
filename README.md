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

Every module b3 has is still in the tree — buzzer, NeoPixels, battery sense,
board LEDs, OLED, both NeoPixel demos, all eight panels. Nothing was deleted.
What changed is the pin map, because the two carrier boards share an MCU and
nothing else.

What is **not fitted** on this build is switched off at a single `#define`
each, so it compiles out rather than failing at runtime:

| feature | b3 | here | state |
|---|---|---|---|
| left / right drive servo | GPIO 6 / 3 | **5 / 10** — J5 / J12, level-shifted | ✅ |
| HC-SR04 | 21 / 20 | **6 / 7** — J10, level-shifted | ✅ |
| head servo | *(none)* | **1** — J6 | ✅ |
| button / OTA | 0 (module BOOT) | **3** — SW1 | ✅ |
| green / red LED | 1 / 10 | **0 / 21** — J7 / J8 | ✅ |
| NeoPixel strip | 5 (or 10) | 20 — J9 | `CONFIG_NEOPIXELS_ENABLED 0` |
| buzzer | 4 | 4 — SW2 pads | `CONFIG_BUZZER_ENABLED 0` |
| battery sense | 4 | 4 — SW2 pads, **no divider on the board** | reads meaningless |
| OLED | I²C 8 / 9 | 8 / 9 — module pads, **not wired** | `CONFIG_OLED_ENABLED 0` |

Turning the strip off is worth more than it looks: its default French-flag
effect calls `delay(1000/60)` on **every** `loop()` iteration, so an absent
strip still paced the whole robot at 60 Hz to animate nothing.

The OLED keeps its runtime presence-probe for the day the wires go on, but with
the feature off nothing touches the I²C bus at all — not even the five-second
re-probe, which would otherwise poke a dead bus forever. Set
`CONFIG_OLED_ENABLED 1` once SDA/SCL/3V3/GND are on the module's pads; nothing
else needs changing. Wiring and the strapping-pin reasoning:
[`02_hardware/README.md`](02_hardware/README.md#attaching-the-oled).

Board source and the full netlist-derived pin map: [`02_hardware/`](02_hardware/).

## Three servos need four PWM timers

b3 allocates **one** LEDC timer and attaches two servos, which exactly fits.
The head makes three, and ESP32Servo does not warn when it runs short — it
prints `All PWM timers allocated! Can't accomodate 50.000 Hz` and **halts
inside `attach()`**, so `setup()` never returns.

That failure is invisible from outside. NimBLE has already started on its own
task by then, so the robot still advertises and the app still connects; it just
never answers anything, because `remotexy_handler()` lives in `loop()` and
`loop()` has never run. It presents as a protocol bug and is not one.

`setup()` now allocates all four.

## The 3-pin headers double as FET gates

J6, J7, J8 and J9 carry the same nets as Q1–Q4's gates, which drive the J1–J4
screw terminals and their yellow indicator LEDs. Anything plugged into a 3-pin
header switches its paired screw terminal in sympathy, so **leave J1–J4
unpopulated**. The upside is free activity indicators: D6–D9 light along with
whatever is on the header, so the link LEDs are visible even with nothing
plugged in.

## Why the button moved off GPIO 0

b3 uses the SuperMini's own BOOT button for the three-second OTA hold, and that
button exists here too. But on this board GPIO 0 is Q2's gate, so the gesture
would switch the J2 output on for the length of the hold — and GPIO 0 is more
useful as the green LED. SW1 is a real button with its own pull-up that drives
nothing else.

## Drive direction

`CONFIG_SERVO_INVERT_DRIVE` flips both channels. Both wheels ran backwards on
this chassis — that is motor polarity, not a left/right mix-up, and swapping
the two pins would have given a robot that spins instead. The flip is applied
to the command before the mix becomes pulses, so trim and the speed cap stay in
the frame they were written for, and it corrects the steering with it: with the
polarity wrong, a spin-left command was coming out as spin-right.

## The head, the radar, and sweep

The head servo is the one thing here that b3 does not have. It gives the app's
**radar** widget a bearing to plot against — the radar carries no value of its
own, it reads the distance gauge and the head slider by id.

**Sweep** is a toggle in the DISTANCE group. The head steps one notch per
telemetry pass rather than on a timer of its own, and the step happens *after*
the pass has sent this cycle's distance and angle. That ordering is the point:
the radar plots (angle, distance) pairs, and stepping independently would paint
readings at bearings the sensor was not pointing at when they were taken. Each
step then gets a full 500 ms to settle before the next measurement. 15° steps
across 10–170° is about 5.5 s per sweep.

Reversal happens **at** each limit, not past it. `moveHead()` clamps anyway,
but bouncing off the clamp parks the head on its end stop for an extra pass
every sweep — a stutter on the scope, and a stalled servo while it lasts.

Dragging the slider takes the head off sweep, otherwise the servo fights you
and the slider snaps back on the next pass. Leaving sweep re-centres.

The bearing is published every telemetry pass. Without that the radar plots at
whatever `head` last reported, which was its default — the head would sweep and
the scope would show a beam frozen at 90°.

`CONFIG_SERVO_HEAD_MIN` / `_MAX` are the **mechanical** limits of your pan
mount, not 0–180. Narrow them to what the mount clears — a servo told to go
past its stop sits there stalled, drawing full stall current off the same rail
as the C3.

## Panels

All eight of b3's, switched with the **Level** selector; the choice is kept in
NVS. Expert has been cut down to what this board actually has:

- **removed** — the DISPLAY group (no OLED) and the SOUND group (no buzzer)
- **removed** — the six strip widgets from LIGHTS; the two board LEDs stay,
  they are real, and their J7/J8 headers light the onboard yellows too
- **moved** — TRIM is no longer its own group; all eight widgets sit inside
  DRIVE, which is where you use them, tuning while driving and watching the pull
- **added** — the head slider, the sweep toggle, and the radar, in DISTANCE

> ⚠️ **`gen_test_layouts.py` writes `test_decls.h`, which nothing includes.**
> The live blobs are hardcoded in `03_bit-rxy.cpp`, so running the generator
> changes nothing that reaches the board. It needs a splice step — S2's
> `gen_layouts.py` has the logic to copy — or it should be deleted. Until then,
> layout edits go into the blobs directly.

## Build and flash

```bash
cd 01_software/01_app && pio run -e esp32-c3-devkitm-1 -t upload
```

After the first USB flash, hold **SW1 for 3 seconds** to enter WiFi OTA mode.
Hold to 8 seconds to forget the saved network. The device is reachable as
`wdiy-move`, and the setup AP is `WDIY-Robot-Setup`.

BLE device name is `diy_app_mv`, distinct from b3's `diy_app_b3` so the two are
told apart in the scanner when both are powered on in the same room.

**The image is at 91.4% of flash** — roughly 110 KB spare on the default
partition table. Enough for tuning, not for another subsystem.

Note that with native USB-CDC the serial monitor attaches *after* boot, so you
will usually see nothing at startup. That is normal, not a fault.

## Before you build one

Three board facts firmware cannot paper over — the long form is in
[`02_hardware/README.md`](02_hardware/README.md):

- **J1–J4 must stay unpopulated** while anything is on J6–J9. Same nets.
- **J6 has no level shifter.** J5, J12 and J10 run through the TXB0104 and get
  clean 5 V logic; J6 carries the raw 3.3 V pin. Most hobby servos accept that.
- **The servo rail needs 470–1000 µF** at the connectors, fed from the input
  terminal rather than through the SuperMini's 5 V pin. Three drive servos on a
  rail with 100 nF browns the C3 out mid-drive, which reads as a random reboot.

## Known issues

- **Trim is applied with the wrong sign**, inherited from b3. It is added to
  both wheels, but the right channel's pulse range is inverted, so a positive
  trim pushes the two wheels in *opposite* physical directions. With b3's
  defaults of `5`/`5` an idle robot slowly rotates on the spot. Either zero the
  two `CONFIG_SERVO_SPEED_STOP_*_OFFSET` values or port S2's fix (`10c1468`),
  which subtracts on the inverted side.
- **`01_software/01_app/02_web/`** is b3's site and still describes b3's pin
  map. Its Pages workflow is set to manual trigger only, so a push cannot
  publish a site that is wrong about this robot. Rewrite it before re-enabling.

Powered by [Workshop-DIY.org](https://workshop-diy.org)
