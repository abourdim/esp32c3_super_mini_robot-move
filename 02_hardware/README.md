# 🔌 Hardware

KiCad source, PCB renders and fab output for **`30_esp32_c3_move`**, the board
this firmware targets. Upstream lives at `01_kicad/30_esp32_c3_move/`.

## Versions

| Version | Status | Folder |
|---|---|---|
| **v1** | ✅ **Current — fabricated, and what the firmware is written against** | [`v1/`](v1/) |
| v2 | 📝 Planning stub, not a revision | [`v2/`](v2/) |

**v2 is not a new board.** Its PCB is byte-identical to v1 and it carries the
same 98 components; the only differences are one schematic block nudged 14 mm,
a project rename, and a to-do note dated 02/06/2025:

> add 2 neo pixel · oled connector · buzzer · battery level · large soldering
> footprint for the transistor

None of those are implemented. It is kept here because it is where that plan is
written down, not because it builds anything different.

## The pin map, and where it came from

Read out of `v1/30_esp32_c3_move.kicad_pcb` by walking footprint pads to nets —
**not** from the schematic, which labels every net `gpio_NN` and records nothing
about what it drives. `00_config.h` is the firmware-side copy of this table.

| GPIO | Goes to | Used for |
|---|---|---|
| 5 | J5, 3-pin, via TXB0104 | left drive servo |
| 10 | J12, 3-pin, via TXB0104 | right drive servo |
| 6 / 7 | J10, 4-pin, via TXB0104 | HC-SR04 trig / echo |
| 1 | J6, 3-pin, **raw 3.3 V** · also Q1 gate → J1 | head servo |
| 3 | SW1, 10k pull-up R10 | button, OTA hold |
| 4 | SW2, 10k pull-up R11 | spare — wired, unused |
| 0 | Q2 gate → J2 screw terminal · also J7 | spare output |
| 21 | Q3 gate → J3 screw terminal · also J8 | spare output |
| 20 | Q4 gate → J4 screw terminal · also J9 | spare output |
| **2, 8, 9** | **nowhere — module pads only** | — |

Each switched output has a 1N4007 flyback (D1–D4), a 10k gate pull-down
(R6–R9) and a yellow state LED (D6–D9 + R1–R4). The SK12D07VG4 slide switch
gates the input rail; J11 (screw) and J13 (JST-PH) both feed it.

## Three things to know before building one

**J1 and J6 are the same GPIO.** GPIO 1 is Q1's gate *and* the J6 signal pin.
A head servo pulses the J1 output at 50 Hz and flickers D6 continuously; a load
on J1 makes the head twitch. Populate one or the other, never both. Firmware
cannot arbitrate this — it is a copper fact.

**J6 has no level shifter.** J5, J12 and J10 all run through the TXB0104 and
get clean 5 V logic. J6 carries the raw 3.3 V pin. Most hobby servos accept
that; a fussy one is the case where this board bites.

**The servo rail needs bulk capacitance.** Three servos share `VCC` with the
ESP32 and v1 has only 100 nF on that rail. An SG90 pulls ~700 mA stalled with
a larger startup surge, and these are drive servos — they run constantly. Add
**470–1000 µF at the servo connectors**, and feed servo power straight from the
input terminal rather than through the SuperMini's 5 V pin, so servo current
does not run through the module. Without this the rail dips and the C3 browns
out mid-drive, which reads as a random reboot.

## No OLED

GPIO 2, 8 and 9 reach the SuperMini's pads and no connector. b3 puts its
SSD1306 on I²C 8/9, so the screen is not portable to this board without wires
soldered to the module itself. That is what v2's "oled connector" note is for.

Powered by [Workshop-DIY.org](https://workshop-diy.org)
