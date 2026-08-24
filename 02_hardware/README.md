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

| GPIO | Connector | Used for |
|---|---|---|
| 5 | J5, 3-pin, via TXB0104 | left drive servo |
| 10 | J12, 3-pin, via TXB0104 | right drive servo |
| 6 / 7 | J10, 4-pin, via TXB0104 | HC-SR04 trig / echo |
| 1 | J6, 3-pin, **raw 3.3 V** · also Q1 gate → J1 | head servo |
| 0 | J7, 3-pin · also Q2 gate → J2 | green LED |
| 21 | J8, 3-pin · also Q3 gate → J3 | red LED |
| 20 | J9, 3-pin · also Q4 gate → J4 | NeoPixel strip |
| 3 | SW1, 10k pull-up R10 | button, OTA hold |
| 4 | SW2, 10k pull-up R11 | 🔧 buzzer + battery sense, bodged to the pads |
| **2, 8, 9** | **nowhere — module pads only** | 🔧 OLED on 8/9, four wires |

The four 3-pin headers are the same nets as the FET gates, so anything on
J6–J9 switches its paired screw terminal too. **Leave J1–J4 unpopulated.** The
compensation is that D6–D9 become free activity indicators for whatever is on
the header.

Each switched output has a 1N4007 flyback (D1–D4), a 10k gate pull-down
(R6–R9) and a yellow state LED (D6–D9 + R1–R4). The SK12D07VG4 slide switch
gates the input rail; J11 (screw) and J13 (JST-PH) both feed it.

## Three things to know before building one

**The 3-pin headers share nets with the screw terminals.** GPIO 1 is Q1's gate
*and* J6's signal pin; the same pairing holds for J7/J2, J8/J3 and J9/J4. A
servo on J6 pulses the J1 output at 50 Hz and flickers D6 continuously; a load
on J1 makes the head twitch. Populate the header or the terminal, never both.
Firmware cannot arbitrate this — it is a copper fact.

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

## Attaching the OLED

GPIO 2, 8 and 9 reach the SuperMini's pads and no connector, so the SSD1306 is
four wires soldered to the module itself. This is what v2's "oled connector"
note is meant to fix.

| OLED pin | goes to |
|---|---|
| SDA | module **GPIO 8** |
| SCL | module **GPIO 9** |
| VCC | module **3V3** — *not* 5V, see below |
| GND | any GND |

**Take VCC from 3V3, not 5V.** Most SSD1306 modules accept either, but the
module's own I2C pull-up resistors go to whatever you feed it. On 5V those
pull-ups would hold SDA and SCL at 5V, and the C3's GPIOs are not 5V tolerant.

**The pull-ups are also what make the board boot.** GPIO 8 and 9 are both
strapping pins on the ESP32-C3, and both need to be high at reset — GPIO 9 low
at reset is what selects download mode. The module's pull-ups hold them there,
which is exactly why this pin pair works on b3 too. Flashing still works
normally: the module's BOOT button pulls GPIO 9 low against the pull-up.

The firmware probes `0x3C` at boot and re-probes every five seconds while no
screen is answering, so a wire that makes contact late or works loose recovers
without a power cycle. Watch serial for `[OLED] found` or `[OLED] appeared on
the bus`. If you get neither, the usual cause is a module strapped to **0x3D**
instead of 0x3C — some boards have a solder jumper for it.

Powered by [Workshop-DIY.org](https://workshop-diy.org)
