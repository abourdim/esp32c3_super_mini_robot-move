#ifndef __DEF_INCLUDE_CONFIG_H__
#define __DEF_INCLUDE_CONFIG_H__


// Distinct from b3 ("diy_app_b3") and the servo-sonar board ("diy_app_s2"), so
// the three robots are told apart in the app's scanner list when more than one
// is powered on in the same room -- which, in a workshop, is the normal case.
#define CONFIG_BLE_DEVICE_NAME "diy_app_mv"

#define DEF_DERIAL_DEBUG


// ===========================================================================
// Board: 30_esp32_c3_move rev v1  (01_kicad/30_esp32_c3_move/v1)
// ===========================================================================
// This is NOT the servo-sonar board this firmware was forked from -- every pin
// moved. The map below was read out of the PCB netlist rather than the
// schematic, because the schematic labels every net `gpio_NN` and says nothing
// about what it drives.
//
// What the board gives us:
//
//   J5  -- 3-pin, GPIO 5,  through the TXB0104 level shifter  -> LEFT drive servo
//   J12 -- 3-pin, GPIO 10, through the TXB0104 level shifter  -> RIGHT drive servo
//   J10 -- 4-pin, GPIO 6/7, through the TXB0104               -> HC-SR04
//   J6  -- 3-pin, GPIO 1,  RAW 3.3V, no level shifter         -> HEAD servo
//   SW1 -- GPIO 3, 10k pull-up to 3V3, switch to GND          -> button / OTA
//   SW2 -- GPIO 4, 10k pull-up to 3V3, switch to GND          -> spare
//   J1..J4 -- screw terminals, low-side 2N7002 switches on GPIO 1/0/21/20
//
// GPIO 2, 8 and 9 reach the SuperMini's pads and nothing else -- there is no
// connector on any of them. That is why this board has no OLED: b3 puts the
// screen on I2C 8/9, and here those pins go nowhere.
// ===========================================================================


// --------------------------------------
// button
//
// SW1 on the carrier board, NOT the module's BOOT button. b3 and the
// servo-sonar board both use GPIO 0 for this, but on THIS board GPIO 0 is
// Q2's gate: holding BOOT for the OTA gesture would switch the J2 screw
// terminal on for three seconds along the way. SW1 is a real button with its
// own 10k pull-up (R10) and drives nothing else.
#define CONFIG_PIN_BUTTON 3

// SW2. Wired, pulled up (R11), and currently unused -- no layout offers a
// second button yet. Defined here so the next person does not have to re-read
// the netlist to find out it exists.
#define CONFIG_PIN_BUTTON_2 4

// --------------------------------------
// WiFi OTA — off by default (no WiFi during normal BLE-driven play). Hold
// the debug button for CONFIG_OTA_HOLD_MS at any point during normal
// operation to enter OTA mode instead. No SSID/password hardcoded here:
// WiFiManager remembers the last network it joined (its own NVS storage),
// and opens a setup access point for you to enter new credentials from a
// phone whenever it can't reconnect — see 17_ota.cpp.
#define CONFIG_OTA_HOSTNAME          "wdiy-move"
#define CONFIG_OTA_HOLD_MS           3000
#define CONFIG_OTA_SETUP_AP_NAME     "WDIY-Robot-Setup"
#define CONFIG_OTA_PORTAL_TIMEOUT_S  180
// Keep holding past CONFIG_OTA_HOLD_MS, all the way to this total, to
// forget the saved WiFi network and force the setup portal open — the
// normal path just reconnects to whatever's already saved.
#define CONFIG_OTA_FORGET_HOLD_MS    8000

// --------------------------------------
// Ultrasonic sensor pins — J10, the dedicated 4-pin HC-SR04 connector
// (VCC / TRIG / ECHO / GND). Both lines run through the TXB0104, so ECHO
// arrives as a clean 3.3V edge instead of the 5V the sensor actually drives.
// This is the one part of this board that is unambiguously better than b3's,
// where ECHO goes straight to the pin.
#define CONFIG_PIN_SR04_TRIG 6   // J10 pin 2
#define CONFIG_PIN_SR04_ECHO 7   // J10 pin 3

// --------------------------------------
// Servo configuration

#define CONFIG_SERVO_SPEED_STOP_LEFT    90
#define CONFIG_SERVO_SPEED_STOP_RIGHT   90

// Per-wheel straight-line trim, in degrees of servo pulse. Positive = that
// wheel drives MORE forward. Applied in the same physical direction on both
// sides (see tasks_joysticks(), which subtracts on the mirrored right channel).
//
// Zero is the correct default: with both at 0 the two wheels get equal and
// opposite full-scale commands, so anything left is real mechanical or
// servo-tolerance error -- which is exactly what these are for.
//
// To tune: drive forward on a flat floor. If it pulls RIGHT, the right wheel is
// slower -- raise RIGHT_OFFSET by 2 and retest. If it pulls LEFT, raise
// LEFT_OFFSET. Expect single digits; more than ~10 usually means a mechanical
// problem rather than a calibration one.
#define CONFIG_SERVO_SPEED_STOP_LEFT_OFFSET    0
#define CONFIG_SERVO_SPEED_STOP_RIGHT_OFFSET   0

// Drive servos. Both go through the TXB0104, so they see real 5V pulses --
// swap these two if the robot drives backwards or spins on the spot, since
// which connector is "left" depends on how the servos are mounted.
#define CONFIG_PIN_SERVO_LEFT  5    // J5,  level-shifted
#define CONFIG_PIN_SERVO_RIGHT 10   // J12, level-shifted

// --------------------------------------
// Head servo — J6. Pans the HC-SR04 so the app's radar widget has an angle to
// plot against; a sweep builds a picture of the room instead of flashing one
// number.
//
// TWO THINGS ABOUT J6 THAT ARE NOT TRUE OF J5/J12:
//
//   1. GPIO 1 is ALSO Q1's gate, which drives the J1 screw terminal and its
//      yellow indicator LED (D6). Every servo pulse switches that FET. Leave
//      J1 UNPOPULATED whenever a head servo is fitted -- otherwise whatever is
//      wired to it pulses at 50Hz along with the head, and D6 flickers
//      continuously. The reverse also holds: use J1 as an output and the head
//      servo will twitch.
//
//   2. J6 does NOT go through the TXB0104. It carries the raw 3.3V GPIO. Most
//      hobby servos accept a 3.3V pulse, but this is the one connector on the
//      board where a fussy unit may need the level shifter it does not have.
#define CONFIG_PIN_SERVO_HEAD 1     // J6, NOT level-shifted

// Where the head points at boot and on STOP. 90 = straight ahead.
#define CONFIG_SERVO_HEAD_CENTER 90

// Mechanical travel limits. A pan mount usually cannot reach the full 0..180
// without the sensor fouling the chassis or pulling its own wires out, and a
// servo told to go there will sit there straining. Narrow these to what your
// mount actually clears.
#define CONFIG_SERVO_HEAD_MIN 10
#define CONFIG_SERVO_HEAD_MAX 170

#endif // __DEF_INCLUDE_CONFIG_H__
