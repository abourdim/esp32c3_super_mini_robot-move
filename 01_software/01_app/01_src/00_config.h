#ifndef __DEF_INCLUDE_CONFIG_H__
#define __DEF_INCLUDE_CONFIG_H__


// Distinct from b3 ("diy_app_b3") so the two are told apart in the app's
// scanner when both are powered on in the same room.
#define CONFIG_BLE_DEVICE_NAME "diy_app_mv"

#define DEF_DERIAL_DEBUG


// ===========================================================================
// Board: 30_esp32_c3_move rev v1  (02_hardware/v1)
// ===========================================================================
// This is a port of b3, kept faithful to it: every module b3 has is still
// here. What changed is the pin map, because the two carrier boards share an
// MCU and nothing else. The map below was read out of the PCB netlist, not the
// schematic -- the schematic names every net `gpio_NN` and records nothing
// about what it drives.
//
// The board's connectors:
//
//   J5  -- 3-pin, GPIO 5,  via TXB0104 level shifter   -> LEFT drive servo
//   J12 -- 3-pin, GPIO 10, via TXB0104 level shifter   -> RIGHT drive servo
//   J10 -- 4-pin, GPIO 6/7, via TXB0104                -> HC-SR04
//   J6  -- 3-pin, GPIO 1,  raw 3.3V                    -> HEAD servo
//   J7  -- 3-pin, GPIO 0   (also Q2 gate -> J2)        -> GREEN led
//   J8  -- 3-pin, GPIO 21  (also Q3 gate -> J3)        -> RED led
//   J9  -- 3-pin, GPIO 20  (also Q4 gate -> J4)        -> NeoPixel strip
//   SW1 -- GPIO 3, 10k pull-up R10                     -> button / OTA
//   SW2 -- GPIO 4, 10k pull-up R11                     -> buzzer + battery
//
// THE 3-PIN HEADERS DOUBLE AS FET GATES. J6/J7/J8/J9 carry the same net as
// Q1/Q2/Q3/Q4's gates, which drive the J1/J2/J3/J4 screw terminals and their
// yellow indicator LEDs. Anything plugged into a 3-pin header switches its
// paired screw terminal in sympathy. Leave J1-J4 UNPOPULATED. The upside: the
// onboard yellow LEDs (D6-D9) become free activity indicators for whatever is
// on the header.
//
// THREE FEATURES NEED WIRES. GPIO 2, 8 and 9 reach the SuperMini's pads and no
// connector at all, and the board has no buzzer and no battery divider. The
// OLED, buzzer and battery sense below are therefore bodge-only -- see the
// notes at each. Every other b3 feature lands on a real connector.
// ===========================================================================


// --------------------------------------
// button
//
// SW1 on the carrier board. b3 uses GPIO 0, the module's own BOOT button, and
// that button physically exists here too -- but on THIS board GPIO 0 is Q2's
// gate, so the three-second OTA hold would switch the J2 output on for the
// duration of the hold, and GPIO 0 is more useful as the green LED. SW1 is a
// real button with its own pull-up that drives nothing else.
#define CONFIG_PIN_BUTTON 3

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
// buzzer
//
// BODGE ONLY -- there is no buzzer footprint on this board. GPIO 4 is SW2's
// pin, so a passive buzzer solders to SW2's pads; SW2 then reads as pressed
// whenever the line is driven low, which nothing uses. Kept on GPIO 4 because
// that is b3's buzzer pin, so the module itself is unchanged.
// OFF: no buzzer fitted. playTone() busy-waits the full note with
// delayMicroseconds(), so leaving it enabled costs loop() time to make
// silence.
#define CONFIG_BUZZER_ENABLED 0

#define CONFIG_PIN_BUZZER 4
#define CONFIG_BUZZER_FREQ 4000

// battery level
//
// BODGE ONLY, and it shares GPIO 4 with the buzzer exactly as b3 does. b3's
// board has a divider feeding this pin; THIS board has none, so without one
// wired to SW2's pads the reading is meaningless rather than merely noisy.
// The Power test panel puts a Buzz button next to the live voltage for the
// same reason it does on b3: the interaction is visible, not hidden.
#define CONFIG_PIN_BATTERY_LEVEL 4

#define BATTERY_MIN_V 3.3  // Minimum battery voltage (adjust according to your battery)
//#define BATTERY_MAX_V 4.2  // Maximum battery voltage (adjust according to your battery)
#define BATTERY_MAX_V 4.5  // Maximum battery voltage (adjust according to your battery)

// --------------------------------------
// neopixel

// OFF: no strip fitted. Worth compiling out rather than letting FastLED
// write into the void -- the default French-flag effect calls delay(1000/60)
// on EVERY loop() iteration, so an absent strip still paces the whole robot
// at 60Hz for nothing.
#define CONFIG_NEOPIXELS_ENABLED 0

#define CONFIG_NEOPIXELS_MAX_LEDS 256      // Max supported (array size)

// J9 is a genuine strip connector -- signal, 5V and GND on one 3-pin header --
// so unlike b3 there is no onboard/external split here. Both cases are the
// same connector; only the active LED count differs.
#if 1
  #define CONFIG_PIN_NEOPIXEL 20
  #define CONFIG_NEOPIXELS_NB_LEDS 4       // Active LED count
#else
  #define CONFIG_PIN_NEOPIXEL 20           // external strip, same connector
  #define CONFIG_NEOPIXELS_NB_LEDS 16      // Active LED count
#endif

#define CONFIG_NEOPIXELS_BRIGHTNESS 15       // Brightness (0-255)



// --------------------------------------
// I2C pins
//
// BODGE ONLY. Unchanged from b3, and for the same reason they work there: on a
// C3 both are strapping pins, and I2C pull-ups hold them high, which is the
// correct boot state. What this board lacks is copper -- GPIO 8 and 9 reach
// the SuperMini's pads and stop. Four wires (SDA, SCL, 3V3, GND) soldered to
// the module bring the whole screen back, eyes and all.
// OFF: GPIO 8 and 9 are not wired on this board, so the screen is compiled
// out entirely rather than merely detected-absent. The runtime guard would
// work -- it does, verified on hardware -- but it still re-probes a dead bus
// every five seconds forever. Set to 1 once the four wires are on the
// module's pads; nothing else needs changing.
#define CONFIG_OLED_ENABLED 0

#define CONFIG_PIN_OLED_SDA 8
#define CONFIG_PIN_OLED_SCL 9

// --------------------------------------
// Ultrasonic sensor pins — J10, the dedicated 4-pin HC-SR04 connector.
// Both lines run through the TXB0104, so ECHO arrives as a clean 3.3V edge
// instead of the 5V b3 feeds straight to a pin. This is the one part of this
// board that is unambiguously better than b3's.
#define CONFIG_PIN_SR04_TRIG 6   // J10 pin 2
#define CONFIG_PIN_SR04_ECHO 7   // J10 pin 3

// --------------------------------------
// Servo configuration

#define CONFIG_SERVO_SPEED_STOP_LEFT    90
#define CONFIG_SERVO_SPEED_STOP_RIGHT   90

#define CONFIG_SERVO_SPEED_STOP_LEFT_OFFSET    5
#define CONFIG_SERVO_SPEED_STOP_RIGHT_OFFSET    5

// Drive servos. Both go through the TXB0104, so they see real 5V pulses.
// Swap these two if the robot drives backwards or spins on the spot: which
// physical connector is "left" depends on how the servos are mounted.
// Both wheels ran backwards: pressing forward drove the robot in reverse.
// That is motor polarity, not a left/right mix-up -- swapping the two pins
// would give a robot that spins instead. Inverting both channels fixes the
// steering with it: with the polarity wrong, a spin-left command was also
// coming out as spin-right, and correcting the sign corrects both.
#define CONFIG_SERVO_INVERT_DRIVE 1

#define CONFIG_PIN_SERVO_LEFT  5    // J5,  level-shifted
#define CONFIG_PIN_SERVO_RIGHT 10   // J12, level-shifted

// --------------------------------------
// Head servo — J6. b3 has no third servo; this board has the connector for it,
// so the HC-SR04 can pan and the app's radar widget has a bearing to plot
// against instead of one number that changes for invisible reasons.
//
// J6 does NOT go through the TXB0104 -- it carries the raw 3.3V GPIO. Most
// hobby servos accept that; this is the one connector where a fussy unit may
// want the level shifter it does not have.
// J6 is wired and the head servo is fitted. Note ESP32PWM::allocateTimer()
// in setup(): three servos need more than the one timer b3 allocates, and
// running short does not warn, it HALTS inside attach().
#define CONFIG_HEAD_SERVO_ENABLED 1

#define CONFIG_PIN_SERVO_HEAD 1     // J6, NOT level-shifted

// Where the head points at boot and on STOP. 90 = straight ahead.
#define CONFIG_SERVO_HEAD_CENTER 90

// Mechanical travel limits of the pan mount, not the servo's 0..180. A servo
// told to go past its stop sits there stalled, drawing full stall current off
// the same rail as the C3 -- which is the brownout that reads as a random
// reboot mid-sweep. Narrow these to what your mount actually clears.
// Sweep mode: the head steps one notch per telemetry pass rather than on a
// timer of its own, so each (angle, distance) pair the radar plots was
// measured with the head already settled at that bearing. Stepping faster
// than the sonar is read just paints readings at angles the sensor was not
// actually pointing at when they were taken.
#define CONFIG_SERVO_HEAD_SWEEP_STEP 15

#define CONFIG_SERVO_HEAD_MIN 10
#define CONFIG_SERVO_HEAD_MAX 170

// --------------------------------------
// Board LEDs. b3 has these on GPIO 10 and 1; both are servos here, so they
// move to two of the spare 3-pin headers. Each header's paired yellow onboard
// LED (D7 for J7, D8 for J8) lights along with it, which makes the link
// indicator visible even with nothing plugged in.
#define CONFIG_PIN_LED_RED 21    // J8
#define CONFIG_PIN_LED_GREEN 0   // J7

// OLED config
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1  // Reset pin not used

#endif // __DEF_INCLUDE_CONFIG_H__
