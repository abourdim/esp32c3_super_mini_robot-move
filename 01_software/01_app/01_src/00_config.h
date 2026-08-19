#ifndef __DEF_INCLUDE_CONFIG_H__
#define __DEF_INCLUDE_CONFIG_H__


#define CONFIG_BLE_DEVICE_NAME "diy_app_s2"

#define DEF_DERIAL_DEBUG



// --------------------------------------
// button
#define CONFIG_PIN_BUTTON 0

// --------------------------------------
// WiFi OTA — off by default (no WiFi during normal BLE-driven play). Hold
// the debug button for CONFIG_OTA_HOLD_MS at any point during normal
// operation to enter OTA mode instead. No SSID/password hardcoded here:
// WiFiManager remembers the last network it joined (its own NVS storage),
// and opens a setup access point for you to enter new credentials from a
// phone whenever it can't reconnect — see 17_ota.cpp.
#define CONFIG_OTA_HOSTNAME          "wdiy-servo-sonar"
#define CONFIG_OTA_HOLD_MS           3000
#define CONFIG_OTA_SETUP_AP_NAME     "WDIY-Robot-Setup"
#define CONFIG_OTA_PORTAL_TIMEOUT_S  180
// Keep holding past CONFIG_OTA_HOLD_MS, all the way to this total, to
// forget the saved WiFi network and force the setup portal open — the
// normal path just reconnects to whatever's already saved.
#define CONFIG_OTA_FORGET_HOLD_MS    8000

// --------------------------------------
// Ultrasonic sensor pins
#define CONFIG_PIN_SR04_TRIG 21
#define CONFIG_PIN_SR04_ECHO 20

// --------------------------------------
// Servo configuration

#define CONFIG_SERVO_SPEED_STOP_LEFT    90
#define CONFIG_SERVO_SPEED_STOP_RIGHT   90

#define CONFIG_SERVO_SPEED_STOP_LEFT_OFFSET    5
#define CONFIG_SERVO_SPEED_STOP_RIGHT_OFFSET    5

// This board wires J2 pin 3 to GPIO10 and J3 pin 3 to GPIO7 (schematic
// 34_richa_c3_servo_sonar, rev v1). b3 uses GPIO6/3 instead, which is the one
// pin difference between the two robots. GPIO10 is free here only because this
// board has no LEDs and no external NeoPixel strip -- on b3 that pin is
// CONFIG_PIN_LED_RED and the strip output.
// Swap these two if the robot drives backwards or spins on the spot: which
// physical connector is "left" depends on how the servos are mounted.
#define CONFIG_PIN_SERVO_LEFT  10  // J2
#define CONFIG_PIN_SERVO_RIGHT 7   // J3

#endif // __DEF_INCLUDE_CONFIG_H__
