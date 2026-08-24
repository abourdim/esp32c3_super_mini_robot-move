///////////////////////////////////////////////
//  esp32c3_super_mini_servo_sonar-rxy       //
//  Two drive servos and an SR04. Nothing    //
//  else -- see 00_config.h for the pin map. //
//////////////////////////////////////////////

#include "01_includes.h"

// ===========================================================================
void setup() {
// ===========================================================================

  // Serial for debugging
  #ifdef DEF_DERIAL_DEBUG
  Serial.begin(115200);
  #endif

  remotexy_init();

  // Allocate timer for ESP32 PWM
  ESP32PWM::allocateTimer(0);

  // Attach servos to their pins
  servoLeft.attach(CONFIG_PIN_SERVO_LEFT);
  servoRight.attach(CONFIG_PIN_SERVO_RIGHT);

  // Stop both servos initially
  stopServos();

  ultrasonic_init();

  g_ultrasonic_distance_cm=0;
  g_elapsed_time_startup_millis = millis();  // Record start time

  // The BOOT button is on the SuperMini module itself, not on the carrier
  // board, so it is still here even though this board routes nothing to
  // GPIO0. It is what opens WiFi OTA mode -- see ota_check_long_press().
  button_init();

  event_connection_state_flag_bo = remotexy_get_connect_flag();  // Set initial state

  g_previous_millis_u32 = 0;

}

// ===========================================================================
void loop() {
// ===========================================================================

  remotexy_handler();

  // Hold the debug button for CONFIG_OTA_HOLD_MS at any point during normal
  // operation to enter WiFi OTA mode instead of driving. Blocks forever if
  // triggered; only returns if the hold threshold hasn't been reached.
  ota_check_long_press();

  tasks_connect();
  tasks_buttons();
  tasks_joysticks();
  tasks_elapsed_time();

  if (events_get_timeout_flag() == EVENTS_TIMEOUT_OCCURED) {

    ultrasonic_get_distance();

    tasks_remotexy();

  }

  events_reset_connect_flag();

  if (events_get_timeout_flag() == EVENTS_TIMEOUT_OCCURED) {
    events_reset_timeout_flag();
  }
}
