///////////////////////////////////////////////
//  esp32c3_super_mini_robot-move            //
//  Board 30_esp32_c3_move v1: two drive     //
//  servos, a panning SR04 head, two         //
//  buttons. See 00_config.h for the pin map //
//  and for why J1 must stay unpopulated.    //
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

  // Attach servos to their pins. All three run at the same 50Hz, so they
  // share the one LEDC timer allocated above -- the C3 has six channels and
  // four timers, and three servos on one timer is well inside that.
  servoLeft.attach(CONFIG_PIN_SERVO_LEFT);
  servoRight.attach(CONFIG_PIN_SERVO_RIGHT);
  servoHead.attach(CONFIG_PIN_SERVO_HEAD);

  // Stop both servos initially
  stopServos();

  // Point the head straight ahead before anything else moves. servoHead was
  // just attached, which on ESP32Servo parks it wherever the channel happened
  // to sit -- centring makes the first radar sweep start from a known bearing
  // instead of from whatever angle the last power cycle left.
  centerHead();

  ultrasonic_init();

  g_ultrasonic_distance_cm=0;
  g_elapsed_time_startup_millis = millis();  // Record start time

  // SW1 on the carrier board (GPIO 3), NOT the module's BOOT button -- on
  // this board GPIO 0 is Q2's gate, so the OTA hold gesture would switch the
  // J2 screw terminal along with it. See ota_check_long_press().
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
