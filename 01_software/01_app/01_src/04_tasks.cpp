#include "01_includes.h"

// ===========================================================================
void tasks_connect(void) {
// ===========================================================================
  events_connection_state_enum l_currentState;

    l_currentState = events_get_connect_flag();

    if(l_currentState == EVENTS_CONNECTION_NO_CHANGE){
      return;
    }
    
    // b3 beeps on connect and disconnect. This board has no buzzer, so the
    // transition is only consumed to clear the flag -- the app still shows the
    // link state, and the servos are stopped by tasks_joysticks() whenever
    // remotexy_get_connect_flag() is false.
    (void)l_currentState;
}

// ===========================================================================
void tasks_buttons(void) {
// ===========================================================================
  // The app button and the BOOT button both used to beep and flash the strip.
  // Neither exists here, so the presses are simply read -- button_pressed() is
  // edge-triggered and must still be called every loop so the OTA long-press
  // detector in 17_ota.cpp sees a consistent button state.
  (void)remotexy_get_button_01();
  (void)button_pressed();
}

// ===========================================================================
void tasks_joysticks(void) {
// ===========================================================================

// Map joystick values to servo speeds
  int32_t x;
  int32_t y;

  

  uint32_t x_abs;
  uint32_t y_abs;
  uint32_t max_x_y;

  // Calculate motor speeds based on joystick position

  x = remotexy_get_joystick_01_x() ;
  y = remotexy_get_joystick_01_y();

  if(x>=0){ x_abs=x; }else { x_abs=-x; }
  if(y>=0){ y_abs=y; }else { y_abs=-y; } 
  if(x_abs>y_abs) { max_x_y = x_abs; } else { max_x_y = y_abs; }  

  // NOTE: do not call remotexy_set_*() here — tasks_joysticks() runs on
  // every loop() iteration. Calling a BLE sendValue() that often (even
  // gated by s_sendingCfg) was found to flood NimBLE's notify() with
  // rc=6 (BLE_HS_ENOMEM) continuously during a connected session. Just
  // stash the value; tasks_remotexy() sends it on the throttled interval.
  g_joystick_speed_pct = max_x_y;


   // Read joystick values into joystick_01_x and joystick_01_y here
  
  int leftSpeed = y+x;  // forward/backward + turn
  int rightSpeed = y-x; // forward/backward - turn

  // Clamp speeds to -100 to 100
  leftSpeed = constrain(leftSpeed, -100, 100);
  rightSpeed = constrain(rightSpeed, -100, 100);

  // Speed slider (DRIVE zone) scales the mix rather than clipping it, so
  // steering geometry is preserved at every ceiling — a hard clamp would
  // straighten out turns as the cap came down.
  const uint8_t cap = remotexy_get_speed_cap();
  if (cap < 100) {
    leftSpeed  = (leftSpeed  * cap) / 100;
    rightSpeed = (rightSpeed * cap) / 100;
  }

  if (! remotexy_get_connect_flag()) {  // Only if connected
    rightSpeed = 0;
    leftSpeed = 0;
  }

  // Map -100 to 100 into servo signals: 0 to 180
  // Assuming 90 = stop, <90 = reverse, >90 = forward
  int leftPulse = map(leftSpeed , -100, 100, 0, 180);
  // Right wheel is mounted as a mirror image of the left one on a
  // differential-drive chassis, so the same rightSpeed sign needs an
  // inverted pulse range to actually spin the same physical direction.
  int rightPulse = map(rightSpeed, -100, 100, 180, 0);

  // Trim is a raw pulse offset, so it lands in each servo's OWN frame -- and
  // the right channel's frame is inverted by the map() above. Adding the same
  // +N to both therefore pushed the two wheels in OPPOSITE physical directions:
  // at full forward the left reached 180 (full speed) while the right sat at
  // 0+N, which is N degrees SHORT of its full speed, so the robot curved right.
  // At rest it was worse -- 90+N on the left creeps forward while 90+N on the
  // right creeps backward, so an idle robot slowly turned on the spot.
  // Subtracting on the inverted side makes a positive trim mean "more forward"
  // on both wheels, which is the only reading that makes the constants usable.
  // Trim now comes from the app (and NVS) rather than the compile-time
  // constants, which remain only as the factory default for a fresh chip.
  // Still subtracted on the right: that channel's pulse range is inverted, so
  // this is what makes a positive trim mean "more forward" on both wheels.
  moveServos(leftPulse  + remotexy_get_trim_left(),
             rightPulse - remotexy_get_trim_right());
}

// ===========================================================================
void tasks_elapsed_time(void) {
// ===========================================================================
  uint32_t l_millis;
  uint32_t l_elapsed;
  uint32_t l_total_seconds;

  l_millis = millis();

  l_elapsed = l_millis - g_elapsed_time_startup_millis;

  l_total_seconds = l_elapsed / 1000;
  g_elapsed_time_hours   = l_total_seconds / 3600;
  g_elapsed_time_minutes = (l_total_seconds % 3600) / 60;
  g_elapsed_time_seconds = l_total_seconds % 60;

}

// ===========================================================================
void tasks_remotexy(void) {
// ===========================================================================
  remotexy_set_onlineGraph_01_distance(g_ultrasonic_distance_cm);
  remotexy_set_onlineGraph_02_speed(g_joystick_speed_pct);

  // Widgets added with the zoned layout. Each one self-gates on the Telemetry
  // level, so this list stays flat rather than nesting the whole block.
  remotexy_send_graph_distance(g_ultrasonic_distance_cm);
  remotexy_send_obstacle_alert(g_ultrasonic_distance_cm);
  remotexy_send_system_labels();
  remotexy_send_button_state();
  remotexy_send_link_rssi();
  remotexy_send_control_echo();
  // Must stay last: ends the forced full refresh that follows a CFG transfer.
  remotexy_telemetry_end();
}




 

