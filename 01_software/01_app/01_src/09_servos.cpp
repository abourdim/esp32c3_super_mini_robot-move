#include "01_includes.h"


// ===========================================================================
void moveServos(int32_t leftSpeed, int32_t rightSpeed) {
// ===========================================================================
  leftSpeed = constrain(leftSpeed, 0, 180);
  rightSpeed = constrain(rightSpeed, 0, 180);

  // moveServos() runs from tasks_joysticks() on every loop() iteration.
  // Skip redundant PWM writes when the value hasn't changed, and cap the
  // rate to ~50Hz otherwise — servos don't respond faster than that, and
  // the app itself already throttles incoming joystick messages to ~200ms.
  static uint32_t s_lastWriteMillis = 0;
  static bool     s_hasWritten = false;
  uint32_t now = millis();
  bool changed = (leftSpeed != (int32_t)g_speed_s1) || (rightSpeed != (int32_t)g_speed_s2);
  if (changed && (!s_hasWritten || (now - s_lastWriteMillis >= 20))) {
    servoLeft.write(leftSpeed);
    servoRight.write(rightSpeed);
    g_speed_s1 = leftSpeed;
    g_speed_s2 = rightSpeed;
    s_lastWriteMillis = now;
    s_hasWritten = true;
  }

  // For debugging
  #ifdef DEF_DERIAL_DEBUG
  // Serial.printf("Left: %d, Right: %d\n", leftSpeed, rightSpeed);
  #endif
}

// ===========================================================================
void stopServos() {
// ===========================================================================
  moveServos(CONFIG_SERVO_SPEED_STOP_LEFT, CONFIG_SERVO_SPEED_STOP_RIGHT);
}

// ===========================================================================
// Sonar head (J6)
// ===========================================================================
// Kept apart from moveServos() on purpose. Those two are continuous-rotation
// servos where the "angle" is really a speed, rewritten on every loop from the
// drive mix; this one is a positional servo that moves when the app asks it to
// and then stays put. Sharing the rate limiter would mean the head fought the
// drive channel for the same 20ms budget for no reason.

static uint8_t s_head_angle = CONFIG_SERVO_HEAD_CENTER;

// ===========================================================================
void moveHead(int32_t angle) {
// ===========================================================================
  // Clamp to what the mount can physically clear, not to 0..180. A servo
  // driven past its mechanical stop sits there stalled, drawing its full
  // stall current off the same rail as the C3 -- which on this board is the
  // brownout that looks like a random reboot mid-sweep.
  angle = constrain(angle, CONFIG_SERVO_HEAD_MIN, CONFIG_SERVO_HEAD_MAX);

  if ((uint8_t)angle == s_head_angle) return;   // no redundant PWM writes
  s_head_angle = (uint8_t)angle;
  servoHead.write(s_head_angle);
}

// ===========================================================================
void centerHead() {
// ===========================================================================
  moveHead(CONFIG_SERVO_HEAD_CENTER);
}

// ===========================================================================
uint8_t headAngle() {
// ===========================================================================
  // The clamped value, deliberately. The radar plots blips at the angle the
  // head is really at; echoing back the unclamped request would smear
  // detections across arc the sensor never actually pointed at.
  return s_head_angle;
}
