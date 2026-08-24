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
// servos whose "angle" is really a speed, rewritten from the drive mix on every
// loop; this one is positional -- it moves when asked and then stays put.
// Sharing their rate limiter would make the head compete for the same 20ms
// budget as the drive channel for no reason.

static uint8_t s_head_angle = CONFIG_SERVO_HEAD_CENTER;

// ===========================================================================
void moveHead(int32_t angle) {
// ===========================================================================
  // Clamped to what the mount clears, not to 0..180. A servo driven past its
  // mechanical stop sits there stalled, pulling full stall current off the same
  // rail as the C3 -- the brownout that reads as a random reboot mid-sweep.
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
  // The clamped value, deliberately. The radar plots blips at the bearing the
  // head really reached; echoing the unclamped request back would smear
  // detections across arc the sensor never pointed at.
  return s_head_angle;
}

// ===========================================================================
// Sweep mode
// ===========================================================================
// A distance sensor on a fixed head gives one number that changes for reasons
// you cannot see. Sweeping it turns the same sensor into a picture of the
// room, which is the whole reason the radar widget exists.
//
// Deliberately stepped by the caller rather than on its own timer -- see the
// call site in tasks_remotexy(). The radar plots (angle, distance) pairs, and
// those two have to have been true at the same moment: stepping independently
// would smear readings across bearings the sensor was not pointing at.

static bool   s_sweep_on  = false;
static int8_t s_sweep_dir = +1;

// ===========================================================================
void head_sweep_set(bool on) {
// ===========================================================================
  if (on == s_sweep_on) return;
  s_sweep_on = on;
  // Leaving sweep parks the head straight ahead rather than abandoning it at
  // whatever bearing the last step reached -- otherwise switching to manual
  // starts from a position the slider does not agree with.
  if (!on) centerHead();
}

// ===========================================================================
bool head_sweep_get(void) {
// ===========================================================================
  return s_sweep_on;
}

// ===========================================================================
void head_sweep_step(void) {
// ===========================================================================
  if (!s_sweep_on) return;

  int32_t next = (int32_t)headAngle() + (int32_t)s_sweep_dir * CONFIG_SERVO_HEAD_SWEEP_STEP;

  // Reverse AT the limit, not past it. moveHead() clamps anyway, but bouncing
  // off the clamp would make the head sit at the end stop for one extra pass
  // every sweep -- visible as a stutter, and a stalled servo while it lasts.
  if (next >= CONFIG_SERVO_HEAD_MAX) { next = CONFIG_SERVO_HEAD_MAX; s_sweep_dir = -1; }
  else if (next <= CONFIG_SERVO_HEAD_MIN) { next = CONFIG_SERVO_HEAD_MIN; s_sweep_dir = +1; }

  moveHead(next);
}
