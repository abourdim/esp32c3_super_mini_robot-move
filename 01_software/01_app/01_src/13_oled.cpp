#include "01_includes.h"

extern Adafruit_SSD1306 display;

#define SIZE_BUFFER 64
char g_display_array[SIZE_BUFFER];

// Top-line override sent by the app (SET oled_text). 21 characters is what
// fits across 128px at text size 1, plus a terminator; anything longer is
// truncated here rather than silently running off the right edge.
#define OLED_TEXT_MAX 21
static char s_oled_text[OLED_TEXT_MAX + 1] = {0};

bool        oled_text_active(void) { return s_oled_text[0] != '\0'; }
const char* oled_text_get(void)    { return s_oled_text; }

void oled_text_set(const char* s) {
  if (s == nullptr) { s_oled_text[0] = '\0'; return; }
  strncpy(s_oled_text, s, OLED_TEXT_MAX);
  s_oled_text[OLED_TEXT_MAX] = '\0';
}

// Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

// Set once from setup(), after probing the bus. Defaults true so that any
// board which never calls the setter behaves exactly as b3 always did.
static bool s_oled_present = true;

void oled_set_present(bool present) { s_oled_present = present; }
bool oled_present(void) { return s_oled_present; }

// ===========================================================================
void oled_init(void) {
// ===========================================================================

#if 0
  // Set custom I2C pins
  Wire.begin(CONFIG_PIN_OLED_SDA, CONFIG_PIN_OLED_SCL);

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    #ifdef DEF_DERIAL_DEBUG
    Serial.println(F("SSD1306 allocation failed"));
    #endif
    // for (;;);
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  #endif
}

// ===========================================================================
// SCREEN MODES — status, face, radar
// ===========================================================================
// Ported from dfrobot-rover by way of maqueen-rxy, both of which draw this on
// the same 128x64 SSD1306. The geometry below is theirs unchanged, so the
// three robots wear the same face.
//
// WHAT THIS PORT DID NOT HAVE TO CARRY. Those two are MakeCode firmwares and
// keep their own framebuffer: the OLED extension there spends six command
// writes plus a data write per column-page, so one filled circle is several
// hundred I2C transactions. Adafruit_GFX already buffers in RAM and blits once
// on display(), so fillRect and drawPixel here are free and only display()
// touches the bus. The whole framebuffer layer is gone, and with it the
// reason the micro:bit versions have to freeze the face while driving.
//
// WHAT THIS BOARD CANNOT DO. There is no accelerometer, so the rover's ALARM
// mood -- small pupils and eyes down when the robot is picked up or tipped --
// has no input and is not implemented. Nor can a nudge wake the sleeping face
// the way it does on the Maqueen. Everything else is here.

#define OLED_SCREEN_STATUS 0
#define OLED_SCREEN_FACE   1
#define OLED_SCREEN_AUTO   2   // status until connected, then the face
#define OLED_SCREEN_RADAR  3
#define OLED_SCREEN_MODES  4

// Five looks, one mood system: every style blinks, worries, is startled and
// falls asleep. Only the corner cut, the eye height and the pupil size differ.
#define OLED_STYLE_ROUND   0
#define OLED_STYLE_CIRCLE  1
#define OLED_STYLE_ROBOT   2
#define OLED_STYLE_BIG     3
#define OLED_STYLE_VISOR   4
#define OLED_STYLES        5

static uint8_t  s_screen_mode = OLED_SCREEN_STATUS;
static uint8_t  s_face_style  = OLED_STYLE_ROUND;
// Whether the pupils track the driving direction. On by default because it is
// the thing that makes the face read as paying attention rather than as a
// picture -- but it is also the one mood behaviour that moves while the robot
// moves, so it is the one worth being able to switch off.
static bool     s_eyes_follow = true;

// ---- eye geometry, identical to the Maqueen's -----------------------------
#define EYE_W   46
#define EYE_H   44
#define EYE_Y   10
#define EYE_LX  10
#define EYE_RX  72
#define PUP     20
#define PUP_SMALL 10
#define BROW    18
#define SMILE_Y 29
#define HAPPY_DEPTH 7
#define HAPPY_THICK 5
#define GAZE    8
#define GAZE_Y  6

#define FACE_OPEN     0
#define FACE_SHUT     1
#define FACE_WORRIED  2
#define FACE_DIZZY    3
#define FACE_HAPPY    4
#define FACE_STARTLE  5

#define FACE_SLEEP_MS      20000
#define FACE_BLINK_SHUT_MS 140
#define FACE_DIZZY_MS      2000
#define FACE_HAPPY_MS      2200
#define FACE_STARTLE_MS    550
#define FACE_ALERT_CM      25      // "something close ahead"

static uint32_t s_next_blink_at  = 0;
static uint32_t s_shut_until     = 0;
static uint32_t s_dizzy_until    = 0;
static uint32_t s_happy_until    = 0;
static uint32_t s_startle_until  = 0;
static uint32_t s_last_command_at = 0;
static bool     s_spun           = false;
static bool     s_wink           = false;
static bool     s_wink_left      = true;
static bool     s_alert_seen     = false;

void oled_note_command(void) {
  s_last_command_at = millis();
  oled_note_activity();
}

// ---- per-style geometry ---------------------------------------------------
static int16_t style_inset(void) {
  if (s_face_style == OLED_STYLE_ROBOT)  return 0;
  if (s_face_style == OLED_STYLE_CIRCLE) return 8;
  if (s_face_style == OLED_STYLE_VISOR)  return 4;
  return 3;
}
static int16_t style_eye_h(void) {
  return (s_face_style == OLED_STYLE_VISOR) ? 24 : EYE_H;
}
static int16_t style_pup_w(void) {
  if (s_face_style == OLED_STYLE_VISOR) return 12;
  if (s_face_style == OLED_STYLE_BIG)   return 24;
  if (s_face_style == OLED_STYLE_ROBOT) return 24;
  return PUP;
}
static int16_t style_pup_h(void) {
  if (s_face_style == OLED_STYLE_VISOR) return 12;
  if (s_face_style == OLED_STYLE_BIG)   return 24;
  if (s_face_style == OLED_STYLE_ROBOT) return 14;   // letterbox, not square
  return PUP;
}

static int16_t clampi(int16_t v, int16_t lo, int16_t hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// cut_right says which side the worried brow bites into: the OUTER edge of
// each eye, so the two are mirror images rather than parallel.
static void draw_eye(int16_t x, uint8_t mode, int16_t dx, int16_t dy, bool cut_right) {
  const int16_t eh = style_eye_h();
  // Centre a short eye in the space a tall one would use, so switching style
  // does not shift the face up the glass.
  const int16_t y = EYE_Y + ((EYE_H - eh) >> 1);

  if (mode == FACE_HAPPY) {
    // An arc with its middle riding UP: the shape a shut, smiling eye makes.
    // One short vertical run per column -- a curve is the only thing here that
    // cannot be faked with rectangles.
    for (int16_t c = 0; c < EYE_W; c++) {
      const int32_t t = (int32_t)c * 200 / (EYE_W - 1) - 100;
      const int16_t rise = (int16_t)((int32_t)HAPPY_DEPTH * (10000 - t * t) / 10000);
      display.fillRect(x + c, y + SMILE_Y - rise, 1, HAPPY_THICK, SSD1306_WHITE);
    }
    return;
  }
  if (mode == FACE_SHUT) {
    // A bar, not a short rectangle: anything taller reads as a squint.
    display.fillRect(x, y + (eh >> 1) - 3, EYE_W, 6, SSD1306_WHITE);
    return;
  }

  const int16_t ins = style_inset();
  // Rounded with two crossed rectangles -- cheaper than a circle, and the
  // corners are the only part anyone notices at this size. An inset of 0
  // collapses both to one rectangle, which is the robot style.
  display.fillRect(x + ins, y, EYE_W - ins * 2, eh, SSD1306_WHITE);
  display.fillRect(x, y + ins, EYE_W, eh - ins * 2, SSD1306_WHITE);

  const bool startled = (mode == FACE_STARTLE);
  const int16_t pw = startled ? PUP_SMALL : style_pup_w();
  const int16_t ph = startled ? PUP_SMALL : style_pup_h();
  const int16_t ox = (EYE_W - pw) >> 1;
  const int16_t oy = (eh - ph) >> 1;
  // CLAMP THE GAZE TO WHAT THIS STYLE CAN HOLD. The pupil is a hole, so one
  // that reaches the border opens the eye into a C -- and the room available
  // depends on the style's own pupil size, eye height and corner inset. A
  // circular eye has under half a rectangle's travel. Clamping against the
  // geometry rather than tuning five constants by eye also means a sixth
  // style cannot reintroduce the bug.
  const int16_t rim  = ins > 0 ? ins : 2;
  const int16_t cdx = clampi(dx, rim + 1 - ox, (EYE_W - rim - 1) - (ox + pw));
  const int16_t cdy = clampi(dy, rim + 1 - oy, (eh   - rim - 1) - (oy + ph));
  // The pupil is a HOLE punched in the white of the eye.
  display.fillRect(x + ox + cdx, y + oy + cdy, pw, ph, SSD1306_BLACK);

  // One lit square inside the hole, high and outward: the catchlight that
  // separates a cartoon eye from a hole cut in a mask. Only BIG has room.
  if (s_face_style == OLED_STYLE_BIG && !startled) {
    display.fillRect(x + ox + cdx + (cut_right ? pw - 9 : 4),
                     y + oy + cdy + 4, 5, 5, SSD1306_WHITE);
  }

  if (mode == FACE_WORRIED) {
    // Brows, as a wedge cleared off the top: deepest at the OUTER edge so the
    // inner ends ride UP. Cut them the other way and the same shape reads as
    // angry. Scaled to the style's own eye height, so a short visor gets a
    // shallow brow instead of having most of it cleared away.
    const int16_t brow = (int16_t)((int32_t)BROW * eh / EYE_H);
    for (int16_t c = 0; c < EYE_W; c++) {
      const int16_t t = cut_right ? c : (EYE_W - 1 - c);
      const int16_t d = (int16_t)((int32_t)t * brow / (EYE_W - 1));
      if (d > 0) display.fillRect(x + c, y, 1, d, SSD1306_BLACK);
    }
  }
}

static void oled_draw_face(uint32_t now) {
  const uint32_t dist = g_ultrasonic_distance_cm;

  // READ THE DRIVE INTENT, NOT THE SERVO OUTPUTS. Reading g_speed_s1/s2 was
  // wrong three times over, and every one of them was visible on the glass:
  //
  //   1. The right servo is mounted mirrored, so its pulse runs BACKWARDS.
  //      Driving straight forward gives (s1-90) = +90 and (s2-90) = -90, so
  //      the sum is zero -- the eyes never looked up -- while the difference
  //      is 180, which read as a hard right turn.
  //   2. The trim is added to both pulses, so a trimmed robot parked at
  //      neutral never equalled the stop constant and `driving` was
  //      permanently true: the face never slept and the pupils never centred.
  //   3. stopServos() writes the UNTRIMMED neutral, so a hard stop looked
  //      like movement in the opposite direction to the trim.
  //
  // Both the joystick and the D-pad land in these two values, so this is what
  // the robot was actually asked to do, before mirroring or trim touch it.
  const int32_t jx = (int32_t)remotexy_get_joystick_01_x();   // turn, right +
  const int32_t jy = (int32_t)remotexy_get_joystick_01_y();   // forward, up +
  const int32_t DEAD = 5;
  const bool driving = (jx > DEAD || jx < -DEAD || jy > DEAD || jy < -DEAD);

  // Spinning on the spot: turning hard with no forward component. Remembered
  // rather than shown at once, so the dizziness lands after it stops --
  // nobody is watching the eyes while the robot spins away from them.
  if (driving) {
    if ((jx > 40 || jx < -40) && jy < 20 && jy > -20) s_spun = true;
    s_last_command_at = now;
  } else if (s_spun) {
    s_spun = false;
    s_dizzy_until = now + FACE_DIZZY_MS;
  }

  // The flinch is the CROSSING into range, not the condition. Watching the
  // condition would leave the robot staring wide-eyed for as long as it sat
  // near a wall, which is a stare rather than a fright.
  const bool alert = (dist > 0 && dist < FACE_ALERT_CM);
  if (alert && !s_alert_seen) s_startle_until = now + FACE_STARTLE_MS;
  s_alert_seen = alert;

  // The button is the one deliberate request for a face in the whole
  // firmware, so it outranks every mood the robot works out for itself.
  if (button_pressed()) s_happy_until = now + FACE_HAPPY_MS;

  uint8_t mode = FACE_OPEN;
  int16_t dx = 0, dy = 0;

  if (now < s_happy_until) {
    mode = FACE_HAPPY;
  } else if (now < s_startle_until) {
    mode = FACE_STARTLE;
    dy = -GAZE_Y;                 // eyes up and wide, away from the thing
  } else if (now < s_dizzy_until) {
    mode = FACE_DIZZY;
    const uint8_t phase = (now / 120) % 4;
    dx = (phase == 0) ? -GAZE : (phase == 2 ? GAZE : 0);
    dy = (phase == 1) ? -6    : (phase == 3 ? 6    : 0);
  } else if (alert) {
    mode = FACE_WORRIED;
    dy = 3;                       // pupils drop a little under the brows
  } else if (now - s_last_command_at > FACE_SLEEP_MS) {
    mode = FACE_SHUT;
  } else if (now >= s_next_blink_at) {
    s_shut_until    = now + FACE_BLINK_SHUT_MS;
    s_next_blink_at = now + 2500 + (uint32_t)random(0, 3500);
    // A quarter of blinks are winks. Same code path, one eye left open, and
    // it buys more character per line than anything else the face does.
    s_wink      = (random(0, 4) == 0);
    s_wink_left = (random(0, 2) == 0);
  }

  if (mode == FACE_OPEN) {
    if (now < s_shut_until) {
      mode = FACE_SHUT;
    } else if (driving && s_eyes_follow) {
      // Looking where it is going, straight off the two axes the app sent.
      if (jy >  DEAD) dy = -GAZE_Y;
      if (jy < -DEAD) dy =  GAZE_Y;
      if (jx >  DEAD) dx =  GAZE;
      if (jx < -DEAD) dx = -GAZE;
    }
  }

  const bool winking = (mode == FACE_SHUT) && s_wink;
  draw_eye(EYE_LX, (winking && !s_wink_left) ? FACE_OPEN : mode, dx, dy, false);
  draw_eye(EYE_RX, (winking &&  s_wink_left) ? FACE_OPEN : mode, dx, dy, true);
}

// ===========================================================================
// RADAR
// ===========================================================================
// A sonar map needs a HEADING to plot each reading against, and this robot's
// HC-SR04 is bolted to the chassis facing forward. Mount it on a servo and
// define CONFIG_PIN_SERVO_HEAD and the scope sweeps properly; without one
// every reading lands at the same heading and the scope draws a single spoke
// -- which is honest, because that is all the robot can see.
#define SCOPE_CX 63
#define SCOPE_CY 63               // origin on the bottom edge
#define SCOPE_R  62
// The rover stretches its scope sideways by two: a half-disc that fits 32
// rows can only have a 31-pixel radius, leaving two thirds of the width black.
// At 64 rows the radius reaches 62 and the semicircle is true.
#define BLIP_MAX      48
#define BLIP_LIFE_MS  5000
#define RADAR_MAX_CM  200

static int16_t  s_blip_angle[BLIP_MAX];
static int16_t  s_blip_cm[BLIP_MAX];
static uint32_t s_blip_at[BLIP_MAX];
static uint8_t  s_blip_n = 0;
static int16_t  s_head_angle = 90;
#ifdef CONFIG_PIN_SERVO_HEAD
static int8_t   s_head_dir = 1;
static uint32_t s_next_head_at = 0;
#define HEAD_MIN  30
#define HEAD_MAX  150
#define HEAD_STEP 6
#define HEAD_STEP_MS 140
#endif

// Deliberately NOT linear, and the same curve the app's radar widget uses so
// the two displays agree about what "close" looks like: the first 10cm gets a
// quarter of the radius. Close things are what matter, and on a linear scale
// they all pile up in the middle.
static int16_t scope_radius(int32_t cm) {
  int32_t r;
  if (cm <= 0)       r = 0;
  else if (cm < 10)  r = cm * 40 / 10;
  else if (cm < 30)  r = 40 + (cm - 10) * 40 / 20;
  else if (cm < 100) r = 80 + (cm - 30) * 80 / 70;
  else               r = 160;
  return (int16_t)(r * SCOPE_R / 160);          // that scale is 0..160
}

static void scope_plot(int16_t deg, int16_t r, uint16_t colour) {
  const float rad = (float)deg * PI / 180.0f;
  display.drawPixel(SCOPE_CX + (int16_t)lroundf(r * cosf(rad)),
                    SCOPE_CY - (int16_t)lroundf(r * sinf(rad)), colour);
}

// Three range rings: 10, 30 and 100cm. The rover draws only two -- at 32 rows
// its 10cm ring lands at a 7-pixel radius and merely thickens the origin.
// Here it reaches 15 and is a ring worth having.
//
// Drawn SOLID, one degree at a time. A dotted arc was the rover's first
// attempt and at this size read as confetti, indistinguishable from the blips
// it is meant to be a backdrop for.
static void scope_rings(void) {
  for (int16_t a = 0; a <= 180; a += 1) scope_plot(a, scope_radius(100), SSD1306_WHITE);
  for (int16_t a = 0; a <= 180; a += 2) scope_plot(a, scope_radius(30),  SSD1306_WHITE);
  for (int16_t a = 0; a <= 180; a += 4) scope_plot(a, scope_radius(10),  SSD1306_WHITE);
  display.fillRect(0, SCOPE_CY, SCREEN_WIDTH, 1, SSD1306_WHITE);   // the floor
}

static void oled_draw_radar(uint32_t now) {
#ifdef CONFIG_PIN_SERVO_HEAD
  if (now >= s_next_head_at) {
    s_next_head_at = now + HEAD_STEP_MS;
    s_head_angle += s_head_dir * HEAD_STEP;
    if (s_head_angle >= HEAD_MAX) { s_head_angle = HEAD_MAX; s_head_dir = -1; }
    else if (s_head_angle <= HEAD_MIN) { s_head_angle = HEAD_MIN; s_head_dir = 1; }
    servoHead.write(s_head_angle);
  }
#endif
  // Nothing bounced back is not a detection: plotting it would draw a wall at
  // maximum range all the way round an empty room.
  const uint32_t cm = g_ultrasonic_distance_cm;
  if (cm > 0 && cm < RADAR_MAX_CM) {
    if (s_blip_n < BLIP_MAX) {
      s_blip_angle[s_blip_n] = s_head_angle;
      s_blip_cm[s_blip_n]    = (int16_t)cm;
      s_blip_at[s_blip_n]    = now;
      s_blip_n++;
    } else {
      for (uint8_t i = 1; i < BLIP_MAX; i++) {
        s_blip_angle[i - 1] = s_blip_angle[i];
        s_blip_cm[i - 1]    = s_blip_cm[i];
        s_blip_at[i - 1]    = s_blip_at[i];
      }
      s_blip_angle[BLIP_MAX - 1] = s_head_angle;
      s_blip_cm[BLIP_MAX - 1]    = (int16_t)cm;
      s_blip_at[BLIP_MAX - 1]    = now;
    }
  }

  scope_rings();
  // Live beam: solid, so it is obviously the thing that is moving.
  for (int16_t r = 0; r <= SCOPE_R; r++) scope_plot(s_head_angle, r, SSD1306_WHITE);
  // Blips persist and fade over five seconds, so a sweep builds a picture of
  // the room instead of flashing one number.
  for (uint8_t i = 0; i < s_blip_n; i++) {
    if (now - s_blip_at[i] > BLIP_LIFE_MS) continue;
    const int16_t r = scope_radius(s_blip_cm[i]);
    scope_plot(s_blip_angle[i], r,     SSD1306_WHITE);
    scope_plot(s_blip_angle[i], r - 1, SSD1306_WHITE);  // one pixel is a speck
  }
}

// ===========================================================================
// IDLE — dim, then sleep, then off
// ===========================================================================
// The power saving is real but modest: an SSD1306 this size draws roughly
// 10-20mA depending on how many pixels are lit, against two servos that
// dominate the budget whenever the robot moves. BURN-IN is the reason this
// exists. The Status screen holds the same banner, the same labels and the
// same layout for hours, and an OLED keeps that permanently.
//
// Blanking the panel would be the obvious answer and is the wrong one twice
// over. A dark screen is indistinguishable from a flat battery, and while
// disconnected the screen's whole job is to show the pairing name so this
// robot can be picked out of a chooser listing several identical ones.
//
// So the idle screen SHOWS something, and that something DRIFTS. Burn-in comes
// from pixels that never change, not from pixels that are lit, and current is
// roughly proportional to lit pixels -- a pair of closed eyes and a small Zzz
// lights very little, so most of the saving survives anyway.
#define IDLE_DIM_MS     30000UL      // lower the contrast
#define IDLE_SLEEP_MS  120000UL      // hand the glass to the sleep picture
#define IDLE_OFF_MS   1500000UL      // 25 min: nobody is looking, panel off
#define DRIFT_STEP_MS    2500UL      // slow enough that nobody sees it move
#define DRIFT_X 8
#define DRIFT_Y 6

static uint32_t s_activity_at = 0;
static bool     s_dimmed    = false;
static bool     s_panel_off = false;
static int8_t   s_drift_x = 0, s_drift_y = 0;
static int8_t   s_drift_sx = 1, s_drift_sy = 1;
static uint32_t s_drift_at = 0;

// Any deliberate act. NOT the ultrasonic: distance readings jitter constantly,
// and waking on "the value changed" would mean the screen never slept at all.
void oled_note_activity(void) { s_activity_at = millis(); }

static void drift_tick(uint32_t now) {
  if (now - s_drift_at < DRIFT_STEP_MS) return;
  s_drift_at = now;
  s_drift_x += s_drift_sx;
  s_drift_y += s_drift_sy;
  if (s_drift_x >= DRIFT_X || s_drift_x <= -DRIFT_X) s_drift_sx = -s_drift_sx;
  if (s_drift_y >= DRIFT_Y || s_drift_y <= -DRIFT_Y) s_drift_sy = -s_drift_sy;
}

// Centred text, offset by the drift. Used for both things that must stay
// readable while idle -- the pairing name and a typed message -- so "drift
// whatever has to be read" is one piece of code rather than two.
static void draw_drifting_text(const char* s, int16_t dy) {
  int16_t x1, y1; uint16_t w, h;
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.getTextBounds(s, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2 + s_drift_x,
                    (SCREEN_HEIGHT - h) / 2 + s_drift_y + dy);
  display.print(s);
}

// Closed eyes and a Zzz. Deliberately the same shut-eye bar the face uses, so
// a sleeping robot looks like the same robot with its eyes shut rather than
// like a different screen -- but small, and drifting, because two static
// high-contrast bars held for hours are the worst burn-in shape on here.
static void oled_draw_sleeping(void) {
  const int16_t cx = SCREEN_WIDTH / 2 + s_drift_x;
  const int16_t cy = SCREEN_HEIGHT / 2 + s_drift_y;
  display.fillRect(cx - 30, cy - 2, 22, 4, SSD1306_WHITE);
  display.fillRect(cx +  8, cy - 2, 22, 4, SSD1306_WHITE);
  // Rising z's, largest nearest the face, so it reads as breath rather than
  // as characters that happen to be there. TWO, not three: a third continues
  // the diagonal past the top-right corner once the drift is at its extreme,
  // and a z half off the panel reads as a fault rather than as sleep.
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(cx + 34, cy - 14);
  display.print('z');
  display.setTextSize(1);
  display.setCursor(cx + 46, cy - 22);
  display.print('z');
}

// What the idle screen shows, in priority order: a typed message, then the
// pairing name while there is no link to answer that question any other way,
// and only when neither is needed, the sleeping face.
static void oled_draw_idle(void) {
  if (oled_text_active()) {
    draw_drifting_text(oled_text_get(), 0);
  } else if (!remotexy_get_connect_flag()) {
    draw_drifting_text(CONFIG_BLE_DEVICE_NAME, 0);
  } else {
    oled_draw_sleeping();
  }
}

static void panel_power(bool on) {
  if (on == !s_panel_off) return;          // never resend a command
  s_panel_off = !on;
  display.ssd1306_command(on ? SSD1306_DISPLAYON : SSD1306_DISPLAYOFF);
}

static void panel_dim(bool d) {
  if (d == s_dimmed) return;
  s_dimmed = d;
  display.dim(d);
}

// ===========================================================================
// CFG TRANSFER PROGRESS
// ===========================================================================
// Drawn from INSIDE sendCfg()'s burst, which is the only place it can be.
// That loop delays 50ms per chunk and does not return until the whole layout
// has gone out -- twenty seconds for the Expert panel on a peer that never
// negotiated its MTU up -- so loop() never runs and oled_update() is never
// called. Nothing was wrong with the display; nothing was asking it to draw.
//
// Deliberately NOT called for every chunk: display() pushes the whole 1KB
// buffer over I2C, which is about 25ms at 400kHz and would add half the
// transfer time again. sendCfg() calls this about twenty times across the
// burst, which is smooth enough to watch and costs well under a second.
void oled_draw_progress(const char* title, unsigned done, unsigned total) {
  if (!s_oled_present) return;   // and this one matters most: it is called
                                 // from inside sendCfg()'s blocking burst,
                                 // so a dead bus here slows the transfer
                                 // the progress bar is meant to be showing.
  if (total == 0) return;
  // A transfer is activity: it must never sleep or dim part-way through, and
  // if the panel had already gone dark this is what brings it back.
  oled_note_activity();
  panel_power(true);
  panel_dim(false);

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(2, 4);
  display.print("Loading panel");
  display.setCursor(2, 16);
  display.print(title);

  const int16_t bx = 4, bw = SCREEN_WIDTH - 8;
  display.drawRect(bx, 32, bw, 14, SSD1306_WHITE);
  const int16_t fill = (int16_t)(((uint32_t)(bw - 4) * done) / total);
  if (fill > 0) display.fillRect(bx + 2, 34, fill, 10, SSD1306_WHITE);

  char buf[24];
  snprintf(buf, sizeof(buf), "%u / %u", done, total);
  int16_t x1, y1; uint16_t w, h;
  display.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - (int16_t)w) / 2, 52);
  display.print(buf);
  display.display();
}

// ---- the app's selectors --------------------------------------------------
void oled_screen_mode_set(const char* v) {
  if      (strcmp(v, "Face")  == 0) s_screen_mode = OLED_SCREEN_FACE;
  else if (strcmp(v, "Auto")  == 0) s_screen_mode = OLED_SCREEN_AUTO;
  else if (strcmp(v, "Radar") == 0) s_screen_mode = OLED_SCREEN_RADAR;
  else                              s_screen_mode = OLED_SCREEN_STATUS;
}

const char* oled_screen_mode_name(void) {
  if (s_screen_mode == OLED_SCREEN_FACE)  return "Face";
  if (s_screen_mode == OLED_SCREEN_AUTO)  return "Auto";
  if (s_screen_mode == OLED_SCREEN_RADAR) return "Radar";
  return "Status";
}

const char* oled_face_style_name(void) {
  if (s_face_style == OLED_STYLE_CIRCLE) return "Circle";
  if (s_face_style == OLED_STYLE_ROBOT)  return "Robot";
  if (s_face_style == OLED_STYLE_BIG)    return "Big";
  if (s_face_style == OLED_STYLE_VISOR)  return "Visor";
  return "Round";
}

void oled_eyes_follow_set(bool on) { s_eyes_follow = on; }
bool oled_eyes_follow_get(void)     { return s_eyes_follow; }

void oled_face_style_set(const char* v) {
  if      (strcmp(v, "Circle") == 0) s_face_style = OLED_STYLE_CIRCLE;
  else if (strcmp(v, "Robot")  == 0) s_face_style = OLED_STYLE_ROBOT;
  else if (strcmp(v, "Big")    == 0) s_face_style = OLED_STYLE_BIG;
  else if (strcmp(v, "Visor")  == 0) s_face_style = OLED_STYLE_VISOR;
  else                               s_face_style = OLED_STYLE_ROUND;
}

// The only question worth answering before a link exists is WHICH robot this
// is: the browser's chooser lists them all alike, so a robot showing eyes or a
// sweeping scope is withholding the one fact you need to pick it out. Both
// pictures therefore wait for a connection -- the rover's rule, and the status
// screen behind them is already showing the BLE name.
static bool face_wanted(void) {
  if (!remotexy_get_connect_flag()) return false;
  return s_screen_mode == OLED_SCREEN_FACE || s_screen_mode == OLED_SCREEN_AUTO;
}

static bool radar_wanted(void) {
  return s_screen_mode == OLED_SCREEN_RADAR && remotexy_get_connect_flag();
}

// ===========================================================================
// The existing readout, now one mode among four.
// ===========================================================================
static void oled_draw_status(void) {
// ===========================================================================
int16_t x1, y1;
uint16_t w, h;
uint8_t l_count;
  // Line 1: whatever the app has sent, otherwise the usual banner.
  // Custom text is drawn at size 1 rather than 2 — the app can send far more
  // characters than "Workshop.3", and at size 2 anything past ~10 would run
  // off the panel silently. The rest of the screen (distance, uptime,
  // battery) is untouched either way.
  display.setCursor(0, 0);

  if( oled_text_active() ) {
    display.setTextSize(1);
    display.print(oled_text_get());
  } else if( remotexy_get_connect_flag() ) {
    display.setTextSize(2);
    display.print("Workshop.3");
  } else {
    display.setTextSize(2);
    memset(g_display_array, 0,SIZE_BUFFER);
    sprintf(g_display_array, "%s", CONFIG_BLE_DEVICE_NAME);

    display.print(g_display_array);
  }

  // Line 3: Centered distance
  display.setTextSize(2);

  memset(g_display_array, 0,SIZE_BUFFER);
  sprintf( g_display_array, "%lu cm", g_ultrasonic_distance_cm);
  //sprintf( g_display_array, "%3u %3u %u", g_speed_s2, g_speed_s1);

  display.getTextBounds(g_display_array, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, 32);
  display.print(g_display_array);

  // Bottom-right: time
  display.setTextSize(1);    

  memset(g_display_array, 0,SIZE_BUFFER);
  sprintf(g_display_array, "%02u:%02u:%02u", g_elapsed_time_hours, g_elapsed_time_minutes, g_elapsed_time_seconds);
  
  display.getTextBounds(g_display_array, 0, 0, &x1, &y1, &w, &h);
  display.setCursor(SCREEN_WIDTH - w - 2, SCREEN_HEIGHT - h);
  display.print(g_display_array);

  // Battery indicator (bottom-left)
  display.setTextSize(1);
  display.setCursor(0, SCREEN_HEIGHT - h);
  display.print(String(g_battery_voltage, 1) + "V (" + String((int32_t)calculateBatteryPercentage(g_battery_voltage)) + "%)");
}

// ===========================================================================
void oled_update( ) {
// ===========================================================================
  if (!s_oled_present) return;   // no screen on the bus -- see oled_set_present()

  const uint32_t now = millis();

  // Wheels turning and the button are the two deliberate acts that do not
  // arrive as widget commands, so they are counted here rather than expecting
  // every caller to remember.
  if (g_speed_s1 != CONFIG_SERVO_SPEED_STOP_LEFT ||
      g_speed_s2 != CONFIG_SERVO_SPEED_STOP_RIGHT || button_pressed()) {
    oled_note_activity();
  }
  const uint32_t idle = now - s_activity_at;
  // The radar is a live instrument: blanking it mid-sweep would read as a
  // crash rather than as sleep, so it is never dimmed and never slept.
  const bool live = radar_wanted();

  if (!live && idle >= IDLE_OFF_MS) {
    panel_power(false);
    return;                       // nothing to draw, and no I2C spent drawing it
  }
  panel_power(true);
  panel_dim(!live && idle >= IDLE_DIM_MS);

  display.clearDisplay();

  if (!live && idle >= IDLE_SLEEP_MS) {
    drift_tick(now);
    oled_draw_idle();
    display.display();
    return;
  }
  // A typed message outranks both pictures, as it does on the rover: someone
  // sent that text to be read, and neither eyes nor a scope can carry it.
  if (oled_text_active()) {
    oled_draw_status();
  } else if (radar_wanted()) {
    oled_draw_radar(now);
  } else if (face_wanted()) {
    oled_draw_face(now);
  } else {
    oled_draw_status();
  }
  display.display();
}
