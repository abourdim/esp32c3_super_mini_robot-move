#ifndef __DEF_INCLUDE_OLED_H__
#define __DEF_INCLUDE_OLED_H__

extern void oled_init(void);

// Whether an SSD1306 actually answered on the I2C bus at boot. b3 always has
// a screen soldered on, so it never needed to ask; on boards where the OLED
// is optional this has to gate every draw. Writing a frame to an absent
// device is not a no-op -- it is ~1KB of transactions that each burn the
// Wire timeout, which at the 100ms update cadence starves loop() badly
// enough that the robot stops answering GETCFGVER and the app hangs on
// "Checking layout version".
extern void oled_set_present(bool present);
extern bool oled_present(void);

// Text pushed from the app's OLED field, drawn as the screen's top line.
// Empty means "no override" and the usual banner is shown instead.
extern bool        oled_text_active(void);
extern const char* oled_text_get(void);
extern void        oled_text_set(const char* s);
extern void oled_update(void);

// Screen mode: "Status" / "Face" / "Auto" / "Radar", from the app's selector.
extern void        oled_screen_mode_set(const char* v);
extern const char* oled_screen_mode_name(void);
// Face style: "Round" / "Circle" / "Robot" / "Big" / "Visor".
extern void        oled_face_style_set(const char* v);
extern const char* oled_face_style_name(void);
// Whether the pupils track the driving direction while the wheels turn.
extern void        oled_eyes_follow_set(bool on);
extern bool        oled_eyes_follow_get(void);
// Tells the face something arrived, so it does not fall asleep mid-session
// on a robot that is being driven by widgets rather than by its wheels.
extern void        oled_note_command(void);
// Any deliberate act: a widget command, the button, or the wheels turning.
// Resets the idle clock behind dim / sleep / panel-off.
extern void        oled_note_activity(void);

// Drawn from inside sendCfg()'s blocking burst, which is the only place it
// can be: that loop never returns to loop(), so oled_update() cannot run.
extern void        oled_draw_progress(const char* title, unsigned done, unsigned total);

#endif // __DEF_INCLUDE_OLED_H__