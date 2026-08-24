#include "01_includes.h"

// ===========================================================================
// BLE control layer — speaks the same protocol as a BBC micro:bit running
// the rxy MakeCode template (https://abourdim.github.io/bit-rxy/), so the
// unmodified web app can drive this robot's widgets over Web Bluetooth.
//
// The function names below are kept identical to the old RemoteXY adapter
// so none of the calling code (04_tasks.cpp, 01_src.ino, 11_events.cpp) had
// to change — only what's behind them did.
//
// Protocol summary:
//   app -> device : "GETCFG"            (sent ~500ms after connect)
//   app -> device : "SET <id> <val...>"
//   device -> app : "CFGBEGIN" / "CFG <chunk>"... / "CFGEND"   (chunk size is
//                   negotiated from the MTU — see sendCfg())
//   device -> app : "UPD <id> <val>"
//
// Two apps drive this robot and they do not agree on the D-pad or the
// joystick, so both encodings of each are accepted. Stock bit-rxy's forms are
// listed first and are handled exactly as before — nothing here changes them:
//
//   D-pad     "SET <id> <up|down|left|right> <0|1>"   bit-rxy, one direction
//             "<a..p>"                                mecanum app, 1-byte mask
//             "M <0..15>"                             text form of that mask
//   joystick  "SET <id> <angle> <distance>"           bit-rxy
//             "SET <id> <x> <y>"                      mecanum app
//
// The two D-pad forms are unambiguous (a mask line is one byte, or starts with
// "M "). The joystick pair is not, so see handleJoystick() for how it is told
// apart. "GETCFGVER" is answered with a CFGVER token so a phone that already
// has this layout skips the transfer entirely; it also records which app is
// talking. See handleLine().
// Transport: Nordic-UART-style GATT service, roles reversed to match the
// micro:bit's convention (0002 = notify device->app, 0003 = write app->device).
// ===========================================================================

#include <NimBLEDevice.h>
#include <Preferences.h>   // NVS-backed store for the Beginner/Expert choice

#define UART_SERVICE_UUID   "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define UART_TX_CHAR_UUID   "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  // notify
#define UART_RX_CHAR_UUID   "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  // write

// Layout CFG: two layouts are compiled in, and the Level select in the SYSTEM
// zone switches between them at runtime (see handleWidget's "level" case).
// Their sources are 01_app/layout_beginner.json and layout_expert.json —
// regenerate either with
//   ./layout_cfg.sh encode layout_expert.json
//   ./layout_cfg.sh decode LAYOUT_CFG_EXPERT_BASE64
// and paste the output over the matching chunks below.
//
//   BEGINNER  3 zones, 9 controls, canvas 818x760
//     DRIVE    (#00d4ff)  joy_drive, btn_horn, dpad_drive
//     SENSING  (#ffb020)  gauge_speed, gauge_distance, sound_alert
//     SYSTEM   (#3ddc97)  battery_level, level, logo
//
//   EXPERT    5 zones, 28 controls, canvas 1230x1270
//     DRIVE     (#00d4ff)  joy_drive, dpad_drive, spd, btn_stop, gauge_speed
//     DISTANCE  (#ffb020)  gauge_distance, alert, graph_dist
//     LIGHTS    (#7c5cff)  toggle_led_r, toggle_led_g, toggle_np, np_effect,
//                          np_r, np_g, np_b, np_bright
//     SOUND     (#ff5c8a)  btn_horn, btn_buzz, sound_alert
//     SYSTEM    (#3ddc97)  battery_level, lbl_vbat, lbl_ver, lbl_uptime,
//                          upd, level, led_button, gauge_rssi, logo
//
// Every id in the beginner layout also exists in the expert one with the same
// type and meaning, so switching never changes what a control does — it only
// reveals more of them. `level` itself is in both, which is what makes the
// switch reversible.
//
// Groups and the three separators are visual-only: they carry no state, are
// never SET or UPD targets, and the controls stay ordinary top-level CFG
// widgets, so a client that does not know these types can ignore the grouping
// metadata. Groups are listed first so they render behind the controls; each
// control also carries groupId so a zone drags as one unit.
//
// Not mirrored from the Keyes 4WD layout: an AUTONOMY zone. This robot has no
// line sensors and no avoid mode, so a Mode selector would be decoration.
//
// The Red/Green LED toggles need care, because unlike the Keyes robot's
// headlights these two pins are the link-status indicator (leds_update()).
// The app therefore does not own them by default: the first toggle takes
// ownership, and onDisconnect() gives it back, so a disconnect is always
// visible even if the user left both LEDs switched off.
//
// The logo is referenced as a path relative to the app's index.html, not
// embedded. The CFG is base64'd twice on its way to the browser, so an inline
// data URI would cost ~1.78x the raw file in both flash and transfer — the
// 16KB Workshop-DIY asset would add ~29KB of CFG. As a path it costs ~40
// bytes, and the artwork can be swapped without re-encoding or reflashing.
// An app copy that lacks assets/workshop-diy-logo.svg simply shows the
// widget's label instead.
//
// Cost: beginner 2352 base64 bytes, expert 6384. At the old fixed CHUNK=18
// the expert layout would be 355 chunks and ~18.1s per connect; sendCfg() now
// sizes chunks to the negotiated MTU, so a desktop browser (MTU ~247) gets
// 180-byte chunks — 14 of them (~1.0s) for beginner, 36 (~2.1s) for expert.
// A peer that only grants the 23-byte minimum still falls back to 18 and the
// old timing. That headroom is also what makes switching levels cheap enough
// to re-send the whole CFG live instead of asking for a reconnect.
//
// Widget -> firmware mapping:
//   level                   -> picks which layout sendCfg() serves; persisted
//                              in NVS, re-sends the CFG on change
//   joy_drive / dpad_drive  -> both write the same s_joy_x/s_joy_y (steering)
//   spd                     -> scales the drive mix (remotexy_get_speed_cap)
//   btn_stop                -> clears inputs and calls stopServos()
//   btn_horn                -> horn/alarm (s_button_01)
//   btn_buzz                -> one buzzer_beep() per press
//   toggle_np / np_effect / np_bright
//                           -> strip state, dispatched from loop()
//   np_r / np_g / np_b      -> one channel each, recombined into the packed
//                              0xRRGGBB that remotexy_get_np_color() returns
//   toggle_led_r / toggle_led_g
//                           -> take the board LEDs over from leds_update()
//   led_button              -> board push button, mirrored as an indicator
//   gauge_rssi              -> BLE link strength via ble_gap_conn_rssi()
//   lbl_vbat                -> raw pack voltage alongside battery_level's %
//   upd                     -> telemetry verbosity gate (Off/Basic/All)
//
// Telemetry is sent ON CHANGE, with a per-value deadband, not once per cycle:
// the expert layout would otherwise push 11 notifications every pass for
// values that mostly sit still, and this radio has history with rc=6 under
// notify() pressure. graph_dist is the deliberate exception — a time series
// needs every sample or the trace shape lies. A full refresh is forced after
// every CFG transfer so a freshly rendered panel is never left blank.
//
//   gauge_speed             -> motor speed magnitude 0-100 (output only)
//   gauge_distance          -> ultrasonic distance 0-200cm (output only)
//   graph_dist              -> same measurement as a history (All only)
//   alert                   -> obstacle toast, edge-triggered with hysteresis
//   battery_level           -> battery percentage 0-100 (output only)
//   lbl_ver / lbl_uptime    -> firmware version and uptime (All only)
//   sound_alert             -> plays a tone on the phone (output only) —
//                              restores the old RemoteXY sound_01 element,
//                              which had no bit-rxy equivalent until the
//                              Sound widget was added
static const char* LAYOUT_CFG_BEGINNER_BASE64 =
  "eyJzY2hlbWFWZXJzaW9uIjoxLCJ0aXRsZSI6IldESVkgUm9ib3QgYjMiLCJjYW52YXMiOnsi"
  "dyI6ODE4LCJoIjo5MzB9LCJ3aWRnZXRzIjpbeyJpZCI6ImdycF9kcml2ZSIsInQiOiJncm91"
  "cCIsImxhYmVsIjoiRFJJVkUiLCJ4Ijo1NiwieSI6NjAsInciOjUwOCwiaCI6Mzc0LCJjb2xv"
  "ciI6IiMwMGQ0ZmYiLCJjaGlsZHJlbiI6WyJkcGFkX2RyaXZlIiwiam95X2RyaXZlIiwiYnRu"
  "X2hvcm4iXX0seyJpZCI6ImdycF9zZW5zZSIsInQiOiJncm91cCIsImxhYmVsIjoiU0VOU0lO"
  "RyIsIngiOjU2LCJ5Ijo1MDAsInciOjQ1OCwiaCI6MjI0LCJjb2xvciI6IiNmZmIwMjAiLCJj"
  "aGlsZHJlbiI6WyJnYXVnZV9zcGVlZCIsImdhdWdlX2Rpc3RhbmNlIiwic291bmRfYWxlcnQi"
  "XX0seyJpZCI6ImdycF9zeXMiLCJ0IjoiZ3JvdXAiLCJsYWJlbCI6IlNZU1RFTSIsIngiOjU1"
  "NCwieSI6NTMwLCJ3IjoyMDgsImgiOjM0NCwiY29sb3IiOiIjM2RkYzk3IiwiY2hpbGRyZW4i"
  "OlsiYmF0dGVyeV9sZXZlbCIsImxldmVsIiwibG9nbyJdfSx7ImlkIjoic2VwX2JhbmQiLCJ0"
  "Ijoic2VwYXJhdG9yIiwieCI6ODAsInkiOjQ2OCwidyI6NTAwLCJoIjo4fSx7ImlkIjoiZHBh"
  "ZF9kcml2ZSIsInQiOiJkcGFkIiwieCI6ODAsInkiOjEwMCwidyI6MjYwLCJoIjoyNjAsImxh"
  "YmVsIjoiRHJpdmUiLCJtb2RlbCI6ImNsYXNzaWMiLCJncm91cElkIjoiZ3JwX2RyaXZlIn0s"
  "eyJpZCI6ImpveV9kcml2ZSIsInQiOiJqb3lzdGljayIsIngiOjM4MCwieSI6MTAwLCJ3Ijox"
  "NjAsImgiOjE2MCwibGFiZWwiOiJEcml2ZSIsIm1vZGVsIjoiY2xhc3NpYyIsImdyb3VwSWQi"
  "OiJncnBfZHJpdmUifSx7ImlkIjoiYnRuX2hvcm4iLCJ0IjoiYnV0dG9uIiwieCI6MzgwLCJ5"
  "IjoyOTAsInciOjEyMCwiaCI6MTIwLCJsYWJlbCI6Ikhvcm4iLCJtb2RlbCI6Im5lbyIsImdy"
  "b3VwSWQiOiJncnBfZHJpdmUifSx7ImlkIjoiZ2F1Z2Vfc3BlZWQiLCJ0IjoiZ2F1Z2UiLCJ4"
  "Ijo4MCwieSI6NTQwLCJ3IjoxNDAsImgiOjE2MCwibGFiZWwiOiJTcGVlZCIsIm1pbiI6MCwi"
  "bWF4IjoxMDAsInVuaXRzIjoiJSIsImRlY2ltYWxzIjowLCJtb2RlbCI6ImNsYXNzaWMiLCJn"
  "cm91cElkIjoiZ3JwX3NlbnNlIn0seyJpZCI6ImdhdWdlX2Rpc3RhbmNlIiwidCI6ImdhdWdl"
  "IiwieCI6MjQwLCJ5Ijo1NDAsInciOjE0MCwiaCI6MTYwLCJsYWJlbCI6IkRpc3RhbmNlIiwi"
  "bWluIjowLCJtYXgiOjIwMCwidW5pdHMiOiJjbSIsImRlY2ltYWxzIjowLCJtb2RlbCI6ImNs"
  "YXNzaWMiLCJncm91cElkIjoiZ3JwX3NlbnNlIn0seyJpZCI6InNvdW5kX2FsZXJ0IiwidCI6"
  "InNvdW5kIiwieCI6NDAwLCJ5Ijo1NzUsInciOjkwLCJoIjo5MCwibGFiZWwiOiJBbGVydCIs"
  "Imdyb3VwSWQiOiJncnBfc2Vuc2UifSx7ImlkIjoiYmF0dGVyeV9sZXZlbCIsInQiOiJiYXR0"
  "ZXJ5IiwieCI6NTc4LCJ5Ijo1NzAsInciOjgwLCJoIjoxMDAsImxhYmVsIjoiQmF0dGVyeSIs"
  "Im1vZGVsIjoidmVydGljYWwiLCJncm91cElkIjoiZ3JwX3N5cyJ9LHsiaWQiOiJsZXZlbCIs"
  "InQiOiJzZWxlY3QiLCJ4Ijo1NzgsInkiOjcwMCwidyI6MTYwLCJoIjo3MCwibGFiZWwiOiJM"
  "ZXZlbCIsIm9wdGlvbnMiOiJCZWdpbm5lcixFeHBlcnQsTW90b3JzLERpc3RhbmNlLExpZ2h0"
  "cyxTb3VuZCxEaXNwbGF5LFBvd2VyIiwiZ3JvdXBJZCI6ImdycF9zeXMifSx7ImlkIjoibG9n"
  "byIsInQiOiJpbWFnZSIsIngiOjU3OCwieSI6NzkwLCJ3IjoxNjAsImgiOjYwLCJsYWJlbCI6"
  "IldvcmtzaG9wLURJWSIsImltYWdlU3JjIjoiYXNzZXRzL3dvcmtzaG9wLWRpeS1sb2dvLnN2"
  "ZyIsImdyb3VwSWQiOiJncnBfc3lzIn1dfQ==";

static const char* LAYOUT_CFG_EXPERT_BASE64 =
  
  
  
  
  
  
  
  
  
  "eyJ0aXRsZSI6IldESVkgUm9ib3QgYjMiLCJ3aWRnZXRzIjpbeyJpZCI6ImdycF9kcml2ZSIs"
  "InQiOiJncm91cCIsImxhYmVsIjoiRFJJVkUiLCJ4IjoxOSwieSI6MTcsInciOjY5MSwiaCI6"
  "NTUxLCJjb2xvciI6IiMwMGQ0ZmYiLCJjaGlsZHJlbiI6WyJkcGFkX2RyaXZlIiwiam95X2Ry"
  "aXZlIiwic3BkIiwiYnRuX3N0b3AiLCJnYXVnZV9zcGVlZCJdLCJtb2RlbCI6InBhbmVsIiwi"
  "cGFkZGluZyI6MTh9LHsiaWQiOiJncnBfZGlzdCIsInQiOiJncm91cCIsImxhYmVsIjoiRElT"
  "VEFOQ0UiLCJ4Ijo3NDQsInkiOjIxLCJ3Ijo0NjcsImgiOjU1OSwiY29sb3IiOiIjZmZiMDIw"
  "IiwiY2hpbGRyZW4iOlsiZ2F1Z2VfZGlzdGFuY2UiLCJhbGVydCIsImdyYXBoX2Rpc3QiXSwi"
  "bW9kZWwiOiJwYW5lbCIsInBhZGRpbmciOjE4fSx7ImlkIjoiZ3JwX2xpZ2h0IiwidCI6Imdy"
  "b3VwIiwibGFiZWwiOiJMSUdIVFMiLCJ4IjoxOSwieSI6NTg1LCJ3Ijo2ODksImgiOjM4NSwi"
  "Y29sb3IiOiIjN2M1Y2ZmIiwiY2hpbGRyZW4iOlsidG9nZ2xlX2xlZF9yIiwidG9nZ2xlX2xl"
  "ZF9nIiwidG9nZ2xlX25wIiwibnBfZWZmZWN0IiwibnBfciIsIm5wX2ciLCJucF9iIiwibnBf"
  "YnJpZ2h0IiwibGVkX3Jfc3RhdGUiLCJsZWRfZ19zdGF0ZSJdLCJtb2RlbCI6InBhbmVsIiwi"
  "cGFkZGluZyI6MTh9LHsiaWQiOiJncnBfc291bmQiLCJ0IjoiZ3JvdXAiLCJsYWJlbCI6IlNP"
  "VU5EIiwieCI6NzQxLCJ5Ijo2MzEsInciOjQ2OCwiaCI6MTg3LCJjb2xvciI6IiNmZjVjOGEi"
  "LCJjaGlsZHJlbiI6WyJidG5faG9ybiIsImJ0bl9idXp6Iiwic291bmRfYWxlcnQiXSwibW9k"
  "ZWwiOiJwYW5lbCIsInBhZGRpbmciOjE4fSx7ImlkIjoiZ3JwX3N5cyIsInQiOiJncm91cCIs"
  "ImxhYmVsIjoiU1lTVEVNIiwieCI6MTAsInkiOjEwMDYsInciOjExODYsImgiOjI1NCwiY29s"
  "b3IiOiIjM2RkYzk3IiwiY2hpbGRyZW4iOlsiYmF0dGVyeV9sZXZlbCIsImxibF92ZXIiLCJs"
  "YmxfdXB0aW1lIiwibGJsX3ZiYXQiLCJ1cGQiLCJsZXZlbCIsImxlZF9idXR0b24iLCJnYXVn"
  "ZV9yc3NpIiwibG9nbyJdLCJtb2RlbCI6InBhbmVsIiwicGFkZGluZyI6MTh9LHsiaWQiOiJn"
  "cnBfZGlzcGxheSIsInQiOiJncm91cCIsImxhYmVsIjoiRElTUExBWSIsIngiOjEzLCJ5Ijox"
  "Mjc4LCJ3IjoxMTkxLCJoIjoyMjIsImNvbG9yIjoiIzAwZTVmZiIsImNoaWxkcmVuIjpbIm9s"
  "ZWRfdGV4dCIsImxibF9vbGVkIiwic2NyZWVuX21vZGUiLCJmYWNlX3N0eWxlIiwiZXllc19m"
  "b2xsb3ciXSwibW9kZWwiOiJwYW5lbCIsInBhZGRpbmciOjE4fSx7ImlkIjoic2VwX2NvbHMi"
  "LCJ0Ijoic2VwYXJhdG9yIiwieCI6NzE5LCJ5IjoyMiwidyI6MTAsImgiOjU0OCwibW9kZWwi"
  "OiJzdWJ0bGUiLCJvcmllbnRhdGlvbiI6InZlcnRpY2FsIiwidGhpY2tuZXNzIjoxfSx7Imlk"
  "Ijoic2VwX2IxIiwidCI6InNlcGFyYXRvciIsIngiOjc0MSwieSI6NTk4LCJ3Ijo0NjQsImgi"
  "OjEyLCJtb2RlbCI6InN1YnRsZSIsIm9yaWVudGF0aW9uIjoiaG9yaXpvbnRhbCIsInRoaWNr"
  "bmVzcyI6MX0seyJpZCI6InNlcF9iMiIsInQiOiJzZXBhcmF0b3IiLCJ4IjozMiwieSI6OTgw"
  "LCJ3IjoxMTc2LCJoIjoxMCwibW9kZWwiOiJzdWJ0bGUiLCJvcmllbnRhdGlvbiI6Imhvcml6"
  "b250YWwiLCJ0aGlja25lc3MiOjF9LHsiaWQiOiJkcGFkX2RyaXZlIiwidCI6ImRwYWQiLCJ4"
  "Ijo0MywieSI6NzUsInciOjQxNiwiaCI6NDc0LCJsYWJlbCI6IkRyaXZlIiwibW9kZWwiOiJj"
  "bGFzc2ljIiwiZ3JvdXBJZCI6ImdycF9kcml2ZSJ9LHsiaWQiOiJqb3lfZHJpdmUiLCJ0Ijoi"
  "am95c3RpY2siLCJ4Ijo0NzQsInkiOjcyLCJ3IjoyMjYsImgiOjIyMSwibGFiZWwiOiJEcml2"
  "ZSIsImdyb3VwSWQiOiJncnBfZHJpdmUiLCJtb2RlbCI6ImNsYXNzaWMifSx7ImlkIjoic3Bk"
  "IiwidCI6InNsaWRlciIsIngiOjYxNywieSI6MzAzLCJ3Ijo3MywiaCI6MjQ2LCJsYWJlbCI6"
  "Ik1heCBzcGVlZCIsIm1heCI6MTAwLCJ2YWx1ZSI6MTAwLCJncm91cElkIjoiZ3JwX2RyaXZl"
  "IiwibW9kZWwiOiJ0cmFjayIsIm1pbiI6MCwic3RlcCI6MX0seyJpZCI6ImJ0bl9zdG9wIiwi"
  "dCI6ImJ1dHRvbiIsIngiOjE4OCwieSI6MjQzLCJ3IjoxMjUsImgiOjEzOCwibGFiZWwiOiJT"
  "VE9QIiwibW9kZWwiOiJmbGF0IiwiZ3JvdXBJZCI6ImdycF9kcml2ZSJ9LHsiaWQiOiJnYXVn"
  "ZV9zcGVlZCIsInQiOiJnYXVnZSIsIngiOjQ2NCwieSI6Mzg4LCJ3IjoxNTAsImgiOjE2NCwi"
  "bGFiZWwiOiJTcGVlZCIsIm1pbiI6MCwibWF4IjoxMDAsInVuaXRzIjoiJSIsImRlY2ltYWxz"
  "IjowLCJtb2RlbCI6ImNsYXNzaWMiLCJncm91cElkIjoiZ3JwX2RyaXZlIn0seyJpZCI6Imdh"
  "dWdlX2Rpc3RhbmNlIiwidCI6ImdhdWdlIiwieCI6NzgxLCJ5IjozMywidyI6MTk1LCJoIjox"
  "NzMsImxhYmVsIjoiRGlzdGFuY2UiLCJtaW4iOjAsIm1heCI6MjAwLCJ1bml0cyI6ImNtIiwi"
  "ZGVjaW1hbHMiOjAsIm1vZGVsIjoiY2xhc3NpYyIsImdyb3VwSWQiOiJncnBfZGlzdCJ9LHsi"
  "aWQiOiJhbGVydCIsInQiOiJub3RpZmljYXRpb24iLCJ4IjoxMDQ3LCJ5Ijo5MCwidyI6OTAs"
  "ImgiOjkwLCJsYWJlbCI6Ik9ic3RhY2xlIiwiZ3JvdXBJZCI6ImdycF9kaXN0In0seyJpZCI6"
  "ImdyYXBoX2Rpc3QiLCJ0IjoiZ3JhcGgiLCJ4Ijo3NzAsInkiOjIzNCwidyI6NDMxLCJoIjoz"
  "MzMsImxhYmVsIjoiRGlzdGFuY2UgY20iLCJtYXgiOjIwMCwiZ3JvdXBJZCI6ImdycF9kaXN0"
  "IiwibW9kZWwiOiJncmlkIiwic2VyaWVzIjoxLCJ3aW5kb3dTZWMiOjMwLCJhdXRvU2NhbGUi"
  "OnRydWUsIm1pbiI6MCwic2hvd0xlZ2VuZCI6dHJ1ZX0seyJpZCI6InRvZ2dsZV9sZWRfciIs"
  "InQiOiJ0b2dnbGUiLCJ4Ijo0NCwieSI6NjIzLCJ3IjoxMDgsImgiOjEwMCwibGFiZWwiOiJS"
  "ZWQgTEVEIiwibW9kZWwiOiJwaWxsIiwiZ3JvdXBJZCI6ImdycF9saWdodCJ9LHsiaWQiOiJ0"
  "b2dnbGVfbGVkX2ciLCJ0IjoidG9nZ2xlIiwieCI6MTY4LCJ5Ijo2MjUsInciOjEwNywiaCI6"
  "MTAwLCJsYWJlbCI6IkdyZWVuIExFRCIsIm1vZGVsIjoicGlsbCIsImdyb3VwSWQiOiJncnBf"
  "bGlnaHQifSx7ImlkIjoidG9nZ2xlX25wIiwidCI6InRvZ2dsZSIsIngiOjQwMywieSI6NjIz"
  "LCJ3IjoxMDAsImgiOjEwMCwibGFiZWwiOiJTdHJpcCIsIm1vZGVsIjoicGlsbCIsImdyb3Vw"
  "SWQiOiJncnBfbGlnaHQifSx7ImlkIjoibnBfZWZmZWN0IiwidCI6InNlbGVjdCIsIngiOjUz"
  "NSwieSI6NjI3LCJ3IjoxNjAsImgiOjcwLCJsYWJlbCI6IlN0cmlwIGVmZmVjdCIsIm9wdGlv"
  "bnMiOiJTb2xpZCxSYWluYm93LEtuaWdodCBSaWRlcixEdWVsIGV5ZSxGcmVuY2ggZmxhZyIs"
  "Imdyb3VwSWQiOiJncnBfbGlnaHQifSx7ImlkIjoibnBfciIsInQiOiJzbGlkZXIiLCJ4Ijoz"
  "NDcsInkiOjc3MywidyI6NzAsImgiOjE4MCwibGFiZWwiOiJSIiwibWF4IjoyNTUsInZhbHVl"
  "IjoyNTUsImdyb3VwSWQiOiJncnBfbGlnaHQiLCJtb2RlbCI6InRyYWNrIiwibWluIjowLCJz"
  "dGVwIjoxfSx7ImlkIjoibnBfZyIsInQiOiJzbGlkZXIiLCJ4Ijo0MjUsInkiOjc3MSwidyI6"
  "NzAsImgiOjE4MCwibGFiZWwiOiJHIiwibWF4IjoyNTUsInZhbHVlIjowLCJncm91cElkIjoi"
  "Z3JwX2xpZ2h0IiwibW9kZWwiOiJ0cmFjayIsIm1pbiI6MCwic3RlcCI6MX0seyJpZCI6Im5w"
  "X2IiLCJ0Ijoic2xpZGVyIiwieCI6NTA2LCJ5Ijo3NzMsInciOjcwLCJoIjoxODAsImxhYmVs"
  "IjoiQiIsIm1heCI6MjU1LCJ2YWx1ZSI6MCwiZ3JvdXBJZCI6ImdycF9saWdodCIsIm1vZGVs"
  "IjoidHJhY2siLCJtaW4iOjAsInN0ZXAiOjF9LHsiaWQiOiJucF9icmlnaHQiLCJ0Ijoic2xp"
  "ZGVyIiwieCI6NjEyLCJ5Ijo3NzIsInciOjkwLCJoIjoxODAsImxhYmVsIjoiQnJpZ2h0bmVz"
  "cyIsIm1heCI6MjU1LCJzdGVwIjo1LCJ2YWx1ZSI6MTUsImdyb3VwSWQiOiJncnBfbGlnaHQi"
  "LCJtb2RlbCI6InRyYWNrIiwibWluIjowfSx7ImlkIjoiYnRuX2hvcm4iLCJ0IjoiYnV0dG9u"
  "IiwieCI6NzczLCJ5Ijo2ODQsInciOjEwMCwiaCI6MTAwLCJsYWJlbCI6Ikhvcm4iLCJncm91"
  "cElkIjoiZ3JwX3NvdW5kIiwibW9kZWwiOiJuZW8ifSx7ImlkIjoiYnRuX2J1enoiLCJ0Ijoi"
  "YnV0dG9uIiwieCI6OTI4LCJ5Ijo2ODIsInciOjEwMCwiaCI6MTAwLCJsYWJlbCI6IkJ1enoi"
  "LCJncm91cElkIjoiZ3JwX3NvdW5kIiwibW9kZWwiOiJuZW8ifSx7ImlkIjoic291bmRfYWxl"
  "cnQiLCJ0Ijoic291bmQiLCJ4IjoxMDc4LCJ5Ijo2ODMsInciOjkwLCJoIjo5MCwibGFiZWwi"
  "OiJBbGVydCIsImdyb3VwSWQiOiJncnBfc291bmQifSx7ImlkIjoiYmF0dGVyeV9sZXZlbCIs"
  "InQiOiJiYXR0ZXJ5IiwieCI6NTQ2LCJ5IjoxMTI2LCJ3Ijo5NSwiaCI6ODYsImxhYmVsIjoi"
  "QmF0dGVyeSIsIm1vZGVsIjoidmVydGljYWwiLCJncm91cElkIjoiZ3JwX3N5cyJ9LHsiaWQi"
  "OiJsYmxfdmVyIiwidCI6ImxhYmVsIiwieCI6MjUsInkiOjEwNTYsInciOjIwMCwiaCI6NTAs"
  "ImxhYmVsIjoiRmlybXdhcmUiLCJtb2RlbCI6ImNhcmQiLCJncm91cElkIjoiZ3JwX3N5cyJ9"
  "LHsiaWQiOiJsYmxfdXB0aW1lIiwidCI6ImxhYmVsIiwieCI6MjM1LCJ5IjoxMDU4LCJ3Ijoy"
  "MDAsImgiOjUwLCJsYWJlbCI6IlVwdGltZSIsIm1vZGVsIjoiY2FyZCIsImdyb3VwSWQiOiJn"
  "cnBfc3lzIn0seyJpZCI6ImxibF92YmF0IiwidCI6ImxhYmVsIiwieCI6NDUyLCJ5IjoxMDU4"
  "LCJ3IjoyMDAsImgiOjUwLCJsYWJlbCI6IkJhdHRlcnkgViIsIm1vZGVsIjoiY2FyZCIsImdy"
  "b3VwSWQiOiJncnBfc3lzIn0seyJpZCI6InVwZCIsInQiOiJzZWxlY3QiLCJ4IjoyNywieSI6"
  "MTEzNiwidyI6MTYwLCJoIjo3MCwibGFiZWwiOiJUZWxlbWV0cnkiLCJvcHRpb25zIjoiT2Zm"
  "LEJhc2ljLEFsbCIsImdyb3VwSWQiOiJncnBfc3lzIn0seyJpZCI6ImxldmVsIiwidCI6InNl"
  "bGVjdCIsIngiOjIzMywieSI6MTEzOCwidyI6MTYwLCJoIjo3MCwibGFiZWwiOiJMZXZlbCIs"
  "Im9wdGlvbnMiOiJCZWdpbm5lcixFeHBlcnQsTW90b3JzLERpc3RhbmNlLExpZ2h0cyxTb3Vu"
  "ZCxEaXNwbGF5LFBvd2VyIiwiZ3JvdXBJZCI6ImdycF9zeXMifSx7ImlkIjoibGVkX2J1dHRv"
  "biIsInQiOiJsZWQiLCJ4Ijo0MjksInkiOjExMzIsInciOjgyLCJoIjo3NywibGFiZWwiOiJC"
  "dXR0b24iLCJtb2RlbCI6ImRvdCIsImNvbG9yT24iOiIjMDBmZjg4IiwiZ3JvdXBJZCI6Imdy"
  "cF9zeXMiLCJjb2xvck9mZiI6IiMyYTJhM2EifSx7ImlkIjoiZ2F1Z2VfcnNzaSIsInQiOiJn"
  "YXVnZSIsIngiOjY2OCwieSI6MTA2NSwidyI6MjIxLCJoIjoxNzgsImxhYmVsIjoiU2lnbmFs"
  "IiwibWluIjotMTAwLCJtYXgiOi0zMCwidW5pdHMiOiJkQm0iLCJkZWNpbWFscyI6MCwibW9k"
  "ZWwiOiJjbGFzc2ljIiwiZ3JvdXBJZCI6ImdycF9zeXMifSx7ImlkIjoibG9nbyIsInQiOiJp"
  "bWFnZSIsIngiOjkwOSwieSI6MTA2NCwidyI6MjYxLCJoIjoxNzIsImxhYmVsIjoiV29ya3No"
  "b3AtRElZIiwiaW1hZ2VTcmMiOiJhc3NldHMvd29ya3Nob3AtZGl5LWxvZ28uc3ZnIiwiZ3Jv"
  "dXBJZCI6ImdycF9zeXMifSx7ImlkIjoib2xlZF90ZXh0IiwidCI6ImVkaXRmaWVsZCIsIngi"
  "OjI1LCJ5IjoxMjkxLCJ3IjoyNjUsImgiOjgwLCJsYWJlbCI6Ik9MRUQgdGV4dCIsInBsYWNl"
  "aG9sZGVyIjoiVHlwZSBmb3IgdGhlIHJvYm90IHNjcmVlbi4uLiIsImdyb3VwSWQiOiJncnBf"
  "ZGlzcGxheSJ9LHsiaWQiOiJsYmxfb2xlZCIsInQiOiJsYWJlbCIsIngiOjMwNCwieSI6MTI5"
  "MiwidyI6Mjc5LCJoIjo4MCwibGFiZWwiOiJPTEVEIHNob3dzIiwibW9kZWwiOiJjYXJkIiwi"
  "Z3JvdXBJZCI6ImdycF9kaXNwbGF5In0seyJpZCI6ImxlZF9yX3N0YXRlIiwidCI6ImxlZCIs"
  "IngiOjUyLCJ5Ijo3NTEsInciOjgwLCJoIjo4MCwibGFiZWwiOiJSZWQgc3RhdHVzIiwibW9k"
  "ZWwiOiJkb3QiLCJjb2xvck9uIjoiI2ZmNTI1MiIsImdyb3VwSWQiOiJncnBfbGlnaHQiLCJj"
  "b2xvck9mZiI6IiMyYTJhM2EifSx7ImlkIjoibGVkX2dfc3RhdGUiLCJ0IjoibGVkIiwieCI6"
  "MTc4LCJ5Ijo3NTQsInciOjgwLCJoIjo4MCwibGFiZWwiOiJHcmVlbiBzdGF0dXMiLCJtb2Rl"
  "bCI6ImRvdCIsImNvbG9yT24iOiIjMDBmZjg4IiwiZ3JvdXBJZCI6ImdycF9saWdodCIsImNv"
  "bG9yT2ZmIjoiIzJhMmEzYSJ9LHsiaWQiOiJzY3JlZW5fbW9kZSIsInQiOiJzZWxlY3QiLCJs"
  "YWJlbCI6IlNjcmVlbiIsIm9wdGlvbnMiOiJTdGF0dXMsRmFjZSxBdXRvLFJhZGFyIiwieCI6"
  "NTk3LCJ5IjoxMjk0LCJ3IjoyOTAsImgiOjc5LCJncm91cElkIjoiZ3JwX2Rpc3BsYXkifSx7"
  "ImlkIjoiZmFjZV9zdHlsZSIsInQiOiJzZWxlY3QiLCJsYWJlbCI6IkZhY2Ugc3R5bGUiLCJv"
  "cHRpb25zIjoiUm91bmQsQ2lyY2xlLFJvYm90LEJpZyxWaXNvciIsIngiOjg5NiwieSI6MTI5"
  "MSwidyI6Mjg5LCJoIjo4MiwiZ3JvdXBJZCI6ImdycF9kaXNwbGF5In0seyJpZCI6ImdycF90"
  "cmltIiwidCI6Imdyb3VwIiwibGFiZWwiOiJUUklNIiwiY29sb3IiOiIjZmZiMDIwIiwieCI6"
  "MTMsInkiOjE1MTcsInciOjExODksImgiOjI3NiwiY2hpbGRyZW4iOlsidHJpbV9sIiwidHJp"
  "bV9sX2RuIiwidHJpbV9sX3VwIiwidHJpbV9sX251bSIsInRyaW1fciIsInRyaW1fcl9kbiIs"
  "InRyaW1fcl91cCIsInRyaW1fcl9udW0iXSwibW9kZWwiOiJwYW5lbCIsInBhZGRpbmciOjE4"
  "fSx7ImlkIjoidHJpbV9sIiwidCI6InNsaWRlciIsIngiOjMwOCwieSI6MTUzNSwidyI6NzAs"
  "ImgiOjE2MCwibGFiZWwiOiJUcmltIEwiLCJtaW4iOi0yMCwibWF4IjoyMCwic3RlcCI6MSwi"
  "dmFsdWUiOjAsImdyb3VwSWQiOiJncnBfdHJpbSIsIm1vZGVsIjoidHJhY2sifSx7ImlkIjoi"
  "dHJpbV9sX2RuIiwidCI6ImJ1dHRvbiIsIngiOjE4MywieSI6MTU5MSwidyI6OTAsImgiOjcw"
  "LCJsYWJlbCI6IkwgLSAxIiwiZ3JvdXBJZCI6ImdycF90cmltIiwibW9kZWwiOiJuZW8ifSx7"
  "ImlkIjoidHJpbV9sX3VwIiwidCI6ImJ1dHRvbiIsIngiOjQwNCwieSI6MTU4NCwidyI6OTAs"
  "ImgiOjcwLCJsYWJlbCI6IkwgKyAxIiwiZ3JvdXBJZCI6ImdycF90cmltIiwibW9kZWwiOiJu"
  "ZW8ifSx7ImlkIjoidHJpbV9sX251bSIsInQiOiJlZGl0ZmllbGQiLCJ4IjoyMzUsInkiOjE3"
  "MDEsInciOjIwMSwiaCI6ODMsImxhYmVsIjoiTCA9IiwiZ3JvdXBJZCI6ImdycF90cmltIiwi"
  "cGxhY2Vob2xkZXIiOiJUeXBlIGhlcmUuLi4ifSx7ImlkIjoidHJpbV9yIiwidCI6InNsaWRl"
  "ciIsIngiOjg0MiwieSI6MTUzMywidyI6NzAsImgiOjE2MCwibGFiZWwiOiJUcmltIFIiLCJt"
  "aW4iOi0yMCwibWF4IjoyMCwic3RlcCI6MSwidmFsdWUiOjAsImdyb3VwSWQiOiJncnBfdHJp"
  "bSIsIm1vZGVsIjoidHJhY2sifSx7ImlkIjoidHJpbV9yX2RuIiwidCI6ImJ1dHRvbiIsIngi"
  "OjcyOSwieSI6MTU4MSwidyI6OTAsImgiOjcwLCJsYWJlbCI6IlIgLSAxIiwiZ3JvdXBJZCI6"
  "ImdycF90cmltIiwibW9kZWwiOiJuZW8ifSx7ImlkIjoidHJpbV9yX3VwIiwidCI6ImJ1dHRv"
  "biIsIngiOjkzOCwieSI6MTU4NCwidyI6OTAsImgiOjcwLCJsYWJlbCI6IlIgKyAxIiwiZ3Jv"
  "dXBJZCI6ImdycF90cmltIiwibW9kZWwiOiJuZW8ifSx7ImlkIjoidHJpbV9yX251bSIsInQi"
  "OiJlZGl0ZmllbGQiLCJ4Ijo3ODAsInkiOjE3MDQsInciOjE5NiwiaCI6ODMsImxhYmVsIjoi"
  "UiA9IiwiZ3JvdXBJZCI6ImdycF90cmltIiwicGxhY2Vob2xkZXIiOiJUeXBlIGhlcmUuLi4i"
  "fSx7ImlkIjoic2VwYXJhdG9yMTAiLCJ0Ijoic2VwYXJhdG9yIiwieCI6NTkwLCJ5IjoxNTM2"
  "LCJ3Ijo0MCwiaCI6MjQwLCJtb2RlbCI6InN1YnRsZSIsIm9yaWVudGF0aW9uIjoidmVydGlj"
  "YWwiLCJ0aGlja25lc3MiOjF9LHsiaWQiOiJleWVzX2ZvbGxvdyIsInQiOiJ0b2dnbGUiLCJ4"
  "IjoyNSwieSI6MTQwMCwidyI6MjIwLCJoIjo5MCwibGFiZWwiOiJFeWVzIGZvbGxvdyIsIm1v"
  "ZGVsIjoicGlsbCIsImdyb3VwSWQiOiJncnBfZGlzcGxheSJ9XSwiY2FudmFzIjp7InciOjEy"
  "MzEsImgiOjE4MzN9fQ==";

static const char* LAYOUT_CFG_TEST_MOTORS_BASE64 =
  
  
  
  
  
  "eyJzY2hlbWFWZXJzaW9uIjoxLCJ0aXRsZSI6ImIzIC0gTW90b3JzIHRlc3QiLCJjYW52YXMi"
  "OnsidyI6ODAwLCJoIjo5NzB9LCJ3aWRnZXRzIjpbeyJpZCI6ImdycF90ZXN0IiwidCI6Imdy"
  "b3VwIiwibGFiZWwiOiJNT1RPUlMiLCJjb2xvciI6IiMwMGQ0ZmYiLCJ4Ijo1NiwieSI6NDIs"
  "InciOjY4OCwiaCI6NzUyLCJjaGlsZHJlbiI6WyJkcGFkX2RyaXZlIiwic3BkIiwiYnRuX3N0"
  "b3AiLCJnYXVnZV9zcGVlZCIsImxldmVsIiwidHJpbV9sIiwidHJpbV9sX2RuIiwidHJpbV9s"
  "X3VwIiwidHJpbV9sX251bSIsInRyaW1fciIsInRyaW1fcl9kbiIsInRyaW1fcl91cCIsInRy"
  "aW1fcl9udW0iXX0seyJpZCI6ImRwYWRfZHJpdmUiLCJ0IjoiZHBhZCIsIngiOjgwLCJ5Ijox"
  "MDAsInciOjMwMCwiaCI6MzAwLCJsYWJlbCI6IkRyaXZlIiwibW9kZWwiOiJjbGFzc2ljIiwi"
  "Z3JvdXBJZCI6ImdycF90ZXN0In0seyJpZCI6InNwZCIsInQiOiJzbGlkZXIiLCJ4Ijo0MjAs"
  "InkiOjEwMCwidyI6OTAsImgiOjIwMCwibGFiZWwiOiJNYXggc3BlZWQiLCJtYXgiOjEwMCwi"
  "dmFsdWUiOjEwMCwiZ3JvdXBJZCI6ImdycF90ZXN0In0seyJpZCI6ImJ0bl9zdG9wIiwidCI6"
  "ImJ1dHRvbiIsIngiOjQyMCwieSI6MzMwLCJ3IjoxMjAsImgiOjEyMCwibGFiZWwiOiJTVE9Q"
  "IiwibW9kZWwiOiJmbGF0IiwiZ3JvdXBJZCI6ImdycF90ZXN0In0seyJpZCI6ImdhdWdlX3Nw"
  "ZWVkIiwidCI6ImdhdWdlIiwieCI6NTcwLCJ5IjoxMDAsInciOjE1MCwiaCI6MTkwLCJsYWJl"
  "bCI6IlNwZWVkIiwibWF4IjoxMDAsInVuaXRzIjoiJSIsImRlY2ltYWxzIjowLCJncm91cElk"
  "IjoiZ3JwX3Rlc3QifSx7ImlkIjoibGV2ZWwiLCJ0Ijoic2VsZWN0IiwieCI6ODAsInkiOjcw"
  "MCwidyI6MjAwLCJoIjo3MCwibGFiZWwiOiJUZXN0Iiwib3B0aW9ucyI6IkJlZ2lubmVyLEV4"
  "cGVydCxNb3RvcnMsRGlzdGFuY2UsTGlnaHRzLFNvdW5kLERpc3BsYXksUG93ZXIiLCJncm91"
  "cElkIjoiZ3JwX3Rlc3QifSx7ImlkIjoibGJsX2hpbnQiLCJ0IjoibGFiZWwiLCJ4Ijo4MCwi"
  "eSI6ODEwLCJ3Ijo2NDAsImgiOjEyMCwibGFiZWwiOiJQcmVzcyBhbiBhcnJvdy4gVGhlIHdo"
  "ZWVscyBzaG91bGQgdHVybiB0aGF0IHdheS4iLCJ2YWx1ZSI6IlByZXNzIGFuIGFycm93LiBU"
  "aGUgd2hlZWxzIHNob3VsZCB0dXJuIHRoYXQgd2F5LiIsIm1vZGVsIjoiY2FyZCJ9LHsiaWQi"
  "OiJ0cmltX2wiLCJ0Ijoic2xpZGVyIiwieCI6ODAsInkiOjQ3MCwidyI6NzAsImgiOjE2MCwi"
  "bGFiZWwiOiJUcmltIEwiLCJtaW4iOi0yMCwibWF4IjoyMCwic3RlcCI6MSwidmFsdWUiOjAs"
  "Imdyb3VwSWQiOiJncnBfdGVzdCJ9LHsiaWQiOiJ0cmltX2xfZG4iLCJ0IjoiYnV0dG9uIiwi"
  "eCI6MTY1LCJ5Ijo0NzAsInciOjkwLCJoIjo3MCwibGFiZWwiOiJMIC0gMSIsImdyb3VwSWQi"
  "OiJncnBfdGVzdCJ9LHsiaWQiOiJ0cmltX2xfdXAiLCJ0IjoiYnV0dG9uIiwieCI6MTY1LCJ5"
  "Ijo1NjAsInciOjkwLCJoIjo3MCwibGFiZWwiOiJMICsgMSIsImdyb3VwSWQiOiJncnBfdGVz"
  "dCJ9LHsiaWQiOiJ0cmltX2xfbnVtIiwidCI6ImVkaXRmaWVsZCIsIngiOjI2NSwieSI6NTE1"
  "LCJ3IjoxMTAsImgiOjcwLCJsYWJlbCI6IkwgPSIsImdyb3VwSWQiOiJncnBfdGVzdCJ9LHsi"
  "aWQiOiJ0cmltX3IiLCJ0Ijoic2xpZGVyIiwieCI6NDIwLCJ5Ijo0NzAsInciOjcwLCJoIjox"
  "NjAsImxhYmVsIjoiVHJpbSBSIiwibWluIjotMjAsIm1heCI6MjAsInN0ZXAiOjEsInZhbHVl"
  "IjowLCJncm91cElkIjoiZ3JwX3Rlc3QifSx7ImlkIjoidHJpbV9yX2RuIiwidCI6ImJ1dHRv"
  "biIsIngiOjUwNSwieSI6NDcwLCJ3Ijo5MCwiaCI6NzAsImxhYmVsIjoiUiAtIDEiLCJncm91"
  "cElkIjoiZ3JwX3Rlc3QifSx7ImlkIjoidHJpbV9yX3VwIiwidCI6ImJ1dHRvbiIsIngiOjUw"
  "NSwieSI6NTYwLCJ3Ijo5MCwiaCI6NzAsImxhYmVsIjoiUiArIDEiLCJncm91cElkIjoiZ3Jw"
  "X3Rlc3QifSx7ImlkIjoidHJpbV9yX251bSIsInQiOiJlZGl0ZmllbGQiLCJ4Ijo2MDUsInki"
  "OjUxNSwidyI6MTEwLCJoIjo3MCwibGFiZWwiOiJSID0iLCJncm91cElkIjoiZ3JwX3Rlc3Qi"
  "fV19";

static const char* LAYOUT_CFG_TEST_DISTANCE_BASE64 =
  
  "eyJzY2hlbWFWZXJzaW9uIjoxLCJ0aXRsZSI6ImIzIC0gRGlzdGFuY2UgdGVzdCIsImNhbnZh"
  "cyI6eyJ3Ijo1NDAsImgiOjgzMH0sIndpZGdldHMiOlt7ImlkIjoiZ3JwX3Rlc3QiLCJ0Ijoi"
  "Z3JvdXAiLCJsYWJlbCI6IkRJU1RBTkNFIiwiY29sb3IiOiIjZmZiMDIwIiwieCI6NTYsInki"
  "OjQyLCJ3Ijo0MjgsImgiOjYxMiwiY2hpbGRyZW4iOlsiZ2F1Z2VfZGlzdGFuY2UiLCJhbGVy"
  "dCIsImdyYXBoX2Rpc3QiLCJsZXZlbCJdfSx7ImlkIjoiZ2F1Z2VfZGlzdGFuY2UiLCJ0Ijoi"
  "Z2F1Z2UiLCJ4Ijo4MCwieSI6MTAwLCJ3IjoxNTAsImgiOjE5MCwibGFiZWwiOiJEaXN0YW5j"
  "ZSIsIm1heCI6MjAwLCJ1bml0cyI6ImNtIiwiZGVjaW1hbHMiOjAsImdyb3VwSWQiOiJncnBf"
  "dGVzdCJ9LHsiaWQiOiJhbGVydCIsInQiOiJub3RpZmljYXRpb24iLCJ4IjoyNjAsInkiOjEx"
  "MCwidyI6MTEwLCJoIjoxMTAsImxhYmVsIjoiT2JzdGFjbGUiLCJncm91cElkIjoiZ3JwX3Rl"
  "c3QifSx7ImlkIjoiZ3JhcGhfZGlzdCIsInQiOiJncmFwaCIsIngiOjgwLCJ5IjozMjAsInci"
  "OjM4MCwiaCI6MjAwLCJsYWJlbCI6IkRpc3RhbmNlIGNtIiwibWF4IjoyMDAsImdyb3VwSWQi"
  "OiJncnBfdGVzdCJ9LHsiaWQiOiJsZXZlbCIsInQiOiJzZWxlY3QiLCJ4Ijo4MCwieSI6NTYw"
  "LCJ3IjoyMDAsImgiOjcwLCJsYWJlbCI6IlRlc3QiLCJvcHRpb25zIjoiQmVnaW5uZXIsRXhw"
  "ZXJ0LE1vdG9ycyxEaXN0YW5jZSxMaWdodHMsU291bmQsRGlzcGxheSxQb3dlciIsImdyb3Vw"
  "SWQiOiJncnBfdGVzdCJ9LHsiaWQiOiJsYmxfaGludCIsInQiOiJsYWJlbCIsIngiOjgwLCJ5"
  "Ijo2NzAsInciOjM4MCwiaCI6MTIwLCJsYWJlbCI6Ik1vdmUgeW91ciBoYW5kIGluIGZyb250"
  "IG9mIHRoZSBzZW5zb3IuIiwidmFsdWUiOiJNb3ZlIHlvdXIgaGFuZCBpbiBmcm9udCBvZiB0"
  "aGUgc2Vuc29yLiIsIm1vZGVsIjoiY2FyZCJ9XX0=";

static const char* LAYOUT_CFG_TEST_LIGHTS_BASE64 =
  
  "eyJzY2hlbWFWZXJzaW9uIjoxLCJ0aXRsZSI6ImIzIC0gTGlnaHRzIHRlc3QiLCJjYW52YXMi"
  "OnsidyI6NjMwLCJoIjo5MDB9LCJ3aWRnZXRzIjpbeyJpZCI6ImdycF90ZXN0IiwidCI6Imdy"
  "b3VwIiwibGFiZWwiOiJMSUdIVFMiLCJjb2xvciI6IiM3YzVjZmYiLCJ4Ijo1NiwieSI6NDIs"
  "InciOjUxOCwiaCI6NjgyLCJjaGlsZHJlbiI6WyJ0b2dnbGVfbGVkX3IiLCJ0b2dnbGVfbGVk"
  "X2ciLCJsZWRfcl9zdGF0ZSIsImxlZF9nX3N0YXRlIiwidG9nZ2xlX25wIiwibnBfZWZmZWN0"
  "IiwibnBfciIsIm5wX2ciLCJucF9iIiwibnBfYnJpZ2h0IiwibGV2ZWwiXX0seyJpZCI6InRv"
  "Z2dsZV9sZWRfciIsInQiOiJ0b2dnbGUiLCJ4Ijo4MCwieSI6MTAwLCJ3IjoxMTAsImgiOjEx"
  "MCwibGFiZWwiOiJSZWQgTEVEIiwibW9kZWwiOiJwaWxsIiwiZ3JvdXBJZCI6ImdycF90ZXN0"
  "In0seyJpZCI6InRvZ2dsZV9sZWRfZyIsInQiOiJ0b2dnbGUiLCJ4IjoyMTAsInkiOjEwMCwi"
  "dyI6MTEwLCJoIjoxMTAsImxhYmVsIjoiR3JlZW4gTEVEIiwibW9kZWwiOiJwaWxsIiwiZ3Jv"
  "dXBJZCI6ImdycF90ZXN0In0seyJpZCI6ImxlZF9yX3N0YXRlIiwidCI6ImxlZCIsIngiOjM1"
  "MCwieSI6MTEwLCJ3Ijo5MCwiaCI6OTAsImxhYmVsIjoiUmVkIGlzIiwibW9kZWwiOiJkb3Qi"
  "LCJjb2xvck9uIjoiI2ZmNTI1MiIsImNvbG9yT2ZmIjoiIzJhMmEzYSIsImdyb3VwSWQiOiJn"
  "cnBfdGVzdCJ9LHsiaWQiOiJsZWRfZ19zdGF0ZSIsInQiOiJsZWQiLCJ4Ijo0NjAsInkiOjEx"
  "MCwidyI6OTAsImgiOjkwLCJsYWJlbCI6IkdyZWVuIGlzIiwibW9kZWwiOiJkb3QiLCJjb2xv"
  "ck9uIjoiIzAwZmY4OCIsImNvbG9yT2ZmIjoiIzJhMmEzYSIsImdyb3VwSWQiOiJncnBfdGVz"
  "dCJ9LHsiaWQiOiJ0b2dnbGVfbnAiLCJ0IjoidG9nZ2xlIiwieCI6ODAsInkiOjI1MCwidyI6"
  "MTEwLCJoIjoxMTAsImxhYmVsIjoiU3RyaXAiLCJtb2RlbCI6InBpbGwiLCJncm91cElkIjoi"
  "Z3JwX3Rlc3QifSx7ImlkIjoibnBfZWZmZWN0IiwidCI6InNlbGVjdCIsIngiOjIxMCwieSI6"
  "MjcwLCJ3IjoxOTAsImgiOjcwLCJsYWJlbCI6IkVmZmVjdCIsIm9wdGlvbnMiOiJTb2xpZCxS"
  "YWluYm93LEtuaWdodCBSaWRlcixEdWVsIGV5ZSxGcmVuY2ggZmxhZyIsImdyb3VwSWQiOiJn"
  "cnBfdGVzdCJ9LHsiaWQiOiJucF9yIiwidCI6InNsaWRlciIsIngiOjgwLCJ5Ijo0MDAsInci"
  "OjgwLCJoIjoxOTAsImxhYmVsIjoiUiIsIm1heCI6MjU1LCJ2YWx1ZSI6MjU1LCJncm91cElk"
  "IjoiZ3JwX3Rlc3QifSx7ImlkIjoibnBfZyIsInQiOiJzbGlkZXIiLCJ4IjoxODAsInkiOjQw"
  "MCwidyI6ODAsImgiOjE5MCwibGFiZWwiOiJHIiwibWF4IjoyNTUsInZhbHVlIjowLCJncm91"
  "cElkIjoiZ3JwX3Rlc3QifSx7ImlkIjoibnBfYiIsInQiOiJzbGlkZXIiLCJ4IjoyODAsInki"
  "OjQwMCwidyI6ODAsImgiOjE5MCwibGFiZWwiOiJCIiwibWF4IjoyNTUsInZhbHVlIjowLCJn"
  "cm91cElkIjoiZ3JwX3Rlc3QifSx7ImlkIjoibnBfYnJpZ2h0IiwidCI6InNsaWRlciIsIngi"
  "OjM5MCwieSI6NDAwLCJ3IjoxMDAsImgiOjE5MCwibGFiZWwiOiJCcmlnaHQiLCJtYXgiOjI1"
  "NSwic3RlcCI6NSwidmFsdWUiOjE1LCJncm91cElkIjoiZ3JwX3Rlc3QifSx7ImlkIjoibGV2"
  "ZWwiLCJ0Ijoic2VsZWN0IiwieCI6ODAsInkiOjYzMCwidyI6MjAwLCJoIjo3MCwibGFiZWwi"
  "OiJUZXN0Iiwib3B0aW9ucyI6IkJlZ2lubmVyLEV4cGVydCxNb3RvcnMsRGlzdGFuY2UsTGln"
  "aHRzLFNvdW5kLERpc3BsYXksUG93ZXIiLCJncm91cElkIjoiZ3JwX3Rlc3QifSx7ImlkIjoi"
  "bGJsX2hpbnQiLCJ0IjoibGFiZWwiLCJ4Ijo4MCwieSI6NzQwLCJ3Ijo0NzAsImgiOjEyMCwi"
  "bGFiZWwiOiJTd2l0Y2ggdGhlIGxpZ2h0cyBvbiwgdGhlbiBtaXggYSBjb2xvdXIuIiwidmFs"
  "dWUiOiJTd2l0Y2ggdGhlIGxpZ2h0cyBvbiwgdGhlbiBtaXggYSBjb2xvdXIuIiwibW9kZWwi"
  "OiJjYXJkIn1dfQ==";

static const char* LAYOUT_CFG_TEST_SOUND_BASE64 =
  
  "eyJzY2hlbWFWZXJzaW9uIjoxLCJ0aXRsZSI6ImIzIC0gU291bmQgdGVzdCIsImNhbnZhcyI6"
  "eyJ3Ijo1NzAsImgiOjUzMH0sIndpZGdldHMiOlt7ImlkIjoiZ3JwX3Rlc3QiLCJ0IjoiZ3Jv"
  "dXAiLCJsYWJlbCI6IlNPVU5EIiwiY29sb3IiOiIjZmY1YzhhIiwieCI6NTYsInkiOjQyLCJ3"
  "Ijo0NTgsImgiOjMxMiwiY2hpbGRyZW4iOlsiYnRuX2hvcm4iLCJidG5fYnV6eiIsInNvdW5k"
  "X2FsZXJ0IiwibGV2ZWwiXX0seyJpZCI6ImJ0bl9ob3JuIiwidCI6ImJ1dHRvbiIsIngiOjgw"
  "LCJ5IjoxMDAsInciOjEyMCwiaCI6MTIwLCJsYWJlbCI6Ikhvcm4iLCJtb2RlbCI6Im5lbyIs"
  "Imdyb3VwSWQiOiJncnBfdGVzdCJ9LHsiaWQiOiJidG5fYnV6eiIsInQiOiJidXR0b24iLCJ4"
  "IjoyMzAsInkiOjEwMCwidyI6MTIwLCJoIjoxMjAsImxhYmVsIjoiQnV6eiIsIm1vZGVsIjoi"
  "bmVvIiwiZ3JvdXBJZCI6ImdycF90ZXN0In0seyJpZCI6InNvdW5kX2FsZXJ0IiwidCI6InNv"
  "dW5kIiwieCI6MzgwLCJ5IjoxMDUsInciOjExMCwiaCI6MTEwLCJsYWJlbCI6IkFsZXJ0Iiwi"
  "Z3JvdXBJZCI6ImdycF90ZXN0In0seyJpZCI6ImxldmVsIiwidCI6InNlbGVjdCIsIngiOjgw"
  "LCJ5IjoyNjAsInciOjIwMCwiaCI6NzAsImxhYmVsIjoiVGVzdCIsIm9wdGlvbnMiOiJCZWdp"
  "bm5lcixFeHBlcnQsTW90b3JzLERpc3RhbmNlLExpZ2h0cyxTb3VuZCxEaXNwbGF5LFBvd2Vy"
  "IiwiZ3JvdXBJZCI6ImdycF90ZXN0In0seyJpZCI6ImxibF9oaW50IiwidCI6ImxhYmVsIiwi"
  "eCI6ODAsInkiOjM3MCwidyI6NDEwLCJoIjoxMjAsImxhYmVsIjoiRWFjaCBidXR0b24gbWFr"
  "ZXMgYSBkaWZmZXJlbnQgc291bmQuIiwidmFsdWUiOiJFYWNoIGJ1dHRvbiBtYWtlcyBhIGRp"
  "ZmZlcmVudCBzb3VuZC4iLCJtb2RlbCI6ImNhcmQifV19";

static const char* LAYOUT_CFG_TEST_DISPLAY_BASE64 =
  
  
  
  
  "eyJzY2hlbWFWZXJzaW9uIjoxLCJ0aXRsZSI6ImIzIC0gRGlzcGxheSB0ZXN0IiwiY2FudmFz"
  "Ijp7InciOjgxNCwiaCI6NjMwfSwid2lkZ2V0cyI6W3siaWQiOiJncnBfdGVzdCIsInQiOiJn"
  "cm91cCIsImxhYmVsIjoiRElTUExBWSIsImNvbG9yIjoiIzAwZTVmZiIsIngiOjU2LCJ5Ijo0"
  "MiwidyI6Njc4LCJoIjoyNzIsImNoaWxkcmVuIjpbIm9sZWRfdGV4dCIsImxibF9vbGVkIiwi"
  "bGV2ZWwiXX0seyJpZCI6Im9sZWRfdGV4dCIsInQiOiJlZGl0ZmllbGQiLCJ4Ijo4MCwieSI6"
  "MTAwLCJ3IjozMDAsImgiOjgwLCJsYWJlbCI6IldyaXRlIGhlcmUiLCJwbGFjZWhvbGRlciI6"
  "IlR5cGUgeW91ciBuYW1lLi4uIiwiZ3JvdXBJZCI6ImdycF90ZXN0In0seyJpZCI6ImxibF9v"
  "bGVkIiwidCI6ImxhYmVsIiwieCI6NDEwLCJ5IjoxMDUsInciOjMwMCwiaCI6NzAsImxhYmVs"
  "IjoiU2NyZWVuIHNob3dzIiwibW9kZWwiOiJjYXJkIiwiZ3JvdXBJZCI6ImdycF90ZXN0In0s"
  "eyJpZCI6ImxldmVsIiwidCI6InNlbGVjdCIsIngiOjgwLCJ5IjozMjAsInciOjIwMCwiaCI6"
  "NzAsImxhYmVsIjoiVGVzdCIsIm9wdGlvbnMiOiJCZWdpbm5lcixFeHBlcnQsTW90b3JzLERp"
  "c3RhbmNlLExpZ2h0cyxTb3VuZCxEaXNwbGF5LFBvd2VyIiwiZ3JvdXBJZCI6ImdycF90ZXN0"
  "In0seyJpZCI6ImxibF9oaW50IiwidCI6ImxhYmVsIiwieCI6ODAsInkiOjQzMCwidyI6NjU0"
  "LCJoIjoxMjAsImxhYmVsIjoiVHlwZSwgdGhlbiBsb29rIGF0IHRoZSByb2JvdCdzIGxpdHRs"
  "ZSBzY3JlZW4uIiwidmFsdWUiOiJUeXBlLCB0aGVuIGxvb2sgYXQgdGhlIHJvYm90J3MgbGl0"
  "dGxlIHNjcmVlbi4iLCJtb2RlbCI6ImNhcmQifSx7ImlkIjoic2NyZWVuX21vZGUiLCJ0Ijoi"
  "c2VsZWN0IiwibGFiZWwiOiJTY3JlZW4iLCJvcHRpb25zIjoiU3RhdHVzLEZhY2UsQXV0byxS"
  "YWRhciIsIngiOjgwLCJ5IjoyMTAsInciOjMwMCwiaCI6ODB9LHsiaWQiOiJmYWNlX3N0eWxl"
  "IiwidCI6InNlbGVjdCIsImxhYmVsIjoiRmFjZSBzdHlsZSIsIm9wdGlvbnMiOiJSb3VuZCxD"
  "aXJjbGUsUm9ib3QsQmlnLFZpc29yIiwieCI6NDEwLCJ5IjoyMTAsInciOjMwMCwiaCI6ODB9"
  "LHsiaWQiOiJleWVzX2ZvbGxvdyIsInQiOiJ0b2dnbGUiLCJ4Ijo0MTAsInkiOjMxNSwidyI6"
  "MjIwLCJoIjo5MCwibGFiZWwiOiJFeWVzIGZvbGxvdyIsIm1vZGVsIjoicGlsbCJ9XX0=";

static const char* LAYOUT_CFG_TEST_POWER_BASE64 =
  
  "eyJzY2hlbWFWZXJzaW9uIjoxLCJ0aXRsZSI6ImIzIC0gUG93ZXIgdGVzdCIsImNhbnZhcyI6"
  "eyJ3Ijo2ODAsImgiOjcwMH0sIndpZGdldHMiOlt7ImlkIjoiZ3JwX3Rlc3QiLCJ0IjoiZ3Jv"
  "dXAiLCJsYWJlbCI6IlBPV0VSIiwiY29sb3IiOiIjM2RkYzk3IiwieCI6NTYsInkiOjQyLCJ3"
  "Ijo1NjgsImgiOjQ4MiwiY2hpbGRyZW4iOlsiYmF0dGVyeV9sZXZlbCIsImxibF92YmF0Iiwi"
  "Z2F1Z2VfcnNzaSIsImJ0bl9idXp6IiwibGV2ZWwiXX0seyJpZCI6ImJhdHRlcnlfbGV2ZWwi"
  "LCJ0IjoiYmF0dGVyeSIsIngiOjgwLCJ5IjoxMDAsInciOjkwLCJoIjoxMjAsImxhYmVsIjoi"
  "QmF0dGVyeSIsIm1vZGVsIjoidmVydGljYWwiLCJncm91cElkIjoiZ3JwX3Rlc3QifSx7Imlk"
  "IjoibGJsX3ZiYXQiLCJ0IjoibGFiZWwiLCJ4IjoyMDAsInkiOjEyMCwidyI6MjIwLCJoIjo2"
  "MCwibGFiZWwiOiJWb2x0cyIsIm1vZGVsIjoiY2FyZCIsImdyb3VwSWQiOiJncnBfdGVzdCJ9"
  "LHsiaWQiOiJnYXVnZV9yc3NpIiwidCI6ImdhdWdlIiwieCI6NDUwLCJ5IjoxMDAsInciOjE1"
  "MCwiaCI6MTkwLCJsYWJlbCI6IlNpZ25hbCIsIm1pbiI6LTEwMCwibWF4IjotMzAsInVuaXRz"
  "IjoiZEJtIiwiZGVjaW1hbHMiOjAsImdyb3VwSWQiOiJncnBfdGVzdCJ9LHsiaWQiOiJidG5f"
  "YnV6eiIsInQiOiJidXR0b24iLCJ4Ijo4MCwieSI6MjcwLCJ3IjoxMjAsImgiOjEyMCwibGFi"
  "ZWwiOiJCdXp6IiwibW9kZWwiOiJuZW8iLCJncm91cElkIjoiZ3JwX3Rlc3QifSx7ImlkIjoi"
  "bGV2ZWwiLCJ0Ijoic2VsZWN0IiwieCI6ODAsInkiOjQzMCwidyI6MjAwLCJoIjo3MCwibGFi"
  "ZWwiOiJUZXN0Iiwib3B0aW9ucyI6IkJlZ2lubmVyLEV4cGVydCxNb3RvcnMsRGlzdGFuY2Us"
  "TGlnaHRzLFNvdW5kLERpc3BsYXksUG93ZXIiLCJncm91cElkIjoiZ3JwX3Rlc3QifSx7Imlk"
  "IjoibGJsX2hpbnQiLCJ0IjoibGFiZWwiLCJ4Ijo4MCwieSI6NTQwLCJ3Ijo1MjAsImgiOjEy"
  "MCwibGFiZWwiOiJCdXp6IHVzZXMgdGhlIHNhbWUgcGluIGFzIHRoZSBiYXR0ZXJ5IHNlbnNv"
  "ciAtLSB3YXRjaCB0aGUgdm9sdHMuIiwidmFsdWUiOiJCdXp6IHVzZXMgdGhlIHNhbWUgcGlu"
  "IGFzIHRoZSBiYXR0ZXJ5IHNlbnNvciAtLSB3YXRjaCB0aGUgdm9sdHMuIiwibW9kZWwiOiJj"
  "YXJkIn1dfQ==";

// ===========================================================================
// State
// ===========================================================================
static NimBLECharacteristic* s_txChar     = nullptr;
static volatile bool         s_connected  = false;
// Guards against a future periodic-output task racing sendCfg()'s burst,
// same as the reference firmware's gSendingCfg.
static volatile bool         s_sendingCfg = false;
static String                s_rxBuffer;

// Set from onWrite() (NimBLE's own host task), consumed from remotexy_handler()
// (the ordinary Arduino loop() task). Ported from esp32-rxy's post-mortem fix:
// calling sendCfg() — a ~900ms burst of ~60 notify() calls — directly and
// synchronously from inside onWrite() blocks NimBLE's host task from doing
// its own buffer-completion housekeeping for the whole burst, starving the
// notify() mbuf pool and causing most sends to fail with rc=6 (ENOMEM).
// Deferring the burst to loop() lets the host task keep servicing itself
// concurrently, so the pool never starves.
static volatile bool         s_getCfgRequested = false;
static volatile bool         s_getCfgVerRequested = false;

// Negotiated ATT MTU, updated by onMTUChange(). sendCfg() sizes its chunks
// from this: the old fixed 18 came from the rxy MakeCode template, not from
// any limit of this radio, and it made a large layout take tens of seconds.
static volatile uint16_t     s_peerMtu = 23;

// Connection handle of the live link, kept so link quality can be sampled.
// BLE_HS_CONN_HANDLE_NONE means "not connected" and is what ble_gap_conn_rssi()
// is guarded against — querying a stale handle returns an error rather than a
// reading, and would otherwise publish a nonsense value every cycle.
static volatile uint16_t     s_connHandle = BLE_HS_CONN_HANDLE_NONE;

static int8_t s_joy_x = 0;   // -100..100, drives tasks_joysticks() (turn)
static int8_t s_joy_y = 0;   // -100..100, drives tasks_joysticks() (forward/back)
static uint8_t s_button_01 = 0;

// dpad_drive writes the same s_joy_x/s_joy_y as joy_drive — both widgets
// drive the identical robot state, whichever was touched most recently wins.
static bool s_dpad_up = false, s_dpad_down = false, s_dpad_left = false, s_dpad_right = false;

// Which app is on the other end, for the ambiguous joystick encoding below.
// Only the mecanum app (keystudio_4wd_mecanum_rxy, v2.13+) opens with
// GETCFGVER; stock bit-rxy has no such command, so seeing it identifies the
// peer. Latched, never cleared on reconnect, because both flags only ever
// widen what we accept — a false positive costs nothing that the value-range
// checks in handleJoystick() do not already override.
static bool s_peerSawCfgVer = false;
// Set once a joystick sample proves the x/y encoding (a negative field).
static bool s_joyIsXY       = false;

// Reported to the app in the SYSTEM zone's Firmware label. Bump on release.
// Kept the B3_ prefix so this file still diffs cleanly against b3's copy --
// only the value distinguishes the robots in the app's Firmware label.
#define B3_FIRMWARE_VERSION "MV-v2"

// ---------------------------------------------------------------------------
// Widget state owned by the app (see 03_bit-rxy.h for the accessors).
// ---------------------------------------------------------------------------
static uint8_t  s_speed_cap   = 100;              // DRIVE  — Speed slider
static uint8_t  s_np_on       = 1;                // LIGHTS — Strip toggle
static uint8_t  s_np_effect   = NP_EFFECT_FRENCH; // LIGHTS — matches the old
                                                  // hardcoded loop() behaviour
static uint8_t  s_np_bright   = CONFIG_NEOPIXELS_BRIGHTNESS;
static uint32_t s_np_color    = 0xFF0000;         // LIGHTS — Strip colour

// LIGHTS — the two board LEDs. These are normally the link-status indicator
// (leds_update() blinks green when connected, red when not), so the app does
// not get them for free: the first toggle takes ownership, and ownership is
// dropped again on disconnect so the "not connected" signal always comes back.
// ---- WHEEL TRIM ----------------------------------------------------------
// Ported from dfrobot-rover, and for the identical reason: these wheels are
// CONTINUOUS-ROTATION servos, so 90 is only nominally stop. Every servo's true
// stop point differs, and a robot told to drive straight creeps or veers.
//
// This robot already had the fix, as CONFIG_SERVO_SPEED_STOP_*_OFFSET applied
// in tasks_joysticks() -- but as compile-time constants, so calibrating a
// second robot meant editing and reflashing. Those constants stay, now as the
// FACTORY DEFAULT: a chip with nothing in NVS behaves exactly as it did
// before, and only a deliberate adjustment changes anything.
#define TRIM_MIN -20
#define TRIM_MAX  20
static int8_t s_trim_l = CONFIG_SERVO_SPEED_STOP_LEFT_OFFSET;
static int8_t s_trim_r = CONFIG_SERVO_SPEED_STOP_RIGHT_OFFSET;
// A slider drag is a stream of values and flash has a finite number of erase
// cycles, so a write per event would spend the chip's life calibrating one
// robot. Two seconds after you stop moving, one write.
#define TRIM_SAVE_SETTLE_MS 2000
static bool     s_trim_save_due = false;
static uint32_t s_trim_save_at  = 0;
static bool     s_trim_echo_due = false;
// Read from NVS before the OLED module exists; handed over in remotexy_init().
static bool     s_eyes_follow_boot = true;
static uint8_t  s_trim_echo_i   = 4;   // 4 = nothing pending

int8_t remotexy_get_trim_l(void) { return s_trim_l; }
int8_t remotexy_get_trim_r(void) { return s_trim_r; }

static bool s_led_manual = false;
static bool s_led_r_on   = false;
static bool s_led_g_on   = false;
static uint8_t  s_upd_level   = UPD_ALL;          // SYSTEM — Telemetry select

// Which layout the robot serves. Both are compiled in; the Level select in
// the SYSTEM zone switches between them, and the choice is kept in NVS so a
// power cycle does not drop an expert user back to the simple panel. A fresh
// device starts on Beginner.
//
// The `level` widget exists in BOTH layouts on purpose — if it were only in
// the expert one, switching to beginner would strand the robot there until a
// reflash.
static Preferences s_prefs;
static uint8_t     s_layout_level = LAYOUT_BEGINNER;

static void layoutLevelLoad(void) {
  // Read-only open: nothing else in this firmware uses NVS, so a missing
  // namespace on a fresh chip is expected rather than an error.
  if (s_prefs.begin("b3", true)) {
    s_layout_level = s_prefs.getUChar("level", LAYOUT_BEGINNER);
    // Defaulting to the compile-time offsets rather than to zero: a robot
    // that was calibrated by those constants must not start veering the day
    // the trim becomes adjustable.
    s_trim_l = (int8_t)s_prefs.getChar("trimL", CONFIG_SERVO_SPEED_STOP_LEFT_OFFSET);
    s_trim_r = (int8_t)s_prefs.getChar("trimR", CONFIG_SERVO_SPEED_STOP_RIGHT_OFFSET);
    s_eyes_follow_boot = s_prefs.getBool("eyes", true);
    s_prefs.end();
  }
  if (s_layout_level >= LAYOUT_COUNT) s_layout_level = LAYOUT_BEGINNER;
  s_trim_l = (int8_t)constrain(s_trim_l, TRIM_MIN, TRIM_MAX);
  s_trim_r = (int8_t)constrain(s_trim_r, TRIM_MIN, TRIM_MAX);
}

static void layoutLevelStore(uint8_t lvl) {
  if (lvl == s_layout_level) return;   // don't burn a flash write on a no-op
  s_layout_level = lvl;
  if (s_prefs.begin("b3", false)) {
    s_prefs.putUChar("level", lvl);
    s_prefs.end();
  }
}

uint8_t remotexy_get_layout_level(void) { return s_layout_level; }

// ---------------------------------------------------------------------------
// Telemetry is sent on change, not every cycle. With the expert layout the
// naive version pushed 11 notifications per cycle for values that mostly do
// not move — and this radio has a documented history of drowning in rc=6
// (BLE_HS_ENOMEM) when notify() is called too freely.
//
// Each value carries a deadband so sensor jitter alone cannot make it "change"
// forever: the ultrasonic reading wobbles by a few mm at rest and RSSI drifts
// a dB or two constantly.
//
// s_telemForce makes the next pass send everything regardless. It is set after
// every CFG transfer, because a freshly rendered panel has no values in it —
// without it, a widget whose value had not changed since before the reconnect
// would sit blank until it happened to move.
// ---------------------------------------------------------------------------
static bool     s_telemForce      = true;
static float    s_lastDist        = NAN;
static float    s_lastSpeed       = NAN;
static float    s_lastBatt        = NAN;
static float    s_lastVbat        = NAN;
static int16_t  s_lastRssi        = 32767;
static int8_t   s_lastBtn         = -1;
static uint32_t s_lastUptimeSec   = 0xFFFFFFFFu;
static bool     s_verSent         = false;
static String   s_lastOled        = "";
static int8_t   s_lastLedR        = -1;
static int8_t   s_lastLedG        = -1;   // impossible first value -> always sends once


// ---------------------------------------------------------------------------
// Which blob each mode serves, and which telemetry ids that panel contains.
//
// The old code gated telemetry on "is this the expert layout", which stopped
// working the moment there was more than one small panel: a Sound panel has no
// gauges at all, and publishing to widget ids the active layout does not
// contain is a notification into nothing. One bit per readout keeps each
// sender honest about where its value is actually displayed.
// ---------------------------------------------------------------------------
#define T_DIST   (1u<<0)
#define T_SPEED  (1u<<1)
#define T_BATT   (1u<<2)
#define T_GRAPH  (1u<<3)
#define T_ALERT  (1u<<4)
#define T_VER    (1u<<5)
#define T_UPTIME (1u<<6)
#define T_VBAT   (1u<<7)
#define T_RSSI   (1u<<8)
#define T_BTN    (1u<<9)
#define T_LEDS   (1u<<10)
#define T_OLED   (1u<<11)
#define T_UPDSEL (1u<<12)
#define T_HEAD   (1u<<15)  // the head slider echo, and with it the radar
// The Speed SLIDER, which is not the same set of panels as the speed gauge:
// Beginner shows the gauge but has no slider to snap back.
#define T_SPDCAP (1u<<13)
// The eight trim widgets: two sliders, two numeric fields, four step buttons.
#define T_TRIM   (1u<<14)

static const char* const LAYOUT_BLOBS[LAYOUT_COUNT] = {
  LAYOUT_CFG_BEGINNER_BASE64,
  LAYOUT_CFG_EXPERT_BASE64,
  LAYOUT_CFG_TEST_MOTORS_BASE64,
  LAYOUT_CFG_TEST_DISTANCE_BASE64,
  LAYOUT_CFG_TEST_LIGHTS_BASE64,
  LAYOUT_CFG_TEST_SOUND_BASE64,
  LAYOUT_CFG_TEST_DISPLAY_BASE64,
  LAYOUT_CFG_TEST_POWER_BASE64,
};

static const uint16_t LAYOUT_TELEM[LAYOUT_COUNT] = {
  /* Beginner */ T_DIST | T_SPEED | T_BATT,
  /* Expert   */ 0xFFFF,
  /* Motors   */ T_SPEED | T_SPDCAP | T_TRIM,
  /* Distance */ T_DIST | T_GRAPH | T_ALERT | T_HEAD,
  /* Lights   */ T_LEDS,
  /* Sound    */ 0,
  /* Display  */ T_OLED,
  /* Power    */ T_BATT | T_VBAT | T_RSSI,
};

// Names shown in the Level select. Must match the enum order and the options
// string baked into every layout's `level` widget.
static const char* const LEVEL_NAMES[LAYOUT_COUNT] = {
  "Beginner", "Expert", "Motors", "Distance",
  "Lights", "Sound", "Display", "Power"
};

static inline bool modeHas(uint16_t bit) {
  return (LAYOUT_TELEM[s_layout_level] & bit) != 0;
}

// Returns true (and latches the new value) when `now` has moved far enough
// from the last transmitted value to be worth a notification.
static bool telemChanged(float now, float& last, float deadband) {
  if (s_telemForce || isnan(last) || fabsf(now - last) >= deadband) {
    last = now;
    return true;
  }
  return false;
}

// Obstacle notification is edge-triggered: the app shows a toast per message,
// so re-sending every cycle while an obstacle sits in front would spam it.
static bool     s_obstacleLatched = false;
#define B3_OBSTACLE_CM      30.0f
#define B3_OBSTACLE_CLEAR_CM 40.0f   // hysteresis, so a jittering reading
                                     // near the threshold cannot chatter

uint8_t  remotexy_get_speed_cap(void)      { return s_speed_cap; }
uint8_t  remotexy_get_np_on(void)          { return s_np_on; }
uint8_t  remotexy_get_np_effect(void)      { return s_np_effect; }
uint8_t  remotexy_get_np_brightness(void)  { return s_np_bright; }
uint32_t remotexy_get_np_color(void)       { return s_np_color; }
uint8_t  remotexy_get_upd_level(void)      { return s_upd_level; }

static void handleLine(const String& line);
static void handleWidget(const String& id, const String& val);
static void handleDpadMask(uint8_t mask);
static bool sendLine(const String& line);
static void sendCfg();

static inline void sendValue(const String& id, const String& val) {
  if (s_sendingCfg) return;  // don't interleave widget updates with a CFG burst
  sendLine("UPD " + id + " " + val);
}

// ===========================================================================
// BLE callbacks
// ===========================================================================
class RemoteXYServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server, NimBLEConnInfo& info) override {
    s_connected = true;
    s_rxBuffer  = "";
    #ifdef DEF_DERIAL_DEBUG
    Serial.printf("[BLE] Client connected  peer=%s\n", info.getAddress().toString().c_str());
    #endif
    // Request a fast connection interval (7.5-15ms, units of 1.25ms) so the
    // controller drains notify()'s buffer pool quickly enough to survive
    // sendCfg()'s ~60-chunk burst.
    server->updateConnParams(info.getConnHandle(), 6, 12, 0, 400);
    s_peerMtu    = 23;  // assume the BLE minimum until the peer negotiates up
    s_connHandle = info.getConnHandle();
  }
  // Records the negotiated MTU so sendCfg() can size its chunks to it. The
  // central drives this: browsers on desktop typically settle well above the
  // 23-byte minimum, but nothing guarantees it, so the value is only ever
  // used as an upper bound with a conservative floor.
  void onMTUChange(uint16_t MTU, NimBLEConnInfo& /*info*/) override {
    s_peerMtu = MTU;
    #ifdef DEF_DERIAL_DEBUG
    Serial.printf("[BLE] MTU negotiated: %u\n", (unsigned)MTU);
    #endif
  }
  void onDisconnect(NimBLEServer* /*server*/, NimBLEConnInfo& /*info*/, int reason) override {
    s_connected = false;
    s_joy_x = 0;
    s_joy_y = 0;
    s_button_01 = 0;
    s_dpad_up = s_dpad_down = s_dpad_left = s_dpad_right = false;
    s_connHandle = BLE_HS_CONN_HANDLE_NONE;
    // Hand the two board LEDs back to the status indicator, so a disconnect
    // is always visible even if the user had taken them over from the app.
    s_led_manual = false;
    #ifdef DEF_DERIAL_DEBUG
    Serial.printf("[BLE] Client disconnected (reason 0x%02x) - re-advertising\n", reason);
    #endif
    NimBLEDevice::startAdvertising();
  }
};

class RemoteXYRxCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* chr, NimBLEConnInfo& /*info*/) override {
    std::string v = chr->getValue();
    #ifdef DEF_DERIAL_DEBUG
    Serial.printf("[BLE] RX %u bytes: '%s'\n", (unsigned)v.size(), v.c_str());
    #endif
    for (size_t i = 0; i < v.size(); ++i) {
      const char c = v[i];
      if (c == '\r') continue;
      if (c == '\n') {
        if (s_rxBuffer.length() > 0) {
          handleLine(s_rxBuffer);
          s_rxBuffer = "";
        }
      } else {
        s_rxBuffer += c;
        if (s_rxBuffer.length() > 256) s_rxBuffer = "";  // overflow guard
      }
    }
  }
};

// ===========================================================================
// Protocol
// ===========================================================================
static bool sendLine(const String& line) {
  if (!s_connected || s_txChar == nullptr) return false;
  String out = line + "\n";
  return s_txChar->notify((const uint8_t*)out.c_str(), out.length());
}

static void sendCfg() {
  s_sendingCfg = true;
  // onConnect() requests a fast connection interval via updateConnParams(),
  // but that's an async negotiation with the central — it takes a round
  // trip or more to actually take effect. The app sends GETCFG almost
  // immediately after connecting, so without this delay the burst below
  // starts while still on the central's slower default interval, which
  // can't drain notify()'s buffer pool fast enough (rc=6 / BLE_HS_ENOMEM
  // on nearly every packet). Give the renegotiation time to land first.
  delay(300);
  const char* p  = LAYOUT_BLOBS[s_layout_level];
  const size_t n = strlen(p);

  // Chunk size follows the negotiated MTU rather than the rxy MakeCode
  // template's fixed 18. The app never cares how big a chunk is — it just
  // appends line.substring(4) — so the only real limit is what fits in one
  // notification: MTU minus 3 bytes of ATT header, minus "CFG " and the
  // trailing newline. Capped at 180 to stay clear of controller buffer
  // pressure, floored at 18 so a peer that only grants the 23-byte minimum
  // still behaves exactly as before. On a desktop browser (MTU ~247) this
  // turns a 260-chunk, 13-second burst into ~26 chunks and ~1.6 seconds.
  // ASK the stack rather than trust the cached value. s_peerMtu is only ever
  // written by onMTUChange(), and if that never fires -- the central never
  // initiates an exchange, or it happened before this connection's callback
  // was in place -- the cache stays at its assumed 23 and every chunk is 18
  // bytes. That is how an 8.5KB Expert layout became 472 chunks and twenty
  // seconds on a link that had actually negotiated room for 180-byte chunks.
  // getPeerMTU() reads the live ATT MTU for the connection, so a missed
  // callback can no longer cost the transfer its speed.
  {
    NimBLEServer* srv = NimBLEDevice::getServer();
    const uint16_t live = srv ? srv->getPeerMTU(s_connHandle) : 0;
    if (live > s_peerMtu) s_peerMtu = live;
  }
  size_t chunk = 18;
  if (s_peerMtu > 23 + 5) {
    chunk = (size_t)s_peerMtu - 3 - 5;   // ATT header + "CFG " + '\n'
    if (chunk > 180) chunk = 180;
    if (chunk < 18)  chunk = 18;
  }
  const size_t CHUNK = chunk;
  const unsigned total = (unsigned)((n + CHUNK - 1) / CHUNK);
  #ifdef DEF_DERIAL_DEBUG
  Serial.printf("[BLE] CFG %u bytes, MTU %u -> chunk %u (%u chunks)\n",
                (unsigned)n, (unsigned)s_peerMtu, (unsigned)CHUNK, total);
  #endif

  // Announce how many chunks are coming. Both apps match this line with
  // startsWith("CFGBEGIN"), so the argument is ignored by anything that does
  // not want it — but a client that reads it can show a truthful progress bar
  // instead of guessing. The existing guess (12 + 4*chunk, capped at 90) was
  // tuned for a ~20-chunk transfer and now pins at 90% for most of the burst,
  // because MTU-sized chunking made the count vary from 14 to 40-plus.
  sendLine("CFGBEGIN " + String(total));

  // The robot's own screen shows the same transfer the app is showing. This
  // loop blocks loop() from here until CFGEND -- twenty seconds on a peer that
  // never negotiated its MTU up -- so oled_update() cannot run and the panel
  // would otherwise sit frozen on whatever it happened to be showing.
  unsigned sent = 0;
  const unsigned tick = (total / 20) ? (total / 20) : 1;
  unsigned nextTick = 0;
  oled_draw_progress(LEVEL_NAMES[s_layout_level], 0, total);

  int dropped = 0;
  for (size_t i = 0; i < n; i += CHUNK) {
    String line = "CFG ";
    for (size_t j = 0; j < CHUNK && (i + j) < n; ++j) line += p[i + j];
    // 50ms (vs. the reference firmware's 15ms): this sketch also runs
    // FastLED/OLED/Servo alongside NimBLE, leaving less controller buffer
    // headroom than the reference's minimal sketch, so the same burst
    // rate that worked there (rc=6 / BLE_HS_ENOMEM) overflows here.
    if (!sendLine(line)) dropped++;
    sent++;
    // Roughly twenty repaints across the whole burst: display() pushes the
    // full 1KB buffer over I2C, so one per chunk would add half the transfer
    // time again for no extra information.
    if (sent >= nextTick || sent == total) {
      oled_draw_progress(LEVEL_NAMES[s_layout_level], sent, total);
      nextTick = sent + tick;
    }
    delay(50);
  }
  sendLine("CFGEND");
  // The app has just rendered a fresh panel with no values in it, so the next
  // telemetry pass must publish everything rather than only what has moved.
  s_telemForce = true;
  s_sendingCfg = false;
  #ifdef DEF_DERIAL_DEBUG
  Serial.printf("[BLE] Sent CFG (dropped=%d)\n", dropped);
  #endif
}

static void handleLine(const String& line) {
  #ifdef DEF_DERIAL_DEBUG
  Serial.printf("[BLE] RX line: '%s'\n", line.c_str());
  #endif

  // Fastest D-pad wire format, and the only one the mecanum app sends: one
  // byte 'a'..'p' encodes the complete 4-bit direction mask (up=1, down=2,
  // left=4, right=8), followed by the newline this line was already split on.
  // Tested before everything else because while driving it is by far the most
  // frequent line — a held direction is re-sent every 300ms as a keepalive.
  if (line.length() == 1) {
    const char c = line.charAt(0);
    if (c >= 'a' && c <= 'p') { handleDpadMask((uint8_t)(c - 'a')); return; }
  }

  // Text form of the same mask. The micro:bit firmware honours it and the
  // mecanum app still filters "M " out of its own send queue, so a future app
  // version may well emit it again; sharing the handler costs one branch.
  if (line.startsWith("M ")) {
    handleDpadMask((uint8_t)line.substring(2).toInt());
    return;
  }

  // Answered as of S2-v2. This used to be swallowed, on the belief that a reply
  // had to reproduce a hash the app computes independently. It does not: the app
  // stores whatever string arrives here verbatim and compares it verbatim on the
  // next connect (saveCachedRemoteConfig/loadCachedRemoteConfig), so the token is
  // entirely ours to choose. Staying silent cost every connection the app's full
  // 3x900ms probe timeout AND the whole layout transfer afterwards.
  //
  // Still records the flag: it tells handleJoystick() which encoding to expect,
  // since only the rxy app sends this command and stock bit-rxy has none.
  if (line == "GETCFGVER") {
    s_peerSawCfgVer = true;
    s_getCfgVerRequested = true;   // reply from the main task, never from here
    return;
  }

  // The app sends this instead of GETCFG when its cached copy already matches
  // the revision we just reported. No transfer happens, so nothing else would
  // raise s_telemForce -- and the panel it just restored from cache is drawn
  // with no values in it. Without this every gauge would sit empty until its
  // reading happened to change.
  if (line.startsWith("CFGOK")) {
    s_telemForce = true;
    #ifdef DEF_DERIAL_DEBUG
    Serial.println("[BLE] Client had this layout cached - transfer skipped");
    #endif
    return;
  }

  // Defer to remotexy_handler() — do NOT call sendCfg() directly here.
  // See s_getCfgRequested above for why running the burst synchronously
  // from this callback (NimBLE's own host task) was a real, documented bug.
  if (line == "GETCFG") { s_getCfgRequested = true; return; }

  if (line.startsWith("SET ")) {
    int sp = line.indexOf(' ', 4);
    if (sp < 0) return;
    String id  = line.substring(4, sp);
    String val = line.substring(sp + 1);
    handleWidget(id, val);
  }
}

// Two apps put two different encodings on the same "SET joy_drive ..." line:
//
//   stock bit-rxy   "<angle> <distance>"  angle 0deg=right, 90deg=up,
//                                         180deg=left, 270deg=down — standard
//                                         math convention, confirmed
//                                         empirically against the live app;
//                                         the "90=down" convention documented
//                                         in some rxy reference comments does
//                                         not match actual runtime behavior.
//                                         distance 0..100.
//   mecanum app     "<x> <y>"             both -100..100, +x = right,
//                                         +y = up.
//
// Both are two numbers, so a sample inside the 0..100 quadrant is genuinely
// ambiguous. Resolved in this order, strongest evidence first:
//
//   1. either field negative   -> x/y    (a distance is never negative)
//   2. peer sent GETCFGVER     -> x/y    (only the mecanum app sends it)
//   3. first field above 100   -> angle  (a valid angle, an impossible x)
//   4. otherwise               -> angle  (bit-rxy's long-standing reading, so
//                                         its behaviour is bit-for-bit
//                                         unchanged by this function)
//
// Rule 1 latches for the session: once x/y is proven, later samples that fall
// back inside the ambiguous quadrant keep being read as x/y.
//
// Either way the output is the -100..100 pair tasks_joysticks() expects
// (x = turn right, y = forward).
static void handleJoystick(const String& val) {
  int sp = val.indexOf(' ');
  if (sp < 0) return;
  const float a = val.substring(0, sp).toFloat();   // angle, or x
  const float b = val.substring(sp + 1).toFloat();  // distance, or y

  if (a < 0.0f || b < 0.0f) s_joyIsXY = true;
  const bool xy = s_joyIsXY || (s_peerSawCfgVer && a <= 100.0f);

  if (xy) {
    s_joy_x = (int8_t)constrain((long)round(a), -100, 100);
    // Negated for the same reason as the angle path below.
    s_joy_y = (int8_t)constrain((long)round(-b), -100, 100);
    return;
  }

  const float rad = a * PI / 180.0f;
  s_joy_x = (int8_t)constrain((long)round(b * cos(rad)), -100, 100);
  // Negated: the robot's forward/back sense turned out to be the mirror
  // of the joystick's up/down sense (confirmed empirically) — turning
  // (x) was already correct, only this axis needed flipping.
  s_joy_y = (int8_t)constrain((long)round(-b * sin(rad)), -100, 100);
}

// D-Pad sends "<up|down|left|right> <0|1>" immediately on press/release
// (unlike the joystick, which only streams while actively dragging), so it
// drives s_joy_x/s_joy_y at a fixed full-speed magnitude from the combined
// state of whichever direction buttons are currently held.
static const int8_t DPAD_SPEED = 100;

// Shared by both D-pad wire formats: whichever one set the four booleans,
// the drive state is derived from their combined value the same way.
static void dpadRecompute() {
  s_joy_x = (int8_t)((s_dpad_right ? DPAD_SPEED : 0) - (s_dpad_left ? DPAD_SPEED : 0));
  // Same forward/back mirror as the joystick (see handleJoystick) — down
  // is physically forward on this chassis.
  s_joy_y = (int8_t)((s_dpad_down  ? DPAD_SPEED : 0) - (s_dpad_up   ? DPAD_SPEED : 0));
}

// "<up|down|left|right> <0|1>" — one direction per message, as stock bit-rxy
// sends it. Unchanged.
static void handleDpad(const String& val) {
  int sp = val.indexOf(' ');
  if (sp < 0) return;
  String dir     = val.substring(0, sp);
  bool   pressed = val.substring(sp + 1) == "1";

  if      (dir == "up")    s_dpad_up    = pressed;
  else if (dir == "down")  s_dpad_down  = pressed;
  else if (dir == "left")  s_dpad_left  = pressed;
  else if (dir == "right") s_dpad_right = pressed;
  else return;

  dpadRecompute();
}

// The mecanum app's mask carries the COMPLETE current button state rather than
// one transition, so a dropped packet is corrected by the next one and stale
// queued events never need replaying. Diagonals need no protocol support: a
// corner press just sets two cardinal bits at once, which dpadRecompute()
// already sums. Mask 0 is the centre STOP, and also every-button-released.
static void handleDpadMask(uint8_t mask) {
  s_dpad_up    = (mask & 0x1) != 0;
  s_dpad_down  = (mask & 0x2) != 0;
  s_dpad_left  = (mask & 0x4) != 0;
  s_dpad_right = (mask & 0x8) != 0;
  dpadRecompute();
}

// np_effect's options list is the one remaining string coupling: these names
// must match the select in the layout JSON.
static uint8_t effectFromName(const String& name) {
  if (name == "Solid")        return NP_EFFECT_SOLID;
  if (name == "Rainbow")      return NP_EFFECT_RAINBOW;
  if (name == "Knight Rider") return NP_EFFECT_KNIGHT;
  if (name == "Duel eye")     return NP_EFFECT_DUEL;
  return NP_EFFECT_FRENCH;
}

// The inverse of effectFromName. Needed because a select cannot be echoed
// back as a number: the app matches the value against its options string.
static const char* effectName(uint8_t e) {
  if (e == NP_EFFECT_SOLID)   return "Solid";
  if (e == NP_EFFECT_RAINBOW) return "Rainbow";
  if (e == NP_EFFECT_KNIGHT)  return "Knight Rider";
  if (e == NP_EFFECT_DUEL)    return "Duel eye";
  return "French flag";
}

// Whichever of the four controls for a wheel you did not touch follows along
// instead of showing a stale number, so the slider, the field and the buttons
// can never disagree about what the trim actually is.
static void trimTouched(void) {
  s_trim_echo_due = true;
  s_trim_save_due = true;
  s_trim_save_at  = millis() + TRIM_SAVE_SETTLE_MS;
}

static void handleWidget(const String& id, const String& val) {
  // --- DRIVE ---------------------------------------------------------------
  if (id == "joy_drive")   { handleJoystick(val); return; }
  if (id == "dpad_drive")  { handleDpad(val); return; }
  // Sonar head. Not persisted the way the trims are: a trim is a calibration
  // you want back after a power cycle, whereas the head is a live control --
  // restoring yesterday's bearing at boot would point the sensor somewhere
  // nobody asked for. Echoed on connect instead, so the slider starts honest.
  if (id == "head")        { moveHead(val.toInt()); return; }

  if (id == "spd")         { s_speed_cap = (uint8_t)constrain(val.toInt(), 0, 100); return; }
  if (id == "btn_stop") {
    if (val != "1") return;   // act on press, ignore the release
    // Clear every input that could immediately re-assert motion, then stop
    // the servos outright rather than waiting for the next drive mix.
    s_joy_x = s_joy_y = 0;
    s_dpad_up = s_dpad_down = s_dpad_left = s_dpad_right = false;
    stopServos();
    return;
  }

  // Anything arriving from the app is the robot being used. Without this the
  // face would fall asleep during a session spent on sliders and toggles,
  // since only the wheels turning counts as activity otherwise.
  oled_note_command();

  // --- TRIM ----------------------------------------------------------------
  // Straight-line calibration, one per wheel. The slider and the numeric field
  // are two ways to set the same number; the buttons nudge it by one, which is
  // the only practical way to find the stop point on a servo whose usable
  // range is a couple of units wide.
  if (id == "trim_l" || id == "trim_r" ||
      id == "trim_l_num" || id == "trim_r_num") {
    const int t = constrain(val.toInt(), TRIM_MIN, TRIM_MAX);
    if (id == "trim_l" || id == "trim_l_num") s_trim_l = (int8_t)t;
    else                                      s_trim_r = (int8_t)t;
    trimTouched();
    return;
  }
  if (id == "trim_l_dn" || id == "trim_l_up" ||
      id == "trim_r_dn" || id == "trim_r_up") {
    if (val != "1") return;                 // press only, not the release
    const int step = (id == "trim_l_up" || id == "trim_r_up") ? 1 : -1;
    if (id == "trim_l_dn" || id == "trim_l_up") {
      s_trim_l = (int8_t)constrain(s_trim_l + step, TRIM_MIN, TRIM_MAX);
    } else {
      s_trim_r = (int8_t)constrain(s_trim_r + step, TRIM_MIN, TRIM_MAX);
    }
    trimTouched();
    return;
  }

  // --- SOUND ---------------------------------------------------------------
  if (id == "btn_horn")    { s_button_01 = (val == "1") ? 1 : 0; return; }
  if (id == "btn_buzz")    { if (val == "1") buzzer_beep(); return; }

  // --- LIGHTS --------------------------------------------------------------
  if (id == "toggle_led_r") { s_led_manual = true; s_led_r_on = (val == "1"); return; }
  if (id == "toggle_led_g") { s_led_manual = true; s_led_g_on = (val == "1"); return; }
  if (id == "toggle_np")   { s_np_on = (val == "1") ? 1 : 0; return; }
  // Colour arrives one channel at a time; recombine into the packed 0xRRGGBB
  // that neopixels_all_clear() wants.
  if (id == "np_r") { s_np_color = (s_np_color & 0x00FFFF) | ((uint32_t)constrain(val.toInt(),0,255) << 16); return; }
  if (id == "np_g") { s_np_color = (s_np_color & 0xFF00FF) | ((uint32_t)constrain(val.toInt(),0,255) <<  8); return; }
  if (id == "np_b") { s_np_color = (s_np_color & 0xFFFF00) | ((uint32_t)constrain(val.toInt(),0,255)      ); return; }
  if (id == "np_effect")   { s_np_effect = effectFromName(val); return; }
  if (id == "np_bright")   { s_np_bright = (uint8_t)constrain(val.toInt(), 0, 255); return; }

  // --- DISPLAY -------------------------------------------------------------
  // Text typed in the app appears on the robot's physical OLED. Clearing the
  // field restores the normal banner. s_telemForce is raised so the mirror
  // label re-publishes immediately rather than waiting for a change.
  if (id == "oled_text") {
    oled_text_set(val.c_str());
    s_telemForce = true;
    return;
  }

  // Which of the four the physical screen shows: the status readout, the
  // face, the sonar scope, or Auto -- status until the app connects, then the
  // face. A typed oled_text still outranks all of them.
  if (id == "screen_mode") {
    oled_screen_mode_set(val.c_str());
    s_telemForce = true;
    return;
  }

  // Five looks for the face. A style is a look, not a behaviour: all five
  // blink, worry, are startled and fall asleep.
  if (id == "face_style") {
    oled_face_style_set(val.c_str());
    return;
  }

  // Whether the pupils track the driving direction. Persisted beside the trim
  // so a robot keeps the character it was given rather than resetting every
  // time it is switched on.
  if (id == "eyes_follow") {
    oled_eyes_follow_set(val == "1");
    if (s_prefs.begin("b3", false)) {
      s_prefs.putBool("eyes", oled_eyes_follow_get());
      s_prefs.end();
    }
    return;
  }

  // --- SYSTEM --------------------------------------------------------------
  if (id == "level") {
    uint8_t want = LAYOUT_BEGINNER;
    for (uint8_t i = 0; i < LAYOUT_COUNT; ++i) {
      if (val == LEVEL_NAMES[i]) { want = i; break; }
    }
    if (want == s_layout_level) return;
    layoutLevelStore(want);
    // Re-send the whole CFG so the app re-renders on the new layout with no
    // reconnect and no Reload Config. Deliberately via the deferred flag and
    // NOT by calling sendCfg() here: this runs inside NimBLE's host task, and
    // running the burst synchronously from it is the documented ENOMEM bug
    // that s_getCfgRequested exists to avoid.
    s_getCfgRequested = true;
    return;
  }
  if (id == "upd") {
    if (val == "Off")        s_upd_level = UPD_OFF;
    else if (val == "Basic") s_upd_level = UPD_BASIC;
    else                     s_upd_level = UPD_ALL;
    return;
  }
}

// ===========================================================================
// Public API (same signatures as the old RemoteXY adapter)
// ===========================================================================
void remotexy_init(void) {
  #ifdef DEF_DERIAL_DEBUG
  Serial.println("[BLE] init - device_name: " CONFIG_BLE_DEVICE_NAME);
  #endif

  // Restore the Beginner/Expert choice before advertising, so the very first
  // GETCFG after a power cycle already answers with the right layout.
  layoutLevelLoad();
  // layoutLevelLoad() runs before anything else has touched the screen, so the
  // restored value is handed to the OLED module here rather than there.
  oled_eyes_follow_set(s_eyes_follow_boot);
  #ifdef DEF_DERIAL_DEBUG
  Serial.printf("[BLE] layout level: %s\n",
                LEVEL_NAMES[s_layout_level]);
  #endif

  NimBLEDevice::init(CONFIG_BLE_DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  // No explicit setMTU() call — NimBLE-Arduino's own built-in default
  // preferred MTU (255) is what sendCfg() sizes its chunks against.
  NimBLEDevice::setSecurityAuth(false, false, false);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  NimBLEDevice::deleteAllBonds();

  #ifdef DEF_DERIAL_DEBUG
  Serial.printf("[BLE] local_mac: %s\n", NimBLEDevice::getAddress().toString().c_str());
  #endif

  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new RemoteXYServerCallbacks());

  NimBLEService* svc = server->createService(UART_SERVICE_UUID);

  s_txChar = svc->createCharacteristic(UART_TX_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);

  NimBLECharacteristic* rxChar = svc->createCharacteristic(
      UART_RX_CHAR_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  rxChar->setCallbacks(new RemoteXYRxCallbacks());

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->setName(CONFIG_BLE_DEVICE_NAME);
  adv->enableScanResponse(false);
  adv->setMinInterval(0x140);  // 320 * 0.625ms = 200ms
  adv->setMaxInterval(0x140);
  NimBLEDevice::startAdvertising();

  #ifdef DEF_DERIAL_DEBUG
  Serial.printf("[BLE] service_uuid: %s\n", UART_SERVICE_UUID);
  Serial.printf("[BLE] tx_char_uuid: %s (notify)\n", UART_TX_CHAR_UUID);
  Serial.printf("[BLE] rx_char_uuid: %s (write)\n", UART_RX_CHAR_UUID);
  Serial.printf("[BLE] advertising: %s\n", (adv && adv->isAdvertising()) ? "YES" : "no");
  #endif
}

// Config revision reported to the app. FNV-1a over the layout blob that is
// actually being served, so switching Level yields a different token and the
// app cannot reuse the wrong panel from cache. The firmware version is folded
// in as well: a rebuild that changes a blob's contents changes the hash, but
// folding the version in means an edit that happens to collide still cannot
// leave a stale panel cached on someone's phone.
static String layoutRevision(void) {
  uint32_t h = 2166136261u;
  const char* p = LAYOUT_BLOBS[s_layout_level];
  while (*p) { h ^= (uint8_t)*p++; h *= 16777619u; }
  for (const char* v = B3_FIRMWARE_VERSION; *v; ++v) { h ^= (uint8_t)*v; h *= 16777619u; }
  char buf[9];
  snprintf(buf, sizeof(buf), "%08x", (unsigned)h);
  return String(buf);
}

void remotexy_handler(void) {
  // Runs the deferred CFG burst on the ordinary Arduino task, not on
  // NimBLE's host task — see s_getCfgRequested for why that matters.
  if (s_getCfgRequested) {
    s_getCfgRequested = false;
    sendCfg();
  }

  // One short notification, and on a cache hit it replaces the entire transfer.
  if (s_getCfgVerRequested) {
    s_getCfgVerRequested = false;
    const String rev = layoutRevision();
    sendLine("CFGVER " + rev);
    #ifdef DEF_DERIAL_DEBUG
    Serial.printf("[BLE] CFGVER %s (layout %u)\n", rev.c_str(), (unsigned)s_layout_level);
    #endif
  }
}

int8_t remotexy_get_joystick_01_x( ) {
  return s_joy_x;
}

int8_t remotexy_get_joystick_01_y( ) {
  return s_joy_y;
}

uint8_t remotexy_get_button_01( ) {
  return s_button_01;
}

float remotexy_get_onlineGraph_01_distance( ) {
  return 0;  // output-only widget, never read back
}

float remotexy_get_onlineGraph_02_speed( ) {
  return 0;
}

float remotexy_get_onlineGraph_03_battery( ) {
  return 0;
}

int8_t remotexy_get_circularBar_01( ) {
  return 0;
}

int16_t remotexy_get_sound_01( ) {
  return 0;
}

uint8_t remotexy_get_connect_flag( ) {
  return s_connected ? 1 : 0;
}

// ===========================================================================
// ===========================================================================
// ===========================================================================

void remotexy_set_joystick_01_x( int8_t p_joystick_01_x) {
  (void)p_joystick_01_x;  // not writable from firmware side
}

void remotexy_set_joystick_01_y( int8_t p_joystick_01_y) {
  (void)p_joystick_01_y;
}

void remotexy_set_button_01(uint8_t p_button_01) {
  (void)p_button_01;
}

// The three original gauges are "Basic" telemetry: they are what the robot is
// doing right now, so they survive everything except Telemetry = Off. All three
// display whole numbers, so a deadband of 1 costs nothing visible.
void remotexy_set_onlineGraph_01_distance(float p_onlineGraph_01_distance) {
  if (!modeHas(T_DIST)) return;
  if (s_upd_level == UPD_OFF) return;
  if (!telemChanged(p_onlineGraph_01_distance, s_lastDist, 1.0f)) return;
  sendValue("gauge_distance", String(p_onlineGraph_01_distance, 0));
}

void remotexy_set_onlineGraph_02_speed( float p_onlineGraph_02_speed) {
  if (!modeHas(T_SPEED)) return;
  if (s_upd_level == UPD_OFF) return;
  if (!telemChanged(p_onlineGraph_02_speed, s_lastSpeed, 1.0f)) return;
  sendValue("gauge_speed", String(p_onlineGraph_02_speed, 0));
}

void remotexy_set_onlineGraph_03_battery( float p_onlineGraph_03_battery) {
  if (!modeHas(T_BATT)) return;
  if (s_upd_level == UPD_OFF) return;
  if (!telemChanged(p_onlineGraph_03_battery, s_lastBatt, 1.0f)) return;
  sendValue("battery_level", String(p_onlineGraph_03_battery, 0));
}

// Deliberately NOT send-on-change. A graph is a time series: the app plots a
// point per message, so suppressing repeats would compress the flat stretches
// and distort the shape of the trace rather than just saving traffic. This is
// the one value that genuinely wants every cycle — which is why it is "All"
// only.
void remotexy_send_graph_distance(float cm) {
  if (!modeHas(T_GRAPH)) return;   // graph_dist is not on this panel
  if (s_upd_level != UPD_ALL) return;
  sendValue("graph_dist", String(cm, 0));
}

// Edge-triggered, with hysteresis: the app raises a toast per message, so
// this fires once on entering the obstacle band and re-arms only after the
// robot has backed well clear of it.
void remotexy_send_obstacle_alert(float cm) {
  if (!modeHas(T_ALERT)) return;   // alert is not on this panel
  if (s_upd_level == UPD_OFF) return;
  // A zero reading means "no echo", not "wall against the sensor" — treating
  // it as an obstacle would fire the alert continuously with nothing there.
  if (cm <= 0.0f) { return; }

  if (!s_obstacleLatched && cm < B3_OBSTACLE_CM) {
    s_obstacleLatched = true;
    sendValue("alert", "Obstacle " + String(cm, 0) + "cm");
  } else if (s_obstacleLatched && cm > B3_OBSTACLE_CLEAR_CM) {
    s_obstacleLatched = false;
  }
}

// Firmware version, uptime and pack voltage. Static-ish text, so "All" only —
// there is no value in spending a notification per cycle on a string that
// rarely changes.
void remotexy_send_system_labels(void) {
  if (!modeHas(T_VER | T_UPTIME | T_VBAT)) return;   // no labels on this panel
  if (s_upd_level != UPD_ALL) return;

  // The version string never changes at runtime — one send per CFG is enough.
  if (modeHas(T_VER) && (s_telemForce || !s_verSent)) {
    sendValue("lbl_ver", B3_FIRMWARE_VERSION);
    s_verSent = true;
  }

  // Uptime is only ever displayed to the second, so only send when the second
  // actually rolls over rather than on every telemetry cycle.
  const uint32_t upSec = g_elapsed_time_hours * 3600u
                       + g_elapsed_time_minutes * 60u
                       + g_elapsed_time_seconds;
  if (modeHas(T_UPTIME) && (s_telemForce || upSec != s_lastUptimeSec)) {
    s_lastUptimeSec = upSec;
    char up[16];
    snprintf(up, sizeof(up), "%02u:%02u:%02u",
             (unsigned)g_elapsed_time_hours,
             (unsigned)g_elapsed_time_minutes,
             (unsigned)g_elapsed_time_seconds);
    sendValue("lbl_uptime", String(up));
  }

  // The battery widget already shows percentage; the raw voltage is what
  // actually reveals a sagging cell, so publish both. 0.05V deadband keeps
  // ADC noise from making the last decimal flicker every cycle.
  if (modeHas(T_VBAT) && telemChanged(g_battery_voltage, s_lastVbat, 0.05f)) {
    sendValue("lbl_vbat", String(g_battery_voltage, 2) + "V");
  }
}

// Board push button, mirrored as an indicator. On change only — it is the only
// physical input the robot has and it spends nearly all of its time released.
void remotexy_send_button_state(void) {
  if (!modeHas(T_BTN)) return;   // led_button is not on this panel
  if (s_upd_level == UPD_OFF) return;
  const int8_t now = button_pressed() ? 1 : 0;
  if (!s_telemForce && now == s_lastBtn) return;
  s_lastBtn = now;
  sendValue("led_button", now ? "1" : "0");
}

// BLE link quality. ble_gap_conn_rssi() is a host-side query against the live
// connection, so it needs the handle captured in onConnect() and returns
// non-zero if that handle is stale — in which case publish nothing rather
// than a fabricated number.
void remotexy_send_link_rssi(void) {
  if (!modeHas(T_RSSI)) return;   // gauge_rssi is not on this panel
  if (s_upd_level != UPD_ALL) return;
  if (s_connHandle == BLE_HS_CONN_HANDLE_NONE) return;
  int8_t rssi = 0;
  if (ble_gap_conn_rssi(s_connHandle, &rssi) != 0) return;
  // 2 dBm deadband: RSSI drifts a dB or so continuously even on a stationary
  // robot, which would otherwise make this the chattiest widget on the panel.
  if (!s_telemForce && abs((int)rssi - (int)s_lastRssi) < 2) return;
  s_lastRssi = rssi;
  sendValue("gauge_rssi", String((int)rssi));
}

// Mirrors the OLED's top line back to the panel, so the virtual display shows
// what the physical one actually reads rather than what was requested. They can
// differ: text longer than 21 characters is truncated to the panel width, and
// with no override the screen shows its banner instead. Sent on change only —
// this string is static most of the time.
void remotexy_send_oled_mirror(void) {
  if (!modeHas(T_OLED)) return;   // lbl_oled is not on this panel
  if (s_upd_level == UPD_OFF) return;
  // With a picture on the glass there is no top line to mirror, so report
  // WHICH picture instead: a label reading "Workshop.3" over a screen showing
  // two eyes would be describing a screen that is not there any more.
  const String now = oled_text_active()
                       ? String(oled_text_get())
                       : (strcmp(oled_screen_mode_name(), "Status") != 0
                            ? String(oled_screen_mode_name())
                            : (remotexy_get_connect_flag()
                                 ? String("Workshop.3")
                                 : String(CONFIG_BLE_DEVICE_NAME)));
  if (!s_telemForce && now == s_lastOled) return;
  s_lastOled = now;
  sendValue("lbl_oled", now);
  // Echo the field itself too, so a reconnect repopulates what was typed
  // instead of showing an empty box over a screen that still has text on it.
  // The typed text itself, so a reconnect repopulates the field instead of
  // showing an empty box over a screen that still has text on it. The Screen
  // and Face style selects are echoed by remotexy_send_control_echo() with
  // every other control, rather than separately here.
  if (s_telemForce) sendValue("oled_text", oled_text_active() ? oled_text_get() : "");
}

// Physical state of the two board LEDs. Sent on change; because these report
// intent rather than blink phase they are steady, so a parked robot costs
// nothing here even while the status indicator is actively flashing.
void remotexy_send_led_state(void) {
  if (!modeHas(T_LEDS)) return;   // led_r/g_state is not on this panel
  if (s_upd_level == UPD_OFF) return;
  const int8_t r = leds_state_r() ? 1 : 0;
  const int8_t g = leds_state_g() ? 1 : 0;
  if (s_telemForce || r != s_lastLedR) { s_lastLedR = r; sendValue("led_r_state", r ? "1" : "0"); }
  if (s_telemForce || g != s_lastLedG) { s_lastLedG = g; sendValue("led_g_state", g ? "1" : "0"); }
}

// Echoes the two selectors back to the app. Without this they render showing
// their FIRST option regardless of the real state — a panel serving the expert
// layout would still read "Level: Beginner", and "Telemetry: Off" while
// telemetry was visibly streaming. Deliberately NOT gated on s_upd_level: the
// Telemetry selector must show its own true position even when set to Off,
// otherwise there is no way to see what you are about to change.
// The panel's controls, as an indexed list. Returns false once `i` is past the
// end for whichever panel is being served.
//
// A LIST rather than a burst of sends, because sending them all in one pass is
// what broke this. Seventeen notify() calls back to back is exactly the
// pattern this radio has a documented history of drowning in with rc=6
// (BLE_HS_ENOMEM) -- sendCfg() delays 50ms per chunk for the same reason. The
// symptom was precise and misleading: the trim SLIDERS arrived and the trim
// FIELDS did not, because the sliders happen to be sent first and the fields
// were far enough down the burst to be dropped. Nothing was wrong with either
// widget; the tail of the burst simply never left.
static bool echoEntry(uint8_t i, String& id, String& val) {
  uint8_t k = 0;
  #define ECHO_ENTRY(cond, wid, wval)                 \
    do { if (cond) { if (k == i) { id = wid; val = wval; return true; } k++; } } while (0)

  // `level` is in every layout — it is the way back — so it is always echoed.
  ECHO_ENTRY(true, "level", LEVEL_NAMES[s_layout_level]);
  ECHO_ENTRY(modeHas(T_SPDCAP), "spd", String((int)s_speed_cap));

  ECHO_ENTRY(modeHas(T_LEDS), "toggle_np",    s_np_on ? "1" : "0");
  ECHO_ENTRY(modeHas(T_LEDS), "np_effect",    effectName(s_np_effect));
  ECHO_ENTRY(modeHas(T_LEDS), "np_bright",    String((int)s_np_bright));
  ECHO_ENTRY(modeHas(T_LEDS), "np_r",         String((int)((s_np_color >> 16) & 0xFF)));
  ECHO_ENTRY(modeHas(T_LEDS), "np_g",         String((int)((s_np_color >>  8) & 0xFF)));
  ECHO_ENTRY(modeHas(T_LEDS), "np_b",         String((int)( s_np_color        & 0xFF)));
  ECHO_ENTRY(modeHas(T_LEDS), "toggle_led_r", s_led_r_on ? "1" : "0");
  ECHO_ENTRY(modeHas(T_LEDS), "toggle_led_g", s_led_g_on ? "1" : "0");

  ECHO_ENTRY(modeHas(T_TRIM), "trim_l",     String((int)s_trim_l));
  ECHO_ENTRY(modeHas(T_TRIM), "trim_r",     String((int)s_trim_r));
  ECHO_ENTRY(modeHas(T_TRIM), "trim_l_num", String((int)s_trim_l));
  ECHO_ENTRY(modeHas(T_TRIM), "trim_r_num", String((int)s_trim_r));

  // The radar reads its bearing from this widget, so this echo is not
  // cosmetic the way a selector's is: without it the radar would plot every
  // blip at the slider default until the head was moved by hand once.
  ECHO_ENTRY(modeHas(T_HEAD), "head", String((int)headAngle()));

  ECHO_ENTRY(modeHas(T_OLED), "screen_mode", oled_screen_mode_name());
  ECHO_ENTRY(modeHas(T_OLED), "face_style",  oled_face_style_name());
  ECHO_ENTRY(modeHas(T_OLED), "eyes_follow", oled_eyes_follow_get() ? "1" : "0");

  ECHO_ENTRY(modeHas(T_UPDSEL), "upd",
             s_upd_level == UPD_OFF ? "Off" : s_upd_level == UPD_BASIC ? "Basic" : "All");
  #undef ECHO_ENTRY
  return false;
}

// Walked a few entries per telemetry pass rather than all at once. At the
// 500ms pass rate a full panel finishes in about three seconds, which is
// nothing against a CFG transfer that has just taken several -- and unlike the
// burst, every value actually arrives.
#define ECHO_PER_PASS 3
static uint8_t s_echo_i      = 0;
static bool    s_echo_active = false;

void remotexy_send_control_echo(void) {
  // A CFG transfer means the panel has just been rendered with whatever the
  // layout declared, so the walk restarts from the top.
  if (s_telemForce) { s_echo_i = 0; s_echo_active = true; }
  if (!s_echo_active || s_sendingCfg) return;

  for (uint8_t n = 0; n < ECHO_PER_PASS; n++) {
    String id, val;
    if (!echoEntry(s_echo_i, id, val)) { s_echo_active = false; return; }
    // notify() reports its own failure, so a dropped send is retried on the
    // next pass instead of being lost the way the old burst lost its tail.
    if (!sendLine("UPD " + id + " " + val)) return;
    s_echo_i++;
  }
}

// Called once at the end of each telemetry pass. Clearing the force flag here
// rather than inside a particular sender means the "send everything" pass is
// exactly one full cycle, whichever widgets happen to be in the layout.
void remotexy_telemetry_end(void) {
  s_telemForce = false;
}

// Called from the same pass. The write is deferred until the value has been
// still for TRIM_SAVE_SETTLE_MS, so dragging a slider across its range costs
// one flash write rather than forty.
void remotexy_trim_tick(void) {
  // The same staging, in miniature. Touching one control echoes to the other
  // three, and firing all four at once is the same burst that lost the fields.
  if (s_trim_echo_due) {
    s_trim_echo_due = false;
    s_trim_echo_i = modeHas(T_TRIM) ? 0 : 4;
  }
  for (uint8_t n = 0; n < 2 && s_trim_echo_i < 4; n++) {
    const char* wid = s_trim_echo_i == 0 ? "trim_l"
                    : s_trim_echo_i == 1 ? "trim_r"
                    : s_trim_echo_i == 2 ? "trim_l_num" : "trim_r_num";
    const int8_t v = (s_trim_echo_i == 0 || s_trim_echo_i == 2) ? s_trim_l : s_trim_r;
    if (s_sendingCfg) break;
    if (!sendLine("UPD " + String(wid) + " " + String((int)v))) break;  // retry next pass
    s_trim_echo_i++;
  }
  if (!s_trim_save_due || (int32_t)(millis() - s_trim_save_at) < 0) return;
  s_trim_save_due = false;
  if (s_prefs.begin("b3", false)) {
    s_prefs.putChar("trimL", s_trim_l);
    s_prefs.putChar("trimR", s_trim_r);
    s_prefs.end();
  }
}

uint8_t remotexy_get_led_manual(void) { return s_led_manual ? 1 : 0; }
uint8_t remotexy_get_led_r(void)      { return s_led_r_on ? 1 : 0; }
uint8_t remotexy_get_led_g(void)      { return s_led_g_on ? 1 : 0; }

void remotexy_set_circularBar_01( int8_t p_circularBar_01) {
  (void)p_circularBar_01;  // covered by gauge_speed already
}

// Maps RemoteXY's sound catalog (07_sounds.h) onto bit-rxy's Sound widget,
// which only has 5 tone effects (see bit-rxy's playSoundEffect()) rather
// than a full audio-file library. p_sound_01 == 0 is tasks_rmotexy_sound()'s
// periodic "no sound" reset — must not itself trigger a sound.
void remotexy_set_sound_01(int16_t p_sound_01) {
  if (p_sound_01 == 0) return;

  const char* effect;
  switch (p_sound_01) {
    case REMOTEXY_SOUND_SUCCESS:
    case REMOTEXY_SOUND_SUCCESS_ALT:
    case REMOTEXY_SOUND_SUCCESS_BEEP:
    case REMOTEXY_SOUND_POWER_ON:
    case REMOTEXY_SOUND_CONFIRM:
    case REMOTEXY_SOUND_CHIME:
      effect = "success"; break;
    case REMOTEXY_SOUND_WARNING:
    case REMOTEXY_SOUND_TIMER:
    case REMOTEXY_SOUND_COUNTDOWN:
      effect = "warn"; break;
    case REMOTEXY_SOUND_ERROR:
    case REMOTEXY_SOUND_CRITICAL:
    case REMOTEXY_SOUND_ALARM:
    case REMOTEXY_SOUND_FAILURE:
    case REMOTEXY_SOUND_ERROR_BEEP:
      effect = "danger"; break;
    case REMOTEXY_SOUND_DOUBLE_BEEP:
    case REMOTEXY_SOUND_SELECT:
      effect = "toggle"; break;
    default:
      effect = "beep"; break;  // BEEP, BEEP_SHORT, BEEP_LONG, CLICK, TAP, etc.
  }
  sendValue("sound_alert", effect);
}

void remotexy_set_connect_flag( uint8_t p_connect_flag) {
  (void)p_connect_flag;  // connect state is owned by the BLE server callbacks
}
