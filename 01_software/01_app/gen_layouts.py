"""Build the four layouts this board can actually serve, and splice them in.

b3 ships eight panels. Five of them drive hardware that is not on this board --
buzzer, NeoPixel strip, two board LEDs, OLED, battery sense -- so Beginner and
Expert are rebuilt here without those widgets, and only the Drive and Distance
test panels survive.

Widget ids are the ones handleWidget() in 03_bit-rxy.cpp actually handles. A
widget whose id has no handler is worse than a missing widget: it looks live
and does nothing, which is exactly the "lbl_hint shows 0" class of bug.

Reads and rewrites the LAYOUT_CFG_*_BASE64 blobs in place, so this is the one
place the panels are defined.
"""
import base64, itertools, json, os, re, textwrap

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = f"{HERE}/01_src/03_bit-rxy.cpp"
PAD, TITLE = 24, 34

# Must match LEVEL_NAMES[] and the layout enum in 03_bit-rxy.h.
LEVELS = "Beginner,Expert,Drive,Distance"


def W(wid, t, x, y, w, h, **kw):
    d = dict(id=wid, t=t, x=x, y=y, w=w, h=h)
    d.update(kw)
    return d


def group(gid, label, color, members):
    x1 = min(m["x"] for m in members); y1 = min(m["y"] for m in members)
    x2 = max(m["x"] + m["w"] for m in members); y2 = max(m["y"] + m["h"] for m in members)
    for m in members:
        m["groupId"] = gid
    return dict(id=gid, t="group", label=label, color=color,
                x=x1 - PAD, y=y1 - PAD - TITLE,
                w=(x2 + PAD) - (x1 - PAD), h=(y2 + PAD) - (y1 - PAD - TITLE),
                children=[m["id"] for m in members])


def build(title, zones, extra=()):
    groups, controls = [], []
    for gid, label, color, members in zones:
        groups.append(group(gid, label, color, members))
        controls += members
    widgets = groups + list(extra) + controls
    cfg = {"schemaVersion": 2, "title": title, "widgets": widgets,
           "canvas": {"w": max(w["x"] + w["w"] for w in widgets) + 56,
                      "h": max(w["y"] + w["h"] for w in widgets) + 56}}

    box = lambda w: (w["x"], w["y"], w["x"] + w["w"], w["y"] + w["h"])
    def ov(a, b):
        a1, b1, a2, b2 = box(a); c1, d1, c2, d2 = box(b)
        return not (a2 <= c1 or c2 <= a1 or b2 <= d1 or d2 <= b1)
    errs = []
    by = {w["id"]: w for w in widgets}
    for g in groups:
        gx1, gy1, gx2, gy2 = box(g)
        for cid in g["children"]:
            x1, y1, x2, y2 = box(by[cid])
            if min(x1 - gx1, gx2 - x2, gy2 - y2) < PAD:
                errs.append(f"{cid} padding in {g['label']}")
            if y1 - gy1 < PAD + TITLE:
                errs.append(f"{cid} under the {g['label']} header")
    for a, b in itertools.combinations(groups, 2):
        if ov(a, b): errs.append(f"group overlap {a['label']} ~ {b['label']}")
    # `extra` widgets sit outside every group but must still not collide with
    # one, and must fit the canvas with room for the margin.
    for e in list(extra):
        for o in groups + controls:
            if ov(e, o): errs.append(f"{e['id']} overlaps {o['id']}")
    for a, b in itertools.combinations(controls, 2):
        if ov(a, b): errs.append(f"control overlap {a['id']} ~ {b['id']}")
    assert not errs, f"{title}: {errs}"
    return cfg


# 110, not the 60 b3 used. Measured in the browser: this label renders its text
# at 75px for two lines inside a 60px box, so the last line was clipped on both
# test panels. The extra room also absorbs a longer translation.
HINT_H = 110


def hint(x, y, w, text):
    """The one-line instruction under a test panel. Deliberately outside the
    group: it explains the exercise, it is not part of it."""
    return W("lbl_hint", "label", x, y, w, HINT_H, label=text, value=text, model="card")


LOGO = lambda x, y: W("logo", "image", x, y, 192, 164, label="Workshop-DIY",
                      imageSrc="assets/workshop-diy-logo.svg")
LEVEL = lambda x, y: W("level", "select", x, y, 160, 70, label="Level", options=LEVELS)

# ── BEGINNER ────────────────────────────────────────────────────────────────
# One way to drive, one number to watch. No speed slider: at this level the
# pad IS the speed control, and a second one only invites "why won't it move".
beginner = build("WDIY Servo-Sonar - Beginner", [
    ("grp_drive", "DRIVE", "#00d4ff", [
        W("dpad_drive", "dpad", 80, 100, 340, 340, label="Drive", model="classic"),
    ]),
    ("grp_see", "WHAT IT SEES", "#ffb020", [
        W("gauge_distance", "gauge", 500, 100, 220, 200, label="Distance",
          min=0, max=200, units="cm", decimals=0, model="classic"),
        W("alert", "notification", 500, 330, 220, 110, label="Obstacle"),
    ]),
    ("grp_sys", "ROBOT", "#3ddc97", [
        LEVEL(80, 640),
        LOGO(280, 600),
    ]),
])

# ── EXPERT ──────────────────────────────────────────────────────────────────
# Everything the board can do. Same three-zone shape as b3's expert panel with
# the LIGHTS, SOUND and DISPLAY zones gone.
expert = build("WDIY Servo-Sonar - Expert", [
    ("grp_drive", "DRIVE", "#00d4ff", [
        W("dpad_drive", "dpad", 80, 100, 320, 320, label="Drive", model="classic"),
        W("joy_drive", "joystick", 440, 100, 300, 300, label="Steer"),
        W("spd", "slider", 790, 100, 90, 220, label="Speed",
          min=0, max=100, step=5, value=100),
        W("btn_stop", "button", 790, 350, 90, 90, label="STOP"),
        W("gauge_speed", "gauge", 940, 110, 200, 190, label="Speed",
          min=0, max=100, decimals=0, model="min"),
    ]),
    ("grp_dist", "DISTANCE", "#ffb020", [
        W("gauge_distance", "gauge", 80, 600, 220, 200, label="Distance",
          min=0, max=200, units="cm", decimals=0, model="classic"),
        W("alert", "notification", 330, 610, 90, 90, label="Obstacle"),
        W("graph_dist", "graph", 460, 600, 480, 210, label="Distance cm",
          model="grid", windowSec=30, series=1),
        W("sound_alert", "sound", 980, 610, 90, 90, label="Alert"),
    ]),
    ("grp_sys", "SYSTEM", "#3ddc97", [
        W("lbl_ver", "label", 80, 950, 200, 50, label="Firmware", model="card"),
        W("lbl_uptime", "label", 80, 1030, 200, 50, label="Uptime", model="card"),
        W("upd", "select", 320, 950, 160, 70, label="Telemetry", options="Off,Basic,All"),
        LEVEL(320, 1040),
        W("led_button", "led", 520, 960, 80, 80, label="Button",
          model="dot", colorOn="#00ff88"),
        W("gauge_rssi", "gauge", 640, 950, 189, 190, label="Signal",
          min=-100, max=-30, units="dBm", decimals=0, model="classic"),
        LOGO(870, 960),
    ]),
])

# ── TEST PANELS ─────────────────────────────────────────────────────────────
# One subsystem each, big targets, one instruction. Rebuilt here rather than
# carried over from b3 so they go through the same checks as the other two --
# carrying them over unvalidated is exactly how the clipped hint shipped.
test_drive = build("Servo-Sonar - Drive test", [
    ("grp_test", "MOTORS", "#00d4ff", [
        W("dpad_drive", "dpad", 80, 100, 300, 300, label="Drive", model="classic"),
        W("spd", "slider", 420, 100, 90, 200, label="Speed",
          min=0, max=100, step=5, value=100),
        W("btn_stop", "button", 420, 330, 120, 120, label="STOP"),
        W("gauge_speed", "gauge", 570, 100, 150, 190, label="Speed %",
          min=0, max=100, decimals=0, model="min"),
        W("level", "select", 80, 490, 200, 70, label="Test", options=LEVELS),
    ]),
], extra=[hint(80, 600, 640, "Press an arrow. The wheels should turn that way.")])

test_distance = build("Servo-Sonar - Distance test", [
    ("grp_test", "DISTANCE", "#ffb020", [
        W("gauge_distance", "gauge", 80, 100, 150, 190, label="Distance cm",
          min=0, max=200, units="cm", decimals=0, model="classic"),
        W("alert", "notification", 260, 110, 110, 110, label="Obstacle"),
        W("graph_dist", "graph", 80, 320, 380, 200, label="Distance cm",
          model="grid", windowSec=30, series=1),
        W("level", "select", 80, 560, 200, 70, label="Test", options=LEVELS),
    ]),
], extra=[hint(80, 670, 380, "Move your hand in front of the sensor.")])

NEW = {"BEGINNER": beginner, "EXPERT": expert,
       "TEST_DRIVE": test_drive, "TEST_DISTANCE": test_distance}

# ── splice ──────────────────────────────────────────────────────────────────
src = open(SRC, encoding="utf-8").read()


def blob_of(name):
    # Must include the `static const char*` prefix: emit() writes the whole
    # declaration, so matching only from LAYOUT_CFG_ leaves the old prefix
    # behind and doubles it on every run.
    m = re.search(rf'static const char\* LAYOUT_CFG_{name}_BASE64 =\s*((?:\s*"[^"]*"\s*)+);', src)
    assert m, name
    return m, "".join(re.findall(r'"([^"]*)"', m.group(1)))


def emit(name, cfg):
    """Render as the wrapped C string literal the file already uses."""
    b64 = base64.b64encode(
        json.dumps(cfg, separators=(",", ":"), ensure_ascii=False).encode()).decode()
    body = "\n".join(f'  "{c}"' for c in textwrap.wrap(b64, 72))
    return f"static const char* LAYOUT_CFG_{name}_BASE64 =\n{body};", len(b64)


report = []
for name in ("BEGINNER", "EXPERT", "TEST_DRIVE", "TEST_DISTANCE"):
    m, old_b64 = blob_of(name)
    cfg = NEW[name]
    text, n = emit(name, cfg)
    src = src[:m.start()] + text + src[m.end():]
    ids = [w["id"] for w in cfg["widgets"] if w["t"] not in ("group", "separator")]
    report.append((name, len(old_b64), n, ids))

open(SRC, "w", encoding="utf-8", newline="\n").write(src)

# Every id must have a handler or a telemetry sender; anything else is a dead
# control on the panel.
HANDLED = {"joy_drive", "dpad_drive", "spd", "btn_stop", "level", "upd"}
SENT = {"gauge_distance", "gauge_speed", "graph_dist", "alert", "sound_alert",
        "lbl_ver", "lbl_uptime", "gauge_rssi", "led_button"}
DECOR = {"logo", "lbl_hint"}
print(f"  {'panel':14} {'was':>6} {'now':>6}  widgets")
dead = []
for name, was, now, ids in report:
    print(f"  {name:14} {was:6} {now:6}  {len(ids)}")
    dead += [f"{name}:{i}" for i in ids if i not in HANDLED | SENT | DECOR]
print()
if dead:
    print("  FAILED - widgets with no handler and no telemetry:", dead)
    raise SystemExit(1)
print("  PASS - every widget on every panel is either driven or fed")
