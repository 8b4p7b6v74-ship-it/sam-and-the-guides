# =====================================================================
#  SAM & THE GUIDES  --  robot firmware (identical on all 3 robots)
# =====================================================================
#  The role (LEAD / GUARD / COMPANION) is assigned by the hub over
#  Bluetooth, so the SAME file runs on every robot.
#
#  Real hardware only: drive, line sensors, front ultrasonic, LED,
#  light 3D-printed arm (gesture/tap). No camera, no sound, no gripper.
#
#  Navigation = FLOOR MARKERS: the route is a line on the floor with a
#  perpendicular mark (both line sensors triggered at once) at every
#  decision point. The LEAD counts markers and, per a route string,
#  goes straight / turns / arrives. GUARD and COMPANION follow the line
#  too but hold their spacing to the robot ahead using the ultrasonic.
#
#  ---- Commands from the hub (newline-terminated) --------------------
#    ROLE LEAD | ROLE GUARD | ROLE COMPANION   assign this robot's role
#    ROUTE SSRA        set the LEAD's per-marker actions
#                        S=straight  L=left  R=right  A=arrive
#    GO                start the mission        STOP  halt (safe)
#    GAP 22            follower target gap (cm)
#    TAP               tap the arm (haptic cue)
#    w a s d q e       manual nudge (one letter, for testing)
#  ---- Telemetry (~5 Hz) ---------------------------------------------
#    T|role|mission|dist|lineL|lineR|marker|state
#  ---- Events --------------------------------------------------------
#    E|READY|hw   E|MARK|n   E|TURN|left   E|ARRIVE|   E|ALERT|obstacle
# =====================================================================

from XRPLib.defaults import *
import sys
import select
import time

# ---------------------------------------------------------------- config
CFG = {
    "loop_ms":     15,
    "tel_ms":      200,
    "base":        0.45,     # cruising effort
    "slew":        0.10,
    # line follow
    "kp":          1.7,
    "kd":          9.0,
    "line_dir":    1,
    "thresh":      0.50,
    "line_white":  True,
    "lost_spin":   0.40,
    # markers
    "mark_gap_ms": 700,      # min time between two markers (debounce)
    # obstacle (lead) / spacing (followers), cm
    "stop_cm":     18.0,
    "gap":         22.0,     # follower target distance to robot ahead
    "gap_band":    6.0,
    # turns
    "pivot_ms":    520,
    "pivot_eff":   0.55,
    # arm
    "arm_down":    40,
    "arm_up":      120,
}

LEAD, GUARD, COMPANION = "LEAD", "GUARD", "COMPANION"
ROLE = COMPANION            # default until the hub assigns one
MISSION = "IDLE"            # IDLE / RUN / ARRIVED
STATE = "READY"
ROUTE = []                  # list of action chars for the LEAD
MARK = 0                    # markers crossed this mission

HW = {"drive": False, "range": False, "line": False, "servo": False, "led": False}

# ---------------------------------------------------------------- utils
def clamp(v, lo=-1.0, hi=1.0):
    return lo if v < lo else hi if v > hi else v


def now():
    return time.ticks_ms()


def since(t):
    return time.ticks_diff(time.ticks_ms(), t)


def emit(kind, msg=""):
    print("E|%s|%s" % (kind, msg))


# ---------------------------------------------------------------- serial in
_poll = select.poll()
_poll.register(sys.stdin, select.POLLIN)
_buf = [""]


def _readchar():
    if _poll.poll(0):
        try:
            return sys.stdin.read(1)
        except Exception:
            return None
    return None


def read_line():
    """Return one full command line, or None. Buffers across calls."""
    ch = _readchar()
    while ch is not None:
        if ch == "\n" or ch == "\r":
            line = _buf[0]
            _buf[0] = ""
            if line != "":
                return line
        else:
            _buf[0] += ch
        ch = _readchar()
    return None


# ---------------------------------------------------------------- hardware
def probe():
    for name, fn in (
        ("drive", lambda: drivetrain.stop()),
        ("range", lambda: rangefinder.distance()),
        ("line", lambda: (reflectance.get_left(), reflectance.get_right())),
        ("servo", lambda: servo_one.set_angle(CFG["arm_down"])),
        ("led", lambda: board.led_off()),
    ):
        try:
            fn(); HW[name] = True
        except Exception:
            pass


def dist_cm():
    if not HW["range"]:
        return 999.0
    try:
        d = rangefinder.distance()
        return 999.0 if (d is None or d <= 0) else d
    except Exception:
        return 999.0


def line_lr():
    if not HW["line"]:
        return (0.0, 0.0)
    try:
        return (reflectance.get_left(), reflectance.get_right())
    except Exception:
        return (0.0, 0.0)


def on_line(v):
    return v < CFG["thresh"] if CFG["line_white"] else v > CFG["thresh"]


def led(on):
    if HW["led"]:
        try:
            board.led_on() if on else board.led_off()
        except Exception:
            pass


def arm(pos):
    if HW["servo"]:
        try:
            servo_one.set_angle(CFG["arm_up"] if pos == "up" else CFG["arm_down"])
        except Exception:
            pass


def arm_tap():
    arm("up"); time.sleep_ms(110); arm("down"); emit("ARM", "tap")


# ---------------------------------------------------------------- drive (slew)
class Drive:
    def __init__(self):
        self.cl = 0.0; self.cr = 0.0; self.tl = 0.0; self.tr = 0.0

    def arcade(self, f, t):
        self.tl = clamp(f + t); self.tr = clamp(f - t)

    def tank(self, l, r):
        self.tl = clamp(l); self.tr = clamp(r)

    def _s(self, c, t):
        s = CFG["slew"]; d = t - c
        return c + (s if d > s else -s if d < -s else d)

    def update(self):
        self.cl = self._s(self.cl, self.tl); self.cr = self._s(self.cr, self.tr)
        if HW["drive"]:
            try:
                drivetrain.set_effort(self.cl, self.cr)
            except Exception:
                pass

    def brake(self):
        self.cl = self.cr = self.tl = self.tr = 0.0
        if HW["drive"]:
            try:
                drivetrain.stop()
            except Exception:
                pass


SD = Drive()

# ---------------------------------------------------------------- shared state
LAST = {"err": 0.0, "sign": 1, "mark_t": 0}
MANU = {"fwd": 0.0, "turn": 0.0}
MOVE = {"on": False, "l": 0.0, "r": 0.0, "end": 0}


def start_move(l, r, ms):
    MOVE["on"] = True; MOVE["l"] = l; MOVE["r"] = r
    MOVE["end"] = time.ticks_add(now(), ms)


# ---------------------------------------------------------------- line follow
def follow(base):
    l, r = line_lr()
    if on_line(l) or on_line(r):
        err = (l - r) * CFG["line_dir"]
        d = err - LAST["err"]; LAST["err"] = err
        if err > 0.02:
            LAST["sign"] = 1
        elif err < -0.02:
            LAST["sign"] = -1
        SD.arcade(base, clamp(CFG["kp"] * err + CFG["kd"] * d, -1, 1))
        return True
    SD.arcade(0.0, CFG["lost_spin"] * LAST["sign"])
    return False


def marker_seen():
    """A perpendicular floor mark = BOTH sensors on the line at once."""
    l, r = line_lr()
    if on_line(l) and on_line(r):
        if since(LAST["mark_t"]) > CFG["mark_gap_ms"]:
            LAST["mark_t"] = now()
            return True
    return False


# ---------------------------------------------------------------- roles
def set_role(r):
    global ROLE
    if r in (LEAD, GUARD, COMPANION):
        ROLE = r
        emit("ROLE", r)


def set_route(s):
    global ROUTE
    ROUTE = [c for c in s.upper() if c in "SLRA"]
    emit("ROUTE", "".join(ROUTE))


def start_mission():
    global MISSION, STATE, MARK
    MISSION = "RUN"; STATE = "RUN"; MARK = 0
    LAST["err"] = 0.0; LAST["mark_t"] = now(); MOVE["on"] = False
    led(False); emit("GO", ROLE)


def stop_mission():
    global MISSION, STATE
    MISSION = "IDLE"; STATE = "STOP"; MOVE["on"] = False
    SD.brake(); emit("STOP", ROLE)


def arrive():
    global MISSION, STATE
    MISSION = "ARRIVED"; STATE = "ARRIVED"
    SD.brake(); led(True); arm_tap(); emit("ARRIVE", "")


# ---------- LEAD: line-follow + count markers + act per the route
def step_lead():
    global STATE, MARK
    if dist_cm() < CFG["stop_cm"]:            # a real obstacle ahead
        STATE = "ALERT"; led(True); SD.brake()
        emit("ALERT", "obstacle"); return
    led(False)
    if marker_seen():
        MARK += 1
        emit("MARK", str(MARK))
        act = ROUTE[MARK - 1] if MARK - 1 < len(ROUTE) else "A"
        if act == "A":
            arrive(); return
        elif act == "L":
            STATE = "TURN"; emit("TURN", "left")
            start_move(-CFG["pivot_eff"], CFG["pivot_eff"], CFG["pivot_ms"]); return
        elif act == "R":
            STATE = "TURN"; emit("TURN", "right")
            start_move(CFG["pivot_eff"], -CFG["pivot_eff"], CFG["pivot_ms"]); return
    STATE = "FOLLOW"
    follow(CFG["base"])


# ---------- GUARD / COMPANION: line-follow, hold the gap to the one ahead
def step_follower():
    global STATE
    d = dist_cm()
    gap = CFG["gap"]; band = CFG["gap_band"]
    if d < gap - band:                        # too close -> hold
        STATE = "HOLD"; SD.tank(0, 0); SD.update(); return
    if d < gap + band:                        # in the band -> ease off
        STATE = "KEEP"; follow(CFG["base"] * 0.55); return
    STATE = "FOLLOW"                          # gap open -> normal speed
    follow(CFG["base"])


def step_manual():
    global STATE
    STATE = "MANUAL"; SD.arcade(MANU["fwd"], MANU["turn"])


def step_idle():
    global STATE
    STATE = "IDLE"; SD.brake()


# ---------------------------------------------------------------- commands
def manual(k):
    global MISSION
    MISSION = "MANUAL"
    b = CFG["base"]
    if k == "w":
        MANU["fwd"], MANU["turn"] = b, 0.0
    elif k == "s":
        MANU["fwd"], MANU["turn"] = -b, 0.0
    elif k == "a":
        MANU["fwd"], MANU["turn"] = 0.0, -b * 0.8
    elif k == "d":
        MANU["fwd"], MANU["turn"] = 0.0, b * 0.8
    elif k == "q":
        MANU["fwd"], MANU["turn"] = b, -b * 0.35
    elif k == "e":
        MANU["fwd"], MANU["turn"] = b, b * 0.35


def handle(line):
    u = line.strip()
    if u == "":
        return
    up = u.upper()
    if up == "STOP" or u == " ":
        stop_mission()
    elif up == "GO":
        start_mission()
    elif up == "TAP":
        arm_tap()
    elif up.startswith("ROLE "):
        set_role(up[5:].strip())
    elif up in (LEAD, GUARD, COMPANION):
        set_role(up)
    elif up.startswith("ROUTE "):
        set_route(up[6:].strip())
    elif up.startswith("GAP "):
        try:
            CFG["gap"] = float(up[4:].strip())
        except Exception:
            pass
    elif up == "F":
        CFG["line_dir"] *= -1; emit("INFO", "dir=%d" % CFG["line_dir"])
    elif len(u) == 1 and u.lower() in "wasdqe":
        manual(u.lower())


# ---------------------------------------------------------------- telemetry
_last_tel = [0]


def tel_tick():
    if since(_last_tel[0]) < CFG["tel_ms"]:
        return
    _last_tel[0] = now()
    l, r = line_lr()
    print("T|%s|%s|%.1f|%.2f|%.2f|%d|%s" % (
        ROLE, MISSION, dist_cm(), l, r, MARK, STATE))


# ---------------------------------------------------------------- main
def run():
    probe()
    hw = ",".join(k for k, v in HW.items() if v) or "none"
    emit("READY", hw)
    while True:
        line = read_line()
        if line is not None:
            handle(line)

        if MOVE["on"]:                        # executing a timed turn
            SD.tank(MOVE["l"], MOVE["r"])
            if since(MOVE["end"]) >= 0:
                MOVE["on"] = False; SD.tank(0, 0)
        elif MISSION == "MANUAL":
            step_manual()
        elif MISSION == "RUN":
            if ROLE == LEAD:
                step_lead()
            else:
                step_follower()
        else:
            step_idle()

        SD.update()
        tel_tick()
        time.sleep_ms(CFG["loop_ms"])


if __name__ == "__main__":
    try:
        run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print("E|ERR|%s" % e)
    finally:
        try:
            SD.brake()
        except Exception:
            pass
        try:
            board.led_off()
        except Exception:
            pass
        emit("STOP", "halt")
