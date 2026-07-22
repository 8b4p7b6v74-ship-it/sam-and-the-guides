# =====================================================================
#  SAM & THE GUIDES  --  robot firmware v2 (identical on all 3 robots)
# =====================================================================
#  Formation:   OPENER  ->  SAM  ->  CLOSER      (travel direction ->)
#
#  OPENER : line sensors + front sonar (obstacles) + optional REAR
#           sonar (tracks Sam behind). Navigates: markers or PATH.
#  SAM    : NO external sensors (blind). Executes hub commands only:
#           speed, timed/gyro turns. Encoders+IMU (on-board) keep it
#           straight.
#  CLOSER : front sonar keeps a constant gap to Sam ahead; steers by
#           line if present, else heading-hold + mirrored turns.
#
#  ---- Commands (newline-terminated) ---------------------------------
#    ROLE OPENER|SAM|CLOSER      assign role
#    ROUTE SSRA                  line mode: marker actions S/L/R/A
#    PATH D150,R90,D80,A         no-line mode: cm / degrees / arrive
#    GO | STOP                   start / safe-halt
#    SPD 0.45                    cruising speed (hub speed control)
#    TURNL | TURNR               execute one 90-degree turn (SAM/CLOSER)
#    GAP 25                      follower target gap (cm)
#    TAP                         arm tap        F  invert line polarity
#    w a s d q e                 manual nudge (testing)
#  ---- Telemetry ~5 Hz -----------------------------------------------
#    T|role|mission|front|rear|lineL|lineR|prog|state
#      prog = markers crossed (line mode) or path step index
#  ---- Events --------------------------------------------------------
#    E|READY|hw  E|ROLE|r  E|MARK|n  E|STEP|n  E|TURN|left|right
#    E|ARRIVE|   E|ALERT|obstacle   E|LOST|sam   E|STOP|
# =====================================================================

from XRPLib.defaults import *
import sys
import select
import time

try:
    from machine import Pin, time_pulse_us
except Exception:
    Pin = None
    time_pulse_us = None

# ---------------------------------------------------------------- config
CFG = {
    "loop_ms":      15,
    "tel_ms":       200,
    "spd":          0.45,     # cruising effort (hub can change: SPD)
    "slew":         0.10,
    # line follow
    "kp":           1.7,
    "kd":           9.0,
    "line_dir":     1,
    "thresh":       0.50,
    "line_white":   True,
    "lost_spin":    0.40,
    "mark_gap_ms":  700,
    # sonar (cm)
    "stop_cm":      18.0,
    "gap":          25.0,     # follower gap to the robot ahead
    "gap_band":     6.0,
    "sam_lost_cm":  60.0,     # opener rear: Sam farther than this = lost
    # rear sonar (OPENER) -- HC-SR04 on the XRP "Qwiic 0" port = GPIO4 / GPIO5.
    # If it reads nothing, swap these two (wiring of Trig/Echo to SDA/SCL).
    "rear_trig":    5,
    "rear_echo":    4,
    # dead-reckoning (no-line mode)
    "wheel_circ":   18.85,    # cm per wheel revolution (XRP 6 cm wheel)
    "turn_tol":     4.0,      # deg
    "turn_eff":     0.5,
    "turn_ms_90":   560,      # timed fallback for 90 deg (no IMU)
    "arm_down":     40,
    "arm_up":       120,
}

OPENER, SAM, CLOSER = "OPENER", "SAM", "CLOSER"
ROLE = SAM
MISSION = "IDLE"              # IDLE / RUN / MANUAL / ARRIVED
STATE = "READY"
NAV = "LINE"                  # LINE (markers) or PATH (dead-reckoning)
ROUTE = []                    # marker actions, line mode
PATH = []                     # [("D",cm)|("L",deg)|("R",deg)|("A",0)]
PROG = 0                      # markers crossed / path step index

HW = {"drive": False, "front": False, "rear": False, "line": False,
      "imu": False, "enc": False, "servo": False, "led": False}
_rtrig = None
_recho = None
_rear_val = 999.0
_rear_t = 0

# ---------------------------------------------------------------- utils
def clamp(v, lo=-1.0, hi=1.0):
    return lo if v < lo else hi if v > hi else v


def now():
    return time.ticks_ms()


def since(t):
    return time.ticks_diff(time.ticks_ms(), t)


def emit(kind, msg=""):
    print("E|%s|%s" % (kind, msg))


_poll = select.poll()
_poll.register(sys.stdin, select.POLLIN)
_buf = [""]


def read_line():
    while _poll.poll(0):
        try:
            ch = sys.stdin.read(1)
        except Exception:
            return None
        if ch in ("\n", "\r"):
            line = _buf[0]
            _buf[0] = ""
            if line:
                return line
        else:
            _buf[0] += ch
    return None


# ---------------------------------------------------------------- hardware
def _heading():
    try:
        return imu.get_heading()
    except AttributeError:
        return imu.get_yaw()


def probe():
    global _rtrig, _recho
    for name, fn in (
        ("drive", lambda: drivetrain.stop()),
        ("front", lambda: rangefinder.distance()),
        ("line", lambda: (reflectance.get_left(), reflectance.get_right())),
        ("imu", _heading),
        ("enc", lambda: left_motor.get_position()),
        ("servo", lambda: servo_one.set_angle(CFG["arm_down"])),
        ("led", lambda: board.led_off()),
    ):
        try:
            fn(); HW[name] = True
        except Exception:
            pass
    if Pin and CFG["rear_trig"] is not None:
        try:
            _rtrig = Pin(CFG["rear_trig"], Pin.OUT)
            _recho = Pin(CFG["rear_echo"], Pin.IN)
            _rtrig.low(); HW["rear"] = True
        except Exception:
            _rtrig = None


def front_cm():
    if not HW["front"]:
        return 999.0
    try:
        d = rangefinder.distance()
    except Exception:
        return 999.0
    # reject noise & sensor-timeout: <3cm spikes and 0xFFFF(65535)/>4m out-of-range
    if d is None or d < 3.0 or d > 400.0:
        return 999.0
    return d


def rear_cm():
    global _rear_val, _rear_t
    if _rtrig is None:
        return 999.0
    if since(_rear_t) < 90:              # cache: never block the control loop
        return _rear_val
    _rear_t = now()
    try:
        _rtrig.low(); time.sleep_us(3)
        _rtrig.high(); time.sleep_us(10); _rtrig.low()
        us = time_pulse_us(_recho, 1, 30000)
        _rear_val = (us / 58.0) if us > 0 else 999.0
    except Exception:
        _rear_val = 999.0
    return _rear_val


def line_lr():
    if not HW["line"]:
        return (1.0, 1.0)     # "no line seen" for white-line logic
    try:
        return (reflectance.get_left(), reflectance.get_right())
    except Exception:
        return (1.0, 1.0)


def on_line(v):
    return v < CFG["thresh"] if CFG["line_white"] else v > CFG["thresh"]


def led(on):
    if HW["led"]:
        try:
            board.led_on() if on else board.led_off()
        except Exception:
            pass


def arm_tap():
    if HW["servo"]:
        try:
            servo_one.set_angle(CFG["arm_up"]); time.sleep_ms(110)
            servo_one.set_angle(CFG["arm_down"])
        except Exception:
            pass
    emit("ARM", "tap")


_enc_last = 0.0
def enc_cm():
    """Average wheel travel in cm since boot (signed). Caches the last good
    read so a transient encoder error can't poison a segment's start distance
    (a spurious 0.0 was making short PATH steps 'complete' instantly)."""
    global _enc_last
    if not HW["enc"]:
        return _enc_last
    try:
        _enc_last = (left_motor.get_position() + right_motor.get_position()) \
            * 0.5 * CFG["wheel_circ"]
    except Exception:
        pass
    return _enc_last


# ---------------------------------------------------------------- drive
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

LAST = {"err": 0.0, "sign": 1, "mark_t": 0, "head": 0.0}
OBST = {"t": 0}               # obstacle debounce: first time front went sub-threshold
MANU = {"fwd": 0.0, "turn": 0.0}
TURN = {"on": False, "target": 0.0, "dirn": 1, "end": 0}
SEG = {"start_cm": 0.0}


# ---------------------------------------------------------------- helpers
def follow_line(base):
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


def hold_heading(base):
    """Drive straight, correcting drift with the IMU when available."""
    if HW["imu"]:
        err = LAST["head"] - _heading()
        while err > 180:
            err -= 360
        while err < -180:
            err += 360
        SD.arcade(base, clamp(err * 0.02, -0.3, 0.3))
    else:
        SD.tank(base, base)


def marker_seen():
    l, r = line_lr()
    if on_line(l) and on_line(r):
        if since(LAST["mark_t"]) > CFG["mark_gap_ms"]:
            LAST["mark_t"] = now()
            return True
    return False


def start_turn(dirn, deg=90):
    """Begin a pivot: gyro-target if IMU, else timed."""
    TURN["on"] = True; TURN["dirn"] = dirn
    if HW["imu"]:
        TURN["target"] = _heading() + dirn * deg
        TURN["end"] = time.ticks_add(now(), 2500)     # safety timeout
    else:
        TURN["target"] = None
        TURN["end"] = time.ticks_add(now(), int(CFG["turn_ms_90"] * deg / 90))
    emit("TURN", "right" if dirn > 0 else "left")


def step_turn():
    """Returns True while still turning."""
    if not TURN["on"]:
        return False
    e = CFG["turn_eff"] * TURN["dirn"]
    SD.tank(e, -e)
    done = False
    if TURN["target"] is not None:
        err = TURN["target"] - _heading()
        while err > 180:
            err -= 360
        while err < -180:
            err += 360
        done = abs(err) < CFG["turn_tol"] or since(TURN["end"]) >= 0
    else:
        done = since(TURN["end"]) >= 0
    if done:
        TURN["on"] = False; SD.tank(0, 0)
        if HW["imu"]:
            LAST["head"] = _heading()
        SEG["start_cm"] = enc_cm()
    return not done


# ---------------------------------------------------------------- missions
def set_role(r):
    global ROLE
    if r in (OPENER, SAM, CLOSER):
        ROLE = r; emit("ROLE", r)


def set_route(s):
    global ROUTE, NAV
    ROUTE = [c for c in s.upper() if c in "SLRA"]
    NAV = "LINE"; emit("ROUTE", "".join(ROUTE))


def set_path(s):
    global PATH, NAV
    PATH = []
    for tok in s.upper().replace(";", ",").split(","):
        tok = tok.strip()
        if not tok:
            continue
        if tok == "A":
            PATH.append(("A", 0))
        elif tok[0] in "DLR":
            try:
                PATH.append((tok[0], float(tok[1:])))
            except Exception:
                pass
    NAV = "PATH"; emit("PATH", str(len(PATH)))


def start_mission():
    global MISSION, STATE, PROG
    MISSION = "RUN"; STATE = "RUN"; PROG = 0
    LAST["err"] = 0.0; LAST["mark_t"] = now()
    if HW["imu"]:
        LAST["head"] = _heading()
    SEG["start_cm"] = enc_cm()
    OBST["t"] = 0
    TURN["on"] = False; led(False); emit("GO", ROLE)


def stop_mission():
    global MISSION, STATE
    MISSION = "IDLE"; STATE = "STOP"; TURN["on"] = False
    SD.brake(); emit("STOP", ROLE)


def arrive():
    global MISSION, STATE
    MISSION = "ARRIVED"; STATE = "ARRIVED"
    SD.brake(); led(True); arm_tap(); emit("ARRIVE", "")


# ---------- OPENER
def step_opener():
    global STATE, PROG
    # obstacle must PERSIST (>300ms) to trigger — the ultrasonic flickers low
    if front_cm() < CFG["stop_cm"]:
        if OBST["t"] == 0:
            OBST["t"] = now()
        elif since(OBST["t"]) > 300:
            STATE = "ALERT"; led(True); SD.brake(); emit("ALERT", "obstacle")
            return
    else:
        OBST["t"] = 0
    if HW["rear"] and CFG["sam_lost_cm"] < rear_cm() < 500:  # 999 = no echo -> ignore
        STATE = "WAIT_SAM"; SD.brake(); emit("LOST", "sam")
        return
    led(False)
    if step_turn():
        STATE = "TURN"; return
    if NAV == "LINE":
        if marker_seen():
            PROG += 1; emit("MARK", str(PROG))
            act = ROUTE[PROG - 1] if PROG - 1 < len(ROUTE) else "A"
            if act == "A":
                arrive(); return
            if act == "L":
                start_turn(-1); return
            if act == "R":
                start_turn(1); return
        STATE = "FOLLOW"; follow_line(CFG["spd"])
    else:                                   # PATH mode (no line)
        if PROG >= len(PATH):
            arrive(); return
        kind, val = PATH[PROG]
        if kind == "A":
            arrive(); return
        if kind in "LR":
            PROG += 1; emit("STEP", str(PROG))
            start_turn(1 if kind == "R" else -1, val); return
        done = enc_cm() - SEG["start_cm"]
        if done >= val:
            PROG += 1; emit("STEP", str(PROG))
            SEG["start_cm"] = enc_cm(); return
        STATE = "DRIVE"; hold_heading(CFG["spd"])


# ---------- SAM (blind: only hub commands)
def step_sam():
    global STATE
    if step_turn():
        STATE = "TURN"; return
    STATE = "DRIVE"; hold_heading(CFG["spd"])


# ---------- CLOSER (gap to Sam ahead + steer by line else heading)
def step_closer():
    global STATE
    if step_turn():
        STATE = "TURN"; return
    d = front_cm()
    gap, band = CFG["gap"], CFG["gap_band"]
    base = CFG["spd"]
    if d < gap - band:
        STATE = "HOLD"; SD.tank(0, 0); return
    if d < gap + band:
        base *= 0.55; STATE = "KEEP"
    else:
        STATE = "FOLLOW"
    l, r = line_lr()
    if HW["line"] and (on_line(l) or on_line(r)):
        follow_line(base)
    else:
        hold_heading(base)


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
    b = CFG["spd"]
    MANU["fwd"], MANU["turn"] = {
        "w": (b, 0.0), "s": (-b, 0.0), "a": (0.0, -b * 0.8),
        "d": (0.0, b * 0.8), "q": (b, -b * 0.35), "e": (b, b * 0.35),
    }.get(k, (0.0, 0.0))


_seen_ids = []                       # recent command ids, for at-most-once execution
def handle(line):
    u = line.strip()
    if not u:
        return
    # Reliable command envelope: "#<id> <cmd>" -> acknowledge receipt IMMEDIATELY
    # (before executing) so the hub can confirm all robots got it, then run <cmd>.
    # Commands without a "#<id>" prefix still work (backward compatible).
    if u[0] == "#":
        sp = u.find(" ")
        cid = u[1:sp] if sp > 0 else u[1:]
        print("A|%s" % cid)          # ACK on receipt (ALWAYS, so the hub stops retrying)
        if cid in _seen_ids:         # duplicate (retry whose original already landed) -> ACK but DON'T re-run
            return
        _seen_ids.append(cid)
        if len(_seen_ids) > 24:
            _seen_ids.pop(0)
        if sp < 0:
            return
        u = u[sp + 1:].strip()
        if not u:
            return
    up = u.upper()
    if up == "STOP" or u == " ":
        stop_mission()
    elif up == "GO":
        start_mission()
    elif up == "TAP":
        arm_tap()
    elif up == "TURNL":
        start_turn(-1)
    elif up == "TURNR":
        start_turn(1)
    elif up.startswith("ROLE "):
        set_role(up[5:].strip())
    elif up in (OPENER, SAM, CLOSER):
        set_role(up)
    elif up.startswith("ROUTE "):
        set_route(up[6:])
    elif up.startswith("PATH "):
        set_path(up[5:])
    elif up.startswith("SPD "):
        try:
            CFG["spd"] = clamp(float(up[4:]), 0.0, 0.9)
        except Exception:
            pass
    elif up.startswith("GAP "):
        try:
            CFG["gap"] = float(up[4:])
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
    print("T|%s|%s|%.1f|%.1f|%.2f|%.2f|%d|%s" % (
        ROLE, MISSION, front_cm(), rear_cm(), l, r, PROG, STATE))


# ---------------------------------------------------------------- main
def run():
    probe()
    hw = ",".join(k for k, v in HW.items() if v) or "none"
    emit("READY", hw)
    while True:
        line = read_line()
        if line is not None:
            handle(line)
        if MISSION == "MANUAL":
            step_manual()
        elif MISSION == "RUN":
            if ROLE == OPENER:
                step_opener()
            elif ROLE == SAM:
                step_sam()
            else:
                step_closer()
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
