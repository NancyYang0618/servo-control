"""
XIANG full robot (4 legs, 8 LX-16A servos): shared core for the boot test, homing and the panel.

Talks to the servos with the raw LX-16A serial protocol instead of PyLX-16A, because PyLX-16A
switches every motor ON the moment it first talks to it. Here a motor only switches on when a
move is asked for, and every first move starts from where the joint already is, so nothing jumps.

Joint names: FL / FR / RL / RR (front-left, front-right, rear-left, rear-right) + _hip / _knee.
Settings (IDs, home angles, safe swing) live in robot_config.json.
"""
import glob
import json
import os
import threading
import time
from pathlib import Path

import serial

try:                                    # a vanished USB port raises termios.error on Linux, not OSError
    import termios
    PORT_ERRORS = (serial.SerialException, OSError, termios.error)
except ImportError:
    PORT_ERRORS = (serial.SerialException, OSError)

HERE = Path(__file__).resolve().parent
CONFIG_FILE = HERE / "robot_config.json"
LOG_DIR = HERE / "logs"

LEGS = ("FL", "FR", "RL", "RR")
LEG_NAMES = {"FL": "front left", "FR": "front right", "RL": "rear left", "RR": "rear right"}
JOINT_ORDER = [f"{leg}_{j}" for leg in LEGS for j in ("hip", "knee")]

# LX-16A command numbers (Hiwonder bus servo protocol)
MOVE_TIME_WRITE, ID_WRITE, TEMP_READ, VIN_READ, POS_READ = 1, 13, 26, 27, 28
LOAD_WRITE, LOAD_READ, LED_WRITE = 31, 32, 33
UNITS_PER_DEG = 1000 / 240          # servo counts 0..1000 = 0..240 degrees


def pi_power_low():
    """True while the Raspberry Pi itself reports under-voltage right now (its own 5 V supply sagging).
    None when it can't tell (not a Pi). Reads the firmware's flags; takes about 10 ms."""
    try:
        import subprocess
        out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True, text=True, timeout=1).stdout
        return bool(int(out.strip().split("=")[1], 16) & 0x1)
    except Exception:
        return None


class BusError(Exception):
    """A problem talking to the servos, with a plain-language hint."""

    def __init__(self, msg, hint="", joint=None):
        super().__init__(msg)
        self.hint, self.joint = hint, joint


def pretty(joint):
    leg, part = joint.split("_")
    return f"{LEG_NAMES[leg]} {part}"


FREE_RANGE = (1.0, 239.0)       # limit-test mode: only the servo's own 0..240 range applies


def load_config():
    """Safe range per joint: a MEASURED limit (limit_min / limit_max, found in limit-test mode) minus
    limit_margin_deg, or by default home +/- swing_deg."""
    cfg = json.loads(CONFIG_FILE.read_text())
    lo_all, hi_all = cfg["servo_limits_deg"]
    margin = cfg.get("limit_margin_deg", 3.0)
    for name, s in cfg["servos"].items():
        swing = cfg["swing_deg"]["hip" if name.endswith("hip") else "knee"]
        s["forced"] = bool(s.get("force_unlock")) and not 0.0 <= s["home"] <= 240.0
        s["holdable"] = 0.0 <= s["home"] <= 240.0 or s["forced"]
        s["min_measured"] = "limit_min" in s
        s["max_measured"] = "limit_max" in s
        s["min"] = (max(FREE_RANGE[0], s["limit_min"] + margin) if s["min_measured"]
                    else max(lo_all, s["home"] - swing))
        s["max"] = (min(FREE_RANGE[1], s["limit_max"] - margin) if s["max_measured"]
                    else min(hi_all, s["home"] + swing))
        if s["min"] >= s["max"]:     # a bad measured limit must never leave an empty range: ignore it
            s["min_measured"] = s["max_measured"] = False
            s["min"] = max(lo_all, s["home"] - swing)
            s["max"] = min(hi_all, s["home"] + swing)
        if s["forced"]:              # home unreachable: give the whole reachable range until limits are marked
            if not s["min_measured"]:
                s["min"] = lo_all
            if not s["max_measured"]:
                s["max"] = hi_all
    return cfg


def save_limit(joint, side, angle):
    """side 'min' or 'max'; angle = servo degrees where the joint stops. None clears it."""
    cfg = json.loads(CONFIG_FILE.read_text())
    key = f"limit_{side}"
    if angle is None:
        cfg["servos"][joint].pop(key, None)
    else:
        cfg["servos"][joint][key] = round(float(angle), 1)
    CONFIG_FILE.write_text(json.dumps(cfg, indent=2) + "\n")


def save_homes(homes):
    """homes: {joint: angle}. Rewrites robot_config.json keeping everything else."""
    cfg = json.loads(CONFIG_FILE.read_text())
    for j, a in homes.items():
        s = cfg["servos"][j]
        if abs(float(a) - s["home"]) > 5.0:        # joint was re-mounted: its old measured limits no longer apply
            s.pop("limit_min", None)
            s.pop("limit_max", None)
        s["home"] = round(float(a), 1)
        if 0.0 <= s["home"] <= 240.0:              # reachable again: no force-unlock needed
            s.pop("force_unlock", None)
    cfg["note"] = cfg["note"].split(" (saved")[0] + f" (saved {time.strftime('%Y-%m-%d %H:%M')})"
    CONFIG_FILE.write_text(json.dumps(cfg, indent=2) + "\n")


def find_port():
    if os.environ.get("XIANG_PORT"):
        return os.environ["XIANG_PORT"]
    for pat in ("/dev/ttyUSB*", "/dev/cu.usbserial*", "/dev/cu.wchusbserial*"):
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[0]
    return None


class Bus:
    """Raw LX-16A bus. Thread safe. Never switches a motor on except through move()."""

    def __init__(self, port=None, timeout=0.05):
        self.port = port or find_port()
        if not self.port:
            raise BusError("BusLinker not found on USB.",
                           "Plug the BusLinker into a Pi USB port with a data cable. It shows up as /dev/ttyUSB0.")
        try:
            self.ser = serial.Serial(self.port, 115200, timeout=timeout)
        except PORT_ERRORS as e:
            raise BusError(f"Cannot open {self.port}: {e}",
                           "Another program may be using it (the panel service?). Unplug the BusLinker USB, "
                           "wait 3 s, plug it back in.")
        self.lock = threading.RLock()

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    @staticmethod
    def _checksum(body):
        return (~sum(body)) & 0xFF

    def _send(self, sid, cmd, params=()):
        body = [sid, len(params) + 3, cmd, *params]
        try:
            self.ser.write(bytes([0x55, 0x55, *body, self._checksum(body)]))
        except PORT_ERRORS as e:
            raise BusError(f"Lost the USB link to the BusLinker ({e}).",
                           "Unplug the BusLinker USB, wait 3 s, plug it back in, then Reconnect.")

    def _ask(self, sid, cmd, nbytes, tries=3):
        """Returns the reply's data bytes, or raises BusError (no answer / garbled)."""
        garbled = False
        with self.lock:
            for _ in range(tries):
                try:
                    self.ser.reset_input_buffer()
                except PORT_ERRORS as e:
                    raise BusError(f"Lost the USB link to the BusLinker ({e}).",
                                   "Unplug the BusLinker USB, wait 3 s, plug it back in, then Reconnect.")
                self._send(sid, cmd)
                try:
                    raw = self.ser.read(nbytes + 6)
                except PORT_ERRORS as e:
                    raise BusError(f"Lost the USB link to the BusLinker ({e}).",
                                   "Unplug the BusLinker USB, wait 3 s, plug it back in, then Reconnect.")
                if (len(raw) == nbytes + 6 and raw[:2] == b"\x55\x55" and raw[2] == sid and raw[4] == cmd
                        and self._checksum(list(raw[2:-1])) == raw[-1]):
                    return list(raw[5:-1])
                garbled = garbled or bool(raw)
                time.sleep(0.01)
        if garbled:
            raise BusError(f"Servo ID {sid} gave a garbled reply.",
                           "A loose cable, or two servos sharing this ID. Check the plugs along the chain.")
        raise BusError(f"Servo ID {sid} is not answering.",
                       "Check its cable in the daisy chain, and that the converter shows about 7 V.")

    # ---- reads
    def angle(self, sid):
        lo, hi = self._ask(sid, POS_READ, 2)
        v = lo | (hi << 8)
        if v > 32767:
            v -= 65536
        return v / UNITS_PER_DEG

    def volts(self, sid):
        lo, hi = self._ask(sid, VIN_READ, 2)
        return (lo | (hi << 8)) / 1000.0

    def temp(self, sid):
        return self._ask(sid, TEMP_READ, 1)[0]

    def motor_on(self, sid):
        return bool(self._ask(sid, LOAD_READ, 1)[0])

    def read_all(self, sid):
        return dict(angle=self.angle(sid), volts=self.volts(sid), temp=self.temp(sid), motor=self.motor_on(sid))

    # ---- writes
    def move(self, sid, angle, ms):
        """Move to angle (0..240) over ms. This switches the motor on."""
        if not 0.0 <= angle <= 240.0:
            raise BusError(f"Servo ID {sid}: {angle:.1f} deg is outside the servo's 0 to 240 range.")
        units = int(round(angle * UNITS_PER_DEG))
        ms = int(max(0, min(30000, ms)))
        with self.lock:
            self._send(sid, MOVE_TIME_WRITE, [units & 0xFF, units >> 8, ms & 0xFF, ms >> 8])

    def motor_off(self, sid):
        with self.lock:
            self._send(sid, LOAD_WRITE, [0])

    def led(self, sid, on):
        with self.lock:
            self._send(sid, LED_WRITE, [0 if on else 1])        # LX-16A: 0 = LED on


class Robot:
    def __init__(self, bus=None, log=print):
        self.cfg = load_config()
        self.bus = bus or Bus()
        self.log = log
        self.stop = threading.Event()
        self.free = False           # limit-test mode: ignore the safe range, keep only 0..240

    @property
    def servos(self):
        return self.cfg["servos"]

    def sid(self, joint):
        return self.servos[joint]["id"]

    def reload(self):
        self.cfg = load_config()

    def limits(self, joint):
        s = self.servos[joint]
        return FREE_RANGE if self.free else (s["min"], s["max"])

    def clamp(self, joint, angle):
        lo, hi = self.limits(joint)
        return max(lo, min(hi, angle))

    def all_off(self):
        """Emergency: every motor limp. Sent twice to every ID, errors ignored."""
        self.stop.set()
        for _ in range(2):
            for j in JOINT_ORDER:
                try:
                    self.bus.motor_off(self.sid(j))
                except BusError:
                    pass

    def blink(self, joint, seconds=3.0):
        sid = self.sid(joint)
        end, on = time.time() + seconds, True
        while time.time() < end:
            self.bus.led(sid, on)
            on = not on
            time.sleep(0.25)
        self.bus.led(sid, False)

    # ---------------------------------------------------------------- boot test
    def boot_test(self, motion=False, wiggle_deg=4.0):
        """Power-on self test. Returns (all_ok, rows). Stage 1 never moves anything.
        Stage 2 (motion=True) moves ONE servo at a time by wiggle_deg and back."""
        c = self.cfg
        rows, ok = [], True
        self.log("Boot test, stage 1: talking to every servo (nothing moves)")
        for j in JOINT_ORDER:
            s = self.servos[j]
            row = dict(joint=j, id=s["id"], home=s["home"], result="PASS", notes=[])
            try:
                r = self.bus.read_all(s["id"])
                row.update(r)
            except BusError as e:
                row.update(result="FAIL", notes=[f"{e} {e.hint}"])
                rows.append(row)
                ok = False
                continue
            v, t, a = r["volts"], r["temp"], r["angle"]
            if v < c["volts"]["low"] or v > c["volts"]["high"]:
                row["result"] = "FAIL"
                row["notes"].append(f"supply {v:.2f} V, needs {c['volts']['low']} to {c['volts']['high']} V")
            if t >= c["temp_c"]["warn"]:
                row["result"] = "FAIL"
                row["notes"].append(f"{t} C, too hot")
            if s["forced"]:
                row["notes"].append(f"force-unlocked: straight (servo {s['home']:.1f}) is out of the servo's reach; "
                                    "it works from about 46 deg bent onward")
            elif not 0.0 <= a <= 240.0:
                row["result"] = "FAIL"
                row["notes"].append(f"reading {a:.1f} deg is outside 0 to 240: the servo cannot hold this joint")
            if not s["holdable"]:
                row["result"] = "FAIL"
                row["notes"].append(f"home {s['home']:.1f} deg is outside 0 to 240: re-mount this joint's discs, "
                                    "then save home again (11_home.py --save-current)")
            off = a - s["home"]
            row["from_home"] = off
            if abs(off) > c["pose_ok_deg"]:
                row["notes"].append(f"{abs(off):.1f} deg away from home (homing will move it)")
            ok &= row["result"] == "PASS"
            rows.append(row)
        if motion:
            self.log("Boot test, stage 2: each servo wiggles a few degrees, one at a time")
            for row in rows:
                if self.stop.is_set():
                    break
                j = row["joint"]
                if row["result"] != "PASS":
                    row["notes"].append("motion test skipped")
                    continue
                row["result"] = self._wiggle(j, row, wiggle_deg)
                ok &= row["result"] == "PASS"
        self.write_log("boot_test", rows)
        return ok, rows

    def _wiggle(self, joint, row, deg):
        sid, s = self.sid(joint), self.servos[joint]
        try:
            start = self.bus.angle(sid)
            if not 0.0 <= start <= 240.0:
                row["notes"].append(f"wiggle skipped: reading {start:.1f} deg is past the servo's 240 deg end")
                return "PASS"
            was_on = self.bus.motor_on(sid)
            self.bus.move(sid, start, 0)                     # hold exactly where it is: no jump
            time.sleep(0.15)
            d = deg if start + deg <= s["max"] else -deg      # wiggle toward the side with room
            errs = []
            for target in (start + d, start):
                self.bus.move(sid, target, 400)
                time.sleep(0.7)
                errs.append(abs(self.bus.angle(sid) - target))
            if not was_on:
                self.bus.motor_off(sid)
            row["wiggle_error"] = max(errs)
            if max(errs) > 2.5:
                row["notes"].append(f"moved but missed by {max(errs):.1f} deg: check for rubbing or a loose horn")
                return "FAIL"
            row["notes"].append(f"wiggle {d:+.0f} deg OK (error {max(errs):.1f} deg)")
            return "PASS"
        except BusError as e:
            row["notes"].append(f"{e} {e.hint}")
            return "FAIL"

    # ---------------------------------------------------------------- homing
    def home(self, joints=None, speed=None, progress=None):
        """Bring joints to home, one LEG at a time (hip and knee together), slowly."""
        joints = joints or JOINT_ORDER
        ok, rows = self.go_to({j: self.servos[j]["home"] for j in joints}, speed=speed, by_leg=True,
                              progress=progress, label="Homing")
        self.write_log("homing", rows)
        return ok, rows

    def pose_targets(self, pose):
        """pose = {joint: degrees from home} -> {joint: servo angle}, only joints that can be held."""
        return {j: self.servos[j]["home"] + float(d) for j, d in pose.items()
                if j in self.servos and self.servos[j]["holdable"]}

    def go_to(self, targets, speed=None, by_leg=True, progress=None, label="Moving"):
        """Glide joints to servo angles {joint: angle} at `speed` deg/s, all arriving together.
        by_leg=True moves one leg at a time (safest); False moves every leg at once.
        Each servo first holds where it is, so switching it on never makes it jump.
        Targets are limited to each joint's safe range. Returns (all_ok, rows)."""
        speed = float(speed or self.cfg["home_speed_deg_s"])
        rows, ok = [], True
        self.stop.clear()
        groups = [[j for j in JOINT_ORDER if j.startswith(leg) and j in targets] for leg in LEGS] if by_leg \
            else [[j for j in JOINT_ORDER if j in targets]]
        for group in groups:
            plan = []
            for j in group:
                s, sid = self.servos[j], self.sid(j)
                want = float(targets[j])
                goal = self.clamp(j, want)
                row = dict(joint=j, id=sid, home=s["home"], target=goal, result="PASS", notes=[])
                rows.append(row)
                if not s["holdable"]:
                    row.update(result="SKIPPED")
                    row["notes"].append(f"home {s['home']:.1f} deg is outside the servo's 0 to 240 range; "
                                        "motor left OFF. Re-mount this joint, then save home again.")
                    ok = False
                    continue
                if abs(goal - want) > 0.05:
                    row["notes"].append(f"target limited to the safe range ({goal - s['home']:+.1f} deg "
                                        f"from home instead of {want - s['home']:+.1f})")
                try:
                    a = self.bus.angle(sid)
                    v = self.bus.volts(sid)
                except BusError as e:
                    row.update(result="FAIL", notes=[f"{e} {e.hint}"])
                    ok = False
                    continue
                row["start"] = a
                if not 0.0 <= a <= 240.0 and not s["forced"]:
                    row.update(result="SKIPPED")
                    row["notes"].append(f"reading {a:.1f} deg is outside 0 to 240, so switching it on would "
                                        "make it jump. Motor left OFF; move the joint by hand into range.")
                    ok = False
                    continue
                if v < self.cfg["volts"]["refuse"] or v > self.cfg["volts"]["block"]:
                    row.update(result="FAIL", notes=[f"supply {v:.2f} V: not moving"])
                    ok = False
                    continue
                plan.append((j, sid, a, goal, row))
            if not plan:
                continue
            if progress:
                legs = sorted({j[:2] for j, *_ in plan}, key=LEGS.index)
                progress(f"{label}: {', '.join(LEG_NAMES[l] for l in legs)}")
            for j, sid, a, g, row in plan:            # 1) hold where it is (no jump)
                if 0.0 <= a <= 240.0:
                    self.bus.move(sid, a, 0)
                else:
                    row["notes"].append("started past the servo's 240 deg end, so it swung into range")
            time.sleep(0.15)
            t_ms = max(300, max(abs(g - a) for _, _, a, g, _ in plan) / speed * 1000)
            for j, sid, a, g, row in plan:            # 2) glide, all arriving together
                self.bus.move(sid, g, t_ms)
            end = time.time() + t_ms / 1000 + 0.5
            while time.time() < end:
                if self.stop.is_set():
                    for j, sid, a, g, row in plan:
                        try:
                            self.bus.move(sid, self.bus.angle(sid), 0)      # freeze where it is
                        except BusError:
                            pass
                        row.update(result="STOPPED")
                    return False, rows
                time.sleep(0.05)
            for j, sid, a, g, row in plan:            # 3) check it arrived
                try:
                    end_a = self.bus.angle(sid)
                except BusError as e:
                    row.update(result="FAIL", notes=[f"{e} {e.hint}"])
                    ok = False
                    continue
                row["end"] = end_a
                err = abs(end_a - g)
                if err > 3.0:
                    row["result"] = "FAIL"
                    row["notes"].append(f"stopped {err:.1f} deg short of its target: something may be blocking it")
                    ok = False
                else:
                    h = row["home"]
                    row["notes"].append(f"{a - h:+.1f} -> {end_a - h:+.1f} deg from home (target {g - h:+.1f})")
        return ok, rows

    # ---------------------------------------------------------------- walking
    def play(self, frames, dt, progress=None, settle_speed=20.0):
        """Play a list of {joint: servo angle} frames, one every dt seconds.
        Starts with a slow move (one leg at a time) into the first frame. Every frame it reads one servo
        back (taking turns), so an unplugged motor or a stalled joint is noticed within about a second.
        Stop (self.stop) makes every joint hold where it is. Returns a summary dict."""
        self.stop.clear()
        ok, rows = self.go_to(frames[0], speed=settle_speed, by_leg=True, progress=progress, label="Getting ready")
        if self.stop.is_set():
            return dict(ok=False, why="stopped while getting ready", rows=rows)
        if not ok:
            return dict(ok=False, why="could not reach the start position", rows=rows)
        joints = list(frames[0])
        ms = int(dt * 1000) + 30                  # a little overlap so motion stays smooth
        worst = {j: 0.0 for j in joints}
        misses, vmin, t_next, low_count = 0, 99.0, time.time(), 0
        check_every = max(1, int(0.5 / dt))      # look at the Pi's own power twice a second
        for i, fr in enumerate(frames):
            if self.stop.is_set():
                self._freeze(joints)
                return dict(ok=False, why="stopped", frame=i, frames=len(frames), worst=worst, vmin=vmin)
            if i % check_every == 0 and pi_power_low():
                low_count += 1
                if low_count >= 2:               # sagging for a second: stop before the Pi resets
                    self._freeze(joints)
                    return dict(ok=False, why="the Pi's own power sagged (the motors drew too much current from "
                                "the shared battery), so the walk stopped before the Pi could reset", frame=i,
                                frames=len(frames), worst=worst, vmin=vmin, power=True)
            elif i % check_every == 0:
                low_count = 0
            for j, a in fr.items():
                self.bus.move(self.sid(j), self.clamp(j, a), ms)
            j = joints[i % len(joints)]           # heartbeat: read one servo back per frame
            try:
                ang = self.bus.angle(self.sid(j))
                if i > 3:
                    prev = frames[i - 1][j]
                    worst[j] = max(worst[j], abs(ang - prev))
                if i % 10 == 0:
                    vmin = min(vmin, self.bus.volts(self.sid(j)))
                misses = 0
            except BusError:
                misses += 1
                if misses >= 3:
                    self._freeze(joints)
                    return dict(ok=False, why=f"{pretty(j)} stopped answering during the walk", frame=i,
                                frames=len(frames), worst=worst, vmin=vmin)
            if progress and i % 20 == 0:
                progress(f"Walking: {100 * i // len(frames)}%")
            t_next += dt
            time.sleep(max(0.0, t_next - time.time()))
        return dict(ok=True, frames=len(frames), worst=worst, vmin=vmin)

    def _freeze(self, joints):
        for j in joints:
            try:
                self.bus.move(self.sid(j), self.bus.angle(self.sid(j)), 0)
            except BusError:
                pass

    # ---------------------------------------------------------------- helpers
    def write_log(self, kind, rows):
        LOG_DIR.mkdir(exist_ok=True)
        f = LOG_DIR / f"{kind}_{time.strftime('%Y%m%d_%H%M%S')}.json"
        f.write_text(json.dumps(dict(time=time.strftime("%Y-%m-%d %H:%M:%S"), rows=rows), indent=1, default=str))
        return f


def print_rows(rows, title):
    print(f"\n{title}")
    print(f"  {'joint':<18}{'ID':>3}  {'from home':>9}  {'servo':>7}  {'home':>7}  {'volts':>6}  {'temp':>5}  result   notes")
    for r in rows:
        a = r.get("end", r.get("angle"))
        num = isinstance(a, (int, float))
        rel = f"{a - r['home']:+8.1f}°" if num else "        -"
        ang = f"{rel}  {a:7.1f}" if num else f"{rel}        -"
        v = f"{r['volts']:6.2f}" if isinstance(r.get("volts"), float) else "     -"
        t = f"{r['temp']:4d}C" if isinstance(r.get("temp"), int) else "    -"
        print(f"  {pretty(r['joint']):<18}{r['id']:>3}  {ang}  {r['home']:7.1f}  {v}  {t}  "
              f"{r['result']:<8} {'; '.join(r['notes'])}")
