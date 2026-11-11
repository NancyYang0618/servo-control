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


class BusError(Exception):
    """A problem talking to the servos, with a plain-language hint."""

    def __init__(self, msg, hint="", joint=None):
        super().__init__(msg)
        self.hint, self.joint = hint, joint


def pretty(joint):
    leg, part = joint.split("_")
    return f"{LEG_NAMES[leg]} {part}"


def load_config():
    cfg = json.loads(CONFIG_FILE.read_text())
    lo_all, hi_all = cfg["servo_limits_deg"]
    for name, s in cfg["servos"].items():
        swing = cfg["swing_deg"]["hip" if name.endswith("hip") else "knee"]
        s["holdable"] = 0.0 <= s["home"] <= 240.0
        s["min"] = max(lo_all, s["home"] - swing)
        s["max"] = min(hi_all, s["home"] + swing)
    return cfg


def save_homes(homes):
    """homes: {joint: angle}. Rewrites robot_config.json keeping everything else."""
    cfg = json.loads(CONFIG_FILE.read_text())
    for j, a in homes.items():
        cfg["servos"][j]["home"] = round(float(a), 1)
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
        except (serial.SerialException, OSError) as e:
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
        except (serial.SerialException, OSError) as e:
            raise BusError(f"Lost the USB link to the BusLinker ({e}).",
                           "Unplug the BusLinker USB, wait 3 s, plug it back in, then Reconnect.")

    def _ask(self, sid, cmd, nbytes, tries=3):
        """Returns the reply's data bytes, or raises BusError (no answer / garbled)."""
        garbled = False
        with self.lock:
            for _ in range(tries):
                try:
                    self.ser.reset_input_buffer()
                except (serial.SerialException, OSError) as e:
                    raise BusError(f"Lost the USB link to the BusLinker ({e}).",
                                   "Unplug the BusLinker USB, wait 3 s, plug it back in, then Reconnect.")
                self._send(sid, cmd)
                try:
                    raw = self.ser.read(nbytes + 6)
                except (serial.SerialException, OSError) as e:
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

    @property
    def servos(self):
        return self.cfg["servos"]

    def sid(self, joint):
        return self.servos[joint]["id"]

    def reload(self):
        self.cfg = load_config()

    def clamp(self, joint, angle):
        s = self.servos[joint]
        return max(s["min"], min(s["max"], angle))

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
            if not 0.0 <= a <= 240.0:
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
        """Bring joints to home, one LEG at a time (hip and knee together), slowly.
        Each servo first holds where it is, so switching it on never makes it jump.
        Returns (all_ok, rows)."""
        speed = speed or self.cfg["home_speed_deg_s"]
        joints = joints or JOINT_ORDER
        rows, ok = [], True
        self.stop.clear()
        for leg in LEGS:
            todo = [j for j in joints if j.startswith(leg)]
            if not todo:
                continue
            plan = []
            for j in todo:
                s, sid = self.servos[j], self.sid(j)
                row = dict(joint=j, id=sid, home=s["home"], result="PASS", notes=[])
                rows.append(row)
                if not s["holdable"]:
                    row.update(result="SKIPPED")
                    row["notes"].append(f"home {s['home']:.1f} deg is outside the servo's 0 to 240 range; "
                                        "motor left OFF. Re-mount this joint, then save home again.")
                    ok = False
                    continue
                try:
                    a = self.bus.angle(sid)
                    v = self.bus.volts(sid)
                except BusError as e:
                    row.update(result="FAIL", notes=[f"{e} {e.hint}"])
                    ok = False
                    continue
                row["start"] = a
                if not 0.0 <= a <= 240.0:
                    row.update(result="SKIPPED")
                    row["notes"].append(f"reading {a:.1f} deg is outside 0 to 240, so switching it on would "
                                        "make it jump. Motor left OFF; move the joint by hand into range.")
                    ok = False
                    continue
                if v < self.cfg["volts"]["refuse"] or v > self.cfg["volts"]["block"]:
                    row.update(result="FAIL", notes=[f"supply {v:.2f} V: not moving"])
                    ok = False
                    continue
                plan.append((j, sid, a, s["home"], row))
            if not plan:
                continue
            if progress:
                progress(f"Homing {LEG_NAMES[leg]} leg")
            # 1) hold where it is (no jump)  2) glide to home at `speed`
            t_ms = 0
            for j, sid, a, h, row in plan:
                self.bus.move(sid, a, 0)
            time.sleep(0.15)
            for j, sid, a, h, row in plan:
                t_ms = max(t_ms, abs(h - a) / speed * 1000)
            t_ms = max(300, t_ms)
            for j, sid, a, h, row in plan:
                self.bus.move(sid, h, t_ms)
            end = time.time() + t_ms / 1000 + 0.5
            while time.time() < end:
                if self.stop.is_set():
                    for j, sid, a, h, row in plan:
                        try:
                            self.bus.move(sid, self.bus.angle(sid), 0)      # freeze where it is
                        except BusError:
                            pass
                        row.update(result="STOPPED")
                    return False, rows
                time.sleep(0.05)
            for j, sid, a, h, row in plan:
                try:
                    end_a = self.bus.angle(sid)
                except BusError as e:
                    row.update(result="FAIL", notes=[f"{e} {e.hint}"])
                    ok = False
                    continue
                row["end"] = end_a
                err = abs(end_a - h)
                if err > 3.0:
                    row["result"] = "FAIL"
                    row["notes"].append(f"stopped {err:.1f} deg short of home: something may be blocking it")
                    ok = False
                else:
                    row["notes"].append(f"{a:.1f} -> {end_a:.1f} deg (home {h:.1f})")
        self.write_log("homing", rows)
        return ok, rows

    # ---------------------------------------------------------------- helpers
    def write_log(self, kind, rows):
        LOG_DIR.mkdir(exist_ok=True)
        f = LOG_DIR / f"{kind}_{time.strftime('%Y%m%d_%H%M%S')}.json"
        f.write_text(json.dumps(dict(time=time.strftime("%Y-%m-%d %H:%M:%S"), rows=rows), indent=1, default=str))
        return f


def print_rows(rows, title):
    print(f"\n{title}")
    print(f"  {'joint':<18}{'ID':>3}  {'angle':>7}  {'home':>7}  {'volts':>6}  {'temp':>5}  result   notes")
    for r in rows:
        ang = r.get("end", r.get("angle"))
        ang = f"{ang:7.1f}" if isinstance(ang, (int, float)) else "      -"
        v = f"{r['volts']:6.2f}" if isinstance(r.get("volts"), float) else "     -"
        t = f"{r['temp']:4d}C" if isinstance(r.get("temp"), int) else "    -"
        print(f"  {pretty(r['joint']):<18}{r['id']:>3}  {ang}  {r['home']:7.1f}  {v}  {t}  "
              f"{r['result']:<8} {'; '.join(r['notes'])}")
