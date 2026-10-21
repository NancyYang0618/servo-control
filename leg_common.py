"""
XIANG Step 4: the Leg class every script uses.

What it does for you:
- connects to the hip and knee servos
- converts "joint angles" (0 = leg straight, + hip = foot forward, + knee = knee bent)
  into servo angles, and refuses to go outside the measured safe range
- after EVERY move it reads both servos back. A move command never gets a reply, so a
  read-back is the only way to notice that a motor stopped answering (cable pulled, no power).
- if a motor stops answering it raises MotorDisconnected; run_pattern() catches it, holds
  the other motor still, waits for you to plug it back in, then recovers to straight
- logs every step to a CSV file in logs/ for your plots and slides
"""

import csv
import json
import math
import time
from pathlib import Path

import serial
import termios
from pylx16a.lx16a import LX16A, ServoError, ServoTimeoutError, ServoChecksumError

import leg_config as C

HERE = Path(__file__).resolve().parent
LIMITS_FILE = HERE / "limits.json"
LOG_DIR = HERE / "logs"


class MotorDisconnected(Exception):
    def __init__(self, name, servo_id, cause):
        super().__init__(f"{name.upper()} motor (ID {servo_id}) is not responding: {type(cause).__name__}")
        self.name, self.servo_id, self.cause = name, servo_id, cause


# ------------------------------------------------------------------ settings
def load_settings():
    """Config defaults, overridden by limits.json from 02_jog_and_measure.py if it exists."""
    s = dict(hip_straight=C.HIP_STRAIGHT, knee_straight=C.KNEE_STRAIGHT,
             hip_sign=C.HIP_SIGN, knee_sign=C.KNEE_SIGN,
             hip_min=C.HIP_MIN, hip_max=C.HIP_MAX, knee_min=C.KNEE_MIN, knee_max=C.KNEE_MAX,
             measured=False)
    if LIMITS_FILE.exists():
        m = json.loads(LIMITS_FILE.read_text())
        s.update({k: m[k] for k in s if k in m})
        # stay SAFETY_MARGIN away from the hard stops you measured
        s["hip_min"] += C.SAFETY_MARGIN
        s["hip_max"] -= C.SAFETY_MARGIN
        s["knee_min"] += C.SAFETY_MARGIN
        s["knee_max"] -= C.SAFETY_MARGIN
        s["measured"] = True
    return s


# ------------------------------------------------------------------ geometry
def foot_position(hip_deg, knee_deg):
    """Joint angles -> foot (x forward, z up) in mm, relative to the hip axis."""
    th = math.radians(hip_deg)
    phi = math.radians(hip_deg - knee_deg)          # shin direction from straight down
    kx, kz = C.THIGH_MM * math.sin(th), -C.THIGH_MM * math.cos(th)
    fx = kx + C.SHIN_FWD_MM * math.cos(phi) + C.SHIN_DOWN_MM * math.sin(phi)
    fz = kz + C.SHIN_FWD_MM * math.sin(phi) - C.SHIN_DOWN_MM * math.cos(phi)
    return fx, fz


def joint_angles_for_foot(x, z, prev_knee=0.0):
    """Foot target (mm from hip axis) -> (hip, knee) joint angles, or None if out of reach.
    Of the two possible knee solutions, picks the one closest to the previous knee angle
    so the motion is smooth."""
    L1 = C.THIGH_MM
    L2 = math.hypot(C.SHIN_DOWN_MM, C.SHIN_FWD_MM)
    delta = math.atan2(C.SHIN_FWD_MM, C.SHIN_DOWN_MM)
    d2 = x * x + z * z
    c = (d2 - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    if abs(c) > 1:
        return None
    gamma = math.atan2(x, -z)
    best = None
    for D in (math.acos(c), -math.acos(c)):
        a1 = gamma + math.atan2(L2 * math.sin(D), L1 + L2 * math.cos(D))
        hip = math.degrees(a1)
        knee = math.degrees(D + delta)
        if knee < -0.5:
            continue
        if best is None or abs(knee - prev_knee) < abs(best[1] - prev_knee):
            best = (hip, knee)
    return best


# ------------------------------------------------------------------ the leg
class Leg:
    def __init__(self, log_name=None, quiet=False):
        self.s = load_settings()
        if not self.s["measured"] and not quiet:
            print("NOTE: limits.json not found, using the cautious ranges in leg_config.py.\n"
                  "      Run 02_jog_and_measure.py to measure the real extremes.\n")
        self._init_port()
        self.hip = self._open(C.HIP_ID, "hip")
        self.knee = self._open(C.KNEE_ID, "knee")
        self.t0 = time.time()
        self.cmd = (0.0, 0.0)
        self.log = None
        if log_name:
            LOG_DIR.mkdir(exist_ok=True)
            path = LOG_DIR / f"{time.strftime('%Y%m%d_%H%M%S')}_{log_name}.csv"
            self._f = open(path, "w", newline="")
            self.log = csv.writer(self._f)
            self.log.writerow(["t_s", "pattern", "hip_cmd", "knee_cmd", "hip_meas", "knee_meas",
                               "foot_x_mm", "foot_z_mm", "event"])
            self.log_path = path
            print(f"logging to {path}")

    # ---- connection
    def _init_port(self):
        try:
            LX16A.initialize(C.PORT, C.TIMEOUT_S)
        except (OSError, termios.error) as e:   # macOS USB-serial driver stuck (termios error 22)
            raise SystemExit(f"The USB port {C.PORT} is stuck ({e}).\n"
                             "Unplug the BusLinker USB cable, wait 3 seconds, plug it back in, then run again.")
        except serial.SerialException as e:
            raise SystemExit(f"Cannot open {C.PORT} ({e}).\n"
                             "Is the BusLinker plugged into the Pi by USB? Try PORT = '/dev/ttyUSB1' in leg_config.py,\n"
                             "or run: ls /dev/ttyUSB*")

    def _open(self, servo_id, name):
        try:
            servo = LX16A(servo_id)
        except (ServoTimeoutError, ServoChecksumError) as e:
            raise MotorDisconnected(name, servo_id, e)
        return servo

    # ---- angle conversion
    def to_servo(self, joint, which):
        st, sg = self.s[f"{which}_straight"], self.s[f"{which}_sign"]
        lo, hi = self.s[f"{which}_min"], self.s[f"{which}_max"]
        a = st + sg * joint
        return max(lo, min(hi, a)), (a < lo or a > hi)

    def to_joint(self, servo_angle, which):
        return (servo_angle - self.s[f"{which}_straight"]) * self.s[f"{which}_sign"]

    # ---- motion
    def move(self, hip_deg, knee_deg, seconds=C.STEP_S, pattern="", event=""):
        """Move to joint angles over `seconds`, then read both motors back."""
        ms = int(1000 * seconds / C.SPEED_SCALE)
        h, h_clip = self.to_servo(hip_deg, "hip")
        k, k_clip = self.to_servo(knee_deg, "knee")
        if (h_clip or k_clip) and not event:
            event = "clamped to safe range"
        for servo, angle, name in ((self.hip, h, "hip"), (self.knee, k, "knee")):
            try:
                servo.move(angle, time=ms)
            except serial.SerialException as e:
                raise MotorDisconnected("usb", 0, e)
            except (ServoTimeoutError, ServoChecksumError) as e:
                raise MotorDisconnected(name, servo.get_id(), e)
        self.cmd = (hip_deg, knee_deg)
        time.sleep(ms / 1000.0)
        hm, km = self.read()
        if self.log:
            fx, fz = foot_position(hm, km)
            self.log.writerow([f"{time.time() - self.t0:.3f}", pattern, f"{hip_deg:.1f}", f"{knee_deg:.1f}",
                               f"{hm:.1f}", f"{km:.1f}", f"{fx:.1f}", f"{fz:.1f}", event])
        return hm, km

    def read(self):
        """Joint angles actually measured by the motors. Raises MotorDisconnected if one is silent."""
        out = []
        for servo, name, sid in ((self.hip, "hip", C.HIP_ID), (self.knee, "knee", C.KNEE_ID)):
            try:
                out.append(self.to_joint(servo.get_physical_angle(), name))
            except serial.SerialException as e:
                raise MotorDisconnected("usb", 0, e)
            except (ServoTimeoutError, ServoChecksumError) as e:
                raise MotorDisconnected(name, sid, e)
        return tuple(out)

    def health(self):
        rows = []
        for servo, name in ((self.hip, "hip"), (self.knee, "knee")):
            rows.append((name, servo.get_vin() / 1000.0, servo.get_temp()))
        return rows

    def go_straight(self, seconds=1.5, pattern="", event="go straight"):
        return self.move(0.0, 0.0, seconds, pattern, event)

    def relax(self):
        """Torque off (the leg goes limp). Ignores motors that are not answering."""
        for servo in (self.hip, self.knee):
            try:
                servo.disable_torque()
            except Exception:
                pass

    def note(self, pattern, event):
        if self.log:
            self.log.writerow([f"{time.time() - self.t0:.3f}", pattern, "", "", "", "", "", "", event])

    def close(self):
        if self.log:
            self._f.close()

    # ---- recovery after a disconnect
    def recover(self, err, pattern="", wait_s=60):
        print("\n" + "=" * 64)
        print(f"!!  {err}")
        if err.name == "usb":
            print("!!  The BusLinker USB cable seems unplugged. Plug it back in.")
        else:
            print(f"!!  Check the {err.name} servo cable and the battery. Plug it back in;")
        print(f"!!  waiting up to {wait_s} s. Press Ctrl+C to give up.")
        print("=" * 64)
        self.note(pattern, f"DISCONNECT: {err}")
        t_end = time.time() + wait_s
        while time.time() < t_end:
            time.sleep(0.5)
            try:
                if err.name == "usb":
                    self._init_port()
                self.hip = self._open(C.HIP_ID, "hip")
                self.knee = self._open(C.KNEE_ID, "knee")
                for servo in (self.hip, self.knee):
                    servo.enable_torque()
                self.read()
            except (MotorDisconnected, ServoError, serial.SerialException, SystemExit):
                print("   still waiting...", end="\r")
                continue
            print("\nBoth motors answer again. Returning slowly to straight.")
            self.note(pattern, "RECONNECTED")
            self.go_straight(2.0, pattern, "recover to straight")
            return True
        print("\nGave up waiting. Stopping safely.")
        self.note(pattern, "GAVE UP")
        return False


def run_pattern(leg, name, targets, cycles=1, seconds=C.STEP_S):
    """Play a list of (hip, knee) joint targets `cycles` times.
    Survives a motor disconnect: waits, recovers, then restarts the current cycle."""
    c = 0
    while c < cycles:
        try:
            for hip, knee in targets:
                leg.move(hip, knee, seconds, pattern=name)
            c += 1
            for n, v, t in leg.health():
                if t > C.MAX_TEMP_C:
                    print(f"{n} servo is {t} C, stopping to let it cool.")
                    return False
        except MotorDisconnected as e:
            if not leg.recover(e, name):
                return False
    return True
