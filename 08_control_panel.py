"""
XIANG leg control panel: drive the two servos from a web page, with plain-language
feedback on every error.

    python3 08_control_panel.py                  # then open http://localhost:8765
    python3 08_control_panel.py --host 0.0.0.0   # on the Pi, to open it from your laptop
                                                 #   at http://<pi-address>:8765

Uses the same settings as the other scripts: leg_config.py (IDs, timing) and limits.json
(straight position, directions, safe range). Patterns come from 03_motions.py.
Directions on the page are named on Nancy's view of the leg, front of the robot to the right:
servo angle DOWN = thigh left/back (hip) and shin right/front (knee).
"""
import argparse
import glob
import importlib
import json
import os
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import serial
from pylx16a.lx16a import (LX16A, ServoArgumentError, ServoChecksumError, ServoError,
                           ServoLogicalError, ServoTimeoutError)

try:
    import termios
    PORT_STUCK_ERRORS = (OSError, termios.error)
except ImportError:                       # Windows
    PORT_STUCK_ERRORS = (OSError,)

import leg_config as C
from leg_common import load_settings

M = importlib.import_module("03_motions")
HERE = Path(__file__).resolve().parent

JOINTS = {"hip": C.HIP_ID, "knee": C.KNEE_ID}
WORDS = {"hip": ("thigh left / back", "thigh right / front"),      # (servo below straight, above)
         "knee": ("shin right / front", "shin left / back")}
PATTERN_INFO = {
    "swing": "Hip swings back and forth, knee straight",
    "curl": "Knee bends and straightens, hip still",
    "wave": "Both joints follow sine waves, out of phase (lecture example)",
    "squat": "Foot stays put while the hip lowers 23 mm",
    "walk": "Elephant walk: straight leg on the ground, low foot lift",
    "step": "Thigh left/back first, then shin right/front, then return",
    "crouch": "Bent-knee walk (dog / bird style)",
    "circle": "Foot traces a 30 mm circle",
    "kick": "Fast kick and return (speed test)",
}

V_REFUSE, V_LOW, V_HIGH, V_BLOCK = 4.5, 6.0, 8.4, 8.8
T_WARN, T_STOP = 55, C.MAX_TEMP_C
LAG_WARN = 6.0            # degrees short of target after a manual move
LAG_PATTERN = 8.0         # degrees of lag during a pattern that is worth reporting


class Panel:
    def __init__(self):
        self.lock = threading.RLock()
        self.events = deque(maxlen=300)
        self.next_id = 1
        self.recent = {}
        self.connected = False
        self.port = None
        self.servos = {}
        self.settings = load_settings()
        self.state = {j: dict(ok=False, angle=None, target=None, vin=None, temp=None, torque=None)
                      for j in JOINTS}
        self.pattern = dict(running=False, name=None, cycle=0, cycles=0, step=0, steps=0)
        self.stop_flag = threading.Event()
        self.hot_lock = False

    # ------------------------------------------------------------- feedback
    def event(self, level, code, msg, hint="", every_s=0):
        """level: info / warn / error. every_s > 0 suppresses repeats of the same code."""
        now = time.time()
        if every_s and now - self.recent.get(code, 0) < every_s:
            return
        self.recent[code] = now
        self.events.appendleft(dict(id=self.next_id, t=time.strftime("%H:%M:%S"), level=level,
                                    code=code, msg=msg, hint=hint))
        self.next_id += 1
        print(f"[{level.upper():5s}] {msg}" + (f"  -> {hint}" if hint else ""), flush=True)

    def name(self, j):
        return f"{j.capitalize()} motor (ID {JOINTS[j]})"

    def handle(self, exc, joint=None, action=""):
        """Turn any exception into a plain-language error event."""
        where = f" while {action}" if action else ""
        if isinstance(exc, ServoTimeoutError):
            j = joint or next((k for k, v in JOINTS.items() if v == getattr(exc, "id_", None)), None)
            if j:
                self.state[j]["ok"] = False
            self.event("error", f"TIMEOUT_{j}", f"{self.name(j) if j else 'A motor'} is not answering{where}.",
                       "Check the cable chain BusLinker -> hip -> knee is pushed in, the battery is on and the "
                       "converter shows about 7 V. Then press Reconnect.", every_s=5)
        elif isinstance(exc, ServoChecksumError):
            self.event("error", "CHECKSUM", f"Garbled reply from motor ID {getattr(exc, 'id_', '?')}{where}.",
                       "Two motors may share the same ID (new motors all ship as ID 1), or a cable is loose. "
                       "Hip must be ID 1 and knee ID 2: use 00_scan_and_set_ids.py with one motor plugged in.",
                       every_s=5)
        elif isinstance(exc, ServoLogicalError):
            self.event("warn", "TORQUE_OFF", f"{self.name(joint) if joint else 'Motor'} has torque off{where}.",
                       "Press Torque on first. (Motors with torque off are limp and ignore move commands.)",
                       every_s=3)
        elif isinstance(exc, ServoArgumentError):
            self.event("warn", "BAD_ANGLE", f"That angle is outside the motor's 0 to 240 degree range{where}.")
        elif isinstance(exc, (serial.SerialException,) + PORT_STUCK_ERRORS):
            self.connected = False
            for j in JOINTS:
                self.state[j]["ok"] = False
            self.event("error", "USB_LOST", f"Lost the USB connection to the BusLinker{where}.",
                       "Unplug the BusLinker USB cable, wait 3 seconds, plug it back in, then press Reconnect. "
                       "If this happens while the leg moves, the supply is probably sagging under load: turn the "
                       "converter's CC screw up (clockwise) about 10 turns.", every_s=5)
        else:
            self.event("error", "UNKNOWN", f"Unexpected error{where}: {type(exc).__name__}: {exc}")

    # ------------------------------------------------------------- connection
    def find_port(self):
        if os.environ.get("XIANG_PORT"):
            return os.environ["XIANG_PORT"]
        for pat in ("/dev/cu.usbserial*", "/dev/cu.wchusbserial*", "/dev/ttyUSB*"):
            hits = sorted(glob.glob(pat))
            if hits:
                return hits[0]
        return None

    def connect(self):
        with self.lock:
            self.connected, self.servos = False, {}
            self.settings = load_settings()
            port = self.find_port()
            if not port:
                self.event("error", "NO_PORT", "BusLinker not found on USB.",
                           "Plug the BusLinker into this computer with a DATA USB cable (some cables only "
                           "charge). On a Mac it appears as /dev/cu.usbserial-..., on the Pi as /dev/ttyUSB0.")
                return False
            try:
                LX16A.initialize(port, C.TIMEOUT_S)
            except PORT_STUCK_ERRORS as e:
                self.event("error", "PORT_STUCK", f"The USB port {port} is stuck ({e}).",
                           "Unplug the BusLinker USB cable, wait 3 seconds, plug it back in, then press Reconnect.")
                return False
            except serial.SerialException as e:
                self.event("error", "PORT_OPEN", f"Cannot open {port}: {e}",
                           "Another program may be using the port: close other leg scripts and try again.")
                return False
            self.port = port
            for j, sid in JOINTS.items():
                try:
                    self.servos[j] = LX16A(sid)
                    self.state[j]["ok"] = True
                except ServoError as e:
                    self.state[j]["ok"] = False
                    self.handle(e, j, "connecting")
            self.connected = bool(self.servos)
            if len(self.servos) == len(JOINTS):
                self.event("info", "CONNECTED", f"Connected on {port}: hip ID {C.HIP_ID} and knee ID {C.KNEE_ID} answer.")
                if not self.settings["measured"]:
                    self.event("warn", "NO_LIMITS", "limits.json not found: using the cautious ranges from leg_config.py.",
                               "Measure the range (07_find_extremes.py) so patterns are not cut short.")
            self.refresh()
            return self.connected

    # ------------------------------------------------------------- reading + checks
    def refresh(self):
        if not self.connected:
            return
        for j, s in list(self.servos.items()):
            st = self.state[j]
            try:
                st["angle"] = s.get_physical_angle()
                st["vin"] = s.get_vin() / 1000.0
                st["temp"] = s.get_temp()
                st["torque"] = s.is_torque_enabled(True)
                st["ok"] = True
            except Exception as e:
                self.handle(e, j, "reading the motor")
                return
            self.check_health(j)

    def check_health(self, j):
        st = self.state[j]
        v, t = st["vin"], st["temp"]
        if v is not None:
            if v < V_REFUSE:
                self.event("error", "V_REFUSE", f"Supply is only {v:.2f} V: the motors refuse to move below 4.5 V.",
                           "Is the battery switched on and the converter wired to the BusLinker POWER terminal? "
                           "USB alone only gives about 4.3 V.", every_s=20)
            elif v < V_LOW:
                self.event("warn", "V_LOW", f"Supply is {v:.2f} V, under the motors' 6 V minimum: weak, jerky motion.",
                           "Charge the battery or turn the converter's CV screw up to about 7 V.", every_s=20)
            elif v > V_HIGH:
                self.event("error", "V_HIGH", f"Supply is {v:.2f} V, above the motors' 8.4 V maximum.",
                           "Turn the converter's CV screw counter-clockwise until the display reads about 7.0 V.",
                           every_s=20)
        if t is not None:
            if t >= T_STOP:
                self.hot_lock = True
                self.event("error", "HOT", f"{self.name(j)} is {t} C: stopped to let it cool.",
                           "Turn torque off and wait until it is under 45 C. Check nothing is blocking the joint.",
                           every_s=20)
                self.stop_flag.set()
            elif t >= T_WARN:
                self.event("warn", "WARM", f"{self.name(j)} is {t} C and getting hot.",
                           "Give it a break soon (it stops by itself at 60 C).", every_s=30)
            elif self.hot_lock and t < 45:
                self.hot_lock = False
                self.event("info", "COOL", "Motors have cooled down: moving is allowed again.")

    def problems(self):
        """Current conditions that need attention, shown in the banner."""
        out = []
        if not self.connected:
            out.append(("error", "Not connected to the BusLinker.", "Plug it in and press Reconnect."))
            return out
        for j in JOINTS:
            st = self.state[j]
            if not st["ok"]:
                out.append(("error", f"{self.name(j)} is not answering.",
                            "Check its cable and the battery, then press Reconnect."))
                continue
            v = st["vin"]
            if v is not None and v < V_REFUSE:
                out.append(("error", f"{j.capitalize()} supply {v:.2f} V: too low to move.",
                            "Battery on? Converter wired to the BusLinker power terminal?"))
            elif v is not None and v > V_HIGH:
                out.append(("error", f"{j.capitalize()} supply {v:.2f} V: above the 8.4 V maximum.",
                            "Turn the converter's CV screw down to about 7 V."))
            elif v is not None and v < V_LOW:
                out.append(("warn", f"{j.capitalize()} supply {v:.2f} V: under 6 V.", "Expect weak motion."))
            if st["temp"] is not None and st["temp"] >= T_WARN:
                out.append(("warn" if st["temp"] < T_STOP else "error",
                            f"{self.name(j)} is {st['temp']} C.", "Let it cool."))
            if st["torque"] is False:
                out.append(("info", f"{j.capitalize()} torque is off (limp).", "Press Torque on to move it."))
        return out

    # ------------------------------------------------------------- commands
    def can_move(self):
        if not self.connected:
            self.event("error", "NOT_CONNECTED", "Cannot move: not connected.", "Press Reconnect.")
            return False
        if self.hot_lock:
            self.event("error", "HOT_LOCK", "Cannot move: a motor is too hot.", "Wait until it cools under 45 C.")
            return False
        vs = [self.state[j]["vin"] for j in JOINTS if self.state[j]["vin"] is not None]
        if vs and max(vs) > V_BLOCK:
            self.event("error", "V_BLOCK", f"Cannot move: supply {max(vs):.2f} V could damage the motors.",
                       "Turn the converter's CV screw down to about 7 V first.")
            return False
        return True

    def clamp(self, j, angle):
        lo, hi = self.settings[f"{j}_min"], self.settings[f"{j}_max"]
        a = max(lo, min(hi, angle))
        return a, abs(a - angle) > 0.05

    def move_joint(self, j, angle, ms=800, check=True):
        with self.lock:
            if not self.can_move() or j not in self.servos:
                return
            a, clipped = self.clamp(j, angle)
            if clipped:
                self.event("warn", f"CLAMP_{j}", f"{j.capitalize()} target {angle:.0f} is outside the safe range; "
                           f"limited to {a:.0f}.", f"Safe range is {self.settings[j + '_min']:.0f} to "
                           f"{self.settings[j + '_max']:.0f} (from limits.json).", every_s=3)
            try:
                s = self.servos[j]
                if self.state[j]["torque"] is False:
                    s.enable_torque()
                    self.state[j]["torque"] = True
                    self.event("info", f"TQ_{j}", f"{j.capitalize()} torque turned on.", every_s=5)
                s.move(a, time=int(ms))
                self.state[j]["target"] = a
            except Exception as e:
                self.handle(e, j, "moving")
                return
        if check:
            threading.Timer(ms / 1000 + 0.4, self.check_lag, args=(j, a)).start()

    def check_lag(self, j, target):
        if self.pattern["running"]:
            return
        with self.lock:
            try:
                ang = self.servos[j].get_physical_angle()
            except Exception as e:
                self.handle(e, j, "checking the move")
                return
            self.state[j]["angle"] = ang
            lag = abs(target - ang)
            if lag > LAG_WARN:
                self.event("warn", f"LAG_{j}", f"{j.capitalize()} stopped {lag:.1f} deg short of its target "
                           f"({ang:.1f} instead of {target:.1f}).",
                           "Something may be blocking the joint (a cable, the clamp, a part touching), the joint "
                           "may be overloaded, or the supply is sagging. Check the leg, then try a smaller move.")

    def jog(self, j, delta):
        st = self.state[j]
        base = st["target"] if st["target"] is not None else (st["angle"] or 120.0)
        self.move_joint(j, base + delta, ms=400)

    def straight(self):
        for j in JOINTS:
            self.move_joint(j, self.settings[f"{j}_straight"], ms=1500)

    def torque(self, on):
        with self.lock:
            for j, s in self.servos.items():
                try:
                    s.enable_torque() if on else s.disable_torque()
                    self.state[j]["torque"] = on
                except Exception as e:
                    self.handle(e, j, "switching torque")
            self.event("info" if on else "warn", "TQ", "Torque ON: motors hold their position." if on else
                       "Torque OFF: motors are limp. Support the leg so it does not drop.")

    def estop(self):
        self.stop_flag.set()
        self.torque(False)

    # ------------------------------------------------------------- patterns
    def start_pattern(self, name, cycles, speed):
        if self.pattern["running"]:
            self.event("warn", "BUSY", "A pattern is already running.", "Press Stop first.")
            return
        if name not in M.PATTERNS:
            self.event("error", "NO_PATTERN", f"Unknown pattern '{name}'.")
            return
        if not self.can_move():
            return
        if speed > 1.0:
            self.event("warn", "FAST", f"Speed x{speed} is faster than tested.",
                       "The lecture warns that motors cannot keep up with commands much faster than 4 per second.")
        self.stop_flag.clear()
        threading.Thread(target=self._run, args=(name, int(cycles), float(speed)), daemon=True).start()

    def _servo_for(self, j, joint_deg):
        return self.settings[f"{j}_straight"] + self.settings[f"{j}_sign"] * joint_deg

    def _run(self, name, cycles, speed):
        targets = M.PATTERNS[name]()
        step_s = C.STEP_S / speed
        p = self.pattern
        p.update(running=True, name=name, cycle=0, cycles=cycles, step=0, steps=len(targets))
        worst = {"hip": 0.0, "knee": 0.0}
        clamped, vmin, warmup = 0, 99.0, 2
        self.event("info", "RUN", f"Running '{name}': {cycles} cycles at x{speed} speed.")
        try:
            with self.lock:
                for j, s in self.servos.items():
                    s.enable_torque()
                    self.state[j]["torque"] = True
            h0, k0 = targets[0]
            for j, ja in (("hip", h0), ("knee", k0)):
                self.move_joint(j, self._servo_for(j, ja), ms=1000, check=False)
            time.sleep(1.1)
            for c in range(cycles):
                p["cycle"] = c + 1
                for i, (h, k) in enumerate(targets):
                    if self.stop_flag.is_set():
                        self.event("info", "STOPPED", f"'{name}' stopped. The leg holds its position.")
                        return
                    p["step"] = i + 1
                    with self.lock:
                        if not self.can_move():
                            raise RuntimeError("movement blocked")
                        for j, ja in (("hip", h), ("knee", k)):
                            a, clip = self.clamp(j, self._servo_for(j, ja))
                            clamped += clip
                            self.servos[j].move(a, time=int(step_s * 1000))
                            self.state[j]["target"] = a
                    time.sleep(step_s)
                    with self.lock:                      # heartbeat: catches a disconnected motor at once
                        for j in JOINTS:
                            ang = self.servos[j].get_physical_angle()
                            self.state[j]["angle"] = ang
                            if warmup <= 0:
                                worst[j] = max(worst[j], abs(self.state[j]["target"] - ang))
                        if i % 4 == 0:
                            for j in JOINTS:
                                v = self.servos[j].get_vin() / 1000.0
                                self.state[j]["vin"] = v
                                vmin = min(vmin, v)
                    warmup -= 1
                with self.lock:
                    for j in JOINTS:
                        self.state[j]["temp"] = self.servos[j].get_temp()
                        self.check_health(j)
            self.event("info", "DONE", f"'{name}' finished: worst lag hip {worst['hip']:.1f} / knee "
                       f"{worst['knee']:.1f} deg, lowest supply {vmin:.2f} V.")
            if clamped:
                self.event("warn", "CLAMPED_RUN", f"{clamped} pattern steps were limited to the safe range, "
                           "so the motion was smaller than designed.",
                           "Measure the real range (07_find_extremes.py) or check the directions in limits.json.")
            if max(worst.values()) > LAG_PATTERN:
                self.event("warn", "LAG_RUN", f"A joint fell up to {max(worst.values()):.1f} deg behind during '{name}'.",
                           "Try a slower speed, check nothing rubs, and check the supply does not sag.")
            if vmin < V_LOW:
                self.event("warn", "SAG", f"Supply dipped to {vmin:.2f} V while moving.",
                           "The converter cannot deliver enough current: turn its CC screw up (clockwise).")
            self.straight()
        except RuntimeError:
            pass
        except Exception as e:
            self.handle(e, None, f"running '{name}'")
            self.event("error", "RUN_FAIL", f"'{name}' stopped because of the error above.")
        finally:
            p["running"] = False

    # ------------------------------------------------------------- state for the page
    def snapshot(self):
        with self.lock:
            joints = {}
            for j in JOINTS:
                st = dict(self.state[j])
                st_straight = self.settings[f"{j}_straight"]
                if st["angle"] is not None:
                    d = st["angle"] - st_straight
                    st["from_straight"] = abs(d)
                    st["direction"] = "straight" if abs(d) < 1.5 else WORDS[j][0 if d < 0 else 1]
                st.update(id=JOINTS[j], straight=st_straight, min=self.settings[f"{j}_min"],
                          max=self.settings[f"{j}_max"], words=WORDS[j])
                joints[j] = st
            return dict(connected=self.connected, port=self.port, joints=joints,
                        pattern=dict(self.pattern), problems=self.problems(), events=list(self.events)[:80],
                        patterns=[dict(name=n, info=PATTERN_INFO.get(n, "")) for n in M.PATTERNS],
                        geometry=dict(thigh=C.THIGH_MM, shin=C.SHIN_DOWN_MM))

    def monitor(self):
        while True:
            time.sleep(0.6)
            if self.connected and not self.pattern["running"]:
                with self.lock:
                    self.refresh()


PANEL = Panel()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype="application/json"):
        data = body if isinstance(body, bytes) else json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            return self._send(200, (HERE / "control_panel.html").read_bytes(), "text/html; charset=utf-8")
        if self.path == "/api/state":
            return self._send(200, PANEL.snapshot())
        self._send(404, {"error": "not found"})

    def do_POST(self):
        n = int(self.headers.get("Content-Length") or 0)
        try:
            b = json.loads(self.rfile.read(n) or b"{}")
        except json.JSONDecodeError:
            return self._send(400, {"error": "bad json"})
        p, P = self.path, PANEL
        try:
            if p == "/api/connect":
                P.connect()
            elif p == "/api/move":
                P.move_joint(b["joint"], float(b["angle"]), int(b.get("ms", 800)))
            elif p == "/api/jog":
                P.jog(b["joint"], float(b["delta"]))
            elif p == "/api/straight":
                P.straight()
            elif p == "/api/torque":
                P.torque(bool(b["on"]))
            elif p == "/api/pattern":
                P.start_pattern(b["name"], b.get("cycles", 2), b.get("speed", 0.5))
            elif p == "/api/stop":
                P.stop_flag.set()
            elif p == "/api/estop":
                P.estop()
            elif p == "/api/clear":
                P.events.clear()
            else:
                return self._send(404, {"error": "not found"})
        except Exception as e:
            P.handle(e, None, f"handling {p}")
        self._send(200, {"ok": True})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8765)
    a = ap.parse_args()
    PANEL.connect()
    threading.Thread(target=PANEL.monitor, daemon=True).start()
    print(f"XIANG control panel: open http://{'localhost' if a.host == '127.0.0.1' else a.host}:{a.port}", flush=True)
    ThreadingHTTPServer((a.host, a.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
