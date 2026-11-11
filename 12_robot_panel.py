#!/usr/bin/env python3
"""
XIANG robot control panel: all 4 legs / 8 servos from a web page, with the boot test,
homing, emergency stop and plain-language feedback on every error.

    python3 12_robot_panel.py --host all     on the Pi; open http://xiang.local:8765 on the laptop

At start-up it connects and runs boot test stage 1 (read only, nothing moves).
Nothing moves until you press a button. Every first move holds the joint where it is first,
so switching a motor on never makes it jump. Settings: robot_config.json.
"""
import argparse
import json
import socket
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from robot import HERE, JOINT_ORDER, LEG_NAMES, LEGS, BusError, Robot, pretty, save_homes

POLL_S = 1.0


class Panel:
    def __init__(self):
        self.robot = None
        self.lock = threading.RLock()
        self.events = deque(maxlen=300)
        self.next_id = 1
        self.recent = {}
        self.busy = None                  # name of the routine running, or None
        self.state = {j: dict(ok=False, angle=None, volts=None, temp=None, motor=None, target=None)
                      for j in JOINT_ORDER}
        self.last_test = None             # dict(kind, time, ok, rows)

    # ---------------------------------------------------------------- feedback
    def event(self, level, code, msg, hint="", every_s=0):
        now = time.time()
        if every_s and now - self.recent.get(code, 0) < every_s:
            return
        self.recent[code] = now
        self.events.appendleft(dict(id=self.next_id, t=time.strftime("%H:%M:%S"), level=level, msg=msg, hint=hint))
        self.next_id += 1
        print(f"[{level.upper():5s}] {msg}" + (f"  -> {hint}" if hint else ""), flush=True)

    def bus_error(self, e, action=""):
        where = f" while {action}" if action else ""
        self.event("error", f"BUS_{e}", f"{e}{where}", getattr(e, "hint", ""), every_s=5)
        if "USB" in str(e) or "BusLinker" in str(e):
            self.robot = None

    # ---------------------------------------------------------------- connection
    def connect(self):
        with self.lock:
            if self.robot:
                self.robot.bus.close()
            self.robot = None
            try:
                self.robot = Robot(log=lambda m: self.event("info", "LOG", m))
            except BusError as e:
                self.event("error", "NO_BUS", str(e), e.hint)
                return False
            self.event("info", "CONNECTED", f"Connected to the BusLinker on {self.robot.bus.port}.")
        self.run_routine("boot test", self._boot_test, False)
        return True

    @property
    def cfg(self):
        return self.robot.cfg if self.robot else None

    # ---------------------------------------------------------------- reading
    def refresh(self):
        r = self.robot
        if not r:
            return
        for j in JOINT_ORDER:
            st = self.state[j]
            try:
                st.update(r.bus.read_all(r.sid(j)), ok=True)
            except BusError as e:
                if "USB" in str(e):
                    self.bus_error(e, "reading")
                    return
                st["ok"] = False
                self.event("error", f"NOANS_{j}", f"{pretty(j).capitalize()} (ID {r.sid(j)}) is not answering.",
                           e.hint, every_s=10)
                continue
            self.check_health(j)

    def check_health(self, j):
        c, st = self.cfg, self.state[j]
        v, t = st["volts"], st["temp"]
        if v < c["volts"]["refuse"]:
            self.event("error", "V_REFUSE", f"Supply is only {v:.2f} V: motors refuse to move below 4.5 V.",
                       "Is the battery on and the converter wired to the BusLinker power terminal?", every_s=20)
        elif v < c["volts"]["low"]:
            self.event("warn", "V_LOW", f"Supply sagged to {v:.2f} V (under 6 V): weak, jerky motion.",
                       "Charge the battery, or turn the converter's CC screw clockwise so it can give more "
                       "current, and check CV is about 7 V.", every_s=20)
        elif v > c["volts"]["high"]:
            self.event("error", "V_HIGH", f"Supply is {v:.2f} V, above the motors' 8.4 V maximum.",
                       "Turn the converter's CV screw counter-clockwise to about 7.0 V.", every_s=20)
        if t >= c["temp_c"]["stop"]:
            self.event("error", "HOT", f"{pretty(j).capitalize()} is {t} C: all motors switched off to cool.",
                       "Wait until it is under 45 C. Check nothing blocks the joint.", every_s=20)
            self.robot.all_off()
        elif t >= c["temp_c"]["warn"]:
            self.event("warn", f"WARM_{j}", f"{pretty(j).capitalize()} is {t} C and getting hot.",
                       "Give it a rest soon (motors switch off at 60 C).", every_s=30)

    def problems(self):
        out = []
        if not self.robot:
            return [("error", "Not connected to the BusLinker.", "Plug it into the Pi and press Reconnect.")]
        c = self.cfg
        for j in JOINT_ORDER:
            st, s = self.state[j], c["servos"][j]
            name = f"{pretty(j).capitalize()} (ID {s['id']})"
            if not st["ok"]:
                out.append(("error", f"{name} is not answering.", "Check its plug in the daisy chain."))
                continue
            if not s["holdable"]:
                out.append(("warn", f"{name}: home {s['home']:.0f} deg is outside the servo's 0 to 240 range, "
                            "so it is locked (motor stays off).",
                            "Re-mount this joint's discs so the leg is straight near 120 deg, then press "
                            "Save pose as home."))
            v = st["volts"]
            if v is not None and v < c["volts"]["refuse"]:
                out.append(("error", f"Servo supply is only {v:.2f} V: the motors cannot move.",
                            "That is USB power alone. Switch the battery and converter on (converter about 7 V "
                            "into the BusLinker power terminal). If it cut out while motors were holding, turn "
                            "the converter's CC screw clockwise to allow more current."))
                break
            if v is not None and v < c["volts"]["low"]:
                out.append(("warn", f"Supply {v:.2f} V at {name}.", "Under 6 V: weak motion."))
                break
        return out

    # ---------------------------------------------------------------- moving
    def can_move(self, j=None):
        if not self.robot:
            self.event("error", "NOT_CONNECTED", "Cannot move: not connected.", "Press Reconnect.")
            return False
        if self.busy:
            self.event("warn", "BUSY", f"Wait: {self.busy} is running.", every_s=2)
            return False
        if j and not self.cfg["servos"][j]["holdable"]:
            self.event("warn", f"LOCKED_{j}", f"{pretty(j).capitalize()} is locked: its home is outside 0 to 240.",
                       "Re-mount the joint, then Save pose as home.", every_s=3)
            return False
        vs = [self.state[k]["volts"] for k in JOINT_ORDER if self.state[k]["volts"]]
        if vs and max(vs) > self.cfg["volts"]["block"]:
            self.event("error", "V_BLOCK", f"Cannot move: supply {max(vs):.2f} V could damage the motors.",
                       "Turn the converter's CV screw down to about 7 V.")
            return False
        return True

    def move(self, j, target, speed=60.0):
        with self.lock:
            if not self.can_move(j):
                return
            r, sid = self.robot, self.robot.sid(j)
            a = r.clamp(j, target)
            if abs(a - target) > 0.05:
                s = self.cfg["servos"][j]
                self.event("warn", f"CLAMP_{j}", f"{pretty(j).capitalize()}: {target:.1f} deg is outside its safe "
                           f"range, limited to {a:.1f}.", f"Safe range {s['min']:.0f} to {s['max']:.0f} deg "
                           f"(home {s['home']:.0f} +/- swing in robot_config.json).", every_s=3)
            try:
                now = r.bus.angle(sid)
                if not 0 <= now <= 240:
                    self.event("error", f"OUT_{j}", f"{pretty(j).capitalize()} reads {now:.1f} deg, outside 0 to 240: "
                               "switching it on would make it jump.", "Move it by hand into range first.")
                    return
                if not r.bus.motor_on(sid):
                    r.bus.move(sid, now, 0)              # hold first: no jump
                    time.sleep(0.05)
                ms = max(200, abs(a - now) / speed * 1000)
                r.bus.move(sid, a, ms)
                self.state[j]["target"] = a
            except BusError as e:
                self.bus_error(e, f"moving the {pretty(j)}")
                return
        threading.Timer(ms / 1000 + 0.4, self.check_lag, args=(j, a)).start()

    def check_lag(self, j, target):
        if not self.robot or self.busy:
            return
        try:
            ang = self.robot.bus.angle(self.robot.sid(j))
        except BusError as e:
            self.bus_error(e, "checking a move")
            return
        lag = abs(ang - target)
        if lag > 4.0:
            self.event("warn", f"LAG_{j}", f"{pretty(j).capitalize()} stopped {lag:.1f} deg short "
                       f"({ang:.1f} instead of {target:.1f}).",
                       "Something may block the joint (a cable, a part touching), or the supply is sagging.")

    def nudge(self, j, delta):
        st = self.state[j]
        base = st["target"] if st["target"] is not None and st["motor"] else st["angle"]
        if base is None:
            return
        self.move(j, base + delta)

    def motors_off(self):
        if self.robot:
            self.robot.all_off()
            for j in JOINT_ORDER:
                self.state[j]["target"] = None
            self.event("warn", "OFF", "All motors switched OFF (limp). Support the robot so it does not drop.")

    def blink(self, j):
        if self.robot:
            threading.Thread(target=self._blink, args=(j,), daemon=True).start()

    def _blink(self, j):
        try:
            self.robot.blink(j, 3.0)
        except BusError as e:
            self.bus_error(e, "blinking")

    # ---------------------------------------------------------------- routines
    def run_routine(self, name, fn, *args):
        if not self.robot:
            self.event("error", "NOT_CONNECTED", f"Cannot run {name}: not connected.", "Press Reconnect.")
            return
        if self.busy:
            self.event("warn", "BUSY", f"Wait: {self.busy} is running.")
            return
        self.busy = name
        self.robot.stop.clear()
        threading.Thread(target=self._run, args=(name, fn, args), daemon=True).start()

    def _run(self, name, fn, args):
        try:
            fn(*args)
        except BusError as e:
            self.bus_error(e, name)
        except Exception as e:
            self.event("error", "UNKNOWN", f"{name} stopped: {type(e).__name__}: {e}")
        finally:
            self.busy = None
            try:
                with self.lock:
                    self.refresh()
            except Exception:
                pass

    def _boot_test(self, motion):
        ok, rows = self.robot.boot_test(motion=motion)
        self.last_test = dict(kind="Boot test" + (" + wiggle" if motion else " (read only)"),
                              time=time.strftime("%H:%M:%S"), ok=ok, rows=rows)
        fails = [r for r in rows if r["result"] != "PASS"]
        if fails:
            self.event("warn", "BOOT", f"Boot test: {len(rows) - len(fails)} of {len(rows)} servos OK. "
                       f"Problems: {', '.join(pretty(r['joint']) for r in fails)}.", "See the results table.")
        else:
            self.event("info", "BOOT", f"Boot test passed: all {len(rows)} servos OK.")

    def _home(self, legs):
        joints = [j for j in JOINT_ORDER if j[:2] in legs]
        ok, rows = self.robot.home(joints, progress=lambda m: self.event("info", "HOMING", m))
        self.last_test = dict(kind="Homing", time=time.strftime("%H:%M:%S"), ok=ok, rows=rows)
        for j in joints:
            if self.cfg["servos"][j]["holdable"]:
                self.state[j]["target"] = self.cfg["servos"][j]["home"]
        bad = [r for r in rows if r["result"] not in ("PASS",)]
        if bad:
            self.event("warn", "HOMED", f"Homing done with problems: {', '.join(pretty(r['joint']) for r in bad)}.",
                       "See the results table.")
        else:
            self.event("info", "HOMED", "Homing done: every joint is home and holding.")

    def save_home(self):
        with self.lock:
            if not self.robot or self.busy:
                return
            angles = {}
            for j in JOINT_ORDER:
                try:
                    angles[j] = self.robot.bus.angle(self.robot.sid(j))
                except BusError as e:
                    self.bus_error(e, "saving home")
                    self.event("error", "SAVE_FAIL", "Home NOT saved: a servo did not answer.")
                    return
            save_homes(angles)
            self.robot.reload()
            self.event("info", "SAVED", "Saved the current pose as home for all 8 servos.",
                       ", ".join(f"{pretty(j)} {a:.1f}" for j, a in angles.items()))

    # ---------------------------------------------------------------- page state
    def snapshot(self):
        c = self.cfg
        joints = {}
        for j in JOINT_ORDER:
            st = dict(self.state[j])
            if c:
                s = c["servos"][j]
                st.update(id=s["id"], home=s["home"], min=s["min"], max=s["max"], holdable=s["holdable"])
                if st["angle"] is not None:
                    st["from_home"] = st["angle"] - s["home"]
            st["name"] = pretty(j)
            joints[j] = st
        vs = [s["volts"] for s in self.state.values() if s["volts"]]
        return dict(connected=bool(self.robot), port=self.robot.bus.port if self.robot else None,
                    busy=self.busy, joints=joints, legs=LEGS, leg_names=LEG_NAMES,
                    vmin=min(vs) if vs else None, problems=self.problems(),
                    last_test=self.last_test, events=list(self.events)[:80])

    def monitor(self):
        while True:
            time.sleep(POLL_S)
            if self.robot and not self.busy:
                try:
                    with self.lock:
                        self.refresh()
                except Exception as e:
                    self.event("error", "MONITOR", f"Reading failed: {e}", every_s=10)


PANEL = Panel()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype="application/json"):
        data = body if isinstance(body, bytes) else json.dumps(body, default=str).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            return self._send(200, (HERE / "robot_panel.html").read_bytes(), "text/html; charset=utf-8")
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
                P.move(b["joint"], float(b["angle"]))
            elif p == "/api/nudge":
                P.nudge(b["joint"], float(b["delta"]))
            elif p == "/api/blink":
                P.blink(b["joint"])
            elif p == "/api/off":
                P.motors_off()
            elif p == "/api/stop":
                if P.robot:
                    P.robot.stop.set()
                P.event("info", "STOP", "Stop pressed: joints hold where they are.")
            elif p == "/api/boot_test":
                P.run_routine("boot test", P._boot_test, bool(b.get("motion")))
            elif p == "/api/home":
                legs = [l for l in b.get("legs", LEGS) if l in LEGS]
                P.run_routine("homing", P._home, legs)
            elif p == "/api/save_home":
                P.save_home()
            elif p == "/api/clear":
                P.events.clear()
            else:
                return self._send(404, {"error": "not found"})
        except BusError as e:
            P.bus_error(e, f"handling {p}")
        except Exception as e:
            P.event("error", "UNKNOWN", f"Error handling {p}: {type(e).__name__}: {e}")
        self._send(200, {"ok": True})


class DualStackServer(ThreadingHTTPServer):
    """IPv6 and IPv4 at once (phone hotspots can give the laptop IPv6 only)."""
    address_family = socket.AF_INET6

    def server_bind(self):
        try:
            self.socket.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
        except OSError:
            pass
        super().server_bind()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1", help="'all' = reachable from other devices")
    ap.add_argument("--port", type=int, default=8765)
    a = ap.parse_args()
    PANEL.connect()
    threading.Thread(target=PANEL.monitor, daemon=True).start()
    if a.host in ("all", "::", "0.0.0.0"):
        server = DualStackServer(("::", a.port), Handler)
        print(f"XIANG robot panel: open http://{socket.gethostname()}.local:{a.port}", flush=True)
    else:
        server = ThreadingHTTPServer((a.host, a.port), Handler)
        print(f"XIANG robot panel: open http://localhost:{a.port}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
