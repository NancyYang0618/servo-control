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

import gait
import json
import socket
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from robot import find_port, HERE, JOINT_ORDER, LEG_NAMES, LEGS, BusError, Robot, pretty, save_homes, save_limit

POLL_S = 1.0
POSES_FILE = HERE / "poses.json"


def load_poses():
    """{name: {joint: degrees from home}}. Degrees from home, so poses survive a new home."""
    try:
        return json.loads(POSES_FILE.read_text())
    except FileNotFoundError:
        return {}
    except json.JSONDecodeError:
        POSES_FILE.rename(POSES_FILE.with_suffix(".broken.json"))
        return {}


def store_poses(poses):
    tmp = POSES_FILE.with_suffix(".tmp")
    tmp.write_text(json.dumps(poses, indent=1) + "\n")
    tmp.replace(POSES_FILE)


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
        self.poses = load_poses()
        self.seq = dict(running=False, pose=None, step=0, steps=0, loop=0, loops=0)

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
        if ("USB" in str(e) or "BusLinker" in str(e)) and self.robot:
            self.robot.bus.close()          # free the dead port so the new one can be opened
            self.robot = None
            self.event("warn", "AUTO_WAIT", "Lost the BusLinker: the panel will reconnect by itself as soon as "
                       "it comes back.", "If it does not within 10 s, unplug the BusLinker USB, wait 3 s and plug "
                       "it back in.", every_s=5)

    # ---------------------------------------------------------------- connection
    def connect(self, auto=False):
        with self.lock:
            if self.robot:
                self.robot.bus.close()
            self.robot = None
            try:
                self.robot = Robot(log=lambda m: self.event("info", "LOG", m))
            except BusError as e:
                self.event("error", "NO_BUS", str(e), e.hint, every_s=30 if auto else 0)
                return False
            self.event("info", "CONNECTED", ("Reconnected automatically" if auto else "Connected") +
                       f" to the BusLinker on {self.robot.bus.port}.")
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
        if not any(self.state[j]["ok"] for j in JOINT_ORDER):
            import os
            if not os.path.exists(r.bus.port) or find_port() != r.bus.port:
                # the BusLinker re-appeared under a new name (USB dropped and came back): start over
                self.bus_error(BusError(f"Lost the USB link to the BusLinker ({r.bus.port} is gone)."), "reading")

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
            elif s["forced"]:
                out.append(("info", f"{name} is force-unlocked: its straight position is past the servo's 240 deg end, "
                            "so it cannot be powered straight; it works from about 46 deg bent onward.",
                            "Re-mounting its discs near 120 deg would give it the full range back."))
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
        if j and not self.cfg["servos"][j]["holdable"] and not self.robot.free:
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
                lo, hi = r.limits(j)
                if r.free:
                    self.event("warn", f"CLAMP_{j}", f"{pretty(j).capitalize()}: the servo itself only turns 0 to "
                               f"240 deg ({lo - s['home']:+.0f} to {hi - s['home']:+.0f} from home); limited.",
                               every_s=3)
                else:
                    self.event("warn", f"CLAMP_{j}", f"{pretty(j).capitalize()}: {target - s['home']:+.1f} deg from "
                               f"home is outside its safe range, limited to {a - s['home']:+.1f}.",
                               f"Safe range {lo - s['home']:+.0f} to {hi - s['home']:+.0f} deg from home. Turn on "
                               "Limit-test mode to go further and find the real limit.", every_s=3)
            try:
                now = r.bus.angle(sid)
                forced = self.cfg["servos"][j]["forced"]
                if not 0 <= now <= 240 and not forced:
                    self.event("error", f"OUT_{j}", f"{pretty(j).capitalize()} reads {now:.1f} deg, outside 0 to 240: "
                               "switching it on would make it jump.", "Move it by hand into range first.")
                    return
                if not 0 <= now <= 240:
                    speed = min(speed, 20.0)            # force-unlocked joint swinging into range: go gently
                    self.event("warn", f"SWING_{j}", f"{pretty(j).capitalize()} starts past the servo's 240 deg end, "
                               f"so it swings about {abs(now - a):.0f} deg into range, slowly.", every_s=5)
                elif not r.bus.motor_on(sid):
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
            h = self.cfg["servos"][j]["home"]
            if self.robot.free:
                self.event("warn", f"LAG_{j}", f"{pretty(j).capitalize()} is BLOCKED at {ang - h:+.1f} deg from home "
                           f"(asked for {target - h:+.1f}). That is probably its limit.",
                           "Back it off 2 to 5 deg now (a blocked motor heats up fast), then press Mark limit here.")
            else:
                self.event("warn", f"LAG_{j}", f"{pretty(j).capitalize()} stopped {lag:.1f} deg short "
                           f"({ang - h:+.1f} instead of {target - h:+.1f} from home).",
                           "Something may block the joint (a cable, a part touching), or the supply is sagging.")

    def nudge(self, j, delta):
        st = self.state[j]
        base = st["target"] if st["target"] is not None and st["motor"] else st["angle"]
        if base is None:
            return
        self.move(j, base + delta, speed=30.0 if self.robot and self.robot.free else 60.0)

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

    # ---------------------------------------------------------------- limit testing
    def set_free(self, on):
        if not self.robot:
            return
        self.robot.free = bool(on)
        if on:
            self.event("warn", "FREE_ON", "Limit-test mode ON: the safe ranges are switched off. Only the servo's "
                       "own 0 to 240 deg range applies.",
                       "Use small steps near a limit. If a joint stops short it is blocked: back it off at once, "
                       "then press Mark limit here. Press Motors OFF if anything goes wrong.")
        else:
            self.event("info", "FREE_OFF", "Limit-test mode OFF: moves are kept inside the safe ranges again "
                       "(your marked limits, minus a small margin).")

    def goto(self, j, deg_from_home):
        if not self.cfg:
            return
        self.move(j, self.cfg["servos"][j]["home"] + float(deg_from_home), speed=30.0)

    def mark_limit(self, j):
        """Save the joint's current angle as its limit on that side of home."""
        if not self.robot:
            return
        s = self.cfg["servos"][j]
        try:
            a = self.robot.bus.angle(s["id"])
        except BusError as e:
            self.bus_error(e, "marking a limit")
            return
        if not 0.5 <= a <= 239.5:
            self.event("warn", "LIMIT_BAD", f"{pretty(j).capitalize()} is at the servo's own end ({a:.1f} deg), "
                       "not at a limit of the leg: not saved.",
                       "The servo cannot turn further this way anyway. Mark limits only where the leg itself stops.")
            return
        if s["forced"]:              # home is out of reach: decide the side by the middle of the reachable range
            side = "min" if a < (s["min"] + s["max"]) / 2 else "max"
        else:
            side = "min" if a < s["home"] else "max"
        other = s.get("limit_max" if side == "min" else "limit_min")
        if other is not None and (a >= other if side == "min" else a <= other):
            self.event("warn", "LIMIT_BAD", f"{pretty(j).capitalize()}: that would put its two limits the wrong way "
                       "round: not saved.", "Press clear limits for this joint and mark both ends again.")
            return
        save_limit(j, side, a)
        self.robot.reload()
        m = self.cfg.get("limit_margin_deg", 3.0)
        self.event("info", f"LIMIT_{j}", f"{pretty(j).capitalize()}: {'lower' if side == 'min' else 'upper'} limit "
                   f"saved at {a - s['home']:+.1f} deg from home (servo {a:.1f}).",
                   f"Normal moves now stop {m:.0f} deg before it.")

    def clear_limits(self, j):
        if not self.robot:
            return
        save_limit(j, "min", None)
        save_limit(j, "max", None)
        self.robot.reload()
        self.event("info", f"LIMIT_{j}", f"{pretty(j).capitalize()}: measured limits cleared, back to the default range.")

    # ---------------------------------------------------------------- walking
    def walk_plan(self, b):
        params = {k: b.get(k) for k in gait.DEFAULTS if b.get(k) not in (None, "")}
        cycles = max(1, min(20, int(b.get("cycles", 2))))
        try:
            w = gait.plan(params, cycles=cycles)
        except gait.GaitError as e:
            return None, [str(e)], cycles
        return w, gait.check(w, self.cfg["servos"]), cycles

    def walk_check(self, b):
        """Plan the walk and report, nothing moves."""
        if not self.cfg:
            return
        w, probs, cycles = self.walk_plan(b)
        if probs:
            self.event("warn", "WALK_CHECK", "This walk can't run: " + "; ".join(probs))
            return
        m = min(x for _, x in w["margins"])
        hip = max(max(abs(a), abs(c)) for j, (a, c) in w["extremes"].items() if j.endswith("hip"))
        knee = max(max(abs(a), abs(c)) for j, (a, c) in w["extremes"].items() if j.endswith("knee"))
        self.event("info", "WALK_CHECK", f"Walk OK: {cycles} cycle(s), {w['distance_mm']:.0f} mm in {w['seconds']:.0f} s "
                   f"({w['distance_mm'] / w['seconds'] / 10:.2f} cm/s). Balance margin at least {m:.0f} mm. "
                   f"Joints up to {hip:.0f} deg (hips) and {knee:.0f} deg (knees).")

    def _walk(self, b, ready_only=False):
        w, probs, cycles = self.walk_plan(b)
        if probs:
            self.event("error", "WALK_NO", "Walk not started: " + "; ".join(probs))
            return
        frames = [gait.to_servo(row, self.cfg["servos"]) for row in w["samples"]]
        if ready_only:
            ok, rows = self.robot.go_to(frames[0], speed=20, by_leg=True, label="Getting ready",
                                        progress=lambda m: self.event("info", "WALK", m))
            self.last_test = dict(kind="Walk: get ready", time=time.strftime("%H:%M:%S"), ok=ok, rows=rows)
            self.event("info" if ok else "warn", "WALK", "Ready to walk: standing in the start position."
                       if ok else "Could not reach the start position: see the results table.")
            return
        self.event("info", "WALK", f"Walking {cycles} cycle(s), {w['distance_mm']:.0f} mm, about {w['seconds']:.0f} s.")
        r = self.robot.play(frames, w["dt"], progress=lambda m: self.event("info", "WALK", m, every_s=2))
        if r.get("rows"):
            self.last_test = dict(kind="Walk: get ready", time=time.strftime("%H:%M:%S"), ok=False, rows=r["rows"])
        if r["ok"]:
            lag = max(r["worst"].values()) if r.get("worst") else 0
            self.event("info", "WALK_DONE", f"Walk finished: {w['distance_mm']:.0f} mm planned. Worst joint lag "
                       f"{lag:.1f} deg, lowest supply {r['vmin']:.2f} V.",
                       "Lag over about 8 deg means the motors could not keep up: lower the speed." if lag > 8 else "")
        else:
            self.event("warn", "WALK_STOP", f"Walk stopped: {r['why']}. Joints hold where they are.",
                       "Try a lower speed, raise the converter's CC (current limit), or add a big capacitor "
                       "(1000 to 2200 uF, 16 V+) across the BusLinker power screws. Best: power the Pi from "
                       "its own USB power bank." if r.get("power") else "")
        for j, a in frames[-1].items():
            self.state[j]["target"] = a

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

    def save_home(self, joints=None):
        joints = [j for j in (joints or JOINT_ORDER) if j in JOINT_ORDER]
        with self.lock:
            if not self.robot or self.busy:
                return
            angles = {}
            for j in joints:
                try:
                    angles[j] = self.robot.bus.angle(self.robot.sid(j))
                except BusError as e:
                    self.bus_error(e, "saving home")
                    self.event("error", "SAVE_FAIL", "Home NOT saved: a servo did not answer.")
                    return
            save_homes(angles)
            self.robot.reload()
            what = "all 8 servos" if len(joints) == len(JOINT_ORDER) else ", ".join(pretty(j) for j in joints)
            self.event("info", "SAVED", f"Saved the current angle as home for {what} (now 0 deg).",
                       ", ".join(f"{pretty(j)} {a:.1f}" for j, a in angles.items()))

    # ---------------------------------------------------------------- poses
    def _pose_name(self, name):
        name = str(name or "").strip()[:30]
        if not name:
            self.event("warn", "POSE_NAME", "Give the pose a name first.")
            return None
        return name

    def pose_save(self, name):
        """Save the robot's current angles (degrees from home) under this name."""
        name = self._pose_name(name)
        if not name or not self.robot:
            return
        with self.lock:
            pose = {}
            for j in JOINT_ORDER:
                s = self.cfg["servos"][j]
                if not s["holdable"]:
                    continue
                try:
                    pose[j] = round(self.robot.bus.angle(s["id"]) - s["home"], 1)
                except BusError as e:
                    self.bus_error(e, "saving the pose")
                    self.event("error", "POSE_FAIL", f"Pose '{name}' NOT saved: a servo did not answer.")
                    return
            new = name not in self.poses
            self.poses[name] = pose
            store_poses(self.poses)
        self.event("info", "POSE_SAVED", f"{'Saved new' if new else 'Updated'} pose '{name}' from the robot.",
                   ", ".join(f"{pretty(j)} {d:+.1f}" for j, d in pose.items()))

    def pose_create(self, name, values, replace=False):
        """Make a pose from typed numbers: values = {joint: degrees from home}. Nothing moves."""
        name = self._pose_name(name)
        if not name:
            return False
        if name in self.poses and not replace:
            self.event("warn", "POSE_EXISTS", f"A pose called '{name}' already exists.",
                       "Pick another name, or tick 'replace if it exists'.")
            return False
        pose, clipped = {}, []
        for j in JOINT_ORDER:
            if j not in values or values[j] in (None, ""):
                continue
            v = float(values[j])
            s = self.cfg["servos"][j] if self.cfg else None
            if s and not s["holdable"]:
                continue
            if s:
                lo, hi = s["min"] - s["home"], s["max"] - s["home"]
                if not lo <= v <= hi:
                    clipped.append(f"{pretty(j)} {v:+.1f} -> {max(lo, min(hi, v)):+.1f}")
                    v = max(lo, min(hi, v))
            pose[j] = round(v, 1)
        self.poses[name] = pose
        store_poses(self.poses)
        self.event("info", "POSE_SAVED", f"Saved pose '{name}' (nothing moved). Press Go to try it.",
                   ", ".join(f"{pretty(j)} {d:+.1f}" for j, d in pose.items()))
        if clipped:
            self.event("warn", "POSE_RANGE", "Some numbers were outside the safe range and were limited: "
                       + "; ".join(clipped), "Hips can go about +/-40 deg from home, knees +/-60.")
        return True

    def pose_set(self, name, joint, value):
        """Fine-tune one joint of a stored pose (degrees from home)."""
        if name not in self.poses or joint not in JOINT_ORDER:
            return
        s = self.cfg["servos"][joint] if self.cfg else None
        v = float(value)
        if s:
            lo, hi = s["min"] - s["home"], s["max"] - s["home"]
            if not lo <= v <= hi:
                self.event("warn", "POSE_RANGE", f"{pretty(joint).capitalize()}: {v:+.1f} deg is outside its safe "
                           f"range; stored as {max(lo, min(hi, v)):+.1f}.", f"Safe range {lo:+.0f} to {hi:+.0f} deg from home.")
                v = max(lo, min(hi, v))
        self.poses[name][joint] = round(v, 1)
        store_poses(self.poses)

    def pose_delete(self, name):
        if self.poses.pop(name, None) is not None:
            store_poses(self.poses)
            self.event("info", "POSE_DEL", f"Deleted pose '{name}'.")

    def pose_rename(self, old, new):
        new = self._pose_name(new)
        if not new or old not in self.poses or new == old:
            return
        if new in self.poses:
            self.event("warn", "POSE_EXISTS", f"A pose called '{new}' already exists.")
            return
        self.poses = {(new if k == old else k): v for k, v in self.poses.items()}
        store_poses(self.poses)

    def _go_pose(self, name, speed, by_leg):
        if name not in self.poses:
            self.event("error", "NO_POSE", f"No pose called '{name}'.")
            return False
        targets = self.robot.pose_targets(self.poses[name])
        ok, rows = self.robot.go_to(targets, speed=speed, by_leg=by_leg, label=f"Pose '{name}'",
                                    progress=lambda m: self.event("info", "POSE_GO", m))
        for r in rows:
            if r["result"] == "PASS":
                self.state[r["joint"]]["target"] = r["target"]
        self.last_test = dict(kind=f"Pose '{name}'", time=time.strftime("%H:%M:%S"), ok=ok, rows=rows)
        bad = [r for r in rows if r["result"] != "PASS"]
        if any(r["result"] == "STOPPED" for r in rows):
            self.event("info", "POSE_STOP", f"Stopped on the way to '{name}'. Joints hold where they are.")
        elif bad:
            self.event("warn", "POSE_BAD", f"Pose '{name}' reached with problems: "
                       f"{', '.join(pretty(r['joint']) for r in bad)}.", "See the results table.")
        else:
            worst = max((abs(r["end"] - r["target"]) for r in rows if "end" in r), default=0)
            self.event("info", "POSE_OK", f"Pose '{name}' reached (worst error {worst:.1f} deg).")
        return ok

    def _sequence(self, names, dwell, loops, speed, by_leg):
        q = self.seq
        q.update(running=True, step=0, steps=len(names), loop=0, loops=loops)
        try:
            for lp in range(loops):
                q["loop"] = lp + 1
                for i, n in enumerate(names):
                    if self.robot.stop.is_set():
                        return
                    q.update(step=i + 1, pose=n)
                    self._go_pose(n, speed, by_leg)
                    if self.robot.stop.is_set():
                        return
                    end = time.time() + dwell
                    while time.time() < end and not self.robot.stop.is_set():
                        time.sleep(0.05)
            self.event("info", "SEQ_DONE", f"Sequence finished: {' -> '.join(names)} x{loops}.")
        finally:
            q.update(running=False, pose=None)

    def start_sequence(self, names, dwell, loops, speed, by_leg):
        names = [n for n in names if n in self.poses]
        if len(names) < 1:
            self.event("warn", "SEQ_EMPTY", "Pick at least one saved pose for the sequence.")
            return
        loops = max(1, min(50, int(loops)))
        dwell = max(0.0, min(30.0, float(dwell)))
        self.run_routine("pose sequence", self._sequence, names, dwell, loops, speed, by_leg)

    # ---------------------------------------------------------------- page state
    def snapshot(self):
        c = self.cfg
        joints = {}
        for j in JOINT_ORDER:
            st = dict(self.state[j])
            if c:
                s = c["servos"][j]
                lo, hi = self.robot.limits(j) if self.robot else (s["min"], s["max"])
                st.update(id=s["id"], home=s["home"], min=lo, max=hi, holdable=s["holdable"],
                          safe_min=s["min"], safe_max=s["max"], limit_min=s.get("limit_min"),
                          limit_max=s.get("limit_max"))
                if st["angle"] is not None:
                    st["from_home"] = st["angle"] - s["home"]
            st["name"] = pretty(j)
            joints[j] = st
        vs = [s["volts"] for s in self.state.values() if s["volts"]]
        return dict(connected=bool(self.robot), port=self.robot.bus.port if self.robot else None,
                    busy=self.busy, joints=joints, legs=LEGS, leg_names=LEG_NAMES,
                    vmin=min(vs) if vs else None, problems=self.problems(),
                    poses=self.poses, seq=self.seq, free=bool(self.robot and self.robot.free),
                    last_test=self.last_test, events=list(self.events)[:80])

    def monitor(self):
        last_try = 0.0
        while True:
            time.sleep(POLL_S)
            if not self.robot and not self.busy and time.time() - last_try > 3.0:
                last_try = time.time()
                if find_port():                    # BusLinker is plugged in (again): reconnect by itself
                    time.sleep(1.0)                # let the USB device settle after it re-appears
                    self.connect(auto=True)
                continue
            if self.robot and not self.busy:
                try:
                    with self.lock:
                        self.refresh()
                except Exception as e:           # anything unexpected while reading: assume the USB link died
                    self.bus_error(BusError(f"Lost the USB link to the BusLinker ({e}).",
                                            "It should reconnect by itself in a few seconds."), "reading")


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
            elif p == "/api/free":
                P.set_free(b.get("on"))
            elif p == "/api/goto":
                P.goto(b["joint"], float(b["deg"]))
            elif p == "/api/limit/mark":
                P.mark_limit(b["joint"])
            elif p == "/api/limit/clear":
                P.clear_limits(b["joint"])
            elif p == "/api/walk/check":
                P.walk_check(b)
            elif p == "/api/walk/ready":
                P.run_routine("getting ready to walk", P._walk, b, True)
            elif p == "/api/walk":
                P.run_routine("walking", P._walk, b, False)
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
                P.save_home(b.get("joints"))
            elif p == "/api/pose/save":
                P.pose_save(b.get("name"))
            elif p == "/api/pose/create":
                if P.pose_create(b.get("name"), b.get("values", {}), bool(b.get("replace"))) and b.get("go"):
                    spd = max(5.0, min(120.0, float(b.get("speed", 30))))
                    nm = str(b["name"]).strip()[:30]
                    P.run_routine(f"pose '{nm}'", P._go_pose, nm, spd, bool(b.get("by_leg", True)))
            elif p == "/api/pose/set":
                P.pose_set(b["name"], b["joint"], b["value"])
            elif p == "/api/pose/delete":
                P.pose_delete(b["name"])
            elif p == "/api/pose/rename":
                P.pose_rename(b["name"], b.get("new"))
            elif p == "/api/pose/go":
                spd = max(5.0, min(120.0, float(b.get("speed", 30))))
                P.run_routine(f"pose '{b['name']}'", P._go_pose, b["name"], spd, bool(b.get("by_leg", True)))
            elif p == "/api/sequence":
                spd = max(5.0, min(120.0, float(b.get("speed", 30))))
                P.start_sequence(b.get("names", []), b.get("dwell", 1.0), b.get("loops", 1), spd,
                                 bool(b.get("by_leg", True)))
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
