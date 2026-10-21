"""
Measure the leg's extreme positions with the motors themselves (rubric: "extreme leg
positions tested and measured"). Use this when the joints are too stiff to turn by hand.

    python3 07_find_extremes.py

For each joint and direction (the other joint held straight), the motor creeps outward
2 degrees at a time. After every nudge it reads the joint back:
  - if the joint falls more than CONTACT_ERR behind the command, something is blocking it:
    that angle is recorded as the contact point and the joint backs off 3 degrees
  - if nothing blocks it, it stops at the CAD-checked cap and records "free to cap"
At each extreme it holds HOLD_S seconds so you can photograph / measure with a protractor.

Writes logs/extremes_measured.csv, logs/extremes_measured.md and logs/extremes_measured.json
(05b_plot_extremes.py turns the json into a pose figure).
Directions are named on Nancy's view of the leg: front of the robot to the RIGHT.
"""
import csv
import json
import time

import serial
from pylx16a.lx16a import LX16A, ServoError

import leg_config as C
from leg_common import LOG_DIR

STRAIGHT = 120.0          # servo angle with the leg straight (alignment grooves)
STEP_DEG = 2.0            # creep step
STEP_S = 0.25             # time per step
CONTACT_ERR = 4.0         # lag that counts as "something is blocking the joint"
BACK_OFF = 3.0
HOLD_S = 3.0

# (label, servo id, direction of servo angle, cap in degrees from straight)
# servo angle DOWN = thigh left/back and shin right/front (Nancy's pose check, 2026-10-21)
SWEEPS = [
    ("thigh LEFT / BACK", C.HIP_ID, -1, 45.0),
    ("thigh RIGHT / FRONT", C.HIP_ID, +1, 45.0),
    ("shin RIGHT / FRONT (knee bends forward)", C.KNEE_ID, -1, 95.0),
    ("shin LEFT / BACK (knee bends backward)", C.KNEE_ID, +1, 95.0),
]

LX16A.initialize(C.PORT, C.TIMEOUT_S)
servos = {C.HIP_ID: LX16A(C.HIP_ID), C.KNEE_ID: LX16A(C.KNEE_ID)}
for s in servos.values():
    s.enable_torque()


def both_straight(ms=1500):
    for s in servos.values():
        s.move(STRAIGHT, time=ms)
    time.sleep(ms / 1000 + 0.3)


results, vmin, tmax = [], 99.0, 0
try:
    both_straight()
    for label, sid, sign, cap in SWEEPS:
        s = servos[sid]
        print(f"\n-> {label}: creeping up to {cap:.0f} deg", flush=True)
        cmd, outcome, meas = STRAIGHT, "free to CAD cap", STRAIGHT
        while abs(cmd - STRAIGHT) < cap:
            cmd = STRAIGHT + sign * min(cap, abs(cmd - STRAIGHT) + STEP_DEG)
            s.move(cmd, time=int(STEP_S * 1000))
            time.sleep(STEP_S + 0.05)
            meas = s.get_physical_angle()
            vmin = min(vmin, s.get_vin() / 1000)
            if abs(cmd - meas) > CONTACT_ERR:
                outcome = "CONTACT (joint blocked)"
                s.move(meas - sign * BACK_OFF, time=300)
                time.sleep(0.4)
                break
        held = s.get_physical_angle()
        tmax = max(tmax, s.get_temp())
        joint = (meas - STRAIGHT)
        print(f"   {outcome}: reached {meas:.1f} servo deg = {abs(joint):.1f} deg from straight. "
              f"Holding {HOLD_S:.0f} s for your photo...", flush=True)
        time.sleep(HOLD_S)
        results.append(dict(direction=label, servo_id=sid, outcome=outcome,
                            extreme_servo_deg=round(meas, 1), deg_from_straight=round(abs(joint), 1),
                            hold_servo_deg=round(held, 1), cap_deg=cap))
        both_straight(1200)
except (ServoError, serial.SerialException) as e:
    print(f"\nSTOPPED: lost contact with a motor ({type(e).__name__}). Check power and USB, then run again.")
finally:
    try:
        both_straight()
    except Exception:
        pass

LOG_DIR.mkdir(exist_ok=True)
stamp = time.strftime("%Y-%m-%d %H:%M")
with open(LOG_DIR / "extremes_measured.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(results[0].keys()) if results else ["direction"])
    w.writeheader()
    w.writerows(results)
(LOG_DIR / "extremes_measured.json").write_text(json.dumps(dict(measured=stamp, results=results), indent=2))
lines = [f"Extreme positions measured {stamp} (motor-driven creep, {STEP_DEG:.0f} deg steps, "
         f"contact = lag over {CONTACT_ERR:.0f} deg). Lowest supply {vmin:.2f} V, max temp {tmax} C.", "",
         "| Direction (front = right) | Result | Servo angle | Degrees from straight | CAD expectation |",
         "|---|---|---|---|---|"]
for r in results:
    exp = "+/-40 deg free" if r["servo_id"] == C.HIP_ID else "up to 110 deg free"
    lines.append(f"| {r['direction']} | {r['outcome']} | {r['extreme_servo_deg']} | {r['deg_from_straight']} | {exp} |")
(LOG_DIR / "extremes_measured.md").write_text("\n".join(lines) + "\n")
print("\n" + "\n".join(lines))
