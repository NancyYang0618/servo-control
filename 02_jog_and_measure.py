"""
Step 3 of the guide: measure the leg's extreme positions (rubric: "extreme leg positions
tested and measured") and find which way each motor turns.

    python3 02_jog_and_measure.py

Part A (motors limp): you move the leg BY HAND to each extreme while the screen shows the
live angle. Press Enter to record. This is the safe way to find the hard stops: the motor
cannot drive the leg into anything.
Part B (motors on, optional): the leg visits each extreme by itself, 5 degrees inside the
measured stop, and holds so you can photograph it and measure with a protractor and ruler.

Writes limits.json (used by every other script) and logs/extremes_table.md for your slides.
"""
import json
import threading
import time

from pylx16a.lx16a import LX16A

import leg_config as C
from leg_common import LIMITS_FILE, LOG_DIR, Leg, MotorDisconnected, foot_position

LX16A.initialize(C.PORT, C.TIMEOUT_S)
hip, knee = LX16A(C.HIP_ID), LX16A(C.KNEE_ID)


def live_read(prompt, servo_watch):
    """Show both angles live until Enter is pressed; return (hip, knee) servo angles."""
    stop = threading.Event()

    def show():
        while not stop.is_set():
            try:
                print(f"   hip {hip.get_physical_angle():6.1f}   knee {knee.get_physical_angle():6.1f}     ", end="\r")
            except Exception:
                print("   (motor not answering)                      ", end="\r")
            time.sleep(0.2)

    print("\n" + prompt)
    th = threading.Thread(target=show, daemon=True)
    th.start()
    input()
    stop.set()
    th.join()
    return hip.get_physical_angle(), knee.get_physical_angle()


print("PART A. Both motors will go LIMP. Hold the leg so it does not drop.")
input("Press Enter when you are holding it...")
hip.disable_torque()
knee.disable_torque()

h0, k0 = live_read("1/5  Put the leg STRAIGHT: line up both alignment grooves with the thigh split gap.\n"
                   "     Press Enter.", None)
hf, _ = live_read("2/5  Swing the HIP so the foot goes FORWARD as far as it can (until it touches\n"
                  "     something or stops). Keep the knee straight. Press Enter.", None)
hb, _ = live_read("3/5  Swing the HIP BACKWARD as far as it can. Press Enter.", None)
_, kb = live_read("4/5  Put the hip back to straight. BEND the KNEE (foot moves backward/up) as far\n"
                  "     as it can. Press Enter.", None)
_, ko = live_read("5/5  Now push the KNEE the OTHER way (past straight) as far as it can. Press Enter.", None)

hip_sign = 1 if hf > h0 else -1
knee_sign = 1 if kb > k0 else -1
m = dict(hip_straight=round(h0, 1), knee_straight=round(k0, 1),
         hip_sign=hip_sign, knee_sign=knee_sign,
         hip_min=round(min(hf, hb), 1), hip_max=round(max(hf, hb), 1),
         knee_min=round(min(kb, ko), 1), knee_max=round(max(kb, ko), 1),
         measured_on=time.strftime("%Y-%m-%d %H:%M"))
LIMITS_FILE.write_text(json.dumps(m, indent=2))

j = lambda a, st, sg: (a - st) * sg
rows = [
    ("Leg straight", 0.0, 0.0),
    ("Hip full forward", j(hf, h0, hip_sign), 0.0),
    ("Hip full backward", j(hb, h0, hip_sign), 0.0),
    ("Knee full bend", 0.0, j(kb, k0, knee_sign)),
    ("Knee other way", 0.0, j(ko, k0, knee_sign)),
]
LOG_DIR.mkdir(exist_ok=True)
lines = ["| Position | Hip (deg) | Knee (deg) | Foot forward (mm) | Foot height above straight (mm) |",
         "|---|---|---|---|---|"]
for name, h, k in rows:
    fx, fz = foot_position(h, k)
    lines.append(f"| {name} | {h:+.1f} | {k:+.1f} | {fx:+.0f} | {fz + C.HIP_HEIGHT_MM:+.0f} |")
table = "\n".join(lines)
(LOG_DIR / "extremes_table.md").write_text(
    f"Measured {m['measured_on']} (servo angles: hip {m['hip_min']} to {m['hip_max']}, "
    f"knee {m['knee_min']} to {m['knee_max']})\n\n" + table + "\n")
print("\n\nSaved limits.json and logs/extremes_table.md:\n")
print(table)
print(f"\nDirections found: HIP_SIGN = {hip_sign:+d}, KNEE_SIGN = {knee_sign:+d} (saved, no need to edit leg_config.py)")
print(f"CAD expected: hip +/-40 deg, knee up to 110 deg. Compare with your numbers above.")

# ---------------------------------------------------------------- part B
if input("\nPART B: let the motors visit each extreme by themselves? [y/N] ").strip().lower() != "y":
    raise SystemExit
leg = Leg(log_name="extremes")
for s in (leg.hip, leg.knee):
    s.enable_torque()
try:
    leg.go_straight(2.0, "extremes")
    for name, h, k in rows[1:]:
        print(f"\n-> {name}  (going 5 deg inside the stop)")
        hm, km = leg.move(h, k, 2.0, "extremes", name)
        fx, fz = foot_position(hm, km)
        print(f"   measured hip {hm:+.1f}, knee {km:+.1f}  ->  foot {fx:+.0f} mm forward, "
              f"{fz + C.HIP_HEIGHT_MM:+.0f} mm vs straight")
        input("   Photograph it / measure with a protractor, then press Enter...")
        leg.go_straight(1.5, "extremes")
except MotorDisconnected as e:
    leg.recover(e, "extremes")
finally:
    leg.close()
print("\nDone. The leg is straight and holding.")
