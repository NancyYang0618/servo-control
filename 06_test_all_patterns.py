"""
Run every motion pattern back to back and print a summary (tracking error, temperature).

    python3 06_test_all_patterns.py            # all patterns, 2 cycles each, half speed
    python3 06_test_all_patterns.py 1.0 3      # full speed, 3 cycles each

Each pattern gets its own log in logs/. The leg returns to straight between patterns.
"""
import csv
import importlib
import sys
import time

import leg_config as C

C.SPEED_SCALE = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
CYCLES = int(sys.argv[2]) if len(sys.argv) > 2 else 2

from leg_common import Leg, MotorDisconnected, run_pattern   # noqa: E402

M = importlib.import_module("03_motions")
summary = []
for name in ("swing", "curl", "wave", "squat", "walk", "circle", "kick"):
    targets = M.PATTERNS[name]()
    leg = Leg(log_name=name, quiet=True)
    try:
        for s in (leg.hip, leg.knee):
            s.enable_torque()
        leg.go_straight(1.5, name)
        leg.move(*targets[0], 1.0, name, "move to start")
        t0 = time.time()
        ok = run_pattern(leg, name, targets, CYCLES)
        dt = time.time() - t0
        leg.go_straight(1.5, name)
        temps = [t for _, _, t in leg.health()]
    except MotorDisconnected as e:
        ok, dt, temps = leg.recover(e, name), 0, [0]
    finally:
        leg.close()
    rows = [r for r in csv.DictReader(open(leg.log_path)) if r["hip_meas"] and r["event"] == ""]
    err_h = max((abs(float(r["hip_cmd"]) - float(r["hip_meas"])) for r in rows), default=0)
    err_k = max((abs(float(r["knee_cmd"]) - float(r["knee_meas"])) for r in rows), default=0)
    clamped = sum(1 for r in csv.DictReader(open(leg.log_path)) if r["event"] == "clamped to safe range")
    summary.append((name, ok, dt, err_h, err_k, max(temps), clamped, leg.log_path.name))
    print(f"{name:7s} {'ok' if ok else 'STOPPED'}  {dt:5.1f} s  max error hip {err_h:4.1f} / knee {err_k:4.1f} deg  "
          f"{max(temps)} C  clamped steps {clamped}", flush=True)

print("\nall done, speed x", C.SPEED_SCALE)
