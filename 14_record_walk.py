#!/usr/bin/env python3
"""
Record a walk: play the elephant walk and read ALL 8 servos back every frame, so the planned and the
measured motor angles can be plotted against time.

    python3 14_record_walk.py                 2 cycles at 1.0x, saves logs/walk_record_<date>.csv
    python3 14_record_walk.py --speed 0.6 --cycles 3

Stop the panel first (it owns the USB port):  sudo systemctl stop xiang-panel
Angles in the CSV are degrees from home in the walking convention (+ hip = foot toward the head,
+ knee = shin toward the head). Ctrl+C switches every motor off.
"""
import argparse
import csv
import time

import gait
from robot import JOINT_ORDER, LOG_DIR, BusError, Robot, pi_power_low


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--cycles", type=int, default=2)
    a = ap.parse_args()
    r = Robot()
    sv = r.servos
    w = gait.plan_elephant(dict(speed=a.speed), cycles=a.cycles)
    probs = gait.check(w, sv)
    if probs:
        raise SystemExit("Walk not started: " + "; ".join(probs))
    frames = [gait.to_servo(row, sv) for row in w["samples"]]
    walk_deg = lambda j, servo: (servo - sv[j]["home"]) * sv[j]["sign"]
    try:
        ok, _ = r.go_to(frames[0], speed=20, by_leg=True, progress=print, label="Getting ready")
        if not ok:
            raise SystemExit("Could not reach the start position.")
        time.sleep(0.5)
        dt, ms = w["dt"], int(w["dt"] * 1000) + 30
        log, t0 = [], time.time()
        t_next = t0
        for i, fr in enumerate(frames):
            for j, ang in fr.items():
                r.bus.move(r.sid(j), r.clamp(j, ang), ms)
            t_cmd = time.time() - t0
            meas = {}
            for j in JOINT_ORDER:
                try:
                    meas[j] = walk_deg(j, r.bus.angle(r.sid(j)))
                except BusError:
                    meas[j] = None
            volts = None
            if i % 5 == 0:
                try:
                    volts = r.bus.volts(r.sid("FL_hip"))
                except BusError:
                    pass
            log.append((t_cmd, {j: walk_deg(j, fr[j]) for j in JOINT_ORDER}, meas, volts,
                        pi_power_low() if i % 5 == 0 else None))
            t_next += dt
            time.sleep(max(0.0, t_next - time.time()))
        print(f"Walk recorded: {len(log)} frames in {time.time() - t0:.1f} s")
    except KeyboardInterrupt:
        r.all_off()
        print("Stopped: motors off.")
        return
    finally:
        r.bus.close()
    LOG_DIR.mkdir(exist_ok=True)
    f = LOG_DIR / f"walk_record_{time.strftime('%Y%m%d_%H%M%S')}_{a.speed:g}x.csv"
    with open(f, "w", newline="") as fh:
        wr = csv.writer(fh)
        wr.writerow(["time_s"] + [f"{j}_planned" for j in JOINT_ORDER] + [f"{j}_measured" for j in JOINT_ORDER]
                    + ["supply_V", "pi_power_low"])
        for t, cmd, meas, v, low in log:
            wr.writerow([f"{t:.3f}"] + [f"{cmd[j]:.2f}" for j in JOINT_ORDER]
                        + ["" if meas[j] is None else f"{meas[j]:.2f}" for j in JOINT_ORDER]
                        + ["" if v is None else f"{v:.2f}", "" if low is None else int(low)])
    print("saved", f)


if __name__ == "__main__":
    main()
