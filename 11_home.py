#!/usr/bin/env python3
"""
XIANG homing: bring every joint to its home angle (all legs straight).

    python3 11_home.py                 show what would move, ask, then home one leg at a time
    python3 11_home.py --leg FL        home only the front-left leg (FL, FR, RL, RR)
    python3 11_home.py --off           after homing, switch the motors off again (limp)
    python3 11_home.py --save-current  stand the robot straight by hand first: saves every
                                       servo's current angle as its new home (nothing moves)

How it stays safe:
  * one leg at a time, hip and knee together, at 30 deg/s (robot_config.json: home_speed_deg_s)
  * each motor is first told to hold exactly where it is, so switching it on never makes it jump
  * a joint whose reading or home is outside the servo's 0 to 240 range is left OFF and reported
  * after the move, each joint is read back; more than 3 deg short counts as a failure
  * Ctrl+C switches every motor off at once (limp): support the robot
"""
import argparse
import sys

from robot import JOINT_ORDER, LEGS, BusError, Robot, pretty, print_rows, save_homes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--leg", choices=LEGS, help="home only this leg")
    ap.add_argument("--off", action="store_true", help="switch motors off after homing")
    ap.add_argument("--yes", action="store_true", help="do not ask before moving")
    ap.add_argument("--save-current", action="store_true", help="save current angles as home")
    a = ap.parse_args()
    try:
        robot = Robot()
    except BusError as e:
        sys.exit(f"{e}\n  -> {e.hint}")
    joints = [j for j in JOINT_ORDER if not a.leg or j.startswith(a.leg)]
    try:
        print(f"  {'joint':<18}{'ID':>3}  {'from home':>9}  {'servo now':>9}  {'servo home':>10}")
        now = {}
        for j in joints:
            s = robot.servos[j]
            try:
                now[j] = robot.bus.angle(s["id"])
                rel = f"{now[j] - s['home']:+8.1f}°" if s["holdable"] else "   LOCKED"
                print(f"  {pretty(j):<18}{s['id']:>3}  {rel}  {now[j]:9.1f}  {s['home']:10.1f}")
            except BusError as e:
                print(f"  {pretty(j):<18}{s['id']:>3}  not answering ({e.hint})")

        if a.save_current:
            bad = [j for j in joints if j not in now]
            if bad:
                sys.exit("Not saved: some servos did not answer.")
            print("\nThis saves the angles in the 'now' column as the new home for these joints.")
            if a.yes or input("Type y to save: ").strip().lower() == "y":
                save_homes({j: now[j] for j in joints})
                print("Saved to robot_config.json.")
                outside = [pretty(j) for j in joints if not 0 <= now[j] <= 240]
                if outside:
                    print("Note: still outside 0 to 240 (cannot be held): " + ", ".join(outside))
            return

        biggest = max((abs(robot.servos[j]["home"] - now[j]) for j in now if robot.servos[j]["holdable"]),
                      default=0)
        print(f"\nHoming moves each leg in turn at {robot.cfg['home_speed_deg_s']} deg/s "
              f"(largest move {biggest:.1f} deg). Robot on its stand, feet free.")
        if not a.yes and input("Type y to start homing: ").strip().lower() != "y":
            print("Nothing moved.")
            return
        ok, rows = robot.home(joints, progress=print)
        print_rows(rows, "Homing result")
        if a.off:
            robot.all_off()
            print("Motors switched off (limp).")
        else:
            print("Motors hold the home position. Run with --off, or press Motors off in the panel, to relax them.")
        sys.exit(0 if ok else 1)
    except KeyboardInterrupt:
        robot.all_off()
        print("\nStopped: every motor switched off (limp). Support the robot.")
    finally:
        robot.bus.close()


if __name__ == "__main__":
    main()
