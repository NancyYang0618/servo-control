#!/usr/bin/env python3
"""
XIANG boot test (power-on self test) for all 8 servos.

    python3 10_boot_test.py            stage 1 only: talk to every servo, nothing moves
    python3 10_boot_test.py --motion   stage 1, then stage 2: each servo wiggles 4 deg and back,
                                       ONE at a time (asks you first)

Stage 1 checks, for every servo: it answers, supply 6 to 8.4 V, temperature, its angle is
readable (0 to 240), its saved home can be held, and how far it is from home.
Stage 2 checks the motor really moves and reaches its target (within 2.5 deg).
A log is saved in logs/boot_test_<date>.json. Ctrl+C at any time switches every motor off.
"""
import argparse
import sys

from robot import BusError, Robot, print_rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--motion", action="store_true", help="also wiggle each servo (asks first)")
    ap.add_argument("--yes", action="store_true", help="do not ask before moving")
    a = ap.parse_args()
    try:
        robot = Robot()
    except BusError as e:
        sys.exit(f"{e}\n  -> {e.hint}")
    try:
        ok, rows = robot.boot_test(motion=False)
        print_rows(rows, "Stage 1 (nothing moved)")
        if a.motion:
            ready = [r for r in rows if r["result"] == "PASS"]
            print(f"\nStage 2 will wiggle {len(ready)} servo(s) by 4 degrees, one at a time.")
            print("The robot should be on its stand with the feet free.")
            if not a.yes and input("Type y to start stage 2: ").strip().lower() != "y":
                print("Stage 2 skipped.")
            else:
                ok, rows = robot.boot_test(motion=True)
                print_rows(rows, "Stage 1 + stage 2")
        fails = [r for r in rows if r["result"] != "PASS"]
        print(f"\nBOOT TEST {'PASSED' if not fails else 'FOUND PROBLEMS'}: "
              f"{len(rows) - len(fails)} of {len(rows)} servos OK.")
        sys.exit(0 if not fails else 1)
    except KeyboardInterrupt:
        robot.all_off()
        print("\nStopped: every motor switched off (limp).")
    finally:
        robot.bus.close()


if __name__ == "__main__":
    main()
