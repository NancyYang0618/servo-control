"""
Step 4 of the guide: different leg motion patterns (rubric: "different leg motion patterns
explored"). Every run is logged to logs/ so you can plot it later.

    python3 03_motions.py            # shows the menu
    python3 03_motions.py walk 3     # run the "walk" pattern 3 times

Patterns
  swing    hip pendulum, knee straight                (one joint)
  curl     knee bends and straightens, hip still      (one joint)
  wave     the lecture's sine waves, out of phase     (both joints, joint space)
  squat    foot stays under the hip, body height dips (both joints, foot space)
  walk     elephant walk: straight pillar leg on the ground, swings from the hip, small knee lift
  step     thigh left/back first, then shin right/front, then both return
  crouch   bent-knee walk (dog / bird style), for comparison
  circle   foot draws a circle                        (inverse kinematics showcase)
  kick     quick forward kick and return              (speed test)

Start every new pattern at half speed: set SPEED_SCALE = 0.5 in leg_config.py.
Ctrl+C at any time: the leg returns slowly to straight and holds.
"""
import math
import sys

import leg_config as C
from leg_common import Leg, MotorDisconnected, joint_angles_for_foot, run_pattern

N = 16                                   # targets per cycle -> 16 x 0.25 s = 4 s per cycle


def swing():
    return [(25 * math.sin(2 * math.pi * i / N), 0.0) for i in range(N)]


def curl():
    return [(0.0, 30 * (1 - math.cos(2 * math.pi * i / N))) for i in range(N)]   # 0 -> 60 -> 0


def wave():
    return [(20 * math.sin(2 * math.pi * i / N), 30 + 30 * math.cos(2 * math.pi * i / N)) for i in range(N)]


def foot_path(points):
    """Convert foot (x, height-above-ground-when-straight) points into joint targets."""
    out, prev = [], 0.0
    for x, h in points:
        sol = joint_angles_for_foot(x, h - C.HIP_HEIGHT_MM, prev)
        if sol is None:
            raise SystemExit(f"foot target ({x:.0f}, {h:.0f}) is out of reach; make the pattern smaller")
        out.append(sol)
        prev = sol[1]
    return out


def squat():
    # foot stays 10 mm forward (its natural spot), "height" rises = hip lowers toward the foot
    return foot_path([(10, 12 * (1 - math.cos(2 * math.pi * i / N))) for i in range(N)])   # dips 24 mm


WALK_HIP = 10.0        # hip swings +/- this many degrees: stride = 2 x 167 x sin(10) = 58 mm
WALK_SWING_HIP = 28.0  # swing step 1: how far the hip swings forward (deg)
WALK_SWING_KNEE = 45.0 # swing step 1: knee bend that lifts the foot clear (deg)
WALK_STANCE_PTS = 14   # targets on the ground  (14 x 0.25 s = 3.5 s)
WALK_SWING_PTS = 10    # targets in the air     (10 x 0.25 s = 2.5 s) -> 6 s per step, elephant pace


def walk():
    """Elephant walk: the leg is a straight pillar on the ground and swings from the hip.
    stance (60 %): knee straight, hip sweeps the whole leg from +WALK_HIP to -WALK_HIP
                   (the leg moves back relative to the body: that is what pushes the body forward)
    swing  (40 %): first the HIP swings forward (knee bends a little to lift the foot), then the
                   KNEE straightens so the foot reaches forward, then the foot plants."""
    n_st = WALK_STANCE_PTS
    pts = []
    for i in range(n_st):
        u = i / (n_st - 1)
        pts.append((WALK_HIP - 2 * WALK_HIP * u, 0.0))
    # swing in a clear order: 1) HIP forward (knee bends to lift the foot, so the knee leads forward)
    #                         2) KNEE forward (knee straightens, shin and foot swing out to reach)
    #                         3) plant, ready for the next stance
    keys = [(0.0, (-WALK_HIP, 0.0)),
            (0.15, (-WALK_HIP + 3.0, 22.0)),                    # unweight: small knee lift so the foot clears
            (0.45, (WALK_SWING_HIP, WALK_SWING_KNEE)),
            (0.85, (WALK_HIP + 4.0, 0.0)),
            (1.0, (WALK_HIP, 0.0))]
    n_sw = WALK_SWING_PTS
    for i in range(1, n_sw + 1):
        f = i / (n_sw + 1)
        for (f0, a), (f1, b) in zip(keys, keys[1:]):
            if f0 <= f <= f1:
                s = 0.5 - 0.5 * math.cos(math.pi * (f - f0) / (f1 - f0))   # smooth start and stop
                pts.append((a[0] + (b[0] - a[0]) * s, a[1] + (b[1] - a[1]) * s))
                break
    return pts


STEP_HIP = -15.0       # step: thigh swings left/back (negative hip = left/back on Nancy's view)
STEP_KNEE = -30.0      # step: then the shin swings right/front (negative knee = shin right/front)


def step():
    """Nancy's step, described on her view of the leg (front to the right):
    1) the thigh swings LEFT / BACK, 2) then the shin swings RIGHT / FRONT, 3) both return.
    With the restored limits.json (both signs +1): servo angle down = negative joint angle =
    thigh left/back and shin right/front (confirmed by Nancy's pose check, 2026-10-21)."""
    keys = [(0.0, (0.0, 0.0)),
            (0.30, (STEP_HIP, 0.0)),            # 1) hip first
            (0.60, (STEP_HIP, STEP_KNEE)),      # 2) then knee
            (0.70, (STEP_HIP, STEP_KNEE)),      #    short hold
            (1.0, (0.0, 0.0))]                  # 3) both return
    n = N
    out = []
    for i in range(n):
        f = i / n
        for (f0, a), (f1, b) in zip(keys, keys[1:]):
            if f0 <= f <= f1:
                s = 0.5 - 0.5 * math.cos(math.pi * (f - f0) / (f1 - f0))
                out.append((a[0] + (b[0] - a[0]) * s, a[1] + (b[1] - a[1]) * s))
                break
    return out


def crouch():
    # The first walk: a crouched, bent-knee gait (dog / bird style), kept for comparison.
    # stance: foot slides back along the ground 60 mm; swing: lifts 25 mm in a low arc.
    # Knee bent the whole time, so the knee juts forward. Not how elephants walk.
    stride, lift, x0, stand = 60.0, 25.0, -10.0, 8.0
    pts = []
    n_st = 10
    for i in range(n_st):                          # stance (60 % of the cycle, like a slow walk)
        pts.append((x0 + stride / 2 - stride * i / (n_st - 1), stand))
    n_sw = N - n_st
    for i in range(1, n_sw + 1):                    # swing
        u = i / (n_sw + 1)
        pts.append((x0 - stride / 2 + stride * u, stand + lift * math.sin(math.pi * u)))
    return foot_path(pts)


def circle():
    r, cx, ch = 15.0, -15.0, 20.0                  # sized to stay inside hip +/-36, knee 0..100
    return foot_path([(cx + r * math.cos(2 * math.pi * i / N), ch + r * math.sin(2 * math.pi * i / N)) for i in range(N)])


def kick():
    return [(0, 40), (-10, 60), (30, 10), (30, 0), (15, 0), (0, 0)]


PATTERNS = dict(swing=swing, curl=curl, wave=wave, squat=squat, walk=walk, step=step, crouch=crouch, circle=circle, kick=kick)


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in PATTERNS:
        print(__doc__)
        name = input("pattern name: ").strip()
        cycles = int(input("how many cycles [3]: ") or 3)
    else:
        name = sys.argv[1]
        cycles = int(sys.argv[2]) if len(sys.argv) > 2 else 3
    targets = PATTERNS[name]()
    print(f"\n{name}: {len(targets)} targets per cycle, {cycles} cycles, speed x{C.SPEED_SCALE}")
    leg = Leg(log_name=name)
    try:
        for s in (leg.hip, leg.knee):
            s.enable_torque()
        leg.go_straight(1.5, name)
        hip0, knee0 = targets[0]
        leg.move(hip0, knee0, 1.0, name, "move to start")        # ease in, no sudden jump
        ok = run_pattern(leg, name, targets, cycles)
        leg.go_straight(1.5, name)
        print("finished" if ok else "stopped early")
    except KeyboardInterrupt:
        print("\nCtrl+C: returning to straight.")
        try:
            leg.go_straight(1.5, name, "ctrl+c")
        except MotorDisconnected:
            pass
    except MotorDisconnected as e:
        leg.recover(e, name)
    finally:
        leg.close()
        print(f"log: {leg.log_path}")


if __name__ == "__main__":
    main()
