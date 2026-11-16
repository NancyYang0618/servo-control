"""
XIANG walking: a statically stable "intermittent crawl" for a 4-legged robot with 2 joints per leg.

Elephant style: a planted leg stays (almost) straight and only swings at the hip like a pendulum, so the
body bobs a few mm instead of the knees doing all the work. The knee bends only to lift a foot while it
swings forward. `crouch` is how much shorter than fully straight a planted leg is (mm).

One cycle (forward):
    lean forward -> lift rear-left, swing it forward, put it down -> lean back
    lean back    -> lift front-left, swing it forward, put it down -> lean forward
    body glides forward half a stride (all four feet on the ground)
    ... same for rear-right and front-right ... body glides forward again.

Why the lean: the legs can only swing forward and back (no sideways joint), and with a foot lifted the
robot stands on a triangle of 3 feet whose long edge runs right under the body's centre. Before a REAR
foot lifts, the planted feet push back so the body moves forward over the front feet; before a FRONT foot
lifts, the body moves back. check() measures how far inside the triangle the centre stays (the
"stability margin") for every lift and refuses a gait that would tip.

Coordinates per leg, relative to its hip axis: x forward (toward the head), z up, millimetres.
Joint angles from home: + hip = foot forward, + knee = shin swings forward. robot_config.json "sign"
maps these to each servo (left and right legs are mirror images).
"""
import math

THIGH = 85.0               # hip axis to knee axis
SHIN = 82.0                # knee axis to the foot, along the shin
FOOT_FWD = 10.0            # the foot sits this far forward of the shin line
LS = math.hypot(SHIN, FOOT_FWD)                  # effective shin, knee to foot contact
BETA = math.atan2(FOOT_FWD, SHIN)                # its angle from the shin line
HIP_X, TRACK_Y = 50.0, 46.0                      # hips 100 mm apart fore-aft, feet 92 mm apart side to side
HIPS = {"FL": (HIP_X, TRACK_Y), "FR": (HIP_X, -TRACK_Y), "RL": (-HIP_X, TRACK_Y), "RR": (-HIP_X, -TRACK_Y)}
LEGS = ("FL", "FR", "RL", "RR")

DEFAULTS = dict(stride=30.0, lift=15.0, lean=15.0, crouch=1.0, speed=0.5,
                t_lean=0.6, t_lift=0.35, t_swing=0.6, t_body=1.0, dt=0.1)


class GaitError(Exception):
    pass


def fk(hip_deg, knee_deg):
    """Joint angles (deg from home) -> foot (x, z) relative to the hip."""
    th = math.radians(hip_deg)
    ph = th + math.radians(knee_deg) + BETA
    kx, kz = THIGH * math.sin(th), -THIGH * math.cos(th)
    return kx + LS * math.sin(ph), kz - LS * math.cos(ph)


KNEE_BEND = "forward"       # which way knees bend to lift a foot: "forward" (shin swings forward) or "back"


def ik(x, z, bend=None):
    """Foot (x, z) relative to the hip -> (hip_deg, knee_deg) from home.
    bend "back": knee points forward, shin swings back (like a person). "forward": the opposite."""
    bend = bend or KNEE_BEND
    d = math.hypot(x, z)
    if d > THIGH + LS - 0.5:
        raise GaitError(f"foot ({x:.0f}, {z:.0f}) mm is out of the leg's reach ({THIGH + LS:.0f} mm)")
    if d < abs(THIGH - LS) + 1:
        raise GaitError(f"foot ({x:.0f}, {z:.0f}) mm is too close to the hip")
    alpha = math.atan2(x, -z)                                    # hip -> foot direction, from straight down
    gamma = math.acos((THIGH ** 2 + d ** 2 - LS ** 2) / (2 * THIGH * d))
    th = alpha + gamma if bend == "back" else alpha - gamma
    kx, kz = THIGH * math.sin(th), -THIGH * math.cos(th)
    ph = math.atan2(x - kx, -(z - kz))                           # knee -> foot direction
    return math.degrees(th), math.degrees(ph - BETA - th)


def margin(feet_x, lifted):
    """Stability margin (mm) with `lifted` in the air: distance from the body centre to the nearest edge
    of the triangle of the other three feet. feet_x: {leg: x relative to its NEUTRAL foot spot}."""
    pts = [(HIPS[l][0] + FOOT_FWD + feet_x[l], HIPS[l][1]) for l in LEGS if l != lifted]
    best = float("inf")
    for i in range(3):
        (x1, y1), (x2, y2), (x3, y3) = pts[i], pts[(i + 1) % 3], pts[(i + 2) % 3]
        cr = lambda px, py: (x2 - x1) * (py - y1) - (y2 - y1) * (px - x1)
        side = 1 if cr(x3, y3) > 0 else -1
        best = min(best, side * cr(0.0, 0.0) / math.hypot(x2 - x1, y2 - y1))
    return best


def keyframes(p, cycles, _offset=None):
    """List of (duration_s, {leg: (x_rel, lifted_mm)}) keyframes. x_rel is relative to the leg's
    neutral foot spot (straight under the hip); lifted_mm is how high the foot is."""
    S, L, k = p["stride"], p["lean"], 1.0 / p["speed"]
    if _offset is None:              # first pass: find the range each foot covers, then centre it on its hip
        fr, _ = keyframes(p, 1, _offset={l: 0.0 for l in LEGS})
        _offset = {}
        for l in LEGS:
            xs = [pos[l][0] for _, pos in fr]
            _offset[l] = -(max(xs) + min(xs)) / 2
    start = {"FL": 0.0, "FR": S / 2, "RL": -S / 2, "RR": 0.0}
    f = {l: start[l] + _offset[l] for l in LEGS}
    z = {l: 0.0 for l in LEGS}
    frames = [(0.0, {l: (f[l], 0.0) for l in LEGS})]
    margins = []

    def snap(t):
        frames.append((t * k, {l: (f[l], z[l]) for l in LEGS}))

    for _ in range(cycles):
        for kind, leg in (("swing", "RL"), ("swing", "FL"), ("body", None),
                          ("swing", "RR"), ("swing", "FR"), ("body", None)):
            if kind == "body":
                for l in LEGS:
                    f[l] -= S / 2
                snap(p["t_body"])
                continue
            lean = L if leg[0] == "R" else -L        # rear lift: body forward; front lift: body back
            for l in LEGS:
                f[l] -= lean
            snap(p["t_lean"])
            margins.append((leg, margin(f, leg)))
            z[leg] = p["lift"]
            snap(p["t_lift"])
            f[leg] += S
            snap(p["t_swing"])
            z[leg] = 0.0
            snap(p["t_lift"])
            for l in LEGS:
                f[l] += lean
            snap(p["t_lean"])
    return frames, margins


def interpolate(frames, dt):
    """Turn keyframes into evenly spaced samples (every dt s), smooth start and stop of each segment."""
    out = [frames[0][1]]
    for (_, a), (dur, b) in zip(frames, frames[1:]):
        n = max(1, round(dur / dt))
        for i in range(1, n + 1):
            s = i / n
            s = s * s * (3 - 2 * s)                  # smoothstep: no jerk at the ends
            out.append({l: (a[l][0] + (b[l][0] - a[l][0]) * s, a[l][1] + (b[l][1] - a[l][1]) * s) for l in LEGS})
    return out


def plan(params=None, cycles=2):
    """Build the walk. Returns dict(samples=[{leg: (hip_deg, knee_deg)}], dt, margins, extremes).
    Raises GaitError if any foot is out of reach."""
    p = dict(DEFAULTS)
    p.update({k: float(v) for k, v in (params or {}).items() if k in DEFAULTS and v is not None})
    frames, margins = keyframes(p, int(cycles))
    samples = []
    ext = {}
    reach = THIGH + LS - max(0.6, p["crouch"])        # planted-leg length, hip to foot
    for pos in interpolate(frames, p["dt"]):
        row = {}
        for l, (x, lift) in pos.items():
            fx = FOOT_FWD + x
            if abs(fx) >= reach - 1:
                raise GaitError(f"{l}: foot {fx:.0f} mm from under the hip is out of reach; shorten the stride or lean")
            h, kn = ik(fx, -math.sqrt(reach ** 2 - fx ** 2) + lift)
            row[l] = (h, kn)
            for name, v in (("hip", h), ("knee", kn)):
                lo, hi = ext.get((l, name), (v, v))
                ext[(l, name)] = (min(lo, v), max(hi, v))
        samples.append(row)
    return dict(samples=samples, dt=p["dt"], margins=margins, params=p,
                extremes={f"{l}_{n}": v for (l, n), v in ext.items()},
                seconds=len(samples) * p["dt"],
                distance_mm=p["stride"] * int(cycles))


def check(walk, servos, min_margin=5.0):
    """Problems that should stop the walk from starting (list of strings; empty = OK).
    servos: robot_config servos with home/min/max/sign."""
    probs = []
    worst = min((m for _, m in walk["margins"]), default=99)
    if worst < min_margin:
        probs.append(f"balance margin only {worst:.1f} mm when a foot lifts (need {min_margin:.0f}). "
                     "Increase the lean or shorten the stride.")
    for j, (lo, hi) in walk["extremes"].items():
        s = servos[j]
        if "sign" not in s:
            probs.append(f"{j}: direction unknown (no 'sign' in robot_config.json)")
            continue
        if not s["holdable"]:
            probs.append(f"{j}: locked")
            continue
        for v in (lo, hi):
            servo = s["home"] + s["sign"] * v
            if not s["min"] - 0.5 <= servo <= s["max"] + 0.5:
                probs.append(f"{j}: needs {v:+.1f} deg from home, outside its safe range "
                             f"({(s['min'] - s['home']) * s['sign']:+.0f} to {(s['max'] - s['home']) * s['sign']:+.0f} "
                             "in walking direction)")
                break
    return probs


def to_servo(walk_row, servos):
    """{leg: (hip, knee)} -> {joint: servo angle}."""
    out = {}
    for l, (h, k) in walk_row.items():
        for name, v in (("hip", h), ("knee", k)):
            j = f"{l}_{name}"
            out[j] = servos[j]["home"] + servos[j]["sign"] * v
    return out


if __name__ == "__main__":
    # self-check: forward and inverse kinematics agree, and print the default walk's numbers
    for x, z in ((10, -167.0), (40, -155), (-30, -150), (50, -135)):
        if math.hypot(x, z) < THIGH + LS - 0.5:
            h, k = ik(x, z)
            fx, fz = fk(h, k)
            assert abs(fx - x) < 0.01 and abs(fz - z) < 0.01, (x, z, fx, fz)
    w = plan(cycles=1)
    print(f"one cycle: {w['seconds']:.1f} s, {w['distance_mm']:.0f} mm -> {w['distance_mm'] / w['seconds'] / 10:.2f} cm/s")
    print("margins (mm):", [(l, round(m, 1)) for l, m in w["margins"]])
    for j, (lo, hi) in sorted(w["extremes"].items()):
        print(f"  {j:8s} {lo:+6.1f} .. {hi:+6.1f} deg from home")
