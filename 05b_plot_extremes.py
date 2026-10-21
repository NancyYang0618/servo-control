"""
Draw the measured extreme positions as stick figures for the slides.

    python3 05b_plot_extremes.py          # reads logs/extremes_measured.json

Makes logs/extremes_measured.png: one panel per extreme, the leg drawn to scale
(thigh 85 mm, shin 82 mm + 10 mm foot offset) with the straight leg in grey behind it.
Front of the robot is drawn to the RIGHT, matching Nancy's view of the real leg.
Needs matplotlib.
"""
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import leg_config as C

HERE = Path(__file__).resolve().parent
data = json.loads((HERE / "logs" / "extremes_measured.json").read_text())
res = data["results"]


def leg_points(thigh_back_deg, shin_front_deg):
    """thigh_back: + swings the thigh left/back. shin_front: + swings the shin right/front
    relative to the thigh. Returns hip, knee, foot (x right = front, z up)."""
    a = math.radians(-thigh_back_deg)                  # thigh angle from straight down, + = front
    knee = (C.THIGH_MM * math.sin(a), -C.THIGH_MM * math.cos(a))
    b = a + math.radians(shin_front_deg)               # shin angle from straight down
    foot = (knee[0] + C.SHIN_FWD_MM * math.cos(b) + C.SHIN_DOWN_MM * math.sin(b),
            knee[1] + C.SHIN_FWD_MM * math.sin(b) - C.SHIN_DOWN_MM * math.cos(b))
    return [(0, 0), knee, foot]


fig, axes = plt.subplots(1, len(res), figsize=(3.2 * len(res), 4.4))
if len(res) == 1:
    axes = [axes]
for ax, r in zip(axes, res):
    d = r["deg_from_straight"]
    hip_joint = r["servo_id"] == C.HIP_ID
    servo_down = r["extreme_servo_deg"] < 120
    if hip_joint:
        tb, sf = (d if servo_down else -d), 0.0
    else:
        tb, sf = 0.0, (d if servo_down else -d)
    base = leg_points(0, 0)
    ax.plot(*zip(*base), "-o", color="0.8", lw=5, ms=6, zorder=1)
    pts = leg_points(tb, sf)
    ax.plot(*zip(*pts), "-o", color="#1f77b4" if hip_joint else "#d62728", lw=5, ms=7, zorder=2)
    ax.plot([0], [0], "s", color="k", ms=8)
    ax.set_aspect("equal")
    ax.set_xlim(-140, 140)
    ax.set_ylim(-185, 25)
    ax.axis("off")
    ax.set_title(f"{r['direction']}\n{d:.1f} deg from straight\n{r['outcome']}", fontsize=8.5)
    ax.text(125, -180, "front ->", ha="right", fontsize=7, color="0.4")
fig.suptitle(f"XIANG leg: measured extreme positions ({data['measured']})", fontsize=11)
fig.tight_layout()
out = HERE / "logs" / "extremes_measured.png"
fig.savefig(out, dpi=170)
print("saved", out)
