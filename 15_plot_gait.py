"""Plot the planned elephant-walk motor angles over time: python3 15_plot_gait.py <out folder> [speed]"""
import sys, csv
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import gait
out, speed = sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
w = gait.plan_elephant(dict(speed=speed), cycles=2)
dt, rows = w["dt"], w["samples"]
t = [i * dt for i in range(len(rows))]
p = w["params"]; T = p["period"] / p["speed"]; beta = p["beta"]
LEGS = ["FL", "FR", "RL", "RR"]
NAME = {"FL": "front left", "FR": "front right", "RL": "rear left", "RR": "rear right"}
COL = {"FL": "#2a78d6", "FR": "#eb6834", "RL": "#1baf7a", "RR": "#eda100"}
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#e6e5e0", "#fcfcfb"
off = gait._elephant_offsets(p["lag"])
tag = f"{speed:g}x"
with open(f"{out}/elephant_walk_joint_angles_{tag}.csv", "w", newline="") as f:
    wr = csv.writer(f)
    wr.writerow(["time_s"] + [f"{l}_{j}_deg_from_home" for l in LEGS for j in ("hip", "knee")])
    for ti, r in zip(t, rows):
        wr.writerow([f"{ti:.2f}"] + [f"{r[l][k]:.2f}" for l in LEGS for k in (0, 1)])
plt.rcParams.update({"font.family": "sans-serif", "font.size": 11, "axes.edgecolor": GRID,
                     "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2})
fig, axes = plt.subplots(3, 1, figsize=(11, 8.6), sharex=True,
                         gridspec_kw=dict(height_ratios=[3, 3, 1.25], hspace=0.16))
fig.patch.set_facecolor(SURF)
fig.subplots_adjust(top=0.84, left=0.12, right=0.95)
for ax, k, title in ((axes[0], 0, "Hip angle (+ = leg swings toward the head)"),
                     (axes[1], 1, "Knee angle (+ = shin swings toward the head; bumps = foot lifted)")):
    ax.set_facecolor(SURF)
    for l in LEGS:
        ys = [r[l][k] for r in rows]
        ax.plot(t, ys, color=COL[l], lw=2, solid_capstyle="round")
        if k == 0:
            ax.annotate(l, (t[-1], ys[-1]), xytext=(6, 0), textcoords="offset points", va="center",
                        fontsize=10, color=INK2)
        else:   # label each knee bump with its leg
            for i in range(1, len(ys) - 1):
                if ys[i] > 20 and ys[i] >= ys[i - 1] and ys[i] >= ys[i + 1]:
                    ax.annotate(l, (t[i], ys[i]), xytext=(0, 5), textcoords="offset points",
                                ha="center", fontsize=9.5, color=INK2)
    ax.axhline(0, color=INK2, lw=0.8, alpha=0.5)
    ax.grid(axis="y", color=GRID, lw=0.8); ax.set_axisbelow(True)
    for s in ("top", "right"): ax.spines[s].set_visible(False)
    ax.set_ylabel("degrees from home")
    ax.set_title(title, loc="left", fontsize=11.5, color=INK, pad=6)
axes[1].set_ylim(-3, 52)
ax = axes[2]; ax.set_facecolor(SURF)
for row, l in enumerate(reversed(LEGS)):
    on = None
    for i, ti in enumerate(t + [t[-1] + dt]):
        down = ((ti / T) - off[l]) % 1.0 < beta
        if down and on is None: on = ti
        if (not down or i == len(t)) and on is not None:
            ax.add_patch(plt.Rectangle((on, row - 0.32), ti - on, 0.64, color=COL[l], lw=0)); on = None
ax.set_yticks(range(4)); ax.set_yticklabels([NAME[l] for l in reversed(LEGS)], fontsize=10)
ax.set_ylim(-0.6, 3.6); ax.set_xlim(0, t[-1] + 0.5)
for s in ("top", "right", "left"): ax.spines[s].set_visible(False)
ax.tick_params(axis="y", length=0)
ax.set_title("Foot on the ground (gaps = foot in the air). Order: rear left, front left, rear right, front right",
             loc="left", fontsize=11.5, color=INK, pad=6)
ax.set_xlabel("time (s)")
fig.text(0.12, 0.965, f"XIANG elephant walk: planned motor angles over time ({tag}, {T:g} s per cycle)",
         fontsize=14, color=INK, fontweight="bold")
fig.text(0.12, 0.935, f"Duty factor {beta:.2f}, stride {p['stride']:.0f} mm, foot lift {p['lift']:.0f} mm, body rock "
         f"{p['rock_rear']:.0f} mm forward / {p['rock_front']:.0f} mm back. 0 = leg straight.",
         fontsize=10, color=INK2)
fig.legend([Line2D([], [], color=COL[l], lw=2.5) for l in LEGS], [f"{l} {NAME[l]}" for l in LEGS],
           ncol=4, frameon=False, loc="upper left", bbox_to_anchor=(0.115, 0.92), fontsize=10)
fig.savefig(f"{out}/elephant_walk_joint_angles_{tag}.png", dpi=200, facecolor=SURF)
print("saved", tag)
