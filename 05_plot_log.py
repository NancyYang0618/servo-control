"""
Step 6 of the guide: turn a log into a slide-ready plot. Run it on the Pi or on your Mac.

    python3 05_plot_log.py logs/20261020_141500_walk.csv

Makes a PNG next to the log with:
  left:  commanded vs measured hip and knee angle over time (does the motor keep up?)
  right: the path the foot actually traced (from the measured angles)
Disconnect / reconnect events are marked with red lines.
Needs matplotlib:  pip3 install matplotlib
"""
import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

path = Path(sys.argv[1])
rows = list(csv.DictReader(open(path)))
data = [r for r in rows if r["hip_meas"]]
t = [float(r["t_s"]) for r in data]
f = lambda k: [float(r[k]) for r in data]

fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4.2), gridspec_kw=dict(width_ratios=[1.6, 1]))
a1.plot(t, f("hip_cmd"), "--", color="#1f77b4", label="hip commanded")
a1.plot(t, f("hip_meas"), "-", color="#1f77b4", label="hip measured")
a1.plot(t, f("knee_cmd"), "--", color="#d62728", label="knee commanded")
a1.plot(t, f("knee_meas"), "-", color="#d62728", label="knee measured")
for r in rows:
    if r["event"].startswith(("DISCONNECT", "RECONNECTED")):
        a1.axvline(float(r["t_s"]), color="red", lw=0.8)
        a1.text(float(r["t_s"]), a1.get_ylim()[1], r["event"].split(":")[0], color="red", fontsize=7, rotation=90, va="top")
a1.set_xlabel("time (s)")
a1.set_ylabel("joint angle (deg)   0 = leg straight")
a1.legend(fontsize=8)
a1.grid(alpha=0.3)

a2.plot(f("foot_x_mm"), f("foot_z_mm"), "-o", ms=2, color="k")
a2.plot([0], [0], "s", color="gray")
a2.text(2, 2, "hip axis", fontsize=8, color="gray")
a2.set_aspect("equal")
a2.set_xlabel("foot forward of hip (mm)")
a2.set_ylabel("foot height vs hip (mm)")
a2.grid(alpha=0.3)
a2.set_title("measured foot path", fontsize=9)

pattern = data[0]["pattern"] if data else path.stem
fig.suptitle(f"XIANG leg: {pattern}", fontsize=11)
fig.tight_layout()
out = path.with_suffix(".png")
fig.savefig(out, dpi=160)
print("saved", out)
