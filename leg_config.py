"""
XIANG Step 4: settings shared by every script. Edit this file, not the scripts.

Angles here are SERVO angles, 0 to 240 degrees, as the LX-16A reports them.
120 is the midpoint the alignment grooves were set to, so 120 / 120 = leg straight.
"""

# ---- Connection ----
import glob as _glob
import os as _os

# BusLinker port: /dev/ttyUSB0 on the Pi, /dev/cu.usbserial-* on a Mac (found automatically).
# Override with:  XIANG_PORT=/dev/ttyUSB1 python3 ...
PORT = _os.environ.get("XIANG_PORT") or next(iter(sorted(_glob.glob("/dev/cu.usbserial*"))), "/dev/ttyUSB0")
TIMEOUT_S = 0.05           # how long to wait for a servo to answer before calling it "not responding"

# ---- Servo IDs (set with lx_setup.py, one servo plugged in at a time) ----
HIP_ID = 1
KNEE_ID = 2

# ---- Leg straight (alignment grooves lined up) ----
HIP_STRAIGHT = 120.0
KNEE_STRAIGHT = 120.0

# ---- Direction: +1 or -1 so that a POSITIVE joint angle means ----
#   hip:  foot swings FORWARD
#   knee: knee BENDS (foot moves backward and up)
# Step 3 of the guide tells you how to find these. Leave as +1 until then.
HIP_SIGN = +1
KNEE_SIGN = +1

# ---- Safe range in SERVO degrees ----
# Start with these conservative values. Step 3 (02_jog_and_measure.py) measures the real
# extremes and writes them to limits.json, which every script loads automatically and then
# shrinks by SAFETY_MARGIN so the leg never runs into its own hard stops.
HIP_MIN, HIP_MAX = 80.0, 160.0       # +/- 40 deg about straight (CAD-checked range)
KNEE_MIN, KNEE_MAX = 60.0, 180.0     # +/- 60 deg to start; CAD allows +/- 110
SAFETY_MARGIN = 5.0                  # degrees kept away from each measured extreme

# ---- Motion timing ----
STEP_S = 0.25              # one new target every 0.25 s (4 Hz), per the lecture slide on tracking
SPEED_SCALE = 1.0          # 0.5 = everything half speed. Use 0.5 the first time you run a pattern.

# ---- Leg geometry for foot-position maths (XIANG-007, mm) ----
THIGH_MM = 85.0            # hip axis to knee axis
SHIN_DOWN_MM = 82.0        # knee axis to foot contact, along the shin (straight down when straight)
SHIN_FWD_MM = 10.0         # foot sits this far forward of the shin line
HIP_HEIGHT_MM = 167.0      # hip axis above the ground when the leg is straight

# ---- Health limits ----
MIN_VOLTS = 6.0            # warn below this (servo works 6 to 8.4 V)
MAX_TEMP_C = 60            # stop the pattern above this
