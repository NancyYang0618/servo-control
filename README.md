# servo-control

Python control code for one leg of **XIANG**, a 3D printed quadruped elephant robot
(Tufts ME193 Printable Robotics, Fall 2026). Each leg has two Hiwonder **LX-16A** serial bus
servos (hip and knee), driven through the **BusLinker** board from a Mac or a Raspberry Pi,
using the [PyLX-16A](https://github.com/ethanlipson/PyLX-16A) library.

## Full robot: 4 legs, 8 servos

| ID | Joint | | ID | Joint |
|---|---|---|---|---|
| 1 | front-left hip | | 5 | rear-left hip |
| 2 | front-left knee | | 6 | rear-left knee |
| 3 | front-right hip | | 7 | rear-right hip |
| 4 | front-right knee | | 8 | rear-right knee |

| File | What it does |
|---|---|
| `12_robot_panel.py` + `robot_panel.html` | **Robot control panel** (starts at boot): every joint with live angle, distance from home, motor on/off, voltage and temperature; slider, nudge and "find" (LED flash) per joint; Boot test, Home all / Home leg, Stop, Motors OFF; plain-language errors |
| `10_boot_test.py` | Power-on self test. Stage 1 reads every servo (answers, 6 to 8.4 V, temperature, angle readable, home holdable); `--motion` adds stage 2, a 4 degree wiggle per servo, one at a time |
| `11_home.py` | Homing: one leg at a time at 30 deg/s to the saved home angles. `--save-current` saves the present pose as home |
| `09_bus_tool.py` | Scan the bus, flash a servo's LED, renumber servos. Read only, never switches a motor on |
| `robot.py` | Shared core: raw LX-16A protocol (no motor switches on until a move is asked for), boot test and homing |
| `robot_config.json` | IDs, home angles, safe swing around home, voltage and temperature limits |

Every first move holds the joint exactly where it is before moving it, so switching a motor on
never makes it jump. A joint whose home is outside the servo's 0 to 240 degree range is locked
(its motor stays off) until it is re-mounted and home is saved again.

## One-leg test scripts (Step 4)

The scripts below were written for a single leg (hip ID 1, knee ID 2). On the full robot, IDs
1 and 2 are the front-left leg; use the robot panel instead.

## What's here

| File | What it does |
|---|---|
| `08_control_panel.py` + `control_panel.html` | **Browser control panel**: sliders and nudge buttons per joint, motion patterns, a live leg drawing, and plain-language feedback on every error |
| `leg_config.py` | All settings: port, motor IDs, speed, safe range, leg geometry |
| `limits.json` | This leg's straight position, joint directions and safe range |
| `leg_common.py` | Shared `Leg` class: safe moves, read-back after every move, disconnect detection and recovery, logging |
| `00_scan_and_set_ids.py` | Scan the bus for motor IDs and rename a motor (one motor plugged in at a time) |
| `01_check.py` | Check both motors answer: voltage, temperature, angle, LED blink |
| `02_jog_and_measure.py` | Measure extremes by moving the limp leg by hand |
| `07_find_extremes.py` | Measure extremes with the motors (creep until blocked), with photo holds |
| `03_motions.py` | Motion patterns: swing, curl, wave, squat, walk, step, crouch, circle, kick |
| `04_disconnect_test.py` | Demonstrates catching an unplugged motor and recovering |
| `06_test_all_patterns.py` | Run every pattern and print tracking error and temperature |
| `05_plot_log.py`, `05b_plot_extremes.py` | Plots for slides from the logs |
| `xiang-panel.service` | systemd service that starts the control panel when the Pi boots |
| `GUIDE.md` | Step-by-step guide: wiring, setup on the Pi, and which rubric each step covers |

## Quick start

Hardware: battery -> DC converter set to **about 7 V** (motors accept 6 to 8.4 V) -> BusLinker
power terminal; BusLinker -> hip servo (ID 1) -> knee servo (ID 2); BusLinker USB -> computer.

```bash
python3 -m venv legenv
source legenv/bin/activate
pip3 install pyserial matplotlib
pip3 install --no-deps pylx16a      # skip its PyQt6 GUI extra (not needed, very large)
python3 08_control_panel.py
```

Then open http://localhost:8765. On a Raspberry Pi, start it with `--host all` and open
`http://xiang.local:8765` from your laptop. The BusLinker port is found automatically
(`/dev/cu.usbserial-*` on a Mac, `/dev/ttyUSB0` on the Pi); override with `XIANG_PORT=...`.

## Safety built in

- Every target is limited to the safe range in `limits.json`.
- A move command gets no reply, so the code reads both motors back after every move. A motor
  that stops answering is caught within one step; the pattern stops and the panel says which
  motor and what to check.
- Moves are blocked above 8.8 V and when a motor reaches 60 C.
- Command rate is about 4 per second, following the course lecture on servo tracking.

Drawn up with AI assistance (Claude), disclosed per the ME193 academic integrity policy.
