# servo-control

Python control code for one leg of **XIANG**, a 3D printed quadruped elephant robot
(Tufts ME193 Printable Robotics, Fall 2026). Each leg has two Hiwonder **LX-16A** serial bus
servos (hip and knee), driven through the **BusLinker** board from a Mac or a Raspberry Pi,
using the [PyLX-16A](https://github.com/ethanlipson/PyLX-16A) library.

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
| `GUIDE.md` | Step-by-step guide: wiring, setup on the Pi, and which rubric each step covers |

## Quick start

Hardware: battery -> DC converter set to **about 7 V** (motors accept 6 to 8.4 V) -> BusLinker
power terminal; BusLinker -> hip servo (ID 1) -> knee servo (ID 2); BusLinker USB -> computer.

```bash
python3 -m venv legenv
source legenv/bin/activate
pip3 install pylx16a pyserial matplotlib
python3 08_control_panel.py
```

Then open http://localhost:8765. On a Raspberry Pi, start it with `--host 0.0.0.0` and open
`http://<pi-address>:8765` from your laptop. The BusLinker port is found automatically
(`/dev/cu.usbserial-*` on a Mac, `/dev/ttyUSB0` on the Pi); override with `XIANG_PORT=...`.

## Safety built in

- Every target is limited to the safe range in `limits.json`.
- A move command gets no reply, so the code reads both motors back after every move. A motor
  that stops answering is caught within one step; the pattern stops and the panel says which
  motor and what to check.
- Moves are blocked above 8.8 V and when a motor reaches 60 C.
- Command rate is about 4 per second, following the course lecture on servo tracking.

Drawn up with AI assistance (Claude), disclosed per the ME193 academic integrity policy.
