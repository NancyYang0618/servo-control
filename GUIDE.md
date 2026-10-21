# XIANG Step 4: driving the leg from the Raspberry Pi

Do the steps in order. Each one tells you what to run, what you should see, and what to
capture for the slides. Paste anything that looks wrong back to Claude.

| File | What it does |
|---|---|
| `leg_config.py` | All settings (port, motor IDs, speed, ranges). The only file you edit. |
| `leg_common.py` | The shared Leg code: safe moves, disconnect detection, logging. |
| `01_check.py` | Talks to both motors, nothing moves. |
| `02_jog_and_measure.py` | Measures the extreme positions and which way each motor turns. |
| `03_motions.py` | Eight motion patterns: swing, curl, wave, squat, walk, crouch, circle, kick. |
| `04_disconnect_test.py` | Shows the code catching an unplugged motor and recovering. |
| `05_plot_log.py` | Turns any log into a plot for the slides. |
| `08_control_panel.py` | **Browser control panel**: sliders, nudge buttons, patterns, live leg drawing, and plain-language error messages. Open http://localhost:8765 after starting it. On the Pi: `python3 08_control_panel.py --host 0.0.0.0`, then open http://PI-ADDRESS:8765 on your laptop. |

---

## Step 0. Set up the hardware safely

1. **Clamp the leg so it hangs in the air.** Bolt the hip clevis to a piece of the body plate
   (or any board) and clamp that to the table edge, so the foot can swing freely without
   hitting the table. Every pattern assumes the foot is off the ground.
2. Battery -> DC converter -> BusLinker power input. **Set the converter to about 7.0 V**
   with a multimeter before you connect the motors (6 to 8.4 V is allowed).
3. BusLinker -> hip servo -> knee servo (daisy chain, one cable between them).
4. BusLinker USB -> Raspberry Pi.

## Step 1. Copy the folder to the Pi and install the library

On your **Mac** (Terminal), copy this folder to the Pi. Replace `pi` and `raspberrypi.local`
with the username and hostname you use for SSH:

```bash
scp -r ~/Desktop/"3D Printed Robots"/Step4_leg_control pi@raspberrypi.local:~/
```

Then SSH into the Pi and set up a virtual environment (lecture 4, p.69 to 70):

```bash
ssh pi@raspberrypi.local
cd ~/Step4_leg_control
python3 --version
python3 -m venv legenv
source legenv/bin/activate
pip3 install pylx16a pyserial matplotlib
sudo usermod -aG dialout $USER
```

`python3 --version` must say **3.10 or newer** (pylx16a needs it). Raspberry Pi OS "Bookworm"
has 3.11. If yours is older, tell Claude before going further.

Log out and SSH back in once (that last line lets you use the USB port without `sudo`).
Every new session afterwards: `cd ~/Step4_leg_control && source legenv/bin/activate`.

Check the port name:

```bash
ls /dev/ttyUSB*
```

You should see `/dev/ttyUSB0`. If it says `ttyUSB1`, change `PORT` in `leg_config.py`.

## Step 2. Motor IDs and the first check

The code expects **hip = ID 1, knee = ID 2**. If you have not set the IDs yet, use the
course's `lx_setup.py` (github.com/RoboLabHub/LewanSoul) with **only one motor plugged in**
at a time:

```bash
python3 lx_setup.py scan
python3 lx_setup.py assign 1 2
```

(The second line changes a motor from ID 1 to ID 2. Two motors with the same ID give
checksum errors.)

Then, with both plugged in:

```bash
python3 01_check.py
```

**You should see** both motors "OK" with about 7 V, and each motor's LED blinks 3 times, hip
first. If the knee blinks first, swap `HIP_ID` and `KNEE_ID` in `leg_config.py`.

## Step 3. Measure the extreme positions (rubric: extreme leg positions tested and measured)

```bash
python3 02_jog_and_measure.py
```

- **Part A:** the motors go limp. **Hold the leg.** Follow the 5 prompts: leg straight
  (grooves lined up), hip fully forward, hip fully back, knee fully bent, knee the other way.
  Move gently by hand until the part touches something or stops. The live angle shows on screen.
- It saves `limits.json`, finds which way each motor turns, and writes
  `logs/extremes_table.md`: hip and knee angles plus how far the foot moved at each extreme.
  **Put this table on a slide**, next to the CAD's expected range (hip ±40, knee 0 to 110).
- **Part B (type y):** the leg visits each extreme by itself, 5 degrees inside the stop, and
  holds. **Photograph each pose and measure it with a protractor and a ruler.** Comparing
  your hand measurement with the motor's reading is a strong slide.
- **If something touches before the CAD said it would** (for example a cable or a cap edge),
  photograph it close up: that is a form/fit issue for the "issues identified and addressed"
  rubric. Tell Claude what touched and where.

## Step 4. Run the motion patterns (rubrics: different motion patterns, photo sequence)

First time for each pattern, set `SPEED_SCALE = 0.5` in `leg_config.py` (half speed).

```bash
python3 03_motions.py swing 3
python3 03_motions.py curl 3
python3 03_motions.py wave 3
python3 03_motions.py squat 3
python3 03_motions.py walk 4
python3 03_motions.py crouch 3
python3 03_motions.py circle 3
python3 03_motions.py kick 2
```

| Pattern | What to say on the slide |
|---|---|
| swing | hip only, pendulum ±25 deg |
| curl | knee only, 0 to 60 deg |
| wave | both joints, sine waves out of phase (the lecture example) |
| squat | foot stays put while the hip lowers 24 mm (both joints together) |
| walk | elephant walk: straight pillar leg on the ground swinging from the hip, 58 mm stride, foot lifted only 15 mm on a forward-only arch, 6 s per step |
| crouch | bent-knee walk (dog / bird style) for comparison: knee stays bent, knee juts forward |
| circle | the foot draws a 30 mm circle: inverse kinematics |
| kick | fastest motion, speed test |

**Film each one from the side** with your phone on something steady, leg against a plain
background, in landscape. The walk video is the best one to post on the discussion board.
Ctrl+C stops any pattern safely: the leg goes back to straight and holds.

## Step 5. Disconnect test (rubric: exception handling catches motor disconnect)

```bash
python3 04_disconnect_test.py
```

The leg swings gently. **Film the leg and the Pi screen together**, then unplug the knee
cable. The screen shows which motor stopped answering; plug it back in and the leg recovers
to straight and keeps going. Do it once more with the USB cable. Ctrl+C to finish.
For the slide: one frame before, one with the error message, one after recovery, plus the
`DISCONNECT` / `RECONNECTED` lines from the log.

## Step 6. Get the logs back to the Mac and make plots

On your **Mac**:

```bash
scp -r pi@raspberrypi.local:~/Step4_leg_control/logs ~/Desktop/"3D Printed Robots"/Step4_leg_control/
```

Then ask Claude to make the plots and pull frames out of your videos for the frame
sequence slide.

## Rubric map (one leg)

| Rubric | Covered by |
|---|---|
| Sequence of photos showing leg in motion (10) | frames from the walk video, plus the foot-path plot |
| Video on the discussion board + 3 comments (10) | walk video; screenshot your post and comments |
| Extreme positions tested and measured (10) | Step 3 table and photos |
| Form/fit issues identified and addressed (10) | anything found in Steps 3 to 5, plus the CAD fix |
| Components bolted with inserts (10) | close-up photos of each screw and insert |
| 3D-print quality, supports removed (10) | close-up photos |
| Different leg motion patterns (10) | Step 4, seven patterns |
| Leg modularity (10) | film removing one cap (2 screws) and swapping a servo |
| Two or more legs in tandem (10) | not possible with one leg |
| Cables routed properly (10) | photos of the slots and clips |
| Exception handling catches motor disconnect (10) | Step 5 |
