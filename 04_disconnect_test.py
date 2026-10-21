"""
Step 5 of the guide: prove the code catches a motor disconnect (rubric: "exception
handling in code catches motor disconnect").

    python3 04_disconnect_test.py

The leg swings gently and forever. While it runs (and while you film the leg AND the
terminal), unplug the KNEE servo cable. Within a quarter of a second the code:
  1. notices the knee stopped answering (the read-back after each move times out)
  2. stops sending moves and prints which motor and what to check
  3. waits for you to plug it back in
  4. re-enables both motors, returns slowly to straight, and carries on swinging
Then try the same with the BusLinker USB cable. Ctrl+C to finish.

Every disconnect and reconnect is written into the log with a timestamp, so you can put
the log lines on a slide next to the video frames.
"""
import math

from leg_common import Leg, MotorDisconnected, run_pattern

N = 16
targets = [(15 * math.sin(2 * math.pi * i / N), 15 + 15 * math.cos(2 * math.pi * i / N)) for i in range(N)]

leg = Leg(log_name="disconnect_test")
try:
    for s in (leg.hip, leg.knee):
        s.enable_torque()
    leg.go_straight(1.5, "disconnect_test")
    print("\nSwinging. Unplug the knee cable now (Ctrl+C to stop).\n")
    while run_pattern(leg, "disconnect_test", targets, cycles=1):
        print("   cycle ok", end="\r")
except KeyboardInterrupt:
    print("\nCtrl+C: returning to straight.")
    try:
        leg.go_straight(1.5, "disconnect_test", "ctrl+c")
    except MotorDisconnected:
        pass
finally:
    leg.close()
    print(f"log: {leg.log_path}  (search it for DISCONNECT and RECONNECTED)")
