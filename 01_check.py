"""
Step 2 of the guide: check that the Pi can talk to both motors. Nothing moves.

    python3 01_check.py

Prints each motor's ID, voltage, temperature and current angle, then blinks each
motor's LED so you can see which one is the hip and which is the knee.
"""
import time

import serial
from pylx16a.lx16a import LX16A, ServoTimeoutError, ServoChecksumError

import leg_config as C

try:
    LX16A.initialize(C.PORT, C.TIMEOUT_S)
except serial.SerialException as e:
    raise SystemExit(f"Cannot open {C.PORT}: {e}\nRun `ls /dev/ttyUSB*` and put the right port in leg_config.py.")

ok = True
for name, sid in (("hip", C.HIP_ID), ("knee", C.KNEE_ID)):
    try:
        s = LX16A(sid)
        v = s.get_vin() / 1000.0
        t = s.get_temp()
        a = s.get_physical_angle()
        print(f"{name:5s} ID {sid}:  {v:4.1f} V   {t:3d} C   angle {a:6.1f} deg   OK")
        if v < C.MIN_VOLTS:
            print(f"      WARNING: {v:.1f} V is low. Charge the battery / check the DC converter (6 to 8.4 V).")
        for _ in range(3):                      # blink so you can see which motor this is
            s.led_power_off(); time.sleep(0.25)
            s.led_power_on(); time.sleep(0.25)
    except (ServoTimeoutError, ServoChecksumError) as e:
        ok = False
        print(f"{name:5s} ID {sid}:  NOT RESPONDING ({type(e).__name__})")

if ok:
    print("\nBoth motors answer. If the LED that blinked first is NOT on the hip motor,"
          "\nswap HIP_ID and KNEE_ID in leg_config.py.")
else:
    print("\nFix this before moving on:"
          "\n - is the battery on and the DC converter set to about 7 V?"
          "\n - is each motor's ID set? (lx_setup.py scan, with one motor plugged in at a time)"
          "\n - two motors with the SAME ID give checksum errors")
