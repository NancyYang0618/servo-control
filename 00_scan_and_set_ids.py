"""
Scan the bus for LX-16A motors and (optionally) give one motor a new ID.

    python3 00_scan_and_set_ids.py                 # list every ID that answers
    python3 00_scan_and_set_ids.py set 1 2         # change the motor with ID 1 to ID 2

Rule for renaming: ONLY ONE MOTOR PLUGGED IN. New servos all ship as ID 1, so with two
plugged in a rename would change both. The script refuses to rename if it hears more than
one motor, or if two motors share an ID (they answer with garbled "checksum" replies).
"""
import sys
import time

import serial
from pylx16a.lx16a import LX16A, ServoTimeoutError, ServoChecksumError

import leg_config as C


def scan(max_id=253):
    found, clashes = [], []
    for i in range(0, max_id + 1):
        try:
            s = LX16A(i)
            found.append((i, s.get_physical_angle(), s.get_vin() / 1000.0))
        except ServoTimeoutError:
            pass
        except ServoChecksumError:
            clashes.append(i)
    return found, clashes


try:
    LX16A.initialize(C.PORT, 0.02)
except serial.SerialException as e:
    raise SystemExit(f"Cannot open {C.PORT}: {e}")

print("scanning IDs 0 to 253 (about 10 s)...")
found, clashes = scan()
for i, a, v in found:
    print(f"  ID {i:3d}   angle {a:6.1f} deg   {v:4.1f} V")
for i in clashes:
    print(f"  ID {i:3d}   GARBLED REPLY: probably two motors both set to ID {i}")
if not found and not clashes:
    print("  no motors answered: check the battery, the converter (about 7 V) and the cables")

if len(sys.argv) == 4 and sys.argv[1] == "set":
    old, new = int(sys.argv[2]), int(sys.argv[3])
    if clashes or len(found) != 1:
        raise SystemExit("\nNot renaming: unplug all motors except the one you want to rename, then run again.")
    if found[0][0] != old:
        raise SystemExit(f"\nNot renaming: the motor on the bus is ID {found[0][0]}, not {old}.")
    if not 0 <= new <= 253:
        raise SystemExit("New ID must be 0 to 253.")
    LX16A(old).set_id(new)
    time.sleep(0.3)
    s = LX16A(new)
    print(f"\nDone: motor is now ID {new} (reads back angle {s.get_physical_angle():.1f}).")
    for _ in range(3):
        s.led_power_off(); time.sleep(0.25); s.led_power_on(); time.sleep(0.25)
