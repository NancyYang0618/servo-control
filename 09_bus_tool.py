#!/usr/bin/env python3
"""
XIANG bus tool: scan the servo bus, blink a servo's LED, and renumber servos.

It talks to the LX-16A servos with raw read commands only. It NEVER switches a motor on and
never sends a move, so it is safe to run while every servo still has the factory ID 1.

    python3 09_bus_tool.py scan              list every ID that answers (0 to 253)
    python3 09_bus_tool.py blink 3           flash the LED of servo 3 so you can see which one it is
    python3 09_bus_tool.py setid 1 5         change servo 1 into servo 5 (only one servo may have ID 1!)
    python3 09_bus_tool.py read 1 2 3        read angle, voltage and temperature of these servos

Why duplicate IDs matter: every servo with the same ID answers at the same time, the replies
collide on the wire and come back garbled ("bad checksum"). The scan reports that as
"SHARED by several servos".
"""
import sys
import time

import serial

from leg_config import PORT

ID_WRITE, ID_READ, VIN_READ, POS_READ, TEMP_READ, LED_WRITE, TORQUE_READ = 13, 14, 27, 28, 26, 33, 32


def checksum(body):
    return (~sum(body)) & 0xFF


def send(bus, sid, cmd, params=()):
    body = [sid, len(params) + 3, cmd, *params]
    bus.write(bytes([0x55, 0x55, *body, checksum(body)]))


def ask(bus, sid, cmd, nbytes, tries=2):
    """Send a read command. Returns (status, data): status is 'ok', 'none' or 'garbled'."""
    status, data = "none", None
    for _ in range(tries):
        bus.reset_input_buffer()
        send(bus, sid, cmd)
        raw = bus.read(nbytes + 6)
        if not raw:
            continue
        ok = (len(raw) == nbytes + 6 and raw[0] == 0x55 and raw[1] == 0x55 and raw[2] == sid
              and raw[4] == cmd and checksum(list(raw[2:-1])) == raw[-1])
        if ok:
            return "ok", list(raw[5:-1])
        status = "garbled"
        time.sleep(0.02)
        bus.reset_input_buffer()
    return status, data


def s16(lo, hi):
    v = lo | (hi << 8)
    return v - 65536 if v > 32767 else v


def read_servo(bus, sid):
    st, pos = ask(bus, sid, POS_READ, 2)
    if st != "ok":
        return st, None
    angle = s16(*pos) * 0.24
    vin = ask(bus, sid, VIN_READ, 2)[1]
    temp = ask(bus, sid, TEMP_READ, 1)[1]
    torque = ask(bus, sid, TORQUE_READ, 1)[1]
    return "ok", {
        "angle": angle,
        "volts": (vin[0] | vin[1] << 8) / 1000 if vin else None,
        "temp": temp[0] if temp else None,
        "torque_on": bool(torque[0]) if torque else None,
    }


def describe(sid, st, info):
    if st == "ok":
        v = f"{info['volts']:.2f} V" if info["volts"] is not None else "? V"
        t = f"{info['temp']} C" if info["temp"] is not None else "? C"
        tq = {True: "motor ON", False: "motor off", None: "?"}[info["torque_on"]]
        return f"  ID {sid:3d}: OK        angle {info['angle']:6.1f} deg   {v}   {t}   {tq}"
    if st == "garbled":
        return (f"  ID {sid:3d}: SHARED by several servos (their replies collide). "
                f"Renumber them one at a time.")
    return f"  ID {sid:3d}: no answer"


def open_bus():
    try:
        return serial.Serial(PORT, 115200, timeout=0.05)
    except (serial.SerialException, OSError) as e:
        sys.exit(f"Cannot open the BusLinker on {PORT}: {e}\n"
                 f"Is its USB cable in a Pi USB port? Try:  ls /dev/ttyUSB*")


def cmd_scan(bus):
    print(f"Scanning IDs 0 to 253 on {PORT} (about 15 s). Nothing will move.")
    found = []
    for sid in range(254):
        st, info = read_servo(bus, sid)
        if st != "none":
            found.append((sid, st, info))
            print(describe(sid, st, info), flush=True)
    clean = [f for f in found if f[1] == "ok"]
    shared = [f for f in found if f[1] == "garbled"]
    print()
    print(f"Answering IDs: {len(found)}   clean: {[f[0] for f in clean]}   shared: {[f[0] for f in shared]}")
    if shared:
        print("At least one ID is used by more than one servo, so the exact servo count can't be read.")
        print("Renumber by connecting servos one at a time (see GUIDE.md, 'Numbering the 8 servos').")
    volts = [f[2]["volts"] for f in clean if f[2]["volts"]]
    if volts and min(volts) < 6.0:
        print(f"WARNING: supply is {min(volts):.2f} V. Servos need 6 to 8.4 V; raise the converter (CV).")


def cmd_blink(bus, sid, seconds=4.0):
    st, _ = ask(bus, sid, POS_READ, 2)
    if st == "none":
        sys.exit(f"Servo {sid} does not answer.")
    print(f"Flashing the LED of servo {sid} for {seconds:.0f} s. Watch which servo blinks.")
    end = time.time() + seconds
    on = True
    while time.time() < end:
        send(bus, sid, LED_WRITE, [0 if on else 1])   # 0 = LED on, 1 = LED off
        on = not on
        time.sleep(0.25)
    send(bus, sid, LED_WRITE, [1])


def cmd_setid(bus, old, new):
    if not (0 <= new <= 253):
        sys.exit("New ID must be 0 to 253.")
    st, _ = ask(bus, old, POS_READ, 2)
    if st == "garbled":
        sys.exit(f"More than one servo has ID {old}. Unplug all but one of them, then try again.")
    if st == "none":
        sys.exit(f"No servo answers as ID {old}.")
    st_new, _ = ask(bus, new, POS_READ, 2)
    if st_new != "none":
        sys.exit(f"ID {new} is already taken by another servo on the bus. Pick a free ID.")
    send(bus, old, ID_WRITE, [new])
    time.sleep(0.2)
    st, info = read_servo(bus, new)
    if st == "ok":
        print(f"Done: servo {old} is now servo {new}.")
        print(describe(new, st, info))
    else:
        sys.exit(f"Sent the change, but servo {new} does not answer yet. Run 'scan' to check.")


def cmd_read(bus, ids):
    for sid in ids:
        st, info = read_servo(bus, sid)
        print(describe(sid, st, info))


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ("scan", "blink", "setid", "read"):
        sys.exit(__doc__)
    bus = open_bus()
    c, args = sys.argv[1], [int(a) for a in sys.argv[2:]]
    if c == "scan":
        cmd_scan(bus)
    elif c == "blink" and len(args) == 1:
        cmd_blink(bus, args[0])
    elif c == "setid" and len(args) == 2:
        cmd_setid(bus, *args)
    elif c == "read" and args:
        cmd_read(bus, args)
    else:
        sys.exit(__doc__)


if __name__ == "__main__":
    main()
