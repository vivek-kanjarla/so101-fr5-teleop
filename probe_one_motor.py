"""
probe_one_motor.py — plug in one motor at a time, this script identifies it.

Run with: ~/teleop_env/bin/python3 ~/teleop/probe_one_motor.py

Continuously pings IDs 1–20. As soon as a motor responds it prints its ID
and position, then waits for you to unplug/replug the next one.
Press Ctrl-C to quit when done.
"""

import sys
import time
import serial

sys.path.insert(0, "/home/slifold/teleop")
from config import SO101_PORT, SO101_BAUDRATE

MAX_ID    = 20
TIMEOUT_S = 0.04   # 40 ms per motor


def build_ping(mid: int) -> bytes:
    length   = 2
    checksum = (~(mid + length + 0x01)) & 0xFF
    return bytes([0xFF, 0xFF, mid, length, 0x01, checksum])


def build_read_pos(mid: int) -> bytes:
    REG, LEN = 56, 2
    instruction = 0x02
    length      = 4
    cs          = (~(mid + length + instruction + REG + LEN)) & 0xFF
    return bytes([0xFF, 0xFF, mid, length, instruction, REG, LEN, cs])


def read_pos(ser: serial.Serial, mid: int) -> float | None:
    ser.reset_input_buffer()
    ser.write(build_read_pos(mid))
    resp = ser.read(8)
    if len(resp) >= 7 and resp[0] == 0xFF and resp[1] == 0xFF and resp[2] == mid:
        import struct
        raw = struct.unpack_from("<H", resp, 5)[0]
        return (raw / 4096.0) * 360.0
    return None


def scan_once(ser: serial.Serial) -> list[int]:
    found = []
    for mid in range(1, MAX_ID + 1):
        ser.reset_input_buffer()
        ser.write(build_ping(mid))
        resp = ser.read(6)
        if resp and len(resp) >= 2 and resp[0] == 0xFF and resp[1] == 0xFF:
            found.append(mid)
    return found


def main():
    print("=" * 55)
    print("  SO-101 One-by-One Motor Probe")
    print("=" * 55)
    print(f"  Port : {SO101_PORT}  Baud : {SO101_BAUDRATE}")
    print()
    print("  Plug in ONE motor (data + power), wait for detection.")
    print("  Then unplug it and plug in the next one.")
    print("  Ctrl-C to finish.\n")

    try:
        ser = serial.Serial(SO101_PORT, SO101_BAUDRATE, timeout=TIMEOUT_S)
    except serial.SerialException as exc:
        print(f"  FAIL  Cannot open {SO101_PORT}: {exc}")
        sys.exit(1)

    known: set[int] = set()
    results: list[tuple[int, float]] = []
    spin = ["|", "/", "-", "\\"]
    tick = 0

    try:
        while True:
            current = set(scan_once(ser))

            new_ids = current - known
            gone_ids = known - current

            for mid in sorted(new_ids):
                pos = read_pos(ser, mid)
                pos_str = f"{pos:.1f}°" if pos is not None else "unknown"
                results.append((mid, pos if pos is not None else 0.0))
                print(f"\r  *** MOTOR FOUND  ID={mid}  position={pos_str} ***")
                print(f"      Unplug this motor and plug in the next one.\n")

            for mid in sorted(gone_ids):
                print(f"\r  ID {mid} disconnected.\n")

            known = current

            tick += 1
            sys.stdout.write(f"\r  Scanning... {spin[tick % 4]}  ({len(known)} connected)")
            sys.stdout.flush()
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\n" + "=" * 55)
        print(f"  Done. {len(results)} motor(s) identified:\n")

        joint_names = [
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll",
            "gripper",
        ]

        for i, (mid, pos) in enumerate(results):
            name = joint_names[i] if i < len(joint_names) else f"joint_{i+1}"
            print(f"  ID {mid}  →  {name}  (position when probed: {pos:.1f}°)")

        print()
        print("  # Paste into ~/teleop/config.py → SO101_MOTORS:")
        print("  SO101_MOTORS = {")
        for i, (mid, _) in enumerate(results):
            name = joint_names[i] if i < len(joint_names) else f"joint_{i+1}"
            suffix = "  # gripper — excluded by default" if name == "gripper" else ""
            print(f'      "{name}": {mid},{suffix}')
        print("  }")
        print("=" * 55)

    finally:
        ser.close()


if __name__ == "__main__":
    main()
