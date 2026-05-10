"""
scan_motor_ids.py — scan the SO-101 servo bus for all responding motor IDs.

Run with: ~/teleop_env/bin/python3 ~/teleop/scan_motor_ids.py

Sends a PING (instruction 0x01) to every ID from 1 to MAX_ID.
Any servo that responds is printed with its ID and current position.
Use the output to confirm/update SO101_MOTORS in config.py.
"""

import glob
import struct
import time
import sys

sys.path.insert(0, "/home/slifold/teleop")
from config import SO101_PORT, SO101_BAUDRATE

MAX_ID       = 20
REG_POS      = 56   # Present Position register (STS3215)
READ_LEN     = 2
TIMEOUT_S    = 0.05  # 50 ms per motor


def build_read_packet(motor_id: int) -> bytes:
    instruction = 0x02  # READ_DATA
    length      = 4     # LEN = num_params + 2
    cs_sum      = (motor_id + length + instruction + REG_POS + READ_LEN) & 0xFF
    checksum    = (~cs_sum) & 0xFF
    return bytes([0xFF, 0xFF, motor_id, length, instruction, REG_POS, READ_LEN, checksum])


def scan(port_path: str) -> list[tuple[int, int, float]]:
    import serial
    found = []
    try:
        ser = serial.Serial(port_path, SO101_BAUDRATE, timeout=TIMEOUT_S)
    except serial.SerialException as exc:
        print(f"  FAIL  Cannot open {port_path}: {exc}")
        print("         Check: cable plugged in? Run: sudo usermod -aG dialout $USER")
        return found

    print(f"  Port {port_path} opened at {SO101_BAUDRATE} baud")
    print(f"  Scanning IDs 1–{MAX_ID}...\n")

    for mid in range(1, MAX_ID + 1):
        ser.reset_input_buffer()
        ser.write(build_read_packet(mid))
        response = ser.read(8)

        if len(response) >= 7 and response[0] == 0xFF and response[1] == 0xFF and response[2] == mid:
            raw = struct.unpack_from("<H", response, 5)[0]
            deg = (raw / 4096.0) * 360.0
            found.append((mid, raw, deg))
            print(f"  ID {mid:>2}  FOUND   raw={raw:>4}  ({deg:>6.1f}°)")
        else:
            print(f"  ID {mid:>2}  .")

        time.sleep(0.01)

    ser.close()
    return found


if __name__ == "__main__":
    print("=" * 55)
    print("  SO-101 Motor ID Scanner")
    print("=" * 55)

    acm = glob.glob("/dev/ttyACM*")
    usb = glob.glob("/dev/ttyUSB*")
    ports = sorted(acm + usb)

    if not ports:
        print("  FAIL  No /dev/ttyACM* or /dev/ttyUSB* ports found.")
        print("         Plug in the SO-101 USB cable, then re-run.")
        sys.exit(1)

    print(f"  Detected ports: {', '.join(ports)}")
    target = SO101_PORT if SO101_PORT in ports else ports[0]
    if SO101_PORT not in ports:
        print(f"  NOTE  Configured port {SO101_PORT} not found — scanning {target} instead")
    print()

    found = scan(target)

    print()
    print("=" * 55)
    if found:
        print(f"  Found {len(found)} motor(s):\n")
        print("  # Paste into config.py → SO101_MOTORS:")
        print("  SO101_MOTORS = {")
        names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
        for i, (mid, raw, deg) in enumerate(found):
            name = names[i] if i < len(names) else f"joint_{i+1}"
            comment = "# gripper — excluded by default" if name == "gripper" else ""
            print(f'      "{name}": {mid},  {comment}')
        print("  }")
    else:
        print("  No motors responded.")
        print("  Check: Bus powered? Baud rate correct (1,000,000)? Cable wired properly?")
    print("=" * 55)
