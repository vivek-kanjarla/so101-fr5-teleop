"""
check_hardware.py — verify SO-101 (USB) and FR5 (Ethernet) hardware.
Run with: ~/teleop_env/bin/python3 ~/teleop/check_hardware.py
"""

import glob
import struct
import sys

sys.path.insert(0, "/home/slifold/teleop")
from config import SO101_PORT, SO101_BAUDRATE, FR5_IP


# ──────────────────────────────────────────────────────────────────────────────
# SO-101 check
# ──────────────────────────────────────────────────────────────────────────────

def check_so101():
    print("=" * 55)
    print("  SO-101 Leader Arm (USB / Feetech STS3215)")
    print("=" * 55)

    # 1. List candidate ports
    acm_ports = glob.glob("/dev/ttyACM*")
    usb_ports = glob.glob("/dev/ttyUSB*")
    all_ports = sorted(acm_ports + usb_ports)
    if all_ports:
        print(f"  Found serial ports: {', '.join(all_ports)}")
    else:
        print("  FAIL  No /dev/ttyACM* or /dev/ttyUSB* ports found")
        print("         Check: Is the SO-101 USB cable plugged in?")
        print("                Run: dmesg | tail -20  — to see USB events")
        return

    # 2. Try opening the configured port
    try:
        import serial
        ser = serial.Serial(SO101_PORT, SO101_BAUDRATE, timeout=0.1)
        print(f"  PASS  Opened {SO101_PORT} at {SO101_BAUDRATE} baud")
    except serial.SerialException as exc:
        print(f"  FAIL  Could not open {SO101_PORT} — {exc}")
        print("         Check: Does the port exist in the list above?")
        print("                Run: sudo usermod -aG dialout $USER  — for permissions")
        return

    # 3. Feetech STS3215 — read present position of motor ID 1
    #    Instruction packet: FF FF ID LEN INST PARAM... CHECKSUM
    #    Read instruction (0x02): read LEN bytes from ADDR
    #    Register 56 (0x38) = Present Position (2 bytes, STS3215 protocol)
    MOTOR_ID    = 1
    REG_POS     = 56
    READ_LEN    = 2
    header      = bytes([0xFF, 0xFF])
    length_byte = 4  # num params + 2
    instruction = 0x02
    payload     = bytes([MOTOR_ID, length_byte, instruction, REG_POS, READ_LEN])
    checksum    = (~sum(payload[2:]) & 0xFF)  # exclude header
    # Proper checksum: ~(ID + LEN + INST + params) & 0xFF
    cs_sum      = (MOTOR_ID + length_byte + instruction + REG_POS + READ_LEN) & 0xFF
    checksum    = (~cs_sum) & 0xFF
    packet      = header + bytes([MOTOR_ID, length_byte, instruction, REG_POS, READ_LEN, checksum])

    try:
        ser.reset_input_buffer()
        ser.write(packet)
        response = ser.read(8)  # FF FF ID LEN ERR POS_L POS_H CHECKSUM
        if len(response) >= 7 and response[0] == 0xFF and response[1] == 0xFF:
            pos_raw = struct.unpack_from("<H", response, 5)[0]
            # STS3215: 0–4095 counts → 0–360°
            pos_deg = (pos_raw / 4096.0) * 360.0
            print(f"  PASS  Motor ID {MOTOR_ID} responded — raw={pos_raw}, ~{pos_deg:.1f}°")
        else:
            print(f"  WARN  Motor ID {MOTOR_ID} — no valid response (got {response.hex() if response else 'nothing'})")
            print("         Check: Motor ID correct? Bus powered? Terminator installed?")
    except Exception as exc:
        print(f"  FAIL  Motor read error — {exc}")
    finally:
        ser.close()


# ──────────────────────────────────────────────────────────────────────────────
# FR5 check
# ──────────────────────────────────────────────────────────────────────────────

def check_fr5():
    print()
    print("=" * 55)
    print("  FR5 Follower Cobot (Ethernet / Fairino SDK)")
    print("=" * 55)

    try:
        from fairino import Robot
    except ImportError:
        print("  FAIL  Could not import fairino — is it installed in this venv?")
        print("         Run: ~/teleop_env/bin/pip install fairino")
        return

    # Connect
    print(f"  Connecting to FR5 at {FR5_IP}...", flush=True)
    try:
        robot = Robot.RPC(FR5_IP)
    except Exception as exc:
        print(f"  FAIL  Robot.RPC({FR5_IP!r}) raised: {exc}")
        print("         Run check_network.py first to verify connectivity.")
        return

    # SDK + firmware versions  (GetSoftwareVersion → (err, model, sdk, firmware))
    try:
        err, model, sdk_ver, fw_ver = robot.GetSoftwareVersion()
        if err == 0:
            print(f"  Robot model         : {model}")
            print(f"  SDK version         : {sdk_ver}")
            print(f"  Firmware version    : {fw_ver}")
            if sdk_ver != fw_ver and sdk_ver.lstrip('v') not in fw_ver:
                print(f"  WARN  SDK/firmware mismatch — check Fairino release notes.")
        else:
            print(f"  WARN  GetSoftwareVersion returned error {err}")
    except Exception as exc:
        print(f"  WARN  Could not read version info: {exc}")

    # Joint positions
    try:
        ret = robot.GetActualJointPosDegree(0)
        # Fairino SDK returns (error_code, [j1, j2, j3, j4, j5, j6])
        if isinstance(ret, (list, tuple)) and len(ret) == 2:
            err, joints = ret
        else:
            err, joints = 0, ret
        if err == 0 and joints:
            labels = ["J1", "J2", "J3", "J4", "J5", "J6"]
            print("  PASS  Current joint positions (degrees):")
            for label, val in zip(labels, joints):
                print(f"          {label}: {val:>8.3f}°")
        else:
            print(f"  FAIL  GetActualJointPosDegree returned error code {err}")
    except Exception as exc:
        print(f"  FAIL  Could not read joint positions: {exc}")


# ──────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    check_so101()
    check_fr5()
    print()
    print("=" * 55)
    print("  Hardware check complete.")
    print("=" * 55)
