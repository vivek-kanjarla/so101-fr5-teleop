"""
check_hardware.py — verify SO-101 and FR5 hardware connectivity.
Run with: python check_hardware.py
"""

import sys
from config import FR5_IP, SO101_PORT, SO101_BAUDRATE, SO101_MOTORS, SO101_GRIPPER_ID


# ── SO-101 ────────────────────────────────────────────────────────────────────

def check_so101():
    print("=" * 55)
    print("  SO-101 Leader Arm (USB Serial / Feetech STS3215)")
    print("=" * 55)

    try:
        from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS
    except ImportError:
        print("  FAIL  Could not import scservo_sdk — run: pip install scservo-sdk")
        return False

    port    = PortHandler(SO101_PORT)
    packet  = PacketHandler(0)   # protocol 0 for STS3215

    if not port.openPort():
        print(f"  FAIL  Could not open {SO101_PORT}")
        print("         Check: Is the USB cable plugged in?")
        print("                Run: ls /dev/ttyACM*  to confirm the port exists")
        print("                Run: groups $USER     to confirm you're in dialout")
        return False
    ok_port = True
    print(f"  PASS  Opened {SO101_PORT} at {SO101_BAUDRATE} baud")

    if not port.setBaudRate(SO101_BAUDRATE):
        print(f"  FAIL  Could not set baud rate {SO101_BAUDRATE}")
        port.closePort()
        return False

    ADDR_PRESENT_POSITION = 56
    ADDR_TORQUE_ENABLE    = 40

    all_ok = True
    for name, mid in list(SO101_MOTORS.items()) + [("gripper", SO101_GRIPPER_ID)]:
        # Disable torque so arm hangs free (same as teleop)
        packet.write1ByteTxRx(port, mid, ADDR_TORQUE_ENABLE, 0)

        raw, result, _ = packet.read2ByteTxRx(port, mid, ADDR_PRESENT_POSITION)
        if result == COMM_SUCCESS:
            signed = raw if raw < 32768 else raw - 65536
            deg    = (signed / 4096.0) * 360.0
            print(f"  PASS  motor {mid:2d}  ({name:<14s})  pos = {deg:8.2f}°")
        else:
            print(f"  FAIL  motor {mid:2d}  ({name:<14s})  read failed (result={result})")
            all_ok = False

    port.closePort()
    return all_ok


# ── FR5 ───────────────────────────────────────────────────────────────────────

def check_fr5():
    print()
    print("=" * 55)
    print("  FR5 Follower Cobot (Ethernet / Fairino SDK)")
    print("=" * 55)

    try:
        from fairino import Robot
    except ImportError:
        print("  FAIL  Could not import fairino — install from Fairino .whl")
        return False

    print(f"  Connecting to FR5 at {FR5_IP}...", flush=True)
    try:
        robot = Robot.RPC(FR5_IP)
        Robot.RPC.is_conect = True
    except Exception as exc:
        print(f"  FAIL  Robot.RPC({FR5_IP!r}) raised: {exc}")
        print("         Run check_network.py first to verify connectivity.")
        return False

    try:
        err, model, sdk_ver, fw_ver = robot.GetSoftwareVersion()
        if err == 0:
            print(f"  Robot model         : {model}")
            print(f"  SDK version         : {sdk_ver}")
            print(f"  Firmware version    : {fw_ver}")
            if sdk_ver != fw_ver and sdk_ver.lstrip('v') not in fw_ver:
                print("  WARN  SDK/firmware mismatch — check Fairino release notes.")
        else:
            print(f"  WARN  GetSoftwareVersion returned error {err}")
    except Exception as exc:
        print(f"  WARN  Could not read version info: {exc}")

    all_ok = True

    try:
        ret = robot.GetActualJointPosDegree(0)
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
            all_ok = False
    except Exception as exc:
        print(f"  FAIL  Could not read joint positions: {exc}")
        all_ok = False

    try:
        ret = robot.GetActualTCPPose(0)
        if isinstance(ret, (list, tuple)) and len(ret) == 2:
            err, pose = ret
        else:
            err, pose = 0, ret
        if err == 0 and pose:
            labels = ["x_mm", "y_mm", "z_mm", "rx_deg", "ry_deg", "rz_deg"]
            print("  PASS  Current TCP pose:")
            for label, val in zip(labels, pose):
                print(f"          {label}: {val:>8.2f}")
    except Exception as exc:
        print(f"  WARN  Could not read TCP pose: {exc}")

    return all_ok


# ── entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    so101_ok = check_so101()
    fr5_ok   = check_fr5()

    print()
    print("=" * 55)
    print("  Summary")
    print("=" * 55)
    print(f"  SO-101  : {'PASS' if so101_ok else 'FAIL'}")
    print(f"  FR5     : {'PASS' if fr5_ok   else 'FAIL'}")
    print("=" * 55)

    if so101_ok and fr5_ok:
        print("  Both checks PASSED — ready to run teleop.py")
        sys.exit(0)
    else:
        print("  One or more checks FAILED — see messages above before running teleop.")
        sys.exit(1)
