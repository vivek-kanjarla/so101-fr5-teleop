"""
check_hardware.py — verify FR5 (Ethernet) hardware connectivity.
Run with: python check_hardware.py
"""

import sys
from config import FR5_IP


def check_fr5():
    print("=" * 55)
    print("  FR5 Follower Cobot (Ethernet / Fairino SDK)")
    print("=" * 55)

    try:
        from fairino import Robot
    except ImportError:
        print("  FAIL  Could not import fairino — is it installed?")
        print("         Run: pip install fairino")
        return

    print(f"  Connecting to FR5 at {FR5_IP}...", flush=True)
    try:
        robot = Robot.RPC(FR5_IP)
    except Exception as exc:
        print(f"  FAIL  Robot.RPC({FR5_IP!r}) raised: {exc}")
        print("         Run check_network.py first to verify connectivity.")
        return

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
    except Exception as exc:
        print(f"  FAIL  Could not read joint positions: {exc}")

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


if __name__ == "__main__":
    check_fr5()
    print()
    print("=" * 55)
    print("  Hardware check complete.")
    print("=" * 55)
