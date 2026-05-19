"""
test_servo_after_gripper.py — isolate whether the gripper pause sequence breaks ServoJ.

Runs four steps in order, printing a PASS/FAIL for each:

  Step 1: Plain ServoJ without any gripper involvement  (baseline — must PASS)
  Step 2: ActGripper before ServoJ                     (does activating gripper break it?)
  Step 3: pause_for_gripper sequence then ServoJ        (does the pause sequence break it?)
  Step 4: pause_for_gripper WITHOUT RobotEnable(1)      (does removing RobotEnable fix it?)

Run from ~/teleop with the venv active:
    python test_servo_after_gripper.py
"""

import time
from fr5 import FR5Controller
from config import GRIPPER_INDEX, GRIPPER_TYPE, GRIPPER_VEL_PCT, GRIPPER_FORCE_PCT, GRIPPER_MAXTIME_MS

JOINT  = 0       # J1 shoulder pan
DEG    = 5.0
CYCLES = 250
HZ     = 125


def move_j1(robot, start, deg=DEG, cycles=CYCLES) -> float:
    """Ramp J1 by deg over cycles. Returns actual movement."""
    step   = deg / cycles
    target = list(start)
    for _ in range(cycles):
        t0 = time.monotonic()
        target[JOINT] += step
        robot.servo_j(target)
        slack = (1 / HZ) - (time.monotonic() - t0)
        if slack > 0:
            time.sleep(slack)
    robot.stop_servo_mode()
    time.sleep(0.3)
    end = robot.get_joint_positions()
    return end[JOINT] - start[JOINT]


def _fresh_connect():
    """Return a connected FR5Controller (connect() already called)."""
    ctrl = FR5Controller()
    ctrl.connect()
    return ctrl


def step1_baseline():
    print("\n── Step 1: plain ServoJ (no gripper) ──")
    ctrl = _fresh_connect()
    start = ctrl.get_joint_positions()
    ctrl.start_servo_mode()
    moved = move_j1(ctrl, start)
    ctrl.disconnect()
    ok = abs(moved) > 0.5
    print(f"  J1 moved {moved:+.2f}°  →  {'PASS' if ok else 'FAIL'}")
    return ok


def step2_act_gripper_then_servo():
    print("\n── Step 2: ActGripper before ServoJ ──")
    ctrl = _fresh_connect()
    err = ctrl.activate_gripper(GRIPPER_INDEX)
    print(f"  ActGripper err={err}")
    time.sleep(0.5)
    start = ctrl.get_joint_positions()
    ctrl.start_servo_mode()
    moved = move_j1(ctrl, start)
    ctrl.disconnect()
    ok = abs(moved) > 0.5
    print(f"  J1 moved {moved:+.2f}°  →  {'PASS' if ok else 'FAIL'}")
    return ok


def step3_pause_with_robot_enable():
    print("\n── Step 3: pause_for_gripper sequence WITH RobotEnable(1) ──")
    ctrl = _fresh_connect()
    ctrl.activate_gripper(GRIPPER_INDEX)
    time.sleep(0.5)
    start = ctrl.get_joint_positions()
    ctrl.start_servo_mode()

    # Simulate pause_for_gripper (current code, with RobotEnable)
    print("  Simulating pause_for_gripper (WITH RobotEnable)...")
    ctrl.stop_servo_mode()
    time.sleep(0.2)
    with ctrl._rpc_lock:
        ctrl._robot.ResetAllError()
    time.sleep(0.1)
    err = ctrl.send_gripper(GRIPPER_INDEX, 0, GRIPPER_VEL_PCT, GRIPPER_FORCE_PCT,
                            GRIPPER_MAXTIME_MS, 1, GRIPPER_TYPE)
    print(f"  MoveGripper err={err}")
    with ctrl._rpc_lock:
        ctrl._robot.RobotEnable(1)
    time.sleep(0.5)
    ctrl.start_servo_mode()

    moved = move_j1(ctrl, start)
    ctrl.disconnect()
    ok = abs(moved) > 0.5
    print(f"  J1 moved {moved:+.2f}°  →  {'PASS' if ok else 'FAIL'}")
    return ok


def step4_pause_without_robot_enable():
    print("\n── Step 4: pause_for_gripper sequence WITHOUT RobotEnable(1) ──")
    ctrl = _fresh_connect()
    ctrl.activate_gripper(GRIPPER_INDEX)
    time.sleep(0.5)
    start = ctrl.get_joint_positions()
    ctrl.start_servo_mode()

    # Simulate pause_for_gripper WITHOUT RobotEnable
    print("  Simulating pause_for_gripper (NO RobotEnable)...")
    ctrl.stop_servo_mode()
    time.sleep(0.2)
    with ctrl._rpc_lock:
        ctrl._robot.ResetAllError()
    time.sleep(0.1)
    err = ctrl.send_gripper(GRIPPER_INDEX, 0, GRIPPER_VEL_PCT, GRIPPER_FORCE_PCT,
                            GRIPPER_MAXTIME_MS, 1, GRIPPER_TYPE)
    print(f"  MoveGripper err={err}")
    # NO RobotEnable(1) here
    with ctrl._rpc_lock:
        ctrl._robot.ResetAllError()   # clear any fault from MoveGripper
    time.sleep(0.2)
    ctrl.start_servo_mode()

    moved = move_j1(ctrl, start)
    ctrl.disconnect()
    ok = abs(moved) > 0.5
    print(f"  J1 moved {moved:+.2f}°  →  {'PASS' if ok else 'FAIL'}")
    return ok


if __name__ == "__main__":
    print("=" * 60)
    print("  GRIPPER → SERVO ISOLATION TEST")
    print("=" * 60)

    r1 = step1_baseline()
    r2 = step2_act_gripper_then_servo()
    r3 = step3_pause_with_robot_enable()
    r4 = step4_pause_without_robot_enable()

    print("\n" + "=" * 60)
    print(f"  Step 1 baseline:                   {'PASS' if r1 else 'FAIL'}")
    print(f"  Step 2 ActGripper then ServoJ:     {'PASS' if r2 else 'FAIL'}")
    print(f"  Step 3 pause WITH RobotEnable(1):  {'PASS' if r3 else 'FAIL'}")
    print(f"  Step 4 pause WITHOUT RobotEnable:  {'PASS' if r4 else 'FAIL'}")
    print("=" * 60)
    if r1 and r2 and not r3 and r4:
        print("  DIAGNOSIS: RobotEnable(1) in pause_for_gripper is breaking ServoJ.")
        print("  FIX: remove RobotEnable(1) from gripper.py pause_for_gripper().")
    elif r1 and not r2:
        print("  DIAGNOSIS: ActGripper itself breaks ServoJ.")
    elif not r1:
        print("  DIAGNOSIS: baseline ServoJ broken — hardware/connection issue.")
    else:
        print("  DIAGNOSIS: inconclusive — check individual step output above.")
