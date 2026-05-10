"""
fr5.py — controls the FR5 follower cobot via the Fairino SDK (ServoJ mode).
"""

import threading
import time

from fairino import Robot
from config import FR5_IP, FR5_SERVO_VEL, FR5_FILTER_T


class FR5Controller:
    def __init__(self):
        self._robot    = None
        # Single lock serialises all RPC calls — xmlrpc.client is not thread-safe.
        # Both the 125 Hz ServoJ loop and the 5 Hz gripper thread acquire this.
        self._rpc_lock = threading.Lock()

    def connect(self):
        self._robot = Robot.RPC(FR5_IP)
        if not Robot.RPC.is_connect:
            print("  [FR5] CNDE unavailable — running on XML-RPC only (fully functional)")
            Robot.RPC.is_connect = True

        self._robot.StopMove()
        self._robot.ResetAllError()
        time.sleep(0.3)
        self._robot.Mode(0)
        self._robot.RobotEnable(1)

    def start_servo_mode(self):
        time.sleep(0.1)
        with self._rpc_lock:
            err = self._robot.ServoMoveStart()
        if err not in (0, None):
            raise IOError(f"ServoMoveStart failed with error {err}")

    def stop_servo_mode(self):
        try:
            with self._rpc_lock:
                self._robot.ServoMoveEnd()
        except Exception:
            pass

    def disconnect(self):
        self.stop_servo_mode()
        self._robot = None

    def get_joint_positions(self) -> list[float]:
        with self._rpc_lock:
            raw = self._robot.GetActualJointPosDegree(0)
        if not isinstance(raw, (list, tuple)) or len(raw) != 2:
            raise IOError(
                f"GetActualJointPosDegree returned unexpected value: {raw!r} "
                "(CNDE may not be connected — kill suspended teleop.py and retry)"
            )
        err, joints = raw
        if err != 0:
            raise IOError(f"GetActualJointPosDegree failed with error {err}")
        return list(joints)

    def servo_j(self, joints_deg: list[float]):
        with self._rpc_lock:
            err = self._robot.ServoJ(
                joints_deg, [0] * 6, FR5_SERVO_VEL, 0, 0.008, FR5_FILTER_T, 0
            )
        if err not in (0, None):
            raise IOError(f"ServoJ failed with error {err}")

    def stop(self):
        try:
            with self._rpc_lock:
                self._robot.StopMotion()
        except Exception:
            pass

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *_):
        self.stop()
        self.disconnect()
