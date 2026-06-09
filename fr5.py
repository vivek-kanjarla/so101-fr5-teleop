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
        Robot.RPC.is_conect = True   # force XML-RPC mode; writing never raises AttributeError

        self._robot.StopMove()
        self._robot.ResetAllError()
        time.sleep(0.3)
        self._robot.Mode(0)
        self._robot.RobotEnable(1)
        time.sleep(0.5)   # servo drives need ~500 ms to fully energise after enable

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

    def reset_errors(self):
        with self._rpc_lock:
            self._robot.ResetAllError()

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

    def get_eef_pose(self) -> list[float]:
        """Return [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg] — TCP pose in flange frame."""
        with self._rpc_lock:
            raw = self._robot.GetActualTCPPose(0)
        if not isinstance(raw, (list, tuple)) or len(raw) != 2:
            raise IOError(f"GetActualTCPPose returned unexpected value: {raw!r}")
        err, pose = raw
        if err != 0:
            raise IOError(f"GetActualTCPPose failed with error {err}")
        return list(pose)

    def get_joint_velocities(self) -> list[float]:
        """Return [v1..v6] — actual joint velocities in deg/s."""
        with self._rpc_lock:
            raw = self._robot.GetActualJointSpeedsDegree(0)
        if not isinstance(raw, (list, tuple)) or len(raw) != 2:
            raise IOError(f"GetActualJointSpeedsDegree returned unexpected value: {raw!r}")
        err, vels = raw
        if err != 0:
            raise IOError(f"GetActualJointSpeedsDegree failed with error {err}")
        return list(vels)

    def servo_j(self, joints_deg: list[float]):
        # Coerce to plain Python float — xmlrpc.client cannot marshal
        # numpy.float64 (raises TypeError). This is the Python→XML-RPC
        # boundary, so normalise here regardless of what the caller passes.
        joints_deg = [float(j) for j in joints_deg]
        with self._rpc_lock:
            err = self._robot.ServoJ(
                joints_deg, [0] * 6, FR5_SERVO_VEL, 0, 0.008, FR5_FILTER_T, 0
            )
        if err not in (0, None):
            raise IOError(f"ServoJ failed with error {err}")

    def inverse_kin(self, desc_pos: list[float]) -> list[float]:
        """Inverse kinematics: TCP pose -> joint angles (deg).

        desc_pos = [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg] (the same space as
        get_eef_pose()). Solved with config=-1 so the Fairino solver seeds from the
        current joint configuration — successive solves stay continuous (no elbow
        flips between ticks). Used by the delta-EEF deploy path:
            target_eef = get_eef_pose() + predicted_delta  ->  inverse_kin  ->  servo_j

        Raises IOError if the target is unreachable / singular (caller should hold
        the previous joint command on failure rather than crash the control loop).
        """
        desc_pos = [float(v) for v in desc_pos]
        with self._rpc_lock:
            raw = self._robot.GetInverseKin(0, desc_pos, -1)   # type=0 absolute pose
        # Fairino getters return [err, value]; some firmware returns a bare err on failure.
        if isinstance(raw, (list, tuple)) and len(raw) == 2:
            err, joints = raw
        else:
            err, joints = raw, None
        if err != 0 or joints is None:
            raise IOError(f"GetInverseKin failed (err={err}) for pose {desc_pos}")
        return list(joints)

    def servo_cart(self, desc_pos: list[float], mode: int = 0):
        """Cartesian servo straight to a TCP pose (alternative to inverse_kin + servo_j).

        desc_pos = [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg].
        mode = 0 absolute target, mode = 1 incremental (a delta) — so a delta-EEF
        policy can drive the arm with mode=1 and skip the explicit IK step.
        Prefer inverse_kin + servo_j if you want to detect unreachable targets
        before commanding motion.
        """
        desc_pos = [float(v) for v in desc_pos]
        with self._rpc_lock:
            err = self._robot.ServoCart(
                mode, desc_pos, [1.0] * 6, 0.0, 0.0, 0.008, 0.0, 0.0
            )
        if err not in (0, None):
            raise IOError(f"ServoCart failed with error {err}")

    def activate_gripper(self, index: int) -> int:
        with self._rpc_lock:
            return self._robot.ActGripper(index, 1)

    def send_gripper(self, index, pct, vel, force, maxtime, blocking, gtype) -> int:
        with self._rpc_lock:
            return self._robot.MoveGripper(
                index, pct, vel, force, maxtime, blocking, gtype, 0, 0, 0
            )

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
