"""
gripper.py — maps SO-101 gripper motor to DH AG-160-95 via Fairino SDK.

MoveGripper is blocked with error 73 while ServoMoveStart is active on any
connection. The fix: when a gripper state change is needed, the background
thread signals the main loop, which pauses ServoJ, sends the gripper command,
then resumes. Joint motion freezes for ~200ms per open/close — acceptable.

Handshake:
  gripper thread  →  sets _cmd_ready, blocks on _servo_paused
  main loop       →  sees _cmd_ready, calls pause_for_gripper()
                     → ServoMoveEnd, sets _servo_paused, waits on _cmd_done
  gripper thread  →  sends MoveGripper, sets _cmd_done, clears _servo_paused
  main loop       →  resumes ServoMoveStart
"""

import threading
import time

from config import (
    GRIPPER_INDEX,
    GRIPPER_OPEN_PCT, GRIPPER_CLOSE_PCT,
    GRIPPER_TYPE, GRIPPER_VEL_PCT, GRIPPER_FORCE_PCT, GRIPPER_MAXTIME_MS,
    SO101_GRIPPER_OPEN_THRESHOLD, SO101_GRIPPER_CLOSE_THRESHOLD,
    SO101_GRIPPER_RANGE,
)


class DHGripperController:
    POLL_HZ = 5

    def __init__(self):
        self._rpc       = None
        self._thread    = None
        self._stop_evt  = threading.Event()
        self._state     = None      # "open" | "closed" | None

        self._pos_deg   = 0.0
        self._pos_lock  = threading.Lock()

        # Handshake events
        self._cmd_ready    = threading.Event()   # gripper → main: I need to send
        self._servo_paused = threading.Event()   # main → gripper: servo stopped, go ahead
        self._cmd_done     = threading.Event()   # gripper → main: command sent, resume

        self._target_pct   = 0                   # position to send when paused

    # ── called from main loop ─────────────────────────────────────────────────

    def update_so101(self, raw_deg: float):
        with self._pos_lock:
            self._pos_deg = raw_deg

    def wants_pause(self) -> bool:
        """True when gripper thread is waiting for ServoJ to pause."""
        return self._cmd_ready.is_set()

    def pause_for_gripper(self, robot):
        """
        Called by main loop when wants_pause() is True.
        Stops servo mode, clears faults, lets gripper send, then resumes.
        Blocks for ~400ms total.
        """
        robot.stop_servo_mode()
        time.sleep(0.2)                     # let controller fully exit servo mode
        with robot._rpc_lock:
            robot._robot.ResetAllError()    # clear faults raised by interrupted ServoJ
        time.sleep(0.1)

        self._servo_paused.set()            # signal gripper thread to proceed
        self._cmd_done.wait(timeout=2.0)    # wait for MoveGripper to complete
        self._cmd_done.clear()
        self._servo_paused.clear()
        self._cmd_ready.clear()

        with robot._rpc_lock:
            robot._robot.RobotEnable(1)
        robot.start_servo_mode()            # re-enter ServoJ mode

    # ── lifecycle ─────────────────────────────────────────────────────────────

    def start(self):
        from fairino import Robot
        from config import FR5_IP
        self._rpc = Robot.RPC(FR5_IP)
        Robot.RPC.is_connect = True

        err = self._rpc.ActGripper(GRIPPER_INDEX, 1)
        if err != 0:
            print(f"[GRIPPER] ActGripper failed (err={err}) — check gripper config on FR5 controller")
            self._rpc = None
            return
        time.sleep(0.5)
        print(f"[GRIPPER] DH AG-160-95 activated (index={GRIPPER_INDEX})")

        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_evt.set()
        # Unblock any pending handshake so thread can exit
        self._servo_paused.set()
        if self._thread:
            self._thread.join(timeout=2)
        self._rpc = None

    # ── background thread ─────────────────────────────────────────────────────

    def _loop(self):
        interval = 1.0 / self.POLL_HZ
        while not self._stop_evt.is_set():
            t0 = time.monotonic()

            with self._pos_lock:
                pos = self._pos_deg

            lo, hi = SO101_GRIPPER_RANGE
            norm = max(0.0, min(1.0, (pos - lo) / (hi - lo)))

            desired = None
            if norm >= SO101_GRIPPER_OPEN_THRESHOLD:
                desired = "open"
            elif norm <= SO101_GRIPPER_CLOSE_THRESHOLD:
                desired = "closed"

            if desired and desired != self._state:
                pct = GRIPPER_OPEN_PCT if desired == "open" else GRIPPER_CLOSE_PCT
                self._target_pct = pct

                # Signal main loop to pause ServoJ
                self._cmd_ready.set()
                # Wait until main loop has stopped servo mode
                self._servo_paused.wait(timeout=3.0)

                if self._stop_evt.is_set():
                    break

                try:
                    err = self._rpc.MoveGripper(
                        GRIPPER_INDEX, pct,
                        GRIPPER_VEL_PCT, GRIPPER_FORCE_PCT,
                        GRIPPER_MAXTIME_MS,
                        1,             # non-blocking
                        GRIPPER_TYPE,
                        0, 0, 0,
                    )
                    if err == 0:
                        self._state = desired
                        print(f"[GRIPPER] {desired.upper()}  (norm={norm:.2f})")
                    else:
                        print(f"[GRIPPER] MoveGripper error {err}")
                except Exception as exc:
                    print(f"[GRIPPER] Exception: {exc}")
                finally:
                    self._cmd_done.set()   # tell main loop to resume

            remaining = interval - (time.monotonic() - t0)
            if remaining > 0:
                self._stop_evt.wait(timeout=remaining)

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *_):
        self.stop()
