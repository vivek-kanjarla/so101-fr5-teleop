"""
teleop.py — main teleoperation loop.

Controls:
  Space  — emergency stop (kills ServoJ, exits)
  R      — toggle recording on/off
  Ctrl-C — graceful exit

Data logging:
  Write the task description to episode_instruction.txt before pressing R.
  Each episode saves a CSV (timestep data) + JSON (metadata) under ./episodes/.
"""

import sys
import time
import threading
from contextlib import contextmanager

from pynput import keyboard

NO_GRIPPER = "--no-gripper" in sys.argv

from config import LOOP_HZ, LOOP_PERIOD, LOG_DIR, MAX_DELTA_DEG_PER_CYCLE, INSTRUCTION_FILE, LOG_STATE_DOWNSAMPLE
from so101 import SO101Reader
from fr5 import FR5Controller
from mapper import so101_to_fr5
from logger import EpisodeLogger
from singularity import check as singularity_check, Level
from gripper import DHGripperController
from camera import D405Camera


@contextmanager
def _camera_cleanup(camera):
    """Ensures camera.stop() runs even if a later context manager fails to enter."""
    try:
        yield
    finally:
        if camera:
            camera.stop()


class TeleopSession:
    def __init__(self):
        self._stop_event   = threading.Event()
        self._rehome_event = threading.Event()
        self._estop        = False
        self._logger       = EpisodeLogger()
        self._fr5_current  = [0.0] * 6
        self._prev_step    = [0.0] * 6   # velocity memory for acceleration limiter
        self._sing_level   = Level.CLEAR
        self._cycle        = 0
        self._comm_errors  = 0
        self._consec_errors = 0   # resets to 0 on every successful servo_j
        self._state_cache  = {"actual": None, "eef": None, "vel": None}

    # ── keyboard ──────────────────────────────────────────────────────────────

    def _on_press(self, key):
        if key == keyboard.Key.space:
            print("\n[E-STOP] Space pressed — stopping motion.")
            self._estop = True
            self._stop_event.set()
        elif hasattr(key, "char") and key.char in ("r", "R"):
            if self._logger.recording:
                path = self._logger.stop()
                print(f"[REC] Stopped — saved to {path}")
            else:
                self._state_cache = {"actual": None, "eef": None, "vel": None}
                self._logger.start()
                instr = self._logger._instruction
                instr_str = f'  instruction: "{instr}"' if instr else f"  (no instruction — write to {INSTRUCTION_FILE} before pressing R)"
                print(f"[REC] Recording started (episode {self._logger._episode_id})\n{instr_str}")
        elif hasattr(key, "char") and key.char in ("h", "H"):
            self._rehome_event.set()

    # ── main loop ─────────────────────────────────────────────────────────────

    def run(self):
        print("Connecting to hardware...")
        print(f"  Tip: write task description to {INSTRUCTION_FILE} before pressing R.")

        camera = D405Camera()
        try:
            camera.start()
            self._logger.set_camera(camera)
        except Exception as exc:
            print(f"[CAMERA] Could not start D405 ({exc}) — continuing without camera.")
            camera = None

        with _camera_cleanup(camera), SO101Reader() as arm, FR5Controller() as robot:
            # Capture home poses — delta mapping means first command is always Δ=0
            so101_home        = arm.read_positions_deg()
            self._fr5_current = robot.get_joint_positions()
            fr5_home          = list(self._fr5_current)

            gripper_ctrl = DHGripperController()
            if NO_GRIPPER:
                print("[GRIPPER] Disabled (--no-gripper flag)")
            else:
                gripper_ctrl.start(robot)

            print(f"SO-101 home: {[f'{v:.1f}' for v in so101_home.values()]}")
            print(f"FR5 home:    {[f'{v:.1f}' for v in fr5_home]}")

            # Enter servo mode right before the loop — must be as close as possible
            robot.start_servo_mode()
            print("Ready. Space=E-STOP  R=record  H=re-home  Ctrl-C=quit")

            listener = keyboard.Listener(on_press=self._on_press)
            listener.start()

            try:
                while not self._stop_event.is_set():
                    t0 = time.monotonic()

                    try:
                        # Re-home: freeze current positions as new reference
                        if self._rehome_event.is_set():
                            self._rehome_event.clear()
                            so101_home = arm.read_positions_deg()
                            fr5_home   = list(self._fr5_current)
                            self._prev_step  = [0.0] * 6   # clear velocity memory
                            self._sing_level = Level.CLEAR
                            print(f"\n[RE-HOME] New home captured — FR5 J1..J6: "
                                  f"{[f'{v:.1f}' for v in fr5_home]}")

                        so101_pos = arm.read_positions_deg()

                        # Update gripper state and handle pause if needed
                        try:
                            gripper_ctrl.update_so101(arm.read_gripper_deg())
                        except Exception:
                            pass
                        if gripper_ctrl.wants_pause():
                            gripper_ctrl.pause_for_gripper(robot)
                            # Reset rate-limiter to current actual position after pause
                            self._fr5_current = robot.get_joint_positions()
                            self._prev_step   = [0.0] * 6   # clear velocity memory

                        # Singularity check on current FR5 position
                        level, scale, msg = singularity_check(self._fr5_current)
                        if level != self._sing_level:
                            if level == Level.DANGER:
                                print(f"\n[SINGULARITY DANGER] {msg} — motion blocked")
                            elif level == Level.WARN:
                                print(f"\n[SINGULARITY WARN]   {msg} — speed reduced to {scale*100:.0f}%")
                            elif level == Level.CLEAR:
                                print("\n[SINGULARITY] Clear")
                            self._sing_level = level

                        if level == Level.DANGER:
                            fr5_cmd = list(self._fr5_current)
                            self._prev_step = [0.0] * 6   # clear velocity memory when blocked
                        else:
                            effective_limit = MAX_DELTA_DEG_PER_CYCLE * scale
                            fr5_cmd, self._prev_step = so101_to_fr5(
                                so101_pos, so101_home, fr5_home,
                                self._fr5_current, self._prev_step,
                                delta_limit=effective_limit,
                            )

                        log_time = time.time()   # capture before servo_j for accurate timestamp
                        robot.servo_j(fr5_cmd)
                        self._fr5_current = fr5_cmd
                        self._cycle += 1
                        self._consec_errors = 0   # servo_j succeeded

                        # ── read actual robot state for logging ───────────────
                        # Stagger the three reads across consecutive qualifying
                        # cycles — one RPC call per cycle (~3 ms) instead of all
                        # three at once (~9 ms). Doing all three in the same cycle
                        # pushed it to ~14 ms, exceeding the 8 ms ServoJ budget
                        # and causing the FR5 to halt whenever recording started.
                        if self._logger.recording and self._cycle % LOG_STATE_DOWNSAMPLE == 0:
                            slot = (self._cycle // LOG_STATE_DOWNSAMPLE) % 3
                            if slot == 0:
                                try:
                                    self._state_cache["actual"] = robot.get_joint_positions()
                                except Exception:
                                    pass
                            elif slot == 1:
                                try:
                                    self._state_cache["eef"] = robot.get_eef_pose()
                                except Exception:
                                    pass
                            else:
                                try:
                                    self._state_cache["vel"] = robot.get_joint_velocities()
                                except Exception:
                                    pass

                        gripper_norm = gripper_ctrl.get_normalized()

                        self._logger.log(
                            log_time, so101_pos, fr5_cmd,
                            fr5_actual=self._state_cache["actual"],
                            fr5_eef=self._state_cache["eef"],
                            gripper_norm=gripper_norm,
                            fr5_vel=self._state_cache["vel"],
                        )

                        # Heartbeat (~1 Hz): confirms the leader is being read
                        # and shows whether the follower is actually commanded
                        # to move. so101 drift ≈ 0 → leader read frozen; so101
                        # drift > 0 but fr5 drift ≈ 0 → mapper problem.
                        if self._cycle % LOOP_HZ == 0:
                            so101_drift = max(abs(so101_pos[k] - so101_home[k]) for k in so101_pos)
                            fr5_drift   = max(abs(c - h) for c, h in zip(fr5_cmd, fr5_home))
                            try:
                                actual = robot.get_joint_positions()
                                actual_str = f"  actual_J1={actual[0]:.1f}°"
                            except Exception:
                                actual_str = "  actual_J1=ERR"
                            print(f"[HB] cyc={self._cycle}  sing={level.value}  "
                                  f"SO101 moved {so101_drift:6.1f}°  →  "
                                  f"FR5 cmd moved {fr5_drift:6.1f}°{actual_str}  errs={self._comm_errors}")

                    except Exception as exc:
                        # A transient SO-101 serial glitch or ServoJ RPC error
                        # must NOT tear down the whole session — skip this one
                        # cycle and retry. Persistent faults surface in the
                        # throttled log and the heartbeat error counter.
                        self._comm_errors += 1
                        self._consec_errors += 1
                        if self._comm_errors == 1 or self._comm_errors % 50 == 0:
                            print(f"[WARN] cycle skipped (error #{self._comm_errors}): {exc!r}")

                        # After 20 consecutive failures the FR5 has almost certainly
                        # exited ServoJ mode (timing fault, robot fault, or network
                        # hiccup). Stop → clear errors → re-enter servo mode so the
                        # session can recover without a restart.
                        if self._consec_errors % 20 == 0:
                            print(f"[RECOVER] {self._consec_errors} consecutive errors — "
                                  "resetting servo mode...")
                            try:
                                robot.stop_servo_mode()
                                time.sleep(0.15)
                                robot.reset_errors()
                                time.sleep(0.05)
                                robot.start_servo_mode()
                                print("[RECOVER] Servo mode restored — resuming teleop.")
                            except Exception as exc2:
                                print(f"[RECOVER] Could not restore servo mode: {exc2!r}")

                        time.sleep(LOOP_PERIOD)
                        continue

                    elapsed = time.monotonic() - t0
                    sleep   = LOOP_PERIOD - elapsed
                    if sleep > 0:
                        time.sleep(sleep)

            except KeyboardInterrupt:
                print("\n[EXIT] Ctrl-C — shutting down.")
            finally:
                gripper_ctrl.stop()
                robot.stop_servo_mode()
                listener.stop()
                if self._logger.recording:
                    path = self._logger.stop()
                    if path:
                        print(f"[REC] Auto-saved episode to {path}")

        if self._estop:
            print("[E-STOP] Motion halted. Check robot state before restarting.")
        print("Teleop session ended.")


if __name__ == "__main__":
    TeleopSession().run()
