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

import time
import threading

from pynput import keyboard

from config import LOOP_PERIOD, LOG_DIR, MAX_DELTA_DEG_PER_CYCLE, INSTRUCTION_FILE, LOG_STATE_DOWNSAMPLE
from so101 import SO101Reader
from fr5 import FR5Controller
from mapper import so101_to_fr5
from logger import EpisodeLogger
from singularity import check as singularity_check, Level
from gripper import DHGripperController


class TeleopSession:
    def __init__(self):
        self._stop_event   = threading.Event()
        self._rehome_event = threading.Event()
        self._estop        = False
        self._logger       = EpisodeLogger()
        self._fr5_current  = [0.0] * 6
        self._sing_level   = Level.CLEAR
        self._cycle        = 0
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
        with SO101Reader() as arm, FR5Controller() as robot:
            # Capture home poses — delta mapping means first command is always Δ=0
            so101_home        = arm.read_positions_deg()
            self._fr5_current = robot.get_joint_positions()
            fr5_home          = list(self._fr5_current)

            gripper_ctrl = DHGripperController()
            gripper_ctrl.start()

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

                    # Re-home: freeze current positions as new reference
                    if self._rehome_event.is_set():
                        self._rehome_event.clear()
                        so101_home = arm.read_positions_deg()
                        fr5_home   = list(self._fr5_current)
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
                    else:
                        effective_limit = MAX_DELTA_DEG_PER_CYCLE * scale
                        fr5_cmd = so101_to_fr5(
                            so101_pos, so101_home, fr5_home,
                            self._fr5_current, delta_limit=effective_limit
                        )

                    log_time = time.time()   # capture before servo_j for accurate timestamp
                    robot.servo_j(fr5_cmd)
                    self._fr5_current = fr5_cmd
                    self._cycle += 1

                    # ── read actual robot state for logging ───────────────────
                    # Reads happen every LOG_STATE_DOWNSAMPLE cycles so the
                    # ~6–9ms RPC overhead doesn't blow the 8ms ServoJ budget.
                    # Cached values fill in the off cycles — every row is complete.
                    if self._logger.recording and self._cycle % LOG_STATE_DOWNSAMPLE == 0:
                        try:
                            self._state_cache["actual"] = robot.get_joint_positions()
                        except Exception:
                            pass
                        try:
                            self._state_cache["eef"] = robot.get_eef_pose()
                        except Exception:
                            pass
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
