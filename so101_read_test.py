"""
so101_read_test.py — isolate the SO-101 leader read path.

Streams all joint + gripper positions for 15 seconds. Physically move the
arm while it runs and watch whether the numbers change.

  Numbers CHANGE when you move the arm → leader reads fine; bug is the mapper.
  Numbers FROZEN                       → leader read path is broken
                                         (torque still enabled, or comms fault).

SO101Reader.open() prints a warning per motor if torque-disable fails — watch
for those at the top of the output.

Run on the Linux machine:
    python so101_read_test.py
"""

import time

from so101 import SO101Reader

DURATION_S = 15
HZ         = 5


def main():
    print("=" * 64)
    print("  SO-101 LEADER READ TEST  —  move the arm while this runs")
    print("=" * 64)

    with SO101Reader() as arm:
        first  = None
        moved  = {}
        period = 1.0 / HZ
        t_end  = time.time() + DURATION_S

        while time.time() < t_end:
            t0 = time.monotonic()
            try:
                pos  = arm.read_positions_deg()
                grip = arm.read_gripper_deg()
            except Exception as exc:
                print(f"  READ FAILED: {exc}")
                time.sleep(period)
                continue

            sample = dict(pos, gripper=grip)
            if first is None:
                first = sample
            for k, v in sample.items():
                moved[k] = max(moved.get(k, 0.0), abs(v - first[k]))

            line = "  ".join(f"{k}={v:7.2f}" for k, v in sample.items())
            print(f"  {line}")

            slack = period - (time.monotonic() - t0)
            if slack > 0:
                time.sleep(slack)

    print("-" * 64)
    print("  Max movement detected per joint (vs first reading):")
    for k, d in moved.items():
        print(f"    {k:14s} {d:7.2f}°{'   <-- MOVED' if d > 1.0 else '   (frozen)'}")
    print("-" * 64)
    if moved and all(d < 1.0 for d in moved.values()):
        print("  RESULT: ALL joints frozen — leader read path is broken.")
        print("          Torque still enabled, or serial comms failing.")
    elif moved:
        print("  RESULT: leader reads work — bug is in the mapper / teleop loop.")
    print("=" * 64)


if __name__ == "__main__":
    main()
