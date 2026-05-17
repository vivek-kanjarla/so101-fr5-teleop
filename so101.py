"""
so101.py — reads joint positions from the SO-101 leader arm via Feetech STS3215 servos.
"""

from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS
from config import SO101_PORT, SO101_BAUDRATE, SO101_MOTORS, SO101_GRIPPER_ID

PROTOCOL_VERSION = 0
ADDR_PRESENT_POSITION = 56  # STS3215 register: present position (2 bytes)
ADDR_TORQUE_ENABLE    = 40  # STS3215 register: torque enable (1 byte, 0=off 1=on)


def _signed(raw: int) -> int:
    """Convert unsigned 16-bit encoder read to signed (-32768..32767).
    STS3215 uses signed representation when crossing the zero position;
    without this, motors near zero read as ~2880° instead of ~0°.
    """
    return raw if raw < 32768 else raw - 65536


class SO101Reader:
    def __init__(self):
        self._port    = PortHandler(SO101_PORT)
        self._packet  = PacketHandler(PROTOCOL_VERSION)
        self._motor_ids = list(SO101_MOTORS.values())
        self._names     = list(SO101_MOTORS.keys())

    def open(self):
        if not self._port.openPort():
            raise RuntimeError(f"Cannot open port {SO101_PORT}")
        if not self._port.setBaudRate(SO101_BAUDRATE):
            raise RuntimeError(f"Cannot set baud rate {SO101_BAUDRATE}")
        # Leader arm must move freely — disable torque on all motors so the
        # servos stop fighting the human and we only read encoder positions.
        # A failed write here leaves a motor stiff and its encoder frozen, so
        # verify each one rather than firing and forgetting.
        for mid in self._motor_ids + [SO101_GRIPPER_ID]:
            result, error = self._packet.write1ByteTxRx(
                self._port, mid, ADDR_TORQUE_ENABLE, 0
            )
            if result != COMM_SUCCESS:
                print(f"[SO-101] WARNING: torque-disable failed on motor {mid} "
                      f"— that joint will be stiff and read frozen")

    def close(self):
        self._port.closePort()

    def read_positions_deg(self) -> dict[str, float]:
        """Return {joint_name: degrees} for all configured motors."""
        self._port.clearPort()  # discard any stale bytes left by a previous gripper read
        positions = {}
        for name, mid in zip(self._names, self._motor_ids):
            raw, result, _ = self._packet.read2ByteTxRx(
                self._port, mid, ADDR_PRESENT_POSITION
            )
            if result != COMM_SUCCESS:
                raise IOError(f"Motor {mid} ({name}) read failed")
            positions[name] = (_signed(raw) / 4096.0) * 360.0
        return positions

    def read_gripper_deg(self) -> float:
        """Return gripper motor (ID 6) position in degrees."""
        raw, result, _ = self._packet.read2ByteTxRx(
            self._port, SO101_GRIPPER_ID, ADDR_PRESENT_POSITION
        )
        if result != COMM_SUCCESS:
            raise IOError(f"Gripper motor {SO101_GRIPPER_ID} read failed")
        return (_signed(raw) / 4096.0) * 360.0

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *_):
        self.close()
