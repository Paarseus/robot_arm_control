"""
Robstride Motor Driver

Communicates with Robstride actuators over CAN bus.
Uses robstride.Client for parameter reads and enable/disable,
robstride_dynamics for bus scanning, and raw CAN frames for MIT mode control.
"""

import struct
from dataclasses import dataclass
from typing import Dict, List, Optional

import numpy as np

try:
    import can
    from robstride import Client

    ROBSTRIDE_AVAILABLE = True
except ImportError:
    ROBSTRIDE_AVAILABLE = False

try:
    from robstride_dynamics import RobstrideBus

    DYNAMICS_AVAILABLE = True
except ImportError:
    DYNAMICS_AVAILABLE = False


# MIT mode parameter ranges (rs-01 defaults, safe for all models)
MIT_P_MAX = 12.566370614359172  # 4*pi rad
MIT_V_MAX = 44.0  # rad/s
MIT_KP_MAX = 500.0
MIT_KD_MAX = 5.0
MIT_T_MAX = 17.0  # Nm
COMM_TYPE_OPERATION_CONTROL = 1
HOST_CAN_ID = 0xFD


@dataclass
class MotorState:
    """Feedback from a motor."""

    position: float = 0.0  # radians
    velocity: float = 0.0  # rad/s
    torque: float = 0.0  # N·m
    temperature: float = 25.0  # Celsius


class RobstrideDriver:
    """
    Low-level driver for Robstride actuators.

    This driver operates in MOTOR space (raw encoder values).
    Sign and offset conversions should be done at a higher level.
    """

    def __init__(self, interface: str = "can0"):
        self.interface = interface
        self._bus: Optional["can.BusABC"] = None
        self._client: Optional["Client"] = None

    def initialize(self) -> bool:
        """
        Initialize CAN bus connection.
        Does NOT enable motors or send any movement commands.
        """
        if not ROBSTRIDE_AVAILABLE:
            print("[Driver] robstride library not installed")
            print("  Install: pip install robstride")
            return False

        try:
            self._bus = can.Bus(channel=self.interface, interface="socketcan")
            self._client = Client(self._bus)
            print(f"[Driver] Connected to {self.interface}")
            return True
        except Exception as e:
            print(f"[Driver] Failed to initialize: {e}")
            return False

    def scan(self) -> List[int]:
        """
        Scan for motors. Does NOT enable or move anything.
        Returns list of motor IDs found.
        """
        if not DYNAMICS_AVAILABLE:
            print("[Driver] robstride_dynamics not installed, cannot scan")
            return []

        try:
            found = RobstrideBus.scan_channel(self.interface)
            return sorted(found.keys()) if found else []
        except Exception as e:
            print(f"[Driver] Scan error: {e}")
            return []

    def enable(self, motor_id: int) -> bool:
        """Enable motor for torque control."""
        if self._client is None:
            return False

        try:
            self._client.enable(motor_id)
            return True
        except Exception as e:
            print(f"[Driver] Enable error (ID {motor_id}): {e}")
            return False

    def disable(self, motor_id: int) -> bool:
        """Disable motor (coast/zero torque)."""
        if self._client is None:
            return False

        try:
            self._client.disable(motor_id)
            return True
        except Exception as e:
            print(f"[Driver] Disable error (ID {motor_id}): {e}")
            return False

    def read(self, motor_id: int) -> MotorState:
        """
        Read motor state. Does NOT send movement commands.
        Returns raw motor position (before sign/offset).
        """
        if self._client is None:
            return MotorState()

        try:
            position = self._client.read_param(motor_id, "mechpos")
            return MotorState(position=float(position))
        except Exception:
            return MotorState()

    def command(
        self,
        motor_id: int,
        position: float,
        velocity: float = 0.0,
        kp: float = 20.0,
        kd: float = 2.0,
        torque: float = 0.0,
    ) -> bool:
        """
        Send MIT mode command (impedance control).
        Position should be in MOTOR space (raw radians).

        Constructs raw CAN frame per Robstride protocol:
          Arb ID: (1 << 24) | (torque_u16 << 8) | motor_id
          Data:   [p_des_u16, v_des_u16, kp_u16, kd_u16] as big-endian u16s
        """
        if self._bus is None:
            return False

        try:
            p = np.clip(position, -MIT_P_MAX, MIT_P_MAX)
            v = np.clip(velocity, -MIT_V_MAX, MIT_V_MAX)
            kp_c = np.clip(kp, 0.0, MIT_KP_MAX)
            kd_c = np.clip(kd, 0.0, MIT_KD_MAX)
            t = np.clip(torque, -MIT_T_MAX, MIT_T_MAX)

            p_u16 = int(((p / MIT_P_MAX) + 1.0) * 0x7FFF)
            v_u16 = int(((v / MIT_V_MAX) + 1.0) * 0x7FFF)
            kp_u16 = int((kp_c / MIT_KP_MAX) * 0xFFFF)
            kd_u16 = int((kd_c / MIT_KD_MAX) * 0xFFFF)
            t_u16 = int(((t / MIT_T_MAX) + 1.0) * 0x7FFF)

            p_u16 = int(np.clip(p_u16, 0, 0xFFFF))
            v_u16 = int(np.clip(v_u16, 0, 0xFFFF))
            kp_u16 = int(np.clip(kp_u16, 0, 0xFFFF))
            kd_u16 = int(np.clip(kd_u16, 0, 0xFFFF))
            t_u16 = int(np.clip(t_u16, 0, 0xFFFF))

            data = struct.pack(">HHHH", p_u16, v_u16, kp_u16, kd_u16)
            arb_id = (COMM_TYPE_OPERATION_CONTROL << 24) | (t_u16 << 8) | motor_id

            self._bus.send(
                can.Message(arbitration_id=arb_id, data=data, is_extended_id=True)
            )
            return True
        except Exception as e:
            print(f"[Driver] Command error (ID {motor_id}): {e}")
            return False

    def shutdown(self) -> None:
        """Clean shutdown."""
        if self._bus is not None:
            try:
                self._bus.shutdown()
            except Exception:
                pass
        self._bus = None
        self._client = None


class MockDriver:
    """Mock driver for testing without hardware."""

    def __init__(self, interface: str = "can0"):
        self._positions: Dict[int, float] = {}

    def initialize(self) -> bool:
        print("[MockDriver] Initialized (no hardware)")
        return True

    def scan(self) -> List[int]:
        return [21, 22, 23, 24, 25]

    def enable(self, motor_id: int) -> bool:
        self._positions.setdefault(motor_id, 0.0)
        return True

    def disable(self, motor_id: int) -> bool:
        return True

    def set_zero(self, motor_id: int) -> bool:
        self._positions[motor_id] = 0.0
        return True

    def read(self, motor_id: int) -> MotorState:
        return MotorState(position=self._positions.get(motor_id, 0.0))

    def command(self, motor_id: int, position: float, **kwargs) -> bool:
        self._positions[motor_id] = position
        return True

    def shutdown(self) -> None:
        pass
