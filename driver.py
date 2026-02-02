"""
Robstride Motor Driver

Communicates with Robstride actuators using Seeed Studio's RobStride library.
Reference: https://wiki.seeedstudio.com/robstride_control/
"""

from dataclasses import dataclass
from typing import Optional, List, Dict

try:
    from robstride import RobstrideBus
    ROBSTRIDE_AVAILABLE = True
except ImportError:
    ROBSTRIDE_AVAILABLE = False


@dataclass
class MotorState:
    """Feedback from a motor."""
    position: float = 0.0      # radians
    velocity: float = 0.0      # rad/s
    torque: float = 0.0        # N·m
    temperature: float = 25.0  # Celsius


class RobstrideDriver:
    """
    Low-level driver for Robstride actuators.

    This driver operates in MOTOR space (raw encoder values).
    Sign and offset conversions should be done at a higher level.
    """

    def __init__(self, interface: str = "can0"):
        self.interface = interface
        self._bus: Optional[RobstrideBus] = None

    def initialize(self) -> bool:
        """
        Initialize CAN bus connection.
        Does NOT enable motors or send any movement commands.
        """
        if not ROBSTRIDE_AVAILABLE:
            print("[Driver] robstride library not installed")
            print("  Install: pip install robstride")
            print("  Or: git clone https://github.com/Seeed-Projects/RobStride_Control")
            return False

        try:
            self._bus = RobstrideBus(self.interface)
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
        if self._bus is None:
            return []

        try:
            found = self._bus.scan_channel()
            return found if found else []
        except Exception as e:
            print(f"[Driver] Scan error: {e}")
            return []

    def enable(self, motor_id: int) -> bool:
        """Enable motor for torque control."""
        if self._bus is None:
            return False

        try:
            self._bus.enable_motors([motor_id])
            return True
        except Exception as e:
            print(f"[Driver] Enable error (ID {motor_id}): {e}")
            return False

    def disable(self, motor_id: int) -> bool:
        """Disable motor (coast/zero torque)."""
        if self._bus is None:
            return False

        try:
            # Zero gains = coast mode
            self._bus.write_operation_frame(
                motor_id=motor_id,
                p_des=0.0, v_des=0.0,
                kp=0.0, kd=0.5,  # Small damping for smooth stop
                t_ff=0.0
            )
            return True
        except Exception as e:
            print(f"[Driver] Disable error (ID {motor_id}): {e}")
            return False

    def set_zero(self, motor_id: int) -> bool:
        """
        Set current position as zero in motor's memory.
        This modifies the motor's internal reference point.
        """
        if self._bus is None:
            return False

        try:
            if hasattr(self._bus, 'set_zero'):
                self._bus.set_zero(motor_id)
            elif hasattr(self._bus, 'set_motor_zero'):
                self._bus.set_motor_zero(motor_id)
            else:
                print("[Driver] set_zero not available in library")
                return False
            return True
        except Exception as e:
            print(f"[Driver] Set zero error (ID {motor_id}): {e}")
            return False

    def read(self, motor_id: int) -> MotorState:
        """
        Read motor state. Does NOT send movement commands.
        Returns raw motor position (before sign/offset).
        """
        if self._bus is None:
            return MotorState()

        try:
            resp = self._bus.read_frame(motor_id)
            return MotorState(
                position=resp.get('position', 0.0),
                velocity=resp.get('velocity', 0.0),
                torque=resp.get('torque', 0.0),
                temperature=resp.get('temperature', 25.0)
            )
        except:
            return MotorState()

    def command(
        self,
        motor_id: int,
        position: float,
        velocity: float = 0.0,
        kp: float = 20.0,
        kd: float = 2.0,
        torque: float = 0.0
    ) -> bool:
        """
        Send MIT mode command (impedance control).
        Position should be in MOTOR space (raw radians).
        """
        if self._bus is None:
            return False

        try:
            self._bus.write_operation_frame(
                motor_id=motor_id,
                p_des=position,
                v_des=velocity,
                kp=kp,
                kd=kd,
                t_ff=torque
            )
            return True
        except Exception as e:
            print(f"[Driver] Command error (ID {motor_id}): {e}")
            return False

    def shutdown(self) -> None:
        """Clean shutdown."""
        self._bus = None


class MockDriver:
    """Mock driver for testing without hardware."""

    def __init__(self, interface: str = "can0"):
        self._positions: Dict[int, float] = {}

    def initialize(self) -> bool:
        print("[MockDriver] Initialized (no hardware)")
        return True

    def scan(self) -> List[int]:
        return [11, 12, 13, 14, 15]

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
