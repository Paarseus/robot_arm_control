"""
Robstride Motor Driver

Communicates with Robstride actuators using Seeed Studio's RobStride library.

Reference: https://wiki.seeedstudio.com/robstride_control/
"""

from dataclasses import dataclass
from typing import Optional, List, Dict

# Seeed Studio RobStride library
# Install: pip install robstride  (or clone from GitHub)
# GitHub: https://github.com/Seeed-Projects/RobStride_Control
try:
    from robstride import RobstrideBus
    ROBSTRIDE_AVAILABLE = True
except ImportError:
    ROBSTRIDE_AVAILABLE = False
    print("[Driver] Warning: robstride library not installed")
    print("         Install with: pip install robstride")
    print("         Or clone: https://github.com/Seeed-Projects/RobStride_Control")


@dataclass
class MotorState:
    """Feedback from a motor."""
    position: float = 0.0      # radians
    velocity: float = 0.0      # rad/s
    torque: float = 0.0        # N·m (estimated)
    temperature: float = 25.0  # Celsius


class RobstrideDriver:
    """
    Driver for Robstride actuators via Seeed Studio library.

    Uses MIT mode (impedance control) for all commands.
    """

    def __init__(self, interface: str = "can0"):
        self.interface = interface
        self._bus: Optional[RobstrideBus] = None

    def initialize(self) -> bool:
        """Initialize CAN bus connection."""
        if not ROBSTRIDE_AVAILABLE:
            print("[Driver] Cannot initialize - robstride library not installed")
            return False

        try:
            self._bus = RobstrideBus(self.interface)
            print(f"[Driver] Connected to {self.interface}")
            return True
        except Exception as e:
            print(f"[Driver] Failed to initialize: {e}")
            return False

    def scan(self) -> List[int]:
        """Scan for connected motors. Returns list of motor IDs."""
        if self._bus is None:
            return []

        try:
            found = self._bus.scan_channel()
            print(f"[Driver] Found motors: {found}")
            return found
        except Exception as e:
            print(f"[Driver] Scan failed: {e}")
            return []

    def enable(self, motor_id: int) -> bool:
        """Enable motor for torque control."""
        if self._bus is None:
            return False

        try:
            self._bus.enable_motors([motor_id])
            return True
        except Exception as e:
            print(f"[Driver] Enable failed for motor {motor_id}: {e}")
            return False

    def disable(self, motor_id: int) -> bool:
        """Disable motor (coast mode)."""
        if self._bus is None:
            return False

        try:
            # Send zero torque command with zero gains to coast
            self._bus.write_operation_frame(
                motor_id=motor_id,
                p_des=0.0,
                v_des=0.0,
                kp=0.0,
                kd=0.0,
                t_ff=0.0
            )
            return True
        except Exception as e:
            print(f"[Driver] Disable failed for motor {motor_id}: {e}")
            return False

    def set_zero(self, motor_id: int) -> bool:
        """Set current position as zero."""
        if self._bus is None:
            return False

        try:
            # This depends on the specific Seeed library version
            # Some versions have set_zero(), others may need different approach
            if hasattr(self._bus, 'set_zero'):
                self._bus.set_zero(motor_id)
            elif hasattr(self._bus, 'set_motor_zero'):
                self._bus.set_motor_zero(motor_id)
            else:
                print(f"[Driver] set_zero not available in this library version")
                return False
            return True
        except Exception as e:
            print(f"[Driver] Set zero failed for motor {motor_id}: {e}")
            return False

    def read(self, motor_id: int) -> MotorState:
        """Read current motor state."""
        if self._bus is None:
            return MotorState()

        try:
            response = self._bus.read_frame(motor_id)
            return MotorState(
                position=response.get('position', 0.0),
                velocity=response.get('velocity', 0.0),
                torque=response.get('torque', 0.0),
                temperature=response.get('temperature', 25.0)
            )
        except Exception as e:
            # Return default state on read failure
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

        Args:
            motor_id: Motor CAN ID
            position: Target position (radians)
            velocity: Target velocity (rad/s)
            kp: Position gain
            kd: Velocity/damping gain
            torque: Feedforward torque (N·m)
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
            print(f"[Driver] Command failed for motor {motor_id}: {e}")
            return False

    def shutdown(self) -> None:
        """Clean shutdown."""
        self._bus = None
        print("[Driver] Shutdown complete")


# For testing without hardware
class MockDriver:
    """Mock driver for testing without hardware."""

    def __init__(self, interface: str = "can0"):
        self.interface = interface
        self._positions: Dict[int, float] = {}

    def initialize(self) -> bool:
        print(f"[MockDriver] Initialized (no hardware)")
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
        print("[MockDriver] Shutdown")
