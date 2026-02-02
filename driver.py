"""
Robstride Motor Driver

Low-level communication with Robstride actuators over CAN bus.
Wrap this around your existing working CAN code.
"""

import time
from dataclasses import dataclass
from typing import Optional, List, Dict

@dataclass
class MotorState:
    """Feedback from a single motor"""
    position: float = 0.0      # radians
    velocity: float = 0.0      # rad/s
    current: float = 0.0       # Amps
    temperature: float = 0.0   # Celsius
    enabled: bool = False
    error_code: int = 0


class RobstrideDriver:
    """
    Driver for Robstride actuators.

    This is a template - fill in with your working CAN code.
    You can also use the existing robstride Python library:
        pip install robstride
    """

    def __init__(self, interface: str = "can0", bitrate: int = 1_000_000):
        self.interface = interface
        self.bitrate = bitrate
        self._bus = None
        self._initialized = False

    def initialize(self) -> bool:
        """
        Initialize CAN interface.

        Returns True if successful.
        """
        try:
            # Option 1: Use python-can directly
            # import can
            # self._bus = can.interface.Bus(
            #     channel=self.interface,
            #     bustype='socketcan',
            #     bitrate=self.bitrate
            # )

            # Option 2: Use robstride library
            # from robstride import RobstrideBus
            # self._bus = RobstrideBus(interface=self.interface)

            # For now, just mark as initialized (replace with real code)
            self._initialized = True
            print(f"[Driver] Initialized CAN interface: {self.interface}")
            return True

        except Exception as e:
            print(f"[Driver] Failed to initialize: {e}")
            return False

    def scan(self, id_range: tuple = (1, 50)) -> List[int]:
        """
        Scan for connected motors.

        Returns list of found motor IDs.
        """
        if not self._initialized:
            return []

        found = []
        print(f"[Driver] Scanning for motors on {self.interface}...")

        # Replace with actual scan code
        # for motor_id in range(id_range[0], id_range[1] + 1):
        #     try:
        #         self._bus.get_feedback(motor_id, timeout=0.01)
        #         found.append(motor_id)
        #     except:
        #         pass

        # Placeholder: return expected IDs
        found = [11, 12, 13, 14, 15]

        print(f"[Driver] Found motors: {found}")
        return found

    def enable(self, motor_id: int) -> bool:
        """Enable a motor for torque control."""
        if not self._initialized:
            return False

        try:
            # self._bus.enable(motor_id)
            print(f"[Driver] Enabled motor {motor_id}")
            return True
        except Exception as e:
            print(f"[Driver] Failed to enable motor {motor_id}: {e}")
            return False

    def disable(self, motor_id: int) -> bool:
        """Disable a motor (coast mode)."""
        if not self._initialized:
            return False

        try:
            # self._bus.disable(motor_id)
            print(f"[Driver] Disabled motor {motor_id}")
            return True
        except Exception as e:
            print(f"[Driver] Failed to disable motor {motor_id}: {e}")
            return False

    def disable_all(self, motor_ids: List[int]) -> None:
        """Disable all motors."""
        for motor_id in motor_ids:
            self.disable(motor_id)

    def set_zero(self, motor_id: int) -> bool:
        """Set current position as zero."""
        if not self._initialized:
            return False

        try:
            # self._bus.set_zero(motor_id)
            print(f"[Driver] Set zero for motor {motor_id}")
            return True
        except Exception as e:
            print(f"[Driver] Failed to set zero for motor {motor_id}: {e}")
            return False

    def read(self, motor_id: int) -> MotorState:
        """Read current motor state."""
        if not self._initialized:
            return MotorState()

        try:
            # feedback = self._bus.get_feedback(motor_id)
            # return MotorState(
            #     position=feedback.position,
            #     velocity=feedback.velocity,
            #     current=feedback.current,
            #     temperature=feedback.temperature,
            #     enabled=True,
            #     error_code=0
            # )

            # Placeholder
            return MotorState(
                position=0.0,
                velocity=0.0,
                current=0.0,
                temperature=25.0,
                enabled=True,
                error_code=0
            )
        except Exception as e:
            return MotorState(error_code=-1)

    def read_all(self, motor_ids: List[int]) -> Dict[int, MotorState]:
        """Read state from multiple motors."""
        return {mid: self.read(mid) for mid in motor_ids}

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
            motor_id: Target motor CAN ID
            position: Target position in radians
            velocity: Target velocity in rad/s
            kp: Position gain (N·m/rad)
            kd: Damping gain (N·m·s/rad)
            torque: Feedforward torque in N·m

        Returns:
            True if command sent successfully
        """
        if not self._initialized:
            return False

        try:
            # self._bus.write_mit_frame(
            #     motor_id,
            #     position=position,
            #     velocity=velocity,
            #     kp=kp,
            #     kd=kd,
            #     torque=torque
            # )
            return True
        except Exception as e:
            print(f"[Driver] Command failed for motor {motor_id}: {e}")
            return False

    def command_position(self, motor_id: int, position: float) -> bool:
        """Simple position command with default gains."""
        return self.command(motor_id, position=position, kp=20.0, kd=2.0)

    def shutdown(self) -> None:
        """Clean shutdown of driver."""
        if self._bus:
            # self._bus.shutdown()
            pass
        self._initialized = False
        print("[Driver] Shutdown complete")


# Convenience function for quick testing
def test_driver():
    """Quick driver test."""
    driver = RobstrideDriver()

    if not driver.initialize():
        print("Failed to initialize driver")
        return

    motors = driver.scan()
    print(f"Found {len(motors)} motors")

    for mid in motors:
        state = driver.read(mid)
        print(f"Motor {mid}: pos={state.position:.3f} rad, temp={state.temperature:.1f}°C")

    driver.shutdown()


if __name__ == "__main__":
    test_driver()
