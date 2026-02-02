"""
Safety Controller

Enforces limits and protects hardware. This layer sits between
your control code and the motors - all commands pass through here.

NEVER bypass this layer when sending commands to real hardware.
"""

import time
import threading
import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Callable, Optional


@dataclass
class JointLimits:
    """Safety limits for a single joint"""
    min_position: float  # degrees
    max_position: float  # degrees
    max_velocity: float  # deg/s
    max_torque: float    # N·m
    max_temperature: float = 70.0  # Celsius


@dataclass
class SafetyViolation:
    """Record of a safety violation"""
    joint: str
    violation_type: str
    commanded: float
    limit: float
    clamped_to: float
    timestamp: float = field(default_factory=time.time)


class SafetyController:
    """
    Central safety management.

    Responsibilities:
    - Enforce joint position limits
    - Enforce velocity limits
    - Monitor temperatures
    - Implement command watchdog
    - Provide emergency stop
    """

    def __init__(
        self,
        joint_configs: Dict[str, dict],
        watchdog_timeout: float = 0.05,
        on_violation: Optional[Callable[[SafetyViolation], None]] = None
    ):
        """
        Args:
            joint_configs: Dict from config.yaml joints section
            watchdog_timeout: Max time between commands before triggering safety (seconds)
            on_violation: Callback when safety limit is hit
        """
        self.limits: Dict[str, JointLimits] = {}
        self.watchdog_timeout = watchdog_timeout
        self.on_violation = on_violation or self._default_violation_handler

        # Parse joint configs into limits
        for name, cfg in joint_configs.items():
            self.limits[name] = JointLimits(
                min_position=cfg.get('min', -180),
                max_position=cfg.get('max', 180),
                max_velocity=cfg.get('max_velocity', 180),
                max_torque=cfg.get('max_torque', 10),
                max_temperature=cfg.get('max_temperature', 70)
            )

        # Watchdog state
        self._last_command_time: Dict[str, float] = {}
        self._watchdog_thread: Optional[threading.Thread] = None
        self._stop_watchdog = threading.Event()
        self._watchdog_callback: Optional[Callable] = None

        # Violation history
        self.violations: List[SafetyViolation] = []
        self._violation_count = 0

    def _default_violation_handler(self, violation: SafetyViolation):
        """Default handler - just print"""
        print(f"[SAFETY] {violation.joint}: {violation.violation_type} "
              f"({violation.commanded:.2f} -> {violation.clamped_to:.2f})")

    def check_position(self, joint: str, position_deg: float) -> float:
        """
        Check and clamp position to safe limits.

        Args:
            joint: Joint name
            position_deg: Commanded position in degrees

        Returns:
            Safe position in RADIANS (clamped if necessary)
        """
        limits = self.limits.get(joint)
        if limits is None:
            print(f"[SAFETY] Warning: Unknown joint '{joint}'")
            return np.radians(position_deg)

        clamped_deg = np.clip(position_deg, limits.min_position, limits.max_position)

        if clamped_deg != position_deg:
            violation = SafetyViolation(
                joint=joint,
                violation_type="position_limit",
                commanded=position_deg,
                limit=limits.min_position if position_deg < limits.min_position else limits.max_position,
                clamped_to=clamped_deg
            )
            self.violations.append(violation)
            self._violation_count += 1
            self.on_violation(violation)

        # Mark command received for watchdog
        self._last_command_time[joint] = time.time()

        return np.radians(clamped_deg)

    def check_velocity(self, joint: str, velocity_deg_s: float) -> float:
        """
        Check and clamp velocity to safe limits.

        Returns velocity in rad/s.
        """
        limits = self.limits.get(joint)
        if limits is None:
            return np.radians(velocity_deg_s)

        clamped = np.clip(velocity_deg_s, -limits.max_velocity, limits.max_velocity)

        if clamped != velocity_deg_s:
            violation = SafetyViolation(
                joint=joint,
                violation_type="velocity_limit",
                commanded=velocity_deg_s,
                limit=limits.max_velocity,
                clamped_to=clamped
            )
            self.violations.append(violation)
            self._violation_count += 1
            self.on_violation(violation)

        return np.radians(clamped)

    def check_torque(self, joint: str, torque: float) -> float:
        """Check and clamp torque to safe limits."""
        limits = self.limits.get(joint)
        if limits is None:
            return torque

        clamped = np.clip(torque, -limits.max_torque, limits.max_torque)

        if clamped != torque:
            violation = SafetyViolation(
                joint=joint,
                violation_type="torque_limit",
                commanded=torque,
                limit=limits.max_torque,
                clamped_to=clamped
            )
            self.violations.append(violation)
            self._violation_count += 1
            self.on_violation(violation)

        return clamped

    def check_temperature(self, joint: str, temperature: float) -> bool:
        """
        Check if temperature is safe.

        Returns True if safe, False if over limit.
        """
        limits = self.limits.get(joint)
        if limits is None:
            return True

        if temperature > limits.max_temperature:
            violation = SafetyViolation(
                joint=joint,
                violation_type="temperature_limit",
                commanded=temperature,
                limit=limits.max_temperature,
                clamped_to=temperature
            )
            self.violations.append(violation)
            self._violation_count += 1
            self.on_violation(violation)
            return False

        return True

    def check_all_positions(self, positions_deg: Dict[str, float]) -> Dict[str, float]:
        """
        Check all joint positions.

        Args:
            positions_deg: Dict of {joint_name: position_in_degrees}

        Returns:
            Dict of {joint_name: safe_position_in_radians}
        """
        return {
            joint: self.check_position(joint, pos)
            for joint, pos in positions_deg.items()
        }

    # Watchdog functionality
    def start_watchdog(self, on_timeout: Callable[[str], None]):
        """
        Start watchdog thread.

        Args:
            on_timeout: Callback when a joint hasn't received commands
        """
        self._watchdog_callback = on_timeout
        self._stop_watchdog.clear()
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop,
            daemon=True
        )
        self._watchdog_thread.start()
        print("[SAFETY] Watchdog started")

    def stop_watchdog(self):
        """Stop watchdog thread."""
        self._stop_watchdog.set()
        if self._watchdog_thread:
            self._watchdog_thread.join(timeout=1.0)
        print("[SAFETY] Watchdog stopped")

    def _watchdog_loop(self):
        """Monitor for command timeouts."""
        while not self._stop_watchdog.is_set():
            current_time = time.time()

            for joint, last_time in list(self._last_command_time.items()):
                elapsed = current_time - last_time
                if elapsed > self.watchdog_timeout:
                    if self._watchdog_callback:
                        self._watchdog_callback(joint)
                    # Reset to avoid repeated triggers
                    self._last_command_time[joint] = current_time

            self._stop_watchdog.wait(timeout=self.watchdog_timeout / 2)

    def get_stats(self) -> dict:
        """Get safety statistics."""
        return {
            'total_violations': self._violation_count,
            'recent_violations': len(self.violations),
            'joints_monitored': list(self.limits.keys())
        }

    def clear_violations(self):
        """Clear violation history."""
        self.violations.clear()


# Convenience wrapper for simple usage
class SimpleSafety:
    """
    Simplified safety wrapper for basic use.

    Example:
        safety = SimpleSafety(config['joints'])
        safe_rad = safety.check('elbow', 150)  # Returns clamped radians
    """

    def __init__(self, joint_configs: Dict[str, dict]):
        self._controller = SafetyController(joint_configs)

    def check(self, joint: str, position_deg: float) -> float:
        """Check position, return safe value in radians."""
        return self._controller.check_position(joint, position_deg)

    def check_all(self, positions_deg: Dict[str, float]) -> Dict[str, float]:
        """Check all positions, return dict of radians."""
        return self._controller.check_all_positions(positions_deg)


if __name__ == "__main__":
    # Test safety controller
    test_joints = {
        'shoulder': {'min': -90, 'max': 90, 'max_torque': 40},
        'elbow': {'min': 0, 'max': 135, 'max_torque': 20},
    }

    safety = SafetyController(test_joints)

    # Test position limiting
    print("Testing position limits:")
    print(f"  shoulder @ 45°: {np.degrees(safety.check_position('shoulder', 45)):.1f}°")
    print(f"  shoulder @ 120°: {np.degrees(safety.check_position('shoulder', 120)):.1f}°")
    print(f"  elbow @ -10°: {np.degrees(safety.check_position('elbow', -10)):.1f}°")

    print(f"\nViolation count: {safety.get_stats()['total_violations']}")
