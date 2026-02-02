"""
Safety Controller

Enforces joint limits. All motor commands should pass through here.
"""

import numpy as np
from typing import Dict


class Safety:
    """
    Enforces joint position and torque limits.

    Usage:
        safety = Safety(config['joints'])
        safe_rad = safety.clamp_position('elbow', 150.0)  # degrees in, radians out
    """

    def __init__(self, joint_configs: Dict[str, dict]):
        """
        Args:
            joint_configs: Joint configuration from config.yaml
        """
        self.limits = {}
        for name, cfg in joint_configs.items():
            self.limits[name] = {
                'min': cfg.get('min', -180),
                'max': cfg.get('max', 180),
                'max_torque': cfg.get('max_torque', 50),
            }

    def clamp_position(self, joint: str, position_deg: float) -> float:
        """
        Clamp position to joint limits.

        Args:
            joint: Joint name
            position_deg: Commanded position in degrees

        Returns:
            Safe position in RADIANS
        """
        limits = self.limits.get(joint)
        if limits is None:
            print(f"[Safety] Unknown joint: {joint}")
            return np.radians(position_deg)

        clamped = np.clip(position_deg, limits['min'], limits['max'])

        if clamped != position_deg:
            print(f"[Safety] {joint}: {position_deg:.1f}° clamped to {clamped:.1f}°")

        return np.radians(clamped)

    def clamp_torque(self, joint: str, torque: float) -> float:
        """Clamp torque to joint limits. Returns N·m."""
        limits = self.limits.get(joint)
        if limits is None:
            return torque

        max_t = limits['max_torque']
        clamped = np.clip(torque, -max_t, max_t)

        if clamped != torque:
            print(f"[Safety] {joint}: torque {torque:.1f} clamped to {clamped:.1f}")

        return clamped

    def clamp_all(self, positions_deg: Dict[str, float]) -> Dict[str, float]:
        """
        Clamp all joint positions.

        Args:
            positions_deg: {joint_name: degrees}

        Returns:
            {joint_name: radians} with limits applied
        """
        return {
            joint: self.clamp_position(joint, pos)
            for joint, pos in positions_deg.items()
        }

    def check_temperature(self, joint: str, temp: float, limit: float = 70.0) -> bool:
        """
        Check if temperature is safe.

        Returns True if safe, False if over limit.
        """
        if temp > limit:
            print(f"[Safety] WARNING: {joint} temperature {temp:.1f}°C exceeds {limit}°C")
            return False
        return True


if __name__ == "__main__":
    # Quick test
    test_config = {
        'shoulder': {'min': -90, 'max': 90},
        'elbow': {'min': 0, 'max': 135},
    }

    safety = Safety(test_config)

    print("Testing position limits:")
    print(f"  shoulder @ 45°: {np.degrees(safety.clamp_position('shoulder', 45)):.1f}°")
    print(f"  shoulder @ 120°: {np.degrees(safety.clamp_position('shoulder', 120)):.1f}°")
    print(f"  elbow @ -10°: {np.degrees(safety.clamp_position('elbow', -10)):.1f}°")
