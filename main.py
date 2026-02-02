#!/usr/bin/env python3
"""
K-Bot Arm Controller

Usage:
    python main.py                    # Run with hardware
    python main.py --test             # Test mode (no hardware)
    python main.py --duration 30      # Run for 30 seconds
"""

import argparse
import signal
import time
import yaml
import numpy as np
from typing import Dict

from driver import RobstrideDriver, MockDriver, MotorState
from safety import Safety


def load_config(path: str = "config.yaml") -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


class ArmController:
    """
    Arm controller with proper sign and offset handling.

    Coordinate frames:
    - JOINT space: What the user sees (degrees, respects sign/offset)
    - MOTOR space: Raw encoder values (radians, no sign/offset)

    All external interfaces use JOINT space.
    Driver uses MOTOR space.
    """

    def __init__(self, config: dict, test_mode: bool = False):
        self.config = config
        self.test_mode = test_mode

        # Joint info
        self.joints = config['joints']
        self.joint_names = list(self.joints.keys())

        # Build lookup tables
        self.motor_ids = {}
        self.signs = {}
        self.offsets = {}

        for name, cfg in self.joints.items():
            self.motor_ids[name] = cfg['motor_id']
            self.signs[name] = cfg.get('sign', 1)
            self.offsets[name] = np.radians(cfg.get('zero_offset', 0.0))

        # Control params
        self.rate = config['control']['rate_hz']
        self.period = 1.0 / self.rate
        self.kp = config['control']['default_kp']
        self.kd = config['control']['default_kd']

        # Components
        if test_mode:
            self.driver = MockDriver()
        else:
            self.driver = RobstrideDriver(config['can']['interface'])

        self.safety = Safety(self.joints)

        # State
        self._running = False
        self._states: Dict[str, float] = {}  # Joint positions in degrees

    def _motor_to_joint(self, name: str, motor_rad: float) -> float:
        """Convert motor radians to joint degrees."""
        sign = self.signs[name]
        offset = self.offsets[name]
        return np.degrees(sign * motor_rad - offset)

    def _joint_to_motor(self, name: str, joint_deg: float) -> float:
        """Convert joint degrees to motor radians."""
        sign = self.signs[name]
        offset = self.offsets[name]
        # joint_deg = degrees(sign * motor_rad - offset)
        # radians(joint_deg) = sign * motor_rad - offset
        # motor_rad = (radians(joint_deg) + offset) / sign
        return (np.radians(joint_deg) + offset) / sign

    def initialize(self) -> bool:
        """Initialize driver and enable motors."""
        print(f"\n{'='*50}")
        print("K-Bot Arm Controller")
        print('='*50)
        print(f"  Mode: {'TEST' if self.test_mode else 'HARDWARE'}")
        print(f"  Rate: {self.rate} Hz")
        print(f"  Joints: {self.joint_names}")

        if not self.driver.initialize():
            print("[ERROR] Driver initialization failed")
            return False

        # Verify motors
        found = self.driver.scan()
        for name in self.joint_names:
            motor_id = self.motor_ids[name]
            status = "OK" if motor_id in found else "MISSING"
            sign = self.signs[name]
            offset_deg = np.degrees(self.offsets[name])
            print(f"  {name}: ID={motor_id} sign={sign:+d} offset={offset_deg:+.1f}° [{status}]")

        # Enable motors
        print("\nEnabling motors...")
        for name in self.joint_names:
            self.driver.enable(self.motor_ids[name])

        print("Ready.")
        return True

    def read_state(self) -> Dict[str, float]:
        """Read joint positions in degrees (JOINT space)."""
        positions = {}
        for name in self.joint_names:
            motor_state = self.driver.read(self.motor_ids[name])
            positions[name] = self._motor_to_joint(name, motor_state.position)
        return positions

    def send_command(self, positions_deg: Dict[str, float]):
        """
        Send position commands in degrees (JOINT space).
        Applies safety limits, sign, and offset automatically.
        """
        # Safety clamp (returns radians, but we need to redo the conversion)
        safe_rad = self.safety.clamp_all(positions_deg)

        for name, joint_rad in safe_rad.items():
            joint_deg = np.degrees(joint_rad)
            motor_rad = self._joint_to_motor(name, joint_deg)

            self.driver.command(
                motor_id=self.motor_ids[name],
                position=motor_rad,
                kp=self.kp,
                kd=self.kd
            )

    def run(self, duration: float = None):
        """Run control loop."""
        print(f"\n{'='*50}")
        print("Control loop running (Ctrl+C to stop)")
        print('='*50 + "\n")

        self._running = True
        start = time.time()
        loop_count = 0

        def on_sigint(sig, frame):
            print("\n[Stopping...]")
            self._running = False

        signal.signal(signal.SIGINT, on_sigint)

        try:
            while self._running:
                loop_start = time.time()
                t = loop_start - start

                if duration and t >= duration:
                    break

                # === YOUR CONTROL LOGIC ===
                # Positions in JOINT space (degrees)
                targets = {
                    'shoulder_pitch': 30 * np.sin(0.5 * t),
                    'shoulder_roll': 0,
                    'elbow': 45,
                    'wrist_pitch': 20 * np.sin(0.8 * t),
                    'wrist_roll': 0,
                }
                # ==========================

                # Read and command
                self._states = self.read_state()
                self.send_command(targets)

                # Log every second
                loop_count += 1
                if loop_count % self.rate == 0:
                    self._log(t, targets)

                # Rate control
                elapsed = time.time() - loop_start
                if elapsed < self.period:
                    time.sleep(self.period - elapsed)

        finally:
            self.shutdown()

    def _log(self, t: float, targets: Dict[str, float]):
        target = targets.get('shoulder_pitch', 0)
        actual = self._states.get('shoulder_pitch', 0)
        print(f"t={t:5.1f}s | target={target:+6.1f}° actual={actual:+6.1f}°")

    def shutdown(self):
        print("\nShutting down...")
        for name in self.joint_names:
            self.driver.disable(self.motor_ids[name])
        self.driver.shutdown()
        print("Done.")


def main():
    parser = argparse.ArgumentParser(description='K-Bot Arm Controller')
    parser.add_argument('--config', default='config.yaml')
    parser.add_argument('--test', action='store_true', help='Test mode')
    parser.add_argument('--duration', type=float, help='Run duration (seconds)')
    args = parser.parse_args()

    config = load_config(args.config)
    controller = ArmController(config, test_mode=args.test)

    if not controller.initialize():
        return 1

    controller.run(duration=args.duration)
    return 0


if __name__ == "__main__":
    exit(main())
