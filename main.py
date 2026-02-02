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
    """Load YAML configuration."""
    with open(path) as f:
        return yaml.safe_load(f)


class ArmController:
    """Main arm controller."""

    def __init__(self, config: dict, test_mode: bool = False):
        self.config = config
        self.test_mode = test_mode

        # Joint info
        self.joints = config['joints']
        self.joint_names = list(self.joints.keys())
        self.motor_ids = {name: cfg['motor_id'] for name, cfg in self.joints.items()}

        # Control params
        self.rate = config['control']['rate_hz']
        self.period = 1.0 / self.rate
        self.kp = config['control']['default_kp']
        self.kd = config['control']['default_kd']

        # Components
        if test_mode:
            self.driver = MockDriver(config['can']['interface'])
        else:
            self.driver = RobstrideDriver(config['can']['interface'])

        self.safety = Safety(self.joints)

        # State
        self._running = False
        self._states: Dict[str, MotorState] = {}

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

        # Scan and enable motors
        found = self.driver.scan()
        expected = list(self.motor_ids.values())
        missing = [m for m in expected if m not in found]

        if missing and not self.test_mode:
            print(f"[WARNING] Missing motors: {missing}")

        print("\nEnabling motors...")
        for name in self.joint_names:
            motor_id = self.motor_ids[name]
            if self.driver.enable(motor_id):
                print(f"  {name} (ID {motor_id}): OK")
            else:
                print(f"  {name} (ID {motor_id}): FAILED")

        print("\nInitialization complete.")
        return True

    def read_state(self) -> Dict[str, MotorState]:
        """Read all motor states."""
        return {
            name: self.driver.read(self.motor_ids[name])
            for name in self.joint_names
        }

    def send_command(self, positions_deg: Dict[str, float]):
        """Send position commands to motors."""
        # Apply safety limits (converts to radians)
        positions_rad = self.safety.clamp_all(positions_deg)

        for name, pos_rad in positions_rad.items():
            self.driver.command(
                motor_id=self.motor_ids[name],
                position=pos_rad,
                kp=self.kp,
                kd=self.kd
            )

    def run(self, duration: float = None):
        """Run control loop."""
        print(f"\n{'='*50}")
        print("Starting control loop (Ctrl+C to stop)")
        print('='*50 + "\n")

        self._running = True
        start = time.time()
        loop_count = 0

        # Graceful shutdown on Ctrl+C
        def on_sigint(sig, frame):
            print("\n[INFO] Stopping...")
            self._running = False

        signal.signal(signal.SIGINT, on_sigint)

        try:
            while self._running:
                loop_start = time.time()
                t = loop_start - start

                # Check duration
                if duration and t >= duration:
                    break

                # === CONTROL LOGIC ===
                # Replace this with your own behavior
                targets = {
                    'shoulder_pitch': 30 * np.sin(0.5 * t),
                    'shoulder_roll': 0,
                    'elbow': 45,
                    'wrist_pitch': 20 * np.sin(0.8 * t),
                    'wrist_roll': 0,
                }
                # =====================

                # Read state
                self._states = self.read_state()

                # Check temps
                for name, state in self._states.items():
                    self.safety.check_temperature(name, state.temperature)

                # Send commands
                self.send_command(targets)

                # Log every second
                loop_count += 1
                if loop_count % self.rate == 0:
                    self._log_status(t, targets)

                # Maintain loop rate
                elapsed = time.time() - loop_start
                if elapsed < self.period:
                    time.sleep(self.period - elapsed)

        finally:
            self.shutdown()

    def _log_status(self, t: float, targets: Dict[str, float]):
        """Print status line."""
        state = self._states.get('shoulder_pitch', MotorState())
        target = targets.get('shoulder_pitch', 0)
        actual = np.degrees(state.position)

        print(f"t={t:5.1f}s | target={target:+6.1f}° actual={actual:+6.1f}° | temp={state.temperature:.0f}°C")

    def shutdown(self):
        """Disable motors and cleanup."""
        print("\nShutting down...")

        for name in self.joint_names:
            self.driver.disable(self.motor_ids[name])

        self.driver.shutdown()
        print("Done.")


def main():
    parser = argparse.ArgumentParser(description='K-Bot Arm Controller')
    parser.add_argument('--config', default='config.yaml', help='Config file')
    parser.add_argument('--test', action='store_true', help='Test mode (no hardware)')
    parser.add_argument('--duration', type=float, help='Run duration in seconds')
    args = parser.parse_args()

    config = load_config(args.config)
    controller = ArmController(config, test_mode=args.test)

    if not controller.initialize():
        return 1

    controller.run(duration=args.duration)
    return 0


if __name__ == "__main__":
    exit(main())
