#!/usr/bin/env python3
"""
K-Bot Arm Controller - Main Entry Point

This is the main control loop. It:
1. Loads configuration
2. Initializes the driver and safety systems
3. Runs the control loop at a fixed rate
4. Handles graceful shutdown

Usage:
    python main.py                    # Run with default config
    python main.py --config my.yaml   # Run with custom config
    python main.py --test             # Test mode (no motor commands)
"""

import argparse
import signal
import time
import yaml
import numpy as np
from typing import Dict

from driver import RobstrideDriver, MotorState
from safety import SafetyController


class ArmController:
    """
    Main arm controller class.

    Manages the control loop and coordinates driver + safety.
    """

    def __init__(self, config_path: str = "config.yaml", test_mode: bool = False):
        self.config = self._load_config(config_path)
        self.test_mode = test_mode

        # Extract joint info
        self.joints = self.config['joints']
        self.joint_names = list(self.joints.keys())
        self.motor_ids = {name: cfg['motor_id'] for name, cfg in self.joints.items()}
        self.id_to_name = {v: k for k, v in self.motor_ids.items()}

        # Control parameters
        self.control_rate = self.config['control']['rate_hz']
        self.control_period = 1.0 / self.control_rate
        self.default_kp = self.config['control']['default_kp']
        self.default_kd = self.config['control']['default_kd']

        # Initialize components
        self.driver = RobstrideDriver(
            interface=self.config['can']['interface'],
            bitrate=self.config['can']['bitrate']
        )
        self.safety = SafetyController(
            joint_configs=self.joints,
            watchdog_timeout=self.config['safety']['watchdog_timeout_ms'] / 1000.0
        )

        # State
        self._running = False
        self._motor_states: Dict[str, MotorState] = {}
        self._target_positions: Dict[str, float] = {}  # degrees
        self._loop_count = 0

    def _load_config(self, path: str) -> dict:
        """Load YAML configuration."""
        with open(path, 'r') as f:
            return yaml.safe_load(f)

    def initialize(self) -> bool:
        """Initialize driver and enable motors."""
        print(f"\n{'='*50}")
        print(f"Initializing K-Bot Arm Controller")
        print(f"{'='*50}")
        print(f"  Test mode: {self.test_mode}")
        print(f"  Control rate: {self.control_rate} Hz")
        print(f"  Joints: {self.joint_names}")
        print()

        if self.test_mode:
            print("[TEST MODE] Skipping hardware initialization")
            return True

        # Initialize driver
        if not self.driver.initialize():
            print("[ERROR] Failed to initialize driver")
            return False

        # Scan for motors
        found = self.driver.scan()
        expected = list(self.motor_ids.values())
        missing = [mid for mid in expected if mid not in found]

        if missing:
            print(f"[WARNING] Missing motors: {missing}")
            print("  Check CAN connections and motor IDs")
            # Continue anyway - might be intentional for partial testing

        # Enable motors
        print("\nEnabling motors...")
        for name in self.joint_names:
            motor_id = self.motor_ids[name]
            if self.driver.enable(motor_id):
                print(f"  {name} (ID {motor_id}): enabled")
            else:
                print(f"  {name} (ID {motor_id}): FAILED")

        # Initialize target positions to zero
        for name in self.joint_names:
            self._target_positions[name] = 0.0

        print("\nInitialization complete.")
        return True

    def read_state(self) -> Dict[str, MotorState]:
        """Read state from all motors."""
        if self.test_mode:
            # Return simulated state
            return {
                name: MotorState(
                    position=np.radians(self._target_positions.get(name, 0)),
                    velocity=0.0,
                    current=0.1,
                    temperature=25.0,
                    enabled=True
                )
                for name in self.joint_names
            }

        states = {}
        for name in self.joint_names:
            motor_id = self.motor_ids[name]
            states[name] = self.driver.read(motor_id)
        return states

    def send_commands(self, positions_deg: Dict[str, float],
                      kp: float = None, kd: float = None):
        """
        Send position commands to all joints.

        Args:
            positions_deg: Dict of {joint_name: target_position_in_degrees}
            kp: Position gain (optional, uses default)
            kd: Damping gain (optional, uses default)
        """
        kp = kp if kp is not None else self.default_kp
        kd = kd if kd is not None else self.default_kd

        # Safety check - converts to radians
        positions_rad = self.safety.check_all_positions(positions_deg)

        if self.test_mode:
            return

        # Send to motors
        for name, pos_rad in positions_rad.items():
            motor_id = self.motor_ids[name]
            self.driver.command(
                motor_id=motor_id,
                position=pos_rad,
                velocity=0.0,
                kp=kp,
                kd=kd,
                torque=0.0
            )

    def set_targets(self, positions_deg: Dict[str, float]):
        """Set target positions (will be sent in next control loop)."""
        self._target_positions.update(positions_deg)

    def run(self, duration: float = None):
        """
        Run the main control loop.

        Args:
            duration: Run for this many seconds (None = run forever)
        """
        print(f"\n{'='*50}")
        print("Starting control loop")
        print("Press Ctrl+C to stop")
        print(f"{'='*50}\n")

        self._running = True
        start_time = time.time()
        self._loop_count = 0

        # Setup signal handler for graceful shutdown
        def signal_handler(sig, frame):
            print("\n[INFO] Shutdown requested...")
            self._running = False

        signal.signal(signal.SIGINT, signal_handler)

        try:
            while self._running:
                loop_start = time.time()

                # Check duration
                if duration and (loop_start - start_time) >= duration:
                    print(f"[INFO] Duration complete ({duration}s)")
                    break

                # === YOUR CONTROL LOGIC GOES HERE ===
                # This example does a gentle sinusoidal motion
                t = loop_start - start_time

                targets = {
                    'shoulder_pitch': 30 * np.sin(0.5 * t),
                    'shoulder_roll': 0,
                    'elbow': 45,
                    'wrist_pitch': 20 * np.sin(0.8 * t),
                    'wrist_roll': 0,
                }
                # ====================================

                # Read state
                self._motor_states = self.read_state()

                # Check temperatures
                for name, state in self._motor_states.items():
                    self.safety.check_temperature(name, state.temperature)

                # Send commands
                self.send_commands(targets)

                # Logging (every second)
                self._loop_count += 1
                if self._loop_count % self.control_rate == 0:
                    elapsed = time.time() - start_time
                    self._print_status(elapsed, targets)

                # Maintain loop rate
                elapsed = time.time() - loop_start
                sleep_time = self.control_period - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                elif elapsed > self.control_period * 1.5:
                    print(f"[WARNING] Loop overrun: {elapsed*1000:.1f}ms")

        finally:
            self.shutdown()

    def _print_status(self, elapsed: float, targets: Dict[str, float]):
        """Print status line."""
        state = self._motor_states.get('shoulder_pitch', MotorState())
        target = targets.get('shoulder_pitch', 0)
        actual = np.degrees(state.position)
        error = target - actual

        print(f"t={elapsed:5.1f}s | "
              f"shoulder_pitch: target={target:+6.1f}° actual={actual:+6.1f}° error={error:+5.2f}° | "
              f"temp={state.temperature:.0f}°C")

    def shutdown(self):
        """Graceful shutdown."""
        print("\nShutting down...")

        # Stop watchdog if running
        self.safety.stop_watchdog()

        if not self.test_mode:
            # Disable all motors
            print("Disabling motors...")
            for name in self.joint_names:
                motor_id = self.motor_ids[name]
                self.driver.disable(motor_id)

            self.driver.shutdown()

        # Print stats
        stats = self.safety.get_stats()
        print(f"\nSession stats:")
        print(f"  Control loops: {self._loop_count}")
        print(f"  Safety violations: {stats['total_violations']}")

        print("Shutdown complete.")


def main():
    parser = argparse.ArgumentParser(description='K-Bot Arm Controller')
    parser.add_argument('--config', default='config.yaml',
                        help='Path to configuration file')
    parser.add_argument('--test', action='store_true',
                        help='Test mode (no hardware)')
    parser.add_argument('--duration', type=float, default=None,
                        help='Run for N seconds (default: run forever)')
    args = parser.parse_args()

    controller = ArmController(
        config_path=args.config,
        test_mode=args.test
    )

    if not controller.initialize():
        print("Initialization failed!")
        return 1

    controller.run(duration=args.duration)
    return 0


if __name__ == "__main__":
    exit(main())
