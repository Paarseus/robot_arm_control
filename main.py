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

from collision import CollisionDetector, JointCollisionConfig
from driver import RobstrideDriver, MockDriver, MotorState
from kinematics import forward_kinematics, get_position
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

        # Collision detection
        collision_cfg = config.get("collision", {})
        self._collision_enabled = collision_cfg.get("enabled", False)
        if self._collision_enabled:
            defaults = collision_cfg.get("defaults", {})
            joint_overrides = collision_cfg.get("joints", {})
            col_configs = {}
            for name in self.joint_names:
                merged = {**defaults, **joint_overrides.get(name, {})}
                col_configs[name] = JointCollisionConfig(
                    protective_torque=merged.get("protective_torque", 8.0),
                    protection_time=merged.get("protection_time", 0.03),
                    position_error_threshold=merged.get("position_error_threshold", 15.0),
                    recovery_time=merged.get("recovery_time", 1.0),
                )
            self._collision = CollisionDetector(col_configs)
        else:
            self._collision = None

        # State
        self._running = False
        self._states: Dict[str, float] = {}  # Joint positions in degrees
        self._feedback: Dict[str, MotorState] = {}  # Raw feedback per joint
        self._targets: Dict[str, float] = {}  # Last targets for position error
        self._ee_pos: np.ndarray = np.zeros(3)  # End-effector [x,y,z] meters

    def _motor_to_joint(self, name: str, motor_rad: float) -> float:
        """Convert motor radians to joint degrees."""
        sign = self.signs[name]
        offset = self.offsets[name]
        return np.degrees(sign * (motor_rad - offset))

    def _joint_to_motor(self, name: str, joint_deg: float) -> float:
        """Convert joint degrees to motor radians."""
        sign = self.signs[name]
        offset = self.offsets[name]
        # joint_deg = degrees(sign * (motor_rad - offset))
        # radians(joint_deg) = sign * (motor_rad - offset)
        # motor_rad = radians(joint_deg) / sign + offset
        return np.radians(joint_deg) / sign + offset

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

        # Seed mock driver positions at home (joint=0° → motor=offset)
        if isinstance(self.driver, MockDriver):
            for name in self.joint_names:
                self.driver._positions[self.motor_ids[name]] = self.offsets[name]

        print("Ready.")
        return True

    def read_state(self) -> Dict[str, float]:
        """Read joint positions in degrees (JOINT space)."""
        positions = {}
        for name in self.joint_names:
            motor_state = self.driver.read(self.motor_ids[name])
            positions[name] = self._motor_to_joint(name, motor_state.position)
        self._ee_pos = get_position(positions)
        return positions

    def get_end_effector(self) -> np.ndarray:
        """Get end-effector [x, y, z] in meters from last read_state()."""
        return self._ee_pos

    def send_command(self, positions_deg: Dict[str, float], timestamp: float = 0.0):
        """
        Send position commands in degrees (JOINT space).
        Applies safety limits, collision-aware kp scaling, sign, and offset.
        """
        safe_rad = self.safety.clamp_all(positions_deg)
        # Store clamped targets (degrees) for accurate position error in collision detection
        self._targets = {name: np.degrees(rad) for name, rad in safe_rad.items()}

        for name, joint_rad in safe_rad.items():
            joint_deg = np.degrees(joint_rad)
            motor_rad = self._joint_to_motor(name, joint_deg)

            kp = self.kp
            if self._collision is not None:
                kp *= self._collision.get_kp_scale(name, timestamp)

            feedback = self.driver.command(
                motor_id=self.motor_ids[name],
                position=motor_rad,
                kp=kp,
                kd=self.kd,
            )
            if isinstance(feedback, MotorState):
                self._feedback[name] = feedback

    def run(self, duration: float = None, collision_test: bool = False):
        """Run control loop."""
        print(f"\n{'='*50}")
        print("Control loop running (Ctrl+C to stop)")
        if self._collision_enabled:
            print("  Collision detection: ENABLED")
        if collision_test:
            print("  Collision test: inject at t=3s, clear at t=6s")
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

                # Collision test: inject/clear disturbance
                if collision_test and isinstance(self.driver, MockDriver):
                    if 3.0 <= t < 6.0:
                        self.driver.inject_disturbance(self.motor_ids["shoulder_pitch"], 15.0)
                    elif t >= 6.0:
                        self.driver.clear_disturbance(self.motor_ids["shoulder_pitch"])

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

                # Read state
                self._states = self.read_state()

                # Update collision detection from previous cycle's feedback
                # (runs before send_command so kp_scale reflects current detection)
                if self._collision is not None:
                    for name in self.joint_names:
                        fb = self._feedback.get(name)
                        if fb is None or not fb.valid:
                            continue  # Skip: no feedback or fabricated — don't feed 0 torque
                        target_deg = self._targets.get(name, 0.0)
                        actual_deg = self._states.get(name, 0.0)
                        pos_error = target_deg - actual_deg
                        self._collision.update(name, fb.torque, pos_error, t)

                # Send command (uses collision kp_scale from update above)
                self.send_command(targets, timestamp=t)

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
        ee = self._ee_pos

        # Torque from feedback
        fb = self._feedback.get('shoulder_pitch')
        torque_str = f"torque={fb.torque:+5.1f}Nm" if fb else "torque=  N/A"

        # Collision status
        col_str = ""
        if self._collision is not None:
            active = self._collision.get_active_collisions()
            if active:
                col_str = f" | COLLISION: {', '.join(active)}"
            else:
                # Show kp scale during recovery
                scale = self._collision.get_kp_scale("shoulder_pitch", t)
                if scale < 1.0:
                    col_str = f" | recovering kp={scale:.0%}"

        print(
            f"t={t:5.1f}s | target={target:+6.1f}° actual={actual:+6.1f}°"
            f" | {torque_str}"
            f" | ee=[{ee[0]:+.3f}, {ee[1]:+.3f}, {ee[2]:+.3f}]"
            f"{col_str}"
        )

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
    parser.add_argument(
        '--collision-test', action='store_true',
        help='Inject collision at t=3s, clear at t=6s (requires --test)',
    )
    args = parser.parse_args()

    if args.collision_test and not args.test:
        print("[ERROR] --collision-test requires --test mode")
        return 1

    config = load_config(args.config)
    controller = ArmController(config, test_mode=args.test)

    if not controller.initialize():
        return 1

    controller.run(duration=args.duration, collision_test=args.collision_test)
    return 0


if __name__ == "__main__":
    exit(main())
