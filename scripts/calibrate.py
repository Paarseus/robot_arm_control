#!/usr/bin/env python3
"""
Motor Calibration

Usage:
    python scripts/calibrate.py --scan           # Find motors
    python scripts/calibrate.py --zero           # Zero all joints
    python scripts/calibrate.py --zero elbow     # Zero one joint
    python scripts/calibrate.py --verify         # Check config vs hardware
"""

import argparse
import sys
import os
import yaml
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from driver import RobstrideDriver


def load_config(path: str = "config.yaml") -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def scan(driver: RobstrideDriver):
    """Scan for motors."""
    print("\nScanning for motors...")
    found = driver.scan()

    if not found:
        print("No motors found!")
        print("\nCheck:")
        print("  1. CAN interface: ip link show can0")
        print("  2. Power to motors")
        print("  3. CAN wiring and termination")
        return

    print(f"\nFound {len(found)} motor(s): {found}\n")

    for motor_id in found:
        state = driver.read(motor_id)
        print(f"  ID {motor_id}: {np.degrees(state.position):+7.2f}°  {state.temperature:.1f}°C")


def zero(driver: RobstrideDriver, config: dict, joint_name: str = None):
    """Set zero position."""
    joints = config['joints']

    if joint_name:
        if joint_name not in joints:
            print(f"Unknown joint: {joint_name}")
            print(f"Available: {list(joints.keys())}")
            return
        to_zero = {joint_name: joints[joint_name]}
    else:
        to_zero = joints

    print("\n" + "=" * 40)
    print("ZERO MOTORS")
    print("=" * 40)
    print("\nThis sets the CURRENT position as zero.")
    print("Ensure arm is in the reference position!\n")

    for name, cfg in to_zero.items():
        print(f"  {name} (ID {cfg['motor_id']})")

    if input("\nProceed? [y/N]: ").lower() != 'y':
        print("Aborted.")
        return

    print("\nZeroing...")
    for name, cfg in to_zero.items():
        motor_id = cfg['motor_id']
        ok = driver.set_zero(motor_id)
        status = "OK" if ok else "FAILED"
        print(f"  {name}: {status}")

    print("\nVerifying...")
    for name, cfg in to_zero.items():
        state = driver.read(cfg['motor_id'])
        print(f"  {name}: {np.degrees(state.position):+.4f}°")


def verify(driver: RobstrideDriver, config: dict):
    """Verify config matches hardware."""
    print("\nVerifying configuration...")

    found = driver.scan()
    joints = config['joints']
    expected = {cfg['motor_id']: name for name, cfg in joints.items()}

    all_ok = True

    for motor_id, name in expected.items():
        if motor_id in found:
            state = driver.read(motor_id)
            print(f"  [OK] {name} (ID {motor_id}): {state.temperature:.1f}°C")
        else:
            print(f"  [MISSING] {name} (ID {motor_id})")
            all_ok = False

    unexpected = [m for m in found if m not in expected]
    for motor_id in unexpected:
        print(f"  [UNEXPECTED] ID {motor_id}")
        all_ok = False

    print(f"\n{'OK' if all_ok else 'Issues found'}")


def main():
    parser = argparse.ArgumentParser(description='Motor calibration')
    parser.add_argument('--config', default='config.yaml')
    parser.add_argument('--scan', action='store_true', help='Scan for motors')
    parser.add_argument('--zero', nargs='?', const='ALL', help='Zero motors')
    parser.add_argument('--verify', action='store_true', help='Verify config')
    args = parser.parse_args()

    if not (args.scan or args.zero or args.verify):
        parser.print_help()
        return

    config = load_config(args.config)
    driver = RobstrideDriver(config['can']['interface'])

    if not driver.initialize():
        print("Driver initialization failed!")
        return

    try:
        if args.scan:
            scan(driver)
        if args.zero:
            joint = None if args.zero == 'ALL' else args.zero
            zero(driver, config, joint)
        if args.verify:
            verify(driver, config)
    finally:
        driver.shutdown()


if __name__ == "__main__":
    main()
