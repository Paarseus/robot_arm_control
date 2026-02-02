#!/usr/bin/env python3
"""
Motor Calibration Script

Use this to:
1. Scan for connected motors
2. Set zero positions
3. Verify motor IDs match config

Usage:
    python scripts/calibrate.py --scan          # Scan for motors
    python scripts/calibrate.py --zero          # Zero all motors at current position
    python scripts/calibrate.py --zero elbow    # Zero specific joint
    python scripts/calibrate.py --verify        # Verify config matches hardware
"""

import argparse
import sys
import os
import yaml
import numpy as np

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from driver import RobstrideDriver


def load_config(path: str = "config.yaml") -> dict:
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def scan_motors(driver: RobstrideDriver):
    """Scan and report all connected motors."""
    print("\nScanning for motors...")
    print("-" * 40)

    found = driver.scan()

    if not found:
        print("No motors found!")
        print("\nTroubleshooting:")
        print("  1. Check CAN interface is up: ip link show can0")
        print("  2. Check power to motors")
        print("  3. Check CAN wiring and termination")
        return

    print(f"\nFound {len(found)} motor(s): {found}")

    print("\nMotor details:")
    for motor_id in found:
        state = driver.read(motor_id)
        print(f"  ID {motor_id}: pos={np.degrees(state.position):+7.2f}° "
              f"temp={state.temperature:.1f}°C")


def zero_motors(driver: RobstrideDriver, config: dict, joint_name: str = None):
    """Set current position as zero for motors."""
    joints = config['joints']

    if joint_name:
        if joint_name not in joints:
            print(f"Unknown joint: {joint_name}")
            print(f"Available: {list(joints.keys())}")
            return
        to_zero = {joint_name: joints[joint_name]}
    else:
        to_zero = joints

    print("\n" + "=" * 50)
    print("ZEROING MOTORS")
    print("=" * 50)
    print("\nWARNING: This will set the CURRENT position as zero.")
    print("Make sure the arm is in the correct reference position!")
    print("\nJoints to zero:")
    for name, cfg in to_zero.items():
        print(f"  - {name} (ID {cfg['motor_id']})")

    response = input("\nProceed? [y/N]: ")
    if response.lower() != 'y':
        print("Aborted.")
        return

    print("\nZeroing...")
    for name, cfg in to_zero.items():
        motor_id = cfg['motor_id']
        if driver.set_zero(motor_id):
            print(f"  {name} (ID {motor_id}): OK")
        else:
            print(f"  {name} (ID {motor_id}): FAILED")

    print("\nVerifying zero positions...")
    for name, cfg in to_zero.items():
        state = driver.read(cfg['motor_id'])
        print(f"  {name}: {np.degrees(state.position):+.4f}°")


def verify_config(driver: RobstrideDriver, config: dict):
    """Verify config matches connected hardware."""
    print("\nVerifying configuration...")
    print("-" * 40)

    found = driver.scan()
    joints = config['joints']
    expected = {cfg['motor_id']: name for name, cfg in joints.items()}

    all_ok = True

    # Check for expected motors
    for motor_id, name in expected.items():
        if motor_id in found:
            state = driver.read(motor_id)
            print(f"  [OK] {name} (ID {motor_id}): temp={state.temperature:.1f}°C")
        else:
            print(f"  [MISSING] {name} (ID {motor_id})")
            all_ok = False

    # Check for unexpected motors
    unexpected = [mid for mid in found if mid not in expected]
    for motor_id in unexpected:
        print(f"  [UNEXPECTED] Unknown motor ID {motor_id}")
        all_ok = False

    if all_ok:
        print("\nConfiguration OK!")
    else:
        print("\nConfiguration has issues - check above.")


def main():
    parser = argparse.ArgumentParser(description='Motor calibration utility')
    parser.add_argument('--config', default='config.yaml', help='Config file path')
    parser.add_argument('--scan', action='store_true', help='Scan for motors')
    parser.add_argument('--zero', nargs='?', const='ALL', help='Zero motors (optionally specify joint)')
    parser.add_argument('--verify', action='store_true', help='Verify config')

    args = parser.parse_args()

    # Need at least one action
    if not (args.scan or args.zero or args.verify):
        parser.print_help()
        return

    config = load_config(args.config)

    driver = RobstrideDriver(
        interface=config['can']['interface'],
        bitrate=config['can']['bitrate']
    )

    if not driver.initialize():
        print("Failed to initialize driver!")
        return

    try:
        if args.scan:
            scan_motors(driver)

        if args.zero:
            joint = None if args.zero == 'ALL' else args.zero
            zero_motors(driver, config, joint)

        if args.verify:
            verify_config(driver, config)

    finally:
        driver.shutdown()


if __name__ == "__main__":
    main()
