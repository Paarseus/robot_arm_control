#!/usr/bin/env python3
"""
Motor Calibration Script

Two-phase calibration:
  Phase 1: Set hardware zero for all motors at once
  Phase 2: Record joint limits one at a time

Usage:
    python scripts/calibrate.py              # Full calibration
    python scripts/calibrate.py --config X   # Use different config file
"""

import argparse
import sys
import os
import yaml
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from driver import RobstrideDriver


def load_config(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def save_config(config: dict, path: str):
    with open(path, 'w') as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)


def read_position(driver: RobstrideDriver, motor_id: int) -> float:
    """Read motor position in degrees."""
    state = driver.read(motor_id)
    return np.degrees(state.position)


def set_zero_all(driver: RobstrideDriver, config: dict) -> bool:
    """
    Phase 1: Set hardware zero for all motors.

    User positions the entire arm at the home/reference pose,
    then we call set_zero() on each motor to save that as zero.
    """
    print("\n" + "=" * 50)
    print("PHASE 1: SET ZERO POSITION")
    print("=" * 50)
    print("\nPosition the arm at the HOME/REFERENCE pose.")
    print("All joints should be at their zero position.")
    print("\nThis will set the hardware zero in each motor's memory.")

    if input("\nArm at home position? [y/N]: ").lower() != 'y':
        print("Aborted.")
        return False

    print("\nSetting zero for all motors...")
    success = True

    for name, cfg in config['joints'].items():
        motor_id = cfg['motor_id']
        if driver.set_zero(motor_id):
            print(f"  {name:20} (ID {motor_id:2}): zero set")
        else:
            print(f"  {name:20} (ID {motor_id:2}): FAILED")
            success = False

    if success:
        print("\nAll motors zeroed successfully.")
    else:
        print("\nSome motors failed to zero. Check connections.")

    return success


def calibrate_limits(driver: RobstrideDriver, config: dict) -> dict:
    """
    Phase 2: Record joint limits for each motor.

    For each joint:
      1. User moves to negative limit -> record position
      2. User moves to positive limit -> record position
      3. Sign is determined from the recorded positions

    Returns dict of {joint_name: {min, max, sign}} for all joints.
    """
    print("\n" + "=" * 50)
    print("PHASE 2: RECORD JOINT LIMITS")
    print("=" * 50)
    print("\nFor each joint, move to the limits when prompted.")
    print("The motor should already be at zero from Phase 1.")

    if input("\nReady to record limits? [y/N]: ").lower() != 'y':
        print("Aborted.")
        return {}

    results = {}

    for name, cfg in config['joints'].items():
        motor_id = cfg['motor_id']

        print(f"\n--- {name} (Motor ID {motor_id}) ---")

        # Record negative limit
        print("\n  Move to NEGATIVE limit, then press Enter...")
        input()
        neg_pos = read_position(driver, motor_id)
        print(f"  Recorded: {neg_pos:+.2f} deg")

        # Record positive limit
        print("\n  Move to POSITIVE limit, then press Enter...")
        input()
        pos_pos = read_position(driver, motor_id)
        print(f"  Recorded: {pos_pos:+.2f} deg")

        # Determine sign: positive limit should be > negative limit
        if pos_pos > neg_pos:
            sign = 1
            min_val = round(neg_pos, 1)
            max_val = round(pos_pos, 1)
        else:
            sign = -1
            min_val = round(pos_pos, 1)
            max_val = round(neg_pos, 1)

        print(f"  Result: sign={sign:+d}, min={min_val:+.1f}, max={max_val:+.1f}")

        results[name] = {
            'min': min_val,
            'max': max_val,
            'sign': sign
        }

    return results


def main():
    parser = argparse.ArgumentParser(description='Motor calibration')
    parser.add_argument('--config', default='config.yaml', help='Config file path')
    args = parser.parse_args()

    config = load_config(args.config)
    driver = RobstrideDriver(config['can']['interface'])

    if not driver.initialize():
        print("Driver init failed. Check CAN interface.")
        return 1

    try:
        # Phase 1: Set zero
        if not set_zero_all(driver, config):
            return 1

        # Phase 2: Record limits
        results = calibrate_limits(driver, config)
        if not results:
            return 1

        # Summary
        print("\n" + "=" * 50)
        print("CALIBRATION SUMMARY")
        print("=" * 50)
        print(f"\n{'Joint':<20} {'Sign':>5} {'Min':>8} {'Max':>8}")
        print("-" * 45)
        for name, r in results.items():
            print(f"{name:<20} {r['sign']:>+5} {r['min']:>+8.1f} {r['max']:>+8.1f}")

        # Save to config
        if input("\nSave to config? [y/N]: ").lower() != 'y':
            print("Not saved.")
            return 0

        for name, r in results.items():
            config['joints'][name]['min'] = r['min']
            config['joints'][name]['max'] = r['max']
            config['joints'][name]['sign'] = r['sign']
            config['joints'][name]['zero_offset'] = 0.0  # Hardware zero used

        save_config(config, args.config)
        print(f"\nSaved to {args.config}")

    finally:
        driver.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
