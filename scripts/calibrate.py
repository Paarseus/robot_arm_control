#!/usr/bin/env python3
"""
Motor Calibration

Interactive calibration where YOU move the motors by hand.
The script reads positions and saves to config.

Usage:
    python scripts/calibrate.py                 # Full calibration wizard
    python scripts/calibrate.py --scan          # Just scan for motors
    python scripts/calibrate.py --read          # Just read positions
    python scripts/calibrate.py --joint elbow   # Calibrate one joint
"""

import argparse
import sys
import os
import yaml
import numpy as np
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from driver import RobstrideDriver


def load_config(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def save_config(config: dict, path: str):
    with open(path, 'w') as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)


def scan(driver: RobstrideDriver) -> list:
    """Scan for motors."""
    print("\nScanning...")
    found = driver.scan()
    if found:
        print(f"Found {len(found)} motor(s): {found}")
    else:
        print("No motors found. Check CAN connection.")
    return found


def read_position(driver: RobstrideDriver, motor_id: int) -> float:
    """Read motor position in degrees."""
    state = driver.read(motor_id)
    return np.degrees(state.position)


def read_all(driver: RobstrideDriver, config: dict):
    """Read all motor positions."""
    print("\nCurrent positions:")
    print("-" * 40)
    for name, cfg in config['joints'].items():
        pos = read_position(driver, cfg['motor_id'])
        print(f"  {name:20} (ID {cfg['motor_id']:2}): {pos:+8.2f}°")


def calibrate_joint(driver: RobstrideDriver, name: str, motor_id: int) -> dict:
    """
    Calibrate one joint interactively.

    Returns dict with: min, max, sign, zero_offset
    """
    print(f"\n{'='*50}")
    print(f"Calibrating: {name} (Motor ID {motor_id})")
    print('='*50)

    # Step 1: Find negative limit
    print("\n[Step 1] Move joint to NEGATIVE limit (min position)")
    print("         Then press Enter...")
    input()
    neg_limit = read_position(driver, motor_id)
    print(f"         Recorded: {neg_limit:+.2f}°")

    # Step 2: Find positive limit
    print("\n[Step 2] Move joint to POSITIVE limit (max position)")
    print("         Then press Enter...")
    input()
    pos_limit = read_position(driver, motor_id)
    print(f"         Recorded: {pos_limit:+.2f}°")

    # Determine sign: if pos_limit > neg_limit, sign = 1, else sign = -1
    if pos_limit > neg_limit:
        sign = 1
        min_deg = neg_limit
        max_deg = pos_limit
    else:
        sign = -1
        min_deg = pos_limit
        max_deg = neg_limit

    # Step 3: Set zero position
    print("\n[Step 3] Move joint to ZERO position (home/reference)")
    print("         Then press Enter...")
    input()
    zero_pos = read_position(driver, motor_id)
    print(f"         Recorded: {zero_pos:+.2f}°")

    # Calculate offset so that this position reads as 0
    zero_offset = sign * zero_pos

    # Adjust min/max relative to zero
    min_adjusted = sign * min_deg - zero_offset
    max_adjusted = sign * max_deg - zero_offset

    # Ensure min < max
    final_min = min(min_adjusted, max_adjusted)
    final_max = max(min_adjusted, max_adjusted)

    # Round to clean values
    final_min = round(final_min, 1)
    final_max = round(final_max, 1)
    zero_offset = round(zero_offset, 2)

    print(f"\n         Result:")
    print(f"           sign: {sign:+d}")
    print(f"           min:  {final_min:+.1f}°")
    print(f"           max:  {final_max:+.1f}°")
    print(f"           zero_offset: {zero_offset:+.2f}°")

    return {
        'min': final_min,
        'max': final_max,
        'sign': sign,
        'zero_offset': zero_offset
    }


def calibrate_all(driver: RobstrideDriver, config: dict, config_path: str, joint_filter: str = None):
    """Run calibration for all (or one) joint."""

    joints = config['joints']

    if joint_filter:
        if joint_filter not in joints:
            print(f"Unknown joint: {joint_filter}")
            print(f"Available: {list(joints.keys())}")
            return
        joints = {joint_filter: joints[joint_filter]}

    print("\n" + "="*50)
    print("CALIBRATION")
    print("="*50)
    print("\nFor each joint, you will:")
    print("  1. Move to negative limit (min)")
    print("  2. Move to positive limit (max)")
    print("  3. Move to zero/home position")
    print("\nThe script records positions - motors stay disabled.")

    if input("\nReady? [y/N]: ").lower() != 'y':
        print("Aborted.")
        return

    results = {}

    for name, cfg in joints.items():
        motor_id = cfg['motor_id']
        result = calibrate_joint(driver, name, motor_id)
        results[name] = result

    # Show summary
    print("\n" + "="*50)
    print("SUMMARY")
    print("="*50)
    print(f"\n{'Joint':<20} {'Sign':>5} {'Min':>8} {'Max':>8} {'Offset':>8}")
    print("-"*55)

    for name, r in results.items():
        print(f"{name:<20} {r['sign']:>+5} {r['min']:>+8.1f} {r['max']:>+8.1f} {r['zero_offset']:>+8.2f}")

    # Save to config
    if input("\nSave to config? [y/N]: ").lower() != 'y':
        print("Not saved.")
        return

    for name, r in results.items():
        config['joints'][name]['min'] = r['min']
        config['joints'][name]['max'] = r['max']
        config['joints'][name]['sign'] = r['sign']
        config['joints'][name]['zero_offset'] = r['zero_offset']

    save_config(config, config_path)
    print(f"\nSaved to {config_path}")


def main():
    parser = argparse.ArgumentParser(description='Motor calibration')
    parser.add_argument('--config', default='config.yaml')
    parser.add_argument('--scan', action='store_true', help='Scan for motors only')
    parser.add_argument('--read', action='store_true', help='Read positions only')
    parser.add_argument('--joint', help='Calibrate specific joint only')
    args = parser.parse_args()

    config = load_config(args.config)
    driver = RobstrideDriver(config['can']['interface'])

    if not driver.initialize():
        print("Driver init failed. Check CAN interface.")
        return 1

    try:
        if args.scan:
            scan(driver)
        elif args.read:
            read_all(driver, config)
        else:
            calibrate_all(driver, config, args.config, args.joint)
    finally:
        driver.shutdown()

    return 0


if __name__ == "__main__":
    exit(main())
