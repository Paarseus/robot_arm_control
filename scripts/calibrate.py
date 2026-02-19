#!/usr/bin/env python3
"""
Motor Calibration Script (Read-Only)

Records motor positions without sending any commands to the hardware.
The user manually positions each joint and the script reads the encoder values.

Phases:
  Phase 1: Record zero offset for all motors (arm at home pose)
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


def _to_native(obj):
    """Recursively convert numpy types to Python-native types for YAML serialization."""
    if isinstance(obj, np.integer):
        return int(obj)
    if isinstance(obj, np.floating):
        return float(obj)
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, dict):
        return {k: _to_native(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_to_native(v) for v in obj]
    return obj


def save_config(config: dict, path: str):
    with open(path, 'w') as f:
        yaml.dump(_to_native(config), f, default_flow_style=False, sort_keys=False)


def read_position(driver: RobstrideDriver, motor_id: int) -> float:
    """Read motor position in degrees."""
    state = driver.read(motor_id)
    return float(np.degrees(state.position))


def record_zero_offsets(driver: RobstrideDriver, config: dict) -> dict:
    """
    Phase 1: Record zero offsets for all motors (read-only).

    User positions the arm at the home/reference pose,
    then we read each motor's current position and store it as the zero offset.
    No commands are sent to the motors.
    """
    print("\n" + "=" * 50)
    print("PHASE 1: RECORD ZERO OFFSETS")
    print("=" * 50)
    print("\nPosition the arm at the HOME/REFERENCE pose.")
    print("All joints should be at their zero position.")
    print("\nThis will ONLY READ positions, nothing is sent to the motors.")

    if input("\nArm at home position? [y/N]: ").lower() != 'y':
        print("Aborted.")
        return {}

    print("\nReading zero offsets...")
    offsets = {}

    for name, cfg in config['joints'].items():
        motor_id = cfg['motor_id']
        pos = read_position(driver, motor_id)
        offsets[name] = round(pos, 2)
        print(f"  {name:20} (ID {motor_id:2}): {pos:+.2f} deg")

    print("\nAll offsets recorded.")
    return offsets


def record_limits(driver: RobstrideDriver, config: dict, offsets: dict) -> dict:
    """
    Phase 2: Record joint limits for each motor (read-only).

    For each joint:
      1. User moves to negative limit -> read position
      2. User moves to positive limit -> read position
      3. Positions are stored relative to the zero offset from Phase 1

    No commands are sent to the motors.
    Returns dict of {joint_name: {min, max, sign, zero_offset}} for all joints.
    """
    print("\n" + "=" * 50)
    print("PHASE 2: RECORD JOINT LIMITS")
    print("=" * 50)
    print("\nFor each joint, move to the limits when prompted.")
    print("This will ONLY READ positions, nothing is sent to the motors.")

    if input("\nReady to record limits? [y/N]: ").lower() != 'y':
        print("Aborted.")
        return {}

    results = {}

    for name, cfg in config['joints'].items():
        motor_id = cfg['motor_id']
        zero = offsets.get(name, 0.0)

        print(f"\n--- {name} (Motor ID {motor_id}, zero offset: {zero:+.2f}) ---")

        # Record negative limit
        print("\n  Move to NEGATIVE limit, then press Enter...")
        input()
        neg_raw = read_position(driver, motor_id)
        neg_pos = neg_raw - zero
        print(f"  Raw: {neg_raw:+.2f} deg, Relative: {neg_pos:+.2f} deg")

        # Record positive limit
        print("\n  Move to POSITIVE limit, then press Enter...")
        input()
        pos_raw = read_position(driver, motor_id)
        pos_pos = pos_raw - zero
        print(f"  Raw: {pos_raw:+.2f} deg, Relative: {pos_pos:+.2f} deg")

        # Determine sign: positive raw should be > negative raw
        sign = 1 if (pos_raw - neg_raw) > 0 else -1

        # Compute limits in JOINT space: joint_deg = sign * motor_deg - zero
        joint_at_neg = sign * neg_raw - zero
        joint_at_pos = sign * pos_raw - zero
        min_val = round(min(joint_at_neg, joint_at_pos), 1)
        max_val = round(max(joint_at_neg, joint_at_pos), 1)

        print(f"  Result: sign={sign:+d}, min={min_val:+.1f}, max={max_val:+.1f}")

        results[name] = {
            'min': min_val,
            'max': max_val,
            'sign': sign,
            'zero_offset': zero
        }

    return results


def main():
    parser = argparse.ArgumentParser(description='Motor calibration (read-only)')
    parser.add_argument('--config', default='config.yaml', help='Config file path')
    args = parser.parse_args()

    config = load_config(args.config)
    driver = RobstrideDriver(config['can']['interface'])

    if not driver.initialize():
        print("Driver init failed. Check CAN interface.")
        return 1

    try:
        # Phase 1: Record zero offsets (read-only)
        offsets = record_zero_offsets(driver, config)
        if not offsets:
            return 1

        # Phase 2: Record limits (read-only)
        results = record_limits(driver, config, offsets)
        if not results:
            return 1

        # Summary
        print("\n" + "=" * 50)
        print("CALIBRATION SUMMARY")
        print("=" * 50)
        print(f"\n{'Joint':<20} {'Sign':>5} {'Min':>8} {'Max':>8} {'Zero':>8}")
        print("-" * 53)
        for name, r in results.items():
            print(f"{name:<20} {r['sign']:>+5} {r['min']:>+8.1f} {r['max']:>+8.1f} {r['zero_offset']:>+8.2f}")

        # Save to config
        if input("\nSave to config? [y/N]: ").lower() != 'y':
            print("Not saved.")
            return 0

        for name, r in results.items():
            config['joints'][name]['min'] = r['min']
            config['joints'][name]['max'] = r['max']
            config['joints'][name]['sign'] = r['sign']
            config['joints'][name]['zero_offset'] = r['zero_offset']

        save_config(config, args.config)
        print(f"\nSaved to {args.config}")

    finally:
        driver.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
