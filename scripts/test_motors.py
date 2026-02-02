#!/usr/bin/env python3
"""
Motor Test

Tests each motor with small movements.

Usage:
    python scripts/test_motors.py                 # Test all
    python scripts/test_motors.py --joint elbow   # Test one
    python scripts/test_motors.py --amplitude 10  # Smaller motion
"""

import argparse
import sys
import os
import time
import yaml
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from driver import RobstrideDriver


def load_config(path: str = "config.yaml") -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def test_motor(driver: RobstrideDriver, name: str, motor_id: int,
               config: dict, amplitude: float) -> bool:
    """Test one motor. Returns True if passed."""

    print(f"\n{'='*40}")
    print(f"Testing: {name} (ID {motor_id})")
    print('='*40)

    joint = config['joints'][name]
    kp = config['control']['default_kp']
    kd = config['control']['default_kd']

    # Clamp amplitude to safe range
    safe_amp = min(amplitude, (joint['max'] - joint['min']) / 4)

    # Test 1: Enable
    print("\n[1] Enable...")
    if not driver.enable(motor_id):
        print("  FAIL")
        return False
    print("  OK")
    time.sleep(0.3)

    # Test 2: Read
    print("\n[2] Read state...")
    state = driver.read(motor_id)
    initial = np.degrees(state.position)
    print(f"  Position: {initial:+.2f}°")
    print(f"  Temperature: {state.temperature:.1f}°C")

    # Test 3: Move positive
    target = initial + safe_amp
    print(f"\n[3] Move to {target:+.1f}°...")

    for _ in range(100):
        driver.command(motor_id, np.radians(target), kp=kp, kd=kd)
        time.sleep(0.01)

    state = driver.read(motor_id)
    actual = np.degrees(state.position)
    error = abs(target - actual)
    print(f"  Actual: {actual:+.1f}°, Error: {error:.2f}°")

    pos_ok = error < 5.0
    print(f"  {'OK' if pos_ok else 'FAIL'}")

    # Test 4: Move negative
    target = initial - safe_amp
    print(f"\n[4] Move to {target:+.1f}°...")

    for _ in range(100):
        driver.command(motor_id, np.radians(target), kp=kp, kd=kd)
        time.sleep(0.01)

    state = driver.read(motor_id)
    actual = np.degrees(state.position)
    error = abs(target - actual)
    print(f"  Actual: {actual:+.1f}°, Error: {error:.2f}°")

    neg_ok = error < 5.0
    print(f"  {'OK' if neg_ok else 'FAIL'}")

    # Return to start
    print(f"\n[5] Return to {initial:+.1f}°...")
    for _ in range(100):
        driver.command(motor_id, np.radians(initial), kp=kp, kd=kd)
        time.sleep(0.01)

    # Disable
    print("\n[6] Disable...")
    driver.disable(motor_id)
    print("  OK")

    passed = pos_ok and neg_ok
    print(f"\nResult: {'PASS' if passed else 'FAIL'}")
    return passed


def main():
    parser = argparse.ArgumentParser(description='Motor test')
    parser.add_argument('--config', default='config.yaml')
    parser.add_argument('--joint', help='Test specific joint')
    parser.add_argument('--amplitude', type=float, default=20.0, help='Motion amplitude (deg)')
    args = parser.parse_args()

    config = load_config(args.config)
    joints = config['joints']

    if args.joint:
        if args.joint not in joints:
            print(f"Unknown joint: {args.joint}")
            print(f"Available: {list(joints.keys())}")
            return
        joints = {args.joint: joints[args.joint]}

    driver = RobstrideDriver(config['can']['interface'])

    if not driver.initialize():
        print("Driver initialization failed!")
        return

    print("\n" + "=" * 40)
    print("MOTOR TEST")
    print("=" * 40)
    print(f"Testing {len(joints)} motor(s), amplitude ±{args.amplitude}°")
    print("\nWARNING: Motors will move!")

    input("\nPress Enter to start...")

    results = {}

    try:
        for name, cfg in joints.items():
            results[name] = test_motor(
                driver, name, cfg['motor_id'],
                config, args.amplitude
            )
    finally:
        print("\nDisabling all motors...")
        for cfg in config['joints'].values():
            driver.disable(cfg['motor_id'])
        driver.shutdown()

    # Summary
    print("\n" + "=" * 40)
    print("SUMMARY")
    print("=" * 40)
    for name, passed in results.items():
        print(f"  {name}: {'PASS' if passed else 'FAIL'}")

    total = sum(results.values())
    print(f"\n{total}/{len(results)} passed")


if __name__ == "__main__":
    main()
