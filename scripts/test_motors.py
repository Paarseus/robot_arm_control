#!/usr/bin/env python3
"""
Motor Test Script

Tests each motor individually to verify:
1. Enable/disable works
2. Position control works
3. Motor is responsive

Usage:
    python scripts/test_motors.py                # Test all motors
    python scripts/test_motors.py --joint elbow  # Test specific joint
    python scripts/test_motors.py --amplitude 10 # Use smaller motion
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
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def test_motor(driver: RobstrideDriver, name: str, motor_id: int,
               config: dict, amplitude: float = 20.0) -> bool:
    """
    Test a single motor.

    Returns True if all tests pass.
    """
    print(f"\n{'='*50}")
    print(f"Testing: {name} (ID {motor_id})")
    print('='*50)

    joint_cfg = config['joints'][name]
    min_pos = joint_cfg['min']
    max_pos = joint_cfg['max']

    # Clamp amplitude to joint limits
    test_amplitude = min(amplitude, (max_pos - min_pos) / 4)

    results = {}

    # Test 1: Enable
    print("\n[Test 1] Enable motor...")
    if driver.enable(motor_id):
        print("  PASS: Motor enabled")
        results['enable'] = True
    else:
        print("  FAIL: Could not enable motor")
        results['enable'] = False
        return False

    time.sleep(0.5)

    # Test 2: Read state
    print("\n[Test 2] Read motor state...")
    state = driver.read(motor_id)
    print(f"  Position: {np.degrees(state.position):+.2f}°")
    print(f"  Velocity: {np.degrees(state.velocity):+.2f}°/s")
    print(f"  Temperature: {state.temperature:.1f}°C")
    results['read'] = True

    # Test 3: Small position command
    print(f"\n[Test 3] Position control (±{test_amplitude:.0f}°)...")

    initial_pos = np.degrees(state.position)
    kp = config['control']['default_kp']
    kd = config['control']['default_kd']

    # Move positive
    target = initial_pos + test_amplitude
    print(f"  Moving to {target:+.1f}°...")

    for _ in range(100):  # 1 second at 100Hz
        driver.command(motor_id, np.radians(target), kp=kp, kd=kd)
        time.sleep(0.01)

    state = driver.read(motor_id)
    actual = np.degrees(state.position)
    error = abs(target - actual)
    print(f"  Target: {target:+.1f}°, Actual: {actual:+.1f}°, Error: {error:.2f}°")

    if error < 5.0:
        print("  PASS: Position tracking OK")
        results['position_pos'] = True
    else:
        print("  FAIL: Large tracking error")
        results['position_pos'] = False

    # Move negative
    target = initial_pos - test_amplitude
    print(f"  Moving to {target:+.1f}°...")

    for _ in range(100):
        driver.command(motor_id, np.radians(target), kp=kp, kd=kd)
        time.sleep(0.01)

    state = driver.read(motor_id)
    actual = np.degrees(state.position)
    error = abs(target - actual)
    print(f"  Target: {target:+.1f}°, Actual: {actual:+.1f}°, Error: {error:.2f}°")

    if error < 5.0:
        print("  PASS: Position tracking OK")
        results['position_neg'] = True
    else:
        print("  FAIL: Large tracking error")
        results['position_neg'] = False

    # Return to initial
    print(f"  Returning to {initial_pos:+.1f}°...")
    for _ in range(100):
        driver.command(motor_id, np.radians(initial_pos), kp=kp, kd=kd)
        time.sleep(0.01)

    # Test 4: Disable
    print("\n[Test 4] Disable motor...")
    if driver.disable(motor_id):
        print("  PASS: Motor disabled")
        results['disable'] = True
    else:
        print("  FAIL: Could not disable motor")
        results['disable'] = False

    # Summary
    print(f"\n--- Results for {name} ---")
    passed = sum(results.values())
    total = len(results)
    print(f"  Passed: {passed}/{total}")

    return passed == total


def main():
    parser = argparse.ArgumentParser(description='Motor test utility')
    parser.add_argument('--config', default='config.yaml', help='Config file')
    parser.add_argument('--joint', help='Test specific joint only')
    parser.add_argument('--amplitude', type=float, default=20.0,
                        help='Test motion amplitude in degrees')

    args = parser.parse_args()

    config = load_config(args.config)

    driver = RobstrideDriver(
        interface=config['can']['interface'],
        bitrate=config['can']['bitrate']
    )

    if not driver.initialize():
        print("Failed to initialize driver!")
        return

    joints = config['joints']

    if args.joint:
        if args.joint not in joints:
            print(f"Unknown joint: {args.joint}")
            print(f"Available: {list(joints.keys())}")
            return
        joints = {args.joint: joints[args.joint]}

    print("\n" + "=" * 50)
    print("MOTOR TEST SEQUENCE")
    print("=" * 50)
    print(f"Testing {len(joints)} motor(s)")
    print(f"Amplitude: ±{args.amplitude}°")
    print("\nWARNING: Motors will move! Ensure arm is safe.")

    input("\nPress Enter to start...")

    results = {}

    try:
        for name, cfg in joints.items():
            results[name] = test_motor(
                driver, name, cfg['motor_id'],
                config, args.amplitude
            )

    finally:
        # Ensure all motors are disabled
        print("\nDisabling all motors...")
        for name, cfg in config['joints'].items():
            driver.disable(cfg['motor_id'])
        driver.shutdown()

    # Final summary
    print("\n" + "=" * 50)
    print("TEST SUMMARY")
    print("=" * 50)
    for name, passed in results.items():
        status = "PASS" if passed else "FAIL"
        print(f"  {name}: {status}")

    total_pass = sum(results.values())
    total = len(results)
    print(f"\nOverall: {total_pass}/{total} motors passed")


if __name__ == "__main__":
    main()
