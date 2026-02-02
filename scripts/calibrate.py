#!/usr/bin/env python3
"""
Motor Calibration

IMPORTANT: This script does NOT move motors automatically.
Motors only move if you explicitly use --zero with endstop movement.

Usage:
    python scripts/calibrate.py --scan              # Find motors (no movement)
    python scripts/calibrate.py --read              # Read positions (no movement)
    python scripts/calibrate.py --verify            # Check config vs hardware
    python scripts/calibrate.py --set-zero          # Set current position as zero
    python scripts/calibrate.py --set-zero elbow    # Set zero for one joint
    python scripts/calibrate.py --save-offsets      # Save current positions as offsets
    python scripts/calibrate.py --test-direction    # Interactive direction test
"""

import argparse
import sys
import os
import yaml
import json
import numpy as np
from datetime import datetime
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from driver import RobstrideDriver


# Calibration data file (separate from config)
CALIBRATION_FILE = "calibration.yaml"


def load_config(path: str = "config.yaml") -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def save_config(config: dict, path: str = "config.yaml"):
    """Save config back to file."""
    with open(path, 'w') as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)


def load_calibration(path: str = CALIBRATION_FILE) -> dict:
    """Load calibration data."""
    if os.path.exists(path):
        with open(path) as f:
            return yaml.safe_load(f) or {}
    return {}


def save_calibration(data: dict, path: str = CALIBRATION_FILE):
    """Save calibration data."""
    with open(path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False)
    print(f"[Calibration] Saved to {path}")


def scan(driver: RobstrideDriver):
    """
    Scan for motors. Does NOT move anything.
    """
    print("\n" + "=" * 40)
    print("SCANNING FOR MOTORS")
    print("=" * 40)
    print("(This does not move any motors)\n")

    found = driver.scan()

    if not found:
        print("No motors found!")
        print("\nTroubleshooting:")
        print("  1. CAN interface up? Run: ip link show can0")
        print("  2. Power connected to motors?")
        print("  3. CAN wiring correct? Check termination resistors.")
        return []

    print(f"Found {len(found)} motor(s): {found}")
    return found


def read_positions(driver: RobstrideDriver, config: dict):
    """
    Read current positions. Does NOT move anything.
    """
    print("\n" + "=" * 40)
    print("READING MOTOR POSITIONS")
    print("=" * 40)
    print("(This does not move any motors)\n")

    joints = config['joints']

    print(f"{'Joint':<20} {'ID':>4} {'Raw (°)':>10} {'Sign':>5} {'Offset':>8} {'Final (°)':>10}")
    print("-" * 65)

    for name, cfg in joints.items():
        motor_id = cfg['motor_id']
        sign = cfg.get('sign', 1)
        offset = cfg.get('zero_offset', 0.0)

        state = driver.read(motor_id)
        raw_deg = np.degrees(state.position)
        final_deg = sign * raw_deg - offset

        print(f"{name:<20} {motor_id:>4} {raw_deg:>+10.2f} {sign:>+5} {offset:>+8.2f} {final_deg:>+10.2f}")


def verify(driver: RobstrideDriver, config: dict):
    """
    Verify config matches hardware. Does NOT move anything.
    """
    print("\n" + "=" * 40)
    print("VERIFYING CONFIGURATION")
    print("=" * 40)
    print("(This does not move any motors)\n")

    found = driver.scan()
    joints = config['joints']
    expected = {cfg['motor_id']: name for name, cfg in joints.items()}

    all_ok = True

    for motor_id, name in expected.items():
        if motor_id in found:
            state = driver.read(motor_id)
            temp = state.temperature
            pos = np.degrees(state.position)
            print(f"  [OK] {name} (ID {motor_id}): {pos:+.1f}°, {temp:.1f}°C")
        else:
            print(f"  [MISSING] {name} (ID {motor_id})")
            all_ok = False

    unexpected = [m for m in found if m not in expected]
    for motor_id in unexpected:
        print(f"  [UNEXPECTED] Motor ID {motor_id} not in config")
        all_ok = False

    print(f"\n{'OK - All motors found' if all_ok else 'ISSUES FOUND'}")
    return all_ok


def set_zero(driver: RobstrideDriver, config: dict, config_path: str, joint_name: str = None):
    """
    Set current position as zero in motor memory.
    Also saves the offset to config file.

    WARNING: This modifies motor's internal zero point.
    """
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
    print("SET ZERO POSITION")
    print("=" * 40)

    print("\nThis will:")
    print("  1. Set CURRENT position as zero in motor memory")
    print("  2. Update zero_offset in config file")
    print("\nMotors to zero:")

    for name, cfg in to_zero.items():
        state = driver.read(cfg['motor_id'])
        pos = np.degrees(state.position)
        print(f"  {name} (ID {cfg['motor_id']}): currently at {pos:+.2f}°")

    print("\nEnsure the arm is in the REFERENCE POSITION!")

    if input("\nProceed? [y/N]: ").lower() != 'y':
        print("Aborted.")
        return

    # Record current positions before zeroing
    pre_zero_positions = {}
    for name, cfg in to_zero.items():
        state = driver.read(cfg['motor_id'])
        pre_zero_positions[name] = np.degrees(state.position)

    # Set zero in motor memory
    print("\nSetting zero...")
    results = {}
    for name, cfg in to_zero.items():
        motor_id = cfg['motor_id']
        ok = driver.set_zero(motor_id)
        results[name] = ok
        status = "OK" if ok else "FAILED"
        print(f"  {name}: {status}")

    # Verify
    print("\nVerifying...")
    for name, cfg in to_zero.items():
        state = driver.read(cfg['motor_id'])
        pos = np.degrees(state.position)
        print(f"  {name}: {pos:+.4f}° (should be ~0)")

    # Update config with zero offsets
    print("\nUpdating config file...")
    for name in to_zero.keys():
        if results[name]:
            # The offset is the position we zeroed at
            config['joints'][name]['zero_offset'] = 0.0  # Reset since motor is now zeroed

    save_config(config, config_path)
    print(f"Config saved to {config_path}")

    # Save calibration record
    calibration = load_calibration()
    calibration['last_zero'] = {
        'timestamp': datetime.now().isoformat(),
        'joints': {name: pre_zero_positions[name] for name in to_zero.keys()}
    }
    save_calibration(calibration)


def save_offsets(driver: RobstrideDriver, config: dict, config_path: str):
    """
    Save current positions as zero offsets WITHOUT modifying motor memory.

    Use this to calibrate via software offset instead of hardware zero.
    """
    print("\n" + "=" * 40)
    print("SAVE CURRENT POSITIONS AS OFFSETS")
    print("=" * 40)

    print("\nThis will save current positions as software offsets.")
    print("Motor memory will NOT be modified.")
    print("\nCurrent positions:")

    joints = config['joints']
    current_positions = {}

    for name, cfg in joints.items():
        state = driver.read(cfg['motor_id'])
        pos = np.degrees(state.position)
        current_positions[name] = pos
        print(f"  {name}: {pos:+.2f}°")

    print("\nEnsure the arm is in the REFERENCE POSITION!")

    if input("\nSave these as zero offsets? [y/N]: ").lower() != 'y':
        print("Aborted.")
        return

    # Update config
    for name, pos in current_positions.items():
        sign = config['joints'][name].get('sign', 1)
        # Offset = current raw position (so that final = sign * raw - offset = 0)
        config['joints'][name]['zero_offset'] = sign * pos

    save_config(config, config_path)
    print(f"\nOffsets saved to {config_path}")

    # Verify
    print("\nVerification (all should be ~0°):")
    for name, cfg in joints.items():
        state = driver.read(cfg['motor_id'])
        raw = np.degrees(state.position)
        sign = cfg.get('sign', 1)
        offset = cfg.get('zero_offset', 0.0)
        final = sign * raw - offset
        print(f"  {name}: {final:+.4f}°")


def test_direction(driver: RobstrideDriver, config: dict, config_path: str):
    """
    Interactive test to determine correct sign for each joint.

    CAUTION: This WILL move motors!
    """
    print("\n" + "=" * 40)
    print("DIRECTION TEST")
    print("=" * 40)
    print("\nThis test will move each motor slightly to determine")
    print("the correct sign convention.")
    print("\nCAUTION: Motors WILL move!")

    if input("\nProceed? [y/N]: ").lower() != 'y':
        print("Aborted.")
        return

    joints = config['joints']
    kp = config['control']['default_kp']
    kd = config['control']['default_kd']

    for name, cfg in joints.items():
        motor_id = cfg['motor_id']
        current_sign = cfg.get('sign', 1)

        print(f"\n--- Testing {name} (ID {motor_id}) ---")

        # Enable motor
        driver.enable(motor_id)
        import time
        time.sleep(0.3)

        # Read current position
        state = driver.read(motor_id)
        start_pos = state.position

        # Move +5 degrees (in motor space)
        target = start_pos + np.radians(5)
        print(f"Moving motor +5° (positive direction)...")

        for _ in range(50):  # 0.5 seconds
            driver.command(motor_id, target, kp=kp, kd=kd)
            time.sleep(0.01)

        # Ask user which way it moved
        print("\nWhich way did the joint move?")
        print("  1. Positive direction (e.g., shoulder forward, elbow bend)")
        print("  2. Negative direction (opposite)")
        print("  3. Didn't move / unsure")

        choice = input("\nChoice [1/2/3]: ").strip()

        # Return to start
        print("Returning to start position...")
        for _ in range(50):
            driver.command(motor_id, start_pos, kp=kp, kd=kd)
            time.sleep(0.01)

        driver.disable(motor_id)

        # Update sign
        if choice == '1':
            new_sign = 1
        elif choice == '2':
            new_sign = -1
        else:
            new_sign = current_sign
            print(f"Keeping current sign: {current_sign}")
            continue

        if new_sign != current_sign:
            config['joints'][name]['sign'] = new_sign
            print(f"Sign updated: {current_sign} -> {new_sign}")
        else:
            print(f"Sign confirmed: {new_sign}")

    # Save
    if input("\nSave direction settings? [y/N]: ").lower() == 'y':
        save_config(config, config_path)
        print(f"Config saved to {config_path}")


def main():
    parser = argparse.ArgumentParser(
        description='Motor calibration utility',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --scan                 # Find motors (safe, no movement)
  %(prog)s --read                 # Read positions (safe, no movement)
  %(prog)s --verify               # Check config matches hardware
  %(prog)s --set-zero             # Zero all motors at current position
  %(prog)s --set-zero elbow       # Zero specific joint
  %(prog)s --save-offsets         # Save current positions as software offsets
  %(prog)s --test-direction       # Interactive direction test (MOVES MOTORS)
        """
    )

    parser.add_argument('--config', default='config.yaml', help='Config file')
    parser.add_argument('--scan', action='store_true', help='Scan for motors')
    parser.add_argument('--read', action='store_true', help='Read current positions')
    parser.add_argument('--verify', action='store_true', help='Verify config vs hardware')
    parser.add_argument('--set-zero', nargs='?', const='ALL', metavar='JOINT',
                        help='Set current position as zero (optionally specify joint)')
    parser.add_argument('--save-offsets', action='store_true',
                        help='Save current positions as software zero offsets')
    parser.add_argument('--test-direction', action='store_true',
                        help='Interactive direction test (CAUTION: moves motors)')

    args = parser.parse_args()

    # Check for at least one action
    if not any([args.scan, args.read, args.verify, args.set_zero,
                args.save_offsets, args.test_direction]):
        parser.print_help()
        return

    config = load_config(args.config)
    driver = RobstrideDriver(config['can']['interface'])

    if not driver.initialize():
        print("Driver initialization failed!")
        print("\nCheck that CAN interface is up:")
        print("  sudo ip link set can0 type can bitrate 1000000")
        print("  sudo ip link set can0 up")
        return

    try:
        # Safe operations (no motor movement)
        if args.scan:
            scan(driver)

        if args.read:
            read_positions(driver, config)

        if args.verify:
            verify(driver, config)

        # Operations that modify config
        if args.set_zero:
            joint = None if args.set_zero == 'ALL' else args.set_zero
            set_zero(driver, config, args.config, joint)

        if args.save_offsets:
            save_offsets(driver, config, args.config)

        # Operations that move motors
        if args.test_direction:
            test_direction(driver, config, args.config)

    finally:
        driver.shutdown()


if __name__ == "__main__":
    main()
