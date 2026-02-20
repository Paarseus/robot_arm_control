#!/usr/bin/env python3
"""
Auto-Calibrate Joint Limits via Collision Detection

Drives each joint in both directions until collision, records the physical limits,
and saves them to config.yaml automatically. All motors stay enabled and held
throughout so non-active joints don't flail.

Prerequisite: config.yaml must already have motor_id, sign, zero_offset per joint
(from calibrate.py Phase 1). This script only discovers min/max.

Usage:
    python scripts/auto_calibrate.py                       # All joints, hardware
    python scripts/auto_calibrate.py --joint elbow          # Single joint
    python scripts/auto_calibrate.py --test -y              # Mock mode
    python scripts/auto_calibrate.py --speed 5 --margin 10  # Custom speed/margin
"""

import argparse
import sys
import os
import time
import yaml
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from collision import CollisionDetector, JointCollisionConfig
from driver import RobstrideDriver, MockDriver, MotorState


# Mock hard stops for --test mode (joint-space degrees)
MOCK_LIMITS = {
    "shoulder_pitch": (-100.0, 200.0),
    "shoulder_roll": (-10.0, 100.0),
    "elbow": (-10.0, 170.0),
    "wrist_pitch": (-10.0, 130.0),
    "wrist_roll": (-180.0, 180.0),
}

# Hold gain for non-active joints (enough to resist gravity, not fight hard)
KP_HOLD = 10.0


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
    with open(path, "w") as f:
        yaml.dump(_to_native(config), f, default_flow_style=False, sort_keys=False)


def motor_to_joint(sign, offset_rad, motor_rad):
    return np.degrees(sign * (motor_rad - offset_rad))


def joint_to_motor(sign, offset_rad, joint_deg):
    return np.radians(joint_deg) / sign + offset_rad


def build_collision_detector(config, joint_name, torque_scale=1.0, max_torque=None):
    """Build a single-joint CollisionDetector. Returns (detector, threshold).

    If no collision config exists, derives protective_torque from max_torque
    (10% of max_torque, so threshold scales with joint capability).
    """
    col_cfg = config.get("collision", {})
    defaults = col_cfg.get("defaults", {})
    joint_override = col_cfg.get("joints", {}).get(joint_name, {})
    merged = {**defaults, **joint_override}
    default_torque = max_torque * 0.1 if max_torque and max_torque > 0 else 8.0
    threshold = merged.get("protective_torque", default_torque) * torque_scale
    detector = CollisionDetector({
        joint_name: JointCollisionConfig(
            protective_torque=threshold,
            protection_time=merged.get("protection_time", 0.03),
            position_error_threshold=360.0,  # disabled — torque-only detection
            recovery_time=merged.get("recovery_time", 1.0),
            velocity_threshold=2.0,  # deg/s — stalled if moving < 2 deg/s
            stall_position_error=5.0,  # degrees — must have >5deg error
            stall_time=0.05,  # 50ms sustained
        )
    })
    return detector, threshold


def command_hold_joints(driver, hold_joints, kd):
    """Command all non-active joints to hold. Warns on high temperature."""
    for hj in hold_joints:
        fb = driver.command(hj["motor_id"], hj["hold_rad"], kp=KP_HOLD, kd=kd)
        if fb.valid and fb.temperature > 60.0:
            print(f"  WARNING: motor {hj['motor_id']} temp={fb.temperature:.0f}°C")


def find_limit(
    driver, motor_id, sign, offset_rad, name, config,
    start_deg, direction, max_travel_deg, kp, kd, torque_scale,
    rate, test_mode, max_torque=None, hold_joints=None,
):
    """
    Search in one direction until collision. Holds all other joints each cycle.

    Returns:
        {"joint_deg": float, "torque": float} or None if no collision found.
    """
    dt = 1.0 / rate
    step_deg = 10.0 * dt  # search at 10 deg/s
    target_deg = start_deg + direction * max_travel_deg
    if hold_joints is None:
        hold_joints = []

    # Fresh detector per direction — no stale state
    detector, threshold = build_collision_detector(config, name, torque_scale, max_torque)

    distance = target_deg - start_deg
    n_steps = max(int(abs(distance) / step_deg), 1)

    dir_label = "POSITIVE" if direction > 0 else "NEGATIVE"
    print(f"\n  [{dir_label}] Searching from {start_deg:+.1f}° toward {target_deg:+.1f}°")

    last_actual = start_deg
    last_velocity = 0.0

    for i in range(n_steps + 1):
        frac = 0.5 * (1.0 - np.cos(np.pi * i / n_steps))
        cmd_deg = start_deg + frac * distance
        cmd_rad = joint_to_motor(sign, offset_rad, cmd_deg)

        now = time.time()
        kp_scale = detector.get_kp_scale(name, now)
        effective_kp = kp * kp_scale

        # Mock hard stop simulation
        if test_mode and isinstance(driver, MockDriver) and name in MOCK_LIMITS:
            mock_min, mock_max = MOCK_LIMITS[name]
            if last_actual < mock_min:
                driver.inject_disturbance(motor_id, -15.0)
            elif last_actual > mock_max:
                driver.inject_disturbance(motor_id, 15.0)
            else:
                driver.clear_disturbance(motor_id)

        fb = driver.command(motor_id, cmd_rad, kp=effective_kp, kd=kd)
        command_hold_joints(driver, hold_joints, kd)

        if fb.valid:
            actual = motor_to_joint(sign, offset_rad, fb.position)
            last_actual = actual
            error = cmd_deg - actual
            velocity_dps = np.degrees(fb.velocity)
            last_velocity = velocity_dps
            detector.update(name, fb.torque, error, now, velocity_dps=velocity_dps)
        else:
            actual = last_actual
            error = 0.0

        # Collision detected — record and return
        if detector.in_collision(name):
            collision_pos = last_actual
            collision_torque = fb.torque if fb.valid else 0.0
            print(
                f"  COLLISION at {collision_pos:+.1f}° "
                f"(torque={collision_torque:+.2f}Nm, vel={last_velocity:+.1f}°/s)"
            )

            # Clear mock disturbance
            if test_mode and isinstance(driver, MockDriver):
                driver.clear_disturbance(motor_id)

            # Wait for recovery before returning
            hold_rad = joint_to_motor(sign, offset_rad, collision_pos)
            wait_count = 0
            while True:
                now = time.time()
                kp_scale = detector.get_kp_scale(name, now)
                effective_kp = kp * kp_scale
                fb = driver.command(motor_id, hold_rad, kp=effective_kp, kd=kd)
                command_hold_joints(driver, hold_joints, kd)
                if fb.valid:
                    hold_actual = motor_to_joint(sign, offset_rad, fb.position)
                    last_actual = hold_actual
                    hold_error = collision_pos - hold_actual
                    velocity_dps = np.degrees(fb.velocity)
                    detector.update(name, fb.torque, hold_error, now, velocity_dps=velocity_dps)

                wait_count += 1
                if wait_count % max(rate // 2, 1) == 0:
                    if detector.in_collision(name):
                        state_str = "COLLISION (kp=0)"
                    else:
                        state_str = f"recovering kp={kp_scale:.0%}"
                    torque_str = f"{fb.torque:+.2f}" if fb.valid else "N/A"
                    print(f"    hold | torque={torque_str}Nm | {state_str}")

                if not detector.in_collision(name) and kp_scale >= 1.0:
                    print(f"    Recovery complete")
                    break
                if wait_count > rate * 10:
                    print(f"    WARNING: recovery timeout")
                    break
                time.sleep(dt)

            return {"joint_deg": collision_pos, "torque": collision_torque}

        # Progress logging (~every 2 seconds + first and last)
        if i % (rate * 2) == 0 or i == n_steps:
            torque_str = f"{fb.torque:+.2f}" if fb.valid else " N/A"
            kp_info = f" kp={effective_kp:.2f}" if kp_scale < 1.0 else ""
            print(
                f"    {cmd_deg:+7.1f}° cmd | {actual:+7.1f}° actual | "
                f"torque={torque_str}Nm | vel={last_velocity:+5.1f}°/s{kp_info}"
            )

        time.sleep(dt)

    # Clear mock disturbance
    if test_mode and isinstance(driver, MockDriver):
        driver.clear_disturbance(motor_id)

    print(f"  No collision found in {max_travel_deg}° of travel")
    return None


def ramp_to(driver, motor_id, sign, offset_rad, name, config,
            from_deg, to_deg, speed_dps, kp, kd, torque_scale, rate, test_mode,
            max_torque=None, hold_joints=None):
    """Collision-aware ramp from one position to another. Holds all other joints.
    Returns final position."""
    dt = 1.0 / rate
    step_deg = speed_dps * dt
    distance = to_deg - from_deg
    n_steps = max(int(abs(distance) / step_deg), 1)
    if hold_joints is None:
        hold_joints = []

    detector, _ = build_collision_detector(config, name, torque_scale, max_torque)

    print(f"\n  [RETURN] {from_deg:+.1f}° -> {to_deg:+.1f}° ({abs(distance):.1f}°)")

    last_actual = from_deg

    for i in range(n_steps + 1):
        frac = 0.5 * (1.0 - np.cos(np.pi * i / n_steps))
        cmd_deg = from_deg + frac * distance
        cmd_rad = joint_to_motor(sign, offset_rad, cmd_deg)

        now = time.time()
        kp_scale = detector.get_kp_scale(name, now)
        effective_kp = kp * kp_scale

        fb = driver.command(motor_id, cmd_rad, kp=effective_kp, kd=kd)
        command_hold_joints(driver, hold_joints, kd)

        if fb.valid:
            actual = motor_to_joint(sign, offset_rad, fb.position)
            last_actual = actual
            error = cmd_deg - actual
            velocity_dps = np.degrees(fb.velocity)
            detector.update(name, fb.torque, error, now, velocity_dps=velocity_dps)

        if detector.in_collision(name):
            print(f"  Collision during return at {last_actual:+.1f}° — holding")
            hold_rad = joint_to_motor(sign, offset_rad, last_actual)
            wait_count = 0
            while True:
                now = time.time()
                kp_scale = detector.get_kp_scale(name, now)
                fb = driver.command(motor_id, hold_rad, kp=kp * kp_scale, kd=kd)
                command_hold_joints(driver, hold_joints, kd)
                if fb.valid:
                    last_actual = motor_to_joint(sign, offset_rad, fb.position)
                    velocity_dps = np.degrees(fb.velocity)
                    detector.update(name, fb.torque, 0.0, now, velocity_dps=velocity_dps)
                wait_count += 1
                if not detector.in_collision(name) and kp_scale >= 1.0:
                    break
                if wait_count > rate * 10:
                    break
                time.sleep(dt)
            break

        if i % (rate * 2) == 0 or i == n_steps:
            torque_str = f"{fb.torque:+.2f}" if fb.valid else " N/A"
            print(f"    {cmd_deg:+7.1f}° -> {last_actual:+7.1f}° | torque={torque_str}Nm")

        time.sleep(dt)

    return last_actual


def calibrate_joint(driver, name, cfg, config, speed_dps, margin, max_travel,
                    kp, kd, torque_scale, rate, test_mode, hold_joints):
    """
    Calibrate one joint: search both directions, record collision positions.
    All other joints are held via hold_joints.

    Returns:
        {"min": float, "max": float, "neg_collision": float, "pos_collision": float}
        or None if calibration failed.
    """
    motor_id = cfg["motor_id"]
    sign = cfg.get("sign", 1)
    offset_rad = np.radians(cfg.get("zero_offset", 0.0))
    max_torque = cfg.get("max_torque")

    # Show effective collision threshold
    col_cfg = config.get("collision", {})
    col_defaults = col_cfg.get("defaults", {})
    col_joint = col_cfg.get("joints", {}).get(name, {})
    col_merged = {**col_defaults, **col_joint}
    default_torque = max_torque * 0.1 if max_torque and max_torque > 0 else 8.0
    eff_threshold = col_merged.get("protective_torque", default_torque) * torque_scale

    # Read current position (motor is already enabled)
    state = driver.read(motor_id)
    start_deg = motor_to_joint(sign, offset_rad, state.position)

    print(f"\n{'=' * 60}")
    print(f"  Joint:      {name} (ID {motor_id})")
    print(f"  Sign:       {sign:+d}")
    print(f"  Zero:       {cfg.get('zero_offset', 0.0):+.2f}°")
    print(f"  Speed:      {speed_dps} deg/s (search: 10 deg/s)")
    print(f"  Max travel: {max_travel}° per direction")
    print(f"  Margin:     {margin}° inward from collision")
    print(f"  Kp:         {kp:.1f} (hold: {KP_HOLD:.1f})")
    print(f"  Collision:  {eff_threshold:.1f}Nm threshold")
    print(f"  Start:      {start_deg:+.1f}°")
    print("=" * 60)

    neg_result = None
    pos_result = None

    try:
        # Search NEGATIVE direction
        neg_result = find_limit(
            driver, motor_id, sign, offset_rad, name, config,
            start_deg, direction=-1, max_travel_deg=max_travel,
            kp=kp, kd=kd, torque_scale=torque_scale, rate=rate, test_mode=test_mode,
            max_torque=max_torque, hold_joints=hold_joints,
        )

        # Ramp back to start
        ramp_to(
            driver, motor_id, sign, offset_rad, name, config,
            from_deg=start_deg if neg_result is None else neg_result["joint_deg"],
            to_deg=start_deg,
            speed_dps=speed_dps, kp=kp, kd=kd,
            torque_scale=torque_scale, rate=rate, test_mode=test_mode,
            max_torque=max_torque, hold_joints=hold_joints,
        )
        time.sleep(0.3)

        # Search POSITIVE direction
        pos_result = find_limit(
            driver, motor_id, sign, offset_rad, name, config,
            start_deg, direction=+1, max_travel_deg=max_travel,
            kp=kp, kd=kd, torque_scale=torque_scale, rate=rate, test_mode=test_mode,
            max_torque=max_torque, hold_joints=hold_joints,
        )

        # Ramp back to start
        ramp_to(
            driver, motor_id, sign, offset_rad, name, config,
            from_deg=start_deg if pos_result is None else pos_result["joint_deg"],
            to_deg=start_deg,
            speed_dps=speed_dps, kp=kp, kd=kd,
            torque_scale=torque_scale, rate=rate, test_mode=test_mode,
            max_torque=max_torque, hold_joints=hold_joints,
        )
    except KeyboardInterrupt:
        print(f"\n  {name} interrupted")
        raise

    # Compute results
    if neg_result is None and pos_result is None:
        print(f"\n  {name}: No collisions found in either direction — SKIPPED")
        return None

    neg_col = neg_result["joint_deg"] if neg_result else None
    pos_col = pos_result["joint_deg"] if pos_result else None

    # Determine which collision is the lower/upper bound
    if neg_col is not None and pos_col is not None:
        lower = min(neg_col, pos_col)
        upper = max(neg_col, pos_col)
    elif neg_col is not None:
        lower = neg_col
        upper = None
    else:
        lower = None
        upper = pos_col

    result = {}
    result["neg_collision"] = neg_col
    result["pos_collision"] = pos_col

    if lower is not None:
        result["min"] = round(lower + margin, 1)
    else:
        result["min"] = None

    if upper is not None:
        result["max"] = round(upper - margin, 1)
    else:
        result["max"] = None

    print(f"\n  {name} results:")
    if neg_col is not None:
        print(f"    Negative collision: {neg_col:+.1f}°")
    else:
        print(f"    Negative collision: NOT FOUND")
    if pos_col is not None:
        print(f"    Positive collision: {pos_col:+.1f}°")
    else:
        print(f"    Positive collision: NOT FOUND")
    if result["min"] is not None:
        print(f"    Min (with margin):  {result['min']:+.1f}°")
    if result["max"] is not None:
        print(f"    Max (with margin):  {result['max']:+.1f}°")

    return result


def main():
    parser = argparse.ArgumentParser(
        description="Auto-calibrate joint limits via collision detection"
    )
    parser.add_argument("--config", default="config.yaml", help="Config file path")
    parser.add_argument("--joint", nargs="+", help="Specific joint(s) to calibrate")
    parser.add_argument(
        "--speed", type=float, default=10.0,
        help="Return ramp speed deg/s (default 10)",
    )
    parser.add_argument(
        "--margin", type=float, default=5.0,
        help="Safety margin inward from collision deg (default 5)",
    )
    parser.add_argument(
        "--max-travel", type=float, default=360.0,
        help="Max travel per direction deg (default 360)",
    )
    parser.add_argument(
        "--kp", type=float, default=5.0,
        help="Position gain for active joint (default 5.0)",
    )
    parser.add_argument(
        "--torque-scale", type=float, default=2.0,
        help="Multiply collision torque thresholds (default 2x)",
    )
    parser.add_argument("--test", action="store_true", help="Use mock driver")
    parser.add_argument("-y", "--yes", action="store_true", help="Skip confirmation prompts")
    args = parser.parse_args()

    config = load_config(args.config)
    all_joints = config["joints"]

    # Validate prerequisites
    for name, cfg in all_joints.items():
        if "zero_offset" not in cfg or "sign" not in cfg:
            print(f"ERROR: {name} missing zero_offset or sign in config.")
            print("Run 'python scripts/calibrate.py' first to set up zero offsets.")
            return 1

    # Which joints to calibrate (may be subset)
    if args.joint:
        for j in args.joint:
            if j not in all_joints:
                print(f"Unknown joint: {j}")
                print(f"Available: {list(all_joints.keys())}")
                return 1
        cal_joints = {j: all_joints[j] for j in args.joint}
    else:
        cal_joints = dict(all_joints)

    if args.test:
        driver = MockDriver()
    else:
        driver = RobstrideDriver(config["can"]["interface"])

    if not driver.initialize():
        print("Driver initialization failed!")
        return 1

    rate = config["control"]["rate_hz"]
    kd = config["control"]["default_kd"]

    print("\n" + "=" * 60)
    print("AUTO-CALIBRATE JOINT LIMITS")
    print("=" * 60)
    print(f"  Mode:       {'MOCK' if args.test else 'HARDWARE'}")
    print(f"  Calibrate:  {', '.join(cal_joints.keys())}")
    print(f"  All motors: enabled + held (kp={KP_HOLD:.0f})")
    print(f"  Speed:      {args.speed} deg/s (return ramp)")
    print(f"  Margin:     {args.margin}° inward from collision")
    print(f"  Max travel: {args.max_travel}° per direction")
    print(f"  Kp:         {args.kp:.1f} (active joint)")
    print(f"  Torque:     {args.torque_scale:.1f}x config thresholds")

    if not args.test and not args.yes:
        resp = input("\n  Proceed with auto-calibration? [y/N]: ")
        if resp.lower() != "y":
            print("Aborted.")
            driver.shutdown()
            return 0

    # --- Enable ALL motors and read starting positions ---
    print("\n  Enabling all motors...")
    joint_info = {}  # name -> {motor_id, sign, offset_rad, hold_rad}
    enable_ok = True
    for name, cfg in all_joints.items():
        motor_id = cfg["motor_id"]
        sign = cfg.get("sign", 1)
        offset_rad = np.radians(cfg.get("zero_offset", 0.0))

        if not driver.enable(motor_id):
            print(f"  FAIL: could not enable {name} (ID {motor_id})")
            enable_ok = False
            continue

        # Seed mock driver at zero offset
        if args.test and isinstance(driver, MockDriver):
            driver._positions[motor_id] = offset_rad

        time.sleep(0.1)
        state = driver.read(motor_id)
        hold_rad = state.position
        joint_deg = motor_to_joint(sign, offset_rad, hold_rad)
        joint_info[name] = {
            "motor_id": motor_id,
            "sign": sign,
            "offset_rad": offset_rad,
            "hold_rad": hold_rad,
        }
        print(f"    {name:<20} ID {motor_id}  pos={joint_deg:+.1f}°  [HOLD]")

    if not enable_ok:
        print("\n  WARNING: Some motors failed to enable. Continuing with available motors.")

    time.sleep(0.3)

    results = {}  # name -> calibrate_joint result

    try:
        for name, cfg in cal_joints.items():
            if name not in joint_info:
                print(f"\n  Skipping {name} — not enabled")
                continue

            # Build hold list: all joints EXCEPT the one being calibrated
            hold_joints = [
                {"motor_id": ji["motor_id"], "hold_rad": ji["hold_rad"]}
                for jn, ji in joint_info.items()
                if jn != name
            ]

            try:
                result = calibrate_joint(
                    driver, name, cfg, config,
                    speed_dps=args.speed, margin=args.margin,
                    max_travel=args.max_travel, kp=args.kp, kd=kd,
                    torque_scale=args.torque_scale, rate=rate,
                    test_mode=args.test, hold_joints=hold_joints,
                )
                if result is not None:
                    results[name] = result
                # Refresh hold position to where motor actually is now
                state = driver.read(joint_info[name]["motor_id"])
                joint_info[name]["hold_rad"] = state.position
            except KeyboardInterrupt:
                print(f"\n  Interrupted during {name}")
                break

    except KeyboardInterrupt:
        print("\n\nAborted.")
    finally:
        print("\nDisabling all motors...")
        for jcfg in all_joints.values():
            driver.disable(jcfg["motor_id"])
        driver.shutdown()

    if not results:
        print("\nNo joints calibrated.")
        return 1

    # Summary table
    print("\n" + "=" * 60)
    print("AUTO-CALIBRATION RESULTS")
    print("=" * 60)
    print(f"  {'Joint':<20} {'Neg Collision':>14} {'Pos Collision':>14} {'Min':>9} {'Max':>9}")
    print("  " + "-" * 58)

    for name, r in results.items():
        neg_str = f"{r['neg_collision']:+.1f}°" if r["neg_collision"] is not None else "N/A"
        pos_str = f"{r['pos_collision']:+.1f}°" if r["pos_collision"] is not None else "N/A"
        min_str = f"{r['min']:+.1f}°" if r["min"] is not None else "N/A"
        max_str = f"{r['max']:+.1f}°" if r["max"] is not None else "N/A"
        print(f"  {name:<20} {neg_str:>14} {pos_str:>14} {min_str:>9} {max_str:>9}")

    # Check for partial results
    partial = [n for n, r in results.items() if r["min"] is None or r["max"] is None]
    if partial:
        print(f"\n  WARNING: Partial results for: {', '.join(partial)}")
        print(f"  Only discovered limits will be saved (other limits unchanged).")

    # Save prompt
    if not args.yes:
        resp = input("\n  Save to config? [y/N]: ")
        if resp.lower() != "y":
            print("  Not saved.")
            return 0

    # Update config — only min/max, preserve everything else
    for name, r in results.items():
        if r["min"] is not None:
            config["joints"][name]["min"] = r["min"]
        if r["max"] is not None:
            config["joints"][name]["max"] = r["max"]

    save_config(config, args.config)
    print(f"\n  Saved to {args.config}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
