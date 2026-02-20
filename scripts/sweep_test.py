#!/usr/bin/env python3
"""
Joint Sweep Test with Collision Detection

Moves each joint slowly from min to max to verify calibration and limit safety.
Uses torque-based collision detection to protect against hitting hard stops.
Tests one joint at a time.

Usage:
    python scripts/sweep_test.py --joint wrist_pitch --margin 0 --speed 3
    python scripts/sweep_test.py --joint elbow --margin 0 --speed 3
    python scripts/sweep_test.py --test --margin 0 -y       # Mock mode
    python scripts/sweep_test.py --no-collision              # Tracking error only
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


TRACKING_ERROR_LIMIT = 15.0  # degrees — abort ramp if exceeded (fallback without collision)
LIMIT_PROXIMITY = 5.0  # degrees — collision within this of a limit is "expected limit stop"


def load_config(path: str = "config.yaml") -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def motor_to_joint(sign, offset_rad, motor_rad):
    return np.degrees(sign * (motor_rad - offset_rad))


def joint_to_motor(sign, offset_rad, joint_deg):
    return np.radians(joint_deg) / sign + offset_rad


def build_collision_detector(config, joint_name, torque_scale=1.0):
    """Build a single-joint CollisionDetector from config.yaml. Returns (detector, threshold).

    torque_scale multiplies the protective_torque threshold (>1 = less sensitive,
    useful for active motion where tracking torque is nonzero).
    """
    col_cfg = config.get("collision", {})
    defaults = col_cfg.get("defaults", {})
    joint_override = col_cfg.get("joints", {}).get(joint_name, {})
    merged = {**defaults, **joint_override}
    threshold = merged.get("protective_torque", 8.0) * torque_scale
    detector = CollisionDetector({
        joint_name: JointCollisionConfig(
            protective_torque=threshold,
            protection_time=merged.get("protection_time", 0.03),
            # Disable position error detection for sweep — tracking lag is expected
            # during active motion. Only torque threshold catches real hard stops.
            position_error_threshold=360.0,
            recovery_time=merged.get("recovery_time", 1.0),
        )
    })
    return detector, threshold


def sweep_joint(
    driver, name, cfg, control_cfg, config, speed_dps, margin,
    test_mode, skip_confirm=False, use_collision=True, kp_override=None,
    torque_scale=3.0, cycles=1,
):
    """Sweep one joint: current -> min -> max -> current. Returns (ok, ramp_results)."""
    motor_id = cfg["motor_id"]
    sign = cfg.get("sign", 1)
    offset_rad = np.radians(cfg.get("zero_offset", 0.0))
    j_min = cfg["min"] + margin
    j_max = cfg["max"] - margin
    if kp_override is not None:
        kp = kp_override
    elif use_collision:
        kp = 5.0  # needs enough authority to move against gravity
    else:
        kp = control_cfg["default_kp"] * 0.5
    kd = control_cfg["default_kd"]
    rate = control_cfg["rate_hz"]
    dt = 1.0 / rate

    # Build collision detector if enabled
    detector = None
    threshold = None
    if use_collision:
        detector, threshold = build_collision_detector(config, name, torque_scale)

    # Skip joints with zero or negative range after margin
    if j_max - j_min < 1.0:
        print(f"  Skipping {name}: range too small after {margin}° margin")
        return True, {}

    print(f"\n{'=' * 60}")
    print(f"  Joint:     {name} (ID {motor_id})")
    print(f"  Sign:      {sign:+d}")
    print(f"  Limits:    {cfg['min']:+.1f}° to {cfg['max']:+.1f}°")
    print(f"  Sweep:     {j_min:+.1f}° to {j_max:+.1f}° ({margin}° margin)")
    print(f"  Speed:     {speed_dps} deg/s")
    print(f"  Kp:        {kp:.1f}")
    if detector:
        print(f"  Collision: ON (threshold={threshold:.1f}Nm, {torque_scale:.0f}x config, protection=30ms)")
    else:
        print(f"  Collision: OFF (tracking error only)")
    print("=" * 60)

    # Per-joint confirmation on hardware
    if not test_mode and not skip_confirm:
        resp = input(f"\n  Sweep {name}? [y/N/q(uit)]: ")
        if resp.lower() == "q":
            raise KeyboardInterrupt
        if resp.lower() != "y":
            print(f"  Skipped {name}")
            return True, {}

    # Enable
    if not driver.enable(motor_id):
        print("  FAIL: could not enable motor")
        return False, {}
    time.sleep(0.3)

    # Seed mock driver to home position so joint reads start at ~0°
    if test_mode and isinstance(driver, MockDriver):
        driver._positions[motor_id] = offset_rad

    # Read current position in joint space
    state = driver.read(motor_id)
    start_joint = motor_to_joint(sign, offset_rad, state.position)
    print(f"  Current position: {start_joint:+.1f}°")

    # Degree increment per control step
    step_deg = speed_dps * dt

    ramp_results = {}  # label -> result string
    # Track actual position from command feedback to avoid driver.read() during
    # active control (read_param responses corrupt MIT mode feedback parsing).
    last_actual = [start_joint]

    def ramp_to(target_deg, label):
        """Ramp from current to target. Returns (final_pos, ok, collision_info).

        collision_info is None if no collision, or {'pos': deg, 'torque': Nm} if collision.
        """
        if detector:
            # Use last known feedback position — avoid driver.read() which
            # leaves read_param responses in the CAN buffer that corrupt
            # subsequent MIT mode feedback parsing.
            current = last_actual[0]
        else:
            state = driver.read(motor_id)
            current = motor_to_joint(sign, offset_rad, state.position)
        distance = target_deg - current
        n_steps = max(int(abs(distance) / step_deg), 1)

        print(
            f"\n  [{label}] {current:+.1f}° -> {target_deg:+.1f}° "
            f"({abs(distance):.1f}°, ~{abs(distance) / speed_dps:.1f}s)"
        )

        for i in range(n_steps + 1):
            # Cosine ease-in/ease-out: smooth acceleration and deceleration
            frac = 0.5 * (1.0 - np.cos(np.pi * i / n_steps))
            cmd_deg = current + frac * distance
            cmd_rad = joint_to_motor(sign, offset_rad, cmd_deg)

            if detector:
                # Collision-aware control loop
                now = time.time()
                kp_scale = detector.get_kp_scale(name, now)
                effective_kp = kp * kp_scale

                fb = driver.command(motor_id, cmd_rad, kp=effective_kp, kd=kd)

                if fb.valid:
                    actual = motor_to_joint(sign, offset_rad, fb.position)
                    last_actual[0] = actual
                    error = cmd_deg - actual
                    detector.update(name, fb.torque, error, now)
                else:
                    actual = last_actual[0]
                    error = 0.0

                # Collision triggered — stop ramp, wait for recovery
                if detector.in_collision(name):
                    collision_pos = actual
                    collision_torque = fb.torque if fb.valid else 0.0
                    print(
                        f"  COLLISION at {collision_pos:+.1f}° "
                        f"(torque={collision_torque:+.2f}Nm)"
                    )

                    # Hold position while compliant, wait for recovery
                    hold_rad = joint_to_motor(sign, offset_rad, collision_pos)
                    wait_count = 0
                    while True:
                        now = time.time()
                        kp_scale = detector.get_kp_scale(name, now)
                        effective_kp = kp * kp_scale
                        fb = driver.command(motor_id, hold_rad, kp=effective_kp, kd=kd)
                        if fb.valid:
                            hold_actual = motor_to_joint(sign, offset_rad, fb.position)
                            last_actual[0] = hold_actual
                            hold_error = collision_pos - hold_actual
                            detector.update(name, fb.torque, hold_error, now)

                        wait_count += 1
                        # Log at 2 Hz during wait
                        if wait_count % max(rate // 2, 1) == 0:
                            if detector.in_collision(name):
                                state_str = "COLLISION (kp=0)"
                            else:
                                state_str = f"recovering kp={kp_scale:.0%}"
                            torque_str = f"{fb.torque:+.2f}" if fb.valid else "N/A"
                            print(f"    hold | torque={torque_str}Nm | {state_str}")

                        # Done when fully recovered
                        if not detector.in_collision(name) and kp_scale >= 1.0:
                            print(f"    Recovery complete (kp restored)")
                            break

                        # Safety timeout
                        if wait_count > rate * 10:
                            print(f"    WARNING: recovery timeout after 10s")
                            break

                        time.sleep(dt)

                    collision_info = {"pos": collision_pos, "torque": collision_torque}
                    return last_actual[0], False, collision_info

                # Progress logging (~every 2 seconds + first and last)
                if i % (rate * 2) == 0 or i == n_steps:
                    torque_str = f"{fb.torque:+.2f}" if fb.valid else " N/A"
                    kp_info = f" kp={effective_kp:.2f}" if kp_scale < 1.0 else ""
                    print(
                        f"    {cmd_deg:+7.1f}° cmd | {actual:+7.1f}° actual | "
                        f"torque={torque_str}Nm{kp_info}"
                    )

            else:
                # No collision detection — original tracking error mode
                driver.command(motor_id, cmd_rad, kp=kp, kd=kd)

                if i % (rate // 5) == 0 or i == n_steps:
                    state = driver.read(motor_id)
                    actual = motor_to_joint(sign, offset_rad, state.position)
                    error = abs(cmd_deg - actual)

                    if i % (rate * 2) == 0 or i == n_steps:
                        print(
                            f"    {cmd_deg:+7.1f}° cmd | {actual:+7.1f}° actual | "
                            f"err {error:5.2f}°"
                        )

                    if error > TRACKING_ERROR_LIMIT:
                        print(
                            f"  ABORT: tracking error {error:.1f}° > "
                            f"{TRACKING_ERROR_LIMIT}° limit"
                        )
                        return actual, False, None

            time.sleep(dt)

        # Use feedback position (no driver.read in collision mode)
        error = abs(target_deg - last_actual[0])
        if error > 5.0:
            print(f"  WARNING: final error {error:.1f}° > 5°")
        return last_actual[0], True, None

    ok = True
    try:
        for cycle in range(cycles):
            if cycles > 1:
                print(f"\n  --- Cycle {cycle + 1}/{cycles} ---")

            # Ramp to min
            label_min = "MIN" if cycles == 1 else f"MIN.{cycle + 1}"
            final, ramp_ok, col_info = ramp_to(j_min, label_min)
            if col_info:
                near_min = abs(col_info["pos"] - cfg["min"]) < LIMIT_PROXIMITY
                note = "expected limit stop" if near_min else "unexpected obstacle"
                ramp_results[label_min] = (
                    f"collision at {col_info['pos']:+.1f}° "
                    f"({col_info['torque']:+.2f}Nm) -- {note}"
                )
            elif ramp_ok:
                ramp_results[label_min] = "OK"
            else:
                ramp_results[label_min] = "FAIL (tracking error)"
                ok = False

            # Ramp to max (directly from min — no return to start mid-cycle)
            label_max = "MAX" if cycles == 1 else f"MAX.{cycle + 1}"
            final, ramp_ok, col_info = ramp_to(j_max, label_max)
            if col_info:
                near_max = abs(col_info["pos"] - cfg["max"]) < LIMIT_PROXIMITY
                note = "expected limit stop" if near_max else "unexpected obstacle"
                ramp_results[label_max] = (
                    f"collision at {col_info['pos']:+.1f}° "
                    f"({col_info['torque']:+.2f}Nm) -- {note}"
                )
            elif ramp_ok:
                ramp_results[label_max] = "OK"
            else:
                ramp_results[label_max] = "FAIL (tracking error)"
                ok = False

        time.sleep(0.3)

        # Return to start
        ramp_to(start_joint, "RETURN")

    except KeyboardInterrupt:
        print("\n  Interrupted!")
        try:
            ramp_to(start_joint, "RETURN")
        except KeyboardInterrupt:
            pass
        ok = False

    # Disable
    driver.disable(motor_id)
    time.sleep(0.2)

    state = driver.read(motor_id)
    final = motor_to_joint(sign, offset_rad, state.position)
    print(f"\n  Final: {final:+.1f}° (started at {start_joint:+.1f}°)")

    # Per-ramp report
    for label, result in ramp_results.items():
        print(f"  [{label}] {result}")

    return ok, ramp_results


# --- Parallel multi-joint sweep ---

RAMPING = "ramping"
COLLISION_WAIT = "collision_wait"
HOLDING = "holding"  # done with this ramp, waiting for other joints
DONE = "done"


def _setup_ramp(s, speed_dps, dt):
    """Set up ramp to current target."""
    target, label = s["targets"][s["target_idx"]]
    s["ramp_from"] = s["last_actual"]
    s["ramp_target"] = target
    s["ramp_label"] = label
    s["ramp_distance"] = target - s["last_actual"]
    s["n_steps"] = max(int(abs(s["ramp_distance"]) / (speed_dps * dt)), 1)
    s["step"] = 0
    s["phase"] = RAMPING
    s["wait_count"] = 0


def _rebuild_targets_after_collision(s, name):
    """After first collision, rebuild targets for back-and-forth within reachable range.

    Instead of trying to reach unreachable calibrated limits, the joint oscillates
    between its start position and the collision point (with a small buffer).
    """
    if s["collision_limit"] is not None:
        return  # Already rebuilt on a previous collision
    if s["col_pos"] is None:
        return
    if s["ramp_label"] == "RETURN":
        return  # Don't rebuild on return ramp

    col_pos = s["col_pos"]
    # Buffer back from collision point toward start
    direction = 1 if col_pos > s["start"] else -1
    buffer = 5.0
    far = col_pos - direction * buffer
    # Extend past start to discover the physical limit in the other direction too.
    # Use the full calibrated range so we explore the entire possible travel.
    calibrated_range = abs(s["j_max"] - s["j_min"])
    near = s["start"] - direction * calibrated_range

    # Don't bother if reachable range is too small
    if abs(far - near) < 10.0:
        s["collision_limit"] = far
        return

    s["collision_limit"] = far

    idx = s["target_idx"]
    remaining_count = len(s["targets"]) - idx - 1

    if remaining_count <= 0:
        return

    # Build alternating near/far targets (last one is always RETURN to start)
    new_tail = []
    for i in range(remaining_count - 1):
        if i % 2 == 0:
            new_tail.append((near, f"BACK.{i // 2 + 1}"))
        else:
            new_tail.append((far, f"FWD.{i // 2 + 1}"))
    new_tail.append((s["start"], "RETURN"))

    s["targets"] = s["targets"][: idx + 1] + new_tail
    print(
        f"  [{name}] Rebuilt targets: back-and-forth {near:+.1f}° to {far:+.1f}° "
        f"(collision at {col_pos:+.1f}°)"
    )


def sweep_parallel(
    driver, joints_cfg, control_cfg, config, speed_dps, margin,
    test_mode, use_collision, kp_override, torque_scale, cycles,
):
    """Sweep multiple joints simultaneously, synchronized between ramps."""
    rate = control_cfg["rate_hz"]
    dt = 1.0 / rate

    states = {}
    for name, cfg in joints_cfg.items():
        motor_id = cfg["motor_id"]
        sign = cfg.get("sign", 1)
        offset_rad = np.radians(cfg.get("zero_offset", 0.0))
        j_min = cfg["min"] + margin
        j_max = cfg["max"] - margin

        if j_max - j_min < 1.0:
            print(f"  Skipping {name}: range too small")
            continue

        if kp_override is not None:
            kp = kp_override
        elif use_collision:
            kp = 5.0
        else:
            kp = control_cfg["default_kp"] * 0.5
        kd = control_cfg["default_kd"]

        detector, threshold = None, None
        if use_collision:
            detector, threshold = build_collision_detector(config, name, torque_scale)

        if not driver.enable(motor_id):
            print(f"  FAIL: could not enable {name}")
            continue

        if test_mode and isinstance(driver, MockDriver):
            driver._positions[motor_id] = offset_rad

        state_read = driver.read(motor_id)
        start = motor_to_joint(sign, offset_rad, state_read.position)

        # Build target sequence: [min, max] * cycles + [start]
        targets = []
        for c in range(cycles):
            lmin = "MIN" if cycles == 1 else f"MIN.{c + 1}"
            lmax = "MAX" if cycles == 1 else f"MAX.{c + 1}"
            targets.append((j_min, lmin))
            targets.append((j_max, lmax))
        targets.append((start, "RETURN"))

        first_target, first_label = targets[0]
        distance = first_target - start
        n_steps = max(int(abs(distance) / (speed_dps * dt)), 1)

        states[name] = {
            "motor_id": motor_id, "sign": sign, "offset_rad": offset_rad,
            "j_min": cfg["min"], "j_max": cfg["max"],
            "kp": kp, "kd": kd, "detector": detector,
            "start": start, "last_actual": start,
            "targets": targets, "target_idx": 0,
            "ramp_from": start, "ramp_target": first_target,
            "ramp_label": first_label, "ramp_distance": distance,
            "n_steps": n_steps, "step": 0,
            "phase": RAMPING, "hold_rad": None, "wait_count": 0,
            "collision_limit": None, "col_pos": None,
            "results": {},
        }
        thr_str = f" (threshold={threshold:.1f}Nm)" if threshold else ""
        print(f"  {name:<20} start={start:+.1f}°{thr_str}")

    if not states:
        return {}

    time.sleep(0.3)
    print(f"\n  Running {len(states)} joints synchronized...")
    print(f"  {'=' * 60}")

    log_counter = 0
    log_interval = int(rate * 2)

    try:
        while not all(s["phase"] == DONE for s in states.values()):
            log_counter += 1
            should_log = log_counter % log_interval == 0

            for name, s in states.items():
                if s["phase"] == DONE:
                    continue

                now = time.time()
                cmd_rad = joint_to_motor(s["sign"], s["offset_rad"], s["last_actual"])

                if s["phase"] == RAMPING:
                    frac = 0.5 * (1.0 - np.cos(np.pi * s["step"] / s["n_steps"]))
                    cmd_deg = s["ramp_from"] + frac * s["ramp_distance"]
                    cmd_rad = joint_to_motor(s["sign"], s["offset_rad"], cmd_deg)

                    if s["detector"]:
                        kp_scale = s["detector"].get_kp_scale(name, now)
                        eff_kp = s["kp"] * kp_scale
                        fb = driver.command(s["motor_id"], cmd_rad, kp=eff_kp, kd=s["kd"])

                        if fb.valid:
                            actual = motor_to_joint(s["sign"], s["offset_rad"], fb.position)
                            s["last_actual"] = actual
                            error = cmd_deg - actual
                            s["detector"].update(name, fb.torque, error, now)

                        if s["detector"].in_collision(name):
                            col_pos = s["last_actual"]
                            col_torque = fb.torque if fb.valid else 0.0
                            label = s["ramp_label"]
                            print(
                                f"  [{name}/{label}] COLLISION at {col_pos:+.1f}° "
                                f"({col_torque:+.2f}Nm)"
                            )
                            if label != "RETURN":
                                near = (
                                    abs(col_pos - s["j_min"]) < LIMIT_PROXIMITY
                                    or abs(col_pos - s["j_max"]) < LIMIT_PROXIMITY
                                )
                                note = "expected limit stop" if near else "unexpected obstacle"
                                s["results"][label] = (
                                    f"collision at {col_pos:+.1f}° "
                                    f"({col_torque:+.2f}Nm) -- {note}"
                                )
                            s["phase"] = COLLISION_WAIT
                            s["hold_rad"] = joint_to_motor(
                                s["sign"], s["offset_rad"], col_pos
                            )
                            s["col_pos"] = col_pos
                            s["wait_count"] = 0
                            continue
                    else:
                        driver.command(s["motor_id"], cmd_rad, kp=s["kp"], kd=s["kd"])

                    if should_log:
                        print(
                            f"    {name:<16} [{s['ramp_label']}] "
                            f"{cmd_deg:+7.1f}° -> {s['last_actual']:+7.1f}°"
                        )

                    s["step"] += 1
                    if s["step"] > s["n_steps"]:
                        label = s["ramp_label"]
                        if label != "RETURN" and label not in s["results"]:
                            s["results"][label] = "OK"
                        # Don't advance — go to HOLDING and wait for other joints
                        s["phase"] = HOLDING
                        s["hold_rad"] = joint_to_motor(
                            s["sign"], s["offset_rad"], s["last_actual"]
                        )

                elif s["phase"] == COLLISION_WAIT:
                    if s["detector"]:
                        kp_scale = s["detector"].get_kp_scale(name, now)
                        eff_kp = s["kp"] * kp_scale
                        fb = driver.command(
                            s["motor_id"], s["hold_rad"], kp=eff_kp, kd=s["kd"]
                        )
                        if fb.valid:
                            actual = motor_to_joint(
                                s["sign"], s["offset_rad"], fb.position
                            )
                            s["last_actual"] = actual
                            hold_deg = motor_to_joint(
                                s["sign"], s["offset_rad"], s["hold_rad"]
                            )
                            error = hold_deg - actual
                            s["detector"].update(name, fb.torque, error, now)

                        s["wait_count"] += 1
                        if not s["detector"].in_collision(name) and kp_scale >= 1.0:
                            print(f"  [{name}] Recovery complete")
                            s["phase"] = HOLDING
                            s["hold_rad"] = joint_to_motor(
                                s["sign"], s["offset_rad"], s["last_actual"]
                            )
                            _rebuild_targets_after_collision(s, name)
                        elif s["wait_count"] > rate * 10:
                            print(f"  [{name}] Recovery timeout")
                            s["phase"] = HOLDING
                            s["hold_rad"] = joint_to_motor(
                                s["sign"], s["offset_rad"], s["last_actual"]
                            )
                            _rebuild_targets_after_collision(s, name)

                elif s["phase"] == HOLDING:
                    # Hold position while waiting for other joints to finish
                    if s["detector"]:
                        kp_scale = s["detector"].get_kp_scale(name, now)
                        eff_kp = s["kp"] * kp_scale
                        fb = driver.command(
                            s["motor_id"], s["hold_rad"], kp=eff_kp, kd=s["kd"]
                        )
                        if fb.valid:
                            s["last_actual"] = motor_to_joint(
                                s["sign"], s["offset_rad"], fb.position
                            )
                    else:
                        driver.command(s["motor_id"], s["hold_rad"], kp=s["kp"], kd=s["kd"])

            # Check if ALL active joints are holding — advance together
            active = [s for s in states.values() if s["phase"] != DONE]
            if active and all(s["phase"] == HOLDING for s in active):
                # Advance all to next target
                all_done = True
                for s in active:
                    s["target_idx"] += 1
                    if s["target_idx"] < len(s["targets"]):
                        all_done = False
                        _setup_ramp(s, speed_dps, dt)
                    else:
                        s["phase"] = DONE

                if not all_done:
                    labels = set(s["ramp_label"] for s in active if s["phase"] == RAMPING)
                    print(f"\n  All joints synced -> {', '.join(labels)}")

            time.sleep(dt)

    except KeyboardInterrupt:
        print("\n  Interrupted!")

    # Disable
    for name, s in states.items():
        driver.disable(s["motor_id"])

    # Build results
    all_results = {}
    for name, s in states.items():
        all_results[name] = (True, s["results"])
    return all_results


def main():
    parser = argparse.ArgumentParser(description="Joint sweep test with collision detection")
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--joint", nargs="+", help="Test specific joint(s)")
    parser.add_argument(
        "--speed", type=float, default=3.0, help="Sweep speed deg/s (default 3)"
    )
    parser.add_argument(
        "--margin", type=float, default=5.0, help="Safety margin from limits deg (default 5)"
    )
    parser.add_argument(
        "--kp", type=float, default=None,
        help="Override kp (default: 5.0 with collision, half config without)",
    )
    parser.add_argument(
        "--torque-scale", type=float, default=3.0,
        help="Multiply collision torque thresholds (default 3x for active motion)",
    )
    parser.add_argument(
        "--cycles", type=int, default=1, help="Number of min/max cycles (default 1)"
    )
    parser.add_argument(
        "--no-collision", action="store_true", help="Disable collision detection"
    )
    parser.add_argument("--parallel", action="store_true", help="Run all joints simultaneously")
    parser.add_argument("--test", action="store_true", help="Use mock driver")
    parser.add_argument("-y", "--yes", action="store_true", help="Skip per-joint confirmation")
    args = parser.parse_args()

    config = load_config(args.config)
    joints = config["joints"]
    use_collision = not args.no_collision

    if args.joint:
        for j in args.joint:
            if j not in joints:
                print(f"Unknown joint: {j}")
                print(f"Available: {list(joints.keys())}")
                return 1
        joints = {j: joints[j] for j in args.joint}

    if args.test:
        driver = MockDriver()
    else:
        driver = RobstrideDriver(config["can"]["interface"])

    if not driver.initialize():
        print("Driver initialization failed!")
        return 1

    print("\n" + "=" * 60)
    print("JOINT SWEEP TEST" + (" (with collision detection)" if use_collision else ""))
    print("=" * 60)

    for jname, jcfg in joints.items():
        j_min = jcfg["min"] + args.margin
        j_max = jcfg["max"] - args.margin
        span = j_max - j_min
        if span < 1.0:
            print(f"  {jname:<20} skipped (range too small after margin)")
        else:
            print(f"  {jname:<20} {j_min:+7.1f}° to {j_max:+7.1f}° ({span:.0f}° sweep)")

    if args.kp is not None:
        display_kp = args.kp
    elif use_collision:
        display_kp = 5.0
    else:
        display_kp = config["control"]["default_kp"] * 0.5
    print(f"\n  Speed:     {args.speed} deg/s")
    print(f"  Margin:    {args.margin}° from limits")
    print(f"  Kp:        {display_kp:.1f}")
    print(f"  Mode:      {'MOCK' if args.test else 'HARDWARE'}")
    if use_collision:
        print(f"  Collision: ON (torque monitoring + compliant reaction)")
    else:
        print(f"  Collision: OFF (tracking error {TRACKING_ERROR_LIMIT}° limit)")
    print(f"  Each joint asks for confirmation before moving.")
    print(f"  Press Ctrl+C during any ramp to stop and return.")

    all_results = {}  # joint_name -> (ok, ramp_results)

    try:
        if args.parallel:
            all_results = sweep_parallel(
                driver, joints, config["control"], config, args.speed,
                args.margin, args.test, use_collision, args.kp,
                args.torque_scale, args.cycles,
            )
        else:
            for jname, jcfg in joints.items():
                ok, ramp_results = sweep_joint(
                    driver, jname, jcfg, config["control"], config, args.speed,
                    args.margin, args.test, skip_confirm=args.yes,
                    use_collision=use_collision, kp_override=args.kp,
                    torque_scale=args.torque_scale, cycles=args.cycles,
                )
                all_results[jname] = (ok, ramp_results)
    except KeyboardInterrupt:
        print("\n\nTest aborted.")
    finally:
        print("\nDisabling all motors...")
        for jcfg in config["joints"].values():
            driver.disable(jcfg["motor_id"])
        driver.shutdown()

    # Summary
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    any_fail = False
    for jname, (ok, ramp_results) in all_results.items():
        if not ramp_results:
            print(f"  {jname:<20} skipped")
            continue
        first = True
        for label, result in ramp_results.items():
            if first:
                print(f"  {jname:<20} {label}: {result}")
                first = False
            else:
                print(f"  {'':<20} {label}: {result}")
        if not ok:
            any_fail = True

    tested = sum(1 for _, (_, rr) in all_results.items() if rr)
    print(f"\n  {tested} joint(s) tested")
    return 1 if any_fail else 0


if __name__ == "__main__":
    sys.exit(main())
