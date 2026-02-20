#!/usr/bin/env python3
"""
Collision Detection Hardware Test

Tests collision detection on a single motor. Holds position and shows
real-time torque feedback. Push the motor to trigger collision detection.

Usage:
    python scripts/test_collision.py --joint elbow
    python scripts/test_collision.py --joint shoulder_pitch --hold 0
    python scripts/test_collision.py --joint elbow --duration 30
"""

import argparse
import os
import signal
import sys
import time

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from collision import CollisionDetector, JointCollisionConfig
from driver import RobstrideDriver, MotorState


def load_config(path: str = "config.yaml") -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def main():
    parser = argparse.ArgumentParser(description="Collision detection hardware test")
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--joint", required=True, help="Joint to test")
    parser.add_argument("--hold", type=float, default=None, help="Hold position (degrees in MOTOR space). Default: current position")
    parser.add_argument("--duration", type=float, default=20.0, help="Test duration (seconds)")
    args = parser.parse_args()

    config = load_config(args.config)
    joints = config["joints"]

    if args.joint not in joints:
        print(f"Unknown joint: {args.joint}")
        print(f"Available: {list(joints.keys())}")
        return 1

    jcfg = joints[args.joint]
    motor_id = jcfg["motor_id"]
    sign = jcfg.get("sign", 1)
    offset_rad = np.radians(jcfg.get("zero_offset", 0.0))
    kp = config["control"]["default_kp"]
    kd = config["control"]["default_kd"]

    # Build collision config for this joint
    col_cfg = config.get("collision", {})
    defaults = col_cfg.get("defaults", {})
    joint_override = col_cfg.get("joints", {}).get(args.joint, {})
    merged = {**defaults, **joint_override}

    det = CollisionDetector({
        args.joint: JointCollisionConfig(
            protective_torque=merged.get("protective_torque", 8.0),
            protection_time=merged.get("protection_time", 0.03),
            position_error_threshold=merged.get("position_error_threshold", 15.0),
            recovery_time=merged.get("recovery_time", 1.0),
        )
    })

    threshold = merged.get("protective_torque", 8.0)

    print(f"\n{'='*50}")
    print(f"Collision Detection Test: {args.joint}")
    print("=" * 50)
    print(f"  Motor ID: {motor_id}")
    print(f"  Sign: {sign:+d}, Offset: {np.degrees(offset_rad):+.1f}°")
    print(f"  kp: {kp}, kd: {kd}")
    print(f"  Torque threshold: {threshold:.1f} Nm")
    print(f"  Duration: {args.duration}s")

    driver = RobstrideDriver(config["can"]["interface"])
    if not driver.initialize():
        print("[ERROR] Driver init failed")
        return 1

    driver.enable(motor_id)
    time.sleep(0.3)

    # Read initial position
    state = driver.read(motor_id)
    if args.hold is not None:
        hold_motor_rad = np.radians(args.hold)
    else:
        hold_motor_rad = state.position

    hold_joint_deg = np.degrees(sign * (hold_motor_rad - offset_rad))
    print(f"\n  Holding at motor={np.degrees(hold_motor_rad):+.1f}° (joint={hold_joint_deg:+.1f}°)")
    print(f"\n  Push the motor to test collision detection.")
    print(f"  Torque > {threshold:.1f} Nm for 30ms → COLLISION (kp drops to 0)")
    print(f"  Release → recovery ramp over 1s")

    if sys.stdin.isatty():
        input("\nPress Enter to start...")

    print(f"\n{'='*50}")
    print("Running... (Ctrl+C to stop)")
    print(f"{'='*50}\n")

    running = True

    def on_sigint(sig, frame):
        nonlocal running
        print("\n[Stopping...]")
        running = False

    signal.signal(signal.SIGINT, on_sigint)

    start = time.time()
    loop_count = 0
    rate = config["control"]["rate_hz"]
    period = 1.0 / rate

    try:
        while running:
            loop_start = time.time()
            t = loop_start - start

            if t >= args.duration:
                break

            # Get kp scale from collision detector
            kp_scale = det.get_kp_scale(args.joint, t)
            effective_kp = kp * kp_scale

            # Send command — feedback includes position, no separate read needed
            fb = driver.command(
                motor_id=motor_id,
                position=hold_motor_rad,
                kp=effective_kp,
                kd=kd,
            )

            # Compute position error from feedback position
            if fb.valid:
                actual_joint_deg = np.degrees(sign * (fb.position - offset_rad))
                pos_error = hold_joint_deg - actual_joint_deg
                det.update(args.joint, fb.torque, pos_error, t)
            else:
                pos_error = 0.0  # No valid data, don't trigger on garbage

            # Log 10x per second
            loop_count += 1
            if loop_count % (rate // 10) == 0:
                torque_str = f"{fb.torque:+6.2f}" if fb.valid else "  N/A "
                col = det.in_collision(args.joint)

                # Visual torque bar
                bar_width = 30
                if fb.valid:
                    bar_fill = min(bar_width, int(abs(fb.torque) / threshold * bar_width))
                else:
                    bar_fill = 0
                bar = "#" * bar_fill + "-" * (bar_width - bar_fill)

                status = ""
                if col:
                    status = " << COLLISION (kp=0)"
                elif kp_scale < 1.0:
                    status = f" << recovering kp={kp_scale:.0%}"

                print(
                    f"t={t:5.1f}s | torque={torque_str}Nm [{bar}] "
                    f"| pos_err={pos_error:+5.1f}° | kp={effective_kp:5.1f}"
                    f"{status}"
                )

            elapsed = time.time() - loop_start
            if elapsed < period:
                time.sleep(period - elapsed)

    finally:
        print("\nDisabling motor...")
        driver.disable(motor_id)
        driver.shutdown()
        print("Done.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
