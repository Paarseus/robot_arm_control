# CLAUDE.md

K-Bot arm control: 5-DOF Robstride actuators over CAN bus. Python 3.10+, MIT license.

## Architecture

```
JOINT space (degrees)     MOTOR space (radians)
        │                         ▲
        ▼                         │
    main.py ──── sign/offset ──── driver.py ──── CAN bus (can0, 1 Mbps)
        │                               │
    safety.py (clamps all commands)      │ (MIT mode feedback: pos+vel+torque+temp)
        │                               ▼
    collision.py (torque monitoring, kp scaling)
        │
    kinematics.py (FK: joint angles → end-effector pose)
```

Two coordinate frames, never mix them:
- **JOINT**: degrees, user-facing, respects sign + zero_offset
- **MOTOR**: radians, raw encoder, no transformations

Conversions (`main.py:ArmController`):
```
motor→joint:  joint_deg = degrees(sign * (motor_rad - offset))
joint→motor:  motor_rad = radians(joint_deg) / sign + offset
```

## Files

```
config.yaml              Robot config: motor IDs, joint limits, gains, safety, collision
driver.py                RobstrideDriver (hardware) + MockDriver (test)
safety.py                Position/torque clamping, temperature monitoring
collision.py             Collision detection: torque threshold + position error, kp recovery ramp
kinematics.py            FK: joint angles → end-effector pose (kbot URDF chain)
main.py                  ArmController, 100Hz control loop, CLI
scripts/set_motor_id.py  Assign CAN IDs to motors (interactive)
scripts/calibrate.py        Read-only calibration: zero offsets + joint limits
scripts/auto_calibrate.py   Auto-discover joint limits via collision detection
scripts/test_motors.py      Per-joint motion verification
```

## Motors

| Joint | ID | Range (deg) | Torque (Nm) |
|-------|----|-------------|-------------|
| shoulder_pitch | 21 | -0.6 to +259.4 | 40 |
| shoulder_roll  | 22 | +0.1 to +116.9 | 40 |
| elbow          | 23 |  0.0 to +191.9 | 20 |
| wrist_pitch    | 24 | +0.1 to +143.4 |  6 |
| wrist_roll     | 25 |  0.0 to 0.0    |  6 |

K-Scale ID convention: tens digit = limb, ones digit = joint. New motors ship at ID 127.

## Setup Workflow (sequential, order matters)

### 1. CAN bus
```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

### 2. Set motor IDs
Connect ONE motor at a time (all ship as ID 127 - multiple will conflict).
```bash
python scripts/set_motor_id.py
# scan → find 127 → "127 21" → verified → disconnect → next motor
```
After all 5 have unique IDs, daisy-chain them.

### 3. Calibrate
Read-only, never sends commands to motors.
```bash
python scripts/calibrate.py
```
- Phase 1: Arm at HOME pose, records zero offsets for all joints
- Phase 2: Per-joint, move to negative/positive limits, records min/max, auto-determines sign
- Saves results to `config.yaml`

### 3b. Auto-calibrate limits (alternative to Phase 2)
```bash
python scripts/auto_calibrate.py                       # All joints, hardware
python scripts/auto_calibrate.py --joint elbow          # Single joint
python scripts/auto_calibrate.py --test -y              # Mock mode
python scripts/auto_calibrate.py --speed 5 --margin 10  # Custom speed/margin
```
- Drives each joint in both directions until collision detection triggers
- Records collision positions, applies safety margin, saves min/max to `config.yaml`
- Requires Phase 1 (zero offsets + sign) already complete
- `--torque-scale` (default 2x) controls collision sensitivity during active motion

### 4. Verify
```bash
python scripts/test_motors.py                # all joints
python scripts/test_motors.py --joint elbow  # single joint
python scripts/test_motors.py --amplitude 10 # smaller motion (default 20 deg)
```

### 5. Run
```bash
python main.py                # hardware
python main.py --test         # mock (no hardware needed)
python main.py --duration 30  # time-limited
```

## Libraries

| Library | Used by | Purpose |
|---------|---------|---------|
| `robstride` | `driver.py` | Motor communication: `Client`, `read_param()`, `enable()`, `disable()` |
| `robstride_dynamics` | `driver.py`, `set_motor_id.py` | Bus scanning: `RobstrideBus.scan_channel()` |
| `python-can` | `driver.py`, `set_motor_id.py` | CAN bus: `can.Bus`, raw MIT mode frames |
| `numpy` | all | Radians/degrees, clipping |
| `pyyaml` | all | Config parsing |

## Kinematics

Forward kinematics using transform chain from kbot URDF (`kscalelabs/kbot-models`).
Pure numpy, no extra dependencies. URDF-style transforms (translation + RPY + axis rotation per joint).

```python
from kinematics import forward_kinematics, get_position, get_all_frames

T = forward_kinematics(joint_angles_deg)  # 4x4 homogeneous transform
pos = get_position(joint_angles_deg)      # [x, y, z] meters
frames = get_all_frames(joint_angles_deg) # base + 5 joint frames

# Verify: python kinematics.py
```

Joint chain maps to kbot URDF right arm:
| Our name | URDF joint | URDF axis |
|----------|-----------|-----------|
| shoulder_pitch | dof_right_shoulder_pitch_03 | Z(-1) |
| shoulder_roll | dof_right_shoulder_roll_03 | Z(+1) |
| elbow | dof_right_shoulder_yaw_02 | Z(+1) |
| wrist_pitch | dof_right_elbow_02 | Z(+1) |
| wrist_roll | dof_right_wrist_00 | Z(-1) |

`ArmController.get_end_effector()` returns [x,y,z] from last `read_state()`.
Control loop logs end-effector position every second.

## Control

MIT mode impedance control via raw CAN frames (Communication Type 1):
- `p_des` (rad), `v_des` (rad/s), `kp` (default 20.0), `kd` (default 2.0), `t_ff` (Nm)
- `command()` returns `MotorState` with position, velocity, torque, temperature from feedback
- `MotorState.valid` is `False` when feedback was fabricated (CAN timeout) — torque unreliable
- Position reads use `robstride.Client.read_param(motor_id, "mechpos")` — read-only, no movement

## Collision Detection

`collision.py` monitors torque feedback to detect collisions and react compliantly.

- **Primary detection**: `|torque| > protective_torque` sustained for `protection_time` (30ms default)
- **Backup detection**: `|position_error| > threshold` (15° default) — triggers immediately
- **Reaction**: kp drops to 0 (compliant/damped mode, kd maintained)
- **Recovery**: linear kp ramp from 0→1 over `recovery_time` (1s default)

Config in `config.yaml` under `collision:` section. Per-joint `protective_torque` thresholds.

Control loop order: `read_state → collision.update → send_command(kp*kp_scale) → log`

```bash
# Test collision detection (mock mode only):
python main.py --test --collision-test --duration 10
# Injects 15Nm disturbance on shoulder_pitch at t=3s, clears at t=6s

# Verify collision.py self-tests:
python collision.py
```

## Safety

Never bypass `safety.py`. All commands route through it.
- Position: hard-clamped per joint (degrees in, radians out)
- Torque: clamped per joint
- Temperature: 70C limit
- Watchdog: 50ms timeout
- Collision: torque monitoring + compliant reaction (see above)
- `calibrate.py` is read-only by design

## Code Style

- `black` with line-length 100
- `ruff` with rules E, F, I

## Pitfalls

- ID conflicts: never plug in multiple ID-127 motors at once
- Coordinate frames: driver = MOTOR (radians), everything else = JOINT (degrees)
- Zero offset in config.yaml is degrees; converted to radians at load time
- Sign: +1 = same direction, -1 = inverted. Auto-set during calibration
- CAN must be up before running anything: `ip link show can0`
- Some motors need power cycle after ID change

## K-Scale Conventions

Follows K-Scale patterns: 10s-group motor IDs, 1 Mbps CAN, MIT mode control.
Differs in using software-side zero offsets (config.yaml) instead of firmware-level (`robstride zero`). Software offsets are safer and easier to recalibrate.

Reference: `kscalelabs/kos`, `kscalelabs/actuator`, `kscalelabs/ktune`
