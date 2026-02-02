# Robot Arm Control

Control software for K-Scale K-Bot humanoid robot arm with Robstride actuators.

## Quick Start

```bash
# Install dependencies
pip install -r requirements.txt

# Test mode (no hardware)
python main.py --test

# Run with hardware
python main.py

# Run for 30 seconds
python main.py --duration 30
```

## Project Structure

```
robot_arm_control/
├── config.yaml     # Robot configuration (motor IDs, limits, gains)
├── driver.py       # Low-level motor communication
├── safety.py       # Safety limits and watchdog
├── main.py         # Main control loop
└── README.md
```

## Configuration

Edit `config.yaml` to match your hardware:

```yaml
joints:
  shoulder_pitch:
    motor_id: 11        # CAN ID of motor
    min: -90            # Minimum position (degrees)
    max: 180            # Maximum position (degrees)
    max_torque: 40      # Torque limit (N·m)
```

## Architecture

```
┌──────────────────┐
│     main.py      │  ← Control loop, behaviors
├──────────────────┤
│    safety.py     │  ← Limits enforcement (never bypass)
├──────────────────┤
│    driver.py     │  ← Motor communication
├──────────────────┤
│   config.yaml    │  ← All parameters
└──────────────────┘
```

## Hardware Setup

### CAN Bus
```bash
# Initialize CAN interface
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

### Motor IDs
Default configuration assumes:
- `shoulder_pitch`: ID 11
- `shoulder_roll`: ID 12
- `elbow`: ID 13
- `wrist_pitch`: ID 14
- `wrist_roll`: ID 15

## Adding Your Motor Driver

Edit `driver.py` and uncomment the relevant sections. The driver supports:

1. **Direct python-can**: For custom CAN implementation
2. **robstride library**: `pip install robstride`

## Safety

The safety controller enforces:
- Position limits (clamping)
- Velocity limits
- Torque limits
- Temperature monitoring
- Command watchdog (50ms default)

**Never bypass the safety layer when commanding real hardware.**

## Development

```bash
# Run tests
pytest

# Format code
black *.py

# Lint
ruff check *.py
```

## Next Steps

1. Implement gravity compensation
2. Add Cartesian control (IK)
3. Trajectory generation
4. Policy deployment (RL)

## License

MIT
