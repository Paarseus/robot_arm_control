# K-Bot Arm ROS2 Migration - Step by Step

## Phase 0: Jetson Setup (Do First)

### 0.1 Flash Jetson Orin Nano
1. Download JetPack 6.2 from NVIDIA
2. Flash via SDK Manager to SD card or NVMe
3. Boot, complete Ubuntu setup
4. Set up CAN bus:
   ```bash
   sudo modprobe can
   sudo ip link set can0 type can bitrate 1000000
   sudo ip link set can0 up
   ```

### 0.2 Install ROS2 Humble
```bash
sudo apt update
sudo apt install ros-humble-ros-base
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 0.3 Install Core Packages
```bash
# MoveIt2
sudo apt install ros-humble-moveit*

# ros2_control
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers

# Tools
sudo apt install ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui
sudo apt install python3-colcon-common-extensions python3-vcstool
```

---

## Phase 1: Workspace Setup

### 1.1 Create Workspace
```bash
mkdir -p ~/kbot_ws/src
cd ~/kbot_ws
source /opt/ros/humble/setup.bash
colcon build
```

### 1.2 Workspace Structure
```
kbot_ws/
├── src/
│   ├── kbot_description/           # URDF from K-Scale + ros2_control xacro
│   ├── kbot_bringup/              # Launch files
│   ├── kbot_bridge/               # Python bridge node
│   └── kbot_moveit_config/       # MoveIt config
```

### 1.3 Clone K-Scale URDF
```bash
cd ~/kbot_ws/src
mkdir -p kbot_description/urdf
# Download K-Scale's URDF directly
curl -o kbot_description/urdf/robot.urdf \
  https://raw.githubusercontent.com/kscalelabs/kscale-assets/master/kbot-v1/robot.urdf
```

### 1.4 Create Packages
```bash
cd ~/kbot_ws/src

# Description package (URDF + configs)
ros2 pkg create kbot_description --cmake-v3 --package-format ament_cmake

# Bringup package (launch files)
ros2 pkg create kbot_bringup --cmake-v3 --package-format ament_python

# Bridge package (your Python control stack + ROS interface)
ros2 pkg create kbot_bridge --cmake-v3 --package-format ament_python

# MoveIt config (will generate via MoveIt setup assistant)
# Don't create manually - do via moveit_setup_assistant
```

---

## Phase 2: Robot Description (URDF + Xacro)

### 2.1 Use K-Scale URDF
File: `kbot_description/urdf/robot.urdf`

**Already available!** Downloaded in step 1.3.

This URDF includes:
- Full humanoid (legs + torso + both arms) - 23 DOF
- Physical dimensions (link lengths, masses, inertias)
- STL meshes for visualization
- Collision geometries

**Your 5-DOF right arm chain:**
```
body1_part → right_shoulder_pitch_03 → right_shoulder_roll_03 → right_shoulder_yaw_02 → right_elbow_02 → right_wrist_00
```

### 2.2 Create ros2_control Xacro
File: `kbot_description/urdf/kbot_ros2_control.xacro`

Use official ros2_control hardware interface format:
```xml
<ros2_control name="KbotSystem" type="system">
  <hardware>
    <plugin>kbot_bridge/KbotBridge</plugin>
    <param name="can_interface">can0</param>
  </hardware>
  <!-- Joint definitions matching your config.yaml -->
  <joint name="right_shoulder_pitch_03">
    <param name="motor_id">21</param>
    <param name="min">-1.61</param>  <!-- -92.3 deg → rad -->
    <param name="max">3.88</param>   <!-- +222.4 deg → rad -->
    ...
  </joint>
</ros2_control>
```

**Map your config.yaml to K-Scale URDF joint names:**
| Your config.yaml | K-Scale URDF Joint | Your Limits (deg) |
|-----------------|-------------------|-------------------|
| shoulder_pitch | right_shoulder_pitch_03 | -92.3 to +222.4 |
| shoulder_roll | right_shoulder_roll_03 | -176.1 to +12.7 |
| elbow | right_shoulder_yaw_02 | -2.4 to +177.4 |
| wrist_pitch | right_elbow_02 | -2.4 to +8.7 |
| wrist_roll | right_wrist_00 | -108.1 to -0.1 |

### 2.3 Create Controllers Config
File: `kbot_description/config/kbot_controllers.yaml`

Use official `joint_trajectory_controller`:
```yaml
joint_trajectory_controller:
  ros__parameters:
    joints: [right_shoulder_pitch_03, right_shoulder_roll_03, right_shoulder_yaw_02, right_elbow_02, right_wrist_00]
    command_interfaces: [position]
    state_interfaces: [position, velocity]
```

### 2.4 Test: Visualize in RViz
```bash
cd ~/kbot_ws
source install/setup.bash
ros2 launch kbot_description display.launch.py
```

---

## Phase 3: Bridge Node (Your Python Stack + ROS)

### 3.1 Port Your Existing Code
Copy into `kbot_bridge/kbot_bridge/`:
- `driver.py` → keep as-is (RobstrideDriver)
- `safety.py` → keep as-is  
- `collision.py` → **this is your safety authority**
- `kinematics.py` → keep as-is (FK validator)

### 3.2 Create Bridge Node
File: `kbot_bridge/kbot_bridge/bridge_node.py`

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

class KbotBridge(Node):
    def __init__(self):
        super().__init__('kbot_bridge')
        
        # Your existing imports
        from driver import RobstrideDriver
        from safety import Safety
        from collision import CollisionDetector
        
        # Initialize your stack
        self.driver = RobstrideDriver()
        self.safety = Safety()
        self.collision = CollisionDetector()
        
        # ROS interfaces
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.cmd_sub = self.create_subscription(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory',
            self.cmd_callback, 10)
        
        # Timer for control loop
        self.timer = self.create_timer(0.01, self.control_loop)  # 100Hz
        
    def cmd_callback(self, msg):
        # Store incoming trajectory command
        pass
        
    def control_loop(self):
        # YOUR EXISTING LOOP + ROS PUBLISHING
        # 1. Read motor state
        # 2. Run collision.update() ← YOUR SAFETY AUTHORITY
        # 3. Apply safety.clamp()
        # 4. Send command with kp * kp_scale
        # 5. Publish /joint_states
        pass
```

### 3.3 Add ROS Services
- `/kbot/set_mode` - normal / compliant / e-stop
- `/kbot/enable` - enable/disable motors
- `/kbot/reset_collision` - reset collision state

### 3.4 Update package.xml
```xml
<exec_depend>robstride</exec_depend>
<exec_depend>python3-numpy</exec_depend>
<exec_depend>python3-yaml</exec_depend>
```

---

## Phase 4: MoveIt2 Integration

### 4.1 Generate MoveIt Config
```bash
source /opt/ros/humble/setup.bash
ros2 run moveit_setup_assistant setup_assistant
```

Steps:
1. Select URDF from `kbot_description`
2. Generate self-collision matrix
3. Define planning group: `arm_group` with 5 joints (right_shoulder_pitch_03, right_shoulder_roll_03, right_shoulder_yaw_02, right_elbow_02, right_wrist_00)
4. Add robot poses (home, ready)
5. Configure controllers → use `joint_trajectory_controller`
6. Save to `kbot_moveit_config`

### 4.2 Update Controllers Config
File: `kbot_moveit_config/config/moveit_controllers.yaml`

```yaml
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  arm_controller:
    type: JointTrajectoryController
    joints: [right_shoulder_pitch_03, right_shoulder_roll_03, right_shoulder_yaw_02, right_elbow_02, right_wrist_00]
```

### 4.3 Test: MoveIt RViz
```bash
ros2 launch kbot_moveit_config demo.launch.py
```

---

## Phase 5: Hardware Bring-up (Incremental Testing)

### 5.1 Test 1: CAN + Driver
```bash
# On Jetson
cd ~/kbot_ws
source install/setup.bash
ros2 run kbot_bridge list_motors
```

Should find your 5 motors (IDs 21-25).

### 5.2 Test 2: Joint State Publishing
```bash
ros2 launch kbot_bringup bringup.launch.py
ros2 topic hz /joint_states
```

Verify you see position/velocity/torque at 100Hz.

### 5.3 Test 3: Enable/Disable
```bash
ros2 service call /kbot/enable std_srvs/srv/Trigger
ros2 service call /kbot/set_mode kbot_bridge/srv/SetMode "mode: compliant"
```

Motors should go compliant (draggable).

### 5.4 Test 4: Trajectory Control
```bash
ros2 launch kbot_bringup bringup.launch.py
# In RViz: Plan → Execute
```

### 5.5 Test 5: Collision Detection
```bash
# Push the arm gently - should detect and go compliant
ros2 topic echo /kbot/collision_status
```

---

## Phase 6: Teleop (Optional - Do Later)

### 6.1 Use MoveIt Servo (Official)
```bash
sudo apt install ros-humble-moveit-servo
```

Configure for your joystick:
```yaml
servo:
  command_out_topic: /joint_trajectory_controller/joint_trajectory
  command_in_topic: /delta_joint_cmds
```

### 6.2 Test Teleop
```bash
ros2 launch kbot_bringup teleop.launch.py
```

---

## Summary: What to Use

| Component | Source | Why |
|-----------|--------|-----|
| Robot description | **K-Scale URDF** (kscale-assets) | Full dimensions, masses, meshes |
| ros2_control | Official `joint_trajectory_controller` | Standard |
| MoveIt2 | Official | Standard planning |
| Hardware interface | Your Python `driver.py` | Robstride-specific |
| Safety | Your `collision.py` | More robust than ROS |
| Bridge | Create new | Wraps your stack |
| Teleop | MoveIt Servo (official) | Standard + tested |

---

## Next Steps

1. **Now**: Flash Jetson + install ROS2 Humble
2. **Then**: Create workspace + clone K-Scale URDF
3. **Then**: Create ros2_control xacro wrapper
4. **Then**: Create bridge node with your Python stack
5. **Then**: Generate MoveIt config
6. **Then**: Hardware test incrementally
