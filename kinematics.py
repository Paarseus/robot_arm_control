"""
Forward Kinematics for K-Bot Right Arm

Transform chain derived from kscalelabs/kbot-models kbot/robot.urdf.
Uses URDF joint origins (translation + RPY) and axis-angle rotation
for direct correspondence with the source URDF.

All public functions accept joint angles in degrees (JOINT space).
Returns 4x4 homogeneous transforms: T[:3, 3] = position (meters),
T[:3, :3] = rotation matrix.
"""

import numpy as np
from typing import Dict, List, Union

# Right arm joint chain from kbot URDF (kscalelabs/kbot-models kbot/robot.urdf)
# Each entry: (name, xyz_meters, rpy_radians, axis)
#   xyz: translation from parent link origin
#   rpy: fixed rotation of joint frame (roll, pitch, yaw)
#   axis: rotation axis for joint variable (all Z-axis in kbot)
_JOINT_CHAIN = [
    # Joint 1: dof_right_shoulder_pitch_03
    ("shoulder_pitch", (-0.120999, 0.002025, 0.336659), (0, -np.pi / 2, 0), -1),
    # Joint 2: dof_right_shoulder_roll_03
    ("shoulder_roll", (0, -0.024250, 0.080000), (np.pi / 2, np.pi / 2, 0), 1),
    # Joint 3: dof_right_shoulder_yaw_02
    ("elbow", (-0.013000, -0.142500, -0.031670), (-np.pi / 2, 0, 0), 1),
    # Joint 4: dof_right_elbow_02
    ("wrist_pitch", (0.019000, -0.010000, -0.137000), (-np.pi / 2, 0, np.pi / 2), 1),
    # Joint 5: dof_right_wrist_00
    ("wrist_roll", (0.017350, 0.103000, 0.018000), (-np.pi / 2, np.pi / 2, 0), -1),
]

JOINT_NAMES = [j[0] for j in _JOINT_CHAIN]


def _rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _rpy_matrix(roll, pitch, yaw):
    """RPY (intrinsic XYZ) to 3x3 rotation matrix."""
    return _rz(yaw) @ _ry(pitch) @ _rx(roll)


def _joint_transform(xyz, rpy, axis_sign, angle):
    """Single joint transform: origin(xyz, rpy) @ Rz(axis_sign * angle)."""
    T = np.eye(4)
    T[:3, :3] = _rpy_matrix(*rpy) @ _rz(axis_sign * angle)
    T[:3, 3] = xyz
    return T


def forward_kinematics(joint_angles_deg: Union[Dict[str, float], np.ndarray]) -> np.ndarray:
    """
    Compute end-effector pose from joint angles.

    Args:
        joint_angles_deg: dict {name: degrees} or array of 5 angles
            Order: shoulder_pitch, shoulder_roll, elbow, wrist_pitch, wrist_roll

    Returns:
        4x4 homogeneous transformation matrix (base to end-effector)
    """
    if isinstance(joint_angles_deg, dict):
        angles = [joint_angles_deg[name] for name in JOINT_NAMES]
    else:
        angles = joint_angles_deg

    q = np.radians(angles)
    T = np.eye(4)
    for i, (_, xyz, rpy, axis_sign) in enumerate(_JOINT_CHAIN):
        T = T @ _joint_transform(xyz, rpy, axis_sign, q[i])
    return T


def get_position(joint_angles_deg: Union[Dict[str, float], np.ndarray]) -> np.ndarray:
    """Get end-effector [x, y, z] in meters."""
    return forward_kinematics(joint_angles_deg)[:3, 3]


def get_all_frames(joint_angles_deg: Union[Dict[str, float], np.ndarray]) -> List[np.ndarray]:
    """Get 4x4 transform for each frame (base + 5 joints = 6 frames)."""
    if isinstance(joint_angles_deg, dict):
        angles = [joint_angles_deg[name] for name in JOINT_NAMES]
    else:
        angles = joint_angles_deg

    q = np.radians(angles)
    frames = [np.eye(4)]
    T = np.eye(4)
    for i, (_, xyz, rpy, axis_sign) in enumerate(_JOINT_CHAIN):
        T = T @ _joint_transform(xyz, rpy, axis_sign, q[i])
        frames.append(T.copy())
    return frames


if __name__ == "__main__":
    # Verify FK with all joints at zero
    zeros = np.zeros(5)
    T = forward_kinematics(zeros)
    pos = T[:3, 3]
    print("FK verification (all joints at 0 deg):")
    print(f"  End-effector position: [{pos[0]:+.4f}, {pos[1]:+.4f}, {pos[2]:+.4f}] m")
    print(f"  Transform:\n{T}")

    # Show all joint frame positions
    print("\nJoint frame positions:")
    frames = get_all_frames(zeros)
    names = ["base"] + JOINT_NAMES
    for name, frame in zip(names, frames):
        p = frame[:3, 3]
        print(f"  {name:20s}: [{p[0]:+.4f}, {p[1]:+.4f}, {p[2]:+.4f}]")
