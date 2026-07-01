# Camera calibration parameters for 4 cameras on ROSIWIT robot.
#
# Camera layout (looking top-down, robot center at origin):
#       front (+x)
#          ^
#          |
#   left<--o-->right
#          |
#       back (-x)
#
# NOTE: "right" camera is at +y (left in base_link) facing +y — the naming
# follows the original URDF convention where camera_right is on the +y side.

import numpy as np

IMG_W = 1280
IMG_H = 720
HFOV = 2.8  # 160 degrees

# Intrinsics from Gazebo camera_info
FX = 110.385
FY = 110.385
CX = 640.5
CY = 360.5

BEV_WIDTH = 800
BEV_HEIGHT = 800
BEV_RESOLUTION = 0.005  # 5mm/pixel, 4m x 4m coverage


def _build_extrinsics(pos, yaw, pitch_down):
    """
    Build camera extrinsics from physical orientation using cross products.
    yaw: 0=+x(front), pi/2=+y, pi=-x(back), -pi/2=-y
    """
    fwd = np.array([
        np.cos(pitch_down) * np.cos(yaw),
        np.cos(pitch_down) * np.sin(yaw),
        -np.sin(pitch_down),
    ])
    world_up = np.array([0, 0, 1])
    right = np.cross(fwd, world_up)
    right /= np.linalg.norm(right)
    down = np.cross(fwd, right)
    R = np.array([right, down, fwd])
    t = R @ np.array(pos, dtype=np.float64)
    return R, t


PITCH = 0.35  # 20° downward tilt

# These MUST match the URDF xacro exactly!
# front:  xyz=(0.21, 0, 0.15),  rpy=(0, 0.35, 0)       → yaw=0
# right:  xyz=(0, 0.21, 0.15),  rpy=(0, 0.35, 1.5708)  → yaw=pi/2
# back:   xyz=(-0.21, 0, 0.15), rpy=(0, 0.35, 3.1416)  → yaw=pi
# left:   xyz=(0, -0.21, 0.15), rpy=(0, 0.35, -1.5708) → yaw=-pi/2

_front_R, _front_t = _build_extrinsics([0.21, 0, 0.15], 0, PITCH)
_right_R, _right_t = _build_extrinsics([0, 0.21, 0.15], np.pi / 2, PITCH)
_back_R, _back_t = _build_extrinsics([-0.21, 0, 0.15], np.pi, PITCH)
_left_R, _left_t = _build_extrinsics([0, -0.21, 0.15], -np.pi / 2, PITCH)

CAMERA_CONFIGS = {
    'front': {
        'position': [0.21, 0, 0.15],
        'R': _front_R, 't': _front_t,
        'topic': '/camera_front/camera/image_raw',
        'info_topic': '/camera_front/camera/camera_info',
    },
    'right': {
        'position': [0, 0.21, 0.15],
        'R': _right_R, 't': _right_t,
        'topic': '/camera_right/camera/image_raw',
        'info_topic': '/camera_right/camera/camera_info',
    },
    'back': {
        'position': [-0.21, 0, 0.15],
        'R': _back_R, 't': _back_t,
        'topic': '/camera_back/camera/image_raw',
        'info_topic': '/camera_back/camera/camera_info',
    },
    'left': {
        'position': [0, -0.21, 0.15],
        'R': _left_R, 't': _left_t,
        'topic': '/camera_left/camera/image_raw',
        'info_topic': '/camera_left/camera/camera_info',
    },
}
