# Camera calibration parameters for 4 fisheye cameras on ROSIWIT robot.
# Each camera has known intrinsics (from Gazebo sensor FOV) and extrinsics (from URDF).
#
# Camera layout (looking top-down, robot center at origin):
#       front (+x)
#          ^
#          |
#   left<--o-->right  (+y left)
#          |
#       back (-x)

import numpy as np

IMG_W = 1280
IMG_H = 720

# Horizontal FOV from Gazebo sensor (190 degrees)
HFOV = 3.316  # radians

# Fisheye focal length: fx = fy = width / HFOV
FX = IMG_W / HFOV  # ~386
FY = IMG_H / HFOV  # ~217
CX = IMG_W / 2.0
CY = IMG_H / 2.0

INTRINSICS = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1]], dtype=np.float64)

# Distortion coefficients for ideal fisheye in Gazebo (zero distortion)
DIST_COEFFS = np.zeros((4, 1), dtype=np.float64)

# Camera extrinsics: pose of each camera in base_link frame
# base_link: x-forward, y-left, z-up
# Camera frame: x-right, y-down, z-forward
# Each camera looks radially outward from the robot center

def _compute_extrinsics(xyz, rpy):
    """
    Compute [R|t] matrix from camera pose in base_link.
    R: rotation from base_link to camera frame
    t: camera position in base_link expressed in camera frame
    """
    rx, ry, rz = rpy
    # Rotation around z-axis (yaw)
    Rz = np.array([
        [np.cos(rz), -np.sin(rz), 0],
        [np.sin(rz),  np.cos(rz), 0],
        [0, 0, 1]
    ])
    # Rotation around y-axis (pitch)
    Ry = np.array([
        [np.cos(ry), 0, np.sin(ry)],
        [0, 1, 0],
        [-np.sin(ry), 0, np.cos(ry)]
    ])
    # Rotation around x-axis (roll)
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(rx), -np.sin(rx)],
        [0, np.sin(rx),  np.cos(rx)]
    ])
    R_rpy = Rz @ Ry @ Rx  # world->camera rotation from RPY

    # Camera frame convention: x=right, y=down, z=forward
    # To convert from base_link (x=forward, y=left, z=up) to camera frame:
    # camera_x = base_y (right = left in base)
    # camera_y = -base_z (down = -up)
    # camera_z = base_x (forward = forward)
    R_conv = np.array([
        [0, 1, 0],
        [0, 0, -1],
        [1, 0, 0]
    ])

    R = R_conv @ R_rpy
    t = R @ np.array(xyz, dtype=np.float64)
    return R, t

CAMERA_CONFIGS = {}

# Front camera: position (0.21, 0, 0), facing +x
R_front, t_front = _compute_extrinsics([0.21, 0, 0], [0, 0, 0])
CAMERA_CONFIGS['front'] = {
    'position': [0.21, 0, 0],
    'R': R_front,
    't': t_front,
    'topic': '/camera_front/camera/image_raw',
    'info_topic': '/camera_front/camera/camera_info',
}

# Right camera: position (0, 0.21, 0), facing +y
R_right, t_right = _compute_extrinsics([0, 0.21, 0], [0, 0, np.pi/2])
CAMERA_CONFIGS['right'] = {
    'position': [0, 0.21, 0],
    'R': R_right,
    't': t_right,
    'topic': '/camera_right/camera/image_raw',
    'info_topic': '/camera_right/camera/camera_info',
}

# Back camera: position (-0.21, 0, 0), facing -x
R_back, t_back = _compute_extrinsics([-0.21, 0, 0], [0, 0, np.pi])
CAMERA_CONFIGS['back'] = {
    'position': [-0.21, 0, 0],
    'R': R_back,
    't': t_back,
    'topic': '/camera_back/camera/image_raw',
    'info_topic': '/camera_back/camera/camera_info',
}

# Left camera: position (0, -0.21, 0), facing -y
R_left, t_left = _compute_extrinsics([0, -0.21, 0], [0, 0, -np.pi/2])
CAMERA_CONFIGS['left'] = {
    'position': [0, -0.21, 0],
    'R': R_left,
    't': t_left,
    'topic': '/camera_left/camera/image_raw',
    'info_topic': '/camera_left/camera/camera_info',
}

# Bird's-eye view output settings
BEV_WIDTH = 1200    # pixels
BEV_HEIGHT = 1200   # pixels
BEV_RESOLUTION = 0.005  # meters per pixel (5mm)
BEV_SIZE = BEV_WIDTH * BEV_RESOLUTION  # 6 meters total coverage
