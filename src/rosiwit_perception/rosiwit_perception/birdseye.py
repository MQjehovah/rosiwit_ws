import numpy as np
import cv2
from scipy.spatial.transform import Rotation as Rot
from .config import (
    IMG_W, IMG_H, FX, FY, CX, CY,
    CAMERA_CONFIGS,
    BEV_WIDTH, BEV_HEIGHT, BEV_RESOLUTION,
)

# Fixed transform: camera_link frame (x=fwd, y=left, z=up) -> optical frame (x=right, y=down, z=fwd)
# optical_x (right) = -link_y    (right = -left)
# optical_y (down)  = -link_z    (down = -up)
# optical_z (fwd)   = +link_x    (fwd = fwd)
_R_LINK_TO_OPTICAL = np.array([
    [0, -1,  0],
    [0,  0, -1],
    [1,  0,  0],
], dtype=np.float64)


def _pinhole_project(world_points, R, t, fx, fy, cx, cy):
    """
    Project world points (N, 3) to pinhole image coordinates.
    Returns: (u, v, valid) where valid is a boolean mask
    """
    pos = R.T @ t
    P_cam = (R @ (world_points - pos).T).T

    x, y, z = P_cam[:, 0], P_cam[:, 1], P_cam[:, 2]
    valid = z > 1e-6

    u = np.full_like(x, -1, dtype=np.float32)
    v = np.full_like(y, -1, dtype=np.float32)

    mask = valid
    u[mask] = fx * x[mask] / z[mask] + cx
    v[mask] = fy * y[mask] / z[mask] + cy

    in_bounds = valid & (u >= 0) & (u < IMG_W - 1) & (v >= 0) & (v < IMG_H - 1)
    return u, v, in_bounds


def precompute_maps():
    return precompute_maps_with_intrinsics(FX, FY, CX, CY)


def precompute_maps_with_intrinsics(fx, fy, cx, cy):
    """
    Precompute BEV remap for all cameras.
    BEV coordinate system:
      - Top of image = front (+x in base_link)
      - Right of image = right (-y in base_link)
    """
    maps = {}

    half_w = BEV_WIDTH / 2.0
    half_h = BEV_HEIGHT / 2.0

    # Build world coordinate grid for each BEV pixel
    # u (col, right) → world_y = (half_w - u) * res   [right = -y_base]
    # v (row, down)  → world_x = (half_h - v) * res   [down = -x_base = back]
    u_idx = np.arange(BEV_WIDTH, dtype=np.float32)
    v_idx = np.arange(BEV_HEIGHT, dtype=np.float32)
    uu, vv = np.meshgrid(u_idx, v_idx)

    world_x = (half_h - vv) * BEV_RESOLUTION   # +x = front
    world_y = (half_w - uu) * BEV_RESOLUTION   # +y = left
    world_z = np.full_like(world_x, -0.11)  # ground is 0.11m below base_link origin

    world_points = np.stack([world_x, world_y, world_z], axis=-1).reshape(-1, 3)

    for cam_name, cfg in CAMERA_CONFIGS.items():
        R = cfg['R']
        t = cfg['t']

        u, v, valid = _pinhole_project(world_points, R, t, fx, fy, cx, cy)

        map_x = u.reshape(BEV_HEIGHT, BEV_WIDTH).astype(np.float32)
        map_y = v.reshape(BEV_HEIGHT, BEV_WIDTH).astype(np.float32)
        mask = valid.reshape(BEV_HEIGHT, BEV_WIDTH).astype(np.uint8) * 255

        # Blend weights: inverse angular distance from camera center ray
        pos = R.T @ t
        center_dir = R.T @ np.array([0, 0, 1], dtype=np.float64)
        center_dir = center_dir / np.linalg.norm(center_dir)

        pts = world_points.reshape(BEV_HEIGHT, BEV_WIDTH, 3)
        to_pt = pts - pos
        norms = np.linalg.norm(to_pt, axis=2)
        dirs = to_pt / (norms[:, :, np.newaxis] + 1e-10)

        dot = np.sum(dirs * center_dir, axis=2)
        angle = np.arccos(np.clip(dot, -1, 1))
        weight = np.where(mask > 0, 1.0 / (angle + 0.01), 0.0)
        weight = weight.astype(np.float32)

        valid_px = np.sum(mask > 0)
        print(f'  {cam_name}: {valid_px} valid BEV pixels ({100*valid_px/(BEV_WIDTH*BEV_HEIGHT):.1f}%)')

        maps[cam_name] = {
            'map_x': map_x,
            'map_y': map_y,
            'mask': mask,
            'weight': weight,
        }

    print(f'AVM maps ready: {len(maps)} cameras')
    return maps


def precompute_maps_from_tf(tf_transforms, fx, fy, cx, cy):
    """
    Build BEV maps using exact TF transforms from Gazebo.
    tf_transforms: dict of frame_name -> geometry_msgs/Transform
    """
    maps = {}
    cam_map = {
        'front_camera_link': 'front',
        'right_camera_link': 'right',
        'back_camera_link': 'back',
        'left_camera_link': 'left',
    }

    half_w = BEV_WIDTH / 2.0
    half_h = BEV_HEIGHT / 2.0
    u_idx = np.arange(BEV_WIDTH, dtype=np.float32)
    v_idx = np.arange(BEV_HEIGHT, dtype=np.float32)
    uu, vv = np.meshgrid(u_idx, v_idx)
    world_x = (half_h - vv) * BEV_RESOLUTION
    world_y = (half_w - uu) * BEV_RESOLUTION
    world_z = np.full_like(world_x, -0.11)  # ground is 0.11m below base_link origin
    world_points = np.stack([world_x, world_y, world_z], axis=-1).reshape(-1, 3)

    for frame, cam_name in cam_map.items():
        tf = tf_transforms[frame]
        # Translation
        tx = tf.translation.x
        ty = tf.translation.y
        tz = tf.translation.z

        # Rotation quaternion -> matrix (base_link -> camera_link)
        q = [tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w]
        R_link = Rot.from_quat(q).as_matrix()  # transforms base_link vecs to camera_link frame

        # Apply optical frame conversion: camera_link -> optical
        # R_optical = R_LINK_TO_OPTICAL @ R_link
        R = _R_LINK_TO_OPTICAL @ R_link
        t = R @ np.array([tx, ty, tz])

        u, v, valid = _pinhole_project(world_points, R, t, fx, fy, cx, cy)

        map_x = u.reshape(BEV_HEIGHT, BEV_WIDTH).astype(np.float32)
        map_y = v.reshape(BEV_HEIGHT, BEV_WIDTH).astype(np.float32)
        mask = valid.reshape(BEV_HEIGHT, BEV_WIDTH).astype(np.uint8) * 255

        pos = R.T @ t
        center_dir = R.T @ np.array([0, 0, 1], dtype=np.float64)
        center_dir = center_dir / np.linalg.norm(center_dir)

        pts = world_points.reshape(BEV_HEIGHT, BEV_WIDTH, 3)
        to_pt = pts - pos
        norms = np.linalg.norm(to_pt, axis=2)
        dirs = to_pt / (norms[:, :, np.newaxis] + 1e-10)
        dot = np.sum(dirs * center_dir, axis=2)
        angle = np.arccos(np.clip(dot, -1, 1))
        weight = np.where(mask > 0, 1.0 / (angle + 0.01), 0.0).astype(np.float32)

        valid_px = np.sum(mask > 0)
        print(f'  {cam_name} ({frame}): pos=({tx:.3f},{ty:.3f},{tz:.3f}) '
              f'{valid_px} valid BEV pixels ({100*valid_px/(BEV_WIDTH*BEV_HEIGHT):.1f}%)')

        maps[cam_name] = {
            'map_x': map_x,
            'map_y': map_y,
            'mask': mask,
            'weight': weight,
        }

    print(f'AVM maps ready (from TF): {len(maps)} cameras')
    return maps


def stitch(frames, maps):
    """
    Winner-take-all stitch: each BEV pixel comes from the single best camera.
    No averaging in overlap → no ghosting.
    """
    cam_list = list(maps.keys())

    # Find best camera per pixel (highest angular weight)
    best_weight = np.zeros((BEV_HEIGHT, BEV_WIDTH), dtype=np.float32)
    best_cam = np.full((BEV_HEIGHT, BEV_WIDTH), -1, dtype=np.int32)
    for i, cam_name in enumerate(cam_list):
        m = maps[cam_name]
        if cam_name not in frames or frames[cam_name] is None:
            continue
        w = m['weight'] * (m['mask'] > 0)
        better = w > best_weight
        best_weight[better] = w[better]
        best_cam[better] = i

    # Warp each camera with cubic interpolation for sharpness
    warped_cache = {}
    for cam_name in cam_list:
        if cam_name not in frames or frames[cam_name] is None:
            continue
        m = maps[cam_name]
        warped = cv2.remap(frames[cam_name], m['map_x'], m['map_y'],
                           cv2.INTER_CUBIC, borderMode=cv2.BORDER_CONSTANT,
                           borderValue=0)
        warped_cache[cam_name] = warped

    # Compose: pure winner-take-all, no blending
    birdseye = np.zeros((BEV_HEIGHT, BEV_WIDTH, 3), dtype=np.uint8)
    for i, cam_name in enumerate(cam_list):
        if cam_name not in warped_cache:
            continue
        region = best_cam == i
        birdseye[region] = warped_cache[cam_name][region]

    birdseye = _draw_vehicle_overlay(birdseye)
    return birdseye


def _draw_vehicle_overlay(img):
    cy, cx = BEV_HEIGHT // 2, BEV_WIDTH // 2
    r = int(0.20 / BEV_RESOLUTION)

    cv2.circle(img, (cx, cy), r, (255, 255, 0), -1)
    cv2.circle(img, (cx, cy), r, (0, 0, 0), 2)

    # Front indicator (triangle pointing up = front)
    pts = np.array([
        [cx, cy - r - int(0.05 / BEV_RESOLUTION)],
        [cx - r // 2, cy - r + 2],
        [cx + r // 2, cy - r + 2],
    ], dtype=np.int32)
    cv2.fillPoly(img, [pts], (0, 255, 0))
    cv2.polylines(img, [pts], True, (0, 0, 0), 1)

    # Camera markers
    colors = {
        'front': (0, 0, 255),   # red
        'right': (0, 255, 0),   # green
        'back': (255, 0, 0),    # blue
        'left': (255, 255, 0),  # cyan
    }
    for name, (dx, dy) in [('front', (0, -1)), ('right', (1, 0)),
                             ('back', (0, 1)), ('left', (-1, 0))]:
        color = colors.get(name, (255, 255, 255))
        px = cx + int(dx * (r + int(0.03 / BEV_RESOLUTION)))
        py = cy + int(dy * (r + int(0.03 / BEV_RESOLUTION)))
        cv2.circle(img, (px, py), 3, color, -1)

    return img
