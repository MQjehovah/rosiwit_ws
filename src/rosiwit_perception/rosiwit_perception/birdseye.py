import numpy as np
import cv2
from .config import (
    IMG_W, IMG_H, FX, FY, CX, CY, HFOV,
    INTRINSICS, DIST_COEFFS, CAMERA_CONFIGS,
    BEV_WIDTH, BEV_HEIGHT, BEV_RESOLUTION,
)


def _fisheye_project(world_points, R, t):
    """
    Project world points (N, 3) to fisheye image coordinates.
    t = R @ camera_pos (camera position in base_link, expressed in camera frame)
    Uses OpenCV fisheye model with zero distortion.

    Returns: (u, v, valid) where valid is a boolean mask
    """
    # Camera position in world frame: pos = R^T @ t = R^T @ R @ pos_world = pos_world
    pos = R.T @ t
    # Transform to camera frame: P_cam = R * (P_world - pos)
    P_cam = (R @ (world_points - pos).T).T  # (N, 3)

    x, y, z = P_cam[:, 0], P_cam[:, 1], P_cam[:, 2]
    valid = z > 1e-6  # point must be in front of camera

    r = np.sqrt(x**2 + y**2)
    theta = np.arctan2(r, z)

    valid &= theta < HFOV / 2

    u = np.full_like(x, -1, dtype=np.float32)
    v = np.full_like(y, -1, dtype=np.float32)

    mask = valid & (r > 1e-6)
    u[mask] = FX * (theta[mask] * x[mask] / r[mask]) + CX
    v[mask] = FY * (theta[mask] * y[mask] / r[mask]) + CY

    center_mask = valid & (r <= 1e-6)
    u[center_mask] = CX
    v[center_mask] = CY

    in_bounds = valid & (u >= 0) & (u < IMG_W - 1) & (v >= 0) & (v < IMG_H - 1)
    return u, v, in_bounds


def precompute_maps():
    """
    Precompute remap arrays and weights for all cameras.
    Returns dict: cam_name -> {map_x, map_y, mask, weight_map}
    """
    maps = {}

    # Output grid: world coordinates for each BEV pixel
    half_w = BEV_WIDTH / 2.0
    half_h = BEV_HEIGHT / 2.0
    x_coords = (np.arange(BEV_WIDTH, dtype=np.float32) - half_w) * BEV_RESOLUTION
    y_coords = (np.arange(BEV_HEIGHT, dtype=np.float32) - half_h) * BEV_RESOLUTION
    xx, yy = np.meshgrid(x_coords, y_coords)
    zz = np.zeros_like(xx)

    world_points = np.stack([xx, yy, zz], axis=-1).reshape(-1, 3)

    for cam_name, cfg in CAMERA_CONFIGS.items():
        R = cfg['R']
        t = cfg['t']

        u, v, valid = _fisheye_project(world_points, R, t)

        map_x = u.reshape(BEV_HEIGHT, BEV_WIDTH).astype(np.float32)
        map_y = v.reshape(BEV_HEIGHT, BEV_WIDTH).astype(np.float32)
        mask = valid.reshape(BEV_HEIGHT, BEV_WIDTH).astype(np.uint8) * 255

        # Compute blend weights: inverse angular distance from camera center ray
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


def stitch(frames, maps):
    """
    Stitch 4 camera frames into a single bird's-eye view.

    Args:
        frames: dict of cam_name -> numpy array (H, W, 3) BGR
        maps: precomputed maps from precompute_maps()

    Returns:
        birdseye: numpy array (BEV_H, BEV_W, 3) BGR
    """
    acc = np.zeros((BEV_HEIGHT, BEV_WIDTH, 3), dtype=np.float32)
    weight_acc = np.zeros((BEV_HEIGHT, BEV_WIDTH), dtype=np.float32)

    for cam_name, m in maps.items():
        if cam_name not in frames:
            continue

        img = frames[cam_name]
        if img is None:
            continue

        # Remap camera image to BEV
        warped = cv2.remap(img, m['map_x'], m['map_y'],
                           cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
                           borderValue=0)

        mask_3ch = cv2.merge([m['mask'], m['mask'], m['mask']]).astype(np.float32) / 255.0
        w_3ch = cv2.merge([m['weight'], m['weight'], m['weight']])

        acc += warped.astype(np.float32) * w_3ch * mask_3ch
        weight_acc += (w_3ch[:, :, 0] * mask_3ch[:, :, 0])

    # Normalize
    weight_acc = np.maximum(weight_acc, 1e-6)
    birdseye = (acc / weight_acc[:, :, np.newaxis]).clip(0, 255).astype(np.uint8)

    # Draw vehicle overlay
    birdseye = _draw_vehicle_overlay(birdseye)

    return birdseye


def _draw_vehicle_overlay(img):
    """Draw a simple robot outline at the center of the BEV image."""
    cy, cx = BEV_HEIGHT // 2, BEV_WIDTH // 2
    r = int(0.20 / BEV_RESOLUTION)  # robot radius (0.20m)
    h = int(0.16 / BEV_RESOLUTION)  # robot length

    # Robot body (circle for cylindrical robot)
    cv2.circle(img, (cx, cy), r, (255, 255, 0), -1)
    cv2.circle(img, (cx, cy), r, (0, 0, 0), 2)

    # Front indicator (triangle)
    pts = np.array([
        [cx, cy - r - int(0.05 / BEV_RESOLUTION)],
        [cx - r // 2, cy - r + 2],
        [cx + r // 2, cy - r + 2],
    ], dtype=np.int32)
    cv2.fillPoly(img, [pts], (0, 255, 0))
    cv2.polylines(img, [pts], True, (0, 0, 0), 1)

    # Camera markers
    colors = {
        'front': (0, 0, 255),
        'right': (0, 255, 0),
        'back': (255, 0, 0),
        'left': (255, 255, 0),
    }
    for name, (dx, dy) in [('front', (0, -1)), ('right', (1, 0)),
                             ('back', (0, 1)), ('left', (-1, 0))]:
        color = colors.get(name, (255, 255, 255))
        px = cx + int(dx * (r + int(0.03 / BEV_RESOLUTION)))
        py = cy + int(dy * (r + int(0.03 / BEV_RESOLUTION)))
        cv2.circle(img, (px, py), 3, color, -1)

    return img
