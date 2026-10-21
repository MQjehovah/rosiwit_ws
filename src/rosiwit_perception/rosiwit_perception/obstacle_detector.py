"""Monocular ground-plane obstacle detection (ROS 无关核心).

思路：用画面底部的“地面颜色”作为参考，分割出颜色明显不同的连通区域；
取每个区域的**底部接触点**投到地面平面（已知相机高度/外参），得到机器
人坐标系下的低矮障碍位置。适用于办公室/仓库等颜色相对一致的地面。

这是轻量 MVP：对纯色地面/纸箱/椅子腿有效，对花纹地毯、强反光地面需要
调阈值；后续可替换为语义分割/深度模型，接口保持不变。
"""

from dataclasses import dataclass
from typing import List, Optional, Sequence

import cv2
import numpy as np


@dataclass
class GroundObstacle:
    """一个地面障碍物（底部接触点）。"""

    u: float
    v: float
    x: float          # base_link 前向（米）
    y: float          # base_link 左向（米）
    radius: float     # 估计半径（米）
    confidence: float


class GroundObstacleDetector:
    def __init__(self,
                 ground_z: float = -0.11,
                 max_distance: float = 3.0,
                 max_lateral: float = 1.5,
                 min_area_ratio: float = 0.0015,
                 color_threshold: float = 35.0,
                 roi_top_ratio: float = 0.30,
                 kernel_size: int = 5):
        self.ground_z = float(ground_z)
        self.max_distance = float(max_distance)
        self.max_lateral = float(max_lateral)
        self.min_area_ratio = float(min_area_ratio)
        self.color_threshold = float(color_threshold)
        self.roi_top_ratio = float(roi_top_ratio)
        self.kernel_size = int(kernel_size)

    def detect(self,
               image: np.ndarray,
               camera_matrix: np.ndarray,
               dist_coeffs: Optional[Sequence[float]],
               R: np.ndarray,
               t: np.ndarray) -> List[GroundObstacle]:
        """检测地面障碍。

        Parameters
        ----------
        image:
            BGR 图像。
        camera_matrix, dist_coeffs:
            相机内参与畸变。
        R, t:
            ``P_optical = R @ (P_base - pos)``，其中 ``pos = R.T @ t``。
        """
        if image is None or image.size == 0:
            return []
        h, w = image.shape[:2]
        k = np.asarray(camera_matrix, dtype=np.float64)
        if k.shape != (3, 3) or k[0, 0] <= 0.0:
            return []
        dist = (np.zeros(5, dtype=np.float64)
                if dist_coeffs is None else np.asarray(dist_coeffs, np.float64))
        r = np.asarray(R, dtype=np.float64)
        tt = np.asarray(t, dtype=np.float64)

        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        floor_med = self._floor_color(lab)
        if floor_med is None:
            return []

        diff = lab.astype(np.float32) - floor_med
        color_dist = np.sqrt(np.sum(diff * diff, axis=2))
        mask = (color_dist > self.color_threshold).astype(np.uint8)

        top = int(h * self.roi_top_ratio)
        mask[:top, :] = 0
        if self.kernel_size > 1:
            kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        if np.count_nonzero(mask) == 0:
            return []

        num, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        min_area = max(20.0, self.min_area_ratio * h * w)
        obstacles = []

        for label in range(1, num):
            area = float(stats[label, cv2.CC_STAT_AREA])
            if area < min_area:
                continue

            ys, xs = np.where(labels == label)
            v = float(ys.max())
            bottom = ys >= (v - 2)
            if not np.any(bottom):
                continue
            u = float(np.median(xs[bottom]))
            if u < 0 or u >= w or v < 0 or v >= h:
                continue

            p = project_pixel_to_ground(u, v, k, dist, r, tt, self.ground_z)
            if p is None:
                continue
            x, y = float(p[0]), float(p[1])
            if x < 0.05 or x > self.max_distance or abs(y) > self.max_lateral:
                continue

            width_px = float(xs.max() - xs.min() + 1)
            radius = width_px * max(x, 0.1) / k[0, 0] * 0.5
            radius = float(np.clip(radius, 0.04, 0.5))
            confidence = float(np.clip(area / (min_area * 4.0), 0.1, 1.0))

            obstacles.append(GroundObstacle(
                u=u, v=v, x=x, y=y, radius=radius, confidence=confidence))

        return obstacles

    def _floor_color(self, lab: np.ndarray):
        h, w = lab.shape[:2]
        y0 = int(h * 0.85)
        left = lab[y0:h, :int(w * 0.25)].reshape(-1, 3)
        right = lab[y0:h, int(w * 0.75):].reshape(-1, 3)
        samples = np.concatenate([left, right], axis=0)
        if samples.size == 0:
            return None
        return np.median(samples, axis=0)


def project_pixel_to_ground(u: float, v: float,
                            camera_matrix: np.ndarray,
                            dist_coeffs: Optional[Sequence[float]],
                            R: np.ndarray,
                            t: np.ndarray,
                            ground_z: float = -0.11):
    """把图像像素投到 ``base_link`` 下的地面平面 z=ground_z。"""
    k = np.asarray(camera_matrix, dtype=np.float64)
    dist = (np.zeros(5, dtype=np.float64)
            if dist_coeffs is None else np.asarray(dist_coeffs, np.float64))
    pts = np.array([[[float(u), float(v)]]], dtype=np.float64)
    try:
        norm = cv2.undistortPoints(pts, k, dist)
    except cv2.error:
        return None
    xn, yn = float(norm[0, 0, 0]), float(norm[0, 0, 1])

    r = np.asarray(R, dtype=np.float64)
    pos = r.T @ np.asarray(t, dtype=np.float64).reshape(3)
    ray = r.T @ np.array([xn, yn, 1.0], dtype=np.float64)
    if abs(ray[2]) < 1e-6:
        return None

    s = (float(ground_z) - pos[2]) / ray[2]
    if s <= 0.0:
        return None
    return pos + s * ray
