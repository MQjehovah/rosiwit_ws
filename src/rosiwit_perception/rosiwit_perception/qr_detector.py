"""QR Code Detection + Pose Estimation (ROS 无关核心).

使用 OpenCV 内置的 :class:`cv2.QRCodeDetector` 做多码检测与解码，并可选地用
``solvePnP`` 估计二维码相对相机的 3D 位姿。

仅依赖 numpy / cv2，不依赖任何 ROS 接口。

坐标约定
--------
- 图像角点顺序统一为：左上 (TL)、右上 (TR)、右下 (BR)、左下 (BL)；
- ``rvec`` / ``tvec`` 为二维码坐标系到相机光学坐标系的变换
  （x 右、y 下、z 向前，与 OpenCV 一致）；
- 二维码自身坐标与图像坐标同向：原点在码中心，x 向右、y 向下、
  z 垂直码面向内；二维码正对相机时旋转为单位四元数。
"""

from dataclasses import dataclass
from typing import List, Optional, Sequence

import cv2
import numpy as np


def order_corners(corners: Sequence[Sequence[float]]) -> np.ndarray:
    """把任意顺序的 4 个角点整理为 TL, TR, BR, BL。

    对透视形变和轻微旋转都稳定；输入形状可以是 (4, 2) 或扁平数组。
    """
    pts = np.asarray(corners, dtype=np.float32).reshape(4, 2)
    sums = pts.sum(axis=1)
    diffs = pts[:, 1] - pts[:, 0]
    return np.array(
        [
            pts[np.argmin(sums)],   # top-left
            pts[np.argmin(diffs)],  # top-right
            pts[np.argmax(sums)],   # bottom-right
            pts[np.argmax(diffs)],  # bottom-left
        ],
        dtype=np.float32,
    )


def rotation_matrix_to_quaternion(rvec: Sequence[float]) -> np.ndarray:
    """旋转向量 -> 四元数 (x, y, z, w)。"""
    rmat, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    return matrix_to_quaternion(rmat)


def matrix_to_quaternion(rmat: np.ndarray) -> np.ndarray:
    """3x3 旋转矩阵 -> 四元数 (x, y, z, w)。

    手写 Shepperd 分支，避免额外依赖 scipy（核心包的 scipy 版本对 numpy
    版本较敏感）。
    """
    m = np.asarray(rmat, dtype=np.float64).reshape(3, 3)
    trace = m[0, 0] + m[1, 1] + m[2, 2]

    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s

    quat = np.array([x, y, z, w], dtype=np.float64)
    norm = np.linalg.norm(quat)
    if norm > 0.0:
        quat /= norm
    return quat


def estimate_marker_pose(corners: Sequence[Sequence[float]],
                         camera_matrix: np.ndarray,
                         dist_coeffs: Optional[np.ndarray] = None,
                         marker_size: float = 0.1):
    """用 solvePnP 估计二维码位姿。

    Parameters
    ----------
    corners:
        TL, TR, BR, BL 四个角点（像素）。
    camera_matrix:
        3x3 相机内参。
    dist_coeffs:
        畸变系数，None 表示无畸变。
    marker_size:
        二维码物理边长（米）。

    Returns
    -------
    (rvec, tvec) 或 ``(None, None)``（求解失败/参数非法）。
    """
    if marker_size <= 0.0:
        return None, None

    k = np.asarray(camera_matrix, dtype=np.float64)
    if k.shape != (3, 3) or k[0, 0] <= 0.0 or k[1, 1] <= 0.0:
        return None, None

    dist = (np.zeros((5, 1), dtype=np.float64)
            if dist_coeffs is None else np.asarray(dist_coeffs, dtype=np.float64))
    obj_pts = _marker_object_points(marker_size)
    img_pts = np.asarray(corners, dtype=np.float64).reshape(4, 2)

    # SQPNP 对平面四点更稳；老版本 OpenCV 没有该标志时退回 ITERATIVE。
    # 注意不要用 IPPE_SQUARE：它要求的角点顺序与图像坐标的
    # TL/TR/BR/BL 不一致，容易得到 180° 翻转或大重投影误差。
    sqpnp = getattr(cv2, 'SOLVEPNP_SQPNP', None)
    flags_list = ([sqpnp] if sqpnp is not None else []) + [cv2.SOLVEPNP_ITERATIVE]

    ok, rvec, tvec = False, None, None
    for flags in flags_list:
        try:
            ok, rvec, tvec = cv2.solvePnP(
                obj_pts, img_pts, k, dist, flags=flags)
        except cv2.error:
            ok = False
        if ok:
            break

    if not ok or tvec is None or tvec[2, 0] <= 0.0:
        return None, None
    return rvec.reshape(3), tvec.reshape(3)


def _marker_object_points(marker_size: float) -> np.ndarray:
    """物点顺序与图像一致：TL, TR, BR, BL，y 轴向下，原点在中心。"""
    half = float(marker_size) / 2.0
    return np.array(
        [
            [-half, -half, 0.0],
            [half, -half, 0.0],
            [half, half, 0.0],
            [-half, half, 0.0],
        ],
        dtype=np.float64,
    )


# 相机光学系 -> 相机 body/link 系 (x 前, y 左, z 上)
_R_OPTICAL_TO_LINK = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
], dtype=np.float64)


def optical_pose_to_body(rvec: Sequence[float], tvec: Sequence[float]):
    """把相机光学系的位姿转换到相机 body/link 系（x 前、y 左、z 上）。"""
    rmat, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    rvec_body, _ = cv2.Rodrigues(_R_OPTICAL_TO_LINK @ rmat)
    tvec_body = _R_OPTICAL_TO_LINK @ np.asarray(
        tvec, dtype=np.float64).reshape(3)
    return rvec_body.reshape(3), tvec_body.reshape(3)


def is_optical_frame(frame_id: str) -> bool:
    """判断坐标系名是否已经是相机光学系。"""
    return 'optical' in (frame_id or '').lower()


def compute_approach_cmd(distance: float,
                         bearing: float,
                         target_distance: float,
                         k_linear: float = 0.8,
                         k_angular: float = 1.2,
                         max_linear: float = 0.2,
                         max_angular: float = 0.6,
                         bearing_gate: float = 0.5,
                         distance_tol: float = 0.03,
                         bearing_tol: float = 0.03):
    """二维码视觉伺服控制律。

    Parameters
    ----------
    distance:
        二维码沿相机光轴的距离（米）。
    bearing:
        二维码相对光轴的方位角（弧度，右侧为正）。
    target_distance:
        期望保持的距离（米）。

    Returns
    -------
    ``(linear_x, angular_z, done)``。``done`` 表示距离和方位都进入容差。
    """
    err = float(distance) - float(target_distance)
    bearing = float(bearing)

    linear = float(np.clip(k_linear * err, -abs(max_linear), abs(max_linear)))
    if abs(bearing) > bearing_gate:
        linear = 0.0

    # bearing > 0 表示二维码在右侧，需要右转（angular.z < 0）
    angular = float(np.clip(-k_angular * bearing,
                            -abs(max_angular), abs(max_angular)))
    done = abs(err) <= distance_tol and abs(bearing) <= bearing_tol
    return linear, angular, done


@dataclass
class QRCodeDetection:
    """单个二维码的检测结果。"""

    text: str
    corners: np.ndarray                 # (4, 2) TL, TR, BR, BL
    center: np.ndarray                  # (2,)
    rvec: Optional[np.ndarray] = None   # (3,)
    tvec: Optional[np.ndarray] = None   # (3,), 相机光学系
    distance: Optional[float] = None    # 米

    @property
    def size_px(self) -> float:
        """二维码四边形较短边的像素长度。"""
        c = self.corners
        return float(min(
            np.linalg.norm(c[0] - c[1]),
            np.linalg.norm(c[1] - c[2]),
        ))

    def pose_quaternion(self) -> Optional[np.ndarray]:
        """位姿四元数 (x, y, z, w)，无位姿时返回 None。"""
        if self.rvec is None:
            return None
        return rotation_matrix_to_quaternion(self.rvec)


class QRDetector:
    """OpenCV 二维码检测器封装（多码优先，单码兜底）。"""

    def __init__(self, min_size_px: float = 0.0):
        self.min_size_px = float(min_size_px)
        self._detector = cv2.QRCodeDetector()

    def detect(self,
               image: np.ndarray,
               camera_matrix: Optional[np.ndarray] = None,
               dist_coeffs: Optional[np.ndarray] = None,
               marker_size: float = 0.1) -> List[QRCodeDetection]:
        """检测并解码图像中的二维码。

        Parameters
        ----------
        image:
            BGR / BGRA / 灰度图像。
        camera_matrix, dist_coeffs, marker_size:
            提供 ``camera_matrix`` 且 ``marker_size > 0`` 时估计位姿。
        """
        if image is None or image.size == 0:
            return []

        gray = image
        if image.ndim == 3:
            if image.shape[2] == 4:
                gray = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)
            else:
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        raw = self._detect_multi(gray)
        if not raw:
            raw = self._detect_single(gray)

        results: List[QRCodeDetection] = []
        for text, corners in raw:
            if self.min_size_px > 0.0:
                c = corners
                size = min(np.linalg.norm(c[0] - c[1]),
                           np.linalg.norm(c[1] - c[2]))
                if size < self.min_size_px:
                    continue

            det = QRCodeDetection(
                text=text,
                corners=corners,
                center=corners.mean(axis=0),
            )
            if camera_matrix is not None:
                rvec, tvec = estimate_marker_pose(
                    corners, camera_matrix, dist_coeffs, marker_size)
                if rvec is not None:
                    det.rvec = rvec
                    det.tvec = tvec
                    det.distance = float(np.linalg.norm(tvec))
            results.append(det)
        return results

    # ------------------------------------------------------------------ #
    # 内部实现
    # ------------------------------------------------------------------ #
    def _detect_multi(self, gray: np.ndarray):
        try:
            ok, texts, points, _ = self._detector.detectAndDecodeMulti(gray)
        except cv2.error:
            return []
        if not ok or points is None or len(points) == 0:
            return []

        found = []
        for text, quad in zip(texts, points):
            corners = order_corners(quad)
            if not text:
                text = self._decode_quad(gray, corners)
            if text:
                found.append((text, corners))
        return found

    def _detect_single(self, gray: np.ndarray):
        try:
            text, points, _ = self._detector.detectAndDecode(gray)
        except cv2.error:
            return []
        if not text or points is None or len(points) == 0:
            return []
        return [(text, order_corners(points))]

    def _decode_quad(self, gray: np.ndarray, corners: np.ndarray) -> str:
        """多码接口已定位但未解码时，用单码 decode 再试一次。"""
        try:
            text, _ = self._detector.decode(gray, corners)
            return text or ''
        except cv2.error:
            return ''
