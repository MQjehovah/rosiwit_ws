"""二维码核心算法测试（无需 ROS 环境）。

运行:
    PYTHONPATH=src/rosiwit_perception python3 -m pytest src/rosiwit_perception/test -q
"""

import cv2
import numpy as np
import pytest

from rosiwit_perception.qr_detector import (
    QRDetector,
    compute_approach_cmd,
    estimate_marker_pose,
    optical_pose_to_body,
    order_corners,
    rotation_matrix_to_quaternion,
)


def _make_qr_image(text: str, qr_px: int = 200, margin: int = 40):
    encoder_create = getattr(cv2, 'QRCodeEncoder_create', None)
    if encoder_create is None:
        pytest.skip('cv2.QRCodeEncoder unavailable')

    qr = encoder_create().encode(text)
    qr = cv2.resize(qr, (qr_px, qr_px), interpolation=cv2.INTER_NEAREST)
    canvas = np.full((qr_px + 2 * margin, qr_px + 2 * margin),
                     255, dtype=np.uint8)
    canvas[margin:margin + qr_px, margin:margin + qr_px] = qr
    return cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)


def test_order_corners_clockwise_from_any_order():
    expected = np.array([[0, 0], [10, 0], [10, 10], [0, 10]], np.float32)
    shuffled = expected[[2, 0, 3, 1]]
    ordered = order_corners(shuffled)
    assert np.allclose(ordered, expected)


def test_detect_single_qr():
    detector = QRDetector(min_size_px=20)
    image = _make_qr_image('ROSIWIT-QR-TEST')
    detections = detector.detect(image)

    assert len(detections) == 1
    det = detections[0]
    assert det.text == 'ROSIWIT-QR-TEST'
    assert det.corners.shape == (4, 2)
    assert det.size_px > 100
    assert det.rvec is None and det.tvec is None


def test_detect_multiple_qr():
    detector = QRDetector(min_size_px=20)
    left = _make_qr_image('LEFT-CODE')
    right = _make_qr_image('RIGHT-CODE')
    canvas = np.full((left.shape[0], left.shape[1] * 2 + 20, 3),
                     255, dtype=np.uint8)
    canvas[:, :left.shape[1]] = left
    canvas[:, left.shape[1] + 20:] = right

    texts = sorted(d.text for d in detector.detect(canvas))
    assert texts == ['LEFT-CODE', 'RIGHT-CODE']


def test_min_size_filter():
    detector = QRDetector(min_size_px=10000)
    assert detector.detect(_make_qr_image('TOO-SMALL')) == []


def test_pose_estimation_recovers_known_translation():
    k = np.array([[500.0, 0.0, 320.0],
                  [0.0, 500.0, 240.0],
                  [0.0, 0.0, 1.0]])
    marker_size = 0.12
    half = marker_size / 2.0
    # 与核心实现一致的物点顺序：TL, TR, BR, BL（y 向下）
    obj = np.array([
        [-half, -half, 0.0],
        [half, -half, 0.0],
        [half, half, 0.0],
        [-half, half, 0.0],
    ])
    rvec_true = np.array([0.15, -0.10, 0.05])
    tvec_true = np.array([0.03, -0.02, 0.80])

    img_pts, _ = cv2.projectPoints(obj, rvec_true, tvec_true, k, None)
    corners = order_corners(img_pts.reshape(4, 2))

    rvec, tvec = estimate_marker_pose(corners, k, None, marker_size)
    assert rvec is not None
    assert np.allclose(tvec, tvec_true, atol=1e-4)

    quat = rotation_matrix_to_quaternion(rvec)
    assert np.isclose(np.linalg.norm(quat), 1.0)


def test_pose_estimation_face_on_is_identity():
    k = np.array([[500.0, 0.0, 320.0],
                  [0.0, 500.0, 240.0],
                  [0.0, 0.0, 1.0]])
    half_px = 50.0
    corners = np.array([
        [320.0 - half_px, 240.0 - half_px],
        [320.0 + half_px, 240.0 - half_px],
        [320.0 + half_px, 240.0 + half_px],
        [320.0 - half_px, 240.0 + half_px],
    ])
    rvec, tvec = estimate_marker_pose(corners, k, None, 0.1)
    quat = rotation_matrix_to_quaternion(rvec)
    # 正对相机：x=0, y=0, z=0, w=1
    assert np.allclose(quat, [0.0, 0.0, 0.0, 1.0], atol=1e-6)


def test_pose_requires_valid_intrinsics():
    corners = np.array([[0, 0], [10, 0], [10, 10], [0, 10]], float)
    rvec, tvec = estimate_marker_pose(corners, np.zeros((3, 3)), None, 0.1)
    assert rvec is None and tvec is None


def test_empty_image_returns_no_detections():
    detector = QRDetector()
    assert detector.detect(np.zeros((100, 100, 3), np.uint8)) == []
    assert detector.detect(None) == []


def test_optical_to_body_axes():
    # 光学系 +z 向前 -> body 系 +x 向前
    rvec_body, tvec_body = optical_pose_to_body([0.0, 0.0, 0.0], [0.0, 0.0, 1.0])
    assert np.allclose(tvec_body, [1.0, 0.0, 0.0])
    # 光学系 +x 向右 -> body 系 -y（左为 +y）
    _, tvec_body = optical_pose_to_body([0.0, 0.0, 0.0], [1.0, 0.0, 0.0])
    assert np.allclose(tvec_body, [0.0, -1.0, 0.0])


def test_approach_control_law():
    # 太远、正对 -> 前进
    linear, angular, done = compute_approach_cmd(1.0, 0.0, 0.6)
    assert linear > 0.0
    assert abs(angular) < 1e-9
    assert not done

    # 二维码在右侧 -> 右转（angular < 0）
    linear, angular, _ = compute_approach_cmd(0.6, 0.3, 0.6)
    assert linear == 0.0
    assert angular < 0.0

    # 已到目标位置 -> done
    _, _, done = compute_approach_cmd(0.6, 0.0, 0.6)
    assert done
