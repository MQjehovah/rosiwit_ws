"""地面障碍检测核心测试。"""

import cv2
import numpy as np

from rosiwit_perception.config import CAMERA_CONFIGS
from rosiwit_perception.obstacle_detector import (
    GroundObstacleDetector,
    project_pixel_to_ground,
)

K = np.array([[500.0, 0.0, 640.0],
              [0.0, 500.0, 360.0],
              [0.0, 0.0, 1.0]])


def _front_model():
    cfg = CAMERA_CONFIGS['front']
    return np.asarray(cfg['R']), np.asarray(cfg['t'])


def test_projection_ground_plane():
    R, t = _front_model()
    p = project_pixel_to_ground(640, 560, K, None, R, t, -0.11)
    assert p is not None
    assert np.isclose(p[2], -0.11)
    assert 0.0 < p[0] < 3.0
    assert abs(p[1]) < 0.1


def test_detect_box_on_floor():
    R, t = _front_model()
    image = np.full((720, 1280, 3), 150, np.uint8)
    cv2.rectangle(image, (580, 400), (700, 560), (40, 40, 40), -1)

    detector = GroundObstacleDetector()
    obstacles = detector.detect(image, K, None, R, t)
    assert len(obstacles) == 1
    obs = obstacles[0]
    assert 0.1 < obs.x < 1.5
    assert abs(obs.y) < 0.15
    assert obs.confidence > 0.1


def test_empty_floor_no_obstacles():
    R, t = _front_model()
    image = np.full((720, 1280, 3), 150, np.uint8)
    detector = GroundObstacleDetector()
    assert detector.detect(image, K, None, R, t) == []


def test_obstacle_behind_camera_filtered():
    R, t = _front_model()
    image = np.full((720, 1280, 3), 150, np.uint8)
    # 画在高于地平线的位置，投影到地面时 s<=0
    cv2.rectangle(image, (600, 100), (680, 180), (40, 40, 40), -1)
    detector = GroundObstacleDetector()
    assert detector.detect(image, K, None, R, t) == []


def test_invalid_intrinsics():
    R, t = _front_model()
    detector = GroundObstacleDetector()
    assert detector.detect(
        np.zeros((100, 100, 3), np.uint8), np.zeros((3, 3)), None,
        R, t) == []
