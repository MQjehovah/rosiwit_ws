import math

import numpy as np

from rosiwit_exploration_ros2 import frontier_explorer as fe


def test_dilate_unknown_does_not_wrap():
    unknown = np.zeros((5, 5), dtype=bool)
    unknown[0, 0] = True
    dilated = fe.dilate_unknown(unknown)
    assert dilated[0, 0]
    assert dilated[0, 1]
    assert dilated[1, 0]
    assert dilated[1, 1]
    assert not dilated[4, 4]
    assert not dilated[4, 0]
    assert not dilated[0, 4]


def test_extract_clusters_filters_small_components():
    mask = np.zeros((6, 6), dtype=bool)
    mask[1, 1] = mask[1, 2] = mask[2, 1] = True
    mask[4, 4] = True
    clusters = fe.extract_clusters(mask, min_size=2)
    assert len(clusters) == 1
    assert len(clusters[0]) == 3


def test_parse_goal_status_token():
    assert fe.parse_goal_status('failed:123456') == ('failed', 123456)
    assert fe.parse_goal_status('ACCEPTED:42') == ('accepted', 42)
    # 旧格式没有 token, 保持兼容.
    assert fe.parse_goal_status('reached') == ('reached', None)
    assert fe.parse_goal_status('failed:not-a-number') == (
        'failed:not-a-number', None)


def test_window_sum_and_integral():
    occupied = np.zeros((4, 4), dtype=bool)
    occupied[1, 1] = True
    occupied[2, 2] = True
    integral = fe.build_integral(occupied)
    assert fe.window_sum(integral, 1, 1, 2, 2) == 1
    assert fe.window_sum(integral, 0, 0, 4, 4) == 2
    assert fe.window_sum(integral, 0, 0, 2, 2) == 1


def test_reachable_free_mask_respects_obstacles():
    free = np.ones((5, 5), dtype=bool)
    free[2, :] = False
    reachable = fe.reachable_free_mask(free, (0, 0))
    assert reachable[0, 0]
    assert reachable[1, 4]
    assert not reachable[3, 0]
    assert not reachable[4, 4]


def test_nearest_free_cell_searches_around_robot():
    free = np.zeros((7, 7), dtype=bool)
    free[3, 3] = True
    assert fe.nearest_free_cell(free, 3, 3) == (3, 3)
    assert fe.nearest_free_cell(free, 4, 4) == (3, 3)
    assert fe.nearest_free_cell(free, 0, 0, radius=2) is None


def test_cell_to_world_uses_cell_center():
    x, y = fe.cell_to_world(2, 3, origin_x=-1.0, origin_y=2.0, resolution=0.5)
    assert math.isclose(x, -1.0 + 3.5 * 0.5)
    assert math.isclose(y, 2.0 + 2.5 * 0.5)


def test_quaternion_from_yaw_is_normalized():
    q = fe.quaternion_from_yaw(math.pi / 2.0)
    assert math.isclose(q.z, math.sin(math.pi / 4.0))
    assert math.isclose(q.w, math.cos(math.pi / 4.0))
    assert math.isclose(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w, 1.0)
