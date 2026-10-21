"""地面纹理地图持久化 / 重定位测试（无需 ROS 环境）。"""

import numpy as np

from rosiwit_perception.ground_texture import GroundTextureMap


def _make_map():
    rng = np.random.default_rng(7)
    gt = GroundTextureMap(
        bev_resolution=0.01,
        bev_size=200,
        initial_canvas_size=800,
        canvas_expand_step=400,
        ground_z=-0.11,
    )
    bev = rng.integers(30, 220, (200, 200, 3), dtype=np.uint8)
    gt.integrate(bev, 0.0, 0.0, 0.0, refine=False)
    return gt, bev


def test_save_load_roundtrip(tmp_path):
    gt, _ = _make_map()
    path = gt.save(tmp_path / 'texture_map')

    assert path.endswith('.npz')
    loaded = GroundTextureMap.load(path)
    assert loaded.canvas_size == gt.canvas_size
    assert loaded.bev_resolution == gt.bev_resolution
    assert loaded.frame_count == gt.frame_count
    assert np.array_equal(loaded.texture_canvas, gt.texture_canvas)
    assert np.allclose(loaded.weight_canvas, gt.weight_canvas)


def test_localize_recovers_offset_pose():
    gt, bev = _make_map()

    ok, x, y, yaw, score = gt.localize(
        bev, 0.08, -0.05, 0.02,
        search_radius=0.15, search_angle=0.05, patch_r=80, min_score=0.2)

    assert ok
    assert score > 0.5
    assert abs(x) < 0.02
    assert abs(y) < 0.02
    assert abs(yaw) < 0.02


def test_refine_pose_keeps_pose_on_blank_map():
    gt = GroundTextureMap(
        bev_resolution=0.01, bev_size=200, initial_canvas_size=400)
    bev = np.full((200, 200, 3), 128, np.uint8)
    # 画布为空，refine 在 frame_count<5 时直接返回
    assert gt.refine_pose(bev, 1.0, 2.0, 0.3) == (1.0, 2.0, 0.3)
