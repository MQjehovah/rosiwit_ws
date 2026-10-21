"""rosiwit_perception — 视觉感知算法核心 (ROS 无关).

包含:
  - config:      相机标定/内外参与 BEV 配置
  - birdseye:    AVM 鸟瞰图逆向映射与拼接 (winner-take-all)
  - ground_texture: 地面纹理全局拼接
  - qr_detector:  二维码检测/解码与 solvePnP 位姿估计
  - obstacle_detector: 单目地面障碍检测
"""
