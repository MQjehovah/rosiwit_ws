# Third-party code notices

## vikit — camera models / vision utilities

- 位置：`include/vikit/`、`src/vikit/`
- 来源：https://github.com/Rhymer-Lcy/rpg_vikit_ros2_fisheye
  （commit `0b5548d9adab58128f3a59a507e96a83acfa8fdf`，2026-06；
  其上游为 https://github.com/uzh-rpg/rpg_vikit）
- 许可证：**GPLv3**（见上游 `vikit_common/package.xml`）
- 合入范围：
  - `vikit_common/include/vikit/*.h` + `*.hpp` → `include/vikit/`
  - `vikit_common/src/*.cpp` → `src/vikit/`
  - `vikit_ros/include/vikit/camera_loader.h`、`params_helper.h` → `include/vikit/`
  - 未合入 `vikit_ros/src/output_helper.cpp` 与 `output_helper.h`（ROS 可视化辅助，本算法未使用）

`src/vikit/` 中的文件保持上游实现不变，仅为编译方式调整（作为本包内的静态库 `vikit` 编译）。
