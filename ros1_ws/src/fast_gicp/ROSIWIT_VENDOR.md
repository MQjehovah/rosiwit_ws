# vendored: fast_gicp (SMRT-AIST)

- 来源: https://github.com/SMRT-AIST/fast_gicp @ 0e7ec14 (2026-11-26, depth-1 clone)
- 用途: matcher=fast_gicp —— 与 PCL GICP 同算法, OpenMP 并行 (协方差估计 + 逐轮线性化)
- 本地补丁 (rosiwit):
  - src/fast_gicp/gicp/fast_gicp.cpp / lsq_registration.cpp 末尾增加
    `pcl::PointXYZINormal` 的显式实例化 (上游只有 PointXYZ/PointXYZI/PointNormal/PointXYZRGB;
    SLAM 的 PointType = PointXYZINormal, 不补会链接期 undefined reference)
- 更新上游时: 重新 clone 后把上述两行实例化补回来
