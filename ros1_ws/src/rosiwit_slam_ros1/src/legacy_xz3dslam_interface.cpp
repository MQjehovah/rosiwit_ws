// ============================================================================
// 旧 xz_3d_slam 接口 (legacy profile) —— 组织方式对照 ros2_ws SlamNode 分区,
// 2026-10-18 从单文件 1138 行拆为五个实现文件 (纯代码搬移, 行为逐字一致):
//
//   legacy_assembly.cpp        装配: 构造/发布与订阅注册/定时器/服务注册/模式机
//   legacy_cloud_publisher.cpp 点云与栅格发布: 输入回调 + 定时发布 + 实时栅格
//   legacy_pose_publisher.cpp  位姿与状态发布: curr_localization_pose + slam/info + TF
//   legacy_warning.cpp         告警: 告警码表 + 频率窗口 + 告警定时器
//   legacy_services.cpp        服务: 初始位姿 + 服务处理器 + 外部客户端
//
// 类声明仍在 legacy_xz3dslam_interface.h (单一类, 未拆);
// 跨文件共享的内部助手在 legacy_xz3dslam_internal.h。
// 生产话题的"哑"行为说明(global_map 等 5 个话题真机上本就不出数据)见
// legacy_cloud_publisher.cpp 头部注释。
// ============================================================================
// 本文件不再包含实现 —— 见上述五个拆分文件。
