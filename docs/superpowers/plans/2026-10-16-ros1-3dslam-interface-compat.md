# 实施计划：rosiwit_slam_ros1 承接 xz_3d_slam 的 ROS1 接口

**对应设计文档**：`docs/superpowers/specs/2026-10-16-ros1-3dslam-interface-compat-design.md`
（本计划只写"怎么做"；"为什么这么定"在设计文档 §5/§6/§8。凡本计划给出具体值的地方，
那个值就是**决定**，不要另行推断。）

**目标**：让整机把 `3d_slam` 换成 `rosiwit_slam` 后，`rosiwit_slam_ros1` 这一个进程同时暴露
两套接口（新的 `rosiwit` 接口 + 旧的 `xz_3d_slam` 接口），话题名 / 服务名 / 消息类型 /
参数名 / frame_id / TF 树形状全部与旧 `xz_3d_slam` 对上。改动只落在
`ros1_ws/src/rosiwit_slam_ros1/`（外加上一轮已做的环境修复，见 T0）。

**参考实现（权威来源，只读）**：
- 旧接口行为：`E:\workspace_slam\3d_slam\src\xz_3d_slam\xz_slam_ros1_interface.cpp`
  （下文引用为 `legacy.cpp:<行号>`）
- 旧接口声明：`E:\workspace_slam\3d_slam\src\xz_3d_slam\xz_slam_ros1_interface.h`
- 旧配置默认值：`E:\workspace_slam\3d_slam\src\xz_3d_slam\conf\slam_conf.json`、
  `conf\warning_code.json`
- 旧 launch：`E:\workspace_slam\3d_slam\src\xz_3d_slam\launch\xz_3d_slam.launch`

---

## T0. 环境前置（**已发现的问题，前 3 项已修，第 4 项需你执行**）

`ros1_ws` 此前从未构建成功过。构建前必须满足：

1. **cmake ≥ 3.16** —— 核心 `src/rosiwit_slam/CMakeLists.txt` 写的是
   `cmake_minimum_required(VERSION 3.16)`，而 melodic 自带 cmake 是 3.10.2（连 `-S/-B` 都不支持）。
   **已做**：安装 cmake 3.16.9 到 `~/.local/cmake-3.16.9-Linux-x86_64`，
   在 `~/.local/bin` 建了 `cmake`/`ctest`/`cpack` 软链，并把 `~/.local/bin` 加进 `~/.bashrc` 的 PATH。
2. **`ros1_ws/src/rosiwit_slam_ros1/COLCON_IGNORE` 会让 catkin 静默跳过整个包** ——
   `catkin_pkg 0.5.2` 认这个文件，后果是 `catkin_make` 返回 0 但什么都没编译。
   **已做**：移动到 `COLCON_IGNORE.removed-by-setup`（`ros1_ws/COLCON_IGNORE` 已挡住 colcon，包内那个是多余且有害的）。
3. **`catkin_package(INCLUDE_DIRS include)` 但包里没有 `include/` 目录** —— CMake 直接报错。
   **已做**：从 `catkin_package()` 删掉 `INCLUDE_DIRS include`（本包不对外导出头文件）。
4. **核心缺少 Ceres 与 Sophus 的头文件**（`ceres_backend.h:7` 无条件 include `<ceres/ceres.h>`、
   `ieskf.h:3` 无条件 include `<sophus/so3.hpp>`；核心 CMake 里 `find_package(... QUIET)` 只管链接，
   不管这两个无条件 include）。
   **已做**（你已安装）：`libceres-dev` + `ros-melodic-sophus` 已就位，且正好匹配：
   `libceres-dev 1.13.0` 提供 `/usr/include/ceres/ceres.h` + `/usr/lib/cmake/Ceres/CeresConfig.cmake`
   （→ `find_package(Ceres)` 命中）；`ros-melodic-sophus 1.0.1` 提供
   `/opt/ros/melodic/include/sophus/so3.hpp` + `/opt/ros/melodic/share/sophus/cmake/SophusConfig.cmake`
   （→ `find_package(Sophus)` 命中，名字大小写正好对上）。
   GTSAM / octomap **不需要**：源码里 gtsam 全部在 `#ifdef USE_GTSAM` 内，octomap 的 .cpp 被 CMake 过滤掉。
5. **Ceres 1.13 没有 `Ceres::ceres` 目标** —— 核心 CMake 写的是
   `target_link_libraries(rosiwit_slam PUBLIC Ceres::ceres)`，而带命名空间的目标自 Ceres 1.14 才有；
   18.04 的 1.13 只导出无命名空间的导入目标 `ceres` 与变量 `CERES_INCLUDE_DIRS`/`CERES_LIBRARIES`。
   **已做**：在薄层 `CMakeLists.txt` 的 `add_subdirectory(核心)` **之前**加垫片
   （`find_package(Ceres QUIET)` + `NOT TARGET Ceres::ceres` 守卫 + `INTERFACE IMPORTED GLOBAL` 桥接）。
   Ceres ≥ 1.14 时守卫为假 → 整段跳过，无副作用。**纯薄层改动，核心一字未动。**
   （不能写成 `add_library(Ceres::ceres ALIAS ceres)`：CMake 不允许对**非 GLOBAL** 的导入目标建 ALIAS。）
6. **核心有 2 处依赖"较新版本"才能编过**（**你已批准改核心**）—— 核心是照 ROS2 Humble/22.04 写的
   （`docker/slam-mapping/Dockerfile` = `ros:humble-ros-base`，`scripts/install_deps.sh` 全是 `ros-humble-*` + colcon），
   18.04/melodic 的依赖更老。`make -k` 全量核对过：核心 57 个编译单元**只有这 2 个文件**编不过，其余 55 个全过。
   a. `src/algorithms/fast_lio2/map_builder/ieskf.cpp`：`Sophus::SO3d::leftJacobian` /
      `leftJacobianInverse` 是较新 Sophus 才有的静态成员（melodic 的 sophus 1.0.1 没有）。
      **改法**：文件内加编译期探测（`decltype` SFINAE，C++17）；**有就走 Sophus 原实现**（新版环境逐位不变），
      没有就走闭式解：
      `J_l(φ) = I + (1-cosθ)/θ²·φ^ + (θ-sinθ)/θ³·φ^²`（θ→0 取级数极限，避免除零），
      然后 `Jr = J_lᵀ`、`JrInv = (J_l⁻¹)ᵀ`（与原代码 `leftJacobian(...).transpose()` 语义一致）。
      **已用有限差分独立验证**（只用老 Sophus 也有的 `SO3d::exp`/`log`，300 个随机 φ∈(-3.13,3.13)）：
      `JrInv` vs 右扰动有限差分 = 3.66e-08；`Jr` vs 其逆 = 3.21e-08；
      `Jr` vs (左扰动⁻¹)ᵀ = 3.15e-08；`Jr·JrInv - I` = 7.29e-15；
      小角度级数分支同量级（2.8e-14）；`Jr(0) - I = JrInv(0) - I = 0`。
      （3e-08 就是 h=1e-7 有限差分自身的噪声底。）
      该检查将固化为 T1 的 gtest —— 薄层能 `#include "ieskf.h"`，可直接调 `Jr`/`JrInv`，无需改核心测试。
   b. `src/algorithms/localization/localization_pipeline.cpp:302`：`pcl::PointXYZINormal(p.x(),p.y(),p.z())`
      的三参构造是 PCL 1.12+ 才有的（已查 PCL 官方文档确认 1.15 提供
      `PointXYZINormal(float _x, float _y, float _z, float _intensity = 0.f)`），
      而 18.04 是 PCL 1.8.1（`pcl::PointXYZINormal` 只有默认构造 + 拷贝构造）。
      **改法**：改为默认构造 + 逐字段赋值 `pt.x/pt.y/pt.z`。默认构造已把
      `intensity`/`curvature`/`normal_*` 置零、`data[3]` 置 1，与新版本三参构造语义一致；无行为差异。
7. **薄层 `CMakeLists.txt` 自身有个从未编过的错** ——
   `target_compile_definitions(rosiwit_slam_ros1_node PRIVATE -pipe -pthread)` 会产生
   `-D-pipe` / `-D-pthread`，g++ 报 `<command-line>:0:1: error: macro names must be identifiers`。
   **已做**：改成 `target_compile_options(...)`。

**基线状态（已验证，T1 的起点）**：上述 1–7 修完后

```bash
catkin_make -j$(nproc) -DCATKIN_WHITELIST_PACKAGES="rosiwit_slam_ros1;xzrobot_ros_dependency"
```

返回 `exit=0`。核心 **57/57** 个 `.o` 全部产出，`devel/lib/librosiwit_slam.so`（2.29 MB）与
`devel/lib/rosiwit_slam_ros1/rosiwit_slam_ros1_node`（533 KB）均生成；日志中可见
`-- rosiwit_slam_ros1: shimmed Ceres::ceres (Ceres 1.13.0, CERES_LIBRARIES='ceres')`。
即：**核心在 melodic/bionic + 全 apt 依赖下可编译**（此前从未验证过）。

**统一构建命令**（后续每个任务都用它）：
```bash
bash -lc 'source ~/.bashrc; source /opt/ros/melodic/setup.bash; \
  cd ~/rosiwit_ws/ros1_ws && \
  catkin_make -j$(nproc) -DCATKIN_WHITELIST_PACKAGES="rosiwit_slam_ros1;xzrobot_ros_dependency"'
```
测试：
```bash
bash -lc 'source ~/.bashrc; source /opt/ros/melodic/setup.bash; \
  cd ~/rosiwit_ws/ros1_ws && catkin_make run_tests_rosiwit_slam_ros1'
```
（用 whitelist 是因为 `rosiwit_navigation_ros1` 也在同一个工作空间里，
它不是本任务范围，先把它排除，避免它的构建噪音干扰。）

---

## 目标文件布局（T1 建立，后续任务往里面填）

```
ros1_ws/src/rosiwit_slam_ros1/
├── CMakeLists.txt                       [改] 拆 lib + exe + gtest
├── package.xml                          [改] 加 xzrobot_ros_dependency / yaml-cpp 测试依赖
├── config/default.yaml                  [不变]
├── conf/slam_conf.json                  [新] 旧配置默认值（从 3d_slam conf 原样搬）
├── conf/warning_code.json               [新] 告警码表（原样搬）
├── launch/rosiwit_slam.launch           [不变]
├── launch/xz_3d_slam.launch             [新] 旧子系统 launch 的 drop-in（只改第 34 行）
├── launch/legacy_dev.launch             [新] 只起本节点(input_profile:=legacy)，供开发机测试
├── scripts/check_legacy_interface.sh    [新] V2 接口一致性校验
├── scripts/legacy_interface.golden      [新] 期望的 topic/service/type 清单
├── srv/*.srv                            [不变]
├── test/
│   ├── fixtures/                        [新] 配置 fixture
│   ├── test_legacy_config.cpp           [新]
│   ├── test_lost_monitor.cpp            [新]
│   ├── test_legacy_convert.cpp          [新]
│   └── test_realtime_grid.cpp           [新]
└── src/
    ├── rosiwit_slam_ros1_node.cpp       [改] 只剩 main
    ├── legacy_config.h/.cpp             [新] 旧 JSON 配置装载（纯逻辑，无 ROS）
    ├── legacy_convert.h/.cpp            [新] 旧消息构造（纯逻辑，只依赖 ROS 消息类型）
    ├── lost_monitor.h/.cpp              [新] 定位丢失/恢复状态机（纯逻辑，无 ROS）
    ├── realtime_grid.h/.cpp             [新] 建图实时栅格（复用核心公开生成器 + worker 线程）
    ├── slam_engine.h/.cpp               [新] 唯一的核心持有者
    ├── rosiwit_interface.h/.cpp         [新] 现有新接口原样搬迁
    └── legacy_xz3dslam_interface.h/.cpp [新] 旧 xz_3d_slam 接口全套
```

---

## T1. 构建骨架 + `LegacyConfig`

**目标**：`catkin_make` 能把核心编过；旧 JSON 配置能按旧覆盖链读出来并全部有默认值；
旧地图路径契约可单测。

**文件**：`CMakeLists.txt`、`package.xml`、`conf/slam_conf.json`、`conf/warning_code.json`、
`src/legacy_config.h`、`src/legacy_config.cpp`、`test/test_legacy_config.cpp`、`test/fixtures/*`

### 1.1 `conf/` 资源
- 把 `E:\workspace_slam\3d_slam\src\xz_3d_slam\conf\slam_conf.json` **原样复制**为
  `conf/slam_conf.json`（它就是"旧配置默认值"的权威来源，含 `laser_params` 子树）。
- 把 `conf\warning_code.json` **原样复制**为 `conf/warning_code.json`（155 行，内容不改）。

### 1.2 `src/legacy_config.h`（无 ROS 依赖，只用 yaml-cpp）

```cpp
// 旧 xz_3d_slam 的配置装载。JSON 用 yaml-cpp 解析(JSON 是 YAML 子集, 不额外引 json 库)。
namespace rosiwit_slam_ros1 {

struct LegacyConfPaths {
    std::string proj_conf_json;    // PROJECT_PATH/conf/slam_conf.json (优先)
    std::string pkg_conf_json;     // 本包 conf/slam_conf.json (兜底)
    std::string opt_sensor_json;   // /opt/xzrobot/configs/sensor_config.json
    std::string opt_slam_json;     // /opt/xzrobot/configs/3d_slam/slam_conf.json (深合并覆盖)
};

struct LegacyConfig {
    std::string laser_type = "";            // mid360|new_mid360|vanjee|new_vanjee|new_robosense|""(见下)
    std::string slam_init_type = "localization";
    bool   is_debug = false;
    bool   debug_publish = false;           // 未配时回落到 is_debug
    double roll_new = 0, pitch_new = 0, yaw_new = 90, x_new = 0, z_new = 0;
    double imu_in_car_frame_yaw_livox = 65, imu_in_car_frame_yaw_wj = 60;
    std::string laser_sub_name = "rslidar_points";
    std::string imu_sub_name = "zyz_imu_data";
    int    publish_hz = 20;                 // 解析出来但发布周期硬编码 0.1s (legacy.cpp:254-255)
    int    rt_grid_map_publish_hz = 1;
    double yaw_calib = 0;                   // 读入时已按 *M_PI/180 转弧度 (legacy.cpp:1563-1565)
    double x_calib = 0, y_calib = 0;
    bool   use_tf = true;
    bool   use_filter_odom = false;
    bool   use_2d_undistort = true;
    double th_2d_range_min = 0.5, th_2d_range_max = 3.0;
    int    test_wait_time = 30;
    bool   use_delete_pts = true;           // 装载后还要 && isSkywalker50(model)
    double delete_pt_in_car_max_x = 0.5,  delete_pt_in_car_min_x = -1.2;
    double delete_pt_in_car_max_y = 0.5,  delete_pt_in_car_min_y = -0.5;
    double delete_pt_in_car_max_z = 0.8,  delete_pt_in_car_min_z = -0.5;
    double use_3d_z_min = 0.0, use_3d_z_max = 20.0;
    double mapping_z_min = 0.0, mapping_z_max = 20.0;   // 缺省取 use_3d_*
    double loc_z_min = 0.0,     loc_z_max = 20.0;       // 缺省取 use_3d_*
    bool   lidar_pts_monitor = true;
    int    lidar_pts_monitor_interval = 10;             // <1 时强制 10
    std::string xz_log_path = "/opt/xzrobot/logs/xzrobot_3d_slam/xz_3d_slam.log";
    bool   print_xz_log_to_file = true;

    /// 按旧顺序装载: sensor_config.json -> 主 slam_conf -> /opt 覆盖 -> laser_params.<base> 合并
    /// 任何文件缺失/解析失败只 WARN, 用内置默认值, 绝不抛异常
    static LegacyConfig load(const LegacyConfPaths& paths, const std::string& machine_model);
};

/// legacy.cpp:12-20 原样: model=="XZ-SW50" / "XZ-SWGT"
bool isSkywalker50(const std::string& model);
bool isSkywalkerGT(const std::string& model);

/// 旧地图路径契约 (设计文档 §5.10 A)
/// base = (map_dir 去掉尾斜杠) + "/map"   // 基名恒为 map, map_name 只是显示标签
std::string legacyMapBase(const std::string& map_dir);

} // namespace rosiwit_slam_ros1
```

### 1.3 `src/legacy_config.cpp` —— 逐条照抄旧实现（**不允许"顺手改正"**）

装载顺序与来源（照抄 `legacy.cpp:32-43` + `loadConfigJson` 1373-1407 + `loadConfFromJsonFile` 1409-1708）：

1. `laser_type` 由 `sensor_config.json` 的 `type_3d` **和** model 共同决定（`legacy.cpp:1377-1404`）：
   - `type_3d` 缺失 → `"Livox_Mid360"`；`type_3d == "Livox"` → `"Livox_Mid360"`
   - `WLR_722` + `isSkywalkerGT` → `new_vanjee`；`WLR_722` + `isSkywalker50` → `vanjee`
   - `Livox_Mid360`/`Livox_Mid360s` + GT → `new_mid360`；+ 50 → `mid360`
   - `Robosense` + GT → `new_robosense`，**并同时**把 `laser_sub_name` 置 `rslidar_points`、`imu_sub_name` 置 `rslidar_imu`
   - 都不命中时 `laser_type` **保持空串**（`legacy.h:368` 初值），后续走 else 分支订阅
2. 主配置 = `proj_conf_json`（旧 `PROJECT_PATH/conf/slam_conf.json`）；本包 `pkg_conf_json` 作兜底。
   **先存在的胜**：两个都试，谁存在用谁；都不存在 → 全默认值 + WARN。
3. 若 `opt_slam_json` 存在 → 用 `merge_patch` 式**深合并**覆盖（嵌套子树按 key 合并，不是整块替换；
   `legacy.cpp:1429`）。
4. 若 `laser_type` 非空且存在 `laser_params` → 取 `base_type`（`new_mid360→mid360`、`new_vanjee→vanjee`、
   `new_robosense→robosense`）子树深合并到根（`legacy.cpp:1441-1452`）。
5. 逐键读取，缺键用上表默认值（`legacy.cpp:1454-1701`）。**注意这几个"看起来不合理但必须照抄"的默认**：
   `pitch_new=0`、`yaw_new=90`、`imu_in_car_frame_yaw_livox=65`、`use_2d_undistort=true`、
   `th_2d_range_min=0.5`、`test_wait_time=30`、`delete_pt_in_car_min_z=-0.5`（不是文件里的 -10）。
6. `yaw_calib` 读入后立即 `* M_PI / 180`（仅在存在该键时；缺键时是 `0`，不乘）。
7. 装载完成后在 `LegacyConfig::load` 内做：`use_delete_pts = use_delete_pts && isSkywalker50(model)`
   （照抄 `legacy.cpp:43`）。**注意这一步发生在装载之后**，所以 fixture 测试要覆盖"配置说 true 但 model 是
   XZ-SWGT → false"。

实现细节：`deepMerge(YAML::Node dst, const YAML::Node& src)` 递归合并映射节点（标量/序列整体覆盖）。
全部用 `try/catch(...)` 包住文件读与解析，异常只 `ROS_WARN`。

`legacyMapBase`：
```cpp
std::string legacyMapBase(const std::string& map_dir) {
    size_t e = map_dir.size();
    while (e > 0 && map_dir[e-1] == '/') --e;   // 去掉结尾的 '/'; "/" 会得到空串
    const std::string dir = map_dir.substr(0, e);
    if (dir.empty()) return "/map";             // 根 "/" (或空串) -> "/map"
    return dir + "/map";
}
```
**注意**（T1 实测修正）：计划初稿写的是 `while (e > 1 && ...)`，对 `map_dir == "/"` 会拼出
`"//map"`。已改为 `e > 0` 并对空结果特判，使 `/` → `/map`（与 T1 的 `LegacyMapBaseRoot` 用例一致）。
空 `map_dir` 也按根处理（宁可拼出绝对路径 `/map` 让写盘权限报错，也不拼成相对路径把地图写到 CWD）。

### 1.4 `test/test_legacy_config.cpp` + `test/fixtures/`

fixtures（提交到仓库，实际落地的清单）：
- `fixtures/slam_conf_min.json` — 只含 `pitch_new: 10`
- `fixtures/slam_conf_override.json` — `pitch_new: 33` + `laser_params.mid360.loc_z_max: 5`
  + `laser_params.vanjee.laser_sub_name: "vanjee_points722"`
- `fixtures/slam_conf_yaw_calib.json` — `{"yaw_calib": 90}`
- `fixtures/slam_conf_tiny_interval.json` — `{"lidar_pts_monitor_interval": 0}`
- `fixtures/slam_conf_log.json` — `xz_log_path` + `print_xz_log_to_file: true`
- `fixtures/sensor_vanjee_gt.json` — `{"type_3d": "WLR_722"}`
- `fixtures/sensor_robosense.json` — `{"type_3d": "Robosense"}`
- `fixtures/sensor_mid360.json` — `{"type_3d": "Livox_Mid360"}`

fixture 路径以编译宏 `LEGACY_FIXTURE_DIR` 传入（不依赖测试进程 CWD）。

测试用例（每个都是一个 `TEST`）：
1. `DefaultsWhenNothingExists` —— 传入全不存在的路径：`pitch_new==0`、`yaw_new==90`、
   `imu_in_car_frame_yaw_livox==65`、`laser_sub_name=="rslidar_points"`、`use_2d_undistort==true`、
   `lidar_pts_monitor_interval==10`、`yaw_calib==0`
2. `ProjConfOverridesDefaults` —— `proj_conf_json=fixtures/slam_conf_min.json` → `pitch_new==10`，其余仍默认
3. `OptSlamConfDeepMergesOverProj` —— 主配置 + `opt_slam_json=fixtures/slam_conf_override.json`
   → `pitch_new==33`；`laser_params.mid360` 子树合并生效（`loc_z_max==5`，需要机型命中
   `new_mid360`：model=`XZ-SWGT` + sensor 缺失 → `type_3d` 默认 `Livox_Mid360`）
4. `LaserParamsSubtreeMergedForBaseType` —— model=`XZ-SWGT`、`type_3d="WLR_722"` →
   `laser_type=="new_vanjee"`；`laser_params.vanjee.laser_sub_name` 生效（fixture 里给
   `laser_params.vanjee.laser_sub_name="vanjee_points722"`）→ `laser_sub_name=="vanjee_points722"`
5. `Mid360BaseTypeStripsNewPrefix` —— `new_mid360` → 合并 `laser_params.mid360` 子树
6. `RobosenseGtForcesTopicNames` —— `type_3d="Robosense"` + `XZ-SWGT` → `laser_type=="new_robosense"`、
   `laser_sub_name=="rslidar_points"`、`imu_sub_name=="rslidar_imu"`
7. `UnknownModelLeavesLaserTypeEmpty` —— model=`""`、`type_3d="Robosense"` → `laser_type==""`
8. `UseDeletePtsGatedByModel` —— 配置 `use_delete_pts: true`，model=`XZ-SWGT` → `false`；
   model=`XZ-SW50` → `true`
9. `YawCalibConvertedToRadiansWhenPresent` —— fixture `yaw_calib: 90` → `yaw_calib ≈ π/2`
10. `PkgConfUsedAsFallback` —— 只给 `pkg_conf_json`
11. `LegacyMapBaseStripsTrailingSlash` —— `/a/b/` → `/a/b/map`
12. `LegacyMapBaseWithoutTrailingSlash` —— `/a/b` → `/a/b/map`
13. `LegacyMapBaseLongPathNotTruncated` —— 40 字符目录 → 原样保留（旧实现 `substr(0,32)` 会截断，这是 D9 的有意修正）
14. `LegacyMapBaseRoot` —— `/` → `/map`

### 1.5 `CMakeLists.txt` 结构

```cmake
set(ROSIWIT_ROS1_LIB_SOURCES
    src/legacy_config.cpp
    # T2 追加 src/realtime_grid.cpp; T3 追加 src/lost_monitor.cpp;
    # T4 追加 src/slam_engine.cpp + src/rosiwit_interface.cpp;
    # T5 追加 src/legacy_convert.cpp + src/legacy_xz3dslam_interface.cpp
)
add_library(rosiwit_slam_ros1_lib ${ROSIWIT_ROS1_LIB_SOURCES})
target_include_directories(rosiwit_slam_ros1_lib PUBLIC src ${catkin_INCLUDE_DIRS})
target_link_libraries(rosiwit_slam_ros1_lib PUBLIC ${catkin_LIBRARIES} rosiwit_slam)

add_executable(rosiwit_slam_ros1_node src/rosiwit_slam_ros1_node.cpp)
target_link_libraries(rosiwit_slam_ros1_node rosiwit_slam_ros1_lib)
add_dependencies(rosiwit_slam_ros1_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
```

`package.xml` 增加：`<depend>xzrobot_ros_dependency</depend>`、`<build_depend>yaml-cpp</build_depend>`、
`<test_depend>gtest</test_depend>`。
`find_package(catkin ...)` 增加 `xzrobot_ros_dependency`，`catkin_package(CATKIN_DEPENDS ...)` 也加。
yaml-cpp 头文件：核心的 `rosiwit_slam` 目标把 `${yaml-cpp_INCLUDE_DIRS}` 声明为 **PRIVATE**
（`src/rosiwit_slam/CMakeLists.txt:60`），薄层不继承；但该路径就是 `/usr/include`（默认搜索路径），
所以 `#include <yaml-cpp/yaml.h>` 直接可用，且核心已 `PUBLIC` 链接 `yaml-cpp`，链接期也无需重复声明。
**不要**为此去改核心的 `PRIVATE`/`PUBLIC`。

gtest（每个测试文件一个 target，`catkin_add_gtest` 在 `CATKIN_ENABLE_TESTING` 下才生成）：
```cmake
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_legacy_config test/test_legacy_config.cpp)
  if(TARGET test_legacy_config)
    target_link_libraries(test_legacy_config rosiwit_slam_ros1_lib)
  endif()
endif()
```

**安装**：`install(DIRECTORY conf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})`、
`install(DIRECTORY scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})`。

**验证**：T0 的构建命令 + `catkin_make run_tests_rosiwit_slam_ros1` 全绿。
**提交**：`feat(ros1): 构建骨架 + xz_3d_slam 旧配置装载器`

### 1.6 T1 实测记录（已完成）

**状态**：`catkin_make`（含 `CATKIN_ENABLE_TESTING=ON`）`exit=0`；
`catkin_make run_tests_rosiwit_slam_ros1` `exit=0`；**gtest 19/19 全绿**（计划里 14 条 + 实测新增 5 条）。

实测新增的用例：
`ProjConfBeatsPkgConf`（proj 优先于 pkg）、`MonitorIntervalBelowOneForcedToTen`
（`<1` 强制 10）、`LogKeysFromConfAndDefaults`（`xz_log_path`/`print_xz_log_to_file`
来自 conf；缺失时用旧 node main 的回落值）、`LegacyMapBaseEmptyDir`、`LegacyMapBaseMultipleTrailingSlashes`。

实测发现并修正的 4 件事：

1. **计划初稿漏了两行逐键读取** —— 旧实现 `legacy.cpp:1535-1547` 会从主配置读
   `laser_sub_name` / `imu_sub_name`。初稿只保留了 `sensor_config` 阶段推出的值，导致
   `laser_params.vanjee.laser_sub_name` 这类覆盖根本不生效（被 `LaserParamsSubtreeMergedForBaseType` 抓到）。
   已补齐两行读取。
2. **`legacyMapBase` 初稿对根路径拼错**（`"/"` → `"//map"`），已按上文修正（`e > 0` + 空结果特判）。
3. **`laser_sub_name` / `imu_sub_name` 键缺失时的回落值有一个有意差别（记作 D21）**：
   旧实现回落硬编码 `"rslidar_points"`/`"zyz_imu_data"`，会**盖掉** `sensor_config` 阶段为
   Robosense 强制写入的话题名；我们改为保留上一步的结果（仅当 conf 里没有该键时）。
   随包发行的 `conf/slam_conf.json` 这两个键都在，故**实际部署行为与旧实现逐位一致**。
4. **`ROS_WARN_STREAM` 打印中文会变 `??????`**（melodic 的 ROS console 按 locale 处理非 ASCII）。
   已把全部 `ROS_WARN` 文案改为 ASCII 英文；`std::cout` 的启动日志保留中文（直写字节，显示正常）。
   这条是"上机器人后读日志"的硬要求，后续任务的所有 ROS 日志都照此办理。

其余偏差：`install(DIRECTORY scripts ...)` 推迟到 T12（`scripts/` 目录那时才建立，
提前写会让 `make install` 失败）。

---

## T2. `RealtimeGrid`：建图模式实时栅格

**目标**：建图模式下能周期产出一张真实数据的 2D 占据栅格（设计文档 §5.11 A）。

**为什么可以不改核心**：核心的栅格生成器与后处理都是**公开头文件**
（`include/slam_core/i_grid_map_generator.h`、`include/algorithms/grid_map/grid_map_factory.h`、
`include/algorithms/grid_map/grid_map_postprocess.h`），且核心被打成**单个** `rosiwit_slam` 库
（`file(GLOB_RECURSE src/*.cpp)`）、include 目录是 `PUBLIC`，薄层已经 `target_link_libraries(... rosiwit_slam)`。
核心私有的只有 `PcdMapManager::generateGridMap()`。

**关键事实（决定测试怎么写）**：
- `GridMapFactory::create("height_projection")` 返回 `IGridMapGenerator`；
  `build(cloud, origin, cfg, info)` 签名见 `i_grid_map_generator.h:62-65`。
- `height_projection_grid_map.cpp:94-95` 的索引是 `col=(x-origin_x)/res`、`row=(y-origin_y)/res`，
  **row 0 = 最小 y**。
- `height_projection` 只产出 **0(自由) / 100(占据)**，没有 unknown（`height_projection_grid_map.cpp:105-112`）。
  旧 `/map` 还有 **10(unknown)**（`legacy.cpp:1745`）—— 这是无法避免的偏差，记 D17。
- `applyGridPostProcess(GridInfo&, const GridPostConfig&)` 在
  `include/algorithms/grid_map/grid_map_postprocess.h:41`；`parsePostPipeline("a,b,c")` 在同文件 :44。
- 核心的栅格配置键（`config/default.yaml:30-51`）：`grid_map_generator`、`grid_min_height`、
  `grid_max_height`、`grid_occupied_thresh`、`grid_dilate_radius`、`grid_close_radius`、
  `grid_min_component_cells`、`grid_max_range`、`grid_ground_method`、`grid_ransac_dist_thr`、
  `grid_min_hits`、`grid_min_misses`、`grid_mark_radius`、`grid_post_pipeline`、`grid_connectivity`、
  `grid_open_radius`、`grid_fill_max_hole_cells`、`grid_skeleton_width`、`grid_line_min_len_m`、
  `grid_axis_snap`。
  映射到 `GridMapConfig`/`GridPostConfig` 的字段名必须**逐字段对照**这两个头文件，
  这就是 D15 说的"镜像配置"——在代码里用注释标出耦合点。

**文件**：`src/realtime_grid.h`、`src/realtime_grid.cpp`、`test/test_realtime_grid.cpp`

`src/realtime_grid.h`：
```cpp
namespace rosiwit_slam_ros1 {

/// 纯计算部分: 从世界点云生成栅格 (与核心同一生成器 + 同一后处理)
bool buildGridFromPoints(const rosiwit_slam::PointVec& pts,
                         const rosiwit_slam::GridMapConfig& cfg,
                         const rosiwit_slam::GridPostConfig& post,
                         const std::string& generator,
                         rosiwit_slam::GridInfo& out);

/// 从 YAML 读栅格配置 (键名见 config/default.yaml:30-51)
void loadGridConfigFromYaml(const std::string& yaml_path,
                            rosiwit_slam::GridMapConfig& cfg,
                            rosiwit_slam::GridPostConfig& post,
                            std::string& generator);

/// worker 线程封装: 按 hz 节流, 调 source() 取点云并重算, 结果加锁给发布方
class RealtimeGrid {
public:
    using Source = std::function<bool(rosiwit_slam::PointVec&)>;
    RealtimeGrid();  ~RealtimeGrid();
    void setConfig(const rosiwit_slam::GridMapConfig&,
                   const rosiwit_slam::GridPostConfig&, const std::string& generator);
    void setSource(Source s);
    void start(double hz);          // hz<=0 时用 1.0
    void stop();
    bool latest(rosiwit_slam::GridInfo& out) const;   // 有就用最新一张, 返回是否有效
};
} // namespace
```
实现要点：单独 `std::thread` + `std::condition_variable` 定时；`getGlobalMap` 与重算是 O(点数)，
**绝不能跑在 ROS 回调线程上**（否则拖累 100Hz 的 `curr_localization_pose` 与 `map→odom` TF）。
`latest()` 用 `std::mutex` 保护一张 `GridInfo`。

**测试** `test/test_realtime_grid.cpp`：
1. `BuildsGridFromSyntheticWall` —— 造一堵墙：x∈[-3.0,3.0]、y=0、z=1.0、**点间距 0.01 m**，
   `grid_max_height=2.0` → `buildGridFromPoints` 返回 true；`width>0 && height>0`；
   `resolution≈0.05`；`data` 里至少有 1 个 `100`
   （墙长与点间距是被实测逼出来的，见下面 T2 实测记录 —— 计划初稿写的 x∈[2.0,3.0] / 0.05 m 是错的）
2. `ReturnsFalseOnEmptyInput` —— 空点云 → 返回 false
3. `FloorPointsBecomeFreeNotOccupied` —— z=0.0 的点（低于 `grid_min_height=0.1`）→ 没有 `100`
4. `RowZeroIsMinY` —— 在 y=+5 造墙，断言占据格的 row 索引大于 height/2（钉住行序，防以后被"修正"）
5. `LatestIsNullBeforeStart` / `LatestHasDataAfterStart` —— 用 `setSource` 喂固定点云，
   `start(20)` 后 sleep 200ms，`latest()` 为真
6. `LoadsGridConfigFromDefaultYaml` —— 读本包 `config/default.yaml` → `generator=="height_projection"`、
   `cfg.min_height≈0.1`、`cfg.max_height≈2.0`、`post.pipeline=={"denoise","close","dilate"}`

**验证**：同上构建 + 单测。
**提交**：`feat(ros1): 建图实时栅格(复用核心公开栅格生成器)`

### 2.1 T2 实测记录（已完成）

**状态**：`catkin_make` `exit=0`；`catkin_make run_tests_rosiwit_slam_ros1` `exit=0`；
**gtest 30/30 全绿**（legacy_config 19 + realtime_grid 11）。

实测新增的用例（计划里 7 条 → 11 条）：`SourceReturningFalseProducesNothing`、
`NonPositiveHzFallsBackTo1`（`hz<=0` 不能变成忙等）、`UnknownGeneratorFallsBack`
（未知生成器名回落 `height_projection`，对齐 `pcd_map_manager.cpp:109-115`）、
`MissingYamlUsesDefaults`。

实测发现的 3 件事：

1. **合成墙必须"点间距 ≪ 分辨率 + 足够长"，否则测不出任何占据格**（首次跑 4 条用例全挂）。
   生成器自己的日志把原因说得很清楚：
   ```
   [GridMap] 123x3 occupied=87 total=369 res=0.05
   [Post:denoise] comps=35 removed_cells=87 min_group=30
   ```
   计划初稿的"x∈[2.0,3.0] 每 0.05 m"有两个坑叠加：
   - **点间距正好等于分辨率**时，`VoxelGrid`(leaf=0.05) 取体素质心后落点与格边界对齐，
     会**稳定**留下空洞 → 6 m 的墙被拆成 **35 个连通域**（87 格），
   - 每个碎片都 < `min_component_cells=30`，被后处理第一步 `denoise` **全部删除** → `100` 数为 0。
   （即使不碎，1 m 的墙也只有 20 格 < 30，同样会被删。）
   改为**点间距 0.01 m（分辨率的 1/5）+ 墙长 6 m（≈120 格）**后一次通过。
   这两条已写进测试文件的注释里，防止以后有人"简化"成等间距。
   注意：**这不是代码 bug**，是核心后处理的既定行为（我们不改核心，只镜像配置，D15），
   真实雷达的点云比这密得多，且产线分辨率是 `map_resolution=0.3`。
2. **`cfg.ground_z` 必须显式钉住**（测试里设 `0.0`）。`height_projection` 的高度是相对
   **检测出的地面平面**算的（`ground_segmentation.cpp`），不钉住的话合成数据的地面拟合不可控
   （也正因如此，`fitGround` 的 `ransac` 分支在只有一堵墙的点云上会退化）。
3. **`height_projection` 只产 0/100，没有 10(unknown)** —— 实测确认了计划里的 D17。

另外：`loadGridConfigFromYaml` 额外读了 `map_resolution` / `map_save_voxel` 两个**非 `grid_*` 键**，
因为核心是在 `generateGridMap()` 里把 `cfg.resolution` 和 `cfg.point_spacing` 赋成这两个值的
（`pcd_map_manager.cpp:271-281`）。薄层没有别的来源，所以在装载时一并定下来，语义与核心等价；
代码里已注明这层耦合。

---

## T3. `LostMonitor`：定位丢失状态机

**目标**：产出旧 `SlamInfo.location_status` 的 0/1/2，并在 0→2 的边沿发出一次"丢失"信号。

**信号来源**（设计文档 §5.11 B）：`ISlamAlgorithm::getPoseCov()` 在核心 `m_status == LOST` 时返回
`false`（`localization_pipeline.cpp:641`），而 `m_status` 由真正的 `MonitorLevel` 驱动
（`localization_pipeline.cpp:620-629`）。`rosiwit_slam_ws` 无需改核心。

**必须处理的两个副作用**（写进代码注释，别只写在文档里）：
- `m_status == INIT`（刚加载地图、还没定位上）时 `getPoseWithCov` 返回 **true** 并给出默认位姿 →
  必须用"是否曾经 LOCALIZED 过"的 latch 区分，否则把 INIT 误报成 SUCCESS。
- 核心把 `MonitorLevel::DEGRADED` 映射成 `LOCALIZED`（`localization_pipeline.cpp:627`）→
  区分不出 CREDIBLE / DEGRADED，所以 1(RECOVERING) 只用于"重定位进行中"，不用于"降级"（D11）。

**文件**：`src/lost_monitor.h`、`src/lost_monitor.cpp`、`test/test_lost_monitor.cpp`

```cpp
namespace rosiwit_slam_ros1 {

enum class LocStatus : int8_t { SUCCESS = 0, RECOVERING = 1, FAILED = 2 };

class LostMonitor {
public:
    /// 进入定位/换图/重定位时调用: 清 latch, 回到"还没定位上"的初始态
    void reset();
    /// 收到一次真实定位输出(SlamOutput.has_new_pose 且处于定位模式)时调用
    void onLocalizedPose();
    /// 地图是否已加载(FAILED 的使能条件)
    void setMapLoaded(bool loaded);
    /// 每个 tick 调一次, 传入 core.getPoseCov() 的结果
    LocStatus update(bool pose_cov_ok);
    /// 一次性边沿: 上一次 update 从"非 FAILED"翻到 FAILED 时返回 true 并清标志
    bool consumeLostEdge();
};
} // namespace
```
`update` 规则：
```
!map_loaded              -> RECOVERING(1)      // 定位链路还没就绪
!ever_localized_         -> RECOVERING(1)      // 等价于旧 relocating_==true
ever_localized_ && !cov  -> FAILED(2)  (+ 置 lost_edge)
ever_localized_ &&  cov  -> SUCCESS(0)
```
`onLocalizedPose()`：`ever_localized_=true`，并且若当前是 FAILED 则清 lost_edge（恢复不算新丢失）。

**测试**：
1. `NotLoadedIsRecovering` —— `setMapLoaded(false)` → 恒 1，`consumeLostEdge()` 恒 false
2. `LoadedButNeverLocalizedIsRecovering` —— `setMapLoaded(true)`、`update(true)` → 1（**这条就是 INIT 陷阱的回归测试**）
3. `AfterLocalizedIsSuccess` —— `onLocalizedPose(); update(true)` → 0
4. `LostAfterLocalizedIsFailed` —— `onLocalizedPose(); update(false)` → 2
5. `LostEdgeFiresExactlyOnce` —— 连续 3 次 `update(false)` → `consumeLostEdge()` 依次 true/false/false
6. `RecoveryReturnsToSuccessAndDoesNotRefire` —— `update(false)`(2) → `update(true)`(0) → `consumeLostEdge()` false
7. `ResetReturnsToRecovering` —— 定位成功后再 `reset()` → `update(true)` 得 1
8. `LostAgainAfterRecoveryFiresAgain` —— 2 → 0 → 2，第二次边沿仍要能报

**验证**：同上。
**提交**：`feat(ros1): 定位丢失状态机(getPoseCov 驱动)`

### 3.1 T3 实测记录（已完成）

**状态**：`catkin_make` `exit=0`；`run_tests` `exit=0`；**gtest 38/38 全绿**
（legacy_config 19 + realtime_grid 11 + lost_monitor 8）。

计划里 T3 的 8 条用例一字未改，一次通过。计划**没写清、实现时必须自己定**的两件事（已写进代码注释）：

1. **`reset()` 不能清 `map_loaded_`**。T11 的接线是"换图 → `reset()`，加载成功后再 `setMapLoaded(true)`"，
   而"重定位 → `reset()`"根本不换地图。若 `reset()` 顺手把 `map_loaded_` 置 false，
   重定位之后就再没任何地方把它置回 true，`update()` 会永久停在 `RECOVERING(1)`、
   永远报不出 FAILED。所以 `reset()` 只清 `ever_localized_` / `last_failed_` / `lost_edge_`。
2. **`update()` 走到 SUCCESS 时也要清 `lost_edge_`**（不只是 `onLocalizedPose()` 里那条）。
   这是用例 6 `RecoveryReturnsToSuccessAndDoesNotRefire` 直接要求的：
   `update(false)`(2) → `update(true)`(0) → `consumeLostEdge()` 必须为 false。
   含义是"**恢复不算新丢失**"这条规则对"经由 `update` 恢复"同样成立。
   生产上不会因此漏报：T11 里 `update()` 和 `consumeLostEdge()` 是同一个 100Hz 定时器里**紧接着**调的，
   真实丢失必然在恢复之前就被消费掉；被作废的只是"丢失后又立刻恢复、发布方还没看过"的那种瞬时抖动
   （这种本来也不该报故障码）。

另外：`LostMonitor` **没有加锁**，因为 T11 把它全部放在同一个 100Hz 定时器里调用（`onLocalizedPose()`
所需的 `has_new_pose` 从 `engine.latestOutput()` 的快照里读，不跨线程），已在头文件写死"单线程调用"约定。

---

## T4. `SlamEngine` + `input_profile` + launch

**目标**：把核心驱动从 `rosiwit_slam_ros1_node.cpp` 抽成 `SlamEngine`（唯一核心持有者），
新增 `~input_profile`，产出两个 launch。

**基线文件**：`src/rosiwit_slam_ros1_node.cpp`（现 376 行，单类 `RosiwitSlamRos1Node`，
`main` 在 369-376）。搬移是**纯重构**：`rosiwit` profile 下行为必须和现在完全一致。

`src/slam_engine.h` 需要暴露给两套 interface 的东西：
```cpp
class SlamEngine {
public:
    SlamEngine(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    bool ok() const;                                   // init 失败时为 false
    rosiwit_slam::ISlamAlgorithm* algo() const;        // 可能为 nullptr
    rosiwit_slam::SlamBase* base() const;              // 用于 tryPopAndProcess
    const std::string& inputProfile() const;           // "rosiwit" | "legacy"
    const std::string& configPath() const;             // 交给 core.init 的路径(可能已被 override)
    // 输出快照(线程安全)
    bool latestOutput(rosiwit_slam::SlamOutput& out) const;
    bool getGlobalMap(rosiwit_slam::PointVec& out) const;
    bool getGridInfo(rosiwit_slam::GridInfo& out) const;
    bool getPoseCov(rosiwit_slam::PoseStamped& out, Eigen::Matrix<double,6,6>& cov) const;
    void onImuMsg(const sensor_msgs::Imu::ConstPtr&);
    void onLidarMsg(const sensor_msgs::PointCloud2::ConstPtr&);
    void onOdomMsg(const nav_msgs::Odometry::ConstPtr&);
    void onInitialPoseMsg(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);
    void setOutputHook(std::function<void(const rosiwit_slam::SlamOutput&)>);  // legacy 侧挂载
    // 新接口的发布器/服务句柄, 供 rosiwit_interface 使用
    ...
};
```

### 4.1 `~input_profile`
- 默认 `rosiwit`（M12）。`pnh_.param<std::string>("input_profile", profile, "rosiwit")`。
- `rosiwit`：完全按现状（`~config_file` YAML → `config_path`，订阅 `imu_topic`/`lidar_topic`）。
- `legacy`：`config_path` 仍是 `~config_file`（核心 YAML），但**额外**用 T1 的 `LegacyConfig` 读出旧配置，
  并以此决定输入话题名/频率/栅格参数。注入核心的 override YAML（`map_directory: "/"`）在 **T10** 做。
  T4 只把 `profile` 与 `LegacyConfig` 存好，先不接发布器/服务。

### 4.2 launch
- `launch/rosiwit_slam.launch`：**不改**（但显式补 `input_profile:=rosiwit`？**不要**——
  它是"不变"档，靠默认值即可）。
- `launch/xz_3d_slam.launch`（drop-in）：逐字复制旧 launch 的第 1-33 行，只把第 34 行改成
  ```xml
  <node name="xz_3d_slam" pkg="rosiwit_slam_ros1" type="rosiwit_slam_ros1_node" output="log">
    <param name="input_profile" value="legacy" />
    <param name="config_file" value="$(find rosiwit_slam_ros1)/config/default.yaml" />
  </node>
  ```
  注意旧第 34 行的原始内容必须先在 T4 开始时从
  `E:\workspace_slam\3d_slam\src\xz_3d_slam\launch\xz_3d_slam.launch` 读出来（行 34 是 SLAM 节点那行），
  不要凭印象写。
- `launch/legacy_dev.launch`（开发机用，不含机器人上的外部节点，让我们能单独起 legacy 模式）：
  ```xml
  <launch>
    <arg name="model" default="XZ-SWGT" />
    <param name="/machine/model" type="string" value="$(arg model)" />
    <node pkg="rosiwit_slam_ros1" type="rosiwit_slam_ros1_node" name="xz_3d_slam" output="screen">
      <param name="input_profile" value="legacy" />
    </node>
  </launch>
  ```

**验证**：
1. 构建通过。
2. `roslaunch rosiwit_slam_ros1 legacy_dev.launch` 能起来，`rosnode info /xz_3d_slam` 显示
   节点名正确；`rostopic list` 里仍出现现有新接口话题（`lio_odom`、`cloud_map`、`grid_map`、
   `body_cloud`、`world_cloud`、`lio_path`）。此阶段 legacy 话题还没加，属预期。
3. `roslaunch rosiwit_slam_ros1 rosiwit_slam.launch` 行为与改动前一致（`lio_odom` 有输出或至少节点正常启动）。

**提交**：`refactor(ros1): 抽出 SlamEngine, 新增 input_profile 与 legacy launch`

### 4.3 T4 实测记录（已完成）

**状态**：`catkin_make` `exit=0`；`run_tests` `exit=0`；**gtest 38/38 全绿**（无新增用例）。
两个 launch 都在本机真跑过，用 `rosnode info` / `rostopic info` / `rosservice list` 逐项核对：

| 项 | `input_profile=rosiwit` | `input_profile=legacy` |
|---|---|---|
| 节点名 | `/rosiwit_slam` | `/xz_3d_slam` ✓ |
| 订阅 | `/imu`、`/lidar`、`/initialpose` | `/vanjee_points722`、`/vanjee_lidar_imu_packets`、`/initialpose` ✓ |
| 发布 | `lio_odom`、**`odom`**、`lio_path`、`body_cloud`、`world_cloud`、`cloud_map`、`grid_map`、**`map`** | `lio_odom`、`lio_path`、`body_cloud`、`world_cloud`、`cloud_map`、`grid_map` |
| `/odom` | 有发布者 ✓ | `rostopic info /odom` → `ERROR: Unknown topic` ✓ |
| `/tf`、`/tf_static` | 有 ✓ | `rostopic info /tf` → `ERROR: Unknown topic` ✓ |
| `/map` | 有 ✓ | 没有 ✓（M3：legacy 下 `/map` 归旧接口独占） |
| 服务 | `/save_map`、`/load_map`、`/save_grid_map`、`/set_slam_mode` ✓ | 同左（旧服务 T8 再加，无重名） |

实测发现并处理的 5 件事：

1. **M5/M3 在 T4 就必须落成代码，不能只"把 profile 存好"**。计划 T4 正文写的是
   "T4 只把 profile 与 LegacyConfig 存好，先不接发布器/服务"，但 `RosiwitInterface` 本身
   是在 T4 建起来的，它一建起来就会去 advertise/publish `odom`、`map` 和两条 TF。
   所以 T4 里就加了 profile 抑制，否则 legacy 下会立刻出现
   "新接口的 `odom` 发布者 + 静态 `map→odom` TF" 与旧接口抢同一条边 —— 那正是 M5 要避免的。
   - `odom` 别名：legacy 下**连 advertise 都不做**（第一次实测只 gate 了 publish，
     `rosnode info` 里仍多出一条永远没数据的发布者，已改成不 advertise）。
   - TF：legacy 下不建 `TransformBroadcaster` / `StaticTransformBroadcaster`。
   - `/map`：legacy 下不 advertise 也不 publish。
2. **drop-in launch 还必须传 `config_file`**（计划里的 launch 片段漏了）。原因：旧节点
   没有 `config_file` 这个参数（它从 `PROJECT_PATH/conf/slam_conf.json` 读自己的配置），
   而我们的核心需要一份**核心 YAML**。不传的话 `init("")` 走核心的内置默认，行为不可控。
   已在本包 launch 里补 `<param name="config_file" value="$(find rosiwit_slam_ros1)/config/default.yaml" />`。
3. **`ROS_INFO` 在 `roslaunch` 下是块缓冲的，`roslaunch ... | head` 看不到启动行**（实测确认）。
   而"上机器人后核对生效参数"是硬需求（见 §9 的 C 项）。因此定下与 T1 那条 ASCII 规则并列的
   第二条日志约定：**所有要被人读的启动行一律 `std::cout << ... << std::endl`**
   （`endl` 强制 flush）。原 `rosiwit_slam_ros1 ready:` 那行保持 `ROS_INFO` 原样不动，
   新信息另起一行 `[slam_engine] profile=... config=... imu=... lidar=...` 用 `std::cout`。
4. **旧配置路径用 `ros::package::getPath("rosiwit_slam_ros1")` 等价替换 `PROJECT_PATH`**。
   查了 `xz_3d_slam/CMakeLists.txt:24/26`：`PROJECT_PATH` 装机后是
   `/opt/ros/melodic/share/xz_3d_slam`、开发时是 `${PROJECT_SOURCE_DIR}`，主配置恒为
   `PROJECT_PATH "/conf/slam_conf.json"`。`ros::package::getPath` 在 devel/overlay 下给源码目录、
   在 install 下给安装目录，两种情形都对得上（实测日志里打出的是源码目录下的 `conf/slam_conf.json`）。
5. **顺手确认了话题名的来源**（本来以为 `laser_params.mid360` 会带话题名，查过才知道不是）：
   `conf/slam_conf.json` 的**顶层**就是 `laser_sub_name: "vanjee_points722"` /
   `imu_sub_name: "vanjee_lidar_imu_packets"`，而 `laser_params.mid360` 子树**只有 z 范围**、
   没有话题名。所以实测日志里 `laser_type='new_mid360'` + `laser_sub='vanjee_points722'`
   是顶层键生效，不是子树合并来的 —— 与旧实现一致，也再次印证 D21 那条
   "发行 conf 两个键都在，故实际部署行为逐位一致"。

另外：`legacy_dev.launch` 用 `model=XZ-SWGT` 起，实测日志显示
`slam_init_type='free'`（来自发行 conf）—— 这条会影响 T5/T11 的初始 `slam_state`，先记下。

---

## T5. `legacy_convert` + 旧接口发布器（14 话题）

**目标**：14 个旧话题全部注册并按旧条件发布；新旧 `/map` 冲突按模式分流解决。

### 5.1 `src/legacy_convert.h/.cpp`（纯逻辑，只依赖 ROS 消息类型，可单测）

```cpp
namespace rosiwit_slam_ros1 {

/// 旧 /map 语义: 行序 = 核心栅格原样 (row 0 = 最小 y), 不翻转
nav_msgs::OccupancyGrid toLegacyGrid(const rosiwit_slam::GridInfo& gi, const std::string& frame);
/// 新语义: 竖直翻转 (row 0 = 最大 y), 值域保留核心的 0/100/-1
nav_msgs::OccupancyGrid toFlippedGrid(const rosiwit_slam::GridInfo& gi, const std::string& frame);

/// 按设计文档 §5.4 组装 slam/info。matching_degree 恒 0(D3), update_request 恒 0
xzrobot_ros_dependency::SlamInfo buildSlamInfo(
    int8_t slam_state, int8_t location_status, const std::string& map_name,
    double x, double y, double yaw, float map_area);

/// 告警 JSON (legacy.cpp 的 faults 结构, module 恒 "xz_3d_slam")
std::string buildWarningJson(const std::string& code, const std::string& level,
                             const std::string& name, const std::string& content);

/// 点云转换: 核心 CloudType -> PointCloud2; 以及 -> 旧的 sensor_msgs/PointCloud
sensor_msgs::PointCloud2 toPointCloud2(const rosiwit_slam::CloudType& c,
                                       const std::string& frame, const ros::Time& stamp);
sensor_msgs::PointCloud  toOldPointCloud(const rosiwit_slam::CloudType& c,
                                         const std::string& frame, const ros::Time& stamp);

/// 旧 TF 用的 yaw: atan2(R(0,1), R(0,0))  (legacy.cpp:1921)
double legacyYawFromRot(const Eigen::Matrix3d& R);
} // namespace
```

**测试** `test/test_legacy_convert.cpp`：
1. `LegacyGridKeepsRowOrder` —— 造 3x2 的 `GridInfo`（width=3,height=2，data[0..5] 互不相同）
   → `toLegacyGrid(...).data == 原 data`
2. `FlippedGridFlipsRows` —— 同一个 `GridInfo` → 首行等于原最后一行
3. `LegacyGridHeaderAndInfo` —— `frame=="map"`、`resolution`/`origin_x`/`origin_y` 透传、`width/height` 正确
4. `SlamInfoFieldMapping` —— 逐字段断言（含 `map_name`、`robot_pose.theta`）
5. `WarningJsonShape` —— 用 `yaml-cpp` 反解（或字符串包含）断言
   `faults[0].module == "xz_3d_slam"`、`code`/`level`/`name`/`content` 都在
6. `LegacyYawFromIdentityIsZero` / `LegacyYawFromPlus90degIsPlusHalfPi`
   —— 钉住 `atan2(R(0,1), R(0,0))`：绕 Z 转 +90° 时 R(0,1)=1、R(0,0)=0 → `legacyYawFromRot` 返回 **+π/2**；
   而 TF 里用的是 **`-yaw`**（`legacy.cpp:1922`），这条测试保证以后不把符号弄反

### 5.2 `src/legacy_xz3dslam_interface.h/.cpp` —— 发布侧

逐条照抄 `legacy.cpp:273-300`（advertise）与 `legacy.cpp:333-511`（发布逻辑）：

| 话题 | 类型 | queue | latch | frame | 频率 | 实现 |
|---|---|---|---|---|---|---|
| `global_map` | PointCloud2 | 1 | **true** | `map` | 进程内一次（`debug_publish` 或定位模式） | `engine.getGlobalMap()` |
| `global_grid_map` | OccupancyGrid | 1 | **true** | `map` | 同上 | `toFlippedGrid(engine.getGridInfo())` |
| `/map` | OccupancyGrid | 10 | **false** | `map` | 见下 | **模式分流** |
| `curr_scan_point` | PointCloud2 | 10 | false | `map` | 0.1s | `SlamOutput.world_cloud`（无则用 body→map 变换） |
| `scan_f_map` | **`sensor_msgs/PointCloud`** | 10 | false | `map` | 0.1s | `curr_scan_point` 降采样后转旧类型 |
| `curr_submap_point` | PointCloud2 | 10 | false | `map` | 0.1s + `debug_publish` | **注册不发**（D2） |
| `or_curr_scan_point` | PointCloud2 | 10 | false | `map` | 0.1s，仅定位 | 当前帧原始输入点云 |
| `curr_localization_pose` | PoseStamped | 10 | false | `map` | **0.01s** | `SlamOutput.pose`，**两种模式都发**（`legacy.cpp:1117-1205`） |
| `mapping_path` | Path | 10 | false | `map` | 0.1s，仅建图 | 累积位姿 |
| `loop_constraint` | MarkerArray | 10 | false | `map` | 0.1s，仅建图 | **注册不发**（D2） |
| `slam/info` | `xzrobot_ros_dependency/SlamInfo` | 10 | false | 无 header | **0.01s** | 见 §5.3 字段表 |
| `points_2d` | PointCloud2 | 10 | false | `map` | — | **注册不发**（旧实现本身也没发过） |
| `slam_warning_code` | String | 10 | false | — | 周期同 `rt_grid_map`（缺省 1s） | T7 |
| `origin_point` | PointCloud2 | 10 | false | `map` | 每帧雷达 | 原始输入点云 |

**定时器**（照抄 `legacy.cpp:247-271`）：
- `pub_time_gap` **硬编码 0.1s**（`legacy.cpp:254-255` 先按 `publish_hz` 算完又覆盖为 0.1，
  `publish_hz` 实际无效 —— 保留这个行为，见 M2）
- pose 定时器 **0.01s**
- `rt_grid_map` 定时器 = `1.0 / rt_grid_map_publish_hz`（缺省 1s）
- `slam_warning_code` 定时器与 `rt_grid_map` 同周期

**`/map` 模式分流（设计文档 §5.3 冲突 1）**：
- `legacy` profile 下本接口注册 `/map`（queue 10，**不 latch**）：
  - **MAPPING** → 用 T2 的 `RealtimeGrid` 产出的栅格，`toLegacyGrid()`（**不翻转**），
    并按旧实现把 `map_area` 塞进 `origin.position.z = map_area * 1e-5`（`legacy.cpp:1749`），
    周期 = `1/rt_grid_map_publish_hz`
  - **LOCALIZATION** → `toFlippedGrid(engine.getGridInfo())`，**周期重发**（不 latch，见下方偏差注）
- `rosiwit` profile 下 `/map` 仍由 `SlamEngine` 持有（现状：latch，2s 定时器只发一次），本接口不注册 `/map`

> **偏差注（记 D18）**：`legacy` profile 下定位模式的 `/map` 改为**周期不 latch**重发，
> 而不是旧实现的"一次"（旧实现其实是 MAPPING 才周期发）。原因：`/map` 只有一个发布器，
> 不能既 latch 又不 latch。副作用是晚订阅者最多等一个周期（≤1s）拿到数据，比 latch 更安全（不会拿到旧模式残留）。

### 5.3 `/slam/info` 字段映射（照抄 `legacy.cpp:1823-1896`）

| 字段 | 值 |
|---|---|
| `slam_state` | 定位模式→1；建图模式→0；无核心/异常→2；FREE 时期→1；**`relocating_` 为真时强制 1**（`legacy.cpp:1891-1892`） |
| `location_status` | **分模式**（`legacy.cpp:1839-1894`）：<br>• **MAPPING** → 恒 `0`<br>• **FREE** → 恒 `1`<br>• **LOCALIZATION** → `relocating_`（= 核心 `isRelocalization()`）为真 → `1`；否则 `getLocalizationStatus()` ∈ {0,1} → `0`，其它 → `2`。我们用 T3 的 `LostMonitor` 等价表达（`0/1/2`）<br>• 最后统一覆盖：`relocating_` 为真时**强制** `1` |
| `update_request` | 恒 0 |
| `matching_degree` | 恒 0（D3） |
| `map_name` | `curr_map_name_`，初值 `"NULL"`，`/switch_map` 时置 `request.map_name` |
| `robot_pose` | `x/y/yaw`，任一 NaN 则三者归零（`legacy.cpp:1826-1830`） |
| `map_area` | 建图模式用 `width*height*resolution²`；其它模式 0 |

### 5.4 ⚠ T5 发布侧：**重新读源码后的修正**（实现前必读）

上面 §5.2 的表是上一轮凭记忆/摘要整理的，**多处不准**。这一轮把
`legacy.cpp:333-371`（`startPub`）、`826-1110`（各发布函数）逐行读了一遍，实际行为如下。
（`warning_code` 侧已核实：`setModule(AlgorithmModule_SLAM)` → `module="xz_3d_slam"`
且 `begin_code_=20003001`（`json_warning_code.cpp:26-29`）；`type` 成员默认就是
`"algorithm fault"`（`json_warning_code.h:68`）；`toJson(vector<int>)` 输出
`{"faults":[{...}]}`，每项 = `warning_code.json` 里那条 + `type`/`module`/`code`(字符串)。
`code` 类字段全是 nlohmann 默认的**字母序**：code, content, level, module, name, type。）

**1) 各话题的真实发布条件（`startPub()` + 各 publish 函数）**

| 话题 | 真实条件 |
|---|---|
| `global_map` | **仅 `debug_publish_`**，且每次进模式只发一次（`debug_map_published_`）。定位用 `localiza.getGlobalMap()`，建图用 `mapping.getGlobalMap()`。类 latch 但不是 ROS 的 latch |
| `global_grid_map` | 两个来源：① 定位模式 `set_grid_map_` 为真时**发一次**并清标志（`set_grid_map_` 由 `LocalizationSetGridMap()` 读 `<map_base>.pgm/.yaml` 后置真，`legacy.cpp:373-455`）；② `debug_publish_` 下走 `publishPointMap()`。**不是**简单的"跟 global_map 一起发" |
| `/map` | `publishRealtimeGridMap()`：**只要 MAPPING，每个周期都发**（不管有没有更新）；`checkRealtimeGridMapChange()` 只决定要不要重算数据，发布是无条件的 |
| `curr_scan_point` | 定位：**仅 `debug_publish_`**；建图：**无条件**。内容见下 |
| `scan_f_map` | **无条件**（两种模式都发）。内容是"降采样 + z 过滤后"的点云转旧类型，不是 `curr_scan_point` 降采样 |
| `or_curr_scan_point` | 定位：**无条件**；建图：**不发**（`legacy.cpp:1102` 那行是注释掉的） |
| `curr_submap_point` | 仅 `debug_publish_`（建图还要 `get_imu_data_ && get_laser_data_`）→ 我们按 D2 注册不发 |
| `loop_constraint` | 仅 `debug_publish_`，且 `publishLoopBetweenKF()` 只在有回环时才发 → 我们按 D2 注册不发 |
| `points_2d` | 旧实现里 `scan_puber_` **从未 publish** → 注册不发 ✓（这条原表是对的） |
| `mapping_path` | 建图且 `get_imu_data_ && get_laser_data_` 时，每 0.1s **无条件**发（整条 path 全量重发，不是增量） |

**2) `curr_scan_point` / `scan_f_map` / `or_curr_scan_point` 的真实内容（`legacy.cpp:966-1110`）**

定位分支（`getCurrScan()` = 当前帧**已转到 map 系**的点云）：
- 降采样到约 2000 点：`down_sample = (N < 2000) ? 1 : round(N/2000)`，按 `i % down_sample == 0` 抽
- **z 过滤**：`p.z > 0 && p.z < 2.0`（map 系下的 z！）
- 过滤后的点云 → `scan_f_map`（`PointCloud2` → `PointCloud`），**无条件发**
- 同一个过滤后的点云也是 `curr_scan_point` 的内容，但**只有 `debug_publish_` 才发**
- `getCurrScanOr()`（原始/畸变帧）→ `down_sample *= 2`，**不做 z 过滤**，无条件发 `or_curr_scan_point`

建图分支：
- `curr_scan = getCurrUndistortPointCloud()`、`curr_or_scan = getCurrDistortPointCloud()`
- 同样降到约 2000 点，**z 过滤是 `p.z > 0 && p.z < 1.0`**（注意是 1.0，不是定位的 2.0）
- 过滤后的点云 → `scan_f_map`，无条件发
- **然后 `pcl::toROSMsg(*curr_scan, curr_ros_scan)` 把 `curr_ros_scan` 覆盖成"完整的去畸变点云"**
  （过滤白算了）→ 无条件发 `curr_scan_point`
- `or_curr_scan_point` 不发

**3) 这些差异在我们这边的处置**

核心没有 `getCurrScanOr()`/`getCurrDistortPointCloud()`（原始畸变帧）这类接口，
也没有"map 系下的当前帧"以外的中间量。所以映射关系是这样（写进代码注释）：

| 旧 | 我们的等价物 |
|---|---|
| `localiza/mapping.getCurrScan()`、`getCurrUndistortPointCloud()` | `SlamOutput.world_cloud`（当前帧已转到世界系） |
| `getCurrScanOr()`、`getCurrDistortPointCloud()` | **T6 订阅侧缓存的"当前帧原始输入点云"**（落在 `origin_point` 同一个缓存里） |
| `mapping.getCurrMappingPath()` | `curr_localization_pose` 的累积 path（原表也这么写） |
| `getGlobalMap()` | `engine.getGlobalMap()` |
| `getRealtimeGridMap()` → `updateRealtimeGridMap()` | T2 的 `RealtimeGrid`（D17：只有 0/100，没有 10=unknown） |

**但降采样阈值（约 2000 点）与 z 过滤（定位 2.0 / 建图 1.0）必须照抄** ——
这是下游（导航/感知/可视化）习惯的数据形态，不能省。`scan_f_map` 必须保持旧类型
`sensor_msgs/PointCloud`。

**4) 顺带确认的其他事实**
- `pub_time_gap` 硬编码 0.1s ✓（`legacy.cpp:254-255`），`publish_hz_` 真的无效（M2 成立）
- pose 定时器 0.01s ✓、`rt_grid_map` 与 `slam_warning_code` 定时器都是
  `1/rt_grid_map_publish_hz_` ✓（`legacy.cpp:256-267`）
- `publishCurrPose()` 里定位分支有一个**同帧去重**：`curr_pose.time_stamp - last < 1e-4` 就 return
  （建图分支的同类判断被注释掉了）。这会连带影响 `curr_localization_pose` 与 `slam/info` 的发布频率
  （`publishLocationInfo()` 在 `publishCurrPose()` 末尾、去重 return 之后才会走到）。
- `publishLocationInfo()` 的最终覆盖（`relocating_` 为真 → `slam_state=1, location_status=1`）在
  switch **之后**，所以它同时改写了 FREE/MAPPING 分支的结果 ✓（§5.3 表已记）
- `matching_degree` 在定位且 `status∈{0,1}` 时其实是 `localiza.getFitness()`，
  不是"恒 0" —— 这是 D3 已知偏差。**新发现**：核心其实已经有
  `LocalizationPipeline::confidence() const { return m_last_match.fitness; }`
  （`localization_pipeline.h:41`），只是没通过 `ISlamAlgorithm` 暴露；将来要么给
  `ISlamAlgorithm` 加个 getter（改核心），要么在薄层 `dynamic_cast` 到
  `LocalizationPipeline`（耦合具体实现类）。本次仍按 D3 恒 0，先把发现记下。

### 5.5 T5a 实测记录（转换层已完成并提交；发布侧待做）

**状态**：`catkin_make` `exit=0`；`run_tests` `exit=0`；**gtest 50/50 全绿**
（legacy_config 19 + realtime_grid 11 + lost_monitor 8 + legacy_convert 12）。

`src/legacy_convert.{h,cpp}` + `test/test_legacy_convert.cpp` 已落地，计划 §5.1 的 6 条用例
全部实现（计划那条 6 拆成了"恒等→0"与"+90°→符号"两条），另加 3 条补充
（`WarningJsonEscapesQuotes`、`ScanShapeZMaxNonPositiveMeansNoFilter`、
`ScanShapeDownsamplesToAbout2000`、`OldPointCloudHasPointsAndHeader`）。

实测踩到并修正的 3 件事 —— **其中第 2 条是计划写错了**：

1. **栅格转换不能设 `header.stamp = ros::Time::now()`**。
   - 旧实现填栅格时**从没设过 stamp**（`legacy.cpp:1717-1731` 的 `updateRealtimeGridMap`、
     `373-455` 的 `LocalizationSetGridMap` 都只设 `frame_id` 与 `info`），所以旧 `/map` 与
     `global_grid_map` 的 stamp **一直是 0**。照抄就是**不设**，要 `now()` 的调用方自己设
     （rosiwit 新接口的 `mapCB` 本来就是自己设的）。
   - 顺带一个测试环境的坑：**没 `ros::init()` 的 gtest 里 `ros::Time::now()` 会抛异常**
     （"Cannot use ros::Time::now() before the first NodeHandle has been created..."），
     所以上面那个错误写法会让 3 条用例直接抛异常挂掉。转换层保持"无 ROS 全局状态"也更好测。

2. **`legacyYawFromRot` 的符号：计划 §5.1 用例 6 的前提写反了。**
   对 `R = Rot(Z,θ)` 有 `R(0,1) = -sinθ`、`R(0,0) = cosθ`，所以
   `atan2(R(0,1), R(0,0)) = -θ` —— 即 `legacyYawFromRot(Rot(Z,+π/2)) = **-π/2**`，
   而**不是**计划里写的 `+π/2`（计划说"R(0,1)=1"，实际是 -1）。
   旧实现紧接着 `AngleAxis(-yaw, Z)`（`legacy.cpp:1922`）再取负，两者合起来正好还原成 `Rot(Z,θ)`。
   也就是说这对 (取 yaw, 用 yaw) 必须**成对照抄**，单独看一半会以为符号写错了。
   已改测试为：`R(0,1) == -1`、`legacyYawFromRot == -π/2`、且
   `AngleAxis(-yaw, Z).toRotationMatrix().isApprox(R)`（把"还原"这一步也钉住）。

3. 老类型转换需要 `#include <sensor_msgs/point_cloud_conversion.h>`
   （`convertPointCloud2ToPointCloud` 在这个头里，不在 `sensor_msgs/PointCloud.h`）。

计划 §5.1 里 `buildSlamInfo` 的 `update_request=0` / `matching_degree=0`(D3) 已按设计实现。
`buildWarningJson` 已按真实的 nlohmann 输出形状实现（键序字母序、`module="xz_3d_slam"`、
`type="algorithm fault"`、`code` 是字符串），并用 `yaml-cpp` 反解断言 —— 没有为此引入 json 库。

**下一步（T5b）**：`src/legacy_xz3dslam_interface.{h,cpp}` —— 14 个话题的注册与发布，
按上面 §5.4 修正后的真实条件实现（尤其 `debug_publish_` 门控、`scan_f_map` 的
"降采样+z 过滤"形态、`or_curr_scan_point` 只在定位发、`/map` 每个 `rt_grid_map` 周期无条件发）。
再按 §5.2 的表注册定时器（0.1s / 0.01s / `1/rt_grid_map_publish_hz`）。

### 5.6 T5b 实测记录（发布侧已完成并提交）

**状态**：`catkin_make` `exit=0`；`run_tests` `exit=0`；**gtest 51/51 全绿**
（legacy_config 19 + realtime_grid 11 + lost_monitor 8 + legacy_convert 13）。
`legacy_dev.launch` 真跑，`rosnode info` / `rostopic type` / `rostopic echo` / `rosservice call` 逐项核对：

- **14 个话题全部注册且类型正确** —— 关键项 `scan_f_map` 是 `sensor_msgs/PointCloud` ✓（其余 13 个与 §5.2 表一致）。
- **FREE 模式**（`slam_init_type=free`）`/slam/info` → `slam_state=1, location_status=1`，`map_name="NULL"`，
  `update_request=0`，`matching_degree=0` ✓ —— 与 §5.3 表逐字段对上。
- `rosservice call /set_slam_mode "mode: 'mapping'"` 回 `success: True`；之后 `/slam/info` →
  `slam_state=0, location_status=0` ✓（MAPPING 分支）。
- `/map` 发布者 = `/xz_3d_slam`，在 MAPPING 下按 1Hz 出数据（无雷达数据时 `width: 0`，符合预期 ——
  常驻对象照发上一张，`legacy.cpp:909`）。**T13 灌合成数据后再验 `width>0`（T10 的验收项）。**
- 启动日志：`[legacy_iface] slam_init_type=free -> mode=FREE debug_publish=0 is_debug=0 use_tf=1`
  与 `timers: pub=0.1s, pose=0.01s, rt_grid=1s (rt_grid_map_publish_hz=1, publish_hz=10 无效-硬编码 0.1)` ✓

实现时定下的 4 件事：

1. **`has_new_pose` 就是对的 latch 信号（已在核心源码上确认）**：定位模式下核心
   **只在 `m_localization->getStatus() == ILocalization::LOCALIZED` 时才产出输出**
   （`slam_pipeline.cpp:178-192`），INIT/DEGRADED 不产出。所以 T11 那条
   "`has_new_pose && 定位模式` → `onLocalizedPose()`" 正好等价于"真的定位上过"，
   INIT 陷阱（`getPoseCov` 在 INIT 返回 true）被绕开了。顺带确认：
   同一处 `out.world_cloud` 就是**已转到世界系**的当前帧，正对旧 `getCurrScan()`。
2. **`slam_mode_` 以核心为唯一权威**：每个 tick 用 `engine.pipelineMode()`
   反推旧 `SLAMMODE`（`mapping`/`localization`/`idle` → MAPPING/LOCALIZATION/FREE）。
   因为旧实现是"两个指针（localiza/mapping）有没有"决定行为，而我们**只有一个核心**，
   只能靠模式区分。启动时把 `slam_init_type` 打到核心上（`free` → `setPipelineMode("idle")`，
   核心确实支持 idle，见 `slam_pipeline.cpp:436-441`）。**这是真机上要留意的一条**：
   `slam_init_type=free` 意味着机器人**开机后核心是 idle、什么都不跑**，
   要等外部调 `/switch_map`（T8）或 `/set_slam_mode` 才动 —— 旧系统就是这个行为
   （`legacy.cpp:57-83` 只赋模式、不建指针）。已在启动日志里显式打出来。
3. **`debug_publish_` 在真机上是 false**，所以 §5.4 里那 5 个"只在 debug_publish_ 下发"的话题
   （`global_map`、`global_grid_map`、`curr_submap_point`、`loop_constraint`、`points_2d`）
   **在真机上本来就不出数据** —— 随包 conf 只有 `"is_debug": false` 且没有 `debug_publish` 键，
   按旧实现回落 `is_debug`（`legacy.cpp:1470-1474`）就是 false。实测启动日志 `debug_publish=0`。
   **这不是 bug，是照抄。** 定位模式下的 `curr_scan_point` 同理（只有 `scan_f_map` 一直发）。
4. **`slam_info.robot_pose.theta` 用标准航向角**，新增 `stdYawFromRot(R) = atan2(R(1,0), R(0,0))`
   （= `-legacyYawFromRot(R)`），并加了单测钉住二者反号。依据：旧实现 `curr_pose.yaw` 的旁证
   （`legacy.cpp:1172-1183` 用 yaw 做 `0.06cos/sin` 的航向偏移 + `sin(yaw/2)/cos(yaw/2)` 造四元数）。
   **待真机核对**：拿 `/slam/info.robot_pose.theta` 与 `/curr_localization_pose` 的 orientation 对一下。

**一处有意的小偏差**：建图模式下旧实现每 0.1s 都发 `curr_localization_pose`（位姿来自
`getCurrMappingPose()`，没数据时是零位姿）；我们在**核心还没产出任何位姿时不发位姿**
（但 `slam/info` 照发 100Hz）。更安全（不会先给下游一串零位姿），记在这里。

**验证脚本自身的小笔误**：查 `/map` 类型时脚本拼成了 `"//map"`（变量里已带 `/`），
所以那一行显示"<缺失>"；`rosnode info` 的发布表里 `/map [nav_msgs/OccupancyGrid]` 是在的 ✓。

### 5.7 T6 实测记录（订阅侧已完成）+ 你 2026-10-16 的答复

**状态**：`catkin_make` `exit=0`；`run_tests` `exit=0`；**gtest 51/51 全绿**。
`legacy_dev.launch` 真跑，`rosnode info /xz_3d_slam` 的订阅集合实测为：

```
/livox/imu   /livox/lidar   /initialpose   /scan   /odom   /lift/state   /lift_type
```
14 个发布话题也仍然齐全 ✓。

**⚠ 修掉了 T4 的一个真机级 bug（这个不修上机完全收不到数据）**

旧 `startSub`（`legacy.cpp:200-207`）选**输入话题靠的是 `laser_type`，不是 `laser_sub_name`**：

```cpp
if (laser_type_ == "mid360" || laser_type_ == "new_mid360") {
    subscribe("livox/imu", 100, imuCallBack);
    subscribe("livox/lidar", 0, livoxLaserCallBack, tcpNoDelay);   // 旧类型是 CustomMsg
} else {
    subscribe(imu_sub_name_, 100, imuWjCallBack);
    subscribe(laser_sub_name_, 0, laserCallBack, tcpNoDelay);
}
```

而我们实测的机型就是 `laser_type='new_mid360'`（XZ-SWGT + Livox_Mid360）。也就是说
**发行 conf 顶层那个 `laser_sub_name: "vanjee_points722"` 对 mid360 机型根本不被使用** ——
T4 当初按 `laser_sub_name` 订阅是错的。已改成按 `laser_type` 分支，实测日志：

```
[slam_engine] legacy laser_type='new_mid360' -> input topics: imu=livox/imu lidar=livox/lidar
```

（D4 仍然成立：我们这边 `livox/lidar` 是 `sensor_msgs/PointCloud2`，不是 livox 的 `CustomMsg`；
queue 0 + `tcpNoDelay` 照旧。）

**订阅的归属划分**（旧的一共 **7** 个，不是计划里写的 10 —— 那是把 mid360 分支与 else 分支的话题重复计数了）：

| 订阅 | 谁建 | 说明 |
|---|---|---|
| `livox/imu` 或 `<imu_sub_name>` | **SlamEngine** | 按 `laser_type` 分支（T4 已修） |
| `livox/lidar` 或 `<laser_sub_name>` | **SlamEngine** | queue 0 + tcpNoDelay ✓ |
| `initialpose` | **SlamEngine** | 旧实现也是无条件订阅，行为一致，不重复建 |
| `scan` | 本接口 | `LaserScan` 10，**只做频率统计** |
| `odom` | 本接口 | `Odometry` 100 → `engine.onOdomMsg()`（旧实现无条件订阅当运动先验；核心 YAML 里 `enable_odom: false`，所以引擎自己没订阅它 —— 不冲突） |
| `lift/state` | 本接口 | `String` 10，**只存字段**（旧的喂 `localiza_ptr_->setCurrLiftState()`，核心无对应接口） |
| `lift_type` | 本接口 | `String` 10，**只存字段 + 变化时打日志** |

**新增的两个引擎钩子**（`setInputSeenHook` / `setRawLidarHook`）：前者复刻旧的
`get_imu_data_ && get_laser_data_` 门控（`legacy.cpp:351`），后者是 `origin_point` 与
定位模式下 `or_curr_scan_point` 的数据源。

**一处有意偏差**：`origin_point` 拿到的点云是**已经按 `lidar_filter_num`（默认 3）抽稀、
并做过 min/max_range 裁剪**的那一份（与交给核心的同一份）；旧实现的 `origin_point` 是
CustomMsg 转换后的更原始的点云。影响只在下游可视化粒度，记在这里。

**你 2026-10-16 的答复（已确认，记入设计文档 §9）**：

- **`slam_info.robot_pose.theta` 与 `/curr_localization_pose` 的方向差异** —— 你说明了
  **这是激光雷达安装角度造成的**，属预期。所以 `stdYawFromRot` /
  `atan2(R(1,0),R(0,0))` 的取值方向**不用再当待核对项**，该项关闭。
- **A（地图文件命名）**：确认就是 `map.*`。`map_dir` 带不带结尾 `/` **不确定** ——
  正好是我们已经做的容错写法：`legacyMapBase()` 去尾斜杠、空则退回 `/`（T1 已实现、有单测），
  两种情形都对。**该项关闭。**
- **B（脱敏配置）**：暂给不出，**后续直接读实机**。不影响继续开发（缺
  `sensor_config.json` 时 `LegacyConfig` 回落 `type_3d=Livox_Mid360`，只告警不抛异常）。
  **该项转为"上机时核对"。**
- **C（首次上机核对生效 `acc_scale`）**：仍开放。

**下一步**：T7（告警码 `slam_warning_code` + 3 个 service client）。

**验证**（发布侧已过；/map 有数据要等 T13）：
1. 构建 + 单测。
2. `legacy_dev.launch` 起来后：
   ```bash
   rostopic list -v | grep -E 'global_map|global_grid_map|curr_scan_point|scan_f_map|curr_submap_point|or_curr_scan_point|curr_localization_pose|mapping_path|loop_constraint|slam/info|points_2d|slam_warning_code|origin_point|/map'
   ```
   14 个话题都在，且 `scan_f_map` 类型是 `sensor_msgs/PointCloud`。
   **T12 会把这一步固化成脚本**，本任务先手工过一遍。
3. `/map`：切到建图模式、喂合成数据后 `rostopic echo -n1 /map` 的 `info.width>0`
   （真正建图数据的验证在 T10）。

**提交**：`feat(ros1): xz_3d_slam 旧接口发布器(14 话题)`

---

## T6. 旧接口订阅器（10 个）

照抄 `legacy.cpp:193-215`（`startSub`）。

- `laser_type ∈ {mid360, new_mid360}` 分支：
  - `livox/imu`（`sensor_msgs/Imu`，queue 100）→ `engine.onImuMsg`
  - `livox/lidar`：旧类型是 `livox_ros_driver2/CustomMsg`（queue 0 + `tcpNoDelay`）。
    **我们按 D1 用 `sensor_msgs/PointCloud2`**，同样 queue 0 + `ros::TransportHints().tcpNoDelay()` → `engine.onLidarMsg`
- else 分支：
  - `<imu_sub_name>`（`Imu`，100）
  - `<laser_sub_name>`（`PointCloud2`，0 + tcpNoDelay）
- 无条件订阅：`initialpose`（`PoseWithCovarianceStamped`，100）、`scan`（`LaserScan`，10，**只做频率统计**）、
  `odom`（`Odometry`，100 → `engine.onOdomMsg`）、`lift/state`（`String`，10，**存字段不生效**）、
  `lift_type`（`String`，10，**存字段不生效**，打日志）

要点：
- `scan` / `odom` 的订阅与 T9 的频率告警共用计数器，所以回调里先把计数写好。
- `odom` 与 `rosiwit` profile 的输出 `odom` 是**同名**的（设计文档 §5.3 冲突 2）：
  `legacy` profile 下 `SlamEngine` 的 `odom` 别名**默认关闭**（参数 `~publish_odom_alias`，
  `rosiwit` profile 下默认 true）——这条在 T4 的 engine 里实现，本任务只保证不会自环。
- `initialpose`：定位模式下 `setInitPose`，**z 强制 0**（照旧）。

**验证**：`legacy_dev.launch` 起来后 `rostopic info /livox/imu`、
`rostopic info /vanjee_points722`（取决于 model）显示本节点是订阅者；
`rosnode info /xz_3d_slam` 的 Subscription 列表包含全部 10 项。
**提交**：`feat(ros1): xz_3d_slam 旧接口订阅器(10 话题)`

---

## T7. 告警码 + 3 个 service client

### 7.1 告警（照抄 `legacy.cpp:457-...` 的 `timerPublishWarningCode` + `legacy.cpp:302-305` 的 `checkTime` 线程）

- `conf/warning_code.json` 已随包（T1）。**从包 share 读，不读 `/opt`**（M6）。
- 频率监视：`lidar` / `imu` / `odom` 三个回调计数，1s 窗口比对阈值 →
  `20003001`(lidar hz low) / `20003002`(imu hz low) / `20003003`(odom hz low)
- 无数据：`20003500`(no lidar data) / `20003501`(no imu data) / `20003502`(no odom data)
- `20003005`(set mode frequent) 由 T8 的 `/slam/mode_switch` 限流触发
- **保留**旧行为的"启动/模式切换后前 100 个 tick 不发告警"（`warning_code_start_task_`
  抑制窗口，`legacy.cpp:184`）
- 发布结构用 `buildWarningJson()`（T5），`module` 恒 `"xz_3d_slam"`

### 7.2 clients（照抄 `legacy.cpp:243-244` + `2140`）

```cpp
get_charging_point_ = nh.serviceClient<xzrobot_ros_dependency::TagDetect>("get_charging_point");
set_detect_mode_   = nh.serviceClient<xzrobot_ros_dependency::CommonIntRequestService>("set_detect_mode");
switch_map_server_ = nh.serviceClient<xzrobot_ros_dependency::SwitchMap>("/switch_map_pro");
```
超时/重试照抄：`set_detect_mode` 0.5s 超时 + 100ms 重试；`switch_map_pro` 1.0s；
`get_charging_point` 1.0s。不可用时 `ROS_WARN` 并按旧逻辑给出 `success=false` / 空 `charging_pile_json`
（D14：外部服务在旧 launch 里与 SLAM 同文件启动，按"还在"实现）。

srv 类型（来自 `ros1_ws/src/xzrobot_ros_dependency/srv/`，已核对）：
- `TagDetect.srv`: 请求空 → 响应 `string charging_pile_json`
- `CommonIntRequestService.srv`: 请求 `int8 mode` → 响应 `bool result`
- `SwitchMap.srv`: 请求 `string map_name, string map_dir, float64 map_resolution, float64[] map_origin,
  bool map_negate, float64 map_occupied_thresh, float64 map_free_thresh,
  geometry_msgs/PoseWithCovarianceStamped initial_pose, string vir_obs` → 响应 `bool success`

**验证**：
1. 构建 + 单测（`buildWarningJson` 在 T5 已测）。
2. `legacy_dev.launch` 后 `rostopic echo -n1 /slam_warning_code`：
   在没有任何雷达/IMU 的机器上，**启动约 1s 后**应出现 `20003500`/`20003501`（注意 100-tick 抑制窗口）。
3. `rosservice info /set_detect_mode` 不存在时本节点不崩（只 WARN）。
### 7.3 ⚠ T7 探测结果（**本轮未实现 T7**；下次开工前必读）

为了 T7 去读了旧源码，结果发现计划 T7 §7.1 的描述**多处不准**。先把事实记下来，避免下次写错：

1. **`checkTime()` 是个空转函数**（`legacy.cpp:2972-3001`）—— 里面真正的超时判断**全被注释掉了**，
   只剩 `ros::ok()` / `ros::master::check()` 两个空 `if` 和 500ms sleep。
   频率监视**不在**这个线程里，而在 **imu / lidar / odom 的回调里**。计划写的"`checkTime` 线程"不准。
2. **频率判定是"1s 窗口计数 + 阈值比较"**，窗口结束时若 `count <= 阈值` 就把码 push 进待报列表
   （push 前去重）。已确认的阈值：
   - IMU：`imu_fre_count_ <= 160` → `20003002`（`legacy.cpp:629-637` 与 `681-689`，两个 IMU 回调各一份）
   - lidar / odom 同型，阈值在 735-800 区段，**下次开工先把这段读了再写**
3. **"前 100 tick 抑制"**：`warning_code_start_task_` 的初值是 `false`
   （`xz_slam_ros1_interface.h:552`），但它在 **`run()` 的末尾**被置 `true`
   （`legacy.cpp:184`，该函数是 101-191），而**旧 node main 第 73 行启动时就调了
   `node->run()`**（`xz_slam_ros1_node.cpp:71-77` 的顺序是 run → startSub → initPub →
   initTimerPub → initServer）。`run()` 还在 `switchToMapping`（2046）与 `/switch_map`
   的换图分支（2345）里被调用。
   **所以抑制窗口在开机时就是开的，切换建图模式后会重新开一次** —— 计划 §7.1 原来的
   说法是**对的**。（我上一轮在 §7.3 曾据 `legacy.cpp:184` 的行号误判成"只在 start_task 之后"，
   现已更正；T7 当时因为还没接上这条而能立刻看到告警，T8 接上后实测开机 100 tick 内不再发，
   两者一致。）抑制窗口期间也解释了：`/slam_warning_code` 开机后约 100 秒内是静默的
   （rt_grid 周期默认 1s × 100 tick）。
4. **`20003500`（no lidar data）是从 lidar 回调内部发的**（`legacy.cpp:3078 / 3202 / 3333`，
   分别对应三个机型分支）—— 语义是"**收到了帧但不可用**（转换失败/点数太少）"，
   不是"没收到帧"。`20003501` 同型（3503）。
5. **接口文件里根本没出现的码**：`20003003`(odom hz low)、`20003004`、`20003006`、
   `20003502`、`20003503`。也就是说 conf 里定义了但旧接口层从不发这些。
6. **`20003504`(pose lost) 在接口文件里一次都没出现** —— 它是**定位模块自己**报的。
   这条对 **T11** 很关键：计划 T11 设计的是"`consumeLostEdge()` 为真就把 20003504 加进待报列表"，
   但旧系统的 20003504 根本不经过接口层。下次开工要先把定位模块那份 20003504 找出来，
   确认我们的 LostMonitor 补报是**补上**还是**重复**。
7. 三个 service client 的 srv 形状已核对（§7.2 的 `TagDetect` / `CommonIntRequestService` /
   `SwitchMap`），这部分计划是对的。

**结论**：T7 不能照 §7.1 直接写。先把第 2 条的 lidar/odom 阈值与第 6 条的 20003504 出处读全，
再动代码 —— 告警码是机器人监控系统消费的通道，发错了比不发更糟。

### 7.4 T7 实测记录（已完成并提交）

**状态**：`catkin_make` `exit=0`；`run_tests` `exit=0`；**gtest 52/52 全绿**
（legacy_config 19 + realtime_grid 11 + lost_monitor 8 + legacy_convert 14）。

**已落地**：
- 告警码表从**本包 share** 读（M6）：实测 `warning_code table loaded: 28 codes from
  .../rosiwit_slam_ros1/conf/warning_code.json`。
- 频率窗口按 §7.3 的事实实现：imu `<= 160` → `20003002`，lidar `<= 6` → `20003001`；
  **odom 不发码**（旧实现只打印）；两个流各自一个 1s 窗口。
- 抑制窗口 `warning_code_start_task_` 初值 `false`，只在 `setWarningSuppression(true)`
  （T8 的 `start_task` 会调）之后才生效 100 个 tick。
- 告警定时器与 `rt_grid_map` 同周期（`legacy.cpp:267`）；没有码就不发（`legacy.cpp:543`）。
- 未定义码的兜底照抄（`name="not define warning code"`、`code=begin_code_+998=20003999`）。
- `addWarningCode(int)` 公开 seam 给 T11 用（核心**不暴露**任何告警码通道，已 grep
  `include/` 下无 `WarningCode` 相关接口，所以 20003504 只能我们自己补报 —— 是"补上"不是重复）。
- 3 个 service client：`get_charging_point`(TagDetect, 1.0s)、`set_detect_mode`
  (CommonIntRequestService, 0.5s + 100ms 重试)、`/switch_map_pro`(SwitchMap, 1.0s)；
  服务不在时只 `ROS_WARN` 并按旧逻辑给 `false` / 空串（D14）。

**端到端验证**（真跑，不只看编译）：以 **2Hz** 喂 `/livox/imu` 5 秒 → 1s 窗口计数 4/3/2
都 `<= 160` → `/slam_warning_code` 收到：

```json
{"faults":[{"code":"20003002","content":"slam subscripe imu frequency is low!",
            "level":"WARNING","module":"xz_3d_slam","name":"imu hz low",
            "type":"algorithm fault"}]}
```

字段集合与顺序与旧实现（nlohmann 字母序 + 补 type/module/code）**逐字一致** ✓。

**有意不复刻的部分**：`20003500` / `20003501`。它们的触发条件是旧 lidar 处理循环里的
"迭代 600 次都没拿到新帧"计数器（`legacy.cpp:3076`），语义是"收到帧但不可用"，
我们的核心没有对应层次。**因此在本机没有雷达/IMU 数据时 `/slam_warning_code` 是静默的**
（频率窗口只在"消息有来但偏慢"时才结束）—— 这与计划 §7 原来的验证预期不同，
但符合我们实际实现的事实。要在真机上看到这两条码，得先有输入数据。

**提交**：`feat(ros1): 告警码模块 + 旧接口 service client`



---

## T8. 旧接口 service server（8 个）

照抄 `legacy.cpp:235-242` 的 advertise 与各自回调。

| 服务 | 类型 | 实现 |
|---|---|---|
| `/slam/mode_switch` | `SlamModeSwitchService` | `mode==0→localization`、`mode==1→mapping`（**与 `SlamInfo` 常量相反**）；非法值 `result` 留 0 并返回 false；**10s 限流**（`legacy.cpp:2121-2137`，不足 10s 直接返回并报 `20003005`）；照旧调 `/set_detect_mode` |
| `/switch_map` | `SwitchMap` | 三分支照抄（见下） |
| `/relocation` | `Relocation` | `setInitPose`：**z 强制 0**；`success` 恒 true、`return true`（`legacy.cpp:2394-2418`）。注意**没有** `>1e4` 归零逻辑（那只在 `/switch_map`） |
| `/stop_map_operation` | `StopMapOperation` | ① `mkdir(req.map_dir, 0755)`（已存在则跳过；**创建失败即 `return false` 且 `success` 留 false**，`legacy.cpp:2424-2454`）② `engine.algo()->saveMap(legacyMapBase(req.map_dir))`（旧实现是 `saveMap(folderPath, "map")`，等价）③ `get_charging_point_`：失败只打日志，`response.charging_pile_json` 取默认构造的空串（`legacy.cpp:2471-2489,2505`）④ `current_pose` 用 `frame_id="map"` + 当前位姿的 xyz + 四元数（`z` 用**真实 z**，不是 -0.07）⑤ `success=true`（仅当 `mapping_ptr_` 非空，否则 `return false`）⑥ `map_resolution`/`map_origin`/`map_negate`/`*_thresh`/`error` **全部留默认空值**（旧代码里那几行是注释掉的，`legacy.cpp:2506-2515`）—— 照旧留空 |
| `/global_localization` | `std_srvs/Empty` | **空实现 `return true`**（`legacy.cpp:2634` 就这一行） |
| `/finish_task` | `CommonBoolService` | 有定位实例时 `updateMap()` → `status=true`；**无定位实例则 `return false` 且 `status` 留 false**（`legacy.cpp:2601-2609`）。我们的降级：定位模式且有核心 → `true` |
| `/start_task` | `CommonBoolService` | 同上（旧实现是 `setUpdateMapFlag()`，`legacy.cpp:2591-2599`） |
| `update_map_operation` | `UpdateMap` | **有意降级（D5）**。旧实现（`legacy.cpp:2538-2589`）是"重建建图实例 + `setIncreaseMapFlag()` + `loadCurrentMap(req.map_dir + \"/map.xzmap\")` + `setInitPose(pos, rot*AngleAxis(angle,Z))`"—— 核心没有"增量扩图后写回"的能力。我们的替代：`engine.algo()->loadMap(legacyMapBase(req.map_dir))` + `setPipelineMode("localization")`，`status` 按加载结果；**入口先打 WARN 说明是降级实现** |

`/switch_map` 三个分支（`legacy.cpp:2140-2340`）：
1. `req.map_dir` 或 `req.map_name` 为空 → `success=false` 直接返回
2. `req.map_dir == curr_map_path_` 且已在定位 → 只重设初始位姿 + 转发 `/switch_map_pro`，
   `success` 取外部结果
3. 换图 → `relocating_=true`；调 `/set_detect_mode`（mode=0，0.5s 超时 + 100ms 重试），
   **该结果单独决定 `success`**；然后：
   ```cpp
   const std::string base = legacyMapBase(req.map_dir);   // "/dir/map"
   if (!engine.algo()->loadMap(base)) { success = false; ROS_ERROR(...); }   // D13: 旧实现不检查加载结果
   engine.algo()->setPipelineMode("localization");
   curr_map_path_ = req.map_dir;
   curr_map_name_ = req.map_name;
   init_pose = 若 |x|,|y| 任一 > 1e4 则全零, 否则用 req.initial_pose; z 强制 0
   engine.algo()->setInitPose(init_pose);
   lost_monitor_.reset();
   ```
   `map_directory` 的 `"/"` 注入（M8）在 T5/T10 的 engine override YAML 里做。

**验证**：
1. 构建。
2. `rosnode info /xz_3d_slam` 的 Services 列表含全部 8 个；类型与上表一致。
3. `rosservice call /global_localization` 返回 `{}`；
   `rosservice call /slam/mode_switch "{mode: 1}"` → `result: 1`；1s 内再调一次 → 触发 `20003005`。
4. `rostopic pub` 一个空 `map_dir` 的 `/switch_map` → `success: false`。
**提交**：`feat(ros1): xz_3d_slam 旧接口 service server(8 个)`

### 8.1 T8 实测记录（已完成并提交）

**状态**：`catkin_make` `exit=0`；**gtest 52/52 全绿**。`legacy_dev.launch` 真跑核对：

| 服务 | 类型 | 实测 |
|---|---|---|
| `/slam/mode_switch` | `SlamModeSwitchService` | `{mode: 1}` → **`result: 1`** ✓；`{mode: 5}` 被拒 ✓ |
| `/switch_map` | `SwitchMap` | 空 `map_dir` → `success: False` ✓；10s 内再调一次被拒 ✓ |
| `/relocation` | `Relocation` | 已注册 ✓ |
| `/stop_map_operation` | `StopMapOperation` | 已注册 ✓ |
| `/global_localization` | `std_srvs/Empty` | 返回空响应 ✓ |
| `/finish_task` / `/start_task` | `CommonBoolService` | 已注册 ✓ |
| `update_map_operation` | `UpdateMap` | 已注册 ✓（降级实现 D5） |

8 个服务与 4 个新接口服务（`/save_map` 等）**无重名** ✓。

**实现时核对出的 3 处与计划 §8 表格不符（已按源码实现）**：

1. **10s 限流 + `20003005` 在 `/switch_map` 里，不在 `/slam/mode_switch`**。
   计划 §8 表格把它写在 `/slam/mode_switch` 那行。源码 `legacy.cpp:2121-2137` 位于
   `switchMap()`（函数头在 2118）。已按源码放。
2. **`switchToMapping` 忽略 `/set_detect_mode` 的结果**：`legacy.cpp:2026` 那句
   `// if (!success) return false;` 是**注释掉的**，所以 `mode==1` 一定 `result=1, true`
   （实测确认）；而 `switchToLocalization` 的对应判断（2111）是活的，`mode==0`
   **要求 `/set_detect_mode` 成功**才给 `result=1`。
3. **`switchToLocalization` 把 `slam_mode_` 置成 `FREE` 而不是 `LOCALIZATION`**（`legacy.cpp:2062`）——
   它先删掉定位实例，真正进定位要等随后的 `/switch_map`。已照抄。

另外 `/stop_map_operation` 的 `saveMap`：旧实现的 `mapping_ptr_->saveMap()` 返回 `void`、
**不检查结果**，所以我们也只在失败时 `ROS_ERROR`、不改 `success`（只记 D13 那条是我们
额外把 `/switch_map` 的 `loadMap` 失败也算进 `success`）。

**抑制窗口**：按 §7.3 更正后的结论，开机就开抑制窗口；实测开机后 2Hz 喂 IMU 6 秒
**没有**告警输出（与 T7 当时的实测相反，正是因为 T8 才把它接上）—— 两条实测互相印证。

**`update_map_operation` 是有意降级（D5）**：入口先打 WARN 说明不等价。



---

## T9. TF `map→odom` + `legacy` profile 的 TF/odom 抑制

照抄 `legacy.cpp:1899-1936`（`sendTransform`）：

```cpp
geometry_msgs::TransformStamped t;
try { t = tf_buffer_.lookupTransform("base_footprint", "odom", ros::Time(0)); }
catch (tf2::TransformException& ex) { ROS_ERROR("%s", ex.what()); return; }   // 失败直接不发
if (isnan(curr_pose_.x) || isnan(curr_pose_.y) || isnan(curr_pose_.yaw)) return;

tf2::Transform Tmf, Tfo, Tmo;
Tmf.setOrigin(tf2::Vector3(curr_pose_.x, curr_pose_.y, -0.07));               // 注意 z=-0.07
double yaw = atan2(R(0,1), R(0,0));                                           // 用 legacyYawFromRot()
Tmf.setRotation(tf2::Quaternion(Eigen::Quaterniond(Eigen::AngleAxisd(-yaw, UnitZ()))));
tf2::fromMsg(t.transform, Tfo);
Tmo = Tmf * Tfo;                                                              // T_mo = T_mf * T_fo
out.header.stamp = ros::Time::now();                                          // 不是位姿时间戳
out.header.frame_id = "map"; out.child_frame_id = "odom";
broadcaster_.sendTransform(out);
```
- `legacy` profile：**停发**静态 `map→odom`（否则与动态边互撞）、**停发**动态 `world_frame→body_frame`；
  只保留上面这条，100Hz（挂在 pose 定时器上），条件 `slam_mode_ != FREE && use_tf_`。
- `rosiwit` profile：保持现状（静态 `map→odom` + 动态 `world_frame→body_frame`）。
- `publishStaticTf()`（`base_link→scan`）**不实现** —— 旧实现的它从未被调用。

**验证**：
1. `legacy_dev.launch` + 手工 `rosrun tf static_transform_publisher` 伪造一条
   `base_footprint→odom`，再喂合成位姿 → `rosrun tf tf_echo map odom` 能查到，
   且 `rosrun tf view_frames` 里 `map→odom` 是 **dynamic**。
2. `rostopic echo /tf_static` 在 `legacy` profile 下**没有** `map→odom`。
**提交**：`feat(ros1): 旧 map->odom 动态 TF, legacy 下抑制新 TF`

### 9.1 T9 实测记录（已完成并提交）

**状态**：`catkin_make` `exit=0`；**gtest 52/52 全绿**。

按 `legacy.cpp:1899-1936` 实现动态 `map→odom`：

```
Tfo = lookupTransform("base_footprint", "odom", Time(0))     // T(base_footprint <- odom)
Tmf = (origin=(x, y, -0.07), rot = AngleAxis(-yaw, Z)),  yaw = atan2(R(0,1), R(0,0))
Tmo = Tmf * Tfo                                              // T(map <- odom)
broadcast(map -> odom), stamp = now
```
调用点：`publishCurrPose()` 末尾，`slam_mode_ != FREE && use_tf_` 才发（`legacy.cpp:1208-1210`）。
lookup 失败就 `ROS_ERROR` + `return`（旧实现同样如此），节点不会崩。
`publishStaticTf()`（`base_link→scan`）**不实现** —— 旧实现的它从未被调用 ✓。

**端到端验证**（不是只看编译）：起一个 `odom→base_footprint` 静态 TF 模拟 `robot_localization`，
再切到建图模式 → `/tf` 里真的出现：

```
frame_id: "map"
child_frame_id: "odom"
translation: {x: 0.0, y: 0.0, z: **-0.07**}
rotation:    {x: 0, y: 0, z: 0, w: 1}
```
`z = -0.07` 正是旧实现那个特征值 ✓（位姿全零时旋转为恒等，符合 `Tmf*Tfo`）。
`/tf` 的发布者只有 `/xz_3d_slam` ✓ —— M5 在 T4 就已经让新接口在 legacy profile 下不发 TF，
所以这条边只有一个发布者，不会打架。`/tf_static` 在 legacy 下没有 `map→odom` ✓。

**两个实现要点（写进代码注释）**：

1. **必须用 `legacyYawFromRot(q)` 再取负**，不能直接用 `curr_pose_.yaw`。
   前者 = `atan2(R(0,1),R(0,0))` = **`-标准航向`**，旧实现紧接着 `AngleAxis(-yaw, Z)` 再取负，
   两者成对照抄才等于 `Rot(Z, θ_std)`。直接用 `curr_pose_.yaw`（标准航向）会**把符号弄反**。
2. **刚体复合用 Eigen 直接算**（`R=R_f·R_o`, `t=R_f·t_o+t_f`），而不是 `tf2::Transform` ——
   语义与 `tf2` 的 `A*B` 完全一致，但省掉了为一个乘法引入 `tf2_geometry_msgs` 依赖。



---

## T10. 接上 `/map` 的 MAPPING 分支 + `map_directory` 注入

**目标**：把 T2 的 `RealtimeGrid` 接进 T5 的 `/map` MAPPING 分支；`legacy` profile 下给
`core.init()` 一份 override YAML，其中 `map_directory: "/"`（M8）。

### 10.1 override YAML（`slam_engine.cpp`，`legacy` profile 才做）
- 基底 = `~config_file`（缺省本包 `config/default.yaml`）
- 只改一个键：`map_directory: "/"`，写到
  `~/.ros/rosiwit_slam_ros1/compat_legacy.yaml`（进程内唯一路径，启动时覆盖写）
- **不做** `acc_scale`/`r_il`/`t_il` 推导（M13 已改：按"现在"的来）
- **必须打印**核心最终生效的值以便诊断"完全没位姿"这类症状：
  ```
  [compat] core yaml = <path>
  [compat] effective acc_scale=<..> r_il=[..] t_il=[..]
  [compat] ignored legacy keys: roll_new=.. pitch_new=.. yaw_new=.. imu_in_car_frame_yaw_livox=..
  ```
  （这就是 D7 要求的"已读未生效"清单 + §5.10 B 的可诊断性）
- `~input_profile=rosiwit` 时**不注入任何 override**，行为与现状一致

### 10.2 接上实时栅格
- `RealtimeGrid::setSource([this](PointVec& p){ return engine_.getGlobalMap(p); })`
- 只在 `legacy` profile 且模式为 MAPPING 时 `start(1.0/rt_grid_map_publish_hz)`；切走时 `stop()`
- `/map` MAPPING 分支：`RealtimeGrid::latest(gi)` → `toLegacyGrid(gi, "map")` →
  `origin.position.z = map_area * 1e-5`，`header.stamp = now()`，非 latch 发布
- `map_area` 同时供 `/slam/info`

**验证**：
1. 构建 + 单测。
2. 用 T13 的合成数据发布器（或先手工 `rostopic pub` 一段 PointCloud2 + Imu）：
   切到建图模式 → `rostopic hz /map` 约 1Hz、`rostopic echo -n1 /map/info` 的 `width/height > 0`。
3. `legacy_dev.launch` 启动日志里能看到 `[compat] core yaml = ...` 与 `effective acc_scale=...` 那一组行。
4. `cat ~/.ros/rosiwit_slam_ros1/compat_legacy.yaml | grep map_directory` → `"/"`
**提交**：`feat(ros1): 建图模式 /map 实时栅格 + map_directory 注入`

### 10.3 T10 实测记录（已完成并提交）

**状态**：`catkin_make` `exit=0`；**gtest 52/52 全绿**。

§10.2 的实时栅格接线**在 T5b 就已经做完了**（`RealtimeGrid::setSource` 取核心全局地图、
`start(rt_grid_map_publish_hz)`、`/map` MAPPING 分支 `toLegacyGrid` + `origin.z = area*1e-5`
无条件发布）。所以 T10 这轮只补 §10.1 的 override YAML 与诊断。

**已完成**：
- legacy profile 下生成 `~/.ros/rosiwit_slam_ros1/compat_legacy.yaml` = **基底原文 + 末尾追加
  `map_directory: "/"`**。做法是**按文本复制再追加**，不是 `YAML::Dump` 重排整个 Node ——
  原文件的注释与排版都保留（实测注入文件里原有 16 行 `#` 注释仍在），也不会被重排改掉语义。
- `~config_file` 为空时（legacy profile）回落到本包 `config/default.yaml` 并告警，
  避免把核心指向空路径而退化成内置默认。
- 打印 D7 / §5.10 B / **待办项 C** 需要的三行 + 一条告警：
  ```
  [compat] core yaml = /home/jmq/.ros/rosiwit_slam_ros1/compat_legacy.yaml
  [compat] effective acc_scale=1 r_il=[1.0, 0.0, ...] t_il=[0.0, 0.0, 0.0905]
  [compat] ignored legacy keys: roll_new=0 pitch_new=45 yaw_new=90 imu_in_car_frame_yaw_livox=65
           (M13: 外参/单位按核心 YAML, 不采纳旧实现的 9.81 与安装角)
  [compat] WARN acc_scale=1.0 —— 若雷达内置 IMU 的加速度是 g 单位, 这里应为 9.81
           (待办项 C: 首次上机必须核对本行)
  ```
  **待办项 C 至此变成"看一行日志"**：核心 `acc_scale` 默认 1.0
  （`fast_lio2/map_builder/commons.h:47`，注释明确说"Livox 内置 IMU 若以 g 为单位要设 9.81"），
  而我们的 `config/default.yaml` 里**没有**这个键，所以上机第一件事就是看这条 WARN。
- `input_profile=rosiwit` 下**不注入、不打这些行**（实测 `[compat]` 行数 0、注入文件不存在）✓。

**一处与计划 §10.2 的有意差别**：计划写 `/map` 要 `header.stamp = now()`，**我们保持 stamp=0**。
理由：旧实现 `updateRealtimeGridMap()`（`legacy.cpp:1717-1731`）从头到尾**没设过 stamp**，
旧 `/map` 的 stamp 一直是 0 —— "对上"优先。若将来 costmap 之类的消费者因为 stamp=0 报错，
再改成 `now()`（改一行），但那就是有意的偏差了。

**验证**：真跑两个 launch，逐项核对上面这些行与文件；注入文件的 `map_directory: "/"` 在第 329 行。



---

## T11. 接上定位丢失上报

- `LegacyXz3dSlamInterface` 持有 `LostMonitor`；在一处高频回调（pose 定时器，100Hz）调
  `engine.getPoseCov(pose, cov)` → `lost_monitor_.update(ok)`
- **分流**（照 `legacy.cpp:1839-1894`）：只有 **LOCALIZATION** 用 `LostMonitor` 的结果；
  **MAPPING** 恒 `0`；**FREE** 恒 `1`；最后若"重定位进行中"则强制 `1`
- `consumeLostEdge()` 为真时把 `20003504` 加入待报列表（`level=FAULT`，来自 `warning_code.json`）
- latch 控制点：
  - `engine` 输出回调里，`SlamOutput.has_new_pose && 定位模式` → `lost_monitor_.onLocalizedPose()`
  - 地图加载成功（`/switch_map`、`update_map_operation`）→ `lost_monitor_.setMapLoaded(true)`
  - 进入定位/换图/重定位 → `lost_monitor_.reset()`

**验证**：
1. 构建 + 单测（T3 已覆盖状态机；这里验证接线）。
2. `legacy_dev.launch` + 加载一张真地图（T13 可用合成的 `map.pcd`+`map.xzmap` 或跳过）：
   - 定位未建立时 `rostopic echo -n1 /slam/info` 的 `location_status == 1`
   - 喂与地图不匹配的雷达数据直到丢失 → `location_status == 2`，且 `slam_warning_code` 里出现 `20003504`
   （这条最难构造，至少验证 latch 逻辑：INIT 阶段不得报 0）
**提交**：`feat(ros1): 定位丢失上报(location_status=2 + 20003504)`

---

## T12. 接口一致性校验脚本（V2）

**目标**：把"对上"变成机器可判的检查。

`scripts/check_legacy_interface.sh`：
- 前置：`roslaunch rosiwit_slam_ros1 legacy_dev.launch`（或已运行的 `/xz_3d_slam`）
- 采集 `rostopic list -v`、`rosservice list` + 每个服务的 `rosservice info` 类型、
  `rosnode info /xz_3d_slam`、`rosparam get /machine/model`
- 与 `scripts/legacy_interface.golden` 逐项比对；缺项/类型不符 → 非 0 退出并打印差异

`scripts/legacy_interface.golden`（人可读，两段）：
```
[topics]
global_map                        sensor_msgs/PointCloud2
global_grid_map                   nav_msgs/OccupancyGrid
/map                              nav_msgs/OccupancyGrid
curr_scan_point                   sensor_msgs/PointCloud2
scan_f_map                        sensor_msgs/PointCloud
curr_submap_point                 sensor_msgs/PointCloud2
or_curr_scan_point                sensor_msgs/PointCloud2
curr_localization_pose            geometry_msgs/PoseStamped
mapping_path                      nav_msgs/Path
loop_constraint                   visualization_msgs/MarkerArray
slam/info                         xzrobot_ros_dependency/SlamInfo
points_2d                         sensor_msgs/PointCloud2
slam_warning_code                 std_msgs/String
origin_point                      sensor_msgs/PointCloud2
[services]
/slam/mode_switch                 xzrobot_ros_dependency/SlamModeSwitchService
/switch_map                       xzrobot_ros_dependency/SwitchMap
/relocation                       xzrobot_ros_dependency/Relocation
/stop_map_operation               xzrobot_ros_dependency/StopMapOperation
/global_localization              std_srvs/Empty
/finish_task                      xzrobot_ros_dependency/CommonBoolService
/start_task                       xzrobot_ros_dependency/CommonBoolService
update_map_operation              xzrobot_ros_dependency/UpdateMap
[params]
/machine/model                    string
```

**验证**：在有节点的环境下跑脚本 → 0 退出。
**提交**：`test(ros1): 旧接口一致性校验脚本 + golden`

---

## T13. 合成数据冒烟（V3）

**目标**：不需要机器人、不需要真实雷达，就能验证行为面。产出
`test/synthetic_publisher.py`（假 IMU 100Hz + 假 PointCloud2 10Hz，可调话题名，
可注入"与地图不匹配"的偏置以触发丢失）。

按设计文档 §7 V3 的六项逐项跑：
1. **建图**：切 mapping、喂 5s → `/map` 有 `width>0` 且 `data` 含 100
2. 定位模式 → `curr_localization_pose` 有输出且 `frame_id == "map"`
3. `/slam/info.slam_state` 随 `/slam/mode_switch` 正确切换（mapping↔localization）
4. `map→odom` TF 能查通（脚本同时伪造 `base_footprint→odom`）
5. 停掉雷达/IMU → 约 1s 后 `slam_warning_code` 报 `20003500`/`20003501`
6. 定位模式下喂不匹配雷达 → `location_status==2` 且报 `20003504`

通过标准：六项符合预期，ROS 日志无 crash/异常。
**提交**：`test(ros1): 合成数据冒烟脚本`

---

## 收尾

- 更新设计文档 §5.1 的文件布局（`legacy_config`/`legacy_convert`/`lost_monitor`/`scripts`/`test`/`launch/legacy_dev.launch`），
  并补记本轮新发现的偏差 **D17**（实时栅格无 unknown=10）与 **D18**（legacy 定位模式 `/map` 改为周期不 latch）。
- 已知未做（明确非目标，见设计文档 §3）：电梯/楼层联动、`points_2d`、回环 `MarkerArray`、
  地图自更新写回、2D 去畸变、车体框删点、标定、`matching_degree`/`fitness` getter。
- 交付形态两条路都给：就地改旧 launch 第 34 行，或把整机 bringup 的
  `$(find xz_3d_slam)/launch/...` 指向本包的 drop-in —— 由你在现场挑。
