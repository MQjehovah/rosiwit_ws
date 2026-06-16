# rosiwit_slam 文档中心

欢迎查阅 rosiwit_slam 项目文档。

> **注意**: 本项目原名为 `fast_lio2_slam`，已重命名为 `rosiwit_slam`（包名），可执行文件名为 `fast_lio2_node`。部分文档中可能仍有旧名引用，请以代码为准。

---

## 📖 文档目录

### 核心文档

| 文档 | 说明 | 适用对象 |
|------|------|----------|
| [API参考文档](API_REFERENCE.md) | 所有模块的API详细说明 | 开发者 |
| [关键函数说明](KEY_FUNCTIONS.md) | 核心算法函数详解 | 算法工程师 |
| [模块关系图](MODULE_RELATIONS.md) | 系统架构与模块关系 | 架构师、开发者 |
| [架构设计文档](architecture.md) | 完整架构设计与实施计划 | 架构师、项目经理 |
| [变更记录](changelog.md) | 版本变更历史 | 全员 |

### 测试文档

| 文档 | 说明 | 适用对象 |
|------|------|----------|
| [测试报告概览](TEST_REPORT_SUMMARY.md) | 测试执行摘要与结果分析 | 测试工程师、项目经理 |
| [缺陷报告](TEST_DEFECTS.md) | 发现的缺陷详情与修复建议 | 开发者、测试工程师 |
| [测试用例文档](TEST_CASES.md) | 测试用例列表与执行状态 | 测试工程师 |

### 快速开始

#### 1. API查询

如果您需要查找特定类或方法的详细信息：

```bash
# 查看 API 参考文档
cat docs/API_REFERENCE.md
```

**常用API**:
- [FastLio2Node](API_REFERENCE.md#fastlio2node---ros2节点主类) - ROS2节点主类
- [IekfEstimator](API_REFERENCE.md#iekfestimator---iekf状态估计器) - IEKF状态估计器
- [MapManager](API_REFERENCE.md#mapmanager---地图管理) - 地图管理
- [ImuProcessor](API_REFERENCE.md#imuprocessor---imu处理) - IMU处理

#### 2. 算法原理

如果您需要了解核心算法实现：

```bash
# 查看关键函数说明
cat docs/KEY_FUNCTIONS.md
```

**核心算法**:
- [IEKF预测与更新](KEY_FUNCTIONS.md#iekf状态估计关键函数) - 状态估计核心算法
- [点云滤波](KEY_FUNCTIONS.md#点云滤波关键函数) - 特征提取与滤波
- [闭环检测](KEY_FUNCTIONS.md#闭环检测关键函数) - Scan Context算法

#### 3. 系统架构

如果您需要理解整体架构：

```bash
# 查看模块关系图
cat docs/MODULE_RELATIONS.md
```

**架构图表**:
- [系统架构概览](MODULE_RELATIONS.md#系统架构概览) - 整体架构图
- [模块依赖关系](MODULE_RELATIONS.md#模块依赖关系) - 模块层次结构
- [数据流向图](MODULE_RELATIONS.md#数据流向图) - 数据处理流程

---

## 🔍 按需求查找

### 新手入门

1. 阅读 [README.md](../README.md) 了解项目概况
2. 查看 [API_REFERENCE.md](API_REFERENCE.md) 的"使用示例"章节
3. 参考 [KEY_FUNCTIONS.md](KEY_FUNCTIONS.md) 的"性能优化建议"

### 功能开发

1. 查看 [API_REFERENCE.md](API_REFERENCE.md) 了解可用接口
2. 参考 [MODULE_RELATIONS.md](MODULE_RELATIONS.md) 了解模块关系
3. 阅读 [architecture.md](../architecture.md) 了解设计思想

### 算法优化

1. 阅读 [KEY_FUNCTIONS.md](KEY_FUNCTIONS.md) 了解算法原理
2. 查看 [API_REFERENCE.md](API_REFERENCE.md) 的配置参数章节
3. 参考 [architecture.md](../architecture.md) 的性能优化章节

### 问题排查

1. 查看 [KEY_FUNCTIONS.md](KEY_FUNCTIONS.md) 的"常见问题解答"
2. 检查 [API_REFERENCE.md](API_REFERENCE.md) 的参数说明
3. 阅读 [MODULE_RELATIONS.md](MODULE_RELATIONS.md) 的数据流向

### 测试相关

1. 查看 [TEST_REPORT_SUMMARY.md](TEST_REPORT_SUMMARY.md) 了解测试结果
2. 阅读 [TEST_DEFECTS.md](TEST_DEFECTS.md) 了解已知缺陷
3. 参考 [TEST_CASES.md](TEST_CASES.md) 了解测试用例详情

---

## 📋 文档维护

### 文档更新历史

| 日期 | 版本 | 更新内容 |
|------|------|----------|
| 2026-04-25 | v1.1.0 | 添加测试文档：测试报告概览、缺陷报告、测试用例 |
| 2026-04-24 | v1.0.0 | 初始文档创建 |

### 文档结构

```
fast_lio2_slam/
├── README.md              # 项目主README
├── architecture.md        # 架构设计文档
├── DEPLOYMENT.md          # 部署指南
├── QUICKSTART.md          # 快速启动指南
├── DEPENDENCIES.md        # 依赖说明
└── docs/
    ├── README.md            # 本文件（文档索引）
    ├── API_REFERENCE.md     # API参考文档
    ├── KEY_FUNCTIONS.md     # 关键函数说明
    ├── MODULE_RELATIONS.md  # 模块关系图
    ├── ENVIRONMENT_SETUP.md # 环境搭建指南
    ├── TEST_REPORT_SUMMARY.md  # 测试报告概览
    ├── TEST_DEFECTS.md      # 缺陷报告
    └── TEST_CASES.md        # 测试用例文档
```

### 贡献指南

如果您发现文档有误或需要补充：

1. 在文档中使用 `TODO` 标记待完善内容
2. 提交Issue说明文档问题
3. 提交PR时更新相关文档

---

## 📞 技术支持

- **项目地址**: [GitHub Repository]
- **问题反馈**: 提交GitHub Issue
- **文档维护**: 文档专员（AI开发团队）

---

## 🔗 相关资源

- [FAST-LIO2论文](https://arxiv.org/abs/2107.06829)
- [FAST-LIO2代码库](https://github.com/hku-mars/FAST_LIO)
- [ROS2文档](https://docs.ros.org/en/humble/)
- [GTSAM文档](https://gtsam.org/)
- [PCL文档](https://pointclouds.org/)
- [Eigen文档](https://eigen.tuxfamily.org/)
- [Sophus文档](https://github.com/strasdat/Sophus)