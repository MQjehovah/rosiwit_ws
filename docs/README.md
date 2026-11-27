# docs 目录索引

> 最后更新 2026-11-27。按"方案 / 交接 / 运维 / 审计 / 历史归档"分类。
> **当前有效**的方案以 `SCHEME_PERSISTENT_MAP.md` 为准，其余方案文档作背景与实验记录。
> **最新交接：`HANDOVER_2026-11-27.md`**（当前设备状态、待办、事故记录）。

## 方案（当前有效）

| 文档 | 内容 |
|---|---|
| **SCHEME_PERSISTENT_MAP.md** | **持久度底图方案总纲（当前唯一有效方案）**：三层地图/计票 w 账本/锚定状态机/ELite 对照/落地路线 |
| ephemeral_voxel_map_plan.md | 滚动子图+锚定 v2 方案（2026-11-21）—— 已并入总纲 §2/§4 |
| LOCALIZATION_ANCHOR_2026-11-25.md | 方向可观测性实验报告 —— 已并入总纲 §7（现场配置差异/冻结逃逸） |
| anchor_submap_design.md | 车底贴地/桥接/zip-up 设计 —— 已并入总纲 §2/§3 |

## 交接（时序记录，向后查阅）

| 文档 | 覆盖 |
|---|---|
| **HANDOVER_2026-11-27.md** | **11-25 下午 ~ 11-27**：驾驶中丢定位的根因修复、自更新侵蚀底图、整点丢失、帧耗时治理、账本写入挪出跟踪线程、实车行驶分析、不丢帧的后续方案、11-27 21:45 硬重启 |
| HANDOVER_2026-11-25.md | 评分诚实化/工作图/初值 TF/子图桥接 + 遗留清单 |
| HANDOVER_2026-11-25_device_sync.md | 设备与仓库的版本/配置差异同步 |
| HANDOVER_2026-11-22.md | 锚判据 v1/会话地图仿真验证 |
| HANDOVER_2026-11-21.md | 更早 |

## 专项记录

| 文档 | 内容 |
|---|---|
| LOCALIZATION_THREADS_2026-11-27.md | 线程模型修订：地图更新线程（队列语义、不加锁的账本读视图）、实时优先级的设计与实测（**RT 已撤回**）、回放数据、账本在完整帧率下的变化 |
| GLOBAL_RELOC_2026-11-26.md | 全局重定位：基础设施、ScanContext 在本图上的召回精度、现场配置 |

## 运维手册

| 文档 | 内容 |
|---|---|
| ros1_deploy_runbook.md | 部署/回滚/版本（含 §6.5 假锁事故、设备版本差异） |
| map_import_runbook.md | 地图导入 |
| robot_map_export.md | 地图导出 |

## 审计与专项

| 文档 | 内容 |
|---|---|
| ros1_thread_model.md | 线程模型 |
| ros1_interface_audit.md | 接口审计 |
| ros1_code_review.md | 代码评审 |
| ros1_replacement_test_report.md | 替换测试报告 |

## 历史归档（已被新方案取代或仅作事故记录）

| 文档 | 状态 |
|---|---|
| ephemeral_voxel_map_plan.md 中被总纲覆盖的章节 | 以总纲为准 |
| anchor_submap_design.md 中"雕刻图"章节 | 已废弃（并集/压制组合改为工作图直更） |
| 各 HANDOVER 中的旧参数值 | 以 `config/default.yaml` + 总纲为准 |
