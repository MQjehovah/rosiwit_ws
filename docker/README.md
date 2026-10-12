# docker/ —— 镜像构建目录

本目录集中管理 rosiwit 各 Docker 镜像。**约定：每个镜像一个子目录**，目录名 = 镜像名（`rosiwit/<目录名>`）。

## 结构

```
docker/
├── README.md              # 本文件
├── build.sh               # 构建辅助脚本(上下文=仓库根)
├── <image-a>/             # 镜像 A, 对应 rosiwit/<image-a>:<tag>
│   ├── Dockerfile
│   └── ...                # entrypoint/配置/脚本等
└── <image-b>/
    └── ...
```

当前镜像：

| 目录 | 镜像 | 说明 |
|---|---|---|
| `slam-mapping/` | `rosiwit/slam-mapping:humble` | rosiwit_slam 离线建图 |

## 构建

所有镜像的**构建上下文都是仓库根目录**（不是各子目录），这样才能直接 `COPY src/<pkg>`，避免源码重复。
根目录 `.dockerignore` 负责把上下文裁剪到最小。

```bash
# 仓库根目录执行
./docker/build.sh slam-mapping          # -> rosiwit/slam-mapping:humble
./docker/build.sh slam-mapping latest   # 指定 tag
./docker/build.sh                       # 列出所有可用镜像
```

## 新增一个镜像

1. `mkdir docker/<新名字> && cd docker/<新名字>`
2. 放 `Dockerfile`（如需额外文件/脚本一并放在此目录）
3. Dockerfile 里 COPY 用**相对仓库根**的路径，例如：
   ```dockerfile
   COPY docker/<新名字>/some-script.sh /some-script.sh
   COPY src/<某个包> /ws/src/<某个包>
   ```
4. 若需要用到 `src/` 下的新包，在根目录 `.dockerignore` 的白名单里补一行 `!/src/<包名>`（当前只放行 `src/rosiwit_slam`）。
5. 写一个该镜像的 `README.md`（可选），并在上面的表格里登记。
6. `./docker/build.sh <新名字>` 构建。

## 说明

- 不要把源码复制进 `docker/`，统一从 `src/` 取，保持单一来源。
- 基础镜像服务器上无法直连 Docker Hub，尽量复用本地已有镜像（如 `ros:humble-ros-base`）。
