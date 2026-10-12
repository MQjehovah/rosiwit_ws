#!/usr/bin/env python3
"""
normalize_bag.py —— 把 /data/bag 下的录包输入归一化为可播放的 rosbag2 目录。

支持的输入(自动识别):
  1. rosbag2 目录 (metadata.yaml + *.db3)         原样可用
  2. *.zip                                        解压后递归识别
  3. 裸 *.db3 (无 metadata.yaml)                  自动生成 metadata.yaml
  4. *.bag (ROS1)                                 用 rosbags-convert 自动转为 rosbag2

输出: 打印归一化后的 rosbag2 目录绝对路径(含 metadata.yaml), 供 ros2 bag play 使用。
"""
import json
import os
import shutil
import sqlite3
import subprocess
import sys
import zipfile
from pathlib import Path

WORK = Path("/tmp/bag_work")

# rosbag2 0.15 期望的 offered_qos_profiles 为整型枚举编码; ROS1 来源的 qos 字符串
# (如 history: unknown) 会让 yaml-cpp 解析失败, 统一写入本默认 profile。
DEFAULT_QOS = (
    "- history: 3\n"
    "  depth: 0\n"
    "  reliability: 1\n"
    "  durability: 1\n"
    "  deadline:\n"
    "    sec: 9223372036\n"
    "    nsec: 854775807\n"
    "  lifespan:\n"
    "    sec: 9223372036\n"
    "    nsec: 854775807\n"
    "  liveliness: 1\n"
    "  liveliness_lease_duration:\n"
    "    sec: 9223372036\n"
    "    nsec: 854775807\n"
    "  avoid_ros_namespace_conventions: false"
)


def clean_work():
    if WORK.exists():
        shutil.rmtree(WORK, ignore_errors=True)
    WORK.mkdir(parents=True, exist_ok=True)


def unpack_zip(zp: Path, dest: Path):
    dest.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(zp) as z:
        z.extractall(dest)


def gen_metadata_sqlite(db3: Path):
    """为裸 db3 / ROS1 转换产物生成 rosbag2 metadata.yaml (version 5, 匹配本镜像 rosbag2 0.15.x)"""
    conn = sqlite3.connect(str(db3))
    cur = conn.cursor()
    try:
        topics = cur.execute(
            "SELECT id, name, type, serialization_format, offered_qos_profiles "
            "FROM topics").fetchall()
    except sqlite3.OperationalError:
        topics = [(r[0], r[1], r[2], r[3], "") for r in cur.execute(
            "SELECT id, name, type, serialization_format FROM topics").fetchall()]
    blocks = []
    total = 0
    t_min, t_max = None, None
    for tid, name, typ, fmt, qos in topics:
        cnt, t0, t1 = cur.execute(
            "SELECT COUNT(*), MIN(timestamp), MAX(timestamp) FROM messages "
            "WHERE topic_id=?", (tid,)).fetchone()
        total += cnt or 0
        if t0 is not None:
            t_min = t0 if t_min is None else min(t_min, t0)
        if t1 is not None:
            t_max = t1 if t_max is None else max(t_max, t1)
        blocks.append(
            "    - topic_metadata:\n"
            f"        name: {name}\n"
            f"        type: {typ or ''}\n"
            f"        serialization_format: {fmt or 'cdr'}\n"
            f"        offered_qos_profiles: {json.dumps(DEFAULT_QOS)}\n"
            f"      message_count: {cnt or 0}\n")
    conn.close()
    if t_min is None:
        t_min, t_max = 0, 0
    dur = int((t_max or 0) - (t_min or 0))
    t0 = int(t_min or 0)
    rel = db3.name
    out = db3.parent / "metadata.yaml"
    out.write_text(
        "rosbag2_bagfile_information:\n"
        "  version: 5\n"
        "  storage_identifier: sqlite3\n"
        "  duration:\n"
        f"    nanoseconds: {dur}\n"
        "  starting_time:\n"
        f"    nanoseconds_since_epoch: {t0}\n"
        f"  message_count: {total}\n"
        "  topics_with_message_count:\n" + "".join(blocks) +
        '  compression_format: ""\n'
        '  compression_mode: ""\n'
        "  relative_file_paths:\n"
        f"    - {rel}\n"
        "  files:\n"
        f"    - path: {rel}\n"
        "      starting_time:\n"
        f"        nanoseconds_since_epoch: {t0}\n"
        "      duration:\n"
        f"        nanoseconds: {dur}\n"
        f"      message_count: {total}\n"
    )
    return out


def convert_ros1_bag(bag: Path):
    """用 rosbags-convert 把 ROS1 .bag 转为 rosbag2, 再用 Humble 兼容的 metadata 覆盖。"""
    out = WORK / "ros2_from_bag"
    if out.exists():
        shutil.rmtree(out, ignore_errors=True)
    # 注意: rosbags-convert 要求 --dst 不存在, 由其自行创建
    try:
        proc = subprocess.run(
            ["rosbags-convert", "--src", str(bag), "--dst", str(out),
             "--dst-storage", "sqlite3"],
            capture_output=True, text=True)
    except FileNotFoundError:
        print("[MAP-ERROR] 未安装 rosbags-convert, 无法转换 ROS1 .bag", file=sys.stderr)
        return None
    if proc.returncode != 0:
        print("[MAP-ERROR] ROS1 .bag 转换失败:\n" + (proc.stderr or proc.stdout)[-800:],
              file=sys.stderr)
        return None
    db3s = list(out.rglob("*.db3"))
    if not db3s:
        print("[MAP-ERROR] ROS1 .bag 转换后未找到 .db3", file=sys.stderr)
        return None
    # rosbags 默认写 metadata version 9, Humble rosbag2 解析失败; 用自研生成器覆盖为 version 4
    try:
        gen_metadata_sqlite(db3s[0])
    except Exception as e:  # noqa: BLE001
        print("[MAP-WARN] 重新生成 metadata 失败, 沿用转换器产物: %s" % e, file=sys.stderr)
    metas = list(out.rglob("metadata.yaml"))
    if not metas:
        print("[MAP-ERROR] ROS1 .bag 转换后未找到 metadata.yaml", file=sys.stderr)
        return None
    return metas[0].parent


def main():
    src = Path(sys.argv[1] if len(sys.argv) > 1 else "/data/bag")
    clean_work()
    staging = WORK / "src"

    # ---- 1. 把输入拷入可写暂存区 ----
    if src.is_file():
        shutil.copyfile(src, staging / src.name)
    else:
        shutil.copytree(src, staging)

    # ---- 2. 解压 zip (递归一层) ----
    for zp in list(staging.rglob("*.zip")):
        dest = zp.parent / (zp.stem + "_unpacked")
        unpack_zip(zp, dest)
        zp.unlink()

    # ---- 3. 定位 metadata.yaml ----
    metas = list(staging.rglob("metadata.yaml"))
    if metas:
        bag_dir = metas[0].parent
        # 若在子目录中, 上提一层到 WORK 根附近保持简短路径
        print(str(bag_dir.resolve()))
        return

    # ---- 4. 裸 db3 -> 生成 metadata ----
    db3s = list(staging.rglob("*.db3"))
    if db3s:
        db3 = db3s[0]
        meta = gen_metadata_sqlite(db3)
        if not meta.is_file():
            print("metadata generate failed", file=sys.stderr)
            sys.exit(2)
        print(str(db3.parent.resolve()))
        return

    # ---- 5. ROS1 bag: 用 rosbags-convert 转为 rosbag2 ----
    bags = list(staging.rglob("*.bag"))
    if bags:
        converted = convert_ros1_bag(bags[0])
        if converted is None:
            sys.exit(4)
        print(str(converted.resolve()))
        return

    print("未找到可识别的录包内容 (metadata.yaml / *.db3 / *.bag / *.zip)",
          file=sys.stderr)
    sys.exit(2)


if __name__ == "__main__":
    main()
