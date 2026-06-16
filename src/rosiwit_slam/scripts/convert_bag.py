#!/usr/bin/env python3
"""
ROS1 bag 到 ROS2 bag 转换脚本

功能:
1. ROS1 bag → ROS2 bag 格式转换
2. Topic 信息检查和兼容性分析
3. Topic 重映射建议生成

使用方法:
    python3 convert_bag.py --input ~/rosiwit_data/eee_01.bag --output ~/rosiwit_data/eee_01_ros2
    python3 convert_bag.py --info ~/rosiwit_data/eee_01.bag  # 仅查看bag信息
    python3 convert_bag.py --check-compatibility ~/rosiwit_data/eee_01.bag --config ouster_os1.yaml

依赖:
    - ros2 bag convert (ROS2 工具)
    - rosbags 库 (Python)
    - 或使用 rosbag2_bag_migration 工具
"""

import argparse
import os
import sys
import subprocess
import json
import yaml
import re
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from datetime import datetime


# SLAM 系统期望的 topic 配置
EXPECTED_TOPICS = {
    "lidar_topic": "/ouster/points",
    "imu_topic": "/ouster/imu",
    "odom_topic": "/odom"
}

# NTU-VIRAL 数据集的 topic 配置
NTU_VIRAL_TOPICS = {
    "lidar": "/os1_cloud_node/points",
    "imu": "/os1_cloud_node/imu",
    "additional": ["/os1_cloud_node/lidar", "/tf", "/tf_static"]
}


class BagConverter:
    """ROS1 bag 到 ROS2 bag 转换器"""
    
    def __init__(self, input_bag: str, output_dir: Optional[str] = None):
        self.input_bag = Path(input_bag)
        self.output_dir = Path(output_dir) if output_dir else None
        
        if not self.input_bag.exists():
            raise FileNotFoundError(f"输入bag文件不存在: {self.input_bag}")
        
    def get_bag_info_ros1(self) -> Dict:
        """获取 ROS1 bag 文件信息"""
        print(f"获取 ROS1 bag 信息: {self.input_bag}")
        
        try:
            result = subprocess.run(
                ["rosbag", "info", str(self.input_bag)],
                capture_output=True,
                text=True,
                timeout=30
            )
            
            if result.returncode != 0:
                print(f"rosbag info 失败: {result.stderr}")
                return self.parse_bag_info_python()
            
            return self.parse_rosbag_info_output(result.stdout)
            
        except FileNotFoundError:
            print("rosbag 命令未找到，使用 Python 解析")
            return self.parse_bag_info_python()
        except Exception as e:
            print(f"获取bag信息失败: {e}")
            return self.parse_bag_info_python()
    
    def parse_rosbag_info_output(self, output: str) -> Dict:
        """解析 rosbag info 命令输出"""
        info = {
            "path": str(self.input_bag),
            "topics": [],
            "duration": 0,
            "messages": 0,
            "size": 0
        }
        
        # 解析基本信息
        for line in output.split('\n'):
            if 'duration:' in line:
                match = re.search(r'duration:\s+(\d+\.?\d*)', line)
                if match:
                    info['duration'] = float(match.group(1))
            
            elif 'messages:' in line:
                match = re.search(r'messages:\s+(\d+)', line)
                if match:
                    info['messages'] = int(match.group(1))
            
            elif 'size:' in line:
                match = re.search(r'size:\s+(\d+)', line)
                if match:
                    info['size'] = int(match.group(1))
            
            elif 'types:' in line:
                # 消息类型部分开始
                pass
        
        # 解析 topics
        topics_section = re.search(r'types:(.+?)(?=end:|$)', output, re.DOTALL)
        if topics_section:
            for line in topics_section.group(1).split('\n'):
                if line.strip() and not line.startswith('types'):
                    topic_match = re.search(r'(\S+)\s+(\S+)\s+\d+\s+msgs', line)
                    if topic_match:
                        info['topics'].append({
                            'name': topic_match.group(2),
                            'type': topic_match.group(1),
                            'count': int(re.search(r'\d+\s+msgs', line).group().split()[0])
                        })
        
        return info
    
    def parse_bag_info_python(self) -> Dict:
        """使用 Python 解析 bag 文件信息"""
        info = {
            "path": str(self.input_bag),
            "topics": [],
            "duration": 0,
            "messages": 0,
            "size": self.input_bag.stat().st_size
        }
        
        # 尝试使用 rosbags 库
        try:
            from rosbags.rosbag1 import Reader
            from rosbags.typesys.stores.ros1_noetic import FIELDTYPES
            
            with Reader(str(self.input_bag)) as reader:
                info['duration'] = reader.end_time - reader.start_time
                info['messages'] = reader.message_count
                
                for conn in reader.connections:
                    info['topics'].append({
                        'name': conn.topic,
                        'type': conn.msgtype,
                        'count': conn.msgcount
                    })
                    
            return info
            
        except ImportError:
            print("rosbags 库未安装，使用基本解析")
            print("建议安装: pip install rosbags")
            
            # 基本文件信息
            return info
    
    def check_topic_compatibility(self, bag_info: Dict, config_file: Optional[str] = None) -> Dict:
        """检查 topic 兼容性"""
        compatibility = {
            "compatible": True,
            "missing_topics": [],
            "mismatched_topics": [],
            "remapping_needed": {},
            "warnings": []
        }
        
        # 加载配置文件
        expected_topics = EXPECTED_TOPICS.copy()
        if config_file and Path(config_file).exists():
            try:
                with open(config_file, 'r') as f:
                    config = yaml.safe_load(f)
                    if 'ros' in config:
                        expected_topics['lidar_topic'] = config['ros'].get('lidar_topic', expected_topics['lidar_topic'])
                        expected_topics['imu_topic'] = config['ros'].get('imu_topic', expected_topics['imu_topic'])
            except Exception as e:
                print(f"配置文件解析失败: {e}")
        
        # 获取 bag 中的 topics
        bag_topics = {t['name']: t for t in bag_info['topics']}
        
        # 检查 LiDAR topic
        lidar_found = False
        for topic_name in bag_topics:
            if 'points' in topic_name or 'lidar' in topic_name or 'cloud' in topic_name:
                if topic_name != expected_topics['lidar_topic']:
                    compatibility['remapping_needed'][topic_name] = expected_topics['lidar_topic']
                    compatibility['warnings'].append(f"LiDAR topic 需要重映射: {topic_name} → {expected_topics['lidar_topic']}")
                lidar_found = True
                break
        
        if not lidar_found:
            compatibility['missing_topics'].append('lidar')
            compatibility['warnings'].append("未找到 LiDAR 点云 topic")
        
        # 检查 IMU topic
        imu_found = False
        for topic_name in bag_topics:
            if 'imu' in topic_name.lower():
                if topic_name != expected_topics['imu_topic']:
                    compatibility['remapping_needed'][topic_name] = expected_topics['imu_topic']
                    compatibility['warnings'].append(f"IMU topic 需要重映射: {topic_name} → {expected_topics['imu_topic']}")
                imu_found = True
                break
        
        if not imu_found:
            compatibility['missing_topics'].append('imu')
            compatibility['warnings'].append("未找到 IMU topic")
        
        # 判断兼容性
        if compatibility['missing_topics']:
            compatibility['compatible'] = False
        
        return compatibility
    
    def generate_remapping_commands(self, remapping: Dict) -> str:
        """生成 topic 重映射命令"""
        if not remapping:
            return ""
        
        commands = []
        for source, target in remapping.items():
            commands.append(f"--remap {source}:={target}")
        
        return " ".join(commands)
    
    def convert_to_ros2(self, output_dir: Optional[str] = None) -> Tuple[bool, Optional[Path]]:
        """转换 ROS1 bag 到 ROS2 格式"""
        output_path = Path(output_dir) if output_dir else self.output_dir
        
        if not output_path:
            # 默认在同一目录创建
            output_path = self.input_bag.parent / (self.input_bag.stem + "_ros2")
        
        print(f"\n开始转换 ROS1 bag → ROS2 bag")
        print(f"输入: {self.input_bag}")
        print(f"输出: {output_path}")
        print()
        
        # 方法 1: 使用 ros2 bag convert
        try:
            result = subprocess.run(
                ["ros2", "bag", "convert", "-i", str(self.input_bag), "-o", str(output_path)],
                capture_output=True,
                text=True,
                timeout=300
            )
            
            if result.returncode == 0:
                print("转换成功 (使用 ros2 bag convert)")
                return True, output_path
            else:
                print(f"ros2 bag convert 失败: {result.stderr}")
                
        except FileNotFoundError:
            print("ros2 bag convert 命令未找到")
        except Exception as e:
            print(f"ros2 bag convert 错误: {e}")
        
        # 方法 2: 使用 rosbags 库
        print("\n尝试使用 rosbags 库转换...")
        try:
            from rosbags.rosbag1 import Reader
            from rosbags.rosbag2 import Writer
            from rosbags.typesys.stores.ros1_noetic import FIELDTYPES
            from rosbags.typesys.stores.ros2_humble import FIELDTYPES as ROS2_FIELDTYPES
            
            return self.convert_with_rosbags(output_path)
            
        except ImportError:
            print("rosbags 库未安装")
            print("安装方法: pip install rosbags")
            print()
        
        # 方法 3: 使用 rosbag2_bag_migration 工具
        print("\n尝试使用 rosbag2_bag_migration...")
        try:
            result = subprocess.run(
                ["ros2", "launch", "rosbag2_bag_migration", "migration_launch.py"],
                capture_output=True,
                text=True,
                timeout=60
            )
            
            if result.returncode == 0:
                # 查找转换后的文件
                converted_file = self.input_bag.parent / (self.input_bag.stem + "_converted")
                if converted_file.exists():
                    return True, converted_file
                    
        except Exception as e:
            print(f"rosbag2_bag_migration 错误: {e}")
        
        print("\n转换失败!")
        print("请确保已安装 ROS2 环境，或使用 rosbags Python 库")
        print("备选方案: 在 ROS2 环境中使用 'ros2 bag play' 配合 'rosbag record' 重新录制")
        
        return False, None
    
    def convert_with_rosbags(self, output_path: Path) -> Tuple[bool, Optional[Path]]:
        """使用 rosbags 库进行转换"""
        print("使用 rosbags 库进行转换...")
        
        try:
            from rosbags.rosbag1 import Reader
            from rosbags.rosbag2 import Writer
            from rosbags.typesys import get_types_from_msg, register_types
            from rosbags.typesys.stores.ros1_noetic import (
                sensor_msgs__msg__PointCloud2,
                sensor_msgs__msg__Imu,
                geometry_msgs__msg__TransformStamped,
            )
            
            # 创建输出目录
            output_path.mkdir(parents=True, exist_ok=True)
            
            # 转换
            with Reader(str(self.input_bag)) as reader:
                with Writer(str(output_path)) as writer:
                    # 设置开始时间
                    writer.start_time = reader.start_time
                    
                    # 添加连接
                    for conn in reader.connections:
                        # 使用相同的消息类型
                        writer.add_connection(conn.topic, conn.msgtype)
                    
                    # 写入消息
                    for conn, timestamp, data in reader.messages():
                        writer.write(conn.topic, conn.msgtype, timestamp, data)
            
            print(f"转换完成: {output_path}")
            return True, output_path
            
        except Exception as e:
            print(f"rosbags 转换失败: {e}")
            return False, None
    
    def verify_ros2_bag(self, bag_path: Path) -> bool:
        """验证 ROS2 bag 文件"""
        print(f"\n验证 ROS2 bag: {bag_path}")
        
        try:
            result = subprocess.run(
                ["ros2", "bag", "info", str(bag_path)],
                capture_output=True,
                text=True,
                timeout=30
            )
            
            if result.returncode == 0:
                print("ROS2 bag 验证成功")
                print(result.stdout)
                return True
            else:
                print(f"验证失败: {result.stderr}")
                return False
                
        except FileNotFoundError:
            print("ros2 命令未找到")
            return False
        except Exception as e:
            print(f"验证错误: {e}")
            return False
    
    def generate_play_command(self, bag_path: Path, remapping: Dict = None) -> str:
        """生成播放命令"""
        cmd = f"ros2 bag play {bag_path} --clock"
        
        if remapping:
            for source, target in remapping.items():
                cmd += f" --remap {source}:={target}"
        
        return cmd


def print_bag_info(info: Dict):
    """打印 bag 信息"""
    print("\n" + "="*60)
    print("  Bag 文件信息")
    print("="*60)
    print(f"路径: {info['path']}")
    print(f"时长: {info['duration']:.1f} 秒")
    print(f"消息数: {info['messages']}")
    print(f"大小: {info['size'] / (1024*1024):.1f} MB")
    print()
    print("Topics:")
    print("-"*60)
    for topic in info['topics']:
        print(f"  {topic['name']}")
        print(f"    类型: {topic['type']}")
        print(f"    数量: {topic['count']}")
    print("="*60)


def print_compatibility_report(compatibility: Dict):
    """打印兼容性报告"""
    print("\n" + "="*60)
    print("  Topic 兼容性分析")
    print("="*60)
    
    status = "✅ 兼容" if compatibility['compatible'] else "❌ 不兼容"
    print(f"状态: {status}")
    
    if compatibility['missing_topics']:
        print("\n缺失的 topics:")
        for topic in compatibility['missing_topics']:
            print(f"  - {topic}")
    
    if compatibility['remapping_needed']:
        print("\n需要重映射:")
        for source, target in compatibility['remapping_needed'].items():
            print(f"  {source} → {target}")
    
    if compatibility['warnings']:
        print("\n警告:")
        for warning in compatibility['warnings']:
            print(f"  ⚠️ {warning}")
    
    print("="*60)


def main():
    parser = argparse.ArgumentParser(
        description="ROS1 bag 到 ROS2 bag 转换脚本",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例用法:
  # 查看 bag 信息
  python3 convert_bag.py --info ~/rosiwit_data/eee_01.bag
  
  # 转换 bag 文件
  python3 convert_bag.py --input ~/rosiwit_data/eee_01.bag --output ~/rosiwit_data/eee_01_ros2
  
  # 检查兼容性并生成重映射命令
  python3 convert_bag.py --check-compatibility ~/rosiwit_data/eee_01.bag --config ouster_os1_16.yaml
  
  # 自动转换并生成播放命令
  python3 convert_bag.py --input ~/rosiwit_data/eee_01.bag --auto-remap
"""
    )
    
    parser.add_argument("--info", type=str, metavar="BAG_FILE", help="仅查看bag文件信息")
    parser.add_argument("--input", type=str, help="输入 ROS1 bag 文件路径")
    parser.add_argument("--output", type=str, help="输出 ROS2 bag 目录路径")
    parser.add_argument("--check-compatibility", type=str, metavar="BAG_FILE", help="检查topic兼容性")
    parser.add_argument("--config", type=str, help="SLAM配置文件路径")
    parser.add_argument("--auto-remap", action="store_true", help="自动生成重映射命令")
    
    args = parser.parse_args()
    
    # 仅查看信息
    if args.info:
        converter = BagConverter(args.info)
        info = converter.get_bag_info_ros1()
        print_bag_info(info)
        sys.exit(0)
    
    # 仅检查兼容性
    if args.check_compatibility:
        converter = BagConverter(args.check_compatibility)
        info = converter.get_bag_info_ros1()
        compatibility = converter.check_topic_compatibility(info, args.config)
        
        print_bag_info(info)
        print_compatibility_report(compatibility)
        
        # 生成重映射命令
        if compatibility['remapping_needed']:
            remap_cmd = converter.generate_remapping_commands(compatibility['remapping_needed'])
            print("\n推荐播放命令:")
            print(f"ros2 bag play <bag_path> {remap_cmd} --clock")
        
        sys.exit(0)
    
    # 转换 bag
    if args.input:
        converter = BagConverter(args.input, args.output)
        
        # 获取信息
        info = converter.get_bag_info_ros1()
        print_bag_info(info)
        
        # 检查兼容性
        compatibility = converter.check_topic_compatibility(info, args.config)
        print_compatibility_report(compatibility)
        
        # 转换
        success, output_path = converter.convert_to_ros2(args.output)
        
        if success:
            # 验证
            converter.verify_ros2_bag(output_path)
            
            # 生成播放命令
            if args.auto_remap and compatibility['remapping_needed']:
                play_cmd = converter.generate_play_command(output_path, compatibility['remapping_needed'])
                print("\n播放命令:")
                print(play_cmd)
            
            print("\n" + "="*60)
            print("转换完成!")
            print(f"ROS2 bag 路径: {output_path}")
            print("="*60)
            sys.exit(0)
        else:
            print("转换失败")
            sys.exit(1)
    
    # 无参数时显示帮助
    parser.print_help()


if __name__ == "__main__":
    main()