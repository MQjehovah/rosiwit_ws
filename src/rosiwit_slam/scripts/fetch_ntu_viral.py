#!/usr/bin/env python3
"""
NTU-VIRAL 数据集下载脚本
从 NTU-VIRAL 公开数据集下载 Ouster OS1-16 LiDAR 数据

数据集信息:
- 来源: https://ntu-aris.github.io/ntu_viral_dataset/
- 传感器: Ouster OS1-16 Gen1 + IMU
- 格式: ROS1 bag (需转换为ROS2)
- 推荐: eee_01 或 sbs_01 序列

使用方法:
    python3 fetch_ntu_viral.py --sequence eee_01 --output ~/rosiwit_data
    python3 fetch_ntu_viral.py --list  # 查看可用序列列表
"""

import argparse
import os
import sys
import subprocess
import json
import hashlib
from pathlib import Path
from typing import Dict, List, Optional
from urllib.parse import urlparse


# NTU-VIRAL 数据集配置
NTU_VIRAL_DATASET = {
    "name": "NTU-VIRAL",
    "url": "https://ntu-aris.github.io/ntu_viral_dataset/",
    "sensor": "Ouster OS1-16 Gen1",
    "format": "ROS1 bag",
    
    # 可用序列列表
    "sequences": {
        "eee_01": {
            "description": "EEE Hall outdoor parking lot scene",
            "size_gb": 8.7,
            "duration_s": 398.7,
            "download_url": "https://researchdata.ntu.edu.sg/api/access/datafile/:persistentId?persistentId=doi:10.21979/N9/C9FJDX/EYJTKZ",
            "filename": "eee_01.zip",
            "bag_name": "eee_01.bag",
            "features": ["outdoor", "parking_lot", "good_loop_closure_candidates"]
        },
        "eee_02": {
            "description": "EEE Hall continuation sequence",
            "size_gb": 7.5,
            "duration_s": 340.2,
            "download_url": "https://researchdata.ntu.edu.sg/api/access/datafile/:persistentId?persistentId=doi:10.21979/N9/C9FJDX/EYJTK0",
            "filename": "eee_02.zip",
            "bag_name": "eee_02.bag",
            "features": ["outdoor", "parking_lot"]
        },
        "sbs_01": {
            "description": "SBS Building outdoor plaza scene",
            "size_gb": 7.8,
            "duration_s": 354.2,
            "download_url": "https://researchdata.ntu.edu.sg/api/access/datafile/:persistentId?persistentId=doi:10.21979/N9/C9FJDX/EYJTK1",
            "filename": "sbs_01.zip",
            "bag_name": "sbs_01.bag",
            "features": ["outdoor", "plaza", "large_loop_closure"]
        },
        "nya_01": {
            "description": "NYA Building indoor/outdoor transition",
            "size_gb": 6.2,
            "duration_s": 280.5,
            "download_url": "https://researchdata.ntu.edu.sg/api/access/datafile/:persistentId?persistentId=doi:10.21979/N9/C9FJDX/EYJTK2",
            "filename": "nya_01.zip",
            "bag_name": "nya_01.bag",
            "features": ["mixed", "indoor_outdoor"]
        },
        "rpg_01": {
            "description": "RPG Building outdoor scene",
            "size_gb": 5.8,
            "duration_s": 265.3,
            "download_url": "https://researchdata.ntu.edu.sg/api/access/datafile/:persistentId?persistentId=doi:10.21979/N9/C9FJDX/EYJTK3",
            "filename": "rpg_01.zip",
            "bag_name": "rpg_01.bag",
            "features": ["outdoor", "building_surrounding"]
        }
    },
    
    # Topic 信息
    "topics": {
        "lidar": "/os1_cloud_node/points",
        "imu": "/os1_cloud_node/imu",
        "lidar_frame": "os_sensor",
        "imu_frame": "os_imu"
    }
}


class DataFetcher:
    """数据集下载管理器"""
    
    def __init__(self, output_dir: str, downloader: str = "wget"):
        self.output_dir = Path(output_dir)
        self.downloader = downloader
        self.dataset_config = NTU_VIRAL_DATASET
        
        # 创建输出目录
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
    def list_sequences(self) -> None:
        """打印可用序列列表"""
        print("\n" + "="*60)
        print(f"  NTU-VIRAL 数据集可用序列")
        print("="*60)
        print(f"传感器: {self.dataset_config['sensor']}")
        print(f"格式: {self.dataset_config['format']}")
        print(f"来源: {self.dataset_config['url']}")
        print("="*60 + "\n")
        
        for seq_name, seq_info in self.dataset_config["sequences"].items():
            features_str = ", ".join(seq_info["features"])
            print(f"[{seq_name}] {seq_info['description']}")
            print(f"    大小: {seq_info['size_gb']} GB | 时长: {seq_info['duration_s']} 秒")
            print(f"    特性: {features_str}")
            print(f"    Topics: {self.dataset_config['topics']['lidar']}, {self.dataset_config['topics']['imu']}")
            print()
    
    def check_disk_space(self, required_gb: float) -> bool:
        """检查磁盘空间是否足够"""
        try:
            # 获取输出目录所在磁盘的空间信息
            stat = os.statvfs(self.output_dir)
            free_gb = (stat.f_frsize * stat.f_bavail) / (1024**3)
            
            print(f"磁盘可用空间: {free_gb:.2f} GB")
            print(f"需要空间: {required_gb:.2f} GB")
            
            if free_gb < required_gb * 1.5:  # 预留50%缓冲
                print(f"警告: 磁盘空间不足，建议预留至少 {required_gb * 1.5:.1f} GB")
                return False
            return True
        except Exception as e:
            print(f"无法检查磁盘空间: {e}")
            return True  # 继续执行
    
    def download_sequence(self, sequence_name: str, use_resume: bool = True) -> Optional[Path]:
        """下载指定序列"""
        if sequence_name not in self.dataset_config["sequences"]:
            print(f"错误: 序列 '{sequence_name}' 不存在")
            print("可用序列: " + ", ".join(self.dataset_config["sequences"].keys()))
            return None
        
        seq_info = self.dataset_config["sequences"][sequence_name]
        output_file = self.output_dir / seq_info["filename"]
        bag_file = self.output_dir / seq_info["bag_name"]
        
        # 检查是否已下载
        if bag_file.exists():
            print(f"数据文件已存在: {bag_file}")
            return bag_file
        
        # 检查是否已下载zip但未解压
        if output_file.exists():
            print(f"ZIP文件已存在: {output_file}")
            print("正在解压...")
            return self.extract_bag(output_file, seq_info)
        
        # 检查磁盘空间
        self.check_disk_space(seq_info["size_gb"])
        
        print(f"\n开始下载序列: {sequence_name}")
        print(f"描述: {seq_info['description']}")
        print(f"大小: {seq_info['size_gb']} GB")
        print(f"保存路径: {output_file}")
        print(f"下载URL: {seq_info['download_url']}")
        print()
        
        # 选择下载工具
        download_cmd = self._build_download_command(
            seq_info["download_url"],
            str(output_file),
            use_resume
        )
        
        print(f"执行下载命令: {download_cmd}")
        print("\n下载中... (请耐心等待，大文件下载可能需要较长时间)")
        print("="*60)
        
        try:
            result = subprocess.run(
                download_cmd,
                shell=True,
                check=True,
                cwd=str(self.output_dir)
            )
            
            if result.returncode == 0 and output_file.exists():
                print("\n下载完成!")
                return self.extract_bag(output_file, seq_info)
            else:
                print(f"下载失败，返回码: {result.returncode}")
                return None
                
        except subprocess.CalledProcessError as e:
            print(f"下载失败: {e}")
            return None
        except Exception as e:
            print(f"下载过程出错: {e}")
            return None
    
    def _build_download_command(self, url: str, output_path: str, use_resume: bool) -> str:
        """构建下载命令"""
        if self.downloader == "wget":
            resume_flag = "-c" if use_resume else ""
            return f"wget {resume_flag} -O '{output_path}' '{url}'"
        
        elif self.downloader == "aria2c":
            resume_flag = "-c" if use_resume else ""
            return f"aria2c -x 16 -s 16 {resume_flag} -d '{self.output_dir}' -o '{os.path.basename(output_path)}' '{url}'"
        
        elif self.downloader == "curl":
            resume_flag = "-C -" if use_resume else ""
            return f"curl {resume_flag} -L -o '{output_path}' '{url}'"
        
        else:
            # 默认使用 wget
            return f"wget -c -O '{output_path}' '{url}'"
    
    def extract_bag(self, zip_file: Path, seq_info: Dict) -> Optional[Path]:
        """解压ZIP文件获取bag文件"""
        bag_file = self.output_dir / seq_info["bag_name"]
        
        if bag_file.exists():
            print(f"bag文件已存在: {bag_file}")
            return bag_file
        
        print(f"\n解压文件: {zip_file}")
        
        try:
            # 使用 unzip 解压
            result = subprocess.run(
                ["unzip", "-o", str(zip_file), "-d", str(self.output_dir)],
                capture_output=True,
                text=True
            )
            
            if result.returncode == 0:
                print("解压完成!")
                
                # 检查bag文件是否存在
                if bag_file.exists():
                    print(f"bag文件: {bag_file}")
                    file_size_mb = bag_file.stat().st_size / (1024 * 1024)
                    print(f"文件大小: {file_size_mb:.1f} MB")
                    return bag_file
                else:
                    # bag文件可能在子目录中
                    print("查找bag文件...")
                    for f in self.output_dir.rglob("*.bag"):
                        print(f"找到bag文件: {f}")
                        return f
                    print("未找到bag文件")
                    return None
            else:
                print(f"解压失败: {result.stderr}")
                return None
                
        except Exception as e:
            print(f"解压过程出错: {e}")
            return None
    
    def get_sequence_info(self, sequence_name: str) -> Dict:
        """获取序列详细信息"""
        if sequence_name in self.dataset_config["sequences"]:
            info = self.dataset_config["sequences"][sequence_name].copy()
            info["topics"] = self.dataset_config["topics"]
            return info
        return {}


def main():
    parser = argparse.ArgumentParser(
        description="NTU-VIRAL 数据集下载脚本",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例用法:
  # 查看可用序列列表
  python3 fetch_ntu_viral.py --list
  
  # 下载 eee_01 序列
  python3 fetch_ntu_viral.py --sequence eee_01 --output ~/rosiwit_data
  
  # 使用 aria2c 多线程下载
  python3 fetch_ntu_viral.py --sequence sbs_01 --downloader aria2c
  
  # 检查磁盘空间
  python3 fetch_ntu_viral.py --check-space eee_01
"""
    )
    
    parser.add_argument("--list", action="store_true", help="列出所有可用序列")
    parser.add_argument("--sequence", type=str, help="要下载的序列名称 (eee_01, sbs_01等)")
    parser.add_argument("--output", type=str, default="~/rosiwit_data", help="输出目录")
    parser.add_argument("--downloader", type=str, default="wget", 
                        choices=["wget", "aria2c", "curl"], help="下载工具")
    parser.add_argument("--check-space", type=str, metavar="SEQUENCE", help="检查下载指定序列的磁盘空间")
    parser.add_argument("--no-resume", action="store_true", help="不使用断点续传")
    
    args = parser.parse_args()
    
    # 处理输出目录路径
    output_dir = os.path.expanduser(args.output)
    
    fetcher = DataFetcher(output_dir, args.downloader)
    
    if args.list:
        fetcher.list_sequences()
        sys.exit(0)
    
    if args.check_space:
        seq_name = args.check_space
        if seq_name in NTU_VIRAL_DATASET["sequences"]:
            seq_info = NTU_VIRAL_DATASET["sequences"][seq_name]
            fetcher.check_disk_space(seq_info["size_gb"])
        else:
            print(f"错误: 序列 '{seq_name}' 不存在")
        sys.exit(0)
    
    if args.sequence:
        result = fetcher.download_sequence(args.sequence, use_resume=not args.no_resume)
        if result:
            print("\n" + "="*60)
            print("下载完成!")
            print(f"bag文件路径: {result}")
            print("="*60)
            sys.exit(0)
        else:
            print("下载失败")
            sys.exit(1)
    
    # 无参数时显示帮助
    parser.print_help()


if __name__ == "__main__":
    main()