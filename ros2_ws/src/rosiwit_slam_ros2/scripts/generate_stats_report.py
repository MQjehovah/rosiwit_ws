#!/usr/bin/env python3
"""
SLAM 建图性能报告生成脚本

功能:
1. 解析 SLAM 输出日志文件
2. 收集性能统计数据
3. 生成结构化报告（text/json/markdown）

使用方法:
    python3 generate_stats_report.py --input ./logs --output stats_report.txt
    python3 generate_stats_report.py --input ./logs --format json --output stats.json
    python3 generate_stats_report.py --input ./logs --format markdown --output stats.md
"""

import argparse
import os
import sys
import json
import re
import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from datetime import datetime
from collections import defaultdict


class StatsReporter:
    """SLAM 性能统计报告生成器"""
    
    # 性能指标正则表达式
    METRICS_PATTERNS = {
        'frame_count': r'Processed frames:\s*(\d+)',
        'total_time': r'Total processing time:\s*(\d+\.?\d*)',
        'avg_fps': r'Average FPS:\s*(\d+\.?\d*)',
        'current_fps': r'Current FPS:\s*(\d+\.?\d*)',
        'cpu_usage': r'CPU usage:\s*(\d+\.?\d*)%',
        'memory_usage': r'Memory usage:\s*(\d+\.?\d*)\s*(KB|MB|GB)',
        'points_processed': r'Points processed:\s*(\d+)',
        'keyframes': r'Keyframes:\s*(\d+)',
        'loop_closures': r'Loop closures:\s*(\d+)',
        'ikd_tree_size': r'ikd-Tree size:\s*(\d+)',
        'optimization_count': r'Optimization count:\s*(\d+)',
        'avg_processing_time': r'Avg processing time:\s*(\d+\.?\d*)',
        'max_processing_time': r'Max processing time:\s*(\d+\.?\d*)',
        'min_processing_time': r'Min processing time:\s*(\d+\.?\d*)',
        'scan_registration_time': r'Scan registration:\s*(\d+\.?\d*)',
        'map_update_time': r'Map update:\s*(\d+\.?\d*)',
        'optimization_time': r'Optimization:\s*(\d+\.?\d*)',
        'elapsed_time': r'Elapsed time:\s*(\d+\.?\d*)',
    }
    
    def __init__(self, input_dir: str, output_file: str, format: str = "text"):
        self.input_dir = Path(input_dir)
        self.output_file = Path(output_file)
        self.format = format
        self.metrics = defaultdict(list)
        self.trajectory = []
        self.log_files = []
        
        # 初始化统计数据
        self.stats = {
            'summary': {},
            'timing': {},
            'quality': {},
            'resources': {},
            'trajectory': {},
            'timestamp': datetime.now().isoformat()
        }
    
    def find_log_files(self) -> List[Path]:
        """查找输入目录中的日志文件"""
        log_files = []
        
        if self.input_dir.is_file():
            return [self.input_dir]
        
        # 查找所有日志文件
        patterns = ['*.log', '*.txt', 'slam_output*', 'profiler_*']
        for pattern in patterns:
            log_files.extend(self.input_dir.glob(pattern))
        
        return log_files
    
    def parse_log_file(self, log_file: Path) -> Dict:
        """解析单个日志文件"""
        metrics = defaultdict(list)
        
        try:
            with open(log_file, 'r', encoding='utf-8', errors='ignore') as f:
                content = f.read()
            
            # 提取各项指标
            for metric_name, pattern in self.METRICS_PATTERNS.items():
                matches = re.findall(pattern, content, re.IGNORECASE)
                if matches:
                    # 处理带单位的值
                    values = []
                    for match in matches:
                        if isinstance(match, tuple):
                            # 带单位的情况 (如 memory usage)
                            value = float(match[0])
                            unit = match[1] if len(match) > 1 else ''
                            if unit == 'GB':
                                value *= 1024  # 转换为 MB
                            elif unit == 'KB':
                                value /= 1024  # 转换为 MB
                            values.append(value)
                        else:
                            values.append(float(match))
                    
                    metrics[metric_name] = values
            
            # 提取轨迹数据
            trajectory_pattern = r'Position:\s*x\s*=\s*(\d+\.?\d*)\s*,\s*y\s*=\s*(\d+\.?\d*)\s*,\s*z\s*=\s*(\d+\.?\d*)'
            trajectory_matches = re.findall(trajectory_pattern, content)
            for match in trajectory_matches:
                self.trajectory.append([float(match[0]), float(match[1]), float(match[2])])
            
            # 提取时间戳数据
            timestamp_pattern = r'\[\s*(\d+\.?\d*)\s*s\]'
            timestamp_matches = re.findall(timestamp_pattern, content)
            if timestamp_matches:
                metrics['timestamps'] = [float(t) for t in timestamp_matches]
            
        except Exception as e:
            print(f"解析日志文件失败 {log_file}: {e}")
        
        return dict(metrics)
    
    def collect_metrics(self) -> None:
        """收集所有日志文件的指标"""
        self.log_files = self.find_log_files()
        
        if not self.log_files:
            print(f"警告: 未找到日志文件，目录 {self.input_dir}")
            return
        
        print(f"找到 {len(self.log_files)} 个日志文件")
        
        for log_file in self.log_files:
            file_metrics = self.parse_log_file(log_file)
            
            # 合并指标
            for key, values in file_metrics.items():
                self.metrics[key].extend(values)
    
    def compute_statistics(self) -> Dict:
        """计算统计数据"""
        stats = {}
        
        # 处理帧数
        if 'frame_count' in self.metrics:
            stats['total_frames'] = max(self.metrics['frame_count'])
        
        # 处理时间统计
        if 'avg_processing_time' in self.metrics:
            times = self.metrics['avg_processing_time']
            stats['avg_processing_time'] = sum(times) / len(times) if times else 0
        
        if 'max_processing_time' in self.metrics:
            stats['max_processing_time'] = max(self.metrics['max_processing_time'])
        
        if 'min_processing_time' in self.metrics:
            stats['min_processing_time'] = min(self.metrics['min_processing_time'])
        
        # FPS 统计
        if 'avg_fps' in self.metrics:
            stats['avg_fps'] = sum(self.metrics['avg_fps']) / len(self.metrics['avg_fps'])
        
        if 'current_fps' in self.metrics:
            fps_values = self.metrics['current_fps']
            if fps_values:
                stats['min_fps'] = min(fps_values)
                stats['max_fps'] = max(fps_values)
        
        # 关键帧和闭环
        if 'keyframes' in self.metrics:
            stats['total_keyframes'] = max(self.metrics['keyframes'])
        
        if 'loop_closures' in self.metrics:
            stats['loop_closures_detected'] = max(self.metrics['loop_closures'])
        
        # ikd-tree 大小
        if 'ikd_tree_size' in self.metrics:
            stats['ikd_tree_size'] = max(self.metrics['ikd_tree_size'])
        
        # 处理的点数
        if 'points_processed' in self.metrics:
            stats['total_points'] = max(self.metrics['points_processed'])
        
        # 优化次数
        if 'optimization_count' in self.metrics:
            stats['optimization_count'] = max(self.metrics['optimization_count'])
        
        # CPU 使用率
        if 'cpu_usage' in self.metrics:
            cpu_values = self.metrics['cpu_usage']
            stats['avg_cpu_usage'] = sum(cpu_values) / len(cpu_values) if cpu_values else 0
            stats['max_cpu_usage'] = max(cpu_values) if cpu_values else 0
        
        # 内存使用
        if 'memory_usage' in self.metrics:
            mem_values = self.metrics['memory_usage']
            stats['avg_memory_mb'] = sum(mem_values) / len(mem_values) if mem_values else 0
            stats['max_memory_mb'] = max(mem_values) if mem_values else 0
        
        # 处理时间细分
        timing_breakdown = {}
        for key in ['scan_registration_time', 'map_update_time', 'optimization_time']:
            if key in self.metrics:
                values = self.metrics[key]
                timing_breakdown[key] = {
                    'avg': sum(values) / len(values) if values else 0,
                    'max': max(values) if values else 0,
                    'min': min(values) if values else 0
                }
        
        return stats
    
    def compute_trajectory_stats(self) -> Dict:
        """计算轨迹统计"""
        if not self.trajectory:
            return {}
        
        traj_stats = {}
        
        # 轨迹长度
        total_distance = 0
        prev_pos = None
        for pos in self.trajectory:
            if prev_pos:
                dist = math.sqrt(
                    (pos[0] - prev_pos[0])**2 +
                    (pos[1] - prev_pos[1])**2 +
                    (pos[2] - prev_pos[2])**2
                )
                total_distance += dist
            prev_pos = pos
        
        traj_stats['trajectory_length'] = total_distance
        
        # 轨迹范围
        if self.trajectory:
            x_coords = [p[0] for p in self.trajectory]
            y_coords = [p[1] for p in self.trajectory]
            z_coords = [p[2] for p in self.trajectory]
            
            traj_stats['x_range'] = [min(x_coords), max(x_coords)]
            traj_stats['y_range'] = [min(y_coords), max(y_coords)]
            traj_stats['z_range'] = [min(z_coords), max(z_coords)]
        
        traj_stats['trajectory_points'] = len(self.trajectory)
        
        return traj_stats
    
    def compute_quality_metrics(self) -> Dict:
        """计算质量指标"""
        quality = {}
        
        # 基于已有数据计算质量指标
        stats = self.stats.get('summary', {})
        
        # 闭环检测率
        if 'loop_closures_detected' in stats and 'total_keyframes' in stats:
            keyframes = stats['total_keyframes']
            loops = stats['loop_closures_detected']
            quality['loop_closure_rate'] = loops / keyframes if keyframes > 0 else 0
        
        # 实时性评估
        if 'avg_fps' in stats:
            quality['real_time_ratio'] = stats['avg_fps'] / 10.0  # 以10Hz为标准
            
        # 处理效率
        if 'avg_processing_time' in stats and stats['avg_processing_time'] > 0:
            quality['processing_efficiency'] = 1.0 / stats['avg_processing_time']
        
        return quality
    
    def generate_report(self) -> Dict:
        """生成完整报告"""
        # 收集指标
        self.collect_metrics()
        
        # 计算统计
        self.stats['summary'] = self.compute_statistics()
        self.stats['timing'] = {
            'breakdown': {},
            'elapsed_time': 0
        }
        self.stats['trajectory'] = self.compute_trajectory_stats()
        self.stats['quality'] = self.compute_quality_metrics()
        self.stats['resources'] = {
            'cpu': {},
            'memory': {}
        }
        
        # 更新时间细分
        for key in ['scan_registration_time', 'map_update_time', 'optimization_time']:
            if key in self.metrics:
                values = self.metrics[key]
                self.stats['timing']['breakdown'][key] = {
                    'avg': sum(values) / len(values) if values else 0,
                    'max': max(values) if values else 0,
                    'min': min(values) if values else 0,
                    'total': sum(values) if values else 0
                }
        
        # 更新资源统计
        if 'avg_cpu_usage' in self.stats['summary']:
            self.stats['resources']['cpu']['avg_usage'] = self.stats['summary']['avg_cpu_usage']
        
        if 'avg_memory_mb' in self.stats['summary']:
            self.stats['resources']['memory']['avg_usage_mb'] = self.stats['summary']['avg_memory_mb']
        
        # 计算总耗时
        if 'elapsed_time' in self.metrics:
            elapsed = self.metrics['elapsed_time']
            self.stats['timing']['elapsed_time'] = max(elapsed) if elapsed else 0
        
        return self.stats
    
    def format_text_report(self) -> str:
        """生成文本格式报告"""
        report = []
        report.append("=" * 60)
        report.append("       Rosiwit SLAM 建图性能报告")
        report.append("=" * 60)
        report.append(f"生成时间: {self.stats['timestamp']}")
        report.append("")
        
        # 处理统计
        summary = self.stats.get('summary', {})
        if summary:
            report.append("【处理统计】")
            report.append("-" * 40)
            
            if 'total_frames' in summary:
                report.append(f"处理帧数: {summary['total_frames']}")
            
            if 'total_points' in summary:
                report.append(f"处理点数: {summary['total_points']}")
            
            if 'avg_fps' in summary:
                report.append(f"平均帧率: {summary['avg_fps']:.2f} Hz")
            
            if 'avg_processing_time' in summary:
                report.append(f"平均处理时间: {summary['avg_processing_time']:.3f} ms")
            
            report.append("")
        
        # 关键帧和闭环
        report.append("【建图质量】")
        report.append("-" * 40)
        
        if 'total_keyframes' in summary:
            report.append(f"关键帧数量: {summary['total_keyframes']}")
        
        if 'loop_closures_detected' in summary:
            report.append(f"闭环检测数: {summary['loop_closures_detected']}")
        
        if 'ikd_tree_size' in summary:
            report.append(f"ikd-Tree大小: {summary['ikd_tree_size']}")
        
        if 'optimization_count' in summary:
            report.append(f"优化次数: {summary['optimization_count']}")
        
        report.append("")
        
        # 时间分析
        timing = self.stats.get('timing', {})
        if timing.get('breakdown'):
            report.append("【时间分析】")
            report.append("-" * 40)
            
            breakdown = timing['breakdown']
            
            for key, values in breakdown.items():
                label = key.replace('_time', '').replace('_', ' ').title()
                report.append(f"{label}:")
                report.append(f"  平均: {values['avg']:.3f} ms")
                report.append(f"  最大: {values['max']:.3f} ms")
                report.append(f"  最小: {values['min']:.3f} ms")
                report.append(f"  总计: {values['total']:.3f} ms")
            
            if timing.get('elapsed_time'):
                report.append(f"总耗时: {timing['elapsed_time']:.1f} 秒")
            
            report.append("")
        
        # 资源使用
        resources = self.stats.get('resources', {})
        if resources:
            report.append("【资源使用】")
            report.append("-" * 40)
            
            cpu = resources.get('cpu', {})
            if cpu:
                report.append(f"平均CPU使用率: {cpu.get('avg_usage', 0):.1f}%")
            
            memory = resources.get('memory', {})
            if memory:
                report.append(f"平均内存使用: {memory.get('avg_usage_mb', 0):.1f} MB")
            
            report.append("")
        
        # 轨迹统计
        trajectory = self.stats.get('trajectory', {})
        if trajectory:
            report.append("【轨迹统计】")
            report.append("-" * 40)
            
            if 'trajectory_length' in trajectory:
                report.append(f"轨迹长度: {trajectory['trajectory_length']:.2f} 米")
            
            if 'trajectory_points' in trajectory:
                report.append(f"轨迹点数: {trajectory['trajectory_points']}")
            
            if 'x_range' in trajectory:
                report.append(f"X范围: [{trajectory['x_range'][0]:.2f}, {trajectory['x_range'][1]:.2f}] 米")
            
            if 'y_range' in trajectory:
                report.append(f"Y范围: [{trajectory['y_range'][0]:.2f}, {trajectory['y_range'][1]:.2f}] 米")
            
            if 'z_range' in trajectory:
                report.append(f"Z范围: [{trajectory['z_range'][0]:.2f}, {trajectory['z_range'][1]:.2f}] 米")
            
            report.append("")
        
        # 质量评估
        quality = self.stats.get('quality', {})
        if quality:
            report.append("【质量评估】")
            report.append("-" * 40)
            
            if 'loop_closure_rate' in quality:
                rate = quality['loop_closure_rate']
                report.append(f"闭环检测率: {rate:.2%}")
            
            if 'real_time_ratio' in quality:
                ratio = quality['real_time_ratio']
                status = "实时" if ratio >= 1.0 else "非实时"
                report.append(f"实时性: {status} ({ratio:.2f}x)")
            
            if 'processing_efficiency' in quality:
                report.append(f"处理效率: {quality['processing_efficiency']:.2f} 帧/秒")
            
            report.append("")
        
        # 输出文件列表
        report.append("【数据来源】")
        report.append("-" * 40)
        for log_file in self.log_files:
            report.append(f"  - {log_file}")
        
        report.append("")
        report.append("=" * 60)
        
        return "\n".join(report)
    
    def format_json_report(self) -> str:
        """生成 JSON 格式报告"""
        return json.dumps(self.stats, indent=2, ensure_ascii=False)
    
    def format_markdown_report(self) -> str:
        """生成 Markdown 格式报告"""
        report = []
        report.append("# Rosiwit SLAM 建图性能报告")
        report.append("")
        report.append(f"**生成时间**: {self.stats['timestamp']}")
        report.append("")
        
        # 处理统计
        summary = self.stats.get('summary', {})
        if summary:
            report.append("## 处理统计")
            report.append("")
            report.append("| 指标 | 值 |")
            report.append("|------|------|")
            
            if 'total_frames' in summary:
                report.append(f"| 处理帧数 | {summary['total_frames']} |")
            
            if 'total_points' in summary:
                report.append(f"| 处理点数 | {summary['total_points']} |")
            
            if 'avg_fps' in summary:
                report.append(f"| 平均帧率 | {summary['avg_fps']:.2f} Hz |")
            
            if 'avg_processing_time' in summary:
                report.append(f"| 平均处理时间 | {summary['avg_processing_time']:.3f} ms |")
            
            report.append("")
        
        # 建图质量
        report.append("## 建图质量")
        report.append("")
        report.append("| 指标 | 值 |")
        report.append("|------|------|")
        
        if 'total_keyframes' in summary:
            report.append(f"| 关键帧数量 | {summary['total_keyframes']} |")
        
        if 'loop_closures_detected' in summary:
            report.append(f"| 闭环检测数 | {summary['loop_closures_detected']} |")
        
        if 'ikd_tree_size' in summary:
            report.append(f"| ikd-Tree大小 | {summary['ikd_tree_size']} |")
        
        report.append("")
        
        # 轨迹统计
        trajectory = self.stats.get('trajectory', {})
        if trajectory:
            report.append("## 轨迹统计")
            report.append("")
            
            if 'trajectory_length' in trajectory:
                report.append(f"- 轨迹长度: **{trajectory['trajectory_length']:.2f} 米**")
            
            if 'trajectory_points' in trajectory:
                report.append(f"- 轨迹点数: {trajectory['trajectory_points']}")
            
            report.append("")
        
        return "\n".join(report)
    
    def write_report(self) -> None:
        """写入报告文件"""
        # 确保输出目录存在
        self.output_file.parent.mkdir(parents=True, exist_ok=True)
        
        # 生成报告
        self.generate_report()
        
        # 格式化输出
        if self.format == 'json':
            content = self.format_json_report()
        elif self.format == 'markdown':
            content = self.format_markdown_report()
        else:
            content = self.format_text_report()
        
        # 写入文件
        with open(self.output_file, 'w', encoding='utf-8') as f:
            f.write(content)
        
        print(f"报告已生成: {self.output_file}")


def main():
    parser = argparse.ArgumentParser(
        description="SLAM 建图性能报告生成脚本",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例用法:
  # 生成文本报告
  python3 generate_stats_report.py --input ./logs --output stats_report.txt
  
  # 生成 JSON 报告
  python3 generate_stats_report.py --input ./logs --format json --output stats.json
  
  # 生成 Markdown 报告
  python3 generate_stats_report.py --input ./logs --format markdown --output stats.md
"""
    )
    
    parser.add_argument("--input", type=str, required=True,
                        help="输入目录或日志文件路径")
    parser.add_argument("--output", type=str, required=True,
                        help="输出报告文件路径")
    parser.add_argument("--format", type=str, default="text",
                        choices=["text", "json", "markdown"],
                        help="报告格式")
    
    args = parser.parse_args()
    
    reporter = StatsReporter(args.input, args.output, args.format)
    reporter.write_report()
    
    # 输出摘要
    print("\n" + "="*40)
    print("性能摘要:")
    
    stats = reporter.stats.get('summary', {})
    if 'total_frames' in stats:
        print(f"  处理帧数: {stats['total_frames']}")
    if 'avg_fps' in stats:
        print(f"  平均帧率: {stats['avg_fps']:.2f} Hz")
    if 'loop_closures_detected' in stats:
        print(f"  闭环检测: {stats['loop_closures_detected']}")
    
    print("="*40)


if __name__ == "__main__":
    main()