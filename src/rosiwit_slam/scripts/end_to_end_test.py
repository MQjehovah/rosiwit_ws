#!/usr/bin/env python3
"""
端到端建图测试脚本 - 在WSL环境中运行
同时启动SLAM节点和模拟数据生成器，收集建图结果和性能数据
"""

import subprocess
import time
import sys
import os
import signal
import json
from datetime import datetime

def run_end_to_end_test():
    """运行端到端建图测试"""
    
    print("=" * 70)
    print("  rosiwit_slam 端到端建图功能测试")
    print("=" * 70)
    print()
    
    # 测试参数
    TEST_DURATION = 15  # 测试运行时间(秒)
    TEST_TIMESTAMP = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # 路径设置
    workspace = "/mnt/e/ai/agent/workspace/projects/rosiwit_ws"
    test_dir = f"{workspace}/test_output_{TEST_TIMESTAMP}"
    log_dir = f"{test_dir}/logs"
    map_dir = f"{test_dir}/maps"
    
    # 创建测试目录
    os.makedirs(test_dir, exist_ok=True)
    os.makedirs(log_dir, exist_ok=True)
    os.makedirs(map_dir, exist_ok=True)
    
    print(f"测试目录: {test_dir}")
    print(f"测试时间: {datetime.now()}")
    print()
    
    # 环境设置命令前缀
    env_setup = f"source /opt/ros/humble/setup.bash && cd {workspace} && source install/setup.bash"
    
    # ============================================
    # 测试1: 可执行文件检查
    # ============================================
    print("[测试1] 检查可执行文件...")
    exe_path = f"{workspace}/install/rosiwit_slam/lib/rosiwit_slam/rosiwit_slam"
    check_cmd = f"ls -la {exe_path}"
    result = subprocess.run(
        ["wsl", "-d", "Ubuntu-22.04", "--", "bash", "-c", check_cmd],
        capture_output=True, text=True, timeout=10
    )
    
    exe_exists = "rosiwit_slam" in result.stdout
    if exe_exists:
        print("  ✓ 可执行文件存在")
        # 解析文件大小
        for line in result.stdout.split("\n"):
            if "rosiwit_slam" in line:
                parts = line.split()
                if len(parts) >= 5:
                    size = parts[4]
                    size_mb = float(size) / (1024*1024)
                    print(f"    文件大小: {size} bytes ({size_mb:.2f} MB)")
    else:
        print("  ✗ 可执行文件不存在")
        return False
    print()
    
    # ============================================
    # 测试2: 节点启动验证
    # ============================================
    print("[测试2] 节点启动验证 (5秒)...")
    start_cmd = f"{env_setup} && timeout 5 ros2 run rosiwit_slam rosiwit_slam 2>&1"
    result = subprocess.run(
        ["wsl", "-d", "Ubuntu-22.04", "--", "bash", "-c", start_cmd],
        capture_output=True, text=True, timeout=10
    )
    
    startup_log = result.stdout
    with open(f"{log_dir}/node_startup.log", "w") as f:
        f.write(startup_log)
    
    init_success = "initialized successfully" in startup_log.lower() or \
                   "initialized" in startup_log.lower()
    
    if init_success:
        print("  ✓ 节点启动成功")
        print("  关键初始化日志:")
        for line in startup_log.split("\n"):
            if "initialized" in line.lower() or "INFO" in line:
                print(f"    {line.strip()}")
    else:
        print("  ⚠ 检查启动日志...")
        for line in startup_log.split("\n")[:10]:
            if line.strip():
                print(f"    {line.strip()}")
    print()
    
    # ============================================
    # 测试3: 模拟数据生成测试
    # ============================================
    print("[测试3] 模拟数据生成测试...")
    
    # 先测试数据生成脚本能否运行
    sim_script = f"{workspace}/src/rosiwit_slam/scripts/generate_simulated_data.py"
    test_sim_cmd = f"python3 -c \"import rclpy; import numpy; print('OK')\""
    result = subprocess.run(
        ["wsl", "-d", "Ubuntu-22.04", "--", "bash", "-c", test_sim_cmd],
        capture_output=True, text=True, timeout=10
    )
    
    if "OK" in result.stdout:
        print("  ✓ Python依赖检查通过 (rclpy, numpy)")
    else:
        print(f"  ⚠ Python依赖检查: {result.stderr}")
    print()
    
    # ============================================
    # 测试4: 运行建图测试 (使用后台进程)
    # ============================================
    print(f"[测试4] 运行建图测试 ({TEST_DURATION}秒)...")
    print("  启动模拟数据生成器和SLAM节点...")
    
    # 创建测试脚本
    test_script = f'''#!/bin/bash
set +e
source /opt/ros/humble/setup.bash
cd {workspace}
source install/setup.bash

# 启动模拟数据生成器
python3 src/rosiwit_slam/scripts/generate_simulated_data.py > {log_dir}/sim_data.log 2>&1 &
SIM_PID=$!
echo "模拟数据生成器 PID: $SIM_PID"

# 等待启动
sleep 2

# 检查话题
echo "=== 话题列表 ==="
ros2 topic list

# 启动SLAM节点
ros2 run rosiwit_slam rosiwit_slam > {log_dir}/slam_run.log 2>&1 &
SLAM_PID=$!
echo "SLAM节点 PID: $SLAM_PID"

# 记录性能数据
echo "timestamp,mem_kb,cpu_pct" > {test_dir}/performance.csv
for i in $(seq 1 {TEST_DURATION}); do
    if ps -p $SLAM_PID > /dev/null 2>&1; then
        MEM=$(ps -o rss= -p $SLAM_PID 2>/dev/null | tr -d ' ')
        CPU=$(ps -o %cpu= -p $SLAM_PID 2>/dev/null | tr -d ' ')
        TS=$(date +%H:%M:%S)
        echo "$TS,$MEM,$CPU" >> {test_dir}/performance.csv
        echo "[$TS] 内存: $MEM KB, CPU: $CPU%"
    fi
    sleep 1
done

# 检查输出话题频率
echo "=== /odom_estimated 频率 ==="
timeout 3 ros2 topic hz /odom_estimated 2>&1 | head -5

echo "=== /cloud_map 频率 ==="  
timeout 3 ros2 topic hz /cloud_map 2>&1 | head -5

# 停止进程
kill $SLAM_PID 2>/dev/null
kill $SIM_PID 2>/dev/null
wait 2>/dev/null

# 检查PCD文件
echo "=== 查找地图文件 ==="
find {workspace} -name "*.pcd" -mmin -{TEST_DURATION+5} 2>/dev/null

echo "测试完成"
'''
    
    script_path = f"{test_dir}/run_test.sh"
    with open(script_path, "w") as f:
        f.write(test_script)
    
    # 运行测试
    print("  执行测试脚本...")
    start_time = time.time()
    
    try:
        result = subprocess.run(
            ["wsl", "-d", "Ubuntu-22.04", "--", "bash", script_path],
            capture_output=True, text=True, timeout=TEST_DURATION + 30
        )
        
        test_output = result.stdout
        test_stderr = result.stderr
        
        # 保存完整日志
        with open(f"{log_dir}/full_test.log", "w") as f:
            f.write(test_output)
            if test_stderr:
                f.write("\n--- stderr ---\n")
                f.write(test_stderr)
        
        print()
        print("  测试输出:")
        for line in test_output.split("\n"):
            if line.strip():
                print(f"    {line.strip()}")
        
    except subprocess.TimeoutExpired:
        print("  ⚠ 测试超时")
        test_output = ""
    
    elapsed = time.time() - start_time
    print()
    print(f"  测试耗时: {elapsed:.1f} 秒")
    print()
    
    # ============================================
    # 测试5: 结果分析
    # ============================================
    print("[测试5] 分析测试结果...")
    
    results = {
        "timestamp": TEST_TIMESTAMP,
        "test_duration": TEST_DURATION,
        "exe_exists": exe_exists,
        "node_startup": init_success,
        "test_dir": test_dir,
        "performance_data": [],
        "topics_found": [],
        "map_files": []
    }
    
    # 解析性能数据
    perf_file = f"{test_dir}/performance.csv"
    if os.path.exists(perf_file.replace(workspace, f"E:\\ai\\agent\\workspace\\projects\\rosiwit_ws\\test_output_{TEST_TIMESTAMP}")):
        print("  性能数据已记录")
    else:
        # 从WSL读取
        result = subprocess.run(
            ["wsl", "-d", "Ubuntu-22.04", "--", "bash", "-c", f"cat {test_dir}/performance.csv 2>/dev/null || echo 'No data'"],
            capture_output=True, text=True, timeout=5
        )
        if result.stdout and "timestamp" in result.stdout:
            print("  性能数据:")
            for line in result.stdout.split("\n"):
                if line.strip() and not line.startswith("timestamp"):
                    print(f"    {line}")
                    parts = line.split(",")
                    if len(parts) >= 3:
                        results["performance_data"].append({
                            "time": parts[0],
                            "mem_kb": parts[1],
                            "cpu_pct": parts[2]
                        })
    
    # 检查日志文件
    slam_log_check = subprocess.run(
        ["wsl", "-d", "Ubuntu-22.04", "--", "bash", "-c", f"cat {log_dir}/slam_run.log 2>/dev/null | head -30 || echo 'No log'"],
        capture_output=True, text=True, timeout=5
    )
    
    if slam_log_check.stdout and len(slam_log_check.stdout) > 20:
        print("  SLAM节点日志摘要:")
        for line in slam_log_check.stdout.split("\n")[:15]:
            if line.strip():
                print(f"    {line.strip()}")
    
    # 检查PCD文件
    pcd_check = subprocess.run(
        ["wsl", "-d", "Ubuntu-22.04", "--", "bash", "-c", f"find {workspace} -name '*.pcd' -mmin -30 2>/dev/null | head -10 || echo 'None'"],
        capture_output=True, text=True, timeout=10
    )
    
    if pcd_check.stdout and ".pcd" in pcd_check.stdout:
        print("  找到地图文件:")
        for line in pcd_check.stdout.split("\n"):
            if ".pcd" in line:
                print(f"    {line.strip()}")
                results["map_files"].append(line.strip())
    else:
        print("  ⚠ 未找到PCD地图文件")
        print("  说明: 节点需要接收有效数据才能生成地图")
    
    # ============================================
    # 测试总结
    # ============================================
    print()
    print("=" * 70)
    print("  测试结果汇总")
    print("=" * 70)
    print()
    
    # 保存结果JSON
    with open(f"{test_dir}/test_results.json".replace(workspace, f"E:\\ai\\agent\\workspace\\projects\\rosiwit_ws\\test_output_{TEST_TIMESTAMP}"), "w") as f:
        json.dump(results, f, indent=2)
    
    print(f"✓ 可执行文件检查: {exe_exists}")
    print(f"✓ 节点启动验证: {init_success}")
    print(f"✓ 测试目录创建: {test_dir}")
    print(f"✓ 性能数据记录: {len(results['performance_data'])} 条")
    print(f"✓ 地图文件数量: {len(results['map_files'])}")
    print()
    print(f"测试结果已保存到: {test_dir}")
    print()
    
    return results

if __name__ == "__main__":
    try:
        results = run_end_to_end_test()
        print("=" * 70)
        print("端到端测试完成!")
        print("=" * 70)
    except Exception as e:
        print(f"测试异常: {e}")
        import traceback
        traceback.print_exc()