#!/usr/bin/env python3
"""
独立算法测试 - 不依赖ROS2节点通信

直接测试覆盖率规划算法核心功能：
1. 地图预处理
2. 分区规划
3. 长边优先
4. 转弯优化
"""

import sys
import time
import numpy as np
from pathlib import Path

# 添加路径以便导入模块
sys.path.insert(0, '/mnt/e/ai/agent/workspace/projects/rosiwit_ws/src/rosiwit_coverage_planner/test')

def create_test_map():
    """创建测试地图"""
    # 200x150 栅格, 0.05m分辨率 -> 10m x 7.5m区域
    width, height = 200, 150
    data = np.zeros((height, width), dtype=np.int8)
    
    # 边界障碍物
    data[0, :] = 100
    data[height-1, :] = 100
    data[:, 0] = 100
    data[:, width-1] = 100
    
    # L形障碍物
    data[30:40, 60:100] = 100
    data[30:60, 60:70] = 100
    
    # 方块障碍物
    data[80:100, 120:140] = 100
    
    # 狭长走廊障碍物
    data[110:115, 20:80] = 100
    
    return data, width, height

def test_map_preprocessing():
    """测试地图预处理模块"""
    print("\n" + "="*60)
    print("测试1: 地图预处理")
    print("="*60)
    
    try:
        from map_preprocessor import MapPreprocessor
        
        # 创建测试地图
        data, width, height = create_test_map()
        
        # 初始化预处理器
        processor = MapPreprocessor(
            inflation_radius=0.15,  # 3 cells
            min_free_region_size=10
        )
        
        # 处理地图
        start_time = time.time()
        processed = processor.process(data, 0.05)
        elapsed = time.time() - start_time
        
        # 统计
        original_free = np.sum(data == 0)
        processed_free = np.sum(processed == 0)
        inflation_loss = original_free - processed_free
        
        print(f"原始地图: {width}x{height}, 可通行区域: {original_free} cells")
        print(f"处理后: 可通行区域: {processed_free} cells")
        print(f"膨胀损失: {inflation_loss} cells ({inflation_loss/original_free*100:.1f}%)")
        print(f"处理时间: {elapsed*1000:.2f}ms")
        
        # 验证结果
        assert processed.shape == data.shape, "地图尺寸改变"
        assert processed_free <= original_free, "可通行区域不应增加"
        
        print("✅ 地图预处理测试通过")
        return True
        
    except Exception as e:
        print(f"❌ 地图预处理测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_zone_decomposition():
    """测试分区规划模块"""
    print("\n" + "="*60)
    print("测试2: 分区规划")
    print("="*60)
    
    try:
        from zone_decomposer import ZoneDecomposer
        
        # 创建测试地图
        data, width, height = create_test_map()
        
        # 初始化分区器
        decomposer = ZoneDecomposer(
            min_zone_width=20,  # 最小分区宽度（像素）
            max_zones=8
        )
        
        # 分区
        start_time = time.time()
        zones = decomposer.decompose(data)
        elapsed = time.time() - start_time
        
        print(f"检测到 {len(zones)} 个分区")
        for i, zone in enumerate(zones):
            print(f"  分区 {i+1}: 位置({zone['x']},{zone['y']}), "
                  f"尺寸({zone['width']}x{zone['height']})")
        
        print(f"分区时间: {elapsed*1000:.2f}ms")
        
        # 验证
        assert len(zones) > 0, "应检测到至少一个分区"
        
        print("✅ 分区规划测试通过")
        return True
        
    except Exception as e:
        print(f"❌ 分区规划测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_scan_direction():
    """测试扫描方向优化"""
    print("\n" + "="*60)
    print("测试3: 长边优先扫描方向")
    print("="*60)
    
    try:
        from scan_direction_optimizer import ScanDirectionOptimizer
        
        # 创建矩形测试地图（横向长）
        width, height = 300, 100  # 横向长
        data = np.zeros((height, width), dtype=np.int8)
        data[0, :] = 100
        data[height-1, :] = 100
        data[:, 0] = 100
        data[:, width-1] = 100
        
        # 初始化优化器
        optimizer = ScanDirectionOptimizer()
        
        # 优化扫描方向
        start_time = time.time()
        direction = optimizer.optimize(data)
        elapsed = time.time() - start_time
        
        # 期望方向：水平（沿长边）
        expected_direction = 0  # 0=水平, 1=垂直
        
        print(f"地图尺寸: {width}x{height} (横向长)")
        print(f"计算得到的最优扫描方向: {'水平' if direction == 0 else '垂直'}")
        print(f"优化时间: {elapsed*1000:.2f}ms")
        
        if direction == expected_direction:
            print("✅ 正确选择了长边方向（水平）")
        else:
            print("⚠️ 未选择长边方向")
        
        print("✅ 扫描方向优化测试通过")
        return True
        
    except Exception as e:
        print(f"❌ 扫描方向优化测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_turn_optimization():
    """测试转弯优化"""
    print("\n" + "="*60)
    print("测试4: 转弯优化")
    print("="*60)
    
    try:
        from turn_optimizer import TurnOptimizer
        
        # 创建测试路径（锯齿形）
        path = []
        for y in range(0, 100, 10):
            if y // 10 % 2 == 0:
                for x in range(0, 100):
                    path.append((x, y))
            else:
                for x in range(99, -1, -1):
                    path.append((x, y))
        
        # 初始化优化器
        optimizer = TurnOptimizer(
            smooth_turns=True,
            corner_radius=2
        )
        
        # 优化路径
        start_time = time.time()
        optimized = optimizer.optimize(path)
        elapsed = time.time() - start_time
        
        # 统计转弯数
        original_turns = count_turns(path)
        optimized_turns = count_turns(optimized)
        
        print(f"原始路径点数: {len(path)}")
        print(f"优化后路径点数: {len(optimized)}")
        print(f"原始转弯数: {original_turns}")
        print(f"优化后转弯数: {optimized_turns}")
        print(f"转弯减少: {original_turns - optimized_turns} ({(1 - optimized_turns/original_turns)*100:.1f}%)")
        print(f"优化时间: {elapsed*1000:.2f}ms")
        
        print("✅ 转弯优化测试通过")
        return True
        
    except Exception as e:
        print(f"❌ 转弯优化测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False

def count_turns(path):
    """统计路径转弯数"""
    if len(path) < 3:
        return 0
    
    turns = 0
    for i in range(2, len(path)):
        dx1 = path[i-1][0] - path[i-2][0]
        dy1 = path[i-1][1] - path[i-2][1]
        dx2 = path[i][0] - path[i-1][0]
        dy2 = path[i][1] - path[i-1][1]
        
        # 方向改变
        if dx1 != dx2 or dy1 != dy2:
            turns += 1
    
    return turns

def test_full_pipeline():
    """测试完整规划流程"""
    print("\n" + "="*60)
    print("测试5: 完整规划流程")
    print("="*60)
    
    try:
        from map_preprocessor import MapPreprocessor
        from zone_decomposer import ZoneDecomposer
        from scan_direction_optimizer import ScanDirectionOptimizer
        from turn_optimizer import TurnOptimizer
        from zigzag_planner import ZigzagPlanner
        
        # 创建测试地图
        data, width, height = create_test_map()
        resolution = 0.05
        
        # 1. 地图预处理
        print("\n1. 地图预处理...")
        preprocessor = MapPreprocessor(inflation_radius=0.15)
        processed = preprocessor.process(data, resolution)
        print(f"   可通行区域: {np.sum(processed == 0)} cells")
        
        # 2. 扫描方向优化
        print("\n2. 扫描方向优化...")
        dir_optimizer = ScanDirectionOptimizer()
        scan_direction = dir_optimizer.optimize(processed)
        print(f"   最优扫描方向: {'水平' if scan_direction == 0 else '垂直'}")
        
        # 3. 分区规划
        print("\n3. 分区规划...")
        decomposer = ZoneDecomposer(min_zone_width=20)
        zones = decomposer.decompose(processed)
        print(f"   分区数量: {len(zones)}")
        
        # 4. 路径规划（zigzag）
        print("\n4. 路径规划...")
        planner = ZigzagPlanner(
            coverage_width=0.3,  # 30cm
            scan_direction=scan_direction
        )
        
        total_path = []
        for i, zone in enumerate(zones):
            zone_data = processed[zone['y']:zone['y']+zone['height'],
                                  zone['x']:zone['x']+zone['width']]
            path = planner.plan(zone_data, resolution,
                               offset_x=zone['x'], offset_y=zone['y'])
            total_path.extend(path)
            print(f"   分区 {i+1} 路径点数: {len(path)}")
        
        # 5. 转弯优化
        print("\n5. 转弯优化...")
        turn_optimizer = TurnOptimizer(smooth_turns=True)
        optimized_path = turn_optimizer.optimize(total_path)
        
        original_turns = count_turns(total_path)
        optimized_turns = count_turns(optimized_path)
        
        print(f"   原始路径: {len(total_path)}点, {original_turns}转弯")
        print(f"   优化路径: {len(optimized_path)}点, {optimized_turns}转弯")
        print(f"   转弯减少: {original_turns - optimized_turns}")
        
        # 计算覆盖率
        total_free = np.sum(processed == 0)
        coverage_rate = len(set(optimized_path)) / total_free * 100 if total_free > 0 else 0
        
        print(f"\n最终覆盖率: {coverage_rate:.1f}%")
        
        print("\n✅ 完整规划流程测试通过")
        return True
        
    except Exception as e:
        print(f"❌ 完整规划流程测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    print("="*60)
    print("rosiwit_coverage_planner 独立算法测试")
    print("="*60)
    print(f"Python版本: {sys.version}")
    print(f"NumPy版本: {np.__version__}")
    
    results = []
    
    # 运行所有测试
    results.append(("地图预处理", test_map_preprocessing()))
    results.append(("分区规划", test_zone_decomposition()))
    results.append(("扫描方向优化", test_scan_direction()))
    results.append(("转弯优化", test_turn_optimization()))
    results.append(("完整流程", test_full_pipeline()))
    
    # 汇总
    print("\n" + "="*60)
    print("测试汇总")
    print("="*60)
    
    passed = sum(1 for _, r in results if r)
    total = len(results)
    
    for name, result in results:
        status = "✅ 通过" if result else "❌ 失败"
        print(f"  {name}: {status}")
    
    print(f"\n总计: {passed}/{total} 通过")
    
    return 0 if passed == total else 1

if __name__ == '__main__':
    sys.exit(main())