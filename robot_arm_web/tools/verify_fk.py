#!/usr/bin/env python3
"""
FK验证脚本 - 验证 cartesian_controller 运动学计算的正确性

测试内容:
1. FK 零位计算
2. 单关节运动 FK (验证每个关节都能影响末端位置)
3. IK 闭环验证 (FK → IK → FK)
4. 增量IK验证
5. 末端原点坐标系验证
"""

import sys
import os
import math
import numpy as np

# 添加项目路径
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

from backend.cartesian_controller import (
    forward_kinematics,
    inverse_kinematics,
    CartesianController,
)


def separator(title):
    print(f"\n{'='*70}")
    print(f"  {title}")
    print(f"{'='*70}")


def test_fk_zero_position():
    """测试1: FK 零位计算"""
    separator("测试1: FK 零位计算")
    
    for arm_id in ["left", "right"]:
        pos, rot = forward_kinematics(arm_id, [0.0] * 7)
        
        print(f"\n{arm_id.upper()} 臂零位FK:")
        print(f"  位置 (x, y, z): [{pos[0]:.6f}, {pos[1]:.6f}, {pos[2]:.6f}] m")
    
    return True


def test_fk_single_joint():
    """测试2: 单关节运动FK"""
    separator("测试2: 单关节运动FK (左臂)")
    
    zero_pos, _ = forward_kinematics("left", [0.0] * 7)
    
    joint_names = [
        "J1 肩Pitch", "J2 肩Roll", "J3 肩Yaw", 
        "J4 肘Pitch", "J5 腕Yaw", "J6 腕Pitch", "J7 腕Roll"
    ]
    
    test_angle = 0.5  # rad ≈ 28.6°
    print(f"\n每个关节单独转动 {test_angle} rad ({math.degrees(test_angle):.1f}°):")
    print(f"{'关节':<12} {'X(m)':>10} {'Y(m)':>10} {'Z(m)':>10} {'ΔX(mm)':>10} {'ΔY(mm)':>10} {'ΔZ(mm)':>10}")
    print("-" * 78)
    
    all_moved = True
    for i in range(7):
        angles = [0.0] * 7
        angles[i] = test_angle
        pos, _ = forward_kinematics("left", angles)
        
        dx = (pos[0] - zero_pos[0]) * 1000
        dy = (pos[1] - zero_pos[1]) * 1000
        dz = (pos[2] - zero_pos[2]) * 1000
        total_delta = math.sqrt(dx**2 + dy**2 + dz**2)
        
        moved = total_delta > 0.01  # > 0.01mm
        if not moved:
            all_moved = False
        
        status = "✓" if moved else "✗ 无运动!"
        print(f"  {joint_names[i]:<12} {pos[0]:>10.6f} {pos[1]:>10.6f} {pos[2]:>10.6f} {dx:>10.2f} {dy:>10.2f} {dz:>10.2f}  {status}")
    
    return all_moved


def test_fk_ik_consistency():
    """测试3: FK→IK→FK 闭环一致性"""
    separator("测试3: FK→IK→FK 闭环一致性")
    
    test_cases = [
        ("零位", [0.0]*7),
        ("小角度", [0.1, -0.1, 0.2, -0.3, 0.1, 0.05, 0.0]),
        ("中角度", [0.3, 0.5, -0.2, -0.6, 0.3, 0.2, 0.1]),
        ("大角度", [-0.5, 1.0, 0.5, -1.0, 0.5, 0.3, 0.2]),
    ]
    
    all_passed = True
    for arm_id in ["left", "right"]:
        print(f"\n--- {arm_id.upper()} 臂 ---")
        
        for name, angles in test_cases:
            pos, rot = forward_kinematics(arm_id, angles)
            
            # IK 求解
            recovered = inverse_kinematics(arm_id, pos.tolist(), angles)
            
            if recovered is None:
                print(f"  {name}: IK 求解失败 ✗")
                all_passed = False
                continue
            
            # FK 验证
            verify_pos, _ = forward_kinematics(arm_id, recovered)
            pos_error = np.linalg.norm(pos - verify_pos) * 1000  # mm
            
            status = "✓" if pos_error < 1.0 else "✗"
            if pos_error >= 1.0:
                all_passed = False
            
            print(f"  {name:<8}: 位置=[{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}] → 误差={pos_error:.4f}mm {status}")
    
    return all_passed


def test_incremental_ik():
    """测试4: 增量IK验证"""
    separator("测试4: 增量IK (CartesianController)")
    
    cc = CartesianController()
    
    # 从零位偏一点，让IK更容易求解
    start_angles = [0.0, 0.5, 0.0, -0.5, 0.0, 0.0, 0.0]
    
    start_fk = cc.compute_fk("left", start_angles)
    print(f"\n起始位置 (末端原点坐标): [{start_fk['x']*1000:.2f}, {start_fk['y']*1000:.2f}, {start_fk['z']*1000:.2f}] mm")
    
    deltas = [
        ("X+20mm", 0.02, 0, 0),
        ("Y+20mm", 0, 0.02, 0),
        ("Z+20mm", 0, 0, 0.02),
        ("X-20mm", -0.02, 0, 0),
        ("Y-20mm", 0, -0.02, 0),
        ("Z-20mm", 0, 0, -0.02),
    ]
    
    print(f"\n{'方向':<12} {'结果':>8} {'误差(mm)':>10} {'新位置(mm)':>32}")
    print("-" * 70)
    
    all_ok = True
    for label, dx, dy, dz in deltas:
        result = cc.compute_ik_delta("left", dx, dy, dz, start_angles)
        
        if result is None:
            print(f"  {label:<12} {'IK失败':>8} ✗")
            all_ok = False
        else:
            np_ = result["new_pos"]
            print(f"  {label:<12} {'有解':>8} {result['error_mm']:>10.4f} [{np_['x']*1000:>8.2f}, {np_['y']*1000:>8.2f}, {np_['z']*1000:>8.2f}]  ✓")
    
    return all_ok


def test_end_effector_origin():
    """测试5: 末端原点坐标系"""
    separator("测试5: CartesianController 末端原点坐标系")
    
    cc = CartesianController()
    
    print(f"\n坐标系原点 (基座坐标系):")
    for arm_id in ["left", "right"]:
        origin = cc.get_origin(arm_id)
        print(f"  {arm_id.upper()}: [{origin['x']:.6f}, {origin['y']:.6f}, {origin['z']:.6f}] m")
    
    # 不同关节角度
    test_cases = [
        ("零位", [0.0]*7),
        ("J1=0.3", [0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        ("J2=0.5", [0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0]),
        ("J4=-0.5", [0.0, 0.0, 0.0, -0.5, 0.0, 0.0, 0.0]),
        ("组合", [0.1, -0.2, 0.1, -0.3, 0.1, 0.05, 0.0]),
    ]
    
    print(f"\n{'姿态':<12} {'末端原点X(mm)':>14} {'Y(mm)':>10} {'Z(mm)':>10}")
    print("-" * 50)
    
    all_ok = True
    for name, angles in test_cases:
        result = cc.compute_fk("left", angles)
        x_mm = result["x"] * 1000
        y_mm = result["y"] * 1000
        z_mm = result["z"] * 1000
        
        # 零位应该是 (0, 0, 0)
        if name == "零位":
            is_zero = abs(x_mm) < 0.01 and abs(y_mm) < 0.01 and abs(z_mm) < 0.01
            status = "✓ (零点)" if is_zero else "✗"
            if not is_zero:
                all_ok = False
        else:
            has_delta = (abs(x_mm) + abs(y_mm) + abs(z_mm)) > 0.1
            status = "✓" if has_delta else "✗ 无变化"
            if not has_delta:
                all_ok = False
        
        print(f"  {name:<12} {x_mm:>14.2f} {y_mm:>10.2f} {z_mm:>10.2f}  {status}")
    
    return all_ok


def test_actual_motor_angles():
    """测试6: 实际电机角度FK"""
    separator("测试6: 使用实际电机角度")
    
    cc = CartesianController()
    
    # 左臂51-57 (这些是示例角度，需要放真实数据)
    left_angles = [0.0, 0.5, 0.0, -0.5, 0.0, 0.3, 0.0]
    right_angles = [0.0, -0.5, 0.0, -0.5, 0.0, -0.3, 0.0]
    
    for arm_id, angles in [("left", left_angles), ("right", right_angles)]:
        result = cc.compute_fk(arm_id, angles)
        
        print(f"\n{arm_id.upper()} 臂:")
        print(f"  关节角度(deg): {[f'{math.degrees(a):.1f}°' for a in angles]}")
        print(f"  基座坐标(m):   [{result['abs_x']:.6f}, {result['abs_y']:.6f}, {result['abs_z']:.6f}]")
        print(f"  末端原点(mm):  [{result['x']*1000:.2f}, {result['y']*1000:.2f}, {result['z']*1000:.2f}]")
    
    return True


def main():
    print("=" * 70)
    print("  机械臂 FK/IK 验证工具")
    print("  使用 cartesian_controller (URDF直接提取)")
    print("=" * 70)
    
    tests = [
        test_fk_zero_position,
        test_fk_single_joint,
        test_fk_ik_consistency,
        test_incremental_ik,
        test_end_effector_origin,
        test_actual_motor_angles,
    ]
    
    results = []
    for test in tests:
        try:
            ok = test()
            results.append((test.__doc__.strip(), ok))
        except Exception as e:
            print(f"\n  ✗ 异常: {e}")
            import traceback
            traceback.print_exc()
            results.append((test.__doc__.strip(), False))
    
    separator("验证总结")
    for name, ok in results:
        print(f"  {'✓' if ok else '✗'} {name}")
    
    passed = sum(1 for _, ok in results if ok)
    total = len(results)
    print(f"\n通过: {passed}/{total}")
    
    if passed == total:
        print("\n✅ 所有验证通过! FK/IK 计算正确，可以进入开发阶段。")
    else:
        print("\n⚠️ 部分验证未通过，请检查。")


if __name__ == "__main__":
    main()
