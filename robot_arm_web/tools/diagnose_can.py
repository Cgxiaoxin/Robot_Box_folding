#!/usr/bin/env python3
"""
CAN 总线诊断工具 - 检查双臂连接和使能状态
"""
import sys
import time
sys.path.insert(0, '..')

from arm_control import ArmController, MODE_PROFILE_POSITION

def diagnose_arm(can_channel, motor_ids, arm_name):
    """诊断单个机械臂"""
    print(f"\n{'='*60}")
    print(f"  诊断 {arm_name} 臂 ({can_channel})")
    print(f"{'='*60}")
    
    try:
        # 1. 连接
        print(f"\n[1/5] 连接 {can_channel}...")
        arm = ArmController(motor_ids=motor_ids, can_channel=can_channel)
        print(f"✓ 连接成功")
        
        # 2. 清除报警
        print(f"\n[2/5] 清除报警...")
        arm.clear_all_alarms()
        time.sleep(0.3)
        print(f"✓ 报警已清除")
        
        # 3. 读取位置
        print(f"\n[3/5] 读取电机位置...")
        arm.read_all_positions()
        time.sleep(0.5)
        
        for mid in motor_ids:
            pos = arm.motors[mid].position
            print(f"  电机 {mid}: {pos:.4f} rad")
        print(f"✓ 位置读取成功")
        
        # 4. 设置模式
        print(f"\n[4/5] 设置轨迹位置模式...")
        arm.set_all_modes(MODE_PROFILE_POSITION)
        time.sleep(0.3)
        print(f"✓ 模式设置成功")
        
        # 5. 使能电机
        print(f"\n[5/5] 使能所有电机...")
        arm.enable_all(True)
        time.sleep(0.5)
        
        # 读取使能状态
        arm.read_all_positions()  # 触发状态更新
        time.sleep(0.3)
        
        print(f"\n使能状态检查:")
        enabled_count = 0
        for mid in motor_ids:
            enabled = arm.motors[mid].enabled
            status = "✓ 已使能" if enabled else "✗ 未使能"
            print(f"  电机 {mid}: {status}")
            if enabled:
                enabled_count += 1
        
        if enabled_count == len(motor_ids):
            print(f"\n✓ {arm_name} 臂所有电机使能成功！")
            return True
        else:
            print(f"\n⚠ {arm_name} 臂有 {len(motor_ids) - enabled_count} 个电机未使能")
            return False
        
    except Exception as e:
        print(f"\n✗ {arm_name} 臂诊断失败: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        try:
            arm.close()
        except:
            pass


def main():
    """主函数"""
    print("\n" + "="*60)
    print("  机械臂 CAN 总线诊断工具")
    print("="*60)
    
    # 左臂诊断
    left_ok = diagnose_arm(
        can_channel="can0",
        motor_ids=[51, 52, 53, 54, 55, 56, 57],
        arm_name="左"
    )
    
    time.sleep(1)
    
    # 右臂诊断
    right_ok = diagnose_arm(
        can_channel="can1",
        motor_ids=[61, 62, 63, 64, 65, 66, 67],
        arm_name="右"
    )
    
    # 总结
    print(f"\n{'='*60}")
    print("  诊断总结")
    print(f"{'='*60}")
    print(f"左臂 (can0): {'✓ 正常' if left_ok else '✗ 异常'}")
    print(f"右臂 (can1): {'✓ 正常' if right_ok else '✗ 异常'}")
    print()
    
    if not (left_ok and right_ok):
        print("可能的问题:")
        if not left_ok:
            print("  - 左臂: 检查 can0 连接、电机供电、电缆接线")
        if not right_ok:
            print("  - 右臂: 检查 can1 连接、电机供电、电缆接线")
        print("  - 检查电机是否有报警（红灯闪烁）")
        print("  - 确认 CAN 总线终端电阻")
        print("  - 运行 candump can0/can1 查看实际通信")
    

if __name__ == "__main__":
    main()
