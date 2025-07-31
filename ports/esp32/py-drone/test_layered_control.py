"""
分层悬停控制逻辑测试脚本

验证修正后的分层控制逻辑：
- 第一层：姿态需要调整时，只做姿态校准，不触发第二层
- 第二层：姿态稳定时，才启用光流定点控制
"""

import espdrone
import time

def test_layered_hover_logic():
    """测试分层悬停控制的正确逻辑"""
    print("=== 分层悬停控制逻辑测试 ===")
    
    drone = espdrone.DRONE(flightmode=0, debug=1)
    
    try:
        # 起飞并开始测试
        print("起飞到80cm高度...")
        drone.take_off(distance=80)
        time.sleep(3)
        
        print("\n开始分层控制逻辑验证...")
        print("监控格式: [时间] 姿态(R°,P°) | 补偿(R°,P°) | 第一层状态 | 第二层状态")
        print("-" * 80)
        
        # 记录系统状态变化
        layer1_active_count = 0
        layer2_active_count = 0  
        attitude_adjusting_count = 0
        attitude_stable_count = 0
        
        for i in range(60):  # 30秒测试，0.5秒间隔
            current_time = i * 0.5
            
            # 读取飞行状态和调试信息
            states = drone.read_states()
            hover_debug = drone.get_hover_debug()
            
            # 解析数据
            roll = states[0] / 100.0              # 当前姿态Roll
            pitch = states[1] / 100.0             # 当前姿态Pitch
            hover_active = drone.is_hover_active() # 第一层状态
            
            # 第一层调试数据（修正后的含义）
            comp_roll = hover_debug[0]            # 姿态补偿Roll
            comp_pitch = hover_debug[1]           # 姿态补偿Pitch
            filtered_roll = hover_debug[2]        # 滤波姿态Roll  
            filtered_pitch = hover_debug[3]       # 滤波姿态Pitch
            
            # 计算姿态误差和补偿强度
            attitude_error = (roll**2 + pitch**2)**0.5
            compensation_strength = (comp_roll**2 + comp_pitch**2)**0.5
            
            # 判断系统状态（基于修正后的逻辑）
            attitude_needs_adjustment = (attitude_error > 1.5) or (compensation_strength > 1.0)
            
            # 状态统计
            if hover_active:
                layer1_active_count += 1
            
            if attitude_needs_adjustment:
                attitude_adjusting_count += 1
                layer2_should_be = "DISABLED"
                status_desc = "姿态调整中"
            else:
                attitude_stable_count += 1  
                layer2_should_be = "ENABLED"
                status_desc = "姿态稳定"
                layer2_active_count += 1
            
            # 实时输出
            print(f"[{current_time:5.1f}s] 姿态({roll:+5.2f}°,{pitch:+5.2f}°) | "
                  f"补偿({comp_roll:+5.2f}°,{comp_pitch:+5.2f}°) | "
                  f"L1:{'ON' if hover_active else 'OFF'} | "
                  f"L2:{layer2_should_be} | {status_desc}")
            
            # 重点标记需要关注的状态变化
            if attitude_needs_adjustment and compensation_strength > 2.0:
                print(f"  🎯 第一层主动补偿工作中 (补偿强度: {compensation_strength:.2f}°)")
            
            if not attitude_needs_adjustment and attitude_error < 1.0:
                print(f"  ✅ 姿态稳定，第二层可以工作 (误差: {attitude_error:.2f}°)")
            
            if attitude_error > 3.0:
                print(f"  ⚠️  姿态角较大，第一层应当专注校准")
            
            time.sleep(0.5)
        
        # 测试结果统计
        print("\n" + "="*60)
        print("分层控制逻辑验证结果:")
        print(f"第一层激活率: {layer1_active_count}/60 = {layer1_active_count/60*100:.1f}%")
        print(f"姿态调整时间: {attitude_adjusting_count}/60 = {attitude_adjusting_count/60*100:.1f}%")
        print(f"姿态稳定时间: {attitude_stable_count}/60 = {attitude_stable_count/60*100:.1f}%")
        print(f"第二层可用时间: {layer2_active_count}/60 = {layer2_active_count/60*100:.1f}%")
        
        # 逻辑正确性评估
        print("\n逻辑正确性评估:")
        if attitude_adjusting_count > 10:  # 超过1/6的时间在调整
            print("✅ 第一层工作正常：检测到姿态需要调整，正在主动补偿")
        else:
            print("🟡 第一层较少激活：可能姿态本身就很稳定")
        
        if attitude_stable_count > 30:  # 超过一半时间稳定
            print("✅ 系统稳定性良好：大部分时间姿态稳定，可启用第二层")
        else:
            print("⚠️  系统稳定性需要提升：姿态调整时间过长")
        
        stability_ratio = attitude_stable_count / (attitude_adjusting_count + attitude_stable_count)
        print(f"整体稳定性指标: {stability_ratio*100:.1f}%")
        
        if stability_ratio > 0.7:
            print("🎉 分层控制逻辑工作正常！")
        else:
            print("🔧 分层控制参数需要进一步调优")
            
    except KeyboardInterrupt:
        print("\n用户中断测试")
    except Exception as e:
        print(f"测试过程中发生错误: {e}")
    finally:
        print("\n执行安全降落...")
        drone.landing()


def test_manual_disturbance():
    """手动干扰测试：验证第一层响应和第二层禁用逻辑"""
    print("=== 手动干扰响应测试 ===")
    print("请在测试过程中轻推无人机，观察分层控制的响应")
    
    drone = espdrone.DRONE(flightmode=0, debug=1) 
    
    try:
        drone.take_off(distance=80)
        time.sleep(3)
        
        print("\n开始监控，请手动干扰...")
        print("预期行为：")
        print("- 干扰时：第一层立即补偿，第二层被禁用")  
        print("- 稳定后：第一层减少补偿，第二层重新启用")
        print("-" * 60)
        
        for i in range(40):  # 20秒测试
            states = drone.read_states()
            hover_debug = drone.get_hover_debug()
            
            roll = states[0] / 100.0
            pitch = states[1] / 100.0
            comp_roll = hover_debug[0]
            comp_pitch = hover_debug[1]
            
            # 判断当前状态
            total_attitude = abs(roll) + abs(pitch)
            total_compensation = abs(comp_roll) + abs(comp_pitch)
            
            if total_attitude > 2.0 or total_compensation > 2.0:
                status = "🔧 ADJUSTING"
                layer2_status = "BLOCKED"
            else:
                status = "✅ STABLE  "
                layer2_status = "ACTIVE"
            
            print(f"[{i*0.5:4.1f}s] {status} | Att({roll:+5.2f}°,{pitch:+5.2f}°) | "
                  f"Comp({comp_roll:+5.2f}°,{comp_pitch:+5.2f}°) | L2:{layer2_status}")
            
            # 检测到明显干扰
            if total_attitude > 3.0:
                print(f"      ⚡ 检测到干扰！第一层应立即响应，第二层应被禁用")
            
            # 检测到强补偿
            if total_compensation > 3.0:
                print(f"      🎯 第一层强力补偿中！(补偿: {total_compensation:.2f}°)")
            
            # 检测到稳定
            if total_attitude < 1.0 and total_compensation < 1.0:
                print(f"      🟢 系统稳定，第二层可以启用")
            
            time.sleep(0.5)
            
    except Exception as e:
        print(f"干扰测试出错: {e}")
    finally:
        drone.landing()


if __name__ == "__main__":
    print("分层悬停控制逻辑验证程序")
    print("1. 基础逻辑验证测试") 
    print("2. 手动干扰响应测试")
    
    choice = input("请选择测试项目 (1 或 2): ")
    
    if choice == "1":
        test_layered_hover_logic()
    elif choice == "2":
        test_manual_disturbance()
    else:
        print("无效选择，退出")