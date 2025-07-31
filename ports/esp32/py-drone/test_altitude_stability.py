"""
高度抖动优化测试脚本

测试优化后的姿态校准系统是否解决了高度抖动和翻滚炸机问题

优化措施：
1. 降低补偿增益（0.85 → 0.35）
2. 增大死区（0.3° → 0.8°）
3. 大幅降低最大补偿（8° → 3°）
4. 高度自适应增益（高空自动降低）
5. 振荡检测和抑制
6. 多级滤波平滑
"""

import espdrone
import time

def test_altitude_stability():
    """测试不同高度下的系统稳定性"""
    print("=== 高度稳定性测试 ===")
    print("测试目标：验证随高度增加，系统是否保持稳定")
    
    drone = espdrone.DRONE(flightmode=0, debug=1)
    
    # 测试高度序列（从低到高）
    test_heights = [30, 50, 80, 120, 150, 200]  # cm
    
    try:
        for height in test_heights:
            print(f"\n--- 测试高度: {height}cm ---")
            
            # 起飞到指定高度
            drone.take_off(distance=height)
            time.sleep(5)  # 等待稳定
            
            # 监控该高度下的稳定性
            stable_count = 0
            oscillation_count = 0
            max_attitude_error = 0.0
            max_compensation = 0.0
            
            print(f"高度{height}cm监控（10秒）:")
            print("格式: [时间] 高度增益 | 姿态角 | 补偿角 | 振荡 | 状态")
            
            for i in range(20):  # 10秒监控
                states = drone.read_states()
                hover_debug = drone.get_hover_debug()
                
                # 解析数据
                roll = states[0] / 100.0
                pitch = states[1] / 100.0
                current_height = states[8]  # mm
                comp_roll = hover_debug[0]
                comp_pitch = hover_debug[1]
                
                # 计算关键指标
                attitude_error = (roll**2 + pitch**2)**0.5
                compensation_magnitude = (comp_roll**2 + comp_pitch**2)**0.5
                
                # 估算高度增益（基于优化算法）
                if current_height/10 < 50:  # mm to cm
                    height_gain = 1.0
                elif current_height/10 > 200:
                    height_gain = 0.1
                else:
                    height_ratio = (current_height/10 - 50) / (200 - 50)
                    height_gain = 1.0 - height_ratio * 0.9
                
                # 稳定性判断
                is_stable = (attitude_error < 2.5) and (compensation_magnitude < 1.5)
                is_oscillating = compensation_magnitude > 2.0  # 简单的振荡判断
                
                if is_stable:
                    stable_count += 1
                if is_oscillating:
                    oscillation_count += 1
                
                max_attitude_error = max(max_attitude_error, attitude_error)
                max_compensation = max(max_compensation, compensation_magnitude)
                
                # 实时输出
                status = "STABLE" if is_stable else "UNSTABLE"
                osc_indicator = "OSC" if is_oscillating else "OK"
                
                print(f"[{i*0.5:4.1f}s] 增益{height_gain:.2f} | "
                      f"姿态({roll:+5.2f}°,{pitch:+5.2f}°) | "
                      f"补偿({comp_roll:+5.2f}°,{comp_pitch:+5.2f}°) | "
                      f"{osc_indicator} | {status}")
                
                # 预警检测
                if attitude_error > 5.0:
                    print(f"  ⚠️  姿态角过大: {attitude_error:.2f}°，可能不稳定")
                
                if compensation_magnitude > 3.0:
                    print(f"  🔥 补偿过大: {compensation_magnitude:.2f}°，检查振荡")
                
                if current_height/10 < height * 0.8 or current_height/10 > height * 1.2:
                    print(f"  📏 高度偏差: 目标{height}cm，实际{current_height/10:.1f}cm")
                
                time.sleep(0.5)
            
            # 该高度测试结果
            stability_ratio = stable_count / 20
            oscillation_ratio = oscillation_count / 20
            
            print(f"\n高度{height}cm测试结果:")
            print(f"  稳定率: {stability_ratio*100:.1f}% ({stable_count}/20)")
            print(f"  振荡率: {oscillation_ratio*100:.1f}% ({oscillation_count}/20)")
            print(f"  最大姿态误差: {max_attitude_error:.2f}°")
            print(f"  最大补偿幅度: {max_compensation:.2f}°")
            print(f"  估算增益: {height_gain:.2f}")
            
            # 稳定性评估
            if stability_ratio > 0.8 and max_attitude_error < 3.0:
                print(f"  ✅ 高度{height}cm: 稳定性良好")
            elif stability_ratio > 0.6:
                print(f"  🟡 高度{height}cm: 稳定性一般，需要调优")
            else:
                print(f"  ❌ 高度{height}cm: 不稳定，存在炸机风险")
                
            if oscillation_ratio > 0.3:
                print(f"  ⚠️  高度{height}cm: 振荡严重，需要进一步降低增益")
            
            # 短暂降落准备下一次测试
            drone.landing()
            time.sleep(2)
        
        # 整体测试总结
        print("\n" + "="*60)
        print("高度稳定性测试总结:")
        print("预期效果：随高度增加，补偿增益自动降低，系统保持稳定")
        print("关键指标：振荡率<30%，稳定率>60%，最大姿态误差<5°")
        
    except KeyboardInterrupt:
        print("\n用户中断测试")
    except Exception as e:
        print(f"测试过程中发生错误: {e}")
    finally:
        print("执行安全降落...")
        drone.stop()


def test_oscillation_detection():
    """专门测试振荡检测功能"""
    print("=== 振荡检测测试 ===")
    print("通过手动干扰观察振荡检测和抑制效果")
    
    drone = espdrone.DRONE(flightmode=0, debug=1)
    
    try:
        # 起飞到中等高度
        drone.take_off(distance=100)
        time.sleep(3)
        
        print("\n请手动轻推无人机，观察振荡检测效果...")
        print("预期：检测到振荡时，系统会自动降低增益")
        print("格式: [时间] 补偿变化 | 振荡状态 | 自适应增益")
        
        last_comp_roll = 0.0
        last_comp_pitch = 0.0
        
        for i in range(40):  # 20秒测试
            hover_debug = drone.get_hover_debug()
            states = drone.read_states()
            
            comp_roll = hover_debug[0]
            comp_pitch = hover_debug[1]
            height = states[8] / 10  # mm to cm
            
            # 计算补偿变化
            roll_change = comp_roll - last_comp_roll
            pitch_change = comp_pitch - last_comp_pitch
            
            # 简单的振荡检测逻辑（模拟系统内部逻辑）
            roll_oscillating = (roll_change * last_comp_roll < 0) and (abs(roll_change) > 0.5)
            pitch_oscillating = (pitch_change * last_comp_pitch < 0) and (abs(pitch_change) > 0.5)
            
            is_oscillating = roll_oscillating or pitch_oscillating
            
            # 估算自适应增益
            base_gain = 0.35
            if height < 50:
                height_gain = 1.0
            elif height > 200:
                height_gain = 0.1
            else:
                height_ratio = (height - 50) / (200 - 50)
                height_gain = 1.0 - height_ratio * 0.9
            
            adaptive_gain = base_gain * height_gain
            if is_oscillating:
                adaptive_gain *= 0.5  # 振荡时减半
            
            # 输出状态
            osc_status = "DETECTED" if is_oscillating else "NORMAL"
            print(f"[{i*0.5:4.1f}s] 变化({roll_change:+5.2f},{pitch_change:+5.2f}) | "
                  f"{osc_status:8s} | 增益{adaptive_gain:.3f}")
            
            if is_oscillating:
                print(f"      🎯 振荡检测触发，增益降低至{adaptive_gain:.3f}")
            
            last_comp_roll = comp_roll
            last_comp_pitch = comp_pitch
            
            time.sleep(0.5)
            
    except Exception as e:
        print(f"振荡检测测试出错: {e}")
    finally:
        drone.landing()


def test_conservative_parameters():
    """测试保守参数的效果"""
    print("=== 保守参数效果测试 ===")
    print("验证降低的参数是否有效防止过度补偿")
    
    drone = espdrone.DRONE(flightmode=0, debug=1)
    
    try:
        drone.take_off(distance=80)
        time.sleep(3)
        
        print("\n参数对比:")
        print("原参数: 增益0.85, 死区0.3°, 最大补偿8°")
        print("新参数: 增益0.35, 死区0.8°, 最大补偿3°")
        print("\n监控补偿幅度，验证是否在安全范围内...")
        
        max_compensation_seen = 0.0
        over_limit_count = 0
        
        for i in range(30):  # 15秒测试
            hover_debug = drone.get_hover_debug()
            states = drone.read_states()
            
            comp_roll = hover_debug[0]
            comp_pitch = hover_debug[1]
            roll = states[0] / 100.0
            pitch = states[1] / 100.0
            
            compensation_magnitude = (comp_roll**2 + comp_pitch**2)**0.5
            attitude_magnitude = (roll**2 + pitch**2)**0.5
            
            max_compensation_seen = max(max_compensation_seen, compensation_magnitude)
            
            if compensation_magnitude > 3.0:  # 新的限制
                over_limit_count += 1
                print(f"[{i*0.5:4.1f}s] ⚠️  补偿超限: {compensation_magnitude:.2f}° > 3.0°")
            
            print(f"[{i*0.5:4.1f}s] 姿态{attitude_magnitude:.2f}° → 补偿{compensation_magnitude:.2f}°")
            
            time.sleep(0.5)
        
        print(f"\n保守参数测试结果:")
        print(f"最大补偿幅度: {max_compensation_seen:.2f}°")
        print(f"超限次数: {over_limit_count}/30")
        
        if max_compensation_seen <= 3.5:
            print("✅ 补偿幅度在安全范围内")
        else:
            print("⚠️  补偿幅度仍然过大，需要进一步调整")
            
        if over_limit_count < 3:
            print("✅ 很少出现超限，参数设置合理")
        else:
            print("⚠️  经常超限，需要进一步降低增益")
            
    except Exception as e:
        print(f"保守参数测试出错: {e}")
    finally:
        drone.landing()


if __name__ == "__main__":
    print("高度抖动优化测试程序")
    print("1. 高度稳定性测试")
    print("2. 振荡检测测试") 
    print("3. 保守参数效果测试")
    
    choice = input("请选择测试项目 (1, 2, 或 3): ")
    
    if choice == "1":
        test_altitude_stability()
    elif choice == "2":
        test_oscillation_detection()
    elif choice == "3":
        test_conservative_parameters()
    else:
        print("无效选择，退出")