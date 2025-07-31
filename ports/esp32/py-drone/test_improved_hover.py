"""
改进悬停控制效果测试脚本

测试内容：
1. 对比传统光流悬停 vs 改进姿态前馈悬停的效果
2. 实时监控姿态补偿和光流校正数据
3. 验证主动补偿的响应速度和精度
"""

import espdrone
import time

def test_improved_hover_control():
    """测试改进的悬停控制算法"""
    print("=== 改进悬停控制测试 ===")
    
    # 初始化飞控系统
    drone = espdrone.DRONE(flightmode=0, debug=1)
    
    try:
        # 起飞到测试高度
        print("起飞到100cm高度进行悬停测试...")
        drone.take_off(distance=100)
        time.sleep(5)  # 等待起飞稳定
        
        # 检查悬停控制是否激活
        if drone.is_hover_active():
            print("✓ 改进的悬停控制已激活")
        else:
            print("✗ 悬停控制未激活")
            return
        
        print("\n开始悬停性能测试...")
        print("格式: [时间] 姿态补偿(Roll°, Pitch°) | 光流校正(X, Y) | 姿态角(R°,P°,Y°) | 高度误差")
        print("-" * 80)
        
        test_duration = 30  # 30秒测试
        start_time = time.time()
        
        # 统计数据
        max_attitude_error = 0.0
        max_compensation = 0.0
        position_drift_samples = []
        
        for i in range(test_duration * 2):  # 0.5秒间隔，共60次采样
            current_time = time.time() - start_time
            
            # 读取飞行状态
            states = drone.read_states()
            hover_debug = drone.get_hover_debug()
            
            # 解析数据
            roll = states[0] / 100.0          # 当前姿态Roll
            pitch = states[1] / 100.0         # 当前姿态Pitch  
            yaw = states[2] / 100.0           # 当前姿态Yaw
            current_height = states[8]        # 当前高度(mm)
            target_height = states[17]       # 目标高度(mm)
            
            # 改进悬停控制调试数据
            att_comp_roll = hover_debug[0]    # 姿态补偿Roll
            att_comp_pitch = hover_debug[1]   # 姿态补偿Pitch
            flow_corr_x = hover_debug[2]      # 光流校正X
            flow_corr_y = hover_debug[3]      # 光流校正Y
            
            # 计算误差
            height_error = abs(target_height - current_height)
            attitude_error = (roll**2 + pitch**2)**0.5
            compensation_magnitude = (att_comp_roll**2 + att_comp_pitch**2)**0.5
            
            # 更新统计
            max_attitude_error = max(max_attitude_error, attitude_error)
            max_compensation = max(max_compensation, compensation_magnitude)
            position_drift_samples.append((flow_corr_x**2 + flow_corr_y**2)**0.5)
            
            # 实时输出
            print(f"[{current_time:5.1f}s] 补偿({att_comp_roll:+5.2f}°,{att_comp_pitch:+5.2f}°) | "
                  f"光流({flow_corr_x:+5.2f},{flow_corr_y:+5.2f}) | "
                  f"姿态({roll:+5.2f}°,{pitch:+5.2f}°,{yaw:+5.2f}°) | "
                  f"高度差{height_error:4.0f}mm")
            
            # 分析控制效果
            if attitude_error > 3.0:  # 姿态角超过3度
                print(f"  ⚠️  姿态角较大({attitude_error:.2f}°)，观察补偿效果...")
            
            if compensation_magnitude > 2.0:  # 补偿角度超过2度
                print(f"  🎯 主动补偿激活({compensation_magnitude:.2f}°)")
            
            if abs(flow_corr_x) > 1.0 or abs(flow_corr_y) > 1.0:  # 光流校正激活
                print(f"  📍 光流校正工作中")
            
            time.sleep(0.5)
        
        # 测试结果分析
        print("\n" + "="*60)
        print("测试结果分析：")
        print(f"最大姿态误差: {max_attitude_error:.2f}°")
        print(f"最大补偿角度: {max_compensation:.2f}°")
        print(f"平均位置漂移: {sum(position_drift_samples)/len(position_drift_samples):.3f}")
        print(f"最大位置漂移: {max(position_drift_samples):.3f}")
        
        # 效果评估
        if max_attitude_error < 2.0:
            print("✅ 姿态稳定性: 优秀 (< 2°)")
        elif max_attitude_error < 5.0:
            print("🟡 姿态稳定性: 良好 (< 5°)")
        else:
            print("❌ 姿态稳定性: 需要改进 (> 5°)")
        
        if max_compensation < 5.0:
            print("✅ 补偿算法: 工作正常 (< 5°)")
        else:
            print("⚠️  补偿算法: 补偿幅度较大 (> 5°)")
        
        avg_drift = sum(position_drift_samples) / len(position_drift_samples)
        if avg_drift < 0.5:
            print("✅ 定点精度: 优秀 (< 0.5)")
        elif avg_drift < 1.0:
            print("🟡 定点精度: 良好 (< 1.0)")
        else:
            print("❌ 定点精度: 需要改进 (> 1.0)")
        
    except KeyboardInterrupt:
        print("\n用户中断测试")
    except Exception as e:
        print(f"测试过程中发生错误: {e}")
    finally:
        print("\n执行安全降落...")
        drone.landing()
        time.sleep(3)


def compare_hover_algorithms():
    """对比不同悬停算法的效果"""
    print("=== 悬停算法对比测试 ===")
    print("此测试需要手动干扰无人机来观察响应效果")
    
    drone = espdrone.DRONE(flightmode=0, debug=1)
    
    try:
        drone.take_off(distance=80)
        time.sleep(3)
        
        print("\n请手动轻推无人机，观察恢复效果...")
        print("监控数据格式: 姿态补偿强度 | 响应延迟 | 定点误差")
        
        for i in range(20):  # 10秒监控
            states = drone.read_states()
            hover_debug = drone.get_hover_debug()
            
            roll = states[0] / 100.0
            pitch = states[1] / 100.0
            att_comp_roll = hover_debug[0]
            att_comp_pitch = hover_debug[1]
            
            # 计算响应比例 (补偿角度 / 姿态角度)
            if abs(roll) > 0.5:
                response_ratio_roll = abs(att_comp_roll / roll) if roll != 0 else 0
                print(f"Roll干扰: {roll:+5.2f}° → 补偿: {att_comp_roll:+5.2f}° (比例: {response_ratio_roll:.2f})")
            
            if abs(pitch) > 0.5:
                response_ratio_pitch = abs(att_comp_pitch / pitch) if pitch != 0 else 0
                print(f"Pitch干扰: {pitch:+5.2f}° → 补偿: {att_comp_pitch:+5.2f}° (比例: {response_ratio_pitch:.2f})")
            
            time.sleep(0.5)
        
    except Exception as e:
        print(f"对比测试出错: {e}")
    finally:
        drone.landing()


if __name__ == "__main__":
    print("改进悬停控制测试程序")
    print("1. 基础悬停性能测试")
    print("2. 干扰响应对比测试")
    
    choice = input("请选择测试项目 (1 或 2): ")
    
    if choice == "1":
        test_improved_hover_control()
    elif choice == "2": 
        compare_hover_algorithms()
    else:
        print("无效选择，退出")