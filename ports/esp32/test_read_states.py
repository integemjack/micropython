#!/usr/bin/env python3
"""
测试扩展后的read_states()函数 - 使用缓存数据
新的数据结构包含13个元素：
[0-8] 原有数据 (保持兼容性)
[9-10] 光流数据 (来自stabilizer缓存)
[11] TOF数据 (来自stabilizer缓存)
[12] 传感器状态
"""

import time

def test_read_states():
    try:
        import drone as d
        
        print("=== 测试扩展后的read_states()函数 ===")
        print("数据格式:")
        print("索引 0-2:  姿态角 (roll, pitch, yaw) * 100")
        print("索引 3-5:  控制量 (roll, pitch, yaw) * 100") 
        print("索引 6:    推力 (0-100)")
        print("索引 7:    电池电压 * 100")
        print("索引 8:    融合高度 (cm)")
        print("索引 9-10: 光流数据 (dpixelx, dpixely) * 100 [缓存]")
        print("索引 11:   TOF距离 * 100 (0.01cm精度) [缓存]")
        print("索引 12:   传感器状态 (bit0=光流, bit1=TOF)")
        print("-" * 50)
        
        for i in range(10):
            data = d.read_states()
            
            if len(data) >= 13:
                # 解析数据
                roll = data[0] / 100.0
                pitch = data[1] / 100.0  
                yaw = data[2] / 100.0
                
                flow_x = data[9] / 100.0
                flow_y = data[10] / 100.0
                tof_distance = data[11] / 100.0  # 0.01cm -> cm
                
                sensor_status = data[12]
                flow_valid = bool(sensor_status & 0x01)
                tof_valid = bool(sensor_status & 0x02)
                
                print(f"#{i:2d} 姿态: R{roll:6.1f}° P{pitch:6.1f}° Y{yaw:6.1f}° | "
                      f"光流: X{flow_x:6.2f} Y{flow_y:6.2f} [{flow_valid}] | "
                      f"TOF: {tof_distance:6.2f}cm [{tof_valid}]")
            else:
                print(f"#{i:2d} 数据长度: {len(data)} (可能是旧版本)")
                
            time.sleep(0.5)
            
    except ImportError:
        print("错误: 无法导入drone模块，请在无人机设备上运行")
    except Exception as e:
        print(f"错误: {e}")

if __name__ == "__main__":
    test_read_states()