# PyDrone read_states() API 更新说明

## 📋 数据格式变更

`drone.read_states()` 函数已从13个元素扩展到**18个元素**，新增了4个电机功率数据和1个目标高度数据。

## 🔧 数据结构 (18个元素)

### 原有数据 (索引 0-12)
| 索引 | 数据内容 | 说明 | 单位/精度 |
|------|----------|------|-----------|
| 0 | attitude.roll | 姿态角Roll | 放大100倍 |
| 1 | attitude.pitch | 姿态角Pitch | 放大100倍 |
| 2 | attitude.yaw | 姿态角Yaw | 放大100倍 |
| 3 | wifiCtrl.roll | 控制输入Roll | 放大100倍 |
| 4 | wifiCtrl.pitch | 控制输入Pitch | 放大100倍 |
| 5 | wifiCtrl.yaw | 控制输入Yaw | 放大100倍 |
| 6 | thrust | 推力输入值 | 0-100 |
| 7 | battery | 电池电压 | 放大100倍 |
| 8 | fusedHeight | 融合高度值 | mm |
| 9 | flow.dpixelx | 光流X方向像素变化 | 放大100倍 |
| 10 | flow.dpixely | 光流Y方向像素变化 | 放大100倍 |
| 11 | tof.distance | TOF距离 | mm |
| 12 | sensor_status | 传感器状态标志 | bit0:光流, bit1:TOF |

### 新增电机功率数据 (索引 13-16)
| 索引 | 数据内容 | 说明 | 范围 |
|------|----------|------|-------|
| **13** | **motorPower.m1** | **M1电机功率** | **0-65535** |
| **14** | **motorPower.m2** | **M2电机功率** | **0-65535** |
| **15** | **motorPower.m3** | **M3电机功率** | **0-65535** |
| **16** | **motorPower.m4** | **M4电机功率** | **0-65535** |

### 新增目标高度数据 (索引 17)
| 索引 | 数据内容 | 说明 | 单位/精度 |
|------|----------|------|-----------|
| **17** | **setHeight** | **目标设定高度** | **mm** |

## 💻 使用示例

### Python代码示例
```python
import espdrone

# 创建drone实例
drone = espdrone.DRONE()

# 读取完整状态 (18个元素)
states = drone.read_states()

# 解析数据
attitude_roll = states[0] / 100.0    # 姿态角Roll (度)
attitude_pitch = states[1] / 100.0   # 姿态角Pitch (度)  
attitude_yaw = states[2] / 100.0     # 姿态角Yaw (度)

thrust_input = states[6]             # 推力输入 (0-100)
battery_voltage = states[7] / 100.0  # 电池电压 (V)
height = states[8]                   # 高度 (mm)

# 新增：电机功率数据
motor_m1_power = states[13]          # M1电机功率 (0-65535)
motor_m2_power = states[14]          # M2电机功率 (0-65535)
motor_m3_power = states[15]          # M3电机功率 (0-65535)  
motor_m4_power = states[16]          # M4电机功率 (0-65535)

# 新增：目标设定高度 (统一为mm单位)
target_height_mm = states[17]        # 目标高度 (mm)
current_height_mm = states[11]       # 当前TOF高度 (mm)

print(f"电机功率: M1={motor_m1_power}, M2={motor_m2_power}, M3={motor_m3_power}, M4={motor_m4_power}")
print(f"目标高度: {target_height_mm}mm (设定值)")
print(f"当前高度: {current_height_mm}mm (TOF测量值)")

# 高度误差分析
height_error = abs(target_height_mm - current_height_mm)
print(f"高度误差: {height_error}mm")

# 专用电机功率读取函数 (4个元素)
motor_powers = drone.read_motor_power()
print(f"电机功率: M1={motor_powers[0]}, M2={motor_powers[1]}, M3={motor_powers[2]}, M4={motor_powers[3]}")
```

### 诊断M1、M3停转问题
```python
import espdrone
import time

drone = espdrone.DRONE()

# 监控电机功率变化
for i in range(10):
    powers = drone.read_motor_power()
    states = drone.read_states()
    
    thrust_input = states[6]
    target_height_mm = states[17]      # 目标高度 (mm)
    current_height_mm = states[11]     # 当前TOF高度 (mm)
    
    print(f"循环{i+1}: 推力={thrust_input}, 目标={target_height_mm}mm, 当前={current_height_mm}mm")
    print(f"        电机: M1={powers[0]}, M2={powers[1]}, M3={powers[2]}, M4={powers[3]}")
    
    # 检查M1和M3是否停转
    if powers[0] == 0 and powers[2] == 0 and thrust_input > 10:
        print("⚠️  警告: M1和M3停转，但推力输入正常!")
    
    # 检查高度控制异常 (单位统一为mm)
    height_error = abs(target_height_mm - current_height_mm)
    if height_error > 100:  # 高度误差超过10cm
        print(f"⚠️  警告: 高度误差过大 {height_error}mm!")
    
    time.sleep(0.5)
```

## 🔍 故障排查用途

新增的电机功率数据特别适用于：

1. **实时监控四个电机的功率分配**
2. **诊断特定电机停转问题**  
3. **验证混控算法是否正确工作**
4. **检查电机功率是否与输入推力匹配**
5. **分析电机功率的动态变化**

## ⚡ 性能说明

- `read_states()`: 返回18个元素的完整状态数据
- `read_motor_power()`: 仅返回4个电机功率数据，调用开销更小
- 两个函数都是实时读取，无缓存延迟

## 🎯 高度控制诊断

新增的**目标设定高度(setHeight)**特别有助于：

1. **对比目标高度vs实际高度** - 判断高度控制器是否正常工作
2. **检测高度控制异常** - 目标高度异常可能导致电机功率问题  
3. **验证定高模式设置** - 确认目标高度是否符合预期
4. **分析着陆逻辑** - 监控目标高度变化，判断是否触发自动着陆

这个更新将大大帮助诊断和解决M1、M3电机停转的问题，特别是与高度控制相关的故障。