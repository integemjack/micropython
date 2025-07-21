# PyDrone 光流传感器和TOF传感器配置指南

## 概述
本文档说明如何配置 PyDrone 默认启动光流传感器（Optical Flow）和TOF传感器。

## 新增文件
- `utils/interface/optical_flow.h` - 光流传感器接口
- `utils/interface/vl53lxx.h` - TOF传感器接口  
- `utils/interface/sensor_config.h` - 传感器配置选项
- `utils/src/optical_flow.c` - 光流传感器实现
- `utils/src/vl53lxx.c` - TOF传感器实现

## 修改的文件
- `utils/src/stabilizer.c` - 添加传感器初始化和数据读取
- `port/system_int.c` - 在系统初始化中启用传感器

## 配置选项
在 `sensor_config.h` 中可以控制传感器的启用/禁用：

```c
#define OPTICAL_FLOW_SENSOR_ENABLED    1  // 1=启用，0=禁用
#define TOF_SENSOR_ENABLED             1  // 1=启用，0=禁用
```

## 功能说明
1. **自动检测**: 系统启动时会尝试初始化传感器
2. **默认启用**: 如果传感器存在，将自动启用并开始读取数据
3. **集成到主循环**: 传感器数据会在500Hz频率下被读取
4. **位置估计**: TOF数据会被传递给位置估计器用于高度控制

## 实际硬件集成
当前实现提供了基础框架。要集成实际硬件：
1. 在 `optical_flow.c` 中添加具体传感器的I2C通信代码
2. 在 `vl53lxx.c` 中添加VL53L0X/VL53L1X的驱动代码
3. 根据实际传感器调整I2C地址和通信协议

## 日志输出
系统启动时会显示：
- "Optical flow sensor enabled" - 如果光流传感器成功初始化
- "TOF sensor enabled" - 如果TOF传感器成功初始化

## 调试
可以通过修改 `sensor_config.h` 中的宏定义来禁用特定传感器进行调试。
