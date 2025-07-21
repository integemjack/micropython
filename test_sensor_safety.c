/*
 * 测试传感器安全性的示例代码
 * 这个文件展示了传感器不存在时程序如何安全运行
 */

#include <stdio.h>
#include <stdbool.h>

// 模拟传感器状态
static bool flow_sensor_present = false;  // 模拟光流传感器不存在
static bool tof_sensor_present = false;   // 模拟TOF传感器不存在

// 模拟的传感器接口函数
bool opticalFlowIsPresent(void) { return flow_sensor_present; }
bool tofSensorIsPresent(void) { return tof_sensor_present; }

bool opticalFlowReadMeasurement(void* data) {
    if (!flow_sensor_present) return false;
    // 实际读取逻辑...
    return true;
}

bool tofSensorReadMeasurement(void* data) {
    if (!tof_sensor_present) return false;
    // 实际读取逻辑...
    return true;
}

// 模拟主循环中的传感器读取
void readOptionalSensors(void)
{
    bool flowAvailable = false;
    bool tofAvailable = false;
    
    printf("开始读取传感器数据...\n");
    
    // 安全读取光流传感器数据
    if (opticalFlowIsPresent()) {
        printf("  光流传感器存在，尝试读取...\n");
        flowAvailable = opticalFlowReadMeasurement(NULL);
        printf("  光流数据读取: %s\n", flowAvailable ? "成功" : "失败");
    } else {
        printf("  光流传感器不存在，跳过读取\n");
    }
    
    // 安全读取TOF传感器数据
    if (tofSensorIsPresent()) {
        printf("  TOF传感器存在，尝试读取...\n");
        tofAvailable = tofSensorReadMeasurement(NULL);
        printf("  TOF数据读取: %s\n", tofAvailable ? "成功" : "失败");
    } else {
        printf("  TOF传感器不存在，跳过读取\n");
    }
    
    printf("传感器读取完成。Flow=%s, TOF=%s\n\n", 
           flowAvailable ? "有效" : "无效",
           tofAvailable ? "有效" : "无效");
}

int main()
{
    printf("=== 传感器安全性测试 ===\n\n");
    
    printf("场景1: 两个传感器都不存在\n");
    flow_sensor_present = false;
    tof_sensor_present = false;
    readOptionalSensors();
    
    printf("场景2: 仅光流传感器存在\n");
    flow_sensor_present = true;
    tof_sensor_present = false;
    readOptionalSensors();
    
    printf("场景3: 仅TOF传感器存在\n");
    flow_sensor_present = false;
    tof_sensor_present = true;
    readOptionalSensors();
    
    printf("场景4: 两个传感器都存在\n");
    flow_sensor_present = true;
    tof_sensor_present = true;
    readOptionalSensors();
    
    printf("✓ 所有测试场景完成，程序安全运行！\n");
    return 0;
}
