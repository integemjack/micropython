/**
 * sensor_config.h - Sensor configuration options
 */

#ifndef SENSOR_CONFIG_H_
#define SENSOR_CONFIG_H_

// ========== 传感器启用/禁用配置 ==========

// Enable/disable optical flow sensor
#define OPTICAL_FLOW_SENSOR_ENABLED    1

// Enable/disable TOF sensor  
#define TOF_SENSOR_ENABLED             1

// ========== 安全配置选项 ==========

// 如果设置为1，传感器检测失败不会阻止系统启动
#define SENSOR_DETECTION_NON_BLOCKING  1

// 传感器I2C通信超时时间 (毫秒)
#define SENSOR_I2C_TIMEOUT_MS         100

// 传感器数据读取重试次数
#define SENSOR_READ_RETRY_COUNT       3

// ========== 光流传感器参数 ==========
#define OPTICAL_FLOW_UPDATE_RATE_HZ    250
#define OPTICAL_FLOW_I2C_ADDR          0x42

// PMW3901 特定配置
#define PMW3901_WHO_AM_I_REG          0x00
#define PMW3901_EXPECTED_ID           0x49

// ========== TOF传感器参数 ==========
#define TOF_SENSOR_UPDATE_RATE_HZ      50
#define TOF_SENSOR_I2C_ADDR           0x29
#define TOF_SENSOR_MAX_RANGE_MM       2000

// VL53L0X 特定配置
#define VL53L0X_WHO_AM_I_REG          0xC0
#define VL53L0X_EXPECTED_ID           0xEE

// VL53L1X 特定配置  
#define VL53L1X_WHO_AM_I_REG          0x010F
#define VL53L1X_EXPECTED_ID           0xEA

// ========== 调试选项 ==========

// 启用传感器检测详细日志
#define SENSOR_DEBUG_DETECTION        1

// 启用传感器数据读取日志
#define SENSOR_DEBUG_DATA_READ        0

#endif /* SENSOR_CONFIG_H_ */
