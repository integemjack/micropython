/**
 * hover_control_improved.c - 第一层控制：姿态校准系统
 * 
 * 职责：确保推力向量垂直，为光流定点创造前提条件
 * 核心思路：
 * 1. 专注姿态稳定：仅负责Roll/Pitch角度的主动补偿
 * 2. 快速响应：检测到姿态偏差立即补偿，无需等待位移
 * 3. 垂直推力保证：为第二层光流控制提供稳定的物理基础
 * 4. 状态输出：向上层报告姿态是否稳定，决定是否启用光流控制
 */

#include "hover_control_improved.h"
#include "pid.h"               // PID控制器
#include "maths.h"
#include "commander.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.h"
#include "sensors_mpu6050_spl06.h"

// 添加缺失的数学函数
#ifndef constrainf
#define constrainf(value, min, max) ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))
#endif

#ifndef fabsf
#define fabsf(x) (((x) < 0) ? -(x) : (x))
#endif

static const char* TAG = "hover_improved";

// 第一层控制：姿态校准参数
#define ATTITUDE_COMPENSATION_GAIN  0.85f   // 姿态补偿增益（主要参数）
#define ATTITUDE_DEADBAND          0.3f     // 姿态角死区（度）
#define MAX_ATTITUDE_COMPENSATION  8.0f     // 最大姿态补偿角度（度）
#define ATTITUDE_STABLE_THRESHOLD  1.5f     // 姿态稳定阈值（度）
#define ATTITUDE_FILTER_ALPHA      0.3f     // 姿态角滤波系数

// 姿态校准控制状态
typedef struct {
    bool enabled;                   // 姿态校准是否启用
    bool attitudeStable;           // 姿态是否稳定（关键状态）
    
    // 当前姿态补偿值
    float compensationRoll;        // Roll轴补偿角度
    float compensationPitch;       // Pitch轴补偿角度
    
    // 姿态滤波值
    float filteredRoll;            // 滤波后的Roll角
    float filteredPitch;           // 滤波后的Pitch角
    
    // 稳定性监控
    uint32_t stableCounter;        // 稳定计数器
    uint32_t unstableCounter;      // 不稳定计数器
    uint32_t lastUpdateTime;       // 上次更新时间
    
} attitudeControl_t;

static attitudeControl_t attitudeState = {
    .enabled = false,
    .attitudeStable = false,
    .compensationRoll = 0.0f,
    .compensationPitch = 0.0f,
    .filteredRoll = 0.0f,
    .filteredPitch = 0.0f,
    .stableCounter = 0,
    .unstableCounter = 0,
    .lastUpdateTime = 0
};

// PID控制器（保持原有结构）
static PidObject hoverPidX, hoverPidY, hoverPidZ;
static bool isPidInit = false;

void improvedHoverControlInit(void)
{
    // 初始化姿态校准状态
    attitudeState.enabled = false;
    attitudeState.attitudeStable = false;
    attitudeState.compensationRoll = 0.0f;
    attitudeState.compensationPitch = 0.0f;
    attitudeState.filteredRoll = 0.0f;
    attitudeState.filteredPitch = 0.0f;
    attitudeState.stableCounter = 0;
    attitudeState.unstableCounter = 0;
    attitudeState.lastUpdateTime = xTaskGetTickCount();
    
    ESP_LOGI(TAG, "Attitude stabilization system initialized (Layer 1 Control)");
}

/**
 * 第一层控制：姿态稳定算法
 * 
 * 职责：确保推力向量垂直，为第二层光流控制创造前提
 * 核心：主动补偿姿态角，防止水平分量产生
 */
bool performAttitudeStabilization(attitude_t* attitude, setpoint_t* setpoint, float dt)
{
    if (!attitudeState.enabled) {
        return false;
    }
    
    // 1. 姿态角滤波（减少噪声）
    attitudeState.filteredRoll = attitudeState.filteredRoll * (1.0f - ATTITUDE_FILTER_ALPHA) + 
                                attitude->roll * ATTITUDE_FILTER_ALPHA;
    attitudeState.filteredPitch = attitudeState.filteredPitch * (1.0f - ATTITUDE_FILTER_ALPHA) + 
                                 attitude->pitch * ATTITUDE_FILTER_ALPHA;
    
    // 2. 计算姿态补偿（核心算法）
    if (fabsf(attitudeState.filteredRoll) > ATTITUDE_DEADBAND) {
        // Roll轴补偿：产生反向角度抵消水平分量
        attitudeState.compensationRoll = -attitudeState.filteredRoll * ATTITUDE_COMPENSATION_GAIN;
    } else {
        // 在死区内逐渐减小补偿
        attitudeState.compensationRoll *= 0.9f;
    }
    
    if (fabsf(attitudeState.filteredPitch) > ATTITUDE_DEADBAND) {
        // Pitch轴补偿：产生反向角度抵消水平分量
        attitudeState.compensationPitch = -attitudeState.filteredPitch * ATTITUDE_COMPENSATION_GAIN;
    } else {
        // 在死区内逐渐减小补偿
        attitudeState.compensationPitch *= 0.9f;
    }
    
    // 3. 限制补偿幅度，防止过度补偿
    attitudeState.compensationRoll = constrainf(attitudeState.compensationRoll, 
                                              -MAX_ATTITUDE_COMPENSATION, 
                                              MAX_ATTITUDE_COMPENSATION);
    attitudeState.compensationPitch = constrainf(attitudeState.compensationPitch, 
                                               -MAX_ATTITUDE_COMPENSATION, 
                                               MAX_ATTITUDE_COMPENSATION);
    
    // 4. 应用姿态补偿到控制输出
    setpoint->attitude.roll += attitudeState.compensationRoll;
    setpoint->attitude.pitch += attitudeState.compensationPitch;
    
    // 5. 评估姿态稳定性（关键判断）
    float totalAttitudeError = fabsf(attitudeState.filteredRoll) + fabsf(attitudeState.filteredPitch);
    float totalCompensation = fabsf(attitudeState.compensationRoll) + fabsf(attitudeState.compensationPitch);
    
    // 判断逻辑：同时满足姿态角小和补偿量小，才认为稳定
    bool currentlyStable = (totalAttitudeError < ATTITUDE_STABLE_THRESHOLD) && 
                          (totalCompensation < 1.0f);  // 补偿量小于1度
    
    if (currentlyStable) {
        attitudeState.stableCounter++;
        attitudeState.unstableCounter = 0;
        
        // 连续稳定15个周期才认为姿态稳定（更严格）
        if (attitudeState.stableCounter >= 15) {
            attitudeState.attitudeStable = true;
        }
    } else {
        attitudeState.unstableCounter++;
        attitudeState.stableCounter = 0;
        
        // 一旦检测到需要调整，立即禁用第二层
        attitudeState.attitudeStable = false;
    }
    
    // 6. 调试输出
    static uint32_t debugCounter = 0;
    if (++debugCounter % 250 == 0) { // 每0.5秒输出一次
        ESP_LOGD(TAG, "Layer1: Att(R=%.2f°,P=%.2f°) Comp(R=%.2f°,P=%.2f°) Status=%s", 
                 attitudeState.filteredRoll, attitudeState.filteredPitch,
                 attitudeState.compensationRoll, attitudeState.compensationPitch,
                 attitudeState.attitudeStable ? "STABLE-OK_FOR_LAYER2" : "ADJUSTING-BLOCK_LAYER2");
    }
    
    // 返回姿态是否稳定（决定是否启用第二层控制）
    return attitudeState.attitudeStable;
}

void improvedHoverControlEnable(bool enable) 
{
    attitudeState.enabled = enable;
    if (enable) {
        // 重置姿态校准状态
        attitudeState.attitudeStable = false;
        attitudeState.compensationRoll = 0.0f;
        attitudeState.compensationPitch = 0.0f;
        attitudeState.filteredRoll = 0.0f;
        attitudeState.filteredPitch = 0.0f;
        attitudeState.stableCounter = 0;
        attitudeState.unstableCounter = 0;
        ESP_LOGI(TAG, "Layer 1 attitude stabilization enabled");
    } else {
        ESP_LOGI(TAG, "Layer 1 attitude stabilization disabled");
    }
}

bool improvedHoverControlIsActive(void)
{
    return attitudeState.enabled;
}

// 获取姿态是否稳定（关键接口，供第二层控制使用）
bool isAttitudeStabilized(void)
{
    return attitudeState.enabled && attitudeState.attitudeStable;
}

/**
 * 第一层控制主函数：专门负责姿态稳定
 * 
 * 返回值：true=姿态稳定，可以启用第二层光流控制
 *         false=姿态不稳定，不应启用光流控制
 */
bool improvedHoverControlUpdate(flowMeasurement_t* flow, tofMeasurement_t* tof, 
                               setpoint_t* setpoint, state_t* state, float dt, float height)
{
    // 执行第一层姿态稳定控制
    bool attitudeStable = performAttitudeStabilization(&state->attitude, setpoint, dt);
    
    // 设置控制模式为姿态角控制（第一层只管姿态）
    setpoint->mode.roll = modeAbs;   // 绝对姿态角控制
    setpoint->mode.pitch = modeAbs;  // 绝对姿态角控制
    setpoint->mode.yaw = modeAbs;    // 保持航向锁定
    
    // 更新时间戳
    attitudeState.lastUpdateTime = xTaskGetTickCount();
    
    // 返回姿态稳定状态，供上层判断是否启用第二层控制
    return attitudeStable;
}

void improvedHoverControlSetTarget(float x, float y, float height)
{
    // 第一层控制不需要位置目标，仅做姿态稳定
    // 此函数保留接口兼容性，但在第一层控制中不使用
    ESP_LOGD(TAG, "Layer 1 control: Target setting ignored (attitude stabilization only)");
}

/**
 * 调试函数：获取第一层控制参数
 */
void getHoverControlDebugInfo(float* attComp, float* flowCorr)
{
    attComp[0] = attitudeState.compensationRoll;    // 姿态补偿Roll
    attComp[1] = attitudeState.compensationPitch;   // 姿态补偿Pitch
    flowCorr[0] = attitudeState.filteredRoll;       // 滤波后姿态Roll（复用数组）
    flowCorr[1] = attitudeState.filteredPitch;      // 滤波后姿态Pitch（复用数组）
}