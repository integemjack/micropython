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

// 第一层控制：姿态校准参数（优化版本，防止高度抖动）
#define ATTITUDE_COMPENSATION_GAIN  1.0f    // 直接补偿到平衡点，确保推力垂直
#define ATTITUDE_DEADBAND          0.8f     // 增大死区，减少小角度噪声的影响
#define MAX_ATTITUDE_COMPENSATION  3.0f     // 大幅降低最大补偿，防止剧烈机动
#define ATTITUDE_STABLE_THRESHOLD  2.5f     // 放宽稳定阈值，减少频繁切换
#define ATTITUDE_FILTER_ALPHA      0.15f    // 降低滤波系数，增强平滑性

// 高度自适应参数
#define HEIGHT_GAIN_SCALE_FACTOR   0.1f     // 高度增益缩放因子
#define MIN_HEIGHT_FOR_SCALING     50.0f    // 开始缩放的最小高度(cm)
#define MAX_HEIGHT_FOR_SCALING     200.0f   // 最大缩放高度(cm)

// 姿态校准控制状态（优化版本）
typedef struct {
    bool enabled;                   // 姿态校准是否启用
    bool attitudeStable;           // 姿态是否稳定（关键状态）
    
    // 当前姿态补偿值
    float compensationRoll;        // Roll轴补偿角度
    float compensationPitch;       // Pitch轴补偿角度
    
    // 姿态滤波值（多级滤波）
    float filteredRoll;            // 一级滤波后的Roll角
    float filteredPitch;           // 一级滤波后的Pitch角
    float smoothedRoll;            // 二级平滑后的Roll角
    float smoothedPitch;           // 二级平滑后的Pitch角
    
    // 振荡检测
    float lastCompensationRoll;    // 上次补偿Roll
    float lastCompensationPitch;   // 上次补偿Pitch
    uint32_t oscillationCounter;   // 振荡计数器
    bool oscillationDetected;      // 振荡检测标志
    
    // 高度自适应
    float currentHeight;           // 当前高度
    float heightBasedGain;         // 基于高度的动态增益
    
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
    .smoothedRoll = 0.0f,
    .smoothedPitch = 0.0f,
    .lastCompensationRoll = 0.0f,
    .lastCompensationPitch = 0.0f,
    .oscillationCounter = 0,
    .oscillationDetected = false,
    .currentHeight = 80.0f,
    .heightBasedGain = 1.0f,
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
 * 高度自适应增益计算
 * 随着高度增加，降低补偿增益，防止高空抖动
 */
void updateHeightBasedGain(float height)
{
    attitudeState.currentHeight = height;
    
    if (height < MIN_HEIGHT_FOR_SCALING) {
        // 低空：使用全增益
        attitudeState.heightBasedGain = 1.0f;
    } else if (height > MAX_HEIGHT_FOR_SCALING) {
        // 高空：使用最小增益
        attitudeState.heightBasedGain = HEIGHT_GAIN_SCALE_FACTOR;
    } else {
        // 中间高度：线性缩放
        float heightRatio = (height - MIN_HEIGHT_FOR_SCALING) / (MAX_HEIGHT_FOR_SCALING - MIN_HEIGHT_FOR_SCALING);
        attitudeState.heightBasedGain = 1.0f - heightRatio * (1.0f - HEIGHT_GAIN_SCALE_FACTOR);
    }
}

/**
 * 振荡检测算法
 * 检测补偿值是否在振荡，如果振荡则降低增益
 */
bool detectOscillation(void)
{
    // 计算补偿变化量
    float rollChange = attitudeState.compensationRoll - attitudeState.lastCompensationRoll;
    float pitchChange = attitudeState.compensationPitch - attitudeState.lastCompensationPitch;
    
    // 检测是否反向变化（振荡特征）
    bool rollOscillating = (rollChange * attitudeState.lastCompensationRoll < 0) && 
                          (fabsf(rollChange) > 0.5f);
    bool pitchOscillating = (pitchChange * attitudeState.lastCompensationPitch < 0) && 
                           (fabsf(pitchChange) > 0.5f);
    
    if (rollOscillating || pitchOscillating) {
        attitudeState.oscillationCounter++;
        if (attitudeState.oscillationCounter > 5) {  // 连续5次振荡
            attitudeState.oscillationDetected = true;
        }
    } else {
        attitudeState.oscillationCounter = (attitudeState.oscillationCounter > 0) ? 
                                          attitudeState.oscillationCounter - 1 : 0;
        if (attitudeState.oscillationCounter == 0) {
            attitudeState.oscillationDetected = false;
        }
    }
    
    // 更新历史值
    attitudeState.lastCompensationRoll = attitudeState.compensationRoll;
    attitudeState.lastCompensationPitch = attitudeState.compensationPitch;
    
    return attitudeState.oscillationDetected;
}

/**
 * 第一层控制：优化的姿态稳定算法
 * 
 * 优化内容：
 * 1. 高度自适应增益：高空自动降低增益
 * 2. 振荡检测：检测到振荡自动减弱补偿
 * 3. 多级滤波：更平滑的姿态角处理
 * 4. 保守参数：防止过度补偿导致炸机
 */
bool performAttitudeStabilization(attitude_t* attitude, setpoint_t* setpoint, float dt, float height)
{
    if (!attitudeState.enabled) {
        return false;
    }
    
    // 0. 更新高度自适应增益
    updateHeightBasedGain(height);
    
    // 1. 多级姿态角滤波（更强的平滑效果）
    // 一级滤波：快速响应
    attitudeState.filteredRoll = attitudeState.filteredRoll * (1.0f - ATTITUDE_FILTER_ALPHA) + 
                                attitude->roll * ATTITUDE_FILTER_ALPHA;
    attitudeState.filteredPitch = attitudeState.filteredPitch * (1.0f - ATTITUDE_FILTER_ALPHA) + 
                                 attitude->pitch * ATTITUDE_FILTER_ALPHA;
    
    // 二级平滑：抑制高频振荡
    float smoothing_alpha = 0.1f;  // 更强的平滑
    attitudeState.smoothedRoll = attitudeState.smoothedRoll * (1.0f - smoothing_alpha) + 
                                attitudeState.filteredRoll * smoothing_alpha;
    attitudeState.smoothedPitch = attitudeState.smoothedPitch * (1.0f - smoothing_alpha) + 
                                 attitudeState.filteredPitch * smoothing_alpha;
    
    // 2. 优化的姿态补偿计算
    // 使用平滑后的角度进行补偿计算，减少高频振荡
    float baseGain = ATTITUDE_COMPENSATION_GAIN;
    
    // 应用高度自适应增益
    float adaptiveGain = baseGain * attitudeState.heightBasedGain;
    
    // 检测振荡并应用振荡抑制
    bool isOscillating = detectOscillation();
    if (isOscillating) {
        adaptiveGain *= 0.5f;  // 振荡时减半增益
        ESP_LOGW(TAG, "Oscillation detected, reducing gain to %.3f", adaptiveGain);
    }
    
    // 直接补偿到平衡点
    if (fabsf(attitudeState.smoothedRoll) > ATTITUDE_DEADBAND) {
        // 直接补偿到0°，确保推力垂直
        attitudeState.compensationRoll = -attitudeState.smoothedRoll * adaptiveGain;
    } else {
        // 在死区内清零补偿
        attitudeState.compensationRoll = 0.0f;
    }
    
    if (fabsf(attitudeState.smoothedPitch) > ATTITUDE_DEADBAND) {
        // 直接补偿到0°，确保推力垂直
        attitudeState.compensationPitch = -attitudeState.smoothedPitch * adaptiveGain;
    } else {
        // 在死区内清零补偿
        attitudeState.compensationPitch = 0.0f;
    }
    
    // 3. 更严格的补偿限制，防止剧烈机动
    attitudeState.compensationRoll = constrainf(attitudeState.compensationRoll, 
                                              -MAX_ATTITUDE_COMPENSATION, 
                                              MAX_ATTITUDE_COMPENSATION);
    attitudeState.compensationPitch = constrainf(attitudeState.compensationPitch, 
                                               -MAX_ATTITUDE_COMPENSATION, 
                                               MAX_ATTITUDE_COMPENSATION);
    
    // 4. 应用姿态补偿到控制输出
    setpoint->attitude.roll += attitudeState.compensationRoll;
    setpoint->attitude.pitch += attitudeState.compensationPitch;
    
    // 5. 优化的稳定性判断（更保守的策略）
    float totalAttitudeError = fabsf(attitudeState.smoothedRoll) + fabsf(attitudeState.smoothedPitch);
    float totalCompensation = fabsf(attitudeState.compensationRoll) + fabsf(attitudeState.compensationPitch);
    
    // 更严格的稳定判断：考虑振荡状态和高度因子
    bool currentlyStable = (totalAttitudeError < ATTITUDE_STABLE_THRESHOLD) && 
                          (totalCompensation < 1.5f) &&  // 放宽补偿阈值
                          (!attitudeState.oscillationDetected);  // 不能有振荡
    
    if (currentlyStable) {
        attitudeState.stableCounter++;
        attitudeState.unstableCounter = 0;
        
        // 更严格的稳定要求：连续25个周期才认为稳定
        if (attitudeState.stableCounter >= 25) {
            attitudeState.attitudeStable = true;
        }
    } else {
        attitudeState.unstableCounter++;
        attitudeState.stableCounter = 0;
        
        // 一旦检测到需要调整，立即禁用第二层
        attitudeState.attitudeStable = false;
    }
    
    // 6. 增强的调试输出
    static uint32_t debugCounter = 0;
    if (++debugCounter % 250 == 0) { // 每0.5秒输出一次
        ESP_LOGD(TAG, "Layer1: H=%.0fcm Gain=%.3f Att(R=%.2f°,P=%.2f°) Comp(R=%.2f°,P=%.2f°) Osc=%s Status=%s", 
                 attitudeState.currentHeight, attitudeState.heightBasedGain,
                 attitudeState.smoothedRoll, attitudeState.smoothedPitch,
                 attitudeState.compensationRoll, attitudeState.compensationPitch,
                 attitudeState.oscillationDetected ? "YES" : "NO",
                 attitudeState.attitudeStable ? "STABLE" : "ADJUSTING");
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
    // 执行第一层优化的姿态稳定控制（传入高度参数）
    bool attitudeStable = performAttitudeStabilization(&state->attitude, setpoint, dt, height);
    
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