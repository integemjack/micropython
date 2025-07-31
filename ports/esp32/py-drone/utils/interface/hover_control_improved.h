/**
 * hover_control_improved.h - 改进的悬停控制接口
 * 
 * 核心改进：基于姿态前馈补偿的主动定点算法
 */

#ifndef __HOVER_CONTROL_IMPROVED_H__
#define __HOVER_CONTROL_IMPROVED_H__

#include <stdbool.h>
#include "stabilizer_types.h"

/**
 * 初始化改进的悬停控制系统
 */
void improvedHoverControlInit(void);

/**
 * 启用/禁用改进的悬停控制
 * @param enable true=启用, false=禁用
 */
void improvedHoverControlEnable(bool enable);

/**
 * 设置悬停目标位置
 * @param x 目标X位置 (cm)
 * @param y 目标Y位置 (cm) 
 * @param height 目标高度 (cm)
 */
void improvedHoverControlSetTarget(float x, float y, float height);

/**
 * 第一层控制：姿态稳定主函数
 * 
 * 职责：确保推力向量垂直，为第二层光流控制创造前提条件
 * 核心算法：主动补偿姿态角，防止产生水平分量
 * 
 * @param flow 光流传感器数据（此层不使用，保持接口兼容）
 * @param tof TOF传感器数据（此层不使用，保持接口兼容）
 * @param setpoint 控制设定值（输出姿态补偿）
 * @param state 当前状态（读取姿态角）
 * @param dt 控制周期 (s)
 * @param height 目标高度（此层不使用，保持接口兼容）
 * @return true=姿态稳定，可启用第二层控制; false=姿态不稳定，禁用第二层控制
 */
bool improvedHoverControlUpdate(flowMeasurement_t* flow, tofMeasurement_t* tof, 
                               setpoint_t* setpoint, state_t* state, float dt, float height);

/**
 * 检查第一层姿态稳定系统是否激活
 * @return true=激活, false=未激活
 */
bool improvedHoverControlIsActive(void);

/**
 * 检查姿态是否稳定（关键接口）
 * 此函数决定是否可以启用第二层光流控制
 * @return true=姿态稳定，可启用光流控制; false=姿态不稳定，禁用光流控制
 */
bool isAttitudeStabilized(void);

/**
 * 调试函数：获取第一层控制参数
 * @param attComp 姿态补偿值输出 [roll, pitch] (度)
 * @param flowCorr 滤波姿态值输出 [filtered_roll, filtered_pitch] (度)
 */
void getHoverControlDebugInfo(float* attComp, float* flowCorr);

#endif // __HOVER_CONTROL_IMPROVED_H__