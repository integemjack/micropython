/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * power_distribution_stock.c - Crazyflie stock power distribution code
 */

#include <string.h>

#include "power_distribution.h"
#include <stdio.h>
#include <string.h>
// #include "log.h"
// #include "param.h"
#include "num.h"
// #include "platform.h"
#include "motors.h"
#include "config.h"

#define TAG "PWR_DIST"

static bool motorSetEnable = false;

static motorPower_t motorPower;

static struct {
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
} motorPowerSet;

#ifndef DEFAULT_IDLE_THRUST
#define DEFAULT_IDLE_THRUST 0
#endif

static uint32_t idleThrust = DEFAULT_IDLE_THRUST;

void powerDistributionInit(void)
{
  motorsInit();
}

bool powerDistributionTest(void)
{
  bool pass = true;

  pass &= motorsTest();

  return pass;
}

#define limitThrust(VAL) limitUint16(VAL)

void powerStop()
{
  motorsSetRatio(MOTOR_M1, 0);
  motorsSetRatio(MOTOR_M2, 0);
  motorsSetRatio(MOTOR_M3, 0);
  motorsSetRatio(MOTOR_M4, 0);
}

void powerDistribution(const control_t *control)
{
  // 安全检查：防止空指针
  if (!control) {
    powerStop();
    return;
  }

  #ifdef QUAD_FORMATION_X
    // X型四旋翼混控算法修复版本
    // 使用浮点运算保持精度，然后转换为整数
    float r = control->roll / 100 / 2.0f;
    float p = control->pitch / 100 / 2.0f;
    float y = control->yaw / 100;
    float t = control->thrust;

    // X型四旋翼标准混控算法：
    // M1(前右): thrust - roll - pitch + yaw
    // M2(后左): thrust - roll + pitch - yaw  
    // M3(后右): thrust + roll + pitch + yaw
    // M4(前左): thrust + roll - pitch - yaw
    float m1_calc = t - r - p + y;
    float m2_calc = t - r + p - y;
    float m3_calc = t + r + p + y;
    float m4_calc = t + r - p - y;
    
    // 确保所有电机功率都大于基础推力t，防止失去动力
    motorPower.m1 = limitThrust((int16_t)(m1_calc > t ? m1_calc : t));
    motorPower.m2 = limitThrust((int16_t)(m2_calc > t ? m2_calc : t));
    motorPower.m3 = limitThrust((int16_t)(m3_calc > t ? m3_calc : t));
    motorPower.m4 = limitThrust((int16_t)(m4_calc > t ? m4_calc : t));
  #else // QUAD_FORMATION_NORMAL
    motorPower.m1 = limitThrust(control->thrust + control->pitch +control->yaw);
    motorPower.m2 = limitThrust(control->thrust - control->roll -control->yaw);
    motorPower.m3 =  limitThrust(control->thrust - control->pitch +control->yaw);
    motorPower.m4 =  limitThrust(control->thrust + control->roll -control->yaw);
  #endif

  if (motorSetEnable)
  {
    motorsSetRatio(MOTOR_M1, motorPowerSet.m1);
    motorsSetRatio(MOTOR_M2, motorPowerSet.m2);
    motorsSetRatio(MOTOR_M3, motorPowerSet.m3);
    motorsSetRatio(MOTOR_M4, motorPowerSet.m4);
  }
  else
  {
    if (motorPower.m1 < idleThrust) {
      motorPower.m1 = idleThrust;
    }
    if (motorPower.m2 < idleThrust) {
      motorPower.m2 = idleThrust;
    }
    if (motorPower.m3 < idleThrust) {
      motorPower.m3 = idleThrust;
    }
    if (motorPower.m4 < idleThrust) {
      motorPower.m4 = idleThrust;
    }

    // 额外安全检查：确保电机功率不为负值或异常值
    uint16_t m1Power = (motorPower.m1 < 0) ? 0 : motorPower.m1;
    uint16_t m2Power = (motorPower.m2 < 0) ? 0 : motorPower.m2;
    uint16_t m3Power = (motorPower.m3 < 0) ? 0 : motorPower.m3;
    uint16_t m4Power = (motorPower.m4 < 0) ? 0 : motorPower.m4;

    motorsSetRatio(MOTOR_M1, m1Power);
    motorsSetRatio(MOTOR_M2, m2Power);
    motorsSetRatio(MOTOR_M3, m3Power);
    motorsSetRatio(MOTOR_M4, m4Power);
  }
  
}

void getMotorPWM(motorPower_t* get)
{
	*get = motorPower;
}

// 测试函数：验证X型四旋翼混控算法
void testQuadMixing(void)
{
  printf("=== X型四旋翼混控算法测试 ===\n");
  
  control_t testControl;
  
  // 测试1：纯推力
  testControl.thrust = 10000;
  testControl.roll = 0;
  testControl.pitch = 0;
  testControl.yaw = 0;
  powerDistribution(&testControl);
  printf("纯推力测试: M1=%d M2=%d M3=%d M4=%d\n", 
         motorPower.m1, motorPower.m2, motorPower.m3, motorPower.m4);
  
  // 测试2：Roll控制（向右滚转）
  testControl.thrust = 10000;
  testControl.roll = 1000;
  testControl.pitch = 0;
  testControl.yaw = 0;
  powerDistribution(&testControl);
  printf("Roll右滚测试: M1=%d M2=%d M3=%d M4=%d\n", 
         motorPower.m1, motorPower.m2, motorPower.m3, motorPower.m4);
  
  // 测试3：Pitch控制（向前俯仰）
  testControl.thrust = 10000;
  testControl.roll = 0;
  testControl.pitch = 1000;
  testControl.yaw = 0;
  powerDistribution(&testControl);
  printf("Pitch前俯测试: M1=%d M2=%d M3=%d M4=%d\n", 
         motorPower.m1, motorPower.m2, motorPower.m3, motorPower.m4);
  
  // 测试4：Yaw控制（右转）
  testControl.thrust = 10000;
  testControl.roll = 0;
  testControl.pitch = 0;
  testControl.yaw = 1000;
  powerDistribution(&testControl);
  printf("Yaw右转测试: M1=%d M2=%d M3=%d M4=%d\n", 
         motorPower.m1, motorPower.m2, motorPower.m3, motorPower.m4);
  
  printf("=== 测试完成 ===\n");
}

