#include <math.h>
#include "pid.h"
#include "commander.h"
#include "config_param.h"
#include "position_pid.h"
//#include "remoter_ctrl.h"
#include "maths.h"
#include "state_estimator.h"
#include "stabilizer.h"  // 包含getSetHeight()声明
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sensors_mpu6050_spl06.h"  // 包含debugpeintf()声明


#define THRUST_BASE  		(20000)	/*基础油门值*/

#define PIDVX_OUTPUT_LIMIT	100.0f	//修复：降低ROLL限幅，提高稳定性	(单位°带0.12的系数)
#define PIDVY_OUTPUT_LIMIT	120.0f 	//PITCH限幅	(单位°带0.15的系数)
#define PIDVZ_OUTPUT_LIMIT	(15000)	/*修复：降低VZ PID输出限幅，防止推力过大*/

#define PIDX_OUTPUT_LIMIT	1200.0f	//X轴速度限幅(单位cm/s 带0.1的系数)
#define PIDY_OUTPUT_LIMIT	1200.0f	//Y轴速度限幅(单位cm/s 带0.1的系数)
#define PIDZ_OUTPUT_LIMIT	60.0f	//修复：降低Z轴速度限幅，更平滑的高度变化


static float thrustLpf = THRUST_BASE;	/*油门低通*/
static float rollOutputLpf = 0.0f;		/*修复：Roll轴输出低通滤波*/

PidObject pidVX;
PidObject pidVY;
PidObject pidVZ;

PidObject pidX;
PidObject pidY;
PidObject pidZ;

void positionControlInit(float velocityPidDt, float posPidDt)
{
	pidInit(&pidVX, 0, configParam.pidPos.vx, velocityPidDt);	/*vx PID初始化*/
	pidInit(&pidVY, 0, configParam.pidPos.vy, velocityPidDt);	/*vy PID初始化*/
	pidInit(&pidVZ, 0, configParam.pidPos.vz, velocityPidDt);	/*vz PID初始化*/
	pidSetOutputLimit(&pidVX, PIDVX_OUTPUT_LIMIT);		/* 输出限幅 */
	pidSetOutputLimit(&pidVY, PIDVY_OUTPUT_LIMIT);		/* 输出限幅 */
	pidSetOutputLimit(&pidVZ, PIDVZ_OUTPUT_LIMIT);		/* 输出限幅 */
	
	pidInit(&pidX, 0, configParam.pidPos.x, posPidDt);			/*x PID初始化*/
	pidInit(&pidY, 0, configParam.pidPos.y, posPidDt);			/*y PID初始化*/
	pidInit(&pidZ, 0, configParam.pidPos.z, posPidDt);			/*z PID初始化*/
	pidSetOutputLimit(&pidX, PIDX_OUTPUT_LIMIT);		/* 输出限幅 */
	pidSetOutputLimit(&pidY, PIDY_OUTPUT_LIMIT);		/* 输出限幅 */
	pidSetOutputLimit(&pidZ, PIDZ_OUTPUT_LIMIT);		/* 输出限幅 */
}

static void velocityController(float* thrust, attitude_t *attitude, setpoint_t *setpoint, const state_t *state)                                                         
{	
	static uint16_t altholdCount = 0;
	
	// 实现渐进式推力控制
	static float baseThrust = 50000.0f;  // 基础推力，从1000开始
	static float onThrust = 0.0f;
	static bool initOnThrust = false;
	// static uint32_t delay = 10;
	static uint32_t lastUpdateTime = 0;
	
	// 检查紧急停止状态
	if (getCommanderEmerStop()) {
		// 紧急停止时立即清零所有输出
		*thrust = 0;
		baseThrust = 50000.0f;  // 重置基础推力
		attitude->pitch = 0;
		attitude->roll = 0;
		return;
	}
	
	// 检查是否在飞行状态，只有飞行时才使用我们的推力控制
	if (!getCommanderKeyFlight()) {
		// 非飞行状态，使用原始逻辑
		goto original_logic;
	}
	
	// 计算姿态控制 - 修复：为Roll轴设置独立的控制系数和滤波
	attitude->pitch = 0.15f * pidUpdate(&pidVX, setpoint->velocity.x - state->velocity.x);
	float rollOutput = 0.12f * pidUpdate(&pidVY, setpoint->velocity.y - state->velocity.y);
	rollOutputLpf += (rollOutput - rollOutputLpf) * 0.3f;  /*修复：Roll轴低通滤波，减少振荡*/
	attitude->roll = rollOutputLpf;
	
	// // 添加更新频率控制 - 每50ms更新一次推力(20Hz)，避免过于激进
	// uint32_t currentTime = xTaskGetTickCount();
	//  // * 10.0f;  // 转换为 mm/s
	// // uint32_t delay = fabs(currentVelZ) + 1;
	// if (currentTime - lastUpdateTime >= 50) {  // 50ms = 20Hz
	// 	lastUpdateTime = currentTime;
		
		// 计算高度误差
		float currentHeight = state->position.z * 10.0f;  // 转换为mm
		float targetHeight = getSetHeight() * 10.0f;  // 获取take_off()设置的目标高度并转换为mm
		float heightError = targetHeight - currentHeight;  // 高度误差
		float currentVelZ = state->velocity.z * 10;
		
		if (heightError > 0) {
			if (currentVelZ <= 500.0f && currentVelZ <= heightError) {
				baseThrust += heightError;
			} else {
				baseThrust -= heightError;
			}
		} else {
			if (currentVelZ > -10.0f && currentVelZ > heightError) {
				baseThrust += heightError;
			} else {
				baseThrust -= heightError;
			}
		}

		// baseThrust += heightError;

		// char debug_str[128];
		// snprintf(debug_str, sizeof(debug_str), "DEBUG: baseThrust=%.1f, currentHeight=%.1f, targetHeight=%.1f, currentVelZ=%.1f\n", baseThrust, currentHeight, targetHeight, currentVelZ);
		// debugpeintf(debug_str);



		baseThrust = constrainf(baseThrust, 18000.0f, 55000.0f);
	// }
	
	*thrust = baseThrust;
	return;
	
original_logic:
	// Roll and Pitch - 修复：为Roll轴设置独立的控制系数和滤波
	attitude->pitch = 0.15f * pidUpdate(&pidVX, setpoint->velocity.x - state->velocity.x);
	float rollOutputOrig = 0.12f * pidUpdate(&pidVY, setpoint->velocity.y - state->velocity.y);
	rollOutputLpf += (rollOutputOrig - rollOutputLpf) * 0.3f;  /*修复：Roll轴低通滤波，减少振荡*/
	attitude->roll = rollOutputLpf;
	
	// Thrust
	float thrustRaw = pidUpdate(&pidVZ, setpoint->velocity.z - state->velocity.z);
	
	// 使用stabilizer中统一的自适应基础推力
	float adaptiveThrust = getAdaptiveBaseThrust();
	
	// 重新设计推力控制：实现平滑的起飞和悬停逻辑
	static float lastThrust = 0;
	static bool isFirstTakeoff = true;
	
	// 起飞时的基础推力应该是悬停推力（约22000-25000）
	float baseHoverThrust = 22000.0f;  // 悬停基础推力
	
	// 如果是刚开始起飞，从悬停推力开始
	if (isFirstTakeoff && getCommanderKeyFlight()) {
		lastThrust = baseHoverThrust;
		isFirstTakeoff = false;
	}
	
	// 如果不再起飞状态，重置标志
	if (!getCommanderKeyFlight()) {
		isFirstTakeoff = true;
		lastThrust = 0;
	}
	
	// 计算目标推力：PID输出 + 自适应基础推力
	float targetThrust = constrainf(thrustRaw + adaptiveThrust, 1000, 35000);  // 进一步降低最大推力
	
	// 修复：根据高度误差智能调整推力限制
	if (getCommanderKeyFlight()) {
		// 获取真实的目标高度
		float targetHeight = getSetHeight() * 10.0f;  // 目标高度转换为mm
		float currentHeight = state->position.z * 10.0f;  // 当前高度转换为mm
		float heightError = currentHeight - targetHeight;  // 高度误差
		
		// 只有在刚起飞且高度低于目标时才强制最小推力
		if (isFirstTakeoff || (currentHeight < targetHeight && lastThrust < baseHoverThrust)) {
			// 起飞阶段或低于目标高度时，确保有足够推力
			targetThrust = (targetThrust < baseHoverThrust) ? baseHoverThrust : targetThrust;
		} else if (heightError > 100.0f) {
			// 超过目标高度10cm以上，允许推力大幅降低以快速下降
			float minThrust = 8000.0f;  // 允许降到更低的推力
			targetThrust = (targetThrust < minThrust) ? minThrust : targetThrust;
		}
		// 其他情况完全跟随PID输出
	}
	
	// 限制推力变化率，但当高度偏差大时允许更快变化
	float maxThrustChange = 1000.0f;  // 默认变化率
	
	// 如果在飞行状态，根据高度误差调整变化率
	if (getCommanderKeyFlight()) {
		float targetHeight = getSetHeight() * 10.0f;
		float currentHeight = state->position.z * 10.0f;
		float heightError = fabsf(currentHeight - targetHeight);
		
		// 高度误差大时，允许更快的推力变化
		if (heightError > 200.0f) {  // 超过20cm误差
			maxThrustChange = 3000.0f;  // 允许快速变化
		} else if (heightError > 100.0f) {  // 超过10cm误差
			maxThrustChange = 2000.0f;  // 中等变化速度
		}
	}
	
	if (fabsf(targetThrust - lastThrust) > maxThrustChange) {
		if (targetThrust > lastThrust) {
			targetThrust = lastThrust + maxThrustChange;
		} else {
			targetThrust = lastThrust - maxThrustChange;
		}
	}
	
	*thrust = targetThrust;
	lastThrust = *thrust;
	
	thrustLpf += (*thrust - thrustLpf) * 0.003f;
	
	if(getCommanderKeyFlight())	/*定高飞行状态*/
	{
		if(fabs(state->acc.z) < 35.f)
		{
			altholdCount++;
			if(altholdCount > 1000)
			{
				altholdCount = 0;
				if(fabs(configParam.thrustBase - thrustLpf) > 1000.f)	/*更新基础油门值*/
					configParam.thrustBase = thrustLpf;
			}
		}else
		{
			altholdCount = 0;
		}
	}else if(getCommanderKeyland())	/*正在降落状态*/
	{
		// 降落模式下保持PID计算的推力值，不清零
		// *thrust 保持PID计算的结果
	}else	/*既不起飞也不降落，油门清零*/
	{
		*thrust = 0;
	}
}

void positionController(float* thrust, attitude_t *attitude, setpoint_t *setpoint, const state_t *state, float dt)                                                
{	
	if (setpoint->mode.x == modeAbs || setpoint->mode.y == modeAbs)
	{
		setpoint->velocity.x = 0.1f * pidUpdate(&pidX, setpoint->position.x - state->position.x);
		setpoint->velocity.y = 0.1f * pidUpdate(&pidY, setpoint->position.y - state->position.y);
	}
	
	if (setpoint->mode.z == modeAbs)
	{
		setpoint->velocity.z = 0.1f * pidUpdate(&pidZ, setpoint->position.z - state->position.z);
		// float heightError = setpoint->position.z - state->position.z;
		
		// // 修复：当接近目标高度时，降低速度设定值，实现平滑到达
		// float targetVelocity = pidUpdate(&pidZ, heightError);
		
		// // 如果高度误差很小（±3cm内），设置很小的速度，基本悬停
		// if (fabsf(heightError) < 30.f) {  // 3cm内
		// 	targetVelocity *= 0.3f;  // 降低到30%的速度
		// } else if (fabsf(heightError) < 50.f) {  // 5cm内
		// 	targetVelocity *= 0.6f;  // 降低到60%的速度
		// }
		
		// // 修复：当高度误差非常小时，直接设置速度为0，实现精确悬停
		// if (fabsf(heightError) < 10.f) {  // 1cm内
		// 	targetVelocity = 0.0f;
		// }
		
		// setpoint->velocity.z = targetVelocity;
	}
	
	velocityController(thrust, attitude, setpoint, state);
}

/*获取定高油门值*/
float getAltholdThrust(void)
{
	return thrustLpf;
}

void positionResetAllPID(void)
{
	pidReset(&pidVX);
	pidReset(&pidVY);
	pidReset(&pidVZ);

	pidReset(&pidX);
	pidReset(&pidY);
	pidReset(&pidZ);
}

void positionPIDwriteToConfigParam(void)
{
	configParam.pidPos.vx.kp  = pidVX.kp;
	configParam.pidPos.vx.ki  = pidVX.ki;
	configParam.pidPos.vx.kd  = pidVX.kd;
	
	configParam.pidPos.vy.kp  = pidVY.kp;
	configParam.pidPos.vy.ki  = pidVY.ki;
	configParam.pidPos.vy.kd  = pidVY.kd;
	
	configParam.pidPos.vz.kp  = pidVZ.kp;
	configParam.pidPos.vz.ki  = pidVZ.ki;
	configParam.pidPos.vz.kd  = pidVZ.kd;
	
	configParam.pidPos.x.kp  = pidX.kp;
	configParam.pidPos.x.ki  = pidX.ki;
	configParam.pidPos.x.kd  = pidX.kd;
	
	configParam.pidPos.y.kp  = pidY.kp;
	configParam.pidPos.y.ki  = pidY.ki;
	configParam.pidPos.y.kd  = pidY.kd;
	
	configParam.pidPos.z.kp  = pidZ.kp;
	configParam.pidPos.z.ki  = pidZ.ki;
	configParam.pidPos.z.kd  = pidZ.kd;
}
