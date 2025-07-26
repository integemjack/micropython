// #include "system.h"
#include "stabilizer.h"
#include "sensors.h"
#include "sensfusion6.h"
#include "commander.h"
#include "anomal_detec.h"
#include "state_control.h"
#include "state_estimator.h"
#include "power_distribution.h"
#include "position_pid.h"
#include "flip.h"
#include "optical_flow.h"
#include "vl53lxx.h"
#include "hover_control.h"
#include "maths.h"
#include "ledseq.h"
#include "sensfusion6.h"

#include "sensors_mpu6050_spl06.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

static bool isInit = false;

static setpoint_t 	setpoint;	/*设置目标状态*/
static sensorData_t sensorData;	/*传感器数据*/
static state_t 		state;		/*四轴姿态*/
static control_t 	control;	/*四轴控制参数*/

static uint16_t velModeTimes = 0;		/*速率模式次数*/
static uint16_t absModeTimes = 0;		/*绝对值模式次数*/
static float setHeight = 0.f;		/*设定目标高度 单位cm*/
static float baroLast = 0.f;
static float baroVelLpf = 0.f;

// 统一高度控制状态
typedef enum {
    HEIGHT_CTRL_MANUAL = 0,    // 手动推力控制
    HEIGHT_CTRL_AUTO,          // 自动高度控制  
    HEIGHT_CTRL_TAKEOFF,       // 起飞模式
    HEIGHT_CTRL_LANDING        // 降落模式
} heightCtrlMode_t;

static heightCtrlMode_t heightMode = HEIGHT_CTRL_MANUAL;
static float manualThrustInput = 0.0f;
static float adaptiveBaseThrust = 20000.0f;

static flowMeasurement_t flowData;
static tofMeasurement_t tofData;
static bool flowAvailable = false;
static bool tofAvailable = false;

void readOpticalFlowSensor(void)
{
    // 重置可用性标志
    // flowAvailable = false;
    // tofAvailable = false;
    
    // 安全读取光流传感器数据
    if (opticalFlowIsPresent()) {
        flowAvailable = opticalFlowReadMeasurement(&flowData);
        // if (!flowAvailable) {
        //     // 如果读取失败，清零数据
        //     memset(&flowData, 0, sizeof(flowData));
        // }
    }
}

void readTofSensor(void)
{
    if (tofSensorIsPresent()) {
        tofAvailable = tofSensorReadMeasurement(&tofData);
    }
}

TaskHandle_t stabilizerHandle = NULL;

void stabilizerTask(void* param);

void stabilizerInit(void)
{
	if(isInit) return;
	stateControlInit();		/*姿态PID初始化*/
	hoverControlInit();		/* 悬停控制初始化 */
	powerDistributionInit();		/*电机初始化*/

	// 单核模式下的优化策略：
	// 稳定器任务保持高优先级，负责飞控核心逻辑
	// 光流任务使用最低优先级，仅在系统空闲时运行
	xTaskCreate(stabilizerTask, STABILIZER_TASK_NAME, STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, &stabilizerHandle);	
	

	isInit = true;
}
void stabilizerDeInit(void)
{
	if( stabilizerHandle != NULL )
	{
		vTaskDelete( stabilizerHandle );
	}
	isInit = false;
}

bool stabilizerTest(void)
{
	bool pass = true;

	pass &= stateControlTest();
	pass &= powerDistributionTest();

	return pass;
}

void setFastAdjustPosParam(uint16_t velTimes, uint16_t absTimes, float height)
{
	if(velTimes != 0 && velModeTimes == 0)
	{
		baroLast = sensorData.baro.asl;
		baroVelLpf = 0.f;

		velModeTimes = velTimes;
	}
	if(absTimes != 0 && absModeTimes ==0)
	{
		setHeight = height;
		absModeTimes = absTimes;
		// 设置目标高度时自动切换到自动高度控制模式
		heightMode = HEIGHT_CTRL_AUTO;
	}		
}

// 统一高度控制核心函数
void updateHeightControl(void)
{
	float currentHeight = getFusedHeight();
	
	// 传感器融合：TOF + 气压计
	float fusedHeight = currentHeight;
	if (tofAvailable && tofData.distance > 0) {
		float tofHeightCm = tofData.distance / 10.0f; // mm to cm
		// TOF在近距离时权重更高
		if (tofHeightCm < 200.0f) {
			float tofWeight = 0.7f - (tofHeightCm / 400.0f) * 0.4f; // 5-200cm: 0.7-0.3权重
			fusedHeight = currentHeight * (1.0f - tofWeight) + tofHeightCm * tofWeight;
		}
	}
	
	switch (heightMode) {
		case HEIGHT_CTRL_MANUAL:
			// 手动模式：使用遥控器推力，但保持基础推力保护
			adaptiveBaseThrust = manualThrustInput * 0.3f + 20000.0f;
			setpoint.mode.z = modeDisable;
			setpoint.thrust = manualThrustInput;
			break;
			
		case HEIGHT_CTRL_AUTO:
		case HEIGHT_CTRL_TAKEOFF:
			// 自动模式：基于高度误差的自适应推力
			{
				float heightError = setHeight - fusedHeight;
				
				if (fabs(heightError) > 5.0f) {
					float thrustAdjustment = heightError * 100.0f; // 每cm误差100推力值
					adaptiveBaseThrust = 20000.0f + constrainf(thrustAdjustment, -8000, 12000);
					
					// 接近目标时平滑过渡
					if (fabs(heightError) < 20.0f) {
						float smoothFactor = fabs(heightError) / 20.0f;
						adaptiveBaseThrust = 20000.0f + thrustAdjustment * smoothFactor;
					}
				} else {
					adaptiveBaseThrust = 20000.0f;
				}
				
				setpoint.mode.z = modeAbs;
				setpoint.position.z = setHeight;
			}
			break;
			
		case HEIGHT_CTRL_LANDING:
			// 降落模式：逐渐减小推力
			adaptiveBaseThrust = 15000.0f + (fusedHeight / setHeight) * 5000.0f;
			setpoint.mode.z = modeVelocity;
			setpoint.velocity.z = -50.0f; // 50cm/s下降
			break;
	}
	
	// 安全限制
	adaptiveBaseThrust = constrainf(adaptiveBaseThrust, 10000.0f, 50000.0f);
}

// 外部接口函数
void setHeightControlMode(int mode) 
{
	heightMode = (heightCtrlMode_t)mode;
}

void setManualThrust(float thrust)
{
	manualThrustInput = thrust;
	if (heightMode == HEIGHT_CTRL_MANUAL) {
		heightMode = HEIGHT_CTRL_MANUAL; // 确保在手动模式
	}
}

void setTargetHeight(float height)
{
	setHeight = height;
	heightMode = HEIGHT_CTRL_AUTO;
}

float getAdaptiveBaseThrust(void)
{
	return adaptiveBaseThrust;
}

static void fastAdjustPosZ(void)
{	

	// 如果TOF传感器可用，使用TOF数据进行高度控制
	// if (tofAvailable && tofData.distance > 0.0f)
	// {
	// 	// 将TOF距离转换为mm到cm单位 (TOF返回mm，setHeight使用cm)
	// 	float tofHeightCm = tofData.distance / 100.0f;
		
	// 	// 添加合理性检查：TOF有效范围通常是4cm-400cm
	// 	if (tofHeightCm >= 0.1f && tofHeightCm <= 400.0f) {
	// 		// 使用TOF测量的实际高度更新状态估计（添加滤波避免突变）
	// 		static float filteredHeight = 0.0f;
	// 		static bool heightInitialized = false;
			
	// 		// 第一次初始化滤波器
	// 		// if (!heightInitialized) {
	// 		// 	filteredHeight = tofHeightCm;
	// 		// 	heightInitialized = true;
	// 		// } else {
	// 		// 	// 低通滤波避免噪声和突变
	// 		// 	float alpha = 0.1f;  // 滤波系数，值越小越平滑
	// 		// 	filteredHeight = alpha * tofHeightCm + (1.0f - alpha) * filteredHeight;
	// 		// }
			
	// 		state.position.z = tofHeightCm;
			
	// 		// 设置为位置绝对模式（不是速度模式）
	// 		setpoint.mode.z = modeAbs;          // 位置绝对控制模式
	// 		setpoint.position.z = setHeight;    // 目标高度位置
	// 		setpoint.velocity.z = 0.0f;         // 目标速度为0（悬停）
	// 	} else {
	// 		// TOF数据超出有效范围，使用默认高度控制
	// 		setpoint.mode.z = modeAbs;
	// 		setpoint.position.z = setHeight;
	// 		setpoint.velocity.z = 0.0f;
	// 	}
    // }
	// else 
	if(velModeTimes > 0)
	{
		velModeTimes--;
		estRstHeight();	/*复位估测高度*/
		
		float baroVel = (sensorData.baro.asl - baroLast) / 0.004f;	/*250Hz*/
		baroLast = sensorData.baro.asl;
		baroVelLpf += (baroVel - baroVelLpf) * 0.35f;

		setpoint.mode.z = modeVelocity;
		state.velocity.z = baroVelLpf;		/*气压计融合*/
		setpoint.velocity.z = -1.0f * baroVelLpf;
		
		if(velModeTimes == 0)
		{
			setHeight = state.position.z;
		}		
	}
	else if(absModeTimes > 0)
	{
		absModeTimes--;
		estRstAll();	/*复位估测*/
		setpoint.mode.z = modeAbs;		
		setpoint.position.z = setHeight;
	}	
}

void stabilizerTask(void* param)
{
	uint32_t tick = 0;
	uint32_t lastWakeTime = xTaskGetTickCount();//getSysTickCnt();
	char debug_str[80];
	
	ledseqRun(SYS_LED, seq_alive);

	while(!sensorsAreCalibrated())
	{
		vTaskDelayUntil(&lastWakeTime, MAIN_LOOP_DT);
	}

	if(opticalFlowIsPresent()) {
		snprintf(debug_str, sizeof(debug_str), "Optical flow sensor present\n");
		debugpeintf(debug_str);
	} else {
		snprintf(debug_str, sizeof(debug_str), "Optical flow sensor not present\n");
		debugpeintf(debug_str);
	}

	if (tofSensorIsPresent()) {
		snprintf(debug_str, sizeof(debug_str), "TOF sensor present\n");
		debugpeintf(debug_str);
	} else {
		snprintf(debug_str, sizeof(debug_str), "TOF sensor not present\n");
		debugpeintf(debug_str);
	}

	while(1) 
	{
		vTaskDelayUntil(&lastWakeTime, 1);

		// if (RATE_DO_EXECUTE(RATE_1_HZ, tick)) {
		// 	debugpeintf("running...\n");
		// }

		if (RATE_DO_EXECUTE(RATE_500_HZ, tick))
		{
			sensorsAcquire(&sensorData, tick);
		}
		if (RATE_DO_EXECUTE(ATTITUDE_ESTIMAT_RATE, tick))
		{
			imuUpdate(sensorData.acc, sensorData.gyro, &state, ATTITUDE_ESTIMAT_DT);
			readOpticalFlowSensor();
		}
		if (RATE_DO_EXECUTE(ATTITUDE_ESTIMAT_RATE / 2, tick))
		{
			readTofSensor();
		}
		if (RATE_DO_EXECUTE(POSITION_ESTIMAT_RATE, tick))
		{
			positionEstimate(&sensorData, &state, POSITION_ESTIMAT_DT);
		}
	
		if (RATE_DO_EXECUTE(RATE_100_HZ, tick) && getIsCalibrated()==true)
		{
			commanderGetSetpoint(&setpoint, &state);
			// 统一高度控制更新
			updateHeightControl();
		}

		if (RATE_DO_EXECUTE(RATE_250_HZ, tick))
		{
			fastAdjustPosZ();
		}		

		if (RATE_DO_EXECUTE(RATE_500_HZ, tick) && (getCommanderCtrlMode() != 0x03))
		{
			flyerFlipCheck(&setpoint, &control, &state);	
		}
		anomalDetec(&sensorData, &state, &control);			
		stateControl(&control, &sensorData, &state, &setpoint, tick);

		if (RATE_DO_EXECUTE(RATE_500_HZ, tick))
		{
			powerDistribution(&control);
		}
		tick++;
	}
}

void getAttitudeData(attitude_t* get)
{
	get->pitch = state.attitude.pitch;
	get->roll = state.attitude.roll;
	get->yaw = state.attitude.yaw;
}
void getControlledQuantiy(control_t* get)
{
	get->roll = control.roll;
	get->pitch = control.pitch;
	get->yaw = control.yaw;
	get->thrust = control.thrust;
	get->flipDir = control.flipDir;
}
bool getLockStatus(void)
{
	return state.isRCLocked;
}

float getBaroData(void)
{
	return sensorData.baro.asl;
}

float getBaroTemp(void)
{
	return sensorData.baro.temperature;
}

void getSensorData(sensorData_t* get)
{
	*get = sensorData;
}

void getStateData(Axis3f* acc, Axis3f* vel, Axis3f* pos)
{
	acc->x = 1.0f * state.acc.x;
	acc->y = 1.0f * state.acc.y;
	acc->z = 1.0f * state.acc.z;
	vel->x = 1.0f * state.velocity.x;
	vel->y = 1.0f * state.velocity.y;
	vel->z = 1.0f * state.velocity.z;
	pos->x = 1.0f * state.position.x;
	pos->y = 1.0f * state.position.y;
	pos->z = 1.0f * state.position.z;
}

// 获取缓存的光流数据
bool getCachedFlowData(flowMeasurement_t* flow)
{
	if (flow && flowAvailable) {
		*flow = flowData;
		return true;
	}
	return false;
}

// 获取缓存的TOF数据
bool getCachedTofData(tofMeasurement_t* tof)
{
	if (tof && tofAvailable) {
		*tof = tofData;
		return true;
	}
	return false;
}

// 获取目标设定高度
float getSetHeight(void)
{
	return setHeight;
}