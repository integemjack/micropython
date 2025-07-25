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

static flowMeasurement_t flowData;
static tofMeasurement_t tofData;
static bool flowAvailable = false;
static bool tofAvailable = false;

void readOptionalSensors(void)
{
    // 重置可用性标志
    flowAvailable = false;
    tofAvailable = false;
    
    // 安全读取光流传感器数据
    if (opticalFlowIsPresent()) {
        flowAvailable = opticalFlowReadMeasurement(&flowData);
        if (!flowAvailable) {
            // 如果读取失败，清零数据
            memset(&flowData, 0, sizeof(flowData));
        }
    }
    
    // 安全读取TOF传感器数据
    if (tofSensorIsPresent()) {
        tofAvailable = tofSensorReadMeasurement(&tofData);
        if (!tofAvailable) {
            // 如果读取失败，清零数据
            memset(&tofData, 0, sizeof(tofData));
        }
    }
}

TaskHandle_t stabilizerHandle = NULL;
TaskHandle_t readOptionalSensorsHandle = NULL;

void stabilizerTask(void* param);
void opticalflowTask(void* param);

// 重置所有stabilizer静态变量 - 解决ESP32重启内存持久化问题
static void resetStabilizerStatics(void)
{
	// 重置高度控制相关变量
	velModeTimes = 0;
	absModeTimes = 0;
	setHeight = 0.0f;
	baroLast = 0.0f;
	baroVelLpf = 0.0f;
	
	// 重置传感器状态
	flowAvailable = false;
	tofAvailable = false;
	
	// 重置状态结构体
	memset(&setpoint, 0, sizeof(setpoint));
	memset(&sensorData, 0, sizeof(sensorData));
	memset(&state, 0, sizeof(state));
	memset(&control, 0, sizeof(control));
	
	// 重置光流和TOF数据
	memset(&flowData, 0, sizeof(flowData));
	memset(&tofData, 0, sizeof(tofData));
}

void stabilizerInit(void)
{
	if(isInit) return;
	
	// *** 关键修复：强制重置所有静态变量 ***
	resetStabilizerStatics();
	
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
	if( readOptionalSensorsHandle != NULL )
	{
		vTaskDelete( readOptionalSensorsHandle );
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
	}		
}

static void fastAdjustPosZ(void)
{	
    // 如果TOF传感器可用，使用TOF数据进行高度控制
	if (tofAvailable) {
		// 将TOF距离转换为mm到cm单位 (TOF返回mm，setHeight使用cm)
		float tofHeightCm = tofData.distance / 10.0f;
		
		// 添加合理性检查：TOF有效范围通常是4cm-400cm
		// if (tofHeightCm >= 4.0f && tofHeightCm <= 400.0f) {
			// 使用TOF测量的实际高度更新状态估计
		state.position.z = tofHeightCm;
		// }
		setpoint.mode.z = modeVelocity;
		setpoint.position.z = setHeight;
		setpoint.velocity.z = 0.0f;
    } else {
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
	
	// 启用悬停控制系统
	if (opticalFlowIsPresent() && tofSensorIsPresent()) {
		hoverControlEnable(true);
		hoverControlSetTarget(0.0f, 0.0f, 30.0f); // 设置默认悬停目标：x=0, y=0, height=30cm
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
		if (RATE_DO_EXECUTE(RATE_200_HZ, tick))
		{
			readOptionalSensors();
		}
		if (RATE_DO_EXECUTE(ATTITUDE_ESTIMAT_RATE, tick))
		{
			imuUpdate(sensorData.acc, sensorData.gyro, &state, ATTITUDE_ESTIMAT_DT);
		}
		if (RATE_DO_EXECUTE(POSITION_ESTIMAT_RATE, tick))
		{
			positionEstimate(&sensorData, &state, POSITION_ESTIMAT_DT);
		}
	
		if (RATE_DO_EXECUTE(RATE_100_HZ, tick) && getIsCalibrated()==true)
		{
			commanderGetSetpoint(&setpoint, &state);
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

		if (RATE_DO_EXECUTE(RATE_500_HZ, tick)) {
			bool rc_active = !getLockStatus(); // getLockStatus() returns true when RC is disconnected
			if (!rc_active) {
				
				// 光流数据融合进飞行控制实现定点悬停
				if (flowAvailable && tofAvailable) {
					// 使用光流和TOF数据进行悬停控制
					hoverControlUpdate(&flowData, &tofData, &setpoint, &state, 0.002f); // 500Hz = 2ms
					
					// 如果悬停控制激活，应用位置修正
					if (hoverControlIsActive()) {
						// 光流提供的位置变化量转换为速度控制
						float flowScaleFactor = 0.1f; // 光流像素到实际位移的比例因子
						
						// 将光流数据转换为位置变化
						float deltaX = flowData.dpixelx * flowScaleFactor;  
						float deltaY = flowData.dpixely * flowScaleFactor;
						
						// 应用位置控制到setpoint
						if (setpoint.mode.x == modeDisable) setpoint.mode.x = modeVelocity;
						if (setpoint.mode.y == modeDisable) setpoint.mode.y = modeVelocity;
						
						// 使用PID控制器进行位置稳定
						setpoint.velocity.x -= deltaX * 2.0f; // 位置修正增益
						setpoint.velocity.y -= deltaY * 2.0f; // 位置修正增益
					}
				}
				
				// snprintf(debug_str, sizeof(debug_str), "Flow[%s]: %.2f,%.2f TOF: %.2f, control: %.2d, %.2d, %.2d, %.2f\n", 
				// 	rc_active ? "RC" : "IDLE",
				// 	flowData.dpixelx, flowData.dpixely, tofData.distance,
				// 	control.roll, control.pitch, control.yaw, control.thrust);
				// debugpeintf(debug_str);
			} else {
				// snprintf(debug_str, sizeof(debug_str), "Flow[%s]: %.2f,%.2f TOF: %.2f, control: %.2d, %.2d, %.2d, %.2f\n", 
				// 	rc_active ? "RC" : "IDLE",
				// 	flowData.dpixelx, flowData.dpixely, tofData.distance,
				// 	control.roll, control.pitch, control.yaw, control.thrust);
				// debugpeintf(debug_str);
			}
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