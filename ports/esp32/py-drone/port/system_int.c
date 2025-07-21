
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "py/obj.h"
#include "anop.h"
#include "system_int.h"
#include "config_param.h"
#include "esp_log.h"

#include "pm_esplane.h"
#include "sensors_mpu6050_spl06.h"
#include "stabilizer.h"
#include "ledseq.h"
#include "motors.h"
#include "optical_flow.h"
#include "vl53lxx.h"
#include "i2cdev.h"

static const char* TAG = "system_int";

static bool isInit = 0;
static bool opticalFlowEnabled = false;
static bool tofSensorEnabled = false;

static bool systemTest(void)
{
	bool pass = true;
	
	pass &= ledseqTest();  //led序列测试
	pass &= pmTest();
	//pass &= commTest();
	pass &= stabilizerTest();
	
	return pass;
}

//初始化系统
void systemInit(void)
{
	if(isInit) return;
	ESP_LOGI(TAG, "start systeminit ..");
	debugpeintf("start systeminit ..\r\n");
	
	//初始化电源管理
	ledseqInit();
	ledseqRun(SYS_LED, seq_alive);
	configParamInit();//初始化参数
	motorsInit();
	pmInit();
	//初始化传感器
	sensorsMpu6050Spl06Init();
	
	// 初始化TOF传感器专用I2C总线
	i2cdrvInit(&tofBus);
	
	// 尝试初始化光流传感器 (不影响主程序运行)
	ESP_LOGI(TAG, "Checking for optical flow sensor...");
	debugpeintf("Checking for optical flow sensor...\r\n");
	opticalFlowEnabled = opticalFlowInit();
	if (opticalFlowEnabled) {
		ESP_LOGI(TAG, "✓ Optical flow sensor enabled and ready");
		debugpeintf("✓ Optical flow sensor enabled and ready\r\n");
	} else {
		ESP_LOGI(TAG, "✗ Optical flow sensor not available - system will continue without it");
		debugpeintf("✗ Optical flow sensor not available - system will continue without it\r\n");
	}
	
	// 尝试初始化TOF传感器 (不影响主程序运行)
	ESP_LOGI(TAG, "Checking for TOF sensor...");
	tofSensorEnabled = tofSensorInit();
	if (tofSensorEnabled) {
		ESP_LOGI(TAG, "✓ TOF sensor enabled and ready");
		debugpeintf("✓ TOF sensor enabled and ready\r\n");
	} else {
		ESP_LOGI(TAG, "✗ TOF sensor not available - system will continue without it");
		debugpeintf("✗ TOF sensor not available - system will continue without it\r\n");
	}
	
	// 显示传感器状态总结
	ESP_LOGI(TAG, "Sensor summary: Flow=%s, TOF=%s", 
		opticalFlowEnabled ? "ON" : "OFF",
		tofSensorEnabled ? "ON" : "OFF");
	
	//初始化姿态处理
	stabilizerInit();
	systemTest();
}

void systemDeInit(void)
{
	stabilizerDeInit();

	// 清理传感器 (安全清理，即使传感器未初始化也不会出错)
	if (opticalFlowEnabled) {
		opticalFlowDeInit();
		opticalFlowEnabled = false;
	}
	
	if (tofSensorEnabled) {
		tofSensorDeInit();
		tofSensorEnabled = false;
	}
	
	sensorsMpu6050Spl06DeInit();
	motorsDeInit();
	pmDeInit();
	ledseqDeInit();
	
	// 清理TOF传感器I2C总线
	i2cDrvDeInit(&tofBus);
	//sensorsI2CdevDeInit();
	
	isInit = false;
}

// 获取传感器状态的函数 (可供其他模块查询)
bool getOpticalFlowStatus(void)
{
	return opticalFlowEnabled;
}

bool getTofSensorStatus(void)
{
	return tofSensorEnabled;
}

