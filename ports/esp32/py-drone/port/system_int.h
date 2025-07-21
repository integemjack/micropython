#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#include <stdbool.h>
#include <stdint.h>

void systemInit(void);
void systemDeInit(void);

// 新增：传感器状态查询函数
bool getOpticalFlowStatus(void);
bool getTofSensorStatus(void);

#endif
