/**
 * vl53lxx.h - VL53L0X/VL53L1X TOF sensor interface
 */

#ifndef VL53LXX_H_
#define VL53LXX_H_

#include <stdbool.h>
#include "stabilizer_types.h"

// Initialize TOF sensor
bool tofSensorInit(void);

// Test TOF sensor
bool tofSensorTest(void);

// Read TOF measurement
bool tofSensorReadMeasurement(tofMeasurement_t* tof);

// Check if TOF sensor is present
bool tofSensorIsPresent(void);

// Deinitialize TOF sensor
void tofSensorDeInit(void);

#endif /* VL53LXX_H_ */
