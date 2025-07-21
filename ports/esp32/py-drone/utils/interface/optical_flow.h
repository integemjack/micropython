/**
 * optical_flow.h - Optical flow sensor interface
 */

#ifndef OPTICAL_FLOW_H_
#define OPTICAL_FLOW_H_

#include <stdbool.h>
#include "stabilizer_types.h"

// Initialize optical flow sensor
bool opticalFlowInit(void);

// Test optical flow sensor
bool opticalFlowTest(void);

// Read optical flow data
bool opticalFlowReadMeasurement(flowMeasurement_t* flow);

// Check if optical flow sensor is present
bool opticalFlowIsPresent(void);

// Deinitialize optical flow sensor
void opticalFlowDeInit(void);

#endif /* OPTICAL_FLOW_H_ */
