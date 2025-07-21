/**
 * stabilizer_additions.c - Additional code for stabilizer with flow and TOF sensors
 * 
 * This code should be integrated into stabilizer.c
 */

#include <stdbool.h>
#include "stabilizer_types.h"
#include "optical_flow.h"
#include "vl53lxx.h"

// Add these variables at the top of stabilizer.c (after existing static variables)
static flowMeasurement_t flowData;
static tofMeasurement_t tofData;
static bool flowAvailable = false;
static bool tofAvailable = false;

// Add this function to read sensor data (call in stabilizerTask)
void readOptionalSensors(void)
{
    // Read optical flow data if available
    if (opticalFlowIsPresent()) {
        flowAvailable = opticalFlowReadMeasurement(&flowData);
    }
    
    // Read TOF data if available  
    if (tofSensorIsPresent()) {
        tofAvailable = tofSensorReadMeasurement(&tofData);
    }
}

// Add this to the position estimation call (modify existing positionEstimate call)
// Replace: positionEstimate(&sensorData, &state, POSITION_ESTIMAT_DT);
// With: positionEstimate(&state, &sensorData, tofAvailable ? &tofData : NULL, POSITION_ESTIMAT_DT, tick);
