/**
 * hover_control.h - Hover/Position hold control using optical flow and TOF sensors
 */

#ifndef HOVER_CONTROL_H_
#define HOVER_CONTROL_H_

#include <stdbool.h>
#include "stabilizer_types.h"
#include "optical_flow.h"
#include "vl53lxx.h"

// Hover control state
typedef struct {
    bool enabled;
    float targetX;      // Target X position in cm
    float targetY;      // Target Y position in cm  
    float targetHeight; // Target height in cm
    
    // Integrated position from optical flow
    float posX;
    float posY;
    
    // Last update timestamp
    uint32_t lastUpdateTime;
} hoverControl_t;

// Initialize hover control
void hoverControlInit(void);

// Enable/disable hover control
void hoverControlEnable(bool enable);

// Set hover target position
void hoverControlSetTarget(float x, float y, float height);

// Update hover control with sensor data
void hoverControlUpdate(flowMeasurement_t* flow, tofMeasurement_t* tof, 
                       setpoint_t* setpoint, state_t* state, float dt);

// Reset hover control state
void hoverControlReset(void);

// Check if hover control is active
bool hoverControlIsActive(void);

#endif /* HOVER_CONTROL_H_ */
