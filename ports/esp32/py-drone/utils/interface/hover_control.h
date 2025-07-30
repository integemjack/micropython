/**
 * hover_control.h - Header for enhanced hover/position hold control
 */

#ifndef HOVER_CONTROL_H
#define HOVER_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

// Forward declarations
typedef struct flowMeasurement_s flowMeasurement_t;
typedef struct tofMeasurement_s tofMeasurement_t;
typedef struct setpoint_s setpoint_t;
typedef struct state_s state_t;

// Enhanced hover control state structure
typedef struct hoverControl_s {
    bool enabled;               // Hover control enable flag
    
    // Target positions
    float targetX;              // Target X position (cm)
    float targetY;              // Target Y position (cm)
    float targetHeight;         // Target height (cm)
    
    // Current estimated positions
    float posX;                 // Current X position estimate (cm)
    float posY;                 // Current Y position estimate (cm)
    float currentHeight;        // Current filtered height (cm)
    
    // Height control related
    float heightVelocity;       // Estimated vertical velocity (cm/s)
    float lastHeight;           // Previous height for velocity calculation
    
    // Timing
    uint32_t lastUpdateTime;    // Last update timestamp
} hoverControl_t;


/**
 * Initialize hover control system
 * Sets up PID controllers for X, Y, and Z axes
 */
void hoverControlInit(void);

/**
 * Enable or disable hover control
 * @param enable true to enable, false to disable
 */
void hoverControlEnable(bool enable);

/**
 * Set hover target position
 * @param x Target X position (cm)
 * @param y Target Y position (cm) 
 * @param height Target height (cm)
 */
void hoverControlSetTarget(float x, float y, float height);

/**
 * Main hover control update function
 * @param flow Optical flow measurement data
 * @param tof TOF distance measurement data
 * @param setpoint Control setpoint to be updated
 * @param state Current drone state
 * @param dt Time step (seconds)
 * @param height Backup height input (cm)
 */
void hoverControlUpdate(flowMeasurement_t* flow, tofMeasurement_t* tof, 
                       setpoint_t* setpoint, state_t* state, float dt, float height);

/**
 * Reset hover control state
 * Clears position estimates and resets PID controllers
 */
void hoverControlReset(void);

/**
 * Check if hover control is active
 * @return true if hover control is enabled
 */
bool hoverControlIsActive(void);

/**
 * Get current hover control state
 * @return Pointer to hover control state structure
 */
hoverControl_t* hoverControlGetState(void);

/**
 * Adjust Z-axis PID controller gains
 * @param kp Proportional gain
 * @param ki Integral gain  
 * @param kd Derivative gain
 */
void hoverControlSetZPidGains(float kp, float ki, float kd);

#endif // HOVER_CONTROL_H