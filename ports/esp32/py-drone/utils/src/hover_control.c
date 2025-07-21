/**
 * hover_control.c - Implementation of hover/position hold using optical flow and TOF
 */

#include "hover_control.h"
#include "position_pid.h"
#include "maths.h"
#include "commander.h"
#include "esp_log.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "hover_control";

// Hover control parameters
#define FLOW_SCALE_FACTOR   0.1f    // Scale factor for optical flow to cm
#define HEIGHT_FILTER_ALPHA 0.2f    // Low pass filter for height measurement
#define POSITION_DECAY      0.995f  // Position estimate decay factor

static hoverControl_t hoverState = {
    .enabled = false,
    .targetX = 0.0f,
    .targetY = 0.0f,
    .targetHeight = 100.0f,  // Default 100cm height
    .posX = 0.0f,
    .posY = 0.0f,
    .lastUpdateTime = 0
};

// PID controllers for hover
static PidObject hoverPidX;
static PidObject hoverPidY;
static bool isPidInit = false;

void hoverControlInit(void)
{
    if (!isPidInit) {
        // Initialize hover PIDs with conservative gains
        pidInit(&hoverPidX, 0, (pidInit_t){.kp = 0.5f, .ki = 0.01f, .kd = 0.1f}, 0.01f);
        pidInit(&hoverPidY, 0, (pidInit_t){.kp = 0.5f, .ki = 0.01f, .kd = 0.1f}, 0.01f);
        
        // Set output limits (velocity in cm/s)
        pidSetOutputLimit(&hoverPidX, 50.0f);
        pidSetOutputLimit(&hoverPidY, 50.0f);
        
        isPidInit = true;
        ESP_LOGI(TAG, "Hover control initialized");
    }
}

void hoverControlEnable(bool enable)
{
    if (enable && !hoverState.enabled) {
        // Reset state when enabling
        hoverControlReset();
        ESP_LOGI(TAG, "Hover control enabled");
    } else if (!enable && hoverState.enabled) {
        ESP_LOGI(TAG, "Hover control disabled");
    }
    
    hoverState.enabled = enable;
}

void hoverControlSetTarget(float x, float y, float height)
{
    hoverState.targetX = x;
    hoverState.targetY = y;
    hoverState.targetHeight = height;
    
    ESP_LOGI(TAG, "Hover target set to: X=%.1f, Y=%.1f, Height=%.1f cm", 
             x, y, height);
}

void hoverControlUpdate(flowMeasurement_t* flow, tofMeasurement_t* tof, 
                       setpoint_t* setpoint, state_t* state, float dt)
{
    if (!hoverState.enabled) {
        return;
    }
    
    uint32_t currentTime = xTaskGetTickCount();
    
    // Update position estimate from optical flow
    if (flow && flow->dt > 0) {
        // Convert flow measurements to velocity
        float flowVelX = flow->dpixelx / flow->dt * FLOW_SCALE_FACTOR;
        float flowVelY = flow->dpixely / flow->dt * FLOW_SCALE_FACTOR;
        
        // Compensate for drone rotation
        float cosYaw = cosf(state->attitude.yaw * DEG_TO_RAD);
        float sinYaw = sinf(state->attitude.yaw * DEG_TO_RAD);
        
        float velX = flowVelX * cosYaw - flowVelY * sinYaw;
        float velY = flowVelX * sinYaw + flowVelY * cosYaw;
        
        // Integrate velocity to get position
        hoverState.posX += velX * dt;
        hoverState.posY += velY * dt;
        
        // Apply decay to account for drift
        hoverState.posX *= POSITION_DECAY;
        hoverState.posY *= POSITION_DECAY;
    }
    
    // Update height from TOF sensor
    float targetHeight = hoverState.targetHeight;
    if (tof && tof->distance > 0) {
        // Convert mm to cm and apply low-pass filter
        float measuredHeight = tof->distance / 10.0f;
        state->position.z = state->position.z * (1.0f - HEIGHT_FILTER_ALPHA) + 
                           measuredHeight * HEIGHT_FILTER_ALPHA;
    }
    
    // Calculate position errors
    float errorX = hoverState.targetX - hoverState.posX;
    float errorY = hoverState.targetY - hoverState.posY;
    
    // Update velocity setpoints using PID
    float velCmdX = pidUpdate(&hoverPidX, errorX);
    float velCmdY = pidUpdate(&hoverPidY, errorY);
    
    // Set velocity mode for X and Y
    setpoint->mode.x = modeVelocity;
    setpoint->mode.y = modeVelocity;
    setpoint->velocity.x = velCmdX;
    setpoint->velocity.y = velCmdY;
    
    // Set absolute mode for Z (height)
    setpoint->mode.z = modeAbs;
    setpoint->position.z = targetHeight;
    
    // Update timestamp
    hoverState.lastUpdateTime = currentTime;
    
    // Log periodically for debugging
    static uint32_t lastLogTime = 0;
    if (currentTime - lastLogTime > 1000) {  // Log every second
        ESP_LOGD(TAG, "Hover: Pos(%.1f,%.1f) Target(%.1f,%.1f) Vel(%.1f,%.1f) Height:%.1f", 
                 hoverState.posX, hoverState.posY,
                 hoverState.targetX, hoverState.targetY,
                 velCmdX, velCmdY,
                 state->position.z);
        lastLogTime = currentTime;
    }
}

void hoverControlReset(void)
{
    hoverState.posX = 0.0f;
    hoverState.posY = 0.0f;
    hoverState.lastUpdateTime = xTaskGetTickCount();
    
    // Reset PIDs
    if (isPidInit) {
        pidReset(&hoverPidX);
        pidReset(&hoverPidY);
    }
    
    ESP_LOGI(TAG, "Hover control state reset");
}

bool hoverControlIsActive(void)
{
    return hoverState.enabled;
}
