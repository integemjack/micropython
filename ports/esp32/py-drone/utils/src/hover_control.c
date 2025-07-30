/**
 * hover_control.c - Implementation of hover/position hold using optical flow and TOF
 * Enhanced with Z-axis PID control
 */

#include "hover_control.h"
#include "position_pid.h"
#include "maths.h"
#include "commander.h"
#include "esp_log.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.h"  // 添加 config.h 引用，其中包含 DEG2RAD 定义
#include "sensors_mpu6050_spl06.h"  // 包含debugpeintf()声明

static const char* TAG = "hover_control";

// Hover control parameters
#define FLOW_SCALE_FACTOR   0.1f    // Scale factor for optical flow to cm
#define HEIGHT_FILTER_ALPHA 0.2f    // Low pass filter for height measurement
#define POSITION_DECAY      0.995f  // Position estimate decay factor
#define HEIGHT_VEL_FILTER_ALPHA 0.3f // Filter for height velocity estimation
#define MIN_TOF_DISTANCE    10.0f   // Minimum valid TOF distance (cm)
#define MAX_TOF_DISTANCE    400.0f  // Maximum valid TOF distance (cm)

static hoverControl_t hoverState = {
    .enabled = false,
    .targetX = 0.0f,
    .targetY = 0.0f,
    .targetHeight = 80.0f,  // Default 80cm height
    .posX = 0.0f,
    .posY = 0.0f,
    .currentHeight = 80.0f,
    .heightVelocity = 0.0f,
    .lastHeight = 80.0f,
    .lastUpdateTime = 0
};

// PID controllers for hover (X, Y, Z)
static PidObject hoverPidX;
static PidObject hoverPidY;
static PidObject hoverPidZ;    // 新增Z轴PID控制器
static bool isPidInit = false;

void hoverControlInit(void)
{
    if (!isPidInit) {
        // Initialize hover PIDs with conservative gains
        pidInit(&hoverPidX, 0, (pidInit_t){.kp = 0.5f, .ki = 0.01f, .kd = 0.1f}, 0.01f);
        pidInit(&hoverPidY, 0, (pidInit_t){.kp = 0.5f, .ki = 0.01f, .kd = 0.1f}, 0.01f);
        
        // Initialize Z-axis PID with different gains optimized for height control
        pidInit(&hoverPidZ, 0, (pidInit_t){.kp = 0.8f, .ki = 0.05f, .kd = 0.2f}, 0.01f);
        
        // Set output limits 
        pidSetOutputLimit(&hoverPidX, 50.0f);  // velocity in cm/s
        pidSetOutputLimit(&hoverPidY, 50.0f);  // velocity in cm/s
        pidSetOutputLimit(&hoverPidZ, 10.0f);  // vertical velocity in cm/s
        
        isPidInit = true;
        ESP_LOGI(TAG, "Hover control initialized with Z-axis PID");
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
                       setpoint_t* setpoint, state_t* state, float dt, float height)
{
    if (!hoverState.enabled) {
        return;
    }
    
    uint32_t currentTime = xTaskGetTickCount();
    
    // Update position estimate from optical flow
    if (flow && flow->dt > 0.001f) {  // 确保dt有效，避免除零错误
        // Convert flow measurements to velocity
        float flowVelX = flow->dpixelx / flow->dt * FLOW_SCALE_FACTOR;
        float flowVelY = flow->dpixely / flow->dt * FLOW_SCALE_FACTOR;
        
        // Compensate for drone rotation
        float cosYaw = cosf(state->attitude.yaw * DEG2RAD);
        float sinYaw = sinf(state->attitude.yaw * DEG2RAD);
        
        float velX = flowVelX * cosYaw - flowVelY * sinYaw;
        float velY = flowVelX * sinYaw + flowVelY * cosYaw;
        
        // Integrate velocity to get position
        hoverState.posX += velX * dt;
        hoverState.posY += velY * dt;
        
        // Apply decay to account for drift
        hoverState.posX *= POSITION_DECAY;
        hoverState.posY *= POSITION_DECAY;
    }
    
    // Enhanced height control with TOF sensor
    bool heightUpdated = false;
    if (tof && tof->distance > 0) {
        // Convert mm to cm and validate range
        float measuredHeight = tof->distance / 10.0f;
        
        // Validate TOF measurement range
        if (measuredHeight >= MIN_TOF_DISTANCE && measuredHeight <= MAX_TOF_DISTANCE) {
            // Apply low-pass filter to height measurement
            hoverState.currentHeight = hoverState.currentHeight * (1.0f - HEIGHT_FILTER_ALPHA) + 
                                     measuredHeight * HEIGHT_FILTER_ALPHA;
            
            // Estimate height velocity using filtered difference
            float heightDiff = hoverState.currentHeight - hoverState.lastHeight;
            float instantVel = (dt > 0.001f) ? heightDiff / dt : 0.0f;
            
            // Filter height velocity to reduce noise
            hoverState.heightVelocity = hoverState.heightVelocity * (1.0f - HEIGHT_VEL_FILTER_ALPHA) +
                                      instantVel * HEIGHT_VEL_FILTER_ALPHA;
            
            hoverState.lastHeight = hoverState.currentHeight;
            heightUpdated = true;
            
            // Update state position for compatibility
            state->position.z = hoverState.currentHeight;
        }
    }
    
    // Calculate position errors for X and Y
    float errorX = hoverState.targetX - hoverState.posX;
    float errorY = hoverState.targetY - hoverState.posY;
    
    // Calculate height error for Z
    float errorZ = hoverState.targetHeight - hoverState.currentHeight;
    
    // Update velocity setpoints using PID controllers
    float velCmdX = 0.12f * pidUpdate(&hoverPidX, errorX);
    float velCmdY = 0.12f * pidUpdate(&hoverPidY, errorY);
    
    // Z-axis PID control - output is vertical velocity command
    float velCmdZ = 0.0f;
    if (heightUpdated) {
        velCmdZ = pidUpdate(&hoverPidZ, errorZ);
        
        // Add velocity feedforward for smoother control
        velCmdZ += 0.1f * hoverState.heightVelocity;
    }
    
    // Set control modes and commands
    // XY: Velocity control mode
    setpoint->mode.x = modeVelocity;
    setpoint->mode.y = modeVelocity;
    setpoint->velocity.x = velCmdX;
    setpoint->velocity.y = velCmdY;
    
    // Z: Velocity control mode (changed from absolute position)
    setpoint->mode.z = modeVelocity;
    setpoint->velocity.z = velCmdZ;
    
    // Alternative: Use thrust mode for more direct control
    // setpoint->mode.z = modeManual;
    // setpoint->thrust = calculateThrustFromVelocity(velCmdZ, state);
    
    // Update timestamp
    hoverState.lastUpdateTime = currentTime;
    
    // Periodic logging for debugging
    static uint32_t lastLogTime = 0;
    if (currentTime - lastLogTime > 2000) {  // Log every 2 seconds
        ESP_LOGD(TAG, "Hover: Pos(%.1f,%.1f,%.1f) Target(%.1f,%.1f,%.1f) VelCmd(%.2f,%.2f,%.2f)", 
                 hoverState.posX, hoverState.posY, hoverState.currentHeight,
                 hoverState.targetX, hoverState.targetY, hoverState.targetHeight,
                 velCmdX, velCmdY, velCmdZ);
        lastLogTime = currentTime;
    }
}

void hoverControlReset(void)
{
    hoverState.posX = 0.0f;
    hoverState.posY = 0.0f;
    hoverState.currentHeight = 80.0f;  // Reset to default height
    hoverState.heightVelocity = 0.0f;
    hoverState.lastHeight = 80.0f;
    hoverState.lastUpdateTime = xTaskGetTickCount();
    
    // Reset all PIDs including Z-axis
    if (isPidInit) {
        pidReset(&hoverPidX);
        pidReset(&hoverPidY);
        pidReset(&hoverPidZ);  // Reset Z-axis PID
    }
    
    ESP_LOGI(TAG, "Hover control state reset");
}

bool hoverControlIsActive(void)
{
    return hoverState.enabled;
}

// New function to get current hover state
hoverControl_t* hoverControlGetState(void)
{
    return &hoverState;
}

// New function to adjust Z-axis PID parameters
void hoverControlSetZPidGains(float kp, float ki, float kd)
{
    if (isPidInit) {
        pidInit_t gains = {.kp = kp, .ki = ki, .kd = kd};
        pidInit(&hoverPidZ, 0, gains, 0.01f);
        pidSetOutputLimit(&hoverPidZ, 30.0f);
        ESP_LOGI(TAG, "Z-axis PID gains updated: kp=%.3f, ki=%.3f, kd=%.3f", kp, ki, kd);
    }
}