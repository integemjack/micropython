#include "sensor_config.h"
/**
 * optical_flow.c - Optical flow sensor implementation
 */

#include "optical_flow.h"
#include "pmw3901_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sensors_mpu6050_spl06.h"
#include "stabilizer.h"
#include <math.h>

static const char* TAG = "optical_flow";

// SPI针脚定义 (匹配根目录ESP32配置)
#define PMW3901_SPI_SCK     18   // GPIO 18
#define PMW3901_SPI_MOSI    21   // GPIO 21  
#define PMW3901_SPI_MISO    17   // GPIO 17
#define PMW3901_SPI_CS      8    // GPIO 8

// Check if this sensor is supported
#if OPTICAL_FLOW_SENSOR_ENABLED

static bool isInit = false;
static bool isPresent = false;


// Function to test if sensor is physically present
static bool testSensorPresence(void)
{
    char debug_str[128];
    
    ESP_LOGI(TAG, "Checking for optical flow sensor...");
    
    // Initialize PMW3901 driver with hardware pins
    esp_err_t ret = pmw3901_init(PMW3901_SPI_SCK, PMW3901_SPI_MOSI, PMW3901_SPI_MISO, PMW3901_SPI_CS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize PMW3901 driver");
        snprintf(debug_str, sizeof(debug_str), "✗ Failed to initialize PMW3901 driver: %s\n", esp_err_to_name(ret));
        debugpeintf(debug_str);
        return false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Begin sensor initialization
    ret = pmw3901_begin();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PMW3901 sensor not detected or failed to initialize");
        snprintf(debug_str, sizeof(debug_str), "✗ PMW3901 sensor not detected: %s\n", esp_err_to_name(ret));
        debugpeintf(debug_str);
        return false;
    }
    
    // Check surface quality for diagnostic (减少输出以避免阻塞)
    uint8_t squal;
    if (pmw3901_read_surface_quality(&squal) == ESP_OK) {
        ESP_LOGI(TAG, "Initial surface quality: %d", squal);
    }
    
    debugpeintf("✓ Optical flow sensor initialization completed successfully\n");
    ESP_LOGI(TAG, "✓ Optical flow sensor initialization completed successfully");
    
    return true;
}

bool opticalFlowInit(void)
{
    if (isInit) {
        return isPresent;
    }

    ESP_LOGI(TAG, "Initializing optical flow sensor...");
    
    // Test if sensor is physically present
    isPresent = testSensorPresence();
    
    if (isPresent) {
        // TODO: Add actual PMW3901 initialization sequence here
        // For now, just confirm detection
        ESP_LOGI(TAG, "Optical flow sensor initialized successfully");
        debugpeintf("Optical flow sensor initialized successfully\n");
    } else {
        ESP_LOGW(TAG, "Optical flow sensor not found - continuing without it");
        debugpeintf("Optical flow sensor not found - continuing without it\n");
    }

    isInit = true;
    return isPresent;
}

bool opticalFlowTest(void)
{
    return isPresent;
}

bool opticalFlowReadMeasurement(flowMeasurement_t* flow)
{
    if (!isPresent || !flow) {
        debugpeintf("Optical flow sensor not present or flow measurement not available\n");
        return false;
    }

    int16_t delta_x, delta_y;
    esp_err_t ret = pmw3901_read_motion_count(&delta_x, &delta_y);
    if (ret != ESP_OK) {
        char debug_str[256];
        snprintf(debug_str, sizeof(debug_str), "Failed to read motion count: %s\n", esp_err_to_name(ret));
        debugpeintf(debug_str);
        return false;
    }
    
    // Get current attitude (pitch and roll) for compensation
    attitude_t attitude;
    getAttitudeData(&attitude);
    
    // Convert angles from degrees to radians for trigonometric functions
    float pitch_rad = -attitude.pitch * M_PI / 180.0f;
    float roll_rad = -attitude.roll * M_PI / 180.0f;
    
    // Apply pitch and roll compensation
    // The optical flow sensor measures motion in the sensor frame, 
    // but we want the motion in the world frame.
    // When the drone is tilted, we need to compensate for this.
    // The compensation accounts for both the scaling and rotation effects of tilt.
    
    // Calculate trigonometric values
    float cos_pitch = cosf(pitch_rad);
    float sin_pitch = sinf(pitch_rad);
    float cos_roll = cosf(roll_rad);
    float sin_roll = sinf(roll_rad);
    
    // Protect against division by zero or very small numbers
    if (fabsf(cos_roll) < 0.05f) {
        cos_roll = 0.05f * (cos_roll >= 0 ? 1 : -1);  // Limit to minimum 0.05
    }
    if (fabsf(cos_pitch) < 0.05f) {
        cos_pitch = 0.05f * (cos_pitch >= 0 ? 1 : -1);  // Limit to minimum 0.05
    }
    
    // Compensate for the tilt using a more accurate transformation
    // This accounts for both the scaling and rotation effects of tilt
    float compensated_delta_x = ((float)delta_x + (float)delta_y * sin_roll) / cos_roll;
    float compensated_delta_y = ((float)delta_y - (float)delta_x * sin_pitch) / cos_pitch;
    
    // Convert to flow measurement
    flow->dpixelx = compensated_delta_x;
    flow->dpixely = compensated_delta_y;
    flow->dt = 0.004f; // 250Hz update rate
    
    // Set standard deviation (simplified)
    flow->stdDevX = 0.5f;
    flow->stdDevY = 0.5f;
    
    return true;
}

bool opticalFlowIsPresent(void)
{
    return isPresent;
}

void opticalFlowDeInit(void)
{
    if (isPresent) {
        pmw3901_deinit();
    }
    isInit = false;
    isPresent = false;
    ESP_LOGI(TAG, "Optical flow sensor deinitialized");
    debugpeintf("Optical flow sensor deinitialized\n");
}

#else // OPTICAL_FLOW_SENSOR_ENABLED

// Stub functions when sensor is disabled in configuration
bool opticalFlowInit(void) { 
    ESP_LOGI(TAG, "Optical flow sensor disabled in configuration");
    debugpeintf("Optical flow sensor disabled in configuration\n");
    return false; 
}
bool opticalFlowTest(void) { return false; }
bool opticalFlowReadMeasurement(flowMeasurement_t* flow) { (void)flow; return false; }
bool opticalFlowIsPresent(void) { return false; }
void opticalFlowDeInit(void) { }

#endif // OPTICAL_FLOW_SENSOR_ENABLED
