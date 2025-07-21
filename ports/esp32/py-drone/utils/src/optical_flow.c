#include "sensor_config.h"
/**
 * optical_flow.c - Optical flow sensor implementation
 */

#include "optical_flow.h"
#include "../../drivers/i2c_bus/include/i2cdev.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "optical_flow";

// PMW3901 optical flow sensor I2C address
#define PMW3901_I2C_ADDR    0x42
#define PMW3901_WHO_AM_I    0x00
#define PMW3901_EXPECTED_ID 0x49

// Check if this sensor is supported
#if OPTICAL_FLOW_SENSOR_ENABLED

static bool isInit = false;
static bool isPresent = false;

// Function to test if sensor is physically present
static bool testSensorPresence(void)
{
    uint8_t whoAmI = 0;
    
    // Try to read WHO_AM_I register from PMW3901
    // Use correct i2cdev API
    if (i2cdevReadByte(I2C0_DEV, PMW3901_I2C_ADDR, PMW3901_WHO_AM_I, &whoAmI)) {
        if (whoAmI == PMW3901_EXPECTED_ID) {
            ESP_LOGI(TAG, "PMW3901 optical flow sensor detected (ID: 0x%02X)", whoAmI);
            return true;
        } else {
            ESP_LOGW(TAG, "Unknown optical flow sensor (ID: 0x%02X)", whoAmI);
            return false;
        }
    }
    
    // If I2C communication fails, sensor is not present
    ESP_LOGW(TAG, "No response from optical flow sensor at address 0x%02X", PMW3901_I2C_ADDR);
    return false;
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
    } else {
        ESP_LOGW(TAG, "Optical flow sensor not found - continuing without it");
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
        return false;
    }

    // TODO: Read actual sensor data
    // For now, provide dummy data only if sensor is present
    flow->dpixelx = 0.0f;
    flow->dpixely = 0.0f;
    flow->stdDevX = 0.0f;
    flow->stdDevY = 0.0f;
    flow->dt = 0.004f; // 250Hz update rate
    
    return true;
}

bool opticalFlowIsPresent(void)
{
    return isPresent;
}

void opticalFlowDeInit(void)
{
    isInit = false;
    isPresent = false;
    ESP_LOGI(TAG, "Optical flow sensor deinitialized");
}

#else // OPTICAL_FLOW_SENSOR_ENABLED

// Stub functions when sensor is disabled in configuration
bool opticalFlowInit(void) { 
    ESP_LOGI(TAG, "Optical flow sensor disabled in configuration");
    return false; 
}
bool opticalFlowTest(void) { return false; }
bool opticalFlowReadMeasurement(flowMeasurement_t* flow) { (void)flow; return false; }
bool opticalFlowIsPresent(void) { return false; }
void opticalFlowDeInit(void) { }

#endif // OPTICAL_FLOW_SENSOR_ENABLED
