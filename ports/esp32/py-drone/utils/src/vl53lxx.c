#include "sensor_config.h"
/**
 * vl53lxx.c - VL53L0X/VL53L1X TOF sensor implementation
 */

#include "vl53lxx.h"
#include "../../drivers/i2c_bus/include/i2cdev.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "tof_sensor";

// VL53L0X/VL53L1X I2C address
#define VL53LXX_I2C_ADDR         0x29
#define VL53L0X_WHO_AM_I         0xC0
#define VL53L0X_EXPECTED_ID      0xEE
#define VL53L1X_WHO_AM_I         0x010F
#define VL53L1X_EXPECTED_ID      0xEA

// Check if this sensor is supported
#if TOF_SENSOR_ENABLED

static bool isInit = false;
static bool isPresent = false;

// Function to test if sensor is physically present
static bool testSensorPresence(void)
{
    uint8_t whoAmI = 0;
    
    // Try VL53L0X first - 使用专用TOF I2C总线
    if (i2cdevReadByte(I2C_TOF_DEV, VL53LXX_I2C_ADDR, VL53L0X_WHO_AM_I, &whoAmI)) {
        if (whoAmI == VL53L0X_EXPECTED_ID) {
            ESP_LOGI(TAG, "VL53L0X TOF sensor detected (ID: 0x%02X)", whoAmI);
            return true;
        }
    }
    
    // Try VL53L1X (different register layout) - 使用专用TOF I2C总线
    uint16_t modelId = 0;
    if (i2cdevReadReg16(I2C_TOF_DEV, VL53LXX_I2C_ADDR, VL53L1X_WHO_AM_I, 1, (uint8_t*)&modelId)) {
        if ((modelId & 0xFF) == VL53L1X_EXPECTED_ID) {
            ESP_LOGI(TAG, "VL53L1X TOF sensor detected (ID: 0x%02X)", modelId & 0xFF);
            return true;
        }
    }
    
    // If I2C communication fails, sensor is not present
    ESP_LOGW(TAG, "No response from TOF sensor at address 0x%02X", VL53LXX_I2C_ADDR);
    return false;
}

bool tofSensorInit(void)
{
    if (isInit) {
        return isPresent;
    }

    ESP_LOGI(TAG, "Initializing TOF sensor...");
    
    // Test if sensor is physically present
    isPresent = testSensorPresence();
    
    if (isPresent) {
        // TODO: Add actual VL53L0X/VL53L1X initialization sequence here
        ESP_LOGI(TAG, "TOF sensor initialized successfully");
    } else {
        ESP_LOGW(TAG, "TOF sensor not found - continuing without it");
    }

    isInit = true;
    return isPresent;
}

bool tofSensorTest(void)
{
    return isPresent;
}

bool tofSensorReadMeasurement(tofMeasurement_t* tof)
{
    if (!isPresent || !tof) {
        return false;
    }

    // TODO: Read actual sensor data
    // For now, provide dummy data only if sensor is present
    tof->timestamp = 0; // Should be actual timestamp
    tof->distance = 100.0f; // Default distance in mm
    tof->stdDev = 1.0f; // Standard deviation
    
    return true;
}

bool tofSensorIsPresent(void)
{
    return isPresent;
}

void tofSensorDeInit(void)
{
    isInit = false;
    isPresent = false;
    ESP_LOGI(TAG, "TOF sensor deinitialized");
}

#else // TOF_SENSOR_ENABLED

// Stub functions when sensor is disabled in configuration
bool tofSensorInit(void) { 
    ESP_LOGI(TAG, "TOF sensor disabled in configuration");
    return false; 
}
bool tofSensorTest(void) { return false; }
bool tofSensorReadMeasurement(tofMeasurement_t* tof) { (void)tof; return false; }
bool tofSensorIsPresent(void) { return false; }
void tofSensorDeInit(void) { }

#endif // TOF_SENSOR_ENABLED
