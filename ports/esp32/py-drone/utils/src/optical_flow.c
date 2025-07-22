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
    debugpeintf("Checking for optical flow sensor...\n");
    
    // Initialize PMW3901 driver with hardware pins
    esp_err_t ret = pmw3901_init(PMW3901_SPI_SCK, PMW3901_SPI_MOSI, PMW3901_SPI_MISO, PMW3901_SPI_CS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize PMW3901 driver");
        snprintf(debug_str, sizeof(debug_str), "✗ Failed to initialize PMW3901 driver: %s\n", esp_err_to_name(ret));
        debugpeintf(debug_str);
        return false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(4000));
    
    // Begin sensor initialization
    ret = pmw3901_begin();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PMW3901 sensor not detected or failed to initialize");
        snprintf(debug_str, sizeof(debug_str), "✗ PMW3901 sensor not detected: %s\n", esp_err_to_name(ret));
        debugpeintf(debug_str);
        return false;
    }
    
    // Check surface quality for diagnostic
    uint8_t squal;
    if (pmw3901_read_surface_quality(&squal) == ESP_OK) {
        snprintf(debug_str, sizeof(debug_str), "Initial surface quality: %d (>20 is good, >80 is excellent)\n", squal);
        debugpeintf(debug_str);
        if (squal < 20) {
            debugpeintf("  Warning: Low surface quality may indicate:\n");
            debugpeintf("  - Sensor too high above surface (optimal: 8-120mm)\n");
            debugpeintf("  - Poor surface texture (needs non-reflective patterns)\n");
            debugpeintf("  - Insufficient lighting\n");
        }
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
    debugpeintf("Initializing optical flow sensor...\n");
    
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
    
    // Convert to flow measurement
    flow->dpixelx = (float)delta_x;
    flow->dpixely = (float)delta_y;
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
