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

// VL53L1X registers
#define VL53L1X_SOFT_RESET               0x0000
#define VL53L1X_I2C_SLAVE__DEVICE_ADDRESS 0x0001
#define VL53L1X_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND 0x0008
#define VL53L1X_VHV_CONFIG__INIT         0x000B
#define VL53L1X_GLOBAL_CONFIG__SPAD_ENABLES_REF_0 0x000D
#define VL53L1X_GLOBAL_CONFIG__SPAD_ENABLES_REF_1 0x000E
#define VL53L1X_GLOBAL_CONFIG__SPAD_ENABLES_REF_2 0x000F
#define VL53L1X_GLOBAL_CONFIG__SPAD_ENABLES_REF_3 0x0010
#define VL53L1X_GLOBAL_CONFIG__SPAD_ENABLES_REF_4 0x0011
#define VL53L1X_GLOBAL_CONFIG__SPAD_ENABLES_REF_5 0x0012
#define VL53L1X_GLOBAL_CONFIG__REF_EN_START_SELECT 0x0013
#define VL53L1X_REF_SPAD_MAN__NUM_REQUESTED_REF_SPADS 0x0014
#define VL53L1X_REF_SPAD_MAN__REF_LOCATION 0x0015
#define VL53L1X_ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS 0x0016
#define VL53L1X_ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS 0x0018
#define VL53L1X_ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS 0x001A
#define VL53L1X_REF_SPAD_CHAR__TOTAL_RATE_TARGET_MCPS 0x001C
#define VL53L1X_ALGO__PART_TO_PART_RANGE_OFFSET_MM 0x001E
#define VL53L1X_MM_CONFIG__INNER_OFFSET_MM 0x0020
#define VL53L1X_MM_CONFIG__OUTER_OFFSET_MM 0x0022
#define VL53L1X_GPIO_HV_MUX__CTRL        0x0030
#define VL53L1X_GPIO__TIO_HV_STATUS      0x0031
#define VL53L1X_SYSTEM__INTERRUPT_CONFIG_GPIO 0x0046
#define VL53L1X_PHASECAL_CONFIG__TIMEOUT_MACROP 0x004B
#define VL53L1X_RANGE_CONFIG__TIMEOUT_MACROP_A_HI 0x005E
#define VL53L1X_RANGE_CONFIG__VCSEL_PERIOD_A 0x0060
#define VL53L1X_RANGE_CONFIG__TIMEOUT_MACROP_B_HI 0x0061
#define VL53L1X_RANGE_CONFIG__VCSEL_PERIOD_B 0x0063
#define VL53L1X_RANGE_CONFIG__SIGMA_THRESH 0x0064
#define VL53L1X_RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS 0x0066
#define VL53L1X_RANGE_CONFIG__VALID_PHASE_LOW 0x0069
#define VL53L1X_RANGE_CONFIG__VALID_PHASE_HIGH 0x006A
#define VL53L1X_SYSTEM__INTERMEASUREMENT_PERIOD 0x006C
#define VL53L1X_SYSTEM__THRESH_HIGH 0x0072
#define VL53L1X_SYSTEM__THRESH_LOW 0x0074
#define VL53L1X_SD_CONFIG__WOI_SD0 0x0078
#define VL53L1X_SD_CONFIG__INITIAL_PHASE_SD0 0x007A
#define VL53L1X_ROI_CONFIG__USER_ROI_CENTRE_SPAD 0x007F
#define VL53L1X_ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE 0x0080
#define VL53L1X_SYSTEM__SEQUENCE_CONFIG 0x0081
#define VL53L1X_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND 0x0008
#define VL53L1X_SYSTEM__GROUPED_PARAMETER_HOLD 0x0082
#define VL53L1X_POWER_MANAGEMENT__GO1_POWER_FORCE 0x0083
#define VL53L1X_SYSTEM__STREAM_COUNT_CTRL 0x0084
#define VL53L1X_FIRMWARE__ENABLE 0x0085
#define VL53L1X_SYSTEM__INTERRUPT_CLEAR 0x0086
#define VL53L1X_SYSTEM__MODE_START 0x0087
#define VL53L1X_RESULT__RANGE_STATUS 0x0089
#define VL53L1X_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0 0x008C
#define VL53L1X_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0 0x008E
#define VL53L1X_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0 0x0090
#define VL53L1X_RESULT__SIGMA_SD0 0x0092
#define VL53L1X_RESULT__PHASE_SD0 0x0094
#define VL53L1X_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 0x0096
#define VL53L1X_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0 0x0098
#define VL53L1X_RESULT__OSC_CALIBRATE_VAL 0x009A
#define VL53L1X_FIRMWARE__SYSTEM_STATUS 0x00E5
#define VL53L1X_IDENTIFICATION__MODEL_ID 0x010F
#define VL53L1X_ROI_CONFIG__MODE_ROI_CENTRE_SPAD 0x013E

// Check if this sensor is supported
#if TOF_SENSOR_ENABLED

static bool isInit = false;
static bool isPresent = false;
static bool isVL53L1X = false;

// VL53L1X default configuration (based on Python reference)
static const uint8_t VL53L1X_DEFAULT_CONFIG[] = {
    0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01,
    0x01, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x20, 0x0B, 0x00, 0x00, 0x02, 0x0A, 0x21, 0x00, 0x00, 0x05, 0x00,
    0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08, 0x00,
    0x00, 0x01, 0xDB, 0x0F, 0x01, 0xF1, 0x0D, 0x01, 0x68, 0x00, 0x80, 0x08,
    0xB8, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x0F, 0x0D, 0x0E, 0x0E, 0x00, 0x00, 0x02, 0xC7, 0xFF,
    0x9B, 0x00, 0x00, 0x00, 0x01, 0x01, 0x40
};

// Helper functions for 16-bit register access
static bool writeReg16_8(uint16_t reg, uint8_t value) {
    return i2cdevWriteReg16(I2C_TOF_DEV, VL53LXX_I2C_ADDR, reg, 1, &value);
}

static bool writeReg16_16(uint16_t reg, uint16_t value) {
    uint8_t data[2] = {(value >> 8) & 0xFF, value & 0xFF};
    return i2cdevWriteReg16(I2C_TOF_DEV, VL53LXX_I2C_ADDR, reg, 2, data);
}

static bool readReg16_8(uint16_t reg, uint8_t* value) {
    return i2cdevReadReg16(I2C_TOF_DEV, VL53LXX_I2C_ADDR, reg, 1, value);
}

static bool readReg16_16(uint16_t reg, uint16_t* value) {
    uint8_t data[2];
    if (i2cdevReadReg16(I2C_TOF_DEV, VL53LXX_I2C_ADDR, reg, 2, data)) {
        *value = (data[0] << 8) | data[1];
        return true;
    }
    return false;
}

static bool vl53l1x_reset(void) {
    if (!writeReg16_8(VL53L1X_SOFT_RESET, 0x00)) return false;
    vTaskDelay(pdMS_TO_TICKS(100));
    if (!writeReg16_8(VL53L1X_SOFT_RESET, 0x01)) return false;
    return true;
}

static bool vl53l1x_init(void) {
    // Reset sensor
    if (!vl53l1x_reset()) {
        ESP_LOGE(TAG, "VL53L1X reset failed");
        return false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Check model ID
    uint16_t model_id;
    if (!readReg16_16(VL53L1X_IDENTIFICATION__MODEL_ID, &model_id)) {
        ESP_LOGE(TAG, "Failed to read VL53L1X model ID");
        return false;
    }
    
    if (model_id != 0xEACC) {
        ESP_LOGE(TAG, "VL53L1X model ID mismatch: expected 0xEACC, got 0x%04X", model_id);
        return false;
    }
    
    // Write default configuration
    for (int i = 0; i < sizeof(VL53L1X_DEFAULT_CONFIG); i++) {
        if (!writeReg16_8(0x002D + i, VL53L1X_DEFAULT_CONFIG[i])) {
            ESP_LOGE(TAG, "Failed to write default config at offset %d", i);
            return false;
        }
    }
    
    // Additional initialization based on Python code
    uint16_t reg_0x0022_value;
    if (!readReg16_16(0x0022, &reg_0x0022_value)) {
        ESP_LOGE(TAG, "Failed to read register 0x0022");
        return false;
    }
    
    if (!writeReg16_16(0x001E, reg_0x0022_value * 4)) {
        ESP_LOGE(TAG, "Failed to write register 0x001E");
        return false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(200));
    
    ESP_LOGI(TAG, "VL53L1X initialized successfully");
    return true;
}

// Function to test if sensor is physically present
static bool testSensorPresence(void)
{
    uint8_t whoAmI = 0;
    
    // Try VL53L0X first - 使用专用TOF I2C总线
    if (i2cdevReadByte(I2C_TOF_DEV, VL53LXX_I2C_ADDR, VL53L0X_WHO_AM_I, &whoAmI)) {
        if (whoAmI == VL53L0X_EXPECTED_ID) {
            ESP_LOGI(TAG, "VL53L0X TOF sensor detected (ID: 0x%02X)", whoAmI);
            isVL53L1X = false;
            return true;
        }
    }
    
    // Try VL53L1X (different register layout) - 使用专用TOF I2C总线
    uint16_t modelId = 0;
    if (readReg16_16(VL53L1X_IDENTIFICATION__MODEL_ID, &modelId)) {
        if (modelId == 0xEACC) {
            ESP_LOGI(TAG, "VL53L1X TOF sensor detected (Model ID: 0x%04X)", modelId);
            isVL53L1X = true;
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
        if (isVL53L1X) {
            // Initialize VL53L1X sensor
            if (vl53l1x_init()) {
                ESP_LOGI(TAG, "VL53L1X TOF sensor initialized successfully");
            } else {
                ESP_LOGE(TAG, "VL53L1X initialization failed");
                isPresent = false;
            }
        } else {
            // VL53L0X initialization would go here
            ESP_LOGI(TAG, "VL53L0X TOF sensor detected but initialization not implemented");
            isPresent = false;
        }
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

    if (isVL53L1X) {
        // Read VL53L1X measurement data (17 bytes starting from 0x0089)
        uint8_t data[17];
        if (!i2cdevReadReg16(I2C_TOF_DEV, VL53LXX_I2C_ADDR, VL53L1X_RESULT__RANGE_STATUS, 17, data)) {
            ESP_LOGW(TAG, "Failed to read VL53L1X measurement data");
            return false;
        }
        
        // Extract distance from bytes 13-14 (final_crosstalk_corrected_range_mm_sd0)
        uint16_t distance_mm = (data[13] << 8) | data[14];
        
        // Extract sigma (standard deviation) from bytes 9-10  
        uint16_t sigma_raw = (data[9] << 8) | data[10];
        float sigma_mm = sigma_raw / 65536.0f; // Convert from 16.16 fixed point
        
        // Get current timestamp
        tof->timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
        tof->distance = (float)distance_mm;
        tof->stdDev = sigma_mm;
        
        return true;
    } else {
        // VL53L0X reading would go here
        ESP_LOGW(TAG, "VL53L0X reading not implemented");
        return false;
    }
}

bool tofSensorIsPresent(void)
{
    return isPresent;
}

void tofSensorDeInit(void)
{
    isInit = false;
    isPresent = false;
    isVL53L1X = false;
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
