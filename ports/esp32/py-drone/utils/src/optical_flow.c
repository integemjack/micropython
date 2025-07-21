#include "sensor_config.h"
/**
 * optical_flow.c - Optical flow sensor implementation
 */

#include "optical_flow.h"
#include "driver/spi_master.h"
#include "driver/gpio.h" 
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "optical_flow";

// PMW3901 optical flow sensor SPI configuration
#define PMW3901_WHO_AM_I    0x00
#define PMW3901_EXPECTED_ID 0x49

// SPI针脚定义 (直接硬编码，匹配根目录ESP32配置)
#define PMW3901_SPI_SCK     18   // GPIO 18
#define PMW3901_SPI_MOSI    21   // GPIO 21  
#define PMW3901_SPI_MISO    17   // GPIO 17
#define PMW3901_SPI_CS      8    // GPIO 8

#define PMW3901_SPI_FREQ    2000000  // 2MHz SPI频率

// Check if this sensor is supported
#if OPTICAL_FLOW_SENSOR_ENABLED

static bool isInit = false;
static bool isPresent = false;
static spi_device_handle_t pmw3901_spi;

// SPI读取函数
static bool pmw3901_spi_read_reg(uint8_t reg, uint8_t* data)
{
    spi_transaction_t trans = {0};
    uint8_t tx_data[2] = {reg, 0x00};  // 读取命令
    uint8_t rx_data[2] = {0};
    
    trans.length = 16;  // 16位 (2字节)
    trans.tx_buffer = tx_data;
    trans.rx_buffer = rx_data;
    
    esp_err_t ret = spi_device_transmit(pmw3901_spi, &trans);
    if (ret == ESP_OK) {
        *data = rx_data[1];  // 第二个字节是数据
        return true;
    }
    return false;
}

// SPI初始化函数
static bool pmw3901_spi_init(void)
{
    spi_bus_config_t bus_cfg = {
        .miso_io_num = PMW3901_SPI_MISO,
        .mosi_io_num = PMW3901_SPI_MOSI,
        .sclk_io_num = PMW3901_SPI_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = PMW3901_SPI_FREQ,
        .mode = 3,  // PMW3901使用SPI模式3
        .spics_io_num = PMW3901_SPI_CS,
        .queue_size = 1,
    };
    
    // 初始化SPI总线
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {  // ESP_ERR_INVALID_STATE表示总线已初始化
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return false;
    }
    
    // 添加设备到SPI总线
    ret = spi_bus_add_device(SPI2_HOST, &dev_cfg, &pmw3901_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return false;
    }
    
    return true;
}

// Function to test if sensor is physically present
static bool testSensorPresence(void)
{
    uint8_t whoAmI = 0;
    
    // 首先初始化SPI
    if (!pmw3901_spi_init()) {
        ESP_LOGE(TAG, "Failed to initialize SPI for PMW3901");
        return false;
    }
    
    // 等待传感器启动
    vTaskDelay(50 / portTICK_PERIOD_MS);
    
    // Try to read WHO_AM_I register from PMW3901 via SPI
    if (pmw3901_spi_read_reg(PMW3901_WHO_AM_I, &whoAmI)) {
        if (whoAmI == PMW3901_EXPECTED_ID) {
            ESP_LOGI(TAG, "PMW3901 optical flow sensor detected via SPI (ID: 0x%02X)", whoAmI);
            return true;
        } else {
            ESP_LOGW(TAG, "Unknown optical flow sensor via SPI (ID: 0x%02X)", whoAmI);
            return false;
        }
    }
    
    // If SPI communication fails, sensor is not present
    ESP_LOGW(TAG, "No response from PMW3901 optical flow sensor via SPI");
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
    if (isPresent && pmw3901_spi) {
        spi_bus_remove_device(pmw3901_spi);
        spi_bus_free(SPI2_HOST);  // 注意：只有在确定没有其他设备使用时才释放总线
    }
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
