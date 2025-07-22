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
#include "sensors_mpu6050_spl06.h"

static const char* TAG = "optical_flow";

// PMW3901 optical flow sensor SPI configuration (from pmw3901.py)
#define PMW3901_WHO_AM_I        0x00
#define PMW3901_EXPECTED_ID     0x49
#define PMW3901_CHIP_ID_INVERSE 0x5F
#define PMW3901_EXPECTED_ID_INV 0xB6

// PMW3901 数据寄存器 (from pmw3901.py)
#define PMW3901_MOTION_LATCH    0x02  // 运动寄存器锁存器
#define PMW3901_DELTA_X_L       0x03  // Delta_X 低字节
#define PMW3901_DELTA_X_H       0x04  // Delta_X 高字节
#define PMW3901_DELTA_Y_L       0x05  // Delta_Y 低字节
#define PMW3901_DELTA_Y_H       0x06  // Delta_Y 高字节

// PMW3901 控制寄存器 (from pmw3901.py)
#define PMW3901_POWER_RESET     0x3A  // 电源复位寄存器 (写入0x5A执行复位)
#define PMW3901_PAGE_SELECT     0x7F  // 页面选择寄存器

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
    uint8_t tx_data[2] = {reg & 0x7F, 0x00};  // 读取命令 (清除MSB)
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

// SPI写入函数 (根据pmw3901.py)
static bool pmw3901_spi_write_reg(uint8_t reg, uint8_t data)
{
    spi_transaction_t trans = {0};
    uint8_t tx_data[2] = {reg | 0x80, data};  // 写入命令 (设置MSB)
    
    trans.length = 16;  // 16位 (2字节)
    trans.tx_buffer = tx_data;
    
    esp_err_t ret = spi_device_transmit(pmw3901_spi, &trans);
    return (ret == ESP_OK);
}

// PMW3901初始化寄存器序列 (从pmw3901.py完整复制)
static void pmw3901_init_registers(void)
{
    pmw3901_spi_write_reg(0x7F, 0x00);
    pmw3901_spi_write_reg(0x61, 0xAD);
    pmw3901_spi_write_reg(0x7F, 0x03);
    pmw3901_spi_write_reg(0x40, 0x00);
    pmw3901_spi_write_reg(0x7F, 0x05);
    pmw3901_spi_write_reg(0x41, 0xB3);
    pmw3901_spi_write_reg(0x43, 0xF1);
    pmw3901_spi_write_reg(0x45, 0x14);
    pmw3901_spi_write_reg(0x5B, 0x32);
    pmw3901_spi_write_reg(0x5F, 0x34);
    pmw3901_spi_write_reg(0x7B, 0x08);
    pmw3901_spi_write_reg(0x7F, 0x06);
    pmw3901_spi_write_reg(0x44, 0x1B);
    pmw3901_spi_write_reg(0x40, 0xBF);
    pmw3901_spi_write_reg(0x4E, 0x3F);
    pmw3901_spi_write_reg(0x7F, 0x08);
    pmw3901_spi_write_reg(0x65, 0x20);
    pmw3901_spi_write_reg(0x6A, 0x18);
    pmw3901_spi_write_reg(0x7F, 0x09);
    pmw3901_spi_write_reg(0x4F, 0xAF);
    pmw3901_spi_write_reg(0x5F, 0x40);
    pmw3901_spi_write_reg(0x48, 0x80);
    pmw3901_spi_write_reg(0x49, 0x80);
    pmw3901_spi_write_reg(0x57, 0x77);
    pmw3901_spi_write_reg(0x7F, 0x0A);
    pmw3901_spi_write_reg(0x42, 0x60);
    pmw3901_spi_write_reg(0x7F, 0x0B);
    pmw3901_spi_write_reg(0x41, 0xDA);
    pmw3901_spi_write_reg(0x45, 0x17);
    pmw3901_spi_write_reg(0x5F, 0x50);
    pmw3901_spi_write_reg(0x7B, 0x08);
    pmw3901_spi_write_reg(0x7F, 0x0C);
    pmw3901_spi_write_reg(0x44, 0x96);
    pmw3901_spi_write_reg(0x5B, 0x65);
    pmw3901_spi_write_reg(0x7F, 0x0D);
    pmw3901_spi_write_reg(0x48, 0x6A);
    pmw3901_spi_write_reg(0x6F, 0x39);
    pmw3901_spi_write_reg(0x7F, 0x0E);
    pmw3901_spi_write_reg(0x40, 0x48);
    pmw3901_spi_write_reg(0x41, 0xDD);
    pmw3901_spi_write_reg(0x6E, 0x8A);
    pmw3901_spi_write_reg(0x7F, 0x0F);
    pmw3901_spi_write_reg(0x5D, 0x73);
    pmw3901_spi_write_reg(0x7F, 0x10);
    pmw3901_spi_write_reg(0x40, 0x19);
    pmw3901_spi_write_reg(0x7F, 0x11);
    pmw3901_spi_write_reg(0x40, 0x40);
    pmw3901_spi_write_reg(0x7F, 0x12);
    pmw3901_spi_write_reg(0x6D, 0x1C);
    pmw3901_spi_write_reg(0x71, 0x23);
    pmw3901_spi_write_reg(0x7F, 0x13);
    pmw3901_spi_write_reg(0x40, 0x20);
    pmw3901_spi_write_reg(0x6E, 0x34);
    pmw3901_spi_write_reg(0x7F, 0x14);
    pmw3901_spi_write_reg(0x40, 0x41);
    pmw3901_spi_write_reg(0x42, 0xC1);
    pmw3901_spi_write_reg(0x74, 0x2B);
    pmw3901_spi_write_reg(0x7F, 0x15);
    pmw3901_spi_write_reg(0x40, 0x90);
    pmw3901_spi_write_reg(0x7F, 0x00);
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
    
    // 执行电源复位 (根据pmw3901.py)
    pmw3901_spi_write_reg(PMW3901_POWER_RESET, 0x5A);
    vTaskDelay(pdMS_TO_TICKS(5)); // 等待5ms
    
    // Try to read WHO_AM_I register from PMW3901 via SPI
    uint8_t chipIdInverse;
    if (pmw3901_spi_read_reg(PMW3901_WHO_AM_I, &whoAmI) &&
        pmw3901_spi_read_reg(PMW3901_CHIP_ID_INVERSE, &chipIdInverse)) {
        
        if (whoAmI == PMW3901_EXPECTED_ID && chipIdInverse == PMW3901_EXPECTED_ID_INV) {
            ESP_LOGI(TAG, "PMW3901 optical flow sensor detected via SPI (ID: 0x%02X, INV: 0x%02X)", 
                     whoAmI, chipIdInverse);
            
            // 清除运动寄存器 (根据pmw3901.py)
            uint8_t dummy;
            pmw3901_spi_read_reg(PMW3901_MOTION_LATCH, &dummy);
            pmw3901_spi_read_reg(PMW3901_DELTA_X_L, &dummy);
            pmw3901_spi_read_reg(PMW3901_DELTA_X_H, &dummy);
            pmw3901_spi_read_reg(PMW3901_DELTA_Y_L, &dummy);
            pmw3901_spi_read_reg(PMW3901_DELTA_Y_H, &dummy);
            vTaskDelay(pdMS_TO_TICKS(1)); // 等待1ms
            
            // 执行完整的传感器初始化序列 (从pmw3901.py)
            pmw3901_init_registers();
            ESP_LOGI(TAG, "PMW3901 initialization complete");
            
            return true;
        } else {
            ESP_LOGW(TAG, "Wrong chip ID from PMW3901: expected 0x%02X/0x%02X, got 0x%02X/0x%02X", 
                     PMW3901_EXPECTED_ID, PMW3901_EXPECTED_ID_INV, whoAmI, chipIdInverse);
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

    // 读取PMW3901传感器数据 (按照pmw3901.py的方法)
    uint8_t delta_x_l, delta_x_h;
    uint8_t delta_y_l, delta_y_h;
    
    // 锁存运动寄存器 (必须先读取0x02来锁存数据)
    uint8_t motion_latch;
    if (!pmw3901_spi_read_reg(PMW3901_MOTION_LATCH, &motion_latch)) {
        return false;
    }
    
    // 读取X和Y位移数据 (16位有符号整数)
    if (!pmw3901_spi_read_reg(PMW3901_DELTA_X_L, &delta_x_l) ||
        !pmw3901_spi_read_reg(PMW3901_DELTA_X_H, &delta_x_h) ||
        !pmw3901_spi_read_reg(PMW3901_DELTA_Y_L, &delta_y_l) ||
        !pmw3901_spi_read_reg(PMW3901_DELTA_Y_H, &delta_y_h)) {
        return false;
    }
    
    // 添加调试信息输出原始寄存器值
    static uint32_t debug_counter = 0;
    if (++debug_counter % 250 == 0) { // 每秒打印一次 (250Hz)
        char debug_str[128];
        snprintf(debug_str, sizeof(debug_str), "PMW3901 Raw: latch=0x%02X, xL=0x%02X, xH=0x%02X, yL=0x%02X, yH=0x%02X\n",
                motion_latch, delta_x_l, delta_x_h, delta_y_l, delta_y_h);
        debugpeintf(debug_str);
    }
    
    // 组合16位有符号数据 (按照pmw3901.py的方法)
    int16_t delta_x = (int16_t)((delta_x_h << 8) | delta_x_l);
    int16_t delta_y = (int16_t)((delta_y_h << 8) | delta_y_l);
    
    // 处理有符号数转换 (按照pmw3901.py的方法)
    if (delta_x & 0x8000) {
        delta_x -= 0x10000;
    }
    if (delta_y & 0x8000) {
        delta_y -= 0x10000;
    }
    
    // 转换为像素位移 (PMW3901输出单位已经是像素)
    flow->dpixelx = (float)delta_x;
    flow->dpixely = (float)delta_y;
    
    // 设置固定的标准差值
    flow->stdDevX = 0.5f;
    flow->stdDevY = 0.5f;
    
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
