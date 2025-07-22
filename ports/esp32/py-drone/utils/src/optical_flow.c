#include "sensor_config.h"
/**
 * optical_flow.c - Optical flow sensor implementation
 */

#include "optical_flow.h"
#include "driver/spi_master.h"
#include "driver/gpio.h" 
#include "esp_log.h"
#include "esp_rom_sys.h"  // for esp_rom_delay_us
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sensors_mpu6050_spl06.h"

static const char* TAG = "optical_flow";

// 光流传感器SPI配置 (支持多种型号)
#define OPTICAL_FLOW_WHO_AM_I        0x00
#define PMW3901_EXPECTED_ID          0x49  // PMW3901
#define PAA5100_EXPECTED_ID          0x07  // PAA5100 或兼容传感器
#define PMW3901_CHIP_ID_INVERSE      0x5F
#define PMW3901_EXPECTED_ID_INV      0xB6

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

#define PMW3901_SPI_FREQ    500000   // 降低到500KHz SPI频率，提高稳定性

// Check if this sensor is supported
#if OPTICAL_FLOW_SENSOR_ENABLED

static bool isInit = false;
static bool isPresent = false;
static spi_device_handle_t pmw3901_spi;

// SPI读取函数 (完全按照pmw3901.py的手动CS控制实现)
static bool pmw3901_spi_read_reg(uint8_t reg, uint8_t* data)
{
    esp_err_t ret;
    uint8_t tx_addr = reg & 0x7F;  // 清除写标志
    uint8_t rx_data = 0;
    
    // 手动控制CS - 拉低开始通信 (对应Python: self.cs.value(0))
    gpio_set_level(PMW3901_SPI_CS, 0);
    esp_rom_delay_us(50);  // 对应Python: utime.sleep_us(50)
    
    // 发送寄存器地址 (对应Python: self.spi.write(bytearray([reg])))
    spi_transaction_t trans1 = {0};
    trans1.length = 8;
    trans1.tx_buffer = &tx_addr;
    
    ret = spi_device_transmit(pmw3901_spi, &trans1);
    if (ret != ESP_OK) {
        gpio_set_level(PMW3901_SPI_CS, 1);  // 出错时释放CS
        return false;
    }
    
    esp_rom_delay_us(50);  // 对应Python: utime.sleep_us(50)
    
    // 读取数据 (对应Python: result = self.spi.read(1))
    spi_transaction_t trans2 = {0};
    trans2.length = 8;
    trans2.rx_buffer = &rx_data;
    
    ret = spi_device_transmit(pmw3901_spi, &trans2);
    
    esp_rom_delay_us(100); // 对应Python: utime.sleep_us(100)
    
    // 手动释放CS (对应Python: self.cs.value(1))
    gpio_set_level(PMW3901_SPI_CS, 1);
    
    if (ret == ESP_OK) {
        *data = rx_data;
        
        // 添加详细的SPI调试信息
        static uint32_t spi_debug_counter = 0;
        if (++spi_debug_counter % 1000 == 0) { // 每4秒打印一次
            char debug_str[80];
            snprintf(debug_str, sizeof(debug_str), "SPI Read: reg=0x%02X -> data=0x%02X\n", reg, rx_data);
            debugpeintf(debug_str);
        }
        
        return true;
    }
    return false;
}

// SPI写入函数 (完全按照pmw3901.py的手动CS控制实现)
static bool pmw3901_spi_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t tx_data[2] = {reg | 0x80, data};  // 写入命令 (设置MSB)
    
    // 手动控制CS - 拉低开始通信 (对应Python: self.cs.value(0))
    gpio_set_level(PMW3901_SPI_CS, 0);
    esp_rom_delay_us(50);  // 对应Python: utime.sleep_us(50)
    
    // 同时发送地址和数据 (对应Python: self.spi.write(bytearray([reg, value])))
    spi_transaction_t trans = {0};
    trans.length = 16;  // 16位 (2字节)
    trans.tx_buffer = tx_data;
    
    esp_err_t ret = spi_device_transmit(pmw3901_spi, &trans);
    
    esp_rom_delay_us(50);  // 对应Python: utime.sleep_us(50)
    
    // 手动释放CS (对应Python: self.cs.value(1))
    gpio_set_level(PMW3901_SPI_CS, 1);
    
    esp_rom_delay_us(200); // 对应Python: utime.sleep_us(200)
    
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
        .mode = 3,  // PMW3901使用SPI模式3 (CPOL=1, CPHA=1)
        .spics_io_num = -1,     // 禁用自动CS控制，改用手动控制
        .queue_size = 1,
        .flags = 0,
        .duty_cycle_pos = 128,  // 50% duty cycle
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
    char debug_str[128];  // 统一的调试字符串缓冲区
    
    // 首先初始化SPI
    if (!pmw3901_spi_init()) {
        ESP_LOGE(TAG, "Failed to initialize SPI for PMW3901");
        snprintf(debug_str, sizeof(debug_str), "Failed to initialize SPI for PMW3901\n");
        debugpeintf(debug_str);
        return false;
    }
    
    // 等待传感器启动 - 增加更长的启动延时
    ESP_LOGI(TAG, "Waiting for PMW3901 startup (100ms)...");
    snprintf(debug_str, sizeof(debug_str), "Waiting for PMW3901 startup (100ms)...");
    debugpeintf(debug_str);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 尝试多种不同的SPI模式来排查
    ESP_LOGI(TAG, "Testing different SPI modes...");
    for (int mode = 0; mode <= 3; mode++) {
        ESP_LOGI(TAG, "Testing SPI mode %d", mode);
        snprintf(debug_str, sizeof(debug_str), "Testing SPI mode %d\n", mode);
        debugpeintf(debug_str);
        
        // 重新配置SPI模式
        spi_bus_remove_device(pmw3901_spi);
        spi_device_interface_config_t test_cfg = {
            .clock_speed_hz = 100000,  // 超低频率测试
            .mode = mode,
            .spics_io_num = -1,
            .queue_size = 1,
        };
        spi_bus_add_device(SPI2_HOST, &test_cfg, &pmw3901_spi);
        
        uint8_t test_val;
        if (pmw3901_spi_read_reg(OPTICAL_FLOW_WHO_AM_I, &test_val)) {
            ESP_LOGI(TAG, "Mode %d result: 0x%02X", mode, test_val);
            snprintf(debug_str, sizeof(debug_str), "Mode %d result: 0x%02X\n", mode, test_val);
            debugpeintf(debug_str);
            if (test_val == PMW3901_EXPECTED_ID) {
                ESP_LOGI(TAG, "Found correct chip ID with mode %d!", mode);
                snprintf(debug_str, sizeof(debug_str), "Found correct chip ID with mode %d!\n", mode);
                debugpeintf(debug_str);
                break;
            }
        }
    }
    
    // 恢复到模式3和原始频率
    spi_bus_remove_device(pmw3901_spi);
    spi_device_interface_config_t restore_cfg = {
        .clock_speed_hz = PMW3901_SPI_FREQ,
        .mode = 3,
        .spics_io_num = -1,
        .queue_size = 1,
    };
    spi_bus_add_device(SPI2_HOST, &restore_cfg, &pmw3901_spi);
    
    // 执行电源复位 (根据pmw3901.py)
    ESP_LOGI(TAG, "Sending power-on reset...");
    pmw3901_spi_write_reg(PMW3901_POWER_RESET, 0x5A);
    vTaskDelay(pdMS_TO_TICKS(50)); // 增加复位后延时到50ms
    
    // 尝试读取WHO_AM_I寄存器来验证传感器存在
    ESP_LOGI(TAG, "Reading WHO_AM_I register (0x%02X)...", OPTICAL_FLOW_WHO_AM_I);
    snprintf(debug_str, sizeof(debug_str), "Reading WHO_AM_I register (0x%02X)...\n", OPTICAL_FLOW_WHO_AM_I);
    debugpeintf(debug_str);
    
    uint8_t chipIdInverse;
    bool read1_success = pmw3901_spi_read_reg(OPTICAL_FLOW_WHO_AM_I, &whoAmI);
    bool read2_success = pmw3901_spi_read_reg(PMW3901_CHIP_ID_INVERSE, &chipIdInverse);
    
    snprintf(debug_str, sizeof(debug_str), "Read results: WHO_AM_I=%s (0x%02X), ID_INV=%s (0x%02X)\n", 
             read1_success ? "SUCCESS" : "FAILED", whoAmI,
             read2_success ? "SUCCESS" : "FAILED", chipIdInverse);
    debugpeintf(debug_str);
    
    if (read1_success && read2_success) {
        snprintf(debug_str, sizeof(debug_str), "Chip verify: WHO_AM_I=0x%02X (exp 0x%02X), ID_INV=0x%02X (exp 0x%02X)\n",
                 whoAmI, PMW3901_EXPECTED_ID, chipIdInverse, PMW3901_EXPECTED_ID_INV);
        debugpeintf(debug_str);
        
        if (whoAmI == PMW3901_EXPECTED_ID && chipIdInverse == PMW3901_EXPECTED_ID_INV) {
            snprintf(debug_str, sizeof(debug_str), "✓ PMW3901 optical flow sensor verified successfully!\n");
            debugpeintf(debug_str);
            
        // 清除运动寄存器 (兼容所有光流传感器)
        snprintf(debug_str, sizeof(debug_str), "Clearing motion registers...\n");
        debugpeintf(debug_str);
        uint8_t dummy;
        pmw3901_spi_read_reg(PMW3901_MOTION_LATCH, &dummy);
        pmw3901_spi_read_reg(PMW3901_DELTA_X_L, &dummy);
        pmw3901_spi_read_reg(PMW3901_DELTA_X_H, &dummy);
        pmw3901_spi_read_reg(PMW3901_DELTA_Y_L, &dummy);
        pmw3901_spi_read_reg(PMW3901_DELTA_Y_H, &dummy);
        vTaskDelay(pdMS_TO_TICKS(1)); // 等待1ms
        
        // 根据传感器类型选择初始化
        if (whoAmI == PMW3901_EXPECTED_ID) {
            ESP_LOGI(TAG, "Executing PMW3901 initialization sequence...");
            pmw3901_init_registers();
        } else {
            ESP_LOGI(TAG, "Skipping complex initialization for non-PMW3901 sensor");
        }
        
        snprintf(debug_str, sizeof(debug_str), "✓ Optical flow sensor initialization completed successfully\n");
        debugpeintf(debug_str);
        ESP_LOGI(TAG, "✓ Optical flow sensor initialization completed successfully");
        
        return true;
    } else {
            // 这个分支现在不应该被执行，因为上面已经处理了所有情况
            snprintf(debug_str, sizeof(debug_str), "✗ Unexpected sensor state\n");
            debugpeintf(debug_str);
            return false;
        }
    }
    
    // SPI通信失败
    snprintf(debug_str, sizeof(debug_str), "✗ SPI communication failure - cannot read PMW3901 registers\n");
    debugpeintf(debug_str);
    snprintf(debug_str, sizeof(debug_str), "  This could indicate:\n");
    debugpeintf(debug_str);
    snprintf(debug_str, sizeof(debug_str), "  1. Hardware connection issues (SPI wiring)\n");
    debugpeintf(debug_str);
    snprintf(debug_str, sizeof(debug_str), "  2. Power supply problems\n");
    debugpeintf(debug_str);
    snprintf(debug_str, sizeof(debug_str), "  3. SPI configuration mismatch\n");
    debugpeintf(debug_str);
    snprintf(debug_str, sizeof(debug_str), "  4. GPIO pin conflicts with other peripherals\n");
    debugpeintf(debug_str);
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
