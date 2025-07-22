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
#define PMW3901_SQUAL           0x07  // Surface Quality Register
#define PMW3901_RAW_DATA_SUM    0x08  // Raw Data Sum Register
#define PMW3901_MAX_RAW_DATA    0x09  // Maximum Raw Data Register
#define PMW3901_MIN_RAW_DATA    0x0A  // Minimum Raw Data Register

// PMW3901 控制寄存器 (from pmw3901.py)
#define PMW3901_POWER_RESET     0x3A  // 电源复位寄存器 (写入0x5A执行复位)
#define PMW3901_PAGE_SELECT     0x7F  // 页面选择寄存器

// SPI针脚定义 (直接硬编码，匹配根目录ESP32配置)
#define PMW3901_SPI_SCK     18   // GPIO 18
#define PMW3901_SPI_MOSI    21   // GPIO 21  
#define PMW3901_SPI_MISO    17   // GPIO 17
#define PMW3901_SPI_CS      8    // GPIO 8

#define PMW3901_SPI_FREQ    4000000  // 4MHz SPI频率，匹配Python实现

// Check if this sensor is supported
#if OPTICAL_FLOW_SENSOR_ENABLED

static bool isInit = false;
static bool isPresent = false;
static spi_device_handle_t pmw3901_spi;

// SPI读取函数 (完全按照pmw3901.py的手动CS控制实现)
static bool pmw3901_spi_read_reg(uint8_t reg, uint8_t* data)
{
    if (pmw3901_spi == NULL) {
        ESP_LOGE(TAG, "SPI device not initialized");
        return false;
    }
    
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
    
    // 检查SPI通信错误
    if (ret != ESP_OK) {
        char debug_str[100];
        snprintf(debug_str, sizeof(debug_str), "SPI Read ERROR: reg=0x%02X, ret=0x%02X\n", reg, ret);
        debugpeintf(debug_str);
        return false;
    }
    
    *data = rx_data;
    
    // 添加详细的SPI调试信息
    static uint32_t spi_debug_counter = 0;
    if (++spi_debug_counter % 1000 == 0) { // 每4秒打印一次
        char debug_str[120];
        snprintf(debug_str, sizeof(debug_str), "SPI Read: reg=0x%02X -> data=0x%02X (CS=%d, Handle=0x%08X)\n", 
                reg, rx_data, gpio_get_level(PMW3901_SPI_CS), (unsigned int)pmw3901_spi);
        debugpeintf(debug_str);
    }
    
    return true;
}

// SPI写入函数 (完全按照pmw3901.py的手动CS控制实现)
static bool pmw3901_spi_write_reg(uint8_t reg, uint8_t data)
{
    if (pmw3901_spi == NULL) {
        ESP_LOGE(TAG, "SPI device not initialized");
        return false;
    }
    
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
    
    // 添加SPI写入调试信息
    if (ret != ESP_OK) {
        char debug_str[100];
        snprintf(debug_str, sizeof(debug_str), "SPI Write ERROR: reg=0x%02X, data=0x%02X, ret=0x%02X\n", reg & 0x7F, data, ret);
        debugpeintf(debug_str);
    }
    
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
    // 首先配置所有SPI引脚
    esp_err_t ret;
    
    // 配置CS引脚作为输出GPIO
    gpio_config_t cs_cfg = {
        .pin_bit_mask = (1ULL << PMW3901_SPI_CS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    
    ret = gpio_config(&cs_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure CS GPIO: %s", esp_err_to_name(ret));
        return false;
    }
    
    // 设置CS为高电平（空闲状态）
    gpio_set_level(PMW3901_SPI_CS, 1);
    
    ESP_LOGI(TAG, "SPI pins configured - SCK:%d, MOSI:%d, MISO:%d, CS:%d", 
             PMW3901_SPI_SCK, PMW3901_SPI_MOSI, PMW3901_SPI_MISO, PMW3901_SPI_CS);
    
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
    
    // 初始化SPI总线 - 使用SPI3_HOST避免冲突
    ret = spi_bus_initialize(SPI3_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return false;
    }
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "SPI3_HOST already initialized, continuing...");
    }
    
    // 添加设备到SPI总线
    ret = spi_bus_add_device(SPI3_HOST, &dev_cfg, &pmw3901_spi);
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
    
    // 添加try-catch风格的错误处理
    ESP_LOGI(TAG, "Starting PMW3901 sensor detection...");
    
    // 输出系统环境信息用于调试
    snprintf(debug_str, sizeof(debug_str), "\n=== SYSTEM ENVIRONMENT INFO ===\n");
    debugpeintf(debug_str);
    snprintf(debug_str, sizeof(debug_str), "SPI Configuration: Host=SPI3, Freq=%dHz, Mode=3\n", PMW3901_SPI_FREQ);
    debugpeintf(debug_str);
    snprintf(debug_str, sizeof(debug_str), "GPIO Pins: SCK=%d, MOSI=%d, MISO=%d, CS=%d\n", 
            PMW3901_SPI_SCK, PMW3901_SPI_MOSI, PMW3901_SPI_MISO, PMW3901_SPI_CS);
    debugpeintf(debug_str);
    snprintf(debug_str, sizeof(debug_str), "Free Heap: %d bytes\n", esp_get_free_heap_size());
    debugpeintf(debug_str);
    snprintf(debug_str, sizeof(debug_str), "System Tick: %d ms\n", (int)(xTaskGetTickCount() * portTICK_PERIOD_MS));
    debugpeintf(debug_str);
    snprintf(debug_str, sizeof(debug_str), "=== END SYSTEM INFO ===\n\n");
    debugpeintf(debug_str);
    
    // 首先初始化SPI
    if (!pmw3901_spi_init()) {
        ESP_LOGE(TAG, "Failed to initialize SPI for PMW3901");
        snprintf(debug_str, sizeof(debug_str), "Failed to initialize SPI for PMW3901\n");
        debugpeintf(debug_str);
        return false;
    }
    
    ESP_LOGI(TAG, "SPI initialization completed successfully");
    
    // 等待传感器启动 - 匹配Python的4秒启动延时
    ESP_LOGI(TAG, "Waiting for PMW3901 startup (4000ms)...");
    snprintf(debug_str, sizeof(debug_str), "Waiting for PMW3901 startup (4000ms)...\n");
    debugpeintf(debug_str);
    vTaskDelay(pdMS_TO_TICKS(4000));
    
    // 执行SPI总线复位序列 (根据Python pmw3901.py的begin()方法)
    ESP_LOGI(TAG, "Performing SPI bus reset...");
    gpio_set_level(PMW3901_SPI_CS, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(PMW3901_SPI_CS, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(PMW3901_SPI_CS, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // 电源复位 (根据pmw3901.py)
    ESP_LOGI(TAG, "Sending power-on reset...");
    pmw3901_spi_write_reg(PMW3901_POWER_RESET, 0x5A);
    vTaskDelay(pdMS_TO_TICKS(5)); // Python使用5ms
    
    // 尝试读取WHO_AM_I寄存器来验证传感器存在
    ESP_LOGI(TAG, "Reading WHO_AM_I register (0x%02X)...", OPTICAL_FLOW_WHO_AM_I);
    snprintf(debug_str, sizeof(debug_str), "Reading WHO_AM_I register (0x%02X)...\n", OPTICAL_FLOW_WHO_AM_I);
    debugpeintf(debug_str);
    
    uint8_t chipIdInverse, revisionId;
    bool read1_success = pmw3901_spi_read_reg(OPTICAL_FLOW_WHO_AM_I, &whoAmI);
    bool read2_success = pmw3901_spi_read_reg(0x01, &revisionId);  // 读取修订版本ID
    bool read3_success = pmw3901_spi_read_reg(PMW3901_CHIP_ID_INVERSE, &chipIdInverse);
    
    snprintf(debug_str, sizeof(debug_str), "Register verification:\n");
    debugpeintf(debug_str);
    snprintf(debug_str, sizeof(debug_str), "  Product ID: %s (0x%02X) - expected 0x49\n", 
             read1_success ? "SUCCESS" : "FAILED", whoAmI);
    debugpeintf(debug_str);
    snprintf(debug_str, sizeof(debug_str), "  Revision ID: %s (0x%02X)\n", 
             read2_success ? "SUCCESS" : "FAILED", revisionId);
    debugpeintf(debug_str);
    snprintf(debug_str, sizeof(debug_str), "  Inverse Product ID: %s (0x%02X) - expected 0xB6\n", 
             read3_success ? "SUCCESS" : "FAILED", chipIdInverse);
    debugpeintf(debug_str);
    
    if (read1_success) {
        // 检查是否为PMW3901 (匹配Python的验证逻辑)
        if (whoAmI == PMW3901_EXPECTED_ID && read3_success && chipIdInverse == PMW3901_EXPECTED_ID_INV) {
            snprintf(debug_str, sizeof(debug_str), "✓ PMW3901 verified: Product=0x%02X, Revision=0x%02X, Inverse=0x%02X\n", 
                    whoAmI, revisionId, chipIdInverse);
            debugpeintf(debug_str);
            ESP_LOGI(TAG, "Executing PMW3901 initialization sequence...");
            pmw3901_init_registers();
        } 
        // 检查是否为PAA5100或兼容型号
        else if (whoAmI == PAA5100_EXPECTED_ID) {
            snprintf(debug_str, sizeof(debug_str), "✓ PAA5100/compatible sensor verified: WHO_AM_I=0x%02X\n", whoAmI);
            debugpeintf(debug_str);
            ESP_LOGI(TAG, "PAA5100/compatible sensor detected, skipping PMW3901-specific init.");
        }
        // 如果两个都不是，则识别失败
        else {
            snprintf(debug_str, sizeof(debug_str), "✗ Chip ID mismatch - Not a valid PMW3901/PAA5100 sensor\n");
            debugpeintf(debug_str);
            snprintf(debug_str, sizeof(debug_str), "  Expected: WHO_AM_I=0x%02X (PMW) or 0x%02X (PAA)\n", PMW3901_EXPECTED_ID, PAA5100_EXPECTED_ID);
            debugpeintf(debug_str);
            snprintf(debug_str, sizeof(debug_str), "  Got:      WHO_AM_I=0x%02X, ID_INV=0x%02X\n", whoAmI, chipIdInverse);
            debugpeintf(debug_str);
            
            // 检查是否是常见的错误情况
            if (whoAmI == 0xFF || whoAmI == 0x00) {
                snprintf(debug_str, sizeof(debug_str), "  Note: 0x%02X suggests SPI line stuck or no pullup\n", whoAmI);
                debugpeintf(debug_str);
            } else if (whoAmI == 0x7F) {
                snprintf(debug_str, sizeof(debug_str), "  Note: 0x7F may indicate timing issues or wrong SPI mode\n");
                debugpeintf(debug_str);
            }
            
            snprintf(debug_str, sizeof(debug_str), "✗ Optical flow sensor not available - system will continue without it\n");
            debugpeintf(debug_str);
            return false;
        }

        // 清除运动寄存器 (按照Python顺序和时序)
        snprintf(debug_str, sizeof(debug_str), "Clearing motion registers...\n");
        debugpeintf(debug_str);
        uint8_t dummy;
        pmw3901_spi_read_reg(0x02, &dummy);  // Motion latch
        pmw3901_spi_read_reg(0x03, &dummy);  // Delta X low
        pmw3901_spi_read_reg(0x04, &dummy);  // Delta X high
        pmw3901_spi_read_reg(0x05, &dummy);  // Delta Y low  
        pmw3901_spi_read_reg(0x06, &dummy);  // Delta Y high
        vTaskDelay(pdMS_TO_TICKS(1));        // Python用1ms延时
        
        // 读取初始表面质量以便诊断
        uint8_t initial_squal;
        if (pmw3901_spi_read_reg(PMW3901_SQUAL, &initial_squal)) {
            snprintf(debug_str, sizeof(debug_str), "Initial surface quality: %d (>20 is good, >80 is excellent)\n", initial_squal);
            debugpeintf(debug_str);
            if (initial_squal < 20) {
                snprintf(debug_str, sizeof(debug_str), "  Warning: Low surface quality may indicate:\n");
                debugpeintf(debug_str);
                snprintf(debug_str, sizeof(debug_str), "  - Sensor too high above surface (optimal: 8-120mm)\n");
                debugpeintf(debug_str);
                snprintf(debug_str, sizeof(debug_str), "  - Poor surface texture (needs non-reflective patterns)\n");
                debugpeintf(debug_str);
                snprintf(debug_str, sizeof(debug_str), "  - Insufficient lighting\n");
                debugpeintf(debug_str);
            }
        }

        snprintf(debug_str, sizeof(debug_str), "✓ Optical flow sensor initialization completed successfully\n");
        debugpeintf(debug_str);
        ESP_LOGI(TAG, "✓ Optical flow sensor initialization completed successfully");
        
        return true;
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

    // === 完全按照Python的read_motion_count()方法实现 ===
    
    // 1. 锁存运动寄存器 (读取0x02来锁存数据)
    uint8_t motion_latch;
    if (!pmw3901_spi_read_reg(0x02, &motion_latch)) {
        return false;
    }
    
    // 2. 按照Python的精确顺序读取运动数据
    uint8_t delta_x_h, delta_x_l, delta_y_h, delta_y_l;
    if (!pmw3901_spi_read_reg(0x04, &delta_x_h) ||  // DX high first (Python order)
        !pmw3901_spi_read_reg(0x03, &delta_x_l) ||  // DX low second  
        !pmw3901_spi_read_reg(0x06, &delta_y_h) ||  // DY high third
        !pmw3901_spi_read_reg(0x05, &delta_y_l)) {  // DY low fourth
        return false;
    }
    
    // 组合16位有符号数据 (完全按照Python方法)
    int16_t delta_x = (delta_x_h << 8) | delta_x_l;
    int16_t delta_y = (delta_y_h << 8) | delta_y_l;
    
    // 处理有符号数转换 (完全按照Python方法)
    if (delta_x & 0x8000) {
        delta_x -= 0x10000;
    }
    if (delta_y & 0x8000) {
        delta_y -= 0x10000;
    }
    
    // 转换为像素位移
    flow->dpixelx = (float)delta_x;
    flow->dpixely = (float)delta_y;
    flow->dt = 0.004f; // 250Hz update rate
    
    // 设置标准差 (简化版本)
    flow->stdDevX = 0.5f;
    flow->stdDevY = 0.5f;
    
    // 静态计数器用于检测传感器失效 (简化版本)
    static uint32_t zero_reading_count = 0;
    static uint32_t last_reinit_time = 0;
    static uint32_t failed_reinit_count = 0;
    
    // 检查是否所有读取都为0（仅检查运动数据，不读取SQUAL避免干扰）
    if (motion_latch == 0 && delta_x == 0 && delta_y == 0) {
        zero_reading_count++;
        
        // 如果连续100次读取都是0（约0.4秒），并且距离上次重新初始化超过5秒，则尝试重新初始化
        if (zero_reading_count > 100) {
            uint32_t current_time = xTaskGetTickCount();
            if ((current_time - last_reinit_time) > pdMS_TO_TICKS(5000)) {
                ESP_LOGW(TAG, "Sensor appears inactive, attempting reinitialize...");
                char debug_str[128];
                snprintf(debug_str, sizeof(debug_str), "Attempting sensor reinitialize (zeros=%d)\n", (int)zero_reading_count);
                debugpeintf(debug_str);
                
                // 执行SPI总线复位序列 (关键步骤!)
                gpio_set_level(PMW3901_SPI_CS, 1);
                vTaskDelay(pdMS_TO_TICKS(1));
                gpio_set_level(PMW3901_SPI_CS, 0);
                vTaskDelay(pdMS_TO_TICKS(1));
                gpio_set_level(PMW3901_SPI_CS, 1);
                vTaskDelay(pdMS_TO_TICKS(1));
                
                // 电源复位
                pmw3901_spi_write_reg(PMW3901_POWER_RESET, 0x5A);
                vTaskDelay(pdMS_TO_TICKS(5));  // Python用5ms不是100ms
                
                // 验证传感器ID
                uint8_t test_whoami = 0;
                if (pmw3901_spi_read_reg(OPTICAL_FLOW_WHO_AM_I, &test_whoami)) {
                    snprintf(debug_str, sizeof(debug_str), "After reset - WHO_AM_I: 0x%02X\n", test_whoami);
                    debugpeintf(debug_str);
                    
                    if (test_whoami == PMW3901_EXPECTED_ID) {
                        pmw3901_init_registers();
                        vTaskDelay(pdMS_TO_TICKS(500)); // 增加更长的稳定时间
                        
                        // === 完整的传感器状态调试输出 ===
                        snprintf(debug_str, sizeof(debug_str), "\n=== PMW3901 COMPLETE DIAGNOSTIC DUMP ===\n");
                        debugpeintf(debug_str);
                        
                        // 核心标识寄存器
                        uint8_t who_am_i, revision, inv_id;
                        pmw3901_spi_read_reg(0x00, &who_am_i);
                        pmw3901_spi_read_reg(0x01, &revision);
                        pmw3901_spi_read_reg(0x5F, &inv_id);
                        snprintf(debug_str, sizeof(debug_str), "ID Registers: WHO_AM_I=0x%02X, REV=0x%02X, INV=0x%02X\n", 
                                who_am_i, revision, inv_id);
                        debugpeintf(debug_str);
                        
                        // 运动和数据寄存器
                        uint8_t motion, dx_l, dx_h, dy_l, dy_h;
                        pmw3901_spi_read_reg(0x02, &motion);
                        pmw3901_spi_read_reg(0x03, &dx_l);
                        pmw3901_spi_read_reg(0x04, &dx_h);
                        pmw3901_spi_read_reg(0x05, &dy_l);
                        pmw3901_spi_read_reg(0x06, &dy_h);
                        snprintf(debug_str, sizeof(debug_str), "Motion Registers: MOT=0x%02X, DX_L=0x%02X, DX_H=0x%02X, DY_L=0x%02X, DY_H=0x%02X\n", 
                                motion, dx_l, dx_h, dy_l, dy_h);
                        debugpeintf(debug_str);
                        
                        // 表面质量和原始数据寄存器
                        uint8_t squal, raw_sum, max_raw, min_raw;
                        pmw3901_spi_read_reg(0x07, &squal);
                        pmw3901_spi_read_reg(0x08, &raw_sum);
                        pmw3901_spi_read_reg(0x09, &max_raw);
                        pmw3901_spi_read_reg(0x0A, &min_raw);
                        snprintf(debug_str, sizeof(debug_str), "Quality Registers: SQUAL=%d, RAW_SUM=%d, MAX=%d, MIN=%d\n", 
                                squal, raw_sum, max_raw, min_raw);
                        debugpeintf(debug_str);
                        
                        // 配置和状态寄存器
                        uint8_t config_bits, observation, data_ready, shutter_l, shutter_h;
                        pmw3901_spi_read_reg(0x0F, &config_bits);
                        pmw3901_spi_read_reg(0x15, &observation);
                        pmw3901_spi_read_reg(0x16, &data_ready);
                        pmw3901_spi_read_reg(0x0B, &shutter_l);
                        pmw3901_spi_read_reg(0x0C, &shutter_h);
                        snprintf(debug_str, sizeof(debug_str), "Status Registers: CFG=0x%02X, OBS=0x%02X, RDY=0x%02X, SHUT_L=0x%02X, SHUT_H=0x%02X\n", 
                                config_bits, observation, data_ready, shutter_l, shutter_h);
                        debugpeintf(debug_str);
                        
                        // 额外的诊断寄存器
                        uint8_t pixel_grab, laser_ctrl, frame_avg;
                        pmw3901_spi_read_reg(0x0D, &pixel_grab);
                        pmw3901_spi_read_reg(0x1A, &laser_ctrl);
                        pmw3901_spi_read_reg(0x17, &frame_avg);
                        snprintf(debug_str, sizeof(debug_str), "Extended: PIXEL=0x%02X, LASER=0x%02X, FRAME_AVG=0x%02X\n", 
                                pixel_grab, laser_ctrl, frame_avg);
                        debugpeintf(debug_str);
                        
                        // 页面寄存器和电源状态
                        uint8_t page_sel, power_reset;
                        pmw3901_spi_read_reg(0x7F, &page_sel);
                        pmw3901_spi_read_reg(0x3A, &power_reset);
                        snprintf(debug_str, sizeof(debug_str), "Control: PAGE=0x%02X, PWR_RST=0x%02X\n", 
                                page_sel, power_reset);
                        debugpeintf(debug_str);
                        
                        // GPIO状态
                        int cs_level = gpio_get_level(PMW3901_SPI_CS);
                        snprintf(debug_str, sizeof(debug_str), "GPIO Status: CS_PIN=%d, CS_LEVEL=%d\n", 
                                PMW3901_SPI_CS, cs_level);
                        debugpeintf(debug_str);
                        
                        // SPI设备句柄状态
                        snprintf(debug_str, sizeof(debug_str), "SPI Status: Handle=0x%08X, Valid=%s\n", 
                                (unsigned int)pmw3901_spi, pmw3901_spi ? "YES" : "NO");
                        debugpeintf(debug_str);
                        
                        snprintf(debug_str, sizeof(debug_str), "=== END DIAGNOSTIC DUMP ===\n\n");
                        debugpeintf(debug_str);
                        
                        if (squal == 0 && raw_sum == 0 && max_raw == 0) {
                            snprintf(debug_str, sizeof(debug_str), "*** SENSOR OPTICAL SYSTEM FAILURE DETECTED ***\n");
                            debugpeintf(debug_str);
                            
                            // 执行最终的光学系统测试
                            snprintf(debug_str, sizeof(debug_str), "Performing final optical system test...\n");
                            debugpeintf(debug_str);
                            
                            // 尝试读取一些其他寄存器来验证
                            uint8_t config_reg = 0, observation_reg = 0;
                            pmw3901_spi_read_reg(0x0F, &config_reg);
                            pmw3901_spi_read_reg(0x15, &observation_reg);
                            
                            snprintf(debug_str, sizeof(debug_str), "Config: 0x%02X, Observation: 0x%02X\n", config_reg, observation_reg);
                            debugpeintf(debug_str);
                            
                            if (config_reg == 0 && observation_reg == 0) {
                                snprintf(debug_str, sizeof(debug_str), "*** HARDWARE FAULT: Optical sensor appears defective ***\n");
                                debugpeintf(debug_str);
                                snprintf(debug_str, sizeof(debug_str), "Disabling optical flow sensor permanently\n");
                                debugpeintf(debug_str);
                                
                                // 禁用传感器并停止重新初始化尝试
                                isPresent = false;
                                return false;
                            }
                            
                            snprintf(debug_str, sizeof(debug_str), "Try: 1) Move closer to textured surface (20-50mm)\n");
                            debugpeintf(debug_str);
                            snprintf(debug_str, sizeof(debug_str), "     2) Use patterned paper/carpet underneath\n");
                            debugpeintf(debug_str);
                            snprintf(debug_str, sizeof(debug_str), "     3) Ensure good lighting conditions\n");
                            debugpeintf(debug_str);
                            snprintf(debug_str, sizeof(debug_str), "     4) Check sensor is facing downward\n");
                            debugpeintf(debug_str);
                        }
                        
                        snprintf(debug_str, sizeof(debug_str), "PMW3901 registers initialized, waiting for stabilization\n");
                        debugpeintf(debug_str);
                    } else if (test_whoami == PAA5100_EXPECTED_ID) {
                        snprintf(debug_str, sizeof(debug_str), "PAA5100 detected, skipping PMW3901-specific init\n");
                        debugpeintf(debug_str);
                    } else {
                        snprintf(debug_str, sizeof(debug_str), "Unknown sensor ID: 0x%02X\n", test_whoami);
                        debugpeintf(debug_str);
                        // 可能不是真正的PMW3901/PAA5100，标记传感器不存在
                        if (test_whoami == 0x00 || test_whoami == 0xFF) {
                            snprintf(debug_str, sizeof(debug_str), "Sensor may not be connected - disabling optical flow\n");
                            debugpeintf(debug_str);
                            isPresent = false;
                            return false;
                        }
                    }
                } else {
                    snprintf(debug_str, sizeof(debug_str), "Failed to read WHO_AM_I after reset\n");
                    debugpeintf(debug_str);
                }
                
                // 清除运动寄存器 (按照Python顺序: 0x02, 0x03, 0x04, 0x05, 0x06)
                uint8_t dummy;
                pmw3901_spi_read_reg(0x02, &dummy);  // Motion latch
                pmw3901_spi_read_reg(0x03, &dummy);  // Delta X low
                pmw3901_spi_read_reg(0x04, &dummy);  // Delta X high
                pmw3901_spi_read_reg(0x05, &dummy);  // Delta Y low
                pmw3901_spi_read_reg(0x06, &dummy);  // Delta Y high
                vTaskDelay(pdMS_TO_TICKS(1)); // Python用1ms延时
                
                last_reinit_time = current_time;
                zero_reading_count = 0;
                
                // 简化的重新初始化检查 (不读取额外寄存器避免干扰)
                vTaskDelay(pdMS_TO_TICKS(100)); // 等待稳定
                failed_reinit_count = 0; // 重置失败计数，给传感器一个机会
                
                ESP_LOGI(TAG, "Sensor reinitialize completed");
            }
        }
    } else {
        zero_reading_count = 0; // 重置计数器
    }
    
    // 简化的调试输出 (每秒一次，避免干扰传感器)
    static uint32_t debug_counter = 0;
    if (++debug_counter % 250 == 0) { // 每秒打印一次 (250Hz)
        char debug_str[200];
        bool motion_detected = (motion_latch & 0x80) != 0;
        
        snprintf(debug_str, sizeof(debug_str), "PMW3901: latch=0x%02X(motion=%d), DX=%d, DY=%d, zeros=%d\n",
                motion_latch, motion_detected ? 1 : 0, delta_x, delta_y, (int)zero_reading_count);
        debugpeintf(debug_str);
        
        if (delta_x != 0 || delta_y != 0) {
            snprintf(debug_str, sizeof(debug_str), "  *** MOTION DETECTED! Sensor working! ***\n");
            debugpeintf(debug_str);
        }
    }
    
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
        spi_bus_free(SPI3_HOST);  // 注意：只有在确定没有其他设备使用时才释放总线
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
