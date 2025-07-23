/**
 * PMW3901 Optical Flow Sensor Driver
 * Based on Bitcraze PMW3901 Arduino library
 * Adapted for ESP32 with ESP-IDF
 */

#include "pmw3901_driver.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"

static const char* TAG = "PMW3901";

// PMW3901 Register addresses
#define PMW3901_PRODUCT_ID         0x00
#define PMW3901_REVISION_ID        0x01
#define PMW3901_MOTION             0x02
#define PMW3901_DELTA_X_L          0x03
#define PMW3901_DELTA_X_H          0x04
#define PMW3901_DELTA_Y_L          0x05
#define PMW3901_DELTA_Y_H          0x06
#define PMW3901_SQUAL              0x07
#define PMW3901_RAW_DATA_SUM       0x08
#define PMW3901_MAXIMUM_RAW_DATA   0x09
#define PMW3901_MINIMUM_RAW_DATA   0x0A
#define PMW3901_SHUTTER_LOWER      0x0B
#define PMW3901_SHUTTER_UPPER      0x0C
#define PMW3901_POWER_UP_RESET     0x3A
#define PMW3901_OBSERVATION        0x15
#define PMW3901_MOTION_BURST       0x16
#define PMW3901_POWER_UP_RESET_VAL 0x5A

// Expected chip identifier
#define PMW3901_CHIP_ID 0x49

// SPI Configuration
#define PMW3901_SPI_FREQ     4000000  // 4MHz
#define PMW3901_SPI_MODE     3        // CPOL=1, CPHA=1

struct pmw3901_dev {
    spi_device_handle_t spi;
    int cs_pin;
    bool initialized;
};

static struct pmw3901_dev dev = {0};

// Register configuration sequence from Bitcraze Arduino library (complete version)
static const uint8_t init_sequence[][2] = {
    {0x7F, 0x00}, {0x61, 0xAD}, {0x7F, 0x03}, {0x40, 0x00}, {0x7F, 0x05},
    {0x41, 0xB3}, {0x43, 0xF1}, {0x45, 0x14}, {0x5B, 0x32}, {0x5F, 0x34},
    {0x7B, 0x08}, {0x7F, 0x06}, {0x44, 0x1B}, {0x40, 0xBF}, {0x4E, 0x3F},
    {0x7F, 0x08}, {0x65, 0x20}, {0x6A, 0x18}, {0x7F, 0x09}, {0x4F, 0xAF},
    {0x5F, 0x40}, {0x48, 0x80}, {0x49, 0x80}, {0x57, 0x77}, {0x60, 0x78},
    {0x61, 0x78}, {0x62, 0x08}, {0x63, 0x50}, {0x7F, 0x0A}, {0x45, 0x60},
    {0x7F, 0x00}, {0x4D, 0x11}, {0x55, 0x80}, {0x74, 0x1F}, {0x75, 0x1F},
    {0x4A, 0x78}, {0x4B, 0x78}, {0x44, 0x08}, {0x45, 0x50}, {0x64, 0xFF},
    {0x65, 0x1F}, {0x7F, 0x14}, {0x65, 0x60}, {0x66, 0x08}, {0x63, 0x78},
    {0x7F, 0x15}, {0x48, 0x58}, {0x7F, 0x07}, {0x41, 0x0D}, {0x43, 0x14},
    {0x4B, 0x0E}, {0x45, 0x0F}, {0x44, 0x42}, {0x4C, 0x80}, {0x7F, 0x10},
    {0x5B, 0x02}, {0x7F, 0x07}, {0x40, 0x41}, {0x70, 0x00}
    // Note: delay(100) and remaining registers will be handled separately
};

static esp_err_t pmw3901_write_reg(uint8_t reg, uint8_t value)
{
    if (!dev.initialized) return ESP_ERR_INVALID_STATE;
    
    uint8_t tx_reg = reg | 0x80u;  // Set write bit (exact Arduino style)
    
    gpio_set_level(dev.cs_pin, 0);
    esp_rom_delay_us(50);
    
    // Send register address (exactly like Arduino SPI.transfer(reg))
    spi_transaction_t trans1 = {
        .length = 8,
        .tx_buffer = &tx_reg,
    };
    
    esp_err_t ret = spi_device_transmit(dev.spi, &trans1);
    if (ret != ESP_OK) {
        gpio_set_level(dev.cs_pin, 1);
        return ret;
    }

    // SPI运行在Core 1，不需要频繁让出CPU时间
    esp_rom_delay_us(10);
    
    // Send value (exactly like Arduino SPI.transfer(value))
    spi_transaction_t trans2 = {
        .length = 8,
        .tx_buffer = &value,
    };
    
    ret = spi_device_transmit(dev.spi, &trans2);
    
    esp_rom_delay_us(50);
    gpio_set_level(dev.cs_pin, 1);
    esp_rom_delay_us(200);  // Arduino has this delay
    // 减少调试输出以避免阻塞遥控器通信
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI WRITE FAILED: reg=0x%02X, value=0x%02X, error=%s", reg, value, esp_err_to_name(ret));
    }
    
    return ret;
}

static esp_err_t pmw3901_read_reg(uint8_t reg, uint8_t* value)
{
    if (!dev.initialized || !value) return ESP_ERR_INVALID_ARG;
    
    uint8_t tx_addr = reg & ~0x80u;  // Clear write bit (exact Arduino style)
    uint8_t dummy_tx = 0;  // Dummy byte for read
    uint8_t rx_data = 0;
    
    gpio_set_level(dev.cs_pin, 0);
    esp_rom_delay_us(50);
    
    // Send register address (exactly like Arduino SPI.transfer(reg))
    spi_transaction_t trans1 = {
        .length = 8,
        .tx_buffer = &tx_addr,
    };
    
    esp_err_t ret = spi_device_transmit(dev.spi, &trans1);
    if (ret != ESP_OK) {
        gpio_set_level(dev.cs_pin, 1);
        return ret;
    }
    
    esp_rom_delay_us(50);
    
    // Read data (exactly like Arduino SPI.transfer(0))
    spi_transaction_t trans2 = {
        .length = 8,
        .tx_buffer = &dummy_tx,  // Send 0 like Arduino
        .rx_buffer = &rx_data,
    };
    
    ret = spi_device_transmit(dev.spi, &trans2);
    
    esp_rom_delay_us(100);
    gpio_set_level(dev.cs_pin, 1);
    
    if (ret == ESP_OK) {
        *value = rx_data;
        // 减少调试输出以避免阻塞遥控器通信
    } else {
        ESP_LOGE(TAG, "SPI READ FAILED: reg=0x%02X, error=%s", reg, esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t pmw3901_init(int sck_pin, int mosi_pin, int miso_pin, int cs_pin)
{
    // Configure CS pin as output
    gpio_config_t cs_cfg = {
        .pin_bit_mask = (1ULL << cs_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    
    esp_err_t ret = gpio_config(&cs_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure CS GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    
    gpio_set_level(cs_pin, 1);  // CS idle high
    dev.cs_pin = cs_pin;
    
    // Configure SPI bus
    spi_bus_config_t bus_cfg = {
        .miso_io_num = miso_pin,
        .mosi_io_num = mosi_pin,
        .sclk_io_num = sck_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 16,
    };
    
    ret = spi_bus_initialize(SPI3_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Add device to SPI bus
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = PMW3901_SPI_FREQ,
        .mode = PMW3901_SPI_MODE,
        .spics_io_num = -1,  // Manual CS control
        .queue_size = 1,
    };
    
    ret = spi_bus_add_device(SPI3_HOST, &dev_cfg, &dev.spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return ret;
    }
    
    dev.initialized = true;
    
    ESP_LOGI(TAG, "SPI initialized, waiting for sensor startup...");
    // vTaskDelay(pdMS_TO_TICKS(100));  // Reduced from 4s to 100ms to avoid blocking system init
    
    return ESP_OK;
}

esp_err_t pmw3901_begin(void)
{
    if (!dev.initialized) {
        ESP_LOGE(TAG, "Device not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Starting PMW3901 sensor initialization...");
    
    // Give sensor some time to fully power up before SPI communication
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // SPI bus reset sequence (from Bitcraze library)
    gpio_set_level(dev.cs_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(dev.cs_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(dev.cs_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Power-up reset
    esp_err_t ret = pmw3901_write_reg(PMW3901_POWER_UP_RESET, PMW3901_POWER_UP_RESET_VAL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send power-up reset");
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // Verify chip ID
    ESP_LOGI(TAG, "Reading and verifying chip ID...");
    uint8_t chip_id = 0;
    ret = pmw3901_read_reg(PMW3901_PRODUCT_ID, &chip_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read chip ID");
        return ret;
    }
    
    ESP_LOGI(TAG, "Read chip ID: 0x%02X (expected 0x%02X)", chip_id, PMW3901_CHIP_ID);
    // Also read the inverse chip ID for complete verification like Python
    uint8_t chip_id_inverse = 0;
    ret = pmw3901_read_reg(0x5F, &chip_id_inverse);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Read inverse chip ID: 0x%02X (expected 0xB6)", chip_id_inverse);
    } else {
        ESP_LOGE(TAG, "Failed to read inverse chip ID");
    }
    
    if (chip_id != PMW3901_CHIP_ID) {
        ESP_LOGE(TAG, "Invalid chip ID: 0x%02X (expected 0x%02X)", chip_id, PMW3901_CHIP_ID);
        
        return ESP_ERR_NOT_FOUND;
    }
    
    ESP_LOGI(TAG, "PMW3901 chip ID verified: 0x%02X", chip_id);
    
    // Read motion registers to clear residual data
    uint8_t dummy;
    pmw3901_read_reg(PMW3901_MOTION, &dummy);
    pmw3901_read_reg(PMW3901_DELTA_X_L, &dummy);
    pmw3901_read_reg(PMW3901_DELTA_X_H, &dummy);
    pmw3901_read_reg(PMW3901_DELTA_Y_L, &dummy);
    pmw3901_read_reg(PMW3901_DELTA_Y_H, &dummy);
    
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Initialize sensor registers (first part)
    ESP_LOGI(TAG, "Writing initialization registers...");
    for (int i = 0; i < sizeof(init_sequence) / sizeof(init_sequence[0]); i++) {
        ret = pmw3901_write_reg(init_sequence[i][0], init_sequence[i][1]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write init register 0x%02X", init_sequence[i][0]);
            return ret;
        }
    }
    
    // Arduino has delay(100) here
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Complete remaining registers from Arduino version
    ret = pmw3901_write_reg(0x32, 0x44);
    if (ret != ESP_OK) return ret;
    ret = pmw3901_write_reg(0x7F, 0x07);
    if (ret != ESP_OK) return ret;
    ret = pmw3901_write_reg(0x40, 0x40);
    if (ret != ESP_OK) return ret;
    ret = pmw3901_write_reg(0x7F, 0x06);
    if (ret != ESP_OK) return ret;
    ret = pmw3901_write_reg(0x62, 0xf0);
    if (ret != ESP_OK) return ret;
    ret = pmw3901_write_reg(0x63, 0x00);
    if (ret != ESP_OK) return ret;
    ret = pmw3901_write_reg(0x7F, 0x0D);
    if (ret != ESP_OK) return ret;
    ret = pmw3901_write_reg(0x48, 0xC0);
    if (ret != ESP_OK) return ret;
    ret = pmw3901_write_reg(0x6F, 0xd5);
    if (ret != ESP_OK) return ret;
    ret = pmw3901_write_reg(0x7F, 0x00);
    if (ret != ESP_OK) return ret;
    ret = pmw3901_write_reg(0x5B, 0xa0);
    if (ret != ESP_OK) return ret;
    ret = pmw3901_write_reg(0x4E, 0xA8);
    if (ret != ESP_OK) return ret;
    ret = pmw3901_write_reg(0x5A, 0x50);
    if (ret != ESP_OK) return ret;
    ret = pmw3901_write_reg(0x40, 0x80);
    if (ret != ESP_OK) return ret;
    
    ESP_LOGI(TAG, "PMW3901 initialization completed successfully");
    return ESP_OK;
}

esp_err_t pmw3901_read_motion_count(int16_t* delta_x, int16_t* delta_y)
{
    if (!dev.initialized || !delta_x || !delta_y) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Latch motion registers by reading motion register
    uint8_t motion;
    esp_err_t ret = pmw3901_read_reg(PMW3901_MOTION, &motion);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 运行在Core 1，不会阻塞WiFi通信，可以连续读取
    
    // Read delta values in exact Python order: 0x04, 0x03, 0x06, 0x05
    uint8_t delta_x_h, delta_x_l, delta_y_h, delta_y_l;
    
    // Python: delta_x = (self.register_read(0x04) << 8) | self.register_read(0x03)
    ret = pmw3901_read_reg(PMW3901_DELTA_X_H, &delta_x_h);  // 0x04 first
    if (ret != ESP_OK) return ret;
    
    ret = pmw3901_read_reg(PMW3901_DELTA_X_L, &delta_x_l);  // 0x03 second
    if (ret != ESP_OK) return ret;
    
    // Python: delta_y = (self.register_read(0x06) << 8) | self.register_read(0x05)
    ret = pmw3901_read_reg(PMW3901_DELTA_Y_H, &delta_y_h);  // 0x06 third
    if (ret != ESP_OK) return ret;
    
    ret = pmw3901_read_reg(PMW3901_DELTA_Y_L, &delta_y_l);  // 0x05 fourth
    if (ret != ESP_OK) return ret;
    
    // Combine bytes exactly like Python: (high << 8) | low
    *delta_x = (int16_t)((delta_x_h << 8) | delta_x_l);
    *delta_y = (int16_t)((delta_y_h << 8) | delta_y_l);
    
    // Convert to signed 16-bit exactly like Python
    if (*delta_x & 0x8000) {
        *delta_x -= 0x10000;
    }
    if (*delta_y & 0x8000) {
        *delta_y -= 0x10000;
    }
    
    return ESP_OK;
}

esp_err_t pmw3901_read_surface_quality(uint8_t* squal)
{
    if (!dev.initialized || !squal) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return pmw3901_read_reg(PMW3901_SQUAL, squal);
}

bool pmw3901_is_initialized(void)
{
    return dev.initialized;
}

void pmw3901_deinit(void)
{
    if (dev.initialized) {
        spi_bus_remove_device(dev.spi);
        spi_bus_free(SPI3_HOST);
        dev.initialized = false;
        dev.spi = NULL;
        ESP_LOGI(TAG, "PMW3901 deinitialized");
    }
}