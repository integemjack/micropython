/**
 * PMW3901 Optical Flow Sensor Driver Header
 * Based on Bitcraze PMW3901 Arduino library
 * Adapted for ESP32 with ESP-IDF
 */

#ifndef PMW3901_DRIVER_H
#define PMW3901_DRIVER_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the PMW3901 sensor with SPI configuration
 * @param sck_pin SPI clock pin
 * @param mosi_pin SPI MOSI pin  
 * @param miso_pin SPI MISO pin
 * @param cs_pin Chip select pin
 * @return ESP_OK on success, error code on failure
 */
esp_err_t pmw3901_init(int sck_pin, int mosi_pin, int miso_pin, int cs_pin);

/**
 * Begin sensor initialization sequence
 * @return ESP_OK on success, error code on failure
 */
esp_err_t pmw3901_begin(void);

/**
 * Read motion count from sensor
 * @param delta_x Pointer to store X movement
 * @param delta_y Pointer to store Y movement
 * @return ESP_OK on success, error code on failure
 */
esp_err_t pmw3901_read_motion_count(int16_t* delta_x, int16_t* delta_y);

/**
 * Read surface quality indicator
 * @param squal Pointer to store surface quality value
 * @return ESP_OK on success, error code on failure
 */
esp_err_t pmw3901_read_surface_quality(uint8_t* squal);

/**
 * Check if sensor is initialized
 * @return true if initialized, false otherwise
 */
bool pmw3901_is_initialized(void);

/**
 * Deinitialize the sensor
 */
void pmw3901_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // PMW3901_DRIVER_H