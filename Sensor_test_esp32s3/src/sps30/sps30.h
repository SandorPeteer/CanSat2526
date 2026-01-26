#pragma once

#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sps30_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the SPS30 sensor on a UART port
 *
 * @param uart_num UART port number (e.g., UART_NUM_2)
 * @param tx_pin GPIO pin for TX
 * @param rx_pin GPIO pin for RX
 * @return ESP_OK on success
 */
esp_err_t sps30_init(uart_port_t uart_num, gpio_num_t tx_pin, gpio_num_t rx_pin);

/**
 * @brief Deinitialize the SPS30 sensor and release UART resources
 *
 * @return ESP_OK on success
 */
esp_err_t sps30_deinit(void);

/**
 * @brief Start measurement mode
 *
 * @param format Output format (float or uint16)
 * @return ESP_OK on success
 */
esp_err_t sps30_start_measurement(sps30_output_format_t format);

/**
 * @brief Stop measurement mode
 *
 * @return ESP_OK on success
 */
esp_err_t sps30_stop_measurement(void);

/**
 * @brief Check if new measurement data is available
 *
 * @param ready Pointer to store result (true if data ready)
 * @return ESP_OK on success
 */
esp_err_t sps30_data_ready(bool *ready);

/**
 * @brief Read measurement values from the sensor
 *
 * @param data Pointer to store measurement data
 * @return ESP_OK on success
 */
esp_err_t sps30_read_measurement(sps30_measurement_t *data);

/**
 * @brief Get device information (serial, product type, firmware version)
 *
 * @param info Pointer to store device info
 * @return ESP_OK on success
 */
esp_err_t sps30_get_device_info(sps30_device_info_t *info);

/**
 * @brief Start the fan cleaning cycle
 *
 * @return ESP_OK on success
 */
esp_err_t sps30_start_fan_cleaning(void);

/**
 * @brief Read the auto-clean interval
 *
 * @param interval_seconds Pointer to store interval in seconds
 * @return ESP_OK on success
 */
esp_err_t sps30_read_auto_clean_interval(uint32_t *interval_seconds);

/**
 * @brief Set the auto-clean interval
 *
 * @param interval_seconds Interval in seconds (0 to disable)
 * @return ESP_OK on success
 */
esp_err_t sps30_write_auto_clean_interval(uint32_t interval_seconds);

/**
 * @brief Put the sensor into sleep mode
 *
 * @return ESP_OK on success
 */
esp_err_t sps30_sleep(void);

/**
 * @brief Wake up the sensor from sleep mode
 *
 * @return ESP_OK on success
 */
esp_err_t sps30_wakeup(void);

/**
 * @brief Reset the sensor
 *
 * @return ESP_OK on success
 */
esp_err_t sps30_reset(void);

#ifdef __cplusplus
}
#endif
