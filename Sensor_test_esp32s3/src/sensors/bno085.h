#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// BNO085 Rotation Vector (quaternion) data
typedef struct {
    float i;              // Quaternion i component
    float j;              // Quaternion j component
    float k;              // Quaternion k component
    float real;           // Quaternion real/w component
    float accuracy_rad;   // Estimated accuracy in radians
    uint8_t accuracy_status; // 0=unreliable, 1=low, 2=medium, 3=high
    uint32_t timestamp_ms;
    bool valid;
} bno085_quaternion_t;

// BNO085 Euler angles (derived from quaternion)
typedef struct {
    float roll;           // Roll angle in degrees (-180 to 180)
    float pitch;          // Pitch angle in degrees (-90 to 90)
    float yaw;            // Yaw/heading in degrees (0 to 360)
    uint8_t accuracy_status;
    uint32_t timestamp_ms;
    bool valid;
} bno085_euler_t;

// BNO085 Accelerometer data
typedef struct {
    float x;              // m/s^2
    float y;
    float z;
    uint8_t accuracy_status;
    uint32_t timestamp_ms;
    bool valid;
} bno085_accel_t;

// BNO085 Gyroscope data
typedef struct {
    float x;              // rad/s
    float y;
    float z;
    uint8_t accuracy_status;
    uint32_t timestamp_ms;
    bool valid;
} bno085_gyro_t;

// Combined IMU data structure
typedef struct {
    bno085_quaternion_t quaternion;
    bno085_euler_t euler;
    bno085_accel_t accel;
    bno085_gyro_t gyro;
    bool calibrated;
} bno085_data_t;

// Pin configuration
typedef struct {
    spi_host_device_t spi_host;  // SPI2_HOST or SPI3_HOST
    gpio_num_t sck_pin;          // SPI Clock
    gpio_num_t mosi_pin;         // SPI MOSI
    gpio_num_t miso_pin;         // SPI MISO
    gpio_num_t cs_pin;           // Chip Select
    gpio_num_t wake_pin;         // PS0/WAKE pin
    gpio_num_t int_pin;          // H_INTN interrupt pin
    gpio_num_t reset_pin;        // NRST reset pin
    uint32_t report_interval_us; // Report interval in microseconds (default 4000 = 250Hz)
} bno085_config_t;

// Initialize BNO085 sensor
esp_err_t bno085_init(const bno085_config_t *cfg);

// Deinitialize BNO085 sensor
esp_err_t bno085_deinit(void);

// Reset the sensor
esp_err_t bno085_reset(void);

// Enable/configure rotation vector reports
esp_err_t bno085_enable_rotation_vector(uint32_t interval_us);

// Enable/configure game rotation vector (no magnetometer)
esp_err_t bno085_enable_game_rotation_vector(uint32_t interval_us);

// Enable/configure rotation vector candidates (stabilized/rotation/game/geomagnetic)
esp_err_t bno085_enable_rotation_vector_candidates(uint32_t interval_us);

// Enable/configure accelerometer reports
esp_err_t bno085_enable_accelerometer(uint32_t interval_us);

// Enable/configure gyroscope reports
esp_err_t bno085_enable_gyroscope(uint32_t interval_us);

// Check if new data is available (interrupt driven)
bool bno085_data_available(void);

// Read all available data (call from task loop)
esp_err_t bno085_read(bno085_data_t *data);

// Get latest quaternion
bool bno085_get_quaternion(bno085_quaternion_t *quat);

// Get latest euler angles
bool bno085_get_euler(bno085_euler_t *euler);

// Convert quaternion to euler angles
void bno085_quaternion_to_euler(const bno085_quaternion_t *quat, bno085_euler_t *euler);

// Perform tare (zero heading)
esp_err_t bno085_tare(void);

// Save calibration to flash
esp_err_t bno085_save_calibration(void);

// Check if sensor is calibrated
bool bno085_is_calibrated(void);

// Get product ID info
esp_err_t bno085_get_product_id(uint8_t *sw_major, uint8_t *sw_minor, uint32_t *sw_part);

#ifdef __cplusplus
}
#endif
