#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Flight Data Record - All sensor data in one compact structure (64 bytes)
// ============================================================================

typedef struct __attribute__((packed)) {
    // Timestamp (4 bytes)
    uint32_t timestamp;         // Unix timestamp (seconds) or uptime if no sync

    // GPS (20 bytes)
    int32_t lat;                // Latitude * 1e7
    int32_t lon;                // Longitude * 1e7
    int32_t alt_mm;             // Altitude in mm (MSL)
    int16_t vel_down;           // Vertical velocity mm/s (positive = descending)
    uint8_t fix_type;           // GPS fix type (0=none, 3=3D)
    uint8_t sats;               // Number of satellites
    uint16_t pdop;              // Position DOP * 100
    uint8_t gps_valid;          // GPS data valid flag

    // IMU - Quaternion (10 bytes)
    int16_t qi;                 // Quaternion i * 16384
    int16_t qj;                 // Quaternion j * 16384
    int16_t qk;                 // Quaternion k * 16384
    int16_t qw;                 // Quaternion w * 16384
    uint8_t imu_accuracy;       // IMU accuracy status
    uint8_t imu_valid;          // IMU data valid

    // Magnetometer (8 bytes)
    int16_t mag_x;              // Raw magnetometer X
    int16_t mag_y;              // Raw magnetometer Y
    int16_t mag_z;              // Raw magnetometer Z
    int16_t heading;            // Heading * 10 (0-3600)

    // Barometer BMP585 (8 bytes)
    int32_t pressure_pa;        // Pressure in Pa (raw, ~101325 at sea level)
    int16_t bmp_temp;           // Temperature * 100
    int16_t baro_alt;           // Barometric altitude in dm (decimeters)

    // SCD40 - CO2/Humidity (6 bytes)
    uint16_t co2;               // CO2 in ppm
    int16_t scd_temp;           // Temperature * 100
    uint16_t humidity;          // Humidity * 100

    // SPS30 - Particulate (8 bytes)
    uint16_t pm1;               // PM1.0 * 10
    uint16_t pm25;              // PM2.5 * 10
    uint16_t pm4;               // PM4.0 * 10
    uint16_t pm10;              // PM10 * 10

    uint8_t reserved;           // Padding to 64 bytes

} flight_record_t;  // Total: 64 bytes

_Static_assert(sizeof(flight_record_t) == 64, "flight_record_t must be 64 bytes");

// ============================================================================
// Flight Recorder API
// ============================================================================

/**
 * @brief Initialize flight recorder with PSRAM ring buffer
 * @param max_records Maximum number of records to store (e.g., 86400 = 24h @ 1Hz)
 * @return ESP_OK on success
 */
esp_err_t flight_recorder_init(size_t max_records);

/**
 * @brief Add a new flight record
 * Records are timestamped automatically
 */
void flight_recorder_add(const flight_record_t *record);

/**
 * @brief Get records from buffer
 * @param out Output array
 * @param max_records Maximum records to return
 * @param offset Start offset from oldest
 * @return Number of records copied
 */
size_t flight_recorder_get(flight_record_t *out, size_t max_records, size_t offset);

/**
 * @brief Get records within time range
 * @param out Output array
 * @param max_out Maximum records to return
 * @param time_range_sec Time range in seconds (0 = all)
 * @param downsample Return every Nth record (1 = all)
 * @return Number of records copied
 */
size_t flight_recorder_get_range(flight_record_t *out, size_t max_out,
                                  uint32_t time_range_sec, size_t downsample);

/**
 * @brief Get total record count
 */
size_t flight_recorder_count(void);

/**
 * @brief Get maximum capacity
 */
size_t flight_recorder_capacity(void);

/**
 * @brief Clear all records
 */
void flight_recorder_clear(void);

/**
 * @brief Get memory usage in bytes
 */
size_t flight_recorder_memory_usage(void);

// ============================================================================
// File Recording API (LittleFS)
// ============================================================================

/**
 * @brief Start recording to LittleFS file
 * @param filename Filename (e.g., "flight_20250125_143000.bin")
 * @return ESP_OK on success
 */
esp_err_t flight_recording_start(const char *filename);

/**
 * @brief Stop current recording
 */
esp_err_t flight_recording_stop(void);

/**
 * @brief Check if recording is active
 */
bool flight_recording_active(void);

/**
 * @brief Write current record to file (call from main loop)
 */
void flight_recording_write(const flight_record_t *record);

/**
 * @brief Get recording info
 */
typedef struct {
    char filename[48];
    uint32_t start_timestamp;
    uint32_t record_count;
    size_t file_size;
} flight_recording_info_t;

esp_err_t flight_recording_get_info(flight_recording_info_t *info);

// ============================================================================
// Data Source Enum (for chart selection)
// ============================================================================

typedef enum {
    FLIGHT_DATA_PM,         // PM1, PM2.5, PM4, PM10
    FLIGHT_DATA_CO2,        // CO2
    FLIGHT_DATA_PRESSURE,   // Barometric pressure
    FLIGHT_DATA_ALTITUDE,   // Barometric + GPS altitude
    FLIGHT_DATA_TEMPERATURE,// All temperature sensors
    FLIGHT_DATA_HUMIDITY,   // Humidity
    FLIGHT_DATA_VELOCITY,   // Vertical velocity
    FLIGHT_DATA_HEADING,    // Compass heading
    FLIGHT_DATA_COUNT
} flight_data_type_t;

/**
 * @brief Get data series for charting
 * @param type Data type to extract
 * @param out_values Output array of float values (up to 4 series)
 * @param out_timestamps Output array of timestamps
 * @param max_points Maximum points
 * @param time_range_sec Time range (0 = all)
 * @param series_count Output: number of data series
 * @return Number of points
 */
size_t flight_recorder_get_chart_data(flight_data_type_t type,
                                       float out_values[][4],
                                       uint32_t *out_timestamps,
                                       size_t max_points,
                                       uint32_t time_range_sec,
                                       uint8_t *series_count);

#ifdef __cplusplus
}
#endif
