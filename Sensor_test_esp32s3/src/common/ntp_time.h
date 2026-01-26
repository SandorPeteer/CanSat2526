#pragma once

#include "esp_err.h"
#include <time.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Time source enumeration
 */
typedef enum {
    TIME_SOURCE_NONE = 0,
    TIME_SOURCE_GPS,
    TIME_SOURCE_NTP,
} time_source_t;

/**
 * @brief Initialize time system
 *
 * Loads timezone from NVS, starts NTP sync if WiFi available
 *
 * @return ESP_OK on success
 */
esp_err_t time_init(void);

/**
 * @brief Set time from GPS
 *
 * GPS time is UTC - this function applies the configured timezone offset
 *
 * @param year GPS year (e.g., 2024)
 * @param month GPS month (1-12)
 * @param day GPS day (1-31)
 * @param hour GPS hour UTC (0-23)
 * @param min GPS minute (0-59)
 * @param sec GPS second (0-59)
 * @param valid GPS time validity flag
 * @return ESP_OK if time was set
 */
esp_err_t time_set_from_gps(uint16_t year, uint8_t month, uint8_t day,
                            uint8_t hour, uint8_t min, uint8_t sec, bool valid);

/**
 * @brief Check if time is synchronized (from any source)
 */
bool time_is_synced(void);

/**
 * @brief Get current time source
 */
time_source_t time_get_source(void);

/**
 * @brief Get current Unix timestamp (seconds since 1970-01-01 00:00:00 UTC)
 *
 * @return Unix timestamp, or 0 if not synced
 */
time_t time_get_unix(void);

/**
 * @brief Get current local time as struct tm
 *
 * @param tm_out Output time structure (local time with timezone applied)
 * @return ESP_OK if time is valid
 */
esp_err_t time_get_local(struct tm *tm_out);

/**
 * @brief Format current local time as ISO 8601 string
 *
 * @param buf Output buffer (at least 20 bytes for "YYYY-MM-DDTHH:MM:SS")
 * @param buf_size Buffer size
 * @return ESP_OK if successful
 */
esp_err_t time_format_iso(char *buf, size_t buf_size);

/**
 * @brief Format current local time for filename (compact format)
 *
 * @param buf Output buffer (at least 16 bytes for "YYYYMMDD_HHMMSS")
 * @param buf_size Buffer size
 * @return ESP_OK if successful
 */
esp_err_t time_format_filename(char *buf, size_t buf_size);

/**
 * @brief Get current timezone string
 *
 * @return Timezone string (e.g., "CET-1CEST,M3.5.0/2,M10.5.0/3")
 */
const char* time_get_timezone(void);

/**
 * @brief Set and save timezone
 *
 * @param tz POSIX timezone string (e.g., "CET-1CEST,M3.5.0/2,M10.5.0/3")
 * @return ESP_OK on success
 */
esp_err_t time_set_timezone(const char *tz);

/**
 * @brief Get timezone offset in minutes from UTC
 *
 * @return Offset in minutes (e.g., 60 for CET, 120 for CEST)
 */
int time_get_utc_offset_minutes(void);

/**
 * @brief Convert GPS UTC time to local timestamp (applying timezone)
 *
 * @param year GPS year
 * @param month GPS month (1-12)
 * @param day GPS day (1-31)
 * @param hour GPS hour UTC (0-23)
 * @param min GPS minute (0-59)
 * @param sec GPS second (0-59)
 * @return Local Unix timestamp
 */
time_t time_gps_to_local_unix(uint16_t year, uint8_t month, uint8_t day,
                              uint8_t hour, uint8_t min, uint8_t sec);

// Legacy compatibility (ntp_time_* functions)
#define ntp_time_init() time_init()
#define ntp_time_is_synced() time_is_synced()
#define ntp_time_get_unix() time_get_unix()
#define ntp_time_get_tm(tm) time_get_local(tm)
#define ntp_time_format_iso(buf, size) time_format_iso(buf, size)
#define ntp_time_format_filename(buf, size) time_format_filename(buf, size)

#ifdef __cplusplus
}
#endif
