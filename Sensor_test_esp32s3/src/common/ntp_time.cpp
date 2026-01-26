#include "ntp_time.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <sys/time.h>
#include <stdint.h>

static const char *TAG = "TIME";

// NVS namespace and keys
static const char *NVS_NAMESPACE = "time_cfg";
static const char *NVS_KEY_TZ = "timezone";

// Default timezone: CET/CEST (Europe/Budapest)
static const char *DEFAULT_TZ = "CET-1CEST,M3.5.0/2,M10.5.0/3";

// State
static bool s_time_synced = false;
static time_source_t s_time_source = TIME_SOURCE_NONE;
static char s_timezone[64] = {0};
static bool s_initialized = false;

// GPS time tracking - prefer GPS if recently updated
static time_t s_last_gps_sync = 0;
static const int GPS_SYNC_VALIDITY_SEC = 10;  // Consider GPS valid for 10 seconds

static int64_t days_from_civil(int y, unsigned m, unsigned d)
{
    y -= m <= 2;
    const int era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return (int64_t)(era * 146097 + (int)doe - 719468);
}

static bool gps_to_unix_utc(uint16_t year, uint8_t month, uint8_t day,
                            uint8_t hour, uint8_t min, uint8_t sec,
                            time_t *out_unix)
{
    if (!out_unix || year < 1970 || month == 0 || month > 12 || day == 0 || day > 31 ||
        hour > 23 || min > 59 || sec > 59) {
        return false;
    }

    int64_t days = days_from_civil((int)year, (unsigned)month, (unsigned)day);
    if (days < 0) {
        return false;
    }

    int64_t total = days * 86400 + (int64_t)hour * 3600 + (int64_t)min * 60 + sec;
    *out_unix = (time_t)total;
    return true;
}

static void ntp_sync_notification_cb(struct timeval *tv)
{
    // Only use NTP if GPS hasn't synced recently
    if (s_time_source == TIME_SOURCE_GPS) {
        time_t now = time(NULL);
        if (now - s_last_gps_sync < GPS_SYNC_VALIDITY_SEC) {
            ESP_LOGD(TAG, "NTP sync ignored, GPS active");
            return;
        }
    }

    s_time_synced = true;
    s_time_source = TIME_SOURCE_NTP;

    time_t now = tv->tv_sec;
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);

    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
    ESP_LOGI(TAG, "NTP synced: %s (local)", buf);
}

// ============================================================================
// NVS Operations
// ============================================================================

static esp_err_t load_timezone(void)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        // Use default
        strncpy(s_timezone, DEFAULT_TZ, sizeof(s_timezone) - 1);
        return ESP_ERR_NVS_NOT_FOUND;
    }

    size_t len = sizeof(s_timezone);
    ret = nvs_get_str(handle, NVS_KEY_TZ, s_timezone, &len);
    nvs_close(handle);

    if (ret != ESP_OK || strlen(s_timezone) == 0) {
        strncpy(s_timezone, DEFAULT_TZ, sizeof(s_timezone) - 1);
        return ESP_ERR_NVS_NOT_FOUND;
    }

    return ESP_OK;
}

static esp_err_t save_timezone(const char *tz)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_set_str(handle, NVS_KEY_TZ, tz);
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }
    nvs_close(handle);

    return ret;
}

// ============================================================================
// Public API
// ============================================================================

esp_err_t time_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing time system...");

    // Load timezone from NVS
    if (load_timezone() == ESP_OK) {
        ESP_LOGI(TAG, "Loaded timezone: %s", s_timezone);
    } else {
        ESP_LOGI(TAG, "Using default timezone: %s", s_timezone);
    }

    // Set timezone
    setenv("TZ", s_timezone, 1);
    tzset();

    // Configure SNTP (will sync when WiFi is available)
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "time.google.com");
    esp_sntp_set_time_sync_notification_cb(ntp_sync_notification_cb);
    esp_sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);

    esp_sntp_init();

    s_initialized = true;

    ESP_LOGI(TAG, "Time system initialized (waiting for GPS or NTP)");

    // Wait a bit for initial NTP sync (non-blocking if GPS provides time)
    int retry = 0;
    const int max_retry = 10;  // 5 seconds
    while (!s_time_synced && retry < max_retry) {
        vTaskDelay(pdMS_TO_TICKS(500));
        retry++;
    }

    if (s_time_synced) {
        ESP_LOGI(TAG, "Initial time sync from NTP");
    } else {
        ESP_LOGW(TAG, "NTP sync timeout (will use GPS or retry)");
    }

    return ESP_OK;
}

esp_err_t time_set_from_gps(uint16_t year, uint8_t month, uint8_t day,
                            uint8_t hour, uint8_t min, uint8_t sec, bool valid)
{
    if (!valid || year < 2020 || year > 2100) {
        return ESP_ERR_INVALID_ARG;
    }

    // Convert GPS UTC time to Unix timestamp (UTC)
    time_t gps_time = 0;
    if (!gps_to_unix_utc(year, month, day, hour, min, sec, &gps_time)) {
        return ESP_ERR_INVALID_ARG;
    }

    // Set system time
    struct timeval tv = {
        .tv_sec = gps_time,
        .tv_usec = 0
    };
    settimeofday(&tv, NULL);

    s_time_synced = true;
    s_time_source = TIME_SOURCE_GPS;
    s_last_gps_sync = time(NULL);

    // Log only occasionally (first time or every minute)
    static time_t last_log = 0;
    if (last_log == 0 || gps_time - last_log > 60) {
        struct tm local_tm;
        localtime_r(&gps_time, &local_tm);
        char buf[32];
        strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &local_tm);
        ESP_LOGI(TAG, "GPS time: %s (local)", buf);
        last_log = gps_time;
    }

    return ESP_OK;
}

bool time_is_synced(void)
{
    return s_time_synced;
}

time_source_t time_get_source(void)
{
    return s_time_source;
}

time_t time_get_unix(void)
{
    if (!s_time_synced) {
        return 0;
    }
    return time(NULL);
}

esp_err_t time_get_local(struct tm *tm_out)
{
    if (!tm_out) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_time_synced) {
        return ESP_ERR_INVALID_STATE;
    }

    time_t now = time(NULL);
    localtime_r(&now, tm_out);
    return ESP_OK;
}

esp_err_t time_format_iso(char *buf, size_t buf_size)
{
    if (!buf || buf_size < 20) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_time_synced) {
        strncpy(buf, "NOT_SYNCED", buf_size);
        return ESP_ERR_INVALID_STATE;
    }

    time_t now = time(NULL);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);

    strftime(buf, buf_size, "%Y-%m-%dT%H:%M:%S", &timeinfo);
    return ESP_OK;
}

esp_err_t time_format_filename(char *buf, size_t buf_size)
{
    if (!buf || buf_size < 16) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_time_synced) {
        // Fallback: use uptime in seconds
        snprintf(buf, buf_size, "uptime_%lu",
                 (unsigned long)(xTaskGetTickCount() * portTICK_PERIOD_MS / 1000));
        return ESP_ERR_INVALID_STATE;
    }

    time_t now = time(NULL);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);

    strftime(buf, buf_size, "%Y%m%d_%H%M%S", &timeinfo);
    return ESP_OK;
}

const char* time_get_timezone(void)
{
    return s_timezone;
}

esp_err_t time_set_timezone(const char *tz)
{
    if (!tz || strlen(tz) == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Save to NVS
    esp_err_t ret = save_timezone(tz);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to save timezone to NVS");
    }

    // Apply immediately
    strncpy(s_timezone, tz, sizeof(s_timezone) - 1);
    setenv("TZ", s_timezone, 1);
    tzset();

    ESP_LOGI(TAG, "Timezone set: %s", s_timezone);
    return ESP_OK;
}

int time_get_utc_offset_minutes(void)
{
    if (!s_time_synced) {
        return 0;
    }

    time_t now = time(NULL);
    struct tm local_tm, utc_tm;
    localtime_r(&now, &local_tm);
    gmtime_r(&now, &utc_tm);

    // Calculate difference in minutes
    time_t local_time = mktime(&local_tm);
    time_t utc_time = mktime(&utc_tm);

    return (int)((local_time - utc_time) / 60);
}

time_t time_gps_to_local_unix(uint16_t year, uint8_t month, uint8_t day,
                              uint8_t hour, uint8_t min, uint8_t sec)
{
    // Convert GPS UTC time to Unix timestamp (UTC)
    time_t gps_time = 0;
    if (!gps_to_unix_utc(year, month, day, hour, min, sec, &gps_time)) {
        return 0;
    }
    return gps_time;
}
