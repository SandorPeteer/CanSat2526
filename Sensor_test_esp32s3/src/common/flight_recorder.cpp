#include "flight_recorder.h"
#include "ntp_time.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "FLIGHT_REC";

// ============================================================================
// PSRAM Ring Buffer
// ============================================================================

static flight_record_t *s_buffer = NULL;
static size_t s_max_records = 0;
static size_t s_head = 0;
static size_t s_count = 0;
static SemaphoreHandle_t s_mutex = NULL;

static size_t flight_recorder_count_in_range(uint32_t time_range_sec)
{
    if (!s_buffer) return 0;

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return 0;

    if (s_count == 0) {
        xSemaphoreGive(s_mutex);
        return 0;
    }

    size_t newest_idx = (s_head == 0) ? s_max_records - 1 : s_head - 1;
    uint32_t newest_ts = s_buffer[newest_idx].timestamp;
    uint32_t cutoff_ts = 0;

    if (time_range_sec > 0 && newest_ts > time_range_sec) {
        cutoff_ts = newest_ts - time_range_sec;
    }

    size_t count = 0;
    size_t start_idx = (s_count < s_max_records) ? 0 : s_head;

    for (size_t i = 0; i < s_count; i++) {
        size_t idx = (start_idx + i) % s_max_records;
        flight_record_t *entry = &s_buffer[idx];
        if (time_range_sec > 0 && entry->timestamp < cutoff_ts) {
            continue;
        }
        count++;
    }

    xSemaphoreGive(s_mutex);
    return count;
}

static int flight_record_time_cmp(const void *a, const void *b)
{
    const flight_record_t *ra = (const flight_record_t *)a;
    const flight_record_t *rb = (const flight_record_t *)b;
    if (ra->timestamp < rb->timestamp) return -1;
    if (ra->timestamp > rb->timestamp) return 1;
    return 0;
}

esp_err_t flight_recorder_init(size_t max_records)
{
    if (s_buffer != NULL) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    size_t size = max_records * sizeof(flight_record_t);

    // Allocate in PSRAM
    s_buffer = (flight_record_t *)heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    if (!s_buffer) {
        ESP_LOGE(TAG, "PSRAM allocation failed (%u bytes)", size);
        return ESP_ERR_NO_MEM;
    }

    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) {
        heap_caps_free(s_buffer);
        s_buffer = NULL;
        ESP_LOGE(TAG, "Mutex creation failed");
        return ESP_ERR_NO_MEM;
    }

    s_max_records = max_records;
    s_head = 0;
    s_count = 0;

    ESP_LOGI(TAG, "Flight recorder initialized: %u records (%u KB) in PSRAM",
             max_records, size / 1024);

    return ESP_OK;
}

void flight_recorder_add(const flight_record_t *record)
{
    if (!s_buffer || !record) return;

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) != pdTRUE) return;

    memcpy(&s_buffer[s_head], record, sizeof(flight_record_t));

    s_head = (s_head + 1) % s_max_records;
    if (s_count < s_max_records) {
        s_count++;
    }

    xSemaphoreGive(s_mutex);
}

size_t flight_recorder_get(flight_record_t *out, size_t max_records, size_t offset)
{
    if (!s_buffer || !out || max_records == 0) return 0;

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return 0;

    if (offset >= s_count) {
        xSemaphoreGive(s_mutex);
        return 0;
    }

    size_t to_copy = s_count - offset;
    if (to_copy > max_records) to_copy = max_records;

    size_t start;
    if (s_count < s_max_records) {
        start = offset;
    } else {
        start = (s_head + offset) % s_max_records;
    }

    for (size_t i = 0; i < to_copy; i++) {
        size_t idx = (start + i) % s_max_records;
        out[i] = s_buffer[idx];
    }

    xSemaphoreGive(s_mutex);
    return to_copy;
}

size_t flight_recorder_get_range(flight_record_t *out, size_t max_out,
                                  uint32_t time_range_sec, size_t downsample)
{
    if (!s_buffer || !out || max_out == 0 || downsample == 0) return 0;

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return 0;

    if (s_count == 0) {
        xSemaphoreGive(s_mutex);
        return 0;
    }

    // Get newest timestamp
    size_t newest_idx = (s_head == 0) ? s_max_records - 1 : s_head - 1;
    uint32_t newest_ts = s_buffer[newest_idx].timestamp;
    uint32_t cutoff_ts = 0;

    if (time_range_sec > 0 && newest_ts > time_range_sec) {
        cutoff_ts = newest_ts - time_range_sec;
    }

    size_t copied = 0;
    size_t skip_counter = 0;
    size_t start_idx = (s_count < s_max_records) ? 0 : s_head;

    for (size_t i = 0; i < s_count && copied < max_out; i++) {
        size_t idx = (start_idx + i) % s_max_records;
        flight_record_t *entry = &s_buffer[idx];

        if (time_range_sec > 0 && entry->timestamp < cutoff_ts) {
            continue;
        }

        if (skip_counter == 0) {
            out[copied++] = *entry;
        }
        skip_counter = (skip_counter + 1) % downsample;
    }

    xSemaphoreGive(s_mutex);
    return copied;
}

size_t flight_recorder_count(void)
{
    return s_count;
}

size_t flight_recorder_capacity(void)
{
    return s_max_records;
}

void flight_recorder_clear(void)
{
    if (!s_buffer) return;

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_head = 0;
        s_count = 0;
        xSemaphoreGive(s_mutex);
    }
}

size_t flight_recorder_memory_usage(void)
{
    return s_max_records * sizeof(flight_record_t);
}

// ============================================================================
// File Recording
// ============================================================================

#define FLIGHT_REC_MAGIC 0x464C5452  // "FLTR"
#define FLIGHT_REC_VERSION 1

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t version;
    uint8_t record_size;
    uint16_t reserved;
    uint32_t start_timestamp;
} flight_file_header_t;

static FILE *s_rec_file = NULL;
static flight_recording_info_t s_rec_info = {};
static SemaphoreHandle_t s_rec_mutex = NULL;

esp_err_t flight_recording_start(const char *filename)
{
    if (!filename) return ESP_ERR_INVALID_ARG;

    if (!s_rec_mutex) {
        s_rec_mutex = xSemaphoreCreateMutex();
        if (!s_rec_mutex) return ESP_ERR_NO_MEM;
    }

    if (xSemaphoreTake(s_rec_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (s_rec_file) {
        fclose(s_rec_file);
        s_rec_file = NULL;
    }

    char path[80];
    snprintf(path, sizeof(path), "/littlefs/%s", filename);

    s_rec_file = fopen(path, "wb");
    if (!s_rec_file) {
        ESP_LOGE(TAG, "Failed to open: %s", path);
        xSemaphoreGive(s_rec_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    // Get current timestamp
    uint32_t now_ts = 0;
    if (time_is_synced()) {
        now_ts = (uint32_t)time_get_unix();
    } else {
        now_ts = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    }

    flight_file_header_t header = {
        .magic = FLIGHT_REC_MAGIC,
        .version = FLIGHT_REC_VERSION,
        .record_size = sizeof(flight_record_t),
        .reserved = 0,
        .start_timestamp = now_ts
    };

    fwrite(&header, sizeof(header), 1, s_rec_file);
    fflush(s_rec_file);

    snprintf(s_rec_info.filename, sizeof(s_rec_info.filename), "%.47s", filename);
    s_rec_info.start_timestamp = now_ts;
    s_rec_info.record_count = 0;
    s_rec_info.file_size = sizeof(header);

    ESP_LOGI(TAG, "Recording started: %s", filename);
    xSemaphoreGive(s_rec_mutex);
    return ESP_OK;
}

esp_err_t flight_recording_stop(void)
{
    if (!s_rec_mutex) return ESP_OK;

    if (xSemaphoreTake(s_rec_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (s_rec_file) {
        fflush(s_rec_file);
        fclose(s_rec_file);
        s_rec_file = NULL;
        ESP_LOGI(TAG, "Recording stopped: %s (%lu records, %lu bytes)",
                 s_rec_info.filename,
                 (unsigned long)s_rec_info.record_count,
                 (unsigned long)s_rec_info.file_size);
    }

    xSemaphoreGive(s_rec_mutex);
    return ESP_OK;
}

bool flight_recording_active(void)
{
    return s_rec_file != NULL;
}

void flight_recording_write(const flight_record_t *record)
{
    if (!s_rec_file || !record) return;

    if (s_rec_mutex && xSemaphoreTake(s_rec_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (s_rec_file) {
            fwrite(record, sizeof(flight_record_t), 1, s_rec_file);
            s_rec_info.record_count++;
            s_rec_info.file_size += sizeof(flight_record_t);

            // Flush every 60 records (~1 minute at 1Hz)
            if (s_rec_info.record_count % 60 == 0) {
                fflush(s_rec_file);
            }
        }
        xSemaphoreGive(s_rec_mutex);
    }
}

esp_err_t flight_recording_get_info(flight_recording_info_t *info)
{
    if (!info) return ESP_ERR_INVALID_ARG;

    if (s_rec_mutex && xSemaphoreTake(s_rec_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        *info = s_rec_info;
        if (s_rec_file) {
            s_rec_info.file_size = ftell(s_rec_file);
            info->file_size = s_rec_info.file_size;
        }
        xSemaphoreGive(s_rec_mutex);
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

// ============================================================================
// Chart Data Extraction
// ============================================================================

size_t flight_recorder_get_chart_data(flight_data_type_t type,
                                       float out_values[][4],
                                       uint32_t *out_timestamps,
                                       size_t max_points,
                                       uint32_t time_range_sec,
                                       uint8_t *series_count)
{
    if (!s_buffer || !out_values || !out_timestamps || max_points == 0) return 0;

    // Calculate downsample factor based on requested range
    size_t total = flight_recorder_count_in_range(time_range_sec);
    size_t downsample = 1;
    if (total > max_points) {
        downsample = (total + max_points - 1) / max_points;
    }

    // Temporary buffer for records
    size_t alloc_size = (max_points + 1) * sizeof(flight_record_t);
    flight_record_t *records = (flight_record_t *)heap_caps_malloc(alloc_size, MALLOC_CAP_SPIRAM);
    if (!records) {
        records = (flight_record_t *)malloc(alloc_size);
    }
    if (!records) return 0;

    size_t count = flight_recorder_get_range(records, max_points, time_range_sec, downsample);
    if (count > 1) {
        bool out_of_order = false;
        for (size_t i = 1; i < count; i++) {
            if (records[i - 1].timestamp > records[i].timestamp) {
                out_of_order = true;
                break;
            }
        }
        if (out_of_order) {
            qsort(records, count, sizeof(flight_record_t), flight_record_time_cmp);
        }
    }

    // Set series count based on data type
    uint8_t num_series = 1;
    switch (type) {
        case FLIGHT_DATA_PM:
            num_series = 4;  // PM1, PM2.5, PM4, PM10
            break;
        case FLIGHT_DATA_ALTITUDE:
            num_series = 2;  // Baro alt, GPS alt
            break;
        case FLIGHT_DATA_TEMPERATURE:
            num_series = 2;  // BMP temp, SCD temp
            break;
        default:
            num_series = 1;
            break;
    }
    if (series_count) *series_count = num_series;

    // Extract data
    for (size_t i = 0; i < count; i++) {
        flight_record_t *r = &records[i];
        out_timestamps[i] = r->timestamp;

        switch (type) {
            case FLIGHT_DATA_PM:
                out_values[i][0] = r->pm1 / 10.0f;
                out_values[i][1] = r->pm25 / 10.0f;
                out_values[i][2] = r->pm4 / 10.0f;
                out_values[i][3] = r->pm10 / 10.0f;
                break;

            case FLIGHT_DATA_CO2:
                out_values[i][0] = (float)r->co2;
                break;

            case FLIGHT_DATA_PRESSURE:
                out_values[i][0] = r->pressure_pa / 100.0f;  // hPa
                break;

            case FLIGHT_DATA_ALTITUDE:
                out_values[i][0] = r->baro_alt / 10.0f;      // meters
                out_values[i][1] = r->alt_mm / 1000.0f;      // GPS meters
                break;

            case FLIGHT_DATA_TEMPERATURE:
                out_values[i][0] = r->bmp_temp / 100.0f;
                out_values[i][1] = r->scd_temp / 100.0f;
                break;

            case FLIGHT_DATA_HUMIDITY:
                out_values[i][0] = r->humidity / 100.0f;
                break;

            case FLIGHT_DATA_VELOCITY:
                out_values[i][0] = r->vel_down / 1000.0f;    // m/s
                break;

            case FLIGHT_DATA_HEADING:
                out_values[i][0] = r->heading / 10.0f;       // degrees
                break;

            default:
                break;
        }
    }

    free(records);
    return count;
}
