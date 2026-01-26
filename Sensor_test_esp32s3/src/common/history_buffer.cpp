#include "history_buffer.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <time.h>

static const char *TAG = "HISTORY";

// Ring buffer in PSRAM
static history_entry_t *s_buffer = NULL;
static size_t s_max_entries = 0;
static size_t s_head = 0;      // Next write position
static size_t s_count = 0;     // Number of valid entries
static SemaphoreHandle_t s_mutex = NULL;

esp_err_t history_init(size_t max_entries)
{
    if (s_buffer != NULL) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    // Allocate in PSRAM (SPIRAM)
    size_t size = max_entries * sizeof(history_entry_t);
    s_buffer = (history_entry_t *)heap_caps_malloc(size, MALLOC_CAP_SPIRAM);

    if (!s_buffer) {
        // Fallback to internal RAM if PSRAM not available
        ESP_LOGW(TAG, "PSRAM allocation failed, trying internal RAM");
        s_buffer = (history_entry_t *)malloc(size);
    }

    if (!s_buffer) {
        ESP_LOGE(TAG, "Failed to allocate history buffer (%u bytes)", size);
        return ESP_ERR_NO_MEM;
    }

    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) {
        free(s_buffer);
        s_buffer = NULL;
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    s_max_entries = max_entries;
    s_head = 0;
    s_count = 0;

    bool in_psram = heap_caps_check_integrity(MALLOC_CAP_SPIRAM, false);
    ESP_LOGI(TAG, "History buffer initialized: %u entries (%u KB) in %s",
             max_entries, size / 1024, in_psram ? "PSRAM" : "internal RAM");

    return ESP_OK;
}

void history_add(const sps30_measurement_t *data)
{
    if (!s_buffer || !data || !data->valid) return;

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) != pdTRUE) return;

    history_entry_t *entry = &s_buffer[s_head];
    entry->timestamp_ms = data->timestamp_ms;
    entry->pm1_0 = data->pm1_0;
    entry->pm2_5 = data->pm2_5;
    entry->pm4_0 = data->pm4_0;
    entry->pm10 = data->pm10;

    s_head = (s_head + 1) % s_max_entries;
    if (s_count < s_max_entries) {
        s_count++;
    }

    xSemaphoreGive(s_mutex);
}

size_t history_get(history_entry_t *out, size_t max_entries, size_t offset)
{
    if (!s_buffer || !out || max_entries == 0) return 0;

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return 0;

    size_t available = s_count;
    if (offset >= available) {
        xSemaphoreGive(s_mutex);
        return 0;
    }

    size_t to_copy = available - offset;
    if (to_copy > max_entries) to_copy = max_entries;

    // Calculate start position in ring buffer
    size_t start;
    if (s_count < s_max_entries) {
        start = offset;
    } else {
        start = (s_head + offset) % s_max_entries;
    }

    // Copy entries
    for (size_t i = 0; i < to_copy; i++) {
        size_t idx = (start + i) % s_max_entries;
        out[i] = s_buffer[idx];
    }

    xSemaphoreGive(s_mutex);
    return to_copy;
}

size_t history_count(void)
{
    return s_count;
}

void history_clear(void)
{
    if (!s_buffer) return;

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_head = 0;
        s_count = 0;
        xSemaphoreGive(s_mutex);
    }
}

size_t history_memory_usage(void)
{
    return s_max_entries * sizeof(history_entry_t);
}

size_t history_max_entries(void)
{
    return s_max_entries;
}

size_t history_get_range(history_entry_t *out, size_t max_out, uint32_t time_range_sec, size_t downsample)
{
    if (!s_buffer || !out || max_out == 0 || downsample == 0) return 0;

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return 0;

    if (s_count == 0) {
        xSemaphoreGive(s_mutex);
        return 0;
    }

    // Get newest timestamp
    size_t newest_idx = (s_head == 0) ? s_max_entries - 1 : s_head - 1;
    uint32_t newest_ts = s_buffer[newest_idx].timestamp_ms;
    uint32_t cutoff_ts = 0;

    if (time_range_sec > 0) {
        uint32_t range_ms = time_range_sec * 1000;
        cutoff_ts = (newest_ts > range_ms) ? (newest_ts - range_ms) : 0;
    }

    // Find entries in range and downsample
    size_t copied = 0;
    size_t skip_counter = 0;

    // Start from oldest
    size_t start_idx = (s_count < s_max_entries) ? 0 : s_head;

    for (size_t i = 0; i < s_count && copied < max_out; i++) {
        size_t idx = (start_idx + i) % s_max_entries;
        history_entry_t *entry = &s_buffer[idx];

        // Check time range
        if (time_range_sec > 0 && entry->timestamp_ms < cutoff_ts) {
            continue;
        }

        // Downsample
        if (skip_counter == 0) {
            out[copied++] = *entry;
        }
        skip_counter = (skip_counter + 1) % downsample;
    }

    xSemaphoreGive(s_mutex);
    return copied;
}

// ============================================================================
// LittleFS Recording - Binary format (12 bytes/entry)
// Format: [magic:4][start_ts:4] then per entry: [delta_ts:2][pm1:2][pm25:2][pm4:2][pm10:2] = 10 bytes
// PM values stored as uint16 * 100 (0.01 resolution, max 655.35 µg/m³)
// ============================================================================

#define REC_MAGIC 0x41535452  // "ASTR"
#define REC_VERSION 1

// Binary record header (8 bytes)
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t start_timestamp_sec;
} rec_header_t;

// Binary record entry (10 bytes)
typedef struct __attribute__((packed)) {
    uint16_t delta_sec;   // Seconds since start (max 18 hours per file)
    uint16_t pm1_x100;
    uint16_t pm25_x100;
    uint16_t pm4_x100;
    uint16_t pm10_x100;
} rec_entry_t;

static FILE *s_rec_file = NULL;
static recording_info_t s_rec_info = {};
static SemaphoreHandle_t s_rec_mutex = NULL;
static uint32_t s_rec_start_sec = 0;

static inline uint16_t float_to_u16(float v) {
    if (v < 0) return 0;
    if (v > 655.35f) return 65535;
    return (uint16_t)(v * 100.0f + 0.5f);
}

esp_err_t recording_start(const char *filename)
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

    char path[64];
    snprintf(path, sizeof(path), "/littlefs%s", filename);

    s_rec_file = fopen(path, "wb");
    if (!s_rec_file) {
        ESP_LOGE(TAG, "Failed to open recording file: %s", path);
        xSemaphoreGive(s_rec_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    // Use uptime seconds to keep recording delta consistent with sensor timestamps
    s_rec_start_sec = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    ESP_LOGI(TAG, "Recording with uptime base: %lu sec", (unsigned long)s_rec_start_sec);
    rec_header_t header = {
        .magic = REC_MAGIC,
        .start_timestamp_sec = s_rec_start_sec
    };
    fwrite(&header, sizeof(header), 1, s_rec_file);
    fflush(s_rec_file);

    snprintf(s_rec_info.filename, sizeof(s_rec_info.filename), "%.31s", filename);
    s_rec_info.start_time_ms = s_rec_start_sec * 1000;
    s_rec_info.entry_count = 0;
    s_rec_info.file_size = sizeof(header);

    ESP_LOGI(TAG, "Recording started (binary): %s", filename);
    xSemaphoreGive(s_rec_mutex);
    return ESP_OK;
}

esp_err_t recording_stop(void)
{
    if (!s_rec_mutex) return ESP_OK;

    if (xSemaphoreTake(s_rec_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (s_rec_file) {
        fclose(s_rec_file);
        s_rec_file = NULL;
        ESP_LOGI(TAG, "Recording stopped: %s (%lu entries, %lu bytes)",
                 s_rec_info.filename,
                 (unsigned long)s_rec_info.entry_count,
                 (unsigned long)s_rec_info.file_size);
    }

    xSemaphoreGive(s_rec_mutex);
    return ESP_OK;
}

bool recording_active(void)
{
    return s_rec_file != NULL;
}

esp_err_t recording_get_info(recording_info_t *info)
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

void recording_write(const sps30_measurement_t *data)
{
    if (!s_rec_file || !data || !data->valid) return;

    if (s_rec_mutex && xSemaphoreTake(s_rec_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (s_rec_file) {
            uint32_t now_sec = data->timestamp_ms / 1000;
            rec_entry_t entry = {
                .delta_sec = (uint16_t)(now_sec - s_rec_start_sec),
                .pm1_x100 = float_to_u16(data->pm1_0),
                .pm25_x100 = float_to_u16(data->pm2_5),
                .pm4_x100 = float_to_u16(data->pm4_0),
                .pm10_x100 = float_to_u16(data->pm10)
            };

            fwrite(&entry, sizeof(entry), 1, s_rec_file);
            s_rec_info.entry_count++;
            s_rec_info.file_size += sizeof(entry);

            // Flush every 60 entries (~1 min) for safety
            if (s_rec_info.entry_count % 60 == 0) {
                fflush(s_rec_file);
            }
        }
        xSemaphoreGive(s_rec_mutex);
    }
}
