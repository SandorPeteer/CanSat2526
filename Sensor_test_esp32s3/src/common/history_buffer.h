#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "../sps30/sps30.h"

#ifdef __cplusplus
extern "C" {
#endif

// History entry - compact version of measurement for storage
typedef struct {
    uint32_t timestamp_ms;
    float pm1_0;
    float pm2_5;
    float pm4_0;
    float pm10;
} history_entry_t;

// Initialize history buffer in PSRAM
// max_entries: number of entries to store (e.g., 3600 = 1 hour at 1Hz)
esp_err_t history_init(size_t max_entries);

// Add measurement to history
void history_add(const sps30_measurement_t *data);

// Get history entries
// Returns actual number of entries copied
size_t history_get(history_entry_t *out, size_t max_entries, size_t offset);

// Get total number of entries stored
size_t history_count(void);

// Clear history
void history_clear(void);

// Get memory usage in bytes
size_t history_memory_usage(void);

// Get max entries capacity
size_t history_max_entries(void);

// Get entries within time range (for charts)
// time_range_sec: 0 = all, otherwise last N seconds
// downsample: 1 = every entry, N = every Nth entry
size_t history_get_range(history_entry_t *out, size_t max_out, uint32_t time_range_sec, size_t downsample);

// ============================================================================
// LittleFS Recording (optional, for long-term storage)
// ============================================================================

// Start recording to LittleFS file
// filename: e.g., "/rec_20240115.csv"
esp_err_t recording_start(const char *filename);

// Stop current recording
esp_err_t recording_stop(void);

// Check if recording is active
bool recording_active(void);

// Get recording info
typedef struct {
    char filename[32];
    uint32_t start_time_ms;
    uint32_t entry_count;
    size_t file_size;
} recording_info_t;

esp_err_t recording_get_info(recording_info_t *info);

// Write current measurement to recording (called from sps30_task)
void recording_write(const sps30_measurement_t *data);

#ifdef __cplusplus
}
#endif
