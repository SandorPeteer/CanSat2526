#include "gps_logger.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>

static const char *TAG = "GPS_LOG";

static bool s_inited = false;
static gps_logger_config_t s_cfg = {};
static SemaphoreHandle_t s_lock = nullptr;
static SemaphoreHandle_t s_debug_lock = nullptr;

static uint8_t *s_ring = nullptr;
static size_t s_ring_size = GPS_DEBUG_RING_BYTES;
static size_t s_ring_pos = 0;
static uint64_t s_ring_total = 0;
static uint64_t s_ubx_abs = 0;
static bool s_ubx_valid = false;
static uint8_t s_prev_byte = 0;
static uint32_t s_debug_seq = 0;
static uint64_t s_debug_ts_us = 0;
static bool s_debug_enabled = false;
static bool s_debug_reset = false;

__attribute__((unused)) static void gps_debug_task(void *arg)
{
    (void)arg;

    uint8_t rx_buf[256];

    for (;;) {
        if (!s_debug_enabled) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (s_debug_reset) {
            uart_flush_input(s_cfg.uart_num);
            if (xSemaphoreTake(s_debug_lock, pdMS_TO_TICKS(200)) == pdTRUE) {
                s_ring_pos = 0;
                s_ring_total = 0;
                s_ubx_valid = false;
                s_prev_byte = 0;
                s_debug_seq = 0;
                s_debug_ts_us = 0;
                xSemaphoreGive(s_debug_lock);
            }
            s_debug_reset = false;
        }

        int r = uart_read_bytes(s_cfg.uart_num, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(20));
        if (r <= 0) {
            continue;
        }

        size_t pos = s_ring_pos;
        uint64_t total = s_ring_total;
        uint8_t prev = s_prev_byte;
        uint64_t ubx_abs = s_ubx_abs;
        bool ubx_updated = false;

        for (int i = 0; i < r; i++) {
            uint8_t b = rx_buf[i];
            s_ring[pos] = b;
            pos = (pos + 1) % s_ring_size;
            total++;

            if (prev == 0xB5 && b == 0x62) {
                ubx_abs = total - 2;  // position of 0xB5
                ubx_updated = true;
            }
            prev = b;
        }

        if (xSemaphoreTake(s_debug_lock, pdMS_TO_TICKS(200)) == pdTRUE) {
            s_ring_pos = pos;
            s_ring_total = total;
            s_prev_byte = prev;
            if (ubx_updated) {
                s_ubx_abs = ubx_abs;
                s_ubx_valid = true;
                s_debug_seq++;
                s_debug_ts_us = esp_timer_get_time();
                ESP_LOGD(TAG, "GPS debug: UBX sync @ %llu", (unsigned long long)ubx_abs);
            }
            xSemaphoreGive(s_debug_lock);
        }
    }
}

esp_err_t gps_logger_init(const gps_logger_config_t *cfg)
{
    if (!cfg || !cfg->log_dir) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_inited) {
        return ESP_OK;
    }

    s_cfg = *cfg;

    // Note: UART is already initialized by ubx_parser, just store config
    s_lock = xSemaphoreCreateMutex();
    if (s_lock == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    s_debug_lock = xSemaphoreCreateMutex();
    if (s_debug_lock == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    s_ring = (uint8_t *)heap_caps_malloc(s_ring_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_ring) {
        return ESP_ERR_NO_MEM;
    }

    // Don't start debug task - it would conflict with ubx_parser reading same UART
    // xTaskCreate(gps_debug_task, "gps_debug", 4096, nullptr, 6, nullptr);

    s_inited = true;
    ESP_LOGI(TAG, "GPS logger ready (log_dir=%s)", s_cfg.log_dir);
    return ESP_OK;
}

bool gps_logger_is_ready(void)
{
    return s_inited;
}

esp_err_t gps_logger_dump(size_t max_bytes,
                          uint32_t timeout_ms,
                          size_t *out_written,
                          char *out_rel_path,
                          size_t out_rel_path_len)
{
    // This function is disabled since ubx_parser owns the UART now
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t gps_logger_debug_start(void)
{
    // Disabled - would conflict with ubx_parser
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t gps_logger_debug_stop(void)
{
    s_debug_enabled = false;
    s_debug_reset = false;
    return ESP_OK;
}

bool gps_logger_debug_enabled(void)
{
    return s_debug_enabled;
}

esp_err_t gps_logger_debug_snapshot(uint8_t *out_buf,
                                    size_t max_len,
                                    size_t *out_len,
                                    uint32_t *out_seq,
                                    uint64_t *out_ts_us,
                                    int *out_anchor_offset,
                                    bool *out_anchor_valid)
{
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t gps_logger_debug_info(size_t *out_len,
                                uint32_t *out_seq,
                                uint64_t *out_ts_us,
                                bool *out_anchor_valid)
{
    return ESP_ERR_NOT_SUPPORTED;
}
