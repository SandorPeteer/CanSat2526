#pragma once

#include <stddef.h>
#include <stdint.h>
#include "driver/uart.h"
#include "esp_err.h"

#define GPS_DEBUG_RING_BYTES 16384

typedef struct {
    uart_port_t uart_num;
    int tx_pin;
    int rx_pin;
    int baud_rate;
    const char *log_dir;
} gps_logger_config_t;

esp_err_t gps_logger_init(const gps_logger_config_t *cfg);
bool gps_logger_is_ready(void);
esp_err_t gps_logger_dump(size_t max_bytes,
                          uint32_t timeout_ms,
                          size_t *out_written,
                          char *out_rel_path,
                          size_t out_rel_path_len);

esp_err_t gps_logger_debug_start(void);
esp_err_t gps_logger_debug_stop(void);
bool gps_logger_debug_enabled(void);
esp_err_t gps_logger_debug_snapshot(uint8_t *out_buf,
                                    size_t max_len,
                                    size_t *out_len,
                                    uint32_t *out_seq,
                                    uint64_t *out_ts_us,
                                    int *out_anchor_offset,
                                    bool *out_anchor_valid);
esp_err_t gps_logger_debug_info(size_t *out_len,
                                uint32_t *out_seq,
                                uint64_t *out_ts_us,
                                bool *out_anchor_valid);
