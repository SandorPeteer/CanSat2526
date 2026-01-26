#include "sps30.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <string.h>
#include <time.h>
#include "../common/ntp_time.h"

static const char *TAG = "SPS30";

// Module state
static uart_port_t s_uart = UART_NUM_MAX;
static bool s_initialized = false;
static QueueHandle_t s_uart_queue = NULL;

// DMA-aligned RX buffer (must be in internal RAM for GDMA)
static DRAM_ATTR uint8_t s_rx_buf[256];

// ============================================================================
// SHDLC Protocol Implementation with GDMA
// ============================================================================

// Escape byte and write to UART (TX uses DMA automatically)
static void shdlc_write_escaped(uint8_t b)
{
    uint8_t esc[2];
    switch (b) {
        case 0x7E: esc[0] = 0x7D; esc[1] = 0x5E; uart_write_bytes(s_uart, esc, 2); break;
        case 0x7D: esc[0] = 0x7D; esc[1] = 0x5D; uart_write_bytes(s_uart, esc, 2); break;
        case 0x11: esc[0] = 0x7D; esc[1] = 0x31; uart_write_bytes(s_uart, esc, 2); break;
        case 0x13: esc[0] = 0x7D; esc[1] = 0x33; uart_write_bytes(s_uart, esc, 2); break;
        default:   uart_write_bytes(s_uart, &b, 1); break;
    }
}

// Send SHDLC frame (TX via DMA)
static void shdlc_send(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    const uint8_t addr = 0x00;
    uint16_t sum = addr + cmd + len;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    const uint8_t chk = (uint8_t)(0xFF - (sum & 0xFF));

    uint8_t start = 0x7E;
    uart_write_bytes(s_uart, &start, 1);
    shdlc_write_escaped(addr);
    shdlc_write_escaped(cmd);
    shdlc_write_escaped(len);
    for (uint8_t i = 0; i < len; i++) {
        shdlc_write_escaped(data[i]);
    }
    shdlc_write_escaped(chk);
    uart_write_bytes(s_uart, &start, 1);
    uart_wait_tx_done(s_uart, pdMS_TO_TICKS(100));
}

// Read SHDLC frame using DMA RingBuffer
// Returns number of unescaped bytes, or -1 on timeout
static int shdlc_read_frame(uint8_t *out, int out_max, uint32_t timeout_ms)
{
    int64_t start_time = esp_timer_get_time();
    int64_t timeout_us = timeout_ms * 1000;
    bool in_frame = false;
    bool esc = false;
    int n = 0;

    uart_event_t event;

    while ((esp_timer_get_time() - start_time) < timeout_us) {
        uint32_t remain_ms = (timeout_us - (esp_timer_get_time() - start_time)) / 1000;
        if (remain_ms < 10) remain_ms = 10;

        // Wait for UART event (DMA fills RingBuffer, event signals data ready)
        if (xQueueReceive(s_uart_queue, &event, pdMS_TO_TICKS(remain_ms))) {
            if (event.type == UART_DATA) {
                // Read available data from RingBuffer (filled by DMA)
                size_t buffered = 0;
                uart_get_buffered_data_len(s_uart, &buffered);

                if (buffered > 0) {
                    size_t to_read = (buffered > sizeof(s_rx_buf)) ? sizeof(s_rx_buf) : buffered;
                    int bytes_read = uart_read_bytes(s_uart, s_rx_buf, to_read, 0);

                    for (int i = 0; i < bytes_read; i++) {
                        uint8_t b = s_rx_buf[i];

                        if (!in_frame) {
                            if (b == 0x7E) {
                                in_frame = true;
                                esc = false;
                                n = 0;
                            }
                            continue;
                        }

                        // End of frame
                        if (b == 0x7E) {
                            return n;
                        }

                        // Handle escape sequences
                        if (esc) {
                            esc = false;
                            if (b == 0x5E) b = 0x7E;
                            else if (b == 0x5D) b = 0x7D;
                            else if (b == 0x31) b = 0x11;
                            else if (b == 0x33) b = 0x13;
                        } else if (b == 0x7D) {
                            esc = true;
                            continue;
                        }

                        if (n < out_max) {
                            out[n++] = b;
                        }
                    }
                }
            } else if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) {
                ESP_LOGW(TAG, "UART overflow, flushing");
                uart_flush_input(s_uart);
                xQueueReset(s_uart_queue);
            }
        }
    }

    return -1;  // timeout
}

// Generic SHDLC request/response
static bool shdlc_request(uint8_t cmd, const uint8_t *tx_data, uint8_t tx_len,
                          uint8_t expect_cmd, uint8_t *out_data, uint8_t *out_len,
                          uint8_t *out_state, uint32_t timeout_ms)
{
    // Flush RX before sending
    uart_flush_input(s_uart);
    xQueueReset(s_uart_queue);

    shdlc_send(cmd, tx_data, tx_len);

    int64_t start_time = esp_timer_get_time();
    int64_t timeout_us = timeout_ms * 1000;

    while ((esp_timer_get_time() - start_time) < timeout_us) {
        uint32_t remain = (timeout_us - (esp_timer_get_time() - start_time)) / 1000;
        if (remain < 30) remain = 30;

        uint8_t buf[160];
        int n = shdlc_read_frame(buf, sizeof(buf), remain);
        if (n < 0) break;  // timeout
        if (n < 5) continue;  // too short

        uint8_t addr = buf[0];
        uint8_t rcmd = buf[1];
        uint8_t state = buf[2];
        uint8_t len = buf[3];

        if (addr != 0x00) continue;
        if (n != (int)(4 + len + 1)) continue;

        // Verify checksum
        uint16_t sum = 0;
        for (int i = 0; i < n; i++) {
            sum += buf[i];
        }
        if ((uint8_t)sum != 0xFF) continue;

        if (rcmd != expect_cmd) continue;

        if (out_state) *out_state = state;
        if (out_len) *out_len = len;
        if (out_data && len > 0) {
            memcpy(out_data, &buf[4], len);
        }

        return true;
    }

    return false;
}

// ============================================================================
// Wake-up sequence
// ============================================================================

static void sps30_wakeup_sequence(void)
{
    uart_set_line_inverse(s_uart, UART_SIGNAL_TXD_INV);
    vTaskDelay(pdMS_TO_TICKS(50));
    uart_set_line_inverse(s_uart, 0);
    vTaskDelay(pdMS_TO_TICKS(50));

    uart_set_line_inverse(s_uart, UART_SIGNAL_TXD_INV);
    vTaskDelay(pdMS_TO_TICKS(50));
    uart_set_line_inverse(s_uart, 0);
    vTaskDelay(pdMS_TO_TICKS(50));

    shdlc_send(0x11, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    shdlc_send(0x11, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(20));

    uart_flush_input(s_uart);
    xQueueReset(s_uart_queue);
}

// ============================================================================
// Public API Implementation
// ============================================================================

esp_err_t sps30_init(uart_port_t uart_num, gpio_num_t tx_pin, gpio_num_t rx_pin)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    uart_config_t uart_cfg = {};
    uart_cfg.baud_rate = 115200;
    uart_cfg.data_bits = UART_DATA_8_BITS;
    uart_cfg.parity = UART_PARITY_DISABLE;
    uart_cfg.stop_bits = UART_STOP_BITS_1;
    uart_cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_cfg.rx_flow_ctrl_thresh = 0;
    uart_cfg.source_clk = UART_SCLK_DEFAULT;

    esp_err_t ret = uart_param_config(uart_num, &uart_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Install UART driver with DMA (GDMA on ESP32-S3)
    // - RX buffer: 4KB (DMA fills this via RingBuffer)
    // - TX buffer: 1KB
    // - Event queue: 16 events (for UART_DATA notifications)
    ret = uart_driver_install(uart_num, 4096, 1024, 16, &s_uart_queue, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_uart = uart_num;
    s_initialized = true;

    ESP_LOGI(TAG, "SPS30 UART+GDMA initialized (TX=%d, RX=%d)", tx_pin, rx_pin);

    vTaskDelay(pdMS_TO_TICKS(200));
    uart_flush_input(s_uart);

    sps30_wakeup_sequence();

    return ESP_OK;
}

esp_err_t sps30_deinit(void)
{
    if (!s_initialized) {
        return ESP_OK;
    }

    uart_driver_delete(s_uart);
    s_uart = UART_NUM_MAX;
    s_uart_queue = NULL;
    s_initialized = false;

    ESP_LOGI(TAG, "SPS30 deinitialized");
    return ESP_OK;
}

esp_err_t sps30_start_measurement(sps30_output_format_t format)
{
    uint8_t data[2] = {0x01, (uint8_t)format};
    uint8_t state = 0;
    uint8_t len = 0;

    bool ok = shdlc_request(0x00, data, 2, 0x00, NULL, &len, &state, 800);
    if (!ok) {
        ESP_LOGW(TAG, "StartMeasurement: no response");
        return ESP_ERR_TIMEOUT;
    }

    if (state != 0x00) {
        ESP_LOGW(TAG, "StartMeasurement: error state=0x%02X", state);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Measurement started (format=0x%02X)", format);
    return ESP_OK;
}

esp_err_t sps30_stop_measurement(void)
{
    uint8_t state = 0;
    uint8_t len = 0;

    bool ok = shdlc_request(0x01, NULL, 0, 0x01, NULL, &len, &state, 800);
    if (!ok) {
        ESP_LOGW(TAG, "StopMeasurement: no response");
        return ESP_ERR_TIMEOUT;
    }

    if (state & 0x80) {
        ESP_LOGW(TAG, "StopMeasurement: error state=0x%02X", state);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Measurement stopped");
    return ESP_OK;
}

esp_err_t sps30_data_ready(bool *ready)
{
    if (!ready) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[1] = {0};
    uint8_t state = 0;
    uint8_t len = 0;

    bool ok = shdlc_request(0x02, NULL, 0, 0x02, data, &len, &state, 200);
    if (!ok) {
        *ready = false;
        return ESP_ERR_TIMEOUT;
    }

    *ready = (len >= 1 && data[0] == 0x01);
    return ESP_OK;
}

esp_err_t sps30_read_measurement(sps30_measurement_t *data)
{
    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(data, 0, sizeof(*data));

    uint8_t rx_data[64];
    uint8_t state = 0;
    uint8_t len = 0;

    bool ok = shdlc_request(0x03, NULL, 0, 0x03, rx_data, &len, &state, 900);
    if (!ok) {
        ESP_LOGW(TAG, "ReadMeasurement: no response");
        return ESP_ERR_TIMEOUT;
    }

    if (state & 0x80) {
        ESP_LOGW(TAG, "ReadMeasurement: error state=0x%02X", state);
        return ESP_ERR_INVALID_STATE;
    }

    if (len == 0) {
        return ESP_OK;
    }

    if (len != 40) {
        ESP_LOGW(TAG, "ReadMeasurement: unexpected len=%u (expected 40)", len);
        return ESP_ERR_INVALID_SIZE;
    }

    auto be_float = [](const uint8_t *p) -> float {
        uint32_t raw = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
                       ((uint32_t)p[2] << 8) | (uint32_t)p[3];
        float f;
        memcpy(&f, &raw, sizeof(f));
        return f;
    };

    data->pm1_0 = be_float(&rx_data[0]);
    data->pm2_5 = be_float(&rx_data[4]);
    data->pm4_0 = be_float(&rx_data[8]);
    data->pm10 = be_float(&rx_data[12]);
    data->nc0_5 = be_float(&rx_data[16]);
    data->nc1_0 = be_float(&rx_data[20]);
    data->nc2_5 = be_float(&rx_data[24]);
    data->nc4_0 = be_float(&rx_data[28]);
    data->nc10 = be_float(&rx_data[32]);
    data->typical_size = be_float(&rx_data[36]);

    // Use real Unix timestamp (seconds) if time is synced, else uptime
    if (time_is_synced()) {
        data->timestamp_ms = (uint32_t)time_get_unix() * 1000;  // seconds -> ms
    } else {
        data->timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);  // uptime ms fallback
    }
    data->valid = true;

    return ESP_OK;
}

esp_err_t sps30_get_device_info(sps30_device_info_t *info)
{
    if (!info) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(info, 0, sizeof(*info));

    uint8_t data[32];
    uint8_t state = 0;
    uint8_t len = 0;

    uint8_t subcmd = 0x00;
    if (shdlc_request(0xD0, &subcmd, 1, 0xD0, data, &len, &state, 800)) {
        if (!(state & 0x80) && len > 0) {
            size_t copy_len = (len < sizeof(info->product_type) - 1) ? len : sizeof(info->product_type) - 1;
            memcpy(info->product_type, data, copy_len);
        }
    }

    subcmd = 0x03;
    if (shdlc_request(0xD0, &subcmd, 1, 0xD0, data, &len, &state, 800)) {
        if (!(state & 0x80) && len > 0) {
            size_t copy_len = (len < sizeof(info->serial_number) - 1) ? len : sizeof(info->serial_number) - 1;
            memcpy(info->serial_number, data, copy_len);
        }
    }

    if (shdlc_request(0xD1, NULL, 0, 0xD1, data, &len, &state, 800)) {
        if (!(state & 0x80) && len >= 2) {
            info->firmware_major = data[0];
            info->firmware_minor = data[1];
        }
    }

    return ESP_OK;
}

esp_err_t sps30_start_fan_cleaning(void)
{
    uint8_t state = 0;
    uint8_t len = 0;

    bool ok = shdlc_request(0x56, NULL, 0, 0x56, NULL, &len, &state, 800);
    if (!ok) {
        return ESP_ERR_TIMEOUT;
    }

    if (state & 0x80) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Fan cleaning started");
    return ESP_OK;
}

esp_err_t sps30_read_auto_clean_interval(uint32_t *interval_seconds)
{
    if (!interval_seconds) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t subcmd = 0x00;
    uint8_t data[4];
    uint8_t state = 0;
    uint8_t len = 0;

    bool ok = shdlc_request(0x80, &subcmd, 1, 0x80, data, &len, &state, 800);
    if (!ok || (state & 0x80) || len < 4) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    *interval_seconds = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
                        ((uint32_t)data[2] << 8) | (uint32_t)data[3];
    return ESP_OK;
}

esp_err_t sps30_write_auto_clean_interval(uint32_t interval_seconds)
{
    uint8_t data[5];
    data[0] = 0x00;
    data[1] = (interval_seconds >> 24) & 0xFF;
    data[2] = (interval_seconds >> 16) & 0xFF;
    data[3] = (interval_seconds >> 8) & 0xFF;
    data[4] = interval_seconds & 0xFF;

    uint8_t state = 0;
    uint8_t len = 0;

    bool ok = shdlc_request(0x80, data, 5, 0x80, NULL, &len, &state, 800);
    if (!ok || (state & 0x80)) {
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

esp_err_t sps30_sleep(void)
{
    uint8_t state = 0;
    uint8_t len = 0;

    bool ok = shdlc_request(0x10, NULL, 0, 0x10, NULL, &len, &state, 800);
    if (!ok || (state & 0x80)) {
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

esp_err_t sps30_wakeup(void)
{
    sps30_wakeup_sequence();
    return ESP_OK;
}

esp_err_t sps30_reset(void)
{
    uint8_t state = 0;
    uint8_t len = 0;

    bool ok = shdlc_request(0xD3, NULL, 0, 0xD3, NULL, &len, &state, 800);
    if (!ok || (state & 0x80)) {
        return ESP_ERR_INVALID_STATE;
    }

    // Wait for sensor to reboot (~100ms according to datasheet, use 500ms for safety)
    vTaskDelay(pdMS_TO_TICKS(500));

    // Restart measurement (sensor is in idle mode after reset)
    return sps30_start_measurement(SPS30_FORMAT_IEEE754_FLOAT);
}
