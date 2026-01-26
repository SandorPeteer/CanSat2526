#include "status_led.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "LED";

// WS2812 on GPIO 48 (ESP32-S3-DevKitC)
#define LED_GPIO 48

// WS2812 timing (in 10MHz ticks = 100ns resolution)
#define WS2812_T0H_TICKS 4   // 0.4us
#define WS2812_T0L_TICKS 8   // 0.8us
#define WS2812_T1H_TICKS 8   // 0.8us
#define WS2812_T1L_TICKS 4   // 0.4us
#define WS2812_RESET_TICKS 500  // 50us

static rmt_channel_handle_t s_rmt_channel = NULL;
static rmt_encoder_handle_t s_encoder = NULL;
static led_status_t s_current_status = LED_STATUS_OFF;
static led_status_t s_temp_status = LED_STATUS_OFF;
static uint64_t s_temp_until_us = 0;
static uint64_t s_last_update_us = 0;
static uint8_t s_blink_phase = 0;
static bool s_initialized = false;

// Color definitions (R, G, B) - keep brightness low to not blind
typedef struct {
    uint8_t r, g, b;
} rgb_t;

static const rgb_t COLOR_OFF     = {0, 0, 0};
static const rgb_t COLOR_RED     = {40, 0, 0};
static const rgb_t COLOR_GREEN   = {0, 40, 0};
static const rgb_t COLOR_BLUE    = {0, 0, 40};
static const rgb_t COLOR_YELLOW  = {40, 30, 0};
static const rgb_t COLOR_ORANGE  = {40, 15, 0};
static const rgb_t COLOR_CYAN    = {0, 30, 40};
static const rgb_t COLOR_DIM_GREEN = {0, 10, 0};

// WS2812 encoder
typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    int state;
    rmt_symbol_word_t reset_code;
} ws2812_encoder_t;

static size_t ws2812_encode(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                            const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state)
{
    ws2812_encoder_t *led_encoder = __containerof(encoder, ws2812_encoder_t, base);
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;

    switch (led_encoder->state) {
        case 0:  // Send RGB data
            encoded_symbols += led_encoder->bytes_encoder->encode(led_encoder->bytes_encoder, channel,
                                                                   primary_data, data_size, &session_state);
            if (session_state & RMT_ENCODING_COMPLETE) {
                led_encoder->state = 1;  // Move to reset
            }
            if (session_state & RMT_ENCODING_MEM_FULL) {
                *ret_state = (rmt_encode_state_t)(session_state & ~RMT_ENCODING_COMPLETE);
                return encoded_symbols;
            }
            // Fall through
        case 1:  // Send reset code
            encoded_symbols += led_encoder->copy_encoder->encode(led_encoder->copy_encoder, channel,
                                                                  &led_encoder->reset_code, sizeof(led_encoder->reset_code),
                                                                  &session_state);
            if (session_state & RMT_ENCODING_COMPLETE) {
                led_encoder->state = 0;  // Reset state
                *ret_state = RMT_ENCODING_COMPLETE;
            }
            break;
    }
    return encoded_symbols;
}

static esp_err_t ws2812_encoder_reset(rmt_encoder_t *encoder)
{
    ws2812_encoder_t *led_encoder = __containerof(encoder, ws2812_encoder_t, base);
    rmt_encoder_reset(led_encoder->bytes_encoder);
    rmt_encoder_reset(led_encoder->copy_encoder);
    led_encoder->state = 0;
    return ESP_OK;
}

static esp_err_t ws2812_encoder_del(rmt_encoder_t *encoder)
{
    ws2812_encoder_t *led_encoder = __containerof(encoder, ws2812_encoder_t, base);
    rmt_del_encoder(led_encoder->bytes_encoder);
    rmt_del_encoder(led_encoder->copy_encoder);
    free(led_encoder);
    return ESP_OK;
}

static esp_err_t create_ws2812_encoder(rmt_encoder_handle_t *ret_encoder)
{
    ws2812_encoder_t *led_encoder = (ws2812_encoder_t *)calloc(1, sizeof(ws2812_encoder_t));
    if (!led_encoder) return ESP_ERR_NO_MEM;

    led_encoder->base.encode = ws2812_encode;
    led_encoder->base.reset = ws2812_encoder_reset;
    led_encoder->base.del = ws2812_encoder_del;

    // Bytes encoder for RGB data
    // Note: rmt_symbol_word_t field order is: duration0, level0, duration1, level1
    rmt_bytes_encoder_config_t bytes_config = {
        .bit0 = {
            .duration0 = WS2812_T0H_TICKS,
            .level0 = 1,
            .duration1 = WS2812_T0L_TICKS,
            .level1 = 0,
        },
        .bit1 = {
            .duration0 = WS2812_T1H_TICKS,
            .level0 = 1,
            .duration1 = WS2812_T1L_TICKS,
            .level1 = 0,
        },
        .flags = {
            .msb_first = 1,
        }
    };
    esp_err_t ret = rmt_new_bytes_encoder(&bytes_config, &led_encoder->bytes_encoder);
    if (ret != ESP_OK) {
        free(led_encoder);
        return ret;
    }

    // Copy encoder for reset signal
    rmt_copy_encoder_config_t copy_config = {};
    ret = rmt_new_copy_encoder(&copy_config, &led_encoder->copy_encoder);
    if (ret != ESP_OK) {
        rmt_del_encoder(led_encoder->bytes_encoder);
        free(led_encoder);
        return ret;
    }

    // Reset code: low for 50us
    led_encoder->reset_code = (rmt_symbol_word_t){
        .duration0 = WS2812_RESET_TICKS,
        .level0 = 0,
        .duration1 = WS2812_RESET_TICKS,
        .level1 = 0,
    };

    *ret_encoder = &led_encoder->base;
    return ESP_OK;
}

esp_err_t status_led_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    // Configure RMT TX channel
    rmt_tx_channel_config_t tx_config = {
        .gpio_num = (gpio_num_t)LED_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,  // 10MHz
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
        .flags = {
            .invert_out = false,
            .with_dma = false,
        }
    };

    esp_err_t ret = rmt_new_tx_channel(&tx_config, &s_rmt_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create RMT TX channel: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = create_ws2812_encoder(&s_encoder);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create WS2812 encoder: %s", esp_err_to_name(ret));
        rmt_del_channel(s_rmt_channel);
        return ret;
    }

    ret = rmt_enable(s_rmt_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable RMT channel: %s", esp_err_to_name(ret));
        rmt_del_encoder(s_encoder);
        rmt_del_channel(s_rmt_channel);
        return ret;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Status LED initialized on GPIO %d", LED_GPIO);

    // Start with LED off
    status_led_set_rgb(0, 0, 0);
    return ESP_OK;
}

static void set_color(const rgb_t *color)
{
    if (!s_initialized) return;
    // WS2812 is GRB order
    uint8_t grb[3] = {color->g, color->r, color->b};

    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
        .flags = {
            .eot_level = 0,
        }
    };

    rmt_transmit(s_rmt_channel, s_encoder, grb, sizeof(grb), &tx_config);
    rmt_tx_wait_all_done(s_rmt_channel, 100);
}

void status_led_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    rgb_t color = {r, g, b};
    set_color(&color);
}

void status_led_set(led_status_t status)
{
    s_current_status = status;
    s_blink_phase = 0;
    s_last_update_us = esp_timer_get_time();
}

led_status_t status_led_get(void)
{
    return s_current_status;
}

void status_led_flash(led_status_t status, uint32_t duration_ms)
{
    s_temp_status = status;
    s_temp_until_us = esp_timer_get_time() + (duration_ms * 1000ULL);
    s_blink_phase = 0;
}

void status_led_update(void)
{
    if (!s_initialized) return;

    uint64_t now = esp_timer_get_time();

    // Check if we have a temporary status active
    led_status_t active_status = s_current_status;
    if (s_temp_until_us > 0) {
        if (now < s_temp_until_us) {
            active_status = s_temp_status;
        } else {
            s_temp_until_us = 0;  // Temp status expired
        }
    }

    // Calculate time since last phase change (for blinking)
    uint64_t elapsed_ms = (now - s_last_update_us) / 1000;

    switch (active_status) {
        case LED_STATUS_OFF:
            set_color(&COLOR_OFF);
            break;

        case LED_STATUS_BOOT:
            // Solid blue
            set_color(&COLOR_BLUE);
            break;

        case LED_STATUS_WIFI_SEARCH:
            // Yellow slow blink (500ms on, 500ms off)
            if (elapsed_ms >= 500) {
                s_blink_phase = !s_blink_phase;
                s_last_update_us = now;
            }
            set_color(s_blink_phase ? &COLOR_YELLOW : &COLOR_OFF);
            break;

        case LED_STATUS_WIFI_CONNECTED:
            // Green single flash then off (200ms on, 1800ms off)
            if (s_blink_phase == 0 && elapsed_ms >= 200) {
                s_blink_phase = 1;
                s_last_update_us = now;
            } else if (s_blink_phase == 1 && elapsed_ms >= 1800) {
                s_blink_phase = 0;
                s_last_update_us = now;
            }
            set_color(s_blink_phase == 0 ? &COLOR_GREEN : &COLOR_OFF);
            break;

        case LED_STATUS_AP_MODE:
            // Orange double flash (100ms on, 100ms off, 100ms on, 700ms off)
            {
                uint32_t cycle_ms = elapsed_ms % 1000;
                if (cycle_ms < 100 || (cycle_ms >= 200 && cycle_ms < 300)) {
                    set_color(&COLOR_ORANGE);
                } else {
                    set_color(&COLOR_OFF);
                }
            }
            break;

        case LED_STATUS_WS_CLIENT:
            // Dim green steady
            set_color(&COLOR_DIM_GREEN);
            break;

        case LED_STATUS_RECORDING:
            // Red slow pulse (fade in/out over 2 seconds)
            {
                uint32_t cycle_ms = elapsed_ms % 2000;
                uint8_t brightness;
                if (cycle_ms < 1000) {
                    brightness = (cycle_ms * 40) / 1000;  // Fade in
                } else {
                    brightness = ((2000 - cycle_ms) * 40) / 1000;  // Fade out
                }
                rgb_t pulse = {brightness, 0, 0};
                set_color(&pulse);
            }
            break;

        case LED_STATUS_GPS_FIX:
            // Cyan single flash per second
            if (s_blink_phase == 0 && elapsed_ms >= 150) {
                s_blink_phase = 1;
                s_last_update_us = now;
            } else if (s_blink_phase == 1 && elapsed_ms >= 850) {
                s_blink_phase = 0;
                s_last_update_us = now;
            }
            set_color(s_blink_phase == 0 ? &COLOR_CYAN : &COLOR_OFF);
            break;

        case LED_STATUS_ERROR:
            // Red fast blink (100ms on, 100ms off)
            if (elapsed_ms >= 100) {
                s_blink_phase = !s_blink_phase;
                s_last_update_us = now;
            }
            set_color(s_blink_phase ? &COLOR_RED : &COLOR_OFF);
            break;

        default:
            set_color(&COLOR_OFF);
            break;
    }
}
