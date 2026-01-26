#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LED status patterns
 */
typedef enum {
    LED_STATUS_OFF = 0,         // LED off
    LED_STATUS_BOOT,            // Blue solid - booting
    LED_STATUS_WIFI_SEARCH,     // Yellow slow blink - searching for WiFi
    LED_STATUS_WIFI_CONNECTED,  // Green single flash - WiFi connected
    LED_STATUS_AP_MODE,         // Orange double flash - AP mode active
    LED_STATUS_WS_CLIENT,       // Green dim steady - WebSocket client connected
    LED_STATUS_RECORDING,       // Red slow pulse - recording active
    LED_STATUS_GPS_FIX,         // Cyan single flash - GPS has fix
    LED_STATUS_ERROR,           // Red fast blink - error condition
    LED_STATUS_COUNT
} led_status_t;

/**
 * @brief Initialize the status LED (WS2812 on GPIO 48)
 * @return ESP_OK on success
 */
esp_err_t status_led_init(void);

/**
 * @brief Set the current LED status pattern
 * @param status The status pattern to display
 */
void status_led_set(led_status_t status);

/**
 * @brief Get current LED status
 * @return Current status pattern
 */
led_status_t status_led_get(void);

/**
 * @brief Set a temporary status that auto-reverts after duration
 * @param status Temporary status to show
 * @param duration_ms How long to show it (then reverts to previous)
 */
void status_led_flash(led_status_t status, uint32_t duration_ms);

/**
 * @brief Update LED - call this from a task periodically (~50ms)
 * This handles blinking patterns and temporary flashes
 */
void status_led_update(void);

/**
 * @brief Set LED to specific RGB color (for custom use)
 * @param r Red 0-255
 * @param g Green 0-255
 * @param b Blue 0-255
 */
void status_led_set_rgb(uint8_t r, uint8_t g, uint8_t b);

#ifdef __cplusplus
}
#endif
