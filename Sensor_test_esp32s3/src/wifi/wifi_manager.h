#pragma once

#include "esp_err.h"
#include "esp_netif.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief WiFi connection configuration
 */
typedef struct {
    const char *ssid;           // Fallback SSID (used if NVS empty)
    const char *password;       // Fallback password (used if NVS empty)
    int max_retry;              // Max connection retries before AP mode (0 = infinite)
    int8_t max_tx_power_dbm;    // Max WiFi TX power in dBm, INT8_MIN = leave default
    const char *ap_ssid;        // AP mode SSID (default: "AstroLink-Setup")
    const char *ap_password;    // AP mode password (NULL = open, min 8 chars if set)
    const char *hostname;       // mDNS hostname (default: "astrolink")
} wifi_manager_config_t;

/**
 * @brief WiFi mode enum
 */
typedef enum {
    WIFI_MGR_MODE_NONE = 0,
    WIFI_MGR_MODE_STA,
    WIFI_MGR_MODE_AP,
} wifi_manager_mode_t;

/**
 * @brief Initialize WiFi manager
 *
 * Flow:
 * 1. Try to load SSID/password from NVS
 * 2. If found, try to connect as STA
 * 3. If connection fails or no credentials, start AP mode
 * 4. In AP mode, serve captive portal for WiFi setup
 *
 * @param config WiFi configuration
 * @return ESP_OK on success (either STA connected or AP started)
 */
esp_err_t wifi_manager_init(const wifi_manager_config_t *config);

/**
 * @brief Get current WiFi mode
 */
wifi_manager_mode_t wifi_manager_get_mode(void);

/**
 * @brief Check if WiFi is connected (STA mode only)
 */
bool wifi_manager_is_connected(void);

/**
 * @brief Get the current IP address
 *
 * @param ip_str Buffer to store IP string (at least 16 bytes)
 * @return ESP_OK if connected and IP obtained
 */
esp_err_t wifi_manager_get_ip(char *ip_str);

/**
 * @brief Save WiFi credentials to NVS
 *
 * @param ssid WiFi SSID
 * @param password WiFi password
 * @return ESP_OK on success
 */
esp_err_t wifi_manager_save_credentials(const char *ssid, const char *password);

/**
 * @brief Clear saved WiFi credentials from NVS
 */
esp_err_t wifi_manager_clear_credentials(void);

/**
 * @brief Check if credentials are saved in NVS
 */
bool wifi_manager_has_saved_credentials(void);

/**
 * @brief Restart WiFi with saved credentials (after AP setup)
 */
esp_err_t wifi_manager_restart_sta(void);

/**
 * @brief Get AP mode SSID (for display)
 */
const char* wifi_manager_get_ap_ssid(void);

/**
 * @brief Disconnect and deinitialize WiFi
 */
void wifi_manager_deinit(void);

// Legacy compatibility
#define wifi_manager_init_sta(cfg) wifi_manager_init(cfg)

#ifdef __cplusplus
}
#endif
