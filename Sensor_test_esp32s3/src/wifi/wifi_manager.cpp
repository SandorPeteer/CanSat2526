#include "wifi_manager.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_netif.h"
#include "lwip/inet.h"
// #include "mdns.h"
#include "esp_mac.h"
#include <limits.h>
#include <string.h>

static const char *TAG = "WIFI";

// NVS namespace and keys
static const char *NVS_NAMESPACE = "wifi_creds";
static const char *NVS_KEY_SSID = "ssid";
static const char *NVS_KEY_PASS = "password";

// Event group bits
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

// State
static EventGroupHandle_t s_wifi_event_group = NULL;
static esp_netif_t *s_sta_netif = NULL;
static esp_netif_t *s_ap_netif = NULL;
static int s_retry_count = 0;
static int s_max_retry = 5;
static bool s_is_connected = false;
static esp_ip4_addr_t s_ip_addr = {};
static wifi_manager_mode_t s_mode = WIFI_MGR_MODE_NONE;
static char s_ap_ssid[33] = "AstroLink-Setup";
static wifi_manager_config_t s_config = {};
static bool s_initialized = false;

// Forward declarations
static esp_err_t start_sta_mode(const char *ssid, const char *password);
static esp_err_t start_ap_mode(void);

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, "STA started, connecting...");
                esp_wifi_connect();
                break;

            case WIFI_EVENT_STA_DISCONNECTED: {
                s_is_connected = false;
                wifi_event_sta_disconnected_t *disconn = (wifi_event_sta_disconnected_t *)event_data;
                ESP_LOGW(TAG, "Disconnected (reason=%d)", disconn->reason);

                if (s_max_retry == 0 || s_retry_count < s_max_retry) {
                    s_retry_count++;
                    ESP_LOGI(TAG, "Retry %d/%d...", s_retry_count, s_max_retry ? s_max_retry : -1);
                    esp_wifi_connect();
                } else {
                    ESP_LOGE(TAG, "Max retries reached");
                    xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                }
                break;
            }

            case WIFI_EVENT_STA_CONNECTED:
                ESP_LOGI(TAG, "Connected to AP");
                break;

            case WIFI_EVENT_AP_START:
                ESP_LOGI(TAG, "AP started: %s", s_ap_ssid);
                break;

            case WIFI_EVENT_AP_STACONNECTED: {
                wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
                ESP_LOGI(TAG, "Client connected, MAC=" MACSTR, MAC2STR(event->mac));
                break;
            }

            case WIFI_EVENT_AP_STADISCONNECTED: {
                wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
                ESP_LOGI(TAG, "Client disconnected, MAC=" MACSTR, MAC2STR(event->mac));
                break;
            }

            default:
                break;
        }
    } else if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
            s_ip_addr = event->ip_info.ip;
            ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&s_ip_addr));
            s_retry_count = 0;
            s_is_connected = true;
            s_mode = WIFI_MGR_MODE_STA;
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        }
    }
}

// ============================================================================
// NVS Operations
// ============================================================================

static esp_err_t nvs_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS flash erase needed");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

esp_err_t wifi_manager_save_credentials(const char *ssid, const char *password)
{
    if (!ssid || strlen(ssid) == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_set_str(handle, NVS_KEY_SSID, ssid);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS set SSID failed: %s", esp_err_to_name(ret));
        nvs_close(handle);
        return ret;
    }

    ret = nvs_set_str(handle, NVS_KEY_PASS, password ? password : "");
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS set password failed: %s", esp_err_to_name(ret));
        nvs_close(handle);
        return ret;
    }

    ret = nvs_commit(handle);
    nvs_close(handle);

    ESP_LOGI(TAG, "Credentials saved for SSID: %s", ssid);
    return ret;
}

esp_err_t wifi_manager_clear_credentials(void)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) return ret;

    nvs_erase_key(handle, NVS_KEY_SSID);
    nvs_erase_key(handle, NVS_KEY_PASS);
    ret = nvs_commit(handle);
    nvs_close(handle);

    ESP_LOGI(TAG, "Credentials cleared");
    return ret;
}

bool wifi_manager_has_saved_credentials(void)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }

    size_t len = 0;
    esp_err_t ret = nvs_get_str(handle, NVS_KEY_SSID, NULL, &len);
    nvs_close(handle);

    return (ret == ESP_OK && len > 1);
}

static esp_err_t load_credentials(char *ssid, size_t ssid_len, char *password, size_t pass_len)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = nvs_get_str(handle, NVS_KEY_SSID, ssid, &ssid_len);
    if (ret != ESP_OK) {
        nvs_close(handle);
        return ret;
    }

    ret = nvs_get_str(handle, NVS_KEY_PASS, password, &pass_len);
    if (ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(handle);
        return ret;
    }

    nvs_close(handle);
    return ESP_OK;
}

// ============================================================================
// STA Mode
// ============================================================================

static esp_err_t start_sta_mode(const char *ssid, const char *password)
{
    ESP_LOGI(TAG, "Starting STA mode, SSID: %s", ssid);

    // Configure WiFi
    wifi_config_t wifi_cfg = {};
    strncpy((char *)wifi_cfg.sta.ssid, ssid, sizeof(wifi_cfg.sta.ssid) - 1);
    if (password && strlen(password) > 0) {
        strncpy((char *)wifi_cfg.sta.password, password, sizeof(wifi_cfg.sta.password) - 1);
        wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        wifi_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;
    }
    wifi_cfg.sta.pmf_cfg.capable = true;
    wifi_cfg.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    if (s_config.max_tx_power_dbm != INT8_MIN) {
        esp_err_t pow_err = esp_wifi_set_max_tx_power(s_config.max_tx_power_dbm);
        if (pow_err == ESP_OK) {
            ESP_LOGI(TAG, "TX power set to %d dBm", s_config.max_tx_power_dbm);
        }
    }
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    // Wait for connection or failure
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdTRUE, pdFALSE, pdMS_TO_TICKS(30000));

    if (bits & WIFI_CONNECTED_BIT) {
        s_mode = WIFI_MGR_MODE_STA;
        return ESP_OK;
    }

    // Connection failed
    esp_wifi_stop();
    return ESP_FAIL;
}

// ============================================================================
// AP Mode
// ============================================================================

static esp_err_t start_ap_mode(void)
{
    ESP_LOGI(TAG, "Starting AP mode: %s", s_ap_ssid);

    // Create AP netif if needed
    if (!s_ap_netif) {
        s_ap_netif = esp_netif_create_default_wifi_ap();
    }

    wifi_config_t wifi_cfg = {};
    strncpy((char *)wifi_cfg.ap.ssid, s_ap_ssid, sizeof(wifi_cfg.ap.ssid) - 1);
    wifi_cfg.ap.ssid_len = strlen(s_ap_ssid);
    wifi_cfg.ap.channel = 1;
    wifi_cfg.ap.max_connection = 4;

    if (s_config.ap_password && strlen(s_config.ap_password) >= 8) {
        strncpy((char *)wifi_cfg.ap.password, s_config.ap_password, sizeof(wifi_cfg.ap.password) - 1);
        wifi_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        wifi_cfg.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    s_mode = WIFI_MGR_MODE_AP;
    s_is_connected = false;

    // Get AP IP
    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(s_ap_netif, &ip_info);
    ESP_LOGI(TAG, "AP IP: " IPSTR, IP2STR(&ip_info.ip));

    return ESP_OK;
}

// ============================================================================
// Public API
// ============================================================================

esp_err_t wifi_manager_init(const wifi_manager_config_t *config)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    if (config) {
        s_config = *config;
        s_max_retry = config->max_retry > 0 ? config->max_retry : 5;

        if (config->ap_ssid && strlen(config->ap_ssid) > 0) {
            strncpy(s_ap_ssid, config->ap_ssid, sizeof(s_ap_ssid) - 1);
        }
    }

    // Initialize NVS
    ESP_ERROR_CHECK(nvs_init());

    // Create event group
    s_wifi_event_group = xEventGroupCreate();

    // Initialize TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create default WiFi STA
    s_sta_netif = esp_netif_create_default_wifi_sta();

    // Initialize WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handlers
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    s_initialized = true;

    // Try to load saved credentials
    char saved_ssid[33] = {0};
    char saved_pass[65] = {0};
    bool has_saved = (load_credentials(saved_ssid, sizeof(saved_ssid),
                                       saved_pass, sizeof(saved_pass)) == ESP_OK &&
                      strlen(saved_ssid) > 0);

    // Determine which credentials to use
    const char *use_ssid = NULL;
    const char *use_pass = NULL;

    if (has_saved) {
        use_ssid = saved_ssid;
        use_pass = saved_pass;
        ESP_LOGI(TAG, "Using saved credentials: %s", saved_ssid);
    } else if (config && config->ssid && strlen(config->ssid) > 0) {
        use_ssid = config->ssid;
        use_pass = config->password;
        ESP_LOGI(TAG, "Using fallback credentials: %s", config->ssid);
    }

    // Try STA mode if we have credentials
    if (use_ssid) {
        esp_err_t ret = start_sta_mode(use_ssid, use_pass);
        if (ret == ESP_OK) {
            return ESP_OK;
        }
        ESP_LOGW(TAG, "STA connection failed, switching to AP mode");
    }

    // Start AP mode
    return start_ap_mode();
}

wifi_manager_mode_t wifi_manager_get_mode(void)
{
    return s_mode;
}

bool wifi_manager_is_connected(void)
{
    return s_is_connected && s_mode == WIFI_MGR_MODE_STA;
}

esp_err_t wifi_manager_get_ip(char *ip_str)
{
    if (!ip_str) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_mode == WIFI_MGR_MODE_STA && s_is_connected) {
        sprintf(ip_str, IPSTR, IP2STR(&s_ip_addr));
        return ESP_OK;
    } else if (s_mode == WIFI_MGR_MODE_AP && s_ap_netif) {
        esp_netif_ip_info_t ip_info;
        esp_netif_get_ip_info(s_ap_netif, &ip_info);
        sprintf(ip_str, IPSTR, IP2STR(&ip_info.ip));
        return ESP_OK;
    }

    return ESP_ERR_WIFI_NOT_CONNECT;
}

esp_err_t wifi_manager_restart_sta(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Stop current WiFi
    esp_wifi_stop();
    vTaskDelay(pdMS_TO_TICKS(100));

    // Reset state
    s_retry_count = 0;
    s_is_connected = false;
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);

    // Load credentials
    char saved_ssid[33] = {0};
    char saved_pass[65] = {0};
    if (load_credentials(saved_ssid, sizeof(saved_ssid), saved_pass, sizeof(saved_pass)) != ESP_OK ||
        strlen(saved_ssid) == 0) {
        ESP_LOGE(TAG, "No saved credentials");
        return start_ap_mode();
    }

    // Try to connect
    esp_err_t ret = start_sta_mode(saved_ssid, saved_pass);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "STA failed, returning to AP mode");
        return start_ap_mode();
    }

    return ESP_OK;
}

const char* wifi_manager_get_ap_ssid(void)
{
    return s_ap_ssid;
}

void wifi_manager_deinit(void)
{
    if (!s_initialized) return;

    // mdns_free();
    esp_wifi_disconnect();
    esp_wifi_stop();
    esp_wifi_deinit();

    if (s_sta_netif) {
        esp_netif_destroy(s_sta_netif);
        s_sta_netif = NULL;
    }

    if (s_ap_netif) {
        esp_netif_destroy(s_ap_netif);
        s_ap_netif = NULL;
    }

    if (s_wifi_event_group) {
        vEventGroupDelete(s_wifi_event_group);
        s_wifi_event_group = NULL;
    }

    s_is_connected = false;
    s_mode = WIFI_MGR_MODE_NONE;
    s_initialized = false;
}
