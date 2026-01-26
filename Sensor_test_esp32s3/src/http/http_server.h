#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the HTTP server with WebSocket support
 *
 * @return ESP_OK on success
 */
esp_err_t http_server_start(void);

/**
 * @brief Stop the HTTP server
 *
 * @return ESP_OK on success
 */
esp_err_t http_server_stop(void);

/**
 * @brief Initialize LittleFS (safe to call even if server not running)
 *
 * @return ESP_OK on success
 */
esp_err_t http_server_fs_init(void);

/**
 * @brief Broadcast binary data to all connected WebSocket clients
 *
 * @param data Binary data to send
 * @param len Length of data in bytes
 * @return Number of clients data was sent to
 */
int http_server_ws_broadcast(const uint8_t *data, size_t len);
int http_server_ws_broadcast_text(const char *data, size_t len);

/**
 * @brief Get number of connected WebSocket clients
 *
 * @return Number of active WebSocket connections
 */
int http_server_ws_client_count(void);

/**
 * @brief Send WebSocket ping to all clients
 *
 * Call this periodically (e.g., every 30s) to keep connections alive
 * through NAT/proxies. Returns number of clients pinged.
 */
int http_server_ws_send_ping(void);

#ifdef __cplusplus
}
#endif
