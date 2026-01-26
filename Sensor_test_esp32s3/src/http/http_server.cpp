#include "http_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_vfs.h"
#include "esp_littlefs.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_partition.h"
#include "../common/system_stats.h"
#include "../common/ntp_time.h"
#include "../common/shared_data.h"
#include "../sps30/sps30.h"
#include "../common/history_buffer.h"
#include "../common/sensor_control.h"
#include "../common/flight_recorder.h"
#include "../wifi/wifi_manager.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <algorithm>
#include <dirent.h>

static const char *TAG = "HTTP";

// Max sockets for HTTP server/client list
static constexpr size_t MAX_OPEN_SOCKETS = 12;

static httpd_handle_t s_server = NULL;

static void ws_broadcast_sensor_state(void);

// Device info cache
static sps30_device_info_t s_device_info = {};
static bool s_device_info_loaded = false;

// LittleFS mount point
static const char *MOUNT_POINT = "/littlefs";
static bool s_fs_mounted = false;

// ============================================================================
// LittleFS
// ============================================================================

static esp_err_t littlefs_init(void)
{
    if (s_fs_mounted) return ESP_OK;

    esp_vfs_littlefs_conf_t conf = {};
    conf.base_path = MOUNT_POINT;
    conf.partition_label = "littlefs";
    conf.format_if_mount_failed = true;
    conf.dont_mount = false;

    esp_err_t ret = esp_vfs_littlefs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LittleFS mount failed: %s", esp_err_to_name(ret));
        return ret;
    }

    size_t total = 0, used = 0;
    esp_littlefs_info("littlefs", &total, &used);
    ESP_LOGI(TAG, "LittleFS mounted: %u KB used / %u KB total", (unsigned)(used/1024), (unsigned)(total/1024));

    s_fs_mounted = true;
    return ESP_OK;
}

// ============================================================================
// WebSocket client helpers
// ============================================================================

static int ws_get_clients(int *client_fds, size_t max_fds)
{
    if (s_server == NULL || client_fds == NULL || max_fds == 0) return 0;

    size_t fds = max_fds;
    esp_err_t ret = httpd_get_client_list(s_server, &fds, client_fds);
    if (ret != ESP_OK) return 0;

    int ws_count = 0;
    for (size_t i = 0; i < fds; i++) {
        int fd = client_fds[i];
        if (httpd_ws_get_fd_info(s_server, fd) == HTTPD_WS_CLIENT_WEBSOCKET) {
            client_fds[ws_count++] = fd;
        }
    }

    return ws_count;
}

// ============================================================================
// File serving from LittleFS
// ============================================================================

static const char* get_content_type(const char *path)
{
    if (strstr(path, ".html")) return "text/html";
    if (strstr(path, ".css"))  return "text/css";
    if (strstr(path, ".js"))   return "application/javascript";
    if (strstr(path, ".json")) return "application/json";
    if (strstr(path, ".png"))  return "image/png";
    if (strstr(path, ".ico"))  return "image/x-icon";
    return "text/plain";
}

static esp_err_t serve_file(httpd_req_t *req, const char *filepath)
{
    char fullpath[128];
    snprintf(fullpath, sizeof(fullpath), "%s%s", MOUNT_POINT, filepath);

    FILE *f = fopen(fullpath, "r");
    if (!f) {
        ESP_LOGW(TAG, "File not found: %s", fullpath);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, get_content_type(filepath));

    char buf[512];
    size_t read_bytes;
    while ((read_bytes = fread(buf, 1, sizeof(buf), f)) > 0) {
        if (httpd_resp_send_chunk(req, buf, read_bytes) != ESP_OK) {
            fclose(f);
            return ESP_FAIL;
        }
    }
    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);

    return ESP_OK;
}

// ============================================================================
// HTTP Handlers
// ============================================================================

static esp_err_t index_handler(httpd_req_t *req)
{
    // In AP mode, redirect to setup page
    if (wifi_manager_get_mode() == WIFI_MGR_MODE_AP) {
        httpd_resp_set_status(req, "302 Found");
        httpd_resp_set_hdr(req, "Location", "/setup");
        httpd_resp_send(req, NULL, 0);
        return ESP_OK;
    }
    return serve_file(req, "/index.html");
}

static esp_err_t static_handler(httpd_req_t *req)
{
    return serve_file(req, req->uri);
}

static esp_err_t ota_page_handler(httpd_req_t *req)
{
    return serve_file(req, "/ota.html");
}

static esp_err_t files_page_handler(httpd_req_t *req)
{
    return serve_file(req, "/files.html");
}

static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        int fd = httpd_req_to_sockfd(req);
        ESP_LOGI(TAG, "WebSocket handshake from fd=%d", fd);
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(ws_pkt));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "WS recv header failed: %s", esp_err_to_name(ret));
        return ret;
    }

    if (ws_pkt.len == 0) {
        return ESP_OK;
    }

    char *payload = (char *)malloc(ws_pkt.len + 1);
    if (!payload) {
        return ESP_ERR_NO_MEM;
    }

    ws_pkt.payload = (uint8_t *)payload;
    ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "WS recv payload failed: %s", esp_err_to_name(ret));
        free(payload);
        return ret;
    }
    payload[ws_pkt.len] = '\0';

    // History request: "H,<range>,<downsample>"
    if (payload[0] == 'H') {
        uint32_t range_sec = 300;
        size_t downsample = 1;
        char *p = strchr(payload, ',');
        if (p) {
            range_sec = (uint32_t)atoi(p + 1);
            char *p2 = strchr(p + 1, ',');
            if (p2) {
                downsample = (size_t)atoi(p2 + 1);
            }
        }
        if (downsample < 1) downsample = 1;

        size_t max_entries = history_count();
        if (max_entries == 0) {
            const char *empty = "{\"type\":\"history\",\"count\":0,\"range\":0,\"data\":[]}";
            httpd_ws_frame_t out = {
                .final = true,
                .fragmented = false,
                .type = HTTPD_WS_TYPE_TEXT,
                .payload = (uint8_t *)empty,
                .len = strlen(empty)
            };
            httpd_ws_send_frame(req, &out);
            free(payload);
            return ESP_OK;
        }

        const size_t max_points = 4000;
        size_t min_downsample = (max_entries + max_points - 1) / max_points;
        if (downsample < min_downsample) downsample = min_downsample;

        history_entry_t *entries = (history_entry_t *)heap_caps_malloc(
            max_points * sizeof(history_entry_t), MALLOC_CAP_SPIRAM);
        if (!entries) {
            entries = (history_entry_t *)malloc(max_points * sizeof(history_entry_t));
        }
        if (!entries) {
            free(payload);
            return ESP_ERR_NO_MEM;
        }

        size_t count = history_get_range(entries, max_points, range_sec, downsample);

        size_t buf_len = 64 + count * 64;
        char *json = (char *)malloc(buf_len);
        if (!json) {
            free(entries);
            free(payload);
            return ESP_ERR_NO_MEM;
        }

        size_t used = (size_t)snprintf(json, buf_len,
            "{\"type\":\"history\",\"count\":%u,\"range\":%lu,\"data\":[",
            (unsigned)count, (unsigned long)range_sec);

        for (size_t i = 0; i < count && used < buf_len; i++) {
            int wrote = snprintf(json + used, buf_len - used, "%s[%lu,%.1f,%.1f,%.1f,%.1f]",
                i > 0 ? "," : "",
                (unsigned long)entries[i].timestamp_ms,
                entries[i].pm1_0, entries[i].pm2_5,
                entries[i].pm4_0, entries[i].pm10);
            if (wrote < 0) break;
            used += (size_t)wrote;
        }

        if (used + 2 < buf_len) {
            snprintf(json + used, buf_len - used, "]}");
        }

        httpd_ws_frame_t out = {
            .final = true,
            .fragmented = false,
            .type = HTTPD_WS_TYPE_TEXT,
            .payload = (uint8_t *)json,
            .len = strlen(json)
        };
        httpd_ws_send_frame(req, &out);

        free(json);
        free(entries);
        free(payload);
        return ESP_OK;
    }

    // Flight chart request: "F,<type>,<range>,<max>"
    if (payload[0] == 'F') {
        int chart_type = 0;
        uint32_t range_sec = 300;
        size_t max_points = 300;
        char *p = strchr(payload, ',');
        if (p) {
            chart_type = atoi(p + 1);
            char *p2 = strchr(p + 1, ',');
            if (p2) {
                range_sec = (uint32_t)atoi(p2 + 1);
                char *p3 = strchr(p2 + 1, ',');
                if (p3) max_points = (size_t)atoi(p3 + 1);
            }
        }
        if (max_points > 500) max_points = 500;
        if (chart_type < 0 || chart_type >= FLIGHT_DATA_COUNT) chart_type = 0;

        // Allocate buffers
        float (*values)[4] = (float (*)[4])heap_caps_malloc(max_points * sizeof(float[4]), MALLOC_CAP_SPIRAM);
        uint32_t *timestamps = (uint32_t *)heap_caps_malloc(max_points * sizeof(uint32_t), MALLOC_CAP_SPIRAM);
        if (!values || !timestamps) {
            if (values) free(values);
            if (timestamps) free(timestamps);
            free(payload);
            return ESP_ERR_NO_MEM;
        }

        uint8_t series_count = 1;
        size_t count = flight_recorder_get_chart_data((flight_data_type_t)chart_type,
            values, timestamps, max_points, range_sec, &series_count);

        // Binary: [0xF0+type][series][countLo][countHi][data...]
        // Data per point: [ts:4][val0:4][val1:4]... (series floats)
        size_t data_size = 4 + count * (4 + series_count * 4);
        uint8_t *bin = (uint8_t *)heap_caps_malloc(data_size, MALLOC_CAP_SPIRAM);
        if (!bin) bin = (uint8_t *)malloc(data_size);
        if (!bin) {
            free(values);
            free(timestamps);
            free(payload);
            return ESP_ERR_NO_MEM;
        }

        bin[0] = 0xF0 + (uint8_t)chart_type;
        bin[1] = series_count;
        bin[2] = (uint8_t)(count & 0xFF);
        bin[3] = (uint8_t)((count >> 8) & 0xFF);

        size_t off = 4;
        for (size_t i = 0; i < count; i++) {
            memcpy(bin + off, &timestamps[i], 4); off += 4;
            for (uint8_t s = 0; s < series_count; s++) {
                memcpy(bin + off, &values[i][s], 4); off += 4;
            }
        }

        httpd_ws_frame_t out = {
            .final = true, .fragmented = false, .type = HTTPD_WS_TYPE_BINARY,
            .payload = bin, .len = off
        };
        httpd_ws_send_frame(req, &out);

        free(bin);
        free(values);
        free(timestamps);
        free(payload);
        return ESP_OK;
    }

    // Sensor toggle: "S,<name>,<0|1>"
    if (payload[0] == 'S') {
        char *p = strchr(payload, ',');
        char *p2 = p ? strchr(p + 1, ',') : NULL;
        if (p && p2) {
            *p2 = '\0';
            const char *name = p + 1;
            bool enable = atoi(p2 + 1) != 0;
            sensor_control_set_by_name(name, enable);
            ws_broadcast_sensor_state();
        }
        free(payload);
        return ESP_OK;
    }

    // Sensor state query: "Q"
    if (payload[0] == 'Q') {
        ws_broadcast_sensor_state();
        free(payload);
        return ESP_OK;
    }

    // Load flight file: "L,<type>,<filename>"
    // Loads saved .bin file and sends as binary chart data
    if (payload[0] == 'L') {
        int chart_type = 0;
        char filename[48] = {0};
        char *p = strchr(payload, ',');
        if (p) {
            chart_type = atoi(p + 1);
            char *p2 = strchr(p + 1, ',');
            if (p2 && strlen(p2 + 1) < sizeof(filename)) {
                strncpy(filename, p2 + 1, sizeof(filename) - 1);
            }
        }
        if (chart_type < 0 || chart_type >= FLIGHT_DATA_COUNT) chart_type = 0;

        char filepath[80];
        snprintf(filepath, sizeof(filepath), "/littlefs/%s", filename);
        FILE *f = fopen(filepath, "rb");
        if (!f) {
            free(payload);
            return ESP_OK;
        }

        // Read header
        uint32_t magic;
        uint8_t version, record_size;
        if (fread(&magic, 4, 1, f) != 1 || magic != 0x464C5452) {
            fclose(f);
            free(payload);
            return ESP_OK;
        }
        fread(&version, 1, 1, f);
        fread(&record_size, 1, 1, f);
        fseek(f, 12, SEEK_SET);  // Skip header

        // Count records
        fseek(f, 0, SEEK_END);
        size_t file_size = ftell(f);
        size_t record_count = (file_size - 12) / sizeof(flight_record_t);
        fseek(f, 12, SEEK_SET);

        // Downsample if needed
        size_t max_points = 500;
        size_t downsample = record_count > max_points ? record_count / max_points : 1;
        size_t out_count = (record_count + downsample - 1) / downsample;
        if (out_count > max_points) out_count = max_points;

        // Allocate buffers
        uint8_t series = (chart_type == 0 || chart_type == 3 || chart_type == 4) ?
                         (chart_type == 0 ? 4 : 2) : 1;
        size_t data_size = 4 + out_count * (4 + series * 4);
        uint8_t *bin = (uint8_t *)heap_caps_malloc(data_size, MALLOC_CAP_SPIRAM);
        if (!bin) { fclose(f); free(payload); return ESP_ERR_NO_MEM; }

        bin[0] = 0xF0 + (uint8_t)chart_type;
        bin[1] = series;
        size_t written = 0;
        size_t off = 4;

        flight_record_t rec;
        size_t idx = 0;
        while (fread(&rec, sizeof(rec), 1, f) == 1 && written < out_count) {
            if (idx % downsample == 0) {
                memcpy(bin + off, &rec.timestamp, 4); off += 4;
                float v[4] = {0};
                switch (chart_type) {
                    case 0: v[0]=rec.pm1/10.0f; v[1]=rec.pm25/10.0f; v[2]=rec.pm4/10.0f; v[3]=rec.pm10/10.0f; break;
                    case 1: v[0]=(float)rec.co2; break;
                    case 2: v[0]=rec.pressure_pa/100.0f; break;
                    case 3: v[0]=rec.baro_alt/10.0f; v[1]=rec.alt_mm/1000.0f; break;
                    case 4: v[0]=rec.bmp_temp/100.0f; v[1]=rec.scd_temp/100.0f; break;
                    case 5: v[0]=rec.humidity/100.0f; break;
                    case 6: v[0]=rec.vel_down/1000.0f; break;
                    case 7: v[0]=rec.heading/10.0f; break;
                }
                for (uint8_t s = 0; s < series; s++) {
                    memcpy(bin + off, &v[s], 4); off += 4;
                }
                written++;
            }
            idx++;
        }
        fclose(f);

        bin[2] = (uint8_t)(written & 0xFF);
        bin[3] = (uint8_t)((written >> 8) & 0xFF);

        httpd_ws_frame_t out = {
            .final = true, .fragmented = false, .type = HTTPD_WS_TYPE_BINARY,
            .payload = bin, .len = off
        };
        httpd_ws_send_frame(req, &out);
        free(bin);
        free(payload);
        return ESP_OK;
    }

    free(payload);
    return ESP_OK;
}

static void ws_broadcast_sensor_state(void)
{
    sensor_control_state_t state = {};
    sensor_control_get_state(&state);

    char json[128];
    int len = snprintf(json, sizeof(json),
        "{\"type\":\"sensors\",\"gps\":%u,\"sps30\":%u,\"compass\":%u,\"bno\":%u,\"bmp585\":%u,\"scd40\":%u}",
        state.gps ? 1 : 0,
        state.sps30 ? 1 : 0,
        state.compass ? 1 : 0,
        state.bno ? 1 : 0,
        state.bmp585 ? 1 : 0,
        state.scd40 ? 1 : 0);
    if (len > 0) {
        http_server_ws_broadcast_text(json, (size_t)len);
    }
}

// ============================================================================
// REST API Handlers
// ============================================================================

static esp_err_t api_info_handler(httpd_req_t *req)
{
    if (!s_device_info_loaded) {
        sps30_get_device_info(&s_device_info);
        s_device_info_loaded = true;
    }

    uint32_t clean_interval = 0;
    sps30_read_auto_clean_interval(&clean_interval);
    int8_t wifi_rssi = 0;
    bool has_wifi_rssi = system_stats_read_wifi_rssi(&wifi_rssi);

    float cpu_temp = NAN;
    bool has_cpu_temp = system_stats_read_cpu_temp(&cpu_temp);

    char rssi_buf[16];
    if (has_wifi_rssi) {
        snprintf(rssi_buf, sizeof(rssi_buf), "%d", wifi_rssi);
    } else {
        strcpy(rssi_buf, "null");
    }

    char temp_buf[32];
    if (has_cpu_temp) {
        snprintf(temp_buf, sizeof(temp_buf), "%.1f", cpu_temp);
    } else {
        strcpy(temp_buf, "null");
    }

    char json[400];
    snprintf(json, sizeof(json),
        "{\"serial\":\"%s\",\"product\":\"%s\",\"fw_major\":%d,\"fw_minor\":%d,\"clean_interval\":%lu,\"wifi_rssi\":%s,\"cpu_temp_c\":%s}",
        s_device_info.serial_number,
        s_device_info.product_type,
        s_device_info.firmware_major,
        s_device_info.firmware_minor,
        (unsigned long)clean_interval,
        rssi_buf,
        temp_buf);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    return ESP_OK;
}

static esp_err_t api_fan_clean_handler(httpd_req_t *req)
{
    esp_err_t ret = sps30_start_fan_cleaning();

    char json[64];
    if (ret == ESP_OK) {
        snprintf(json, sizeof(json), "{\"ok\":true}");
        ESP_LOGI(TAG, "Fan cleaning triggered via API");
    } else {
        snprintf(json, sizeof(json), "{\"ok\":false,\"error\":\"Failed: %s\"}", esp_err_to_name(ret));
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    return ESP_OK;
}

static esp_err_t api_reset_handler(httpd_req_t *req)
{
    esp_err_t ret = sps30_reset();

    char json[64];
    if (ret == ESP_OK) {
        snprintf(json, sizeof(json), "{\"ok\":true}");
        ESP_LOGI(TAG, "Sensor reset via API");
        s_device_info_loaded = false;
    } else {
        snprintf(json, sizeof(json), "{\"ok\":false,\"error\":\"Failed: %s\"}", esp_err_to_name(ret));
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    return ESP_OK;
}

// ============================================================================
// OTA Handler
// ============================================================================

static esp_err_t api_ota_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "OTA update started, size=%d", req->content_len);

    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (!update_partition) {
        ESP_LOGE(TAG, "No OTA partition found");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Writing to partition: %s (offset=0x%lx, size=%lu)",
             update_partition->label, update_partition->address, update_partition->size);

    esp_ota_handle_t ota_handle;
    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
        return ESP_FAIL;
    }

    char *buf = (char *)malloc(4096);
    if (!buf) {
        esp_ota_abort(ota_handle);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory allocation failed");
        return ESP_FAIL;
    }

    int received = 0;
    int total = req->content_len;
    int ret;

    while (received < total) {
        ret = httpd_req_recv(req, buf, std::min(4096, total - received));
        if (ret <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) continue;
            ESP_LOGE(TAG, "Receive error: %d", ret);
            free(buf);
            esp_ota_abort(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive failed");
            return ESP_FAIL;
        }

        err = esp_ota_write(ota_handle, buf, ret);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            free(buf);
            esp_ota_abort(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA write failed");
            return ESP_FAIL;
        }

        received += ret;
        if ((received % (100 * 1024)) < 4096) {
            ESP_LOGI(TAG, "OTA progress: %d / %d KB", received / 1024, total / 1024);
        }
    }

    free(buf);

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA end failed");
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Set boot partition failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OTA update successful! Rebooting...");
    httpd_resp_sendstr(req, "OK");

    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();

    return ESP_OK;
}

// ============================================================================
// LittleFS OTA (raw image upload)
// ============================================================================

static esp_err_t api_fs_ota_handler(httpd_req_t *req)
{
    if (req->content_len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
        return ESP_FAIL;
    }

    const esp_partition_t *part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "littlefs");
    if (!part) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "LittleFS partition not found");
        return ESP_FAIL;
    }

    if ((size_t)req->content_len > part->size) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Image too large for partition");
        return ESP_FAIL;
    }

    if (s_fs_mounted) {
        esp_vfs_littlefs_unregister("littlefs");
        s_fs_mounted = false;
    }

    size_t erase_size = ((size_t)req->content_len + 4095) & ~((size_t)4095);
    if (erase_size > part->size) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Erase size exceeds partition");
        return ESP_FAIL;
    }

    esp_err_t err = esp_partition_erase_range(part, 0, erase_size);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Erase failed");
        return ESP_FAIL;
    }

    char *buf = (char *)malloc(4096);
    if (!buf) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory allocation failed");
        return ESP_FAIL;
    }

    size_t offset = 0;
    while (offset < (size_t)req->content_len) {
        int to_read = std::min(4096, (int)((size_t)req->content_len - offset));
        int r = httpd_req_recv(req, buf, to_read);
        if (r <= 0) {
            if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
            free(buf);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive failed");
            return ESP_FAIL;
        }

        err = esp_partition_write(part, offset, buf, r);
        if (err != ESP_OK) {
            free(buf);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Write failed");
            return ESP_FAIL;
        }

        offset += (size_t)r;
    }

    free(buf);

    httpd_resp_sendstr(req, "OK");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

// ============================================================================
// LittleFS File Upload (for remote web UI updates)
// ============================================================================

static esp_err_t api_fs_upload_handler(httpd_req_t *req)
{
    if (!s_fs_mounted) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "LittleFS not mounted");
        return ESP_FAIL;
    }

    // Get filename from query string: /api/fs-upload?file=/index.html
    char query[64] = {0};
    char filename[48] = {0};

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing ?file= parameter");
        return ESP_FAIL;
    }

    if (httpd_query_key_value(query, "file", filename, sizeof(filename)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing file parameter");
        return ESP_FAIL;
    }

    // Build full path
    char filepath[80];
    snprintf(filepath, sizeof(filepath), "%s%s", MOUNT_POINT, filename);

    ESP_LOGI(TAG, "FS upload: %s (%d bytes)", filepath, req->content_len);

    // Open file for writing
    FILE *f = fopen(filepath, "wb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open file for writing: %s", filepath);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "File open failed");
        return ESP_FAIL;
    }

    char *buf = (char *)malloc(2048);
    if (!buf) {
        fclose(f);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory allocation failed");
        return ESP_FAIL;
    }

    int received = 0;
    int total = req->content_len;
    int ret;

    while (received < total) {
        ret = httpd_req_recv(req, buf, std::min(2048, total - received));
        if (ret <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) continue;
            ESP_LOGE(TAG, "Receive error: %d", ret);
            free(buf);
            fclose(f);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive failed");
            return ESP_FAIL;
        }

        size_t written = fwrite(buf, 1, ret, f);
        if (written != (size_t)ret) {
            ESP_LOGE(TAG, "Write error: wrote %d of %d", (int)written, ret);
            free(buf);
            fclose(f);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Write failed");
            return ESP_FAIL;
        }

        received += ret;
    }

    free(buf);
    fclose(f);

    ESP_LOGI(TAG, "FS upload complete: %s (%d bytes)", filename, received);

    httpd_resp_set_type(req, "application/json");
    char json[96];
    snprintf(json, sizeof(json), "{\"ok\":true,\"file\":\"%s\",\"size\":%d}", filename, received);
    httpd_resp_sendstr(req, json);
    return ESP_OK;
}

// List files in LittleFS
static esp_err_t api_fs_list_handler(httpd_req_t *req)
{
    if (!s_fs_mounted) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "LittleFS not mounted");
        return ESP_FAIL;
    }

    DIR *dir = opendir(MOUNT_POINT);
    if (!dir) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to open directory");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr_chunk(req, "{\"files\":[");

    struct dirent *entry;
    bool first = true;
    char buf[96];
    char name[32];

    while ((entry = readdir(dir)) != NULL) {
        char filepath[64];
        // Truncate long filenames
        snprintf(name, sizeof(name), "%.31s", entry->d_name);
        snprintf(filepath, sizeof(filepath), "%s/%s", MOUNT_POINT, name);

        struct stat st;
        if (stat(filepath, &st) == 0) {
            snprintf(buf, sizeof(buf), "%s{\"name\":\"%.31s\",\"size\":%ld}",
                     first ? "" : ",", name, st.st_size);
            httpd_resp_sendstr_chunk(req, buf);
            first = false;
        }
    }

    closedir(dir);
    httpd_resp_sendstr_chunk(req, "]}");
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

// AstroLink favicon (embedded SVG as ICO is complex, serve SVG via data URI redirect)
static const char FAVICON_SVG[] =
    "<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 32 32'>"
    "<defs><linearGradient id='g' x1='0%' y1='0%' x2='100%' y2='100%'>"
    "<stop offset='0%' style='stop-color:#1e3a5f'/>"
    "<stop offset='100%' style='stop-color:#0d1b2a'/>"
    "</linearGradient></defs>"
    "<circle cx='16' cy='16' r='15' fill='url(#g)' stroke='#4a9eff' stroke-width='1'/>"
    "<circle cx='16' cy='12' r='3' fill='#ffd700'/>"  // Star
    "<path d='M8 22 Q16 18 24 22' stroke='#4a9eff' stroke-width='2' fill='none'/>"  // Arc
    "<circle cx='10' cy='20' r='1.5' fill='#4a9eff'/>"  // Dot
    "<circle cx='22' cy='20' r='1.5' fill='#4a9eff'/>"  // Dot
    "<path d='M16 12 L16 6 M16 12 L20 8 M16 12 L12 8' stroke='#ffd700' stroke-width='1'/>"  // Star rays
    "</svg>";

static esp_err_t favicon_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "image/svg+xml");
    httpd_resp_set_hdr(req, "Cache-Control", "public, max-age=86400");
    httpd_resp_send(req, FAVICON_SVG, strlen(FAVICON_SVG));
    return ESP_OK;
}

// ============================================================================
// History & Recording API
// ============================================================================

// GET /api/history?range=300&downsample=1
// range: seconds (0=all, 300=5min, 3600=1h, 86400=24h)
// downsample: 1=all points, 10=every 10th, etc
static esp_err_t api_history_handler(httpd_req_t *req)
{
    char query[64] = {0};
    uint32_t range_sec = 300;  // Default 5 minutes
    size_t downsample = 1;

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char val[16];
        if (httpd_query_key_value(query, "range", val, sizeof(val)) == ESP_OK) {
            range_sec = atoi(val);
        }
        if (httpd_query_key_value(query, "downsample", val, sizeof(val)) == ESP_OK) {
            downsample = atoi(val);
            if (downsample < 1) downsample = 1;
        }
    }

    size_t max_points = history_count();
    if (max_points == 0) {
        httpd_resp_set_type(req, "application/json");
        char empty[64];
        snprintf(empty, sizeof(empty), "{\"count\":0,\"range\":%lu,\"data\":[]}",
                 (unsigned long)range_sec);
        httpd_resp_sendstr(req, empty);
        return ESP_OK;
    }

    history_entry_t *entries = (history_entry_t *)heap_caps_malloc(
        max_points * sizeof(history_entry_t), MALLOC_CAP_SPIRAM);
    if (!entries) {
        entries = (history_entry_t *)malloc(max_points * sizeof(history_entry_t));
    }
    if (!entries) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No memory");
        return ESP_FAIL;
    }

    size_t count = history_get_range(entries, max_points, range_sec, downsample);

    httpd_resp_set_type(req, "application/json");

    // Stream JSON array
    httpd_resp_sendstr_chunk(req, "{\"count\":");
    char buf[128];
    snprintf(buf, sizeof(buf), "%u,\"range\":%lu,\"data\":[", (unsigned)count, (unsigned long)range_sec);
    httpd_resp_sendstr_chunk(req, buf);

    for (size_t i = 0; i < count; i++) {
        snprintf(buf, sizeof(buf), "%s[%lu,%.1f,%.1f,%.1f,%.1f]",
                 i > 0 ? "," : "",
                 (unsigned long)entries[i].timestamp_ms,
                 entries[i].pm1_0, entries[i].pm2_5,
                 entries[i].pm4_0, entries[i].pm10);
        httpd_resp_sendstr_chunk(req, buf);
    }

    httpd_resp_sendstr_chunk(req, "]}");
    httpd_resp_sendstr_chunk(req, NULL);

    free(entries);
    return ESP_OK;
}

// GET /api/export?range=3600
// Returns CSV file for download
static esp_err_t api_export_handler(httpd_req_t *req)
{
    char query[32] = {0};
    uint32_t range_sec = 3600;  // Default 1 hour

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char val[16];
        if (httpd_query_key_value(query, "range", val, sizeof(val)) == ESP_OK) {
            range_sec = atoi(val);
        }
    }

    // Get all points (no downsampling for export)
    const size_t MAX_EXPORT = 86400;  // Max 24h
    history_entry_t *entries = (history_entry_t *)heap_caps_malloc(
        MAX_EXPORT * sizeof(history_entry_t), MALLOC_CAP_SPIRAM);

    if (!entries) {
        // Fallback to smaller buffer
        entries = (history_entry_t *)malloc(3600 * sizeof(history_entry_t));
        if (!entries) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No memory");
            return ESP_FAIL;
        }
    }

    size_t count = history_get_range(entries, MAX_EXPORT, range_sec, 1);

    httpd_resp_set_type(req, "text/csv");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"astrolink_export.csv\"");

    // CSV header
    httpd_resp_sendstr_chunk(req, "timestamp_ms,pm1_0,pm2_5,pm4_0,pm10\n");

    char buf[64];
    for (size_t i = 0; i < count; i++) {
        snprintf(buf, sizeof(buf), "%lu,%.2f,%.2f,%.2f,%.2f\n",
                 (unsigned long)entries[i].timestamp_ms,
                 entries[i].pm1_0, entries[i].pm2_5,
                 entries[i].pm4_0, entries[i].pm10);
        httpd_resp_sendstr_chunk(req, buf);
    }

    httpd_resp_sendstr_chunk(req, NULL);
    free(entries);
    return ESP_OK;
}

// POST /api/recording/start - Start flight data recording
static esp_err_t api_recording_start_handler(httpd_req_t *req)
{
    // Generate filename with timestamp
    char stamp[32] = {0};
    if (time_format_filename(stamp, sizeof(stamp)) != ESP_OK || stamp[0] == '\0') {
        snprintf(stamp, sizeof(stamp), "uptime_%lu",
                 (unsigned long)(esp_timer_get_time() / 1000000ULL));
    }
    char filename[48];
    snprintf(filename, sizeof(filename), "flight_%s.bin", stamp);

    esp_err_t ret = flight_recording_start(filename);

    char json[96];
    if (ret == ESP_OK) {
        snprintf(json, sizeof(json), "{\"ok\":true,\"file\":\"%s\"}", filename);
    } else {
        snprintf(json, sizeof(json), "{\"ok\":false,\"error\":\"%s\"}", esp_err_to_name(ret));
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json);
    return ESP_OK;
}

// POST /api/recording/stop
static esp_err_t api_recording_stop_handler(httpd_req_t *req)
{
    flight_recording_info_t info = {};
    flight_recording_get_info(&info);

    esp_err_t ret = flight_recording_stop();

    char json[128];
    snprintf(json, sizeof(json), "{\"ok\":%s,\"file\":\"%s\",\"records\":%lu}",
             ret == ESP_OK ? "true" : "false",
             info.filename,
             (unsigned long)info.record_count);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json);
    return ESP_OK;
}

// GET /api/recording/status
static esp_err_t api_recording_status_handler(httpd_req_t *req)
{
    flight_recording_info_t info = {};
    bool active = flight_recording_active();

    if (active) {
        flight_recording_get_info(&info);
    }

    // Calculate duration
    uint32_t now_ts = 0;
    if (time_is_synced()) {
        now_ts = (uint32_t)time_get_unix();
    } else {
        now_ts = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    }
    uint32_t dur_sec = (active && now_ts >= info.start_timestamp)
        ? (now_ts - info.start_timestamp)
        : 0;

    char json[192];
    snprintf(json, sizeof(json),
             "{\"active\":%s,\"file\":\"%s\",\"records\":%lu,\"size\":%lu,\"duration_sec\":%lu,\"buffer_count\":%lu,\"buffer_capacity\":%lu}",
             active ? "true" : "false",
             info.filename,
             (unsigned long)info.record_count,
             (unsigned long)info.file_size,
             (unsigned long)dur_sec,
             (unsigned long)flight_recorder_count(),
             (unsigned long)flight_recorder_capacity());

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json);
    return ESP_OK;
}

// GET /api/fs-download?file=/rec_123.bin
// Downloads file and converts binary to CSV on the fly
static esp_err_t api_fs_download_handler(httpd_req_t *req)
{
    if (!s_fs_mounted) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "LittleFS not mounted");
        return ESP_FAIL;
    }

    char query[96] = {0};
    char filename[48] = {0};
    char raw_param[8] = {0};

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "file", filename, sizeof(filename)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing ?file= parameter");
        return ESP_FAIL;
    }

    // Check for raw=1 parameter (download binary as-is)
    bool raw_download = false;
    if (httpd_query_key_value(query, "raw", raw_param, sizeof(raw_param)) == ESP_OK) {
        raw_download = (raw_param[0] == '1');
    }

    char filepath[80];
    snprintf(filepath, sizeof(filepath), "%s%s", MOUNT_POINT, filename);

    FILE *f = fopen(filepath, "rb");
    if (!f) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
        return ESP_FAIL;
    }

    // Check if it's a flight recording file (and not raw download)
    bool is_flight_bin = strstr(filename, "flight_") != NULL && strstr(filename, ".bin") != NULL && !raw_download;

    if (is_flight_bin) {
        // Read and validate flight header
        uint32_t magic;
        uint8_t version, record_size;
        uint16_t reserved;
        uint32_t start_ts;

        if (fread(&magic, 4, 1, f) != 1 || magic != 0x464C5452) {  // "FLTR"
            fclose(f);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Invalid flight file");
            return ESP_FAIL;
        }
        fread(&version, 1, 1, f);
        fread(&record_size, 1, 1, f);
        fread(&reserved, 2, 1, f);
        fread(&start_ts, 4, 1, f);

        // Convert to CSV for download
        httpd_resp_set_type(req, "text/csv");
        char disposition[96];
        snprintf(disposition, sizeof(disposition), "attachment; filename=\"%.48s.csv\"", filename);
        httpd_resp_set_hdr(req, "Content-Disposition", disposition);

        // Full flight data CSV header
        httpd_resp_sendstr_chunk(req,
            "timestamp,lat,lon,alt_gps_m,vel_down_ms,fix,sats,"
            "qi,qj,qk,qw,imu_acc,"
            "mag_x,mag_y,mag_z,heading,"
            "pressure_hpa,temp_bmp,alt_baro_m,"
            "co2,temp_scd,humidity,"
            "pm1,pm25,pm4,pm10\n");

        flight_record_t rec;
        char line[256];
        while (fread(&rec, sizeof(rec), 1, f) == 1) {
            snprintf(line, sizeof(line),
                "%lu,%.7f,%.7f,%.1f,%.2f,%d,%d,"
                "%.4f,%.4f,%.4f,%.4f,%d,"
                "%d,%d,%d,%.1f,"
                "%.2f,%.2f,%.1f,"
                "%d,%.2f,%.2f,"
                "%.1f,%.1f,%.1f,%.1f\n",
                (unsigned long)rec.timestamp,
                rec.lat / 1e7, rec.lon / 1e7, rec.alt_mm / 1000.0f, rec.vel_down / 1000.0f,
                rec.fix_type, rec.sats,
                rec.qi / 16384.0f, rec.qj / 16384.0f, rec.qk / 16384.0f, rec.qw / 16384.0f,
                rec.imu_accuracy,
                rec.mag_x, rec.mag_y, rec.mag_z, rec.heading / 10.0f,
                rec.pressure_pa / 100.0f, rec.bmp_temp / 100.0f, rec.baro_alt / 10.0f,
                rec.co2, rec.scd_temp / 100.0f, rec.humidity / 100.0f,
                rec.pm1 / 10.0f, rec.pm25 / 10.0f, rec.pm4 / 10.0f, rec.pm10 / 10.0f);
            httpd_resp_sendstr_chunk(req, line);
        }
        httpd_resp_sendstr_chunk(req, NULL);
    } else {
        // Raw file download
        httpd_resp_set_type(req, "application/octet-stream");
        char disposition[96];
        snprintf(disposition, sizeof(disposition), "attachment; filename=\"%.48s\"", filename);
        httpd_resp_set_hdr(req, "Content-Disposition", disposition);

        char buf[512];
        size_t read;
        while ((read = fread(buf, 1, sizeof(buf), f)) > 0) {
            httpd_resp_send_chunk(req, buf, read);
        }
        httpd_resp_send_chunk(req, NULL, 0);
    }

    fclose(f);
    return ESP_OK;
}

// GET /api/gps - returns GPS data (full NAV-PVT)
static esp_err_t api_gps_handler(httpd_req_t *req)
{
    ubx_nav_pvt_t gps;
    shared_data_get_gps(&gps);

    char json[512];
    snprintf(json, sizeof(json),
        "{\"fix\":%d,\"sat\":%d,\"lat\":%.7f,\"lon\":%.7f,\"alt\":%.1f,"
        "\"height\":%.1f,\"speed\":%.1f,\"heading\":%.1f,"
        "\"hacc\":%lu,\"vacc\":%lu,\"sacc\":%lu,\"pdop\":%u,"
        "\"veln\":%ld,\"vele\":%ld,\"veld\":%ld,"
        "\"flags\":%u,\"flags2\":%u,\"valid\":%u,"
        "\"itow\":%lu,\"tacc\":%lu,"
        "\"time\":\"%04d-%02d-%02dT%02d:%02d:%02d\"}",
        gps.fix_type, gps.num_sv, gps.lat_deg, gps.lon_deg, gps.alt_m,
        gps.height / 1000.0f, gps.speed_kmh, gps.heading_deg,
        (unsigned long)gps.hacc, (unsigned long)gps.vacc, (unsigned long)gps.sacc, gps.pdop,
        (long)gps.vel_n, (long)gps.vel_e, (long)gps.vel_d,
        gps.flags, gps.flags2, gps.valid,
        (unsigned long)gps.itow, (unsigned long)gps.tacc,
        gps.year, gps.month, gps.day, gps.hour, gps.min, gps.sec);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json);
    return ESP_OK;
}

// GET /api/compass - returns magnetometer data
static esp_err_t api_compass_handler(httpd_req_t *req)
{
    qmc5883l_data_t mag;
    shared_data_get_compass(&mag);

    char json[96];
    snprintf(json, sizeof(json),
        "{\"x\":%d,\"y\":%d,\"z\":%d,\"heading\":%.1f}",
        mag.x, mag.y, mag.z, mag.heading_deg);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json);
    return ESP_OK;
}

// DELETE /api/fs-delete?file=/rec_123.bin
static esp_err_t api_fs_delete_handler(httpd_req_t *req)
{
    if (!s_fs_mounted) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "LittleFS not mounted");
        return ESP_FAIL;
    }

    char query[64] = {0};
    char filename[48] = {0};

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "file", filename, sizeof(filename)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing ?file= parameter");
        return ESP_FAIL;
    }

    char filepath[80];
    snprintf(filepath, sizeof(filepath), "%s%s", MOUNT_POINT, filename);

    if (remove(filepath) == 0) {
        ESP_LOGI(TAG, "Deleted: %s", filename);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":true}");
    } else {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
        return ESP_FAIL;
    }

    return ESP_OK;
}

// ============================================================================
// Flight Data API
// ============================================================================

// GET /api/flight/chart?type=0&range=300&max=200
// type: 0=PM, 1=CO2, 2=Pressure, 3=Altitude, 4=Temperature, 5=Humidity, 6=Velocity, 7=Heading
static esp_err_t api_flight_chart_handler(httpd_req_t *req)
{
    char query[64] = {0};
    char type_str[4] = "0";
    char range_str[8] = "300";
    char max_str[8] = "200";

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        httpd_query_key_value(query, "type", type_str, sizeof(type_str));
        httpd_query_key_value(query, "range", range_str, sizeof(range_str));
        httpd_query_key_value(query, "max", max_str, sizeof(max_str));
    }

    flight_data_type_t data_type = (flight_data_type_t)atoi(type_str);
    uint32_t time_range = (uint32_t)atoi(range_str);
    size_t max_points = (size_t)atoi(max_str);

    if (data_type >= FLIGHT_DATA_COUNT) data_type = FLIGHT_DATA_PM;
    if (max_points > 500) max_points = 500;

    // Allocate output arrays in PSRAM
    size_t alloc_size = max_points * sizeof(float) * 4;
    float (*values)[4] = (float (*)[4])heap_caps_malloc(alloc_size, MALLOC_CAP_SPIRAM);
    uint32_t *timestamps = (uint32_t *)heap_caps_malloc(max_points * sizeof(uint32_t), MALLOC_CAP_SPIRAM);

    if (!values || !timestamps) {
        if (values) heap_caps_free(values);
        if (timestamps) heap_caps_free(timestamps);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory allocation failed");
        return ESP_FAIL;
    }

    uint8_t series_count = 0;
    size_t count = flight_recorder_get_chart_data(data_type, values, timestamps,
                                                   max_points, time_range, &series_count);

    // Build JSON response with chunked encoding
    httpd_resp_set_type(req, "application/json");

    // Data type names and series labels
    const char *type_names[] = {"pm", "co2", "pressure", "altitude", "temperature", "humidity", "velocity", "heading"};
    const char *series_labels[][4] = {
        {"PM1", "PM2.5", "PM4", "PM10"},      // PM
        {"CO2", "", "", ""},                   // CO2
        {"hPa", "", "", ""},                   // Pressure
        {"Baro", "GPS", "", ""},               // Altitude
        {"BMP", "SCD", "", ""},                // Temperature
        {"%", "", "", ""},                     // Humidity
        {"m/s", "", "", ""},                   // Velocity
        {"deg", "", "", ""}                    // Heading
    };
    const char *units[] = {"µg/m³", "ppm", "hPa", "m", "°C", "%", "m/s", "°"};

    // Start JSON
    char buf[256];
    snprintf(buf, sizeof(buf),
        "{\"type\":\"%s\",\"unit\":\"%s\",\"series\":%d,\"labels\":[",
        type_names[data_type], units[data_type], series_count);
    httpd_resp_sendstr_chunk(req, buf);

    // Series labels
    for (int s = 0; s < series_count; s++) {
        snprintf(buf, sizeof(buf), "%s\"%s\"", s > 0 ? "," : "", series_labels[data_type][s]);
        httpd_resp_sendstr_chunk(req, buf);
    }
    httpd_resp_sendstr_chunk(req, "],\"data\":[");

    // Data points: [timestamp, val1, val2, ...]
    for (size_t i = 0; i < count; i++) {
        snprintf(buf, sizeof(buf), "%s[%lu", i > 0 ? "," : "", (unsigned long)timestamps[i]);
        httpd_resp_sendstr_chunk(req, buf);

        for (int s = 0; s < series_count; s++) {
            snprintf(buf, sizeof(buf), ",%.2f", values[i][s]);
            httpd_resp_sendstr_chunk(req, buf);
        }
        httpd_resp_sendstr_chunk(req, "]");
    }

    httpd_resp_sendstr_chunk(req, "]}");
    httpd_resp_sendstr_chunk(req, NULL);

    heap_caps_free(values);
    heap_caps_free(timestamps);
    return ESP_OK;
}

// GET /api/flight/latest - Get latest single record
static esp_err_t api_flight_latest_handler(httpd_req_t *req)
{
    flight_record_t rec;
    size_t count = flight_recorder_get(&rec, 1, flight_recorder_count() > 0 ? flight_recorder_count() - 1 : 0);

    if (count == 0) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"valid\":false}");
        return ESP_OK;
    }

    char json[512];
    snprintf(json, sizeof(json),
        "{\"valid\":true,\"ts\":%lu,"
        "\"gps\":{\"lat\":%.7f,\"lon\":%.7f,\"alt\":%.1f,\"vel\":%.2f,\"fix\":%d,\"sat\":%d},"
        "\"imu\":{\"qi\":%.4f,\"qj\":%.4f,\"qk\":%.4f,\"qw\":%.4f,\"acc\":%d},"
        "\"mag\":{\"x\":%d,\"y\":%d,\"z\":%d,\"hdg\":%.1f},"
        "\"baro\":{\"p\":%.2f,\"t\":%.2f,\"alt\":%.1f},"
        "\"env\":{\"co2\":%d,\"t\":%.2f,\"h\":%.2f},"
        "\"pm\":{\"1\":%.1f,\"25\":%.1f,\"4\":%.1f,\"10\":%.1f}}",
        (unsigned long)rec.timestamp,
        rec.lat / 1e7, rec.lon / 1e7, rec.alt_mm / 1000.0f, rec.vel_down / 1000.0f,
        rec.fix_type, rec.sats,
        rec.qi / 16384.0f, rec.qj / 16384.0f, rec.qk / 16384.0f, rec.qw / 16384.0f,
        rec.imu_accuracy,
        rec.mag_x, rec.mag_y, rec.mag_z, rec.heading / 10.0f,
        rec.pressure_pa / 100.0f, rec.bmp_temp / 100.0f, rec.baro_alt / 10.0f,
        rec.co2, rec.scd_temp / 100.0f, rec.humidity / 100.0f,
        rec.pm1 / 10.0f, rec.pm25 / 10.0f, rec.pm4 / 10.0f, rec.pm10 / 10.0f);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json);
    return ESP_OK;
}

// ============================================================================
// WiFi & Settings API
// ============================================================================

// GET /api/wifi/status
static esp_err_t api_wifi_status_handler(httpd_req_t *req)
{
    wifi_manager_mode_t mode = wifi_manager_get_mode();
    bool connected = wifi_manager_is_connected();
    char ip[16] = {0};
    wifi_manager_get_ip(ip);

    const char *mode_str = "none";
    if (mode == WIFI_MGR_MODE_STA) mode_str = "sta";
    else if (mode == WIFI_MGR_MODE_AP) mode_str = "ap";

    char json[256];
    snprintf(json, sizeof(json),
        "{\"mode\":\"%s\",\"connected\":%s,\"ip\":\"%s\",\"ap_ssid\":\"%s\",\"has_saved\":%s}",
        mode_str,
        connected ? "true" : "false",
        ip,
        wifi_manager_get_ap_ssid(),
        wifi_manager_has_saved_credentials() ? "true" : "false");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json);
    return ESP_OK;
}

// POST /api/wifi/connect - body: {"ssid":"xxx","password":"yyy"}
static esp_err_t api_wifi_connect_handler(httpd_req_t *req)
{
    char buf[256] = {0};
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No body");
        return ESP_FAIL;
    }

    // Simple JSON parsing (no external library)
    char ssid[33] = {0};
    char password[65] = {0};

    // Find "ssid":"value"
    char *ssid_start = strstr(buf, "\"ssid\"");
    if (ssid_start) {
        ssid_start = strchr(ssid_start + 6, '\"');
        if (ssid_start) {
            ssid_start++;
            char *ssid_end = strchr(ssid_start, '\"');
            if (ssid_end && (ssid_end - ssid_start) < (int)sizeof(ssid)) {
                strncpy(ssid, ssid_start, ssid_end - ssid_start);
            }
        }
    }

    // Find "password":"value"
    char *pass_start = strstr(buf, "\"password\"");
    if (pass_start) {
        pass_start = strchr(pass_start + 10, '\"');
        if (pass_start) {
            pass_start++;
            char *pass_end = strchr(pass_start, '\"');
            if (pass_end && (pass_end - pass_start) < (int)sizeof(password)) {
                strncpy(password, pass_start, pass_end - pass_start);
            }
        }
    }

    if (strlen(ssid) == 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing ssid");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "WiFi connect request: SSID=%s", ssid);

    // Save credentials
    esp_err_t save_ret = wifi_manager_save_credentials(ssid, password);
    if (save_ret != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save credentials");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true,\"message\":\"Credentials saved. Restarting...\"}");

    // Schedule restart
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();

    return ESP_OK;
}

// POST /api/wifi/clear - clear saved credentials
static esp_err_t api_wifi_clear_handler(httpd_req_t *req)
{
    esp_err_t ret = wifi_manager_clear_credentials();

    char json[64];
    snprintf(json, sizeof(json), "{\"ok\":%s}", ret == ESP_OK ? "true" : "false");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json);
    return ESP_OK;
}

// GET /api/time/status
static esp_err_t api_time_status_handler(httpd_req_t *req)
{
    bool synced = time_is_synced();
    time_source_t source = time_get_source();
    const char *tz = time_get_timezone();
    int offset = time_get_utc_offset_minutes();

    char time_str[32] = "--";
    if (synced) {
        time_format_iso(time_str, sizeof(time_str));
    }

    const char *source_str = "none";
    if (source == TIME_SOURCE_GPS) source_str = "gps";
    else if (source == TIME_SOURCE_NTP) source_str = "ntp";

    char json[256];
    snprintf(json, sizeof(json),
        "{\"synced\":%s,\"source\":\"%s\",\"time\":\"%s\",\"timezone\":\"%s\",\"utc_offset_min\":%d}",
        synced ? "true" : "false",
        source_str,
        time_str,
        tz,
        offset);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json);
    return ESP_OK;
}

// POST /api/time/timezone - body: {"tz":"CET-1CEST,M3.5.0/2,M10.5.0/3"}
static esp_err_t api_time_timezone_handler(httpd_req_t *req)
{
    char buf[128] = {0};
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No body");
        return ESP_FAIL;
    }

    // Find "tz":"value"
    char tz[64] = {0};
    char *tz_start = strstr(buf, "\"tz\"");
    if (tz_start) {
        tz_start = strchr(tz_start + 4, '\"');
        if (tz_start) {
            tz_start++;
            char *tz_end = strchr(tz_start, '\"');
            if (tz_end && (tz_end - tz_start) < (int)sizeof(tz)) {
                strncpy(tz, tz_start, tz_end - tz_start);
            }
        }
    }

    if (strlen(tz) == 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing tz");
        return ESP_FAIL;
    }

    esp_err_t set_ret = time_set_timezone(tz);

    char json[64];
    snprintf(json, sizeof(json), "{\"ok\":%s}", set_ret == ESP_OK ? "true" : "false");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json);
    return ESP_OK;
}

// POST /api/time/sync?unix=<timestamp>
static esp_err_t api_time_sync_handler(httpd_req_t *req)
{
    char query[64] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing query");
        return ESP_FAIL;
    }

    char val[16] = {0};
    if (httpd_query_key_value(query, "unix", val, sizeof(val)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing unix param");
        return ESP_FAIL;
    }

    time_t unix_ts = (time_t)strtol(val, NULL, 10);
    if (unix_ts < 1700000000) {  // Sanity check: after 2023
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid timestamp");
        return ESP_FAIL;
    }

    struct timeval tv = { .tv_sec = unix_ts, .tv_usec = 0 };
    settimeofday(&tv, NULL);

    ESP_LOGI(TAG, "Time synced from browser: %ld", (long)unix_ts);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

// WiFi setup page for AP mode
static const char WIFI_SETUP_HTML[] = R"rawliteral(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>AstroLink WiFi Setup</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:-apple-system,system-ui,sans-serif;background:#0d1b2a;color:#e0e0e0;min-height:100vh;display:flex;align-items:center;justify-content:center}
.card{background:#1b2838;border-radius:12px;padding:24px;width:90%;max-width:400px;box-shadow:0 4px 20px rgba(0,0,0,0.5)}
h1{color:#4a9eff;font-size:1.5rem;margin-bottom:8px;display:flex;align-items:center;gap:10px}
h1 svg{width:32px;height:32px}
.subtitle{color:#888;font-size:0.9rem;margin-bottom:24px}
.form-group{margin-bottom:16px}
label{display:block;color:#aaa;font-size:0.85rem;margin-bottom:6px}
input{width:100%;padding:12px;border:1px solid #333;border-radius:8px;background:#0d1b2a;color:#fff;font-size:1rem}
input:focus{outline:none;border-color:#4a9eff}
button{width:100%;padding:14px;border:none;border-radius:8px;background:#4a9eff;color:#fff;font-size:1rem;font-weight:600;cursor:pointer;transition:background 0.2s}
button:hover{background:#3a8eef}
button:disabled{background:#333;cursor:not-allowed}
.status{margin-top:16px;padding:12px;border-radius:8px;text-align:center;display:none}
.status.error{display:block;background:#5c2a2a;color:#ff6b6b}
.status.success{display:block;background:#2a5c2a;color:#6bff6b}
.info{margin-top:20px;padding-top:16px;border-top:1px solid #333;font-size:0.8rem;color:#666}
</style>
</head>
<body>
<div class="card">
<h1>
<svg viewBox="0 0 32 32"><defs><linearGradient id="g" x1="0%" y1="0%" x2="100%" y2="100%"><stop offset="0%" style="stop-color:#1e3a5f"/><stop offset="100%" style="stop-color:#0d1b2a"/></linearGradient></defs><circle cx="16" cy="16" r="15" fill="url(#g)" stroke="#4a9eff" stroke-width="1"/><circle cx="16" cy="12" r="3" fill="#ffd700"/><path d="M8 22 Q16 18 24 22" stroke="#4a9eff" stroke-width="2" fill="none"/><circle cx="10" cy="20" r="1.5" fill="#4a9eff"/><circle cx="22" cy="20" r="1.5" fill="#4a9eff"/></svg>
AstroLink
</h1>
<p class="subtitle">WiFi Configuration</p>
<form id="wifiForm">
<div class="form-group">
<label for="ssid">WiFi Network Name (SSID)</label>
<input type="text" id="ssid" name="ssid" required maxlength="32" autocomplete="off">
</div>
<div class="form-group">
<label for="password">Password</label>
<input type="password" id="password" name="password" maxlength="64" autocomplete="off">
</div>
<button type="submit" id="submitBtn">Connect</button>
</form>
<div class="status" id="status"></div>
<div class="info">
After saving, the device will restart and connect to your WiFi network. If connection fails, it will return to AP mode.
</div>
</div>
<script>
document.getElementById('wifiForm').addEventListener('submit',async function(e){
e.preventDefault();
const btn=document.getElementById('submitBtn');
const status=document.getElementById('status');
btn.disabled=true;btn.textContent='Connecting...';
status.className='status';status.style.display='none';
try{
const res=await fetch('/api/wifi/connect',{
method:'POST',
headers:{'Content-Type':'application/json'},
body:JSON.stringify({
ssid:document.getElementById('ssid').value,
password:document.getElementById('password').value
})
});
const data=await res.json();
if(data.ok){
status.className='status success';
status.textContent='Saved! Device restarting...';
status.style.display='block';
}else{
throw new Error(data.error||'Failed');
}
}catch(err){
status.className='status error';
status.textContent='Error: '+err.message;
status.style.display='block';
btn.disabled=false;btn.textContent='Connect';
}
});
</script>
</body>
</html>
)rawliteral";

static esp_err_t wifi_setup_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, WIFI_SETUP_HTML, strlen(WIFI_SETUP_HTML));
    return ESP_OK;
}

// ============================================================================
// Public API
// ============================================================================

esp_err_t http_server_fs_init(void)
{
    return littlefs_init();
}

esp_err_t http_server_start(void)
{
    if (s_server != NULL) {
        ESP_LOGW(TAG, "Server already running");
        return ESP_OK;
    }

    system_stats_init();

    // Initialize LittleFS
    esp_err_t ret = littlefs_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "LittleFS init failed, static files won't be served");
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.max_open_sockets = MAX_OPEN_SOCKETS;
    config.backlog_conn = 8;
    config.max_uri_handlers = 40;
    config.stack_size = 8192;  // Larger stack for OTA
    config.keep_alive_enable = true;
    config.keep_alive_idle = 15;
    config.keep_alive_interval = 5;
    config.keep_alive_count = 3;

    ret = httpd_start(&s_server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret));
        return ret;
    }

    // Index page
    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &index_uri);

    // OTA page
    httpd_uri_t ota_page_uri = {
        .uri = "/ota",
        .method = HTTP_GET,
        .handler = ota_page_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &ota_page_uri);

    // Files page
    httpd_uri_t files_page_uri = {
        .uri = "/files",
        .method = HTTP_GET,
        .handler = files_page_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &files_page_uri);

    // Static files (CSS, JS)
    httpd_uri_t css_uri = {
        .uri = "/style.css",
        .method = HTTP_GET,
        .handler = static_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &css_uri);

    httpd_uri_t js_uri = {
        .uri = "/app.js",
        .method = HTTP_GET,
        .handler = static_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &js_uri);

    // WebSocket endpoint
    httpd_uri_t ws_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = ws_handler,
        .user_ctx = NULL,
        .is_websocket = true,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &ws_uri);

    // API endpoints
    httpd_uri_t api_info_uri = {
        .uri = "/api/info",
        .method = HTTP_GET,
        .handler = api_info_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &api_info_uri);

    httpd_uri_t api_fan_clean_uri = {
        .uri = "/api/fan-clean",
        .method = HTTP_POST,
        .handler = api_fan_clean_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &api_fan_clean_uri);

    httpd_uri_t api_reset_uri = {
        .uri = "/api/reset",
        .method = HTTP_POST,
        .handler = api_reset_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &api_reset_uri);

    httpd_uri_t api_ota_uri = {
        .uri = "/api/ota",
        .method = HTTP_POST,
        .handler = api_ota_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &api_ota_uri);

    httpd_uri_t api_fs_ota_uri = {
        .uri = "/api/fs-ota",
        .method = HTTP_POST,
        .handler = api_fs_ota_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &api_fs_ota_uri);

    // LittleFS file management
    httpd_uri_t api_fs_upload_uri = {
        .uri = "/api/fs-upload",
        .method = HTTP_POST,
        .handler = api_fs_upload_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &api_fs_upload_uri);

    httpd_uri_t api_fs_list_uri = {
        .uri = "/api/fs-list",
        .method = HTTP_GET,
        .handler = api_fs_list_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &api_fs_list_uri);

    httpd_uri_t api_fs_download_uri = {
        .uri = "/api/fs-download",
        .method = HTTP_GET,
        .handler = api_fs_download_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &api_fs_download_uri);

    httpd_uri_t api_fs_delete_uri = {
        .uri = "/api/fs-delete",
        .method = HTTP_POST,
        .handler = api_fs_delete_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &api_fs_delete_uri);

    // GPS API
    httpd_uri_t api_gps_uri = {
        .uri = "/api/gps",
        .method = HTTP_GET,
        .handler = api_gps_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &api_gps_uri);

    // Compass API
    httpd_uri_t api_compass_uri = {
        .uri = "/api/compass",
        .method = HTTP_GET,
        .handler = api_compass_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &api_compass_uri);

    // Favicon (both .ico and .svg for Safari compatibility)
    httpd_uri_t favicon_uri = {
        .uri = "/favicon.ico",
        .method = HTTP_GET,
        .handler = favicon_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &favicon_uri);

    httpd_uri_t favicon_svg_uri = {
        .uri = "/favicon.svg",
        .method = HTTP_GET,
        .handler = favicon_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &favicon_svg_uri);

    httpd_uri_t apple_touch_uri = {
        .uri = "/apple-touch-icon.png",
        .method = HTTP_GET,
        .handler = static_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &apple_touch_uri);

    httpd_uri_t apple_touch_pre_uri = {
        .uri = "/apple-touch-icon-precomposed.png",
        .method = HTTP_GET,
        .handler = static_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &apple_touch_pre_uri);

    // History & Recording API
    httpd_uri_t api_history_uri = {
        .uri = "/api/history",
        .method = HTTP_GET,
        .handler = api_history_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &api_history_uri);

    httpd_uri_t api_export_uri = {
        .uri = "/api/export",
        .method = HTTP_GET,
        .handler = api_export_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &api_export_uri);

    httpd_uri_t api_rec_start_uri = {
        .uri = "/api/recording/start",
        .method = HTTP_POST,
        .handler = api_recording_start_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &api_rec_start_uri);

    httpd_uri_t api_rec_stop_uri = {
        .uri = "/api/recording/stop",
        .method = HTTP_POST,
        .handler = api_recording_stop_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &api_rec_stop_uri);

    httpd_uri_t api_rec_status_uri = {
        .uri = "/api/recording/status",
        .method = HTTP_GET,
        .handler = api_recording_status_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &api_rec_status_uri);

    // WiFi API
    httpd_uri_t api_wifi_status_uri = {
        .uri = "/api/wifi/status",
        .method = HTTP_GET,
        .handler = api_wifi_status_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &api_wifi_status_uri);

    httpd_uri_t api_wifi_connect_uri = {
        .uri = "/api/wifi/connect",
        .method = HTTP_POST,
        .handler = api_wifi_connect_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &api_wifi_connect_uri);

    httpd_uri_t api_wifi_clear_uri = {
        .uri = "/api/wifi/clear",
        .method = HTTP_POST,
        .handler = api_wifi_clear_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &api_wifi_clear_uri);

    // Time API
    httpd_uri_t api_time_status_uri = {
        .uri = "/api/time/status",
        .method = HTTP_GET,
        .handler = api_time_status_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &api_time_status_uri);

    httpd_uri_t api_time_timezone_uri = {
        .uri = "/api/time/timezone",
        .method = HTTP_POST,
        .handler = api_time_timezone_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &api_time_timezone_uri);

    httpd_uri_t api_time_sync_uri = {
        .uri = "/api/time/sync",
        .method = HTTP_POST,
        .handler = api_time_sync_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &api_time_sync_uri);

    // Flight Data API
    httpd_uri_t api_flight_chart_uri = {
        .uri = "/api/flight/chart",
        .method = HTTP_GET,
        .handler = api_flight_chart_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &api_flight_chart_uri);

    httpd_uri_t api_flight_latest_uri = {
        .uri = "/api/flight/latest",
        .method = HTTP_GET,
        .handler = api_flight_latest_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &api_flight_latest_uri);

    // WiFi Setup page (for AP mode)
    httpd_uri_t wifi_setup_uri = {
        .uri = "/setup",
        .method = HTTP_GET,
        .handler = wifi_setup_handler,
        .user_ctx = NULL,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
    httpd_register_uri_handler(s_server, &wifi_setup_uri);

    ESP_LOGI(TAG, "HTTP server started on port %d (LittleFS: %s, WiFi+Time API: enabled)",
             config.server_port, s_fs_mounted ? "OK" : "FAIL");
    return ESP_OK;
}

esp_err_t http_server_stop(void)
{
    if (s_server == NULL) return ESP_OK;

    esp_err_t ret = httpd_stop(s_server);
    s_server = NULL;

    ESP_LOGI(TAG, "HTTP server stopped");
    return ret;
}

int http_server_ws_broadcast(const uint8_t *data, size_t len)
{
    if (s_server == NULL || data == NULL || len == 0) return 0;

    int sent_count = 0;
    int client_fds[MAX_OPEN_SOCKETS];
    int ws_count = ws_get_clients(client_fds, MAX_OPEN_SOCKETS);

    httpd_ws_frame_t ws_pkt = {
        .final = true,
        .fragmented = false,
        .type = HTTPD_WS_TYPE_BINARY,
        .payload = (uint8_t *)data,
        .len = len
    };

    for (int i = 0; i < ws_count; i++) {
        int fd = client_fds[i];
        esp_err_t ret = httpd_ws_send_frame_async(s_server, fd, &ws_pkt);
        if (ret == ESP_OK) {
            sent_count++;
        } else {
            ESP_LOGW(TAG, "Failed to send to fd=%d: %s", fd, esp_err_to_name(ret));
        }
    }

    return sent_count;
}

int http_server_ws_broadcast_text(const char *data, size_t len)
{
    if (s_server == NULL || data == NULL || len == 0) return 0;

    int sent_count = 0;
    int client_fds[MAX_OPEN_SOCKETS];
    int ws_count = ws_get_clients(client_fds, MAX_OPEN_SOCKETS);

    httpd_ws_frame_t ws_pkt = {
        .final = true,
        .fragmented = false,
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)data,
        .len = len
    };

    for (int i = 0; i < ws_count; i++) {
        int fd = client_fds[i];
        esp_err_t ret = httpd_ws_send_frame_async(s_server, fd, &ws_pkt);
        if (ret == ESP_OK) {
            sent_count++;
        }
    }

    return sent_count;
}

int http_server_ws_client_count(void)
{
    int client_fds[MAX_OPEN_SOCKETS];
    return ws_get_clients(client_fds, MAX_OPEN_SOCKETS);
}
