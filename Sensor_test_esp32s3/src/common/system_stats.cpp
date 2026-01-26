#include "system_stats.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "driver/temperature_sensor.h"
#include <string.h>

static const char *TAG = "SYS";
static temperature_sensor_handle_t s_temp_sensor = NULL;

bool system_stats_init(void)
{
    if (s_temp_sensor) {
        return true;
    }

    const struct {
        int min_c;
        int max_c;
    } ranges[] = {
        { 20, 60 },
        { 10, 50 },
        { 0, 80 }
    };

    for (size_t i = 0; i < sizeof(ranges) / sizeof(ranges[0]); i++) {
        temperature_sensor_config_t config = {};
        config.range_min = ranges[i].min_c;
        config.range_max = ranges[i].max_c;
        config.clk_src = TEMPERATURE_SENSOR_CLK_SRC_DEFAULT;
        config.flags.allow_pd = 0;

        esp_err_t err = temperature_sensor_install(&config, &s_temp_sensor);
        if (err != ESP_OK) {
            s_temp_sensor = NULL;
            continue;
        }

        if (temperature_sensor_enable(s_temp_sensor) == ESP_OK) {
            ESP_LOGI(TAG, "Temp sensor range set: %d..%d C", config.range_min, config.range_max);
            return true;
        }

        temperature_sensor_uninstall(s_temp_sensor);
        s_temp_sensor = NULL;
    }

    ESP_LOGW(TAG, "Temp sensor init failed: no valid range");
    return false;
}

bool system_stats_read_cpu_temp(float *out_celsius)
{
    if (!s_temp_sensor || !out_celsius) {
        return false;
    }
    return temperature_sensor_get_celsius(s_temp_sensor, out_celsius) == ESP_OK;
}

bool system_stats_read_wifi_rssi(int8_t *out_rssi)
{
    wifi_ap_record_t ap_info;
    memset(&ap_info, 0, sizeof(ap_info));
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        if (out_rssi) {
            *out_rssi = ap_info.rssi;
        }
        return true;
    }
    return false;
}
