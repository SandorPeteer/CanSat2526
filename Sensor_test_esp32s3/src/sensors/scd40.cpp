#include "scd40.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "common/i2c_bus.h"

static const char *TAG = "SCD40";

#define SCD40_DEFAULT_ADDR 0x62

#define CMD_START_PERIODIC_MEASUREMENT 0x21B1
#define CMD_STOP_PERIODIC_MEASUREMENT  0x3F86
#define CMD_READ_MEASUREMENT           0xEC05
#define CMD_GET_DATA_READY             0xE4B8

static i2c_port_t s_i2c = I2C_NUM_0;
static uint8_t s_addr = SCD40_DEFAULT_ADDR;
static bool s_inited = false;

static uint8_t scd40_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x31);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static esp_err_t scd40_write_cmd(uint16_t cmd)
{
    uint8_t buf[2] = {(uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF)};
    return i2c_master_write_to_device(s_i2c, s_addr, buf, sizeof(buf), pdMS_TO_TICKS(100));
}

static esp_err_t scd40_read_words(uint16_t cmd, uint16_t *words, size_t count, uint32_t delay_ms)
{
    if (count == 0 || count > 3 || !words) return ESP_ERR_INVALID_ARG;

    esp_err_t ret = scd40_write_cmd(cmd);
    if (ret != ESP_OK) return ret;

    if (delay_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }

    uint8_t buf[9] = {0};
    size_t len = count * 3;
    ret = i2c_master_read_from_device(s_i2c, s_addr, buf, len, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) return ret;

    for (size_t i = 0; i < count; ++i) {
        uint8_t msb = buf[i * 3];
        uint8_t lsb = buf[i * 3 + 1];
        uint8_t crc = buf[i * 3 + 2];
        uint8_t calc = scd40_crc8(&buf[i * 3], 2);
        if (crc != calc) {
            ESP_LOGW(TAG, "CRC mismatch on word %u", (unsigned)i);
            return ESP_FAIL;
        }
        words[i] = (uint16_t)((msb << 8) | lsb);
    }

    return ESP_OK;
}

esp_err_t scd40_init(const scd40_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    if (s_inited) return ESP_OK;

    s_i2c = cfg->i2c_port;
    s_addr = (cfg->i2c_addr != 0) ? cfg->i2c_addr : SCD40_DEFAULT_ADDR;

    esp_err_t ret = i2c_bus_init(s_i2c, cfg->sda_pin, cfg->scl_pin, cfg->freq_hz);
    if (ret != ESP_OK) return ret;

    scd40_write_cmd(CMD_STOP_PERIODIC_MEASUREMENT);
    vTaskDelay(pdMS_TO_TICKS(500));
    s_inited = true;
    ESP_LOGI(TAG, "SCD40 ready (I2C%d, addr=0x%02X)", s_i2c, s_addr);
    return ESP_OK;
}

esp_err_t scd40_start_periodic_measurement(void)
{
    if (!s_inited) return ESP_ERR_INVALID_STATE;
    return scd40_write_cmd(CMD_START_PERIODIC_MEASUREMENT);
}

esp_err_t scd40_stop_periodic_measurement(void)
{
    if (!s_inited) return ESP_ERR_INVALID_STATE;
    return scd40_write_cmd(CMD_STOP_PERIODIC_MEASUREMENT);
}

bool scd40_data_ready(void)
{
    if (!s_inited) return false;
    uint16_t status = 0;
    if (scd40_read_words(CMD_GET_DATA_READY, &status, 1, 1) != ESP_OK) {
        return false;
    }
    return (status & 0x07FF) != 0;
}

esp_err_t scd40_read(scd40_data_t *out)
{
    if (!s_inited || !out) return ESP_ERR_INVALID_STATE;

    if (!scd40_data_ready()) {
        return ESP_ERR_TIMEOUT;
    }

    uint16_t raw[3] = {};
    esp_err_t ret = scd40_read_words(CMD_READ_MEASUREMENT, raw, 3, 5);
    if (ret != ESP_OK) return ret;

    float co2 = (float)raw[0];
    float temp = -45.0f + (175.0f * ((float)raw[1] / 65535.0f));
    float rh = 100.0f * ((float)raw[2] / 65535.0f);

    out->co2_ppm = co2;
    out->temperature_c = temp;
    out->humidity_rh = rh;
    out->timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    out->valid = true;
    return ESP_OK;
}
