#include "qmc5883l.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "common/i2c_bus.h"
#include <math.h>

static const char *TAG = "QMC";

#define QMC5883L_ADDR 0x0D

// Registers
#define REG_DATA_X_LSB 0x00
#define REG_STATUS     0x06
#define REG_CONTROL1   0x09
#define REG_CONTROL2   0x0A
#define REG_SET_RESET  0x0B

// Status bits
#define STATUS_DRDY 0x01
#define STATUS_OVL  0x02

static i2c_port_t s_i2c = I2C_NUM_0;
static bool s_inited = false;

static esp_err_t write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_write_to_device(s_i2c, QMC5883L_ADDR, buf, 2, pdMS_TO_TICKS(100));
}

static esp_err_t read_regs(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_write_read_device(s_i2c, QMC5883L_ADDR, &reg, 1, buf, len, pdMS_TO_TICKS(100));
}

esp_err_t qmc5883l_init(const qmc5883l_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    if (s_inited) return ESP_OK;

    s_i2c = cfg->i2c_port;

    esp_err_t ret = i2c_bus_init(s_i2c, cfg->sda_pin, cfg->scl_pin, cfg->freq_hz);
    if (ret != ESP_OK) return ret;

    // Soft reset
    ret = write_reg(REG_CONTROL2, 0x80);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "QMC5883L not found");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Set/Reset period
    ret = write_reg(REG_SET_RESET, 0x01);
    if (ret != ESP_OK) return ret;

    // Control1: continuous mode, 200Hz ODR, 8G range, 512 OSR
    // OSR=00 (512), RNG=01 (8G), ODR=11 (200Hz), MODE=01 (continuous)
    ret = write_reg(REG_CONTROL1, 0x1D);
    if (ret != ESP_OK) return ret;

    s_inited = true;
    ESP_LOGI(TAG, "QMC5883L ready (I2C%d, SDA=%d, SCL=%d)", s_i2c, cfg->sda_pin, cfg->scl_pin);
    return ESP_OK;
}

bool qmc5883l_data_ready(void)
{
    if (!s_inited) return false;
    uint8_t status = 0;
    if (read_regs(REG_STATUS, &status, 1) != ESP_OK) return false;
    return (status & STATUS_DRDY) != 0;
}

esp_err_t qmc5883l_read(qmc5883l_data_t *out)
{
    if (!s_inited || !out) return ESP_ERR_INVALID_STATE;

    uint8_t buf[6];
    esp_err_t ret = read_regs(REG_DATA_X_LSB, buf, 6);
    if (ret != ESP_OK) return ret;

    out->x = (int16_t)(buf[1] << 8 | buf[0]);
    out->y = (int16_t)(buf[3] << 8 | buf[2]);
    out->z = (int16_t)(buf[5] << 8 | buf[4]);

    // Check overflow
    uint8_t status = 0;
    read_regs(REG_STATUS, &status, 1);
    out->overflow = (status & STATUS_OVL) != 0;

    // Calculate heading (assuming sensor is level)
    float heading = atan2f((float)out->y, (float)out->x) * 180.0f / M_PI;
    if (heading < 0) heading += 360.0f;
    out->heading_deg = heading;

    out->timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    return ESP_OK;
}
