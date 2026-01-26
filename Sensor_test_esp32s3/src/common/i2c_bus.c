#include "common/i2c_bus.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>

static const char *TAG = "I2C_BUS";

static bool s_inited = false;
static i2c_port_t s_port = I2C_NUM_0;
static gpio_num_t s_sda = GPIO_NUM_NC;
static gpio_num_t s_scl = GPIO_NUM_NC;
static uint32_t s_freq = 0;

static void i2c_bus_recover(gpio_num_t sda, gpio_num_t scl)
{
    if (sda == GPIO_NUM_NC || scl == GPIO_NUM_NC) return;

    gpio_config_t io_cfg = {};
    io_cfg.pin_bit_mask = (1ULL << sda) | (1ULL << scl);
    io_cfg.mode = GPIO_MODE_INPUT_OUTPUT_OD;
    io_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    io_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_cfg.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_cfg);

    gpio_set_level(sda, 1);
    gpio_set_level(scl, 1);
    esp_rom_delay_us(5);

    for (int i = 0; i < 9; ++i) {
        gpio_set_level(scl, 0);
        esp_rom_delay_us(5);
        gpio_set_level(scl, 1);
        esp_rom_delay_us(5);
    }

    // STOP condition
    gpio_set_level(sda, 0);
    esp_rom_delay_us(5);
    gpio_set_level(scl, 1);
    esp_rom_delay_us(5);
    gpio_set_level(sda, 1);
    esp_rom_delay_us(5);
}

esp_err_t i2c_bus_init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t freq_hz)
{
    if (s_inited) {
        if (port != s_port || sda != s_sda || scl != s_scl || freq_hz != s_freq) {
            ESP_LOGE(TAG, "I2C already inited (port=%d sda=%d scl=%d freq=%lu)",
                     s_port, s_sda, s_scl, (unsigned long)s_freq);
            return ESP_ERR_INVALID_STATE;
        }
        return ESP_OK;
    }

    i2c_bus_recover(sda, scl);
    vTaskDelay(pdMS_TO_TICKS(2));

    i2c_config_t i2c_cfg = {};
    i2c_cfg.mode = I2C_MODE_MASTER;
    i2c_cfg.sda_io_num = sda;
    i2c_cfg.scl_io_num = scl;
    i2c_cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_cfg.master.clk_speed = freq_hz;

    esp_err_t ret = i2c_param_config(port, &i2c_cfg);
    if (ret != ESP_OK) return ret;

    ret = i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) return ret;

    s_inited = true;
    s_port = port;
    s_sda = sda;
    s_scl = scl;
    s_freq = freq_hz;

    ESP_LOGI(TAG, "I2C ready (I2C%d, SDA=%d, SCL=%d, %lu Hz)",
             port, sda, scl, (unsigned long)freq_hz);
    return ESP_OK;
}
