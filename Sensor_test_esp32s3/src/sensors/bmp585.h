#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float temperature_c;
    float pressure_pa;
    float pressure_hpa;
    float altitude_m;
    uint32_t timestamp_ms;
    bool valid;
} bmp585_data_t;

typedef struct {
    i2c_port_t i2c_port;
    gpio_num_t sda_pin;
    gpio_num_t scl_pin;
    gpio_num_t int_pin; // GPIO_NUM_NC to disable INT
    uint32_t freq_hz;
    uint8_t i2c_addr; // 0 = auto-detect
    float sea_level_pa;
    bool int_active_low;
} bmp585_config_t;

esp_err_t bmp585_init(const bmp585_config_t *cfg);
esp_err_t bmp585_read(bmp585_data_t *out);
esp_err_t bmp585_set_power(bool enable);

#ifdef __cplusplus
}
#endif
