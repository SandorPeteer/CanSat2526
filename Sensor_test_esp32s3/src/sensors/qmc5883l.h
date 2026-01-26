#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
    float heading_deg;    // 0-360, magnetic north
    bool overflow;
    uint32_t timestamp_ms;
} qmc5883l_data_t;

typedef struct {
    i2c_port_t i2c_port;
    gpio_num_t sda_pin;
    gpio_num_t scl_pin;
    uint32_t freq_hz;     // typically 400000
} qmc5883l_config_t;

esp_err_t qmc5883l_init(const qmc5883l_config_t *cfg);
esp_err_t qmc5883l_read(qmc5883l_data_t *out);
bool qmc5883l_data_ready(void);

#ifdef __cplusplus
}
#endif
