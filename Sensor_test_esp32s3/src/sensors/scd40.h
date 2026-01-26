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
    float co2_ppm;
    float temperature_c;
    float humidity_rh;
    uint32_t timestamp_ms;
    bool valid;
} scd40_data_t;

typedef struct {
    i2c_port_t i2c_port;
    gpio_num_t sda_pin;
    gpio_num_t scl_pin;
    uint32_t freq_hz;
    uint8_t i2c_addr; // 0 = default 0x62
} scd40_config_t;

esp_err_t scd40_init(const scd40_config_t *cfg);
esp_err_t scd40_start_periodic_measurement(void);
esp_err_t scd40_stop_periodic_measurement(void);
bool scd40_data_ready(void);
esp_err_t scd40_read(scd40_data_t *out);

#ifdef __cplusplus
}
#endif
