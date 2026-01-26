#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool system_stats_init(void);
bool system_stats_read_cpu_temp(float *out_celsius);
bool system_stats_read_wifi_rssi(int8_t *out_rssi);

#ifdef __cplusplus
}
#endif
