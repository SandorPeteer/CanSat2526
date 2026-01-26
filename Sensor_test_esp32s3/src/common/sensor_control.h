#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SENSOR_GPS = 0,
    SENSOR_SPS30,
    SENSOR_COMPASS,
    SENSOR_BNO,
    SENSOR_BMP585,
    SENSOR_SCD40,
} sensor_id_t;

typedef struct {
    bool gps;
    bool sps30;
    bool compass;
    bool bno;
    bool bmp585;
    bool scd40;
} sensor_control_state_t;

void sensor_control_init(const sensor_control_state_t *defaults);
void sensor_control_set(sensor_id_t sensor, bool enable);
bool sensor_control_get(sensor_id_t sensor);
bool sensor_control_set_by_name(const char *name, bool enable);
void sensor_control_get_state(sensor_control_state_t *out);

#ifdef __cplusplus
}
#endif
