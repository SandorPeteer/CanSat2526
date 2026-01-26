#include "sensor_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include <string.h>

static sensor_control_state_t s_state = {};
static portMUX_TYPE s_state_mux = portMUX_INITIALIZER_UNLOCKED;

void sensor_control_init(const sensor_control_state_t *defaults)
{
    portENTER_CRITICAL(&s_state_mux);
    if (defaults) {
        s_state = *defaults;
    } else {
        memset(&s_state, 0, sizeof(s_state));
    }
    portEXIT_CRITICAL(&s_state_mux);
}

void sensor_control_set(sensor_id_t sensor, bool enable)
{
    portENTER_CRITICAL(&s_state_mux);
    switch (sensor) {
        case SENSOR_GPS:
            s_state.gps = enable;
            break;
        case SENSOR_SPS30:
            s_state.sps30 = enable;
            break;
        case SENSOR_COMPASS:
            s_state.compass = enable;
            break;
        case SENSOR_BNO:
            s_state.bno = enable;
            break;
        case SENSOR_BMP585:
            s_state.bmp585 = enable;
            break;
        case SENSOR_SCD40:
            s_state.scd40 = enable;
            break;
        default:
            break;
    }
    portEXIT_CRITICAL(&s_state_mux);
}

bool sensor_control_get(sensor_id_t sensor)
{
    bool enabled = false;
    portENTER_CRITICAL(&s_state_mux);
    switch (sensor) {
        case SENSOR_GPS:
            enabled = s_state.gps;
            break;
        case SENSOR_SPS30:
            enabled = s_state.sps30;
            break;
        case SENSOR_COMPASS:
            enabled = s_state.compass;
            break;
        case SENSOR_BNO:
            enabled = s_state.bno;
            break;
        case SENSOR_BMP585:
            enabled = s_state.bmp585;
            break;
        case SENSOR_SCD40:
            enabled = s_state.scd40;
            break;
        default:
            enabled = false;
            break;
    }
    portEXIT_CRITICAL(&s_state_mux);
    return enabled;
}

bool sensor_control_set_by_name(const char *name, bool enable)
{
    if (!name) {
        return false;
    }

    if (strcmp(name, "gps") == 0) {
        sensor_control_set(SENSOR_GPS, enable);
        return true;
    }
    if (strcmp(name, "sps30") == 0) {
        sensor_control_set(SENSOR_SPS30, enable);
        return true;
    }
    if (strcmp(name, "compass") == 0) {
        sensor_control_set(SENSOR_COMPASS, enable);
        return true;
    }
    if (strcmp(name, "bno") == 0) {
        sensor_control_set(SENSOR_BNO, enable);
        return true;
    }
    if (strcmp(name, "bmp585") == 0) {
        sensor_control_set(SENSOR_BMP585, enable);
        return true;
    }
    if (strcmp(name, "scd40") == 0) {
        sensor_control_set(SENSOR_SCD40, enable);
        return true;
    }

    return false;
}

void sensor_control_get_state(sensor_control_state_t *out)
{
    if (!out) {
        return;
    }
    portENTER_CRITICAL(&s_state_mux);
    *out = s_state;
    portEXIT_CRITICAL(&s_state_mux);
}
