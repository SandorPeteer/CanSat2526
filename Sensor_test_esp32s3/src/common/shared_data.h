#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "../sps30/sps30_types.h"
#include "../gps/ubx_parser.h"
#include "../sensors/qmc5883l.h"
#include "../sensors/bno085.h"
#include "../sensors/bmp585.h"
#include "../sensors/scd40.h"

#ifdef __cplusplus
extern "C" {
#endif

void shared_data_init(void);
void shared_data_update(const sps30_measurement_t *data);
bool shared_data_get(sps30_measurement_t *data);
bool shared_data_has_new(void);
const uint8_t* shared_data_get_binary(size_t *size);
void shared_data_lock(void);
void shared_data_unlock(void);

// GPS data
void shared_data_update_gps(const ubx_nav_pvt_t *gps);
bool shared_data_get_gps(ubx_nav_pvt_t *gps);

// Compass data
void shared_data_update_compass(const qmc5883l_data_t *compass);
bool shared_data_get_compass(qmc5883l_data_t *compass);

// BNO085 IMU data
void shared_data_update_bno(const bno085_data_t *bno);
bool shared_data_get_bno(bno085_data_t *bno);

// BMP585 data
void shared_data_update_bmp585(const bmp585_data_t *bmp);
bool shared_data_get_bmp585(bmp585_data_t *bmp);

// SCD40 data
void shared_data_update_scd40(const scd40_data_t *scd);
bool shared_data_get_scd40(scd40_data_t *scd);

#ifdef __cplusplus
}
#endif

// ============================================================================
// Implementation (header-only for simplicity)
// ============================================================================

#ifdef SHARED_DATA_IMPLEMENTATION

#include <string.h>

static SemaphoreHandle_t s_data_mutex = NULL;
static sps30_measurement_t s_current_data = {};
static ubx_nav_pvt_t s_gps_data = {};
static qmc5883l_data_t s_compass_data = {};
static bno085_data_t s_bno_data = {};
static bmp585_data_t s_bmp585_data = {};
static scd40_data_t s_scd40_data = {};
static bool s_has_new_data = false;

// Binary format for WebSocket: SPS30(44 bytes) + BNO085(28 bytes) = 72 bytes
static uint8_t s_binary_data[72];

void shared_data_init(void)
{
    if (s_data_mutex == NULL) {
        s_data_mutex = xSemaphoreCreateMutex();
    }
    memset(&s_current_data, 0, sizeof(s_current_data));
    memset(&s_bno_data, 0, sizeof(s_bno_data));
    memset(&s_bmp585_data, 0, sizeof(s_bmp585_data));
    memset(&s_scd40_data, 0, sizeof(s_scd40_data));
    memset(s_binary_data, 0, sizeof(s_binary_data));
}

void shared_data_update(const sps30_measurement_t *data)
{
    if (!data || !s_data_mutex) return;

    xSemaphoreTake(s_data_mutex, portMAX_DELAY);

    memcpy(&s_current_data, data, sizeof(s_current_data));
    s_has_new_data = true;

    // Prepare binary data for WebSocket (little-endian for web clients)
    float *floats = (float *)s_binary_data;
    floats[0] = data->pm1_0;
    floats[1] = data->pm2_5;
    floats[2] = data->pm4_0;
    floats[3] = data->pm10;
    floats[4] = data->nc0_5;
    floats[5] = data->nc1_0;
    floats[6] = data->nc2_5;
    floats[7] = data->nc4_0;
    floats[8] = data->nc10;
    floats[9] = data->typical_size;
    uint32_t *ts = (uint32_t *)&s_binary_data[40];
    *ts = data->timestamp_ms;

    xSemaphoreGive(s_data_mutex);
}

bool shared_data_get(sps30_measurement_t *data)
{
    if (!data || !s_data_mutex) return false;

    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    memcpy(data, &s_current_data, sizeof(*data));
    s_has_new_data = false;
    xSemaphoreGive(s_data_mutex);

    return data->valid;
}

bool shared_data_has_new(void)
{
    if (!s_data_mutex) return false;

    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    bool result = s_has_new_data;
    xSemaphoreGive(s_data_mutex);

    return result;
}

const uint8_t* shared_data_get_binary(size_t *size)
{
    if (size) {
        *size = sizeof(s_binary_data);
    }
    return s_binary_data;
}

void shared_data_lock(void)
{
    if (s_data_mutex) {
        xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    }
}

void shared_data_unlock(void)
{
    if (s_data_mutex) {
        xSemaphoreGive(s_data_mutex);
    }
}

void shared_data_update_gps(const ubx_nav_pvt_t *gps)
{
    if (!gps || !s_data_mutex) return;
    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    memcpy(&s_gps_data, gps, sizeof(s_gps_data));
    xSemaphoreGive(s_data_mutex);
}

bool shared_data_get_gps(ubx_nav_pvt_t *gps)
{
    if (!gps || !s_data_mutex) return false;
    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    memcpy(gps, &s_gps_data, sizeof(*gps));
    xSemaphoreGive(s_data_mutex);
    return s_gps_data.valid_fix;
}

void shared_data_update_compass(const qmc5883l_data_t *compass)
{
    if (!compass || !s_data_mutex) return;
    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    memcpy(&s_compass_data, compass, sizeof(s_compass_data));
    xSemaphoreGive(s_data_mutex);
}

bool shared_data_get_compass(qmc5883l_data_t *compass)
{
    if (!compass || !s_data_mutex) return false;
    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    memcpy(compass, &s_compass_data, sizeof(*compass));
    xSemaphoreGive(s_data_mutex);
    return !s_compass_data.overflow;
}

void shared_data_update_bno(const bno085_data_t *bno)
{
    if (!bno || !s_data_mutex) return;
    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    memcpy(&s_bno_data, bno, sizeof(s_bno_data));

    // Append quaternion + accuracy + timestamp after SPS30 block
    float *floats = (float *)&s_binary_data[44];
    floats[0] = bno->quaternion.i;
    floats[1] = bno->quaternion.j;
    floats[2] = bno->quaternion.k;
    floats[3] = bno->quaternion.real;
    floats[4] = bno->quaternion.accuracy_rad;

    uint32_t *ts = (uint32_t *)&s_binary_data[64];
    *ts = bno->quaternion.timestamp_ms;

    s_binary_data[68] = bno->quaternion.accuracy_status;
    s_binary_data[69] = 0;
    s_binary_data[70] = 0;
    s_binary_data[71] = 0;

    xSemaphoreGive(s_data_mutex);
}

bool shared_data_get_bno(bno085_data_t *bno)
{
    if (!bno || !s_data_mutex) return false;
    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    memcpy(bno, &s_bno_data, sizeof(*bno));
    xSemaphoreGive(s_data_mutex);
    return bno->quaternion.valid;
}

void shared_data_update_bmp585(const bmp585_data_t *bmp)
{
    if (!bmp || !s_data_mutex) return;
    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    memcpy(&s_bmp585_data, bmp, sizeof(s_bmp585_data));
    xSemaphoreGive(s_data_mutex);
}

bool shared_data_get_bmp585(bmp585_data_t *bmp)
{
    if (!bmp || !s_data_mutex) return false;
    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    memcpy(bmp, &s_bmp585_data, sizeof(*bmp));
    xSemaphoreGive(s_data_mutex);
    return bmp->valid;
}

void shared_data_update_scd40(const scd40_data_t *scd)
{
    if (!scd || !s_data_mutex) return;
    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    memcpy(&s_scd40_data, scd, sizeof(s_scd40_data));
    xSemaphoreGive(s_data_mutex);
}

bool shared_data_get_scd40(scd40_data_t *scd)
{
    if (!scd || !s_data_mutex) return false;
    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    memcpy(scd, &s_scd40_data, sizeof(*scd));
    xSemaphoreGive(s_data_mutex);
    return scd->valid;
}

#endif // SHARED_DATA_IMPLEMENTATION
