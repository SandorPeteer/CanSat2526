#include "bno085.h"

#include "BNO08x.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <new>

static const char *TAG = "BNO085";
static const uint32_t BNO085_SPI_FREQ_HZ = 2000000;

static BNO08x *s_imu = nullptr;
static bno085_config_t s_cfg = {};
static bool s_inited = false;
static bool s_data_pending = false;
static bno085_data_t s_latest = {};
static portMUX_TYPE s_data_mux = portMUX_INITIALIZER_UNLOCKED;

static uint8_t accuracy_to_status(BNO08xAccuracy accuracy)
{
    switch (accuracy) {
    case BNO08xAccuracy::LOW:
        return 1;
    case BNO08xAccuracy::MED:
        return 2;
    case BNO08xAccuracy::HIGH:
        return 3;
    default:
        return 0;
    }
}

static void update_rotation(bno085_data_t &dst, const bno08x_quat_t &quat,
                            const bno08x_euler_angle_t &euler, uint32_t ts_ms)
{
    dst.quaternion.i = quat.i;
    dst.quaternion.j = quat.j;
    dst.quaternion.k = quat.k;
    dst.quaternion.real = quat.real;
    dst.quaternion.accuracy_rad = quat.rad_accuracy;
    dst.quaternion.accuracy_status = accuracy_to_status(quat.accuracy);
    dst.quaternion.timestamp_ms = ts_ms;
    dst.quaternion.valid = true;

    dst.euler.roll = euler.x;
    dst.euler.pitch = euler.y;
    dst.euler.yaw = euler.z;
    dst.euler.accuracy_status = accuracy_to_status(euler.accuracy);
    dst.euler.timestamp_ms = ts_ms;
    dst.euler.valid = true;

    dst.calibrated = (dst.quaternion.accuracy_status >= 2);
}

static void update_accel(bno085_data_t &dst, const bno08x_accel_t &accel, uint32_t ts_ms)
{
    dst.accel.x = accel.x;
    dst.accel.y = accel.y;
    dst.accel.z = accel.z;
    dst.accel.accuracy_status = accuracy_to_status(accel.accuracy);
    dst.accel.timestamp_ms = ts_ms;
    dst.accel.valid = true;
}

static void update_gyro(bno085_data_t &dst, const bno08x_gyro_t &gyro, uint32_t ts_ms)
{
    dst.gyro.x = gyro.x;
    dst.gyro.y = gyro.y;
    dst.gyro.z = gyro.z;
    dst.gyro.accuracy_status = accuracy_to_status(gyro.accuracy);
    dst.gyro.timestamp_ms = ts_ms;
    dst.gyro.valid = true;
}

static bool poll_reports(bno085_data_t *snapshot)
{
    bno085_data_t current = {};
    portENTER_CRITICAL(&s_data_mux);
    current = s_latest;
    portEXIT_CRITICAL(&s_data_mux);

    if (!s_imu) {
        if (snapshot) {
            *snapshot = current;
        }
        return false;
    }

    bool updated = false;
    bool rv_updated = false;
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    if (!rv_updated && s_imu->rpt.rv_ARVR_stabilized.has_new_data()) {
        update_rotation(current,
                        s_imu->rpt.rv_ARVR_stabilized.get_quat(),
                        s_imu->rpt.rv_ARVR_stabilized.get_euler(true),
                        now_ms);
        rv_updated = true;
        updated = true;
    }

    if (!rv_updated && s_imu->rpt.rv_ARVR_stabilized_game.has_new_data()) {
        update_rotation(current,
                        s_imu->rpt.rv_ARVR_stabilized_game.get_quat(),
                        s_imu->rpt.rv_ARVR_stabilized_game.get_euler(true),
                        now_ms);
        rv_updated = true;
        updated = true;
    }

    if (!rv_updated && s_imu->rpt.rv.has_new_data()) {
        update_rotation(current,
                        s_imu->rpt.rv.get_quat(),
                        s_imu->rpt.rv.get_euler(true),
                        now_ms);
        rv_updated = true;
        updated = true;
    }

    if (!rv_updated && s_imu->rpt.rv_game.has_new_data()) {
        update_rotation(current,
                        s_imu->rpt.rv_game.get_quat(),
                        s_imu->rpt.rv_game.get_euler(true),
                        now_ms);
        rv_updated = true;
        updated = true;
    }

    if (!rv_updated && s_imu->rpt.rv_geomagnetic.has_new_data()) {
        update_rotation(current,
                        s_imu->rpt.rv_geomagnetic.get_quat(),
                        s_imu->rpt.rv_geomagnetic.get_euler(true),
                        now_ms);
        updated = true;
    }

    if (s_imu->rpt.accelerometer.has_new_data()) {
        update_accel(current, s_imu->rpt.accelerometer.get(), now_ms);
        updated = true;
    }

    if (s_imu->rpt.cal_gyro.has_new_data()) {
        update_gyro(current, s_imu->rpt.cal_gyro.get(), now_ms);
        updated = true;
    }

    if (updated) {
        portENTER_CRITICAL(&s_data_mux);
        s_latest = current;
        s_data_pending = true;
        portEXIT_CRITICAL(&s_data_mux);
    }

    if (snapshot) {
        *snapshot = current;
    }

    return updated;
}

esp_err_t bno085_init(const bno085_config_t *cfg)
{
    if (!cfg) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_inited) {
        return ESP_OK;
    }

    s_cfg = *cfg;

    if (s_cfg.wake_pin != GPIO_NUM_NC) {
        gpio_config_t wake_cfg = {};
        wake_cfg.mode = GPIO_MODE_OUTPUT;
        wake_cfg.pin_bit_mask = (1ULL << s_cfg.wake_pin);
        gpio_config(&wake_cfg);
        // PS0/WAKE must be high during reset to select SPI.
        gpio_set_level(s_cfg.wake_pin, 1);
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    bno08x_config_t imu_cfg(
        s_cfg.spi_host,
        s_cfg.mosi_pin,
        s_cfg.miso_pin,
        s_cfg.sck_pin,
        s_cfg.cs_pin,
        s_cfg.int_pin,
        s_cfg.reset_pin,
        BNO085_SPI_FREQ_HZ,
        false);

    s_imu = new (std::nothrow) BNO08x(imu_cfg);
    if (!s_imu) {
        return ESP_ERR_NO_MEM;
    }

    if (!s_imu->initialize()) {
        delete s_imu;
        s_imu = nullptr;
        return ESP_FAIL;
    }

    // Keep PS0/WAKE high in SPI mode.

    portENTER_CRITICAL(&s_data_mux);
    s_latest = {};
    s_data_pending = false;
    portEXIT_CRITICAL(&s_data_mux);

    s_inited = true;
    ESP_LOGI(TAG, "BNO08x initialized via esp32_BNO08x component");
    return ESP_OK;
}

esp_err_t bno085_deinit(void)
{
    if (!s_inited) {
        return ESP_OK;
    }

    delete s_imu;
    s_imu = nullptr;
    s_inited = false;

    portENTER_CRITICAL(&s_data_mux);
    s_latest = {};
    s_data_pending = false;
    portEXIT_CRITICAL(&s_data_mux);

    return ESP_OK;
}

esp_err_t bno085_reset(void)
{
    if (!s_inited || !s_imu) {
        return ESP_ERR_INVALID_STATE;
    }

    return s_imu->hard_reset() ? ESP_OK : ESP_FAIL;
}

esp_err_t bno085_enable_rotation_vector(uint32_t interval_us)
{
    if (!s_inited || !s_imu) {
        return ESP_ERR_INVALID_STATE;
    }

    return s_imu->rpt.rv.enable(interval_us) ? ESP_OK : ESP_FAIL;
}

esp_err_t bno085_enable_game_rotation_vector(uint32_t interval_us)
{
    if (!s_inited || !s_imu) {
        return ESP_ERR_INVALID_STATE;
    }

    return s_imu->rpt.rv_game.enable(interval_us) ? ESP_OK : ESP_FAIL;
}

esp_err_t bno085_enable_rotation_vector_candidates(uint32_t interval_us)
{
    if (!s_inited || !s_imu) {
        return ESP_ERR_INVALID_STATE;
    }

    bool ok = true;
    ok &= s_imu->rpt.rv_ARVR_stabilized.enable(interval_us);
    ok &= s_imu->rpt.rv_ARVR_stabilized_game.enable(interval_us);
    ok &= s_imu->rpt.rv.enable(interval_us);
    ok &= s_imu->rpt.rv_game.enable(interval_us);
    ok &= s_imu->rpt.rv_geomagnetic.enable(interval_us);
    return ok ? ESP_OK : ESP_FAIL;
}

esp_err_t bno085_enable_accelerometer(uint32_t interval_us)
{
    if (!s_inited || !s_imu) {
        return ESP_ERR_INVALID_STATE;
    }

    return s_imu->rpt.accelerometer.enable(interval_us) ? ESP_OK : ESP_FAIL;
}

esp_err_t bno085_enable_gyroscope(uint32_t interval_us)
{
    if (!s_inited || !s_imu) {
        return ESP_ERR_INVALID_STATE;
    }

    return s_imu->rpt.cal_gyro.enable(interval_us) ? ESP_OK : ESP_FAIL;
}

bool bno085_data_available(void)
{
    if (!s_inited || !s_imu) {
        return false;
    }

    (void)poll_reports(nullptr);

    bool pending = false;
    portENTER_CRITICAL(&s_data_mux);
    pending = s_data_pending;
    portEXIT_CRITICAL(&s_data_mux);
    return pending;
}

esp_err_t bno085_read(bno085_data_t *data)
{
    if (!s_inited || !s_imu) {
        return ESP_ERR_INVALID_STATE;
    }

    bno085_data_t snapshot = {};
    (void)poll_reports(&snapshot);

    bool pending = false;
    portENTER_CRITICAL(&s_data_mux);
    pending = s_data_pending;
    s_data_pending = false;
    portEXIT_CRITICAL(&s_data_mux);

    if (data) {
        *data = snapshot;
    }

    return pending ? ESP_OK : ESP_ERR_NOT_FOUND;
}

bool bno085_get_quaternion(bno085_quaternion_t *quat)
{
    if (!s_inited || !quat) {
        return false;
    }

    portENTER_CRITICAL(&s_data_mux);
    *quat = s_latest.quaternion;
    portEXIT_CRITICAL(&s_data_mux);
    return quat->valid;
}

bool bno085_get_euler(bno085_euler_t *euler)
{
    if (!s_inited || !euler) {
        return false;
    }

    portENTER_CRITICAL(&s_data_mux);
    *euler = s_latest.euler;
    portEXIT_CRITICAL(&s_data_mux);
    return euler->valid;
}

void bno085_quaternion_to_euler(const bno085_quaternion_t *quat, bno085_euler_t *euler)
{
    if (!quat || !euler) {
        return;
    }

    float dqw = quat->real;
    float dqx = quat->i;
    float dqy = quat->j;
    float dqz = quat->k;

    float norm = sqrtf(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz);
    if (norm == 0.0f) {
        return;
    }
    dqw /= norm;
    dqx /= norm;
    dqy /= norm;
    dqz /= norm;

    float ysqr = dqy * dqy;
    float t0 = +2.0f * (dqw * dqx + dqy * dqz);
    float t1 = +1.0f - 2.0f * (dqx * dqx + ysqr);
    float roll = atan2f(t0, t1);

    float t2 = +2.0f * (dqw * dqy - dqz * dqx);
    if (t2 > 1.0f) t2 = 1.0f;
    if (t2 < -1.0f) t2 = -1.0f;
    float pitch = asinf(t2);

    float t3 = +2.0f * (dqw * dqz + dqx * dqy);
    float t4 = +1.0f - 2.0f * (ysqr + dqz * dqz);
    float yaw = atan2f(t3, t4);

    euler->roll = roll * 57.2957795f;
    euler->pitch = pitch * 57.2957795f;
    euler->yaw = yaw * 57.2957795f;
    if (euler->yaw < 0.0f) {
        euler->yaw += 360.0f;
    }
}

esp_err_t bno085_tare(void)
{
    if (!s_inited || !s_imu) {
        return ESP_ERR_INVALID_STATE;
    }

    return s_imu->rpt.rv.tare() ? ESP_OK : ESP_FAIL;
}

esp_err_t bno085_save_calibration(void)
{
    if (!s_inited || !s_imu) {
        return ESP_ERR_INVALID_STATE;
    }

    return s_imu->dynamic_calibration_save() ? ESP_OK : ESP_FAIL;
}

bool bno085_is_calibrated(void)
{
    if (!s_inited) {
        return false;
    }

    portENTER_CRITICAL(&s_data_mux);
    bool calibrated = s_latest.calibrated;
    portEXIT_CRITICAL(&s_data_mux);
    return calibrated;
}

esp_err_t bno085_get_product_id(uint8_t *sw_major, uint8_t *sw_minor, uint32_t *sw_part)
{
    if (!s_inited || !s_imu) {
        return ESP_ERR_INVALID_STATE;
    }

    sh2_ProductIds_t ids = s_imu->get_product_IDs();
    if (ids.numEntries == 0) {
        return ESP_ERR_NOT_FOUND;
    }

    const sh2_ProductId_t &id = ids.entry[0];
    if (sw_major) *sw_major = id.swVersionMajor;
    if (sw_minor) *sw_minor = id.swVersionMinor;
    if (sw_part) *sw_part = id.swPartNumber;

    return ESP_OK;
}
