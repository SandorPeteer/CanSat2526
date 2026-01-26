#include "flight_data_collector.h"
#include "flight_recorder.h"
#include "shared_data.h"
#include "ntp_time.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char *TAG = "FLIGHT_COLL";
static const uint64_t WARMUP_US = 15000000ULL;

esp_err_t flight_data_collector_init(void)
{
    ESP_LOGI(TAG, "Flight data collector initialized");
    return ESP_OK;
}

void flight_data_collector_sample(flight_record_t *record)
{
    if (!record) return;

    memset(record, 0, sizeof(flight_record_t));

    // Timestamp
    if (time_is_synced()) {
        record->timestamp = (uint32_t)time_get_unix();
    } else {
        record->timestamp = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    }

    // GPS
    ubx_nav_pvt_t gps;
    if (shared_data_get_gps(&gps)) {
        record->lat = gps.lat;
        record->lon = gps.lon;
        record->alt_mm = gps.hmsl;
        record->vel_down = (int16_t)(gps.vel_d / 10);  // mm/s to cm/s, capped
        record->fix_type = gps.fix_type;
        record->sats = gps.num_sv;
        record->pdop = gps.pdop;
        record->gps_valid = gps.valid_fix ? 1 : 0;
    }

    // IMU (BNO085)
    bno085_data_t bno;
    if (shared_data_get_bno(&bno) && bno.quaternion.valid) {
        // Scale quaternion to int16 (-1..1 -> -16384..16384)
        record->qi = (int16_t)(bno.quaternion.i * 16384.0f);
        record->qj = (int16_t)(bno.quaternion.j * 16384.0f);
        record->qk = (int16_t)(bno.quaternion.k * 16384.0f);
        record->qw = (int16_t)(bno.quaternion.real * 16384.0f);
        record->imu_accuracy = bno.quaternion.accuracy_status;
        record->imu_valid = 1;
    }

    // Magnetometer (QMC5883L)
    qmc5883l_data_t mag;
    if (shared_data_get_compass(&mag) && !mag.overflow) {
        record->mag_x = mag.x;
        record->mag_y = mag.y;
        record->mag_z = mag.z;
        record->heading = (int16_t)(mag.heading_deg * 10.0f);  // 0-360 -> 0-3600
    }

    // Barometer (BMP585)
    bmp585_data_t bmp;
    if (shared_data_get_bmp585(&bmp) && bmp.valid) {
        record->pressure_pa = (int32_t)(bmp.pressure_pa);
        record->bmp_temp = (int16_t)(bmp.temperature_c * 100.0f);
        float alt_m = 44330.0f * (1.0f - powf(bmp.pressure_hpa / 1013.25f, 0.1903f));
        record->baro_alt = (int16_t)(alt_m * 10.0f);  // meters to decimeters
    }

    // CO2/Humidity (SCD40)
    scd40_data_t scd;
    if (shared_data_get_scd40(&scd) && scd.valid) {
        record->co2 = (uint16_t)scd.co2_ppm;
        record->scd_temp = (int16_t)(scd.temperature_c * 100.0f);
        record->humidity = (uint16_t)(scd.humidity_rh * 100.0f);
    }

    // Particulate (SPS30)
    sps30_measurement_t sps;
    if (shared_data_get(&sps) && sps.valid) {
        record->pm1 = (uint16_t)(sps.pm1_0 * 10.0f);
        record->pm25 = (uint16_t)(sps.pm2_5 * 10.0f);
        record->pm4 = (uint16_t)(sps.pm4_0 * 10.0f);
        record->pm10 = (uint16_t)(sps.pm10 * 10.0f);
    }
}

void flight_data_collector_tick(void)
{
    static uint64_t start_us = 0;
    if (start_us == 0) {
        start_us = esp_timer_get_time();
    }

    if (esp_timer_get_time() - start_us < WARMUP_US) {
        return;
    }

    flight_record_t record;
    flight_data_collector_sample(&record);

    // Add to PSRAM ring buffer
    flight_recorder_add(&record);

    // Write to file if recording
    if (flight_recording_active()) {
        flight_recording_write(&record);
    }
}
