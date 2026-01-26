#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_chip_info.h"
#include "esp_psram.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_private/esp_clk.h"
#include "soc/spi_mem_struct.h"
#include "soc/spi_mem_reg.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "sps30/sps30.h"
#include "gps/ubx_parser.h"
#include "sensors/qmc5883l.h"
#include "sensors/bno085.h"
#include "sensors/bmp585.h"
#include "sensors/scd40.h"
#define SHARED_DATA_IMPLEMENTATION
#include "common/shared_data.h"
#include "common/history_buffer.h"
#include "common/ntp_time.h"
#include "common/sensor_control.h"
#include "common/system_stats.h"
#include "common/flight_recorder.h"
#include "common/flight_data_collector.h"
#include "wifi/wifi_manager.h"
#include "http/http_server.h"

static const char *TAG = "MAIN";

static uint32_t mspi_core_clk_mhz(const spi_mem_dev_t *dev)
{
    switch (dev->core_clk_sel.core_clk_sel) {
    case 0:
        return 80;
    case 1:
        return 120;
    case 2:
        return 160;
    case 3:
        return 240;
    default:
        return 0;
    }
}

static uint32_t mspi_flash_clk_mhz(void)
{
    uint32_t core_mhz = mspi_core_clk_mhz(&SPIMEM0);
    if (core_mhz == 0) {
        return 0;
    }
    uint32_t reg = REG_READ(SPI_MEM_CLOCK_REG(0));
    if (reg & SPI_MEM_CLK_EQU_SYSCLK) {
        return core_mhz;
    }
    uint32_t n = (reg >> SPI_MEM_CLKCNT_N_S) & SPI_MEM_CLKCNT_N_V;
    return core_mhz / (n + 1);
}

static uint32_t mspi_psram_clk_mhz(void)
{
    uint32_t core_mhz = mspi_core_clk_mhz(&SPIMEM0);
    if (core_mhz == 0) {
        return 0;
    }
    uint32_t reg = REG_READ(SPI_MEM_SRAM_CLK_REG(0));
    if (reg & SPI_MEM_SCLK_EQU_SYSCLK) {
        return core_mhz;
    }
    uint32_t n = (reg >> SPI_MEM_SCLKCNT_N_S) & SPI_MEM_SCLKCNT_N_V;
    return core_mhz / (n + 1);
}

// ---- Configuration ----
// WiFi credentials (change these to your network)
#define WIFI_SSID     "SPnET"
#define WIFI_PASSWORD "SP1234sp"
static constexpr int8_t WIFI_TX_POWER_DBM = 20;

// SPS30 UART pins (UART2)
static constexpr gpio_num_t SPS30_TX_PIN = GPIO_NUM_14;
static constexpr gpio_num_t SPS30_RX_PIN = GPIO_NUM_13;
static constexpr uart_port_t SPS30_UART = UART_NUM_2;

// GPS UART pins (UART1) - GPS TX -> ESP RX, GPS RX -> ESP TX
static constexpr gpio_num_t GPS_TX_PIN = GPIO_NUM_17;
static constexpr gpio_num_t GPS_RX_PIN = GPIO_NUM_18;
static constexpr uart_port_t GPS_UART = UART_NUM_1;

// I2C pins (QMC5883L)
static constexpr gpio_num_t I2C_SDA_PIN = GPIO_NUM_8;
static constexpr gpio_num_t I2C_SCL_PIN = GPIO_NUM_9;
static constexpr gpio_num_t BMP585_INT_PIN = GPIO_NUM_12;

// BNO085 SPI pins
static constexpr spi_host_device_t BNO085_SPI_HOST = SPI2_HOST;
static constexpr gpio_num_t BNO085_SCK_PIN = GPIO_NUM_5;
static constexpr gpio_num_t BNO085_MOSI_PIN = GPIO_NUM_6;
static constexpr gpio_num_t BNO085_MISO_PIN = GPIO_NUM_7;
static constexpr gpio_num_t BNO085_CS_PIN = GPIO_NUM_15;
static constexpr gpio_num_t BNO085_WAKE_PIN = GPIO_NUM_16;
static constexpr gpio_num_t BNO085_INT_PIN = GPIO_NUM_10;
static constexpr gpio_num_t BNO085_RESET_PIN = GPIO_NUM_11;
static constexpr uint32_t BNO085_REPORT_INTERVAL_US = 20000; // 20ms (50Hz)

static volatile bool s_compass_ready = false;
static const sensor_control_state_t kDefaultSensors = {
    .gps = true,
    .sps30 = true,
    .compass = true,
    .bno = true,
    .bmp585 = true,
    .scd40 = true,
};

static esp_err_t init_gps_uart(void)
{
    ubx_config_t gps_cfg = {};
    gps_cfg.uart_num = GPS_UART;
    gps_cfg.tx_pin = GPS_TX_PIN;
    gps_cfg.rx_pin = GPS_RX_PIN;
    gps_cfg.baud_rate = 115200;
    return ubx_init(&gps_cfg);
}

static esp_err_t init_sps30_uart(void)
{
    esp_err_t ret = sps30_init(SPS30_UART, SPS30_TX_PIN, SPS30_RX_PIN);
    if (ret != ESP_OK) {
        return ret;
    }

    sps30_stop_measurement();
    vTaskDelay(pdMS_TO_TICKS(100));

    if (!sensor_control_get(SENSOR_SPS30)) {
        sps30_sleep();
        return ESP_OK;
    }

    return sps30_start_measurement(SPS30_FORMAT_IEEE754_FLOAT);
}

static esp_err_t init_bno085_spi(void)
{
    bno085_config_t cfg = {};
    cfg.spi_host = BNO085_SPI_HOST;
    cfg.sck_pin = BNO085_SCK_PIN;
    cfg.mosi_pin = BNO085_MOSI_PIN;
    cfg.miso_pin = BNO085_MISO_PIN;
    cfg.cs_pin = BNO085_CS_PIN;
    cfg.wake_pin = BNO085_WAKE_PIN;
    cfg.int_pin = BNO085_INT_PIN;
    cfg.reset_pin = BNO085_RESET_PIN;
    cfg.report_interval_us = BNO085_REPORT_INTERVAL_US;

    esp_err_t ret = bno085_init(&cfg);
    if (ret != ESP_OK) {
        return ret;
    }

    if (sensor_control_get(SENSOR_BNO)) {
        ret = bno085_enable_rotation_vector(cfg.report_interval_us);
        if (ret != ESP_OK) return ret;
        vTaskDelay(pdMS_TO_TICKS(10));
        ret = bno085_enable_accelerometer(cfg.report_interval_us);
        if (ret != ESP_OK) return ret;
        vTaskDelay(pdMS_TO_TICKS(10));
        ret = bno085_enable_gyroscope(cfg.report_interval_us);
        if (ret != ESP_OK) return ret;
    }

    return ESP_OK;
}

static esp_err_t init_compass_i2c(void)
{
    qmc5883l_config_t mag_cfg = {};
    mag_cfg.i2c_port = I2C_NUM_0;
    mag_cfg.sda_pin = I2C_SDA_PIN;
    mag_cfg.scl_pin = I2C_SCL_PIN;
    mag_cfg.freq_hz = 400000;

    return qmc5883l_init(&mag_cfg);
}

static esp_err_t init_bmp585_i2c(void)
{
    bmp585_config_t cfg = {};
    cfg.i2c_port = I2C_NUM_0;
    cfg.sda_pin = I2C_SDA_PIN;
    cfg.scl_pin = I2C_SCL_PIN;
    cfg.int_pin = BMP585_INT_PIN;
    cfg.int_active_low = true;
    cfg.freq_hz = 400000;
    cfg.i2c_addr = 0;
    cfg.sea_level_pa = 101325.0f;
    return bmp585_init(&cfg);
}

static esp_err_t init_scd40_i2c(void)
{
    scd40_config_t cfg = {};
    cfg.i2c_port = I2C_NUM_0;
    cfg.sda_pin = I2C_SDA_PIN;
    cfg.scl_pin = I2C_SCL_PIN;
    cfg.freq_hz = 400000;
    cfg.i2c_addr = 0;
    return scd40_init(&cfg);
}

// ============================================================================
// SPS30 Sensor Task
// ============================================================================

static void sps30_task(void *arg)
{
    (void)arg;

    // Measurement loop - directly read values (skip data_ready, like Arduino code)
    // The sensor returns len=0 when no new data is available
    sps30_measurement_t data;
    bool active = false;

    while (true) {
        if (!sensor_control_get(SENSOR_SPS30)) {
            if (active) {
                sps30_stop_measurement();
                sps30_sleep();
                active = false;
            }
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (!active) {
            sps30_wakeup();
            vTaskDelay(pdMS_TO_TICKS(50));
            sps30_start_measurement(SPS30_FORMAT_IEEE754_FLOAT);
            active = true;
        }

        esp_err_t read_ret = sps30_read_measurement(&data);

        if (read_ret == ESP_OK && data.valid) {
            // Update shared data for WebSocket
            shared_data_update(&data);

            // Add to history buffer (PSRAM)
            history_add(&data);

            // Write to LittleFS recording if active
            recording_write(&data);
        }

        // Poll at ~1s (sensor updates every ~1s)
        vTaskDelay(pdMS_TO_TICKS(900));
    }
}

// ============================================================================
// GPS + Compass Task
// ============================================================================

static void gps_task(void *arg)
{
    (void)arg;

    ubx_nav_pvt_t pvt;
    qmc5883l_data_t mag;

    while (true) {
        bool gps_on = sensor_control_get(SENSOR_GPS);
        bool compass_on = sensor_control_get(SENSOR_COMPASS);
        if (!gps_on && !compass_on) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        // Read GPS (blocks up to 200ms)
        if (gps_on && ubx_read(&pvt, 200)) {
            shared_data_update_gps(&pvt);

            // Sync system time from GPS if valid
            // valid bit 0 = validDate, bit 1 = validTime
            if ((pvt.valid & 0x03) == 0x03 && pvt.year >= 2020) {
                time_set_from_gps(pvt.year, pvt.month, pvt.day,
                                  pvt.hour, pvt.min, pvt.sec, true);
            }
        }

        // Read compass
        if (s_compass_ready && compass_on && qmc5883l_data_ready()) {
            if (qmc5883l_read(&mag) == ESP_OK) {
                shared_data_update_compass(&mag);
            }
        }
    }
}

// ============================================================================
// BNO085 IMU Task (SPI)
// ============================================================================

static void bno085_task(void *arg)
{
    (void)arg;
    bno085_data_t data = {};
    bool active = false;

    while (true) {
        if (!sensor_control_get(SENSOR_BNO)) {
            if (active) {
                active = false;
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (!active) {
            bno085_enable_rotation_vector(BNO085_REPORT_INTERVAL_US);
            bno085_enable_accelerometer(BNO085_REPORT_INTERVAL_US);
            bno085_enable_gyroscope(BNO085_REPORT_INTERVAL_US);
            active = true;
        }

        if (bno085_read(&data) == ESP_OK) {
            if (data.quaternion.valid) {
                shared_data_update_bno(&data);
            }
        }
        vTaskDelay(1);
    }
}

// ============================================================================
// BMP585 Task
// ============================================================================

static void bmp585_task(void *arg)
{
    (void)arg;
    bmp585_data_t data = {};
    bool active = false;

    while (true) {
        if (!sensor_control_get(SENSOR_BMP585)) {
            if (active) {
                bmp585_set_power(false);
                active = false;
            }
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (!active) {
            bmp585_set_power(true);
            active = true;
        }

        if (bmp585_read(&data) == ESP_OK && data.valid) {
            shared_data_update_bmp585(&data);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ============================================================================
// SCD40 Task
// ============================================================================

static void scd40_task(void *arg)
{
    (void)arg;
    scd40_data_t data = {};
    bool active = false;
    bool waited = false;
    uint64_t last_ok_us = 0;
    const uint64_t restart_timeout_us = 20000000;

    while (true) {
        if (!sensor_control_get(SENSOR_SCD40)) {
            if (active) {
                scd40_stop_periodic_measurement();
                active = false;
                waited = false;
                last_ok_us = 0;
            }
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (!active) {
            scd40_stop_periodic_measurement();
            vTaskDelay(pdMS_TO_TICKS(500));
            scd40_start_periodic_measurement();
            active = true;
            waited = false;
            last_ok_us = 0;
        }

        if (!waited) {
            vTaskDelay(pdMS_TO_TICKS(5000));
            waited = true;
        }

        if (scd40_read(&data) == ESP_OK && data.valid) {
            shared_data_update_scd40(&data);
            last_ok_us = esp_timer_get_time();
        }

        if (waited && last_ok_us != 0) {
            uint64_t now = esp_timer_get_time();
            if (now - last_ok_us > restart_timeout_us) {
                scd40_stop_periodic_measurement();
                vTaskDelay(pdMS_TO_TICKS(500));
                scd40_start_periodic_measurement();
                waited = false;
                last_ok_us = 0;
                continue;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ============================================================================
// WebSocket Broadcast Task
// ============================================================================

enum ws_frame_type_t : uint8_t {
    WS_FRAME_BNO = 1,
    WS_FRAME_COMPASS = 2,
    WS_FRAME_BMP585 = 3,
    WS_FRAME_SCD40 = 4,
    WS_FRAME_SPS30 = 5,
    WS_FRAME_GPS = 6,
    WS_FRAME_SYS = 7,
    WS_FRAME_TIME = 8,
};

static inline void pack_u8(uint8_t *buf, size_t *off, uint8_t v)
{
    buf[(*off)++] = v;
}

static inline void pack_u16(uint8_t *buf, size_t *off, uint16_t v)
{
    memcpy(buf + *off, &v, sizeof(v));
    *off += sizeof(v);
}

static inline void pack_u32(uint8_t *buf, size_t *off, uint32_t v)
{
    memcpy(buf + *off, &v, sizeof(v));
    *off += sizeof(v);
}

static inline void pack_i16(uint8_t *buf, size_t *off, int16_t v)
{
    memcpy(buf + *off, &v, sizeof(v));
    *off += sizeof(v);
}

static inline void pack_i32(uint8_t *buf, size_t *off, int32_t v)
{
    memcpy(buf + *off, &v, sizeof(v));
    *off += sizeof(v);
}

static inline void pack_f32(uint8_t *buf, size_t *off, float v)
{
    memcpy(buf + *off, &v, sizeof(v));
    *off += sizeof(v);
}

static void ws_broadcast_task(void *arg)
{
    (void)arg;
    uint64_t last_gps_us = 0;
    uint64_t last_compass_us = 0;
    uint64_t last_sps_us = 0;
    uint64_t last_bno_us = 0;
    uint64_t last_bmp_us = 0;
    uint64_t last_scd_us = 0;
    uint64_t last_sys_us = 0;
    uint64_t last_time_us = 0;
    const uint64_t imu_interval_us = 40000;
    const uint64_t compass_interval_us = 40000;
    const uint64_t bmp_interval_us = 100000;
    const uint64_t slow_interval_us = 500000;
    const uint64_t sys_interval_us = 500000;
    const uint64_t time_interval_us = 1000000;
    ubx_nav_pvt_t gps = {};
    qmc5883l_data_t mag = {};
    sps30_measurement_t sps = {};
    bno085_data_t bno = {};
    bmp585_data_t bmp = {};
    scd40_data_t scd = {};
    uint32_t last_sps_ts = 0;
    uint32_t last_bno_ts = 0;
    uint32_t last_bmp_ts = 0;
    uint32_t last_scd_ts = 0;

    while (true) {
        // Broadcast loop ~50Hz
        vTaskDelay(pdMS_TO_TICKS(20));

        int client_count = http_server_ws_client_count();
        if (client_count <= 0) {
            continue;
        }

        uint64_t now = esp_timer_get_time();

        if (sensor_control_get(SENSOR_SPS30)) {
            if (shared_data_get(&sps) && sps.timestamp_ms != last_sps_ts &&
                now - last_sps_us >= slow_interval_us) {
                uint8_t msg[1 + 10 * 4 + 4];
                size_t off = 0;
                pack_u8(msg, &off, WS_FRAME_SPS30);
                pack_f32(msg, &off, sps.pm1_0);
                pack_f32(msg, &off, sps.pm2_5);
                pack_f32(msg, &off, sps.pm4_0);
                pack_f32(msg, &off, sps.pm10);
                pack_f32(msg, &off, sps.nc0_5);
                pack_f32(msg, &off, sps.nc1_0);
                pack_f32(msg, &off, sps.nc2_5);
                pack_f32(msg, &off, sps.nc4_0);
                pack_f32(msg, &off, sps.nc10);
                pack_f32(msg, &off, sps.typical_size);
                pack_u32(msg, &off, sps.timestamp_ms);
                http_server_ws_broadcast(msg, off);
                last_sps_ts = sps.timestamp_ms;
                last_sps_us = now;
            }
        }

        if (sensor_control_get(SENSOR_BNO)) {
            if (shared_data_get_bno(&bno) && bno.quaternion.valid) {
                uint32_t ts = bno.quaternion.timestamp_ms;
                if (ts != last_bno_ts && now - last_bno_us >= imu_interval_us) {
                    uint8_t msg[1 + 4 * 4 + 4 + 1 + 4];
                    size_t off = 0;
                    pack_u8(msg, &off, WS_FRAME_BNO);
                    pack_f32(msg, &off, bno.quaternion.i);
                    pack_f32(msg, &off, bno.quaternion.j);
                    pack_f32(msg, &off, bno.quaternion.k);
                    pack_f32(msg, &off, bno.quaternion.real);
                    pack_f32(msg, &off, bno.quaternion.accuracy_rad);
                    pack_u8(msg, &off, bno.quaternion.accuracy_status);
                    pack_u32(msg, &off, ts);
                    http_server_ws_broadcast(msg, off);
                    last_bno_ts = ts;
                    last_bno_us = now;
                }
            }
        }

        if (sensor_control_get(SENSOR_BMP585)) {
            if (shared_data_get_bmp585(&bmp) && bmp.valid) {
                if (bmp.timestamp_ms != last_bmp_ts && now - last_bmp_us >= bmp_interval_us) {
                    uint8_t msg[1 + 3 * 4 + 4];
                    size_t off = 0;
                    pack_u8(msg, &off, WS_FRAME_BMP585);
                    pack_f32(msg, &off, bmp.temperature_c);
                    pack_f32(msg, &off, bmp.pressure_pa);
                    pack_f32(msg, &off, bmp.altitude_m);
                    pack_u32(msg, &off, bmp.timestamp_ms);
                    http_server_ws_broadcast(msg, off);
                    last_bmp_ts = bmp.timestamp_ms;
                    last_bmp_us = now;
                }
            }
        }

        if (sensor_control_get(SENSOR_SCD40)) {
            if (shared_data_get_scd40(&scd) && scd.valid) {
                if (scd.timestamp_ms != last_scd_ts && now - last_scd_us >= slow_interval_us) {
                    uint8_t msg[1 + 3 * 4 + 4];
                    size_t off = 0;
                    pack_u8(msg, &off, WS_FRAME_SCD40);
                    pack_f32(msg, &off, scd.co2_ppm);
                    pack_f32(msg, &off, scd.temperature_c);
                    pack_f32(msg, &off, scd.humidity_rh);
                    pack_u32(msg, &off, scd.timestamp_ms);
                    http_server_ws_broadcast(msg, off);
                    last_scd_ts = scd.timestamp_ms;
                    last_scd_us = now;
                }
            }
        }

        if (sensor_control_get(SENSOR_GPS) && now - last_gps_us >= slow_interval_us) {
            shared_data_get_gps(&gps);
            uint8_t msg[1 + 87];
            size_t off = 0;
            pack_u8(msg, &off, WS_FRAME_GPS);
            pack_u32(msg, &off, gps.itow);
            pack_u16(msg, &off, gps.year);
            pack_u8(msg, &off, gps.month);
            pack_u8(msg, &off, gps.day);
            pack_u8(msg, &off, gps.hour);
            pack_u8(msg, &off, gps.min);
            pack_u8(msg, &off, gps.sec);
            pack_u8(msg, &off, gps.valid);
            pack_u32(msg, &off, gps.tacc);
            pack_u8(msg, &off, gps.fix_type);
            pack_u8(msg, &off, gps.flags);
            pack_u8(msg, &off, gps.flags2);
            pack_u8(msg, &off, gps.num_sv);
            pack_i32(msg, &off, gps.lon);
            pack_i32(msg, &off, gps.lat);
            pack_i32(msg, &off, gps.height);
            pack_i32(msg, &off, gps.hmsl);
            pack_u32(msg, &off, gps.hacc);
            pack_u32(msg, &off, gps.vacc);
            pack_i32(msg, &off, gps.vel_n);
            pack_i32(msg, &off, gps.vel_e);
            pack_i32(msg, &off, gps.vel_d);
            pack_i32(msg, &off, gps.gspeed);
            pack_i32(msg, &off, gps.heading);
            pack_u32(msg, &off, gps.sacc);
            pack_u32(msg, &off, gps.headacc);
            pack_u16(msg, &off, gps.pdop);
            pack_u8(msg, &off, gps.flags3);
            pack_i32(msg, &off, gps.head_veh);
            pack_i16(msg, &off, gps.mag_dec);
            pack_u16(msg, &off, gps.mag_acc);
            pack_u32(msg, &off, gps.timestamp_ms);
            http_server_ws_broadcast(msg, off);
            last_gps_us = now;
        }

        if (sensor_control_get(SENSOR_COMPASS) && now - last_compass_us >= compass_interval_us) {
            if (shared_data_get_compass(&mag)) {
                uint8_t msg[1 + 3 * 2 + 4 + 4];
                size_t off = 0;
                pack_u8(msg, &off, WS_FRAME_COMPASS);
                pack_i16(msg, &off, mag.x);
                pack_i16(msg, &off, mag.y);
                pack_i16(msg, &off, mag.z);
                pack_f32(msg, &off, mag.heading_deg);
                pack_u32(msg, &off, mag.timestamp_ms);
                http_server_ws_broadcast(msg, off);
            }
            last_compass_us = now;
        }

        if (now - last_sys_us >= sys_interval_us) {
            int8_t wifi_rssi = 0;
            float cpu_temp = 0.0f;
            uint8_t flags = 0;

            if (system_stats_read_wifi_rssi(&wifi_rssi)) {
                flags |= 0x01;
            }
            if (system_stats_read_cpu_temp(&cpu_temp)) {
                flags |= 0x02;
            }

            uint8_t msg[1 + 1 + 2 + 4 + 4];
            size_t off = 0;
            pack_u8(msg, &off, WS_FRAME_SYS);
            pack_u8(msg, &off, flags);
            pack_i16(msg, &off, (int16_t)wifi_rssi);
            pack_f32(msg, &off, cpu_temp);
            pack_u32(msg, &off, (uint32_t)(now / 1000));
            http_server_ws_broadcast(msg, off);
            last_sys_us = now;
        }

        if (now - last_time_us >= time_interval_us) {
            const bool synced = time_is_synced();
            const time_source_t source = time_get_source();
            const int16_t offset_min = (int16_t)time_get_utc_offset_minutes();
            const uint32_t unix_ts = synced ? (uint32_t)time_get_unix() : 0;

            uint8_t flags = synced ? 0x01 : 0x00;
            uint8_t msg[1 + 1 + 1 + 2 + 4];
            size_t off = 0;
            pack_u8(msg, &off, WS_FRAME_TIME);
            pack_u8(msg, &off, flags);
            pack_u8(msg, &off, (uint8_t)source);
            pack_i16(msg, &off, offset_min);
            pack_u32(msg, &off, unix_ts);
            http_server_ws_broadcast(msg, off);
            last_time_us = now;
        }
    }
}

// ============================================================================
// Main Entry Point
// ============================================================================

extern "C" void app_main(void)
{
    // Initialize USB Serial JTAG for console output
    usb_serial_jtag_driver_config_t usb_serial_config = {
        .tx_buffer_size = 1024,
        .rx_buffer_size = 1024,
    };
    usb_serial_jtag_driver_install(&usb_serial_config);
    usb_serial_jtag_vfs_use_driver();

    // Wait for USB host to connect
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Set log levels (quiet by default, MAIN prints summary)
    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set("MAIN", ESP_LOG_INFO);
    esp_log_level_set("WIFI", ESP_LOG_ERROR);
    esp_log_level_set("wifi", ESP_LOG_ERROR);
    esp_log_level_set("wifi_init", ESP_LOG_ERROR);
    esp_log_level_set("esp_netif_handlers", ESP_LOG_ERROR);
    esp_log_level_set("NTP", ESP_LOG_ERROR);
    esp_log_level_set("HTTP", ESP_LOG_ERROR);
    esp_log_level_set("httpd_ws", ESP_LOG_ERROR);
    esp_log_level_set("httpd_txrx", ESP_LOG_ERROR);
    esp_log_level_set("SPS30", ESP_LOG_ERROR);
    esp_log_level_set("UBX", ESP_LOG_ERROR);
    esp_log_level_set("QMC", ESP_LOG_ERROR);
    esp_log_level_set("BNO085", ESP_LOG_INFO);
    esp_log_level_set("BMP585", ESP_LOG_INFO);
    esp_log_level_set("SCD40", ESP_LOG_INFO);
    esp_log_level_set("HISTORY", ESP_LOG_ERROR);
    esp_log_level_set("gpio", ESP_LOG_ERROR);
    esp_log_level_set("pp", ESP_LOG_ERROR);
    esp_log_level_set("net80211", ESP_LOG_ERROR);
    esp_log_level_set("phy_init", ESP_LOG_ERROR);

    // Print system info
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);

    ESP_LOGI(TAG, "=== ESP32-S3 Multi Sensor Hub ===");
    ESP_LOGI(TAG, "Chip: ESP32-S3 rev %d, %d cores", chip_info.revision, chip_info.cores);
    ESP_LOGI(TAG, "Flash: %lu MB", flash_size / (1024 * 1024));
    ESP_LOGI(TAG, "PSRAM: %d KB", esp_psram_get_size() / 1024);
    ESP_LOGI(TAG, "Free heap: %lu KB", esp_get_free_heap_size() / 1024);
    ESP_LOGI(TAG, "CPU: %u MHz, APB: %u MHz, XTAL: %u MHz",
             (unsigned)(esp_clk_cpu_freq() / 1000000),
             (unsigned)(esp_clk_apb_freq() / 1000000),
             (unsigned)(esp_clk_xtal_freq() / 1000000));
    ESP_LOGI(TAG, "MSPI core: %u MHz, Flash: %u MHz, PSRAM: %u MHz",
             (unsigned)mspi_core_clk_mhz(&SPIMEM0),
             (unsigned)mspi_flash_clk_mhz(),
             (unsigned)mspi_psram_clk_mhz());
    ESP_LOGI(TAG, "==========================================");

    esp_err_t isr_ret = gpio_install_isr_service(0);
    if (isr_ret != ESP_OK && isr_ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "GPIO ISR: failed");
    }

    // Initialize shared data
    shared_data_init();
    sensor_control_init(&kDefaultSensors);

    // Initialize history buffer in PSRAM (24 hours @ 1Hz = 86400 entries, ~1.6MB)
    history_init(86400);

    // Initialize flight recorder in PSRAM (24 hours @ 1Hz = 86400 entries, ~5.4MB)
    flight_recorder_init(86400);
    flight_data_collector_init();

    // Connect to WiFi (tries NVS credentials first, then fallback, then AP mode)
    ESP_LOGI(TAG, "WiFi: initializing...");

    wifi_manager_config_t wifi_cfg = {
        .ssid = WIFI_SSID,              // Fallback credentials
        .password = WIFI_PASSWORD,
        .max_retry = 5,
        .max_tx_power_dbm = WIFI_TX_POWER_DBM,
        .ap_ssid = "AstroLink-Setup",   // AP mode SSID
        .ap_password = NULL,            // Open AP
        .hostname = "astrolink"         // mDNS hostname
    };

    wifi_manager_init(&wifi_cfg);
    wifi_manager_mode_t wifi_mode = wifi_manager_get_mode();

    // Initialize time system AFTER WiFi (needs TCP/IP stack)
    time_init();

    if (wifi_mode == WIFI_MGR_MODE_STA) {
        char ip_str[16];
        wifi_manager_get_ip(ip_str);
        ESP_LOGI(TAG, "WiFi: STA mode, IP: %s", ip_str);
    } else if (wifi_mode == WIFI_MGR_MODE_AP) {
        ESP_LOGW(TAG, "WiFi: AP mode - %s (192.168.4.1)", wifi_manager_get_ap_ssid());
    } else {
        ESP_LOGE(TAG, "WiFi: failed");
    }

    ESP_LOGI(TAG, "Init: GPS UART1...");
    if (init_gps_uart() == ESP_OK) {
        ESP_LOGI(TAG, "GPS UART1: OK");
        xTaskCreate(gps_task, "gps", 4096, NULL, 8, NULL);
    } else {
        ESP_LOGE(TAG, "GPS UART1: failed");
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "Init: SPS30 UART2...");
    if (init_sps30_uart() == ESP_OK) {
        ESP_LOGI(TAG, "SPS30 UART2: OK");
        xTaskCreate(sps30_task, "sps30", 4096, NULL, 10, NULL);
    } else {
        ESP_LOGE(TAG, "SPS30 UART2: failed");
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "Init: BNO085 SPI...");
    if (init_bno085_spi() == ESP_OK) {
        ESP_LOGI(TAG, "BNO085 SPI: OK");
        xTaskCreatePinnedToCore(bno085_task, "bno085", 4096, NULL, 5, NULL, 1);
    } else {
        ESP_LOGE(TAG, "BNO085 SPI: failed");
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "Init: Compass I2C...");
    if (init_compass_i2c() == ESP_OK) {
        s_compass_ready = true;
        ESP_LOGI(TAG, "Compass I2C: OK");
    } else {
        ESP_LOGE(TAG, "Compass I2C: failed");
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "Init: BMP585 I2C...");
    if (init_bmp585_i2c() == ESP_OK) {
        ESP_LOGI(TAG, "BMP585 I2C: OK");
        xTaskCreate(bmp585_task, "bmp585", 4096, NULL, 6, NULL);
    } else {
        ESP_LOGE(TAG, "BMP585 I2C: failed");
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "Init: SCD40 I2C...");
    if (init_scd40_i2c() == ESP_OK) {
        ESP_LOGI(TAG, "SCD40 I2C: OK");
        xTaskCreate(scd40_task, "scd40", 4096, NULL, 6, NULL);
    } else {
        ESP_LOGE(TAG, "SCD40 I2C: failed");
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // Start HTTP server (works in both STA and AP mode)
    if (wifi_mode != WIFI_MGR_MODE_NONE) {
        system_stats_init();
        if (http_server_start() == ESP_OK) {
            ESP_LOGI(TAG, "HTTP: OK");
            xTaskCreate(ws_broadcast_task, "ws_broadcast", 4096, NULL, 5, NULL);
        } else {
            ESP_LOGE(TAG, "HTTP: failed");
        }
    }

    // Main loop - collect flight data at 1Hz
    while (true) {
        flight_data_collector_tick();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
