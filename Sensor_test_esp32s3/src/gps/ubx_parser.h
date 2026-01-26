#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

// UBX-NAV-PVT parsed data
typedef struct {
    uint32_t itow;          // GPS time of week [ms]
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;          // Validity flags
    uint32_t tacc;          // Time accuracy [ns]
    int32_t nano;           // Nanoseconds
    uint8_t fix_type;       // 0=none, 1=dead, 2=2D, 3=3D, 4=GNSS+DR, 5=time
    uint8_t flags;          // Fix flags
    uint8_t flags2;
    uint8_t num_sv;         // Satellites used
    int32_t lon;            // Longitude [1e-7 deg]
    int32_t lat;            // Latitude [1e-7 deg]
    int32_t height;         // Height above ellipsoid [mm]
    int32_t hmsl;           // Height above MSL [mm]
    uint32_t hacc;          // Horizontal accuracy [mm]
    uint32_t vacc;          // Vertical accuracy [mm]
    int32_t vel_n;          // NED north velocity [mm/s]
    int32_t vel_e;          // NED east velocity [mm/s]
    int32_t vel_d;          // NED down velocity [mm/s]
    int32_t gspeed;         // Ground speed [mm/s]
    int32_t heading;        // Heading of motion [1e-5 deg]
    uint32_t sacc;          // Speed accuracy [mm/s]
    uint32_t headacc;       // Heading accuracy [1e-5 deg]
    uint16_t pdop;          // Position DOP [0.01]
    uint8_t flags3;
    int32_t head_veh;       // Vehicle heading [1e-5 deg]
    int16_t mag_dec;        // Magnetic declination [1e-2 deg]
    uint16_t mag_acc;       // Mag decl accuracy [1e-2 deg]
    // Computed
    double lat_deg;
    double lon_deg;
    float alt_m;
    float speed_kmh;
    float heading_deg;
    bool valid_fix;
    uint32_t timestamp_ms;
} ubx_nav_pvt_t;

typedef struct {
    uart_port_t uart_num;
    int tx_pin;
    int rx_pin;
    int baud_rate;
} ubx_config_t;

esp_err_t ubx_init(const ubx_config_t *cfg);
bool ubx_read(ubx_nav_pvt_t *out, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
