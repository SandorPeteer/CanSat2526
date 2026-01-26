#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// SPS30 output data format
typedef enum {
    SPS30_FORMAT_IEEE754_FLOAT = 0x03,  // Big-endian IEEE754 float (40 bytes)
    SPS30_FORMAT_UINT16 = 0x05          // Unsigned 16-bit integer (20 bytes)
} sps30_output_format_t;

// SPS30 measurement data (float format)
typedef struct {
    float pm1_0;           // Mass Concentration PM1.0 [µg/m³]
    float pm2_5;           // Mass Concentration PM2.5 [µg/m³]
    float pm4_0;           // Mass Concentration PM4.0 [µg/m³]
    float pm10;            // Mass Concentration PM10 [µg/m³]
    float nc0_5;           // Number Concentration PM0.5 [#/cm³]
    float nc1_0;           // Number Concentration PM1.0 [#/cm³]
    float nc2_5;           // Number Concentration PM2.5 [#/cm³]
    float nc4_0;           // Number Concentration PM4.0 [#/cm³]
    float nc10;            // Number Concentration PM10 [#/cm³]
    float typical_size;    // Typical Particle Size [µm]
    uint32_t timestamp_ms; // Measurement timestamp (ms since boot)
    bool valid;            // Data validity flag
} sps30_measurement_t;

// SPS30 device information
typedef struct {
    char serial_number[32];
    char product_type[8];
    uint8_t firmware_major;
    uint8_t firmware_minor;
} sps30_device_info_t;

// SHDLC command codes
typedef enum {
    SPS30_CMD_START_MEASUREMENT     = 0x00,
    SPS30_CMD_STOP_MEASUREMENT      = 0x01,
    SPS30_CMD_READ_DATA_READY       = 0x02,
    SPS30_CMD_READ_MEASURED_VALUES  = 0x03,
    SPS30_CMD_SLEEP                 = 0x10,
    SPS30_CMD_WAKEUP                = 0x11,
    SPS30_CMD_START_FAN_CLEANING    = 0x56,
    SPS30_CMD_READ_AUTO_CLEAN_INTERVAL = 0x80,
    SPS30_CMD_WRITE_AUTO_CLEAN_INTERVAL = 0x80,
    SPS30_CMD_READ_PRODUCT_TYPE     = 0xD0,
    SPS30_CMD_READ_SERIAL_NUMBER    = 0xD3,
    SPS30_CMD_READ_VERSION          = 0xD1,
    SPS30_CMD_READ_DEVICE_STATUS    = 0xD2,
    SPS30_CMD_CLEAR_DEVICE_STATUS   = 0xD2,
    SPS30_CMD_RESET                 = 0xD3
} sps30_cmd_t;

// SHDLC protocol constants
#define SHDLC_START_STOP_BYTE  0x7E
#define SHDLC_ESCAPE_BYTE      0x7D
#define SHDLC_ESCAPE_XOR       0x20

// SPS30 SHDLC address (always 0x00 for SPS30)
#define SPS30_SHDLC_ADDR       0x00

// Response timeout in ms
#define SPS30_RESPONSE_TIMEOUT_MS  100

// Maximum SHDLC frame size
#define SPS30_MAX_FRAME_SIZE   128

#ifdef __cplusplus
}
#endif
