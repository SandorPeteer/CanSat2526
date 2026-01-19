// ESP32-S3-WROOM-1 board – A BOARDON LÁTHATÓ EGYSZERŰSÍTETT PIN SZÁMOZÁSSAL

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>
#include <math.h>
#include <string.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include <Adafruit_NeoPixel.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_BMP5xx.h>
#include <Adafruit_Sensor.h>
#include <SensirionErrors.h>
#include <SensirionI2CScd4x.h>
#include <SensirionUartSps30.h>

// I2C busz pinek (board szamozas)
static constexpr int PIN_I2C_SDA = 8;
static constexpr int PIN_I2C_SCL = 9;

// UART#1 (GPS)
static constexpr int PIN_UART1_RX = 18;
static constexpr int PIN_UART1_TX = 17;
static constexpr uint32_t UART1_BAUD = 115200;

// UART#2 (SPS30)
static constexpr int PIN_UART2_RX = 13;
static constexpr int PIN_UART2_TX = 14;
static constexpr uint32_t UART2_BAUD = 115200;

// SPI bus
static constexpr int PIN_SPI_SCK = 5;
static constexpr int PIN_SPI_MOSI = 6;
static constexpr int PIN_SPI_MISO = 7;

// BNO085 (SPI + INT + RST + CS)
static constexpr int PIN_BNO_INT = 10;
static constexpr int PIN_BNO_RST = 11;
static constexpr int PIN_BNO_CS = 15;
static constexpr int PIN_BNO_WAKE = 16;  // PS0/WAKE
static constexpr uint8_t BNO_I2C_ADDR_PRIMARY = 0x4B;
static constexpr uint8_t BNO_I2C_ADDR_ALT = 0x4A;
static constexpr uint32_t BNO_REPORT_US_ROT = 10000;   // 10ms = 100Hz
static constexpr uint32_t BNO_REPORT_US_FAST = 10000;  // 10ms = 100Hz
static constexpr uint32_t BNO_STALL_MS = 2000;
static constexpr uint32_t BNO_RECOVER_BACKOFF_MS = 2500;
static constexpr uint32_t BNO_STARTUP_GRACE_MS = 2500;
static constexpr uint32_t BNO_RESET_LOW_MS = 20;
static constexpr uint32_t BNO_RESET_BOOT_MS = 800;
static constexpr bool BNO_SPI_PROBE = false;
static constexpr uint8_t BNO_CAL_MASK =
    SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG | SH2_CAL_PLANAR;
static constexpr uint32_t I2C_FREQ_HZ = 100000;
static constexpr uint32_t I2C_TIMEOUT_MS = 50;
static constexpr uint8_t I2C_CLEAR_PULSES = 16;

// Watchdog és supervisor
static constexpr uint32_t WDT_TIMEOUT_S = 30;          // Watchdog timeout (hard reset)
static constexpr uint32_t SENSOR_DEAD_TIMEOUT_MS = 60000;  // Ha 60 sec-ig nincs szenzor adat -> restart
#if defined(SH2_STABILIZED_ROTATION_VECTOR)
static constexpr sh2_SensorId_t BNO_ROT_SENSOR =
    SH2_STABILIZED_ROTATION_VECTOR;
#elif defined(SH2_ARVR_STABILIZED_RV)
static constexpr sh2_SensorId_t BNO_ROT_SENSOR = SH2_ARVR_STABILIZED_RV;
#else
static constexpr sh2_SensorId_t BNO_ROT_SENSOR = SH2_ROTATION_VECTOR;
#endif

static constexpr char WIFI_SSID[] = "SPnET";
static constexpr char WIFI_PASS[] = "SP1234sp";

static constexpr uint32_t SPS_BOOT_WAIT_MS = 300;
static constexpr uint32_t SPS_GUARD_MS = 20;
static constexpr uint32_t SPS_STOP_DELAY_MS = 50;
static constexpr uint32_t SPS_START_SETTLE_MS = 1200;
static constexpr uint32_t SPS_POLL_MS = 1100;
static constexpr uint32_t SPS_CLEAN_DURATION_MS = 10000;
static constexpr uint8_t SPS_MAX_TIMEOUTS = 3;
static constexpr uint8_t SPS_MAX_EXEC_ERRORS = 3;
static constexpr uint32_t SPS_RX_FLUSH_MS = 20;
static constexpr uint32_t SPS_AUTO_CLEAN_INTERVAL_S = 604800;
static constexpr uint32_t SPS_TASK_DELAY_MS = 20;
static constexpr uint32_t SPS_STATUS_POLL_INTERVAL_MS = 1000;

static constexpr uint32_t SPS_STATUS_SPEED_BIT = 1UL << 21;
static constexpr uint32_t SPS_STATUS_LASER_BIT = 1UL << 5;
static constexpr uint32_t SPS_STATUS_FAN_BIT = 1UL << 4;
static constexpr uint16_t SPS_NO_DATA_ERROR = 0x0402;
static constexpr uint16_t SPS_TIMEOUT_ERROR = 0x0206;

static constexpr int PIN_BMP585_INT = 12;
static constexpr float BMP_SEA_LEVEL_HPA = 1013.25f;
static constexpr uint32_t BMP_POLL_INTERVAL_MS = 250;
static constexpr uint16_t BMP_QNH_AVG_WINDOW = 30;
static constexpr uint32_t BMP_QNH_SAMPLE_MS = 1000;

static constexpr uint32_t MAG_READ_INTERVAL_MS = 200;
static constexpr uint8_t MAG_ADDR_HMC5883L = 0x1E;
static constexpr uint8_t MAG_ADDR_QMC5883L = 0x0D;

static constexpr uint32_t BNO_READ_INTERVAL_MS = 20;
static constexpr uint32_t SCD_READ_INTERVAL_MS = 5000;

static constexpr uint8_t LED_BRIGHTNESS = 8;
static constexpr uint32_t LED_PULSE_PERIOD_MS = 2000;
static constexpr uint32_t LED_ACTIVITY_WINDOW_MS = 4000;
static constexpr uint32_t WS_FAST_MS = 100;  // 10Hz WebSocket frame rate
static constexpr uint32_t WS_GPS_MS = 1000;  // 1Hz GPS frame rate
static constexpr bool BNO_DEBUG_LOG = false;

#if defined(RGB_BUILTIN)
static constexpr int PIN_STATUS_LED = RGB_BUILTIN;
#elif defined(NEOPIXEL_PIN)
static constexpr int PIN_STATUS_LED = NEOPIXEL_PIN;
#else
static constexpr int PIN_STATUS_LED = 48;
#endif

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
SensirionUartSps30 sps30;
Adafruit_BNO08x bno08x(-1);  // Reset handled manually with longer boot delay.
Adafruit_BMP5xx bmp585;
SensirionI2CScd4x scd4x;
Adafruit_NeoPixel status_led(1, PIN_STATUS_LED, NEO_GRB + NEO_KHZ800);

static String buildTelemetryJson(bool include_raw);
static String buildGpsRawJson();
static void initSps30();

struct Sps30Data {
  bool valid = false;
  bool last_read_ok = false;
  float mc1p0 = 0.0f;
  float mc2p5 = 0.0f;
  float mc4p0 = 0.0f;
  float mc10p0 = 0.0f;
  float nc0p5 = 0.0f;
  float nc1p0 = 0.0f;
  float nc2p5 = 0.0f;
  float nc4p0 = 0.0f;
  float nc10p0 = 0.0f;
  float typical_particle_size = 0.0f;
  int16_t last_error = 0;
  char last_error_text[64] = "";
  int16_t status_error = 0;
  char status_error_text[64] = "";
  uint32_t status_register = 0;
  char status_flags[32] = "OK";
  uint32_t last_status_ms = 0;
  int16_t last_clean_error = 0;
  char last_clean_error_text[64] = "";
  uint32_t last_clean_ms = 0;
  uint32_t auto_clean_interval_s = 0;
  uint32_t last_read_ms = 0;
};

enum class MagType : uint8_t {
  NONE = 0,
  HMC5883L,
  QMC5883L,
};

struct MagData {
  bool ok = false;
  MagType type = MagType::NONE;
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float heading_deg = 0.0f;
  uint32_t last_read_ms = 0;
  char type_label[12] = "NONE";
};

struct Bmp585Data {
  bool ok = false;
  bool alt_valid = false;
  float temp_c = 0.0f;
  float pressure_hpa = 0.0f;
  float altitude_m = 0.0f;
  char error_text[64] = "";
  uint32_t last_read_ms = 0;
};

struct Bno085Data {
  bool ok = false;
  float yaw_deg = 0.0f;
  float pitch_deg = 0.0f;
  float roll_deg = 0.0f;
  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;
  float lax = 0.0f;
  float lay = 0.0f;
  float laz = 0.0f;
  float gx = 0.0f;
  float gy = 0.0f;
  float gz = 0.0f;
  float mx = 0.0f;
  float my = 0.0f;
  float mz = 0.0f;
  float grav_x = 0.0f;
  float grav_y = 0.0f;
  float grav_z = 0.0f;
  uint8_t rot_accuracy = 0;
  uint8_t accel_accuracy = 0;
  uint8_t lin_accuracy = 0;
  uint8_t gyro_accuracy = 0;
  uint8_t mag_accuracy = 0;
  uint8_t grav_accuracy = 0;
  char error_text[64] = "";
  uint32_t last_read_ms = 0;
};

struct Scd40Data {
  bool ok = false;
  float co2_ppm = 0.0f;
  float temp_c = 0.0f;
  float rh = 0.0f;
  char error_text[64] = "";
  uint32_t last_read_ms = 0;
};

struct GpsData {
  bool fix = false;
  bool latlng_valid = false;
  bool alt_valid = false;
  bool sats_valid = false;
  bool hdop_valid = false;
  bool time_valid = false;
  bool date_valid = false;
  bool last_known_valid = false;
  double lat = 0.0;
  double lng = 0.0;
  double last_lat = 0.0;
  double last_lng = 0.0;
  double last_alt_m = 0.0;
  double alt_m = 0.0;
  uint32_t sats = 0;
  double hdop = 0.0;
  uint8_t fix_type = 0;
  uint8_t flags = 0;
  uint8_t hour = 0;
  uint8_t minute = 0;
  uint8_t second = 0;
  uint8_t day = 0;
  uint8_t month = 0;
  uint16_t year = 0;
  uint32_t location_age_ms = 0;
  uint32_t time_age_ms = 0;
  uint32_t date_age_ms = 0;
  uint32_t chars = 0;
  uint32_t sentences_with_fix = 0;
  uint32_t failed_checksum = 0;
  uint32_t ubx_pvt_count = 0;
  uint32_t ubx_dop_count = 0;
  uint32_t ubx_last_ms = 0;
  uint32_t last_fix_ms = 0;
  // NAV-PVT extra fields
  int32_t vel_n_mm_s = 0;      // velocity north (mm/s)
  int32_t vel_e_mm_s = 0;      // velocity east (mm/s)
  int32_t vel_d_mm_s = 0;      // velocity down (mm/s)
  int32_t g_speed_mm_s = 0;    // ground speed (mm/s)
  int32_t head_mot = 0;        // heading of motion (1e-5 deg)
  uint32_t h_acc_mm = 0;       // horizontal accuracy (mm)
  uint32_t v_acc_mm = 0;       // vertical accuracy (mm)
  uint32_t s_acc_mm_s = 0;     // speed accuracy (mm/s)
  uint32_t head_acc = 0;       // heading accuracy (1e-5 deg)
  uint16_t p_dop = 0;          // position DOP (0.01)
  bool vel_valid = false;
};

static Sps30Data sps_data;
static portMUX_TYPE sps_mux = portMUX_INITIALIZER_UNLOCKED;
static Bmp585Data bmp_data;
static MagData mag_data;
static Bno085Data bno_data;
static Scd40Data scd_data;
static GpsData gps_data;

// GPS smoothing (EMA)
static constexpr float GPS_EMA_ALPHA = 0.3f;  // 0.3 = gyorsabb követés, 0.1 = simább
static double gps_smooth_lat = 0.0;
static double gps_smooth_lng = 0.0;
static double gps_smooth_alt = 0.0;
static bool gps_smooth_init = false;
enum class SpsState : uint8_t {
  BOOT_WAIT = 0,
  SEND_STOP,
  SEND_START,
  RUN_POLL,
  CLEAN_STOP,
  CLEAN_START,
  CLEAN_WAIT,
  RECOVER_STOP,
  RECOVER_START,
};
static SpsState sps_state = SpsState::BOOT_WAIT;
static uint32_t sps_next_action_ms = 0;
static uint32_t sps_next_read_ms = 0;
static uint32_t sps_next_status_ms = 0;
static uint32_t sps_clean_until_ms = 0;
static uint8_t sps_timeout_count = 0;
static uint8_t sps_exec_error_count = 0;
static uint32_t sps_recovery_count = 0;
static volatile bool sps_clean_requested = false;
static bool sps_config_pending = false;
static uint32_t last_http_ms = 0;
static uint32_t last_ws_fast_ms = 0;
static uint32_t last_ws_gps_ms = 0;
static uint32_t last_any_sensor_ok_ms = 0;
static uint32_t supervisor_restart_count = 0;
static bool sps_enabled = true;
static bool sps_sleeping = false;
static bool bmp_enabled = true;
static bool bmp_sleeping = false;
static bool mag_enabled = true;
static bool mag_sleeping = false;
static bool bno_enabled = true;
static bool bno_sleeping = false;
static bool bno_reset_requested = false;
static bool scd_enabled = true;
static bool scd_sleeping = false;
static bool gps_enabled = true;
static bool system_reset_requested = false;
static uint32_t system_reset_ms = 0;
static wifi_power_t wifi_tx_target = WIFI_POWER_19_5dBm;
static bool wifi_tx_applied = false;
static int wifi_tx_set_err = 0;
static int wifi_tx_apply_err = 0;
static int wifi_ps_err = 0;
static TaskHandle_t sps_task_handle = nullptr;
static volatile bool bmp_data_ready = false;
static uint32_t last_bmp_poll_ms = 0;
static bool bmp_present = false;
static bool bno_present = false;
static bool scd_present = false;
static bool bno_reports_enabled = false;
static uint32_t last_bno_read_ms = 0;
static uint32_t last_scd_read_ms = 0;
static uint32_t bno_next_probe_ms = 0;
static uint32_t bno_init_ms = 0;
static uint8_t bno_i2c_addr = 0;
static sh2_SensorValue_t bno_value;
static sh2_SensorId_t bno_rot_sensor_id = BNO_ROT_SENSOR;
static uint32_t bno_event_count = 0;
static uint32_t bno_rot_count = 0;
static uint32_t bno_last_event_ms = 0;
static uint32_t bno_last_rot_ms = 0;
static uint32_t bno_last_log_ms = 0;
static uint8_t bno_last_sensor_id = 0;
static uint32_t bno_recovery_count = 0;
enum class BmpAltMode : uint8_t {
  QNH = 0,
  REL = 1,
};
static BmpAltMode bmp_alt_mode = BmpAltMode::QNH;
static float bmp_ref_hpa = BMP_SEA_LEVEL_HPA;
static float bmp_rel_ref_hpa = 0.0f;
static bool bmp_rel_ref_valid = false;
static float bmp_qnh_samples[BMP_QNH_AVG_WINDOW] = {0.0f};
static size_t bmp_qnh_index = 0;
static size_t bmp_qnh_count = 0;
static float bmp_qnh_sum = 0.0f;
static uint32_t bmp_qnh_last_ms = 0;

enum class UbxState : uint8_t {
  SYNC1 = 0,
  SYNC2,
  CLASS,
  ID,
  LEN1,
  LEN2,
  PAYLOAD,
  CK_A,
  CK_B,
};

struct UbxParser {
  UbxState state = UbxState::SYNC1;
  uint8_t cls = 0;
  uint8_t id = 0;
  uint16_t length = 0;
  uint16_t index = 0;
  uint8_t ck_a = 0;
  uint8_t ck_b = 0;
  uint8_t ck_a_calc = 0;
  uint8_t ck_b_calc = 0;
  uint8_t payload[96] = {0};
};

static UbxParser ubx;
static constexpr size_t GPS_RAW_BUFFER_SIZE = 256;
static uint8_t gps_raw[GPS_RAW_BUFFER_SIZE] = {0};
static size_t gps_raw_head = 0;
static size_t gps_raw_len = 0;

static const char kIndexHtml[] PROGMEM = R"HTML(
<!doctype html>
<html lang="hu">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>CanSat2526 Telemetry</title>
  <style>
    @import url('https://fonts.googleapis.com/css2?family=Space+Grotesk:wght@400;600;700&family=JetBrains+Mono:wght@400;600&display=swap');
    :root {
      --bg1: #0a141b;
      --bg2: #1a2e3a;
      --bg3: #3a2a1c;
      --panel: #111c24;
      --text: #e8f0f4;
      --muted: #93a6b2;
      --accent: #2cc4b7;
      --accent2: #f0a04b;
      --danger: #f95d5d;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: "Space Grotesk", sans-serif;
      color: var(--text);
      background:
        radial-gradient(circle at 10% 20%, rgba(44, 196, 183, 0.15), transparent 40%),
        radial-gradient(circle at 90% 10%, rgba(240, 160, 75, 0.15), transparent 45%),
        linear-gradient(120deg, var(--bg1), var(--bg2) 60%, var(--bg3));
      min-height: 100vh;
    }
    body::before {
      content: "";
      position: fixed;
      inset: -20% 10% auto auto;
      width: 320px;
      height: 320px;
      background: linear-gradient(135deg, rgba(44, 196, 183, 0.35), rgba(240, 160, 75, 0.2));
      filter: blur(30px);
      border-radius: 50%;
      pointer-events: none;
    }
    header {
      padding: 28px 20px 12px;
      text-align: center;
    }
    h1 {
      margin: 0;
      font-weight: 700;
      letter-spacing: 0.04em;
    }
    p.sub {
      margin: 8px 0 0;
      color: var(--muted);
    }
    main {
      max-width: 1200px;
      margin: 0 auto;
      padding: 20px;
      display: grid;
      gap: 20px;
      grid-template-columns: 1fr;
    }
    .grid-row {
      display: grid;
      gap: 20px;
      grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
      align-items: stretch;
    }
    .grid-row .card {
      height: 100%;
    }
    .card {
      background: linear-gradient(160deg, rgba(255, 255, 255, 0.02), rgba(255, 255, 255, 0.0));
      border: 1px solid rgba(255, 255, 255, 0.08);
      border-radius: 18px;
      padding: 18px;
      backdrop-filter: blur(6px);
      box-shadow: 0 18px 40px rgba(0, 0, 0, 0.25);
      animation: rise 0.6s ease forwards;
      opacity: 0;
    }
    .card:nth-child(2) { animation-delay: 0.1s; }
    .card:nth-child(3) { animation-delay: 0.2s; }
    .card h2 {
      margin: 0 0 10px;
      font-size: 1.1rem;
      letter-spacing: 0.03em;
      text-transform: uppercase;
    }
    .stat {
      display: flex;
      align-items: center;
      justify-content: space-between;
      margin: 10px 0 6px;
      color: var(--muted);
      font-size: 0.9rem;
    }
    .stat-right {
      display: flex;
      align-items: center;
      gap: 8px;
    }
    .qual {
      font-size: 0.7rem;
      font-weight: 600;
      text-transform: uppercase;
      padding: 3px 8px;
      border-radius: 999px;
      background: rgba(255, 255, 255, 0.08);
      color: var(--muted);
    }
    .qual.good { background: rgba(44, 196, 183, 0.2); color: var(--accent); }
    .qual.moderate { background: rgba(240, 160, 75, 0.2); color: var(--accent2); }
    .qual.usg { background: rgba(255, 140, 60, 0.2); color: #ff8c3c; }
    .qual.unhealthy { background: rgba(249, 93, 93, 0.2); color: var(--danger); }
    .qual.very { background: rgba(214, 69, 65, 0.25); color: #d64541; }
    .qual.hazard { background: rgba(139, 30, 30, 0.25); color: #8b1e1e; }
    .bar {
      background: rgba(255, 255, 255, 0.08);
      border-radius: 999px;
      overflow: hidden;
      position: relative;
      height: 18px;
    }
    .bar .fill {
      height: 100%;
      width: 0%;
      background: linear-gradient(90deg, var(--accent), var(--accent2));
      transition: width 0.6s ease;
    }
    .bar .value {
      position: absolute;
      right: 8px;
      top: 50%;
      transform: translateY(-50%);
      font-family: "JetBrains Mono", monospace;
      font-size: 0.78rem;
      color: #0a1115;
    }
    .gps-grid { display: grid; gap: 10px; }
    .gps-row {
      display: flex;
      justify-content: space-between;
      align-items: center;
      flex-wrap: wrap;
      gap: 8px;
      padding: 10px 12px;
      border-radius: 12px;
      background: rgba(255, 255, 255, 0.05);
    }
    .gps-row > div:first-child {
      flex: 0 0 auto;
      min-width: 90px;
    }
    .gps-row > div:last-child {
      flex: 1 1 160px;
      min-width: 0;
      text-align: right;
      overflow-wrap: anywhere;
      word-break: break-word;
    }
    .gps-val {
      text-align: right;
    }
    .gps-coord {
      font-size: 0.82rem;
      line-height: 1.2;
    }
    .gps-debug {
      margin-top: 6px;
    }
    .gps-debug summary {
      cursor: pointer;
      font-size: 0.82rem;
      color: var(--muted);
      margin-bottom: 8px;
    }
    .gps-debug summary::marker {
      color: var(--muted);
    }
    .gps-debug .gps-row {
      align-items: flex-start;
    }
    .gps-debug .gps-row div:last-child {
      max-width: 60%;
      word-break: break-word;
      text-align: right;
    }
    .dashboard-card {
      display: grid;
      gap: 12px;
      align-content: start;
    }
    .dial-group {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
      justify-items: center;
    }
    .dashboard {
      display: grid;
      gap: 12px;
    }
    .trend-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(90px, 1fr));
      gap: 8px;
    }
    .trend-card {
      padding: 10px 12px;
      border-radius: 14px;
      background: rgba(255, 255, 255, 0.04);
      border: 1px solid rgba(255, 255, 255, 0.06);
      box-shadow: inset 0 0 16px rgba(0, 0, 0, 0.25);
    }
    .trend-head {
      display: flex;
      justify-content: space-between;
      align-items: center;
      flex-wrap: wrap;
      row-gap: 4px;
      margin-bottom: 6px;
    }
    .trend-label {
      font-size: 0.68rem;
      text-transform: uppercase;
      letter-spacing: 0.16em;
      color: var(--muted);
      flex: 1 1 100%;
    }
    .trend-value {
      font-family: "JetBrains Mono", monospace;
      font-size: 0.85rem;
      color: var(--text);
      flex: 1 1 100%;
      text-align: right;
    }
    .trend-svg {
      width: 100%;
      height: 28px;
    }
    .trend-line {
      fill: none;
      stroke: rgba(44, 196, 183, 0.9);
      stroke-width: 2;
      stroke-linecap: round;
      stroke-linejoin: round;
    }
    .dial-wrap {
      display: grid;
      gap: 10px;
      justify-items: center;
    }
    .dial-title {
      font-size: 0.72rem;
      text-transform: uppercase;
      letter-spacing: 0.18em;
      color: var(--muted);
    }
    .dial {
      --angle: -135deg;
      --ring: conic-gradient(from 225deg, rgba(255, 255, 255, 0.08) 0deg 270deg,
                             rgba(255, 255, 255, 0.05) 270deg 360deg);
      width: 150px;
      height: 150px;
      border-radius: 50%;
      position: relative;
      background: radial-gradient(circle at 45% 35%, rgba(18, 28, 36, 0.7),
                                  rgba(10, 16, 20, 0.95));
      box-shadow:
        inset 0 0 24px rgba(0, 0, 0, 0.5),
        0 12px 22px rgba(0, 0, 0, 0.35);
    }
    .dial::before {
      content: "";
      position: absolute;
      inset: 6px;
      border-radius: 50%;
      background: var(--ring);
      box-shadow: inset 0 0 10px rgba(44, 196, 183, 0.18);
    }
    .dial::after {
      content: "";
      position: absolute;
      inset: 18px;
      border-radius: 50%;
      background:
        radial-gradient(circle at 40% 35%, rgba(26, 38, 46, 0.85),
                        rgba(10, 16, 20, 0.96));
      border: 1px solid rgba(255, 255, 255, 0.08);
      box-shadow: inset 0 0 12px rgba(0, 0, 0, 0.4);
    }
    .dial.temp {
      --ring: conic-gradient(from 225deg,
                             #4fc3f7 0deg 60deg,
                             #44d0b1 60deg 115deg,
                             #2ecc71 115deg 170deg,
                             #f4d03f 170deg 220deg,
                             #f39c12 220deg 250deg,
                             #e74c3c 250deg 270deg,
                             rgba(255, 255, 255, 0.05) 270deg 360deg);
    }
    .dial.press {
      --ring: conic-gradient(from 225deg,
                             #5dade2 0deg 50deg,
                             #48c9b0 50deg 120deg,
                             #a3e635 120deg 175deg,
                             #f1c40f 175deg 215deg,
                             #f39c12 215deg 245deg,
                             #e67e22 245deg 260deg,
                             #e74c3c 260deg 270deg,
                             rgba(255, 255, 255, 0.05) 270deg 360deg);
    }
    .dial-needle {
      position: absolute;
      width: 3px;
      height: 46px;
      top: 50%;
      left: 50%;
      transform-origin: bottom center;
      transform: translate(-50%, -100%) rotate(var(--angle));
      background: linear-gradient(180deg, #e8f4f6, #4fc3f7);
      border-radius: 999px;
      box-shadow: 0 0 10px rgba(79, 195, 247, 0.5);
      z-index: 3;
    }
    .dial-center {
      position: absolute;
      width: 10px;
      height: 10px;
      border-radius: 50%;
      background: #e9f1f3;
      top: 50%;
      left: 50%;
      transform: translate(-50%, -50%);
      z-index: 4;
      box-shadow: 0 0 8px rgba(255, 255, 255, 0.4);
    }
    .dial-readout {
      position: absolute;
      left: 0;
      right: 0;
      bottom: 14px;
      display: grid;
      place-items: center;
      text-align: center;
      z-index: 5;
      gap: 2px;
    }
    .dial-value {
      font-family: "JetBrains Mono", monospace;
      font-size: 1.05rem;
      color: var(--text);
    }
    .dial-unit {
      font-size: 0.72rem;
      color: var(--muted);
    }
    .compass-wrap {
      display: flex;
      justify-content: center;
      padding: 12px 0 6px;
    }
    .compass {
      width: 160px;
      height: 160px;
      border-radius: 50%;
      background:
        radial-gradient(circle at 50% 50%, rgba(17, 28, 36, 0.7), rgba(12, 18, 24, 0.95));
      border: 1px solid rgba(255, 255, 255, 0.15);
      box-shadow:
        inset 0 0 24px rgba(0, 0, 0, 0.6),
        0 12px 26px rgba(0, 0, 0, 0.35);
      position: relative;
      transform-style: preserve-3d;
    }
    .mag-3d {
      width: 170px;
      height: 170px;
      margin: 6px auto 10px;
      perspective: 600px;
    }
    .mag-scene {
      width: 100%;
      height: 100%;
      position: relative;
      transform-style: preserve-3d;
    }
    .cansat {
      position: absolute;
      top: 50%;
      left: 50%;
      transform-style: preserve-3d;
      transform: translate(-50%, -50%);
      transition: transform 0.08s linear;
      cursor: pointer;
    }
    .cansat.off {
      opacity: 0.4;
      filter: grayscale(1);
    }
    .cansat-body {
      position: absolute;
      width: 64px;
      height: 110px;
      transform-style: preserve-3d;
      transform: translateX(-32px) translateY(-55px);
    }
    .cansat-shell {
      position: absolute;
      width: 64px;
      height: 90px;
      top: 10px;
      transform-style: preserve-3d;
    }
    .cansat-seg {
      position: absolute;
      width: 14px;
      height: 90px;
      left: 25px;
      top: 0;
      background: linear-gradient(180deg,
        rgba(12, 22, 30, 0.95) 0%,
        rgba(44, 196, 183, 0.35) 50%,
        rgba(12, 22, 30, 0.95) 100%);
      border: 1px solid rgba(44, 196, 183, 0.18);
      box-shadow: inset 0 0 8px rgba(0, 0, 0, 0.45);
      backface-visibility: hidden;
    }
    .cansat-shell .cansat-seg:nth-child(1) { transform: rotateY(0deg) translateZ(28px); }
    .cansat-shell .cansat-seg:nth-child(2) { transform: rotateY(30deg) translateZ(28px); }
    .cansat-shell .cansat-seg:nth-child(3) { transform: rotateY(60deg) translateZ(28px); }
    .cansat-shell .cansat-seg:nth-child(4) { transform: rotateY(90deg) translateZ(28px); }
    .cansat-shell .cansat-seg:nth-child(5) { transform: rotateY(120deg) translateZ(28px); }
    .cansat-shell .cansat-seg:nth-child(6) { transform: rotateY(150deg) translateZ(28px); }
    .cansat-shell .cansat-seg:nth-child(7) { transform: rotateY(180deg) translateZ(28px); }
    .cansat-shell .cansat-seg:nth-child(8) { transform: rotateY(210deg) translateZ(28px); }
    .cansat-shell .cansat-seg:nth-child(9) { transform: rotateY(240deg) translateZ(28px); }
    .cansat-shell .cansat-seg:nth-child(10) { transform: rotateY(270deg) translateZ(28px); }
    .cansat-shell .cansat-seg:nth-child(11) { transform: rotateY(300deg) translateZ(28px); }
    .cansat-shell .cansat-seg:nth-child(12) { transform: rotateY(330deg) translateZ(28px); }
    .cansat-cap {
      position: absolute;
      width: 56px;
      height: 56px;
      left: 0;
      border-radius: 50%;
      background: radial-gradient(circle at 35% 30%,
        rgba(235, 248, 255, 0.55),
        rgba(18, 30, 38, 0.95));
      border: 1px solid rgba(255, 255, 255, 0.12);
      box-shadow: 0 0 10px rgba(44, 196, 183, 0.25);
      transform: rotateX(90deg);
      transform-style: preserve-3d;
      backface-visibility: hidden;
    }
    .cansat-cap.top {
      top: -18px;
      left: 4px;
    }
    .cansat-cap.bottom {
      top: 72px;
      left: 4px;
      filter: brightness(0.85);
      background: radial-gradient(circle at 50% 65%,
        rgba(40, 55, 65, 0.9),
        rgba(12, 18, 24, 0.98));
      box-shadow: 0 -8px 14px rgba(0, 0, 0, 0.5);
      transform: rotateX(-90deg);
    }
    .cansat-base {
      position: absolute;
      width: 60px;
      height: 10px;
      left: 2px;
      top: 90px;
      border-radius: 50%;
      background: radial-gradient(ellipse at 50% 30%,
        rgba(20, 32, 40, 0.9),
        rgba(8, 12, 16, 0.98) 60%,
        rgba(0, 0, 0, 0) 72%);
      transform: translateZ(26px);
      opacity: 0.9;
    }
    .cansat-strap {
      position: absolute;
      width: 68px;
      height: 10px;
      left: -2px;
      top: 34px;
      background: linear-gradient(90deg,
        rgba(255, 190, 80, 0.55) 0%,
        rgba(255, 220, 120, 0.9) 50%,
        rgba(255, 190, 80, 0.55) 100%);
      border-radius: 6px;
      box-shadow: 0 0 8px rgba(255, 190, 80, 0.35);
      transform: translateZ(28px);
    }
    .cansat-strap.lower {
      top: 66px;
      transform: translateZ(28px);
    }
    .cansat-panel {
      position: absolute;
      width: 16px;
      height: 32px;
      top: 36px;
      background: linear-gradient(180deg,
        rgba(18, 38, 48, 0.95),
        rgba(50, 90, 110, 0.85));
      border-radius: 4px;
      border: 1px solid rgba(80, 150, 170, 0.45);
      box-shadow: inset 0 0 8px rgba(0, 0, 0, 0.55);
    }
    .cansat-panel.left {
      left: -12px;
      transform: translateZ(18px) rotateY(-55deg);
    }
    .cansat-panel.right {
      right: -12px;
      transform: translateZ(18px) rotateY(55deg);
    }
    .cansat-bay {
      position: absolute;
      width: 28px;
      height: 20px;
      left: 18px;
      top: 20px;
      border-radius: 5px;
      background: linear-gradient(160deg,
        rgba(30, 70, 90, 0.95),
        rgba(15, 35, 50, 0.95));
      border: 1px solid rgba(120, 200, 220, 0.3);
      box-shadow:
        inset 0 0 8px rgba(0, 0, 0, 0.6),
        0 4px 8px rgba(0, 0, 0, 0.35);
      transform: translateZ(28px);
    }
    .cansat-port {
      position: absolute;
      width: 12px;
      height: 12px;
      left: 26px;
      top: 52px;
      border-radius: 50%;
      background: radial-gradient(circle, rgba(100, 200, 255, 0.6) 0%, rgba(20, 60, 100, 0.8) 100%);
      border: 1px solid rgba(100, 200, 255, 0.4);
      box-shadow: 0 0 8px rgba(100, 200, 255, 0.5);
      transform: translateZ(28px);
    }
    .cansat-led {
      position: absolute;
      width: 8px;
      height: 8px;
      left: 28px;
      top: -30px;
      border-radius: 50%;
      background: #ff3b3b;
      opacity: 0;
      box-shadow: 0 0 6px rgba(255, 80, 80, 0.5);
      transform: translateZ(24px);
    }
    .cansat-antenna {
      position: absolute;
      width: 4px;
      height: 22px;
      left: 30px;
      top: -40px;
      border-radius: 4px;
      background: linear-gradient(180deg,
        rgba(235, 248, 255, 0.85),
        rgba(80, 120, 140, 0.95));
      box-shadow: 0 0 6px rgba(200, 240, 255, 0.4);
      transform: translateZ(20px);
    }
    .cansat-antenna::after {
      content: "";
      position: absolute;
      width: 10px;
      height: 10px;
      top: -8px;
      left: -3px;
      border-radius: 50%;
      border: 1px solid rgba(200, 240, 255, 0.5);
      box-shadow: 0 0 6px rgba(200, 240, 255, 0.4);
    }
    .cansat.imu-active .cansat-led {
      animation: imu-blink 0.8s steps(2, end) infinite;
    }
    .cansat.off .cansat-led {
      animation: none;
      opacity: 0;
    }
    .imu-map {
      margin: 12px 0 6px;
      display: grid;
      gap: 8px;
      padding: 10px 12px;
      border-radius: 12px;
      background: rgba(255, 255, 255, 0.04);
      border: 1px solid rgba(255, 255, 255, 0.06);
    }
    .imu-map-title {
      font-size: 0.7rem;
      text-transform: uppercase;
      letter-spacing: 0.16em;
      color: var(--muted);
    }
    .imu-map-row {
      display: grid;
      grid-template-columns: 52px minmax(0, 1fr) auto;
      gap: 8px;
      align-items: center;
    }
    .imu-map-label {
      font-size: 0.78rem;
      color: var(--muted);
    }
    .imu-map-invert {
      display: inline-flex;
      align-items: center;
      gap: 6px;
      font-size: 0.75rem;
      color: var(--muted);
      white-space: nowrap;
    }
    .imu-map-actions {
      display: flex;
      justify-content: flex-end;
    }
    .co2-row {
      display: flex;
      align-items: center;
      gap: 8px;
    }
    .co2-row .bar {
      flex: 1 1 auto;
      min-width: 120px;
    }
    .co2-row .qual {
      min-width: 70px;
      text-align: center;
    }
    .scd-hero {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 14px;
      margin: 8px 0 12px;
      align-items: center;
    }
    .scd-ring {
      position: relative;
      width: 100%;
      max-width: 180px;
      margin: 0 auto;
      aspect-ratio: 1 / 1;
      --ring-color: var(--accent);
      filter: drop-shadow(0 0 10px rgba(44, 196, 183, 0.2));
    }
    .scd-ring svg {
      width: 100%;
      height: 100%;
      transform: rotate(-90deg);
    }
    .scd-ring .ring-track {
      stroke: rgba(255, 255, 255, 0.08);
      stroke-width: 8;
      fill: none;
    }
    .scd-ring .ring-progress {
      stroke: var(--ring-color);
      stroke-width: 8;
      fill: none;
      stroke-linecap: round;
      stroke-dasharray: 1;
      stroke-dashoffset: 1;
      transition: stroke-dashoffset 0.6s ease, stroke 0.4s ease;
    }
    .scd-ring.good { --ring-color: var(--accent); animation: ring-pulse 2.4s ease-in-out infinite; }
    .scd-ring.moderate { --ring-color: var(--accent2); }
    .scd-ring.usg { --ring-color: #ff8c3c; }
    .scd-ring.unhealthy { --ring-color: var(--danger); }
    .scd-ring.hazard { --ring-color: #8b1e1e; }
    @keyframes ring-pulse {
      0%, 100% { filter: drop-shadow(0 0 6px rgba(44, 196, 183, 0.35)); }
      50% { filter: drop-shadow(0 0 14px rgba(44, 196, 183, 0.6)); }
    }
    .ring-center {
      position: absolute;
      inset: 0;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      text-align: center;
      gap: 4px;
    }
    .ring-title {
      font-size: 0.7rem;
      letter-spacing: 0.16em;
      text-transform: uppercase;
      color: var(--muted);
    }
    .ring-value {
      font-size: 1.4rem;
      font-weight: 600;
    }
    .ring-unit {
      font-size: 0.7rem;
      color: var(--muted);
      margin-left: 4px;
      text-transform: uppercase;
    }
    .ring-status {
      font-size: 0.7rem;
      letter-spacing: 0.1em;
      text-transform: uppercase;
      color: var(--muted);
    }
    .scd-temp {
      background: rgba(255, 255, 255, 0.04);
      border: 1px solid rgba(255, 255, 255, 0.08);
      border-radius: 14px;
      padding: 12px;
      display: grid;
      gap: 8px;
      --temp-color: var(--accent);
    }
    .scd-temp.good { --temp-color: var(--accent); }
    .scd-temp.moderate { --temp-color: var(--accent2); }
    .scd-temp.unhealthy { --temp-color: var(--danger); }
    .temp-head {
      display: flex;
      align-items: baseline;
      justify-content: space-between;
      gap: 10px;
    }
    .temp-title {
      font-size: 0.78rem;
      color: var(--muted);
      text-transform: uppercase;
      letter-spacing: 0.1em;
    }
    .temp-value {
      font-size: 1.4rem;
      font-weight: 600;
    }
    .temp-unit {
      font-size: 0.7rem;
      color: var(--muted);
      margin-left: 4px;
    }
    .temp-bar {
      position: relative;
      height: 12px;
      border-radius: 999px;
      background: rgba(255, 255, 255, 0.08);
      overflow: hidden;
    }
    .temp-bar .fill {
      height: 100%;
      width: 0%;
      background: linear-gradient(90deg, var(--temp-color), rgba(255,255,255,0.1));
      transition: width 0.6s ease;
    }
    .temp-bar .marker {
      position: absolute;
      top: 50%;
      left: 0%;
      width: 10px;
      height: 10px;
      border-radius: 50%;
      transform: translate(-50%, -50%);
      background: var(--temp-color);
      box-shadow: 0 0 8px var(--temp-color);
      transition: left 0.6s ease;
    }
    .temp-status {
      font-size: 0.7rem;
      letter-spacing: 0.12em;
      text-transform: uppercase;
      color: var(--muted);
    }
    .co2-scale {
      display: grid;
      gap: 6px;
      margin: 6px 0 12px;
    }
    .co2-scale-track {
      position: relative;
      display: flex;
      height: 10px;
      border-radius: 999px;
      overflow: hidden;
      background: rgba(255, 255, 255, 0.06);
      border: 1px solid rgba(255, 255, 255, 0.08);
      --co2-good: 8.7%;
      --co2-moderate: 8.7%;
      --co2-usg: 17.4%;
      --co2-unhealthy: 65.2%;
      --co2-hazard: 0%;
    }
    .co2-scale-track .segment { height: 100%; flex: 0 0 auto; }
    .co2-scale-track .good { width: var(--co2-good); background: rgba(44, 196, 183, 0.35); }
    .co2-scale-track .moderate { width: var(--co2-moderate); background: rgba(240, 160, 75, 0.35); }
    .co2-scale-track .usg { width: var(--co2-usg); background: rgba(255, 140, 60, 0.35); }
    .co2-scale-track .unhealthy { width: var(--co2-unhealthy); background: rgba(249, 93, 93, 0.35); }
    .co2-scale-track .hazard { width: var(--co2-hazard); background: rgba(139, 30, 30, 0.35); }
    .co2-scale-marker {
      position: absolute;
      top: -4px;
      left: 0%;
      width: 2px;
      height: 18px;
      background: #e8f3f4;
      box-shadow: 0 0 6px rgba(232, 243, 244, 0.6);
      transform: translateX(-50%);
      transition: left 0.6s ease;
    }
    .co2-scale-labels {
      position: relative;
      height: 16px;
      font-size: 0.65rem;
      color: var(--muted);
      letter-spacing: 0.08em;
    }
    .co2-scale-labels span {
      position: absolute;
      transform: translateX(-50%);
    }
    .co2-scale-labels span.edge-start { transform: translateX(0); }
    .co2-scale-labels span.edge-end { transform: translateX(-100%); }
    .particle-box {
      position: relative;
      height: 180px;
      border-radius: 16px;
      overflow: hidden;
      background: radial-gradient(circle at 20% 20%, rgba(64, 120, 130, 0.2), transparent 55%),
                  radial-gradient(circle at 80% 70%, rgba(30, 60, 70, 0.3), transparent 55%),
                  rgba(8, 12, 16, 0.45);
      border: 1px solid rgba(255, 255, 255, 0.08);
    }
    .particle-layer {
      position: absolute;
      inset: 0;
      pointer-events: none;
    }
    .particle-dot {
      position: absolute;
      border-radius: 50%;
      opacity: 0.85;
      animation: dust-float var(--dur, 6s) ease-in-out infinite;
      animation-delay: var(--delay, 0s);
      box-shadow: 0 0 6px rgba(255, 255, 255, 0.18);
    }
    @keyframes dust-float {
      0% { transform: translate(0, 0); opacity: 0.6; }
      50% { transform: translate(var(--dx, 6px), var(--dy, -8px)); opacity: 1; }
      100% { transform: translate(0, 0); opacity: 0.7; }
    }
    .particle-layer.pm05 .particle-dot { background: rgba(80, 210, 255, 0.8); }
    .particle-layer.pm10 .particle-dot { background: rgba(120, 240, 180, 0.75); }
    .particle-layer.pm25 .particle-dot { background: rgba(255, 210, 110, 0.8); }
    .particle-layer.pm40 .particle-dot { background: rgba(255, 150, 90, 0.8); }
    .particle-layer.pm100 .particle-dot { background: rgba(255, 90, 90, 0.85); }
    .particle-overlay {
      position: absolute;
      inset: 12px;
      display: flex;
      flex-direction: column;
      justify-content: space-between;
      color: var(--muted);
      pointer-events: none;
    }
    .particle-title {
      font-size: 0.7rem;
      letter-spacing: 0.2em;
      text-transform: uppercase;
      color: var(--muted);
    }
    .particle-total {
      font-size: 1.3rem;
      font-weight: 600;
      color: #e8f3f4;
    }
    .particle-legend {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
      margin-top: 12px;
    }
    .particle-item {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 8px;
      padding: 8px 10px;
      border-radius: 10px;
      background: rgba(255, 255, 255, 0.04);
      border: 1px solid rgba(255, 255, 255, 0.06);
      font-size: 0.78rem;
    }
    .particle-label {
      display: inline-flex;
      align-items: center;
      gap: 6px;
      color: var(--muted);
    }
    .particle-swatch {
      width: 10px;
      height: 10px;
      border-radius: 50%;
      box-shadow: 0 0 6px rgba(255, 255, 255, 0.2);
    }
    .particle-swatch.pm05 { background: rgba(80, 210, 255, 0.85); }
    .particle-swatch.pm10 { background: rgba(120, 240, 180, 0.8); }
    .particle-swatch.pm25 { background: rgba(255, 210, 110, 0.85); }
    .particle-swatch.pm40 { background: rgba(255, 150, 90, 0.85); }
    .particle-swatch.pm100 { background: rgba(255, 90, 90, 0.9); }
    .mini-hero {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 12px;
      margin: 8px 0 12px;
      align-items: center;
    }
    .mini-tile {
      padding: 12px;
      border-radius: 14px;
      background: rgba(255, 255, 255, 0.04);
      border: 1px solid rgba(255, 255, 255, 0.08);
      display: grid;
      gap: 6px;
      min-height: 84px;
    }
    .mini-label {
      font-size: 0.68rem;
      letter-spacing: 0.16em;
      text-transform: uppercase;
      color: var(--muted);
    }
    .mini-value {
      font-size: 1.2rem;
      font-weight: 600;
    }
    .mini-value .unit {
      font-size: 0.7rem;
      color: var(--muted);
      margin-left: 4px;
      text-transform: uppercase;
    }
    .mini-sub {
      font-size: 0.7rem;
      text-transform: uppercase;
      letter-spacing: 0.12em;
      color: var(--muted);
    }
    .mini-tile.good { border-color: rgba(44, 196, 183, 0.4); }
    .mini-tile.warn { border-color: rgba(243, 156, 18, 0.4); }
    .mini-tile.bad { border-color: rgba(249, 93, 93, 0.4); }
    .gps-hero {
      display: grid;
      grid-template-columns: minmax(0, 1.1fr) minmax(0, 1fr) minmax(0, 1fr);
      gap: 12px;
      margin: 8px 0 12px;
      align-items: center;
    }
    .gps-ring {
      max-width: 160px;
      justify-self: center;
    }
    .gps-ring.ok { --ring-color: #22d3ee; }
    .gps-ring.warn { --ring-color: #ff8c3c; }
    .gps-ring.bad {
      --ring-color: var(--danger);
      filter: drop-shadow(0 0 12px rgba(249, 93, 93, 0.35));
    }
    .hw-hero .mini-value {
      font-size: 1.35rem;
    }
    .scd-trends {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 10px;
      margin-bottom: 8px;
    }
    @media (max-width: 900px) {
      .scd-hero {
        grid-template-columns: 1fr;
      }
      .scd-trends {
        grid-template-columns: 1fr;
      }
      .mini-hero {
        grid-template-columns: 1fr;
      }
      .gps-hero {
        grid-template-columns: 1fr;
      }
    }
    .btn.map-link {
      text-decoration: none;
      display: inline-flex;
      align-items: center;
      gap: 6px;
    }
    .cubesat {
      position: absolute;
      top: 50%;
      left: 50%;
      transform-style: preserve-3d;
      transform: translate(-50%, -50%);
      transition: transform 0.06s linear;
      cursor: pointer;
    }
    .cubesat.off {
      opacity: 0.35;
      filter: grayscale(1);
    }
    .cubesat-body {
      position: absolute;
      width: 86px;
      height: 86px;
      transform-style: preserve-3d;
      transform: translate(-43px, -43px);
      transform-origin: 50% 50%;
    }
    .cubesat-face {
      position: absolute;
      width: 86px;
      height: 86px;
      background: linear-gradient(145deg, #15212a 0%, #2b4a5c 45%, #111a21 100%);
      border: 1px solid rgba(140, 210, 230, 0.18);
      box-shadow: inset 0 0 14px rgba(0, 0, 0, 0.55);
      backface-visibility: hidden;
      transform-origin: 50% 50%;
    }
    .cubesat-face.front { transform: translateZ(43px); }
    .cubesat-face.back { transform: rotateY(180deg) translateZ(43px); }
    .cubesat-face.right { transform: rotateY(90deg) translateZ(43px); }
    .cubesat-face.left { transform: rotateY(-90deg) translateZ(43px); }
    .cubesat-face.top { transform: rotateX(90deg) translateZ(43px); }
    .cubesat-face.bottom { transform: rotateX(-90deg) translateZ(43px); }
    .cubesat-panel {
      position: absolute;
      width: 92px;
      height: 92px;
      left: -3px;
      top: -3px;
      border-radius: 4px;
      background: linear-gradient(135deg,
        rgba(12, 50, 70, 0.95),
        rgba(18, 90, 120, 0.9));
      border: 1px solid rgba(90, 180, 220, 0.5);
      box-shadow: inset 0 0 10px rgba(0, 0, 0, 0.6);
      backface-visibility: hidden;
    }
    .cubesat-panel.front { transform: translateZ(46px); }
    .cubesat-panel.right { transform: rotateY(90deg) translateZ(46px); }
    .cubesat-panel.top {
      transform: rotateX(90deg) translateZ(46px);
    }
    .cubesat-rails {
      position: absolute;
      width: 86px;
      height: 86px;
      transform: translateZ(0);
      transform-style: preserve-3d;
      pointer-events: none;
    }
    .cubesat-rail {
      position: absolute;
      width: 86px;
      height: 6px;
      left: 0;
      background: linear-gradient(90deg,
        rgba(220, 220, 220, 0.35),
        rgba(255, 255, 255, 0.6),
        rgba(220, 220, 220, 0.35));
      box-shadow: 0 0 6px rgba(255, 255, 255, 0.25);
    }
    .cubesat-rail.top { top: -3px; transform: translateZ(43px); }
    .cubesat-rail.bottom { bottom: -3px; transform: translateZ(43px); }
    .cubesat-rail.left {
      width: 6px;
      height: 86px;
      left: -3px;
      top: 0;
      transform: translateZ(43px);
    }
    .cubesat-rail.right {
      width: 6px;
      height: 86px;
      right: -3px;
      top: 0;
      transform: translateZ(43px);
    }
    .cubesat-antenna {
      position: absolute;
      width: 4px;
      height: 52px;
      left: 72px;
      top: -24px;
      border-radius: 4px;
      background: linear-gradient(180deg, #cfe9f6 0%, #6d8ea2 100%);
      box-shadow: 0 0 8px rgba(200, 240, 255, 0.4);
      transform: translateZ(46px);
    }
    .cubesat-antenna-tip {
      position: absolute;
      width: 10px;
      height: 10px;
      left: -3px;
      top: -10px;
      border-radius: 50%;
      background: rgba(255, 60, 60, 0.4);
      box-shadow: 0 0 6px rgba(255, 80, 80, 0.4);
      opacity: 0.2;
    }
    .cubesat.imu-active .cubesat-antenna-tip {
      animation: antenna-blink 0.7s steps(2, end) infinite;
    }
    .cubesat.off .cubesat-antenna-tip {
      animation: none;
      opacity: 0;
    }
    @keyframes antenna-blink {
      0%, 40% {
        opacity: 1;
        box-shadow: 0 0 12px rgba(255, 70, 70, 0.9);
      }
      60%, 100% {
        opacity: 0.2;
        box-shadow: 0 0 4px rgba(255, 70, 70, 0.3);
      }
    }
    @keyframes imu-blink {
      0%, 45% {
        opacity: 1;
        box-shadow: 0 0 12px rgba(255, 80, 80, 0.85);
      }
      55%, 100% {
        opacity: 0.2;
        box-shadow: 0 0 4px rgba(255, 80, 80, 0.35);
      }
    }
    .compass::before {
      content: "";
      position: absolute;
      inset: 10px;
      border-radius: 50%;
      border: 1px dashed rgba(255, 255, 255, 0.15);
      box-shadow: inset 0 0 12px rgba(44, 196, 183, 0.18);
    }
    .compass .tick {
      position: absolute;
      left: 50%;
      top: 6px;
      width: 2px;
      height: 10px;
      background: rgba(255, 255, 255, 0.5);
      transform-origin: 50% 74px;
    }
    .compass .tick.t2 { height: 6px; opacity: 0.35; }
    .compass .label {
      position: absolute;
      font-size: 0.75rem;
      letter-spacing: 0.2em;
      color: var(--muted);
      font-weight: 600;
    }
    .compass .label.n { top: 6px; left: 50%; transform: translateX(-50%); color: var(--accent); }
    .compass .label.e { right: 10px; top: 50%; transform: translateY(-50%); }
    .compass .label.s { bottom: 6px; left: 50%; transform: translateX(-50%); }
    .compass .label.w { left: 10px; top: 50%; transform: translateY(-50%); }
    .needle {
      position: absolute;
      inset: 24px;
      transform: rotate(0deg);
      transition: transform 0.4s ease;
    }
    .needle::before {
      content: "";
      position: absolute;
      left: 50%;
      top: 18%;
      width: 0;
      height: 0;
      border-left: 10px solid transparent;
      border-right: 10px solid transparent;
      border-bottom: 44px solid #f0a04b;
      transform: translateX(-50%) translateZ(2px);
      filter: drop-shadow(0 6px 6px rgba(0, 0, 0, 0.35));
    }
    .needle::after {
      content: "";
      position: absolute;
      left: 50%;
      top: 50%;
      width: 8px;
      height: 60px;
      background: linear-gradient(180deg, rgba(240, 160, 75, 0.0), rgba(44, 196, 183, 0.85));
      transform: translateX(-50%) translateY(-10%);
      border-radius: 999px;
      box-shadow: 0 0 8px rgba(44, 196, 183, 0.5);
    }
    .compass.off {
      opacity: 0.5;
      filter: grayscale(0.7);
    }
    .needle.off::before,
    .needle.off::after {
      filter: grayscale(1);
      box-shadow: none;
    }
    .btn {
      border: 1px solid rgba(255, 255, 255, 0.2);
      background: rgba(44, 196, 183, 0.15);
      color: var(--accent);
      font-family: "Space Grotesk", sans-serif;
      font-size: 0.78rem;
      letter-spacing: 0.03em;
      padding: 6px 12px;
      border-radius: 999px;
      cursor: pointer;
      transition: transform 0.2s ease, background 0.2s ease;
    }
    .btn.active {
      background: rgba(44, 196, 183, 0.4);
      color: #0f2b2c;
    }
    .btn.off {
      background: rgba(249, 93, 93, 0.2);
      color: var(--danger);
      border-color: rgba(249, 93, 93, 0.35);
    }
    .btn:active {
      transform: scale(0.98);
    }
    .bar.small {
      height: 10px;
    }
    .bar.small .value {
      font-size: 0.65rem;
      right: 6px;
    }
    .row-actions {
      display: inline-flex;
      gap: 8px;
      align-items: center;
      justify-content: flex-end;
      flex-wrap: wrap;
      max-width: 100%;
    }
    .qnh-buffer {
      display: grid;
      grid-template-columns: 1fr;
      gap: 8px;
      width: 100%;
    }
    .qnh-buffer .bar {
      width: 100%;
      min-width: 0;
    }
    .qnh-actions {
      display: flex;
      gap: 8px;
      flex-wrap: wrap;
      justify-content: flex-end;
    }
    .input {
      width: 90px;
      padding: 6px 8px;
      border-radius: 8px;
      border: 1px solid rgba(255, 255, 255, 0.2);
      background: rgba(12, 16, 19, 0.6);
      color: var(--text);
      font-family: "JetBrains Mono", monospace;
      font-size: 0.78rem;
    }
    .raw-log {
      margin: 0;
      padding: 8px;
      max-width: 100%;
      max-height: 120px;
      overflow: auto;
      white-space: pre-wrap;
      word-break: break-all;
      border-radius: 12px;
      background: rgba(12, 16, 19, 0.6);
      border: 1px solid rgba(255, 255, 255, 0.08);
      color: #cdd7da;
      font-family: "JetBrains Mono", monospace;
      font-size: 0.72rem;
      line-height: 1.25;
    }
    .tag {
      padding: 2px 6px;
      border-radius: 999px;
      background: rgba(255, 255, 255, 0.08);
      font-size: 0.7rem;
      color: var(--muted);
    }
    .gps-row span {
      font-family: "JetBrains Mono", monospace;
      color: var(--text);
    }
    .badge {
      padding: 4px 10px;
      border-radius: 999px;
      font-size: 0.8rem;
      font-weight: 600;
      background: rgba(44, 196, 183, 0.2);
      color: var(--accent);
    }
    .badge.bad {
      background: rgba(249, 93, 93, 0.2);
      color: var(--danger);
    }
    .hw-status { font-weight: 600; }
    .hw-status.ok { color: var(--accent); }
    .hw-status.err { color: var(--danger); }
    .hw-status.off { color: var(--muted); }
    .hidden { display: none !important; }
    .hw-reinit {
      padding: 4px 10px;
      font-size: 0.7rem;
      letter-spacing: 0.04em;
    }
    .fan {
      width: 24px; height: 24px;
    }
    .fan-rotor {
      animation: fan-spin var(--fan-speed, 2s) linear infinite;
      animation-play-state: paused;
      transform-origin: 50% 50%;
      transform-box: fill-box;
    }
    .fan.slow { --fan-speed: 1.5s; }
    .fan.fast { --fan-speed: 0.15s; }
    .fan.slow .fan-rotor,
    .fan.fast .fan-rotor {
      animation-play-state: running;
    }
    .fan.off { opacity: 0.3; }
    @keyframes fan-spin { from { transform: rotate(0deg); } to { transform: rotate(360deg); } }
    .fan-hero {
      display: flex;
      flex-direction: column;
      align-items: center;
      gap: 10px;
      margin: 6px 0 16px;
    }
    .fan-hero-frame {
      position: relative;
      width: 170px;
      height: 170px;
    }
    .fan-shell {
      position: absolute;
      inset: 0;
      width: 100%;
      height: 100%;
    }
    .fan-shell-frame {
      fill: rgba(18, 24, 28, 0.7);
      stroke: rgba(124, 160, 170, 0.4);
      stroke-width: 2;
    }
    .fan-shell-ring {
      fill: rgba(10, 14, 18, 0.7);
      stroke: rgba(74, 190, 198, 0.1);
      stroke-width: 2;
    }
    .fan-shell-screw {
      fill: rgba(200, 220, 230, 0.25);
      stroke: rgba(255, 255, 255, 0.15);
      stroke-width: 1;
    }
    .fan-hero-blades {
      position: absolute;
      inset: 18px;
      width: calc(100% - 36px);
      height: calc(100% - 36px);
      border-radius: 50%;
      display: flex;
      align-items: center;
      justify-content: center;
      overflow: hidden;
      background: rgba(10, 14, 18, 0.2);
    }
    .fan-rotor {
      position: absolute;
      inset: 0;
      border-radius: 50%;
      background:
          radial-gradient(circle at center, rgba(0, 10, 20, 1) 0 17%, transparent 17%),
          repeating-conic-gradient(from 15deg,
          rgba(50, 150, 150, 0.7) 0 14deg,
          rgba(50, 150, 150, 0.08) 38deg 14deg,
          transparent 34deg 60deg
        );
      will-change: transform;
      transform: translateZ(0);
    }
    .fan-hero-blades .fan-hub {
      width: 16px;
      height: 16px;
      border-radius: 50%;
      background: rgba(0, 0, 0, 0.7);
      box-shadow: inset 0 0 4px rgba(255, 255, 255, 0.08);
    }
    .fan-hero-label {
      font-size: 0.78rem;
      letter-spacing: 0.28em;
      text-transform: uppercase;
      color: var(--muted);
    }
    @media (max-width: 780px) {
      .fan-hero-frame { width: 140px; height: 140px; }
      .fan-hero-blades { inset: 16px; }
    }
    .status-led-wrap { display: flex; align-items: center; gap: 12px; padding: 10px 0 6px; }
    .status-led {
      width: 18px; height: 18px; border-radius: 50%;
      background: var(--danger);
      box-shadow: 0 0 8px var(--danger), inset 0 0 4px rgba(255,255,255,0.3);
    }
    .status-led.alive {
      background: #22d3ee;
      box-shadow: 0 0 12px #22d3ee, inset 0 0 4px rgba(255,255,255,0.4);
      animation: led-pulse 2s ease-in-out infinite;
    }
    .wifi-bars {
      display: inline-flex;
      align-items: flex-end;
      gap: 3px;
      height: 18px;
      opacity: 0.9;
      transition: opacity 0.3s ease;
      margin-left: auto;
    }
    .wifi-bars.hidden { opacity: 0; }
    .wifi-bar {
      width: 4px;
      height: 6px;
      border-radius: 2px;
      background: rgba(255, 255, 255, 0.2);
      transition: height 0.3s ease, background 0.3s ease;
    }
    .wifi-bars.level-1 .wifi-bar:nth-child(1),
    .wifi-bars.level-2 .wifi-bar:nth-child(-n+2),
    .wifi-bars.level-3 .wifi-bar:nth-child(-n+3),
    .wifi-bars.level-4 .wifi-bar:nth-child(-n+4) {
      background: #22d3ee;
      box-shadow: 0 0 6px rgba(34, 211, 238, 0.6);
    }
    .wifi-bar:nth-child(1) { height: 6px; }
    .wifi-bar:nth-child(2) { height: 10px; }
    .wifi-bar:nth-child(3) { height: 14px; }
    .wifi-bar:nth-child(4) { height: 18px; }
    .power-hero {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 10px;
      margin: 6px 0 12px;
    }
    .power-tile {
      padding: 10px 12px;
      border-radius: 14px;
      background: rgba(255, 255, 255, 0.04);
      border: 1px solid rgba(255, 255, 255, 0.08);
      display: grid;
      gap: 6px;
      min-height: 84px;
    }
    .power-title {
      font-size: 0.7rem;
      letter-spacing: 0.18em;
      text-transform: uppercase;
      color: var(--muted);
    }
    .power-value {
      font-size: 1.1rem;
      font-weight: 600;
      color: #e8f3f4;
    }
    .power-unit {
      font-size: 0.7rem;
      color: var(--muted);
      margin-left: 4px;
    }
    .power-sub {
      font-size: 0.68rem;
      color: var(--muted);
      letter-spacing: 0.08em;
      text-transform: uppercase;
    }
    .power-bar {
      height: 8px;
      border-radius: 999px;
      background: rgba(255, 255, 255, 0.08);
      overflow: hidden;
    }
    .power-bar .fill {
      height: 100%;
      width: 0%;
      background: linear-gradient(90deg, var(--accent), rgba(255,255,255,0.15));
      transition: width 0.6s ease;
    }
    .ws-dot {
      width: 10px;
      height: 10px;
      border-radius: 50%;
      background: rgba(255, 255, 255, 0.2);
      justify-self: flex-start;
      box-shadow: none;
    }
    .ws-dot.active {
      background: #22d3ee;
      box-shadow: 0 0 10px rgba(34, 211, 238, 0.7);
      animation: led-pulse 1.6s ease-in-out infinite;
    }
    @media (max-width: 900px) {
      .power-hero { grid-template-columns: 1fr; }
    }
    @keyframes led-pulse {
      0%, 100% { opacity: 1; box-shadow: 0 0 12px #22d3ee; }
      50% { opacity: 0.5; box-shadow: 0 0 6px #22d3ee; }
    }
    .boot-bar-wrap { flex: 1; display: block; }
    .boot-bar-wrap.active .fill { animation-play-state: running; opacity: 1; }
    .boot-bar-wrap.idle .fill { animation-play-state: paused; opacity: 0.35; }
    .boot-status {
      display: flex;
      align-items: center;
      gap: 12px;
      padding: 8px 0 2px;
    }
    .boot-bar {
      height: 8px; border-radius: 4px;
      background: rgba(255,255,255,0.1);
      overflow: hidden;
    }
    .boot-bar .fill {
      height: 100%; width: 30%;
      background: linear-gradient(90deg, var(--accent), #22d3ee);
      animation: boot-progress 1.5s ease-in-out infinite;
    }
    @keyframes boot-progress {
      0% { transform: translateX(-100%); }
      100% { transform: translateX(400%); }
    }
    footer {
      text-align: center;
      color: var(--muted);
      font-size: 0.8rem;
      padding: 12px 0 20px;
    }
    @keyframes rise {
      from { transform: translateY(18px); opacity: 0; }
      to { transform: translateY(0); opacity: 1; }
    }
  </style>
</head>
<body>
  <header>
    <h1>CanSat2526 Telemetry</h1>
  </header>
  <main>
    <section class="grid-row">
      <section class="card dashboard-card">
        <h2>Dashboard</h2>
        <div class="dashboard">
          <div class="dial-group">
            <div class="dial-wrap">
              <div class="dial-title">Temp</div>
              <div class="dial temp" id="tempDial">
                <div class="dial-needle"></div>
                <div class="dial-center"></div>
                <div class="dial-readout">
                  <div class="dial-value" id="tempDialValue">--</div>
                  <div class="dial-unit">C</div>
                </div>
              </div>
            </div>
            <div class="dial-wrap">
              <div class="dial-title">Pressure</div>
              <div class="dial press" id="pressDial">
                <div class="dial-needle"></div>
                <div class="dial-center"></div>
                <div class="dial-readout">
                  <div class="dial-value" id="pressDialValue">--</div>
                  <div class="dial-unit">hPa</div>
                </div>
              </div>
            </div>
          </div>
          <div class="trend-grid">
            <div class="trend-card">
              <div class="trend-head">
                <div class="trend-label">PM2.5</div>
                <div class="trend-value" id="trendPm25Value">--</div>
              </div>
              <svg class="trend-svg" viewBox="0 0 100 30" id="trendPm25">
                <polyline class="trend-line" points=""></polyline>
              </svg>
            </div>
            <div class="trend-card">
              <div class="trend-head">
                <div class="trend-label">PM10</div>
                <div class="trend-value" id="trendPm10Value">--</div>
              </div>
              <svg class="trend-svg" viewBox="0 0 100 30" id="trendPm10">
                <polyline class="trend-line" points=""></polyline>
              </svg>
            </div>
            <div class="trend-card">
              <div class="trend-head">
                <div class="trend-label">Temp</div>
                <div class="trend-value" id="trendTempValue">--</div>
              </div>
              <svg class="trend-svg" viewBox="0 0 100 30" id="trendTemp">
                <polyline class="trend-line" points=""></polyline>
              </svg>
            </div>
            <div class="trend-card">
              <div class="trend-head">
                <div class="trend-label">Pressure</div>
                <div class="trend-value" id="trendPressValue">--</div>
              </div>
              <svg class="trend-svg" viewBox="0 0 100 30" id="trendPress">
                <polyline class="trend-line" points=""></polyline>
              </svg>
            </div>
            <div class="trend-card">
              <div class="trend-head">
                <div class="trend-label">GPS Alt</div>
                <div class="trend-value" id="trendAltValue">--</div>
              </div>
              <svg class="trend-svg" viewBox="0 0 100 30" id="trendAlt">
                <polyline class="trend-line" points=""></polyline>
              </svg>
            </div>
            <div class="trend-card">
              <div class="trend-head">
                <div class="trend-label">WiFi RSSI</div>
                <div class="trend-value" id="trendRssiValue">--</div>
              </div>
              <svg class="trend-svg" viewBox="0 0 100 30" id="trendRssi">
                <polyline class="trend-line" points=""></polyline>
              </svg>
            </div>
          </div>
        </div>
      </section>
      <section class="card">
      <h2>SPS30 particle</h2>
      <div class="stat"><span>PM1.0 (ug/m3)</span><div class="stat-right"><span class="value" id="pm1Value">--</span><span class="qual" id="pm1Qual">--</span></div></div>
      <div class="bar" id="pm1Bar"><div class="fill"></div><div class="value">--</div></div>
      <div class="stat"><span>PM2.5 (ug/m3)</span><div class="stat-right"><span class="value" id="pm25Value">--</span><span class="qual" id="pm25Qual">--</span></div></div>
      <div class="bar" id="pm25Bar"><div class="fill"></div><div class="value">--</div></div>
      <div class="stat"><span>PM4.0 (ug/m3)</span><div class="stat-right"><span class="value" id="pm4Value">--</span><span class="qual" id="pm4Qual">--</span></div></div>
      <div class="bar" id="pm4Bar"><div class="fill"></div><div class="value">--</div></div>
      <div class="stat"><span>PM10 (ug/m3)</span><div class="stat-right"><span class="value" id="pm10Value">--</span><span class="qual" id="pm10Qual">--</span></div></div>
      <div class="bar" id="pm10Bar"><div class="fill"></div><div class="value">--</div></div>
      <div class="stat"><span>Typical size (um)</span><span class="value" id="tpValue">--</span></div>
      <div class="stat"><span>Quality scale</span><span class="value">PM2.5/PM10 (EPA)</span></div>
      </section>
      <section class="card">
      <h2>SPS30 number</h2>
      <div class="particle-box" id="spsParticleBox">
        <div class="particle-layer pm05" id="spsParticlesNc05"></div>
        <div class="particle-layer pm10" id="spsParticlesNc10"></div>
        <div class="particle-layer pm25" id="spsParticlesNc25"></div>
        <div class="particle-layer pm40" id="spsParticlesNc40"></div>
        <div class="particle-layer pm100" id="spsParticlesNc100"></div>
        <div class="particle-overlay">
          <div class="particle-title">Particles / cm3</div>
          <div class="particle-total" id="spsParticleTotal">--</div>
        </div>
      </div>
      <div class="particle-legend">
        <div class="particle-item">
          <div class="particle-label"><span class="particle-swatch pm05"></span>NC0.5</div>
          <span id="nc05">--</span>
        </div>
        <div class="particle-item">
          <div class="particle-label"><span class="particle-swatch pm10"></span>NC1.0</div>
          <span id="nc10">--</span>
        </div>
        <div class="particle-item">
          <div class="particle-label"><span class="particle-swatch pm25"></span>NC2.5</div>
          <span id="nc25">--</span>
        </div>
        <div class="particle-item">
          <div class="particle-label"><span class="particle-swatch pm40"></span>NC4.0</div>
          <span id="nc40">--</span>
        </div>
        <div class="particle-item">
          <div class="particle-label"><span class="particle-swatch pm100"></span>NC10</div>
          <span id="nc100">--</span>
        </div>
      </div>
      </section>
    </section>
    <section class="grid-row">
      <section class="card">
      <h2>GPS status</h2>
      <div class="gps-hero">
        <div class="scd-ring gps-ring" id="gpsSatRingWrap">
          <svg viewBox="0 0 120 120" aria-hidden="true">
            <circle class="ring-track" cx="60" cy="60" r="46"></circle>
            <circle class="ring-progress" id="gpsSatRing" cx="60" cy="60" r="46"></circle>
          </svg>
          <div class="ring-center">
            <div class="ring-title">Sat</div>
            <div class="ring-value"><span id="gpsSatsBig">--</span><span class="ring-unit">sat</span></div>
            <div class="ring-status" id="gpsFixStatus">--</div>
          </div>
        </div>
        <div class="mini-tile gps-tile" id="gpsHdopTile">
          <div class="mini-label">HDOP</div>
          <div class="mini-value"><span id="gpsHdopBig">--</span></div>
          <div class="mini-sub" id="gpsHdopQual">--</div>
          <div class="bar small" id="gpsHdopBar"><div class="fill"></div><div class="value">--</div></div>
        </div>
        <div class="mini-tile gps-tile" id="gpsAgeTile">
          <div class="mini-label">Fix age</div>
          <div class="mini-value"><span id="gpsAgeBig">--</span><span class="unit">s</span></div>
          <div class="mini-sub" id="gpsAgeStatus">--</div>
          <div class="bar small" id="gpsAgeBar"><div class="fill"></div><div class="value">--</div></div>
        </div>
      </div>
      <div class="gps-grid">
        <div class="gps-row"><div>Fix</div><div class="badge" id="gpsFix">NO FIX</div></div>
        <div class="gps-row"><div>Time (UTC)</div><div><span id="gpsTime">--</span></div></div>
        <div class="gps-row"><div>Date (UTC)</div><div><span id="gpsDate">--</span></div></div>
        <div class="gps-row"><div>Coordinates</div><div class="gps-val gps-coord"><span id="gpsLat">--</span><br><span id="gpsLng">--</span></div></div>
        <div class="gps-row"><div>Maps</div><div><a class="btn map-link" id="gpsMapLink" href="#" target="_blank" rel="noopener">Open in Maps</a></div></div>
        <div class="gps-row"><div>Altitude</div><div><span id="gpsAlt">--</span> m</div></div>
        <div class="gps-row"><div>Sat</div><div><span id="gpsSats">--</span></div></div>
        <div class="gps-row"><div>HDOP</div><div><span id="gpsHdop">--</span></div></div>
        <details class="gps-debug">
          <summary>Debug</summary>
          <div class="gps-grid">
            <div class="gps-row"><div>Last known</div><div class="gps-val gps-coord"><span id="gpsLastLat">--</span><br><span id="gpsLastLng">--</span></div></div>
            <div class="gps-row"><div>Fix type</div><div><span id="gpsFixType">--</span></div></div>
            <div class="gps-row"><div>Flags</div><div><span id="gpsFlags">--</span></div></div>
            <div class="gps-row"><div>Age</div><div><span id="gpsAge">--</span> s</div></div>
            <div class="gps-row"><div>Speed</div><div><span id="gpsSpeed">--</span></div></div>
            <div class="gps-row"><div>Heading</div><div><span id="gpsHeading">--</span></div></div>
            <div class="gps-row"><div>Vel N/E/D</div><div><span id="gpsVelNED">--</span></div></div>
            <div class="gps-row"><div>H/V Acc</div><div><span id="gpsAcc">--</span></div></div>
            <div class="gps-row"><div>Speed Acc</div><div><span id="gpsSpeedAcc">--</span></div></div>
            <div class="gps-row"><div>PDOP</div><div><span id="gpsPdop">--</span></div></div>
            <div class="gps-row"><div>RX</div><div><span id="gpsRx">--</span></div></div>
            <div class="gps-row"><div>UBX</div><div><span id="gpsUbx">--</span></div></div>
          </div>
        </details>
      </div>
      </section>
      <section class="card">
      <h2>BMP585</h2>
      <div class="mini-hero bmp-hero">
        <div class="mini-tile" id="bmpTempTile">
          <div class="mini-label">Temp</div>
          <div class="mini-value"><span id="bmpTempBig">--</span><span class="unit">C</span></div>
          <div class="mini-sub" id="bmpTempQual">--</div>
          <div class="bar small" id="bmpTempBar"><div class="fill"></div><div class="value">--</div></div>
        </div>
        <div class="mini-tile" id="bmpPressTile">
          <div class="mini-label">Pressure</div>
          <div class="mini-value"><span id="bmpPressBig">--</span><span class="unit">hPa</span></div>
          <div class="mini-sub" id="bmpPressQual">--</div>
          <div class="bar small" id="bmpPressBar"><div class="fill"></div><div class="value">--</div></div>
        </div>
        <div class="mini-tile" id="bmpAltTile">
          <div class="mini-label">Altitude</div>
          <div class="mini-value"><span id="bmpAltBig">--</span><span class="unit">m</span></div>
          <div class="mini-sub" id="bmpAltQual">--</div>
          <div class="bar small" id="bmpAltBar"><div class="fill"></div><div class="value">--</div></div>
        </div>
      </div>
      <div class="gps-grid">
        <div class="gps-row"><div>Temp</div><div><span id="bmpTemp">--</span> C</div></div>
        <div class="gps-row"><div>Pressure</div><div><span id="bmpPress">--</span> hPa</div></div>
        <div class="gps-row"><div>Altitude</div><div><span id="bmpAlt">--</span> m</div></div>
        <div class="gps-row"><div>Alt mode</div><div class="row-actions"><button class="btn" id="bmpModeQnh">QNH</button><button class="btn" id="bmpModeRel">REL</button></div></div>
        <div class="gps-row"><div>QNH</div><div class="row-actions"><input class="input" id="bmpQnh" value="1013.3"><button class="btn" id="bmpSetQnh">SET</button></div></div>
        <div class="gps-row"><div>QNH avg</div><div class="row-actions"><span id="bmpQnhAvg">--</span> hPa<span class="tag" id="bmpQnhSamples">--</span></div></div>
        <div class="gps-row"><div>QNH buffer</div><div class="row-actions qnh-buffer"><div class="qnh-bar-wrap"><div class="bar small" id="bmpQnhBar"><div class="fill"></div><div class="value">--</div></div></div><div class="qnh-actions"><button class="btn" id="bmpUseAvg">USE</button><button class="btn" id="bmpResetAvg">RESET</button></div></div></div>
        <div class="gps-row"><div>Rel base</div><div class="row-actions"><span id="bmpRel">--</span> hPa<button class="btn" id="bmpSetRel">RESET</button></div></div>
        <div class="gps-row"><div>Status</div><div><span id="bmpStatus">--</span></div></div>
      </div>
      </section>
      <section class="card">
      <h2>Magnetometer</h2>
      <div class="compass-wrap">
        <div class="compass off" id="compass">
          <div class="tick" style="transform: translateX(-50%) rotate(0deg);"></div>
          <div class="tick t2" style="transform: translateX(-50%) rotate(30deg);"></div>
          <div class="tick t2" style="transform: translateX(-50%) rotate(60deg);"></div>
          <div class="tick" style="transform: translateX(-50%) rotate(90deg);"></div>
          <div class="tick t2" style="transform: translateX(-50%) rotate(120deg);"></div>
          <div class="tick t2" style="transform: translateX(-50%) rotate(150deg);"></div>
          <div class="tick" style="transform: translateX(-50%) rotate(180deg);"></div>
          <div class="tick t2" style="transform: translateX(-50%) rotate(210deg);"></div>
          <div class="tick t2" style="transform: translateX(-50%) rotate(240deg);"></div>
          <div class="tick" style="transform: translateX(-50%) rotate(270deg);"></div>
          <div class="tick t2" style="transform: translateX(-50%) rotate(300deg);"></div>
          <div class="tick t2" style="transform: translateX(-50%) rotate(330deg);"></div>
          <div class="label n">N</div>
          <div class="label e">E</div>
          <div class="label s">S</div>
          <div class="label w">W</div>
          <div class="needle off" id="magNeedle"></div>
        </div>
      </div>
      <div class="gps-grid">
        <div class="gps-row"><div>Type</div><div><span id="magType">--</span></div></div>
        <div class="gps-row"><div>X</div><div><span id="magX">--</span></div></div>
        <div class="gps-row"><div>Y</div><div><span id="magY">--</span></div></div>
        <div class="gps-row"><div>Z</div><div><span id="magZ">--</span></div></div>
        <div class="gps-row"><div>Heading</div><div><span id="magHeading">--</span> deg</div></div>
      </div>
      </section>
    </section>
    <section class="grid-row">
      <section class="card">
      <h2>BNO085 IMU</h2>
      <div class="mag-3d">
        <div class="mag-scene">
          <div class="cubesat off" id="imuCube">
            <div class="cubesat-body">
              <div class="cubesat-face front"></div>
              <div class="cubesat-face back"></div>
              <div class="cubesat-face right"></div>
              <div class="cubesat-face left"></div>
              <div class="cubesat-face top"></div>
              <div class="cubesat-face bottom"></div>
              <div class="cubesat-panel front"></div>
              <div class="cubesat-panel right"></div>
              <div class="cubesat-panel top"></div>
              <div class="cubesat-rails">
                <div class="cubesat-rail top"></div>
                <div class="cubesat-rail bottom"></div>
                <div class="cubesat-rail left"></div>
                <div class="cubesat-rail right"></div>
              </div>
              <div class="cubesat-antenna">
                <div class="cubesat-antenna-tip"></div>
              </div>
            </div>
          </div>
        </div>
      </div>
      <div class="imu-map">
        <div class="imu-map-title">IMU orientation</div>
        <div class="imu-map-row">
          <div class="imu-map-label">Yaw</div>
          <select class="input" id="imuMapYaw">
            <option value="yaw">Yaw</option>
            <option value="pitch">Pitch</option>
            <option value="roll">Roll</option>
          </select>
          <label class="imu-map-invert"><input type="checkbox" id="imuInvYaw">Invert</label>
        </div>
        <div class="imu-map-row">
          <div class="imu-map-label">Pitch</div>
          <select class="input" id="imuMapPitch">
            <option value="yaw">Yaw</option>
            <option value="pitch">Pitch</option>
            <option value="roll">Roll</option>
          </select>
          <label class="imu-map-invert"><input type="checkbox" id="imuInvPitch">Invert</label>
        </div>
        <div class="imu-map-row">
          <div class="imu-map-label">Roll</div>
          <select class="input" id="imuMapRoll">
            <option value="yaw">Yaw</option>
            <option value="pitch">Pitch</option>
            <option value="roll">Roll</option>
          </select>
          <label class="imu-map-invert"><input type="checkbox" id="imuInvRoll">Invert</label>
        </div>
        <div class="imu-map-actions">
          <button class="btn" id="imuMapReset">RESET MAP</button>
        </div>
      </div>
      <div class="gps-grid">
        <div class="gps-row"><div>Status</div><div><span id="bnoStatus">--</span></div></div>
        <div class="gps-row"><div>Yaw</div><div><span id="bnoYaw">--</span> deg</div></div>
        <div class="gps-row"><div>Pitch</div><div><span id="bnoPitch">--</span> deg</div></div>
        <div class="gps-row"><div>Roll</div><div><span id="bnoRoll">--</span> deg</div></div>
      </div>
      </section>
      <section class="card">
      <h2>SCD40</h2>
      <div class="scd-hero">
        <div class="scd-ring" id="co2RingWrap">
          <svg viewBox="0 0 120 120" aria-hidden="true">
            <circle class="ring-track" cx="60" cy="60" r="46"></circle>
            <circle class="ring-progress" id="co2Ring" cx="60" cy="60" r="46"></circle>
          </svg>
          <div class="ring-center">
            <div class="ring-title">CO2</div>
            <div class="ring-value"><span id="scdCo2Big">--</span><span class="ring-unit">ppm</span></div>
            <div class="ring-status" id="scdCo2Status">--</div>
          </div>
        </div>
        <div class="scd-ring" id="rhRingWrap">
          <svg viewBox="0 0 120 120" aria-hidden="true">
            <circle class="ring-track" cx="60" cy="60" r="46"></circle>
            <circle class="ring-progress" id="rhRing" cx="60" cy="60" r="46"></circle>
          </svg>
          <div class="ring-center">
            <div class="ring-title">RH</div>
            <div class="ring-value"><span id="scdRhBig">--</span><span class="ring-unit">%</span></div>
            <div class="ring-status" id="scdRhStatus">--</div>
          </div>
        </div>
        <div class="scd-temp" id="scdTempWrap">
          <div class="temp-head">
            <div class="temp-title">Temp</div>
            <div class="temp-value"><span id="scdTempBig">--</span><span class="temp-unit">C</span></div>
          </div>
          <div class="temp-bar" id="scdTempBar">
            <div class="fill"></div>
            <div class="marker"></div>
          </div>
          <div class="temp-status" id="scdTempStatus">--</div>
        </div>
      </div>
      <div class="co2-scale">
        <div class="co2-scale-track" id="co2ScaleTrack">
          <div class="segment good"></div>
          <div class="segment moderate"></div>
          <div class="segment usg"></div>
          <div class="segment unhealthy"></div>
          <div class="segment hazard"></div>
          <div class="co2-scale-marker" id="co2ScaleMarker"></div>
        </div>
        <div class="co2-scale-labels">
          <span class="edge-start" style="left: 0%;">400</span>
          <span style="left: 8.7%;">800</span>
          <span style="left: 17.4%;">1200</span>
          <span style="left: 34.8%;">2000</span>
          <span class="edge-end" style="left: 100%;">5000+</span>
        </div>
      </div>
      <div class="scd-trends">
        <div class="trend-card">
          <div class="trend-head">
            <div class="trend-label">CO2 / min</div>
            <div class="trend-value" id="trendCo2Value">--</div>
          </div>
          <svg class="trend-svg" viewBox="0 0 100 30" id="trendCo2Min">
            <polyline class="trend-line" points=""></polyline>
          </svg>
        </div>
        <div class="trend-card">
          <div class="trend-head">
            <div class="trend-label">Temp / min</div>
            <div class="trend-value" id="trendCo2TempValue">--</div>
          </div>
          <svg class="trend-svg" viewBox="0 0 100 30" id="trendCo2TempMin">
            <polyline class="trend-line" points=""></polyline>
          </svg>
        </div>
        <div class="trend-card">
          <div class="trend-head">
            <div class="trend-label">RH / min</div>
            <div class="trend-value" id="trendCo2RhValue">--</div>
          </div>
          <svg class="trend-svg" viewBox="0 0 100 30" id="trendCo2RhMin">
            <polyline class="trend-line" points=""></polyline>
          </svg>
        </div>
      </div>
      <div class="gps-grid">
        <div class="gps-row"><div>CO2</div><div><span id="scdCo2">--</span> ppm</div></div>
        <div class="gps-row"><div>Temp</div><div><span id="scdTemp">--</span> C</div></div>
        <div class="gps-row"><div>RH</div><div><span id="scdRh">--</span> %</div></div>
        <div class="gps-row"><div>Status</div><div><span id="scdStatus">--</span></div></div>
      </div>
      </section>
    </section>
    <section class="grid-row">
      <section class="card">
      <h2>System</h2>
      <div class="fan-hero">
        <div class="fan-hero-frame">
          <svg class="fan-shell" viewBox="0 0 160 160" aria-hidden="true">
            <rect class="fan-shell-frame" x="6" y="6" width="148" height="148" rx="18" ry="18"></rect>
            <circle class="fan-shell-ring" cx="80" cy="80" r="54"></circle>
            <circle class="fan-shell-screw" cx="24" cy="24" r="6"></circle>
            <circle class="fan-shell-screw" cx="136" cy="24" r="6"></circle>
            <circle class="fan-shell-screw" cx="24" cy="136" r="6"></circle>
            <circle class="fan-shell-screw" cx="136" cy="136" r="6"></circle>
          </svg>
          <div class="fan fan-hero-blades off" id="spsFan" aria-label="SPS30 fan">
            <div class="fan-rotor" aria-hidden="true"></div>
            <div class="fan-hub" aria-hidden="true"></div>
          </div>
        </div>
        <div class="fan-hero-label">SPS30 FAN</div>
      </div>
      <div class="gps-grid">
        <div class="gps-row"><div>SPS30 status</div><div><span id="spsStatus">--</span></div></div>
        <div class="gps-row"><div>SPS30 error</div><div><span id="spsError">--</span></div></div>
        <div class="gps-row"><div>SPS30 flags</div><div><span id="spsFlags">--</span></div></div>
        <div class="gps-row"><div>Fan clean</div><div><button class="btn" id="cleanBtn">START</button></div></div>
        <div class="gps-row"><div>Last clean</div><div><span id="cleanAge">--</span> s</div></div>
        <div class="gps-row"><div>Auto clean</div><div><span id="cleanInterval">--</span> s</div></div>
        <div class="gps-row"><div>Uptime</div><div><span id="uptime">--</span> s</div></div>
        <div class="gps-row"><div>WiFi</div><div><span id="wifiMode">--</span></div></div>
        <div class="gps-row"><div>IP</div><div><span id="wifiIp">--</span></div></div>
      </div>
      </section>
      <section class="card">
        <h2>Power</h2>
        <div class="status-led-wrap">
          <div class="status-led" id="systemLed"></div>
          <span id="systemOnline">Online</span>
          <div class="wifi-bars hidden" id="wifiBars" aria-label="WiFi signal">
            <span class="wifi-bar"></span>
            <span class="wifi-bar"></span>
            <span class="wifi-bar"></span>
            <span class="wifi-bar"></span>
          </div>
        </div>
        <div class="power-hero">
          <div class="power-tile">
            <div class="power-title">CPU</div>
            <div class="power-value"><span id="cpuFreq">--</span><span class="power-unit">MHz</span></div>
            <div class="power-bar" id="cpuBar"><div class="fill"></div></div>
          </div>
          <div class="power-tile">
            <div class="power-title">Heap</div>
            <div class="power-value"><span id="heapFree">--</span><span class="power-unit">kB</span></div>
            <div class="power-sub">min <span id="heapMin">--</span> kB</div>
            <div class="power-bar" id="heapBar"><div class="fill"></div></div>
          </div>
          <div class="power-tile">
            <div class="power-title">WS</div>
            <div class="power-value"><span id="wsStatus">--</span></div>
            <div class="power-sub">clients <span id="wsClients">--</span></div>
            <div class="ws-dot" id="wsDot"></div>
          </div>
        </div>
        <div class="gps-grid">
          <div class="gps-row"><div>WiFi RSSI</div><div><span id="wifiRssi">--</span> dBm</div></div>
          <div class="gps-row"><div>WiFi TX</div><div class="row-actions"><select class="input" id="wifiPowerSelect"><option value="19.5">MAX 19.5</option><option value="15">HIGH 15</option><option value="11">MED 11</option><option value="7">LOW 7</option><option value="2">MIN 2</option></select><button class="btn" id="wifiPowerSet">SET</button><span class="tag" id="wifiTx">--</span></div></div>
          <div class="gps-row"><div>SPS30</div><div><button class="btn" id="spsToggle">ON</button></div></div>
          <div class="gps-row"><div>GPS</div><div><button class="btn" id="gpsToggle">ON</button></div></div>
          <div class="gps-row"><div>BMP585</div><div><button class="btn" id="bmpToggle">ON</button></div></div>
          <div class="gps-row"><div>Mag</div><div><button class="btn" id="magToggle">ON</button></div></div>
          <div class="gps-row"><div>BNO085</div><div><button class="btn" id="bnoToggle">ON</button></div></div>
          <div class="gps-row"><div>SCD40</div><div><button class="btn" id="scdToggle">ON</button></div></div>
          <div class="gps-row"><div>System</div><div><button class="btn off" id="systemReset">RESET</button></div></div>
        </div>
        <div class="boot-status">
          <div class="boot-bar-wrap active" id="bootBarWrap"><div class="boot-bar"><div class="fill"></div></div></div>
          <span id="systemStatus">Connecting...</span>
        </div>
      </section>
      <section class="card">
      <h2>Hardware Status</h2>
      <div class="mini-hero hw-hero">
        <div class="mini-tile" id="hwHealthTile">
          <div class="mini-label">Health</div>
          <div class="mini-value"><span id="hwHealthValue">--</span><span class="unit">%</span></div>
          <div class="mini-sub" id="hwHealthSub">--</div>
          <div class="bar small" id="hwHealthBar"><div class="fill"></div><div class="value">--</div></div>
        </div>
        <div class="mini-tile" id="hwCountTile">
          <div class="mini-label">Sensors</div>
          <div class="mini-value"><span id="hwOkCount">--</span>/<span id="hwTotalCount">--</span></div>
          <div class="mini-sub" id="hwFailCount">--</div>
        </div>
        <div class="mini-tile" id="hwRestartTile">
          <div class="mini-label">Restarts</div>
          <div class="mini-value" id="hwRestartBig">--</div>
          <div class="mini-sub">Supervisor</div>
        </div>
      </div>
      <div class="gps-grid">
        <div class="gps-row"><div>SPS30</div><div class="row-actions"><span id="hwSps" class="hw-status">--</span><span id="hwSpsRetry" class="hw-status off hidden"></span><button class="btn off hw-reinit hidden" id="hwSpsReinit">REINIT</button></div></div>
          <div class="gps-row"><div>BMP585</div><div class="row-actions"><span id="hwBmp" class="hw-status">--</span><button class="btn off hw-reinit hidden" id="hwBmpReinit">REINIT</button></div></div>
          <div class="gps-row"><div>BNO085</div><div class="row-actions"><span id="hwBno" class="hw-status">--</span><button class="btn off hw-reinit hidden" id="hwBnoReinit">REINIT</button></div></div>
          <div class="gps-row"><div>SCD40</div><div class="row-actions"><span id="hwScd" class="hw-status">--</span><button class="btn off hw-reinit hidden" id="hwScdReinit">REINIT</button></div></div>
          <div class="gps-row"><div>Magnetometer</div><div class="row-actions"><span id="hwMag" class="hw-status">--</span><button class="btn off hw-reinit hidden" id="hwMagReinit">REINIT</button></div></div>
          <div class="gps-row"><div>GPS</div><div class="row-actions"><span id="hwGps" class="hw-status">--</span><button class="btn off hw-reinit hidden" id="hwGpsReinit">REINIT</button></div></div>
          <div class="gps-row"><div>Supervisor</div><div><span id="hwRestarts" class="hw-status off">--</span></div></div>
        </div>
      </section>
    </section>
  </main>
  <footer>Auto refresh 1s</footer>
  <script>
    const maxPm = 500;
    function setBar(id, value, max, decimals) {
      const bar = document.getElementById(id);
      if (!bar) {
        return;
      }
      const fill = bar.querySelector('.fill');
      const label = bar.querySelector('.value');
      if (value === null || value === undefined) {
        fill.style.width = '0%';
        label.textContent = '--';
        return;
      }
      const capped = Math.min(value, max);
      const pct = max > 0 ? (capped / max) * 100 : 0;
      fill.style.width = pct.toFixed(1) + '%';
      const dec = decimals === undefined ? 1 : decimals;
      label.textContent = value.toFixed(dec);
    }
    function setBarRange(id, value, min, max, decimals) {
      const bar = document.getElementById(id);
      if (!bar) {
        return;
      }
      const fill = bar.querySelector('.fill');
      const label = bar.querySelector('.value');
      if (value === null || value === undefined || !Number.isFinite(value)) {
        fill.style.width = '0%';
        label.textContent = '--';
        return;
      }
      const span = max - min;
      const clamped = Math.min(Math.max(value, min), max);
      const pct = span > 0 ? ((clamped - min) / span) * 100 : 0;
      fill.style.width = pct.toFixed(1) + '%';
      const dec = decimals === undefined ? 1 : decimals;
      label.textContent = value.toFixed(dec);
    }
    function setText(id, value, decimals) {
      const el = document.getElementById(id);
      if (!el) {
        return;
      }
      if (value === null || value === undefined) {
        el.textContent = '--';
        return;
      }
      if (typeof value === 'number' && decimals !== undefined) {
        el.textContent = value.toFixed(decimals);
      } else {
        el.textContent = value;
      }
    }
    function setLink(id, href, label) {
      const el = document.getElementById(id);
      if (!el) {
        return;
      }
      if (!href) {
        el.setAttribute('href', '#');
        el.classList.add('off');
        el.textContent = label || 'Open in Maps';
        return;
      }
      el.setAttribute('href', href);
      el.classList.remove('off');
      if (label) {
        el.textContent = label;
      }
    }
    function classifyCo2(value) {
      if (value <= 800) return { label: 'Good', cls: 'good' };
      if (value <= 1200) return { label: 'Moderate', cls: 'moderate' };
      if (value <= 2000) return { label: 'USG', cls: 'usg' };
      if (value <= 5000) return { label: 'Unhealthy', cls: 'unhealthy' };
      return { label: 'Hazard', cls: 'hazard' };
    }
    function applyCo2Qual(value) {
      const el = document.getElementById('co2Qual');
      const ring = document.getElementById('co2RingWrap');
      const status = document.getElementById('scdCo2Status');
      if (!el) {
        if (!ring && !status) {
          return;
        }
      }
      if (!Number.isFinite(value)) {
        if (el) {
          el.textContent = '--';
          el.className = 'qual';
        }
        if (ring) {
          ring.classList.remove('good', 'moderate', 'usg', 'unhealthy', 'hazard');
        }
        if (status) {
          status.textContent = '--';
        }
        return;
      }
      const res = classifyCo2(value);
      if (el) {
        el.textContent = res.label;
        el.className = 'qual ' + res.cls;
      }
      if (ring) {
        ring.classList.remove('good', 'moderate', 'usg', 'unhealthy', 'hazard');
        ring.classList.add(res.cls);
      }
      if (status) {
        status.textContent = res.label;
      }
    }
    function classifyTemp(value) {
      if (value < 10) return { label: 'Cold', cls: 'unhealthy' };
      if (value < 28) return { label: 'OK', cls: 'good' };
      if (value < 35) return { label: 'Warm', cls: 'moderate' };
      return { label: 'Hot', cls: 'unhealthy' };
    }
    function applyTempState(value) {
      const wrap = document.getElementById('scdTempWrap');
      const status = document.getElementById('scdTempStatus');
      if (!wrap && !status) {
        return;
      }
      if (!Number.isFinite(value)) {
        if (wrap) {
          wrap.classList.remove('good', 'moderate', 'unhealthy');
        }
        if (status) {
          status.textContent = '--';
        }
        return;
      }
      const res = classifyTemp(value);
      if (wrap) {
        wrap.classList.remove('good', 'moderate', 'unhealthy');
        wrap.classList.add(res.cls);
      }
      if (status) {
        status.textContent = res.label;
      }
    }
    function classifyRh(value) {
      if (value < 30) return { label: 'Dry', cls: 'unhealthy' };
      if (value < 60) return { label: 'OK', cls: 'good' };
      if (value < 75) return { label: 'Humid', cls: 'moderate' };
      return { label: 'Wet', cls: 'unhealthy' };
    }
    function applyRhState(value) {
      const wrap = document.getElementById('rhRingWrap');
      const status = document.getElementById('scdRhStatus');
      if (!wrap && !status) {
        return;
      }
      if (!Number.isFinite(value)) {
        if (wrap) {
          wrap.classList.remove('good', 'moderate', 'unhealthy');
        }
        if (status) {
          status.textContent = '--';
        }
        return;
      }
      const res = classifyRh(value);
      if (wrap) {
        wrap.classList.remove('good', 'moderate', 'unhealthy');
        wrap.classList.add(res.cls);
      }
      if (status) {
        status.textContent = res.label;
      }
    }
    function formatVec3(x, y, z, decimals) {
      if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
        return null;
      }
      const dec = decimals === undefined ? 1 : decimals;
      return x.toFixed(dec) + ' / ' + y.toFixed(dec) + ' / ' + z.toFixed(dec);
    }
    function formatCount(value) {
      if (value === null || value === undefined) {
        return '--';
      }
      if (value >= 1000000) {
        return (value / 1000000).toFixed(1) + 'M';
      }
      if (value >= 1000) {
        return (value / 1000).toFixed(1) + 'k';
      }
      return String(value);
    }
    function pad2(value) {
      return String(value).padStart(2, '0');
    }
    function formatGpsTime(time) {
      if (!time || !time.valid) {
        return null;
      }
      return pad2(time.hour) + ':' + pad2(time.minute) + ':' + pad2(time.second);
    }
    function formatGpsDate(date) {
      if (!date || !date.valid) {
        return null;
      }
      return date.year + '-' + pad2(date.month) + '-' + pad2(date.day);
    }
    const TREND_LEN = 60;
    const MINUTE_TREND_LEN = 60;
    const trends = {
      pm25: [],
      pm10: [],
      temp: [],
      press: [],
      alt: [],
      rssi: []
    };
    const minuteTrends = {
      co2: { series: [], minute: null, sum: 0, count: 0 },
      temp: { series: [], minute: null, sum: 0, count: 0 },
      rh: { series: [], minute: null, sum: 0, count: 0 }
    };
    function pushTrend(series, value, decimals) {
      if (value === null || value === undefined) {
        return false;
      }
      let next = value;
      if (decimals !== undefined) {
        next = Number(value.toFixed(decimals));
      }
      if (series.length > 0 && series[series.length - 1] === next) {
        return false;
      }
      series.push(next);
      if (series.length > TREND_LEN) {
        series.shift();
      }
      return true;
    }
    function pushMinuteTrend(bucket, value, decimals) {
      if (!Number.isFinite(value)) {
        return false;
      }
      const minute = Math.floor(Date.now() / 60000);
      let next = value;
      if (decimals !== undefined) {
        next = Number(value.toFixed(decimals));
      }
      if (bucket.minute === null || bucket.minute !== minute) {
        bucket.minute = minute;
        bucket.sum = next;
        bucket.count = 1;
        bucket.series.push(next);
        if (bucket.series.length > MINUTE_TREND_LEN) {
          bucket.series.shift();
        }
        return true;
      }
      bucket.sum += next;
      bucket.count += 1;
      const avg = bucket.sum / bucket.count;
      bucket.series[bucket.series.length - 1] =
          decimals !== undefined ? Number(avg.toFixed(decimals)) : avg;
      return true;
    }
    function renderSparkline(id, series, minValue, maxValue, minSpan) {
      const svg = document.getElementById(id);
      if (!svg) {
        return;
      }
      const line = svg.querySelector('polyline');
      if (!line || series.length < 2) {
        if (line) {
          line.setAttribute('points', '');
        }
        return;
      }
      let min = minValue;
      let max = maxValue;
      const autoScale = min === undefined || max === undefined || min === null || max === null;
      if (autoScale) {
        min = Math.min(...series);
        max = Math.max(...series);
        if (minSpan !== undefined && max - min < minSpan) {
          const mid = (min + max) / 2;
          min = mid - minSpan / 2;
          max = mid + minSpan / 2;
        }
        const pad = (max - min) * 0.1 || 1;
        min -= pad;
        max += pad;
      }
      const w = 100;
      const h = 30;
      const step = w / (series.length - 1);
      const points = series.map((val, idx) => {
        const clamped = Math.min(Math.max((val - min) / (max - min), 0), 1);
        const x = idx * step;
        const y = h - clamped * h;
        return x.toFixed(1) + ',' + y.toFixed(1);
      }).join(' ');
      line.setAttribute('points', points);
    }
    let lastWifiRssi = null;
    let lastWsClients = null;
    function wifiLevelFromRssi(rssi) {
      if (!Number.isFinite(rssi)) {
        return 0;
      }
      if (rssi >= -55) return 4;
      if (rssi >= -65) return 3;
      if (rssi >= -75) return 2;
      if (rssi >= -85) return 1;
      return 0;
    }
    function updateWifiBars(online) {
      const bars = document.getElementById('wifiBars');
      if (!bars) {
        return;
      }
      bars.classList.toggle('hidden', !online);
      const level = wifiLevelFromRssi(lastWifiRssi);
      bars.classList.remove('level-0', 'level-1', 'level-2', 'level-3', 'level-4');
      bars.classList.add('level-' + level);
    }
    function updateWsTile(online) {
      const status = document.getElementById('wsStatus');
      const clients = document.getElementById('wsClients');
      const dot = document.getElementById('wsDot');
      if (!status && !clients && !dot) {
        return;
      }
      const live = online && wsConnected;
      if (status) status.textContent = live ? 'LIVE' : 'OFF';
      if (clients) {
        const count = Number.isFinite(lastWsClients) ? lastWsClients : 0;
        clients.textContent = live ? String(count) : '--';
      }
      if (dot) dot.classList.toggle('active', live);
    }
    const spsParticleState = {
      counts: {},
      initialized: false
    };
    function computeParticleCount(value) {
      if (!Number.isFinite(value) || value <= 0) {
        return 0;
      }
      const minDots = 4;
      const maxDots = 30;
      const scaled = Math.log10(1 + value);
      const ratio = Math.min(scaled / 3.2, 1);
      return Math.max(minDots, Math.round(minDots + ratio * (maxDots - minDots)));
    }
    function rebuildParticleLayer(layer, count) {
      if (!layer) {
        return;
      }
      layer.innerHTML = '';
      for (let i = 0; i < count; i++) {
        const dot = document.createElement('span');
        dot.className = 'particle-dot';
        const size = 2 + Math.random() * 4;
        dot.style.width = size.toFixed(1) + 'px';
        dot.style.height = size.toFixed(1) + 'px';
        dot.style.left = (Math.random() * 100).toFixed(2) + '%';
        dot.style.top = (Math.random() * 100).toFixed(2) + '%';
        dot.style.setProperty('--dx', (Math.random() * 14 - 7).toFixed(1) + 'px');
        dot.style.setProperty('--dy', (Math.random() * 14 - 7).toFixed(1) + 'px');
        dot.style.setProperty('--dur', (4 + Math.random() * 6).toFixed(1) + 's');
        dot.style.setProperty('--delay', (-Math.random() * 6).toFixed(1) + 's');
        layer.appendChild(dot);
      }
    }
    function updateSpsParticles(nc05, nc10, nc25, nc40, nc100) {
      const values = {
        nc05: Number.isFinite(nc05) ? nc05 : 0,
        nc10: Number.isFinite(nc10) ? nc10 : 0,
        nc25: Number.isFinite(nc25) ? nc25 : 0,
        nc40: Number.isFinite(nc40) ? nc40 : 0,
        nc100: Number.isFinite(nc100) ? nc100 : 0
      };
      const layers = {
        nc05: document.getElementById('spsParticlesNc05'),
        nc10: document.getElementById('spsParticlesNc10'),
        nc25: document.getElementById('spsParticlesNc25'),
        nc40: document.getElementById('spsParticlesNc40'),
        nc100: document.getElementById('spsParticlesNc100')
      };
      let changed = false;
      for (const key of Object.keys(values)) {
        const count = computeParticleCount(values[key]);
        if (spsParticleState.counts[key] !== count) {
          spsParticleState.counts[key] = count;
          rebuildParticleLayer(layers[key], count);
          changed = true;
        }
      }
      spsParticleState.initialized = spsParticleState.initialized || changed;
    }
    function setRingProgress(id, value, min, max) {
      const ring = document.getElementById(id);
      if (!ring) {
        return;
      }
      const radius = ring.r.baseVal.value;
      const circumference = 2 * Math.PI * radius;
      ring.style.strokeDasharray = circumference.toFixed(1);
      if (!Number.isFinite(value)) {
        ring.style.strokeDashoffset = circumference.toFixed(1);
        return;
      }
      const pct = (value - min) / (max - min);
      const clamped = Math.min(Math.max(pct, 0), 1);
      ring.style.strokeDashoffset =
          ((1 - clamped) * circumference).toFixed(1);
    }
    function setScaleMarker(id, value, min, max) {
      const marker = document.getElementById(id);
      if (!marker) {
        return;
      }
      if (!Number.isFinite(value)) {
        marker.style.left = '0%';
        return;
      }
      const pct = (value - min) / (max - min);
      const clamped = Math.min(Math.max(pct, 0), 1);
      marker.style.left = (clamped * 100).toFixed(1) + '%';
    }
    function setPowerBar(id, value, min, max) {
      const bar = document.getElementById(id);
      if (!bar) {
        return;
      }
      const fill = bar.querySelector('.fill');
      if (!fill) {
        return;
      }
      if (!Number.isFinite(value)) {
        fill.style.width = '0%';
        return;
      }
      const pct = (value - min) / (max - min);
      const clamped = Math.min(Math.max(pct, 0), 1);
      fill.style.width = (clamped * 100).toFixed(1) + '%';
    }
    function setTempBar(id, value, min, max) {
      const bar = document.getElementById(id);
      if (!bar) {
        return;
      }
      const fill = bar.querySelector('.fill');
      const marker = bar.querySelector('.marker');
      if (!Number.isFinite(value)) {
        fill.style.width = '0%';
        marker.style.left = '0%';
        return;
      }
      const pct = (value - min) / (max - min);
      const clamped = Math.min(Math.max(pct, 0), 1);
      const percent = (clamped * 100).toFixed(1) + '%';
      fill.style.width = percent;
      marker.style.left = percent;
    }
    function setDial(id, valueId, value, min, max, decimals) {
      const dial = document.getElementById(id);
      const label = document.getElementById(valueId);
      if (!dial || !label) {
        return;
      }
      if (value === null || value === undefined) {
        dial.style.setProperty('--angle', '-135deg');
        label.textContent = '--';
        return;
      }
      const pct = (value - min) / (max - min);
      const clamped = Math.min(Math.max(pct, 0), 1);
      const angle = -135 + clamped * 270;
      dial.style.setProperty('--angle', angle.toFixed(1) + 'deg');
      const dec = decimals === undefined ? 1 : decimals;
      label.textContent = value.toFixed(dec);
    }
    let wsConnected = false;
    let wsSocket = null;
    let lastWsMessageMs = 0;
    let wifiPowerTouchedMs = 0;
    const WS_STALE_MS = 5000;
    const WS_WATCH_MS = 2000;
    const SLOW_REFRESH_MS = 2000;
    let slowRefreshHandle = null;
    const sensorState = {
      sps30: true,
      gps: true,
      bmp: true,
      mag: true,
      bno: true,
      scd: true
    };
    const wifiPowerSteps = [19.5, 15, 11, 7, 2];
    function setToggleBtn(id, enabled) {
      const btn = document.getElementById(id);
      if (!btn) {
        return;
      }
      btn.textContent = enabled ? 'ON' : 'OFF';
      btn.classList.toggle('active', enabled);
      btn.classList.toggle('off', !enabled);
    }
    function setWifiPowerSelect(dbm) {
      const select = document.getElementById('wifiPowerSelect');
      if (!select || !Number.isFinite(dbm)) {
        return;
      }
      let closest = wifiPowerSteps[0];
      let best = Math.abs(dbm - closest);
      for (const step of wifiPowerSteps) {
        const diff = Math.abs(dbm - step);
        if (diff < best) {
          best = diff;
          closest = step;
        }
      }
      select.value = String(closest);
    }
    function applyData(data) {
      setBar('pm1Bar', data.sps.mc1p0, maxPm);
      setBar('pm25Bar', data.sps.mc2p5, maxPm);
      setBar('pm4Bar', data.sps.mc4p0, maxPm);
      setBar('pm10Bar', data.sps.mc10p0, maxPm);
      setText('pm1Value', data.sps.mc1p0, 1);
      setText('pm25Value', data.sps.mc2p5, 1);
      setText('pm4Value', data.sps.mc4p0, 1);
      setText('pm10Value', data.sps.mc10p0, 1);
      setText('tpValue', data.sps.typical_particle_size, 2);
      setText('nc05', data.sps.nc0p5, 1);
      setText('nc10', data.sps.nc1p0, 1);
      setText('nc25', data.sps.nc2p5, 1);
      setText('nc40', data.sps.nc4p0, 1);
      setText('nc100', data.sps.nc10p0, 1);
      updateSpsParticles(data.sps.nc0p5, data.sps.nc1p0, data.sps.nc2p5,
                         data.sps.nc4p0, data.sps.nc10p0);
      const totalParticles = data.sps.nc0p5 + data.sps.nc1p0 + data.sps.nc2p5 +
                             data.sps.nc4p0 + data.sps.nc10p0;
      setText('spsParticleTotal',
              Number.isFinite(totalParticles) ? formatCount(totalParticles) : null);
      applyQual('pm1Qual', data.sps.mc1p0, 'pm25');
      applyQual('pm25Qual', data.sps.mc2p5, 'pm25');
      applyQual('pm4Qual', data.sps.mc4p0, 'pm25');
      applyQual('pm10Qual', data.sps.mc10p0, 'pm10');
      const pm25Changed = pushTrend(trends.pm25, data.sps.mc2p5, 1);
      const pm10Changed = pushTrend(trends.pm10, data.sps.mc10p0, 1);
      if (pm25Changed) {
        renderSparkline('trendPm25', trends.pm25, null, null, 5);
      }
      if (pm10Changed) {
        renderSparkline('trendPm10', trends.pm10, null, null, 5);
      }
      setText('trendPm25Value', data.sps.mc2p5, 1);
      setText('trendPm10Value', data.sps.mc10p0, 1);
      const fixBadge = document.getElementById('gpsFix');
      if (data.gps.fix) {
        fixBadge.textContent = 'FIX OK';
        fixBadge.classList.remove('bad');
      } else {
        fixBadge.textContent = 'NO FIX';
        fixBadge.classList.add('bad');
      }
      const gpsRing = document.getElementById('gpsSatRingWrap');
      setText('gpsSatsBig', data.gps.sats);
      setRingProgress('gpsSatRing', data.gps.sats, 0, 20);
      setText('gpsFixStatus', data.gps.fix ? 'FIX OK' : 'NO FIX');
      if (gpsRing) {
        gpsRing.classList.remove('ok', 'warn', 'bad');
        gpsRing.classList.add(data.gps.fix ? 'ok' : 'bad');
      }
      setText('gpsLat', data.gps.lat, 6);
      setText('gpsLng', data.gps.lng, 6);
      if (Number.isFinite(data.gps.lat) && Number.isFinite(data.gps.lng)) {
        const url = 'https://maps.google.com/?q=' +
                    data.gps.lat.toFixed(6) + ',' + data.gps.lng.toFixed(6);
        setLink('gpsMapLink', url, 'Open in Maps');
      } else {
        setLink('gpsMapLink', null, 'Open in Maps');
      }
      setText('gpsLastLat', data.gps.last_lat, 6);
      setText('gpsLastLng', data.gps.last_lng, 6);
      setText('gpsAlt', data.gps.alt, 1);
      setText('gpsSats', data.gps.sats);
      setText('gpsHdop', data.gps.hdop, 1);
      setText('gpsHdopBig', data.gps.hdop, 1);
      setBar('gpsHdopBar', data.gps.hdop, 10, 1);
      const hdopTile = document.getElementById('gpsHdopTile');
      const hdopLabel = document.getElementById('gpsHdopQual');
      if (hdopTile) {
        hdopTile.classList.remove('good', 'warn', 'bad');
      }
      if (Number.isFinite(data.gps.hdop)) {
        let hdopState = 'OK';
        let hdopClass = 'good';
        if (data.gps.hdop <= 1.0) {
          hdopState = 'GOOD';
        } else if (data.gps.hdop <= 2.0) {
          hdopState = 'OK';
        } else if (data.gps.hdop <= 5.0) {
          hdopState = 'FAIR';
          hdopClass = 'warn';
        } else {
          hdopState = 'POOR';
          hdopClass = 'bad';
        }
        if (hdopLabel) hdopLabel.textContent = hdopState;
        if (hdopTile) hdopTile.classList.add(hdopClass);
      } else {
        if (hdopLabel) hdopLabel.textContent = '--';
      }
      const altChanged = pushTrend(trends.alt, data.gps.alt, 1);
      if (altChanged) {
        renderSparkline('trendAlt', trends.alt, null, null, 2);
      }
      setText('trendAltValue', data.gps.alt, 1);
      setText('gpsFixType', data.gps.fix_type);
      setText('gpsFlags', data.gps.flags);
      setText('gpsTime', formatGpsTime(data.gps.time));
      setText('gpsDate', formatGpsDate(data.gps.date));
      const ageMs = data.gps.age_ms;
      const ageSec = ageMs === null ? null : ageMs / 1000;
      setText('gpsAge', ageSec, 1);
      setText('gpsAgeBig', ageSec, 1);
      setBar('gpsAgeBar', ageSec, 10, 1);
      const ageTile = document.getElementById('gpsAgeTile');
      const ageStatus = document.getElementById('gpsAgeStatus');
      if (ageTile) {
        ageTile.classList.remove('good', 'warn', 'bad');
      }
      if (Number.isFinite(ageSec)) {
        let ageLabel = 'FRESH';
        let ageClass = 'good';
        if (ageSec <= 1.0) {
          ageLabel = 'FRESH';
        } else if (ageSec <= 3.0) {
          ageLabel = 'OK';
          ageClass = 'warn';
        } else {
          ageLabel = 'STALE';
          ageClass = 'bad';
        }
        if (ageStatus) ageStatus.textContent = ageLabel;
        if (ageTile) ageTile.classList.add(ageClass);
      } else {
        if (ageStatus) ageStatus.textContent = '--';
      }
      // Velocity and accuracy fields
      if (data.gps.vel) {
        const v = data.gps.vel;
        const speedKmh = (v.gs / 1000.0 * 3.6).toFixed(1);
        const headDeg = (v.head / 100000.0).toFixed(1);
        const headAccDeg = (v.head_acc / 100000.0).toFixed(1);
        const sAccKmh = (v.s_acc / 1000.0 * 3.6).toFixed(2);
        setText('gpsSpeed', v.valid ? speedKmh + ' km/h' : '--');
        setText('gpsHeading', v.valid ? headDeg + '\u00b0 (\u00b1' + headAccDeg + '\u00b0)' : '--');
        setText('gpsVelNED', (v.n/1000).toFixed(2) + ' / ' + (v.e/1000).toFixed(2) + ' / ' + (v.d/1000).toFixed(2) + ' m/s');
        setText('gpsSpeedAcc', '\u00b1' + sAccKmh + ' km/h');
      } else {
        setText('gpsSpeed', '--');
        setText('gpsHeading', '--');
        setText('gpsVelNED', '--');
        setText('gpsSpeedAcc', '--');
      }
      if (data.gps.acc) {
        const a = data.gps.acc;
        setText('gpsAcc', (a.h/1000).toFixed(2) + ' / ' + (a.v/1000).toFixed(2) + ' m');
      } else {
        setText('gpsAcc', '--');
      }
      setText('gpsPdop', data.gps.pdop !== undefined ? (data.gps.pdop / 100).toFixed(2) : '--');
      if (data.gps.rx) {
        const rx = data.gps.rx;
        setText('gpsRx',
                formatCount(rx.chars) + ' ch, ' +
                formatCount(rx.fix) + ' fix, ' +
                formatCount(rx.bad) + ' bad');
      } else {
        setText('gpsRx', null);
      }
      if (data.gps.ubx) {
        const ubx = data.gps.ubx;
        const age = ubx.age_ms === null ? '--' : (ubx.age_ms / 1000).toFixed(1) + 's';
        setText('gpsUbx',
                'PVT ' + formatCount(ubx.pvt) +
                ', DOP ' + formatCount(ubx.dop) +
                ', age ' + age);
      } else {
        setText('gpsUbx', null);
      }
      setText('spsStatus', data.sps.status);
      setText('spsError', data.sps.error_text || '--');
      setText('spsFlags', data.sps.status_flags || '--');
      const bmpOk = data.bmp585 && data.bmp585.ok;
      const bmp = data.bmp585 || {};
      setText('bmpTemp', bmpOk ? bmp.temp_c : null, 2);
      setText('bmpPress', bmpOk ? bmp.pressure_hpa : null, 2);
      setText('bmpAlt', bmp.altitude_m, 1);
      setText('bmpStatus', bmpOk ? 'OK' : (bmp.error_text || '--'));
      setText('bmpTempBig', bmpOk ? bmp.temp_c : null, 1);
      setText('bmpPressBig', bmpOk ? bmp.pressure_hpa : null, 1);
      setText('bmpAltBig', bmpOk ? bmp.altitude_m : null, 1);
      setBarRange('bmpTempBar', bmpOk ? bmp.temp_c : null, -10, 50, 1);
      setBarRange('bmpPressBar', bmpOk ? bmp.pressure_hpa : null, 950, 1050, 1);
      setBarRange('bmpAltBar', bmpOk ? bmp.altitude_m : null, -100, 2000, 0);
      const bmpTiles = ['bmpTempTile', 'bmpPressTile', 'bmpAltTile'];
      bmpTiles.forEach((id) => {
        const tile = document.getElementById(id);
        if (!tile) return;
        tile.classList.remove('good', 'warn', 'bad');
        if (!bmpOk) {
          tile.classList.add('bad');
        }
      });
      const tempQual = document.getElementById('bmpTempQual');
      if (tempQual) {
        let label = '--';
        let cls = 'good';
        if (bmpOk && Number.isFinite(bmp.temp_c)) {
          if (bmp.temp_c < 0) { label = 'COLD'; cls = 'warn'; }
          else if (bmp.temp_c < 10) { label = 'COOL'; cls = 'good'; }
          else if (bmp.temp_c < 30) { label = 'OK'; cls = 'good'; }
          else if (bmp.temp_c < 40) { label = 'WARM'; cls = 'warn'; }
          else { label = 'HOT'; cls = 'bad'; }
        }
        tempQual.textContent = label;
        const tile = document.getElementById('bmpTempTile');
        if (tile && bmpOk) tile.classList.add(cls);
      }
      const pressQual = document.getElementById('bmpPressQual');
      if (pressQual) {
        let label = '--';
        let cls = 'good';
        if (bmpOk && Number.isFinite(bmp.pressure_hpa)) {
          if (bmp.pressure_hpa < 990) { label = 'LOW'; cls = 'warn'; }
          else if (bmp.pressure_hpa > 1035) { label = 'HIGH'; cls = 'warn'; }
          else { label = 'OK'; cls = 'good'; }
        }
        pressQual.textContent = label;
        const tile = document.getElementById('bmpPressTile');
        if (tile && bmpOk) tile.classList.add(cls);
      }
      const altQual = document.getElementById('bmpAltQual');
      if (altQual) {
        altQual.textContent = bmpOk ? (bmp.alt_mode || '--') : '--';
      }
      setDial('tempDial', 'tempDialValue', bmpOk ? bmp.temp_c : null, -10, 50, 1);
      setDial('pressDial', 'pressDialValue', bmpOk ? bmp.pressure_hpa : null, 950, 1050, 1);
      const tempChanged = pushTrend(trends.temp, bmpOk ? bmp.temp_c : null, 2);
      const pressChanged =
          pushTrend(trends.press, bmpOk ? bmp.pressure_hpa : null, 2);
      if (tempChanged) {
        renderSparkline('trendTemp', trends.temp, null, null, 1);
      }
      if (pressChanged) {
        renderSparkline('trendPress', trends.press, null, null, 2);
      }
      setText('trendTempValue', bmpOk ? bmp.temp_c : null, 1);
      setText('trendPressValue', bmpOk ? bmp.pressure_hpa : null, 1);
      setText('bmpRel', bmp.rel_ref_hpa, 1);
      setText('bmpQnhAvg', bmp.qnh_avg, 1);
      const qnhWindow = bmp.qnh_window || 0;
      setText('bmpQnhSamples',
              bmp.qnh_samples !== undefined ? ('n=' + bmp.qnh_samples + '/' + (qnhWindow || '--')) : '--');
      setBar('bmpQnhBar', bmp.qnh_samples, qnhWindow, 0);
      const qnhInput = document.getElementById('bmpQnh');
      if (qnhInput && document.activeElement !== qnhInput &&
          bmp.ref_hpa !== null && bmp.ref_hpa !== undefined) {
        qnhInput.value = Number(bmp.ref_hpa).toFixed(1);
      }
      const qnhBtn = document.getElementById('bmpModeQnh');
      const relBtn = document.getElementById('bmpModeRel');
      if (qnhBtn && relBtn) {
        const mode = (bmp.alt_mode || '').toUpperCase();
        qnhBtn.classList.toggle('active', mode === 'QNH');
        relBtn.classList.toggle('active', mode === 'REL');
      }
      const magOk = data.mag && data.mag.ok;
      setText('magType', data.mag.type || '--');
      setText('magX', magOk ? data.mag.x : null, 1);
      setText('magY', magOk ? data.mag.y : null, 1);
      setText('magZ', magOk ? data.mag.z : null, 1);
      setText('magHeading', magOk ? data.mag.heading : null, 1);
      updateCompass(magOk ? data.mag.heading : null);
      const bno = data.bno085 || {};
      const bnoOk = bno.ok;
      setText('bnoStatus', bnoOk ? 'OK' : (bno.error_text || '--'));
      setText('bnoYaw', bnoOk ? bno.yaw : null, 1);
      setText('bnoPitch', bnoOk ? bno.pitch : null, 1);
      setText('bnoRoll', bnoOk ? bno.roll : null, 1);
      updateImuModel(bnoOk ? bno.yaw : null,
                     bnoOk ? bno.pitch : null,
                     bnoOk ? bno.roll : null);
      const scd = data.scd40 || {};
      const scdOk = scd.ok;
      setText('scdCo2', scdOk ? scd.co2_ppm : null, 0);
      setText('scdCo2Big', scdOk ? scd.co2_ppm : null, 0);
      setText('scdTemp', scdOk ? scd.temp_c : null, 2);
      setText('scdTempBig', scdOk ? scd.temp_c : null, 1);
      setText('scdRh', scdOk ? scd.rh : null, 1);
      setText('scdRhBig', scdOk ? scd.rh : null, 1);
      setText('scdStatus', scdOk ? 'OK' : (scd.error_text || '--'));
      setRingProgress('co2Ring', scdOk ? scd.co2_ppm : null, 400, 5000);
      setScaleMarker('co2ScaleMarker', scdOk ? scd.co2_ppm : null, 400, 5000);
      setRingProgress('rhRing', scdOk ? scd.rh : null, 0, 100);
      setTempBar('scdTempBar', scdOk ? scd.temp_c : null, 0, 50);
      applyCo2Qual(scdOk ? scd.co2_ppm : null);
      applyTempState(scdOk ? scd.temp_c : null);
      applyRhState(scdOk ? scd.rh : null);
      const co2MinChanged =
          pushMinuteTrend(minuteTrends.co2, scdOk ? scd.co2_ppm : null, 0);
      if (co2MinChanged) {
        renderSparkline('trendCo2Min', minuteTrends.co2.series, null, null, 200);
      }
      const co2MinLast = minuteTrends.co2.series.length
                             ? minuteTrends.co2.series[minuteTrends.co2.series.length - 1]
                             : null;
      setText('trendCo2Value', co2MinLast, 0);
      const tempMinChanged =
          pushMinuteTrend(minuteTrends.temp, scdOk ? scd.temp_c : null, 1);
      if (tempMinChanged) {
        renderSparkline('trendCo2TempMin', minuteTrends.temp.series, null, null, 2);
      }
      const tempMinLast = minuteTrends.temp.series.length
                              ? minuteTrends.temp.series[minuteTrends.temp.series.length - 1]
                              : null;
      setText('trendCo2TempValue', tempMinLast, 1);
      const rhMinChanged =
          pushMinuteTrend(minuteTrends.rh, scdOk ? scd.rh : null, 1);
      if (rhMinChanged) {
        renderSparkline('trendCo2RhMin', minuteTrends.rh.series, null, null, 2);
      }
      const rhMinLast = minuteTrends.rh.series.length
                            ? minuteTrends.rh.series[minuteTrends.rh.series.length - 1]
                            : null;
      setText('trendCo2RhValue', rhMinLast, 1);
      setText('cleanAge', data.sps.clean.last_ago_s);
      setText('cleanInterval', data.sps.clean.auto_interval_s);
      setText('uptime', data.system.uptime_s);
      setText('wifiMode', data.system.mode);
      setText('wifiIp', data.system.ip);
      const rssiRaw = data.system.rssi;
      const rssiDrv = data.system.rssi_drv;
      const rssiValue = Number.isFinite(rssiDrv) ? rssiDrv : rssiRaw;
      if (Number.isFinite(rssiValue)) {
        const rssiText = Number.isFinite(rssiDrv) && Number.isFinite(rssiRaw)
                             ? rssiDrv + ' / ' + rssiRaw
                             : String(rssiValue);
        setText('wifiRssi', rssiText);
      } else {
        setText('wifiRssi', null);
      }
      if (Number.isFinite(rssiValue)) {
        lastWifiRssi = rssiValue;
      }
      const rssiChanged = pushTrend(trends.rssi, rssiValue, 0);
      if (rssiChanged) {
        renderSparkline('trendRssi', trends.rssi, null, null, 5);
      }
      setText('trendRssiValue', rssiValue, 0);
      const cpuMhz = data.system.cpu_mhz;
      const heapFree = data.system.heap_free;
      const heapMin = data.system.heap_min;
      const heapTotal = data.system.heap_total;
      setText('cpuFreq', Number.isFinite(cpuMhz) ? cpuMhz : null, 0);
      setPowerBar('cpuBar', Number.isFinite(cpuMhz) ? cpuMhz : null, 80, 240);
      const heapFreeKb = Number.isFinite(heapFree) ? Math.round(heapFree / 1024) : null;
      const heapMinKb = Number.isFinite(heapMin) ? Math.round(heapMin / 1024) : null;
      setText('heapFree', heapFreeKb, 0);
      setText('heapMin', heapMinKb, 0);
      if (Number.isFinite(heapFree) && Number.isFinite(heapTotal) && heapTotal > 0) {
        setPowerBar('heapBar', heapFree / heapTotal * 100, 0, 100);
      } else {
        setPowerBar('heapBar', null, 0, 100);
      }
      if (Number.isFinite(data.system.ws_clients)) {
        lastWsClients = data.system.ws_clients;
      }
      if (data.system.tx_dbm !== undefined) {
        const txDbm = data.system.tx_dbm;
        const targetDbm = data.system.tx_target_dbm;
        const txRaw = data.system.tx_raw;
        const drvRaw = data.system.tx_drv_raw;
        const drvDbm = data.system.tx_drv_dbm;
        const drvErr = data.system.tx_drv_err;
        const setErr = data.system.tx_set_err;
        const applyErr = data.system.tx_apply_err;
        if (Number.isFinite(txDbm)) {
          const targetText = Number.isFinite(targetDbm)
                                 ? ' / target ' + targetDbm.toFixed(1)
                                 : '';
          const rawText = Number.isFinite(txRaw)
                              ? ' (' + txRaw + ')'
                              : '';
          setText('wifiTx', txDbm.toFixed(1) + ' dBm' + targetText + rawText);
        } else {
          setText('wifiTx', null);
        }
        const psErr = data.system.ps_err;
        if (Number.isFinite(drvDbm)) {
          setText('wifiTxDrv',
                  drvDbm.toFixed(2) + ' dBm (' + drvRaw + ') err=' +
                  drvErr + ' set=' + setErr + ' apply=' + applyErr +
                  ' ps=' + psErr);
        } else {
          setText('wifiTxDrv',
                  'err=' + drvErr + ' set=' + setErr + ' apply=' + applyErr +
                  ' ps=' + psErr);
        }
        const select = document.getElementById('wifiPowerSelect');
        const idle = !select || document.activeElement !== select;
        if (idle && (Date.now() - wifiPowerTouchedMs) > 2000) {
          if (Number.isFinite(targetDbm)) {
            setWifiPowerSelect(targetDbm);
          } else {
            setWifiPowerSelect(txDbm);
          }
        }
      }
      if (data.system.sensors) {
        sensorState.sps30 = !!data.system.sensors.sps30;
        sensorState.gps = !!data.system.sensors.gps;
        sensorState.bmp = !!data.system.sensors.bmp;
        sensorState.mag = !!data.system.sensors.mag;
        sensorState.bno = !!data.system.sensors.bno;
        sensorState.scd = !!data.system.sensors.scd;
        setToggleBtn('spsToggle', sensorState.sps30);
        setToggleBtn('gpsToggle', sensorState.gps);
        setToggleBtn('bmpToggle', sensorState.bmp);
        setToggleBtn('magToggle', sensorState.mag);
        setToggleBtn('bnoToggle', sensorState.bno);
        setToggleBtn('scdToggle', sensorState.scd);
      }
      if (data.system.hw) {
        const hw = data.system.hw;
        setHwStatus('hwSps', hw.sps, 'hwSpsReinit');
        setHwStatus('hwBmp', hw.bmp, 'hwBmpReinit');
        setHwStatus('hwBno', hw.bno, 'hwBnoReinit');
        setHwStatus('hwScd', hw.scd, 'hwScdReinit');
        setHwStatus('hwMag', hw.mag, 'hwMagReinit');
        setHwStatus('hwGps', hw.gps, 'hwGpsReinit');
        const spsRetry = document.getElementById('hwSpsRetry');
        if (spsRetry) {
          const showRetry = !!hw.sps && !!hw.sps_retry;
          spsRetry.textContent = showRetry ? '(retry: ' + hw.sps_retry + ')' : '';
          spsRetry.classList.toggle('hidden', !showRetry);
        }
        const restarts = document.getElementById('hwRestarts');
        if (restarts) restarts.textContent = 'restarts: ' + (hw.restarts || 0);
        const hwValues = [hw.sps, hw.bmp, hw.bno, hw.scd, hw.mag, hw.gps];
        const total = hwValues.length;
        const okCount = hwValues.filter(Boolean).length;
        const failCount = total - okCount;
        const healthPct = total ? Math.round((okCount / total) * 100) : 0;
        setText('hwOkCount', okCount);
        setText('hwTotalCount', total);
        setText('hwHealthValue', total ? healthPct : null, 0);
        setBar('hwHealthBar', total ? healthPct : null, 100, 0);
        const failLabel = document.getElementById('hwFailCount');
        if (failLabel) {
          failLabel.textContent = failCount ? (failCount + ' fail') : 'All OK';
        }
        const healthSub = document.getElementById('hwHealthSub');
        if (healthSub) {
          healthSub.textContent = total ? (okCount + '/' + total + ' OK') : '--';
        }
        const healthTile = document.getElementById('hwHealthTile');
        if (healthTile) {
          healthTile.classList.remove('good', 'warn', 'bad');
          if (!total) {
            healthTile.classList.add('warn');
          } else if (healthPct >= 85) {
            healthTile.classList.add('good');
          } else if (healthPct >= 60) {
            healthTile.classList.add('warn');
          } else {
            healthTile.classList.add('bad');
          }
        }
        const restartBig = document.getElementById('hwRestartBig');
        if (restartBig) restartBig.textContent = String(hw.restarts || 0);
        updateFan(hw.sps, hw.sps_cleaning);
      }
      updateSystemLed(true);
    }
    function setHwStatus(id, ok, buttonId) {
      const el = document.getElementById(id);
      if (!el) return;
      const button = buttonId ? document.getElementById(buttonId) : null;
      const showButton = !!button && !ok;
      el.textContent = ok ? 'OK' : 'FAIL';
      el.className = 'hw-status ' + (ok ? 'ok' : 'err');
      if (showButton) {
        el.classList.add('hidden');
        button.classList.remove('hidden');
      } else {
        el.classList.remove('hidden');
        if (button) button.classList.add('hidden');
      }
    }
    function updateFan(spsOk, cleaning) {
      const fan = document.getElementById('spsFan');
      if (!fan) return;
      fan.classList.remove('off', 'slow', 'fast');
      if (!spsOk) { fan.classList.add('off'); }
      else if (cleaning) { fan.classList.add('fast'); }
      else { fan.classList.add('slow'); }
    }
    let systemAlive = false;
    let lastDataMs = 0;
    function updateSystemLed(gotData) {
      if (gotData) { lastDataMs = Date.now(); systemAlive = true; }
      const led = document.getElementById('systemLed');
      const bar = document.getElementById('bootBarWrap');
      const status = document.getElementById('systemStatus');
      const onlineLabel = document.getElementById('systemOnline');
      if (!led) return;
      const isAlive = systemAlive && (Date.now() - lastDataMs < 5000);
      led.classList.toggle('alive', isAlive);
      if (bar) {
        bar.classList.toggle('active', !isAlive);
        bar.classList.toggle('idle', isAlive);
      }
      if (onlineLabel) onlineLabel.textContent = isAlive ? 'Online' : 'Offline';
      if (status) status.textContent = isAlive ? 'Loaded' : 'Connecting...';
      updateWifiBars(isAlive);
      updateWsTile(isAlive);
    }
    setInterval(() => updateSystemLed(false), 1000);
    function applyFastFrame(view) {
      const flags = view.getUint8(1);
      const bnoOk = (flags & 0x01) !== 0;
      const magOk = (flags & 0x02) !== 0;
      const yaw = bnoOk ? view.getInt16(2, true) / 10 : null;
      const pitch = bnoOk ? view.getInt16(4, true) / 10 : null;
      const roll = bnoOk ? view.getInt16(6, true) / 10 : null;
      const magX = magOk ? view.getInt16(8, true) / 10 : null;
      const magY = magOk ? view.getInt16(10, true) / 10 : null;
      const magZ = magOk ? view.getInt16(12, true) / 10 : null;
      const heading = magOk ? view.getUint16(14, true) / 10 : null;
      setText('bnoStatus', bnoOk ? 'OK' : null);
      setText('bnoYaw', yaw, 1);
      setText('bnoPitch', pitch, 1);
      setText('bnoRoll', roll, 1);
      updateImuModel(yaw, pitch, roll);
      setText('magX', magX, 1);
      setText('magY', magY, 1);
      setText('magZ', magZ, 1);
      setText('magHeading', heading, 1);
      updateCompass(heading);
    }
    function applyGpsFrame(view) {
      const flags = view.getUint8(1);
      const fix = (flags & 0x01) !== 0;
      const latlngOk = (flags & 0x02) !== 0;
      const altOk = (flags & 0x04) !== 0;
      const hdopOk = (flags & 0x08) !== 0;
      const satsOk = (flags & 0x10) !== 0;
      const lat = latlngOk ? view.getInt32(2, true) / 1e7 : null;
      const lng = latlngOk ? view.getInt32(6, true) / 1e7 : null;
      const alt = altOk ? view.getInt32(10, true) / 100 : null;
      const sats = satsOk ? view.getUint8(14) : null;
      const hdop = hdopOk ? view.getUint16(15, true) / 10 : null;
      const fixType = view.getUint8(17);
      const fixBadge = document.getElementById('gpsFix');
      if (fixBadge) {
        if (fix) {
          fixBadge.textContent = 'FIX OK';
          fixBadge.classList.remove('bad');
        } else {
          fixBadge.textContent = 'NO FIX';
          fixBadge.classList.add('bad');
        }
      }
      setText('gpsLat', lat, 6);
      setText('gpsLng', lng, 6);
      if (Number.isFinite(lat) && Number.isFinite(lng)) {
        const url = 'https://maps.google.com/?q=' +
                    lat.toFixed(6) + ',' + lng.toFixed(6);
        setLink('gpsMapLink', url, 'Open in Maps');
      } else {
        setLink('gpsMapLink', null, 'Open in Maps');
      }
      setText('gpsAlt', alt, 1);
      setText('gpsSats', sats);
      setText('gpsHdop', hdop, 1);
      setText('gpsFixType', fixType);
      const altChanged = pushTrend(trends.alt, alt, 1);
      if (altChanged) {
        renderSparkline('trendAlt', trends.alt, null, null, 2);
      }
      setText('trendAltValue', alt, 1);
    }
    async function refreshOnce() {
      try {
        const res = await fetch('/api', { cache: 'no-store' });
        const data = await res.json();
        applyData(data);
      } catch (err) {
        console.log(err);
      }
    }
    function connectWs() {
      if (!('WebSocket' in window)) {
        setInterval(refreshOnce, 1000);
        return;
      }
      const scheme = location.protocol === 'https:' ? 'wss' : 'ws';
      const wsUrl = scheme + '://' + location.host + '/ws';
      let retry = 500;
      const openSocket = () => {
        const socket = new WebSocket(wsUrl);
        wsSocket = socket;
        socket.binaryType = 'arraybuffer';
        socket.onopen = () => {
          wsConnected = true;
          retry = 500;
          lastWsMessageMs = Date.now();
          refreshOnce();
          if (!slowRefreshHandle) {
            slowRefreshHandle = setInterval(refreshOnce, SLOW_REFRESH_MS);
          }
        };
        socket.onmessage = (event) => {
          try {
            lastWsMessageMs = Date.now();
            if (typeof event.data === 'string') {
              const data = JSON.parse(event.data);
              applyData(data);
              return;
            }
            const view = new DataView(event.data);
            const type = view.getUint8(0);
            if (type === 0x01) {
              applyFastFrame(view);
            } else if (type === 0x02) {
              applyGpsFrame(view);
            }
          } catch (err) {
            console.log(err);
          }
        };
        socket.onclose = () => {
          wsConnected = false;
          wsSocket = null;
          setTimeout(openSocket, retry);
          retry = Math.min(retry * 1.5, 5000);
        };
        socket.onerror = () => {
          if (wsSocket) {
            wsSocket.close();
          }
        };
      };
      openSocket();
      if (!slowRefreshHandle) {
        slowRefreshHandle = setInterval(refreshOnce, SLOW_REFRESH_MS);
      }
      setInterval(() => {
        if (!wsSocket || !wsConnected) {
          return;
        }
        if ((Date.now() - lastWsMessageMs) > WS_STALE_MS) {
          refreshOnce();
          wsSocket.close();
        }
      }, WS_WATCH_MS);
    }
    function updateCompass(heading) {
      const compass = document.getElementById('compass');
      const needle = document.getElementById('magNeedle');
      if (heading === null || heading === undefined) {
        compass.classList.add('off');
        needle.classList.add('off');
        needle.style.transform = 'rotate(0deg)';
        return;
      }
      compass.classList.remove('off');
      needle.classList.remove('off');
      needle.style.transform = 'rotate(' + heading.toFixed(1) + 'deg)';
    }
    const imuState = {
      active: false,
      rawYaw: 0,
      rawPitch: 0,
      rawRoll: 0,
      mapYaw: 0,
      mapPitch: 0,
      mapRoll: 0,
      yaw: 0,
      pitch: 0,
      roll: 0,
      targetYaw: 0,
      targetPitch: 0,
      targetRoll: 0,
      offsetYaw: 0,
      offsetPitch: 0,
      offsetRoll: 0
    };
    function lerpAngle(current, target, alpha) {
      const diff = ((target - current + 540) % 360) - 180;
      return current + diff * alpha;
    }
    function wrapAngle(angle) {
      return ((angle + 540) % 360) - 180;
    }
    const IMU_MAP_KEY = 'imu-map';
    const IMU_AXES = ['yaw', 'pitch', 'roll'];
    const imuMapDefaults = {
      yaw: { src: 'yaw', sign: 1 },
      pitch: { src: 'pitch', sign: 1 },
      roll: { src: 'roll', sign: 1 }
    };
    function normalizeImuMap(map) {
      const out = {};
      IMU_AXES.forEach((axis) => {
        const entry = map && map[axis] ? map[axis] : {};
        const src = IMU_AXES.includes(entry.src) ? entry.src : axis;
        const sign = entry.sign === -1 ? -1 : 1;
        out[axis] = { src, sign };
      });
      return out;
    }
    function loadImuMap() {
      try {
        const raw = localStorage.getItem(IMU_MAP_KEY);
        return normalizeImuMap(raw ? JSON.parse(raw) : null);
      } catch (err) {
        return normalizeImuMap(null);
      }
    }
    let imuMap = loadImuMap();
    function saveImuMap() {
      try {
        localStorage.setItem(IMU_MAP_KEY, JSON.stringify(imuMap));
      } catch (err) {
      }
    }
    function mapImuAngles(yaw, pitch, roll) {
      const srcVals = { yaw, pitch, roll };
      const out = {};
      IMU_AXES.forEach((axis) => {
        const cfg = imuMap[axis];
        const value = srcVals[cfg.src];
        out[axis] = (value === null || value === undefined) ? null : value * cfg.sign;
      });
      return out;
    }
    function resetImuOffsets() {
      imuState.offsetYaw = 0;
      imuState.offsetPitch = 0;
      imuState.offsetRoll = 0;
      imuState.yaw = 0;
      imuState.pitch = 0;
      imuState.roll = 0;
      imuState.targetYaw = 0;
      imuState.targetPitch = 0;
      imuState.targetRoll = 0;
    }
    function updateImuModel(yaw, pitch, roll) {
      const cube = document.getElementById('imuCube');
      if (!cube || yaw === null || yaw === undefined ||
          pitch === null || pitch === undefined ||
          roll === null || roll === undefined) {
        if (cube) {
          cube.classList.add('off');
          cube.classList.remove('imu-active');
          cube.style.transform = 'translate(-50%, -50%)';
        }
        imuState.active = false;
        return;
      }
      cube.classList.remove('off');
      cube.classList.add('imu-active');
      const mapped = mapImuAngles(yaw, pitch, roll);
      imuState.rawYaw = yaw;
      imuState.rawPitch = pitch;
      imuState.rawRoll = roll;
      imuState.mapYaw = mapped.yaw;
      imuState.mapPitch = mapped.pitch;
      imuState.mapRoll = mapped.roll;
      imuState.targetYaw = wrapAngle(mapped.yaw - imuState.offsetYaw);
      imuState.targetPitch = mapped.pitch - imuState.offsetPitch;
      imuState.targetRoll = mapped.roll - imuState.offsetRoll;
      imuState.active = true;
    }
    function animateImu(now) {
      const cube = document.getElementById('imuCube');
      if (!cube) {
        requestAnimationFrame(animateImu);
        return;
      }
      if (!animateImu.last) {
        animateImu.last = now;
      }
      const dt = Math.min((now - animateImu.last) / 1000, 0.1);
      animateImu.last = now;
      const alpha = 1 - Math.exp(-dt / 0.1);
      if (imuState.active) {
        imuState.yaw = lerpAngle(imuState.yaw, imuState.targetYaw, alpha);
        imuState.pitch += (imuState.targetPitch - imuState.pitch) * alpha;
        imuState.roll += (imuState.targetRoll - imuState.roll) * alpha;
        cube.style.transform =
          'translate(-50%, -50%) ' +
          'rotateZ(' + imuState.yaw.toFixed(1) + 'deg) ' +
          'rotateX(' + imuState.pitch.toFixed(1) + 'deg) ' +
          'rotateY(' + imuState.roll.toFixed(1) + 'deg)';
      }
      requestAnimationFrame(animateImu);
    }
    requestAnimationFrame(animateImu);
    document.getElementById('cleanBtn').addEventListener('click', async () => {
      const btn = document.getElementById('cleanBtn');
      btn.textContent = '...';
      try {
        await fetch('/clean', { method: 'POST' });
      } catch (err) {
        console.log(err);
      }
      btn.textContent = 'START';
      if (!wsConnected) {
        refreshOnce();
      }
    });
    document.getElementById('bmpModeQnh').addEventListener('click', async () => {
      try {
        await fetch('/bmp/mode?value=qnh', { method: 'POST' });
      } catch (err) {
        console.log(err);
      }
      if (!wsConnected) {
        refreshOnce();
      }
    });
    document.getElementById('bmpModeRel').addEventListener('click', async () => {
      try {
        await fetch('/bmp/mode?value=rel', { method: 'POST' });
      } catch (err) {
        console.log(err);
      }
      if (!wsConnected) {
        refreshOnce();
      }
    });
    document.getElementById('bmpSetQnh').addEventListener('click', async () => {
      const input = document.getElementById('bmpQnh');
      const value = parseFloat(input.value);
      if (!Number.isFinite(value)) {
        return;
      }
      try {
        await fetch('/bmp/qnh?value=' + value.toFixed(1), { method: 'POST' });
      } catch (err) {
        console.log(err);
      }
      if (!wsConnected) {
        refreshOnce();
      }
    });
    document.getElementById('bmpSetRel').addEventListener('click', async () => {
      try {
        await fetch('/bmp/rel', { method: 'POST' });
      } catch (err) {
        console.log(err);
      }
      if (!wsConnected) {
        refreshOnce();
      }
    });
    document.getElementById('bmpUseAvg').addEventListener('click', async () => {
      const btn = document.getElementById('bmpUseAvg');
      btn.textContent = '...';
      let ok = false;
      let avgValue = null;
      const avgText = document.getElementById('bmpQnhAvg').textContent;
      const avgParsed = parseFloat(avgText);
      try {
        const res = await fetch('/bmp/qnh/avg', { method: 'POST' });
        const body = await res.json().catch(() => null);
        ok = res.ok;
        if (body && typeof body.avg === 'number') {
          avgValue = body.avg;
        }
        btn.textContent = ok ? 'USE' : 'ERR';
      } catch (err) {
        console.log(err);
        btn.textContent = 'ERR';
      }
      let useValue = Number.isFinite(avgValue) ? avgValue : avgParsed;
      if (Number.isFinite(useValue)) {
        const qnhInput = document.getElementById('bmpQnh');
        if (qnhInput) {
          qnhInput.value = useValue.toFixed(1);
        }
      }
      if (!ok && Number.isFinite(useValue)) {
        try {
          const res = await fetch('/bmp/qnh?value=' + useValue.toFixed(1),
                                  { method: 'POST' });
          ok = res.ok;
          btn.textContent = ok ? 'USE' : 'ERR';
        } catch (err) {
          console.log(err);
        }
      }
      refreshOnce();
      setTimeout(() => {
        btn.textContent = 'USE';
      }, 800);
    });
    document.getElementById('bmpResetAvg').addEventListener('click', async () => {
      try {
        await fetch('/bmp/qnh/reset', { method: 'POST' });
      } catch (err) {
        console.log(err);
      }
      if (!wsConnected) {
        refreshOnce();
      }
    });
    document.getElementById('wifiPowerSet').addEventListener('click', async () => {
      const select = document.getElementById('wifiPowerSelect');
      if (!select) {
        return;
      }
      wifiPowerTouchedMs = Date.now();
      const value = select.value;
      try {
        await fetch('/wifi/power?value=' + encodeURIComponent(value), { method: 'POST' });
      } catch (err) {
        console.log(err);
      }
      if (!wsConnected) {
        refreshOnce();
      }
    });
    const wifiSelect = document.getElementById('wifiPowerSelect');
    if (wifiSelect) {
      wifiSelect.addEventListener('change', () => {
        wifiPowerTouchedMs = Date.now();
      });
    }
    function bindToggle(id, key, endpoint) {
      const btn = document.getElementById(id);
      if (!btn) {
        return;
      }
      btn.addEventListener('click', async () => {
        const enable = !sensorState[key];
        try {
          await fetch('/sensor/' + endpoint + '?value=' + (enable ? 'on' : 'off'),
                      { method: 'POST' });
        } catch (err) {
          console.log(err);
        }
        if (!wsConnected) {
          refreshOnce();
        }
      });
    }
    bindToggle('spsToggle', 'sps30', 'sps30');
    bindToggle('gpsToggle', 'gps', 'gps');
    bindToggle('bmpToggle', 'bmp', 'bmp');
    bindToggle('magToggle', 'mag', 'mag');
    bindToggle('bnoToggle', 'bno', 'bno');
    bindToggle('scdToggle', 'scd', 'scd');
    function bindReinit(id, endpoint) {
      const btn = document.getElementById(id);
      if (!btn) {
        return;
      }
      btn.addEventListener('click', async () => {
        btn.textContent = '...';
        try {
          await fetch(endpoint, { method: 'POST' });
        } catch (err) {
          console.log(err);
        }
        btn.textContent = 'REINIT';
        if (!wsConnected) {
          refreshOnce();
        }
      });
    }
    bindReinit('hwSpsReinit', '/sensor/sps30/reinit');
    bindReinit('hwBmpReinit', '/sensor/bmp/reinit');
    bindReinit('hwBnoReinit', '/sensor/bno/reinit');
    bindReinit('hwScdReinit', '/sensor/scd/reinit');
    bindReinit('hwMagReinit', '/sensor/mag/reinit');
    bindReinit('hwGpsReinit', '/sensor/gps/reinit');
    const bnoResetBtn = document.getElementById('bnoReset');
    if (bnoResetBtn) {
      bnoResetBtn.addEventListener('click', async () => {
        bnoResetBtn.textContent = '...';
        try {
          await fetch('/bno/reset', { method: 'POST' });
        } catch (err) {
          console.log(err);
        }
        bnoResetBtn.textContent = 'RESET';
        if (!wsConnected) {
          refreshOnce();
        }
      });
    }
    document.getElementById('systemReset').addEventListener('click', async () => {
      const btn = document.getElementById('systemReset');
      btn.textContent = '...';
      systemAlive = false;
      lastDataMs = 0;
      updateSystemLed(false);
      try {
        await fetch('/system/reset', { method: 'POST' });
      } catch (err) {
        console.log(err);
      }
      btn.textContent = 'RESET';
    });
    const imuCube = document.getElementById('imuCube');
    if (imuCube) {
      imuCube.title = 'Reset orientation';
      imuCube.addEventListener('click', () => {
        if (!imuState.active) {
          return;
        }
        imuState.offsetYaw = imuState.mapYaw;
        imuState.offsetPitch = imuState.mapPitch;
        imuState.offsetRoll = imuState.mapRoll;
        imuState.yaw = 0;
        imuState.pitch = 0;
        imuState.roll = 0;
        imuState.targetYaw = 0;
        imuState.targetPitch = 0;
        imuState.targetRoll = 0;
      });
    }
    (function initImuMapControls() {
      const yawSel = document.getElementById('imuMapYaw');
      const pitchSel = document.getElementById('imuMapPitch');
      const rollSel = document.getElementById('imuMapRoll');
      const yawInv = document.getElementById('imuInvYaw');
      const pitchInv = document.getElementById('imuInvPitch');
      const rollInv = document.getElementById('imuInvRoll');
      const resetBtn = document.getElementById('imuMapReset');
      if (!yawSel || !pitchSel || !rollSel || !yawInv || !pitchInv || !rollInv) {
        return;
      }
      const applyUi = () => {
        yawSel.value = imuMap.yaw.src;
        pitchSel.value = imuMap.pitch.src;
        rollSel.value = imuMap.roll.src;
        yawInv.checked = imuMap.yaw.sign < 0;
        pitchInv.checked = imuMap.pitch.sign < 0;
        rollInv.checked = imuMap.roll.sign < 0;
      };
      const updateMap = () => {
        imuMap = normalizeImuMap({
          yaw: { src: yawSel.value, sign: yawInv.checked ? -1 : 1 },
          pitch: { src: pitchSel.value, sign: pitchInv.checked ? -1 : 1 },
          roll: { src: rollSel.value, sign: rollInv.checked ? -1 : 1 }
        });
        saveImuMap();
        resetImuOffsets();
        if (imuState.active) {
          updateImuModel(imuState.rawYaw, imuState.rawPitch, imuState.rawRoll);
        }
      };
      applyUi();
      yawSel.addEventListener('change', updateMap);
      pitchSel.addEventListener('change', updateMap);
      rollSel.addEventListener('change', updateMap);
      yawInv.addEventListener('change', updateMap);
      pitchInv.addEventListener('change', updateMap);
      rollInv.addEventListener('change', updateMap);
      if (resetBtn) {
        resetBtn.addEventListener('click', () => {
          imuMap = normalizeImuMap(imuMapDefaults);
          saveImuMap();
          applyUi();
          resetImuOffsets();
          if (imuState.active) {
            updateImuModel(imuState.rawYaw, imuState.rawPitch, imuState.rawRoll);
          }
        });
      }
    })();
    function classifyPm25(value) {
      if (value <= 12) return { label: 'Good', cls: 'good' };
      if (value <= 35.4) return { label: 'Moderate', cls: 'moderate' };
      if (value <= 55.4) return { label: 'USG', cls: 'usg' };
      if (value <= 150.4) return { label: 'Unhealthy', cls: 'unhealthy' };
      if (value <= 250.4) return { label: 'Very', cls: 'very' };
      return { label: 'Hazard', cls: 'hazard' };
    }
    function classifyPm10(value) {
      if (value <= 54) return { label: 'Good', cls: 'good' };
      if (value <= 154) return { label: 'Moderate', cls: 'moderate' };
      if (value <= 254) return { label: 'USG', cls: 'usg' };
      if (value <= 354) return { label: 'Unhealthy', cls: 'unhealthy' };
      if (value <= 424) return { label: 'Very', cls: 'very' };
      return { label: 'Hazard', cls: 'hazard' };
    }
    function applyQual(id, value, scale) {
      const el = document.getElementById(id);
      if (value === null || value === undefined) {
        el.textContent = '--';
        el.className = 'qual';
        return;
      }
      const res = scale === 'pm10' ? classifyPm10(value) : classifyPm25(value);
      el.textContent = res.label;
      el.className = 'qual ' + res.cls;
    }
    connectWs();
  </script>
</body>
</html>
)HTML";

static String jsonFloat(double value, bool valid, uint8_t decimals) {
  if (!valid) {
    return "null";
  }
  return String(value, static_cast<unsigned int>(decimals));
}

static String jsonUint(uint32_t value, bool valid) {
  if (!valid) {
    return "null";
  }
  return String(value);
}

static String spsStatusString(const Sps30Data& data) {
  if (data.last_read_ok) {
    return "OK";
  }
  if (data.last_error != 0) {
    return String("ERR ") + String(data.last_error);
  }
  return "NO DATA";
}

static uint16_t spsHighLevel(int16_t error) {
  return static_cast<uint16_t>(error) & 0xFF00;
}

static uint8_t spsLowLevel(int16_t error) {
  return static_cast<uint16_t>(error) & 0x00FF;
}

static bool spsIsExecutionError(int16_t error) {
  return spsHighLevel(error) == HighLevelError::ExecutionError;
}

static bool spsIsCrcError(int16_t error) {
  if (spsHighLevel(error) != HighLevelError::ReadError) {
    return false;
  }
  uint8_t low = spsLowLevel(error);
  return low == LowLevelError::ChecksumError || low == LowLevelError::CRCError;
}

static void spsFlushRx(uint32_t duration_ms) {
  uint32_t start = millis();
  while (millis() - start < duration_ms) {
    while (Serial2.available() > 0) {
      Serial2.read();
    }
    delay(1);
  }
}

static uint16_t ubxU16(const uint8_t* data) {
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8);
}

static uint32_t ubxU32(const uint8_t* data) {
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

static int32_t ubxI32(const uint8_t* data) {
  return static_cast<int32_t>(ubxU32(data));
}

static void ubxReset() {
  ubx.state = UbxState::SYNC1;
  ubx.cls = 0;
  ubx.id = 0;
  ubx.length = 0;
  ubx.index = 0;
  ubx.ck_a = 0;
  ubx.ck_b = 0;
  ubx.ck_a_calc = 0;
  ubx.ck_b_calc = 0;
}

static void ubxChecksumAdd(uint8_t value) {
  ubx.ck_a_calc = static_cast<uint8_t>(ubx.ck_a_calc + value);
  ubx.ck_b_calc = static_cast<uint8_t>(ubx.ck_b_calc + ubx.ck_a_calc);
}

static void gpsRawPush(uint8_t value) {
  gps_raw[gps_raw_head] = value;
  gps_raw_head = (gps_raw_head + 1) % GPS_RAW_BUFFER_SIZE;
  if (gps_raw_len < GPS_RAW_BUFFER_SIZE) {
    gps_raw_len++;
  }
}

static String gpsRawHex() {
  if (gps_raw_len == 0) {
    return "";
  }
  String out;
  out.reserve(gps_raw_len * 3);
  size_t start =
      (gps_raw_head + GPS_RAW_BUFFER_SIZE - gps_raw_len) % GPS_RAW_BUFFER_SIZE;
  for (size_t i = 0; i < gps_raw_len; i++) {
    uint8_t value = gps_raw[(start + i) % GPS_RAW_BUFFER_SIZE];
    if (i > 0) {
      out += ' ';
    }
    char buf[3] = {0};
    snprintf(buf, sizeof(buf), "%02X", value);
    out += buf;
  }
  return out;
}

static void ubxHandleNavPvt(const uint8_t* payload, uint16_t length,
                            uint32_t now_ms) {
  if (length < 92) {
    return;
  }
  gps_data.ubx_pvt_count++;
  gps_data.ubx_last_ms = now_ms;
  uint16_t year = ubxU16(payload + 4);
  uint8_t month = payload[6];
  uint8_t day = payload[7];
  uint8_t hour = payload[8];
  uint8_t minute = payload[9];
  uint8_t second = payload[10];
  uint8_t valid = payload[11];
  uint8_t fix_type = payload[20];
  uint8_t flags = payload[21];
  uint8_t num_sv = payload[23];
  int32_t lon = ubxI32(payload + 24);
  int32_t lat = ubxI32(payload + 28);
  int32_t h_msl = ubxI32(payload + 36);

  bool fix_ok = (flags & 0x01) != 0 && fix_type >= 2;

  gps_data.time_valid = (valid & 0x02) != 0;
  if (gps_data.time_valid) {
    gps_data.hour = hour;
    gps_data.minute = minute;
    gps_data.second = second;
  }

  gps_data.date_valid = (valid & 0x01) != 0;
  if (gps_data.date_valid) {
    gps_data.day = day;
    gps_data.month = month;
    gps_data.year = year;
  }

  gps_data.fix_type = fix_type;
  gps_data.flags = flags;
  gps_data.fix = fix_ok;
  gps_data.latlng_valid = fix_ok;
  if (fix_ok) {
    double raw_lat = static_cast<double>(lat) * 1e-7;
    double raw_lng = static_cast<double>(lon) * 1e-7;
    double raw_alt = static_cast<double>(h_msl) / 1000.0;
    // EMA smoothing
    if (!gps_smooth_init) {
      gps_smooth_lat = raw_lat;
      gps_smooth_lng = raw_lng;
      gps_smooth_alt = raw_alt;
      gps_smooth_init = true;
    } else {
      gps_smooth_lat += GPS_EMA_ALPHA * (raw_lat - gps_smooth_lat);
      gps_smooth_lng += GPS_EMA_ALPHA * (raw_lng - gps_smooth_lng);
      gps_smooth_alt += GPS_EMA_ALPHA * (raw_alt - gps_smooth_alt);
    }
    gps_data.lat = gps_smooth_lat;
    gps_data.lng = gps_smooth_lng;
    gps_data.alt_m = gps_smooth_alt;
    gps_data.alt_valid = true;
    gps_data.last_lat = gps_data.lat;
    gps_data.last_lng = gps_data.lng;
    gps_data.last_alt_m = gps_data.alt_m;
    gps_data.last_known_valid = true;
    gps_data.last_fix_ms = now_ms;
    gps_data.sentences_with_fix++;
  } else {
    gps_data.alt_valid = false;
    if (!gps_data.last_known_valid && lat != 0 && lon != 0) {
      gps_data.last_lat = static_cast<double>(lat) * 1e-7;
      gps_data.last_lng = static_cast<double>(lon) * 1e-7;
      gps_data.last_alt_m = static_cast<double>(h_msl) / 1000.0;
      gps_data.last_known_valid = true;
    }
  }

  gps_data.sats = num_sv;
  gps_data.sats_valid = true;

  // Accuracy fields (always available)
  gps_data.h_acc_mm = ubxU32(payload + 40);
  gps_data.v_acc_mm = ubxU32(payload + 44);

  // Velocity fields
  gps_data.vel_n_mm_s = ubxI32(payload + 48);
  gps_data.vel_e_mm_s = ubxI32(payload + 52);
  gps_data.vel_d_mm_s = ubxI32(payload + 56);
  gps_data.g_speed_mm_s = ubxI32(payload + 60);
  gps_data.head_mot = ubxI32(payload + 64);
  gps_data.s_acc_mm_s = ubxU32(payload + 68);
  gps_data.head_acc = ubxU32(payload + 72);
  gps_data.vel_valid = fix_ok;

  // PDOP
  gps_data.p_dop = ubxU16(payload + 76);
  if (fix_ok && gps_data.p_dop > 0 && gps_data.p_dop < 2000) {
    gps_data.hdop = static_cast<double>(gps_data.p_dop) / 100.0;
    gps_data.hdop_valid = true;
  }
}

static void ubxHandleNavDop(const uint8_t* payload, uint16_t length) {
  if (length < 18) {
    return;
  }
  gps_data.ubx_dop_count++;
  uint16_t h_dop = ubxU16(payload + 10);
  gps_data.hdop = static_cast<double>(h_dop) / 100.0;
  gps_data.hdop_valid = true;
}

static void ubxHandleFrame(uint8_t cls, uint8_t id,
                           const uint8_t* payload, uint16_t length,
                           uint32_t now_ms) {
  if (cls == 0x01 && id == 0x07) {
    ubxHandleNavPvt(payload, length, now_ms);
  } else if (cls == 0x01 && id == 0x04) {
    ubxHandleNavDop(payload, length);
  }
}

static void ubxParseByte(uint8_t value, uint32_t now_ms) {
  switch (ubx.state) {
    case UbxState::SYNC1:
      if (value == 0xB5) {
        ubx.state = UbxState::SYNC2;
      }
      break;
    case UbxState::SYNC2:
      if (value == 0x62) {
        ubx.state = UbxState::CLASS;
        ubx.ck_a_calc = 0;
        ubx.ck_b_calc = 0;
      } else {
        ubx.state = UbxState::SYNC1;
      }
      break;
    case UbxState::CLASS:
      ubx.cls = value;
      ubxChecksumAdd(value);
      ubx.state = UbxState::ID;
      break;
    case UbxState::ID:
      ubx.id = value;
      ubxChecksumAdd(value);
      ubx.state = UbxState::LEN1;
      break;
    case UbxState::LEN1:
      ubx.length = value;
      ubxChecksumAdd(value);
      ubx.state = UbxState::LEN2;
      break;
    case UbxState::LEN2:
      ubx.length |= static_cast<uint16_t>(value) << 8;
      ubxChecksumAdd(value);
      ubx.index = 0;
      ubx.state = ubx.length == 0 ? UbxState::CK_A : UbxState::PAYLOAD;
      break;
    case UbxState::PAYLOAD:
      if (ubx.index < sizeof(ubx.payload)) {
        ubx.payload[ubx.index] = value;
      }
      ubx.index++;
      ubxChecksumAdd(value);
      if (ubx.index >= ubx.length) {
        ubx.state = UbxState::CK_A;
      }
      break;
    case UbxState::CK_A:
      ubx.ck_a = value;
      ubx.state = UbxState::CK_B;
      break;
    case UbxState::CK_B:
      ubx.ck_b = value;
      if (ubx.ck_a == ubx.ck_a_calc && ubx.ck_b == ubx.ck_b_calc) {
        ubxHandleFrame(ubx.cls, ubx.id, ubx.payload, ubx.length, now_ms);
      } else {
        gps_data.failed_checksum++;
      }
      ubxReset();
      break;
  }
}

static void buildSpsStatusFlags(uint32_t status, char* buffer, size_t length) {
  if (length == 0) {
    return;
  }
  buffer[0] = '\0';
  bool first = true;
  auto append = [&](const char* label) {
    size_t used = strlen(buffer);
    if (used + 1 >= length) {
      return;
    }
    if (!first) {
      strncat(buffer, ",", length - used - 1);
      used = strlen(buffer);
      if (used + 1 >= length) {
        return;
      }
    }
    strncat(buffer, label, length - used - 1);
    first = false;
  };

  if (status & SPS_STATUS_SPEED_BIT) {
    append("SPEED");
  }
  if (status & SPS_STATUS_LASER_BIT) {
    append("LASER");
  }
  if (status & SPS_STATUS_FAN_BIT) {
    append("FAN");
  }
  if (buffer[0] == '\0') {
    strncpy(buffer, "OK", length);
    buffer[length - 1] = '\0';
  }
}

static void i2cBusRecoveryPins(int sda_pin, int scl_pin) {
  pinMode(sda_pin, INPUT_PULLUP);
  pinMode(scl_pin, INPUT_PULLUP);
  delayMicroseconds(5);

  pinMode(scl_pin, OUTPUT_OPEN_DRAIN);
  pinMode(sda_pin, OUTPUT_OPEN_DRAIN);
  digitalWrite(sda_pin, HIGH);
  for (uint8_t i = 0; i < I2C_CLEAR_PULSES; i++) {
    digitalWrite(scl_pin, LOW);
    delayMicroseconds(5);
    digitalWrite(scl_pin, HIGH);
    delayMicroseconds(5);
  }
  digitalWrite(sda_pin, LOW);
  delayMicroseconds(5);
  digitalWrite(scl_pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(sda_pin, HIGH);
  delayMicroseconds(5);
  pinMode(sda_pin, INPUT_PULLUP);
  pinMode(scl_pin, INPUT_PULLUP);
}

static void configureI2cBus() {
  Wire.setClock(I2C_FREQ_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
}

static void restartI2cBus() {
  i2cBusRecoveryPins(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.end();
  delay(5);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  configureI2cBus();
}

static bool i2cReadBytes(uint8_t address, uint8_t reg, uint8_t* buffer,
                         size_t length) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  uint8_t read_len = Wire.requestFrom(address, static_cast<uint8_t>(length));
  if (read_len != length) {
    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }
  for (size_t i = 0; i < length; i++) {
    buffer[i] = Wire.read();
  }
  return true;
}

static bool i2cPing(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

static bool i2cWriteByte(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static bool detectHmc5883l() {
  uint8_t id[3] = {0};
  if (!i2cPing(MAG_ADDR_HMC5883L)) {
    return false;
  }
  if (!i2cReadBytes(MAG_ADDR_HMC5883L, 0x0A, id, 3)) {
    return false;
  }
  return id[0] == 'H' && id[1] == '4' && id[2] == '3';
}

static bool detectQmc5883l() {
  uint8_t id = 0;
  if (!i2cPing(MAG_ADDR_QMC5883L)) {
    return false;
  }
  return i2cReadBytes(MAG_ADDR_QMC5883L, 0x0D, &id, 1);
}

static void initMagnetometer() {
  mag_data.ok = false;
  mag_data.type = MagType::NONE;
  strncpy(mag_data.type_label, "NONE", sizeof(mag_data.type_label));
  mag_data.type_label[sizeof(mag_data.type_label) - 1] = '\0';

  if (detectHmc5883l()) {
    if (i2cWriteByte(MAG_ADDR_HMC5883L, 0x00, 0x70) &&
        i2cWriteByte(MAG_ADDR_HMC5883L, 0x01, 0xA0) &&
        i2cWriteByte(MAG_ADDR_HMC5883L, 0x02, 0x00)) {
      mag_data.type = MagType::HMC5883L;
      strncpy(mag_data.type_label, "HMC5883L",
              sizeof(mag_data.type_label));
      mag_data.type_label[sizeof(mag_data.type_label) - 1] = '\0';
    }
    return;
  }

  if (detectQmc5883l()) {
    if (i2cWriteByte(MAG_ADDR_QMC5883L, 0x0B, 0x01) &&
        i2cWriteByte(MAG_ADDR_QMC5883L, 0x09, 0x1D) &&
        i2cWriteByte(MAG_ADDR_QMC5883L, 0x0A, 0x00)) {
      mag_data.type = MagType::QMC5883L;
      strncpy(mag_data.type_label, "QMC5883L",
              sizeof(mag_data.type_label));
      mag_data.type_label[sizeof(mag_data.type_label) - 1] = '\0';
    }
    return;
  }
}

static void magEnterSleep() {
  if (mag_data.type == MagType::HMC5883L) {
    i2cWriteByte(MAG_ADDR_HMC5883L, 0x02, 0x02);
  } else if (mag_data.type == MagType::QMC5883L) {
    i2cWriteByte(MAG_ADDR_QMC5883L, 0x09, 0x00);
  }
}

static void quatToEuler(float qr, float qi, float qj, float qk,
                        float* yaw, float* pitch, float* roll) {
  float ys = 2.0f * (qr * qk + qi * qj);
  float yc = 1.0f - 2.0f * (qj * qj + qk * qk);
  float ps = 2.0f * (qr * qj - qk * qi);
  float rs = 2.0f * (qr * qi + qj * qk);
  float rc = 1.0f - 2.0f * (qi * qi + qj * qj);
  ps = fmaxf(-1.0f, fminf(1.0f, ps));
  *yaw = atan2f(ys, yc) * 180.0f / 3.1415926f;
  *pitch = asinf(ps) * 180.0f / 3.1415926f;
  *roll = atan2f(rs, rc) * 180.0f / 3.1415926f;
}

static void updateMagnetometer() {
  if (!mag_enabled) {
    if (!mag_sleeping) {
      magEnterSleep();
      mag_sleeping = true;
    }
    mag_data.ok = false;
    return;
  }
  if (mag_sleeping) {
    initMagnetometer();
    mag_sleeping = false;
  }
  if (mag_data.type == MagType::NONE) {
    return;
  }
  uint32_t now = millis();
  if (now - mag_data.last_read_ms < MAG_READ_INTERVAL_MS) {
    return;
  }
  mag_data.last_read_ms = now;

  uint8_t raw[6] = {0};
  int16_t x = 0;
  int16_t y = 0;
  int16_t z = 0;
  bool read_ok = false;

  if (mag_data.type == MagType::HMC5883L) {
    read_ok = i2cReadBytes(MAG_ADDR_HMC5883L, 0x03, raw, 6);
    if (read_ok) {
      x = static_cast<int16_t>((raw[0] << 8) | raw[1]);
      z = static_cast<int16_t>((raw[2] << 8) | raw[3]);
      y = static_cast<int16_t>((raw[4] << 8) | raw[5]);
    }
  } else if (mag_data.type == MagType::QMC5883L) {
    read_ok = i2cReadBytes(MAG_ADDR_QMC5883L, 0x00, raw, 6);
    if (read_ok) {
      x = static_cast<int16_t>((raw[1] << 8) | raw[0]);
      y = static_cast<int16_t>((raw[3] << 8) | raw[2]);
      z = static_cast<int16_t>((raw[5] << 8) | raw[4]);
    }
  }

  if (!read_ok) {
    mag_data.ok = false;
    return;
  }

  mag_data.x = static_cast<float>(x);
  mag_data.y = static_cast<float>(y);
  mag_data.z = static_cast<float>(z);
  float heading = atan2f(mag_data.y, mag_data.x) * 180.0f / 3.1415926f;
  if (heading < 0.0f) {
    heading += 360.0f;
  }
  mag_data.heading_deg = heading;
  mag_data.ok = true;
}

static bool enableBnoReports() {
  // Stabilized / ARVR / Game rotation vector preference, fallback to raw.
  struct Candidate {
    sh2_SensorId_t id;
    const char* label;
  };
  const Candidate candidates[] = {
#if defined(SH2_ARVR_STABILIZED_RV)
      {SH2_ARVR_STABILIZED_RV, "ARVR"},
#endif
#if defined(SH2_STABILIZED_ROTATION_VECTOR)
      {SH2_STABILIZED_ROTATION_VECTOR, "STAB"},
#endif
      {SH2_GAME_ROTATION_VECTOR, "GAME"},
      {SH2_ROTATION_VECTOR, "ROT"},
  };
  for (const auto& candidate : candidates) {
    if (bno08x.enableReport(candidate.id, BNO_REPORT_US_ROT)) {
      bno_rot_sensor_id = candidate.id;
      return true;
    }
  }
  return false;
}

static void bnoHardReset(uint32_t low_ms, uint32_t boot_ms) {
  pinMode(PIN_BNO_WAKE, OUTPUT);
  digitalWrite(PIN_BNO_WAKE, HIGH);  // PS0 high during reset -> SPI mode
  delay(2);
  pinMode(PIN_BNO_RST, OUTPUT);
  digitalWrite(PIN_BNO_RST, HIGH);
  delay(5);
  digitalWrite(PIN_BNO_RST, LOW);
  delay(low_ms);
  digitalWrite(PIN_BNO_RST, HIGH);
  delay(boot_ms);
  digitalWrite(PIN_BNO_WAKE, LOW);  // PS0 low = WAKE
}

static void logBnoSpiPins(const char* label) {
  Serial.printf("BNO SPI %s: INT=%s RST=%s CS=%s WAKE=%s\n",
                label,
                digitalRead(PIN_BNO_INT) ? "HIGH" : "LOW",
                digitalRead(PIN_BNO_RST) ? "HIGH" : "LOW",
                digitalRead(PIN_BNO_CS) ? "HIGH" : "LOW",
                digitalRead(PIN_BNO_WAKE) ? "HIGH" : "LOW");
}

static void bnoSpiProbe(const char* label) {
  if (!BNO_SPI_PROBE) {
    return;
  }
  const uint8_t tx[4] = {0xAA, 0x55, 0xF0, 0x0F};
  uint8_t rx[4] = {0};
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(PIN_BNO_CS, LOW);
  delayMicroseconds(2);
  for (size_t i = 0; i < sizeof(tx); i++) {
    rx[i] = SPI.transfer(tx[i]);
  }
  digitalWrite(PIN_BNO_CS, HIGH);
  SPI.endTransaction();
  Serial.printf("BNO SPI probe %s: RX=%02X %02X %02X %02X\n",
                label, rx[0], rx[1], rx[2], rx[3]);
}

static void bnoRecoverySequence() {
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_BNO_CS);
  bnoHardReset(BNO_RESET_LOW_MS, BNO_RESET_BOOT_MS);
}

static bool bnoDevOn() {
  return sh2_devOn() == SH2_OK;
}

static void configureBnoCalibration() {
  sh2_setCalConfig(BNO_CAL_MASK);
}

static bool initBno085() {
  bno_data.ok = false;
  strncpy(bno_data.error_text, "NOT FOUND", sizeof(bno_data.error_text));
  bno_data.error_text[sizeof(bno_data.error_text) - 1] = '\0';
  bno_present = false;
  bno_reports_enabled = false;
  last_bno_read_ms = 0;
  bno_init_ms = 0;
  bno_last_event_ms = 0;
  bno_last_rot_ms = 0;
  bno_last_log_ms = 0;
  bno_event_count = 0;
  bno_rot_count = 0;
  bno_last_sensor_id = 0;
  bno_i2c_addr = 0;
  bnoRecoverySequence();

  bool started = false;
  for (uint8_t attempt = 0; attempt < 3 && !started; attempt++) {
    delay(50);
    started = bno08x.begin_SPI(PIN_BNO_CS, PIN_BNO_INT, &SPI);
    if (started) {
      break;
    }
    delay(100);
    bnoRecoverySequence();
  }
  if (!started) {
    return false;
  }
  bno_present = true;
  if (!bnoDevOn()) {
    strncpy(bno_data.error_text, "DEVON FAIL", sizeof(bno_data.error_text));
    bno_data.error_text[sizeof(bno_data.error_text) - 1] = '\0';
    return false;
  }
  configureBnoCalibration();
  delay(100);  // Let BNO stabilize before enabling reports

  // Check for reset and clear it
  bno08x.wasReset();

  bno_reports_enabled = enableBnoReports();
  if (!bno_reports_enabled) {
    strncpy(bno_data.error_text, "REPORT FAIL", sizeof(bno_data.error_text));
    bno_data.error_text[sizeof(bno_data.error_text) - 1] = '\0';
    return false;
  }

  delay(100);  // Let reports start flowing
  bno_data.error_text[0] = '\0';
  last_bno_read_ms = millis();
  bno_init_ms = last_bno_read_ms;
  bno_next_probe_ms = 0;
  return true;
}

static void initBno085Blocking() {
  uint32_t attempt = 0;
  while (!initBno085()) {
    delay(300);
  }
}

static void updateBno085() {
  if (!bno_present || !bno_enabled || !bno_reports_enabled) {
    return;
  }
  uint32_t now = millis();

  // Reset check
  if (bno08x.wasReset()) {
    bnoDevOn();
    configureBnoCalibration();
    bno_reports_enabled = enableBnoReports();
    if (!bno_reports_enabled) {
      strncpy(bno_data.error_text, "REPORT FAIL", sizeof(bno_data.error_text));
      bno_data.ok = false;
      return;
    }
  }

  // Egyetlen getSensorEvent - csak rotation vector jön
  if (bno08x.getSensorEvent(&bno_value)) {
    bno_event_count++;
    bno_last_event_ms = now;
    bno_last_sensor_id = bno_value.sensorId;
    if (bno_value.sensorId == bno_rot_sensor_id) {
      quatToEuler(bno_value.un.rotationVector.real,
                  bno_value.un.rotationVector.i,
                  bno_value.un.rotationVector.j,
                  bno_value.un.rotationVector.k,
                  &bno_data.yaw_deg,
                  &bno_data.pitch_deg,
                  &bno_data.roll_deg);
      bno_data.rot_accuracy = bno_value.status;
      bno_data.ok = true;
      bno_data.last_read_ms = now;
      last_bno_read_ms = bno_data.last_read_ms;
      bno_last_rot_ms = now;
      bno_rot_count++;
      bno_data.error_text[0] = '\0';
    } else {
      last_bno_read_ms = now;  // Keep link alive even if we get non-rot events.
    }
  }
  if (BNO_DEBUG_LOG && (now - bno_last_log_ms) >= 1000) {
    bno_last_log_ms = now;
    uint32_t event_age = bno_last_event_ms ? (now - bno_last_event_ms) : 0;
    uint32_t rot_age = bno_last_rot_ms ? (now - bno_last_rot_ms) : 0;
    Serial.printf("BNO evt=%lu rot=%lu last_id=%u INT=%s evt_age=%lu rot_age=%lu\n",
                  static_cast<unsigned long>(bno_event_count),
                  static_cast<unsigned long>(bno_rot_count),
                  static_cast<unsigned>(bno_last_sensor_id),
                  digitalRead(PIN_BNO_INT) ? "HIGH" : "LOW",
                  static_cast<unsigned long>(event_age),
                  static_cast<unsigned long>(rot_age));
  }
}

static void updateBnoRecovery() {
  if (!bno_enabled || bno_sleeping) {
    return;
  }
  uint32_t now = millis();
  if (bno_next_probe_ms != 0 && now < bno_next_probe_ms) {
    return;
  }
  if (bno_init_ms != 0 && (now - bno_init_ms) < BNO_STARTUP_GRACE_MS) {
    return;
  }
  bool should_recover = false;
  if (!bno_present || !bno_reports_enabled) {
    should_recover = true;
  } else if (last_bno_read_ms == 0) {
    should_recover = now > BNO_STALL_MS;
  } else if (now - last_bno_read_ms > BNO_STALL_MS) {
    should_recover = true;
  }
  if (!should_recover) {
    return;
  }
  bno_next_probe_ms = now + BNO_RECOVER_BACKOFF_MS;
  bno_recovery_count++;
  while (!initBno085()) {
    bno_recovery_count++;
    delay(200);
  }
}

static void initScd40() {
  scd_data.ok = false;
  strncpy(scd_data.error_text, "NOT FOUND", sizeof(scd_data.error_text));
  scd_data.error_text[sizeof(scd_data.error_text) - 1] = '\0';
  scd_present = false;
  last_scd_read_ms = 0;
  scd4x.begin(Wire);
  uint16_t err = scd4x.stopPeriodicMeasurement();
  if (err != 0) {
    char error_message[64];
    errorToString(err, error_message, sizeof(error_message));
    strncpy(scd_data.error_text, error_message, sizeof(scd_data.error_text));
    scd_data.error_text[sizeof(scd_data.error_text) - 1] = '\0';
  }
  err = scd4x.startPeriodicMeasurement();
  if (err != 0) {
    char error_message[64];
    errorToString(err, error_message, sizeof(error_message));
    strncpy(scd_data.error_text, error_message, sizeof(scd_data.error_text));
    scd_data.error_text[sizeof(scd_data.error_text) - 1] = '\0';
    return;
  }
  scd_present = true;
  scd_data.error_text[0] = '\0';
}

static void updateScd40() {
  if (!scd_present) {
    return;
  }
  if (!scd_enabled) {
    if (!scd_sleeping) {
      scd4x.stopPeriodicMeasurement();
      scd_sleeping = true;
    }
    scd_data.ok = false;
    strncpy(scd_data.error_text, "SLEEP", sizeof(scd_data.error_text));
    scd_data.error_text[sizeof(scd_data.error_text) - 1] = '\0';
    return;
  }
  if (scd_sleeping) {
    scd4x.startPeriodicMeasurement();
    scd_sleeping = false;
    last_scd_read_ms = 0;
  }
  uint32_t now = millis();
  if (now - last_scd_read_ms < SCD_READ_INTERVAL_MS) {
    return;
  }
  bool data_ready = false;
  uint16_t ready_err = scd4x.getDataReadyFlag(data_ready);
  if (ready_err != 0) {
    scd_data.ok = false;
    char error_message[64];
    errorToString(ready_err, error_message, sizeof(error_message));
    strncpy(scd_data.error_text, error_message, sizeof(scd_data.error_text));
    scd_data.error_text[sizeof(scd_data.error_text) - 1] = '\0';
    return;
  }
  if (!data_ready) {
    return;
  }
  last_scd_read_ms = now;
  uint16_t co2 = 0;
  float temp = 0.0f;
  float rh = 0.0f;
  uint16_t err = scd4x.readMeasurement(co2, temp, rh);
  if (err != 0 || co2 == 0) {
    scd_data.ok = false;
    if (err != 0) {
      char error_message[64];
      errorToString(err, error_message, sizeof(error_message));
      strncpy(scd_data.error_text, error_message, sizeof(scd_data.error_text));
      scd_data.error_text[sizeof(scd_data.error_text) - 1] = '\0';
    } else {
      strncpy(scd_data.error_text, "NO DATA", sizeof(scd_data.error_text));
      scd_data.error_text[sizeof(scd_data.error_text) - 1] = '\0';
    }
    return;
  }
  scd_data.co2_ppm = static_cast<float>(co2);
  scd_data.temp_c = temp;
  scd_data.rh = rh;
  scd_data.ok = true;
  scd_data.error_text[0] = '\0';
  scd_data.last_read_ms = now;
}

void IRAM_ATTR bmp585InterruptHandler() {
  bmp_data_ready = true;
}

static float bmpAltitudeFromRef(float pressure_hpa, float ref_hpa) {
  if (pressure_hpa <= 0.0f || ref_hpa <= 0.0f) {
    return 0.0f;
  }
  return 44330.0f * (1.0f - powf(pressure_hpa / ref_hpa, 0.1903f));
}

static float bmpQnhFromGps(float pressure_hpa, float gps_alt_m) {
  if (pressure_hpa <= 0.0f) {
    return 0.0f;
  }
  float ratio = 1.0f - (gps_alt_m / 44330.0f);
  if (ratio <= 0.0f) {
    return 0.0f;
  }
  return pressure_hpa / powf(ratio, 5.255f);
}

static const char* bmpAltModeString() {
  return bmp_alt_mode == BmpAltMode::REL ? "REL" : "QNH";
}

static void bmpQnhPush(float value) {
  if (bmp_qnh_count < BMP_QNH_AVG_WINDOW) {
    bmp_qnh_samples[bmp_qnh_index] = value;
    bmp_qnh_sum += value;
    bmp_qnh_count++;
  } else {
    bmp_qnh_sum -= bmp_qnh_samples[bmp_qnh_index];
    bmp_qnh_samples[bmp_qnh_index] = value;
    bmp_qnh_sum += value;
  }
  bmp_qnh_index = (bmp_qnh_index + 1) % BMP_QNH_AVG_WINDOW;
}

static float bmpQnhAverage() {
  if (bmp_qnh_count == 0) {
    return 0.0f;
  }
  return bmp_qnh_sum / static_cast<float>(bmp_qnh_count);
}

static bool bmp_isr_attached = false;
static void initBmp585() {
  if (bmp_isr_attached) {
    detachInterrupt(digitalPinToInterrupt(PIN_BMP585_INT));
    bmp_isr_attached = false;
  }
  pinMode(PIN_BMP585_INT, INPUT);
  bmp_data = Bmp585Data();
  bmp_data.ok = false;
  bmp_data.alt_valid = false;
  strncpy(bmp_data.error_text, "NOT FOUND", sizeof(bmp_data.error_text));
  bmp_data.error_text[sizeof(bmp_data.error_text) - 1] = '\0';
  bmp_present = false;
  bmp_sleeping = false;
  bmp_data_ready = false;
  last_bmp_poll_ms = 0;
  bmp_alt_mode = BmpAltMode::QNH;
  bmp_ref_hpa = BMP_SEA_LEVEL_HPA;
  bmp_rel_ref_hpa = 0.0f;
  bmp_rel_ref_valid = false;
  bmp_qnh_index = 0;
  bmp_qnh_count = 0;
  bmp_qnh_sum = 0.0f;
  bmp_qnh_last_ms = 0;
  if (bmp585.begin(BMP5XX_DEFAULT_ADDRESS, &Wire) ||
      bmp585.begin(BMP5XX_ALTERNATIVE_ADDRESS, &Wire)) {
    bmp_present = true;
    bmp585.setTemperatureOversampling(BMP5XX_OVERSAMPLING_8X);   // 8x oversampling
    bmp585.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);     // 16x oversampling (precíz)
    bmp585.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);         // IIR szűrő zajcsökkentéshez
    bmp585.setOutputDataRate(BMP5XX_ODR_05_HZ);                  // 0.5Hz, de oversampled
    bmp585.setPowerMode(BMP5XX_POWERMODE_NORMAL);
    bmp585.enablePressure(true);
    if (bmp585.configureInterrupt(BMP5XX_INTERRUPT_PULSED,
                                  BMP5XX_INTERRUPT_ACTIVE_HIGH,
                                  BMP5XX_INTERRUPT_PUSH_PULL,
                                  BMP5XX_INTERRUPT_DATA_READY, true)) {
      attachInterrupt(digitalPinToInterrupt(PIN_BMP585_INT),
                      bmp585InterruptHandler, RISING);
      bmp_isr_attached = true;
      bmp_data.error_text[0] = '\0';
    } else {
      strncpy(bmp_data.error_text, "INT CFG FAIL",
              sizeof(bmp_data.error_text));
      bmp_data.error_text[sizeof(bmp_data.error_text) - 1] = '\0';
    }
  }
}

static void updateBmp585() {
  if (!bmp_present) {
    return;
  }
  if (!bmp_enabled) {
    if (!bmp_sleeping) {
      bmp585.setPowerMode(BMP5XX_POWERMODE_STANDBY);
      bmp_sleeping = true;
    }
    bmp_data.ok = false;
    bmp_data.alt_valid = false;
    strncpy(bmp_data.error_text, "SLEEP", sizeof(bmp_data.error_text));
    bmp_data.error_text[sizeof(bmp_data.error_text) - 1] = '\0';
    return;
  }
  if (bmp_sleeping) {
    bmp585.setPowerMode(BMP5XX_POWERMODE_NORMAL);
    bmp_sleeping = false;
    bmp_data_ready = false;
    last_bmp_poll_ms = millis();
  }
  uint32_t now = millis();
  if (!bmp_data_ready) {
    if (now - last_bmp_poll_ms < BMP_POLL_INTERVAL_MS) {
      return;
    }
    last_bmp_poll_ms = now;
    if (!bmp585.dataReady()) {
      return;
    }
  }
  bmp_data_ready = false;
  if (bmp585.performReading()) {
    bmp_data.temp_c = bmp585.temperature;
    bmp_data.pressure_hpa = bmp585.pressure;
    bmp_data.ok = true;
    if (gps_data.fix && gps_data.alt_valid &&
        (now - bmp_qnh_last_ms) >= BMP_QNH_SAMPLE_MS) {
      float qnh = bmpQnhFromGps(bmp_data.pressure_hpa, gps_data.alt_m);
      if (qnh > 800.0f && qnh < 1100.0f) {
        bmpQnhPush(qnh);
        bmp_qnh_last_ms = now;
      }
    }
    float ref_hpa = bmp_alt_mode == BmpAltMode::REL
                        ? bmp_rel_ref_hpa
                        : bmp_ref_hpa;
    bool ref_valid = bmp_alt_mode == BmpAltMode::REL
                         ? bmp_rel_ref_valid
                         : (bmp_ref_hpa > 0.0f);
    bmp_data.alt_valid = bmp_data.ok && ref_valid;
    if (bmp_data.alt_valid) {
      bmp_data.altitude_m = bmpAltitudeFromRef(bmp_data.pressure_hpa, ref_hpa);
    } else {
      bmp_data.altitude_m = 0.0f;
    }
    bmp_data.error_text[0] = '\0';
    bmp_data.last_read_ms = now;
  } else {
    bmp_data.ok = false;
    bmp_data.alt_valid = false;
    strncpy(bmp_data.error_text, "READ FAIL", sizeof(bmp_data.error_text));
    bmp_data.error_text[sizeof(bmp_data.error_text) - 1] = '\0';
  }
}

static void noteHttpActivity() {
  last_http_ms = millis();
}

static void onWsEvent(AsyncWebSocket* server_ptr,
                      AsyncWebSocketClient* client,
                      AwsEventType type,
                      void* arg,
                      uint8_t* data,
                      size_t len) {
  (void)server_ptr;
  (void)client;
  (void)arg;
  (void)data;
  (void)len;
  if (type == WS_EVT_CONNECT || type == WS_EVT_DATA) {
    noteHttpActivity();
  }
  if (type == WS_EVT_CONNECT && client) {
    client->setCloseClientOnQueueFull(true);
    client->keepAlivePeriod(10);
    last_ws_fast_ms = 0;
    last_ws_gps_ms = 0;
  }
  if (type == WS_EVT_DATA && client && arg && data && len > 0) {
    AwsFrameInfo* info = static_cast<AwsFrameInfo*>(arg);
    if (info->final && info->index == 0 && info->len == len &&
        info->opcode == WS_TEXT && len <= 16) {
      // Ignore small text messages (legacy hello/ping).
    }
  }
}

static uint8_t pulseLevel() {
  float phase = (millis() % LED_PULSE_PERIOD_MS) /
                static_cast<float>(LED_PULSE_PERIOD_MS);
  float level = 0.5f + 0.5f * sinf(phase * 6.2831853f);
  return static_cast<uint8_t>(level * 255.0f);
}

static uint32_t scaledColor(uint8_t r, uint8_t g, uint8_t b, uint8_t level) {
  uint8_t rr = static_cast<uint8_t>((r * level) / 255);
  uint8_t gg = static_cast<uint8_t>((g * level) / 255);
  uint8_t bb = static_cast<uint8_t>((b * level) / 255);
  return status_led.Color(rr, gg, bb);
}

static void updateStatusLed() {
  uint8_t level = pulseLevel();
  bool recent_http = (millis() - last_http_ms) < LED_ACTIVITY_WINDOW_MS;

  uint32_t color = 0;
  if (WiFi.status() == WL_CONNECTED) {
    if (recent_http) {
      color = scaledColor(120, 160, 255, level / 2);
    } else {
      color = scaledColor(60, 255, 120, level);
    }
  } else {
    color = scaledColor(255, 60, 60, level / 2);
  }

  status_led.setPixelColor(0, color);
  status_led.show();
}

static const char* wifiModeString() {
  wifi_mode_t mode = WiFi.getMode();
  switch (mode) {
    case WIFI_STA:
      return "STA";
    case WIFI_AP:
      return "AP";
    case WIFI_AP_STA:
      return "AP+STA";
    default:
      return "OFF";
  }
}

static float wifiPowerDbm(wifi_power_t power) {
  return static_cast<int8_t>(power) * 0.25f;
}

static bool wifiPowerFromValue(const String& value, wifi_power_t* out_power) {
  String v = value;
  v.trim();
  v.toLowerCase();
  if (v == "max") {
    *out_power = WIFI_POWER_19_5dBm;
    return true;
  }
  if (v == "min") {
    *out_power = WIFI_POWER_2dBm;
    return true;
  }
  float dbm = v.toFloat();
  if (!isfinite(dbm)) {
    return false;
  }
  dbm = fmaxf(2.0f, fminf(19.5f, dbm));
  int8_t raw = static_cast<int8_t>(lroundf(dbm * 4.0f));
  *out_power = static_cast<wifi_power_t>(raw);
  return true;
}

static bool parseOnOff(AsyncWebServerRequest* request, bool* out_value) {
  if (!request->hasParam("value")) {
    return false;
  }
  String value = request->getParam("value")->value();
  value.toLowerCase();
  if (value == "on" || value == "1" || value == "true") {
    *out_value = true;
    return true;
  }
  if (value == "off" || value == "0" || value == "false") {
    *out_value = false;
    return true;
  }
  return false;
}

static void updateWifiPower() {
  wl_status_t status = WiFi.status();
  if (status != WL_CONNECTED) {
    wifi_tx_applied = false;
    return;
  }
  if (!wifi_tx_applied) {
    esp_err_t err =
        esp_wifi_set_max_tx_power(static_cast<int8_t>(wifi_tx_target));
    wifi_tx_apply_err = static_cast<int>(err);
    wifi_tx_applied = (err == ESP_OK);
  }
}

static void gpsFlushRx() {
  while (Serial1.available() > 0) {
    Serial1.read();
  }
}

static void initGps() {
  Serial1.begin(UART1_BAUD, SERIAL_8N1, PIN_UART1_RX, PIN_UART1_TX);
  gpsFlushRx();
  ubxReset();
  gps_data = GpsData();
  gps_raw_head = 0;
  gps_raw_len = 0;
}

static int16_t scaleToInt16(float value, float scale) {
  if (!isfinite(value)) {
    return 0;
  }
  float scaled = value * scale;
  if (scaled > 32767.0f) {
    scaled = 32767.0f;
  } else if (scaled < -32768.0f) {
    scaled = -32768.0f;
  }
  return static_cast<int16_t>(lroundf(scaled));
}

static uint16_t scaleToUint16(float value, float scale) {
  if (!isfinite(value) || value < 0.0f) {
    return 0;
  }
  float scaled = value * scale;
  if (scaled > 65535.0f) {
    scaled = 65535.0f;
  }
  return static_cast<uint16_t>(lroundf(scaled));
}

static int32_t scaleToInt32(double value, double scale) {
  if (!isfinite(value)) {
    return 0;
  }
  double scaled = value * scale;
  if (scaled > 2147483647.0) {
    scaled = 2147483647.0;
  } else if (scaled < -2147483648.0) {
    scaled = -2147483648.0;
  }
  return static_cast<int32_t>(llround(scaled));
}

static void writeInt16LE(uint8_t* buffer, int16_t value) {
  buffer[0] = static_cast<uint8_t>(value & 0xFF);
  buffer[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

static void writeUint16LE(uint8_t* buffer, uint16_t value) {
  buffer[0] = static_cast<uint8_t>(value & 0xFF);
  buffer[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

static void writeInt32LE(uint8_t* buffer, int32_t value) {
  buffer[0] = static_cast<uint8_t>(value & 0xFF);
  buffer[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  buffer[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  buffer[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

static size_t buildWsFastFrame(uint8_t* buffer, size_t length) {
  if (length < 16) {
    return 0;
  }
  const uint8_t type = 0x01;
  uint8_t flags = 0;
  if (bno_data.ok) {
    flags |= 0x01;
  }
  if (mag_data.ok) {
    flags |= 0x02;
  }
  buffer[0] = type;
  buffer[1] = flags;
  writeInt16LE(buffer + 2, scaleToInt16(bno_data.yaw_deg, 10.0f));
  writeInt16LE(buffer + 4, scaleToInt16(bno_data.pitch_deg, 10.0f));
  writeInt16LE(buffer + 6, scaleToInt16(bno_data.roll_deg, 10.0f));
  writeInt16LE(buffer + 8, scaleToInt16(mag_data.x, 10.0f));
  writeInt16LE(buffer + 10, scaleToInt16(mag_data.y, 10.0f));
  writeInt16LE(buffer + 12, scaleToInt16(mag_data.z, 10.0f));
  writeUint16LE(buffer + 14, scaleToUint16(mag_data.heading_deg, 10.0f));
  return 16;
}

static size_t buildWsGpsFrame(uint8_t* buffer, size_t length) {
  if (length < 18) {
    return 0;
  }
  const uint8_t type = 0x02;
  uint8_t flags = 0;
  if (gps_data.fix) {
    flags |= 0x01;
  }
  if (gps_data.latlng_valid) {
    flags |= 0x02;
  }
  if (gps_data.alt_valid) {
    flags |= 0x04;
  }
  if (gps_data.hdop_valid) {
    flags |= 0x08;
  }
  if (gps_data.sats_valid) {
    flags |= 0x10;
  }
  buffer[0] = type;
  buffer[1] = flags;
  int32_t lat = gps_data.latlng_valid
                    ? scaleToInt32(gps_data.lat, 1e7)
                    : 0;
  int32_t lon = gps_data.latlng_valid
                    ? scaleToInt32(gps_data.lng, 1e7)
                    : 0;
  int32_t alt_cm = gps_data.alt_valid
                       ? scaleToInt32(gps_data.alt_m, 100.0)
                       : 0;
  writeInt32LE(buffer + 2, lat);
  writeInt32LE(buffer + 6, lon);
  writeInt32LE(buffer + 10, alt_cm);
  buffer[14] = static_cast<uint8_t>(
      gps_data.sats_valid ? gps_data.sats : 0);
  writeUint16LE(buffer + 15,
                static_cast<uint16_t>(gps_data.hdop_valid
                                           ? scaleToUint16(gps_data.hdop, 10.0f)
                                           : 0));
  buffer[17] = gps_data.fix_type;
  return 18;
}

static String buildTelemetryJson(bool include_raw) {
  Sps30Data sps_snapshot;
  portENTER_CRITICAL(&sps_mux);
  sps_snapshot = sps_data;
  portEXIT_CRITICAL(&sps_mux);
  Bmp585Data bmp_snapshot = bmp_data;
  MagData mag_snapshot = mag_data;
  Bno085Data bno_snapshot = bno_data;
  Scd40Data scd_snapshot = scd_data;
  String json;
  json.reserve(include_raw ? 2048 : 1800);
  json += "{";
  json += "\"sps\":{";
  json += "\"valid\":";
  json += sps_snapshot.valid ? "true" : "false";
  json += ",\"status\":\"";
  json += spsStatusString(sps_snapshot);
  json += "\"";
  json += ",\"mc1p0\":";
  json += jsonFloat(sps_snapshot.mc1p0, sps_snapshot.valid, 1);
  json += ",\"mc2p5\":";
  json += jsonFloat(sps_snapshot.mc2p5, sps_snapshot.valid, 1);
  json += ",\"mc4p0\":";
  json += jsonFloat(sps_snapshot.mc4p0, sps_snapshot.valid, 1);
  json += ",\"mc10p0\":";
  json += jsonFloat(sps_snapshot.mc10p0, sps_snapshot.valid, 1);
  json += ",\"nc0p5\":";
  json += jsonFloat(sps_snapshot.nc0p5, sps_snapshot.valid, 1);
  json += ",\"nc1p0\":";
  json += jsonFloat(sps_snapshot.nc1p0, sps_snapshot.valid, 1);
  json += ",\"nc2p5\":";
  json += jsonFloat(sps_snapshot.nc2p5, sps_snapshot.valid, 1);
  json += ",\"nc4p0\":";
  json += jsonFloat(sps_snapshot.nc4p0, sps_snapshot.valid, 1);
  json += ",\"nc10p0\":";
  json += jsonFloat(sps_snapshot.nc10p0, sps_snapshot.valid, 1);
  json += ",\"typical_particle_size\":";
  json += jsonFloat(sps_snapshot.typical_particle_size, sps_snapshot.valid, 2);
  json += ",\"error_code\":";
  json += String(sps_snapshot.last_error);
  json += ",\"error_text\":\"";
  json += sps_snapshot.last_error_text;
  json += "\"";
  json += ",\"status_register\":";
  json += String(sps_snapshot.status_register);
  json += ",\"status_flags\":\"";
  json += sps_snapshot.status_flags;
  json += "\"";
  json += ",\"status_error_code\":";
  json += String(sps_snapshot.status_error);
  json += ",\"status_error_text\":\"";
  json += sps_snapshot.status_error_text;
  json += "\"";
  json += ",\"clean\":{";
  json += "\"last_ago_s\":";
  if (sps_snapshot.last_clean_ms == 0) {
    json += "null";
  } else {
    json += String((millis() - sps_snapshot.last_clean_ms) / 1000);
  }
  json += ",\"auto_interval_s\":";
  json += String(sps_snapshot.auto_clean_interval_s);
  json += ",\"error_code\":";
  json += String(sps_snapshot.last_clean_error);
  json += ",\"error_text\":\"";
  json += sps_snapshot.last_clean_error_text;
  json += "\"";
  json += "}";
  json += "},";
  json += "\"bmp585\":{";
  json += "\"ok\":";
  json += bmp_snapshot.ok ? "true" : "false";
  json += ",\"temp_c\":";
  json += jsonFloat(bmp_snapshot.temp_c, bmp_snapshot.ok, 2);
  json += ",\"pressure_hpa\":";
  json += jsonFloat(bmp_snapshot.pressure_hpa, bmp_snapshot.ok, 2);
  json += ",\"altitude_m\":";
  json += jsonFloat(bmp_snapshot.altitude_m, bmp_snapshot.alt_valid, 1);
  json += ",\"alt_mode\":\"";
  json += bmpAltModeString();
  json += "\"";
  json += ",\"ref_hpa\":";
  json += jsonFloat(bmp_ref_hpa, true, 1);
  json += ",\"rel_ref_hpa\":";
  json += jsonFloat(bmp_rel_ref_hpa, bmp_rel_ref_valid, 1);
  json += ",\"qnh_avg\":";
  json += jsonFloat(bmpQnhAverage(), bmp_qnh_count > 0, 1);
  json += ",\"qnh_samples\":";
  json += String(bmp_qnh_count);
  json += ",\"qnh_window\":";
  json += String(BMP_QNH_AVG_WINDOW);
  json += ",\"error_text\":\"";
  json += bmp_snapshot.error_text;
  json += "\"";
  json += "},";
  json += "\"mag\":{";
  json += "\"ok\":";
  json += mag_snapshot.ok ? "true" : "false";
  json += ",\"type\":\"";
  json += mag_snapshot.type_label;
  json += "\"";
  json += ",\"x\":";
  json += jsonFloat(mag_snapshot.x, mag_snapshot.ok, 1);
  json += ",\"y\":";
  json += jsonFloat(mag_snapshot.y, mag_snapshot.ok, 1);
  json += ",\"z\":";
  json += jsonFloat(mag_snapshot.z, mag_snapshot.ok, 1);
  json += ",\"heading\":";
  json += jsonFloat(mag_snapshot.heading_deg, mag_snapshot.ok, 1);
  json += "},";
  json += "\"bno085\":{";
  json += "\"ok\":";
  json += bno_snapshot.ok ? "true" : "false";
  json += ",\"yaw\":";
  json += jsonFloat(bno_snapshot.yaw_deg, bno_snapshot.ok, 1);
  json += ",\"pitch\":";
  json += jsonFloat(bno_snapshot.pitch_deg, bno_snapshot.ok, 1);
  json += ",\"roll\":";
  json += jsonFloat(bno_snapshot.roll_deg, bno_snapshot.ok, 1);
  json += ",\"error_text\":\"";
  json += bno_snapshot.error_text;
  json += "\"";
  json += "},";
  json += "\"scd40\":{";
  json += "\"ok\":";
  json += scd_snapshot.ok ? "true" : "false";
  json += ",\"co2_ppm\":";
  json += jsonFloat(scd_snapshot.co2_ppm, scd_snapshot.ok, 0);
  json += ",\"temp_c\":";
  json += jsonFloat(scd_snapshot.temp_c, scd_snapshot.ok, 2);
  json += ",\"rh\":";
  json += jsonFloat(scd_snapshot.rh, scd_snapshot.ok, 1);
  json += ",\"error_text\":\"";
  json += scd_snapshot.error_text;
  json += "\"";
  json += "},";
  json += "\"gps\":{";
  json += "\"fix\":";
  json += gps_data.fix ? "true" : "false";
  json += ",\"lat\":";
  json += jsonFloat(gps_data.lat, gps_data.latlng_valid, 6);
  json += ",\"lng\":";
  json += jsonFloat(gps_data.lng, gps_data.latlng_valid, 6);
  json += ",\"last_lat\":";
  json += jsonFloat(gps_data.last_lat, gps_data.last_known_valid, 6);
  json += ",\"last_lng\":";
  json += jsonFloat(gps_data.last_lng, gps_data.last_known_valid, 6);
  json += ",\"alt\":";
  json += jsonFloat(gps_data.alt_m, gps_data.alt_valid, 1);
  json += ",\"sats\":";
  json += jsonUint(gps_data.sats, gps_data.sats_valid);
  json += ",\"hdop\":";
  json += jsonFloat(gps_data.hdop, gps_data.hdop_valid, 1);
  json += ",\"fix_type\":";
  json += String(gps_data.fix_type);
  json += ",\"flags\":";
  json += String(gps_data.flags);
  json += ",\"age_ms\":";
  json += gps_data.latlng_valid ? String(gps_data.location_age_ms) : "null";
  json += ",\"time\":{";
  json += "\"valid\":";
  json += gps_data.time_valid ? "true" : "false";
  json += ",\"hour\":";
  json += gps_data.time_valid ? String(gps_data.hour) : "null";
  json += ",\"minute\":";
  json += gps_data.time_valid ? String(gps_data.minute) : "null";
  json += ",\"second\":";
  json += gps_data.time_valid ? String(gps_data.second) : "null";
  json += "}";
  json += ",\"date\":{";
  json += "\"valid\":";
  json += gps_data.date_valid ? "true" : "false";
  json += ",\"year\":";
  json += gps_data.date_valid ? String(gps_data.year) : "null";
  json += ",\"month\":";
  json += gps_data.date_valid ? String(gps_data.month) : "null";
  json += ",\"day\":";
  json += gps_data.date_valid ? String(gps_data.day) : "null";
  json += "}";
  json += ",\"rx\":{";
  json += "\"chars\":";
  json += String(gps_data.chars);
  json += ",\"fix\":";
  json += String(gps_data.sentences_with_fix);
  json += ",\"bad\":";
  json += String(gps_data.failed_checksum);
  json += "}";
  if (include_raw) {
    json += ",\"raw_len\":";
    json += String(gps_raw_len);
    json += ",\"raw_hex\":\"";
    json += gpsRawHex();
    json += "\"";
  }
  json += ",\"ubx\":{";
  json += "\"pvt\":";
  json += String(gps_data.ubx_pvt_count);
  json += ",\"dop\":";
  json += String(gps_data.ubx_dop_count);
  json += ",\"age_ms\":";
  if (gps_data.ubx_last_ms == 0) {
    json += "null";
  } else {
    json += String(millis() - gps_data.ubx_last_ms);
  }
  json += "}";
  // NAV-PVT extra fields
  json += ",\"vel\":{";
  json += "\"valid\":";
  json += gps_data.vel_valid ? "true" : "false";
  json += ",\"n\":";
  json += String(gps_data.vel_n_mm_s);
  json += ",\"e\":";
  json += String(gps_data.vel_e_mm_s);
  json += ",\"d\":";
  json += String(gps_data.vel_d_mm_s);
  json += ",\"gs\":";
  json += String(gps_data.g_speed_mm_s);
  json += ",\"head\":";
  json += String(gps_data.head_mot);
  json += ",\"s_acc\":";
  json += String(gps_data.s_acc_mm_s);
  json += ",\"head_acc\":";
  json += String(gps_data.head_acc);
  json += "}";
  json += ",\"acc\":{";
  json += "\"h\":";
  json += String(gps_data.h_acc_mm);
  json += ",\"v\":";
  json += String(gps_data.v_acc_mm);
  json += "}";
  json += ",\"pdop\":";
  json += String(gps_data.p_dop);
  json += "},";
  json += "\"system\":{";
  json += "\"uptime_s\":";
  json += String(millis() / 1000);
  json += ",\"mode\":\"";
  json += wifiModeString();
  json += "\"";
  json += ",\"ip\":\"";
  json += WiFi.localIP().toString();
  json += "\"";
  json += ",\"rssi\":";
  int rssi = 0;
  wifi_ap_record_t ap_info;
  esp_err_t rssi_err = esp_wifi_sta_get_ap_info(&ap_info);
  if (WiFi.status() == WL_CONNECTED) {
    rssi = WiFi.RSSI();
    json += String(rssi);
  } else {
    json += "null";
  }
  json += ",\"rssi_drv\":";
  if (rssi_err == ESP_OK) {
    json += String(ap_info.rssi);
  } else {
    json += "null";
  }
  json += ",\"rssi_err\":";
  json += String(static_cast<int>(rssi_err));
  int8_t tx_raw = 0;
  esp_err_t tx_err = esp_wifi_get_max_tx_power(&tx_raw);
  json += ",\"tx_dbm\":";
  json += String(wifiPowerDbm(WiFi.getTxPower()), 1);
  json += ",\"tx_raw\":";
  json += String(static_cast<int8_t>(WiFi.getTxPower()));
  json += ",\"tx_target_dbm\":";
  json += String(wifiPowerDbm(wifi_tx_target), 1);
  json += ",\"tx_drv_raw\":";
  if (tx_err == ESP_OK) {
    json += String(tx_raw);
  } else {
    json += "null";
  }
  json += ",\"tx_drv_dbm\":";
  if (tx_err == ESP_OK) {
    json += String(static_cast<float>(tx_raw) / 4.0f, 2);
  } else {
    json += "null";
  }
  json += ",\"tx_drv_err\":";
  json += String(static_cast<int>(tx_err));
  json += ",\"tx_set_err\":";
  json += String(wifi_tx_set_err);
  json += ",\"tx_apply_err\":";
  json += String(wifi_tx_apply_err);
  json += ",\"ps_err\":";
  json += String(wifi_ps_err);
  json += ",\"cpu_mhz\":";
  json += String(ESP.getCpuFreqMHz());
  json += ",\"heap_free\":";
  json += String(ESP.getFreeHeap());
  json += ",\"heap_min\":";
  json += String(ESP.getMinFreeHeap());
  json += ",\"heap_total\":";
  json += String(ESP.getHeapSize());
  json += ",\"ws_clients\":";
  json += String(ws.count());
  json += ",\"sensors\":{";
  json += "\"sps30\":";
  json += sps_enabled ? "true" : "false";
  json += ",\"gps\":";
  json += gps_enabled ? "true" : "false";
  json += ",\"bmp\":";
  json += bmp_enabled ? "true" : "false";
  json += ",\"mag\":";
  json += mag_enabled ? "true" : "false";
  json += ",\"bno\":";
  json += bno_enabled ? "true" : "false";
  json += ",\"scd\":";
  json += scd_enabled ? "true" : "false";
  json += "}";
  json += ",\"hw\":{";
  json += "\"sps\":";
  json += sps_snapshot.valid ? "true" : "false";
  json += ",\"sps_retry\":";
  json += String(sps_recovery_count);
  json += ",\"sps_cleaning\":";
  json += (sps_state == SpsState::CLEAN_STOP || sps_state == SpsState::CLEAN_START || sps_state == SpsState::CLEAN_WAIT) ? "true" : "false";
  json += ",\"bmp\":";
  json += bmp_present ? "true" : "false";
  json += ",\"bno\":";
  json += bno_present ? "true" : "false";
  json += ",\"bno_retry\":";
  json += String(bno_recovery_count);
  json += ",\"scd\":";
  json += scd_present ? "true" : "false";
  json += ",\"mag\":";
  json += mag_snapshot.ok ? "true" : "false";
  json += ",\"gps\":";
  json += (gps_data.chars > 0) ? "true" : "false";
  json += ",\"restarts\":";
  json += String(supervisor_restart_count);
  json += "}";
  json += "}";
  json += "}";
  return json;
}

static String buildGpsRawJson() {
  String json;
  json.reserve(900);
  json += "{";
  json += "\"type\":\"gps_raw\"";
  json += ",\"raw_len\":";
  json += String(gps_raw_len);
  json += ",\"raw_hex\":\"";
  json += gpsRawHex();
  json += "\"";
  json += "}";
  return json;
}

static void handleRoot(AsyncWebServerRequest* request) {
  noteHttpActivity();
  AsyncWebServerResponse* response =
      request->beginResponse(200, "text/html", kIndexHtml);
  response->addHeader("Cache-Control",
                      "no-store, no-cache, must-revalidate, max-age=0");
  response->addHeader("Pragma", "no-cache");
  response->addHeader("Expires", "0");
  request->send(response);
}

static void handleApi(AsyncWebServerRequest* request) {
  noteHttpActivity();
  AsyncWebServerResponse* response =
      request->beginResponse(200, "application/json", buildTelemetryJson(false));
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

static void handleClean(AsyncWebServerRequest* request) {
  noteHttpActivity();
  sps_clean_requested = true;
  request->send(200, "application/json", "{\"ok\":true}");
}

static void handleBmpMode(AsyncWebServerRequest* request) {
  noteHttpActivity();
  if (!request->hasParam("value")) {
    request->send(400, "application/json", "{\"ok\":false}");
    return;
  }
  String mode = request->getParam("value")->value();
  mode.toLowerCase();
  if (mode == "rel") {
    bmp_alt_mode = BmpAltMode::REL;
    request->send(200, "application/json", "{\"ok\":true}");
    return;
  }
  if (mode == "qnh") {
    bmp_alt_mode = BmpAltMode::QNH;
    request->send(200, "application/json", "{\"ok\":true}");
    return;
  }
  request->send(400, "application/json", "{\"ok\":false}");
}

static void handleBmpQnh(AsyncWebServerRequest* request) {
  noteHttpActivity();
  if (!request->hasParam("value")) {
    request->send(400, "application/json", "{\"ok\":false}");
    return;
  }
  float value = request->getParam("value")->value().toFloat();
  if (value < 800.0f || value > 1100.0f) {
    request->send(400, "application/json", "{\"ok\":false}");
    return;
  }
  bmp_ref_hpa = value;
  bmp_alt_mode = BmpAltMode::QNH;
  request->send(200, "application/json", "{\"ok\":true}");
}

static void handleBmpRel(AsyncWebServerRequest* request) {
  noteHttpActivity();
  if (!bmp_data.ok) {
    request->send(409, "application/json", "{\"ok\":false}");
    return;
  }
  bmp_rel_ref_hpa = bmp_data.pressure_hpa;
  bmp_rel_ref_valid = true;
  bmp_alt_mode = BmpAltMode::REL;
  request->send(200, "application/json", "{\"ok\":true}");
}

static void handleBmpQnhAvg(AsyncWebServerRequest* request) {
  noteHttpActivity();
  String json;
  json.reserve(128);
  if (bmp_qnh_count < 5) {
    json += "{\"ok\":false,\"count\":";
    json += String(bmp_qnh_count);
    json += "}";
    request->send(409, "application/json", json);
    return;
  }
  float avg = bmpQnhAverage();
  bmp_ref_hpa = avg;
  bmp_alt_mode = BmpAltMode::QNH;
  json += "{\"ok\":true,\"avg\":";
  json += String(avg, 1);
  json += ",\"count\":";
  json += String(bmp_qnh_count);
  json += "}";
  request->send(200, "application/json", json);
}

static void handleBmpQnhReset(AsyncWebServerRequest* request) {
  noteHttpActivity();
  bmp_qnh_index = 0;
  bmp_qnh_count = 0;
  bmp_qnh_sum = 0.0f;
  bmp_qnh_last_ms = 0;
  request->send(200, "application/json", "{\"ok\":true}");
}

static void handleWifiPower(AsyncWebServerRequest* request) {
  noteHttpActivity();
  if (!request->hasParam("value")) {
    request->send(400, "application/json", "{\"ok\":false}");
    return;
  }
  wifi_power_t power = WIFI_POWER_19_5dBm;
  if (!wifiPowerFromValue(request->getParam("value")->value(), &power)) {
    request->send(400, "application/json", "{\"ok\":false}");
    return;
  }
  wifi_tx_target = power;
  wifi_tx_applied = false;
  wifi_tx_set_err =
      static_cast<int>(esp_wifi_set_max_tx_power(static_cast<int8_t>(power)));
  wifi_tx_apply_err = wifi_tx_set_err;
  request->send(200, "application/json", "{\"ok\":true}");
}

static void handleSystemReset(AsyncWebServerRequest* request) {
  noteHttpActivity();
  system_reset_requested = true;
  system_reset_ms = millis() + 200;
  request->send(200, "application/json", "{\"ok\":true}");
}

static void handleSensorSps(AsyncWebServerRequest* request) {
  noteHttpActivity();
  bool enable = true;
  if (!parseOnOff(request, &enable)) {
    request->send(400, "application/json", "{\"ok\":false}");
    return;
  }
  sps_enabled = enable;
  if (enable) {
    sps_sleeping = true;
    sps_state = SpsState::BOOT_WAIT;
    sps_next_action_ms = millis() + SPS_BOOT_WAIT_MS;
    sps_timeout_count = 0;
    sps_exec_error_count = 0;
    sps_config_pending = true;
  } else {
    sps_sleeping = false;
    sps_next_action_ms = millis() + SPS_GUARD_MS;
  }
  request->send(200, "application/json", "{\"ok\":true}");
}

static void handleSensorSpsReinit(AsyncWebServerRequest* request) {
  noteHttpActivity();
  sps_enabled = true;
  sps_sleeping = false;
  initSps30();
  request->send(200, "application/json", "{\"ok\":true}");
}

static void handleSensorGps(AsyncWebServerRequest* request) {
  noteHttpActivity();
  bool enable = true;
  if (!parseOnOff(request, &enable)) {
    request->send(400, "application/json", "{\"ok\":false}");
    return;
  }
  gps_enabled = enable;
  if (enable) {
    gpsFlushRx();
    gps_raw_head = 0;
    gps_raw_len = 0;
  }
  request->send(200, "application/json", "{\"ok\":true}");
}

static void handleSensorGpsReinit(AsyncWebServerRequest* request) {
  noteHttpActivity();
  gps_enabled = true;
  initGps();
  request->send(200, "application/json", "{\"ok\":true}");
}

static void handleSensorBmp(AsyncWebServerRequest* request) {
  noteHttpActivity();
  bool enable = true;
  if (!parseOnOff(request, &enable)) {
    request->send(400, "application/json", "{\"ok\":false}");
    return;
  }
  bmp_enabled = enable;
  if (enable) {
    bmp_sleeping = true;
    bmp_data_ready = false;
    last_bmp_poll_ms = 0;
  } else {
    bmp_sleeping = false;
  }
  request->send(200, "application/json", "{\"ok\":true}");
}

static void handleSensorBmpReinit(AsyncWebServerRequest* request) {
  noteHttpActivity();
  bmp_enabled = true;
  bmp_sleeping = false;
  initBmp585();
  request->send(200, "application/json", "{\"ok\":true}");
}

static void handleSensorMag(AsyncWebServerRequest* request) {
  noteHttpActivity();
  bool enable = true;
  if (!parseOnOff(request, &enable)) {
    request->send(400, "application/json", "{\"ok\":false}");
    return;
  }
  mag_enabled = enable;
  if (enable) {
    mag_sleeping = false;
    initMagnetometer();
  } else {
    mag_sleeping = false;
  }
  request->send(200, "application/json", "{\"ok\":true}");
}

static void handleSensorMagReinit(AsyncWebServerRequest* request) {
  noteHttpActivity();
  mag_enabled = true;
  mag_sleeping = false;
  initMagnetometer();
  request->send(200, "application/json", "{\"ok\":true}");
}

static void handleSensorBno(AsyncWebServerRequest* request) {
  noteHttpActivity();
  bool enable = true;
  if (!parseOnOff(request, &enable)) {
    request->send(400, "application/json", "{\"ok\":false}");
    return;
  }
  bno_enabled = enable;
  if (enable) {
    bno_sleeping = false;
    bno_next_probe_ms = 0;
    initBno085();
  } else {
    bno_sleeping = true;
    bno_data.ok = false;
    strncpy(bno_data.error_text, "SLEEP", sizeof(bno_data.error_text));
    bno_data.error_text[sizeof(bno_data.error_text) - 1] = '\0';
  }
  request->send(200, "application/json", "{\"ok\":true}");
}

static void handleSensorBnoReinit(AsyncWebServerRequest* request) {
  noteHttpActivity();
  bno_enabled = true;
  bno_sleeping = false;
  bno_reset_requested = false;
  bno_next_probe_ms = 0;
  bno_recovery_count = 0;
  initBno085();
  request->send(200, "application/json", "{\"ok\":true}");
}

static void handleBnoReset(AsyncWebServerRequest* request) {
  noteHttpActivity();
  bno_enabled = true;
  bno_sleeping = false;
  bno_reset_requested = true;
  bno_next_probe_ms = 0;
  request->send(200, "application/json", "{\"ok\":true}");
}

static void handleSensorScd(AsyncWebServerRequest* request) {
  noteHttpActivity();
  bool enable = true;
  if (!parseOnOff(request, &enable)) {
    request->send(400, "application/json", "{\"ok\":false}");
    return;
  }
  scd_enabled = enable;
  if (enable) {
    scd_sleeping = false;
    initScd40();
  } else {
    scd4x.stopPeriodicMeasurement();
    scd_sleeping = true;
    scd_data.ok = false;
    strncpy(scd_data.error_text, "SLEEP", sizeof(scd_data.error_text));
    scd_data.error_text[sizeof(scd_data.error_text) - 1] = '\0';
  }
  request->send(200, "application/json", "{\"ok\":true}");
}

static void handleSensorScdReinit(AsyncWebServerRequest* request) {
  noteHttpActivity();
  scd_enabled = true;
  scd_sleeping = false;
  initScd40();
  request->send(200, "application/json", "{\"ok\":true}");
}

static void updateGps() {
  if (!gps_enabled) {
    return;
  }
  uint32_t now = millis();
  while (Serial1.available() > 0) {
    uint8_t value = static_cast<uint8_t>(Serial1.read());
    gpsRawPush(value);
    gps_data.chars++;
    ubxParseByte(value, now);
  }

  if (gps_data.last_fix_ms != 0) {
    gps_data.location_age_ms = now - gps_data.last_fix_ms;
  }
}

static void initSps30() {
  sps30.begin(Serial2);
  Sps30Data fresh;
  fresh.auto_clean_interval_s = SPS_AUTO_CLEAN_INTERVAL_S;
  portENTER_CRITICAL(&sps_mux);
  sps_data = fresh;
  portEXIT_CRITICAL(&sps_mux);
  sps_state = SpsState::BOOT_WAIT;
  sps_next_action_ms = millis() + SPS_BOOT_WAIT_MS;
  sps_next_read_ms = 0;
  sps_next_status_ms = millis() + SPS_STATUS_POLL_INTERVAL_MS;
  sps_clean_until_ms = 0;
  sps_timeout_count = 0;
  sps_exec_error_count = 0;
  sps_recovery_count = 0;
  sps_clean_requested = false;
  sps_config_pending = true;
  sps_sleeping = false;
}

static void updateSps30() {
  uint32_t now = millis();
  if (!sps_enabled) {
    if (now < sps_next_action_ms) {
      return;
    }
    Sps30Data next_data;
    portENTER_CRITICAL(&sps_mux);
    next_data = sps_data;
    portEXIT_CRITICAL(&sps_mux);
    if (!sps_sleeping) {
      spsFlushRx(SPS_RX_FLUSH_MS);
      sps30.stopMeasurement();
      sps30.sleep();
      sps_sleeping = true;
      next_data.valid = false;
      next_data.last_read_ok = false;
      next_data.last_error = 0;
      strncpy(next_data.last_error_text, "SLEEP",
              sizeof(next_data.last_error_text));
      next_data.last_error_text[sizeof(next_data.last_error_text) - 1] = '\0';
    }
    sps_next_action_ms = now + SPS_GUARD_MS;
    portENTER_CRITICAL(&sps_mux);
    sps_data = next_data;
    portEXIT_CRITICAL(&sps_mux);
    return;
  }
  if (sps_sleeping) {
    sps_sleeping = false;
    sps_state = SpsState::BOOT_WAIT;
    sps_next_action_ms = now + SPS_BOOT_WAIT_MS;
    sps_timeout_count = 0;
    sps_exec_error_count = 0;
    sps_config_pending = true;
  }
  if (now < sps_next_action_ms) {
    return;
  }

  Sps30Data next_data;
  portENTER_CRITICAL(&sps_mux);
  next_data = sps_data;
  portEXIT_CRITICAL(&sps_mux);

  auto setError = [&](int16_t err) {
    next_data.last_error = err;
    if (err == 0) {
      next_data.last_error_text[0] = '\0';
    } else {
      errorToString(static_cast<uint16_t>(err), next_data.last_error_text,
                    sizeof(next_data.last_error_text));
    }
  };

  auto scheduleRecover = [&](uint32_t delay_ms) {
    sps_state = SpsState::RECOVER_STOP;
    sps_next_action_ms = now + delay_ms;
  };

  switch (sps_state) {
    case SpsState::BOOT_WAIT: {
      // Egyszerű inicializálás - egyből SEND_STOP-ra ugrunk
      sps_next_action_ms = now + SPS_GUARD_MS;
      sps_state = SpsState::SEND_STOP;
      break;
    }

    case SpsState::SEND_STOP: {
      int16_t err = sps30.stopMeasurement();
      setError(err);
      sps_state = SpsState::SEND_START;
      sps_next_action_ms = now + SPS_STOP_DELAY_MS;
      break;
    }

    case SpsState::SEND_START: {
      int16_t err =
          sps30.startMeasurement(SPS30_OUTPUT_FORMAT_OUTPUT_FORMAT_FLOAT);
      setError(err);
      if (err != 0) {
        sps_recovery_count++;
        scheduleRecover(SPS_STOP_DELAY_MS);
        break;
      }
      sps_timeout_count = 0;
      sps_exec_error_count = 0;
      sps_next_read_ms = now + SPS_START_SETTLE_MS;
      sps_next_status_ms = now + SPS_STATUS_POLL_INTERVAL_MS;
      sps_state = SpsState::RUN_POLL;
      sps_next_action_ms = now + SPS_GUARD_MS;
      sps_config_pending = true;
      break;
    }

    case SpsState::CLEAN_STOP: {
      spsFlushRx(SPS_RX_FLUSH_MS);
      int16_t err = sps30.stopMeasurement();
      setError(err);
      sps_state = SpsState::CLEAN_START;
      sps_next_action_ms = now + SPS_STOP_DELAY_MS;
      break;
    }

    case SpsState::CLEAN_START: {
      int16_t err = sps30.startFanCleaning();
      next_data.last_clean_error = err;
      if (err == 0) {
        next_data.last_clean_error_text[0] = '\0';
        next_data.last_clean_ms = now;
      } else {
        errorToString(static_cast<uint16_t>(err),
                      next_data.last_clean_error_text,
                      sizeof(next_data.last_clean_error_text));
      }
      if (err != 0) {
        sps_state = SpsState::SEND_START;
        sps_next_action_ms = now + SPS_GUARD_MS;
        break;
      }
      sps_clean_until_ms = now + SPS_CLEAN_DURATION_MS;
      sps_state = SpsState::CLEAN_WAIT;
      sps_next_action_ms = sps_clean_until_ms;
      break;
    }

    case SpsState::CLEAN_WAIT:
      if (now < sps_clean_until_ms) {
        sps_next_action_ms = sps_clean_until_ms;
        break;
      }
      sps_state = SpsState::SEND_START;
      sps_next_action_ms = now + SPS_GUARD_MS;
      break;

    case SpsState::RECOVER_STOP: {
      spsFlushRx(SPS_RX_FLUSH_MS);
      sps30.wakeUpSequence();  // Timeout után felébresztjük
      delay(20);
      int16_t err = sps30.stopMeasurement();
      setError(err);
      sps_state = SpsState::RECOVER_START;
      sps_next_action_ms = now + SPS_STOP_DELAY_MS;
      break;
    }

    case SpsState::RECOVER_START: {
      int16_t err =
          sps30.startMeasurement(SPS30_OUTPUT_FORMAT_OUTPUT_FORMAT_FLOAT);
      setError(err);
      sps_timeout_count = 0;
      sps_exec_error_count = 0;
      sps_next_read_ms = now + SPS_START_SETTLE_MS;
      sps_next_status_ms = now + SPS_STATUS_POLL_INTERVAL_MS;
      sps_state = SpsState::RUN_POLL;
      sps_next_action_ms = now + SPS_GUARD_MS;
      sps_config_pending = true;
      break;
    }

    case SpsState::RUN_POLL: {
      if (sps_clean_requested) {
        sps_clean_requested = false;
        sps_state = SpsState::CLEAN_STOP;
        sps_next_action_ms = now + SPS_GUARD_MS;
        break;
      }

      if (sps_config_pending &&
          now + SPS_GUARD_MS < sps_next_read_ms) {
        int16_t err =
            sps30.writeAutoCleaningInterval(SPS_AUTO_CLEAN_INTERVAL_S);
        next_data.auto_clean_interval_s = SPS_AUTO_CLEAN_INTERVAL_S;
        next_data.last_clean_error = err;
        if (err == 0) {
          next_data.last_clean_error_text[0] = '\0';
        } else {
          errorToString(static_cast<uint16_t>(err),
                        next_data.last_clean_error_text,
                        sizeof(next_data.last_clean_error_text));
        }
        sps_config_pending = false;
        sps_next_action_ms = now + SPS_GUARD_MS;
        break;
      }

      if (now >= sps_next_status_ms &&
          now + SPS_GUARD_MS < sps_next_read_ms) {
        uint32_t status_reg = 0;
        uint8_t reserved = 0;
        int16_t status_err =
            sps30.readDeviceStatusRegister(false, status_reg, reserved);
        next_data.last_status_ms = now;
        next_data.status_error = status_err;
        if (status_err == 0) {
          next_data.status_register = status_reg;
          next_data.status_error_text[0] = '\0';
          buildSpsStatusFlags(status_reg, next_data.status_flags,
                              sizeof(next_data.status_flags));
        } else {
          errorToString(static_cast<uint16_t>(status_err),
                        next_data.status_error_text,
                        sizeof(next_data.status_error_text));
        }
        sps_next_status_ms = now + SPS_STATUS_POLL_INTERVAL_MS;
        sps_next_action_ms = now + SPS_GUARD_MS;
        break;
      }

      if (now < sps_next_read_ms) {
        sps_next_action_ms = sps_next_read_ms;
        break;
      }

      float mc1p0 = 0.0f;
      float mc2p5 = 0.0f;
      float mc4p0 = 0.0f;
      float mc10p0 = 0.0f;
      float nc0p5 = 0.0f;
      float nc1p0 = 0.0f;
      float nc2p5 = 0.0f;
      float nc4p0 = 0.0f;
      float nc10p0 = 0.0f;
      float typical_particle_size = 0.0f;

      int16_t error = sps30.readMeasurementValuesFloat(
          mc1p0, mc2p5, mc4p0, mc10p0, nc0p5, nc1p0, nc2p5, nc4p0, nc10p0,
          typical_particle_size);

      if (error == 0) {
        next_data.valid = true;
        next_data.last_read_ok = true;
        next_data.mc1p0 = mc1p0;
        next_data.mc2p5 = mc2p5;
        next_data.mc4p0 = mc4p0;
        next_data.mc10p0 = mc10p0;
        next_data.nc0p5 = nc0p5;
        next_data.nc1p0 = nc1p0;
        next_data.nc2p5 = nc2p5;
        next_data.nc4p0 = nc4p0;
        next_data.nc10p0 = nc10p0;
        next_data.typical_particle_size = typical_particle_size;
        next_data.last_read_ms = now;
        setError(0);
        sps_timeout_count = 0;
        sps_exec_error_count = 0;
        sps_next_read_ms = now + SPS_POLL_MS;
        sps_next_action_ms = now + SPS_GUARD_MS;
        break;
      }

      if (error == SPS_NO_DATA_ERROR) {
        sps_next_read_ms = now + SPS_POLL_MS;
        sps_next_action_ms = now + SPS_GUARD_MS;
        break;
      }

      next_data.last_read_ok = false;
      setError(error);

      if (error == SPS_TIMEOUT_ERROR) {
        sps_timeout_count =
            static_cast<uint8_t>(sps_timeout_count + 1);
        spsFlushRx(SPS_RX_FLUSH_MS);
        if (sps_timeout_count >= SPS_MAX_TIMEOUTS) {
          scheduleRecover(SPS_GUARD_MS);
          break;
        }
        sps_next_read_ms = now + SPS_POLL_MS;
        sps_next_action_ms = now + SPS_GUARD_MS;
        break;
      }

      if (spsIsExecutionError(error)) {
        sps_exec_error_count =
            static_cast<uint8_t>(sps_exec_error_count + 1);
        scheduleRecover(SPS_GUARD_MS);
        break;
      }

      if (spsIsCrcError(error)) {
        sps_next_read_ms = now + SPS_POLL_MS;
        sps_next_action_ms = now + SPS_GUARD_MS;
        break;
      }

      sps_next_read_ms = now + SPS_POLL_MS;
      sps_next_action_ms = now + SPS_GUARD_MS;
      break;
    }
  }

  portENTER_CRITICAL(&sps_mux);
  sps_data = next_data;
  portEXIT_CRITICAL(&sps_mux);
}

static void spsTask(void* arg) {
  (void)arg;
  for (;;) {
    updateSps30();
    vTaskDelay(pdMS_TO_TICKS(SPS_TASK_DELAY_MS));
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("CanSat2526");

  // Watchdog init - ha 30s-ig nem reseteljük, hard reset
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);

  last_any_sensor_ok_ms = millis();

  i2cBusRecoveryPins(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  configureI2cBus();
  pinMode(PIN_BNO_INT, INPUT_PULLUP);
  pinMode(PIN_BNO_WAKE, OUTPUT);
  digitalWrite(PIN_BNO_WAKE, HIGH);
  pinMode(PIN_BNO_RST, OUTPUT);
  digitalWrite(PIN_BNO_RST, HIGH);
  pinMode(PIN_BNO_CS, OUTPUT);
  digitalWrite(PIN_BNO_CS, HIGH);
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_BNO_CS);


  Serial2.begin(UART2_BAUD, SERIAL_8N1, PIN_UART2_RX, PIN_UART2_TX);
  initGps();

  initMagnetometer();
  initScd40();
  initBmp585();

  status_led.begin();
  status_led.clear();
  status_led.setBrightness(LED_BRIGHTNESS);
  status_led.show();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  wifi_ps_err = static_cast<int>(esp_wifi_set_ps(WIFI_PS_NONE));
  wifi_tx_target = WIFI_POWER_19_5dBm;
  wifi_tx_applied = false;
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t wifi_start = millis();
  while (WiFi.status() != WL_CONNECTED &&
         (millis() - wifi_start) < 20000) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("WiFi STA OK, IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.printf("WiFi STA FAIL, status: %d\n", WiFi.status());
  }

  initSps30();

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api", HTTP_GET, handleApi);
  server.on("/clean", HTTP_POST, handleClean);
  server.on("/bmp/mode", HTTP_POST, handleBmpMode);
  server.on("/bmp/qnh", HTTP_POST, handleBmpQnh);
  server.on("/bmp/rel", HTTP_POST, handleBmpRel);
  server.on("/bmp/qnh/avg", HTTP_POST, handleBmpQnhAvg);
  server.on("/bmp/qnh/reset", HTTP_POST, handleBmpQnhReset);
  server.on("/wifi/power", HTTP_POST, handleWifiPower);
  server.on("/sensor/sps30", HTTP_POST, handleSensorSps);
  server.on("/sensor/sps30/reinit", HTTP_POST, handleSensorSpsReinit);
  server.on("/sensor/gps", HTTP_POST, handleSensorGps);
  server.on("/sensor/gps/reinit", HTTP_POST, handleSensorGpsReinit);
  server.on("/sensor/bmp", HTTP_POST, handleSensorBmp);
  server.on("/sensor/bmp/reinit", HTTP_POST, handleSensorBmpReinit);
  server.on("/sensor/mag", HTTP_POST, handleSensorMag);
  server.on("/sensor/mag/reinit", HTTP_POST, handleSensorMagReinit);
  server.on("/sensor/bno", HTTP_POST, handleSensorBno);
  server.on("/sensor/bno/reinit", HTTP_POST, handleSensorBnoReinit);
  server.on("/bno/reset", HTTP_POST, handleBnoReset);
  server.on("/sensor/scd", HTTP_POST, handleSensorScd);
  server.on("/sensor/scd/reinit", HTTP_POST, handleSensorScdReinit);
  server.on("/system/reset", HTTP_POST, handleSystemReset);
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();

  xTaskCreatePinnedToCore(spsTask, "spsTask", 4096, nullptr, 1,
                          &sps_task_handle, 1);
}

void loop() {
  if (system_reset_requested && millis() >= system_reset_ms) {
    ESP.restart();
  }
  updateGps();
  updateBmp585();
  updateMagnetometer();
  if (bno_reset_requested) {
    bno_reset_requested = false;
    initBno085();
  }
  updateBno085();
  updateBnoRecovery();
  updateScd40();
  updateWifiPower();
  uint32_t now = millis();
  if (ws.count() > 0) {
    if ((now - last_ws_fast_ms) >= WS_FAST_MS && ws.availableForWriteAll()) {
      uint8_t frame[16];
      size_t len = buildWsFastFrame(frame, sizeof(frame));
      if (len > 0) {
        ws.binaryAll(frame, len);
        last_ws_fast_ms = now;
        noteHttpActivity();
      }
    }
    if ((now - last_ws_gps_ms) >= WS_GPS_MS && ws.availableForWriteAll()) {
      uint8_t frame[18];
      size_t len = buildWsGpsFrame(frame, sizeof(frame));
      if (len > 0) {
        ws.binaryAll(frame, len);
        last_ws_gps_ms = now;
        noteHttpActivity();
      }
    }
  }
  ws.cleanupClients();
  updateStatusLed();

  // Watchdog feed - jelezzük, hogy a loop fut
  esp_task_wdt_reset();

  // Supervisor - ha bármelyik szenzor működik, frissítjük az időt
  Sps30Data sps_snap;
  portENTER_CRITICAL(&sps_mux);
  sps_snap = sps_data;
  portEXIT_CRITICAL(&sps_mux);

  bool any_sensor_ok = sps_snap.valid || bmp_data.ok || bno_data.ok ||
                       scd_data.ok || mag_data.ok || gps_data.fix;
  if (any_sensor_ok) {
    last_any_sensor_ok_ms = now;
  }

  // Ha túl régóta nincs működő szenzor -> restart
  if ((now - last_any_sensor_ok_ms) > SENSOR_DEAD_TIMEOUT_MS) {
    supervisor_restart_count++;
    ESP.restart();
  }

}
