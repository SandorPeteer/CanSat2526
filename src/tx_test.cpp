#include <Arduino.h>
#include <Wire.h>

#include <SPI.h>
#include <Preferences.h>
#include "esp_task_wdt.h"

/* ---------------------------------------------------------------------------
   I2C bus beragadás elleni védelem – SCL manuális clockolása
--------------------------------------------------------------------------- */
static void i2c_unstick(int sclPin, int sdaPin)
{
  pinMode(sclPin, OUTPUT);
  pinMode(sdaPin, INPUT_PULLUP);

  // 9 clock impulzus, ha SDA fogva van tartva
  for (int i = 0; i < 9; i++)
  {
    digitalWrite(sclPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(sclPin, LOW);
    delayMicroseconds(5);
  }

  // STOP feltétel generálása
  pinMode(sdaPin, OUTPUT);
  digitalWrite(sdaPin, LOW);
  delayMicroseconds(5);
  digitalWrite(sclPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(sdaPin, HIGH);
  delayMicroseconds(5);
}

// -----------------------------------------------------------------------------
//  HARDVER PIN DEFINÍCIÓK (EZEKET A SAJÁT BEKÖTÉSEDHEZ IGAZÍTSD!)
// -----------------------------------------------------------------------------

// I2C BME280 – ESP32-CAM-on általában:
#define BME_SDA_PIN 21 // állítsd be a valós SDA pinre
#define BME_SCL_PIN 22 // állítsd be a valós SCL pinre

// SX1278 LoRa (példa bekötés – ezt a saját modulodhoz igazítsd)
#define LORA_SCK 18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SS 5
#define LORA_RST 26
#define LORA_DIO0 25 // !!! kritikus, ez az interrupt láb !!!
#define LORA_DIO1 36
#define LORA_DIO2 33

#define CAM_LED_PIN 4

// ---------------------------------------------------------------------------
//  LoRa frekvencia beállítások (globális)
// ---------------------------------------------------------------------------
static uint32_t LORA_FREQ_HZ = 433200000; // alap vivőfrekvencia (Hz)
static int32_t LORA_OFFSET_HZ = 5500;     // kézi offset (Hz)

// -----------------------------------------------------------------------------
//  BME280 REGISZTEREK / ÁLLANDÓK
// -----------------------------------------------------------------------------

static const uint8_t BME_REG_ID = 0xD0;
static const uint8_t BME_REG_RESET = 0xE0;
static const uint8_t BME_REG_STATUS = 0xF3;
static const uint8_t BME_REG_CTRL_MEAS = 0xF4;
static const uint8_t BME_REG_CTRL_HUM = 0xF2;
static const uint8_t BME_REG_CONFIG = 0xF5;
static const uint8_t BME_REG_PRESS_MSB = 0xF7;

// BME280 chip ID
static const uint8_t BME_ID = 0x60;

// Kimaxolt oversampling (x16) + IIR=16, forced mód
// ctrl_hum: osrs_h = 0b101 (x16)
static const uint8_t BME_CTRL_HUM = 0b00000101;
// ctrl_meas: osrs_t=101, osrs_p=101, mode=01 (forced)
static const uint8_t BME_CTRL_MEAS = 0b10110101; // 0xB5
// config: filter=100 (16x), t_sb=000 (0.5ms), spi3w_en=0
static const uint8_t BME_CONFIG = 0b00010000; // 0x10

// BME időzítések
static const uint32_t BME_MAX_MEAS_TIME_MS = 120; // 112,6 ms → 120 ms biztonságosan

// -----------------------------------------------------------------------------
//  SX1278 (SX1276 kompatibilis) REGISZTEREK / ÁLLANDÓK
// -----------------------------------------------------------------------------

// Regiszter címek
#define REG_FIFO 0x00
#define REG_OP_MODE 0x01
#define REG_FRF_MSB 0x06
#define REG_FRF_MID 0x07
#define REG_FRF_LSB 0x08
#define REG_PA_CONFIG 0x09
#define REG_OCP 0x0B
#define REG_LNA 0x0C
#define REG_FIFO_ADDR_PTR 0x0D
#define REG_FIFO_TX_BASE 0x0E
#define REG_FIFO_RX_BASE 0x0F
#define REG_IRQ_FLAGS 0x12
#define REG_RX_NB_BYTES 0x13
#define REG_MODEM_CONFIG1 0x1D
#define REG_MODEM_CONFIG2 0x1E
#define REG_SYMB_TIMEOUT_LSB 0x1F
#define REG_PREAMBLE_MSB 0x20
#define REG_PREAMBLE_LSB 0x21
#define REG_PAYLOAD_LENGTH 0x22
#define REG_MODEM_CONFIG3 0x26
#define REG_DIO_MAPPING1 0x40
#define REG_VERSION 0x42
#define REG_PA_DAC 0x4D

// LoRa op-mode bitek
#define LONG_RANGE_MODE 0x80
#define MODE_SLEEP 0x00
#define MODE_STDBY 0x01
#define MODE_TX 0x03

// Csomagméret
static const uint8_t LORA_MAX_PAYLOAD = 48;

// -----------------------------------------------------------------------------
//  FRAME SZINKRON BYTE-OK
// -----------------------------------------------------------------------------
static const uint8_t SYNC_FULL  = 0xA5; // teljes (keyframe)
static const uint8_t SYNC_DELTA = 0xA4; // delta frame (kulcskerethez viszonyítva)

// Keyframe (FULL) hossza a jelenlegi formátumban: 38 byte
static const uint8_t FULL_FRAME_LEN = 38;
// Delta frame fix hossza: 23 byte
static const uint8_t DELTA_FRAME_LEN = 23;

// -----------------------------------------------------------------------------
//  GLOBÁLIS ÁLLAPOT – BME280 KALIBRÁCIÓ
// -----------------------------------------------------------------------------

static uint8_t g_bme_addr = 0x76; // autodetekcióval állítjuk be

// Kalibrációs konstansok (Bosch BME280 datasheet alapján)
static uint16_t dig_T1;
static int16_t dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static uint8_t dig_H1;
static int16_t dig_H2;
static uint8_t dig_H3;
static int16_t dig_H4, dig_H5;
static int8_t dig_H6;

static int32_t t_fine;

// -----------------------------------------------------------------------------
//  GLOBÁLIS ÁLLAPOT – BME280 MÉRÉSI ÁLLAPOTGÉP
// -----------------------------------------------------------------------------

enum BmeState
{
  BME_IDLE,
  BME_MEASURING
};

static BmeState g_bme_state = BME_IDLE;
static uint32_t g_bme_meas_start_ms = 0;

// -----------------------------------------------------------------------------
//  NYERS MINTA BUFFER (10 Hz → mozgóátlag)
// -----------------------------------------------------------------------------

struct RawSample
{
  float tempC;
  float rh;
  float press_hPa;
};

static const uint8_t RAW_BUF_SIZE = 16;
static RawSample g_raw_buf[RAW_BUF_SIZE];
static uint8_t g_raw_head = 0;
static uint8_t g_raw_count = 0;
static uint8_t g_bme_warmup = 0;

// Mozgóátlag ablakméret (5–10 közé tehető, itt 5)
static const uint8_t MA_WINDOW = 5;

// -----------------------------------------------------------------------------
//  TELEMETRIA MINTA + BITMASK + PACK
// -----------------------------------------------------------------------------

struct TeleSample
{
  float tempC;
  float rh;
  float press_hPa;
};
static void generate_counter_sample(TeleSample &ts);

static const uint8_t MAX_SAMPLES_PER_PACKET = 8;

// Telemetria buffer egy LoRa csomaghoz
static TeleSample g_tele_buf[MAX_SAMPLES_PER_PACKET];
static bool g_tele_flag_changed[MAX_SAMPLES_PER_PACKET];
static uint8_t g_tele_count = 0;
static void telemetry_flush_packet();

// Előző minta (változásdetektáláshoz)

static bool g_have_prev_sample = false;
static uint8_t g_prev_pack_global[8] = {0};

// MET számláló (0,5 s tick)
static uint16_t g_MET = 0;
static uint32_t g_last_MET_ms = 0;
static uint32_t g_last_met_store_ms = 0;
Preferences g_prefs;

// --- Frame sequence counter and reset event flag ---
static uint8_t g_frame_seq = 0;
static bool g_reset_event_pending = true;

// --- Delta/kulcsframe állapot ---
static bool g_have_prev_abs30 = false;
static uint32_t g_prev_abs30 = 0;              // legutóbb elküldött abszolút minta (30 bit, last sample)
static uint8_t g_last_keyframe_seq = 0;        // utolsó elküldött kulcsframe seq
static uint8_t g_packets_since_keyframe = 0;   // delta frames száma a legutóbbi kulcsframe óta

// Kulcsframe periodicitás (receiver bekapcsolás miatti gyors lock + drift elleni védelem)
// 4 csomag = kb. 16 s-onként biztos kulcsframe (8 minta/csomag * 0.5 s)
static const uint8_t KEYFRAME_PERIOD_PACKETS = 4;

// -----------------------------------------------------------------------------
//  RENDSZER ÁLLAPOT – MET INDÍTÁS SZENZOROK ALAPJÁN
// -----------------------------------------------------------------------------

enum SystemState
{
  SYS_WAIT_SENSORS, // szenzorok inicializálása, első érvényes minta várása
  SYS_FLIGHT        // MET fut, repülés alatt
};

static SystemState g_sys_state = SYS_WAIT_SENSORS;
static bool g_sensor_ready = false; // akkor lesz true, ha van legalább 1 érvényes mozgóátlag minta

// -----------------------------------------------------------------------------
//  BME280 – ALAP I2C SEGÉDFÜGGVÉNYEK
// -----------------------------------------------------------------------------

static void bme_write8(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(g_bme_addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static uint8_t bme_read8(uint8_t reg)
{
  Wire.beginTransmission(g_bme_addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)g_bme_addr, 1);
  if (Wire.available())
  {
    return Wire.read();
  }
  return 0;
}

static void bme_read_bytes(uint8_t reg, uint8_t *buf, uint8_t len)
{
  Wire.beginTransmission(g_bme_addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)g_bme_addr, (int)len);
  uint8_t idx = 0;
  while (Wire.available() && idx < len)
  {
    buf[idx++] = Wire.read();
  }
}

// -----------------------------------------------------------------------------
//  BME280 – KALIBRÁCIÓ KIOLVASÁSA
// -----------------------------------------------------------------------------

static void bme_read_calibration()
{
  uint8_t buf[26];

  // T1..P9: 0x88..0xA1
  bme_read_bytes(0x88, buf, 24);
  dig_T1 = (uint16_t)(buf[1] << 8 | buf[0]);
  dig_T2 = (int16_t)(buf[3] << 8 | buf[2]);
  dig_T3 = (int16_t)(buf[5] << 8 | buf[4]);

  dig_P1 = (uint16_t)(buf[7] << 8 | buf[6]);
  dig_P2 = (int16_t)(buf[9] << 8 | buf[8]);
  dig_P3 = (int16_t)(buf[11] << 8 | buf[10]);
  dig_P4 = (int16_t)(buf[13] << 8 | buf[12]);
  dig_P5 = (int16_t)(buf[15] << 8 | buf[14]);
  dig_P6 = (int16_t)(buf[17] << 8 | buf[16]);
  dig_P7 = (int16_t)(buf[19] << 8 | buf[18]);
  dig_P8 = (int16_t)(buf[21] << 8 | buf[20]);
  dig_P9 = (int16_t)(buf[23] << 8 | buf[22]);

  dig_H1 = bme_read8(0xA1);

  // H2..H6: 0xE1..0xE7
  uint8_t hbuf[7];
  bme_read_bytes(0xE1, hbuf, 7);

  dig_H2 = (int16_t)(hbuf[1] << 8 | hbuf[0]);
  dig_H3 = hbuf[2];
  dig_H4 = (int16_t)((hbuf[3] << 4) | (hbuf[4] & 0x0F));
  dig_H5 = (int16_t)((hbuf[5] << 4) | (hbuf[4] >> 4));
  dig_H6 = (int8_t)hbuf[6];
}

// -----------------------------------------------------------------------------
//  BME280 – DETEKTÁLÁS + KONFIGURÁLÁS (FORCED MODE, x16, IIR=16)
// -----------------------------------------------------------------------------

static bool bme_begin()
{
  // I2C init
  // I2C busz felszabadítása reset után
  i2c_unstick(BME_SCL_PIN, BME_SDA_PIN);
  Wire.begin(BME_SDA_PIN, BME_SCL_PIN);
  Wire.setClock(400000);

  // Két cím próbálása: 0x76, 0x77
  uint8_t addrs[2] = {0x76, 0x77};
  bool ok = false;

  for (int i = 0; i < 2; ++i)
  {
    g_bme_addr = addrs[i];
    uint8_t id = bme_read8(BME_REG_ID);
    if (id == BME_ID)
    {
      ok = true;
      break;
    }
  }
  if (!ok)
  {
    return false;
  }

  // Soft reset
  bme_write8(BME_REG_RESET, 0xB6);
  delay(5);
  // Extra reset ciklus biztosítja, hogy a szenzor minden regisztere tiszta
  bme_write8(BME_REG_RESET, 0xB6);
  delay(5);

  // Kalibráció beolvasása
  bme_read_calibration();

  // Konfigurálás: filter, oversampling
  bme_write8(BME_REG_CONFIG, BME_CONFIG);
  bme_write8(BME_REG_CTRL_HUM, BME_CTRL_HUM);
  // ctrl_meas-ben a mode biteket mindig forced-ra állítjuk, amikor mérni akarunk
  bme_write8(BME_REG_CTRL_MEAS, BME_CTRL_MEAS);

  g_bme_state = BME_IDLE;
  return true;
}

// -----------------------------------------------------------------------------
//  BME280 – KOMPENZÁLÓ FÜGGVÉNYEK (BOSCH ALGORITMUS)
// -----------------------------------------------------------------------------

static float bme_compensate_T(int32_t adc_T)
{
  int32_t var1, var2;
  var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) *
          ((int32_t)dig_T2)) >>
         11;
  var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
            ((adc_T >> 4) - ((int32_t)dig_T1))) >>
           12) *
          ((int32_t)dig_T3)) >>
         14;
  t_fine = var1 + var2;
  float T = (t_fine * 5 + 128) >> 8;
  return T / 100.0f;
}

static float bme_compensate_P(int32_t adc_P)
{
  int64_t var1, var2, p;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
  var2 = var2 + (((int64_t)dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) +
         ((var1 * (int64_t)dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1) * (int64_t)dig_P1) >> 33;

  if (var1 == 0)
  {
    return 0; // elkerüljük a 0-val osztást
  }

  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);

  return (float)p / 256.0f / 100.0f; // hPa
}

static float bme_compensate_H(int32_t adc_H)
{
  int32_t v_x1_u32r;

  v_x1_u32r = (t_fine - ((int32_t)76800));
  v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) -
                  (((int32_t)dig_H5) * v_x1_u32r)) +
                 ((int32_t)16384)) >>
                15) *
               (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) *
                    (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) + 32768)) >>
                   10) +
                  2097152) *
                     ((int32_t)dig_H2) +
                 8192) >>
                14));
  v_x1_u32r = (v_x1_u32r -
               (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                 ((int32_t)dig_H1)) >>
                4));
  v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
  v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
  float h = (v_x1_u32r >> 12) / 1024.0f;
  return h; // % RH
}

// -----------------------------------------------------------------------------
//  BME280 – MÉRÉSI ÁLLAPOTGÉP (NEM BLOKKOLÓ)
// -----------------------------------------------------------------------------

// Mérés indítása forced módban
static void bme_start_measurement()
{
  // ctrl_hum + config már be van állítva; most újraírjuk a ctrl_meas-t forced módra
  bme_write8(BME_REG_CTRL_MEAS, BME_CTRL_MEAS);
  g_bme_meas_start_ms = millis();
  g_bme_state = BME_MEASURING;
}

// Ha van kész mérés, kiolvassa és bepakolja a nyers bufferbe
static void bme_poll()
{
  uint32_t now = millis();
  static uint32_t lastCheckMs = 0;
  if (g_bme_state == BME_IDLE)
  {
    // új mérés indítása
    bme_start_measurement();
    lastCheckMs = now; // poll időzítés újramérése
    return;
  }

  // 70ms minimum wait before checking status
  if (now - g_bme_meas_start_ms < 70)
  {
    return; // too early to poll
  }

  // poll only every 10ms after 70ms
  if (now - lastCheckMs < 10)
  {
    return;
  }
  lastCheckMs = now;

  uint8_t status = bme_read8(BME_REG_STATUS);
  bool measuring = status & 0x08;

  if (!measuring || (now - g_bme_meas_start_ms) > BME_MAX_MEAS_TIME_MS)
  {
    // Kész a mérés (vagy timeout) → adat kiolvasása
    uint8_t data[8];
    bme_read_bytes(BME_REG_PRESS_MSB, data, 8);

    int32_t adc_P = ((int32_t)data[0] << 12) |
                    ((int32_t)data[1] << 4) |
                    ((int32_t)data[2] >> 4);

    int32_t adc_T = ((int32_t)data[3] << 12) |
                    ((int32_t)data[4] << 4) |
                    ((int32_t)data[5] >> 4);

    int32_t adc_H = ((int32_t)data[6] << 8) | (int32_t)data[7];

    float T = bme_compensate_T(adc_T);
    float P = bme_compensate_P(adc_P); // hPa
    float H = bme_compensate_H(adc_H); // %

    // Nyers bufferbe rakjuk (gyűrűs)
    RawSample rs;
    rs.tempC = T;
    rs.rh = H;
    rs.press_hPa = P;

    // Warmup: skip the first 20 BME readings
    if (g_bme_warmup < 20)
    {
      g_bme_warmup++;
    }
    else
    {
      g_raw_buf[g_raw_head] = rs;
      g_raw_head = (g_raw_head + 1) % RAW_BUF_SIZE;
      if (g_raw_count < RAW_BUF_SIZE)
      {
        g_raw_count++;
      }
    }

    // Vissza IDLE állapotba (következő loop-ban újra indítjuk)
    g_bme_state = BME_IDLE;
  }
}

// -----------------------------------------------------------------------------
//  MOZGÓÁTLAG – LEGUTÓBBI N RAW MINTÁBÓL
// -----------------------------------------------------------------------------

static bool compute_moving_average(TeleSample &out)
{
  if (g_raw_count == 0)
    return false;

  uint8_t n = g_raw_count < MA_WINDOW ? g_raw_count : MA_WINDOW;

  float sumT = 0.0f;
  float sumH = 0.0f;
  float sumP = 0.0f;

  // g_raw_head az első "szabad" elem indexe, visszafelé lépkedünk
  int idx = (int)g_raw_head;
  for (uint8_t i = 0; i < n; ++i)
  {
    idx = (idx - 1);
    if (idx < 0)
      idx += RAW_BUF_SIZE;

    const RawSample &rs = g_raw_buf[idx];
    sumT += rs.tempC;
    sumH += rs.rh;
    sumP += rs.press_hPa;
  }

  out.tempC = sumT / n;
  out.rh = sumH / n;
  out.press_hPa = sumP / n;
  return true;
}

// -----------------------------------------------------------------------------
//  T/RH/P TÖMÖRÍTÉS 5 BYTE-OS PACK-KÁ (BITRE PONTOSAN)
// -----------------------------------------------------------------------------

// A Telemetria doksi szerint:
// T: 11 bit (0,1 °C, –40,0…+87,9 °C)
// RH: 7 bit (0–100 %)
// P: 12 bit (822…1077 hPa, 0,1 hPa)

static uint8_t encode_dynamic_pack(const TeleSample &s, uint8_t *out)
{
  // 1) Sensor field bit-widths (expandable later)
  struct Field
  {
    uint32_t val;
    uint8_t bits;
  };
  Field fields[4];

  // ---- Temperature → 11 bit ----
  float T_c = s.tempC;
  if (T_c < -40.0f)
    T_c = -40.0f;
  if (T_c > 87.9f)
    T_c = 87.9f;
  uint32_t T_code = (uint32_t)lroundf((T_c + 40.0f) * 10.0f);
  fields[0] = {T_code, 11};

  // ---- Humidity → 7 bit ----
  float H = s.rh;
  if (H < 0.0f)
    H = 0.0f;
  if (H > 100.0f)
    H = 100.0f;
  uint32_t RH_code = (uint32_t)lroundf(H);
  fields[1] = {RH_code, 7};

  // ---- Pressure encode (12 bit total: 0.1 hPa resolution) ----
  float P = s.press_hPa;
  if (P < 822.0f)  P = 822.0f;
  if (P > 1077.0f) P = 1077.0f;

  // 12‑bit pressure code: 0..2550 representing 822.0–1077.0 hPa in 0.1 hPa units
  uint32_t P_code12 = (uint32_t)lroundf((P - 822.0f) * 10.0f); // 0..2550

  // integer part (0..255)
  uint32_t P_int_code = P_code12 / 10;
  if (P_int_code > 255) P_int_code = 255;

  // fractional part (0..9)
  uint32_t P_frac_code = P_code12 % 10;
  if (P_frac_code > 9) P_frac_code = 9;

  fields[2] = {P_int_code, 8};
  fields[3] = {P_frac_code, 4};

  // 2) Total bit count
  uint16_t total_bits = 0;
  for (int i = 0; i < 4; i++)
    total_bits += fields[i].bits;

  uint8_t total_bytes = (total_bits + 7) / 8;

  // 3) Bit packing LSB‑first
  uint64_t acc = 0;
  uint16_t bitpos = 0;
  for (int i = 0; i < 4; i++)
  {
    uint64_t v = fields[i].val & ((1ULL << fields[i].bits) - 1);
    acc |= (v << bitpos);
    bitpos += fields[i].bits;
  }

  // 4) Output → minimal number of bytes
  for (uint8_t i = 0; i < total_bytes; i++)
  {
    out[i] = (acc >> (8 * i)) & 0xFF;
  }

  return total_bytes;
}

// -----------------------------------------------------------------------------
//  30 BITES (PAZARLÁSMENTES) MINTA KÓDOLÁS + BITFOLYAMOS PACK (8 minta -> 30 byte)
// -----------------------------------------------------------------------------

static uint32_t encode_sample_30bit_lsb(const TeleSample &s)
{
  float T_c = s.tempC;
  if (T_c < -40.0f) T_c = -40.0f;
  if (T_c > 87.9f)  T_c = 87.9f;
  uint32_t T_code = (uint32_t)lroundf((T_c + 40.0f) * 10.0f) & ((1u << 11) - 1);

  float H = s.rh;
  if (H < 0.0f)   H = 0.0f;
  if (H > 100.0f) H = 100.0f;
  uint32_t RH_code = (uint32_t)lroundf(H) & ((1u << 7) - 1);

  float P = s.press_hPa;
  if (P < 822.0f)  P = 822.0f;
  if (P > 1077.0f) P = 1077.0f;

  uint32_t P_code12 = (uint32_t)lroundf((P - 822.0f) * 10.0f);
  uint32_t P_int_code = P_code12 / 10;
  if (P_int_code > 255) P_int_code = 255;
  uint32_t P_frac_code = P_code12 % 10;
  if (P_frac_code > 9) P_frac_code = 9;

  uint32_t v = 0;
  v |= (T_code & ((1u << 11) - 1)) << 0;
  v |= (RH_code & ((1u << 7) - 1)) << 11;
  v |= (P_int_code & 0xFFu)        << 18;
  v |= (P_frac_code & 0x0Fu)       << 26;
  return v;
}

static void bitstream_write_lsb(uint8_t *dst, uint16_t &bitpos, uint32_t v, uint8_t bits)
{
  for (uint8_t i = 0; i < bits; ++i)
  {
    uint16_t byteIndex = bitpos >> 3;
    uint8_t bitIndex = bitpos & 7;
    if (v & (1u << i))
      dst[byteIndex] |= (1u << bitIndex);
    else
      dst[byteIndex] &= ~(1u << bitIndex);
    bitpos++;
  }
}

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len);
// -----------------------------------------------------------------------------
//  DELTA FRAME KÓDOLÁS
//  - abszolút 30-bit kódokból számolunk deltákat
//  - bitkiosztás / minta: dT(5) + dRH(4) + dP(6) = 15 bit
//    * dT  : -16..+15  (0.1°C kód lépés)
//    * dRH :  -8.. +7  (% kód lépés)
//    * dP  : -32..+31  (0.1 hPa kód lépés, 12-bit P_code12)
//  - ha bármely delta kilóg, automatikusan FULL keyframe megy.
// -----------------------------------------------------------------------------

static inline uint16_t abs_T_code(uint32_t v30) { return (uint16_t)(v30 & 0x7FFu); }
static inline uint8_t  abs_RH_code(uint32_t v30) { return (uint8_t)((v30 >> 11) & 0x7Fu); }
static inline uint16_t abs_P_code12(uint32_t v30)
{
  uint16_t p_int = (uint16_t)((v30 >> 18) & 0xFFu);
  uint16_t p_frac = (uint16_t)((v30 >> 26) & 0x0Fu);
  if (p_frac > 9) p_frac = 9;
  return (uint16_t)(p_int * 10u + p_frac); // 0..2559 (mi 0..2550-ig használjuk)
}

static inline uint32_t twos_comp_pack(int32_t v, uint8_t bits)
{
  // v már tartományon belül legyen
  uint32_t mask = (bits >= 32) ? 0xFFFFFFFFu : ((1u << bits) - 1u);
  return ((uint32_t)v) & mask;
}

static bool build_delta_frame(uint8_t *out_frame, uint16_t base_MET, uint8_t flags_base, uint8_t seq)
{
  // out_frame mérete: DELTA_FRAME_LEN
  for (uint8_t i = 0; i < DELTA_FRAME_LEN; ++i) out_frame[i] = 0;

  out_frame[0] = SYNC_DELTA;
  out_frame[1] = (uint8_t)(base_MET & 0xFF);
  out_frame[2] = (uint8_t)(base_MET >> 8);

  // FLAGS: FULL=0, DELTA=1
  uint8_t flags = flags_base;
  flags &= ~(1u << 0); // FULL_STATE = 0
  flags |=  (1u << 1); // DELTA_STATE = 1
  out_frame[3] = flags;

  out_frame[4] = seq;
  out_frame[5] = g_last_keyframe_seq; // referenciakulcs seq

  // Delta bitstream a 6. byte-tól (15 byte, összesen 120 bit)
  uint16_t bitpos = 6u * 8u;

  uint32_t prev = g_prev_abs30;
  for (uint8_t i = 0; i < MAX_SAMPLES_PER_PACKET; ++i)
  {
    uint32_t cur = encode_sample_30bit_lsb(g_tele_buf[i]) & ((1u << 30) - 1);

    int32_t dT  = (int32_t)abs_T_code(cur)  - (int32_t)abs_T_code(prev);
    int32_t dRH = (int32_t)abs_RH_code(cur) - (int32_t)abs_RH_code(prev);
    int32_t dP  = (int32_t)abs_P_code12(cur) - (int32_t)abs_P_code12(prev);

    // tartomány ellenőrzés
    if (dT < -16 || dT > 15) return false;
    if (dRH < -8 || dRH > 7) return false;
    if (dP < -32 || dP > 31) return false;

    uint32_t pT  = twos_comp_pack(dT, 5);
    uint32_t pRH = twos_comp_pack(dRH, 4);
    uint32_t pP  = twos_comp_pack(dP, 6);

    bitstream_write_lsb(out_frame, bitpos, pT, 5);
    bitstream_write_lsb(out_frame, bitpos, pRH, 4);
    bitstream_write_lsb(out_frame, bitpos, pP, 6);

    prev = cur;
  }

  // CRC over bytes 0..20 (21 bytes)
  uint16_t crc = crc16_ccitt(out_frame, 21);
  out_frame[21] = (uint8_t)(crc & 0xFF);
  out_frame[22] = (uint8_t)(crc >> 8);

  return true;
}

// -----------------------------------------------------------------------------
//  CRC16-CCITT (poly 0x1021, init 0xFFFF) – payload integrity (appended in frame)
// -----------------------------------------------------------------------------

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; ++i)
  {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; ++b)
    {
      if (crc & 0x8000)
        crc = (uint16_t)((crc << 1) ^ 0x1021);
      else
        crc = (uint16_t)(crc << 1);
    }
  }
  return crc;
}

// -----------------------------------------------------------------------------
//  VÁLTOZÁSDETEKTÁLÁS – PACK SZINTEN
// -----------------------------------------------------------------------------

static bool packs_equal(const uint8_t *a, const uint8_t *b, uint8_t len)
{
  for (uint8_t i = 0; i < len; ++i)
  {
    if (a[i] != b[i])
      return false;
  }
  return true;
}

// -----------------------------------------------------------------------------
//  SX1278 – ALAP SPI/REG KEZELÉS
// -----------------------------------------------------------------------------

static uint8_t lora_read_reg(uint8_t addr)
{
  digitalWrite(LORA_SS, LOW);
  SPI.transfer(addr & 0x7F);
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(LORA_SS, HIGH);
  return val;
}

static void lora_write_reg(uint8_t addr, uint8_t val)
{
  digitalWrite(LORA_SS, LOW);
  SPI.transfer(addr | 0x80);
  SPI.transfer(val);
  digitalWrite(LORA_SS, HIGH);
}

// FIFO-ba több byte írása
static void lora_write_fifo(const uint8_t *data, uint8_t len)
{
  digitalWrite(LORA_SS, LOW);
  SPI.transfer(REG_FIFO | 0x80);
  for (uint8_t i = 0; i < len; ++i)
  {
    SPI.transfer(data[i]);
  }
  digitalWrite(LORA_SS, HIGH);
}

static void lora_set_lora_opmode(uint8_t mode)
{
  uint8_t op = LONG_RANGE_MODE | (mode & 0x07);
  lora_write_reg(REG_OP_MODE, op);
}

static void lora_set_opmode(uint8_t mode)
{
  uint8_t op = lora_read_reg(REG_OP_MODE);
  op = (op & 0xF8) | (mode & 0x07);
  lora_write_reg(REG_OP_MODE, op);
}

// -----------------------------------------------------------------------------
//  SX1278 – INIT (433 MHz, LoRa, SF12, BW125, CR4/8, +20 dBm, preamble 8)
// -----------------------------------------------------------------------------

void IRAM_ATTR onTxDone();
static bool lora_begin()
{
  pinMode(LORA_SS, OUTPUT);
  pinMode(LORA_RST, OUTPUT);
  pinMode(LORA_DIO0, INPUT);
  attachInterrupt(digitalPinToInterrupt(LORA_DIO0), onTxDone, RISING);

  pinMode(CAM_LED_PIN, OUTPUT);
  digitalWrite(CAM_LED_PIN, LOW);

  digitalWrite(LORA_SS, HIGH);

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setFrequency(8000000);

  // Reset
  digitalWrite(LORA_RST, LOW);
  delay(10);
  digitalWrite(LORA_RST, HIGH);
  delay(10);

  // LoRa mód, sleep → standby
  lora_write_reg(REG_OP_MODE, LONG_RANGE_MODE | MODE_SLEEP);
  delay(10);
  lora_set_lora_opmode(MODE_STDBY);
  delay(10);

  // Verzió ellenőrzés
  uint8_t ver = lora_read_reg(REG_VERSION);
  if (ver == 0x00 || ver == 0xFF)
  {
    return false; // nincs modul / hibás
  }

  // Frekvencia: beállítás LORA_FREQ_HZ és LORA_OFFSET_HZ alapján
  // FRF = Freq / Fstep, Fstep = 32MHz / 2^19 ≈ 61.035 Hz
  uint64_t targetFreq = (uint64_t)LORA_FREQ_HZ + (int64_t)LORA_OFFSET_HZ;
  double fstep = 32000000.0 / 524288.0; // datasheet: Fxtal/2^19
  uint32_t frf = (uint32_t)(targetFreq / fstep);

  lora_write_reg(REG_FRF_MSB, (uint8_t)((frf >> 16) & 0xFF));
  lora_write_reg(REG_FRF_MID, (uint8_t)((frf >> 8) & 0xFF));
  lora_write_reg(REG_FRF_LSB, (uint8_t)(frf & 0xFF));

  // PA konfiguráció: PA_BOOST, ~+20 dBm
  // 17–20 dBm környéke: PaConfig ≈ 0x8F, PaDac ≈ 0x87 (20 dBm mód)
  lora_write_reg(REG_PA_CONFIG, 0x8F); // PA_BOOST, max power
  lora_write_reg(REG_PA_DAC, 0x87);    // +20 dBm mód

  // OCP limit módosítása (0x0F ≠ teljes kikapcsolás, hanem magas áramhatár)
  lora_write_reg(REG_OCP, 0x0F); // OCP OFF – teljes áram a PA-nak

  // LNA boost on
  lora_write_reg(REG_LNA, 0x23);

  // ModemConfig1: BW=125 kHz, CR=4/8, explicit header
  // BW=125k → 0b0111, CR=4/8 → 0b100, explicit header=0
  // 0b0111 1000 = 0x78
  lora_write_reg(REG_MODEM_CONFIG1, 0x78);

  // ModemConfig2: SF12, CRC ON, SymbTimeoutMSB=0
  // SF12 → 0b1100, CRC ON → bit2=1
  // 0b1100 0100 = 0xC4
  lora_write_reg(REG_MODEM_CONFIG2, 0xC4);

  // Symbol timeout (LSB) – nagy érték, hogy ne timeout-oljon RX-ben
  lora_write_reg(REG_SYMB_TIMEOUT_LSB, 0x64);

  // ModemConfig3: LowDataRateOptimize ON (SF11/12 + BW125), AGC Auto ON
  lora_write_reg(REG_MODEM_CONFIG3, 0x0C);

  // Preamble = 8
  lora_write_reg(REG_PREAMBLE_MSB, 0x00);
  lora_write_reg(REG_PREAMBLE_LSB, 8);

  // FIFO base address
  lora_write_reg(REG_FIFO_TX_BASE, 0x00);
  lora_write_reg(REG_FIFO_RX_BASE, 0x00);

  // DIO0 → TxDone
  lora_write_reg(REG_DIO_MAPPING1, 0x40);

  // Standby
  lora_set_lora_opmode(MODE_STDBY);
  return true;
}

// -----------------------------------------------------------------------------
//  SX1278 – KÜLDÉS (NEM BLOKKOLÓ, TXDONE-t loop-ban kezeljük)
// -----------------------------------------------------------------------------

// --- Globális állapot a nem blokkoló TX-hez ---
static bool g_tx_in_progress = false;
static uint32_t g_tx_start_us = 0;
static volatile bool g_tx_done_irq = false;
void IRAM_ATTR onTxDone()
{
  g_tx_done_irq = true;
}

static void lora_send_packet(const uint8_t *data, uint8_t len)
{
  // Fail-safe payload clamp
  if (len > LORA_MAX_PAYLOAD)
    len = LORA_MAX_PAYLOAD;

  // Send telemetry frame to serial port (USB CDC) for ground station monitoring
  noInterrupts();
  Serial.write(data, len);
  Serial.flush();
  interrupts();

  // Rövid villanás jelzés – nem blokkoló
  digitalWrite(CAM_LED_PIN, HIGH);
  g_tx_start_us = micros(); // LED off handled in loop()

  // TX FIFO pointer
  lora_write_reg(REG_FIFO_ADDR_PTR, 0x00);
  // Payload hossza
  lora_write_reg(REG_PAYLOAD_LENGTH, len);
  // FIFO feltöltése
  lora_write_fifo(data, len);

  // IRQ flagek törlése
  lora_write_reg(REG_IRQ_FLAGS, 0xFF);

  // TX mód
  lora_set_lora_opmode(MODE_TX);
  g_tx_in_progress = true;
}

// -----------------------------------------------------------------------------
//  TELEMETRIA PACK GYŰJTÉS, BITMASK, CSOMAGKÉSZÍTÉS
// -----------------------------------------------------------------------------

static uint32_t g_last_tele_ms = 0; // 2 Hz telemetria minta időzítése

static void telemetry_add_sample(const TeleSample &ts)
{
  if (g_tele_count >= MAX_SAMPLES_PER_PACKET)
  {
    telemetry_flush_packet();
  }

  uint8_t idx = (g_tele_count < MAX_SAMPLES_PER_PACKET)
                    ? g_tele_count
                    : (MAX_SAMPLES_PER_PACKET - 1);

  uint8_t curr_pack[8]; // Max possible size
  uint8_t curr_pack_len = encode_dynamic_pack(ts, curr_pack);

  bool changed = true;
  if (g_have_prev_sample)
  {
    changed = !packs_equal(curr_pack, g_prev_pack_global, curr_pack_len);
  }
  // Save previous pack
  for (uint8_t i = 0; i < curr_pack_len; ++i)
  {
    g_prev_pack_global[i] = curr_pack[i];
  }
  // If previous was longer, clear any trailing bytes
  for (uint8_t i = curr_pack_len; i < sizeof(g_prev_pack_global); ++i)
  {
    g_prev_pack_global[i] = 0;
  }

  g_have_prev_sample = true;
  g_tele_buf[idx] = ts;
  g_tele_flag_changed[idx] = changed;

  if (g_tele_count < MAX_SAMPLES_PER_PACKET)
  {
    g_tele_count++;
  }
}

// LoRa csomag összeállítása és elküldése, ha van minta
static void sample_send_debug(const TeleSample &ts, uint16_t MET)
{
  uint8_t dbg[32];
  uint8_t d = 0;

  dbg[d++] = 0xA6;
  dbg[d++] = (uint8_t)(MET & 0xFF);
  dbg[d++] = (uint8_t)(MET >> 8);

  uint8_t *pt = (uint8_t *)(&ts.tempC);
  uint8_t *ph = (uint8_t *)(&ts.rh);
  uint8_t *pp = (uint8_t *)(&ts.press_hPa);

  for (int i = 0; i < 4; i++)
    dbg[d++] = pt[i];
  for (int i = 0; i < 4; i++)
    dbg[d++] = ph[i];
  for (int i = 0; i < 4; i++)
    dbg[d++] = pp[i];

  noInterrupts(); // prevent ISR from interrupting USB CDC frame
  Serial.write(dbg, d);
  Serial.flush(); // force immediate transmission
  interrupts();   // re-enable IRQs
}

static void telemetry_flush_packet()
{
  // We only transmit when the packet is complete (8 samples) and TX is idle.
  if (g_tele_count < MAX_SAMPLES_PER_PACKET)
    return;
  if (g_tx_in_progress)
    return;

  // ---------------------------------------------------------------------------
  //  Frame döntés: FULL keyframe vs DELTA frame
  //  - Receiver bármikor bekapcsolható → időnként biztosan küldünk FULL-t
  //  - DELTA csak akkor mehet, ha van előző abszolút referencia és minden delta tartományon belül
  // ---------------------------------------------------------------------------

  // Compute base MET for the first sample in this packet.
  uint16_t base_MET = (uint16_t)(g_MET - (MAX_SAMPLES_PER_PACKET - 1));
  if (base_MET > g_MET)
    base_MET = 0;

  // FLAGS alap (TIME_VALID + SENSOR_VALID + RESET_EVENT)
  uint8_t flags_base = 0;
  flags_base |= (1u << 2); // TIME_VALID
  flags_base |= (1u << 3); // SENSOR_VALID
  if (g_reset_event_pending)
    flags_base |= (1u << 4); // RESET_EVENT

  // Changed detektálás (csomag szinten)
  bool any_changed = false;
  for (uint8_t i = 0; i < MAX_SAMPLES_PER_PACKET; ++i)
  {
    if (g_tele_flag_changed[i])
    {
      any_changed = true;
      break;
    }
  }

  // Ha semmi sem változott, akkor alapból nem küldünk – DE kulcsframe periódusra azért figyelünk.
  if (!any_changed && !g_reset_event_pending)
  {
    if (g_packets_since_keyframe < KEYFRAME_PERIOD_PACKETS)
    {
      g_tele_count = 0;
      return;
    }
  }

  // Kulcsframe kényszerek
  bool force_keyframe = false;
  if (!g_have_prev_abs30) force_keyframe = true;                 // nincs referencia
  if (g_reset_event_pending) force_keyframe = true;              // reset esemény → FULL
  if (g_packets_since_keyframe >= KEYFRAME_PERIOD_PACKETS) force_keyframe = true; // időszakos FULL

  // --- Próbáljuk DELTA-t, ha lehet ---
  if (!force_keyframe)
  {
    uint8_t dframe[DELTA_FRAME_LEN];
    bool ok = build_delta_frame(dframe, base_MET, flags_base, g_frame_seq);
    if (ok)
    {
      lora_send_packet(dframe, DELTA_FRAME_LEN);
      g_tele_count = 0;

      // referencia frissítése: a csomag utolsó mintája legyen a következő delta alapja
      g_prev_abs30 = encode_sample_30bit_lsb(g_tele_buf[MAX_SAMPLES_PER_PACKET - 1]) & ((1u << 30) - 1);
      g_have_prev_abs30 = true;

      g_frame_seq++;
      g_packets_since_keyframe++;
      g_reset_event_pending = false;
      return;
    }

    // ha a delta nem fér bele a tartományba → FULL-ra esünk vissza
    force_keyframe = true;
  }

  // --- FULL keyframe (38 byte) ---
  // 0:  SYNC (0xA5)
  // 1-2: base MET (little-endian)
  // 3:  FLAGS (FULL_STATE=1)
  // 4:  SEQUENCE COUNTER
  // 5:  VALIDITY (bit0..2)
  // 6-35: DATA (30 bytes = 8 samples x 30 bits)
  // 36-37: CRC16-CCITT over bytes 0..35
  uint8_t frame[FULL_FRAME_LEN];
  for (uint8_t i = 0; i < sizeof(frame); ++i)
    frame[i] = 0;

  frame[0] = SYNC_FULL;
  frame[1] = (uint8_t)(base_MET & 0xFF);
  frame[2] = (uint8_t)(base_MET >> 8);

  // FULL_STATE=1, DELTA_STATE=0
  uint8_t flags = flags_base;
  flags |= (1u << 0);
  flags &= ~(1u << 1);
  frame[3] = flags;

  frame[4] = g_frame_seq;

  // VALIDITY: egyszerű, de determinisztikus (ha bármi változott, akkor mindhárom valid)
  uint8_t validity = 0;
  if (any_changed)
  {
    validity |= (1u << 0);
    validity |= (1u << 1);
    validity |= (1u << 2);
  }
  frame[5] = validity;

  // Data block: write 8×30-bit samples starting at byte 6
  uint16_t bitpos = 6u * 8u;
  for (uint8_t i = 0; i < MAX_SAMPLES_PER_PACKET; ++i)
  {
    uint32_t v30 = encode_sample_30bit_lsb(g_tele_buf[i]) & ((1u << 30) - 1);
    bitstream_write_lsb(frame, bitpos, v30, 30);
  }

  uint16_t crc = crc16_ccitt(frame, 36);
  frame[36] = (uint8_t)(crc & 0xFF);
  frame[37] = (uint8_t)(crc >> 8);

  lora_send_packet(frame, FULL_FRAME_LEN);
  g_tele_count = 0;

  // FULL referencia frissítése
  g_prev_abs30 = encode_sample_30bit_lsb(g_tele_buf[MAX_SAMPLES_PER_PACKET - 1]) & ((1u << 30) - 1);
  g_have_prev_abs30 = true;

  g_last_keyframe_seq = g_frame_seq;
  g_packets_since_keyframe = 0;

  g_frame_seq++;
  g_reset_event_pending = false;
}

// -----------------------------------------------------------------------------
//  SETUP / LOOP
// -----------------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  delay(1000);

  // Serial.println("CanSat BME280 + LoRa telemetria indul...");

  // NVS / MET visszatoltes
  g_prefs.begin("cansat", false);
  uint16_t storedMet = g_prefs.getUShort("met", 0);

  if (storedMet == 0)
  {
    // Teljes áramtalanítás utáni indulás → új repülés
    g_MET = 0;
  }
  else
  {
    // WDT / szoftveres reset → repülés folytatása
    g_MET = storedMet;
  }

  // Watchdog inicializalas (3s timeout, reset engedelyezve)
  esp_task_wdt_init(3, true);
  esp_task_wdt_add(NULL); // jelenlegi task

  // BME280 KI KAPCSOLVA – TESZT ÜZEMMÓD (COUNTER)
  g_sensor_ready = true;      // azonnal FLIGHT mód engedélyezése
  g_sys_state = SYS_FLIGHT;

  if (!lora_begin())
  {
    // Serial.println("LoRa modul NEM TALALHATO!");
  }
  else
  {
    // Serial.println("LoRa OK");
  }

  g_last_MET_ms = millis();
  g_last_tele_ms = millis();
  g_last_met_store_ms = millis();

  // Delta rendszer indulás: első csomag biztosan FULL legyen
  g_have_prev_abs30 = false;
  g_packets_since_keyframe = KEYFRAME_PERIOD_PACKETS; // hogy az első kész csomagnál FULL menjen
  g_last_keyframe_seq = 0;
}

void loop()
{
  uint32_t now = millis();

  // 1) BME280 állapotgép: folyamatos mérés forced móddal, max. oversamplinggel
  // bme_poll();

  // 1/b) Rendszer állapot: ha már van legalább egy érvényes szenzorminta, indulhat a MET (FLIGHT mód)
  if (g_sys_state == SYS_WAIT_SENSORS && g_sensor_ready)
  {
    g_sys_state = SYS_FLIGHT;
    g_last_MET_ms = now;
    g_last_tele_ms = now;
    g_last_met_store_ms = now;
    // Serial.println("[SYS] MET START – szenzorok keszek, FLIGHT mod.");
  }

  // 2) MET tick – 0,5 s-onként nő, de csak FLIGHT módban
  if (g_sys_state == SYS_FLIGHT)
  {
    if (now - g_last_MET_ms >= 500)
    {
      g_last_MET_ms += 500;
      g_MET++;
    }
  }

  // 3) 2 Hz telemetria minta (0,5 s-onként) – mozgóátlag a nyers bufferből
  if (now - g_last_tele_ms >= 500)
  {
    g_last_tele_ms += 500;

    TeleSample ts;
    generate_counter_sample(ts);
    telemetry_add_sample(ts);
  }


  // MET periodikus mentese NVS-be (10 masodpercenkent), csak FLIGHT modban
  if (g_sys_state == SYS_FLIGHT && g_MET > 0)
  {
    if (now - g_last_met_store_ms >= 10000)
    {
      g_last_met_store_ms = now;
      g_prefs.putUShort("met", g_MET);
    }
  }

  // --- Nem blokkoló TX kezelés ---
  if (g_tx_in_progress)
  {
    // LED OFF 5000us után
    if (digitalRead(CAM_LED_PIN) == HIGH && (micros() - g_tx_start_us) > 5000)
    {
      digitalWrite(CAM_LED_PIN, LOW);
    }

    // TX DONE via IRQ flag
    if (g_tx_done_irq)
    {
      g_tx_done_irq = false;
      lora_write_reg(REG_IRQ_FLAGS, 0xFF);
      lora_set_lora_opmode(MODE_STDBY);
      g_tx_in_progress = false;
    }
  }

  // Watchdog reset – ha a loop lefut rendesen, nem indul ujra az ESP32
  esp_task_wdt_reset();
}

// -----------------------------------------------------------------------------
//  COUNTER-ALAPÚ TESZT ADAT (BME HELYETT)
// -----------------------------------------------------------------------------

static uint16_t cnt_T = 0;   // 0..1279  (-40.0 .. +87.9 C, 0.1C)
static uint8_t  cnt_H = 0;   // 0..100   (%)
static uint16_t cnt_P = 0;   // 0..2550  (822.0 .. 1077.0 hPa, 0.1hPa)
static int8_t dir_T = 1;
static int8_t dir_H = 1;
static int8_t dir_P = 1;

static void generate_counter_sample(TeleSample &ts)
{
  ts.tempC = -40.0f + (cnt_T * 0.1f);
  ts.rh = (float)cnt_H;
  ts.press_hPa = 822.0f + (cnt_P * 0.1f);

  // Ping-pong (fel-le) számlálás: nincs nagy ugrás (pl. 100 -> 0)
  // T: 0..1279
  if (dir_T > 0)
  {
    if (cnt_T >= 1279) { dir_T = -1; } else { cnt_T++; }
  }
  else
  {
    if (cnt_T == 0) { dir_T = 1; } else { cnt_T--; }
  }

  // H: 0..100
  if (dir_H > 0)
  {
    if (cnt_H >= 100) { dir_H = -1; } else { cnt_H++; }
  }
  else
  {
    if (cnt_H == 0) { dir_H = 1; } else { cnt_H--; }
  }

  // P: 0..2550
  if (dir_P > 0)
  {
    if (cnt_P >= 2550) { dir_P = -1; } else { cnt_P++; }
  }
  else
  {
    if (cnt_P == 0) { dir_P = 1; } else { cnt_P--; }
  }
}