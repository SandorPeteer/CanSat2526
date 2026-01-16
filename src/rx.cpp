#include <string.h>
#include <Arduino.h>

#include <SPI.h>
#include "esp_task_wdt.h"
#include <stdio.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <math.h>

// ---------------------------------------------------------------------------
//  Prioritás: nyers LoRa payload továbbítás Serial felé.
// ---------------------------------------------------------------------------

// ----------------- RINGBUFFER DEFINÍCIÓ -----------------
struct LoRaFrame {
    uint8_t len;
    uint8_t data[64];
    uint32_t timestamp;
    uint8_t meta; // bit0: PHY_CRC_OK
};

static volatile LoRaFrame rb[32];
static volatile uint8_t rb_head = 0;
static volatile uint8_t rb_tail = 0;
static volatile uint32_t rb_dropped = 0;

// Decoder stat counters (csak diagnosztika / OLED)
static uint32_t g_cnt_full_ok = 0;
static uint32_t g_cnt_delta_ok = 0;
static uint32_t g_cnt_len_err = 0;
static uint32_t g_cnt_crc_err = 0;
static uint32_t g_cnt_sync_err = 0;
static uint32_t g_cnt_dsyn = 0;
static uint32_t g_cnt_rng_err = 0;
static uint32_t g_cnt_phy_crc_err = 0;
static uint32_t g_cnt_seq_disc = 0;
static uint32_t g_cnt_seq_lost = 0;

// AFC státuszjelzés (csillag az OLED-en, ha offset mentve lett)
static bool offset_saved_flag = false;
static uint32_t star_until = 0;

// RF peak hold (kulcsframe-ig): a legjobb (max) RSSI/SNR értékek megőrzése
static bool g_peak_valid = false;
static int16_t g_peak_rssi_dbm = -150;
static int16_t g_peak_snr_qdb = (int16_t)lroundf(-32.0f * 4.0f);
static int32_t g_peak_fei_hz = 0;       // peak |FEI|, signed value held
static int32_t g_peak_offset_hz = 0;    // peak |OFFSET|, signed value held

static const uint32_t RX_LED_MS = 200;

// "Instant RSSI" (RegRssiValue) zajmintavétel késleltetése RX után,
// hogy ne a csomag utórezgését mérjük.
static const uint32_t NOISE_SAMPLE_DELAY_MS = 30;


static inline bool rb_is_full() {
    return ((rb_head + 1) & 31) == rb_tail;
}

static inline bool rb_is_empty() {
    return rb_head == rb_tail;
}

static inline void rb_push(const uint8_t *buf, uint8_t len)
{
    if (rb_is_full()) { rb_dropped++; return; }

    uint8_t idx = rb_head;
    rb[idx].len = len;
    memcpy((void*)rb[idx].data, buf, len);
    rb[idx].timestamp = millis();
    rb[idx].meta = 0x01; // PHY_CRC_OK

    rb_head = (idx + 1) & 31;
}

static inline void rb_push_meta(const uint8_t *buf, uint8_t len, uint8_t meta)
{
    if (rb_is_full()) { rb_dropped++; return; }

    uint8_t idx = rb_head;
    rb[idx].len = len;
    memcpy((void*)rb[idx].data, buf, len);
    rb[idx].timestamp = millis();
    rb[idx].meta = meta;

    rb_head = (idx + 1) & 31;
}

static inline bool rb_pop(LoRaFrame &out)
{
    if (rb_is_empty()) return false;

    uint8_t idx = rb_tail;
    out.len = rb[idx].len;
    out.timestamp = rb[idx].timestamp;
    memcpy(out.data, (const void*)rb[idx].data, out.len);
    out.meta = rb[idx].meta;

    rb_tail = (idx + 1) & 31;
    return true;
}

// -----------------------------------------------------------------------------
//  HARDVER PIN DEFINÍCIÓK (EZEKET A SAJÁT BEKÖTÉSEDHEZ IGAZÍTSD!)
// -----------------------------------------------------------------------------

// I2C BME280 – ESP32-CAM-on általában:

// SX1278 LoRa (tx.cpp bekötésével azonos)
#define LORA_SCK 18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SS 5
#define LORA_RST 26
#define LORA_DIO0 25
#define LORA_DIO1 36
#define LORA_DIO2 33

#define CAM_LED_PIN 4

// -----------------------------------------------------------------------------
//  LoRa frekvencia beállítások (globális)
// ---------------------------------------------------------------------------
static uint32_t LORA_FREQ_HZ = 433200000; // alap vivőfrekvencia (Hz)
static int32_t LORA_OFFSET_HZ = 0;     // kézi offset (Hz)
static int32_t last_saved_offset = 0;

int32_t fei_accumulator = 0;
uint8_t fei_count = 0;

static uint32_t led_on_until = 0;


struct TeleSample
{
  float tempC;
  float rh;
  float press_hPa;
};

// ---------------------------------------------------------------------------
//  FRAME DEFINÍCIÓK (TX kód alapján)
// ---------------------------------------------------------------------------

static const uint8_t SYNC_FULL  = 0xA5; // FULL keyframe
static const uint8_t SYNC_DELTA = 0xA4; // DELTA frame

static const uint8_t FULL_FRAME_LEN  = 38;
static const uint8_t DELTA_FRAME_LEN = 23;

static const uint8_t MAX_SAMPLES_PER_PACKET = 8;

// FLAGS bitek (TX logika szerint)
// bit0: FULL_STATE
// bit1: DELTA_STATE
// bit2: TIME_VALID
// bit3: SENSOR_VALID
// bit4: RESET_EVENT

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
#define REG_FIFO_RX_CURRENT_ADDR 0x10
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

#define REG_FEI_MSB 0x28
#define REG_FEI_MID 0x29
#define REG_FEI_LSB 0x2A
#define REG_PKT_SNR_VALUE 0x19
#define REG_PKT_RSSI_VALUE 0x1A
#define REG_RSSI_VALUE 0x1B
#define REG_MODEM_STAT 0x18
// LoRa packet counters (SX1276/77/78/79 family)
#define REG_RX_HEADER_CNT_MSB 0x14
#define REG_RX_HEADER_CNT_LSB 0x15
#define REG_RX_PACKET_CNT_MSB 0x16
#define REG_RX_PACKET_CNT_LSB 0x17

// SX1278 @ 433 MHz uses the LF port (137–525 MHz) → RSSI offset -164 dBm.
// (HF port 862–1020 MHz would use -157 dBm.)
static const int32_t LORA_RSSI_OFFSET_DBM = -164;

static const double LORA_FSTEP = 32000000.0 / 524288.0;           // 32 MHz / 2^19
static const double LORA_FEI_COEFF = (double)(1UL << 24) / 32000000.0; // 2^24 / 32 MHz

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


// LoRa op-mode bitek
#define LONG_RANGE_MODE 0x80
#define MODE_SLEEP 0x00
#define MODE_STDBY 0x01
#define MODE_TX 0x03
#define MODE_RXCONTINUOUS 0x05
#define IRQ_RX_DONE 0x40
#define IRQ_PAYLOAD_CRC_ERROR 0x20

// Csomagméret
static const uint8_t LORA_MAX_PAYLOAD = 48;

// -----------------------------------------------------------------------------
//  VÁLTOZÁSDETEKTÁLÁS – PACK SZINTEN
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//  SX1278 – ALAP SPI/REG KEZELÉS
// -----------------------------------------------------------------------------

static void lora_read_fifo(uint8_t *data, uint8_t len)
{
  digitalWrite(LORA_SS, LOW);
  // Read from FIFO register (address 0x00, MSB=0 for read)
  SPI.transfer(REG_FIFO & 0x7F);
  for (uint8_t i = 0; i < len; ++i)
  {
    data[i] = SPI.transfer(0x00);
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
//  LoRa FEI (Frequency Error Indicator) – hibajel visszaszámítása Hz-be
// -----------------------------------------------------------------------------
static double lora_get_freq_error_hz(double bw_khz)
{
  // RegFeiMsb/Mid/Lsb – LoRa üzemmódban 20 bites, előjeles érték
  uint8_t fe_msb = lora_read_reg(REG_FEI_MSB);
  uint8_t fe_mid = lora_read_reg(REG_FEI_MID);
  uint8_t fe_lsb = lora_read_reg(REG_FEI_LSB);

  int32_t fei_raw = (int32_t)(((uint32_t)(fe_msb & 0x0F) << 16) |
                              ((uint32_t)fe_mid << 8) |
                              (uint32_t)fe_lsb);

  // 20 bites kétkomplementes kiterjesztése 32 bitre
  if (fei_raw & 0x80000)
  {
    fei_raw |= 0xFFF00000;
  }

  // Datasheet képlet (LoRa mód):
  // F_err_Hz = LoRaFeiValue * 2^24 / 32e6 * (BW_kHz / 500)
  double freqErrHz = (double)fei_raw * LORA_FEI_COEFF * (bw_khz / 500.0);
  return freqErrHz;
}

static double lora_get_bw_khz()
{
  uint8_t mc1 = lora_read_reg(REG_MODEM_CONFIG1);
  uint8_t bw_bits = (mc1 >> 4) & 0x0F;
  switch (bw_bits)
  {
    case 0: return 7.8;
    case 1: return 10.4;
    case 2: return 15.6;
    case 3: return 20.8;
    case 4: return 31.25;
    case 5: return 41.7;
    case 6: return 62.5;
    case 7: return 125.0;
    case 8: return 250.0;
    case 9: return 500.0;
    default: return 125.0;
  }
}

static float lora_get_snr_db()
{
  int8_t raw = (int8_t)lora_read_reg(REG_PKT_SNR_VALUE);
  return ((float)raw) / 4.0f; // 0.25 dB steps
}

static void afc_update(double freqErrHz)
{
  // --- Only allow NVS saving if this static flag is true ---
  static bool allowed_to_save = true;

  // --- Automatikus frekvencia finomhangolás (AFC) ---
  // Csak akkor korrigálunk, ha az eltérés érdemi (500 Hz < |err| < 40 kHz),
  // így a kvantálási zajra és nagyon kis ingadozásokra nem reagálunk.
  double errAbs = (freqErrHz < 0.0) ? -freqErrHz : freqErrHz;
  if (errAbs > 500.0 && errAbs < 40000.0)
  {
    int32_t stepHz;
    if (freqErrHz >= 0.0)
      stepHz = (int32_t)(freqErrHz + 0.5);
    else
      stepHz = (int32_t)(freqErrHz - 0.5);

    LORA_OFFSET_HZ -= stepHz;

    // Offset korlátozása ±60 kHz tartományra
    if (LORA_OFFSET_HZ > 60000)  LORA_OFFSET_HZ = 60000;
    if (LORA_OFFSET_HZ < -60000) LORA_OFFSET_HZ = -60000;

    // Új FRF beállítása: F_target = LORA_FREQ_HZ + LORA_OFFSET_HZ
    uint64_t targetFreq = (uint64_t)LORA_FREQ_HZ + (int64_t)LORA_OFFSET_HZ;
    uint32_t frf = (uint32_t)(targetFreq / LORA_FSTEP);

    // Átmenetileg STDBY módba lépünk a PLL állításához, majd vissza RX-be
    lora_set_lora_opmode(MODE_STDBY);
    lora_write_reg(REG_FRF_MSB, (uint8_t)((frf >> 16) & 0xFF));
    lora_write_reg(REG_FRF_MID, (uint8_t)((frf >> 8) & 0xFF));
    lora_write_reg(REG_FRF_LSB, (uint8_t)(frf & 0xFF));
    lora_set_lora_opmode(MODE_RXCONTINUOUS);
  }

  // --- NVS SAVE: ha már közel vagyunk (|FEI| < 200 Hz), és az offset érdemben változott ---
  if (allowed_to_save && fabs(freqErrHz) < 200.0)
  {
      int32_t current_offset = LORA_OFFSET_HZ;

      // Feltétel: legalább 20% változás vagy minimum 1500 Hz eltérés
      int32_t diff   = current_offset - last_saved_offset;
      int32_t thresh = (abs(last_saved_offset) * 20) / 100; // 20%
      if (thresh < 1500) thresh = 1500;

      if (abs(diff) > thresh)
      {
          nvs_handle_t h2;
          if (nvs_open("lora", NVS_READWRITE, &h2) == ESP_OK)
          {
              nvs_set_i32(h2, "offset", current_offset);
              nvs_commit(h2);
              nvs_close(h2);
              last_saved_offset = current_offset;
              offset_saved_flag = true;
              star_until = millis() + 5000; // 5 másodpercig jelenjen meg a csillag
          }
          // Addig nem mentünk újra, amíg a hiba újra 200 Hz fölé nem megy
          allowed_to_save = false;
      }
  }

  // --- Reset allowed_to_save, ha a hiba újra 200 Hz fölé megy ---
  if (fabs(freqErrHz) > 200.0)
  {
      allowed_to_save = true;
  }
}

// ---------------------------------------------------------------------------
//  Serial meta frame (RF metrics) – LoRa payload után küldve (bináris)
// ---------------------------------------------------------------------------

static uint8_t crc8_xor(const uint8_t *data, uint8_t len)
{
  uint8_t c = 0;
  for (uint8_t i = 0; i < len; ++i) c ^= data[i];
  return c;
}

static uint8_t clamp_u8(int v, int lo, int hi)
{
  if (v < lo) v = lo;
  if (v > hi) v = hi;
  return (uint8_t)v;
}

static uint8_t link_score_0_100(int rssi_dbm, float snr_db, int32_t fei_abs_hz, uint32_t phy_crc_err_recent, uint32_t seq_lost_recent)
{
  // RSSI: -120..-60 => 0..100
  int rssiScore = (rssi_dbm + 120) * 100 / 60;
  rssiScore = (rssiScore < 0) ? 0 : (rssiScore > 100 ? 100 : rssiScore);

  // SNR: -20..+10 => 0..100
  int snrScore = (int)lroundf((snr_db + 20.0f) * (100.0f / 30.0f));
  snrScore = (snrScore < 0) ? 0 : (snrScore > 100 ? 100 : snrScore);

  // FEI stability: 0..5000Hz => 100..0  (felette 0)
  int feiScore = 100 - (int)((fei_abs_hz * 100) / 5000);
  feiScore = (feiScore < 0) ? 0 : (feiScore > 100 ? 100 : feiScore);

  // Loss penalties (heurisztika, “ESA-s” konzervatív): ha van hiba, vágjunk
  int penalty = 0;
  if (phy_crc_err_recent > 0) penalty += 20;
  if (seq_lost_recent > 0) penalty += 10;

  int score = (45 * snrScore + 35 * rssiScore + 20 * feiScore) / 100;
  score -= penalty;
  if (score < 0) score = 0;
  if (score > 100) score = 100;
  return (uint8_t)score;
}

static uint8_t build_rf_meta_frame(uint8_t *out, uint8_t maxLen, uint8_t payload_len, const LoRaFrame &f);

// -----------------------------------------------------------------------------
//  SX1278 – INIT (433 MHz, LoRa, SF12, BW125, CR4/8, +20 dBm, preamble 8)
// -----------------------------------------------------------------------------

void IRAM_ATTR onRxDone();
static volatile bool g_rx_done = false;
static const char* g_lora_status = "OK";
// --- Interpolation buffer for outlier handling ---
static bool lora_begin()
{
  pinMode(LORA_SS, OUTPUT);
  pinMode(LORA_RST, OUTPUT);
  pinMode(LORA_DIO0, INPUT);
  attachInterrupt(digitalPinToInterrupt(LORA_DIO0), onRxDone, RISING);

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

  // DIO0 → RxDone
  lora_write_reg(REG_DIO_MAPPING1, 0x00);

  // Standby
  lora_set_lora_opmode(MODE_STDBY);
  // Continuous RX mode
  lora_set_lora_opmode(0x05); // MODE_RXCONTINUOUS
  return true;
}

// -----------------------------------------------------------------------------
//  SX1278 – KÜLDÉS (NEM BLOKKOLÓ, TXDONE-t loop-ban kezeljük)
// -----------------------------------------------------------------------------

void IRAM_ATTR onRxDone() {
    uint8_t irq = lora_read_reg(REG_IRQ_FLAGS);

    if (irq & IRQ_RX_DONE) {
        bool phy_crc_ok = (irq & IRQ_PAYLOAD_CRC_ERROR) == 0;
        uint8_t len = lora_read_reg(REG_RX_NB_BYTES);
        uint8_t addr = lora_read_reg(REG_FIFO_RX_CURRENT_ADDR);
        lora_write_reg(REG_FIFO_ADDR_PTR, addr);

        if (len > 64) len = 64;

        uint8_t tmp[64];
        lora_read_fifo(tmp, len);

        if (!phy_crc_ok) g_cnt_phy_crc_err++;
        rb_push_meta(tmp, len, phy_crc_ok ? 0x01 : 0x00);

        led_on_until = millis() + RX_LED_MS;
        digitalWrite(CAM_LED_PIN, HIGH);
    }

    lora_write_reg(REG_IRQ_FLAGS, 0xFF);
}

// ---------------------------------------------------------------------------
//  CRC16-CCITT (poly 0x1021, init 0xFFFF) – frame integrity
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
//  Bitstream helpers (LSB-first, ugyanaz mint TX oldalon)
// ---------------------------------------------------------------------------

static uint32_t bitstream_read_lsb(const uint8_t *src, uint16_t &bitpos, uint8_t bits)
{
  uint32_t v = 0;
  for (uint8_t i = 0; i < bits; ++i)
  {
    uint16_t byteIndex = bitpos >> 3;
    uint8_t bitIndex = bitpos & 7;
    uint8_t b = (src[byteIndex] >> bitIndex) & 1u;
    v |= ((uint32_t)b << i);
    bitpos++;
  }
  return v;
}

static inline int32_t sign_extend(uint32_t v, uint8_t bits)
{
  if (bits == 0 || bits >= 32) return (int32_t)v;
  uint32_t m = 1u << (bits - 1u);
  uint32_t mask = (1u << bits) - 1u;
  v &= mask;
  return (v ^ m) - m;
}

// ---------------------------------------------------------------------------
//  30-bit abszolút minta dekódolás
// ---------------------------------------------------------------------------

struct AbsCodes
{
  uint16_t T_code = 0;     // 0..1279
  uint8_t  RH_code = 0;    // 0..100
  uint16_t P_code12 = 0;   // 0..2550  (0.1 hPa, 822.0 offset nélkül)
};

struct RxState
{
  AbsCodes codes;
  bool valid = false;
  bool have_seq = false;
  uint8_t last_seq = 0;
};

static RxState g_rx;

// ---------------------------------------------------------------------------
//  Telemetria állapot + history (OLED trendekhez)
// ---------------------------------------------------------------------------

struct TelemetryState
{
  bool have_last = false;
  AbsCodes last_codes;
  uint16_t last_met = 0;
  uint16_t last_base_met = 0;
  uint8_t last_sync = 0;
  uint8_t last_flags = 0;
  uint8_t last_validity = 0;
  uint8_t last_seq = 0;
  uint8_t last_ref_key_seq = 0;
  uint32_t last_rx_ms = 0;
  bool last_app_ok = false;
  bool last_phy_ok = false;
};

static TelemetryState g_tel;

static const uint8_t HIST_LEN = 64;
static AbsCodes g_hist_codes[HIST_LEN];
static uint16_t g_hist_met[HIST_LEN];
static uint32_t g_hist_ts_ms[HIST_LEN];
static uint8_t g_hist_head = 0;
static uint8_t g_hist_count = 0;

static void telemetry_history_push(uint16_t met, const AbsCodes &codes, uint32_t ts_ms)
{
  g_hist_codes[g_hist_head] = codes;
  g_hist_met[g_hist_head] = met;
  g_hist_ts_ms[g_hist_head] = ts_ms;
  g_hist_head = (uint8_t)((g_hist_head + 1) % HIST_LEN);
  if (g_hist_count < HIST_LEN) g_hist_count++;
}

// ---------------------------------------------------------------------------
//  Link metrics + AFC stat
// ---------------------------------------------------------------------------

struct LinkMetrics
{
  int rssi_dbm = 0;
  int rssi_inst_dbm = 0;
  float snr_db = 0.0f;
  double fei_hz = 0.0;
  double bw_khz = 125.0;
  uint8_t bw_bits = 7;
  uint8_t sf = 12;
  uint8_t cr = 4;
  uint8_t modem_stat = 0;
  uint8_t irq_flags = 0;
  uint8_t rx_nb_bytes = 0;
  uint16_t rx_header_cnt = 0;
  uint16_t rx_packet_cnt = 0;
  uint8_t link_score = 0;
  uint32_t last_rx_ms = 0;
  uint16_t rx_pps = 0;
  uint32_t rx_total = 0;
};

static LinkMetrics g_link;

static uint8_t build_rf_meta_frame(uint8_t *out, uint8_t maxLen, uint8_t payload_len, const LoRaFrame &f)
{
  // Format (v1):
  // [0]  0xD3
  // [1]  0xD4
  // [2]  ver = 1
  // [3]  total_len (including crc8)  (fixed for v1)
  // [4]  payload_len (original LoRa len)
  // [5]  flags: bit0 PHY_OK, bit1 APP_OK, bit2 FULL, bit3 DELTA
  // [6]  rssi_pkt_dbm (int8)
  // [7]  rssi_inst_dbm (int8)
  // [8]  snr_qdb (int8)  SNR*4
  // [9-10] fei_hz (int16 LE, clamped to ±60000)
  // [11-12] offset_hz (int16 LE, clamped to ±60000)
  // [13] bw_bits
  // [14] sf
  // [15] cr (1..4)
  // [16] rx_nb_bytes
  // [17] modem_stat
  // [18] irq_flags
  // [19-20] rx_header_cnt (uint16 LE)
  // [21-22] rx_packet_cnt (uint16 LE)
  // [23] link_score (0..100)
  // [24] seq (last decoded)
  // [25-26] met (uint16 LE, last decoded)
  // [27] crc8_xor over bytes [2..26]
  const uint8_t total_len = 28;
  if (maxLen < total_len) return 0;

  out[0] = 0xD3;
  out[1] = 0xD4;
  out[2] = 1;
  out[3] = total_len;
  out[4] = payload_len;

  uint8_t flags = 0;
  bool phy_ok = (f.meta & 0x01) != 0;
  if (phy_ok) flags |= 1u << 0;
  if (g_tel.last_app_ok) flags |= 1u << 1;
  if (g_tel.last_sync == SYNC_FULL) flags |= 1u << 2;
  if (g_tel.last_sync == SYNC_DELTA) flags |= 1u << 3;
  out[5] = flags;

  out[6] = (uint8_t)(int8_t)g_link.rssi_dbm;
  out[7] = (uint8_t)(int8_t)g_link.rssi_inst_dbm;
  out[8] = (uint8_t)(int8_t)lroundf(g_link.snr_db * 4.0f);

  int32_t fei = (int32_t)lround(g_link.fei_hz);
  if (fei > 60000) fei = 60000;
  if (fei < -60000) fei = -60000;
  int16_t fei16 = (int16_t)fei;
  out[9] = (uint8_t)(fei16 & 0xFF);
  out[10] = (uint8_t)((fei16 >> 8) & 0xFF);

  int32_t ofs = (int32_t)LORA_OFFSET_HZ;
  if (ofs > 60000) ofs = 60000;
  if (ofs < -60000) ofs = -60000;
  int16_t ofs16 = (int16_t)ofs;
  out[11] = (uint8_t)(ofs16 & 0xFF);
  out[12] = (uint8_t)((ofs16 >> 8) & 0xFF);

  out[13] = g_link.bw_bits;
  out[14] = g_link.sf;
  out[15] = g_link.cr;
  out[16] = g_link.rx_nb_bytes;
  out[17] = g_link.modem_stat;
  out[18] = g_link.irq_flags;

  out[19] = (uint8_t)(g_link.rx_header_cnt & 0xFF);
  out[20] = (uint8_t)(g_link.rx_header_cnt >> 8);
  out[21] = (uint8_t)(g_link.rx_packet_cnt & 0xFF);
  out[22] = (uint8_t)(g_link.rx_packet_cnt >> 8);

  out[23] = g_link.link_score;
  out[24] = g_tel.last_seq;
  out[25] = (uint8_t)(g_tel.last_met & 0xFF);
  out[26] = (uint8_t)(g_tel.last_met >> 8);

  out[27] = crc8_xor(&out[2], 25);
  return total_len;
}

static const uint8_t FEI_HIST_LEN = 32;
static int32_t g_fei_hist_hz[FEI_HIST_LEN];
static uint8_t g_fei_head = 0;
static uint8_t g_fei_count = 0;

static void fei_history_push(double fei_hz)
{
  int32_t v = (int32_t)lround(fei_hz);
  g_fei_hist_hz[g_fei_head] = v;
  g_fei_head = (uint8_t)((g_fei_head + 1) % FEI_HIST_LEN);
  if (g_fei_count < FEI_HIST_LEN) g_fei_count++;
}

static const uint8_t RF_HIST_LEN = 64;
static int16_t g_rssi_hist_dbm[RF_HIST_LEN];
static int16_t g_snr_hist_qdb[RF_HIST_LEN]; // SNR * 4
static int16_t g_fei_hist_dbhz[RF_HIST_LEN];
static uint8_t g_rf_head = 0;
static uint8_t g_rf_count = 0;

static void rf_history_push(int rssi_dbm, float snr_db, double fei_hz)
{
  g_rssi_hist_dbm[g_rf_head] = (int16_t)rssi_dbm;
  g_snr_hist_qdb[g_rf_head] = (int16_t)lroundf(snr_db * 4.0f);
  g_fei_hist_dbhz[g_rf_head] = (int16_t)lround(fei_hz / 10.0); // Hz/10
  g_rf_head = (uint8_t)((g_rf_head + 1) % RF_HIST_LEN);
  if (g_rf_count < RF_HIST_LEN) g_rf_count++;
}

static const uint8_t HEALTH_HIST_LEN = 64;
static uint8_t g_phy_ok_hist[HEALTH_HIST_LEN];
static uint8_t g_app_ok_hist[HEALTH_HIST_LEN];
static uint8_t g_health_head = 0;
static uint8_t g_health_count = 0;

static void health_history_push(bool phy_ok, bool app_ok)
{
  g_phy_ok_hist[g_health_head] = phy_ok ? 1 : 0;
  g_app_ok_hist[g_health_head] = app_ok ? 1 : 0;
  g_health_head = (uint8_t)((g_health_head + 1) % HEALTH_HIST_LEN);
  if (g_health_count < HEALTH_HIST_LEN) g_health_count++;
}

static inline bool absstate_in_range(const AbsCodes &st)
{
  if (st.T_code > 1279u) return false;
  if (st.RH_code > 100u) return false;
  if (st.P_code12 > 2550u) return false;
  return true;
}

static void decode_abs_to_sample(const AbsCodes &st, TeleSample &s)
{
  float T_c = ((float)st.T_code / 10.0f) - 40.0f;
  float H = (float)st.RH_code;
  float P = 822.0f + ((float)st.P_code12 / 10.0f);

  s.tempC = T_c;
  s.rh = H;
  s.press_hPa = P;
}

static void telemetry_decode_frame(const uint8_t *buf, uint8_t len, uint32_t ts_ms)
{
  if (len == 0) return;

  g_tel.last_rx_ms = ts_ms;
  const uint8_t sync = buf[0];
  g_tel.last_sync = sync;
  g_tel.last_app_ok = false;

  if (sync == SYNC_FULL)
  {
    if (len != FULL_FRAME_LEN)
    {
      g_lora_status = "LEN";
      g_cnt_len_err++;
    }
    else
    {
      uint16_t crc_rx = (uint16_t)buf[36] | ((uint16_t)buf[37] << 8);
      uint16_t crc_calc = crc16_ccitt(buf, 36);
      if (crc_rx != crc_calc)
      {
        g_lora_status = "CRC";
        g_cnt_crc_err++;
        g_rx.valid = false;
      }
      else
      {
        uint16_t base_MET = (uint16_t)buf[1] | ((uint16_t)buf[2] << 8);
        uint8_t flags = buf[3];
        uint8_t seq = buf[4];
        uint8_t validity = buf[5];
        g_tel.last_base_met = base_MET;
        g_tel.last_flags = flags;
        g_tel.last_validity = validity;
        g_tel.last_seq = seq;
        g_tel.last_ref_key_seq = 0;

        // SEQ discontinuity → dobjuk az RX állapotot (TX oldalon is ez a javaslat)
        if (g_rx.have_seq)
        {
          uint8_t expected = (uint8_t)(g_rx.last_seq + 1u);
          if (seq != expected)
          {
            g_cnt_seq_disc++;
            uint8_t delta = (uint8_t)(seq - expected);
            if (delta > 0 && delta <= 32) g_cnt_seq_lost += (uint32_t)delta;
            g_rx.valid = false;
          }
        }
        g_rx.have_seq = true;
        g_rx.last_seq = seq;

        // 8×30-bit abszolút minta (LSB-first)
        uint16_t bitpos = 6u * 8u;
        AbsCodes st;
        AbsCodes last_sample_st;
        bool ok = true;

        for (uint8_t i = 0; i < MAX_SAMPLES_PER_PACKET; ++i)
        {
          uint32_t v30 = bitstream_read_lsb(buf, bitpos, 30);
          uint16_t T_code = (uint16_t)(v30 & 0x7FFu);
          uint8_t RH_code = (uint8_t)((v30 >> 11) & 0x7Fu);
          uint16_t P_int = (uint16_t)((v30 >> 18) & 0xFFu);
          uint16_t P_frac = (uint16_t)((v30 >> 26) & 0x0Fu);
          if (P_frac > 9u) P_frac = 9u;
          uint16_t P_code12 = (uint16_t)(P_int * 10u + P_frac);

          st.T_code = T_code;
          st.RH_code = RH_code;
          st.P_code12 = P_code12;
          ok = absstate_in_range(st);
          if (!ok)
          {
            g_lora_status = "RNG";
            g_cnt_rng_err++;
            g_rx.valid = false;
            break;
          }

          last_sample_st = st;
          uint16_t met_i = (uint16_t)(base_MET + i);
          telemetry_history_push(met_i, st, ts_ms);
        }

        // Állapot frissítés delta-hoz: a csomag utolsó abszolút mintája
        if (ok)
        {
          g_rx.codes = last_sample_st;
          g_rx.valid = true;
          g_lora_status = (validity == 0 && ((flags & (1u << 4)) == 0)) ? "SUPR" : "FULL";
          g_cnt_full_ok++;

          g_tel.have_last = true;
          g_tel.last_codes = last_sample_st;
          g_tel.last_met = (uint16_t)(base_MET + (MAX_SAMPLES_PER_PACKET - 1));
          g_tel.last_app_ok = true;
        }
      }
    }
  }
  else if (sync == SYNC_DELTA)
  {
    if (len != DELTA_FRAME_LEN)
    {
      g_lora_status = "LEN";
      g_cnt_len_err++;
    }
    else
    {
      uint16_t crc_rx = (uint16_t)buf[21] | ((uint16_t)buf[22] << 8);
      uint16_t crc_calc = crc16_ccitt(buf, 21);
      if (crc_rx != crc_calc)
      {
        g_lora_status = "CRC";
        g_cnt_crc_err++;
        g_rx.valid = false;
      }
      else
      {
        uint16_t base_MET = (uint16_t)buf[1] | ((uint16_t)buf[2] << 8);
        uint8_t flags = buf[3];
        uint8_t seq = buf[4];
        uint8_t ref_key_seq = buf[5];
        g_tel.last_base_met = base_MET;
        g_tel.last_flags = flags;
        g_tel.last_validity = 0;
        g_tel.last_seq = seq;
        g_tel.last_ref_key_seq = ref_key_seq;

        // SEQ discontinuity → eldobás FULL-ig
        if (g_rx.have_seq)
        {
          uint8_t expected = (uint8_t)(g_rx.last_seq + 1u);
          if (seq != expected)
          {
            g_cnt_seq_disc++;
            uint8_t delta = (uint8_t)(seq - expected);
            if (delta > 0 && delta <= 32) g_cnt_seq_lost += (uint32_t)delta;
            g_rx.valid = false;
          }
        }
        g_rx.have_seq = true;
        g_rx.last_seq = seq;

        if (!g_rx.valid)
        {
          g_lora_status = "DSYN";
          g_cnt_dsyn++;
        }
        else
        {
          uint16_t bitpos = 6u * 8u;
          AbsCodes st = g_rx.codes;

          for (uint8_t i = 0; i < MAX_SAMPLES_PER_PACKET; ++i)
          {
            int32_t dT = sign_extend(bitstream_read_lsb(buf, bitpos, 5), 5);
            int32_t dRH = sign_extend(bitstream_read_lsb(buf, bitpos, 4), 4);
            int32_t dP = sign_extend(bitstream_read_lsb(buf, bitpos, 6), 6);

            int32_t T_code = (int32_t)st.T_code + dT;
            int32_t RH_code = (int32_t)st.RH_code + dRH;
            int32_t P_code12 = (int32_t)st.P_code12 + dP;

            if (T_code < 0 || T_code > 1279 ||
                RH_code < 0 || RH_code > 100 ||
                P_code12 < 0 || P_code12 > 2550)
            {
              g_rx.valid = false;
              g_lora_status = "RNG";
              g_cnt_rng_err++;
              break;
            }

            st.T_code = (uint16_t)T_code;
            st.RH_code = (uint8_t)RH_code;
            st.P_code12 = (uint16_t)P_code12;
            uint16_t met_i = (uint16_t)(base_MET + i);
            telemetry_history_push(met_i, st, ts_ms);
          }

          if (g_rx.valid)
          {
            g_rx.codes = st;
            g_rx.valid = true;
            g_lora_status = (flags & (1u << 4)) ? "RST" : "DLTA";
            g_cnt_delta_ok++;

            g_tel.have_last = true;
            g_tel.last_codes = st;
            g_tel.last_met = (uint16_t)(base_MET + (MAX_SAMPLES_PER_PACKET - 1));
            g_tel.last_app_ok = true;
          }
        }
      }
    }
  }
  else
  {
    g_lora_status = "SYNC";
    g_cnt_sync_err++;
  }
}

static double lora_get_freq_mhz()
{
  uint8_t frfMsb = lora_read_reg(REG_FRF_MSB);
  uint8_t frfMid = lora_read_reg(REG_FRF_MID);
  uint8_t frfLsb = lora_read_reg(REG_FRF_LSB);
  uint32_t frf = ((uint32_t)frfMsb << 16) | ((uint32_t)frfMid << 8) | frfLsb;
  double freq_hz = (double)frf * (32000000.0 / 524288.0);
  return freq_hz / 1e6;
}

static void lora_get_modem_strings(uint8_t &sf, const char *&bwStr, const char *&crStr)
{
  uint8_t mc1 = lora_read_reg(REG_MODEM_CONFIG1);
  uint8_t mc2 = lora_read_reg(REG_MODEM_CONFIG2);

  uint8_t bw = (mc1 >> 4) & 0x0F;
  uint8_t cr = (mc1 >> 1) & 0x07;
  sf = (mc2 >> 4) & 0x0F;

  bwStr = "BW?";
  switch (bw)
  {
    case 0: bwStr = "7.8"; break;
    case 1: bwStr = "10.4"; break;
    case 2: bwStr = "15.6"; break;
    case 3: bwStr = "20.8"; break;
    case 4: bwStr = "31.25"; break;
    case 5: bwStr = "41.7"; break;
    case 6: bwStr = "62.5"; break;
    case 7: bwStr = "125"; break;
    case 8: bwStr = "250"; break;
    case 9: bwStr = "500"; break;
  }

  crStr = "CR?";
  switch (cr)
  {
    case 1: crStr = "4/5"; break;
    case 2: crStr = "4/6"; break;
    case 3: crStr = "4/7"; break;
    case 4: crStr = "4/8"; break;
  }
}


static void process_received_frame(const LoRaFrame &f, uint32_t now_ms)
{
  bool phy_ok = ((f.meta & 0x01) != 0);
  g_tel.last_phy_ok = phy_ok;

  // Link metrics a legutóbbi csomag alapján (regiszterek "last packet" értékek)
  uint8_t pkt_rssi_reg = lora_read_reg(REG_PKT_RSSI_VALUE);
  g_link.snr_db = lora_get_snr_db();

  // Datasheet (LoRa):
  // RSSI(dBm) = RSSI_OFFSET + RssiReg
  // Packet strength: RSSI_OFFSET + PacketRssi + PacketSnr*0.25 when SNR < 0
  double pkt_strength = (double)LORA_RSSI_OFFSET_DBM + (double)pkt_rssi_reg;
  if (g_link.snr_db < 0.0f)
    pkt_strength += (double)g_link.snr_db;
  // High signal linearity correction (when SNR>=0 and RSSI>-100dBm): -164 + 16/15*PacketRssi
  if (g_link.snr_db >= 0.0f && pkt_strength > -100.0)
    pkt_strength = (double)LORA_RSSI_OFFSET_DBM + (16.0 / 15.0) * (double)pkt_rssi_reg;
  g_link.rssi_dbm = (int)lround(pkt_strength);

  g_link.bw_khz = lora_get_bw_khz();
  g_link.fei_hz = lora_get_freq_error_hz(g_link.bw_khz);
  // Modem paraméterek (BW/SF/CR) kiolvasása itt (mert `lora_get_bw_khz()` csak a BW-t adja vissza).
  {
    uint8_t mc1 = lora_read_reg(REG_MODEM_CONFIG1);
    uint8_t mc2 = lora_read_reg(REG_MODEM_CONFIG2);
    g_link.bw_bits = (mc1 >> 4) & 0x0F;
    g_link.cr = (mc1 >> 1) & 0x07;
    g_link.sf = (mc2 >> 4) & 0x0F;
    if (g_link.cr < 1) g_link.cr = 1;
    if (g_link.cr > 4) g_link.cr = 4;
  }
  g_link.modem_stat = lora_read_reg(REG_MODEM_STAT);
  g_link.irq_flags = lora_read_reg(REG_IRQ_FLAGS);
  g_link.rx_nb_bytes = f.len;
  g_link.rx_header_cnt = (uint16_t)lora_read_reg(REG_RX_HEADER_CNT_LSB) | ((uint16_t)lora_read_reg(REG_RX_HEADER_CNT_MSB) << 8);
  g_link.rx_packet_cnt = (uint16_t)lora_read_reg(REG_RX_PACKET_CNT_LSB) | ((uint16_t)lora_read_reg(REG_RX_PACKET_CNT_MSB) << 8);
  g_link.last_rx_ms = now_ms;
  g_link.rx_total++;

  // AFC-t csak PHY CRC OK csomagra futtatjuk (stabilabb, kevésbé zajérzékeny).
  rf_history_push(g_link.rssi_dbm, g_link.snr_db, g_link.fei_hz);

  // Peak hold: csak PHY OK csomagból számoljuk, hogy ne zajosodjon el.
  if (phy_ok)
  {
    int16_t snr_qdb = (int16_t)lroundf(g_link.snr_db * 4.0f);
    int32_t fei_hz = (int32_t)lround(g_link.fei_hz);
    int32_t ofs_hz = (int32_t)LORA_OFFSET_HZ;
    if (!g_peak_valid)
    {
      g_peak_valid = true;
      g_peak_rssi_dbm = (int16_t)g_link.rssi_dbm;
      g_peak_snr_qdb = snr_qdb;
      g_peak_fei_hz = fei_hz;
      g_peak_offset_hz = ofs_hz;
    }
    else
    {
      if ((int16_t)g_link.rssi_dbm > g_peak_rssi_dbm) g_peak_rssi_dbm = (int16_t)g_link.rssi_dbm;
      if (snr_qdb > g_peak_snr_qdb) g_peak_snr_qdb = snr_qdb;
      if (abs(fei_hz) > abs(g_peak_fei_hz)) g_peak_fei_hz = fei_hz;
      if (abs(ofs_hz) > abs(g_peak_offset_hz)) g_peak_offset_hz = ofs_hz;
    }
  }

  if (phy_ok)
  {
    fei_history_push(g_link.fei_hz);
    afc_update(g_link.fei_hz);
  }

  // RX rate (pps) becslés
  static uint32_t win_start = 0;
  static uint16_t win_cnt = 0;
  if (win_start == 0) win_start = now_ms;
  win_cnt++;
  uint32_t dt = now_ms - win_start;
  if (dt >= 1000)
  {
    g_link.rx_pps = (uint16_t)((uint32_t)win_cnt * 1000u / (dt ? dt : 1u));
    win_cnt = 0;
    win_start = now_ms;
  }

  // Telemetria dekód (PHY CRC hibás csomagot nem tekintünk megbízhatónak)
  if (phy_ok)
  {
    telemetry_decode_frame(f.data, f.len, f.timestamp);

    // Kulcsframe (FULL) sikeres dekódja → peak reset a következő szakaszhoz.
    if (g_tel.last_app_ok && g_tel.last_sync == SYNC_FULL)
    {
      g_peak_valid = true;
      g_peak_rssi_dbm = (int16_t)g_link.rssi_dbm;
      g_peak_snr_qdb = (int16_t)lroundf(g_link.snr_db * 4.0f);
      g_peak_fei_hz = (int32_t)lround(g_link.fei_hz);
      g_peak_offset_hz = (int32_t)LORA_OFFSET_HZ;
    }
  }
  else
  {
    g_lora_status = "PHY";
    g_tel.last_app_ok = false;
  }

  // Link quality score (0..100) – utolsó 1s ablak hibáinak heurisztikája
  static uint32_t last_phy_crc = 0;
  static uint32_t last_seq_lost = 0;
  static uint32_t last_score_ms = 0;
  uint32_t phy_recent = 0;
  uint32_t seq_recent = 0;
  if (last_score_ms == 0) last_score_ms = now_ms;
  if ((now_ms - last_score_ms) >= 1000)
  {
    phy_recent = g_cnt_phy_crc_err - last_phy_crc;
    seq_recent = g_cnt_seq_lost - last_seq_lost;
    last_phy_crc = g_cnt_phy_crc_err;
    last_seq_lost = g_cnt_seq_lost;
    last_score_ms = now_ms;
  }
  g_link.link_score = link_score_0_100(g_link.rssi_dbm, g_link.snr_db, abs((int32_t)lround(g_link.fei_hz)), phy_recent, seq_recent);

  health_history_push(phy_ok, g_tel.last_app_ok);
}


// -----------------------------------------------------------------------------
//  SETUP / LOOP
// -----------------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  delay(1000);

  // --- NVS INIT + OFFSET LOAD ---
  nvs_flash_init();
  nvs_handle_t h;
  if (nvs_open("lora", NVS_READONLY, &h) == ESP_OK)
  {
      int32_t saved_ofs = 0;
      if (nvs_get_i32(h, "offset", &saved_ofs) == ESP_OK)
      {
          LORA_OFFSET_HZ = saved_ofs;
          last_saved_offset = saved_ofs;
      }
      nvs_close(h);
  }

  // Serial.println("CanSat BME280 + LoRa telemetria indul...");

  // Watchdog inicializalas (3s timeout, reset engedelyezve)
  esp_task_wdt_init(3, true);
  esp_task_wdt_add(NULL); // jelenlegi task


  if (!lora_begin())
  {
    // LoRa modul nem található – opcionálisan ide jöhetne hiba kijelzés
  }
}

void loop()
{
  uint32_t now = millis();

  // META küldés állapotgép: payload már kiment, várunk zajmintára (RegRssiValue),
  // majd META frame-et küldünk (payload után közvetlenül).
  static bool meta_pending = false;
  static uint32_t meta_due_ms = 0;
  static LoRaFrame meta_frame;
  static uint8_t meta_payload_len = 0;

  LoRaFrame f;
  bool processed_any = false;
  if (!meta_pending)
  {
    // Szigorú ordering: amíg az előző csomag meta-ja nincs kiküldve, nem küldünk új payloadot.
    if (rb_pop(f))
    {
      // 1) Nyers LoRa payload forward (python dekódernek)
      Serial.write(f.data, f.len);

      // 2) Link/AFC + telemetria állapot frissítése
      process_received_frame(f, now);

      // 3) META küldés késleltetése a zajmérés miatt
      meta_pending = true;
      meta_due_ms = now + NOISE_SAMPLE_DELAY_MS;
      meta_frame = f;
      meta_payload_len = f.len;

      processed_any = true;
    }
  }

  // META küldés: RegRssiValue késleltetett mintavétel után
  if (meta_pending && (int32_t)(now - meta_due_ms) >= 0)
  {
    uint8_t inst_rssi_reg = lora_read_reg(REG_RSSI_VALUE);
    g_link.rssi_inst_dbm = (int)inst_rssi_reg + LORA_RSSI_OFFSET_DBM;

    uint8_t meta[32];
    uint8_t metaLen = build_rf_meta_frame(meta, sizeof(meta), meta_payload_len, meta_frame);
    if (metaLen)
      Serial.write(meta, metaLen);

    meta_pending = false;
  }

  // turn off LED when time expires
  if (millis() > led_on_until)
  {
    digitalWrite(CAM_LED_PIN, LOW);
  }


  // Watchdog reset – ha a loop lefut rendesen, nem indul ujra az ESP32
  esp_task_wdt_reset();

}
