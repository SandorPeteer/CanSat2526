# Sensor_test_esp32s3

ESP32-S3 alapú multi-szenzor hub a CanSat projekt számára. Valós idejű adatgyűjtés, WebSocket-alapú webes felület és repülési adatrögzítés.

## Hardver

- **MCU:** ESP32-S3 (N16R8 - 16MB Flash, 8MB PSRAM)
- **Szenzorok:**
  - **SPS30** - Porszennyezettség mérő (PM1.0, PM2.5, PM4.0, PM10) - UART
  - **BNO085** - 9-DoF IMU (gyorsulásmérő, giroszkóp, magnetométer, fúziós quaternion) - SPI
  - **BMP585** - Nyomás/hőmérséklet szenzor, magasságmérés - I2C
  - **SCD40** - CO2/hőmérséklet/páratartalom szenzor - I2C
  - **QMC5883L** - 3-tengelyes magnetométer (iránytű) - I2C
  - **u-blox GPS** - Pozíció, sebesség, magasság, idő - UART

## Főbb funkciók

### Megvalósított funkciók

- **WiFi Manager** - NVS-ben tárolt hitelesítő adatok, fallback AP mód, captive portal
- **HTTP szerver** - LittleFS-ből kiszolgált statikus fájlok (HTML/CSS/JS)
- **WebSocket** - Valós idejű bináris adatfolyam minden szenzorról (~50Hz IMU, 1Hz többi)
- **Webes Dashboard** - Reszponzív UI, oldalanként szenzor részletek, iránytű vizualizáció
- **Flight Recorder** - PSRAM ring buffer (24 óra @ 1Hz = 86400 rekord, ~5.4MB)
- **History Buffer** - SPS30 adatok történeti tárolása PSRAM-ban
- **NTP időszinkron** - Automatikus időszinkron WiFi esetén, GPS fallback
- **Szenzor vezérlés** - Szenzorok egyenkénti ki/bekapcsolása webes felületről
- **Rendszer statisztikák** - WiFi RSSI, CPU hőmérséklet, uptime

### Pin kiosztás

| Szenzor | Interfész | Pinek |
|---------|-----------|-------|
| SPS30 | UART2 | TX:14, RX:13 |
| GPS | UART1 | TX:17, RX:18 |
| I2C (BMP585, SCD40, QMC5883L) | I2C0 | SDA:8, SCL:9 |
| BNO085 | SPI2 | SCK:5, MOSI:6, MISO:7, CS:15, INT:10, RST:11, WAKE:16 |
| BMP585 INT | GPIO | 12 |

## Webes felület

A beépített webes felület (`http://astrolink.local` vagy az IP cím):

- **Dashboard** - Minden szenzor összefoglalója egy helyen
- **SPS30** - Részletes PM értékek és részecskeszám
- **GPS & Compass** - Pozíció, sebesség, műholdak, iránytű
- **BNO085 IMU** - Quaternion, Euler szögek, 3D vizualizáció
- **BMP585** - Nyomás, hőmérséklet, barometrikus magasság
- **SCD40 CO2** - CO2 koncentráció, hőmérséklet, páratartalom
- **Files** - Rögzített repülési adatok kezelése
- **System** - WiFi beállítások, rendszer infó

## Építés és feltöltés

### Követelmények

- PlatformIO
- ESP-IDF framework (automatikusan települ)

### Fordítás

```bash
pio run -e esp32s3_sps30_idf
```

### Feltöltés

```bash
pio run -e esp32s3_sps30_idf -t upload
```

### Fájlrendszer feltöltése

```bash
pio run -e esp32s3_sps30_idf -t uploadfs
```

## Projekt struktúra

```
src/
├── main.cpp              # Fő belépési pont, task-ok létrehozása
├── common/
│   ├── flight_recorder   # PSRAM ring buffer repülési adatoknak
│   ├── flight_data_collector # Szenzor adatok összegyűjtése
│   ├── history_buffer    # SPS30 history PSRAM-ban
│   ├── ntp_time          # Időszinkronizáció
│   ├── sensor_control    # Szenzor ki/be kapcsolás
│   ├── shared_data       # Thread-safe szenzor adat megosztás
│   └── system_stats      # Rendszer statisztikák
├── gps/
│   ├── ubx_parser        # u-blox UBX protokoll parser
│   └── gps_logger        # GPS naplózás
├── http/
│   └── http_server       # HTTP + WebSocket szerver
├── sensors/
│   ├── bno085            # BNO085 IMU driver (SPI)
│   ├── bmp585            # BMP585 nyomásszenzor driver
│   ├── scd40             # SCD40 CO2 szenzor driver
│   └── qmc5883l          # Magnetométer driver
├── sps30/
│   └── sps30             # SPS30 porszenzor driver (UART)
└── wifi/
    └── wifi_manager      # WiFi kezelő (STA + AP mód)

data/                     # Webes felület (LittleFS)
├── index.html
├── style.css
├── app.js
└── favicon.svg

components/
└── esp32_BNO08x/         # BNO08x ESP-IDF komponens
```

## Licensz

MIT
