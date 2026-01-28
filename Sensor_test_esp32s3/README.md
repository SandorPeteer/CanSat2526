# AstroLink ESP32-S3 Sensor System

CanSat / High-Altitude Balloon flight computer with environmental sensors.

## Hardware Overview

```
                          ┌─────────────────────────────────────┐
                          │         ESP32-S3-DevKitC            │
                          │                                     │
     ┌────────────┐       │  GPIO 8  ──────── I2C SDA           │       ┌────────────┐
     │  QMC5883L  │◄──────│  GPIO 9  ──────── I2C SCL           │──────►│   BMP585   │
     │ Magnetomtr │       │                                     │       │ Barometer  │
     └────────────┘       │  GPIO 12 ──────── BMP585 INT        │       └────────────┘
                          │                                     │
     ┌────────────┐       │                                     │       ┌────────────┐
     │   SCD40    │◄──────│  (shared I2C bus)                   │──────►│  BNO085    │
     │  CO2/Temp  │       │                                     │       │    IMU     │
     └────────────┘       │  GPIO 5  ──────── SPI SCK           │       └────────────┘
                          │  GPIO 6  ──────── SPI MOSI          │
                          │  GPIO 7  ──────── SPI MISO          │
                          │  GPIO 15 ──────── BNO085 CS         │
                          │  GPIO 16 ──────── BNO085 WAKE       │
                          │  GPIO 10 ──────── BNO085 INT        │
                          │  GPIO 11 ──────── BNO085 RESET      │
                          │                                     │
     ┌────────────┐       │  GPIO 14 ──────── SPS30 TX (UART2)  │
     │   SPS30    │◄──────│  GPIO 13 ──────── SPS30 RX (UART2)  │
     │ Particulate│       │                                     │
     └────────────┘       │                                     │
                          │  GPIO 17 ──────── GPS TX (UART1)    │       ┌────────────┐
                          │  GPIO 18 ──────── GPS RX (UART1)    │──────►│  NEO-M8N   │
                          │                                     │       │    GPS     │
                          │  GPIO 48 ──────── WS2812 LED        │       └────────────┘
                          │                                     │
                          └─────────────────────────────────────┘
```

## Pin Assignment Table

| GPIO | Function       | Peripheral    | Notes                    |
|------|----------------|---------------|--------------------------|
| 5    | SPI SCK        | BNO085        | SPI Clock                |
| 6    | SPI MOSI       | BNO085        | SPI Data Out             |
| 7    | SPI MISO       | BNO085        | SPI Data In              |
| 8    | I2C SDA        | Shared Bus    | QMC5883L, BMP585, SCD40  |
| 9    | I2C SCL        | Shared Bus    | QMC5883L, BMP585, SCD40  |
| 10   | INT            | BNO085        | Interrupt (H_INTN)       |
| 11   | RESET          | BNO085        | Reset (NRST)             |
| 12   | INT            | BMP585        | Data Ready Interrupt     |
| 13   | UART2 RX       | SPS30         | Particle Sensor RX       |
| 14   | UART2 TX       | SPS30         | Particle Sensor TX       |
| 15   | SPI CS         | BNO085        | Chip Select              |
| 16   | WAKE           | BNO085        | PS0/WAKE pin             |
| 17   | UART1 TX       | GPS           | ESP TX -> GPS RX         |
| 18   | UART1 RX       | GPS           | ESP RX <- GPS TX         |
| 48   | DATA           | WS2812        | Status LED (RGB)         |

## Sensor Details

### I2C Bus (400 kHz)
| Sensor    | I2C Address | Function                        |
|-----------|-------------|---------------------------------|
| QMC5883L  | 0x0D        | 3-axis Magnetometer (Compass)   |
| BMP585    | 0x47        | Barometric Pressure & Temp      |
| SCD40     | 0x62        | CO2, Temperature, Humidity      |

### SPI Bus (BNO085)
| Signal | GPIO | Description          |
|--------|------|----------------------|
| SCK    | 5    | Serial Clock         |
| MOSI   | 6    | Master Out Slave In  |
| MISO   | 7    | Master In Slave Out  |
| CS     | 15   | Chip Select (active low) |
| INT    | 10   | Interrupt output     |
| RESET  | 11   | Hardware reset       |
| WAKE   | 16   | Wake/PS0 select      |

### UART Interfaces
| Interface | Baud Rate | TX GPIO | RX GPIO | Device        |
|-----------|-----------|---------|---------|---------------|
| UART1     | 38400     | 17      | 18      | NEO-M8N GPS   |
| UART2     | 115200    | 14      | 13      | SPS30         |

## Wiring Diagrams

### I2C Sensors (QMC5883L, BMP585, SCD40)
```
ESP32-S3          Sensors (all on same bus)
─────────         ─────────────────────────
GPIO 8 (SDA) ───┬─── QMC5883L SDA
                ├─── BMP585 SDA
                └─── SCD40 SDA

GPIO 9 (SCL) ───┬─── QMC5883L SCL
                ├─── BMP585 SCL
                └─── SCD40 SCL

3.3V ───────────┬─── QMC5883L VCC
                ├─── BMP585 VCC
                └─── SCD40 VCC

GND ────────────┬─── QMC5883L GND
                ├─── BMP585 GND
                └─── SCD40 GND

GPIO 12 ────────────── BMP585 INT (optional)
```

### BNO085 IMU (SPI)
```
ESP32-S3          BNO085
─────────         ──────
GPIO 5 (SCK)  ─── SCK
GPIO 6 (MOSI) ─── DI (MOSI)
GPIO 7 (MISO) ─── DO (MISO)
GPIO 15 (CS)  ─── CS
GPIO 10 (INT) ─── INT
GPIO 11 (RST) ─── RESET
GPIO 16 (WAK) ─── PS0/WAKE
3.3V          ─── VCC
GND           ─── GND
              ─── PS1 -> GND (SPI mode)
```

### SPS30 Particulate Sensor (UART)
```
ESP32-S3          SPS30
─────────         ─────
GPIO 14 (TX) ──── RX (Pin 2)
GPIO 13 (RX) ──── TX (Pin 3)
5V           ──── VCC (Pin 1)
GND          ──── GND (Pin 4)
                  SEL (Pin 5) -> GND (UART mode)
```

### NEO-M8N GPS (UART)
```
ESP32-S3          NEO-M8N
─────────         ───────
GPIO 17 (TX) ──── RX
GPIO 18 (RX) ──── TX
3.3V         ──── VCC
GND          ──── GND
```

### WS2812 Status LED
```
ESP32-S3          WS2812
─────────         ──────
GPIO 48      ──── DIN
3.3V         ──── VCC
GND          ──── GND
```

## Power Requirements

| Component   | Voltage | Typical Current | Peak Current |
|-------------|---------|-----------------|--------------|
| ESP32-S3    | 3.3V    | 80mA            | 500mA (WiFi) |
| BNO085      | 3.3V    | 12mA            | 25mA         |
| QMC5883L    | 3.3V    | 2mA             | 3mA          |
| BMP585      | 3.3V    | 0.5mA           | 1mA          |
| SCD40       | 3.3V    | 15mA            | 75mA         |
| SPS30       | 5V      | 60mA            | 80mA         |
| NEO-M8N     | 3.3V    | 25mA            | 50mA         |
| WS2812      | 3.3V    | 1mA             | 60mA         |

**Total estimated:** ~200mA typical, ~800mA peak

## Software Configuration

### WiFi
- AP Mode: `AstroLink-XXXX` (last 4 digits of MAC)
- Default password: `astrolink`
- Web interface: `http://192.168.4.1`

### Data Recording
- PSRAM ring buffer: up to 24h @ 1Hz (5.5MB)
- LittleFS file storage for permanent recordings
- Binary format: 64-byte records with all sensor data

### LED Status Indicators
| Pattern              | Meaning                    |
|----------------------|----------------------------|
| Solid Blue           | GPS acquiring fix          |
| Slow Green Pulse     | GPS fix acquired           |
| Fast Green Blink     | Recording active           |
| Purple Fade          | WiFi connected             |
| Yellow Blink         | Sensor error               |
| Red Blink            | Critical error             |

## Build & Flash

```bash
# Build firmware
pio run

# Upload firmware
pio run -t upload

# Upload filesystem (LittleFS)
pio run -t uploadfs

# Monitor serial output
pio device monitor
```
