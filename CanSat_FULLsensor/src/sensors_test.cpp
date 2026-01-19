/*
 * BNO085 test - based on Adafruit example
 */
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// Custom pins for ESP32-S3
static constexpr int PIN_I2C_SDA = 8;
static constexpr int PIN_I2C_SCL = 9;
static constexpr int PIN_BNO_INT = 10;
static constexpr int PIN_BNO_RST = 11;

Adafruit_BNO08x bno08x(PIN_BNO_RST);
sh2_SensorValue_t sensorValue;

void setReports() {
  Serial.println("Setting desired reports");

  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 50000)) {
    Serial.println("Could not enable rotation vector");
  } else {
    Serial.println("Rotation vector OK");
  }

  if (!bno08x.enableReport(SH2_ACCELEROMETER, 50000)) {
    Serial.println("Could not enable accelerometer");
  } else {
    Serial.println("Accelerometer OK");
  }

  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 50000)) {
    Serial.println("Could not enable gyroscope");
  } else {
    Serial.println("Gyroscope OK");
  }

  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION, 50000)) {
    Serial.println("Could not enable linear acceleration");
  } else {
    Serial.println("Linear accel OK");
  }

  if (!bno08x.enableReport(SH2_GRAVITY, 50000)) {
    Serial.println("Could not enable gravity");
  } else {
    Serial.println("Gravity OK");
  }

  if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 50000)) {
    Serial.println("Could not enable magnetometer");
  } else {
    Serial.println("Magnetometer OK");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  delay(1000);

  Serial.println("Adafruit BNO08x test!");

  // Initialize I2C with custom pins - try slower clock
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(100000);  // 100kHz - slower for debugging

  // I2C scan
  Serial.println("I2C scan:");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  Found: 0x%02X\n", addr);
    }
  }

  // Try to initialize - use default address (0x4A or 0x4B)
  if (!bno08x.begin_I2C(0x4B, &Wire)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("BNO08x Found!");

  for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
    Serial.print("Part ");
    Serial.print(bno08x.prodIds.entry[n].swPartNumber);
    Serial.print(": Version :");
    Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swBuildNumber);
    Serial.print(" Build ");
    Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
  }

  setReports();

  Serial.println("Reading events");
  delay(100);
}

void loop() {
  static uint32_t loopCount = 0;
  static uint32_t eventCount = 0;
  static uint32_t resetCount = 0;
  static uint32_t lastStatus = 0;

  loopCount++;
  delay(10);

  if (bno08x.wasReset()) {
    resetCount++;
    Serial.printf("[%lu] sensor was reset (#%lu)\n", millis(), resetCount);
    setReports();
  }

  // Status minden 3 másodpercben
  if (millis() - lastStatus > 3000) {
    lastStatus = millis();
    Serial.printf("[%lu] loops=%lu events=%lu resets=%lu INT=%d\n",
                  millis(), loopCount, eventCount, resetCount, digitalRead(PIN_BNO_INT));
  }

  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  eventCount++;

  switch (sensorValue.sensorId) {
    case SH2_ACCELEROMETER:
      Serial.print("Accelerometer - x: ");
      Serial.print(sensorValue.un.accelerometer.x);
      Serial.print(" y: ");
      Serial.print(sensorValue.un.accelerometer.y);
      Serial.print(" z: ");
      Serial.println(sensorValue.un.accelerometer.z);
      break;
    case SH2_GYROSCOPE_CALIBRATED:
      Serial.print("Gyro - x: ");
      Serial.print(sensorValue.un.gyroscope.x);
      Serial.print(" y: ");
      Serial.print(sensorValue.un.gyroscope.y);
      Serial.print(" z: ");
      Serial.println(sensorValue.un.gyroscope.z);
      break;
    case SH2_MAGNETIC_FIELD_CALIBRATED:
      Serial.print("Magnetic Field - x: ");
      Serial.print(sensorValue.un.magneticField.x);
      Serial.print(" y: ");
      Serial.print(sensorValue.un.magneticField.y);
      Serial.print(" z: ");
      Serial.println(sensorValue.un.magneticField.z);
      break;
    case SH2_LINEAR_ACCELERATION:
      Serial.print("Linear Acceration - x: ");
      Serial.print(sensorValue.un.linearAcceleration.x);
      Serial.print(" y: ");
      Serial.print(sensorValue.un.linearAcceleration.y);
      Serial.print(" z: ");
      Serial.println(sensorValue.un.linearAcceleration.z);
      break;
    case SH2_ROTATION_VECTOR:
      Serial.print("Rotation Vector - r: ");
      Serial.print(sensorValue.un.rotationVector.real);
      Serial.print(" i: ");
      Serial.print(sensorValue.un.rotationVector.i);
      Serial.print(" j: ");
      Serial.print(sensorValue.un.rotationVector.j);
      Serial.print(" k: ");
      Serial.println(sensorValue.un.rotationVector.k);
      break;
  }
}
