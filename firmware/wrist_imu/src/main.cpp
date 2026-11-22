#include <Arduino.h>
#include <Wire.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>

const uint8_t ADDR = 0x68;   // AD0 tied to GND

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transport(Serial);
  delay(3000);               // gives you time to open the monitor
  Wire.begin(21, 22);        // SDA, SCL
  Wire.setClock(100000);

  writeReg(0x6B, 0x00);      // wake IMU from sleep
  delay(100);
  writeReg(0x1B, 0x00);   // gyro +/-250 dps
  writeReg(0x1C, 0x00);   // accel +/-2 g

  Wire.beginTransmission(ADDR);
  Wire.write(0x75);          // WHO_AM_I
  Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)ADDR, (size_t)1, true);
  uint8_t id = Wire.available() ? Wire.read() : 0xFF;
  Serial.printf("WHO_AM_I: 0x%02X (0x70 = MPU-6500)\n", id);
}

void loop() {
  Wire.beginTransmission(ADDR);
  Wire.write(0x3B);          // start of accel data
  Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)ADDR, (size_t)14, true);
  if (Wire.available() < 14) {
    Serial.println("read failed, check wiring");
    delay(500);
    return;
  }

  int16_t v[7];
  for (int i = 0; i < 7; i++) {
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    v[i] = (hi << 8) | lo;
  }

  // Default ranges: +/-2 g (16384 LSB/g), +/-250 dps (131 LSB/dps)
  Serial.printf("acc[g] %.3f %.3f %.3f | gyro[dps] %.2f %.2f %.2f\n",
                v[0] / 16384.0f, v[1] / 16384.0f, v[2] / 16384.0f,
                v[4] / 131.0f,   v[5] / 131.0f,   v[6] / 131.0f);
  delay(100);
}