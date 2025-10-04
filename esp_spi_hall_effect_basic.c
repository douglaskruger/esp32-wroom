/* =============================================================================
   Copyright © 2025 Skynet Consulting Ltd.

   This file is the property of Skynet Consulting Ltd. and shall not be reproduced,
   copied, or used as the basis for the manufacture or sale of equipment without
   the expressed written permission of Skynet Consulting Ltd.
   =============================================================================
*/
static const char module_id[] __attribute__((used)) = "$Id: esp_spi_hall_effect_basic.c $";

/*
 * Description:
 * The following program presents a SPI - Hall Effect controller 
 * on the ESP32C - WROOM
*/
#include <SPI.h>

#define CS_PIN 5  // Chip Select (adjust if needed)

void writeRegister(uint8_t reg, uint16_t value) {
  uint32_t frame = (1UL << 31) | ((reg & 0x7F) << 25) | ((value & 0xFFFF) << 9);
  uint8_t buf[4] = {frame >> 24, frame >> 16, frame >> 8, frame};
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(buf, 4);
  digitalWrite(CS_PIN, HIGH);
}

uint16_t readRegister(uint8_t reg) {
  uint32_t frame = ((reg & 0x7F) << 25);
  uint8_t tx[4] = {frame >> 24, frame >> 16, frame >> 8, frame};
  uint8_t rx[4];
  digitalWrite(CS_PIN, LOW);
  SPI.transferBytes(tx, rx, 4);
  digitalWrite(CS_PIN, HIGH);
  return (rx[2] << 8) | rx[3];
}

void setup() {
  Serial.begin(115200);
  SPI.begin(18, 19, 23); // SCK=18, MISO=19, MOSI=23
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  // Configure TMAG5170
  writeRegister(0x00, 0x0002); // DEVICE_CONFIG: continuous mode
  writeRegister(0x01, 0x0700); // SENSOR_CONFIG: X,Y,Z enabled, fast conversion, ±25 mT
  writeRegister(0x02, 0x0000); // SYSTEM_CONFIG: standard data type

  delay(10);
  Serial.println("TMAG5170 configured.");
}

void loop() {
  int16_t x = (int16_t)readRegister(0x09);
  int16_t y = (int16_t)readRegister(0x0A);
  int16_t z = (int16_t)readRegister(0x0B);
  int16_t temp = (int16_t)readRegister(0x0C);

  Serial.printf("X=%d, Y=%d, Z=%d, Temp=%d\n", x, y, z, temp);
  delay(1000);
}

