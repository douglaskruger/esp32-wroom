/* =============================================================================
   Copyright © 2025 Skynet Consulting Ltd.

   This file is the property of Skynet Consulting Ltd. and shall not be reproduced,
   copied, or used as the basis for the manufacture or sale of equipment without
   the expressed written permission of Skynet Consulting Ltd.
   =============================================================================
*/
static const char module_id[] __attribute__((used)) = "$Id: esp32_spi_hall_effect.c $";

/*
 * Description:
 * The following program presents a SPI - Hall Effect controller 
 * on the ESP32C - WROOM
*/
#include <SPI.h>

#define CS_PIN 5  // Chip Select pin (ESP32 GPIO5)

// SPI: SCK=18, MISO=19, MOSI=23
SPISettings tmagSPISettings(1000000, MSBFIRST, SPI_MODE0);

// ---------- CRC-3 (polynomial 0xB) ----------
uint8_t calcCRC3(uint32_t frame) {
  // Only top 29 bits are used
  uint32_t data = frame >> 3;
  uint8_t crc = 0;
  for (int i = 28; i >= 0; i--) {
    uint8_t bit = ((data >> i) & 1) ^ ((crc >> 2) & 1);
    crc = ((crc << 1) & 0x7);
    if (bit) crc ^= 0x3; // poly = x^3 + x + 1 => 0b011
  }
  return crc & 0x7;
}

// ---------- SPI helpers with CRC ----------
void writeRegister(uint8_t reg, uint16_t value) {
  uint32_t frame = (1UL << 31) | ((reg & 0x7F) << 25) | ((value & 0xFFFF) << 9);
  uint8_t crc = calcCRC3(frame);
  frame |= crc; // append 3-bit CRC
  uint8_t buf[4] = {frame >> 24, frame >> 16, frame >> 8, frame};
  digitalWrite(CS_PIN, LOW);
  SPI.beginTransaction(tmagSPISettings);
  for (int i = 0; i < 4; i++) SPI.transfer(buf[i]);
  SPI.endTransaction();
  digitalWrite(CS_PIN, HIGH);
}

uint16_t readRegister(uint8_t reg) {
  uint32_t frame = ((reg & 0x7F) << 25);
  uint8_t crc = calcCRC3(frame);
  frame |= crc; // append CRC
  uint8_t tx[4] = {frame >> 24, frame >> 16, frame >> 8, frame};
  uint8_t rx[4];
  digitalWrite(CS_PIN, LOW);
  SPI.beginTransaction(tmagSPISettings);
  for (int i = 0; i < 4; i++) rx[i] = SPI.transfer(tx[i]);
  SPI.endTransaction();
  digitalWrite(CS_PIN, HIGH);

  // Extract payload & CRC from response
  uint32_t resp = ((uint32_t)rx[0] << 24) | ((uint32_t)rx[1] << 16) | ((uint32_t)rx[2] << 8) | rx[3];
  uint16_t data = (resp >> 9) & 0xFFFF;
  uint8_t rx_crc = resp & 0x7;
  uint8_t calc_crc = calcCRC3(resp & ~0x7);

  if (rx_crc != calc_crc) {
    Serial.printf("❌ CRC mismatch! rx=0x%X calc=0x%X\n", rx_crc, calc_crc);
  }

  return data;
}

// ---------- Diagnostics ----------
void checkStatus() {
  uint16_t status = readRegister(0x08); // SYS_STATUS
  if (status == 0) return;

  Serial.printf("SYS_STATUS = 0x%04X\n", status);
  if (status & (1 << 9)) Serial.println("❌ SPI CRC error flagged by TMAG5170");
  if (status & (1 << 10)) Serial.println("❌ VCC undervoltage");
  if (status & (1 << 11)) Serial.println("❌ VCC overvoltage");
  if (status & (1 << 4)) Serial.println("❌ Analog front-end error");
  if (status & (1 << 5)) Serial.println("⚠️  Device reset occurred");
}

void setup() {
  Serial.begin(115200);
  SPI.begin(18, 19, 23);
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  delay(100);

  // Enable CRC in DEVICE_CONFIG
  //writeRegister(0x00, 0x000A); // continuous mode + CRC enabled
  //writeRegister(0x01, 0x071B); // SENSOR_CONFIG: XYZ, ±25 mT
  //writeRegister(0x02, 0x0000); // SYSTEM_CONFIG: standard result
  writeRegister(0x00, 0b0000000000101010); // continuous mode + CRC enabled  - 0b 0000 0000 0010 1010
  writeRegister(0x01, 0b0000000011100000); // SENSOR_CONFIG: XYZ, ±25 mT     - 0b 0000 0000 1110 0000
  writeRegister(0x02, 0b0000000000000000); // SYSTEM_CONFIG: standard result - 0b 0000 0000 0000 0000

  Serial.println("✅ TMAG5170 configured with CRC enabled.");
}

void loop() {
  int16_t x_raw = (int16_t)readRegister(0x09);
  int16_t y_raw = (int16_t)readRegister(0x0A);
  int16_t z_raw = (int16_t)readRegister(0x0B);
  int16_t t_raw = (int16_t)readRegister(0x0C);

  float x_mT = (float)x_raw / 1308.0f;
  float y_mT = (float)y_raw / 1308.0f;
  float z_mT = (float)z_raw / 1308.0f;
  float tempC = 25.0f + ((float)t_raw - 17522.0f) / 60.0f;

  Serial.printf("X=%.3f mT, Y=%.3f mT, Z=%.3f mT, Temp=%.2f °C (%d 0x%04X)\n",
                x_mT, y_mT, z_mT, tempC, t_raw, t_raw);

  checkStatus();
  delay(1000);
}
