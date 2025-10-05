#include <Arduino.h>
#include <SPI.h>

/* -------------------- PIN DEFINITIONS -------------------- */
#define PIN_NUM_MISO  19
#define PIN_NUM_MOSI  23
#define PIN_NUM_CLK   18
#define PIN_NUM_CS     5
#define PIN_NUM_ALERT  4   // optional ALERT pin

/* -------------------- SPI INSTANCE -------------------- */
SPIClass spi(VSPI);

/* -------------------- TMAG5170 DEFINES -------------------- */
#define TMAG5170_FRAME_NUM_BYTES 4
#define DEVICE_CONFIG_ADDRESS    ((uint8_t) 0x00)
#define DEVICE_CONFIG_OPERATING_MODE_ActiveMeasureMode ((uint16_t)0x0020)

/* -------------------- HAL FUNCTIONS -------------------- */
void delay_ms(const uint32_t delay_time_ms) { delay(delay_time_ms); }
void delay_us(const uint32_t delay_time_us) { delayMicroseconds(delay_time_us); }

void setCS(const bool state) { digitalWrite(PIN_NUM_CS, state ? HIGH : LOW); }
bool getCS(void) { return digitalRead(PIN_NUM_CS); }

void setALERT(const bool state) {
  pinMode(PIN_NUM_ALERT, OUTPUT);
  digitalWrite(PIN_NUM_ALERT, state ? HIGH : LOW);
}
bool getALERT(void) {
  pinMode(PIN_NUM_ALERT, INPUT_PULLUP);
  return digitalRead(PIN_NUM_ALERT);
}

void spiSendReceiveArrays(const uint8_t DataTx[], uint8_t DataRx[], const uint8_t byteLength) {
  setCS(LOW);
  for (uint8_t i = 0; i < byteLength; i++) {
    DataRx[i] = spi.transfer(DataTx[i]);
  }
  setCS(HIGH);
}

uint8_t spiSendReceiveByte(const uint8_t dataTx) {
  uint8_t dataRx;
  setCS(LOW);
  dataRx = spi.transfer(dataTx);
  setCS(HIGH);
  return dataRx;
}

bool waitForALERTinterrupt(const uint32_t timeout_ms) {
  uint32_t start = millis();
  while ((millis() - start) < timeout_ms) {
    if (digitalRead(PIN_NUM_ALERT) == LOW) return true;
  }
  return false;
}

/* -------------------- TMAG5170 BASIC DRIVER -------------------- */
void sendAndReceiveFrame(uint8_t dataTx[], uint8_t dataRx[]) {
  spiSendReceiveArrays(dataTx, dataRx, TMAG5170_FRAME_NUM_BYTES);
}

void writeToRegister(uint8_t address, uint16_t data_to_write) {
  uint8_t dataTx[4] = {0};
  uint8_t dataRx[4] = {0};
  dataTx[0] = (address & 0x7F);    // Write command (MSB=0)
  dataTx[1] = (data_to_write >> 8);
  dataTx[2] = (data_to_write & 0xFF);
  dataTx[3] = 0x00; // CRC ignored for simplicity
  sendAndReceiveFrame(dataTx, dataRx);
}

uint16_t normalReadRegister(uint8_t address) {
  uint8_t dataTx[4] = {0};
  uint8_t dataRx[4] = {0};
  dataTx[0] = (address | 0x80); // Read command (MSB=1)
  sendAndReceiveFrame(dataTx, dataRx);
  return ((uint16_t)dataRx[1] << 8) | dataRx[2];
}

/* Startup: reset + configure */
void TMAG5170startup() {
  delay_ms(50);
  // Enter Active Measure Mode
  uint16_t config = normalReadRegister(DEVICE_CONFIG_ADDRESS);
  config = (config & ~0x0070) | DEVICE_CONFIG_OPERATING_MODE_ActiveMeasureMode;
  writeToRegister(DEVICE_CONFIG_ADDRESS, config);
}

/* Example measurement getters */
uint16_t getXresult() { return normalReadRegister(0x06); }
uint16_t getYresult() { return normalReadRegister(0x07); }
uint16_t getZresult() { return normalReadRegister(0x08); }

/* -------------------- ARDUINO SETUP/LOOP -------------------- */
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_NUM_CS, OUTPUT);
  digitalWrite(PIN_NUM_CS, HIGH);

  pinMode(PIN_NUM_ALERT, INPUT_PULLUP);

  spi.begin(PIN_NUM_CLK, PIN_NUM_MISO, PIN_NUM_MOSI, PIN_NUM_CS);
  spi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1)); // SPI mode 1

  Serial.println("Initializing TMAG5170...");
  TMAG5170startup();
  Serial.println("TMAG5170 ready.");
}

void loop() {
  uint16_t x = getXresult();
  uint16_t y = getYresult();
  uint16_t z = getZresult();

  Serial.printf("Raw X=%u, Y=%u, Z=%u\n", x, y, z);
  delay(500);
}
