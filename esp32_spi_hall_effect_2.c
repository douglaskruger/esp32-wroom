/* =============================================================================
   Copyright © 2025 Skynet Consulting Ltd.

   This file is the property of Skynet Consulting Ltd. and shall not be reproduced,
   copied, or used as the basis for the manufacture or sale of equipment without
   the expressed written permission of Skynet Consulting Ltd.
   =============================================================================
*/
static const char module_id[] __attribute__((used)) = "$Id: set_gpio.c $";

/*
 * Description:
 * SPI Hall Effect controller (TMAG5170) on ESP32C-WROOM with CLI.
*/
#include <SPI.h>
#define TMAG_SPI_MODE_SPI4   0x0   // <-- If datasheet says 4-wire SPI = 0

#define readReg 0x80
#define writeReg 0x0
#define STAY 0xffff
#define axis_X      0x0
#define axis_Y      0x1
#define axis_Z      0x2
//------------------Register Address------------------------
#define DEVICE_CONFIG 0x0
#define SENSOR_CONFIG 0x1
#define SYSTEM_CONFIG 0x2
#define ALERT_CONFIG 0x3
#define X_THRX_CONFIG 0x4
#define Y_THRX_CONFIG 0x5
#define Z_THRX_CONFIG 0x6
#define T_THRX_CONFIG 0x7
#define CONV_STATUS 0x8
#define X_CH_RESULT 0x9
#define Y_CH_RESULT 0xA
#define Z_CH_RESULT 0xB
#define TEMP_RESULT 0xC
#define AFE_STATUS 0xD
#define SYS_STATUS 0xE
#define TEST_CONFIG 0xF
#define OSC_MONITOR 0x10
#define MAG_GAIN_CONFIG 0x11
#define ANGLE_RESULT 0x13
#define MAGNITUDE_RESULT 0x14

#define start_DE_CRC      0x0f000400  
#define DeviceConfigData  0b0101000000001000
#define DeviceStart       0b0101000000101000
#define RANGE_50mT 0x0

//------------------DEVICE_CONFIG------------------------
#define CONV_AVG_MASK 0x7000
#define CONV_AVG_1x 0x0
#define CONV_AVG_2x 0x1000
#define CONV_AVG_4x 0x2000
#define CONV_AVG_8x 0x3000
#define CONV_AVG_16x 0x4000
#define CONV_AVG_32x 0x5000

#define MAG_TEMPCO_MASK 0x300
#define MAG_TEMPCO_0pd 0x0
#define MAG_TEMPCO_0R12pd 0x100
#define MAG_TEMPCO_0R2pd 0x300

#define OPERATING_MODE_MASK 0x70
#define OPERATING_MODE_ConfigurationMode 0x0
#define OPERATING_MODE_StandbyMode 0x10
#define OPERATING_MODE_activeMeasureMode 0x20
#define OPERATING_MODE_ActiveTriggerMode 0x30
#define OPERATING_MODE_WakeupAndSleepMode 0x40
#define OPERATING_MODE_SleepMode 0x50
#define OPERATING_MODE_DeepsleepMode 0x60

#define T_CH_EN_TempChannelDisabled 0x0
#define T_CH_EN_TempChannelEnabled 0x8

#define T_RATE_sameAsOtherSensors 0x0
#define T_RATE_oncePerConversionSet 0x4

#define T_HLT_EN_tempLimitCheckOff 0x0
#define T_HLT_EN_tempLimitCheckOn 0x2

#define TEMP_COMP_EN_TempCompensationDisenabled 0x0
#define TEMP_COMP_EN_TempCompensationEnabled 0x1
//------------------SENSOR_CONFIG------------------------
#define ANGLE_EN_NoAngleCalculation 0x0
#define ANGLE_EN_X_Y 0x4000
#define ANGLE_EN_Y_Z 0x8000
#define ANGLE_EN_Z_X 0xC000

#define SLEEPTIME_1ms 0x0
#define SLEEPTIME_5ms 0x400
#define SLEEPTIME_10ms 0x800
#define SLEEPTIME_15ms 0xC00
#define SLEEPTIME_20ms 0x1000
#define SLEEPTIME_30ms 0x1400
#define SLEEPTIME_50ms 0x1800
#define SLEEPTIME_100ms 0x1C00
#define SLEEPTIME_500ms 0x2000
#define SLEEPTIME_1000ms 0x2400

#define MAG_CH_EN_MASK 0x3C0
#define MAG_CH_EN_OFF 0x0
#define MAG_CH_EN_Xenabled 0x40
#define MAG_CH_EN_Yenabled 0x80
#define MAG_CH_EN_XYenabled 0xC0
#define MAG_CH_EN_Zenabled 0x100
#define MAG_CH_EN_ZXenabled 0x140
#define MAG_CH_EN_YZenabled 0x180
#define MAG_CH_EN_XYZenaled 0x1C0
#define MAG_CH_EN_XYXenaled 0x200
#define MAG_CH_EN_YXYenaled 0x240
#define MAG_CH_EN_YZYenaled 0x280
#define MAG_CH_EN_ZYZenaled 0x2C0
#define MAG_CH_EN_ZXZenaled 0x300
#define MAG_CH_EN_XZXenaled 0x340
#define MAG_CH_EN_XYZYXenaled 0x380
#define MAG_CH_EN_XYZZYXenaled 0x3C0

#define Z_RANGE_MASK 0x30
#define Z_RANGE_50mT 0x0
#define Z_RANGE_25mT 0x10
#define Z_RANGE_100mT 0x20

#define Y_RANGE_MASK 0xC
#define Y_RANGE_50mT 0x0
#define Y_RANGE_25mT 0x4
#define Y_RANGE_100mT 0x8

#define X_RANGE_MASK 0x3
#define X_RANGE_50mT 0x0
#define X_RANGE_25mT 0x1
#define X_RANGE_100mT 0x2
//------------------SYSTEM_CONFIG------------------------
#define DIAG_SEL_AllDataPath 0x0
#define DIAG_SEL_enabledDataPath 0x1000
#define DIAG_SEL_enabledDataPathInsequence 0x2000
#define DIAG_SEL_enabledDataPathInsequence 0x3000

#define TRIGGER_MODE_MASK 0x600
#define TRIGGER_MODE_SPI 0x0
#define TRIGGER_MODE_nCSsyncPulse 0x200
#define TRIGGER_MODE_ALERTsyncPulse 0x400

#define DATA_TYPE_32bit 0x0
#define DATA_TYPE_12bit_XY 0x40
#define DATA_TYPE_12bit_XZ 0x80
#define DATA_TYPE_12bit_ZY 0xC0
#define DATA_TYPE_12bit_XT 0x100
#define DATA_TYPE_12bit_YT 0x140
#define DATA_TYPE_12bit_ZT 0x180
#define DATA_TYPE_12bit_AM 0x1C0

#define DIAG_EN_AFEdiagnosticsDisabled 0x0
#define DIAG_EN_ExecutionOftheDiagnosticsSelectedInDEVICE_CFG 0x20

#define Z_HLT_EN_ZaxisLimitCheckoff 0x0
#define Z_HLT_EN_ZaxisLimitCheckon 0x4

#define Y_HLT_EN_YaxisLimitCheckoff 0x0
#define Y_HLT_EN_YaxisLimitCheckon 0x2

#define X_HLT_EN_XaxisLimitCheckoff 0x0
#define X_HLT_EN_XaxisLimitCheckon 0x1
//------------------ALERT_CONFIG------------------------
#define ALERT_LATCH_sourcesNotLatched 0x0
#define ALERT_LATCH_sourcesLatched 0x2000

#define ALERT_MODE_interruptMode 0x0
#define ALERT_MODE_comparatorMode 0x1000

#define STATUS_ALRT_ALERTisNotAsserted 0x0
#define STATUS_ALRT_ALERTisAsserted 0x800

#define RSLT_ALRT_ALERTisNotUsedToSignal 0x0
#define RSLT_ALRT_ALERTisUsedToSignal 0x100

#define THRX_COUNT_1_ConversionResult 0x0
#define THRX_COUNT_2_ConversionResult 0x10
#define THRX_COUNT_3_ConversionResult 0x20
#define THRX_COUNT_4_ConversionResult 0x30

#define T_THRX_ALRT_ALERTisNotUsedToSignal 0x0
#define T_THRX_ALRT_ALERTisUsedToSignal 0x8

#define Z_THRX_ALRT_ALERTisNotUsedToSignal 0x0
#define Z_THRX_ALRT_ALERTisUsedToSignal 0x4

#define Y_THRX_ALRT_ALERTisNotUsedToSignal 0x0
#define Y_THRX_ALRT_ALERTisUsedToSignal 0x2

#define X_THRX_ALRT_ALERTisNotUsedToSignal 0x0
#define X_THRX_ALRT_ALERTisUsedToSignal 0x1
//------------------TEST_CONFIG------------------------
#define CRC_DIS_CRCenabled 0x0
#define CRC_DIS_CRCdisabled 0x4

#define OSC_CNT_CTL_ResetCounters 0x0
#define OSC_CNT_CTL_StartOscCounterdrivenbyHFOSC 0x1
#define OSC_CNT_CTL_StartOscCounterdrivenbyLFOSC 0x2
#define OSC_CNT_CTL_stopCounter 0x3
//------------------MAG_GAIN_CONFIG------------------------
#define GAIN_SELECTION_noAxisSelected 0x0
#define GAIN_SELECTION_XisSelected 0x4000
#define GAIN_SELECTION_YisSelected 0x8000
#define GAIN_SELECTION_ZisSelected 0xC000

// Instead of T_CH_EN_enabled
#define T_CH_EN_enabled      T_CH_EN_TempChannelEnabled

// Instead of MAG_TEMPCO_compensated
#define MAG_TEMPCO_compensated MAG_TEMPCO_0R12pd  

// ---------- Pin Assignments ----------
const int PIN_SPI_SCK  = 18;
const int PIN_SPI_MISO = 19;
const int PIN_SPI_MOSI = 23;
const int PIN_CS       = 5;

// ---------- SPI Settings ----------
SPISettings tmagSPISettings(1000000, MSBFIRST, SPI_MODE0);

// ---------- CLI Buffers ----------
#define CMD_BUF_LEN 64
char cmdBuffer[CMD_BUF_LEN];
int cmdIndex = 0;

// ---------- State ----------
bool samplingEnabled = false;
unsigned long sampleInterval = 5000; // default 5s
unsigned long lastSampleTime = 0;

// ---------- CRC-3 (polynomial 0xB) ----------
uint8_t calcCRC3(uint32_t frame) {
  uint32_t data = frame >> 3;
  uint8_t crc = 0;
  for (int i = 28; i >= 0; i--) {
    uint8_t bit = ((data >> i) & 1) ^ ((crc >> 2) & 1);
    crc = ((crc << 1) & 0x7);
    if (bit) crc ^= 0x3;
  }
  return crc & 0x7;
}

// ---------- SPI Helpers ----------
void writeRegister(uint8_t reg, uint16_t value) {
  uint32_t frame = (1UL << 31) | ((reg & 0x7F) << 25) | ((value & 0xFFFF) << 9);
  uint8_t crc = calcCRC3(frame);
  frame |= crc;
  uint8_t buf[4] = {frame >> 24, frame >> 16, frame >> 8, frame};
  Serial.printf("** writeRegister Tx: frame=0x%X, crc=0x%X\n", frame, crc);
  digitalWrite(PIN_CS, LOW);
  SPI.beginTransaction(tmagSPISettings);
  for (int i = 0; i < 4; i++) SPI.transfer(buf[i]);
  SPI.endTransaction();
  digitalWrite(PIN_CS, HIGH);
}

uint16_t readRegister(uint8_t reg) {
  uint32_t frame = ((reg & 0x7F) << 25);
  uint8_t crc = calcCRC3(frame);
  frame |= crc;
  uint8_t tx[4] = {frame >> 24, frame >> 16, frame >> 8, frame};
  Serial.printf("** readRegister Tx: frame=0x%X, crc=0x%X\n", frame, crc);
  uint8_t rx[4];
  digitalWrite(PIN_CS, LOW);
  SPI.beginTransaction(tmagSPISettings);
  for (int i = 0; i < 4; i++) rx[i] = SPI.transfer(tx[i]);
  SPI.endTransaction();
  digitalWrite(PIN_CS, HIGH);

  uint32_t resp = ((uint32_t)rx[0] << 24) | ((uint32_t)rx[1] << 16) | ((uint32_t)rx[2] << 8) | rx[3];
  uint16_t data = (resp >> 9) & 0xFFFF;
  uint8_t rx_crc = resp & 0x7;
  uint8_t calc_crc = calcCRC3(resp & ~0x7);
  Serial.printf("** readRegister Rx: resp=0x%X, data=0x%X, rx=0x%X, calc=0x%X\n", resp, data, rx_crc, calc_crc);

  if (rx_crc != calc_crc) {
    Serial.printf("❌ CRC mismatch! rx=0x%X calc=0x%X\n", rx_crc, calc_crc);
  }

  return data;
}

// ---------- Diagnostics ----------
void checkStatus() {
  uint16_t status = readRegister(SYS_STATUS); // use SYS_STATUS constant
  if (status == 0) return;

  Serial.printf("SYS_STATUS = 0x%04X\n", status);
  if (status & (1 << 9))  Serial.println("❌ SPI CRC error flagged by TMAG5170");
  if (status & (1 << 10)) Serial.println("❌ VCC undervoltage");
  if (status & (1 << 11)) Serial.println("❌ VCC overvoltage");
  if (status & (1 << 4))  Serial.println("❌ Analog front-end error");
  if (status & (1 << 5))  Serial.println("⚠️  Device reset occurred");
}

// ---------- CLI ----------
void processCommand(const char *cmd) {
    // START sampling
    if (strcasecmp(cmd, "START") == 0) {
        samplingEnabled = true;
        Serial.println("✅ Sampling started.");
    }
    // STOP sampling
    else if (strcasecmp(cmd, "STOP") == 0) {
        samplingEnabled = false;
        Serial.println("🛑 Sampling stopped.");
    }
    // STATUS of sampling
    else if (strcasecmp(cmd, "STATUS") == 0) {
        Serial.printf("ℹ️ Sampling is %s, interval=%lu ms\n",
                      samplingEnabled ? "ENABLED" : "DISABLED", sampleInterval);
    }
    // SET RATE <ms>
    else if (strncasecmp(cmd, "SET RATE", 8) == 0) {
        unsigned long val = atol(cmd + 8);
        if (val >= 50) {
            sampleInterval = val;
            Serial.printf("✅ Sampling interval set to %lu ms\n", sampleInterval);
        } else {
            Serial.println("⚠️ Interval too short! Must be >= 50 ms.");
        }
    }
    // SET RANGE <axis> <25|50|100>
    else if (strncasecmp(cmd, "SET RANGE", 9) == 0) {
        int axis = -1;
        int range = 0;
        if (sscanf(cmd + 9, "%d %d", &axis, &range) == 2) {
            uint16_t regVal = readRegister(SENSOR_CONFIG);

            switch(axis) {
                case 0: // X
                    regVal &= ~X_RANGE_MASK;
                    if (range == 25) regVal |= X_RANGE_25mT;
                    else if (range == 50) regVal |= X_RANGE_50mT;
                    else if (range == 100) regVal |= X_RANGE_100mT;
                    else { Serial.println("⚠️ Invalid range! Use 25, 50, or 100"); break; }
                    writeRegister(SENSOR_CONFIG, regVal);
                    Serial.printf("✅ X-axis range set to ±%d mT\n", range);
                    break;
                case 1: // Y
                    regVal &= ~Y_RANGE_MASK;
                    if (range == 25) regVal |= Y_RANGE_25mT;
                    else if (range == 50) regVal |= Y_RANGE_50mT;
                    else if (range == 100) regVal |= Y_RANGE_100mT;
                    else { Serial.println("⚠️ Invalid range! Use 25, 50, or 100"); break; }
                    writeRegister(SENSOR_CONFIG, regVal);
                    Serial.printf("✅ Y-axis range set to ±%d mT\n", range);
                    break;
                case 2: // Z
                    regVal &= ~Z_RANGE_MASK;
                    if (range == 25) regVal |= Z_RANGE_25mT;
                    else if (range == 50) regVal |= Z_RANGE_50mT;
                    else if (range == 100) regVal |= Z_RANGE_100mT;
                    else { Serial.println("⚠️ Invalid range! Use 25, 50, or 100"); break; }
                    writeRegister(SENSOR_CONFIG, regVal);
                    Serial.printf("✅ Z-axis range set to ±%d mT\n", range);
                    break;
                default:
                    Serial.println("⚠️ Invalid axis! Use 0=X, 1=Y, 2=Z");
            }
        } else {
            Serial.println("Usage: SET RANGE <axis> <25|50|100>   (axis: 0=X,1=Y,2=Z)");
        }
    }
    // SET SLEEP <ms>
    else if (strncasecmp(cmd, "SET SLEEP", 9) == 0) {
        int sleepMs = atoi(cmd + 9);
        uint16_t regVal = readRegister(SENSOR_CONFIG);

        if      (sleepMs <= 1)    regVal = (regVal & ~0x1C00) | SLEEPTIME_1ms;
        else if (sleepMs <= 5)    regVal = (regVal & ~0x1C00) | SLEEPTIME_5ms;
        else if (sleepMs <= 10)   regVal = (regVal & ~0x1C00) | SLEEPTIME_10ms;
        else if (sleepMs <= 15)   regVal = (regVal & ~0x1C00) | SLEEPTIME_15ms;
        else if (sleepMs <= 20)   regVal = (regVal & ~0x1C00) | SLEEPTIME_20ms;
        else if (sleepMs <= 30)   regVal = (regVal & ~0x1C00) | SLEEPTIME_30ms;
        else if (sleepMs <= 50)   regVal = (regVal & ~0x1C00) | SLEEPTIME_50ms;
        else if (sleepMs <= 100)  regVal = (regVal & ~0x1C00) | SLEEPTIME_100ms;
        else if (sleepMs <= 500)  regVal = (regVal & ~0x1C00) | SLEEPTIME_500ms;
        else                       regVal = (regVal & ~0x1C00) | SLEEPTIME_1000ms;

        writeRegister(SENSOR_CONFIG, regVal);
        Serial.printf("✅ Sleep interval set to ~%d ms\n", sleepMs);
    }
    // LIST pins
    else if (strcasecmp(cmd, "LIST") == 0) {
        Serial.println("📌 Pin Configuration:");
        Serial.printf("   CS   = GPIO %d\n", PIN_CS);
        Serial.printf("   SCK  = GPIO %d\n", PIN_SPI_SCK);
        Serial.printf("   MISO = GPIO %d\n", PIN_SPI_MISO);
        Serial.printf("   MOSI = GPIO %d\n", PIN_SPI_MOSI);
    }
    // CS ON/OFF
    else if (strcasecmp(cmd, "CS ON") == 0) {
        digitalWrite(PIN_CS, LOW);
        Serial.println("📉 CS enabled (LOW)");
    }
    else if (strcasecmp(cmd, "CS OFF") == 0) {
        digitalWrite(PIN_CS, HIGH);
        Serial.println("📈 CS disabled (HIGH)");
    }
    // HELP command
    else if (strcasecmp(cmd, "HELP") == 0) {
        Serial.println("Available commands:");
        Serial.println("  START            - Start sampling");
        Serial.println("  STOP             - Stop sampling");
        Serial.println("  STATUS           - Show sampling status");
        Serial.println("  SET RATE <ms>    - Set sampling interval");
        Serial.println("  SET RANGE <axis> <25|50|100> - Set axis range (0=X,1=Y,2=Z)");
        Serial.println("  SET SLEEP <ms>   - Set sensor sleep interval");
        Serial.println("  LIST             - Show pin assignments");
        Serial.println("  CS ON            - Enable chip select (LOW)");
        Serial.println("  CS OFF           - Disable chip select (HIGH)");
        Serial.println("  HELP             - Show this help");
    }
    // Unknown command
    else {
        Serial.println("Unknown command. Type HELP for a list of commands.");
    }
}

void handleSerialInput() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdIndex > 0) {
        cmdBuffer[cmdIndex] = '\0';
        processCommand(cmdBuffer);
        cmdIndex = 0;
      }
    } else if (cmdIndex < CMD_BUF_LEN - 1) {
      cmdBuffer[cmdIndex++] = c;
    }
  }
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  delay(100);

  //writeRegister(0x00, 0b0000000000101010); // continuous mode + CRC enabled
  //writeRegister(0x01, 0b0000000011100000); // SENSOR_CONFIG: XYZ, ±25 mT
  //writeRegister(0x02, 0b0000000000000000); // SYSTEM_CONFIG: standard result

  // --- Device Config ---
  // Active measure mode + CRC enabled + 4-wire SPI
  uint16_t dev_cfg = OPERATING_MODE_activeMeasureMode 
                   | CRC_DIS_CRCenabled
                   | TMAG_SPI_MODE_SPI4;
  writeRegister(DEVICE_CONFIG, dev_cfg);

  // --- Sensor Config ---
  // Enable XYZ channels, ±25mT range on all axes
  uint16_t sens_cfg = MAG_CH_EN_XYZenaled
                    | X_RANGE_25mT
                    | Y_RANGE_25mT
                    | Z_RANGE_25mT;
  writeRegister(SENSOR_CONFIG, sens_cfg);

  // --- System Config ---
  // 32-bit result format, temperature enabled, magnetic enabled
  uint16_t sys_cfg = DATA_TYPE_32bit
                   | T_CH_EN_enabled
                   | MAG_TEMPCO_compensated;
  writeRegister(SYSTEM_CONFIG, sys_cfg);

  Serial.println("✅ TMAG5170 configured with CRC enabled.");
  Serial.println("Commands: START | STOP | STATUS | SET RATE <ms> | LIST | CS ON | CS OFF");
}

// ---------- Loop ----------
void loop() {
  handleSerialInput();

  if (samplingEnabled && millis() - lastSampleTime >= sampleInterval) {
    lastSampleTime = millis();

    int16_t x_raw = (int16_t)readRegister(X_CH_RESULT);
    int16_t y_raw = (int16_t)readRegister(Y_CH_RESULT);
    int16_t z_raw = (int16_t)readRegister(Z_CH_RESULT);
    int16_t t_raw = (int16_t)readRegister(TEMP_RESULT);

    float x_mT = (float)x_raw / 1308.0f;
    float y_mT = (float)y_raw / 1308.0f;
    float z_mT = (float)z_raw / 1308.0f;
    float tempC = 25.0f + ((float)t_raw - 17522.0f) / 60.0f;

    Serial.printf("X=%.3f mT, Y=%.3f mT, Z=%.3f mT, Temp=%.2f °C\n",
                  x_mT, y_mT, z_mT, tempC);

    checkStatus();
  }
}
