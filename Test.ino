//D:\Homework\Forsberg Design 2 Schematic\Design2\Test
//"D:\Homework\Forsberg Design 2 Schematic\Design2\Test\grab_arducam_frame.py"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HardwareSerial.h>

#include "ov2640_regs.h"   // your regs header

// ========= SECOND UART FOR ESP8266 (PB11=RX, PB10=TX) =========
HardwareSerial ESP_SERIAL(PB11, PB10);   // USART on PB11 (RX), PB10 (TX)

// ========= CAN using HAL directly (bxCAN for STM32F0) =========
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_can.h"

CAN_HandleTypeDef   hcan;
CAN_RxHeaderTypeDef RxHeader;
uint8_t             RxData[8];
bool                can_initialized = false;

// interrupt flag: set in ISR, consumed in loop()
volatile bool       can_rx_pending  = false;

// === CAN loopback / debug state ===
bool     canTestLoop     = false;
uint32_t lastCanTestTime = 0;
uint32_t can_loop_count  = 0;

// ========= GPS STATE FROM CAN (ASCII "LA.."/"LO..") =========
bool     gps_frame_seen   = false;
bool     gps_lock         = false;
uint8_t  gps_raw_bytes[8];
uint8_t  gps_raw_len      = 0;
uint32_t gps_last_id      = 0;

// Raw ASCII LA / LO strings and decoded decimal degrees
bool     gps_have_lat     = false;
bool     gps_have_lon     = false;
bool     gps_latched      = false;   // we have both LA/LO
bool     gps_fix_valid    = false;   // LA/LO not blank

float    gps_latitude     = 0.0f;
float    gps_longitude    = 0.0f;

char     gps_la_str[9]    = "LA00.00N";  // + '\0'
char     gps_lo_str[9]    = "LO000.0E";

#define CAN_ID_GPS_ASCII   0x101    // GPS board LA/LO ID

// ========= I2C ADDRESSES =========
#define OV2640_ADDR   0x30      // Camera SCCB
#define OLED_ADDR     0x3C      // SSD1306
#define VCNL_ADDR     0x60      // VCNL3040

// ========= I2C PINS =========
#define I2C_SCL_PIN   PB6
#define I2C_SDA_PIN   PB7

// ========= OLED =========
#define OLED_WIDTH    128
#define OLED_HEIGHT   64
#define OLED_RESET    -1
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);

bool foundOLED  = false;

// ========= SOFT-SPI PINS (shared) =========
#define SOFT_SCK_PIN   PB13
#define SOFT_MOSI_PIN  PB14
#define SOFT_MISO_PIN  PB15

#define CAM_CS_PIN     PB12     // ArduCHIP CS
#define SRAM_CS_PIN    PA4      // IS62WVS2568 CS

// ========= ARDUCHIP REGISTERS / CONSTANTS =========
#define ARDUCHIP_TEST1       0x00
#define ARDUCHIP_FIFO        0x04
#define ARDUCHIP_TRIG        0x41
#define FIFO_SIZE1           0x42
#define FIFO_SIZE2           0x43
#define FIFO_SIZE3           0x44

#define FIFO_CLEAR_MASK      0x01
#define FIFO_START_MASK      0x02

#define CAP_DONE_MASK        0x08
#define BURST_FIFO_READ      0x3C

// ========= SRAM (IS62WVS2568) SPI COMMANDS =========
#define SRAM_CMD_WRITE  0x02
#define SRAM_CMD_READ   0x03
#define SRAM_CMD_WRMR   0x01
#define SRAM_CMD_RDMR   0x05

// ========= GLOBAL FLAGS / STATE =========
bool    cam_spi_ok        = false;
bool    cam_i2c_ok        = false;
bool    cam_inited        = false;

bool    sram_ok           = false;
uint8_t sram_written      = 0xA5;
uint8_t sram_read_sample  = 0x00;
uint8_t sram_mr_before    = 0x00;
uint8_t sram_mr_after     = 0x00;

bool     foundVCNL        = false;
uint16_t vcnl_last_prox   = 0;
bool     proxLoop         = false;

String   cmdBuffer;

// ========= FORWARD DECLARATIONS =========
bool     can_init();
bool     can_send_test(uint32_t id, const uint8_t* data, uint8_t len);
void     sendProxToESP(uint16_t prox);
void     sendGpsAsciiToESP(float lat, float lon);
void     oledShowStatus();
void     printHelp();
void     handleCommand(const String &rawCmd);
void     sendError(const char *msg);
bool     testSRAM_soft();
uint16_t vcnlReadProximity();
bool     vcnlInit();
bool     arduchipTestSPI();
void     captureAndStreamJPEG();
void     captureAndStreamJPEGTo8051();

// ========= SIMPLE ASCII HELPERS =========
bool is_blank_la(const char *la) {
  // "LA00.00N" -> treat 00.00 as blank
  return (la[2] == '0' && la[3] == '0' && la[5] == '0' && la[6] == '0');
}

bool is_blank_lo(const char *lo) {
  // "LO000.0E" -> treat 000.0 as blank
  return (lo[2] == '0' && lo[3] == '0' && lo[4] == '0' && lo[6] == '0');
}

// VERY SIMPLE conversion from "LAdd.mmN" / "LOddd.mE" to decimal degrees
float decode_la_deg(const char *la) {
  int   deg      = (la[2] - '0') * 10 + (la[3] - '0');
  int   mins_int = (la[5] - '0') * 10 + (la[6] - '0');
  float minutes  = (float)mins_int;
  float result   = (float)deg + minutes / 60.0f;
  if (la[7] == 'S') result = -result;
  return result;
}

float decode_lo_deg(const char *lo) {
  int   deg      = (lo[2] - '0') * 100 + (lo[3] - '0') * 10 + (lo[4] - '0');
  int   mins_int = (lo[6] - '0');      // one digit -> *10 minutes
  float minutes  = (float)mins_int * 10.0f;
  float result   = (float)deg + minutes / 60.0f;
  if (lo[7] == 'W') result = -result;
  return result;
}

// ========= CAN INIT (100 kbps, NORMAL, RX interrupt) =========
bool can_init() {
  __HAL_RCC_CAN1_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9;   // PB8=RX, PB9=TX
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_CAN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  hcan.Instance = CAN;

  // 48MHz / (Prescaler * (1 + BS1 + BS2)) = 100 kbps
  // 48MHz / (120 * (1 + 2 + 1)) = 100000
  hcan.Init.Prescaler           = 120;               // 100 kbps
  hcan.Init.Mode                = CAN_MODE_NORMAL;   // *** NORMAL for real GPS board ***
  hcan.Init.SyncJumpWidth       = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1            = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2            = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode   = DISABLE;
  hcan.Init.AutoBusOff          = DISABLE;
  hcan.Init.AutoWakeUp          = DISABLE;
  hcan.Init.AutoRetransmission  = ENABLE;
  hcan.Init.ReceiveFifoLocked   = DISABLE;
  hcan.Init.TransmitFifoPriority= DISABLE;
  
  if (HAL_CAN_Init(&hcan) != HAL_OK) {
    return false;
  }
  
  // Accept ALL standard/ext IDs into FIFO0
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterBank           = 0;
  sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh         = 0x0000;
  sFilterConfig.FilterIdLow          = 0x0000;
  sFilterConfig.FilterMaskIdHigh     = 0x0000;
  sFilterConfig.FilterMaskIdLow      = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation     = ENABLE;
  
  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
    return false;
  }
  
  if (HAL_CAN_Start(&hcan) != HAL_OK) {
    return false;
  }

  // Enable RX FIFO0 message-pending interrupt
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
    return false;
  }

  // On STM32F072, CAN shares the CEC_CAN_IRQn
  HAL_NVIC_SetPriority(CEC_CAN_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(CEC_CAN_IRQn);
  
  can_initialized = true;
  return true;
}

// CAN TX helper (used by "test" and test on/off)
bool can_send_test(uint32_t id, const uint8_t* data, uint8_t len) {
  if (!can_initialized) return false;
  if (len > 8) len = 8;

  CAN_TxHeaderTypeDef TxHeader;
  uint32_t txMailbox;

  TxHeader.StdId = id;
  TxHeader.ExtId = 0;
  TxHeader.RTR   = CAN_RTR_DATA;
  TxHeader.IDE   = CAN_ID_STD;
  TxHeader.DLC   = len;
  TxHeader.TransmitGlobalTime = DISABLE;

  HAL_StatusTypeDef st = HAL_CAN_AddTxMessage(&hcan, &TxHeader, (uint8_t*)data, &txMailbox);
  return (st == HAL_OK);
}

// ========= CAN IRQ / CALLBACKS =========

// Actual IRQ handler name for F0 CAN (CEC_CAN_IRQn)
extern "C" void CEC_CAN_IRQHandler(void) {
  HAL_CAN_IRQHandler(&hcan);
}

// RX FIFO0 callback: copy frame into globals, set flag
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_cb) {
  if (hcan_cb->Instance != CAN) return;

  if (HAL_CAN_GetRxMessage(hcan_cb, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
    can_rx_pending = true;
  }
}

// ========= SOFT-SPI HELPERS =========
void softSPIInit() {
  pinMode(SOFT_SCK_PIN, OUTPUT);
  pinMode(SOFT_MOSI_PIN, OUTPUT);
  pinMode(SOFT_MISO_PIN, INPUT);

  digitalWrite(SOFT_SCK_PIN, LOW);
  digitalWrite(SOFT_MOSI_PIN, LOW);

  pinMode(CAM_CS_PIN, OUTPUT);
  pinMode(SRAM_CS_PIN, OUTPUT);
  digitalWrite(CAM_CS_PIN, HIGH);
  digitalWrite(SRAM_CS_PIN, HIGH);
}

uint8_t softSPITransfer(uint8_t out) {
  uint8_t in = 0;
  for (int i = 7; i >= 0; i--) {
    digitalWrite(SOFT_MOSI_PIN, (out >> i) & 0x01);
    digitalWrite(SOFT_SCK_PIN, HIGH);
    in <<= 1;
    if (digitalRead(SOFT_MISO_PIN)) {
      in |= 0x01;
    }
    digitalWrite(SOFT_SCK_PIN, LOW);
  }
  return in;
}

// ========= ARDUCHIP REGISTER R/W =========
void arduchipWriteReg(uint8_t addr, uint8_t val) {
  digitalWrite(SRAM_CS_PIN, HIGH);
  digitalWrite(CAM_CS_PIN, HIGH);
  digitalWrite(CAM_CS_PIN, LOW);
  softSPITransfer(addr | 0x80);
  softSPITransfer(val);
  digitalWrite(CAM_CS_PIN, HIGH);
}

uint8_t arduchipReadReg(uint8_t addr) {
  digitalWrite(SRAM_CS_PIN, HIGH);
  digitalWrite(CAM_CS_PIN, HIGH);
  digitalWrite(CAM_CS_PIN, LOW);
  softSPITransfer(addr & 0x7F);
  uint8_t val = softSPITransfer(0x00);
  digitalWrite(CAM_CS_PIN, HIGH);
  return val;
}

void arduchipResetCPLD() {
  arduchipWriteReg(0x07, 0x80);
  delay(50);
  arduchipWriteReg(0x07, 0x00);
  delay(50);
}

bool arduchipTestSPI() {
  arduchipWriteReg(ARDUCHIP_TEST1, 0x55);
  delay(5);
  uint8_t v = arduchipReadReg(ARDUCHIP_TEST1);
  cam_spi_ok = (v == 0x55);
  return cam_spi_ok;
}

void arduchipFlushFIFO() {
  arduchipWriteReg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

void arduchipClearFIFODoneFlag() {
  arduchipWriteReg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

void arduchipStartCapture() {
  arduchipWriteReg(ARDUCHIP_FIFO, FIFO_START_MASK);
}

bool arduchipWaitCaptureDone(uint32_t timeout_ms) {
  uint32_t start = millis();
  while ((millis() - start) < timeout_ms) {
    uint8_t trig = arduchipReadReg(ARDUCHIP_TRIG);
    if (trig & CAP_DONE_MASK) {
      return true;
    }
    delay(5);
  }
  return false;
}

uint32_t arduchipReadFIFOLength() {
  uint8_t size1 = arduchipReadReg(FIFO_SIZE1);
  uint8_t size2 = arduchipReadReg(FIFO_SIZE2);
  uint8_t size3 = arduchipReadReg(FIFO_SIZE3);
  return ((uint32_t)size3 << 16) | ((uint32_t)size2 << 8) | (uint32_t)size1;
}

void arduchipStartBurstRead() {
  digitalWrite(SRAM_CS_PIN, HIGH);
  digitalWrite(CAM_CS_PIN, HIGH);
  digitalWrite(CAM_CS_PIN, LOW);
  softSPITransfer(BURST_FIFO_READ);
}

void arduchipEndBurstRead() {
  digitalWrite(CAM_CS_PIN, HIGH);
}

// ========= OV2640 SENSOR (I2C) =========
bool ovWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(OV2640_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

uint8_t ovReadReg(uint8_t reg) {
  Wire.beginTransmission(OV2640_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(OV2640_ADDR, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0xFF;
}

bool ovApplyTable(const struct sensor_reg *tbl) {
  uint16_t i = 0;
  while (!(tbl[i].reg == 0xFF && tbl[i].val == 0xFF)) {
    if (!ovWriteReg(tbl[i].reg, tbl[i].val)) {
      return false;
    }
    i++;
  }
  return true;
}

bool ov2640Detect() {
  ovWriteReg(0xFF, 0x01);
  uint8_t high = ovReadReg(OV2640_CHIPID_HIGH);
  uint8_t low  = ovReadReg(OV2640_CHIPID_LOW);
  if (high == 0x26 && (low == 0x41 || low == 0x42)) {
    cam_i2c_ok = true;
    return true;
  }
  cam_i2c_ok = false;
  return false;
}

bool ov2640Init_JPEG_160x120() {
  ovWriteReg(0xFF, 0x01);
  ovWriteReg(0x12, 0x80);
  delay(80);
  if (!ovApplyTable(OV2640_JPEG_INIT))       return false;
  if (!ovApplyTable(OV2640_YUV422))          return false;
  if (!ovApplyTable(OV2640_JPEG))            return false;
  if (!ovApplyTable(OV2640_160x120_JPEG))    return false;
  return true;
}

// ========= SRAM =========
uint8_t sramReadModeReg() {
  digitalWrite(CAM_CS_PIN, HIGH);
  digitalWrite(SRAM_CS_PIN, HIGH);
  digitalWrite(SRAM_CS_PIN, LOW);
  softSPITransfer(SRAM_CMD_RDMR);
  uint8_t mr = softSPITransfer(0x00);
  digitalWrite(SRAM_CS_PIN, HIGH);
  return mr;
}

void sramWriteModeReg(uint8_t mr) {
  digitalWrite(CAM_CS_PIN, HIGH);
  digitalWrite(SRAM_CS_PIN, HIGH);
  digitalWrite(SRAM_CS_PIN, LOW);
  softSPITransfer(SRAM_CMD_WRMR);
  softSPITransfer(mr);
  digitalWrite(SRAM_CS_PIN, HIGH);
}

void sramWriteByte(uint32_t addr, uint8_t data) {
  digitalWrite(CAM_CS_PIN, HIGH);
  digitalWrite(SRAM_CS_PIN, HIGH);
  digitalWrite(SRAM_CS_PIN, LOW);
  softSPITransfer(SRAM_CMD_WRITE);
  softSPITransfer((addr >> 16) & 0xFF);
  softSPITransfer((addr >> 8)  & 0xFF);
  softSPITransfer(addr & 0xFF);
  softSPITransfer(data);
  digitalWrite(SRAM_CS_PIN, HIGH);
}

uint8_t sramReadByte(uint32_t addr) {
  digitalWrite(CAM_CS_PIN, HIGH);
  digitalWrite(SRAM_CS_PIN, HIGH);
  digitalWrite(SRAM_CS_PIN, LOW);
  softSPITransfer(SRAM_CMD_READ);
  softSPITransfer((addr >> 16) & 0xFF);
  softSPITransfer((addr >> 8)  & 0xFF);
  softSPITransfer(addr & 0xFF);
  (void)softSPITransfer(0x00);
  uint8_t val = softSPITransfer(0x00);
  digitalWrite(SRAM_CS_PIN, HIGH);
  return val;
}

bool testSRAM_soft() {
  uint32_t addr = 0x000010;
  sram_written   = 0xA5;
  sram_mr_before = sramReadModeReg();
  sramWriteModeReg(0x00);
  delay(2);
  sram_mr_after  = sramReadModeReg();
  sramWriteByte(addr, sram_written);
  delay(5);
  sram_read_sample = sramReadByte(addr);
  sram_ok = (sram_read_sample == sram_written);
  return sram_ok;
}

// ========= VCNL3040 =========
void vcnlWrite16(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(VCNL_ADDR);
  Wire.write(reg);
  Wire.write(value & 0xFF);
  Wire.write((value >> 8) & 0xFF);
  Wire.endTransmission();
}

uint16_t vcnlRead16(uint8_t reg) {
  uint8_t lo = 0, hi = 0;
  Wire.beginTransmission(VCNL_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(VCNL_ADDR, (uint8_t)2);
  if (Wire.available() >= 2) {
    lo = Wire.read();
    hi = Wire.read();
  }
  return ((uint16_t)hi << 8) | lo;
}

bool vcnlInit() {
  uint16_t conf1 = vcnlRead16(0x03);
  conf1 &= ~0x0001;
  vcnlWrite16(0x03, conf1);
  uint16_t conf4 = vcnlRead16(0x04);
  conf4 &= ~0x0007;
  conf4 |= 0x0004;
  vcnlWrite16(0x04, conf4);
  delay(10);
  return true;
}

uint16_t vcnlReadProximity() {
  return vcnlRead16(0x08);
}

// ========= PROX TO ESP =========
void sendProxToESP(uint16_t prox) {
  ESP_SERIAL.print("TOF");
  ESP_SERIAL.print(prox);
  ESP_SERIAL.print('\n');
}

// ========= GPS ASCII TO ESP =========
void sendGpsAsciiToESP(float lat, float lon) {
  ESP_SERIAL.print("GPS");
  ESP_SERIAL.print(lat, 6);
  ESP_SERIAL.print(',');
  ESP_SERIAL.print(lon, 6);
  ESP_SERIAL.print('\n');
}

// ========= OLED =========
void oledShowStatus() {
  if (!foundOLED) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Header
  display.setCursor(0, 0);
  display.println(" HW STATUS ");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  display.setCursor(0, 12);
  // Line: Cam + SRAM + Prox
  display.print("Cam:");
  display.print(cam_spi_ok ? "OK " : "X  ");
  display.print("SRAM:");
  display.print(sram_ok ? "OK " : "X  ");
  display.print("P:");
  display.println(vcnl_last_prox);

  // LA / LO lines
  display.print("LA:");
  display.println(gps_la_str);
  display.print("LO:");
  display.println(gps_lo_str);

  display.display();
}

void printStatusSerial() {
  Serial.println(F("=== STATUS ==="));
  Serial.print(F("CamI2C: "));  Serial.println(cam_i2c_ok ? F("OK") : F("X"));
  Serial.print(F("CamSPI: "));  Serial.println(cam_spi_ok ? F("OK") : F("X"));
  Serial.print(F("SRAM: "));    Serial.print(sram_ok ? F("OK ") : F("X  "));
  Serial.print(F("0x"));
  if (sram_read_sample < 16) Serial.print('0');
  Serial.println(sram_read_sample, HEX);

  Serial.print(F("VCNL prox: "));
  Serial.println(vcnl_last_prox);

  Serial.print(F("GPS frames: "));
  Serial.println(gps_frame_seen ? F("YES") : F("NO"));

  Serial.print(F("Latched: "));
  Serial.println(gps_latched ? F("YES") : F("NO"));

  Serial.print(F("Fix: "));
  Serial.println(gps_fix_valid ? F("OK") : F("NO"));

  Serial.print(F("LA: "));
  Serial.println(gps_la_str);
  Serial.print(F("LO: "));
  Serial.println(gps_lo_str);

  Serial.print(F("CAN test: "));
  Serial.print(canTestLoop ? F("ON ") : F("OFF"));
  Serial.print(F("  #"));
  Serial.println(can_loop_count);
  Serial.println();
}

// ========= ERROR TX =========
void sendError(const char *msg) {
  Serial.write('E');
  Serial.write('R');
  Serial.write('R');
  Serial.write('\n');
  Serial.println(msg);
}

// ========= JPEG CAPTURE: PC + ESP =========
void captureAndStreamJPEG() {
  if (!cam_spi_ok || !cam_i2c_ok || !cam_inited) {
    sendError("Cam not ready");
    return;
  }

  arduchipFlushFIFO();
  arduchipClearFIFODoneFlag();
  arduchipStartCapture();

  if (!arduchipWaitCaptureDone(3000)) {
    sendError("CAP timeout");
    return;
  }

  uint32_t length = arduchipReadFIFOLength();
  if (length == 0 || length > 60000UL) {
    sendError("Bad len");
    arduchipClearFIFODoneFlag();
    return;
  }

  Serial.write('I');
  Serial.write('M');
  Serial.write('G');

  uint32_t img_len = length;
  Serial.write((uint8_t *)&img_len, 4);

  ESP_SERIAL.write('I');
  ESP_SERIAL.write('M');
  ESP_SERIAL.write('G');

  arduchipStartBurstRead();
  for (uint32_t i = 0; i < length; i++) {
    uint8_t b = softSPITransfer(0x00);
    Serial.write(b);
    ESP_SERIAL.write(b);
  }
  arduchipEndBurstRead();
  arduchipClearFIFODoneFlag();
}

// ========= JPEG capture to 8051 over ESP_SERIAL =========
void captureAndStreamJPEGTo8051() {
  if (!cam_spi_ok || !cam_i2c_ok || !cam_inited) {
    sendError("Cam not ready (8051)");
    return;
  }

  Serial.println(F("[8051] JPEG 160x120"));

  arduchipFlushFIFO();
  arduchipClearFIFODoneFlag();
  arduchipStartCapture();

  if (!arduchipWaitCaptureDone(3000)) {
    sendError("CAP tmo 8051");
    return;
  }

  uint32_t fifo_len = arduchipReadFIFOLength();
  Serial.print(F("[8051] len="));
  Serial.println(fifo_len);
  
  if (fifo_len == 0 || fifo_len > 60000UL) {
    sendError("Bad len");
    return;
  }

  ESP_SERIAL.write('I');
  ESP_SERIAL.write('M');
  ESP_SERIAL.write('G');

  arduchipStartBurstRead();
  for (uint32_t i = 0; i < fifo_len; i++) {
    uint8_t b = softSPITransfer(0x00);
    ESP_SERIAL.write(b);
  }
  arduchipEndBurstRead();
  arduchipClearFIFODoneFlag();

  Serial.println(F("[8051] JPEG sent"));
}

// ========= COMMANDS =========
void printHelp() {
  Serial.println(F("===== COMMANDS ====="));
  Serial.println(F(" help         : show this list"));
  Serial.println(F(" status       : re-run tests, print + OLED"));
  Serial.println(F(" cam          : ArduCAM SPI test"));
  Serial.println(F(" sram         : SRAM test"));
  Serial.println(F(" prox         : one VCNL proximity read -> ESP"));
  Serial.println(F(" proxloop on  : continuous proximity -> ESP"));
  Serial.println(F(" proxloop off : stop continuous prox"));
  Serial.println(F(" oled         : refresh OLED status"));
  Serial.println(F(" gps          : print + SEND current GPS / LA / LO to ESP"));
  Serial.println(F(" test         : send indoor LA00.00N / LO000.0E via CAN + ESP"));
  Serial.println(F(" test on      : CAN test spam on 0x555"));
  Serial.println(F(" test off     : stop CAN test spam"));
  Serial.println(F(" C            : capture JPEG to PC + ESP"));
  Serial.println(F(" Z            : capture JPEG to 8051 over ESP"));
  Serial.println(F("===================="));
  Serial.println();
}

void handleCommand(const String &rawCmd) {
  String cmd = rawCmd;
  cmd.trim();
  cmd.toLowerCase();
  if (cmd.length() == 0) return;

  Serial.print(F("> "));
  Serial.println(cmd);

  if (cmd == "help") {
    printHelp();

  } else if (cmd == "status") {
    cam_spi_ok = arduchipTestSPI();
    sram_ok    = testSRAM_soft();
    if (foundVCNL) {
      vcnl_last_prox = vcnlReadProximity();
    }
    printStatusSerial();
    oledShowStatus();

  } else if (cmd == "cam") {
    cam_spi_ok = arduchipTestSPI();
    Serial.print(F("CamSPI: "));
    Serial.println(cam_spi_ok ? F("OK") : F("X"));

  } else if (cmd == "sram") {
    sram_ok = testSRAM_soft();
    printStatusSerial();
    oledShowStatus();

  } else if (cmd == "prox") {
    if (!foundVCNL) {
      Serial.println(F("VCNL?"));
    } else {
      vcnl_last_prox = vcnlReadProximity();
      Serial.print(F("prox="));
      Serial.println(vcnl_last_prox);
      oledShowStatus();

      sendProxToESP(vcnl_last_prox);
    }

  } else if (cmd == "proxloop on") {
    proxLoop = true;
    Serial.println(F("proxloop ON"));

  } else if (cmd == "proxloop off") {
    proxLoop = false;
    Serial.println(F("proxloop OFF"));

  } else if (cmd == "oled") {
    oledShowStatus();
    Serial.println(F("oled"));

  } else if (cmd == "gps") {
    if (!gps_frame_seen) {
      Serial.println(F("GPS none"));
    } else {
      Serial.print(F("raw: "));
      for (uint8_t i = 0; i < gps_raw_len; i++) {
        if (gps_raw_bytes[i] < 0x10) Serial.print('0');
        Serial.print(gps_raw_bytes[i], HEX);
        Serial.print(' ');
      }
      Serial.println();

      Serial.print(F("LA: "));
      Serial.println(gps_la_str);
      Serial.print(F("LO: "));
      Serial.println(gps_lo_str);
      Serial.print(F("latched: "));
      Serial.println(gps_latched ? F("Y") : F("N"));
      Serial.print(F("fix: "));
      Serial.println(gps_fix_valid ? F("OK") : F("NO"));
      if (gps_fix_valid) {
        Serial.print(F("lat="));
        Serial.println(gps_latitude, 6);
        Serial.print(F("lon="));
        Serial.println(gps_longitude, 6);
      }

      // NEW: always send whatever LA/LO we have to the ESP if present
      if (gps_have_lat && gps_have_lon) {
        float lat = decode_la_deg(gps_la_str);
        float lon = decode_lo_deg(gps_lo_str);
        sendGpsAsciiToESP(lat, lon);
        Serial.println(F("GPS sent -> ESP"));
      } else {
        Serial.println(F("GPS not sent to ESP (no LA/LO yet)"));
      }
    }

  // Indoor-style blank GPS test (LA00.00N / LO000.0E)
  } else if (cmd == "test") {
    if (!can_initialized) {
      Serial.println(F("CAN?"));
      return;
    }

    uint8_t fixedlat[8] = { 'L','A','0','0','.','0','0','N' }; // "LA00.00N"
    uint8_t fixedlon[8] = { 'L','O','0','0','0','.','0','E' }; // "LO000.0E"

    if (can_send_test(CAN_ID_GPS_ASCII, fixedlat, 8)) {
      Serial.println(F("LA00.00N sent"));
    } else {
      Serial.println(F("LA fail"));
    }

    delay(5);

    if (can_send_test(CAN_ID_GPS_ASCII, fixedlon, 8)) {
      Serial.println(F("LO000.0E sent"));
    } else {
      Serial.println(F("LO fail"));
    }

    memcpy(gps_la_str, fixedlat, 8);
    gps_la_str[8] = '\0';
    memcpy(gps_lo_str, fixedlon, 8);
    gps_lo_str[8] = '\0';

    gps_have_lat   = true;
    gps_have_lon   = true;
    gps_frame_seen = true;

    // Got LA/LO but this is indoor/blank -> mark as NO FIX
    gps_latched    = true;
    gps_fix_valid  = false;
    gps_lock       = false;

    // NEW: compute floats + send to ESP immediately (will be 0,0)
    gps_latitude  = decode_la_deg(gps_la_str);
    gps_longitude = decode_lo_deg(gps_lo_str);
    sendGpsAsciiToESP(gps_latitude, gps_longitude);
    Serial.println(F("Test GPS -> ESP"));

    oledShowStatus();

  } else if (cmd == "test on") {
    can_loop_count = 0;
    canTestLoop    = true;
    Serial.println(F("CAN test ON"));

  } else if (cmd == "test off") {
    canTestLoop = false;
    Serial.print(F("CAN test OFF #"));
    Serial.println(can_loop_count);

  } else {
    Serial.println(F("unk cmd"));
  }
}

// ========= SETUP =========
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  ESP_SERIAL.begin(115200);

  Wire.setSCL(I2C_SCL_PIN);
  Wire.setSDA(I2C_SDA_PIN);
  Wire.begin();
  Wire.setClock(100000);
  
  if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    foundOLED = true;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("STM32F072 START");
    display.display();
  }
  
  softSPIInit();
  
  Serial.println();
  Serial.println(F("STM32F072 + Cam + SRAM + VCNL + CAN GPS"));
  Serial.println(F("Reset ArduCAM"));
  arduchipResetCPLD();
  
  cam_spi_ok = arduchipTestSPI();
  Serial.print(F("CamSPI: "));
  Serial.println(cam_spi_ok ? F("OK") : F("X"));
  
  Serial.println(F("OV2640 detect"));
  cam_i2c_ok = ov2640Detect();
  Serial.print(F("OV2640: "));
  Serial.println(cam_i2c_ok ? F("OK") : F("X"));
  
  Serial.println(F("OV2640 JPEG 160x120"));
  if (ov2640Init_JPEG_160x120()) {
    cam_inited = true;
    Serial.println(F("Cam init OK"));
  } else {
    Serial.println(F("Cam init X"));
  }

  Serial.println(F("SRAM test"));
  sram_ok = testSRAM_soft();
  
  Serial.println(F("VCNL check"));
  Wire.beginTransmission(VCNL_ADDR);
  if (Wire.endTransmission() == 0) {
    foundVCNL = true;
    vcnlInit();
    vcnl_last_prox = vcnlReadProximity();
    Serial.println(F("VCNL OK"));
  } else {
    foundVCNL = false;
    Serial.println(F("VCNL X"));
  }
  
  Serial.println(F("CAN init..."));
  if (can_init()) {
    Serial.println(F("CAN OK (NORMAL 100k, RX IRQ)"));
  } else {
    Serial.println(F("CAN X"));
  }
  
  printStatusSerial();
  oledShowStatus();
  
  Serial.println(F("Ready. Type 'help' for commands."));
}

// ========= LOOP =========
void loop() {
  // Serial command handling
  while (Serial.available()) {
    char c = (char)Serial.read();

    // One-char camera commands
    if (c == 'C' || c == 'c') {
      captureAndStreamJPEG();
      continue;
    }
    if (c == 'Z' || c == 'z') {      // changed from R to Z
      captureAndStreamJPEGTo8051();
      continue;
    }

    if (c == '\r') continue;
    if (c == '\n') {
      handleCommand(cmdBuffer);
      cmdBuffer = "";
    } else {
      cmdBuffer += c;
      if (cmdBuffer.length() > 80) cmdBuffer = "";
    }
  }

  // CAN debug spam: 1 frame/sec on 0x555 (TX only)
  if (canTestLoop && can_initialized) {
    if (millis() - lastCanTestTime > 1000) {
      lastCanTestTime = millis();

      uint8_t testData[8] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04 };
      if (can_send_test(0x555, testData, 8)) {
        can_loop_count++;
        Serial.print(F("CAN 0x555 #"));
        Serial.println(can_loop_count);
      } else {
        Serial.println(F("CAN tx X"));
      }
      oledShowStatus();
    }
  }

  // CAN receive: ANY frame (GPS is specifically on 0x101)
  if (can_rx_pending) {
    noInterrupts();
    can_rx_pending = false;
    interrupts();

    gps_frame_seen = true;
    gps_last_id    = RxHeader.StdId;
    gps_raw_len    = RxHeader.DLC;

    for (uint8_t i = 0; i < 8; i++) {
      gps_raw_bytes[i] = RxData[i];
    }
    
    Serial.print(F("CAN 0x"));
    Serial.print(RxHeader.StdId, HEX);
    Serial.print(F(" len "));
    Serial.print(RxHeader.DLC);
    Serial.print(F(" : "));
    for (uint8_t i = 0; i < RxHeader.DLC; i++) {
      if (RxData[i] < 0x10) Serial.print('0');
      Serial.print(RxData[i], HEX);
      Serial.print(' ');
    }
    Serial.println();

    if (RxHeader.StdId == CAN_ID_GPS_ASCII && RxHeader.DLC == 8) {
      if (RxData[0] == 'L' && RxData[1] == 'A') {
        memcpy(gps_la_str, RxData, 8);
        gps_la_str[8] = '\0';
        gps_have_lat = true;
        Serial.print(F("LA: "));
        Serial.println(gps_la_str);
      } else if (RxData[0] == 'L' && RxData[1] == 'O') {
        memcpy(gps_lo_str, RxData, 8);
        gps_lo_str[8] = '\0';
        gps_have_lon = true;
        Serial.print(F("LO: "));
        Serial.println(gps_lo_str);
      }

      if (gps_have_lat && gps_have_lon) {
        bool blank = is_blank_la(gps_la_str) || is_blank_lo(gps_lo_str);

        if (!blank) {
          gps_latitude   = decode_la_deg(gps_la_str);
          gps_longitude  = decode_lo_deg(gps_lo_str);
          gps_latched    = true;
          gps_fix_valid  = true;
          gps_lock       = true;

          Serial.println(F("GPS FIX"));
          Serial.print(F("lat="));
          Serial.println(gps_latitude, 6);
          Serial.print(F("lon="));
          Serial.println(gps_longitude, 6);

          sendGpsAsciiToESP(gps_latitude, gps_longitude);
        } else {
          gps_latched    = true;   // pair seen
          gps_fix_valid  = false;
          gps_lock       = false;
          Serial.println(F("GPS blank (no fix)"));
        }
      }
    }

    oledShowStatus();
  }

  // VCNL periodic read + PROX→ESP
  static uint32_t lastProx = 0;
  if (foundVCNL && (millis() - lastProx > 250)) {
    lastProx = millis();
    vcnl_last_prox = vcnlReadProximity();
    if (proxLoop) {
      Serial.print(F("prox="));
      Serial.println(vcnl_last_prox);
      sendProxToESP(vcnl_last_prox);
    }
    oledShowStatus();
  }
  
  // Heartbeat LED
  static uint32_t t = 0;
  static bool led = false;
  if (millis() - t > 500) {
    t = millis();
    led = !led;
    digitalWrite(LED_BUILTIN, led);
  }
}
