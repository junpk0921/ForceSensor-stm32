#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <EEPROM.h>
#include <string.h>
#include "HX711.h"

// [ 하드웨어 설정 ]
TwoWire Wire2(PB11, PB10); 

#define HX_DOUT      PA1     
#define HX_SCK       PA0     
#define STATUS_LED   PA8     

#define I2C_SLAVE_ADDR 0x08
#define ADXL_ADDR      0x53

// [ EZMAKER -> STM32 명령 ]
#define CMD_SAVE_CALIB_RATIO  0xC1

// [ EEPROM 저장 주소 ]
#define EEPROM_MAGIC_ADDR     0
#define EEPROM_FACTOR_ADDR    4
#define EEPROM_MAGIC_VALUE    0x46534341UL   // 'FSCA'

// [ 보정 계수 유효 범위 ]
#define CAL_FACTOR_MIN        50.0f
#define CAL_FACTOR_MAX        5000.0f

// [ 출력/계산 상수 ]
const float G_CONST = 9.80665f;
const float DEADBAND_WEIGHT = 0.5f;   // g
const float DEADBAND_ACCEL  = 0.01f;  // g

// [ 데이터 프로토콜 ] - 14Byte
typedef union {
  struct __attribute__((packed)) {
    int32_t weight_g100;      // g x 100
    int32_t force_N1000;      // N x 1000
    int16_t accX_g1000;       // g x 1000
    int16_t accY_g1000;       // g x 1000
    int16_t accZ_g1000;       // g x 1000
  } val;
  uint8_t buffer[14];
} Packet_t;

HX711 scale;

volatile Packet_t txData; 

const float alpha_sync = 0.3f;

// 기본 보정 계수
float base_calibration_factor = 400.3f;

float prev_weight = 0.0f;           
float prev_gx = 0.0f;
float prev_gy = 0.0f;
float prev_gz = 0.0f;

// STM32 내부 가속도 offset
float offset_gx = 0.0f;
float offset_gy = 0.0f;
float offset_gz = 0.0f;

unsigned long lastUpdateTime = 0;
const unsigned long INTERVAL = 10; // 100Hz

// EZMAKER에서 받은 보정 비율 저장 요청
volatile bool pending_calibration_save = false;
volatile uint8_t pending_ratio_bytes[4];

// ------------------------------------------------------------
// EEPROM에서 보정 계수 불러오기
// ------------------------------------------------------------
void loadCalibrationFactor() {
  uint32_t magic = 0;
  float saved_factor = 0.0f;

  EEPROM.get(EEPROM_MAGIC_ADDR, magic);
  EEPROM.get(EEPROM_FACTOR_ADDR, saved_factor);

  if (magic == EEPROM_MAGIC_VALUE &&
      saved_factor >= CAL_FACTOR_MIN &&
      saved_factor <= CAL_FACTOR_MAX) {
    base_calibration_factor = saved_factor;
  }
}

// ------------------------------------------------------------
// EEPROM에 보정 계수 저장
// ------------------------------------------------------------
void saveCalibrationFactor(float factor) {
  if (factor < CAL_FACTOR_MIN || factor > CAL_FACTOR_MAX) {
    return;
  }

  uint32_t magic = EEPROM_MAGIC_VALUE;

  EEPROM.put(EEPROM_MAGIC_ADDR, magic);
  EEPROM.put(EEPROM_FACTOR_ADDR, factor);
}

// ------------------------------------------------------------
// ADXL345 1회 읽기
// ------------------------------------------------------------
bool readADXL345(float &gx, float &gy, float &gz) {
  Wire2.beginTransmission(ADXL_ADDR);
  Wire2.write(0x32);

  if (Wire2.endTransmission(false) != 0) {
    return false;
  }

  Wire2.requestFrom(ADXL_ADDR, (uint8_t)6);

  if (Wire2.available() != 6) {
    return false;
  }

  int16_t ax_raw = (Wire2.read() | (Wire2.read() << 8));
  int16_t ay_raw = (Wire2.read() | (Wire2.read() << 8));
  int16_t az_raw = (Wire2.read() | (Wire2.read() << 8));

  gx = ax_raw / 256.0f;
  gy = ay_raw / 256.0f;
  gz = az_raw / 256.0f;

  return true;
}

// ------------------------------------------------------------
// STM32 내부 가속도 offset 보정
// 부팅 시 현재 자세를 기준점으로 저장
// ------------------------------------------------------------
void calibrateAccelOffset() {
  float sum_gx = 0.0f;
  float sum_gy = 0.0f;
  float sum_gz = 0.0f;

  int valid_samples = 0;
  const int samples = 50;

  for (int i = 0; i < samples; i++) {
    float gx, gy, gz;

    if (readADXL345(gx, gy, gz)) {
      sum_gx += gx;
      sum_gy += gy;
      sum_gz += gz;
      valid_samples++;
    }

    delay(5);
  }

  if (valid_samples > 0) {
    offset_gx = sum_gx / valid_samples;
    offset_gy = sum_gy / valid_samples;
    offset_gz = sum_gz / valid_samples;
  }
}

// ------------------------------------------------------------
// EZMAKER에서 I2C Write 명령 수신
// CMD_SAVE_CALIB_RATIO + float correction_ratio, 총 5바이트
// ------------------------------------------------------------
void onI2CReceive(int numBytes) {
  if (numBytes < 5) {
    while (Wire.available()) Wire.read();
    return;
  }

  uint8_t cmd = Wire.read();

  if (cmd == CMD_SAVE_CALIB_RATIO) {
    for (int i = 0; i < 4; i++) {
      if (Wire.available()) {
        pending_ratio_bytes[i] = Wire.read();
      }
    }

    pending_calibration_save = true;
  }

  while (Wire.available()) {
    Wire.read();
  }
}

// ------------------------------------------------------------
// EZMAKER에서 I2C Read 요청 시 최종 계산 패킷 전송
// ------------------------------------------------------------
void onI2CRequest() {
  Wire.write((const uint8_t*)txData.buffer, 14);

  digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
}

// ------------------------------------------------------------
// 보정 저장 요청 처리
// ------------------------------------------------------------
void processCalibrationSaveRequest() {
  if (!pending_calibration_save) {
    return;
  }

  uint8_t ratio_bytes[4];

  noInterrupts();
  for (int i = 0; i < 4; i++) {
    ratio_bytes[i] = pending_ratio_bytes[i];
  }
  pending_calibration_save = false;
  interrupts();

  float correction_ratio = 1.0f;
  memcpy(&correction_ratio, ratio_bytes, sizeof(float));

  if (correction_ratio < 0.1f || correction_ratio > 10.0f) {
    return;
  }

  float new_factor = base_calibration_factor / correction_ratio;

  if (new_factor >= CAL_FACTOR_MIN && new_factor <= CAL_FACTOR_MAX) {
    base_calibration_factor = new_factor;

    scale.set_scale(base_calibration_factor);
    saveCalibrationFactor(base_calibration_factor);

    for (int i = 0; i < 2; i++) {
      digitalWrite(STATUS_LED, HIGH);
      delay(80);
      digitalWrite(STATUS_LED, LOW);
      delay(80);
    }
  }
}

void setup() {
  pinMode(STATUS_LED, OUTPUT);

  // 부팅 표시
  for (int i = 0; i < 3; i++) {
    digitalWrite(STATUS_LED, HIGH);
    delay(100);
    digitalWrite(STATUS_LED, LOW);
    delay(100);
  }

  // 저장된 HX711 보정 계수 불러오기
  loadCalibrationFactor();

  // HX711 초기화
  scale.begin(HX_DOUT, HX_SCK);
  scale.set_scale(base_calibration_factor);  
  scale.tare(10); 

  // ADXL345 초기화
  Wire2.begin();

  // 100Hz 출력 설정
  Wire2.beginTransmission(ADXL_ADDR);
  Wire2.write(0x2C);
  Wire2.write(0x0A);
  Wire2.endTransmission();

  // 측정 모드 설정
  Wire2.beginTransmission(ADXL_ADDR);
  Wire2.write(0x2D);
  Wire2.write(0x08);
  Wire2.endTransmission();

  delay(100);

  // STM32 내부에서 가속도 offset 보정
  calibrateAccelOffset();

  // I2C Slave 시작
  Wire.begin(I2C_SLAVE_ADDR);    
  Wire.onRequest(onI2CRequest);
  Wire.onReceive(onI2CReceive);
}

void loop() {
  processCalibrationSaveRequest();

  unsigned long currentTime = millis();

  if (currentTime - lastUpdateTime >= INTERVAL) {
    lastUpdateTime = currentTime;

    // 1. ADXL345 읽기 및 필터
    float gx, gy, gz;

    if (readADXL345(gx, gy, gz)) {
      prev_gx = alpha_sync * gx + (1.0f - alpha_sync) * prev_gx;
      prev_gy = alpha_sync * gy + (1.0f - alpha_sync) * prev_gy;
      prev_gz = alpha_sync * gz + (1.0f - alpha_sync) * prev_gz;
    }

    // 2. HX711 읽기 및 필터
    if (scale.is_ready()) {
      float raw_weight = -scale.get_units(1);  
      prev_weight = alpha_sync * raw_weight + (1.0f - alpha_sync) * prev_weight;
    }

    // 3. STM32 내부에서 최종값 계산
    float weight_g = prev_weight;

    if (fabs(weight_g) < DEADBAND_WEIGHT) {
      weight_g = 0.0f;
    }

    float weight_kg = weight_g / 1000.0f;
    float force_N = weight_kg * G_CONST;

    float accX_g = -((prev_gx) - offset_gx);
    float accY_g = -((prev_gy) - offset_gy);
    float accZ_g = -((prev_gz) - offset_gz);

    if (fabs(accX_g) < DEADBAND_ACCEL) accX_g = 0.0f;
    if (fabs(accY_g) < DEADBAND_ACCEL) accY_g = 0.0f;
    if (fabs(accZ_g) < DEADBAND_ACCEL) accZ_g = 0.0f;

    // 4. I2C 전송 패킷 갱신
    noInterrupts();

    txData.val.weight_g100 = (int32_t)roundf(weight_g * 100.0f);
    txData.val.force_N1000 = (int32_t)roundf(force_N * 1000.0f);
    txData.val.accX_g1000 = (int16_t)roundf(accX_g * 1000.0f);
    txData.val.accY_g1000 = (int16_t)roundf(accY_g * 1000.0f);
    txData.val.accZ_g1000 = (int16_t)roundf(accZ_g * 1000.0f);

    interrupts();
  }
}