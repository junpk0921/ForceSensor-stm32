#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "HX711.h"

// [ 하드웨어 설정 ]
TwoWire Wire2(PB11, PB10); 
#define HX_DOUT   PA1     
#define HX_SCK    PA0     
#define CALIB_BUTTON D0  
#define STATUS_LED   PA8     
#define I2C_SLAVE_ADDR 0x08
#define ADXL_ADDR      0x53

// [ 데이터 프로토콜 ] - 13Byte로 확장
typedef union {
  struct __attribute__((packed)) {
    int32_t weight_g100;    
    int16_t angle_deg100;   
    int16_t accX_g1000;     
    int16_t accY_g1000;     
    int16_t accZ_g1000;     
    uint8_t btn_pressed;    // [추가] 버튼 상태 (0: 평상시, 1: 눌림)
  } val;
  uint8_t buffer[13];      
} Packet_t;

HX711 scale;
volatile Packet_t txData; 
const float alpha_sync = 0.3f; 
float base_calibration_factor = 350.2f;
float prev_weight = 0.0f;           
float prev_gx = 0.0f, prev_gy = 0.0f, prev_gz = 0.0f;
unsigned long lastUpdateTime = 0;
const unsigned long INTERVAL = 10; 

// 버튼 디바운싱
bool lastBtnState = HIGH;
unsigned long lastDebounceTime = 0;

void onI2CRequest() {
  Wire.write((const uint8_t*)txData.buffer, 13); // 13바이트 전송
  // 전송 후 버튼 플래그 초기화 (Master가 확인했음을 가정)
  txData.val.btn_pressed = 0; 
}

void setup() {
  pinMode(STATUS_LED, OUTPUT);
  pinMode(CALIB_BUTTON, INPUT_PULLUP);
  
  for (int i = 0; i < 3; i++) {
    digitalWrite(STATUS_LED, HIGH); delay(100);
    digitalWrite(STATUS_LED, LOW); delay(100);
  }

  scale.begin(HX_DOUT, HX_SCK);
  scale.set_scale(base_calibration_factor);  
  scale.tare(10); 

  Wire2.begin();
  Wire2.beginTransmission(ADXL_ADDR);
  Wire2.write(0x2C); Wire2.write(0x0A);
  Wire2.endTransmission();
  Wire2.beginTransmission(ADXL_ADDR);
  Wire2.write(0x2D); Wire2.write(0x08);
  Wire2.endTransmission();

  Wire.begin(I2C_SLAVE_ADDR);    
  Wire.onRequest(onI2CRequest);  
}

void loop() {
  unsigned long currentTime = millis();

  // --- [Step 1] 버튼 체크 및 플래그 설정 ---
  bool currentBtnState = digitalRead(CALIB_BUTTON);
  if (currentBtnState == LOW && lastBtnState == HIGH) {
    if (currentTime - lastDebounceTime > 150) {
      digitalWrite(STATUS_LED, HIGH); 
      txData.val.btn_pressed = 1; // Master에게 보정 요청 신호 보냄
      lastDebounceTime = currentTime;
    }
  }
  else if (currentBtnState == HIGH) {
    digitalWrite(STATUS_LED, LOW);
  }
  lastBtnState = currentBtnState;

  // --- [Step 2] 센서 업데이트 (100Hz) ---
  if (currentTime - lastUpdateTime >= INTERVAL) {
    lastUpdateTime = currentTime;

    // IMU 읽기
    Wire2.beginTransmission(ADXL_ADDR);
    Wire2.write(0x32);
    if (Wire2.endTransmission(false) == 0) {
      Wire2.requestFrom(ADXL_ADDR, 6);  
      if (Wire2.available() == 6) {
        int16_t ax_raw = (Wire2.read() | (Wire2.read() << 8));
        int16_t ay_raw = (Wire2.read() | (Wire2.read() << 8));
        int16_t az_raw = (Wire2.read() | (Wire2.read() << 8));
        prev_gx = alpha_sync * (ax_raw / 256.0f) + (1.0f - alpha_sync) * prev_gx;
        prev_gy = alpha_sync * (ay_raw / 256.0f) + (1.0f - alpha_sync) * prev_gy;
        prev_gz = alpha_sync * (az_raw / 256.0f) + (1.0f - alpha_sync) * prev_gz;
      }
    } 

    // 로드셀 읽기
    if (scale.is_ready()) {
      float raw_weight = -scale.get_units(1);  
      prev_weight = alpha_sync * raw_weight + (1.0f - alpha_sync) * prev_weight;
    }

    float theta_deg = atan2(prev_gy, prev_gz) * 180.0f / PI;

    noInterrupts();
    txData.val.weight_g100 = (int32_t)roundf(prev_weight * 100.0f);
    txData.val.angle_deg100 = (int16_t)(theta_deg * 100.0f); 
    txData.val.accX_g1000 = (int16_t)(prev_gx * 1000.0f);
    txData.val.accY_g1000 = (int16_t)(prev_gy * 1000.0f);
    txData.val.accZ_g1000 = (int16_t)(prev_gz * 1000.0f);
    interrupts();
  }
}