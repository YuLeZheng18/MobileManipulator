#include "ImuSensor.h"
extern "C" {
#include "wit_c_sdk.h"
#include "REG.h"
}
#include <math.h>

static ImuSensor* s_self = nullptr;
static volatile char s_update = 0;
static const float G = 9.80665f;
static const float D2R = (float)M_PI / 180.0f;

static HardwareSerial* s_ser = nullptr;
static void witSendImpl(uint8_t* p, uint32_t n) { if (s_ser) { s_ser->write(p, n); s_ser->flush(); } }
static void witDelay(uint16_t ms) { delay(ms); }
static void witOnData(uint32_t reg, uint32_t num) { s_update = 1; }

void ImuSensor::begin(HardwareSerial& serial) {
  serial_ = &serial;
  s_self = this;
  s_ser = &serial;
  WitInit(WIT_PROTOCOL_NORMAL, 0x50);
  WitSerialWriteRegister(witSendImpl);
  WitRegisterCallBack(witOnData);
  WitDelayMsRegister(witDelay);
}

void ImuSensor::poll() {
  if (!serial_) return;
  while (serial_->available()) WitSerialDataIn(serial_->read());
  if (s_update) {
    s_update = 0;
    for (int i = 0; i < 3; i++) {
      data_.acc[i] = sReg[AX + i] / 32768.0f * 16.0f * G;
      data_.gyro[i] = sReg[GX + i] / 32768.0f * 2000.0f * D2R;
      data_.angle[i] = sReg[Roll + i] / 32768.0f * 180.0f * D2R;
    }
    data_.valid = true;
    frames_++;
  }
}

void ImuSensor::provision100Hz115200() {
  if (!serial_) return;
  // 先跟当前(9600)数据流同步 ~300ms, 确保 SDK 收发就绪
  uint32_t t0 = millis();
  while (millis() - t0 < 300) { while (serial_->available()) WitSerialDataIn(serial_->read()); }

  // 顺序: 先切波特(立即生效) -> 主机跟切 -> 再设输出率 -> 最后 SAVE 持久化
  // (先切波特再设100Hz, 避免 9600 下 100Hz 输出把总线塞爆的窗口)
  WitSetUartBaud(WIT_BAUD_115200);   // IMU 立即切到 115200
  delay(200);
  serial_->updateBaudRate(115200);   // 主机串口跟着切
  delay(200);
  WitSetOutputRate(RRATE_100HZ);     // 115200 下设 100Hz 输出
  delay(200);
  WitWriteReg(SAVE, SAVE_PARAM);     // 写入 IMU flash, 掉电不丢
  delay(200);
}

int ImuSensor::probeWake() {
  if (!serial_) return -1;
  while (serial_->available()) serial_->read();   // 清缓冲
  frames_ = 0;
  WitSetContent(RSW_ACC | RSW_GYRO | RSW_ANGLE);  // 开启输出内容 (波特匹配才生效)
  delay(100);
  WitSetOutputRate(RRATE_10HZ);                    // 10Hz
  delay(100);
  uint32_t t0 = millis();
  while (millis() - t0 < 600) poll();
  return (int)frames_;
}
