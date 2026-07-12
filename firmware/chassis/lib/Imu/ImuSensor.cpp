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
  }
}
