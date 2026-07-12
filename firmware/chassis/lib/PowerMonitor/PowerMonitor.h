#pragma once
#include <stdint.h>

// INA226 电源监测 (I2C): 母线电压 / 电流 / 功率
// 封装 demo 中已验证的 Arduino-INA226 库
struct PowerData {
  float voltage = 0;  // V
  float current = 0;  // A
  float power = 0;    // W
};

class PowerMonitor {
public:
  // sda/scl: I2C 引脚; addr: 器件地址(默认0x40)
  // rShunt: 采样电阻(欧); maxCurrent: 预期最大电流(A)
  bool begin(uint8_t sda, uint8_t scl, uint8_t addr,
             float rShunt = 0.01f, float maxCurrent = 4.0f);

  PowerData read();

private:
  uint8_t addr_ = 0x40;
};
