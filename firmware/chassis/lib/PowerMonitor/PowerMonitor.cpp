#include "PowerMonitor.h"
#include "config.h"
#include <Wire.h>
#include <INA226.h>

static INA226 s_ina;

bool PowerMonitor::begin(uint8_t sda, uint8_t scl, uint8_t addr,
                         float rShunt, float maxCurrent) {
  addr_ = addr;
  Wire.begin(sda, scl);
  Wire.setClock(400000);
  s_ina.begin(addr_);  // 实测芯片在 0x50, 必须传地址
  s_ina.configure(INA226_AVERAGES_1, INA226_BUS_CONV_TIME_1100US,
                  INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);
  s_ina.calibrate(rShunt, maxCurrent);
  return true;
}

PowerData PowerMonitor::read() {
  PowerData d;
  d.voltage = s_ina.readBusVoltage() * INA226_VBUS_GAIN;
  d.current = s_ina.readShuntCurrent();
  d.power = s_ina.readBusPower();
  return d;
}
