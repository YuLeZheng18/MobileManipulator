#pragma once
#include <stdint.h>

// ESP32-S3 PCNT 硬件正交编码器 (4 倍频)
// 每个实例占用一个 PCNT unit (S3 共 4 个, 正好四轮)
// 计数溢出由 PCNT 硬件限值事件累加, 读取返回 int64 累积值
class Encoder {
public:
  // unit: PCNT 单元号 0-3; pinA/pinB: 正交 A/B 相引脚
  void begin(int unit, uint8_t pinA, uint8_t pinB);

  // 累积计数 (含溢出累加)
  long long getCount();

  // 读增量并把基准移到当前 (供控制周期算转速)
  long long readDelta();

private:
  int unit_ = 0;
  long long base_ = 0;
};
