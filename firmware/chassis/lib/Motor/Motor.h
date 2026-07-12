#pragma once
#include <Arduino.h>

// BM50 驱控一体无刷电机单电机驱动
// 接口: DIR(方向) + PWM(LEDC 调速, 低电平有效需反相) + 共享 BRAKE(由上层统一管)
// 编码器读取走 PCNT, 不在本类内 (见 Encoder 模块)
class Motor {
public:
  // dirPin: 方向脚; pwmPin: PWM 脚; ledcChannel: LEDC 通道(0-7)
  void begin(uint8_t dirPin, uint8_t pwmPin, uint8_t ledcChannel);

  // 设置输出: duty 范围 -255..+255, 符号决定方向, 0 停转
  // 内部处理 DIR 电平 + BM50 低电平有效的占空比反相
  void setPwm(int duty);

private:
  uint8_t dirPin_ = 0;
  uint8_t pwmPin_ = 0;
  uint8_t ch_ = 0;
};

// 公共刹车/急停 (4 电机共线, GPIO42; 低电平刹车, 高电平正常转)
namespace MotorBrake {
  void begin(uint8_t brakePin);
  void engage();   // 刹车 (拉低)
  void release();  // 松刹车, 允许转动 (拉高)
}
