#include "Motor.h"
#include "config.h"

void Motor::begin(uint8_t dirPin, uint8_t pwmPin, uint8_t ledcChannel) {
  dirPin_ = dirPin;
  pwmPin_ = pwmPin;
  ch_ = ledcChannel;
  pinMode(dirPin_, OUTPUT);
  ledcSetup(ch_, PWM_FREQ_HZ, PWM_RESOLUTION);  // arduino-esp32 2.x API
  ledcAttachPin(pwmPin_, ch_);
  setPwm(0);
}

void Motor::setPwm(int duty) {
  duty = constrain(duty, -PID_OUT_LIMIT, PID_OUT_LIMIT);
  digitalWrite(dirPin_, duty >= 0 ? HIGH : LOW);
  int mag = abs(duty);
  // BM50 PWM 低电平有效: 占空比反相
  int maxDuty = (1 << PWM_RESOLUTION) - 1;
  ledcWrite(ch_, maxDuty - mag);  // 2.x: 写通道号
}

namespace MotorBrake {
  static uint8_t s_brakePin = 0;
  void begin(uint8_t brakePin) {
    s_brakePin = brakePin;
    pinMode(s_brakePin, OUTPUT);
    engage();  // 上电默认刹车, 安全
  }
  void engage()  { digitalWrite(s_brakePin, LOW); }
  void release() { digitalWrite(s_brakePin, HIGH); }
}
