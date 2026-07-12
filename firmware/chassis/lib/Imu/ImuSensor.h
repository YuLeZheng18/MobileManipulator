#pragma once
#include <Arduino.h>

// 维特 HWT906P IMU (串口) 封装, 基于 wit_c_sdk
// 输出已转为 ROS sensor_msgs/Imu 单位: 加速度 m/s^2, 角速度 rad/s, 角度 rad
struct ImuData {
  float acc[3] = {0, 0, 0};    // m/s^2
  float gyro[3] = {0, 0, 0};   // rad/s
  float angle[3] = {0, 0, 0};  // rad (roll,pitch,yaw)
  bool valid = false;
};

class ImuSensor {
public:
  // serial: 已 begin 的硬件串口; 内部注册 wit 回调
  void begin(HardwareSerial& serial);

  // 在 sensor_task 中高频调用: 喂入串口字节, 解析帧
  void poll();

  ImuData get() const { return data_; }

private:
  HardwareSerial* serial_ = nullptr;
  ImuData data_;
};
