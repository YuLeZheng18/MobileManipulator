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

  // 累计成功解析帧数 (用于 IMU_PROVISION 模式估算输出率/判活)
  uint32_t frames() const { return frames_; }

  // 一次性出厂配置: 假定当前 serial_ 为 IMU 出厂默认 9600 波特,
  // 依次将 IMU 设为 115200 波特 + 100Hz 输出并 SAVE(掉电不丢),
  // 内部会把 serial_ 同步切到 115200。仅供 IMU_PROVISION 模式调用一次。
  void provision100Hz115200();

  // 主动唤醒探测: 假定 serial_ 已按某波特打开, 发送"开启acc/gyro/angle输出+10Hz"命令,
  // 再监听 ~600ms, 返回期间解析到的有效帧数(>0 即该波特匹配且 IMU 已被唤醒)。
  int probeWake();

private:
  HardwareSerial* serial_ = nullptr;
  ImuData data_;
  uint32_t frames_ = 0;
};
