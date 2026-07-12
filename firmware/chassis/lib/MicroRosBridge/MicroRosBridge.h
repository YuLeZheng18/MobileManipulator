#pragma once
#include <stdint.h>

// micro-ROS 通信桥 (运行于 microros_task, Core 0)
// 订阅 /cmd_vel /pump_cmd; 发布 /odom /imu /battery
// 共享数据用 portMUX 临界区保护, 供 control_task / sensor_task 读写

struct CmdVel { float vx = 0, vy = 0, wz = 0; };

namespace MicroRos {
  // 在 microros_task 内调用: 配置串口传输 + 初始化节点/实体, 然后进入 spin (阻塞)
  void task(void* arg);

  // --- 线程安全共享数据接口 ---
  CmdVel getCmd();                                  // control_task 读目标速度
  int8_t getPump();                                 // control_task 读气泵命令 (PUMP_STOP/SUCK/RELEASE)
  void setOdom(float x, float y, float yaw,
               float vx, float vy, float wz);       // control_task 写里程计
  void setImu(const float acc[3], const float gyro[3],
              const float angle[3]);                // sensor_task 写 IMU
  void setBattery(float voltage, float current);    // sensor_task 写电源

  bool isConnected();       // agent 是否已连接
  uint32_t cmdAgeMs();      // 距上次收到 /cmd_vel 的毫秒数 (失效保护看门狗用)
}
