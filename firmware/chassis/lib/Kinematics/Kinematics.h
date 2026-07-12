#pragma once
#include <stdint.h>

// 四轮角落全向轮运动学 + 里程计
// 坐标系: +X 车头, +Y 车左, 右手系
// 轮序约定: 0=左前(+X+Y) 1=左后(-X+Y) 2=右后(-X-Y) 3=右前(+X-Y)
// theta: X轴到左前轮方向向量的夹角 = arctan(半车宽/半车长), 非正方形底盘 != PI/4

struct Odom {
  float x = 0, y = 0, yaw = 0;   // 累积位姿 (m, m, rad)
  float vx = 0, vy = 0, wz = 0;  // 当前机体速度 (m/s, m/s, rad/s)
};

class Kinematics {
public:
  // wheelRadius: 轮半径(m); robotRadius: 轮组中心到底盘中心距离(m)
  // theta: 轮安装角(rad), 待实车测量后填入; pulsePerRev: 编码器每转脉冲(4倍频后含减速比)
  void setParams(float wheelRadius, float robotRadius, float theta, float pulsePerRev);

  // 逆解: 机体速度(vx,vy,wz) -> 四轮目标线速度 out[4] (m/s)
  void inverse(float vx, float vy, float wz, float out[4]);

  // 正解 + 里程计积分: wheelSpeed[4] 为四轮当前线速度(m/s), dt 为时间间隔(s)
  void updateOdom(const float wheelSpeed[4], float dt);

  // 工具: 编码器增量脉冲 -> 该轮线速度(m/s)
  float pulsesToSpeed(long deltaPulses, float dt) const;

  Odom getOdom() const { return odom_; }
  void resetOdom() { odom_ = Odom{}; }

private:
  float r_ = 0;
  float R_ = 0;
  float sT_ = 0.707f;  // sin(theta)
  float cT_ = 0.707f;  // cos(theta)
  float ppr_ = 0;
  Odom odom_;
};
