#include "Kinematics.h"
#include <math.h>

void Kinematics::setParams(float wheelRadius, float robotRadius, float theta, float pulsePerRev) {
  r_ = wheelRadius;
  R_ = robotRadius;
  sT_ = sinf(theta);
  cT_ = cosf(theta);
  ppr_ = pulsePerRev;
}

// 角落四全向轮逆解
// 0=左前 1=左后 2=右后 3=右前
void Kinematics::inverse(float vx, float vy, float wz, float out[4]) {
  float t = wz * R_;
  out[0] = -sT_*vx + cT_*vy + t;
  out[1] = -sT_*vx - cT_*vy + t;
  out[2] =  sT_*vx - cT_*vy + t;
  out[3] =  sT_*vx + cT_*vy + t;
}

void Kinematics::updateOdom(const float w[4], float dt) {
  float vx = (w[2] + w[3] - w[0] - w[1]) / (4.0f * sT_);
  float vy = (w[0] - w[1] + w[3] - w[2]) / (4.0f * cT_);
  float wz = (w[0] + w[1] + w[2] + w[3]) / (4.0f * R_);
  odom_.vx = vx; odom_.vy = vy; odom_.wz = wz;
  float c = cosf(odom_.yaw), s = sinf(odom_.yaw);
  odom_.x += (vx * c - vy * s) * dt;
  odom_.y += (vx * s + vy * c) * dt;
  odom_.yaw += wz * dt;
}

float Kinematics::pulsesToSpeed(long deltaPulses, float dt) const {
  if (ppr_ <= 0 || dt <= 0) return 0;
  float rev = (float)deltaPulses / ppr_;
  float dist = rev * 2.0f * (float)M_PI * r_;
  return dist / dt;
}
