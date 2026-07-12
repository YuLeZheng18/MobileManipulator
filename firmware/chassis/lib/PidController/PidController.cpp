#include "Arduino.h"
#include "PidController.h"

// 构造函数，传入三个PID参数
PidController::PidController(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

float PidController::update(float current)
{
    error_ = target_ - current; //  计算error

    error_sum_ += error_; //  计算error_sum
    // 抗饱和: 把积分项 ki*error_sum 钳在输出量程内 (ki>0 时 i_lim=out_max/ki),
    // 避免积分绕到远超输出上限、饱和后长时间退不回来 -> 持续满速
    float i_lim = (ki_ > 0.0f) ? (out_max_ / ki_) : intergral_up_;
    if (error_sum_ > i_lim)
        error_sum_ = i_lim;
    if (error_sum_ < -i_lim)
        error_sum_ = -i_lim;

    derror_ = error_ - prev_error_; // 计算误差变化率
    prev_error_ = error_;           // 方便下次计算使用

    float output = kp_ * error_ + ki_ * error_sum_ + kd_ * derror_;

    if (output > out_max_)
        output = out_max_;
    if (output < out_min_)
        output = out_min_;

    return output;
}

void PidController::update_target(float target)
{
    target_ = target;
}

void PidController::update_pid(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PidController::reset()
{
    error_sum_ = 0;
    prev_error_ = 0;
    error_ = 0;
    derror_ = 0;
    kp_ = 0;
    ki_ = 0;
    kd_ = 0;
    intergral_up_ = 2500;
    out_min_ = 0;
    out_max_ = 0;
}
void PidController::out_limit(float min, float max)
{
    out_min_ = min;
    out_max_ = max;
}
void PidController::clearIntegral()
{
    error_sum_ = 0;
    prev_error_ = 0;
    error_ = 0;
    derror_ = 0;
}
