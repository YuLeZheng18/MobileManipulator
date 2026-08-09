#ifndef __PID_CONTROLLER_H__
#define __PID_CONTROLLER_H__

class PidController
{
public:
    PidController() = default;
    PidController(float kp, float ki, float kd);

private:
    // PID 参数，可以调节的
    float target_;
    float out_min_;
    float out_max_;
    float kp_;
    float ki_;
    float kd_;
    float intergral_up_ = 2500; // 积分上限
    float dt_ = 0.01f;          // 控制周期 (s); 积分项乘它, 使 ki 与采样率解耦
    float integral_band_ = 1e9f;// 积分分离阈值 (m/s); 默认极大=不分离(退化为always积分)
    // pid 中间过程值
    float error_;
    float error_sum_;
    float derror_;
    float prev_error_;

public:
    float update(float current);                   // 提供当前值，返回下次输出值，也就是PID的结果
    void update_target(float target);              // 更新目标值
    void update_pid(float kp, float ki, float kd); // 更新PID参数
    void reset();                                  // 重置PID
    void out_limit(float min, float max);          // 设置输出限制
    void clearIntegral();                          // 只清积分/微分历史 (失效保护用, 不动增益)
    void set_dt(float dt);                         // 设置控制周期 (积分/微分用)
    void set_integral_band(float band);            // 设置积分分离阈值 (误差超此值不积分)
    float integral() const { return error_sum_; }  // 读积分状态 (诊断用: 判断是否饱和)
};

#endif // __PID_CONTROLLER_H__