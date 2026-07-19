#pragma once
// ESP32-S3 (N16R8) 底盘引脚与参数定义 — 引脚已定稿, 见硬件接线表

// ===== 电机 DIR 方向脚 (输出) =====
#define M1_DIR 38
#define M2_DIR 39
#define M3_DIR 40
#define M4_DIR 41

// ===== 电机 PWM 调速脚 (LEDC, 注意 BM50 为低电平有效, 占空比需反相) =====
#define M1_PWM 2
#define M2_PWM 47
#define M3_PWM 4
#define M4_PWM 5

// ===== 公共刹车 / 急停 (4 电机共线, 低电平刹车 / 高电平正常转) =====
#define MOTOR_BRAKE 42

// ===== 编码器 (PCNT 硬件计数, A/B 正交) =====
#define M1_ENC_A 8
#define M1_ENC_B 9
#define M2_ENC_A 10
#define M2_ENC_B 11
#define M3_ENC_A 12
#define M3_ENC_B 13
#define M4_ENC_A 14
#define M4_ENC_B 15

// ===== I2C (INA226 电源监测) =====
#define I2C_SDA 16
#define I2C_SCL 17
#define INA226_ADDR 0x50      // 实测: TI 兼容片(mfr=0x5449, die=0x2270)固定在 0x50
#define INA226_R_SHUNT   0.1f    // 模块采样电阻 (Ω); 注意 0.1R 电流满量程仅 ±0.82A
#define INA226_MAX_CURR  0.8192f // 预期最大电流 (A) = 81.92mV / 0.1Ω, 用于电流标定
#define INA226_VBUS_GAIN 1.0275f // 母线电压单点增益修正 (万用表24.16 / 芯片23.514)

// ===== IMU HWT906P (硬件串口 UART1; ESP32 TX=18->IMU RX, ESP32 RX=21->IMU TX) =====
#define IMU_TX 18
#define IMU_RX 21
#define IMU_BAUD 115200   // 已用 IMU_PROVISION 一次性配为 115200+100Hz 并 SAVE (2026-07-19)

// ===== 其他 PWM 输出 (LEDC) =====
#define PUMP1_PWM 7   // 气泵    (物理接 GPIO7)
#define PUMP2_PWM 6   // 电磁阀  (物理接 GPIO6)
#define LIDAR_PWM 1   // 雷达电机 MOTOCTL (RPLIDAR A3M1)

// ===== 雷达电机 PWM (RPLIDAR A3M1 MOTOCTL 要求 25kHz 方波) =====
// ch6 → timer3, 与电机 20kHz(timer0/1) 和舵机 50Hz(timer2) 互不干扰
#define LIDAR_CH         6      // LEDC 通道
#define LIDAR_PWM_FREQ   25000  // 25kHz (规格: 24500-25500Hz)
#define LIDAR_DUTY_DEF   153    // ~60% 占空比 (8位255*0.6), 典型10Hz转速

// ===== 板载 RGB LED (状态指示, 可选) =====
#define RGB_LED 48

// ===== LEDC PWM 配置 =====
#define PWM_FREQ_HZ      20000  // 20kHz, 超出可听范围
#define PWM_RESOLUTION   8      // 8 位, 占空比 0-255 (匹配 BM50 例程 254 上限)

// ===== 气泵/电磁阀 舵机信号 (标准 50Hz 舵机 PWM; write(0)=关 / write(180)=开) =====
// 用独立 LEDC 通道 4/5, 避开电机的 0-3; 50Hz 走 timer2, 与电机 20kHz(timer0/1)不冲突
#define PUMP1_CH         4      // 气泵 LEDC 通道 (PUMP1_PWM=GPIO7)
#define PUMP2_CH         5      // 电磁阀 LEDC 通道 (PUMP2_PWM=GPIO6)
#define SERVO_FREQ_HZ    50     // 舵机标准 50Hz (20ms 周期)
#define SERVO_RESOLUTION 14     // ESP32-S3 LEDC 占空比分辨率硬件上限 14 位 (16 会 ledcSetup 失败)
#define SERVO_MIN_US     544    // write(0)   脉宽 (Arduino Servo 默认下限)
#define SERVO_MAX_US     2400   // write(180) 脉宽 (Arduino Servo 默认上限)

// ===== 气泵命令 (/pump_cmd, std_msgs/Int8) =====
#define PUMP_STOP        0      // 停止保压: 气泵关 + 阀关
#define PUMP_SUCK        1      // 吸气:    气泵开(180) + 阀关(0), 封闭抽真空
#define PUMP_RELEASE     2      // 释放:    气泵关(0)  + 阀开(180), 进气卸物

// ===== micro-ROS =====
#define MICROROS_DOMAIN_ID 42   // 与整机 ROS2 环境(.bashrc ROS_DOMAIN_ID=42)一致, 免去命令前缀

// ===== 控制频率 =====
#define CONTROL_RATE_HZ  100    // control_task 固定周期
// 失效保护: 超过此毫秒数没收到 /cmd_vel (或 micro-ROS 掉线) -> 目标归零+清积分, 底盘停
#define CMD_TIMEOUT_MS   500
#define ODOM_PUB_MS      50     // /odom 发布周期
#define SENSOR_RATE_HZ   50     // sensor_task IMU/INA226 采集

// ===== 四轮角落 omni 运动学参数 (CAD 实测, 四轮对称) =====
#define WHEEL_RADIUS_M   0.0391f   // 有效滚动半径 (直行标定 2026-07-12: 实测1.43m / odom报1.5012m, 名义41mm高报~4.7%)
#define ROBOT_RADIUS_M   0.1969f   // 有效转动半径 (角度标定 2026-07-12: 转2圈 odom733.69° / 实际772.4°(弦18.3cm反推), CAD207.3mm高报~5%)
#define WHEEL_THETA_RAD  0.785398f // 轮安装角 = PI/4 (CAD 实测 44.97~45.11°, 对称)
#define ENCODER_PPR      380.0f    // 每轮圈脉冲: 5对极×2边沿×2相(4倍频)=20/电机圈 ×19减速比

// ===== 轮速闭环 PID (空载整定基线; 落地带载需在地面复调, 见 Task #7) =====
// 误差单位 m/s, 输出 duty(±255); 定频位置式, 100Hz. KD=0 (微分放大量化纹波)
#define PID_KP   350
#define PID_KI   10
#define PID_KD   0.0
#define PID_OUT_LIMIT 255
// 测速 EMA 低通系数: 100Hz+380PPR 下低速每tick仅~3脉冲, 不滤波会量化抖/换向抖
#define SPEED_FILTER_ALPHA 0.3f

// ===== 速度指令加速度斜坡 (逆解前限制 vx/vy/wz 变化率) =====
// 防 /cmd_vel 阶跃猛冲: 轮子瞬间冲高速>抓地力->四轮打滑不一致->车身扭歪(高速尤甚)
// 100Hz 下每 tick 最多变 accel/100; 打滑仍歪就调小, 太肉就调大
#define MAX_LIN_ACCEL 0.6f   // 最大线加速度 (m/s^2): 0->0.5m/s 约 0.83s
#define MAX_ANG_ACCEL 3.0f   // 最大角加速度 (rad/s^2)
