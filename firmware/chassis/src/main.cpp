#include <Arduino.h>
#include "config.h"
#include "Motor.h"
#include "Encoder.h"
#include "Kinematics.h"
#include "PidController.h"
#include "ImuSensor.h"
#include "PowerMonitor.h"
#include "MicroRosBridge.h"

// 调试串口走 UART0 (GPIO43/44); Serial 被 micro-ROS 占用(原生USB)
#define DBG Serial0

static Motor motors[4];
static Encoder encoders[4];
static PidController pid[4];
static Kinematics kin;
static ImuSensor imu;
static PowerMonitor power;

static const uint8_t DIR_PINS[4] = {M1_DIR, M2_DIR, M3_DIR, M4_DIR};
static const uint8_t PWM_PINS[4] = {M1_PWM, M2_PWM, M3_PWM, M4_PWM};
static const uint8_t ENCA[4] = {M1_ENC_A, M2_ENC_A, M3_ENC_A, M4_ENC_A};
static const uint8_t ENCB[4] = {M1_ENC_B, M2_ENC_B, M3_ENC_B, M4_ENC_B};

static void control_task(void* arg);
static void sensor_task(void* arg);

// 气泵/电磁阀: 标准 50Hz 舵机信号, write(0)=关 / write(180)=开
static void servo_begin(uint8_t pin, uint8_t ch) {
  ledcSetup(ch, SERVO_FREQ_HZ, SERVO_RESOLUTION);  // arduino-esp32 2.x API
  ledcAttachPin(pin, ch);
}
static void servo_write_angle(uint8_t ch, int angle) {
  angle = constrain(angle, 0, 180);
  int us = map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  uint32_t maxDuty = (1u << SERVO_RESOLUTION) - 1;
  uint32_t duty = (uint32_t)((uint64_t)us * maxDuty / 20000);  // 20ms 周期
  ledcWrite(ch, duty);
}
// 按气泵命令切换气泵(ch4)与电磁阀(ch5)
static void apply_pump(int8_t cmd) {
  switch (cmd) {
    case PUMP_SUCK:    servo_write_angle(PUMP2_CH, 0);   servo_write_angle(PUMP1_CH, 180); break;  // 关阀 + 开泵
    case PUMP_RELEASE: servo_write_angle(PUMP1_CH, 0);   servo_write_angle(PUMP2_CH, 180); break;  // 关泵 + 开阀
    default:           servo_write_angle(PUMP1_CH, 0);   servo_write_angle(PUMP2_CH, 0);   break;  // 停止保压
  }
}

#ifdef ENCODER_TEST
// 编译期编码器验证模式: 只松刹车让轮子能手转, 串口打印四轮累计计数
// 不启动电机闭环/micro-ROS/传感器, 手转 10 圈应 ≈ 3800 计数 (4 倍频)
void setup() {
  DBG.begin(115200);
  DBG.println("\n[chassis] ENCODER_TEST mode");
  for (int i = 0; i < 4; i++) motors[i].begin(DIR_PINS[i], PWM_PINS[i], i);  // setPwm(0)=BM50 停
  MotorBrake::begin(MOTOR_BRAKE);
  for (int i = 0; i < 4; i++) encoders[i].begin(i, ENCA[i], ENCB[i]);
  MotorBrake::release();  // 松刹车, 轮子可手动回转
}

void loop() {
  long long c[4];
  for (int i = 0; i < 4; i++) c[i] = encoders[i].getCount();
  DBG.printf("M1=%lld  M2=%lld  M3=%lld  M4=%lld\n", c[0], c[1], c[2], c[3]);
  delay(200);
}
#elif defined(MOTOR_TEST)
// 电机转向验证: 逐个电机给小正占空比, 打印该轮编码器增量
// 增量为正 = 电机正向与编码器/运动学一致 (PID 可直接用); 为负 = 该电机 DIR 需翻
void setup() {
  DBG.begin(115200);
  DBG.println("\n[chassis] MOTOR_TEST mode (轮子架空!)");
  for (int i = 0; i < 4; i++) motors[i].begin(DIR_PINS[i], PWM_PINS[i], i);
  MotorBrake::begin(MOTOR_BRAKE);
  for (int i = 0; i < 4; i++) encoders[i].begin(i, ENCA[i], ENCB[i]);
  MotorBrake::release();  // 松刹车才能转
}

void loop() {
  const int duty = 80;              // 小正占空比 (0-255)
  static bool done = false;
  if (done) { delay(500); return; }
  for (int i = 0; i < 4; i++) {
    encoders[i].readDelta();        // 清零基准
    motors[i].setPwm(duty);
    delay(500);
    motors[i].setPwm(0);
    long long d = encoders[i].readDelta();
    DBG.printf("M%d: duty=+%d -> delta=%lld  (%s)\n",
               i + 1, duty, d, d > 0 ? "OK 同向" : "反向需翻");
    delay(1000);                    // 间隔便于观察
  }
  DBG.println("---- sweep done ----");
  done = true;
}
#elif defined(PID_TEST)
// 闭环 PID 在线整定: 串口发 'k kp ki kd' 改增益 / 't target' 阶跃(m/s) / 's' 停
// 打印 CSV: t_ms,tgt,m1,m2,m3,m4,duty1  (四轮同参同目标, BM50 一致取一套参数)
static float g_kp = PID_KP, g_ki = PID_KI, g_kd = PID_KD;
static float g_target = 0.0f;
static float g_alpha = 0.3f;            // 测速 EMA 低通系数 (越小越平滑/越滞后)
static float g_filt[4] = {0, 0, 0, 0};  // 滤波后轮速
static int   g_log = 0;                 // 剩余打印 tick 数
static char  g_buf[64]; static int g_len = 0;

static void applyGains() {
  for (int i = 0; i < 4; i++) {
    pid[i].reset();                      // 注意: reset 会清零增益+限幅+积分
    pid[i].update_pid(g_kp, g_ki, g_kd);
    pid[i].out_limit(-PID_OUT_LIMIT, PID_OUT_LIMIT);
  }
}

static void handleCmd(char* s) {
  float a, b, c;
  if (s[0] == 'k' && sscanf(s + 1, "%f %f %f", &a, &b, &c) == 3) {
    g_kp = a; g_ki = b; g_kd = c; applyGains();
    DBG.printf("# gains kp=%.4f ki=%.4f kd=%.4f\n", g_kp, g_ki, g_kd);
  } else if (s[0] == 'f' && sscanf(s + 1, "%f", &a) == 1) {
    g_alpha = a;
    DBG.printf("# alpha=%.3f\n", g_alpha);
  } else if (s[0] == 't' && sscanf(s + 1, "%f", &a) == 1) {
    g_target = a; applyGains();          // 清积分, 做干净阶跃
    for (int i = 0; i < 4; i++) g_filt[i] = 0;
    g_log = 200;                         // 打印约 2s
    DBG.printf("# step target=%.3f alpha=%.3f  cols: t_ms,tgt,f1,f2,f3,f4,duty1\n", g_target, g_alpha);
  } else if (s[0] == 's') {
    g_target = 0; applyGains(); g_log = 0;
    for (int i = 0; i < 4; i++) { g_filt[i] = 0; motors[i].setPwm(0); }
    DBG.println("# stop");
  } else {
    DBG.println("# ? cmds: 'k kp ki kd' | 'f alpha' | 't target' | 's'");
  }
}

void setup() {
  DBG.begin(115200);
  for (int i = 0; i < 4; i++) motors[i].begin(DIR_PINS[i], PWM_PINS[i], i);
  MotorBrake::begin(MOTOR_BRAKE);
  for (int i = 0; i < 4; i++) encoders[i].begin(i, ENCA[i], ENCB[i]);
  kin.setParams(WHEEL_RADIUS_M, ROBOT_RADIUS_M, WHEEL_THETA_RAD, ENCODER_PPR);
  applyGains();
  MotorBrake::release();
  DBG.println("\n[chassis] PID_TEST  cmds: 'k kp ki kd' | 't target' | 's'");
}

void loop() {
  const float dt = 0.01f;                // 100Hz
  while (DBG.available()) {
    char ch = DBG.read();
    if (ch == '\n' || ch == '\r') { if (g_len) { g_buf[g_len] = 0; handleCmd(g_buf); g_len = 0; } }
    else if (g_len < (int)sizeof(g_buf) - 1) g_buf[g_len++] = ch;
  }
  static uint32_t last = 0; uint32_t now = millis();
  if (now - last >= 10) {
    last = now;
    int duty0 = 0;
    for (int i = 0; i < 4; i++) {
      long long d = encoders[i].readDelta();
      float m = kin.pulsesToSpeed((long)d, dt);
      g_filt[i] = g_alpha * m + (1.0f - g_alpha) * g_filt[i];   // EMA 低通
      pid[i].update_target(g_target);
      int duty = (int)pid[i].update(g_filt[i]);
      motors[i].setPwm(duty);
      if (i == 0) duty0 = duty;
    }
    if (g_log > 0) {
      DBG.printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%d\n",
                 now, g_target, g_filt[0], g_filt[1], g_filt[2], g_filt[3], duty0);
      g_log--;
    }
  }
}
#else

void setup() {
  DBG.begin(115200);
  DBG.println("\n[chassis] booting...");

  // 电机 + 公共刹车 (LEDC 通道 0-3 给四轮)
  for (int i = 0; i < 4; i++) motors[i].begin(DIR_PINS[i], PWM_PINS[i], i);
  MotorBrake::begin(MOTOR_BRAKE);

  // 编码器 PCNT unit 0-3
  for (int i = 0; i < 4; i++) encoders[i].begin(i, ENCA[i], ENCB[i]);

  // PID
  for (int i = 0; i < 4; i++) {
    pid[i].update_pid(PID_KP, PID_KI, PID_KD);
    pid[i].out_limit(-PID_OUT_LIMIT, PID_OUT_LIMIT);
  }

  // 运动学参数 (TODO: 填实测值后生效)
  kin.setParams(WHEEL_RADIUS_M, ROBOT_RADIUS_M, WHEEL_THETA_RAD, ENCODER_PPR);

  // 雷达电机 MOTOCTL (RPLIDAR A3M1, 25kHz, 通道6/timer3, 上电直接启转)
  ledcSetup(LIDAR_CH, LIDAR_PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(LIDAR_PWM, LIDAR_CH);
  ledcWrite(LIDAR_CH, LIDAR_DUTY_DEF);  // ~60% 对应约10Hz, 按实车调整

  // 气泵 + 电磁阀 舵机通道 (上电默认全关)
  servo_begin(PUMP1_PWM, PUMP1_CH);
  servo_begin(PUMP2_PWM, PUMP2_CH);
  apply_pump(PUMP_STOP);

  // IMU (UART1) + INA226
  Serial1.begin(9600, SERIAL_8N1, IMU_RX, IMU_TX);
  imu.begin(Serial1);
  power.begin(I2C_SDA, I2C_SCL, INA226_ADDR, INA226_R_SHUNT, INA226_MAX_CURR);

  // 三任务分核
  xTaskCreatePinnedToCore(MicroRos::task, "microros", 16384, NULL, 5, NULL, 0);
  xTaskCreatePinnedToCore(control_task, "control", 8192, NULL, 6, NULL, 1);
  xTaskCreatePinnedToCore(sensor_task,  "sensor",  8192, NULL, 4, NULL, 0);

  MotorBrake::release();  // 松刹车, 允许运动
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }  // 空闲, 实际工作在任务里

// Core 1: 固定 100Hz 控制闭环
static void control_task(void* arg) {
  const TickType_t period = pdMS_TO_TICKS(1000 / CONTROL_RATE_HZ);
  TickType_t last = xTaskGetTickCount();
  const float dt = 1.0f / CONTROL_RATE_HZ;
  float target[4];
  int8_t lastPump = PUMP_STOP;
  float rvx = 0, rvy = 0, rwz = 0;                 // 斜坡后的当前速度指令 (跨周期保持)
  const float dLin = MAX_LIN_ACCEL / CONTROL_RATE_HZ;  // 每 tick 最大线速度变化
  const float dAng = MAX_ANG_ACCEL / CONTROL_RATE_HZ;  // 每 tick 最大角速度变化
  while (true) {
    // 失效保护: 掉线或 cmd_vel 超时(含上电首个命令前) -> 强制停车, 防止带旧目标/旧积分冲出
    bool failsafe = !MicroRos::isConnected() || MicroRos::cmdAgeMs() > CMD_TIMEOUT_MS;
    CmdVel c = MicroRos::getCmd();
    if (failsafe) { c.vx = c.vy = c.wz = 0.0f; rvx = rvy = rwz = 0.0f; }  // 清斜坡状态, 重连从0起

    // 加速度斜坡: 逆解前平滑逼近目标, 防阶跃猛冲导致轮子打滑扭歪车身
    rvx += constrain(c.vx - rvx, -dLin, dLin);
    rvy += constrain(c.vy - rvy, -dLin, dLin);
    rwz += constrain(c.wz - rwz, -dAng, dAng);
    kin.inverse(rvx, rvy, rwz, target);  // 目标轮速 (m/s)

    // 气泵: 命令变化才切舵机信号 (LEDC 会持续输出保持当前状态)
    int8_t pump = MicroRos::getPump();
    if (pump != lastPump) { apply_pump(pump); lastPump = pump; }

    static float wheelSpeed[4] = {0, 0, 0, 0};  // EMA 滤波后轮速 (跨周期保持)
    for (int i = 0; i < 4; i++) {
      long long d = encoders[i].readDelta();
      float meas = kin.pulsesToSpeed((long)d, dt);
      wheelSpeed[i] = SPEED_FILTER_ALPHA * meas + (1.0f - SPEED_FILTER_ALPHA) * wheelSpeed[i];
      if (failsafe) {
        pid[i].clearIntegral();   // 停车时清积分, 杜绝下次上电带饱和积分窜速
        motors[i].setPwm(0);
      } else {
        pid[i].update_target(target[i]);
        int duty = (int)pid[i].update(wheelSpeed[i]);
        motors[i].setPwm(duty);
      }
    }

    kin.updateOdom(wheelSpeed, dt);
    Odom o = kin.getOdom();
    MicroRos::setOdom(o.x, o.y, o.yaw, o.vx, o.vy, o.wz);

    vTaskDelayUntil(&last, period);
  }
}

// Core 0: 传感器采集 (IMU 串口 + INA226)
static void sensor_task(void* arg) {
  const TickType_t period = pdMS_TO_TICKS(1000 / SENSOR_RATE_HZ);
  TickType_t last = xTaskGetTickCount();
  while (true) {
    imu.poll();
    ImuData d = imu.get();
    if (d.valid) MicroRos::setImu(d.acc, d.gyro, d.angle);

    PowerData p = power.read();
    MicroRos::setBattery(p.voltage, p.current);

    vTaskDelayUntil(&last, period);
  }
}
#endif  // ENCODER_TEST
