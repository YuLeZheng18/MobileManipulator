#include "MicroRosBridge.h"
#include "config.h"
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int8.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/battery_state.h>
#include <micro_ros_utilities/string_utilities.h>
#include <rmw_microros/rmw_microros.h>
#include <math.h>

static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;
static rclc_executor_t executor;

static rcl_subscription_t sub_cmd;
static geometry_msgs__msg__Twist msg_cmd;
static rcl_subscription_t sub_pump;
static std_msgs__msg__Int8 msg_pump;
static rcl_publisher_t pub_odom, pub_imu, pub_bat;
static nav_msgs__msg__Odometry msg_odom;
static sensor_msgs__msg__Imu msg_imu;
static sensor_msgs__msg__BatteryState msg_bat;
static rcl_timer_t timer;

// --- 共享数据 + 临界区 ---
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static CmdVel g_cmd;
static volatile int8_t g_pump = 0;   // 气泵命令 (PUMP_STOP/SUCK/RELEASE)
static struct { float x,y,yaw,vx,vy,wz; } g_odom = {};
static struct { float acc[3],gyro[3],angle[3]; } g_imu = {};
static struct { float v,i; } g_bat = {};
static volatile bool g_connected = false;
static volatile uint32_t g_cmd_ms = 0;   // 上次收到 /cmd_vel 的时刻 (millis)

CmdVel MicroRos::getCmd() { portENTER_CRITICAL(&mux); CmdVel c=g_cmd; portEXIT_CRITICAL(&mux); return c; }
int8_t MicroRos::getPump() { return g_pump; }
void MicroRos::setOdom(float x,float y,float yaw,float vx,float vy,float wz){
  portENTER_CRITICAL(&mux); g_odom={x,y,yaw,vx,vy,wz}; portEXIT_CRITICAL(&mux); }
void MicroRos::setImu(const float a[3],const float g[3],const float an[3]){
  portENTER_CRITICAL(&mux); for(int i=0;i<3;i++){g_imu.acc[i]=a[i];g_imu.gyro[i]=g[i];g_imu.angle[i]=an[i];} portEXIT_CRITICAL(&mux); }
void MicroRos::setBattery(float v,float i){ portENTER_CRITICAL(&mux); g_bat={v,i}; portEXIT_CRITICAL(&mux); }
bool MicroRos::isConnected(){ return g_connected; }
uint32_t MicroRos::cmdAgeMs(){ return millis() - g_cmd_ms; }

// 收到 /cmd_vel: 存入共享区
static void cmd_callback(const void* msgin) {
  const geometry_msgs__msg__Twist* m = (const geometry_msgs__msg__Twist*)msgin;
  portENTER_CRITICAL(&mux);
  g_cmd.vx = m->linear.x; g_cmd.vy = m->linear.y; g_cmd.wz = m->angular.z;
  portEXIT_CRITICAL(&mux);
  g_cmd_ms = millis();
}

// 收到 /pump_cmd: 存气泵命令, 由 control_task 驱动舵机信号
static void pump_callback(const void* msgin) {
  const std_msgs__msg__Int8* m = (const std_msgs__msg__Int8*)msgin;
  g_pump = m->data;
}

// 定时器: 发布 odom / imu / battery
static void timer_callback(rcl_timer_t* t, int64_t) {
  if (!t) return;
  int64_t ms = rmw_uros_epoch_millis();
  int32_t sec = (int32_t)(ms / 1000);
  uint32_t nsec = (uint32_t)((ms % 1000) * 1000000);

  // 取共享数据快照
  portENTER_CRITICAL(&mux);
  auto od = g_odom; auto im = g_imu; auto bt = g_bat;
  portEXIT_CRITICAL(&mux);

  // --- odom ---
  msg_odom.header.stamp.sec = sec; msg_odom.header.stamp.nanosec = nsec;
  msg_odom.pose.pose.position.x = od.x;
  msg_odom.pose.pose.position.y = od.y;
  msg_odom.pose.pose.orientation.z = sinf(od.yaw * 0.5f);
  msg_odom.pose.pose.orientation.w = cosf(od.yaw * 0.5f);
  msg_odom.twist.twist.linear.x = od.vx;
  msg_odom.twist.twist.linear.y = od.vy;
  msg_odom.twist.twist.angular.z = od.wz;
  rcl_publish(&pub_odom, &msg_odom, NULL);

  // --- imu (只发原始, 不积分) ---
  msg_imu.header.stamp.sec = sec; msg_imu.header.stamp.nanosec = nsec;
  msg_imu.linear_acceleration.x = im.acc[0];
  msg_imu.linear_acceleration.y = im.acc[1];
  msg_imu.linear_acceleration.z = im.acc[2];
  msg_imu.angular_velocity.x = im.gyro[0];
  msg_imu.angular_velocity.y = im.gyro[1];
  msg_imu.angular_velocity.z = im.gyro[2];
  // 姿态四元数 (用 yaw 角, roll/pitch 也可填; 这里给 HWT906P 融合的 yaw)
  msg_imu.orientation.z = sinf(im.angle[2] * 0.5f);
  msg_imu.orientation.w = cosf(im.angle[2] * 0.5f);
  rcl_publish(&pub_imu, &msg_imu, NULL);

  // --- battery ---
  msg_bat.header.stamp.sec = sec; msg_bat.header.stamp.nanosec = nsec;
  msg_bat.voltage = bt.v;
  msg_bat.current = bt.i;
  rcl_publish(&pub_bat, &msg_bat, NULL);
}

// 节流宏: 每 MS 毫秒执行一次 X (micro-ROS 官方 reconnect 例程写法)
#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static volatile int64_t _last = -1; \
  if (_last == -1) _last = (int64_t)millis(); \
  if ((int64_t)millis() - _last > (MS)) { X; _last = (int64_t)millis(); } \
} while (0)

// 创建全部 rcl 实体 (session/node/pub/sub/timer/executor)。任一步失败返回 false。
static bool create_entities() {
  allocator = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, MICROROS_DOMAIN_ID);
  rcl_ret_t rc = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  rcl_init_options_fini(&init_options);
  if (rc != RCL_RET_OK) return false;

  if (rclc_node_init_default(&node, "chassis_driver", "", &support) != RCL_RET_OK) return false;

  // 发布者 (best_effort, 高频传感器数据)
  // 话题名 /wheel_odom (非 /odom): /odom 由上位机 robot_localization EKF 输出独占,
  // 固件这路只是轮式里程计原始量, 作为 EKF 输入 (架构 §5.2)。frame_id 仍为 "odom"。
  if (rclc_publisher_init_best_effort(&pub_odom, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/wheel_odom") != RCL_RET_OK) return false;
  if (rclc_publisher_init_best_effort(&pub_imu, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu") != RCL_RET_OK) return false;
  if (rclc_publisher_init_best_effort(&pub_bat, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState), "/battery") != RCL_RET_OK) return false;

  // 订阅者 /cmd_vel (best_effort)
  if (rclc_subscription_init_best_effort(&sub_cmd, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel") != RCL_RET_OK) return false;
  // 订阅者 /pump_cmd (reliable, 命令不可丢)
  if (rclc_subscription_init_default(&sub_pump, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "/pump_cmd") != RCL_RET_OK) return false;

  if (rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50), timer_callback) != RCL_RET_OK) return false;

  // 句柄数 = sub_cmd + sub_pump + timer = 3
  executor = rclc_executor_get_zero_initialized_executor();
  if (rclc_executor_init(&executor, &support.context, 3, &allocator) != RCL_RET_OK) return false;
  rclc_executor_add_subscription(&executor, &sub_cmd, &msg_cmd, &cmd_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_pump, &msg_pump, &pump_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);
  return true;
}

// 销毁全部实体。set..destroy_session_timeout(0): 代理已掉线时不要阻塞等待回应。
static void destroy_entities() {
  rmw_context_t* rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

  rcl_publisher_fini(&pub_odom, &node);
  rcl_publisher_fini(&pub_imu, &node);
  rcl_publisher_fini(&pub_bat, &node);
  rcl_subscription_fini(&sub_cmd, &node);
  rcl_subscription_fini(&sub_pump, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void MicroRos::task(void* arg) {
  // 原生 USB 串口传输 (Serial = USB-OTG)
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  // frame_id 是消息缓冲区字段, 与会话无关, 只设一次 (避免每次重连都重新分配)
  msg_odom.header.frame_id = micro_ros_string_utilities_set(msg_odom.header.frame_id, "odom");
  msg_odom.child_frame_id  = micro_ros_string_utilities_set(msg_odom.child_frame_id, "base_link");
  msg_imu.header.frame_id  = micro_ros_string_utilities_set(msg_imu.header.frame_id, "imu_link");

  // 状态机: 启动顺序无关 + 掉线自愈 (micro-ROS 官方 ESP32 reconnect 模式)
  enum State { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
  State state = WAITING_AGENT;

  while (true) {
    switch (state) {
      case WAITING_AGENT:
        // 每 500ms ping 一次代理, 通了才往下走 (期间让出 CPU 喂看门狗)
        EXECUTE_EVERY_N_MS(500,
          state = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) ? AGENT_AVAILABLE : WAITING_AGENT;);
        vTaskDelay(pdMS_TO_TICKS(10));
        break;

      case AGENT_AVAILABLE:
        if (create_entities()) {
          rmw_uros_sync_session(1000);   // 时间同步 (供 timer_callback 打时间戳)
          g_connected = true;
          state = AGENT_CONNECTED;
        } else {
          destroy_entities();            // 半成品清理, 回退重试
          state = WAITING_AGENT;
          vTaskDelay(pdMS_TO_TICKS(500));
        }
        break;

      case AGENT_CONNECTED: {
        // 每 200ms 探活: ping(100ms×5). 电机 20kHz PWM 会在原生USB上打EMI, 偶发丢包,
        // 单次丢 ping 不拆会话, 连续两轮都失败才判掉线 (~400ms 内确认代理真没了)
        static int miss = 0;
        EXECUTE_EVERY_N_MS(200,
          if (rmw_uros_ping_agent(100, 5) != RMW_RET_OK) {
            if (++miss >= 2) { state = AGENT_DISCONNECTED; miss = 0; }
          } else miss = 0;);
        if (state == AGENT_CONNECTED) {
          rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
          vTaskDelay(pdMS_TO_TICKS(5));  // 让出 CPU 喂看门狗
        }
        break;
      }

      case AGENT_DISCONNECTED:
        g_connected = false;
        destroy_entities();
        state = WAITING_AGENT;
        break;
    }
  }
}
