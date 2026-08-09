#include "Encoder.h"
#include "driver/pcnt.h"

// 溢出阈值 (int16 上下限附近触发事件累加)
static const int16_t PCNT_H_LIM = 30000;
static const int16_t PCNT_L_LIM = -30000;

// ⚠️ 类型必须是 32 位, 不能是 long long: ESP32-S3 是 32 位核, 64 位读写要两条指令,
// 而本变量在 ISR 里写、在 control_task(100Hz) 里经 getCount() 读。撕裂读(低32位新/
// 高32位旧)会产生 2^32 量级的假跳变 —— 表现为**偶发**里程计飞车, 因为只在跨 32 位
// 边界那一刻发生。int32 单指令读写天然原子, 且量程足够:
// 30000 脉冲/次 → 2^31/30000 ≈ 7.2 万次溢出, 按标定 380PPR/轮圈、轮径 39.1mm 折算
// 约 4400km 里程才回绕, 远超本机寿命。
static volatile int32_t s_overflow[PCNT_UNIT_MAX] = {0};

// ⚠️ 只处理 arg 传进来的那一个 unit, **绝不遍历全表**。
// 原实现在此扫 for(u=0..PCNT_UNIT_MAX) 读所有 unit 状态, 而本回调经
// pcnt_isr_handler_add 给四个 unit **各注册了一份**(见 begin() 末尾, 四轮各调一次)。
// 任一轮溢出 → 四份 handler 全被调 → 每份又扫全表 → 同一个溢出事件被重复累加最多
// 4 次 × 30000 脉冲。下游放大成三重故障(2026-08-08 实测):
//   轮速被读成 143m/s → PID 误差 ×KP350 → 占空比顶到反向满量程 = 轮子突然乱转;
//   Kinematics::updateOdom 纯积分, 一个 tick 掀掉 296° 航向 = 里程计飞且永不恢复;
//   四轮同时窜速拉崩电流 → 掉压复位(chassis_diag 的 up 归零)。
static void IRAM_ATTR pcntOnReach(void* arg) {
  const int u = (int)(intptr_t)arg;
  uint32_t status = 0;
  pcnt_get_event_status((pcnt_unit_t)u, &status);
  if (status & PCNT_EVT_H_LIM) s_overflow[u] += PCNT_H_LIM;
  if (status & PCNT_EVT_L_LIM) s_overflow[u] += PCNT_L_LIM;
}

void Encoder::begin(int unit, uint8_t pinA, uint8_t pinB) {
  unit_ = unit;
  pcnt_config_t cfg = {};
  cfg.unit = (pcnt_unit_t)unit;
  cfg.counter_h_lim = PCNT_H_LIM;
  cfg.counter_l_lim = PCNT_L_LIM;

  // 通道 0: pulse=A / ctrl=B, 对 A 相双边沿计数, B 相电平定方向
  cfg.pulse_gpio_num = pinA;
  cfg.ctrl_gpio_num  = pinB;
  cfg.channel   = PCNT_CHANNEL_0;
  cfg.pos_mode  = PCNT_COUNT_INC;      // A 上升沿 (方向对齐运动学正向, 实车手转标定)
  cfg.neg_mode  = PCNT_COUNT_DEC;      // A 下降沿
  cfg.lctrl_mode = PCNT_MODE_KEEP;     // B 低: 保持
  cfg.hctrl_mode = PCNT_MODE_REVERSE;  // B 高: 反向
  pcnt_unit_config(&cfg);

  // 通道 1: pulse=B / ctrl=A, 把 B 相双边沿也算进来 -> 真 4 倍频正交
  // 模式与通道 0 成对镜像 (ctrl 高低互换), 保证两通道计数同向
  cfg.pulse_gpio_num = pinB;
  cfg.ctrl_gpio_num  = pinA;
  cfg.channel   = PCNT_CHANNEL_1;
  cfg.pos_mode  = PCNT_COUNT_INC;      // B 上升沿 (与通道0同步翻向)
  cfg.neg_mode  = PCNT_COUNT_DEC;      // B 下降沿
  cfg.lctrl_mode = PCNT_MODE_REVERSE;  // A 低: 反向
  cfg.hctrl_mode = PCNT_MODE_KEEP;     // A 高: 保持
  pcnt_unit_config(&cfg);
  pcnt_set_filter_value((pcnt_unit_t)unit, 100);
  pcnt_filter_enable((pcnt_unit_t)unit);
  pcnt_event_enable((pcnt_unit_t)unit, PCNT_EVT_H_LIM);
  pcnt_event_enable((pcnt_unit_t)unit, PCNT_EVT_L_LIM);
  pcnt_counter_pause((pcnt_unit_t)unit);
  pcnt_counter_clear((pcnt_unit_t)unit);
  static bool isr_installed = false;
  if (!isr_installed) { pcnt_isr_service_install(0); isr_installed = true; }
  // 把 unit 号作为 arg 传给 ISR: 回调据此只碰自己那一路 (见 pcntOnReach 的警告)
  pcnt_isr_handler_add((pcnt_unit_t)unit, pcntOnReach, (void*)(intptr_t)unit);
  pcnt_counter_resume((pcnt_unit_t)unit);
}

long long Encoder::getCount() {
  int16_t c = 0;
  pcnt_get_counter_value((pcnt_unit_t)unit_, &c);
  return s_overflow[unit_] + c;
}

long long Encoder::readDelta() {
  long long now = getCount();
  long long d = now - base_;
  base_ = now;
  return d;
}
