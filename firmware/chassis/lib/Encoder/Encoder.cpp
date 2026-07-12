#include "Encoder.h"
#include "driver/pcnt.h"

// 溢出阈值 (int16 上下限附近触发事件累加)
static const int16_t PCNT_H_LIM = 30000;
static const int16_t PCNT_L_LIM = -30000;
static volatile long long s_overflow[PCNT_UNIT_MAX] = {0};

static void IRAM_ATTR pcntOnReach(void* arg) {
  uint32_t status[PCNT_UNIT_MAX];
  for (int u = 0; u < PCNT_UNIT_MAX; u++) {
    pcnt_get_event_status((pcnt_unit_t)u, &status[u]);
    if (status[u] & PCNT_EVT_H_LIM) s_overflow[u] += PCNT_H_LIM;
    if (status[u] & PCNT_EVT_L_LIM) s_overflow[u] += PCNT_L_LIM;
  }
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
  pcnt_isr_handler_add((pcnt_unit_t)unit, pcntOnReach, nullptr);
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
