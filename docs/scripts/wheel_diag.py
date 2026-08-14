#!/usr/bin/env python3
"""间歇性单轮失效定位 — 逐帧检测, 记录每一次掉线段的时长与嫌疑轮.

原理: 角落四全向轮纯平移时四轮速度代数和恰为 0 (Kinematics.cpp:25 的 wz 分子),
少一个轮子这个和就不为零, 固件于是报出**车不存在的自转**。以 vx=0.2m/s 前进为例,
右后轮停转 -> odom 报 wz=-10.3deg/s、vy=+0.05m/s、vx 掉到 75%。信号很大, 无需平均,
故可逐帧判定, 适合抓偶发。

    死掉的轮      假 vy    假 wz
    0 左前          +      + (逆时针)
    1 左后          -      +
    2 右后          +      - (顺时针)
    3 右前          -      -

判读掉线**时长**可区分成因:
    0.3~2s 能自行恢复, 且指令越大恢复越快 -> 摩擦卡死, PID 积分顶到满占空比才挣脱(机械)
    时长随机、与指令大小无关              -> 接线/接触不良(电气)

用法: 后台长跑, 边开边记。按住 R1 正常开(以平移为主), 结束按 Ctrl-C。
    python3 wheel_diag.py 300
"""
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

WHEEL = {(1, 1): '0 左前', (-1, 1): '1 左后',
         (1, -1): '2 右后', (-1, -1): '3 右前'}

WZ_THR = 0.05      # rad/s, 假自转判定阈 (单轮失效理论值 ~0.18, 静态噪声 <0.02)
VY_THR = 0.015     # m/s,   假横移判定阈
MIN_CMD_VX = 0.03  # 指令低于此不判 (静止/极慢时信噪比不够)


class Diag(Node):
    def __init__(self):
        super().__init__('wheel_diag')
        be = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd, 10)
        self.create_subscription(Odometry, '/wheel_odom', self.cb_odom, be)
        self.cmd = (0.0, 0.0, 0.0)
        self.t0 = time.time()
        self.events = []       # 已结束的掉线段 (start, dur, wheel, cmd_vx, ratio)
        self.cur = None        # 进行中的掉线段
        self.n_odom = 0
        self.n_judged = 0      # 参与判定的帧数(纯平移窗口)

    def cb_cmd(self, m):
        self.cmd = (m.linear.x, m.linear.y, m.angular.z)

    def cb_odom(self, m):
        self.n_odom += 1
        cvx, cvy, cwz = self.cmd
        now = time.time() - self.t0
        t = m.twist.twist

        # 只在"指令纯平移"窗口判定: 指令自带转向时 odom 的 wz 本该非零, 无法区分
        if abs(cvx) < MIN_CMD_VX or abs(cwz) > 0.02 or abs(cvy) > 0.02:
            self._close(now)
            return
        self.n_judged += 1

        bad = abs(t.angular.z) > WZ_THR and abs(t.linear.y) > VY_THR
        if not bad:
            self._close(now)
            return

        sig = (1 if t.linear.y > 0 else -1, 1 if t.angular.z > 0 else -1)
        who = WHEEL[sig]
        ratio = t.linear.x / cvx if cvx else 0.0
        if self.cur is None:
            self.cur = [now, now, who, cvx, ratio, 1]
            print(f"  [{now:6.1f}s] 掉线开始 -> {who}  "
                  f"wz={math.degrees(t.angular.z):+6.1f}d/s vy={t.linear.y:+.3f} "
                  f"vx跟随={100*ratio:.0f}%", flush=True)
        else:
            self.cur[1] = now
            self.cur[5] += 1
            if who != self.cur[2]:      # 段内换轮 = 判定不稳, 记下来
                self.cur[2] = self.cur[2] + '/' + who

    def _close(self, now):
        if self.cur is None:
            return
        start, end, who, cvx, ratio, n = self.cur
        dur = end - start
        if n >= 3:                      # 至少 3 帧(~0.15s), 滤掉单帧毛刺
            self.events.append((start, dur, who, cvx, ratio))
            print(f"  [{now:6.1f}s] 掉线结束, 持续 {dur:.2f}s  ({who})", flush=True)
        self.cur = None


def report(n):
    print("\n" + "=" * 70)
    span = time.time() - n.t0
    print(f"总时长 {span:.0f}s   /wheel_odom {n.n_odom} 帧   "
          f"其中纯平移可判帧 {n.n_judged}")
    if n.n_judged < 50:
        print("可判帧太少 — 摇杆推得少或一直带着转向, 结论不可信, 重测。")
        return
    if not n.events:
        print("\n未捕获掉线。本轮四轮全程正常 —— 偶发故障没在此窗口内出现,")
        print("延长时间再跑, 或复现时立刻看这里的输出。")
        return

    print(f"\n捕获 {len(n.events)} 次掉线:")
    print(f"  {'起始':>8} {'时长':>7} {'嫌疑轮':<12} {'指令vx':>8} {'vx跟随':>7}")
    for st, dur, who, cvx, ratio in n.events:
        print(f"  {st:7.1f}s {dur:6.2f}s {who:<12} {cvx:+7.3f} {100*ratio:6.0f}%")

    from collections import Counter
    cnt = Counter(e[2] for e in n.events)
    durs = [e[1] for e in n.events]
    total_bad = sum(durs)
    print(f"\n嫌疑轮分布: {dict(cnt)}")
    print(f"掉线总时长 {total_bad:.1f}s, 占可判时段的 "
          f"{100*total_bad/span:.1f}% (粗略)")
    print(f"时长 min={min(durs):.2f}s max={max(durs):.2f}s "
          f"mean={sum(durs)/len(durs):.2f}s")

    print("\n===== 判读 =====")
    top, n_top = cnt.most_common(1)[0]
    if n_top / len(n.events) > 0.7:
        print(f"{100*n_top/len(n.events):.0f}% 的掉线都指向 {top} -> 固定单点故障。")
        ratios = [e[4] for e in n.events if e[2] == top]
        mr = sum(ratios) / len(ratios)
        print(f"该轮掉线时 vx 跟随率均值 {100*mr:.0f}% (完全失效理论值 75%)")
        if mr > 0.9:
            print("  跟随率接近 100% -> 轮子其实**在转**, 是这一路**编码器**没数;")
            print("  编码器丢数还会让 PID 误判为'没转'而顶满占空比 -> 该轮实际窜速。")
            print("  查: 编码器 A/B 相接线与插头(对应 GPIO 见 config.h:20-27)。")
        else:
            print("  跟随率明显偏低 -> 轮子真的没转, 是电机/驱动器/动力线故障。")
            print("  查: 该轮 BM50 驱动器接线、DIR/PWM 插头、电机动力线。")
    else:
        print("掉线分散在多个轮 -> 不像单点故障。可能是共用件:")
        print("  公共刹车线 (MOTOR_BRAKE=GPIO42, 四轮共线, 低电平刹车) 接触不良,")
        print("  或电池压降导致驱动器集体欠压 —— 后者看 /battery 电压是否随之下坠。")

    if max(durs) < 2.5 and len(durs) > 1:
        print(f"\n单次掉线都在 {max(durs):.1f}s 内自行恢复: 符合摩擦卡死 + PID 积分")
        print("顶到满占空比后挣脱的特征 (PidController.cpp:19 积分限幅=255/KI)。")


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0
    rclpy.init()
    n = Diag()
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
    print(f">>> 记录中 (最多 {dur:.0f}s) — 按住 R1 正常开, 以前进/横移为主 <<<")
    print(">>> 复现时这里会实时打印; 开够了按 Ctrl-C 出报告 <<<\n", flush=True)
    try:
        time.sleep(dur)
    except KeyboardInterrupt:
        pass
    n._close(time.time() - n.t0)
    report(n)
    n.destroy_node()
    rclpy.shutdown()


main()
