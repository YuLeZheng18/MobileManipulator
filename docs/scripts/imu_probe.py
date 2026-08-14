#!/usr/bin/env python3
"""IMU yaw 行为诊断: 区分"持续零偏漂移"与"离散跳变".

直行 1.6m 那趟 IMU 累计转角报了 -15.46deg 而车实际几乎没转, 本脚本判定成因:
  漂移 -> 每帧增量符号一致、量级微小且稳定, 漂移率 deg/s 基本恒定
  跳变 -> 绝大多数帧增量近 0, 少数帧出现大跳 (打印 top 增量即可看出)
静止采集即可, 车不用动.
"""
import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Probe(Node):
    def __init__(self):
        super().__init__('imu_probe')
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Imu, '/imu', self.cb, qos)
        self.deltas = []       # (dt, dyaw_deg)
        self.yaws = []
        self.gz = []           # 角速度 z
        self.t0 = None
        self.tprev = None
        self.yprev = None

    def cb(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        y = yaw_of(m.orientation)
        self.gz.append(m.angular_velocity.z)
        if self.t0 is None:
            self.t0 = t
        self.yaws.append((t - self.t0, math.degrees(y)))
        if self.yprev is not None:
            d = y - self.yprev
            while d > math.pi:
                d -= 2 * math.pi
            while d < -math.pi:
                d += 2 * math.pi
            dt = t - self.tprev
            self.deltas.append((dt, math.degrees(d)))
        self.tprev, self.yprev = t, y


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    rclpy.init()
    n = Probe()
    import time
    end = time.time() + dur
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)

    if len(n.deltas) < 5:
        print(f"数据不足: {len(n.deltas)} 帧")
        return

    ds = [d for _, d in n.deltas]
    total = sum(ds)
    span = n.yaws[-1][0] - n.yaws[0][0]
    print(f"采集 {len(n.yaws)} 帧, 时长 {span:.2f}s")
    print(f"yaw 起始 {n.yaws[0][1]:+.3f}deg  ->  结束 {n.yaws[-1][1]:+.3f}deg")
    print(f"累计转角 {total:+.3f}deg   等效漂移率 {total/span:+.4f} deg/s")
    print(f"\n每帧增量统计 (deg):")
    print(f"  mean={sum(ds)/len(ds):+.6f}  min={min(ds):+.4f}  max={max(ds):+.4f}")
    pos = sum(1 for d in ds if d > 1e-9)
    neg = sum(1 for d in ds if d < -1e-9)
    zero = len(ds) - pos - neg
    print(f"  正增量 {pos}  负增量 {neg}  零 {zero}   -> 符号一致性: "
          f"{max(pos,neg)/len(ds)*100:.1f}%")

    big = sorted(n.deltas, key=lambda x: -abs(x[1]))[:8]
    print(f"\n最大 8 个增量 (dt, dyaw):")
    for dt, d in big:
        print(f"   dt={dt:.4f}s  dyaw={d:+.4f}deg   ({d/dt if dt>0 else 0:+.2f} deg/s)")

    # 大跳贡献占比: 判定漂移 vs 跳变的关键
    thr = 0.05
    jump_sum = sum(d for _, d in n.deltas if abs(d) > thr)
    print(f"\n|增量|>{thr}deg 的帧贡献了 {jump_sum:+.3f}deg / 总 {total:+.3f}deg "
          f"({100*jump_sum/total if abs(total)>1e-6 else 0:.1f}%)")

    gz_mean = sum(n.gz) / len(n.gz)
    print(f"\n角速度 z: mean={gz_mean:+.6f} rad/s = {math.degrees(gz_mean):+.4f} deg/s")
    print(f"  (静止时该值即陀螺零偏; 与上面漂移率对比可判断 yaw 是否由它积分而来)")

    print("\n===== 判读 =====")
    if abs(total) < 1.0:
        print("累计漂移 <1deg: 本次采集未复现问题(可能与运动/振动相关, 需动态复测)")
    elif max(pos, neg) / len(ds) > 0.8:
        print("符号高度一致 -> 持续零偏漂移。EKF 把 IMU yaw 当绝对观测会被拖偏。")
    elif abs(jump_sum / total) > 0.7 if abs(total) > 1e-6 else False:
        print("大跳主导 -> 离散跳变(丢包/解析错), 不是漂移。")
    else:
        print("混合特征, 看上面明细判断。")

    n.destroy_node()
    rclpy.shutdown()


main()
