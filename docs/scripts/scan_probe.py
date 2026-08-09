#!/usr/bin/env python3
"""量 /scan 的角度约定与自遮挡扇区。停在空地上跑,10 帧取每角度最小值。

输出两部分:
  1. 表头元数据 —— angle_min/max/increment 的**符号**是判断驱动角度手性的直接证据
  2. 每 10° 一桶的最小距离 —— 恒定很近的桶就是被机械臂/线材挡住的扇区

角度按 REP-103 报告(0°=+x 车头, 逆时针为正), 故 +90°=左, -90°=右, 180°=车后。
"""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

NBUCKET = 36  # 每桶 10°


class Probe(Node):
    def __init__(self, nframe):
        super().__init__('scan_probe')
        self.nframe = nframe
        self.frames = 0
        self.meta = None
        self.buckets = [math.inf] * NBUCKET
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, '/scan', self.cb, qos)

    def cb(self, m: LaserScan):
        if self.meta is None:
            self.meta = (m.header.frame_id, m.angle_min, m.angle_max,
                         m.angle_increment, m.range_min, m.range_max, len(m.ranges))
        for i, r in enumerate(m.ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue
            a = m.angle_min + i * m.angle_increment
            b = int(((math.degrees(a) + 180.0) % 360.0) // 10.0) % NBUCKET
            if r < self.buckets[b]:
                self.buckets[b] = r
        self.frames += 1


def main():
    nframe = int(sys.argv[1]) if len(sys.argv) > 1 else 10
    rclpy.init()
    n = Probe(nframe)
    while rclpy.ok() and n.frames < nframe:
        rclpy.spin_once(n, timeout_sec=2.0)
    if n.meta is None:
        print('没收到 /scan')
        return
    fid, amin, amax, ainc, rmin, rmax, cnt = n.meta
    print(f'frame_id      = {fid}')
    print(f'angle_min     = {math.degrees(amin):+8.2f} deg')
    print(f'angle_max     = {math.degrees(amax):+8.2f} deg')
    print(f'angle_incr    = {math.degrees(ainc):+8.4f} deg   <== 符号即手性证据')
    print(f'FOV           = {math.degrees(amax - amin):8.2f} deg  ({cnt} 点)')
    print(f'range         = {rmin:.3f} ~ {rmax:.1f} m')
    print(f'帧数          = {n.frames}\n')
    print('  角度区间        最小距离    (0=车头 +90=左 -90=右 180=车后)')
    for b in range(NBUCKET):
        lo = -180 + b * 10
        v = n.buckets[b]
        s = f'{v:6.3f} m' if math.isfinite(v) else '   ---  '
        bar = '#' * int(min(v, 3.0) / 3.0 * 30) if math.isfinite(v) else ''
        print(f'  [{lo:+4d},{lo + 10:+4d})   {s}  {bar}')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
