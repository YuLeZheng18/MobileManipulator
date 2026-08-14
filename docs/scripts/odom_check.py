#!/usr/bin/env python3
"""里程计标定复核 — 对比 EKF /odom 与轮式 /wheel_odom 的位移/航向累计.

用法 (在 Nano 上跑, 车推/开之前启动):
    python3 odom_check.py            # 交互: 回车开始记录, 再回车结束
    python3 odom_check.py 30         # 记录 30 秒后自动结束

判读:
  直行 2m  -> 看 dist, 与卷尺实测比. 偏差 >5% 说明轮径/减速比标定要修.
  原地 360 -> 看 yaw_total, 应接近 360. 偏小=转多了才报够(标定值偏大), 反之偏小.
  EKF 与 wheel 两列的差异反映 IMU 的修正量; 原地转时 wheel 若明显偏离 360 而 EKF 接近,
  说明 IMU 在救场(轮子打滑); 两者都偏则是标定问题.
"""
import math
import sys
import threading

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Track:
    """累计路径长度与连续航向(解卷绕)."""

    def __init__(self, label):
        self.label = label
        self.x0 = self.y0 = self.yaw0 = None
        self.x = self.y = self.yaw = 0.0
        self.dist = 0.0          # 累计走过的路径长度
        self.yaw_total = 0.0     # 累计转过的角度(带符号, 解卷绕)
        self._px = self._py = None
        self._pyaw = None
        self.n = 0

    def update(self, x, y, yaw):
        self.n += 1
        if self.x0 is None:
            self.x0, self.y0, self.yaw0 = x, y, yaw
            self._px, self._py, self._pyaw = x, y, yaw
            return
        self.dist += math.hypot(x - self._px, y - self._py)
        d = yaw - self._pyaw
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        self.yaw_total += d
        self._px, self._py, self._pyaw = x, y, yaw
        self.x, self.y, self.yaw = x, y, yaw

    def report(self):
        if self.x0 is None:
            return f"{self.label:12s}: no data"
        dx, dy = self.x - self.x0, self.y - self.y0
        straight = math.hypot(dx, dy)
        return (f"{self.label:12s}: msgs={self.n:5d}  "
                f"净位移={straight:6.3f}m (dx={dx:+.3f} dy={dy:+.3f})  "
                f"路径长={self.dist:6.3f}m  "
                f"累计转角={math.degrees(self.yaw_total):+8.2f}deg  "
                f"当前yaw={math.degrees(self.yaw):+7.2f}deg")


class Checker(Node):
    def __init__(self):
        super().__init__('odom_check')
        self.ekf = Track('EKF /odom')
        self.wheel = Track('/wheel_odom')
        self.imu_yaw_total = 0.0
        self._imu_prev = None
        self.imu_n = 0
        self.recording = False

        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, '/odom', self.cb_ekf, 10)
        self.create_subscription(Odometry, '/wheel_odom', self.cb_wheel, qos)
        self.create_subscription(Imu, '/imu', self.cb_imu, qos)

    def cb_ekf(self, m):
        if self.recording:
            p = m.pose.pose
            self.ekf.update(p.position.x, p.position.y, yaw_of(p.orientation))

    def cb_wheel(self, m):
        if self.recording:
            p = m.pose.pose
            self.wheel.update(p.position.x, p.position.y, yaw_of(p.orientation))

    def cb_imu(self, m):
        if not self.recording:
            return
        self.imu_n += 1
        y = yaw_of(m.orientation)
        if self._imu_prev is not None:
            d = y - self._imu_prev
            while d > math.pi:
                d -= 2 * math.pi
            while d < -math.pi:
                d += 2 * math.pi
            self.imu_yaw_total += d
        self._imu_prev = y

    def report(self):
        print("\n" + "=" * 78)
        print(self.ekf.report())
        print(self.wheel.report())
        print(f"{'IMU yaw':12s}: msgs={self.imu_n:5d}  "
              f"累计转角={math.degrees(self.imu_yaw_total):+8.2f}deg")
        print("=" * 78)
        e, w = self.ekf, self.wheel
        if e.n > 1 and w.n > 1:
            if abs(w.dist) > 0.05:
                print(f"路径长 EKF/wheel 比值: {e.dist / w.dist:.4f}")
            if abs(math.degrees(w.yaw_total)) > 5:
                print(f"转角   EKF/wheel 比值: {e.yaw_total / w.yaw_total:.4f}")
            print("\n判读: 直行看'路径长'与卷尺比; 原地转看'累计转角'与 360 比.")
            print("      两者都偏 -> 标定问题; 仅 wheel 偏而 EKF 接近 -> IMU 在修正打滑.")


def main():
    rclpy.init()
    n = Checker()
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else None

    t = threading.Thread(target=rclpy.spin, args=(n,), daemon=True)
    t.start()

    if dur:
        print(f"[3 秒后开始记录, 持续 {dur:.0f} 秒]")
        import time
        time.sleep(3)
        n.recording = True
        print(">>> 记录中... 现在开车/推车 <<<")
        time.sleep(dur)
        n.recording = False
    else:
        input("按回车开始记录...")
        n.recording = True
        input(">>> 记录中... 动作做完后按回车结束 <<<\n")
        n.recording = False

    n.report()
    n.destroy_node()
    rclpy.shutdown()


main()
