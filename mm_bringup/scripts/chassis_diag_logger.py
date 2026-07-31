#!/usr/bin/env python3
"""/chassis_diag 落盘记录器 (真机工具, 跑在 Nano 上)。

存在的理由: /chassis_diag 与被观测对象共享同一条链路 —— 传输层死了它自己也一起哑,
不会在故障瞬间报警。判读靠对比"停摆前最后一条"与"恢复后第一条"的 up 值, 所以必须有人
把消息落盘; 只在终端 echo 的话, 停摆时人不在看, 那条关键记录就丢了。

判读(本脚本在 RESUME 行自动给出结论, 不用人再推):
  up 继续涨   -> 芯片没重启, 是任务/传输层死了
  up 重新计   -> 芯片复位过, 看 rst 定性 (0=无法判定/烧录后首启 1=上电 3=软复位
                 4=看门狗panic 5=中断看门狗 6=任务看门狗 9=brownout)
"""

import re
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

UP_RE = re.compile(r"up=(\d+)s")


class ChassisDiagLogger(Node):
    def __init__(self):
        super().__init__("chassis_diag_logger")

        self.declare_parameter("log_path", "/home/dong/chassis_diag.log")
        # 话题 2s 一条, 5s 没来即认定停发 (留 2.5 个周期容错, 避免抖动误报)
        self.declare_parameter("gap_threshold_sec", 5.0)

        self.path = self.get_parameter("log_path").value
        self.gap_threshold = float(self.get_parameter("gap_threshold_sec").value)

        self.last_msg_wall = None   # 上一条消息的到达时刻 (wall clock)
        self.last_up = None         # 上一条消息里的 up 值
        self.last_line = None       # 上一条消息原文, GAP 时要记下来
        self.in_gap = False

        # /chassis_diag 是 best_effort, QoS 必须匹配否则完全收不到
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(String, "/chassis_diag", self.on_diag, qos)

        # 用 wall clock 而非 ROS 时钟: 要记录的正是"链路死了多久", 不能依赖链路
        self.create_timer(1.0, self.check_gap)

        self._write(f"=== logger started, gap_threshold={self.gap_threshold}s ===")
        self.get_logger().info(f"记录 /chassis_diag 到 {self.path}")

    def _write(self, text):
        stamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open(self.path, "a") as f:
            f.write(f"[{stamp}] {text}\n")
            # 每条都 flush: 停摆时进程可能被 kill, 缓冲区里丢掉的恰好是最关键的最后一条
            f.flush()

    @staticmethod
    def _parse_up(text):
        m = UP_RE.search(text)
        return int(m.group(1)) if m else None

    def on_diag(self, msg):
        now = time.monotonic()
        up = self._parse_up(msg.data)

        if self.in_gap:
            self.in_gap = False
            verdict = self._verdict(up)
            self._write(f"RESUME {msg.data}  <<< {verdict}")
        else:
            self._write(msg.data)

        self.last_msg_wall = now
        self.last_up = up
        self.last_line = msg.data

    def _verdict(self, up_now):
        """恢复后给判读结论。up 是否延续是唯一判据。"""
        if up_now is None or self.last_up is None:
            return "无法判读 (up 值缺失)"
        if up_now >= self.last_up:
            return (
                f"芯片没重启 (up {self.last_up}->{up_now} 延续) "
                f"=> 任务/传输层死了"
            )
        return (
            f"芯片复位过 (up {self.last_up}->{up_now} 重新计) "
            f"=> 看 rst 定性"
        )

    def check_gap(self):
        if self.last_msg_wall is None or self.in_gap:
            return
        silent = time.monotonic() - self.last_msg_wall
        if silent > self.gap_threshold:
            self.in_gap = True
            self._write(
                f"*** GAP: {silent:.1f}s 无消息, 停发前最后一条: {self.last_line} ***"
            )


def main():
    rclpy.init()
    node = ChassisDiagLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._write("=== logger stopped ===")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
