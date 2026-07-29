#!/usr/bin/env python3
"""诊断: 沿托盘正上方竖直下降路径逐点算 IK, 比较两种腕朝向的可达性.

用途: 放置段"垂直直下"覆盖不足时, 判是"托盘朝向改了导致不可达"还是别的原因.
输出每个 z 的 error_code: 1=有解, -31=无 IK 解, 其余见 moveit_msgs/MoveItErrorCodes.
"""
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

# 归一化四元数. old = 原始标定朝向(yaw≈180°); new = 2026-07-29 左旋 5° 后(yaw≈-175°)
ORI = {
    "old_180": (0.009000, 0.000000, 0.999960, 0.000000),
    "new_175": (0.008991, 0.000393, 0.999048, -0.043619),
}
TRAY = {0: (-0.235, 0.047), 1: (-0.235, -0.062)}


class IkProbe(Node):
    def __init__(self):
        super().__init__("ik_probe")
        self.cli = self.create_client(GetPositionIK, "/compute_ik")
        self.js = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

    def _on_js(self, m):
        self.js = m

    def probe(self, x, y, z, q):
        req = GetPositionIK.Request()
        req.ik_request.group_name = "arm"
        req.ik_request.ik_link_name = "suction_tip"
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec = 1
        if self.js is not None:
            req.ik_request.robot_state.joint_state = self.js
        p = PoseStamped()
        p.header.frame_id = "base_link"
        p.pose.position.x, p.pose.position.y, p.pose.position.z = x, y, z
        p.pose.orientation.x, p.pose.orientation.y = q[0], q[1]
        p.pose.orientation.z, p.pose.orientation.w = q[2], q[3]
        req.ik_request.pose_stamped = p
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        r = fut.result()
        return r.error_code.val if r is not None else -99


def main():
    rclpy.init()
    n = IkProbe()
    n.cli.wait_for_service(timeout_sec=10.0)
    for _ in range(50):
        rclpy.spin_once(n, timeout_sec=0.1)
        if n.js is not None:
            break
    print("joint_states:", "有" if n.js is not None else "无(IK 用默认种子)")
    for tray, (x, y) in TRAY.items():
        print(f"\n=== {tray} 号托盘 xy=({x},{y}) ===")
        print(f"{'z':>7} | " + " | ".join(f"{k:>9}" for k in ORI))
        for zi in range(20, 5, -1):
            z = zi / 100.0
            cells = [f"{n.probe(x, y, z, q):>9}" for q in ORI.values()]
            print(f"{z:7.3f} | " + " | ".join(cells))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
