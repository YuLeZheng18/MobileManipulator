#!/usr/bin/env python3
"""诊断: 复刻 computeCartesianPath 的连续下降, 区分"撞上了"与"关节走不过去".

每个 z 用上一点的解做 IK 种子(连续分支), 先带碰撞求解; 失败则关碰撞再求一次:
  带碰撞失败 + 关碰撞成功 -> 碰撞挡住, 再问 check_state_validity 要接触对
  两者都失败                -> 运动学/关节限位走不过去, 缩小盒子模型没用
"""
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

TRAY = {0: (-0.235, 0.047), 1: (-0.235, -0.062)}
Q_NEW = (0.008991, 0.000393, 0.999048, -0.043619)


class Probe(Node):
    def __init__(self):
        super().__init__("descent_probe")
        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        self.sv = self.create_client(GetStateValidity, "/check_state_validity")
        self.js = None
        self.create_subscription(JointState, "/joint_states", self._cb, 10)

    def _cb(self, m):
        self.js = m

    def solve(self, x, y, z, q, seed, collide):
        req = GetPositionIK.Request()
        req.ik_request.group_name = "arm"
        req.ik_request.ik_link_name = "suction_tip"
        req.ik_request.avoid_collisions = collide
        req.ik_request.timeout.sec = 1
        if seed is not None:
            req.ik_request.robot_state.joint_state = seed
        p = PoseStamped()
        p.header.frame_id = "base_link"
        p.pose.position.x, p.pose.position.y, p.pose.position.z = x, y, z
        p.pose.orientation.x, p.pose.orientation.y = q[0], q[1]
        p.pose.orientation.z, p.pose.orientation.w = q[2], q[3]
        req.ik_request.pose_stamped = p
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        r = fut.result()
        if r is None:
            return -99, None
        return r.error_code.val, r.solution.joint_state

    def contacts(self, state):
        req = GetStateValidity.Request()
        req.robot_state.joint_state = state
        req.group_name = "arm"
        fut = self.sv.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        r = fut.result()
        if r is None:
            return None
        return [(c.contact_body_1, c.contact_body_2) for c in r.contacts]


def main():
    rclpy.init()
    n = Probe()
    n.ik.wait_for_service(timeout_sec=10.0)
    n.sv.wait_for_service(timeout_sec=10.0)
    for _ in range(50):
        rclpy.spin_once(n, timeout_sec=0.1)
        if n.js is not None:
            break
    names = ["Joint_11", "Joint_12", "Joint_13", "Joint_14", "Joint_15", "Joint_16"]

    for tray, (x, y) in TRAY.items():
        print(f"\n=== {tray} 号托盘 连续下降 xy=({x},{y}) ===")
        seed = n.js
        for zi in range(200, 100, -5):
            z = zi / 1000.0
            code, sol = n.solve(x, y, z, Q_NEW, seed, True)
            if code == 1:
                seed = sol
                vals = [sol.position[sol.name.index(j)] for j in names if j in sol.name]
                print(f"  z={z:.3f} OK   " + " ".join(f"{v:+.3f}" for v in vals))
                continue
            code2, sol2 = n.solve(x, y, z, Q_NEW, seed, False)
            if code2 == 1:
                cs = n.contacts(sol2)
                print(f"  z={z:.3f} 碰撞挡住 (关碰撞可解). 接触对: {cs}")
            else:
                print(f"  z={z:.3f} 运动学不可达 (带碰撞{code} 关碰撞{code2}) <- 缩小盒子没用")
            break
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
