#!/usr/bin/env python3
"""Mock 盒子识别 (仿真专用, sim-only).

省掉真节点的"读深度图 -> AI 推理", 直接从 Gazebo 上帝真值取盒子位姿, 按接口契约 §1
发到同名话题 /perception/object_pose, 下游 grasp_node 分不出真假.

真值来源: 世界插件 gazebo_ros_state 发的 /gazebo/link_states (每 link 世界位姿).
用 base_link 与盒子 link 的世界位姿合成出"盒子相对 base_link", 加半高得顶面中心, 只留 yaw
(roll=pitch=0, 4-DOF top-down), 定频发布供闭环取新鲜观测.
"""

import math

import numpy as np
import rclpy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


def q_conjugate(q):
    # q = (x, y, z, w), 单位四元数的逆 = 共轭
    return np.array([-q[0], -q[1], -q[2], q[3]])


def q_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def q_rotate(q, v):
    # 用四元数 q 旋转向量 v: v' = q * (v,0) * q^-1
    qv = np.array([v[0], v[1], v[2], 0.0])
    r = q_multiply(q_multiply(q, qv), q_conjugate(q))
    return r[:3]


def yaw_from_quat(q):
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class MockObjectDetector(Node):
    def __init__(self):
        super().__init__('mock_object_detector')
        self.declare_parameter('base_link_name', 'mm_robot::base_link')
        self.declare_parameter('box_link_name', 'grasp_box::box_link')
        self.declare_parameter('box_height', 0.025)
        self.declare_parameter('output_frame', 'base_link')
        self.declare_parameter('topic', '/perception/object_pose')
        self.declare_parameter('publish_rate', 15.0)

        self.base_name = self.get_parameter('base_link_name').value
        self.box_name = self.get_parameter('box_link_name').value
        self.box_height = float(self.get_parameter('box_height').value)
        self.output_frame = self.get_parameter('output_frame').value
        rate = float(self.get_parameter('publish_rate').value)

        self._latest = {}  # link 全名 -> (pos np3, quat np4)
        self._warned = False

        self.sub = self.create_subscription(
            LinkStates, '/gazebo/link_states', self.on_link_states, 10)
        self.pub = self.create_publisher(
            PoseStamped, self.get_parameter('topic').value, 10)
        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(
            f'mock_object_detector: {self.box_name} -> {self.output_frame} '
            f'@ {rate:.0f}Hz, 顶面中心(box_height={self.box_height})')

    def on_link_states(self, msg: LinkStates):
        for name in (self.base_name, self.box_name):
            if name in msg.name:
                i = msg.name.index(name)
                p = msg.pose[i].position
                o = msg.pose[i].orientation
                self._latest[name] = (
                    np.array([p.x, p.y, p.z]),
                    np.array([o.x, o.y, o.z, o.w]),
                )

    def on_timer(self):
        if self.base_name not in self._latest or self.box_name not in self._latest:
            if not self._warned:
                self.get_logger().warn(
                    f'等待 link_states 里出现 {self.base_name} 与 {self.box_name} ...')
                self._warned = True
            return

        p_base, q_base = self._latest[self.base_name]
        p_box, q_box = self._latest[self.box_name]

        # 盒子顶面中心(世界系): 盒子平放, 世界 +z 即顶面法向, 加半高
        p_top_world = p_box + np.array([0.0, 0.0, self.box_height / 2.0])

        # 变到 base_link 系: box_in_base = inverse(base_in_world) * top_in_world
        q_base_inv = q_conjugate(q_base)
        rel_pos = q_rotate(q_base_inv, p_top_world - p_base)
        rel_q = q_multiply(q_base_inv, q_box)

        # 4-DOF top-down: 只保留绕竖直轴 yaw, roll=pitch=0
        yaw = yaw_from_quat(rel_q)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.output_frame
        msg.pose.position.x = float(rel_pos[0])
        msg.pose.position.y = float(rel_pos[1])
        msg.pose.position.z = float(rel_pos[2])
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = MockObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
