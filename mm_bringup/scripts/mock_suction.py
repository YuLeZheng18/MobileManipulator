#!/usr/bin/env python3
"""Mock 吸盘假吸附 (仿真专用, sim-only).

真机吸盘靠气泵抽真空吸住药盒, 仿真里没有真空/吸力(那是真机才验的物理量, 见架构 §8.3),
所以本节点只跟"吸着/没吸着"这个逻辑状态, 用高频瞬移伪造"吸附跟随":

  订 /pump_cmd (std_msgs/Int8, 与真机同话题同类型):
    1=吸  : 若吸盘 TCP 贴近药盒顶面 -> 立刻标记吸附 (不模拟真机建真空那几秒)
    0=保压: 保持吸附 (对应搬运阶段, 真机关泵靠封闭真空拎着走)
    2=释放: 解除吸附, 盒子交回物理自由落体

  吸附期间定频(默认 50Hz)调 /gazebo/set_entity_state 把盒子瞬移到吸盘 TCP 正下方,
  twist 清零(消除残留速度, 释放后从静止自然下落). 没有真 fixed joint, 靠高频重摆伪造连接.

吸盘 TCP 位姿: 由腕部 Link_29 世界位姿(取自 /gazebo/link_states) 加 URDF 固定偏置算得
(suction_tip 在 Link_29 -Z 0.095m); 盒中心再往下半个盒高, 使盒顶面贴住吸盘.
"""

import numpy as np
import rclpy
from gazebo_msgs.msg import EntityState, LinkStates
from gazebo_msgs.srv import SetEntityState
from rclpy.node import Node
from std_msgs.msg import Int8

PUMP_STOP = 0     # 保压
PUMP_SUCK = 1     # 吸
PUMP_RELEASE = 2  # 释放


def q_conjugate(q):
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
    qv = np.array([v[0], v[1], v[2], 0.0])
    r = q_multiply(q_multiply(q, qv), q_conjugate(q))
    return r[:3]


class MockSuction(Node):
    def __init__(self):
        super().__init__('mock_suction')
        self.declare_parameter('wrist_link_name', 'mm_robot::Link_29')
        self.declare_parameter('box_model_name', 'grasp_box')
        self.declare_parameter('box_link_name', 'grasp_box::box_link')
        self.declare_parameter('pump_topic', '/pump_cmd')
        # suction_tip 相对 Link_29 的固定偏置 (Link_29 -Z 方向 9.5cm), 见 URDF Joint_suction_tip
        self.declare_parameter('tip_offset_z', -0.095)
        self.declare_parameter('box_half_height', 0.0125)
        self.declare_parameter('attach_threshold', 0.05)  # TCP 距盒顶面 <5cm 才吸得住

        self.wrist_name = self.get_parameter('wrist_link_name').value
        self.box_model = self.get_parameter('box_model_name').value
        self.box_link = self.get_parameter('box_link_name').value
        self.tip_off = np.array([0.0, 0.0, float(self.get_parameter('tip_offset_z').value)])
        self.half_h = float(self.get_parameter('box_half_height').value)
        self.thresh = float(self.get_parameter('attach_threshold').value)

        self._latest = {}      # link 全名 -> (pos np3, quat np4)
        self._cmd = PUMP_STOP
        self._attached = False
        self._future = None

        self.sub_links = self.create_subscription(
            LinkStates, '/gazebo/link_states', self.on_link_states, 10)
        self.sub_pump = self.create_subscription(
            Int8, self.get_parameter('pump_topic').value, self.on_pump, 10)
        self.cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        # 携运直接挂在 link_states 回调上(事件驱动, 用最新腕部真值). 不走 ROS 定时器:
        # 仿真 /clock 只有 10Hz, sim-time 定时器会被压到 10Hz -> 两次瞬移间盒子自由落体 ~5cm 下垂;
        # link_states ~30Hz(wall) 触发一次瞬移, 下落窗口 ~34ms, 下垂降到 <1cm.
        self.get_logger().info(
            f'mock_suction: 吸盘 TCP=({self.wrist_name} -Z {-self.tip_off[2]:.3f}m), '
            f'吸附阈值 {self.thresh * 100:.0f}cm, 携运随 /gazebo/link_states 触发. '
            f'订 {self.get_parameter("pump_topic").value}')

    def on_link_states(self, msg: LinkStates):
        for name in (self.wrist_name, self.box_link):
            if name in msg.name:
                i = msg.name.index(name)
                p = msg.pose[i].position
                o = msg.pose[i].orientation
                self._latest[name] = (
                    np.array([p.x, p.y, p.z]),
                    np.array([o.x, o.y, o.z, o.w]),
                )
        self._update()

    def on_pump(self, msg: Int8):
        self._cmd = int(msg.data)
        if self._cmd == PUMP_RELEASE and self._attached:
            self._attached = False
            self.get_logger().info('释放: 解除吸附, 盒子交回物理自由落体')

    def _tip_world(self):
        p29, q29 = self._latest[self.wrist_name]
        return p29 + q_rotate(q29, self.tip_off), q29

    def _update(self):
        if self.wrist_name not in self._latest:
            return

        # 未吸附 + 收到吸气指令: 就近判定能不能吸上
        if not self._attached and self._cmd == PUMP_SUCK:
            if self.box_link in self._latest:
                tip_pos, _ = self._tip_world()
                p_box, _ = self._latest[self.box_link]
                box_top = p_box + np.array([0.0, 0.0, self.half_h])
                dist = float(np.linalg.norm(tip_pos - box_top))
                if dist < self.thresh:
                    self._attached = True
                    self.get_logger().info(f'吸气: TCP 距盒顶 {dist * 100:.1f}cm < 阈值, 吸附成功')

        # 吸附期间(吸/保压都算): 每收到一帧腕部真值就把盒子瞬移到 TCP 正下方
        if self._attached:
            self._carry()

    def _carry(self):
        if not self.cli.service_is_ready():
            return
        if self._future is not None and not self._future.done():
            return  # 上一次调用还没回, 不堆积
        p29, q29 = self._latest[self.wrist_name]
        # 盒中心 = TCP 再沿 Link_29 -Z 下移半个盒高, 使盒顶面贴住吸盘; 盒姿态跟腕部
        box_center = p29 + q_rotate(q29, self.tip_off + np.array([0.0, 0.0, -self.half_h]))

        st = EntityState()
        st.name = self.box_model
        st.reference_frame = ''  # 空 = world 系
        st.pose.position.x = float(box_center[0])
        st.pose.position.y = float(box_center[1])
        st.pose.position.z = float(box_center[2])
        st.pose.orientation.x = float(q29[0])
        st.pose.orientation.y = float(q29[1])
        st.pose.orientation.z = float(q29[2])
        st.pose.orientation.w = float(q29[3])
        # twist 全零: 消除瞬移残留速度, 释放时从静止起落
        req = SetEntityState.Request()
        req.state = st
        self._future = self.cli.call_async(req)


def main():
    rclpy.init()
    node = MockSuction()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
