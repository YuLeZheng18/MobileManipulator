#!/usr/bin/env python3
"""把 grasp_box 摆到车当前位姿右侧可达点 (仿真专用, sim-only, 全流程绿跑辅助).

全场只有一个盒子, 静态钉在 world 某处; 车道节点离它好几米, 导航到货架后盒子仍在原地,
S4 抓取 IK 必失败. 本节点在车"导航到位"时 (订 /lane_navigator/status 收到
"<trigger_target>:SUCCEEDED"), 按此刻实时 base_link 真值 ⊕ base_link 系偏置
(offset_x,offset_y), 把盒子瞬移到车右侧、世界高度 box_rest_z 处 —— 正是机械臂零位
调通的可达配置。mock_object_detector 随即报出新真值, S4 抓取成立。

用"到位后的实时 base_link"算落点是关键: 早前按目标节点位姿(lane_graph.yaml)提前摆盒,
但车导航到节点存在 cm 级欠冲/偏航, 每轮 AMCL 位姿又不同, 盒子相对"实际 base_link"落点
飘忽, 常落进粗定位残差精修 xy 收不回的构型. 改成到位后读实时 base_link, 每轮都落在
机械臂实测甜点 (0.15,-0.28), 粗定位一次落准.

⚠️ 只在"第一次抓取前"用: 抓起后 mock_suction 会把盒子瞬移跟随腕部(骑在托盘上随车走),
   卸货任务(从托盘取盒)绝不能再挪它。故自动触发默认只匹配 grasp 任务的目标 (trigger_target=p2)。

触发两种(都留):
  自动: 参数 trigger_target(默认 "p2"), 订 /lane_navigator/status, 收到
        "<trigger_target>:SUCCEEDED" 时按实时 base_link 摆一次。
  手动: 服务 /sim/place_box_reachable (std_srvs/Trigger), 同样按实时 base_link 摆
        (供"车不动单测抓取")。

真机不用本节点(真机货物真实摆放、真感知识别); 纯仿真施工辅助。
"""
import math

import numpy as np

import rclpy
from gazebo_msgs.msg import EntityState, LinkStates
from gazebo_msgs.srv import SetEntityState
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


def q_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def q_conjugate(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


def q_rotate(q, v):
    qv = np.array([v[0], v[1], v[2], 0.0])
    r = q_multiply(q_multiply(q, qv), q_conjugate(q))
    return r[:3]


class PlaceBoxHelper(Node):
    def __init__(self):
        super().__init__('place_box_helper')
        self.declare_parameter('base_link_name', 'mm_robot::base_link')
        self.declare_parameter('box_model_name', 'grasp_box')
        # 盒子在 base_link 系的落点标定值: (0.15,-0.28) 已实测整段 pick 一次通过
        # (粗定位 exy=0 → 精修 5s 收敛 → 直插 → 吸取). 侧向 0.28m 为舒适半径
        # (早前 0.38m 贴最大伸展奇异点 ABORT); 前向 0.15m 落肩前工作区 (落肩后 Joint_12 顶后限位).
        # 因按到位后实时 base_link 摆盒, 无需再预扣导航欠冲, 直接用标定值.
        self.declare_parameter('offset_x', 0.15)
        self.declare_parameter('offset_y', -0.28)
        # 盒子落地世界高度 (半盒高 0.0125, 与 world 原始 grasp_box z 一致)
        self.declare_parameter('box_rest_z', 0.0125)
        # 自动触发的目标车道节点名 (匹配 /lane_navigator/status; 空 = 只用手动服务)
        self.declare_parameter('trigger_target', 'p2')

        self.base_name = self.get_parameter('base_link_name').value
        self.box_model = self.get_parameter('box_model_name').value
        self.off = np.array([
            float(self.get_parameter('offset_x').value),
            float(self.get_parameter('offset_y').value),
            0.0,
        ])
        self.rest_z = float(self.get_parameter('box_rest_z').value)
        self.trigger_target = self.get_parameter('trigger_target').value

        self._base = None            # (pos np3, quat np4) base_link world 真值
        self._last_handled = None    # 已处理过的目标 (防重复触发)

        self.create_subscription(LinkStates, '/gazebo/link_states', self.on_links, 10)
        self.create_subscription(String, '/lane_navigator/status', self.on_nav_status, 10)
        self.cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.srv = self.create_service(Trigger, '/sim/place_box_reachable', self.on_service)

        self.get_logger().info(
            f'place_box_helper 就绪: 盒子={self.box_model} → 到位后车右侧 '
            f'({self.off[0]:.2f},{self.off[1]:.2f}) 世界z={self.rest_z:.4f}; '
            f'自动触发 /lane_navigator/status=="{self.trigger_target or "(禁用)"}:SUCCEEDED", '
            f'手动服务 /sim/place_box_reachable')

    def on_links(self, msg: LinkStates):
        if self.base_name in msg.name:
            i = msg.name.index(self.base_name)
            p = msg.pose[i].position
            o = msg.pose[i].orientation
            self._base = (np.array([p.x, p.y, p.z]),
                          np.array([o.x, o.y, o.z, o.w]))

    def on_nav_status(self, msg: String):
        if not self.trigger_target:
            return
        want = f'{self.trigger_target}:SUCCEEDED'
        if msg.data != want or msg.data == self._last_handled:
            return
        if self._base is None:
            self.get_logger().error(
                f'收到 "{msg.data}" 但无 base_link 真值 (/gazebo/link_states 未到?), 不摆盒')
            return
        self._last_handled = msg.data
        self.get_logger().info(f'收到 "{msg.data}" → 按到位实时 base_link 摆盒')
        p_base, q_base = self._base
        world = p_base + q_rotate(q_base, self.off)
        ok, why = self.set_box(world, q_base)
        if not ok:
            self.get_logger().error(f'自动摆盒失败: {why}')

    def on_service(self, req, res):
        # 手动: 按实时 base_link 摆 (车不动单测抓取用)
        if self._base is None:
            res.success, res.message = False, '无 base_link 真值 (/gazebo/link_states 未到?)'
            return res
        p_base, q_base = self._base
        world = p_base + q_rotate(q_base, self.off)
        res.success, res.message = self.set_box(world, q_base)
        return res

    def set_box(self, world, quat):
        if not self.cli.service_is_ready():
            return False, '/gazebo/set_entity_state 未就绪'
        st = EntityState()
        st.name = self.box_model
        st.reference_frame = ''  # world 系
        st.pose.position.x = float(world[0])
        st.pose.position.y = float(world[1])
        st.pose.position.z = float(self.rest_z)  # 固定落地高度 (不随底盘 z 抖动)
        st.pose.orientation.x = float(quat[0])
        st.pose.orientation.y = float(quat[1])
        st.pose.orientation.z = float(quat[2])
        st.pose.orientation.w = float(quat[3])
        req = SetEntityState.Request()
        req.state = st
        self.cli.call_async(req)
        msg = (f'盒子已摆到 world ({world[0]:.3f},{world[1]:.3f},{self.rest_z:.4f}) '
               f'= 车右侧 {-self.off[1]:.2f}m')
        self.get_logger().info(msg)
        return True, msg


def main():
    rclpy.init()
    node = PlaceBoxHelper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
