#!/usr/bin/env python3
"""独立发布车道图 RViz markers, 不需要跑完整导航栈.

用法:
    source /opt/ros/humble/setup.bash
    source install/setup.bash   # (如果用 colcon build 过)
    python3 src/docs/scripts/lane_graph_viz.py
然后在 RViz 里加载 src/mm_navigation/config/rviz/nav_real.rviz (或确保已有 LaneGraph display).
"""
import math
import os
import sys

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
GRAPH = os.path.join(ROOT, 'mm_navigation/config/lane_graph.yaml')


def load_graph():
    data = yaml.safe_load(open(GRAPH))
    nodes = {k: (float(v['x']), float(v['y']), float(v.get('yaw', 0.0)))
             for k, v in data['nodes'].items()}
    edges = [tuple(e) for e in data['edges']]
    frame = data.get('frame_id', 'map')
    return nodes, edges, frame


def make_markers(nodes, edges, frame):
    ma = MarkerArray()
    now_stamp = None  # filled at publish time

    # 边: 灰色线段
    edge_m = Marker()
    edge_m.header.frame_id = frame
    edge_m.ns = 'edges'
    edge_m.id = 0
    edge_m.type = Marker.LINE_LIST
    edge_m.action = Marker.ADD
    edge_m.scale.x = 0.03
    edge_m.color.r = 0.6
    edge_m.color.g = 0.6
    edge_m.color.b = 0.6
    edge_m.color.a = 1.0
    for a, b in edges:
        for name in (a, b):
            p = Point()
            p.x, p.y, p.z = nodes[name][0], nodes[name][1], 0.05
            edge_m.points.append(p)
    ma.markers.append(edge_m)

    for idx, (name, (x, y, yaw)) in enumerate(nodes.items()):
        # 节点球: 绿色
        sph = Marker()
        sph.header.frame_id = frame
        sph.ns = 'nodes'
        sph.id = idx
        sph.type = Marker.SPHERE
        sph.action = Marker.ADD
        sph.pose.position.x = x
        sph.pose.position.y = y
        sph.pose.position.z = 0.05
        sph.scale.x = sph.scale.y = sph.scale.z = 0.12
        sph.color.r = 0.1
        sph.color.g = 0.9
        sph.color.b = 0.3
        sph.color.a = 1.0
        ma.markers.append(sph)

        # 节点名
        txt = Marker()
        txt.header.frame_id = frame
        txt.ns = 'labels'
        txt.id = idx
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.pose.position.x = x
        txt.pose.position.y = y + 0.12
        txt.pose.position.z = 0.25
        txt.scale.z = 0.15
        txt.color.r = txt.color.g = txt.color.b = 1.0
        txt.color.a = 1.0
        txt.text = name
        ma.markers.append(txt)

        # 目标朝向箭头: 黄色
        arw = Marker()
        arw.header.frame_id = frame
        arw.ns = 'yaws'
        arw.id = idx
        arw.type = Marker.ARROW
        arw.action = Marker.ADD
        arw.scale.x = 0.02
        arw.scale.y = 0.05
        arw.color.r = 1.0
        arw.color.g = 0.9
        arw.color.b = 0.0
        arw.color.a = 1.0
        tip = Point()
        tip.x = x + 0.25 * math.cos(yaw)
        tip.y = y + 0.25 * math.sin(yaw)
        tip.z = 0.05
        base = Point()
        base.x, base.y, base.z = x, y, 0.05
        arw.points = [base, tip]
        ma.markers.append(arw)

    return ma


class LaneVizNode(Node):
    def __init__(self):
        super().__init__('lane_graph_viz')
        latched = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, 'lane_graph_markers', latched)
        nodes, edges, frame = load_graph()
        ma = make_markers(nodes, edges, frame)
        stamp = self.get_clock().now().to_msg()
        for m in ma.markers:
            m.header.stamp = stamp
        self.pub.publish(ma)
        n_nodes = len(nodes)
        n_edges = len(edges)
        self.get_logger().info(
            f'Published lane graph: {n_nodes} nodes, {n_edges} edges -> /lane_graph_markers')
        self.get_logger().info('Keep this node alive so RViz can subscribe (Ctrl+C to exit).')


def main():
    rclpy.init()
    node = LaneVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
