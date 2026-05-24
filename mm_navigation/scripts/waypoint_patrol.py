#!/usr/bin/env python3
"""
仓储四区巡逻节点 — 逐点导航循环巡逻
"""
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose, Spin
from geometry_msgs.msg import PoseStamped, Twist
from tf2_ros import Buffer, TransformException, TransformListener


class WaypointPatrol(Node):
    def __init__(self):
        super().__init__('waypoint_patrol')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._spin_client = ActionClient(self, Spin, 'spin')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.waypoints = [
            self.make_pose(-1.6, 1.8, 0.0),
            self.make_pose(-1.6, -0.5, 0.0),
            self.make_pose(-5.8, -0.5, 3.14),
            self.make_pose(-5.8, 1.8, 0.0),
        ]
        self.current_idx = 0
        self.active = False
        self.retry_timer = None
        self.navigation_goal_handle = None
        self.navigation_check_timer = None
        self.canceling_navigation = False
        self.goal_seq = 0
        self.active_goal_seq = None
        self.active_waypoint_idx = None
        self.final_translate_timer = None
        self.final_translate_waypoint_idx = None
        self.position_reached_tolerance = 0.08
        self.navigation_takeover_tolerance = 0.18
        self.final_translate_speed = 0.16
        self.aligning_after_navigation = False
        self.get_logger().info('Waiting for navigate_to_pose action server...')
        self._client.wait_for_server()
        self.get_logger().info('Waiting for spin action server...')
        self._spin_client.wait_for_server()
        self.get_logger().info('Connected. Starting patrol.')
        self.send_next()

    def make_pose(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.orientation.w = 1.0
        if yaw != 0.0:
            p.pose.orientation.z = math.sin(yaw / 2.0)
            p.pose.orientation.w = math.cos(yaw / 2.0)
        return p

    def send_next(self):
        if self.active:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )
        except TransformException as ex:
            self.get_logger().warn(f'Waiting for robot pose before waypoint {self.current_idx + 1}: {ex}')
            self.schedule_retry()
            return

        goal_pose = self.waypoints[self.current_idx]
        target_yaw = self.yaw_from_quaternion(goal_pose.pose.orientation)
        current_yaw = self.yaw_from_quaternion(transform.transform.rotation)
        spin_angle = self.normalize_angle(target_yaw - current_yaw)

        if abs(spin_angle) < 0.08:
            if self.aligning_after_navigation:
                self.finish_waypoint()
            else:
                self.send_navigation_goal()
            return

        goal = Spin.Goal()
        goal.target_yaw = spin_angle
        goal.time_allowance = Duration(seconds=30.0).to_msg()
        self.active = True
        self.get_logger().info(
            f'{"Final aligning" if self.aligning_after_navigation else "Spinning"} '
            f'{spin_angle:.2f} rad before waypoint {self.current_idx + 1}/{len(self.waypoints)}.'
        )
        self._send_spin_future = self._spin_client.send_goal_async(goal)
        self._send_spin_future.add_done_callback(self.spin_response_callback)

    def schedule_retry(self):
        if self.retry_timer is not None:
            return

        def retry_once():
            self.retry_timer.cancel()
            self.retry_timer = None
            self.send_next()

        self.retry_timer = self.create_timer(0.5, retry_once)

    def send_navigation_goal(self):
        goal_seq = self.goal_seq + 1
        waypoint_idx = self.current_idx
        self.goal_seq = goal_seq
        self.active_goal_seq = goal_seq
        self.active_waypoint_idx = waypoint_idx
        self.canceling_navigation = False

        goal = NavigateToPose.Goal()
        goal.pose = self.waypoints[waypoint_idx]
        self.active = True
        self.get_logger().info(
            f'Dispatching waypoint {waypoint_idx + 1}/{len(self.waypoints)}: '
            f'x={goal.pose.pose.position.x:.2f}, y={goal.pose.pose.position.y:.2f}'
        )
        self._send_goal_future = self._client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(
            lambda future: self.goal_response_callback(future, goal_seq, waypoint_idx)
        )

    def clear_navigation_state(self):
        if self.navigation_check_timer is not None:
            self.navigation_check_timer.cancel()
            self.navigation_check_timer = None
        self.navigation_goal_handle = None
        self.canceling_navigation = False
        self.active_goal_seq = None
        self.active_waypoint_idx = None

    def is_active_navigation(self, goal_seq, waypoint_idx):
        return (
            self.active_goal_seq == goal_seq
            and self.active_waypoint_idx == waypoint_idx
            and self.current_idx == waypoint_idx
        )

    def spin_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Spin goal rejected, retrying.')
            self.active = False
            self.schedule_retry()
            return
        self._spin_result_future = goal_handle.get_result_async()
        self._spin_result_future.add_done_callback(self.spin_result_callback)

    def spin_result_callback(self, future):
        result = future.result()
        self.active = False
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f'Spin failed with status {result.status}, retrying.')
            self.schedule_retry()
            return
        if self.aligning_after_navigation:
            self.finish_waypoint()
        else:
            self.send_navigation_goal()

    def goal_response_callback(self, future, goal_seq, waypoint_idx):
        if not self.is_active_navigation(goal_seq, waypoint_idx):
            return

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'Waypoint {waypoint_idx + 1} goal rejected')
            self.active = False
            self.clear_navigation_state()
            self.schedule_retry()
            return
        self.get_logger().info('Goal accepted, navigating...')
        self.navigation_goal_handle = goal_handle
        self.start_navigation_check(goal_seq, waypoint_idx)
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(
            lambda result_future: self.result_callback(result_future, goal_seq, waypoint_idx)
        )

    def start_navigation_check(self, goal_seq, waypoint_idx):
        if self.navigation_check_timer is not None:
            self.navigation_check_timer.cancel()

        self.navigation_check_timer = self.create_timer(
            0.05,
            lambda: self.check_navigation_progress(goal_seq, waypoint_idx),
        )

    def check_navigation_progress(self, goal_seq, waypoint_idx):
        if not self.is_active_navigation(goal_seq, waypoint_idx):
            return
        if self.canceling_navigation or self.navigation_goal_handle is None:
            return
        if not self.is_waypoint_near(waypoint_idx, self.navigation_takeover_tolerance):
            return

        self.canceling_navigation = True
        self.get_logger().info(f'Waypoint {waypoint_idx + 1} position reached, taking over final yaw.')
        cancel_future = self.navigation_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(
            lambda future: self.navigation_cancel_callback(future, goal_seq, waypoint_idx)
        )

    def navigation_cancel_callback(self, future, goal_seq, waypoint_idx):
        if not self.is_active_navigation(goal_seq, waypoint_idx):
            return

        cancel_response = future.result()
        if len(cancel_response.goals_canceling) == 0:
            self.canceling_navigation = False
            return

        self.active = False
        self.clear_navigation_state()
        self.start_final_translate(waypoint_idx)

    def start_final_translate(self, waypoint_idx):
        if self.final_translate_timer is not None:
            self.final_translate_timer.cancel()

        self.active = True
        self.final_translate_waypoint_idx = waypoint_idx
        self.get_logger().info(f'Fine translating to waypoint {waypoint_idx + 1}.')
        self.final_translate_timer = self.create_timer(0.05, self.final_translate_step)

    def final_translate_step(self):
        waypoint_idx = self.final_translate_waypoint_idx
        if waypoint_idx != self.current_idx:
            self.stop_final_translate()
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException:
            self.publish_stop()
            return

        goal_pose = self.waypoints[waypoint_idx]
        dx = goal_pose.pose.position.x - transform.transform.translation.x
        dy = goal_pose.pose.position.y - transform.transform.translation.y
        distance = math.hypot(dx, dy)
        if distance <= self.position_reached_tolerance:
            self.stop_final_translate()
            self.aligning_after_navigation = True
            self.send_next()
            return

        speed = min(self.final_translate_speed, max(0.03, distance * 0.8))
        vx_world = speed * dx / distance
        vy_world = speed * dy / distance
        current_yaw = self.yaw_from_quaternion(transform.transform.rotation)
        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)

        cmd = Twist()
        cmd.linear.x = cos_yaw * vx_world + sin_yaw * vy_world
        cmd.linear.y = -sin_yaw * vx_world + cos_yaw * vy_world
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def stop_final_translate(self):
        if self.final_translate_timer is not None:
            self.final_translate_timer.cancel()
            self.final_translate_timer = None
        self.final_translate_waypoint_idx = None
        self.active = False
        self.publish_stop()

    def publish_stop(self):
        self.cmd_vel_pub.publish(Twist())

    def result_callback(self, future, goal_seq, waypoint_idx):
        if not self.is_active_navigation(goal_seq, waypoint_idx):
            return

        result = future.result()
        self.active = False
        self.clear_navigation_state()
        if result.status == GoalStatus.STATUS_SUCCEEDED or self.is_waypoint_near(waypoint_idx, self.navigation_takeover_tolerance):
            self.get_logger().info(f'Waypoint {waypoint_idx + 1} position reached, taking over final translation.')
            self.start_final_translate(waypoint_idx)
            return

        self.get_logger().warn(f'Waypoint {waypoint_idx + 1} failed with status {result.status}. Retrying.')
        self.schedule_retry()

    def is_current_waypoint_near(self):
        return self.is_waypoint_near(self.current_idx, self.position_reached_tolerance)

    def is_waypoint_near(self, waypoint_idx, tolerance):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException:
            return False

        goal_pose = self.waypoints[waypoint_idx]
        dx = goal_pose.pose.position.x - transform.transform.translation.x
        dy = goal_pose.pose.position.y - transform.transform.translation.y
        return math.hypot(dx, dy) <= tolerance

    def finish_waypoint(self):
        self.get_logger().info(f'Waypoint {self.current_idx + 1} reached.')
        self.aligning_after_navigation = False
        self.current_idx = (self.current_idx + 1) % len(self.waypoints)
        self.send_next()

    @staticmethod
    def yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPatrol()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
