#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelSmoother(Node):
    def __init__(self):
        super().__init__('cmd_vel_smoother')
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel_smoothed')
        self.declare_parameter('rate', 30.0)
        self.declare_parameter('linear_acceleration', 0.8)
        self.declare_parameter('angular_acceleration', 1.8)
        self.declare_parameter('command_timeout', 0.3)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.rate = float(self.get_parameter('rate').value)
        self.linear_acceleration = float(self.get_parameter('linear_acceleration').value)
        self.angular_acceleration = float(self.get_parameter('angular_acceleration').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)

        self.target = Twist()
        self.current = Twist()
        self.last_command_time = self.get_clock().now()
        self.publisher = self.create_publisher(Twist, output_topic, 10)
        self.subscription = self.create_subscription(Twist, input_topic, self.command_callback, 10)
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

    def command_callback(self, msg):
        self.target = msg
        self.last_command_time = self.get_clock().now()

    def timer_callback(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_command_time).nanoseconds / 1e9
        target = self.target if elapsed <= self.command_timeout else Twist()
        dt = 1.0 / self.rate

        self.current.linear.x = self.step(self.current.linear.x, target.linear.x, self.linear_acceleration * dt)
        self.current.linear.y = self.step(self.current.linear.y, target.linear.y, self.linear_acceleration * dt)
        self.current.angular.z = self.step(self.current.angular.z, target.angular.z, self.angular_acceleration * dt)
        self.publisher.publish(self.current)

    @staticmethod
    def step(current, target, max_delta):
        delta = target - current
        if math.fabs(delta) <= max_delta:
            return target
        return current + math.copysign(max_delta, delta)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSmoother()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
