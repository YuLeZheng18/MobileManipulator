#!/usr/bin/env python3
"""
自定义键盘控制节点
按键布局：
- w: 前进
- s: 后退
- a: 左移
- d: 右移
- q: 左斜前方
- e: 右斜前方
- z: 左斜后方
- c: 右斜后方
- 左箭头: 左旋转
- 右箭头: 右旋转
- 上箭头: 加速（线速度）
- 下箭头: 减速（线速度）

所有按键都是按住移动，松开停止
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

class CustomKeyboardControl(Node):
    def __init__(self):
        super().__init__('custom_keyboard_control')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 速度参数
        self.linear_speed = 0.5  # 线速度 (m/s)
        self.angular_speed = 1.0  # 角速度 (rad/s)
        self.speed_step = 0.1  # 速度调整步长
        self.max_speed = 2.0
        self.min_speed = 0.1
        
        self.get_logger().info('自定义键盘控制节点已启动')
        self.get_logger().info(f'当前线速度: {self.linear_speed} m/s')
        self.get_logger().info(f'当前角速度: {self.angular_speed} rad/s')
        self.print_help()
        
        # 初始化终端设置
        self.old_settings = termios.tcgetattr(sys.stdin)
        
    def print_help(self):
        print('\n========== 键盘控制 =========')
        print('  q   w   e    - 左斜前 | 前进 | 右斜前')
        print('  a       d    - 左移   |        | 右移')
        print('  z   s   c    - 左斜后 | 后退 | 右斜后')
        print('  ←  左旋转    →  右旋转')
        print('  ↑  加速       ↓  减速')
        print('  空格/其他键  - 停止')
        print('  CTRL-C       - 退出')
        print('================================\n')
    
    def get_key(self):
        """非阻塞读取键盘输入"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = None
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        return key
    
    def run(self):
        """主循环"""
        twist = Twist()
        
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key is None:
                    # 没有按键输入，发送停止命令
                    twist = Twist()
                    self.publisher_.publish(twist)
                    continue
                
                # 方向键处理（ANSI转义序列）
                if key == '\x1b':
                    # 读取完整的转义序列
                    tty.setraw(sys.stdin.fileno())
                    key2 = sys.stdin.read(2)
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
                    
                    if key2 == '[A':  # 上箭头 - 加速
                        self.linear_speed = min(self.linear_speed + self.speed_step, self.max_speed)
                        self.get_logger().info(f'线速度: {self.linear_speed:.2f} m/s')
                        continue
                    elif key2 == '[B':  # 下箭头 - 减速
                        self.linear_speed = max(self.linear_speed - self.speed_step, self.min_speed)
                        self.get_logger().info(f'线速度: {self.linear_speed:.2f} m/s')
                        continue
                    elif key2 == '[D':  # 左箭头 - 左旋转
                        twist.linear.x = 0.0
                        twist.linear.y = 0.0
                        twist.angular.z = self.angular_speed
                    elif key2 == '[C':  # 右箭头 - 右旋转
                        twist.linear.x = 0.0
                        twist.linear.y = 0.0
                        twist.angular.z = -self.angular_speed
                    else:
                        twist = Twist()
                elif key == 'w':  # 前进
                    twist.linear.x = self.linear_speed
                    twist.linear.y = 0.0
                    twist.angular.z = 0.0
                elif key == 's':  # 后退
                    twist.linear.x = -self.linear_speed
                    twist.linear.y = 0.0
                    twist.angular.z = 0.0
                elif key == 'a':  # 左移
                    twist.linear.x = 0.0
                    twist.linear.y = self.linear_speed
                    twist.angular.z = 0.0
                elif key == 'd':  # 右移
                    twist.linear.x = 0.0
                    twist.linear.y = -self.linear_speed
                    twist.angular.z = 0.0
                elif key == 'q':  # 左斜前方
                    twist.linear.x = self.linear_speed
                    twist.linear.y = self.linear_speed
                    twist.angular.z = 0.0
                elif key == 'e':  # 右斜前方
                    twist.linear.x = self.linear_speed
                    twist.linear.y = -self.linear_speed
                    twist.angular.z = 0.0
                elif key == 'z':  # 左斜后方
                    twist.linear.x = -self.linear_speed
                    twist.linear.y = self.linear_speed
                    twist.angular.z = 0.0
                elif key == 'c':  # 右斜后方
                    twist.linear.x = -self.linear_speed
                    twist.linear.y = -self.linear_speed
                    twist.angular.z = 0.0
                else:  # 其他键 - 停止
                    twist = Twist()
                
                self.publisher_.publish(twist)
                rclpy.spin_once(self, timeout_sec=0.01)
                
        except KeyboardInterrupt:
            self.get_logger().info('收到中断信号，停止机器人')
            twist = Twist()
            self.publisher_.publish(twist)
        finally:
            # 恢复终端设置
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CustomKeyboardControl()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
