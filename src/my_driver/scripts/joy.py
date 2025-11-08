#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math,os
import termios
import tty
import sys
import json
class JoyTeleopNode(Node):
    def __init__(self):
        super().__init__('joy_teleop_node')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('linear_scale', 0.1)
        self.declare_parameter('angular_scale',math.pi/3.0)
        self.pub_cmd_vel = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.sub_joy = self.create_subscription(Joy, self.get_parameter('joy_topic').value, self.joy_callback, 10)
        self.state_pub=self.create_publisher(String,"robot_state",10)
        self.timer= self.create_timer(0.02, self.timerpublish)  # 定时器用于定期检查Joy消息
        self.emergency_stop = False

        # 速度缩放参数
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value

        self.get_logger().info("Joystick Teleop Node started.")
        self.last_twist= Twist()
        self.twist= Twist()
        self.a_button_pressed = False  # 用于跟踪A键状态
        self.keyboard_a_pressed = False  # 用于跟踪键盘A键状态
        self.is_interactive = os.isatty(sys.stdin.fileno())
    def joy_callback(self, msg: Joy):
        # B键一般是按钮1（不同手柄可能不同，需根据实际情况调整）
        b_button = msg.buttons[1]
        self.a_button_pressed = msg.buttons[0]  # A键一般是按钮0
        if b_button:
            self.emergency_stop = True
            self.get_logger().warn("Emergency Stop Activated!")
            self.twist = Twist()
            self.twist.linear.x = 0.0
            self.twist.linear.y = 0.0
            self.twist.angular.z = 0.0
            # self.pub_cmd_vel.publish(twist)
        else:
            self.emergency_stop = False
        self.twist = Twist()

        if not self.emergency_stop:
            # 左摇杆：axes[0] 为左右（y轴），axes[1] 为前后（x轴）
            self.twist.linear.x = msg.axes[1] * self.linear_scale 
            self.twist.linear.y = msg.axes[0] * self.linear_scale
            # 右摇杆：axes[3] 为左右（控制yaw）
            self.twist.angular.z = msg.axes[3] * self.angular_scale
            if(abs(self.twist.linear.x) <0.05):
                self.twist.linear.x = 0.0
            if(abs(self.twist.linear.y) <0.05):
                self.twist.linear.y = 0.0
            if(abs(self.twist.angular.z) < 0.05):
                self.twist.angular.z = 0.0
        else:
            self.twist.linear.x = 0.0
            self.twist.linear.y = 0.0
            self.twist.angular.z = 0.0
        #比较是否不变,不变就不发步
            # 发布速度命令
            # self.get_logger().info(f"Publishing Twist: {twist}")
            # self.pub_cmd_vel.publish(twist)
        # self.last_twist = self.twist
    def timerpublish(self):
        # 定时发布速度命令，保持通信
        if not self.emergency_stop:
            # self.last_twist= self.twist
            #如果一样就不发
            if (self.twist.linear.x == self.last_twist.linear.x and
                self.twist.linear.y == self.last_twist.linear.y and
                self.twist.angular.z == self.last_twist.angular.z):
                return
            if self.twist.linear.x != 0.0 or self.twist.linear.y != 0.0 or self.twist.angular.z != 0.0:
                self.pub_cmd_vel.publish(self.twist)   
            self.last_twist  = self.twist       
        else:
            # 如果处于紧急停止状态，发布零速度命令
            self.pub_cmd_vel.publish(self.twist)
        #看键盘是否按下A键
        if  self.is_interactive:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
            if key == 'a' or key == 'A':
                self.keyboard_a_pressed = True
            else:
                self.keyboard_a_pressed = False
        else:
            self.keyboard_a_pressed = False
        # else:
            # self.keyboard_a_pressed = False
        if self.keyboard_a_pressed or self.a_button_pressed:
            json_msg= {
                "nav_state":"IDLE"
            }
            json_str = json.dumps(json_msg)
            self.state_pub.publish(String(data=json_str))
def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Joy Teleop Node stopped by user.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
