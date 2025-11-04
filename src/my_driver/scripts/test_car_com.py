#!/usr/bin/env python3
'''
串口改成队列发送
'''
import rclpy,sys
from rclpy.node import Node
from geometry_msgs.msg import Twist
sys.path.append('/home/Elaina/ros2_driver/src') 
# print(sys.path)
from protocol_lib.myserial import AsyncSerial_t
from protocol_lib.get_log_shootdata_local import DataSimulator
from rclpy.time import Time
import struct


class Communicate_t(Node):
    def __init__(self):
        super().__init__('communicate_t')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('serial_port', '/dev/serial_qh')
        self.declare_parameter('serial_baudrate', 230400)
        self.sub=self.create_subscription(Twist,
        self.get_parameter('cmd_vel_topic').value,
        self.cmd_topic_callback,10)
        self.serial=AsyncSerial_t(self.get_parameter("serial_port"),self.get_parameter("serial_baudrate"))
        # 初始化串口通信
    def cmd_topic_callback(self,velCmd:Twist):
        """cmd_vel话题回调函数"""
        linear_x = velCmd.linear.x  # 前进/后退速度，单位：米/秒
        angular_z = velCmd.angular.z  # 旋转速度，单位：弧度/秒
        linear_y=velCmd.linear.y
        dataBytes=b'\xFF'+struct.pack('<fff',linear_x,linear_y,angular_z)+b'\xFE'
        self.serial.write(dataBytes)
def main(args=None):
    rclpy.init(args=args)
    node=Communicate_t()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
