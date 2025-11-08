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
from geometry_msgs.msg import Vector3Stamped
import struct


class Communicate_t(Node):
    def __init__(self):
        super().__init__('communicate_t')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('serial_baudrate', 115200)
        self.sub=self.create_subscription(Twist,
        self.get_parameter('cmd_vel_topic').value,
        self.cmd_topic_callback,10)
        self.publisher = self.create_publisher(Vector3Stamped, 'odom', 10)
        self.serial=AsyncSerial_t(self.get_parameter("serial_port").value,self.get_parameter("serial_baudrate").value)
        self.serial.startListening(self.data_callback)
        # 初始化串口通信
    def cmd_topic_callback(self,velCmd:Twist):
        """cmd_vel话题回调函数"""
        linear_x = velCmd.linear.x  # 前进/后退速度，单位：米/秒
        angular_z = velCmd.angular.z  # 旋转速度，单位：弧度/秒
        linear_y=velCmd.linear.y
        dataBytes=b'\xFF'+struct.pack('<fff',linear_x,linear_y,angular_z)+b'\xFE'
        self.serial.write(dataBytes)
        
    def data_callback(self, data: bytes):
        """
        串口数据回调函数 — 当收到下位机数据时触发
        """
        if not data or len(data) < 13:
            return

        # 找帧头
        header_index = data.find(b'\xFA')
        if header_index == -1 or len(data) - header_index < 13:
            self.get_logger().warn("未找到有效帧头或数据不完整")
            return

        # 取出完整帧
        packet = data[header_index:header_index + 13]

        # 校验帧头
        if packet[0] != 0xFA:
            self.get_logger().warn("帧头错误")
            return

        try:
            x, y, yaw = struct.unpack('<fff', packet[1:13])
            # 构造 Vector3Stamped 消息
            msg = Vector3Stamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'  # 可根据需求改为 base_link / map 等
            msg.vector.x = x
            msg.vector.y = y
            msg.vector.z = yaw  # 将 yaw 存入 z 分量
            self.publisher.publish(msg)
            self.get_logger().info(f"收到位姿: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")

        except struct.error as e:
            self.get_logger().error(f"数据解析错误: {e}")
            return
        
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
