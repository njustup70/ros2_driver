#!/usr/bin/env python3
import rclpy
import struct
from std_msgs.msg import Float32MultiArray
from rclpy.node import Node
from std_msgs.msg import String
import time
import sys
import numpy as np
sys.path.append('/home/Elaina/ros2_driver/src') 
from protocol_lib.myserial import AsyncSerial_t


class SerialReceiver(Node):
    def __init__(self):
        super().__init__('serial_receiver')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('serial_baudrate', 115200)
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('serial_baudrate').value
        self.serial = AsyncSerial_t(port, baud)

        self.serial.startListening(self.data_callback)

        self.publisher = self.create_publisher(Float32MultiArray, 'serial_data', 10)

        self.get_logger().info(f"串口监听已启动: {port} @ {baud}bps")

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
            # 按C++ float(4字节小端)解析三组数据
            x, y, yaw = struct.unpack('<fff', packet[1:13])

            # 发布为 ROS2 Float32MultiArray
            msg = Float32MultiArray()
            msg.data = [x, y, yaw]
            self.publisher.publish(msg)

            self.get_logger().info(f"收到位姿: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")

        except struct.error:
            self.get_logger().warn("数据解析失败，丢弃帧")
            return

def main(args=None):
    rclpy.init(args=args)
    node = SerialReceiver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("exit by Keyboard Interrupt")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()