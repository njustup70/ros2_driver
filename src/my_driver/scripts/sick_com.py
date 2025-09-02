#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import String
import json, time, math, struct, numpy as np

# 串口库路径
import sys
sys.path.append('/home/Elaina/ros2_driver/src')
from protocol_lib.myserial import AsyncSerial_t


class tf:
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw


class SickHandler:
    """
    串口数据处理器
    可被外部 import 并由外部串口驱动调用 handle_data(data)
    """
    def __init__(self, node: Node):
        self.node = node
        self.vel_pub = self.node.create_publisher(Vector3Stamped, '/sick/vel', 10)
        self.local_pub = self.node.create_publisher(Vector3Stamped, '/odom', 10)
        self.sick_pub = self.node.create_publisher(String, '/sick/lidar', 10)
        self.tf_last = tf()
        self.last_timestamp = time.time()

    def validate_data(self, data: bytes) -> bool:
        databag = data.rstrip(b'\x00')
        if len(databag) < 2:
            return False
        data_valid = databag[1:-1]
        checksum = sum(data_valid) & 0xFF
        return checksum == databag[-1] and checksum == databag[0]

    def handle_data(self, data: bytes):
        timestamp = time.time()
        if not self.validate_data(data):
            return

        if data[1] == 0x30:
            x = struct.unpack('<f', data[2:6])[0]
            y = struct.unpack('<f', data[6:10])[0]
            yaw = struct.unpack('<f', data[10:14])[0]
            tf_data = tf(x, y, yaw)

            twist_msg = Vector3Stamped()
            local_msg = Vector3Stamped()

            if self.tf_last.x != 0.0 or self.tf_last.y != 0.0:
                dt = timestamp - self.last_timestamp
                twist_msg.vector.x = (tf_data.x - self.tf_last.x) / dt
                twist_msg.vector.y = -(tf_data.y - self.tf_last.y) / dt

                dyaw = tf_data.yaw - self.tf_last.yaw
                if dyaw > math.pi:
                    dyaw -= 2 * math.pi
                elif dyaw < -math.pi:
                    dyaw += 2 * math.pi
                twist_msg.vector.z = dyaw / dt

            local_msg.vector.x = tf_data.x
            local_msg.vector.y = tf_data.y
            local_msg.vector.z = tf_data.yaw

            now = self.node.get_clock().now().to_msg()
            local_msg.header.stamp = now
            twist_msg.header.stamp = now

            self.local_pub.publish(local_msg)
            self.vel_pub.publish(twist_msg)

            self.tf_last = tf_data
            self.last_timestamp = timestamp

        elif data[1] == 0x31:
            laser_data = []
            bias = [0, 0, 0, 0.188, 0.313, 0.313, 0.293, 0.293]
            for i in range(2, 18, 2):
                value: np.int16 = struct.unpack('>h', data[i:i + 2])[0]
                assert -32768 <= value <= 32767
                real_value = (value * 0.2167 + 63.1689) / 1000.0
                real_value = round(real_value, 3) + bias[i // 2 - 1]
                laser_data.append(real_value)

            laser_data_json = json.dumps(laser_data)
            laser_msg = String(data=laser_data_json)
            self.sick_pub.publish(laser_msg)


class SickCommunicateNode(Node):
    """
    串口节点封装
    直接运行时创建 AsyncSerial 并启动监听
    """
    def __init__(self):
        super().__init__('sick_communicate')
        self.declare_parameter('serial_port', '/dev/serial_sick')
        self.declare_parameter('serial_baudrate', 460800)

        # 串口只有在直接运行时创建
        self.serial = AsyncSerial_t(
            self.get_parameter('serial_port').value,
            self.get_parameter('serial_baudrate').value
        )
        self.handler = SickHandler(self)
        self.serial.startListening(self.handler.handle_data)


def main(args=None):
    rclpy.init(args=args)
    node = SickCommunicateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
