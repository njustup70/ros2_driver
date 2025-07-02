#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, json, os,time,math,struct
import numpy as np
sys.path.append('/home/Elaina/ros2_driver/src') 
from protocol_lib.myserial import AsyncSerial_t
from std_msgs.msg import String
from geometry_msgs.msg import Vector3Stamped
class tf:
    def __init__(self, x=0.0, y=0.0,yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
class SickCommunicate_t(Node):
    def __init__(self):
        super().__init__('sick_communicate')
        self.declare_parameter('serial_port', '/dev/serial_sick')
        self.declare_parameter('serial_baudrate', 460800)
        self.serial = AsyncSerial_t(
            self.get_parameter('serial_port').value,
            self.get_parameter('serial_baudrate').value
        )
        self.vel_pub= self.create_publisher(Vector3Stamped, '/sick/vel', 10)  # 发布速度话题
        self.local_pub=self.create_publisher(Vector3Stamped, '/sick/local', 10)  # 发布本地坐标话题
        self.sick_pub=self.create_publisher(String, '/sick/lidar', 10)  # 发布激光数据话题
        # 启动串口监听
        self.serial.startListening(self.data_callback)

        self.tf_last = tf()  # 初始化tf数据
        self.last_timestamp = time.time()
    def ValidationData(self,data:bytes)->bool:
        '''
        数据包校验函数
        '''
        databag = data.rstrip(b'\x00')
        if len(data) < 2:
            return False
        data_valid=databag[1:-1]
        checksum = 0
        for byte in data_valid:
            checksum += byte
        checksum &= 0xFF  # 保留低8位
        assert checksum < 256, "Checksum must be less than 256"
        return checksum == databag[-1] and checksum==databag[0]
        
    def data_callback(self, data: bytes):
        # 处理从串口接收到的数据
        timestamp= time.time()
        # print(f"Received data: {data.hex()} at {timestamp}")
        if not self.ValidationData(data):
            return
        # 解析数据
        if data[1]==0x30:
            # 解析坐标数据
            x = struct.unpack('<f', data[2:6])[0]
            y = struct.unpack('<f', data[6:10])[0]
            yaw = struct.unpack('<f', data[10:14])[0]
            tf_data = tf(x, y, yaw)
            #算出速度
            twist_msg = Vector3Stamped()
            if self.tf_last.x != 0.0 or self.tf_last.y != 0.0:
                dt = timestamp - self.last_timestamp
                twist_msg.vector.x = (tf_data.x - self.tf_last.x) / dt
                twist_msg.vector.y = -(tf_data.y - self.tf_last.y) / dt
                #增加过零点
                dyaw= (tf_data.yaw - self.tf_last.yaw)
                if dyaw >math.pi :
                    dyaw -= 2*math.pi
                elif dyaw < -math.pi:
                    dyaw += 2*math.pi
                twist_msg.vector.z = dyaw / dt
                # print(dt)
            local_msg = Vector3Stamped()
            local_msg.vector.x = tf_data.x
            local_msg.vector.y = tf_data.y
            local_msg.vector.z = tf_data.yaw
            local_msg.header.stamp= self.get_clock().now().to_msg()  # 设置时间戳
            twist_msg.header.stamp = self.get_clock().now().to_msg()  # 设置
            self.local_pub.publish(local_msg)  # 发布本地坐标消息-0.00018086249265487702
            self.last_timestamp = timestamp
            self.tf_last = tf_data
            self.vel_pub.publish(twist_msg)  # 发布速度消息
        elif data[1]==0x31:
            # 解析传感器数据
            # 8路激光 int16_t
            laser_data = []
            bias=[0,0,0,0.188,0.313,0.313,0.293,0.293]
            for i in range(2, 18, 2):
                #大端
                # value = int.from_bytes(data[i:i+2], byteorder='big', signed=True)
                value:np.int16=struct.unpack('>h', data[i:i+2])[0]  # 使用大端字节序解析
                # laser_data.append(value)
                assert -32768 <= value <= 32767, "Laser data out of range"
                real_value= (value* 0.2167 + 63.1689) / 1000.0
                #保留三位小数-0.00018086249265487702
                real_value = round(real_value, 3)+bias[i//2-1]  # 偏移量
                laser_data.append(real_value)
            #增加到json
            laser_data_json = json.dumps(laser_data)
            laser_data_msg = String(data=laser_data_json)
            self.sick_pub.publish(laser_data_msg)
def main(args=None):
    rclpy.init(args=args)
    sick_communicate = SickCommunicate_t()
    try:
        rclpy.spin(sick_communicate)
    except KeyboardInterrupt:
        pass
    finally:
        sick_communicate.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()