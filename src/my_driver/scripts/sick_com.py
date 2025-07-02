import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, json, os,time,math,struct
import numpy as np
sys.path.append('/home/Elaina/ros2_driver/src') 
from protocol_lib.myserial import AsyncSerial_t
from std_msgs.msg import String
class tf:
    def __init__(self, x=0.0, y=0.0,yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
class SickCommunicate_t(Node):
    def __init__(self):
        super().__init__('sick_communicate')
        self.declare_parameter('serial_port', '/dev/serial_sick')
        self.declare_parameter('serial_baudrate', 230400)
        self.serial = AsyncSerial_t(
            self.get_parameter('serial_port').value,
            self.get_parameter('serial_baudrate').value
        )
        self.vel_pub= self.create_publisher(Twist, '/sick/vel', 10)  # 发布速度话题
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
            twist_msg = Twist()
            if self.tf_last.x != 0.0 or self.tf_last.y != 0.0:
                dt = timestamp - self.last_timestamp
                twist_msg.linear.x = (tf_data.x - self.tf_last.x) / dt
                twist_msg.linear.y = (tf_data.y - self.tf_last.y) / dt
                #增加过零点
                dyaw= (tf_data.yaw - self.tf_last.yaw)
                if dyaw >math.pi :
                    dyaw -= 2*math.pi
                elif dyaw < -math.pi:
                    dyaw += 2*math.pi
                twist_msg.angular.z = dyaw / dt
            self.last_timestamp = timestamp
            self.tf_last = tf_data
            self.vel_pub.publish(twist_msg)  # 发布速度消息
        elif data[1]==0x31:
            # 解析传感器数据
            # 8路激光 int16_t
            laser_data = []
            for i in range(2, 18, 2):
                #大端
                # value = int.from_bytes(data[i:i+2], byteorder='big', signed=True)
                value:np.int16=struct.unpack('>h', data[i:i+2])[0]  # 使用大端字节序解析
                laser_data.append(value)
                assert -32768 <= value <= 32767, "Laser data out of range"
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