#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys,json,os
import numpy as np
import time
from threading import Thread, Event
sys.path.append('/home/Elaina/ros2_driver/src') 
# print(sys.path)
from protocol_lib.myserial import AsyncSerial_t
from rclpy.time import Time
import struct
from tf2_ros import TransformListener,Buffer
from std_msgs.msg import String
class Communicate_t(Node):
    def __init__(self):
        super().__init__('communicate_t')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('serial_port', '/dev/serial_ch340')
        self.declare_parameter('serial_baudrate', 115200)
        self.sub=self.create_subscription(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            self.cmd_topic_callback,
            10)
        self.serial=AsyncSerial_t(
                self.get_parameter('serial_port').value,
                self.get_parameter('serial_baudrate').value)
        
        self.last_msg_time = time.time()
        self.watchdog_timeout = 0.5 #0.5秒超时
        self.stop_event = Event()
        self.watchdog_thread = Thread(target=self.watchdog_loop)
        self.watchdog_thread.daemon = True
        self.watchdog_thread.start()
        self.buffer= Buffer()
        self.tf_listener = TransformListener(self.buffer, self)
        self.tf_timer= self.create_timer(0.02, self.tf_timer_callback)  # 50Hz 定时器
        self.serial.startListening(self.data_callback)#监听线程还开启自动重连
        self.robot_state_pub = self.create_publisher(String, 'robot_state', 10)
        self.robot_state_sub = self.create_subscription(String,'robot_state',self.robot_state_callback,1)
        #self.serial.startListening()#监听线程还开启自动重连
    def cmd_topic_callback(self, msg:Twist):
        self.last_msg_time = time.time()
        #获得信息发给串口
        linear_x=msg.linear.x
        linear_y=msg.linear.y
        angular_z=msg.angular.z
        # data_header=b'0xFE'
        # 头部数据16进制FE
        data_header=b'\x10'
        # data_tail=b'\xFB'
        # 浮点准化成小端4字节
        linear_x=struct.pack('<f',linear_x)
        linear_y=struct.pack('<f',linear_y)
        angular_z=struct.pack('<f',angular_z)
        # 拼接数据
        data= data_header+linear_x+linear_y+angular_z
        self.serial.write(self.ValidationData(data))
    def ValidationData(self,data:bytes):
        #帧头为中间data16进制之和
        #帧尾为中间data16按位异或
        head=np.uint8(0)
        for byte in data:
            head += np.uint8(byte)
        head=bytes([head])
        tail=head
        # print(f"head: {head}, tail: {tail}")
        return head+data+tail
    def watchdog_loop(self):
        """看门狗线程循环，定期检查是否超时"""
        while not self.stop_event.is_set():
            current_time = time.time()
            # 检查是否超时
            if current_time - self.last_msg_time > self.watchdog_timeout:
                # 超时则发送零速度
                zero_twist = Twist()
                zero_twist.linear.x = 0.0
                zero_twist.linear.y = 0.0
                zero_twist.angular.z = 0.0
                self.cmd_topic_callback(zero_twist)
                self.get_logger().debug('Watchdog timeout, sent zero velocity')
            # 短暂休眠避免CPU占用过高
            time.sleep(0.05)
    def data_callback(self, data:bytes):
        """串口数据回调函数"""
        # print(f"Received data: {data}")
        # 这里可以添加对接收到数据的处理逻辑
        # 例如解析数据，更新状态等   
        if data==b'\x34\x33\x00\x20': #如果是0x34 0x33 0x0 0x20
            json_data={
                "nav_state":"IDLE"
            }
            self.robot_state_pub.publish(String(data=json.dumps(json_data)))
        # print([hex(b) for b in data])
    def robot_state_callback(self, msg: String):
        data= json.loads(msg.data)
        if 'nav_state' in data:
            nav_state = data['nav_state']
            if nav_state ==  'ALIGNED':
                # print("Received nav_state: ALIGNED")
                # 发送对齐完成信号
                data=b'\0x22'
                data_all=data+data
                self.serial.write(self.ValidationData(data_all))  
    def tf_timer_callback(self):
        """定时器回调 - 将自身的tf转发给stm32"""
        try:
            transform=self.buffer.lookup_transform('map', 'base_link', time=Time())
        except Exception as e:
            # print(f"TF lookup failed: {e}")
            return
        #右手系转化到左手系
        x= transform.transform.translation.x
        y= -(8-transform.transform.translation.y)
        #从w 和 z中计算yaw角
        z= transform.transform.rotation.z
        w= transform.transform.rotation.w
        yaw = np.arctan2(2.0 * (w * z), 1.0 - 2.0 * (z * z))
        header=b'\x20'
        x=struct.pack('<f',x)
        y=struct.pack('<f',y)
        yaw=struct.pack('<f',yaw)
        # 拼接数据
        data= header+x+y+yaw
        self.serial.write(self.ValidationData(data))

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
