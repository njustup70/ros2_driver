#!/usr/bin/env python3
'''
串口改成队列发送
'''
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
from protocol_lib.get_log_shootdata_local import DataSimulator
from rclpy.time import Time
import struct
from tf2_ros import TransformListener,Buffer
from std_msgs.msg import String
from sick_com import SickHandler
from fuck_slam import FuckSlam

class Communicate_t(Node):
    def __init__(self):
        super().__init__('communicate_t')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('serial_port', '/dev/serial_ch340')
        self.declare_parameter('serial_baudrate', 230400)
        self.sub=self.create_subscription(Twist,
        self.get_parameter('cmd_vel_topic').value,
        self.cmd_topic_callback,10)
        # 初始化串口通信
        self.serial=AsyncSerial_t(
                self.get_parameter('serial_port').value,
                self.get_parameter('serial_baudrate').value)
        self.serial.startListening(self.data_callback,wait_time=0.01)#监听线程还开启自动重连
        # 看门狗相关变量
        self.last_msg_time = time.time()
        self.watchdog_timeout = 0.5 #0.5秒超时
        self.stop_event = Event()
        self.watchdog_thread = Thread(target=self.watchdog_loop)
        self.watchdog_thread.daemon = True
        self.watchdog_thread.start()
        # tf相关
        self.buffer= Buffer()
        self.tf_listener = TransformListener(self.buffer, self)
        self.tf_timer= self.create_timer(0.02, self.tf_timer_callback)  # 50Hz 定时器
        # ros2状态相关
        self.robot_state_pub = self.create_publisher(String, 'robot_state', 10)
        self.robot_state_sub = self.create_subscription(String,'robot_state',self.robot_state_callback,1)
        self.aligned_state = False  # 用于跟踪对齐状态
        #self.serial.startListening()#监听线程还开启自动重连
        self.serial_queue = []  # 用于存储串口数据
        self.serial_publish_timer = self.create_timer(0.002, self.publish_serial_data)  #500hz 定时器
        # sick数据处理
        self.sick_handler = SickHandler(self)
    def cmd_topic_callback(self, msg:Twist):
        '''
        速度指令发送
        '''
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
        # self.serial.write(self.ValidationData(data))
        self.queue_add_data(data)  # 将数据添加到队列中
    
    def ValidationData(self,data:bytes):
        """验证数据格式，添加帧头和帧尾"""
        #帧头为中间data16进制之和
        #帧尾为中间data16按位异或
        assert isinstance(data, bytes), "Data must be of type bytes"
        head=np.uint8(0)
        for byte in data:
            head += np.uint8(byte)
        head=bytes([head])
        tail=head
        # print(f"head: {head}, tail: {tail}")
        # print(f"Sending data: {[hex(b) for b in data]}")
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

    def ValidationData_sick_com(self,data:bytes) -> bool:
        '''
            sick分支sick_com.py中的数据包校验函数(ValidationData)
            返回一个bool值 true表示校验通过
            只有当head和tail都校验通过 该数据帧视为有效
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

    def data_callback(self, data:bytes):
        """串口数据回调函数"""
        # print(f"Received data: {data}")
        # 这里可以添加对接收到数据的处理逻辑
        # 例如解析数据，更新状态等   
        databag = data.rstrip(b'\x00')
        if len(data) < 2:
            return False
        data_valid=databag[1:-1]
        checksum = 0
        for byte in data_valid:
            checksum += byte
        checksum &= 0xFF  # 保留低8位
        assert checksum < 256, "Checksum must be less than 256"

        if checksum == databag[-1] and checksum == databag[0]: 
            self.sick_handler.handle_data(data)  # 处理sick数据
            # 校验通过,表示数据帧没有损坏
            if data[1] == '0x40':
                json_data={
                    'reset_slam': True
                }
                self.robot_state_pub.publish(String(data=json.dumps(json_data)))
                print("Reset Slam_docker !!!")

            if data[1] == '0x34':
                # 校验数据类型,0x34表示类型为射击参数
                logger = DataSimulator(port='/dev/serial_ch340',baudrate=230400)
                parsed = logger.parse_laser_frame(data)
                print(f"Parsed Results: "
                    f"Time={parsed['TimeOffset']}s, "
                    f"SetRpm={parsed['SetRpm']}, "
                    f"DeltaRpm_Up={parsed['DeltaRpm_Up']}, "
                    f"DeltaRpm_Down={parsed['DeltaRpm_Down']}, "
                    f"Velo={parsed['Velo']}")
                logger.save_to_json(parsed)

        if data==b'\x34\x33\x00\x20': #如果是0x34 0x33 0x0 0x20
            json_data={
                "nav_state":"IDLE"
            }
            self.robot_state_pub.publish(String(data=json.dumps(json_data)))
        # print([hex(b) for b in data])
    
    def robot_state_callback(self, msg: String):
        """导航和机器人其他状态回调函数

        Args:
            msg (String): _description_
        """
        data= json.loads(msg.data)
        if 'nav_state' in data:
            nav_state = data['nav_state']
            if nav_state ==  'ALIGNED':
                print("Received nav_state: ALIGNED")
                # 发送对齐完成信号
                self.aligned_state = True
            else:
                self.aligned_state = False  # 如果不是对齐状态，重置对齐状态
            if self.aligned_state :
                self.aligned_state = False  # 重置对齐状态
                for i in range(10):
                    data:bytes=b'\x23\x23'
                    self.queue_add_data(data)  # 将数据添加到队列中
    
    def tf_timer_callback(self):
        """定时器回调 - 将自身的tf转发给stm32"""
        if self.aligned_state:
            for i in range(10):
                data:bytes=b'\x23\x23'
                self.serial.write(self.ValidationData(data))
                time.sleep(0.02)  # 20ms间隔
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
        # self.serial.write(self.ValidationData(data))
        self.queue_add_data(data)  # 将数据添加到队列中
    
    def queue_add_data(self, data: bytes):
        '''将数据添加到串口队列中'''
        assert isinstance(data, bytes), "Data must be of type bytes"
        if len(self.serial_queue) >20: # 队列长度限制为5
            self.serial_queue.pop(0)
            print("Serial queue is full, removing oldest data")
        self.serial_queue.append(data)
    
    def publish_serial_data(self):
        """定时发布串口数据"""
        if self.serial_queue:
            data = self.serial_queue.pop(0)
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
