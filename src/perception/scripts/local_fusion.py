#!/usr/bin/env python3
import rclpy
from my_tf import MyTf
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, TransformListener, Buffer,StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped,Vector3Stamped
import json,os,math,numpy as np
import rclpy.time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from itertools import product
class fusion_node_t(Node):
    def __init__(self):
        super().__init__('fusion_node')
        self.declare_parameter('odom_frame','odom_wheel')  #地图坐标系下轮式里程计坐标
        self.declare_parameter('laser_frame', 'laser_link')  #地图坐标系下的激光雷达坐标
        self.declare_parameter('laser_base_frame', 'laser_base_link')  #地图坐标系下slam 的base_link坐标
        self.declare_parameter('base_frame', 'base_link') # 地图坐标系下融合码盘的base_link坐标
        self.declare_parameter('slam_to_map_left_frame','slam_to_map') #slam原点地图左下角的偏移
        self.declare_parameter('fusion_hz', 10)    #修正频率
        self.declare_parameter('map_frame_vec',['camera_init']) # 被监听的tf地图坐标 
        self.declare_parameter('base_frame_vec',['body','aft_mapped'])  # 被监听的tf基座坐标
        self.declare_parameter('slam_hz', 200)              # 被监听的tf频率
        self.declare_parameter('odom_topic','/odom')   #轮式里程计
        self.declare_parameter('slam_debug', True)  # 是否开启slam调试
        # self.declare_parameter('base_link_to_map',[0.39,-0.357,0.0]) #base_link到map 左下角的偏移  右手系
        self.declare_parameter('base_to_laser', [-0.13255, 0.3288, 0.0])  # 激光雷达到base_link的偏移 右手系
        self.declare_parameter('loc_to_map',[0.4938,-0.6706,-0.0141955]) # slam原点到地图左下角的偏移 右手系
        self.declare_parameter('loc_to_map_2',[0,0,0])# 第二个地图的偏移
        self.declare_parameter('map_num',1) #地图编号
        self.laser_frame= self.get_parameter('laser_frame').value  #激光初始点下的激光雷达坐标
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value #轮式里程计坐标
        self.base_frame = self.get_parameter('base_frame').value #发布的base_link坐标
        self.slam_to_map_left_frame = self.get_parameter('slam_to_map_left_frame').value  #slam原点到激光雷达原点的偏移
        self.laser_base_frame = self.get_parameter('laser_base_frame').value  #激光雷达坐标
        self.fusion_hz = self.get_parameter('fusion_hz').value
        # self.map_frame = self.get_parameter('map_frame')
        self.map_frame_vec = self.get_parameter('map_frame_vec').value  #被监听的tf地图坐标
        self.base_frame_vec = self.get_parameter('base_frame_vec').value  #被监听
        # self.base_frame = self.get_parameter('base_frame') 
        self.tf_hz = self.get_parameter('slam_hz').value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.timer = self.create_timer(1.0/self.tf_hz,self.slam_tf_callback)
        self.fusion_timer = self.create_timer(1.0/self.fusion_hz, self.fusion_callback)
        self.tf_publish_timer = self.create_timer(0.5, self.tf_manage)
        self.map_num = self.get_parameter('map_num').value  # 地图编号
        self.odom_sub=self.create_subscription(Vector3Stamped, self.odom_topic, self.odom_callback, 10)
        self.robot_sub= self.create_subscription(String, 'robot_state', self.robot_state_callback, 1)
        #创建均值滤波器
        self.tf_overage_x = []
        self.tf_overage_y = []
        self.tf_overage_w=[]
        self.tf_overage_z=[]
        self.tf_overage_yaw=np.array([], dtype=np.float32)  # 用于存储yaw的均值滤波
        #轮式里程计相关
        self.odom_x=0.0
        self.odom_y=0.0
        self.odom_yaw=0.0
        self.map_odom_x=0.0
        self.map_odom_y=0.0
        self.map_odom_yaw=0.0
        if self.get_parameter('slam_debug').value:
            self.odom_frame='odom_wheel_debug'
            self.base_frame='base_link_debug'
            self.laser_frame='laser_link_debug'
            self.laser_base_frame='laser_base_link_debug'
            self.slam_to_map_left_frame='slam_to_map_debug'
    def slam_tf_callback(self):
        transform=TransformStamped()
        if len(self.map_frame_vec) >0 and len(self.base_frame_vec) > 0:
            #尝试找出其中能用的tf
            for map_frame, base_frame in product(self.map_frame_vec, self.base_frame_vec): #遍历所有frame组合
                try:
                    if not self.tf_buffer.can_transform(map_frame, base_frame, rclpy.time.Time()):
                        # self.get_logger().warn(f"Transform from {map_frame} to {base_frame} not available")
                        continue
                    transform = self.tf_buffer.lookup_transform(map_frame, base_frame, rclpy.time.Time())
                    break  # 找到一个可用的就退出循环
                except Exception as e:
                    self.get_logger().error(f"Failed to get transform from {map_frame} to {base_frame}: {e}")
        else:
            try:
                if not self.tf_buffer.can_transform(map_frame, base_frame, rclpy.time.Time()):
                    # self.get_logger().warn(f"Transform from {map_frame} to {base_frame} not available")
                    return
                transform = self.tf_buffer.lookup_transform(map_frame, base_frame, rclpy.time.Time())
            except Exception as e:
                self.get_logger().error(f"Failed to get transform: {e}")
                return
        # self.tf_overage_buffer.append(transform)
        self.tf_overage_x.append(transform.transform.translation.x)
        self.tf_overage_y.append(transform.transform.translation.y)
        self.tf_overage_w.append(transform.transform.rotation.w)
        self.tf_overage_z.append(transform.transform.rotation.z)
        yaw= 2 * math.atan2(transform.transform.rotation.z, transform.transform.rotation.w)
        self.tf_overage_yaw = np.append(self.tf_overage_yaw, yaw)
    def fusion_callback(self):
        #将tf_overage_x,y,w,z进行均值滤波
        x_diff,y_diff,yaw_diff = 0.0,0.0,0.0
        if len(self.tf_overage_x) != 0: # 如果有slam 数据
            # 计算均值
            laser_odom_x = sum(self.tf_overage_x) / len(self.tf_overage_x)
            laser_odom_y = sum(self.tf_overage_y) / len(self.tf_overage_y)
            w = sum(self.tf_overage_w) / len(self.tf_overage_w)
            z = sum(self.tf_overage_z) / len(self.tf_overage_z)
            sin_sum=np.sum(np.sin(self.tf_overage_yaw))
            cos_sum=np.sum(np.cos(self.tf_overage_yaw))
            mean_yaw = math.atan2(sin_sum, cos_sum)  # 计算均值yaw
            
            #清空缓存
            self.tf_overage_x.clear()
            self.tf_overage_y.clear()
            self.tf_overage_w.clear()
            self.tf_overage_z.clear()
            self.tf_overage_yaw = np.array([], dtype=np.float32)  # 清空yaw缓存
            #算出x,y,w,z与轮式里程计的偏差
            #从w z 算出yaw
            # yaw = 2*math.atan2(z, w)
            yaw = mean_yaw  # 使用均值yaw
            #将激光雷达发布到slam原点的tf
            self.tf_publish(self.slam_to_map_left_frame, self.laser_frame, laser_odom_x, laser_odom_y, yaw)
            try:
                base_link_tf= self.tf_buffer.lookup_transform('map_left_corner', self.laser_base_frame, rclpy.time.Time())
            except Exception as e:
                return
            x_base_slam = base_link_tf.transform.translation.x
            y_base_slam = base_link_tf.transform.translation.y
            w= base_link_tf.transform.rotation.w
            z= base_link_tf.transform.rotation.z
            yaw_base_slam= 2 * math.atan2(z, w)  # 计算yaw
            # self.odom_yaw = yaw
            # laser-odom的tf
            dyaw= yaw_base_slam - self.odom_yaw
            if dyaw > math.pi:
                dyaw -= 2 * math.pi
            elif dyaw < -math.pi:
                dyaw += 2 * math.pi


            # 平移部分
            x_diff=x_base_slam-(self.odom_x*math.cos(dyaw)-self.odom_y*math.sin(dyaw))
            y_diff=y_base_slam-(self.odom_x*math.sin(dyaw)+self.odom_y*math.cos(dyaw))
            yaw_diff=dyaw

            # self.tf_publish("map","laser_odom",x,y,yaw)      #激光雷达slam的tf 调试用转化到base_link坐标系
        #发布轮式偏移的tf
        
        self.tf_publish('map_left_corner', self.odom_frame, x_diff, y_diff, yaw_diff) #距离上电原点的偏
        if self.odom_x == 0.0 and self.odom_y == 0.0 and self.odom_yaw == 0.0:
            self.tf_publish(self.odom_frame, self.base_frame, 0.0, 0.0, 0.0)
            return
    def odom_callback(self, msg:Vector3Stamped):
        self.odom_x = msg.vector.x
        self.odom_y = -msg.vector.y
        self.odom_yaw = msg.vector.z
        self.tf_publish(self.odom_frame, self.base_frame, self.odom_x, self.odom_y, self.odom_yaw)
        #发布轮式里程计的tf
    def sick_callback(self, msg: String):
        """处理激光雷达数据"""
        # msg是8路float 数据的字符串表示
        
        laser_data = json.loads(msg.data)
        if len(laser_data) != 8:
            self.get_logger().warn("激光雷达数据长度不正确")
            return
        # 将激光数据转换为numpy数组
        self.laser_array = np.array(laser_data, dtype=np.float32)
    def tf_publish(self,base_frame:str,child_frame:str,x,y,yaw):
        #先从yaw 算出w z
        w = math.cos(yaw / 2)
        z = math.sin(yaw / 2)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = base_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.w = w
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = z
        self.tf_broadcaster.sendTransform(transform)
    def tf_static_publish(self, base_frame: str, child_frame: str, x: float, y: float, yaw: float):
        """发布静态tf"""
        transform = TransformStamped()
        w = math.cos(yaw / 2)
        z = math.sin(yaw / 2)
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = base_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        transform.transform.rotation.w = w
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        self.static_tf_broadcaster.sendTransform(transform)
    def tf_manage(self):
        """初始化静态tf"""
        self.tf_publish('map','odom',0.0,0.0,0.0) #地图坐标系到轮式里程计坐标系
        self.tf_publish(
            'odom', 'map_left_corner',0.0,8.0,0.0
        ) # 从地图右下角到地图左下角
        laser_to_base = self.get_parameter('base_to_laser').value
        self.tf_publish(
            self.laser_frame,self.laser_base_frame,
            laser_to_base[0], laser_to_base[1],laser_to_base[2] 
        )# 激光雷达到base_link的偏移
        #选择地图1还是地图2
        x_bias, y_bias, yaw_bias = 0.0, 0.0, 0.0
        if self.map_num==1 or self.map_num==2:  # 偶数地图编号
            slam_to_laser_init =self.get_parameter('loc_to_map').value
            if self.map_num==2:
                x_bias=15.0+slam_to_laser_init[0]  # 偶数地图编号，x轴偏移
                y_bias=-8.0+slam_to_laser_init[1]
                yaw_bias=slam_to_laser_init[2]+ math.pi  # 偶数
            else:
                x_bias=slam_to_laser_init[0]
                y_bias=slam_to_laser_init[1]
                yaw_bias=slam_to_laser_init[2]                
        elif self.map_num==3 or self.map_num==4:  # 奇数地图编号
            slam_to_laser_init = self.get_parameter('loc_to_map_2').value
            if self.map_num==4:
                x_bias=15.0+slam_to_laser_init[0]
                y_bias=-8.0+slam_to_laser_init[1]
                yaw_bias=slam_to_laser_init[2]+ math.pi  # 奇数地图
            else:
                x_bias=slam_to_laser_init[0]
                y_bias=slam_to_laser_init[1]
                yaw_bias=slam_to_laser_init[2]
        self.tf_publish(
            'map_left_corner', self.slam_to_map_left_frame,
           x_bias, y_bias, yaw_bias
        )
        # 初始化全场的tf
    def robot_state_callback(self, msg: String):
        """处理机器人状态消息"""
        
        data = json.loads(msg.data)
        if 'map_num' in data:
            self.map_num = data['map_num']
            print(f"\033[92m 地图编号改为{self.map_num} \033[0m")
def main(args=None):
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = fusion_node_t()
    exe = MultiThreadedExecutor()
    exe.add_node(node)
    try:
        exe.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    print("Starting fusion node...")
    main()
    