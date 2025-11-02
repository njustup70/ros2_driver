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
from nav_msgs.msg import Odometry

class fusion_node_t(Node):
    def __init__(self):
        super().__init__('fusion_node')
        self.declare_parameter('odom_frame','odom_wheel')  #地图坐标系下轮式里程计坐标,也就是根据slam融合纠正过累积误差的坐标
        self.declare_parameter('base_frame', 'base_link') # 地图坐标系下融合码盘的base_link坐标
        self.declare_parameter('improved_slam_hz', 100)    #修正频率
        self.declare_parameter('slam_odom',['camera_init']) # 被监听的tf地图坐标 
        self.declare_parameter('slam_base_link',['body','aft_mapped'])  # 被监听的tf基座坐标
        self.declare_parameter('slam_hz', 200)              # 被监听的tf频率
        self.declare_parameter('odom_topic','/odom')   #轮式里程计
        self.declare_parameter('base_to_laser', [-0.23751, -0.24275, 0.0])  # 激光雷达到base_link的偏移 右手系
        self.declare_parameter('riqiang_y', -0.10975) #日墙时候的雷达y偏移
        self.declare_parameter('slam_to_map',[0.46876+0.26775,-0.08475-0.0815,0.0])
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value #轮式里程计坐标
        self.base_frame = self.get_parameter('base_frame').value #发布的base_link坐标
        self.improved_slam_hz = self.get_parameter('improved_slam_hz').value
        self.slam_odom = self.get_parameter('slam_odom').value  #被监听的tf地图坐标
        self.slam_base_link = self.get_parameter('slam_base_link').value  #被监听
        self.slam_hz = self.get_parameter('slam_hz').value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        #创建均值滤波器 主要用于slam_tf_callback(self)函数
        self.tf_overage_x = []
        self.tf_overage_y = []
        self.tf_overage_w=[]
        self.tf_overage_z=[]
        self.tf_overage_yaw=np.array([], dtype=np.float32)  # 用于存储yaw的均值滤波

        #创建slam数据缓存
        self.slam_x = 0.0
        self.slam_y = 0.0
        self.slam_yaw = 0.0
        self.slam_stamp = None 
        #
        self.odom_x=0.0
        self.odom_y=0.0
        self.odom_yaw=0.0

        #两个定时器回调和两个订阅者回调
        self.slam_timer = self.create_timer(1.0/self.slam_hz,self.slam_tf_callback)
        self.improved_slam_timer = self.create_timer(1.0/self.improved_slam_hz, self.publish_improved_slam)
        self.slam_odom_pub = self.create_publisher(Odometry,'slam_improved',10)
        self.improved_slam_sub  = self.create_subscription(Odometry,'/slam_improved',self.improved_slam_callback,10)
        self.odom_sub = self.create_subscription(Vector3Stamped,self.odom_topic,self.fuse_callback,10)
        self.odom_pub= self.create_publisher(Vector3Stamped, 'base_link_odom', 10)
        

    def slam_tf_callback(self):
        """功能描述:这个函数是一个定时器回调函数,频率200hz,执行寻找特定某段tf的功能,如果找到了特定的tf,就记录下来x,y,w,z和yaw"""
        """
            参数声明:
            self.slam_odom 这是slam的发布的里程计
            self.slam_base_link 这是slam发布的车体(也就是lidar的坐标)
            self.tf_overage_x slam发布的那段tf的x
            self.tf_overage_y slam发布的那段tf的y
            self.tf_overage_z slam发布的那段tf的w
            self.tf_overage_w slam发布的那段tf的z
            self.tf_overage_yaw slam发布的那段tf的yaw
        """
        transform=TransformStamped()
        if len(self.slam_odom) >0 and len(self.slam_base_link) > 0:
            #尝试找出其中能用的tf
            for map_frame, base_frame in product(self.slam_odom, self.slam_base_link): #遍历所有frame组合
                try:
                    if not self.tf_buffer.can_transform(map_frame, base_frame, rclpy.time.Time()):
                        # self.get_logger().warn(f"Transform from {map_frame} to {base_frame} not available")
                        # print('找不到slam的tf')
                        continue
                    transform = self.tf_buffer.lookup_transform(map_frame, base_frame, rclpy.time.Time())
                    # print('找到slam的tf')
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
        self.tf_overage_x.append(transform.transform.translation.x)
        # print(f'最新添加的X值: {transform.transform.translation.x:.6f}')
        # print(f'当前tf_overage_x列表: {self.tf_overage_x}')
        self.tf_overage_y.append(transform.transform.translation.y)
        self.tf_overage_w.append(transform.transform.rotation.w)
        self.tf_overage_z.append(transform.transform.rotation.z)
        yaw= 2 * math.atan2(transform.transform.rotation.z, transform.transform.rotation.w)
        self.tf_overage_yaw = np.append(self.tf_overage_yaw, yaw)

    def publish_improved_slam(self):
        """功能描述:这个函数是一个定时器函数,频率为100hz,把在slam_tf_callback()函数里存储下来的x,y,z,w,yaw进行一个均值滤波之后通过一个话题发布(100hz发布)"""
        """
            参数说明：
            base_to_laser 因为从slam那段tf获得的xyzwyaw是基于雷达的位置的,所以如果想获得车体的位置，需要雷达到车体的转换
            slam_to_map 同上，我们最终需要的是车体在地图中的位置，所以还需要车体到地图起点的转换
            x_list 之前tf的x数据是缓存了，所以这个变量是把缓存的x数据copy过来，其余类似参数同理，这个变量只是局部变量，所以不需要手动清除
            laser_odom_x 均值滤波之后的x
        """

        base_to_laser = self.get_parameter('base_to_laser').value  # [x_offset, y_offset, yaw_offset]
        slam_to_map = self.get_parameter('slam_to_map').value  # [map_origin_x, map_origin_y]
        

        
        if len(self.tf_overage_x) == 0:
            return
            
        # 创建数据的本地副本，以便在锁外进行计算 [3](@ref)
        x_list = self.tf_overage_x.copy()
        # print(f'{x_list}')
        y_list = self.tf_overage_y.copy()
        w_list = self.tf_overage_w.copy()
        z_list = self.tf_overage_z.copy()
        yaw_list = self.tf_overage_yaw.copy()
            
            # 清空缓存，为下一轮数据采集做准备
        self.tf_overage_x.clear()
        self.tf_overage_y.clear()
        self.tf_overage_w.clear()
        self.tf_overage_z.clear()
        self.tf_overage_yaw = np.array([], dtype=np.float32)
        
        laser_odom_x = sum(x_list) / len(x_list)
        # print(f'当前laser_odom_x列表: {laser_odom_x}')
        laser_odom_y = sum(y_list) / len(y_list)
        w = sum(w_list) / len(w_list)
        z = sum(z_list) / len(z_list)
        
        # 计算角度均值（正确处理角度环绕）
        sin_sum = np.sum(np.sin(yaw_list))
        cos_sum = np.sum(np.cos(yaw_list))
        mean_yaw = math.atan2(sin_sum, cos_sum)  # 计算均值yaw
        
        # ==================== Odometry消息发布（原有逻辑） ====================
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "camera_init_improved"  # 父坐标系
        odom_msg.child_frame_id = "aft_mapped_improved"   # 子坐标系
        
        odom_msg.pose.pose.position.x = laser_odom_x 
        odom_msg.pose.pose.position.y = laser_odom_y 
        odom_msg.pose.pose.position.z = 0.0
        
        # 将yaw转换为四元数
        qw = math.cos(mean_yaw / 2.0)
        qz = math.sin(mean_yaw / 2.0)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        
        # 设置协方差
        odom_msg.pose.covariance = [0.1] * 36
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0
        odom_msg.twist.covariance = [0.1] * 36
        
        self.slam_odom_pub.publish(odom_msg)

    def improved_slam_callback(self,msg:Odometry):
        """功能描述：这是一个ros2订阅者的回调函数，一旦有提升过频率的slam话题发布，就会记录相应的数据，为之后的数据融合做准备"""
        """
            参数声明：
            self.slam_x 用于记录最新一次话题的数据，其他同理
        """
        try:
            self.slam_x = msg.pose.pose.position.x
            self.slam_y = msg.pose.pose.position.y
            orientation = msg.pose.pose.orientation
            siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
            cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
            self.slam_yaw = math.atan2(siny_cosp, cosy_cosp)
            self.slam_stamp = msg.header.stamp
        except Exception as e:
            self.get_logger().error(f"SLAM回调处理错误: {e}")


    def fuse_callback(self,msg:Vector3Stamped):
        """功能描述：这是ros2的/odom话题订阅者的回调函数，一旦有odom话题发布，就会获取记录的slam的数据，并进行数据融合，把轮式里程计的偏差纠正后，通过话题发布车体坐标"""
        """
            参数声明：
            self.odom_x 有话题就记录下来数据，其余同理
            base_link_tf 获取map到车体中心的tf的偏移
        """
        x_diff,y_diff,yaw_diff = 0.0,0.0,0.0
        self.odom_x = msg.vector.x
        self.odom_y = -msg.vector.y
        self.odom_yaw = msg.vector.z
        self.tf_publish(self.odom_frame, self.base_frame, self.odom_x, self.odom_y, self.odom_yaw)
        dyaw= self.slam_yaw - self.odom_yaw
        if dyaw > math.pi:
            dyaw -= 2 * math.pi
        elif dyaw < -math.pi:
            dyaw += 2 * math.pi
        x_diff=self.slam_x-(self.odom_x*math.cos(dyaw)-self.odom_y*math.sin(dyaw))
        y_diff=self.slam_y-(self.odom_x*math.sin(dyaw)+self.odom_y*math.cos(dyaw))
        yaw_diff=dyaw
        if self.slam_x != 0.0 and self.slam_y != 0.0:
            self.tf_publish('map_left_corner', self.odom_frame, x_diff, y_diff, yaw_diff)      
        try:
            base_link_tf = self.tf_buffer.lookup_transform('map_left_corner', self.base_frame, rclpy.time.Time())
            base_link_odom= Vector3Stamped()
            base_link_odom.header.stamp = self.get_clock().now().to_msg()
            base_link_odom.header.frame_id = self.odom_frame
            base_link_odom.vector.x = base_link_tf.transform.translation.x
            base_link_odom.vector.y = base_link_tf.transform.translation.y 
            base_link_odom.vector.z = 2 * math.atan2(base_link_tf.transform.rotation.z, base_link_tf.transform.rotation.w)  # 计算yaw
            self.odom_pub.publish(base_link_odom)  # 发布最终车体位置
        except Exception as e:
            return
        
    def tf_publish(self,base_frame:str,child_frame:str,x,y,yaw):
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
        try:
            self.tf_broadcaster.sendTransform(transform)
        except Exception as e:
            self.get_logger().error(f"Failed to publish transform from {base_frame} to {child_frame}: {e}")
            return

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