#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from geometry_msgs.msg import TransformStamped,Vector3Stamped
import json,os,math,numpy as np
import rclpy.time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from SiLocator import SiLocator, SickData, Vec3, SICK_NUMS
class fusion_node_t(Node):
    def __init__(self):
        super().__init__('fusion_node')
        self.declare_parameter('odom_frame','odom_wheel')  #轮式里程计坐标
        self.declare_parameter('publish_tf_name', 'base_link')
        self.declare_parameter('fusion_hz', 10)    #修正频率
        self.declare_parameter('map_frame', 'camera_init')  # 被监听的tf地图坐标
        self.declare_parameter('base_frame', 'body')       # 被监听的tf基座坐标
        self.declare_parameter('slam_hz', 200)              # 被监听的tf频率
        self.declare_parameter('odom_topic','/odom')   #轮式里程计
        self.declare_parameter('sick_topic', '/sick/lidar')  #激光雷达点数据
        self.declare_parameter('lidar_slam_topic', '/lidar_slam/odom')  #激光雷达slam
        self.declare_parameter('lidar_x_bias',-0.236)  #激光雷达到odom的偏移
        self.declare_parameter('lidar_y_bias', -0.267) #激光雷达到odom的偏移
        self.declare_parameter('use_sick', False)  # 是否使用点激光数据
        self.odom_topic = self.get_parameter('odom_topic').value
        self.sick_topic = self.get_parameter('sick_topic').value
        self.use_sick = self.get_parameter('use_sick').value
        self.publish_tf_name = self.get_parameter('publish_tf_name').value
        self.fusion_hz = self.get_parameter('fusion_hz').value
        # self.map_frame = self.get_parameter('map_frame')
        # self.base_frame = self.get_parameter('base_frame') 
        self.odom_frame = self.get_parameter('odom_frame').value #轮式里程计的坐标系
        self.tf_hz = self.get_parameter('slam_hz').value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0/self.tf_hz,self.slam_tf_callback)
        self.fusion_timer = self.create_timer(1.0/self.fusion_hz, self.fusion_callback)
        self.odom_sub=self.create_subscription(Vector3Stamped, self.odom_topic, self.odom_callback, 10)
        #创建均值滤波器
        self.tf_overage_x = []
        self.tf_overage_y = []
        self.tf_overage_w=[]
        self.tf_overage_z=[]
        #轮式里程计相关
        self.odom_x=0.0
        self.odom_y=0.0
        self.odom_yaw=0.0
        self.map_odom_x=0.0
        self.map_odom_y=0.0
        self.map_odom_yaw=0.0
        #sick 相关
        self.odom_tf=Vec3()
        self.silo_tf=Vec3()
        self.chas_tf=Vec3()
        self.laser_array = np.zeros((8,), dtype=np.float32)  # 初始化激光数据数组
        self.my_locator = SiLocator()
        if self.use_sick:
            self.sick_sub = self.create_subscription(String, self.get_parameter('sick_topic').value, self.sick_callback, 1)
            self.sick_update_timer=self.create_timer(0.01,self.sick_update)
            self.sick_point_pub=self.create_publisher(PointCloud2, '/sick/pointcloud', 1)
            
    def slam_tf_callback(self):
        transform=TransformStamped()
        map_frame = self.get_parameter('map_frame').value
        base_frame = self.get_parameter('base_frame').value
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
    def fusion_callback(self):
        #将tf_overage_x,y,w,z进行均值滤波
        x_diff,y_diff,yaw_diff = 0.0,0.0,0.0
        if len(self.tf_overage_x) != 0: # 如果有slam 数据
            # 计算均值
            x = sum(self.tf_overage_x) / len(self.tf_overage_x)
            y = sum(self.tf_overage_y) / len(self.tf_overage_y)
            w = sum(self.tf_overage_w) / len(self.tf_overage_w)
            z = sum(self.tf_overage_z) / len(self.tf_overage_z)
            #清空缓存
            self.tf_overage_x.clear()
            self.tf_overage_y.clear()
            self.tf_overage_w.clear()
            self.tf_overage_z.clear()
            #算出x,y,w,z与轮式里程计的偏差
            #从w z 算出yaw
            yaw = 2*math.atan2(z, w)
            #将激光雷达x y 转化到base_link坐标系车体x y 
            x_bias = self.get_parameter('lidar_x_bias').value
            y_bias = self.get_parameter('lidar_y_bias').value
            x=x+x_bias*math.cos(yaw) - y_bias*math.sin(yaw)-x_bias #全局的原点要先变
            y=y+x_bias*math.sin(yaw) + y_bias*math.cos(yaw)-y_bias #全局的原点要先变
            
            # laser-odom的tf
            dyaw= yaw - self.odom_yaw

            # 平移部分
            x_diff=x-(self.odom_x*math.cos(dyaw)-self.odom_y*math.sin(dyaw))
            y_diff=y-(self.odom_x*math.sin(dyaw)+self.odom_y*math.cos(dyaw))
            yaw_diff=dyaw
            if yaw_diff > math.pi:
                yaw_diff -= 2 * math.pi
            elif yaw_diff < -math.pi:
                yaw_diff += 2 * math.pi

            self.tf_publish("map","laser_odom",x,y,yaw)      #激光雷达slam的tf 调试用转化到base_link坐标系
        #发布轮式偏移的tf
        
        self.tf_publish("odom_transform", self.odom_frame, x_diff, y_diff, yaw_diff)
        if self.odom_x == 0.0 and self.odom_y == 0.0 and self.odom_yaw == 0.0:
            self.tf_publish(self.odom_frame, self.publish_tf_name, 0.0, 0.0, 0.0)
    def odom_callback(self, msg:Vector3Stamped):
        self.odom_x = msg.vector.x
        self.odom_y = -msg.vector.y
        self.odom_yaw = msg.vector.z
        self.tf_publish(self.odom_frame, self.publish_tf_name, self.odom_x, self.odom_y, self.odom_yaw)
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
    def sick_update(self):
        #从odom_transform到base_link获得
        if self.tf_buffer.can_transform('odom_transform',self.publish_tf_name, rclpy.time.Time()):
            transform = self.tf_buffer.lookup_transform('odom_transform', self.publish_tf_name, rclpy.time.Time())
            self.odom_tf.x = transform.transform.translation.x+0.2
            self.odom_tf.y = -transform.transform.translation.y+0.2 #坐标系是反的并且加一个初始点偏移
            yaw= 2 * math.atan2(transform.transform.rotation.z, transform.transform.rotation.w)
            self.odom_tf.z = yaw
        else: 
            return
        self.chas_tf=self.odom_tf+self.silo_tf
        #提取最后n 路的数据
        distance= self.laser_array[3:]
    
        theta=[180,77.85,360-77.85,360-37.7,37.7]
        # self.my_locator.update_sick_data(0,d/istance[0],)
        for i in range(SICK_NUMS):
            self.my_locator.update_sick_data(
                i,
                distance[i],
                theta[i]
            )
        grd,orivec,cost= self.my_locator.grad_decent(self.chas_tf,self.silo_tf)
        # print(f'grd:{grd},orivec:{orivec},cost:{cost}')
        print(f'odom:{self.odom_tf}')
        print(f'silo:{self.silo_tf}')
        print(f'chassis:{self.chas_tf}')
        # print(self.silo_tf)        #将sick点云转换为PointCloud2消息
        pointcloud = PointCloud2()
        pointcloud.header.stamp = self.get_clock().now().to_msg()
        pointcloud.header.frame_id = self.get_parameter('publish_tf_name').value
        pointcloud.height = 1
        pointcloud.width = SICK_NUMS
        pointcloud.is_dense = True
        pointcloud.is_bigendian = False
        pointcloud.point_step = 12  # 每个点的字节数
        pointcloud.row_step = pointcloud.point_step * SICK_NUMS
        import struct
        from sensor_msgs.msg import PointField

        # 定义 PointField 列表（XYZ，每个都是 float32）
        pointcloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # 构造实际点数据
        points_bytes = bytearray()
        for i in range(SICK_NUMS):
            r = distance[i]
            theta_deg = theta[i]
            theta_rad = math.radians(theta_deg)
            x = r * math.cos(theta_rad)
            y = r * math.sin(theta_rad)
            z = 0.0  # 2D 激光默认 Z=0

            # 按 float32 小端序打包
            points_bytes.extend(struct.pack('<fff', x, y, z))

        pointcloud.data = bytes(points_bytes)
        # 发布点云消息
        self.sick_point_pub.publish(pointcloud)
        self.tf_publish('odom','odom_transform',self.silo_tf.x,8-self.silo_tf.y,self.silo_tf.z)  # 发布sick的tf
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
    