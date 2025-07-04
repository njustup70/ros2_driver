#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from geometry_msgs.msg import TransformStamped,Vector3Stamped
import json,os,math
import rclpy.time

class fusion_node_t(Node):
    def __init__(self):
        super().__init__('fusion_node')
        self.declare_parameter('odom_frame','odom_wheel')  #轮式里程计坐标
        self.declare_parameter('publish_tf_name', 'base_link_debug')
        self.declare_parameter('fusion_hz', 10)    #修正频率
        self.declare_parameter('map_frame', 'camera_init')  # 被监听的tf地图坐标
        self.declare_parameter('base_frame', 'body')       # 被监听的tf基座坐标
        self.declare_parameter('slam_hz', 200)              # 被监听的tf频率
        self.declare_parameter('odom_topic','/sick/vel')   #轮式里程计速度
        self.declare_parameter('sick_topic', '/sick/lidar')  #激光雷达点数据
        self.declare_parameter('use_sick', True)  # 是否使用点激光数据
        self.odom_topic = self.get_parameter('odom')
        self.sick_topic = self.get_parameter('sick_topic')
        self.use_sick = self.get_parameter('use_sick')
        self.publish_tf_name = self.get_parameter('publish_tf_name')
        self.fusion_hz = self.get_parameter('fusion_hz')
        # self.map_frame = self.get_parameter('map_frame')
        # self.base_frame = self.get_parameter('base_frame')
        self.odom_frame = self.get_parameter('odom_frame')
        self.tf_hz = self.get_parameter('slam_hz')
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
        self.odom_x=0.0
        self.odom_y=0.0
        self.odom_yaw=0.0
    def slam_tf_callback(self):
        transform=TransformStamped()
        map_frame = self.get_parameter('map_frame').value
        base_frame = self.get_parameter('base_frame').value
        try:
            if not self.tf_buffer.can_transform(map_frame, base_frame, rclpy.time.Time()):
                self.get_logger().warn(f"Transform from {map_frame} to {base_frame} not available")
                # return
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
        if len(self.tf_overage_x) == 0:
            return
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
        x_diff = x - self.odom_x
        y_diff = y - self.odom_y
        yaw_diff = yaw - self.odom_yaw
        if yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        elif yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi
        #发布轮式偏移的tf
        self.tf_publish("odom_transformed", self.odom_frame, x_diff, y_diff, yaw_diff)
    def odom_callback(self, msg:Vector3Stamped):
        self.odom_x = msg.vector.x
        self.odom_y = msg.vector.y
        self.odom_yaw = msg.vector.z
        self.tf_publish(self.odom_frame, self.publish_tf_name, msg.vector.x, msg.vector.y, msg.vector.z)
        #发布轮式里程计的tf
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
    rclpy.init(args=args)
    node = fusion_node_t()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy.shutdown()
        rclpy.ok()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()