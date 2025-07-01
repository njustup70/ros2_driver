#!/usr/bin/env python3
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
import math
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from EFK import FlexibleKalmanFilter, MovingAverageFilter, ExponentialMovingAverageFilter

class KalmanNode(Node):
    def __init__(self):
        super().__init__('kalman_node')
        self.get_logger().info("Kalman滤波器节点已启动")
        self.declare_parameter('imu_topic', '/livox/imu/normal')
        self.declare_parameter('publish_tf_name', 'base_link')
        self.declare_parameter('hz', 100)
        self.declare_parameter('kalman_model', 0)
        self.declare_parameter('map_frame', 'camera_init')  # 被监听的tf地图坐标
        self.declare_parameter('base_frame', 'body')       # 被监听的tf基座坐标
        self.declare_parameter('tf_hz', 10.0)              # 被监听的tf频率
        
        # 时间参数
        self.dt = 1.0 / self.get_parameter('hz').value
        self.last_time = self.get_clock().now()
        
        # 初始化滤波器（状态维度从9维缩减为5维：[x, y, z, w, vx, vy, omega]）
        self.kf = FlexibleKalmanFilter(dim_x=7)
        self.kf.x = np.zeros((7, 1))  # 默认初始化为0向量
        
        # 构建状态转移矩阵
        dt = self.dt
        self.B = np.zeros((7, 3))
        self.B[4, 0] = 1.0  # vx_cmd -> vx
        self.B[5, 1] = 1.0  # vy_cmd -> vy
        self.B[2, 2] = dt   # vyaw_cmd -> theta (通过积分)
        self.B[6, 2] = 1.0  # vyaw_cmd -> omega
        
        # 测量矩阵（仅保留TF测量部分）
        self.H_tf = np.zeros((4, 7))
        self.H_tf[[0, 1, 2, 3], [0, 1, 2, 3]] = 1.0  # TF测量 [px, py, z, w]
        
        # 过程噪声协方差矩阵（移除IMU相关维度）
        self.kf.Q = np.diag([0.0005, 0.0005, 0.006, 0.006, 0.01, 0.01, 0.01])
        
        # 测量噪声协方差矩阵（仅保留TF部分）
        self.R_tf = np.diag([0.001, 0.001, 0.006, 0.006])  # TF测量噪声（x,y,z,w）
        
        # 初始估计误差协方差（移除IMU相关维度）
        self.kf.P = np.diag([0.1, 0.1, 0.01, 0.01, 0.5, 0.5, 0.1])
        
        # 控制输入缓存
        self.R_cmd = np.diag([0.5, 0.5, 0.5])  # 控制输入噪声协方差
        self.H_cmd = np.zeros((3, 7))
        self.H_cmd[[0, 1, 2], [4, 5, 6]] = 1.0  # 控制输入 [vx, vy, vyaw] -> [vx, vy, omega]
        self.cmd_vel = np.zeros(3)  # 控制输入[vx, vy, vyaw]
        self.odom = np.zeros(5)     # [x, y, yaw, z, w]
        
        # 创建订阅者（移除IMU订阅）
        # self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 创建定时器
        self.tf_timer = self.create_timer(1.0 / self.get_parameter('tf_hz').value, self.tf_timer_callback)
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.vel_pub = self.create_publisher(Twist, '/vel_predict', 1)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.active = False

    def cmd_vel_callback(self, msg: Twist):
        """处理速度指令"""
        self.cmd_vel[0] = msg.linear.x
        self.cmd_vel[1] = msg.linear.y
        self.cmd_vel[2] = msg.angular.z
        z = np.array([self.cmd_vel[0], self.cmd_vel[1], self.cmd_vel[2]]).reshape(-1, 1)
        self.kf.update(z, H=self.H_cmd, R=self.R_cmd)

    def tf_timer_callback(self):
        map_tf = self.get_parameter('map_frame').value
        base_tf = self.get_parameter('base_frame').value
        if not self.tf_buffer.can_transform(map_tf, base_tf, Time()):
            return
            
        try:
            transform_temp = self.tf_buffer.lookup_transform(map_tf, base_tf, Time())
        except Exception as e:
            return
            
        # 启动激活状态
        self.active = True
        translation = transform_temp.transform.translation
        rotation = transform_temp.transform.rotation

        # 保存位置和朝向
        self.odom[0] = translation.x
        self.odom[1] = translation.y
        self.odom[2] = self.get_yaw_from_quaternion(
            rotation.x, rotation.y, rotation.z, rotation.w
        )
        self.odom[3] = rotation.z  # z坐标
        self.odom[4] = rotation.w  # 四元数w
        
        # 执行TF更新（移除IMU相关更新）
        z = np.array([self.odom[0], self.odom[1], self.odom[3], self.odom[4]]).reshape(-1, 1)
        self.kf.update(z, H=self.H_tf, R=self.R_tf)

    def timer_callback(self):
        if not self.active:
            return
            
        """执行预测步骤（移除IMU相关计算）"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # 计算当前朝向（从四元数获取）
        yaw = self.get_yaw_from_quaternion(0, 0, self.kf.x[2, 0], self.kf.x[3, 0])
        
        # 构建5维状态转移矩阵（移除IMU相关维度）
        F = np.eye(7)
        F[0, 4] = dt * np.cos(yaw)  # x = x + vx*cos(yaw)*dt - vy*sin(yaw)*dt
        F[0, 5] = -dt * np.sin(yaw)
        F[1, 4] = dt * np.sin(yaw)  # y = y + vx*sin(yaw)*dt + vy*cos(yaw)*dt
        F[1, 5] = dt * np.cos(yaw)
        F[2, 6] = 0.5 * dt       # z = z + 0.5*w*omega*dt
        F[3, 6] = -0.5 * dt      # w = w - 0.5*z*omega*dt
        
        self.kf.F = F
        self.kf.predict()

        self.publish_fused_state()
        vel = Twist()
        vel.linear.x = self.kf.x[4, 0]  # vx
        vel.linear.y = self.kf.x[5, 0]  # vy
        vel.angular.z = self.kf.x[6, 0]  # omega
        self.vel_pub.publish(vel)

    def publish_fused_state(self):
        """发布融合后的状态（移除IMU相关字段）"""
        tf_pub = TransformStamped()
        tf_pub.header.stamp = self.get_clock().now().to_msg()
        tf_pub.header.frame_id = 'odom_transform'
        tf_pub.child_frame_id = self.get_parameter('publish_tf_name').value
        
        tf_pub.transform.translation.x = self.kf.x[0, 0]
        tf_pub.transform.translation.y = self.kf.x[1, 0]
        tf_pub.transform.translation.z = 0.0
        
        tf_pub.transform.rotation.x = 0.0
        tf_pub.transform.rotation.y = 0.0
        tf_pub.transform.rotation.z = self.kf.x[2, 0]  # 四元数z
        tf_pub.transform.rotation.w = self.kf.x[3, 0]  # 四元数w
        
        # 四元数归一化
        z = self.kf.x[2, 0]
        w = self.kf.x[3, 0]
        norm = math.sqrt(z**2 + w**2)
        if norm > 0:
            tf_pub.transform.rotation.z /= norm
            tf_pub.transform.rotation.w /= norm
        else:
            tf_pub.transform.rotation.z = 0.0
            tf_pub.transform.rotation.w = 1.0
            
        self.tf_broadcaster.sendTransform(tf_pub)

    @staticmethod
    def get_yaw_from_quaternion(x, y, z, w):
        """根据四元数返回yaw偏航角"""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = KalmanNode()
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
    main()