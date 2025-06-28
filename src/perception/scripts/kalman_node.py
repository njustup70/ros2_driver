#!/usr/bin/env python3
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
import math
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from EFK import FlexibleKalmanFilter,MovingAverageFilter, ExponentialMovingAverageFilter

class KalmanNode(Node):
    def __init__(self):
        super().__init__('kalman_node')
        self.get_logger().info("Kalman滤波器节点已启动")
        self.declare_parameter('imu_topic', '/livox/imu/normal')
        self.declare_parameter('publish_tf_name', 'base_link')
        self.declare_parameter('hz',100)
        self.declare_parameter('kalman_model',0)
        
        # 时间参数
        self.dt = 1.0/self.get_parameter('hz').value
        self.last_time = self.get_clock().now()
        
        self.mf = MovingAverageFilter(window_size=10000)  # 初始化移动平均滤波器
        self.ef = ExponentialMovingAverageFilter(alpha=0.2)  # 初始化移动平均滤波器

        # 状态向量 [x, y, yaw]
        # self.kf = KalmanFilter(dim_x=8, dim_z=8)
        self.kf = FlexibleKalmanFilter(dim_x=9)
        self.kf.x = np.zeros((9, 1))  # 默认初始化为0向量
        
        # 构建状态转移矩阵
        # 状态向量: {s} [px, py, theta, vx, vy, omega,ax,ay] 直接使用theta会有突变问题
        # 状态向量: {s} [px, py, z,w, vx, vy, omega,ax,ay]
        dt = self.dt
        # 控制输入向量：{b}[vx,vy,vyaw]
        # 输入向量为命令输入
        self.B= np.zeros((9, 3))
        self.B[3, 0] = 1.0  # vx_cmd -> vx
        self.B[4, 1] = 1.0  # vy_cmd -> vy
        self.B[2, 2] = dt    # vyaw_cmd -> theta (通过积分)
        self.B[5, 2] = 1.0   # vyaw_cmd -> omega
        # self.kf.B = self.B
        
        # 测量矩阵 - 组合版本
        self.H_imu= np.zeros((3, 9))
        self.H_imu[[0,1,2],[7,8,6]] = 1.0  # IMU测量 [ax, ay, vyaw] -> [ax, ay, omega]
        self.H_tf = np.zeros((4, 9))
        self.H_tf[[0,1,2,3],[0,1,2,3]] = 1.0  # TF测量 [px, py, z,w] -> [px, py, z ,w]
        # 过程噪声协方差矩阵
        self.kf.Q = np.diag([0.001, 0.001, 0.006,0.006, 0.01, 0.01, 0.01,0.01, 0.01])
        # 测量噪声协方差矩阵
        self.R = np.diag([0.001, 0.001, 0.08, 0.4, 0.4, 0.01 ,0.001, 0.001])
        
        # 测量噪声协方差矩阵（根据传感器精度调整）
        self.R_tf = np.diag([1.0, 1.0, 0.2,0.2])  # TF测量噪声（x,y,z,w）
        self.R_imu = np.diag([1.0, 1.0, 1.0])    # IMU测量噪声（ax,ay,ayaw）
        
        # 初始估计误差协方差
        self.kf.P = np.diag([0.1, 0.1, 0.01,0.01, 0.5, 0.5, 0.1, 1.0, 1.0])
        
        # 控制输入和测量缓存
        self.R_cmd=np.diag([0.5,0.5,0.5])  # 控制输入噪声协方差
        self.H_cmd=np.zeros((3, 9))
        self.H_cmd[[0,1,2],[4,5,6]] = 1.0  # 控制输入 [vx, vy, vyaw] -> [vx, vy, omega]
        self.cmd_vel = np.zeros(3)  # 控制输入[vx, vy, vyaw]
        self.odom = np.zeros(5)     # [x, y, yaw,z,w]
        # self.odom[4]=1
        self.imu_data = [0.0,0.0,0.0]
        # self.imu_data = np.zeros(3) # [ax, ay, ayaw]
        
        # 创建订阅者
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 1)
        # self.create_subscription(TFMessage, '/tf', self.tf_callback, 1)
        self.create_subscription(Imu, self.get_parameter('imu_topic').value, self.imu_callback, 1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # 创建定时器
        self.tf_timer=self.create_timer(1.0/300.0, self.tf_timer_callback)
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.active=False
    def cmd_vel_callback(self, msg: Twist):
        """处理速度指令"""
        self.cmd_vel[0] = msg.linear.x
        self.cmd_vel[1] = msg.linear.y
        self.cmd_vel[2] = msg.angular.z
        z=np.array([self.cmd_vel[0], self.cmd_vel[1], self.cmd_vel[2]]).reshape(-1, 1)  # 简化后
        self.kf.update(z, H=self.H_cmd, R=self.R_cmd)
    def tf_timer_callback(self):
        try:
            transform_temp = self.tf_buffer.lookup_transform('laser_map', 'lio_base_link',time=Time())
        except Exception as e:
            # self.get_logger().error(f"TF lookup failed: {e}")
            return
        #启动激活状态
        self.active=True
        # 提取位置信息
        translation = transform_temp.transform.translation
        rotation = transform_temp.transform.rotation

        # 保存x, y, yaw
        self.odom[0] = translation.x
        self.odom[1] = translation.y
        self.odom[2] = self.get_yaw_from_quaternion(
            rotation.x, rotation.y, rotation.z, rotation.w
        )
        self.odom[3]= rotation.z  # 添加z坐标
        self.odom[4] = rotation.w  # 添加四元数w
        #现构造行再转化成列向量
        z=np.array([self.odom[0], self.odom[1], self.odom[3],self.odom[4]]).reshape(-1, 1)  # 简化后
        # self.kf.update(z)
        self.kf.update(z, H=self.H_tf, R=self.R_tf)
        # 执行基于TF的更新
        # self.update_tf()
    def imu_callback(self, msg: Imu):
        """处理IMU消息"""
        # print("imu_callback")
        # 提取加速度与角速度（转换为弧度）
        self.imu_data[0] = msg.linear_acceleration.x
        self.imu_data[1] = msg.linear_acceleration.y
        self.imu_data[2] = msg.angular_velocity.z  # 已经是rad/s
        yaw = self.odom[2]
        ax = (self.imu_data[0]*np.cos(yaw) - self.imu_data[1]*np.sin(yaw))
        
        ay = (self.imu_data[0]*np.sin(yaw) + self.imu_data[1]*np.cos(yaw))
        z = np.array([self.imu_data[2], self.imu_data[0], self.imu_data[1]]).reshape(-1, 1)  # 简化后
        self.kf.update(z, H=self.H_imu, R=self.R_imu)
    def timer_callback(self):
        if self.active is False:
            return
        """定时器回调 - 执行预测步骤"""
        # print("Timer callback triggered")
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # dt = self.dt  # 实际时间差
        # yaw=self.kf.x[2, 0]  # 获取当前yaw角
        yaw=self.get_yaw_from_quaternion(0,0,self.kf.x[2, 0],self.kf.x[3, 0])
        #状态量为 [px, py, z, w, vx, vy, omega, ax, ay]
        F=np.array([
            [1, 0, 0, dt*np.cos(yaw), -dt*np.sin(yaw), 0, 0, 0],
            [0, 1, 0, dt*np.sin(yaw), dt*np.cos(yaw), 0, 0, 0],
            [0, 0, 1, 0, 0, dt, 0, 0],
            [0, 0, 0, 1, 0, 0, dt, 0],
            [0, 0, 0, 0, 1, 0, 0, dt],
            [0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 1],
        ])
        F=np.eye(9)
        F[0, 4] = dt * np.cos(yaw) #x=x+vx*cos(yaw)*dt-vx*sin(yaw)*dt
        F[0, 5] = -dt * np.sin(yaw)
        F[1, 4] = dt * np.sin(yaw) #y=y+vx*sin(yaw)*dt+vx*cos(yaw)*dt
        F[1, 5] = dt * np.cos(yaw)
        F[2, 6] = dt*0.5
        F[3, 6] = dt*-0.5
        #z=z+0.5*w*omega*dt
        #w=w-0.5*z*omega*dt

        self.kf.F = F # 更新状态转移矩阵
       
        # 执行预测步骤（考虑控制输入）
        vx = self.cmd_vel[0]
        vy = self.cmd_vel[1]
        omega = self.cmd_vel[2]
        # mat_u = np.array([vx, vy, omega])
        mat_u = np.array([0,0,0])

        self.kf.predict()

        # 更新移动平均滤波器
        # self.mf.update(self.kf.x[0, 0])
        # self.mf.update(self.kf.x[1, 0])
        # self.ef.update(self.kf.x[0, 0])
        # self.ef.update(self.kf.x[1, 0])
        self.publish_fused_state()
                        
        
    def publish_fused_state(self):
        """发布融合后的状态"""
        tf_pub=TransformStamped()
        tf_pub.header.stamp = self.get_clock().now().to_msg()
        tf_pub.header.frame_id = 'odom_transform'
        tf_pub.child_frame_id = self.get_parameter('publish_tf_name').value
        tf_pub.transform.translation.x = self.kf.x[0, 0]
        tf_pub.transform.translation.y = self.kf.x[1, 0]
        tf_pub.transform.translation.z = 0.0
        tf_pub.transform.rotation.x = 0.0
        tf_pub.transform.rotation.y = 0.0
        tf_pub.transform.rotation.z=self.kf.x[2, 0]  # 使用z
        tf_pub.transform.rotation.w=self.kf.x[3, 0]  # 使用w
        # tf_pub.transform.rotation.z = math.sin(self.kf.x[2, 0] / 2.0)
        # tf_pub.transform.rotation.w = math.cos(self.kf.x[2, 0] / 2.0)
        # if tf_pub.transform.rotation
        if tf_pub.transform.rotation.z==0.0 and tf_pub.transform.rotation.w==0.0:
            # self.get_logger().warn("Quaternion is zero, setting to default")
            tf_pub.transform.rotation.z = 0.0
            tf_pub.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf_pub)
        
        # 这里可以添加发布融合后状态的代码
        #构造新的tf
        # self.get_logger().debug(f"Fused State: {fused_state.flatten()}")
        
    @staticmethod
    def get_yaw_from_quaternion(x, y, z, w):
        """根据四元数返回yaw偏航角"""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw    
def main(args=None):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    rclpy.init(args=args)
    node = KalmanNode()
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