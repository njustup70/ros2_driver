#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import numpy as np
import math
import scipy.linalg

class KalmanFilter:
    def __init__(self, pos_noise=0.001, vel_noise=0.01, accel_noise=0.1, measurement_noise=0.0001):
        """
        改进的卡尔曼滤波器 - 解决角度过零问题和速度响应问题
        
        关键改进：
        1. 分离位置/速度/加速度过程噪声
        2. 角度残差归一化处理
        3. 优化状态转移矩阵
        4. 改进初始化逻辑
        
        状态变量: [位置, 速度, 加速度]
        """
        # 初始状态转移矩阵 (后续会根据dt动态更新)
        self.F = np.eye(3)
        
        # 观测矩阵 (只能观测位置)
        self.H = np.array([[1, 0, 0]])
        
        # 过程噪声协方差 (分离位置/速度/加速度噪声)
        self.Q = np.diag([pos_noise, vel_noise, accel_noise])
        
        # 测量噪声协方差
        self.R = np.array([[measurement_noise]])
        
        # 状态协方差
        self.P = np.eye(3) * 10  # 初始较大不确定性
        
        # 初始状态
        self.x = np.zeros(3)
        
        # 滤波器初始化标志
        self.initialized = False
    
    def predict(self, dt=0.01, is_angle=False):
        self.F = np.array([
            [1, dt, 0.5 * dt**2],
            [0, 1, dt],
            [0, 0, 1]
        ])
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

        # NEW: 如果是角度状态，对位置 wrap
        if is_angle:
            self.x[0] = self.wrap_angle(self.x[0])
        return self.x

    def update(self, measurement, is_angle=False):
        if not self.initialized:
            self.x[0] = measurement
            self.initialized = True
            return self.x

        y = measurement - np.dot(self.H, self.x)

        if is_angle:
            y = self.wrap_angle(y)

        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)

        I = np.eye(self.P.shape[0])
        self.P = np.dot(I - np.dot(K, self.H), self.P)

        # NEW: 如果是角度状态，对位置 wrap
        if is_angle:
            self.x[0] = self.wrap_angle(self.x[0])
        return self.x

    def wrap_angle(self, angle):
        """更安全的 wrap"""
        wrapped = (angle + np.pi) % (2 * np.pi) - np.pi
        return wrapped
    
    def get_position(self):
        """获取估计的位置"""
        return self.x[0]
    
    def get_velocity(self):
        """获取估计的速度"""
        return self.x[1]
    
    def get_acceleration(self):
        """获取估计的加速度"""
        return self.x[2]

class SickKalmanFilter(Node):
    def __init__(self):
        super().__init__('sick_kalman_filter')
        
        # 订阅位置话题
        self.subscription = self.create_subscription(
            Twist, 
            '/sick/local',
            self.position_callback,
            1
        )
        
        # 发布估计结果
        self.vel_pub = self.create_publisher(Twist, '/odom/vel_est', 10)
        self.accel_pub = self.create_publisher(Imu, '/odom/accel_est', 10)
        
        # 初始化卡尔曼滤波器 (分离噪声参数)
        # 位置噪声小，速度噪声中等，加速度噪声大
        self.kf_x = KalmanFilter(
            pos_noise=1e-6,  # 位置噪声较小
            vel_noise=2e-5,
            accel_noise=1e-2,
            measurement_noise=25e-6
        )
        self.kf_y = KalmanFilter(
            pos_noise=1e-6,
            vel_noise=2e-5,
            accel_noise=1e-2,
            measurement_noise=25e-6
        )
        self.kf_yaw = KalmanFilter(
            pos_noise=1e-6,
            vel_noise=1e-5,
            accel_noise=0.06,
            measurement_noise=0.0001  # 角度测量噪声稍大,
        )
        
        # 时间跟踪
        self.last_time = self.get_clock().now()
        self.first_message = True
        
        self.get_logger().info('改进版卡尔曼滤波节点已启动，优化了角度过零问题和速度响应')

    def position_callback(self, msg):
        """处理位置信息的回调函数"""
        # 获取当前时间
        current_time = self.get_clock().now()
        
        # 计算时间间隔
        if self.first_message:
            self.last_time = current_time
            self.first_message = False
            return
            
        # 转换为秒
        dt = 0.01
        self.last_time = current_time
        
        if dt <= 0 or dt > 0.1:  # 确保时间间隔合理
            dt = 0.01
        # print(f'时间间隔: {dt:.4f}秒')
        # 位置数据
        x = msg.linear.x
        y = msg.linear.y
        yaw = self.normalize_angle(msg.angular.z)
        
        # 预测步骤
        self.kf_x.predict(dt)
        self.kf_y.predict(dt)
        self.kf_yaw.predict(dt,is_angle=True)
        
        # 更新步骤：对于yaw启用角度归一化处理
        self.kf_x.update(x)
        self.kf_y.update(y)
        self.kf_yaw.update(yaw, is_angle=True)
        
        # 发布估计的速度
        vel_msg = Twist()
        vel_msg.linear.x = self.kf_x.get_velocity()
        vel_msg.linear.y = self.kf_y.get_velocity()
        vel_msg.angular.z = self.kf_yaw.get_velocity()
        self.vel_pub.publish(vel_msg)
        
        # 发布估计的加速度
        accel_msg = Imu()
        accel_msg.header.stamp = current_time.to_msg()
        accel_msg.header.frame_id = 'base_link'
        
        accel_msg.linear_acceleration.x = self.kf_x.get_acceleration()
        accel_msg.linear_acceleration.y = self.kf_y.get_acceleration()
        accel_msg.angular_velocity.z = self.kf_yaw.get_acceleration()
        # accel_msg.linear.x = self.kf_x.get_acceleration()
        # accel_msg.linear.y = self.kf_y.get_acceleration()
        # accel_msg.angular.z = self.kf_yaw.get_acceleration()
        self.accel_pub.publish(accel_msg)
        
        # 调试信息
        self.get_logger().debug(f'dt={dt:.4f}s, X_vel={vel_msg.linear.x:.3f}, Y_vel={vel_msg.linear.y:.3f}, Yaw_vel={vel_msg.angular.z:.3f}')
    
    def normalize_angle(self, angle):
        """将角度归一化到[-π, π]范围"""
        # 更高效的角度归一化方法
        normalized_angle = angle % (2 * math.pi)
        if normalized_angle > math.pi:
            normalized_angle -= 2 * math.pi
        elif normalized_angle < -math.pi:
            normalized_angle += 2 * math.pi
        return normalized_angle

def main(args=None):
    rclpy.init(args=args)
    kalman_filter_node = SickKalmanFilter()
    try:
        rclpy.spin(kalman_filter_node)
    except KeyboardInterrupt:
        pass
    finally:
        kalman_filter_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()