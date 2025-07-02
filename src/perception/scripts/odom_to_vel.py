#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import math
import scipy.linalg

class KalmanFilter:
    def __init__(self, process_noise=0.1, measurement_noise=0.01):
        """
        卡尔曼滤波器实现 - 针对位置、速度、加速度的估计
        
        状态变量: [位置, 速度, 加速度]
        """
        # 状态转移矩阵 (假设匀速运动模型)
        self.F = np.array([
            [1, 0.1, 0.005],
            [0, 1, 0.1],
            [0, 0, 1]
        ])
        
        # 观测矩阵 (只能观测位置)
        self.H = np.array([[1, 0, 0]])
        
        # 过程噪声协方差 (假设加速度噪声)
        self.Q = np.eye(3) * process_noise
        
        # 测量噪声协方差
        self.R = np.array([[measurement_noise]])
        
        # 状态协方差
        self.P = np.eye(3)
        
        # 初始状态
        self.x = np.zeros(3)
    
    def predict(self, dt=0.01):
        """
        预测步骤 - 基于物理模型预测下一时刻状态
        """
        # 更新状态转移矩阵以适应实际时间间隔
        self.F = np.array([
            [1, dt, 0.5*dt**2],
            [0, 1, dt],
            [0, 0, 1]
        ])
        
        # 预测状态
        self.x = np.dot(self.F, self.x)
        
        # 预测协方差
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x
    
    def update(self, measurement):
        """
        更新步骤 - 使用新的测量值修正预测
        """
        # 计算卡尔曼增益
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        
        # 更新状态估计
        y = measurement - np.dot(self.H, self.x)
        self.x = self.x + np.dot(K, y)
        
        # 更新协方差估计
        I = np.eye(self.P.shape[0])
        self.P = np.dot(I - np.dot(K, self.H), self.P)
        
        return self.x
    
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
            10
        )
        
        # 发布估计结果
        self.vel_pub = self.create_publisher(Twist, '/sick/vel_est', 10)
        self.accel_pub = self.create_publisher(Twist, '/sick/accel_est', 10)
        
        # 初始化卡尔曼滤波器 (x, y, yaw方向)
        self.kf_x = KalmanFilter(process_noise=0.1, measurement_noise=0.0001)
        self.kf_y = KalmanFilter(process_noise=0.1, measurement_noise=0.0001)
        self.kf_yaw = KalmanFilter(process_noise=0.1, measurement_noise=0.0001)
        
        # 时间跟踪
        self.last_time = self.get_clock().now()
        self.first_message = True
        
        self.get_logger().info('卡尔曼滤波节点已启动，正在等待位置数据...')

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
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time
        
        # 位置数据
        x = msg.linear.x
        y = msg.linear.y
        yaw = self.normalize_angle(msg.angular.z)
        
        # 预测步骤
        self.kf_x.predict(dt)
        self.kf_y.predict(dt)
        self.kf_yaw.predict(dt)
        
        # 更新步骤
        self.kf_x.update(x)
        self.kf_y.update(y)
        self.kf_yaw.update(yaw)
        
        # 发布估计的速度
        vel_msg = Twist()
        vel_msg.linear.x = self.kf_x.get_velocity()
        vel_msg.linear.y = self.kf_y.get_velocity()
        vel_msg.angular.z = self.kf_yaw.get_velocity()
        self.vel_pub.publish(vel_msg)
        
        # 发布估计的加速度
        accel_msg = Twist()
        accel_msg.linear.x = self.kf_x.get_acceleration()
        accel_msg.linear.y = self.kf_y.get_acceleration()
        accel_msg.angular.z = self.kf_yaw.get_acceleration()
        self.accel_pub.publish(accel_msg)
    
    def normalize_angle(self, angle):
        """将角度归一化到[-π, π]范围"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

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