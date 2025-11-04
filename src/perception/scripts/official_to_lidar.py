#!/usr/bin/env python3
"""
secondary_imu_transformer.py
功能：对已转换的IMU话题进行二次变换（应用外参矩阵）
说明：此节点应作为数据流中的第二个处理节点运行。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np

class SecondaryIMUTransformer(Node):
    """
    二次IMU转换节点
    订阅第一次转换后的话题，应用外参矩阵，发布到最终话题。
    """

    def __init__(self):
        super().__init__('secondary_imu_transformer')
        
        # 声明节点参数（可在启动时重映射）
        self.declare_parameter('input_topic', '/rsimu')
        self.declare_parameter('output_topic', '/livox/imu')
        self.declare_parameter('output_frame_id', 'imu_link_final')
        
        # 获取参数值
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.output_frame_id = self.get_parameter('output_frame_id').value
        
        # 定义外参矩阵 [根据您的实际参数配置]
        # 外参旋转矩阵 (3x3, 从第一次转换后的坐标系到最终坐标系的旋转)
        self.extrinsic_R = np.array([
            [0.000280, -0.999993, 0.003629],
            [-0.999946, -0.000242, 0.010384],
            [-0.010384, -0.003632, -0.999939]
        ], dtype=np.float64)
        
        # 外参平移向量 (3x1)
        self.extrinsic_T = np.array([0.004250, 0.004180, -0.004460], dtype=np.float64)
        
        # 配置QoS策略（与IMU数据特性匹配）
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50
        )
        
        # 创建发布器和订阅器
        self.publisher_ = self.create_publisher(Imu, output_topic, qos_profile)
        self.subscription = self.create_subscription(
            Imu,
            input_topic,
            self.imu_callback,
            qos_profile
        )
        
        self.get_logger().info(f'二次IMU转换节点已启动')
        self.get_logger().info(f'订阅: {input_topic} -> 发布: {output_topic}')
        self.get_logger().info(f'输出坐标系: {self.output_frame_id}')

    def transform_vector3(self, vector, rotation_matrix):
        """
        使用旋转矩阵变换三维矢量
        Args:
            vector: 原始矢量 [x, y, z]
            rotation_matrix: 3x3旋转矩阵
        Returns:
            变换后的矢量
        """
        return rotation_matrix @ vector  # 矩阵乘法

    def imu_callback(self, msg):
        """
        IMU数据回调函数：应用外参矩阵进行二次变换
        """
        try:
            # 创建新的IMU消息
            transformed_msg = Imu()
            
            # 保留原始时间戳，更新坐标系ID
            transformed_msg.header.stamp = msg.header.stamp
            transformed_msg.header.frame_id = self.output_frame_id
            
            # 1. 变换角速度矢量 [核心操作]
            angular_velocity_orig = np.array([
                msg.angular_velocity.x, 
                msg.angular_velocity.y, 
                msg.angular_velocity.z
            ])
            angular_velocity_transformed = self.transform_vector3(
                angular_velocity_orig, self.extrinsic_R
            )
            transformed_msg.angular_velocity.x = angular_velocity_transformed[0]
            transformed_msg.angular_velocity.y = angular_velocity_transformed[1]
            transformed_msg.angular_velocity.z = angular_velocity_transformed[2]
            
            # 2. 变换线性加速度矢量 [核心操作]
            linear_acceleration_orig = np.array([
                msg.linear_acceleration.x, 
                msg.linear_acceleration.y, 
                msg.linear_acceleration.z
            ])
            linear_acceleration_transformed = self.transform_vector3(
                linear_acceleration_orig, self.extrinsic_R
            )
            transformed_msg.linear_acceleration.x = linear_acceleration_transformed[0]
            transformed_msg.linear_acceleration.y = linear_acceleration_transformed[1]
            transformed_msg.linear_acceleration.z = linear_acceleration_transformed[2]
            
            # 3. 方向信息保持不变（标记为无效）
            transformed_msg.orientation.x = 0.0
            transformed_msg.orientation.y = 0.0
            transformed_msg.orientation.z = 0.0
            transformed_msg.orientation.w = 1.0
            transformed_msg.orientation_covariance = [-1.0] + [0.0] * 8
            
            # 4. 复制协方差矩阵（保持不变）
            transformed_msg.angular_velocity_covariance = msg.angular_velocity_covariance
            transformed_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
            
            # 发布最终转换后的消息
            self.publisher_.publish(transformed_msg)
            
        except Exception as e:
            self.get_logger().error(f'二次处理IMU数据时出错: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = SecondaryIMUTransformer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到键盘中断信号，关闭节点...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()