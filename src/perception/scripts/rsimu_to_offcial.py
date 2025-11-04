#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class MinimalIMUTransformer(Node):
    def __init__(self):
        super().__init__('minimal_imu_transformer')
        # ... 参数声明、发布订阅器创建等初始化代码与之前相同 ...
        # 例如：
        self.declare_parameter('input_topic', '/rslidar_imu_data')
        self.declare_parameter('output_topic', '/imu/data_transformed')
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50
        )
        
        self.publisher_ = self.create_publisher(Imu, output_topic, qos_profile)
        self.subscription = self.create_subscription(
            Imu,
            input_topic,
            self.imu_callback,
            qos_profile
        )
        self.get_logger().info(f'极简IMU转换器已启动: {input_topic} -> {output_topic}')

    def imu_callback(self, msg):
        """极简IMU数据回调：直接对加速度和角速度取反"""
        try:
            transformed_msg = Imu()
            # 保留原始时间戳
            transformed_msg.header.stamp = msg.header.stamp
            transformed_msg.header.frame_id = 'imu_link_transformed'  # 或使用参数配置的新坐标系ID
            
            # **核心修改：直接对每个分量取反**
            # 1. 处理角速度
            transformed_msg.angular_velocity.x = msg.angular_velocity.x
            transformed_msg.angular_velocity.y = msg.angular_velocity.y
            transformed_msg.angular_velocity.z = msg.angular_velocity.z
            
            # 2. 处理线性加速度
            transformed_msg.linear_acceleration.x = -msg.linear_acceleration.x
            transformed_msg.linear_acceleration.y = -msg.linear_acceleration.y
            transformed_msg.linear_acceleration.z = -msg.linear_acceleration.z
            
            # 3. 方向信息和协方差矩阵处理保持不变
            transformed_msg.orientation.x = 0.0
            transformed_msg.orientation.y = 0.0
            transformed_msg.orientation.z = 0.0
            transformed_msg.orientation.w = 1.0
            transformed_msg.orientation_covariance = [-1.0] + [0.0] * 8
            
            transformed_msg.angular_velocity_covariance = msg.angular_velocity_covariance
            transformed_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
            
            self.publisher_.publish(transformed_msg)
            
        except Exception as e:
            self.get_logger().error(f'处理IMU数据时出错: {str(e)}')

def main(args=None):
    # ... main函数保持不变 ...
    rclpy.init(args=args)
    node = MinimalIMUTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()