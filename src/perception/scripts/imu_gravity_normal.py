#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class ImuGravityFixer(Node):
    def __init__(self):
        super().__init__('imu_gravity_fixer')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('imu_topic', '/imu_raw'),
                ('output_topic', '/imu_fixed')
            ]
        )

        self.imu_topic = self.get_parameter('imu_topic').value
        self.output_topic = self.get_parameter('output_topic').value

        self.sub = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)
        self.pub = self.create_publisher(Imu, self.output_topic, 10)

        self.gravity = 9.81
        self.sample_count = 0
        self.acc_norm_sum = 0.0
        self.needs_conversion = False

        self.get_logger().info("IMU 重力单位修正节点已启动")

    def imu_callback(self, msg: Imu):
        acc = msg.linear_acceleration
        norm = np.linalg.norm([acc.x, acc.y, acc.z])

        # 前10帧用于判断是否单位为g
        if self.sample_count < 10:
            self.acc_norm_sum += norm
            self.sample_count += 1
            return
        elif self.sample_count == 10:
            avg_norm = self.acc_norm_sum / 10
            if avg_norm < 5:  # 单位可能是 g
                self.get_logger().warn(f"检测到非标准加速度单位 (平均={avg_norm:.2f})，将进行单位转换")
                self.needs_conversion = True
            else:
                self.get_logger().info(f"加速度单位正常 (平均={avg_norm:.2f})")
            self.sample_count += 1  # 确保只判断一次

        # 转换加速度单位（如果需要）
        if self.needs_conversion:
            msg.linear_acceleration.x *= self.gravity
            msg.linear_acceleration.y *= self.gravity
            msg.linear_acceleration.z *= self.gravity

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuGravityFixer()
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
