#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import math
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

        self.sub = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 1)
        self.pub = self.create_publisher(Imu, self.output_topic, 1)

        self.gravity = 9.81
        self.sample_count = 0
        self.acc_norm_sum = 0.0
        self.needs_conversion = False

        self.get_logger().info("IMU 重力单位修正节点已启动")

    def imu_callback(self, msg: Imu):
        try:
            acc = msg.linear_acceleration

            # 不用 numpy，改纯 Python
            xsq = acc.x * acc.x
            ysq = acc.y * acc.y
            zsq = acc.z * acc.z

            # 检查是否都是有限数
            if not all(math.isfinite(v) for v in [acc.x, acc.y, acc.z]):
                self.get_logger().warn(f"收到非法加速度值: x={acc.x}, y={acc.y}, z={acc.z}")
                return

            norm = math.sqrt(xsq + ysq + zsq)

            if not math.isfinite(norm):
                self.get_logger().warn(f"norm 非法: {norm}")
                return

            if self.sample_count < 10:
                self.acc_norm_sum += norm
                self.sample_count += 1
            elif self.sample_count == 10:
                avg_norm = self.acc_norm_sum / 10
                if avg_norm < 5:
                    self.get_logger().warn(f"检测到非标准单位 (平均={avg_norm:.2f})，启用转换")
                    self.needs_conversion = True
                else:
                    self.get_logger().info(f"单位正常 (平均={avg_norm:.2f})")
                self.sample_count += 1

            if self.needs_conversion:
                msg.linear_acceleration.x *= self.gravity
                msg.linear_acceleration.y *= self.gravity
                msg.linear_acceleration.z *= self.gravity

            self.pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"IMU 回调异常: {e}")

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
