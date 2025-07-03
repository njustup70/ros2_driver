#!/usr/bin/env python3
import docker,subprocess,json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
class fuck_slam_t(Node):
    def __init__(self):
        super().__init__('fuck_slam')
        self.declare_parameter('docker_name','fastlio2_container')
        self.docker_name=self.get_parameter('docker_name').get_parameter_value().string_value
        self.robot_state_sub=self.create_subscription(String,'robot_state',self.robot_service,1)
        try:
            result = subprocess.check_output(
            ["sudo", "docker", "ps","-a"],
            stderr=subprocess.STDOUT
        ).decode()
            print(result)
        except subprocess.CalledProcessError as e:
            print(e)
            # print("无法检测端口:", e.output.decode(), file=sys.stderr)
            return False
        try:
            result=subprocess.check_output(
            ["sudo","docker","start",self.docker_name],
            stderr=subprocess.STDOUT
        ).decode()
        except subprocess.CalledProcessError as e:
            print(e)
    def robot_service(self,msg:String):
        json_data= json.loads(msg.data)
        if "reset_slam" in json_data:
            if json_data["reset_slam"]:
                try:
                    result = subprocess.check_output(
                        ["sudo", "docker", "stop", self.docker_name],
                        stderr=subprocess.STDOUT
                    ).decode()
                    print(result)
                except subprocess.CalledProcessError as e:
                    print(e.output.decode())
                print("SLAM重置中...")
                try:
                    result = subprocess.check_output(
                        ["sudo", "docker", "start", self.docker_name],
                        stderr=subprocess.STDOUT
                    ).decode()
                    print(result)
                except subprocess.CalledProcessError as e:
                    print(e.output.decode())
                self.get_logger().info("SLAM重置成功")
def main(args=None):
    rclpy.init(args=args)
    node=fuck_slam_t()
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