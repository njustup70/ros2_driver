import rclpy,json
from rclpy.node import Node
from std_msgs.msg import String

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.pub=self.create_publisher(String,'robot_state',10)
        self.tim=self.create_timer(0.1,self.timer_callback)
    def timer_callback(self):
        msg=String()
        msg.data=json.dumps({"nav_state": "ALIGNED"})
        self.pub.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")
def main(args=None):
    rclpy.init(args=args)
    node=TestNode()
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