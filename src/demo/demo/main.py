import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class HelloWorldPublisher(Node):
    def __init__(self):
        super().__init__('hello_world_publisher')
        self.publisher_ = self.create_publisher(String, 'hello_world_topic', 10)
        self.timer_ = self.create_timer(1.0, self.publish_hello_world)
        self.get_logger().info('Hello World Publisher has been started')

    def publish_hello_world(self):
        msg = String()
        msg.data = 'Hello, World!'
        self.get_logger().info('Publishing message')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    hello_world_publisher = HelloWorldPublisher()
    rclpy.spin(hello_world_publisher)
    hello_world_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()