import rclpy
from rclpy.node import Node

def main():
    rclpy.init()
    node = Node('test_node')
    node.get_logger().info('Hello, ROS 2!')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

