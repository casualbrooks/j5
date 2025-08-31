import rclpy
from rclpy.node import Node


class HelloVoice(Node):
    def __init__(self):
        super().__init__("j5_voice_hello")
        self.get_logger().info("j5_voice hello node is alive.")


def main():
    rclpy.init()
    node = HelloVoice()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
