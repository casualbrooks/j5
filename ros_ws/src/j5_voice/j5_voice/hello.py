import rclpy
from rclpy.node import Node


class HelloVoice(Node):
    def __init__(self):
        super().__init__("j5_voice_hello")
        self.get_logger().info("j5_voice hello node is alive.")


def main():
    rclpy.init()
    node = HelloVoice()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
