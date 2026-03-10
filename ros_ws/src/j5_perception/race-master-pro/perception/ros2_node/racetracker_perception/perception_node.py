"""
ROS2 Perception Node — Subscribes to camera topics, runs AI inference, publishes detections.
Requires ROS2 (Humble or later) and rclpy installed.

Usage:
  ros2 run racetracker_perception_stack perception_node
"""

import sys

try:
    import rclpy
    from rclpy.node import Node

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("ROS2 (rclpy) not available. Use standalone mode instead.")
    print("To install: follow docs/ROS2_INTEGRATION.md")


if ROS2_AVAILABLE:
    from sensor_msgs.msg import Image, CameraInfo

    class RaceMasterPerceptionNode(Node):
        """ROS2 node that subscribes to camera image topics and publishes racer detections."""

        def __init__(self):
            super().__init__("racetracker_perception")

            # Declare parameters
            self.declare_parameter("camera_topics", ["/camera/cam1/image_raw"])
            self.declare_parameter("model_path", "perception/models/yolov8n.pt")
            self.declare_parameter("confidence_threshold", 0.7)
            self.declare_parameter(
                "ws_url", "ws://localhost:8080/ws?client_type=cv_system"
            )

            # Get parameters
            camera_topics = (
                self.get_parameter("camera_topics")
                .get_parameter_value()
                .string_array_value
            )
            self.model_path = (
                self.get_parameter("model_path").get_parameter_value().string_value
            )
            self.confidence_threshold = (
                self.get_parameter("confidence_threshold")
                .get_parameter_value()
                .double_value
            )

            # Create subscribers for each camera topic
            self.subscriptions = []
            for topic in camera_topics:
                sub = self.create_subscription(
                    Image,
                    topic,
                    lambda msg, t=topic: self.image_callback(msg, t),
                    10,
                )
                self.subscriptions.append(sub)
                self.get_logger().info(f"Subscribed to camera topic: {topic}")

            self.get_logger().info("Race Master Pro — Perception Node started")
            self.get_logger().info(f"Model: {self.model_path}")
            self.get_logger().info(f"Confidence threshold: {self.confidence_threshold}")

        def image_callback(self, msg: Image, topic: str):
            """Process an incoming camera image."""
            # TODO: Convert ROS Image to numpy array
            # TODO: Run through AI detection pipeline
            # TODO: Publish detections via WebSocket bridge
            self.get_logger().debug(
                f"Received frame from {topic}: {msg.width}x{msg.height}"
            )

    def main(args=None):
        rclpy.init(args=args)
        node = RaceMasterPerceptionNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

else:

    def main(args=None):
        print("ERROR: ROS2 is not available.")
        print("Please install ROS2 Humble or later, or use standalone mode:")
        print("  python -m perception.standalone.standalone_runner")
        sys.exit(1)


if __name__ == "__main__":
    main()
