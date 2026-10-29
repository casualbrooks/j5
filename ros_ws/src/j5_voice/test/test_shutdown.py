from unittest.mock import patch

import rclpy
from j5_voice.hello import main


def test_interrupt_shuts_down_ros_context():
    with patch("j5_voice.hello.rclpy.spin", side_effect=KeyboardInterrupt):
        main()
    assert not rclpy.ok()


def test_already_shutdown_context_is_safe():
    def stop(_node):
        rclpy.shutdown()

    with patch("j5_voice.hello.rclpy.spin", side_effect=stop):
        main()
    assert not rclpy.ok()
