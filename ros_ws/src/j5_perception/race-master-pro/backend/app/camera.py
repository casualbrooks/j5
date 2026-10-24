"""One camera owner, with USB and ROS Image adapters sharing the same frames."""

import threading
import time
from pathlib import Path

import cv2
import numpy as np

from app import database


class Camera:
    def __init__(self):
        self.thread = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.frame = None
        self.frame_time = 0.0
        self.error = None
        self.source = None
        self.video_second = None

    def start(self, source):
        if self.thread and self.thread.is_alive():
            if source == self.source:
                return
            raise ValueError("Stop the camera before changing its source")
        self.stop_event.clear()
        self.frame, self.frame_time, self.error, self.source = None, 0.0, None, source
        self.video_second = None
        self.thread = threading.Thread(
            target=self._run, daemon=True, name="race-camera"
        )
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=5)
        if self.thread and self.thread.is_alive():
            raise ValueError("Camera did not stop; check the device before restarting")
        self.frame, self.frame_time = None, 0.0

    def latest(self):
        with self.lock:
            return (None if self.frame is None else self.frame.copy()), self.frame_time

    def _store(self, frame, video_second=None):
        with self.lock:
            self.frame = frame
            self.frame_time = time.monotonic()
            self.video_second = video_second
            self.error = None

    def _run(self):
        try:
            if self.source.startswith("ros:"):
                self._ros(self.source[4:])
            elif self.source.startswith("video:"):
                self._video(self.source[6:])
            else:
                self._usb(self.source)
        except Exception as exc:
            self.error = str(exc)

    def _usb(self, source):
        cap = cv2.VideoCapture(int(source) if source.isdigit() else source)
        try:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)
            if not cap.isOpened():
                raise RuntimeError(
                    "Camera could not open. Check USB connection, video-group permissions, and competing camera processes."
                )
            while not self.stop_event.is_set():
                ok, frame = cap.read()
                if not ok:
                    raise RuntimeError("Camera stopped providing frames")
                self._store(frame)
        finally:
            cap.release()

    def _video(self, filename):
        """Replay a managed upload or an existing Pi recording at its frame rate."""
        path = Path(filename)
        if not path.is_absolute():
            path = Path(database.DB_PATH).parent / "videos" / path
        path = path.resolve()
        allowed = {
            Path(database.DB_PATH).parent.joinpath("videos").resolve(),
            Path("/home/racetime"),
        }
        if path.parent not in allowed or path.suffix.lower() != ".mp4":
            raise ValueError(
                "Replay videos must be uploaded or stored in /home/racetime"
            )
        cap = cv2.VideoCapture(str(path))
        if not cap.isOpened():
            raise RuntimeError(f"Could not open replay video: {path.name}")
        try:
            fps = cap.get(cv2.CAP_PROP_FPS)
            if not 1 <= fps <= 240:
                raise RuntimeError("Replay video has no usable frame rate")
            deadline = time.monotonic()
            while not self.stop_event.is_set():
                ok, frame = cap.read()
                if not ok:
                    self.error = "Replay video finished; reconnect to restart it"
                    break
                self._store(frame, cap.get(cv2.CAP_PROP_POS_MSEC) / 1000)
                deadline += 1 / fps
                self.stop_event.wait(max(0, deadline - time.monotonic()))
        finally:
            cap.release()

    def _ros(self, topic):
        import rclpy
        from rclpy.context import Context
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image

        context = Context()
        rclpy.init(context=context)
        node = rclpy.create_node("race_manager_camera", context=context)

        def receive(msg):
            if msg.encoding not in ("rgb8", "bgr8"):
                self.error = f"Unsupported ROS image encoding: {msg.encoding}; publish rgb8 or bgr8"
                return
            rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
            frame = rows[:, : msg.width * 3].reshape(msg.height, msg.width, 3).copy()
            if msg.encoding == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            self._store(frame)

        node.create_subscription(Image, topic, receive, qos_profile_sensor_data)
        try:
            while not self.stop_event.is_set():
                rclpy.spin_once(node, timeout_sec=0.1)
        finally:
            node.destroy_node()
            context.shutdown()
