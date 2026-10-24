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
        self.pause_event = threading.Event()
        self.lock = threading.Lock()
        self.frame = None
        self.frame_time = 0.0
        self.error = None
        self.source = None
        self.video_second = None
        self.video_duration = None
        self.video_fps = None
        self.end_reached = False
        self.start_second = 0.0
        self.end_second = None
        self.pause_after_first = False

    def start(self, source, start_second=0.0, end_second=None, paused=False):
        if self.thread and self.thread.is_alive():
            if source == self.source:
                return
            raise ValueError("Stop the camera before changing its source")
        self.stop_event.clear()
        self.pause_event.clear()
        self.frame, self.frame_time, self.error, self.source = None, 0.0, None, source
        self.video_second = None
        self.video_duration = None
        self.video_fps = None
        self.end_reached = False
        self.start_second = start_second
        self.end_second = end_second
        self.pause_after_first = paused
        self.thread = threading.Thread(
            target=self._run, daemon=True, name="race-camera"
        )
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        self.pause_event.clear()
        if self.thread:
            self.thread.join(timeout=5)
        if self.thread and self.thread.is_alive():
            raise ValueError("Camera did not stop; check the device before restarting")
        self.frame, self.frame_time = None, 0.0
        self.end_reached = False

    @property
    def paused(self):
        return bool(
            self.source
            and self.source.startswith("video:")
            and self.pause_event.is_set()
        )

    def pause(self):
        if (
            self.source
            and self.source.startswith("video:")
            and self.thread
            and self.thread.is_alive()
        ):
            self.pause_event.set()

    def resume(self):
        self.pause_event.clear()

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

    @staticmethod
    def video_path(filename):
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
        return path

    def _video(self, filename):
        """Replay an MP4 at its frame rate, optionally within marked race bounds."""
        path = self.video_path(filename)
        cap = cv2.VideoCapture(str(path))
        if not cap.isOpened():
            raise RuntimeError(f"Could not open replay video: {path.name}")
        try:
            fps = cap.get(cv2.CAP_PROP_FPS)
            if not 1 <= fps <= 240:
                raise RuntimeError("Replay video has no usable frame rate")
            frame_count = cap.get(cv2.CAP_PROP_FRAME_COUNT)
            self.video_fps = fps
            self.video_duration = frame_count / fps if frame_count > 0 else None
            if self.video_duration and self.start_second >= self.video_duration:
                raise RuntimeError("Race start is beyond the end of this recording")
            if self.start_second:
                cap.set(cv2.CAP_PROP_POS_FRAMES, round(self.start_second * fps))
            deadline = time.monotonic()
            while not self.stop_event.is_set():
                if self.pause_event.is_set():
                    paused_at = time.monotonic()
                    while self.pause_event.is_set() and not self.stop_event.is_set():
                        self.stop_event.wait(0.05)
                    deadline += time.monotonic() - paused_at
                    continue
                ok, frame = cap.read()
                if not ok:
                    self.end_reached = True
                    self.pause_event.set()
                    self.error = (
                        "Replay video ended; scrub to another frame or finish the race"
                    )
                    break
                frame_index = max(0, cap.get(cv2.CAP_PROP_POS_FRAMES) - 1)
                video_second = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000
                if video_second <= 0 and frame_index > 0:
                    video_second = frame_index / fps
                self._store(frame, video_second)
                if self.pause_after_first:
                    self.pause_after_first = False
                    self.pause_event.set()
                if self.end_second is not None and video_second >= self.end_second:
                    self.end_reached = True
                    self.pause_event.set()
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
