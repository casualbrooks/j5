#!/usr/bin/env python3
"""ROS2 camera publisher with optional MJPEG preview server for Integration tab.

Publishes frames from a V4L2 camera as sensor_msgs/msg/Image on a ROS topic and,
when enabled, serves a browser preview at:
  - /           (simple HTML page)
  - /stream.mjpg (MJPEG stream)
"""

from __future__ import annotations

import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from socketserver import ThreadingMixIn

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image


class _ThreadedHTTPServer(ThreadingMixIn, ThreadingHTTPServer):
    daemon_threads = True


class CameraPublisher(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("camera_ros_publisher")
        self.args = args

        qos = QoSProfile(depth=args.qos_depth)
        self.publisher = self.create_publisher(Image, args.topic, qos)

        self.capture = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
        if not self.capture.isOpened():
            raise RuntimeError(f"Unable to open camera device: {args.device}")

        self.capture.set(
            cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.pixel_format)
        )
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        self.capture.set(cv2.CAP_PROP_FPS, args.fps)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self._latest_frame = None
        self._latest_jpeg = None
        self._frame_lock = threading.Lock()
        self._frames_published = 0
        self._start_time = time.time()

        actual_w = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.capture.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f"Camera opened: {args.device} @ {actual_w}x{actual_h} {actual_fps:.2f}fps ({args.pixel_format})"
        )
        self.get_logger().info(f"Publishing ROS images to: {args.topic}")

        self.timer = self.create_timer(1.0 / float(args.fps), self._tick)

        self._http_server = None
        self._http_thread = None
        if args.serve_preview:
            self._start_preview_server()

    def _tick(self):
        ok, frame = self.capture.read()
        if not ok:
            self.get_logger().warning("Camera read failed")
            return

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.frame_id
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = frame.tobytes()
        self.publisher.publish(msg)

        if self.args.serve_preview:
            ok_jpg, enc = cv2.imencode(
                ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.args.jpeg_quality]
            )
            if ok_jpg:
                with self._frame_lock:
                    self._latest_frame = frame
                    self._latest_jpeg = enc.tobytes()

        self._frames_published += 1
        if self._frames_published % int(max(self.args.fps, 5) * 5) == 0:
            elapsed = max(time.time() - self._start_time, 1e-6)
            rate = self._frames_published / elapsed
            self.get_logger().info(
                f"Published {self._frames_published} frames ({rate:.2f} fps)"
            )

    def _start_preview_server(self):
        publisher = self

        class Handler(BaseHTTPRequestHandler):
            def _send_index(self):
                body = (
                    "<html><body style='background:#111;color:#eee;font-family:sans-serif'>"
                    "<h3>Camera Preview</h3><img src='/stream.mjpg' "
                    "style='max-width:95vw;border:1px solid #555'/>"
                    "</body></html>"
                ).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _send_stream(self):
                boundary = b"frame"
                self.send_response(200)
                self.send_header("Age", "0")
                self.send_header("Cache-Control", "no-cache, private")
                self.send_header("Pragma", "no-cache")
                self.send_header(
                    "Content-Type", "multipart/x-mixed-replace; boundary=frame"
                )
                self.end_headers()

                while True:
                    with publisher._frame_lock:
                        jpg = publisher._latest_jpeg
                    if jpg is None:
                        time.sleep(0.05)
                        continue
                    try:
                        self.wfile.write(b"--" + boundary + b"\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(
                            f"Content-Length: {len(jpg)}\r\n\r\n".encode("utf-8")
                        )
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")
                        time.sleep(1.0 / max(1.0, publisher.args.preview_fps))
                    except BrokenPipeError:
                        return
                    except ConnectionResetError:
                        return

            def do_GET(self):  # noqa: N802
                if self.path == "/" or self.path.startswith("/index"):
                    self._send_index()
                elif self.path.startswith("/stream.mjpg"):
                    self._send_stream()
                else:
                    self.send_response(404)
                    self.end_headers()

            def log_message(self, fmt, *args):
                return

        self._http_server = _ThreadedHTTPServer(
            (self.args.preview_host, self.args.preview_port), Handler
        )
        self._http_thread = threading.Thread(
            target=self._http_server.serve_forever, daemon=True
        )
        self._http_thread.start()
        self.get_logger().info(
            f"Preview server running at http://{self.args.preview_host}:{self.args.preview_port}/stream.mjpg"
        )

    def shutdown(self):
        if self._http_server is not None:
            self._http_server.shutdown()
            self._http_server.server_close()
        if self.capture is not None:
            self.capture.release()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="ROS2 camera publisher + optional preview server"
    )
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--topic", default="/camera/cam1/image_raw")
    parser.add_argument("--frame-id", default="cam1")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--pixel-format", default="MJPG", choices=["MJPG", "YUYV"])
    parser.add_argument("--qos-depth", type=int, default=5)

    parser.add_argument("--serve-preview", action="store_true")
    parser.add_argument("--preview-host", default="0.0.0.0")
    parser.add_argument("--preview-port", type=int, default=8091)
    parser.add_argument("--preview-fps", type=float, default=12.0)
    parser.add_argument("--jpeg-quality", type=int, default=70)

    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = CameraPublisher(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
