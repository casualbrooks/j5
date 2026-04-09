"""
ROS2 Perception Node — Subscribes to camera topics, runs AI inference, publishes detections.
Requires ROS2 (Humble or later) and rclpy installed.

Usage:
  ros2 run racetracker_perception perception_node
"""

import sys
import threading
import asyncio
import json
import time
import queue
from collections import OrderedDict
from dataclasses import dataclass
from pathlib import Path

try:
    import rclpy
    from rclpy.node import Node

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("ROS2 (rclpy) not available. Use standalone mode instead.")
    print("To install: follow docs/ROS2_INTEGRATION.md")


if ROS2_AVAILABLE:
    from sensor_msgs.msg import Image

    try:
        import websockets
    except ImportError:
        websockets = None  # type: ignore

    try:
        import cv2
        import numpy as np
        from cv_bridge import CvBridge
    except ImportError:
        cv2 = None  # type: ignore
        np = None  # type: ignore
        CvBridge = None  # type: ignore

    try:
        from ultralytics import YOLO
    except ImportError:
        YOLO = None  # type: ignore

    @dataclass
    class TrackerState:
        centroid: tuple[float, float]
        disappeared: int = 0

    class SimpleCentroidTracker:
        """Lightweight centroid tracker for moving objects."""

        def __init__(self, max_disappeared: int = 8, max_distance: float = 90.0):
            self.max_disappeared = max_disappeared
            self.max_distance = max_distance
            self.next_object_id = 1
            self.objects: OrderedDict[int, TrackerState] = OrderedDict()

        def _register(self, centroid: tuple[float, float]):
            self.objects[self.next_object_id] = TrackerState(centroid=centroid)
            self.next_object_id += 1

        def _deregister(self, object_id: int):
            self.objects.pop(object_id, None)

        def update(
            self, centroids: list[tuple[float, float]]
        ) -> OrderedDict[int, TrackerState]:
            if np is None:
                return self.objects
            if not centroids:
                stale_ids: list[int] = []
                for object_id, state in self.objects.items():
                    state.disappeared += 1
                    if state.disappeared > self.max_disappeared:
                        stale_ids.append(object_id)
                for object_id in stale_ids:
                    self._deregister(object_id)
                return self.objects

            if not self.objects:
                for centroid in centroids:
                    self._register(centroid)
                return self.objects

            object_items = list(self.objects.items())
            object_ids = [item[0] for item in object_items]
            existing = np.array(
                [item[1].centroid for item in object_items], dtype=float
            )
            incoming = np.array(centroids, dtype=float)
            distances = np.linalg.norm(existing[:, None] - incoming[None, :], axis=2)

            rows = distances.min(axis=1).argsort()
            used_rows: set[int] = set()
            used_cols: set[int] = set()

            for row in rows:
                if row in used_rows:
                    continue
                for col in np.argsort(distances[row]):
                    col = int(col)
                    if col in used_cols:
                        continue
                    if float(distances[row, col]) > self.max_distance:
                        continue
                    object_id = object_ids[row]
                    self.objects[object_id].centroid = centroids[col]
                    self.objects[object_id].disappeared = 0
                    used_rows.add(row)
                    used_cols.add(col)
                    break

            for row, object_id in enumerate(object_ids):
                if row in used_rows:
                    continue
                self.objects[object_id].disappeared += 1
                if self.objects[object_id].disappeared > self.max_disappeared:
                    self._deregister(object_id)

            for col, centroid in enumerate(centroids):
                if col not in used_cols:
                    self._register(centroid)
            return self.objects

    class RaceMasterPerceptionNode(Node):
        """ROS2 node that subscribes to camera image topics and publishes racer detections."""

        def __init__(self):
            super().__init__("racetracker_perception")

            # Declare parameters
            self.declare_parameter("camera_topics", ["/camera/cam1/image_raw"])
            self.declare_parameter("auto_discover_camera_topics", True)
            self.declare_parameter("use_yolo", True)
            self.declare_parameter("fallback_to_motion_tracking", True)
            self.declare_parameter("yolo_target_classes", [])
            self.declare_parameter("yolo_min_box_area", 0.0)
            self.declare_parameter("motion_min_area", 250.0)
            self.declare_parameter("motion_min_box_size", 10.0)
            self.declare_parameter("model_path", "perception/models/yolov8n.pt")
            self.declare_parameter("confidence_threshold", 0.5)
            self.declare_parameter(
                "ws_url", "ws://localhost:8080/ws?client_type=cv_system"
            )

            # Get parameters
            camera_topics = (
                self.get_parameter("camera_topics")
                .get_parameter_value()
                .string_array_value
            )
            self.auto_discover_camera_topics = (
                self.get_parameter("auto_discover_camera_topics")
                .get_parameter_value()
                .bool_value
            )
            self.use_yolo = (
                self.get_parameter("use_yolo").get_parameter_value().bool_value
            )
            self.fallback_to_motion_tracking = (
                self.get_parameter("fallback_to_motion_tracking")
                .get_parameter_value()
                .bool_value
            )
            self.yolo_target_classes = {
                int(value)
                for value in self.get_parameter("yolo_target_classes")
                .get_parameter_value()
                .integer_array_value
            }
            self.yolo_min_box_area = (
                self.get_parameter("yolo_min_box_area")
                .get_parameter_value()
                .double_value
            )
            self.motion_min_area = (
                self.get_parameter("motion_min_area").get_parameter_value().double_value
            )
            self.motion_min_box_size = (
                self.get_parameter("motion_min_box_size")
                .get_parameter_value()
                .double_value
            )
            self.model_path = (
                self.get_parameter("model_path").get_parameter_value().string_value
            )
            self.confidence_threshold = (
                self.get_parameter("confidence_threshold")
                .get_parameter_value()
                .double_value
            )
            self.ws_url = (
                self.get_parameter("ws_url").get_parameter_value().string_value
            )
            self._ws_stop = threading.Event()
            self._ws_thread = None
            self._detection_queue: queue.Queue[dict] = queue.Queue(maxsize=128)
            self._bridger = CvBridge() if CvBridge is not None else None
            self._trackers: dict[str, SimpleCentroidTracker] = {}
            self._background_models: dict[str, object] = {}
            self._image_subscription_topics: set[str] = set()
            self._configured_camera_topics: set[str] = {
                topic.strip() for topic in camera_topics if topic.strip()
            }
            self._yolo_model = None
            self._frames_seen = 0
            self._yolo_candidate_frames = 0
            self._motion_candidate_frames = 0
            self._detections_sent = 0
            self._load_yolo_model()

            # Create subscribers for each configured camera topic
            self.image_subscriptions = []
            for topic in camera_topics:
                self._subscribe_to_camera_topic(topic, announce_reason="configured")
            self._topic_refresh_timer = self.create_timer(
                5.0, self._refresh_camera_subscriptions
            )
            self._health_timer = self.create_timer(10.0, self._log_detection_health)

            self.get_logger().info("Race Master Pro — Perception Node started")
            self.get_logger().info(f"Model: {self.model_path}")
            self.get_logger().info(f"Confidence threshold: {self.confidence_threshold}")
            self.get_logger().info(f"WebSocket target: {self.ws_url}")
            self.get_logger().info(
                f"Auto-discover camera topics: {self.auto_discover_camera_topics}"
            )
            self.get_logger().info(f"YOLO enabled: {self.use_yolo}")
            self.get_logger().info(
                f"Motion fallback enabled: {self.fallback_to_motion_tracking}"
            )
            self.get_logger().info(
                "YOLO target classes: "
                + (
                    ",".join(str(item) for item in sorted(self.yolo_target_classes))
                    if self.yolo_target_classes
                    else "all"
                )
            )
            if cv2 is None or np is None:
                self.get_logger().warning(
                    "OpenCV/Numpy not available. Install python3-opencv + numpy for motion tracking detections."
                )
            if self._bridger is None:
                self.get_logger().warning(
                    "cv_bridge not available. Install ROS cv_bridge to convert Image frames."
                )
            self._start_ws_bridge()
            self._refresh_camera_subscriptions()

        def _log_detection_health(self):
            if self._frames_seen == 0:
                return
            mode = "yolo" if self._yolo_model is not None else "motion-only"
            self.get_logger().info(
                "Detection health: "
                f"mode={mode}, frames={self._frames_seen}, "
                f"yolo_frames={self._yolo_candidate_frames}, "
                f"motion_frames={self._motion_candidate_frames}, "
                f"detections_sent={self._detections_sent}"
            )

        def _load_yolo_model(self):
            if not self.use_yolo:
                return
            if YOLO is None:
                self.get_logger().warning(
                    "ultralytics is not installed; YOLO inference disabled."
                )
                return
            model_path = Path(self.model_path)
            try:
                # Allow both local paths and Ultralytics model identifiers
                # (e.g. "yolov8n.pt"), which may download/resolve at runtime.
                self._yolo_model = YOLO(self.model_path)
                if model_path.exists():
                    self.get_logger().info(f"YOLO model loaded from {model_path}")
                else:
                    self.get_logger().info(
                        f"YOLO model loaded using identifier '{self.model_path}'"
                    )
            except Exception as exc:
                self.get_logger().warning(
                    f"Failed to load YOLO model '{self.model_path}': {exc}"
                )
                self._yolo_model = None

        def _subscribe_to_camera_topic(
            self, topic: str, announce_reason: str = "auto-discovered"
        ):
            clean_topic = topic.strip()
            if not clean_topic or clean_topic in self._image_subscription_topics:
                return
            sub = self.create_subscription(
                Image,
                clean_topic,
                lambda msg, t=clean_topic: self.image_callback(msg, t),
                10,
            )
            self.image_subscriptions.append(sub)
            self._image_subscription_topics.add(clean_topic)
            self._trackers[clean_topic] = SimpleCentroidTracker()
            if cv2 is not None:
                self._background_models[clean_topic] = (
                    cv2.createBackgroundSubtractorMOG2(
                        history=500, varThreshold=24, detectShadows=False
                    )
                )
            self.get_logger().info(
                f"Subscribed to camera topic: {clean_topic} ({announce_reason})"
            )

        def _refresh_camera_subscriptions(self):
            active_topics = {
                name
                for name, topic_types in self.get_topic_names_and_types()
                if "sensor_msgs/msg/Image" in topic_types
            }
            if self.auto_discover_camera_topics:
                known_topics = set(self._image_subscription_topics)
                for topic in sorted(active_topics - known_topics):
                    self._subscribe_to_camera_topic(
                        topic, announce_reason="auto-discovered"
                    )
            configured_with_publishers = self._configured_camera_topics & active_topics
            if (
                self._configured_camera_topics
                and active_topics
                and not configured_with_publishers
            ):
                sample_topics = ", ".join(sorted(active_topics)[:5])
                configured_topics = ", ".join(sorted(self._configured_camera_topics))
                self.get_logger().warning(
                    "No active image publishers on configured perception camera topics. "
                    f"Configured topics: {configured_topics}. "
                    f"Available image topics: {sample_topics}"
                )

        def _start_ws_bridge(self):
            if websockets is None:
                self.get_logger().warning(
                    "python package 'websockets' is not installed; cv_system websocket connection is disabled."
                )
                return
            self._ws_thread = threading.Thread(target=self._ws_worker, daemon=True)
            self._ws_thread.start()

        def _ws_worker(self):
            asyncio.run(self._ws_worker_async())

        async def _ws_worker_async(self):
            while not self._ws_stop.is_set():
                try:
                    async with websockets.connect(self.ws_url) as ws:
                        self.get_logger().info(
                            "Connected to backend websocket as cv_system client."
                        )
                        last_ping = 0.0
                        while not self._ws_stop.is_set():
                            now = time.time()
                            if now - last_ping >= 10.0:
                                await ws.send(
                                    json.dumps(
                                        {
                                            "type": "ping",
                                            "timestamp": now,
                                        }
                                    )
                                )
                                last_ping = now
                            try:
                                payload = self._detection_queue.get_nowait()
                            except queue.Empty:
                                await asyncio.sleep(0.03)
                                continue
                            await ws.send(
                                json.dumps(
                                    {
                                        "type": "visionDetection",
                                        "data": payload,
                                        "timestamp": now,
                                    }
                                )
                            )
                except Exception as exc:
                    self.get_logger().warning(
                        f"WebSocket bridge disconnected ({exc}); retrying in 3s."
                    )
                    await asyncio.sleep(3.0)

        def _motion_candidates(self, frame, subtractor) -> list[dict]:
            if cv2 is None or np is None:
                return []
            mask = subtractor.apply(frame)
            mask = cv2.GaussianBlur(mask, (5, 5), 0)
            _, thresh = cv2.threshold(mask, 200, 255, cv2.THRESH_BINARY)
            kernel = np.ones((3, 3), dtype=np.uint8)
            cleaned = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=2)
            cleaned = cv2.dilate(cleaned, kernel, iterations=2)

            contours, _ = cv2.findContours(
                cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            candidates: list[dict] = []
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < self.motion_min_area:
                    continue
                x, y, w, h = cv2.boundingRect(contour)
                if w < self.motion_min_box_size or h < self.motion_min_box_size:
                    continue
                confidence = min(0.99, 0.5 + (float(area) / 5000.0))
                candidates.append(
                    {
                        "centroid": (x + (w / 2.0), y + (h / 2.0)),
                        "confidence": confidence,
                        "source": "motion",
                    }
                )
            return candidates

        def _yolo_candidates(self, frame) -> list[dict]:
            if self._yolo_model is None:
                return []
            try:
                results = self._yolo_model.predict(
                    source=frame,
                    verbose=False,
                    conf=float(self.confidence_threshold),
                )
            except Exception as exc:
                self.get_logger().warning(f"YOLO inference failed: {exc}")
                return []
            if not results:
                return []
            boxes = getattr(results[0], "boxes", None)
            if boxes is None:
                return []
            candidates: list[dict] = []
            for box in boxes:
                conf_values = box.conf.tolist() if box.conf is not None else []
                if not conf_values:
                    continue
                confidence = float(conf_values[0])
                cls_values = box.cls.tolist() if box.cls is not None else []
                class_id = int(cls_values[0]) if cls_values else -1
                if (
                    self.yolo_target_classes
                    and class_id not in self.yolo_target_classes
                ):
                    continue
                coords = box.xyxy.tolist()[0] if box.xyxy is not None else []
                if len(coords) != 4:
                    continue
                x1, y1, x2, y2 = coords
                box_area = max(0.0, (x2 - x1) * (y2 - y1))
                if box_area < self.yolo_min_box_area:
                    continue
                centroid = ((x1 + x2) / 2.0, (y1 + y2) / 2.0)
                candidates.append(
                    {
                        "centroid": centroid,
                        "confidence": confidence,
                        "source": "yolo",
                    }
                )
            return candidates

        def _normalize_candidates(self, raw_candidates) -> list[dict]:
            normalized: list[dict] = []
            for candidate in raw_candidates:
                if isinstance(candidate, dict):
                    centroid = candidate.get("centroid")
                    confidence = candidate.get("confidence")
                    source = candidate.get("source", "unknown")
                elif isinstance(candidate, (tuple, list)) and len(candidate) >= 2:
                    centroid = candidate[0]
                    confidence = candidate[1]
                    source = "legacy"
                else:
                    continue
                if (
                    not isinstance(centroid, (tuple, list))
                    or len(centroid) != 2
                    or confidence is None
                ):
                    continue
                try:
                    normalized.append(
                        {
                            "centroid": (float(centroid[0]), float(centroid[1])),
                            "confidence": float(confidence),
                            "source": str(source),
                        }
                    )
                except (TypeError, ValueError):
                    continue
            return normalized

        def image_callback(self, msg: Image, topic: str):
            """Process an incoming camera image."""
            if self._bridger is None or cv2 is None or np is None:
                return
            subtractor = self._background_models.get(topic)
            tracker = self._trackers.get(topic)
            if subtractor is None or tracker is None:
                return

            try:
                frame = self._bridger.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception as exc:
                self.get_logger().warning(f"Failed to decode frame from {topic}: {exc}")
                return

            self._frames_seen += 1
            candidates = self._normalize_candidates(self._yolo_candidates(frame))
            if candidates:
                self._yolo_candidate_frames += 1
            if not candidates and self.fallback_to_motion_tracking:
                candidates = self._normalize_candidates(
                    self._motion_candidates(frame, subtractor)
                )
                if candidates:
                    self._motion_candidate_frames += 1
            centroids = [candidate["centroid"] for candidate in candidates]

            tracked = tracker.update(centroids)
            for object_id, state in tracked.items():
                if state.disappeared != 0:
                    continue
                confidence = self.confidence_threshold
                detection_source = "yolo" if self._yolo_model is not None else "motion"
                if candidates:
                    distances = [
                        (
                            idx,
                            np.hypot(
                                state.centroid[0] - candidate["centroid"][0],
                                state.centroid[1] - candidate["centroid"][1],
                            ),
                        )
                        for idx, candidate in enumerate(candidates)
                    ]
                    nearest_idx = min(distances, key=lambda item: item[1])[0]
                    nearest_candidate = candidates[nearest_idx]
                    confidence = float(nearest_candidate["confidence"])
                    detection_source = str(nearest_candidate["source"])
                if confidence < self.confidence_threshold:
                    continue
                detection = {
                    "object_id": f"cv-{topic.replace('/', '-')}-track-{object_id}",
                    "camera_topic": topic,
                    "position_x": round(state.centroid[0], 1),
                    "position_y": round(state.centroid[1], 1),
                    "confidence": round(confidence, 3),
                    "detection_source": detection_source,
                }
                try:
                    self._detection_queue.put_nowait(detection)
                    self._detections_sent += 1
                except queue.Full:
                    self.get_logger().debug(
                        "Detection queue full; dropping frame detections."
                    )
                    break

        def destroy_node(self):
            self._ws_stop.set()
            if self._ws_thread and self._ws_thread.is_alive():
                self._ws_thread.join(timeout=2.0)
            return super().destroy_node()

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
