"""
Standalone perception runner — works without ROS2.
Captures frames from cameras (or generates mock frames) and runs AI inference.
Pushes detections to the backend via WebSocket.
"""

import asyncio
import argparse
import json
import time
from collections import OrderedDict
from dataclasses import dataclass
from typing import Optional

try:
    import websockets
except ImportError:
    websockets = None  # type: ignore

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None  # type: ignore
    np = None  # type: ignore


@dataclass
class TrackerState:
    centroid: tuple[float, float]
    bbox: tuple[float, float, float, float] | None = None
    velocity: tuple[float, float] = (0.0, 0.0)
    disappeared: int = 0


def _bbox_iou(
    a: tuple[float, float, float, float] | None,
    b: tuple[float, float, float, float] | None,
) -> float:
    if a is None or b is None:
        return 0.0
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    ax2, ay2 = ax + aw, ay + ah
    bx2, by2 = bx + bw, by + bh
    ix1, iy1 = max(ax, bx), max(ay, by)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    union = (aw * ah) + (bw * bh) - inter
    return inter / union if union > 0.0 else 0.0


class SimpleCentroidTracker:
    """Lightweight centroid tracker for moving objects."""

    def __init__(
        self,
        max_disappeared: int = 20,
        max_distance: float = 140.0,
        min_iou: float = 0.03,
    ):
        self.max_disappeared = max_disappeared
        self.max_distance = max_distance
        self.min_iou = min_iou
        self.next_object_id = 1
        self.objects: OrderedDict[int, TrackerState] = OrderedDict()

    def _register(
        self,
        centroid: tuple[float, float],
        bbox: tuple[float, float, float, float] | None = None,
    ):
        self.objects[self.next_object_id] = TrackerState(centroid=centroid, bbox=bbox)
        self.next_object_id += 1

    def _deregister(self, object_id: int):
        self.objects.pop(object_id, None)

    def update(
        self,
        centroids: list[tuple[float, float]],
        bboxes: list[tuple[float, float, float, float]] | None = None,
    ) -> OrderedDict[int, TrackerState]:
        incoming_bboxes = bboxes or [None] * len(centroids)
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
            for idx, centroid in enumerate(centroids):
                self._register(centroid, incoming_bboxes[idx])
            return self.objects

        object_items = list(self.objects.items())
        object_ids = [item[0] for item in object_items]
        existing = np.array(
            [
                (
                    item[1].centroid[0] + item[1].velocity[0],
                    item[1].centroid[1] + item[1].velocity[1],
                )
                for item in object_items
            ],
            dtype=float,
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
                iou = _bbox_iou(self.objects[object_id].bbox, incoming_bboxes[col])
                if (
                    self.objects[object_id].disappeared == 0
                    and self.objects[object_id].bbox is not None
                    and incoming_bboxes[col] is not None
                    and iou < self.min_iou
                ):
                    continue
                prev_x, prev_y = self.objects[object_id].centroid
                next_x, next_y = centroids[col]
                velocity = (next_x - prev_x, next_y - prev_y)
                self.objects[object_id].velocity = (
                    (self.objects[object_id].velocity[0] * 0.4) + (velocity[0] * 0.6),
                    (self.objects[object_id].velocity[1] * 0.4) + (velocity[1] * 0.6),
                )
                self.objects[object_id].centroid = centroids[col]
                self.objects[object_id].bbox = incoming_bboxes[col]
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
                self._register(centroid, incoming_bboxes[col])

        return self.objects


def merge_nearby_boxes(
    boxes: list[tuple[int, int, int, int]], merge_distance: int = 24
) -> list[tuple[int, int, int, int]]:
    """Merge nearby or overlapping bounding boxes to reduce fragment detections."""
    if not boxes:
        return []
    merged = boxes[:]
    changed = True
    while changed:
        changed = False
        next_boxes: list[tuple[int, int, int, int]] = []
        while merged:
            x, y, w, h = merged.pop(0)
            x1, y1, x2, y2 = x, y, x + w, y + h
            idx = 0
            while idx < len(merged):
                ox, oy, ow, oh = merged[idx]
                ox1, oy1, ox2, oy2 = ox, oy, ox + ow, oy + oh
                close_x = ox1 <= x2 + merge_distance and ox2 >= x1 - merge_distance
                close_y = oy1 <= y2 + merge_distance and oy2 >= y1 - merge_distance
                if close_x and close_y:
                    x1, y1 = min(x1, ox1), min(y1, oy1)
                    x2, y2 = max(x2, ox2), max(y2, oy2)
                    merged.pop(idx)
                    changed = True
                else:
                    idx += 1
            next_boxes.append((x1, y1, x2 - x1, y2 - y1))
        merged = next_boxes
    return merged


class StandaloneRunner:
    """Run the perception pipeline without ROS2."""

    def __init__(
        self,
        ws_url: str = "ws://localhost:8080/ws?client_type=cv_system",
        cameras: Optional[list[dict]] = None,
        use_mock: bool = True,
        mock_racer_count: int = 4,
        confidence_threshold: float = 0.7,
    ):
        self.ws_url = ws_url
        self.cameras = cameras or [{"id": "cam1", "name": "Main Camera", "source": "0"}]
        self.use_mock = use_mock
        self.mock_racer_count = mock_racer_count
        self.confidence_threshold = confidence_threshold
        self._ws = None
        self._running = False
        self._captures: dict = {}
        self._trackers: dict[str, SimpleCentroidTracker] = {}
        self._background_models: dict[str, object] = {}

    async def connect(self):
        """Connect to the backend WebSocket."""
        if websockets is None:
            print(
                "Warning: 'websockets' package not installed. Run: pip install websockets"
            )
            return
        try:
            self._ws = await websockets.connect(self.ws_url)
            print(f"Connected to backend at {self.ws_url}")
        except Exception as e:
            print(f"Failed to connect to backend: {e}")
            self._ws = None

    async def send_detection(self, detection: dict):
        """Send a detection to the backend via WebSocket."""
        if self._ws:
            try:
                await self._ws.send(
                    json.dumps(
                        {
                            "type": "visionDetection",
                            "data": detection,
                            "timestamp": time.time(),
                        }
                    )
                )
            except Exception:
                pass

    def open_cameras(self):
        """Open video captures for configured cameras."""
        if cv2 is None:
            print("OpenCV not available — using mock mode")
            self.use_mock = True
            return

        if self.use_mock:
            print("Mock mode enabled — skipping camera capture")
            return

        for cam in self.cameras:
            try:
                source = (
                    int(cam["source"]) if cam["source"].isdigit() else cam["source"]
                )
                cap = cv2.VideoCapture(source)
                if cap.isOpened():
                    self._captures[cam["id"]] = cap
                    self._trackers[cam["id"]] = SimpleCentroidTracker()
                    if cv2 is not None:
                        self._background_models[cam["id"]] = (
                            cv2.createBackgroundSubtractorMOG2(
                                history=700, varThreshold=32, detectShadows=False
                            )
                        )
                    print(f"Opened camera: {cam['name']} ({cam['source']})")
                else:
                    print(f"Failed to open camera: {cam['name']} ({cam['source']})")
            except Exception as e:
                print(f"Error opening camera {cam['name']}: {e}")

    def close_cameras(self):
        """Release all video captures."""
        for cap in self._captures.values():
            cap.release()
        self._captures.clear()
        self._trackers.clear()
        self._background_models.clear()

    async def generate_mock_detections(self) -> list[dict]:
        """Generate mock detections for development/testing."""
        detections = []
        for i in range(self.mock_racer_count):
            # Simulate racers moving around an elliptical track
            t = time.time() + i * (2 * 3.14159 / self.mock_racer_count)
            speed_factor = 0.5 + (i * 0.1)  # Each racer has slightly different speed
            angle = (t * speed_factor) % (2 * 3.14159)

            x = 400 + 250 * float(
                np.cos(angle) if np else __import__("math").cos(angle)
            )
            y = 200 + 120 * float(
                np.sin(angle) if np else __import__("math").sin(angle)
            )

            import random

            confidence = 0.85 + random.random() * 0.14

            detections.append(
                {
                    "object_id": f"mock-cam1-track-{i+1}",
                    "camera_id": "cam1",
                    "position_x": round(x, 1),
                    "position_y": round(y, 1),
                    "confidence": round(confidence, 3),
                }
            )
        return detections

    async def process_frame(self, cam_id: str, frame) -> list[dict]:
        """Run lightweight motion-based detection + centroid tracking."""
        if cv2 is None or np is None:
            return []

        subtractor = self._background_models.get(cam_id)
        tracker = self._trackers.get(cam_id)
        if subtractor is None or tracker is None:
            return []

        mask = subtractor.apply(frame)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        _, thresh = cv2.threshold(mask, 200, 255, cv2.THRESH_BINARY)
        kernel = np.ones((3, 3), dtype=np.uint8)
        cleaned = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=2)
        cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel, iterations=3)
        cleaned = cv2.dilate(cleaned, kernel, iterations=2)

        contours, _ = cv2.findContours(
            cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        raw_boxes: list[tuple[int, int, int, int]] = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 550:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if w < 16 or h < 16:
                continue
            aspect = w / float(h)
            if aspect < 0.25 or aspect > 4.0:
                continue
            raw_boxes.append((x, y, w, h))

        merged_boxes = merge_nearby_boxes(raw_boxes)
        candidates: list[tuple[tuple[float, float], tuple[int, int, int, int]]] = [
            ((x + (w / 2.0), y + (h / 2.0)), (x, y, w, h))
            for x, y, w, h in merged_boxes
        ]

        tracked = tracker.update(
            [centroid for centroid, _bbox in candidates],
            [bbox for _centroid, bbox in candidates],
        )
        detections: list[dict] = []
        frame_height, frame_width = frame.shape[:2]
        for object_id, state in tracked.items():
            if state.disappeared > 2:
                continue
            closest_bbox = None
            if state.disappeared == 0 and candidates:
                closest_bbox = min(
                    candidates,
                    key=lambda item: (item[0][0] - state.centroid[0]) ** 2
                    + (item[0][1] - state.centroid[1]) ** 2,
                )[1]
            if closest_bbox is None and state.bbox is not None:
                closest_bbox = (
                    int(state.bbox[0]),
                    int(state.bbox[1]),
                    int(state.bbox[2]),
                    int(state.bbox[3]),
                )
            bbox_x, bbox_y, bbox_w, bbox_h = (
                closest_bbox if closest_bbox else (None, None, None, None)
            )
            detections.append(
                {
                    "object_id": f"cv-{cam_id}-track-{object_id}",
                    "camera_id": cam_id,
                    "position_x": round(state.centroid[0], 1),
                    "position_y": round(state.centroid[1], 1),
                    "confidence": 0.95,
                    "bbox_x": bbox_x,
                    "bbox_y": bbox_y,
                    "bbox_width": bbox_w,
                    "bbox_height": bbox_h,
                    "frame_width": frame_width,
                    "frame_height": frame_height,
                }
            )
        return detections

    async def run(self):
        """Main run loop."""
        print("\n" + "=" * 50)
        print("Race Master Pro — Standalone Perception Mode")
        print("=" * 50)
        print(f"Mock mode: {self.use_mock}")
        print(f"Cameras: {len(self.cameras)}")
        print(f"Confidence threshold: {self.confidence_threshold}")
        print()

        await self.connect()
        self.open_cameras()
        self._running = True

        try:
            while self._running:
                if self.use_mock:
                    detections = await self.generate_mock_detections()
                else:
                    detections = []
                    for cam_id, cap in self._captures.items():
                        ret, frame = cap.read()
                        if ret:
                            dets = await self.process_frame(cam_id, frame)
                            detections.extend(dets)

                for det in detections:
                    if det["confidence"] >= self.confidence_threshold:
                        await self.send_detection(det)

                await asyncio.sleep(0.033)  # ~30 FPS

        except KeyboardInterrupt:
            print("\nStopping perception runner...")
        finally:
            self._running = False
            self.close_cameras()
            if self._ws:
                await self._ws.close()
            print("Perception runner stopped.")


def main():
    """CLI entry point."""
    parser = argparse.ArgumentParser(
        description="Race Master Pro standalone perception runner"
    )
    parser.add_argument(
        "--ws-url", default="ws://localhost:8080/ws?client_type=cv_system"
    )
    parser.add_argument("--camera-source", default="0", help="Camera source index/path")
    parser.add_argument("--camera-id", default="cam1")
    parser.add_argument(
        "--use-mock", action="store_true", help="Use generated mock detections"
    )
    parser.add_argument("--mock-racer-count", type=int, default=4)
    parser.add_argument("--confidence-threshold", type=float, default=0.7)
    args = parser.parse_args()

    runner = StandaloneRunner(
        ws_url=args.ws_url,
        cameras=[
            {"id": args.camera_id, "name": "Main Camera", "source": args.camera_source}
        ],
        use_mock=args.use_mock,
        mock_racer_count=args.mock_racer_count,
        confidence_threshold=args.confidence_threshold,
    )
    asyncio.run(runner.run())


if __name__ == "__main__":
    main()
