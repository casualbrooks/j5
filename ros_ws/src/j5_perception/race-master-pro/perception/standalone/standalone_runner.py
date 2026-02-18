"""
Standalone perception runner — works without ROS2.
Captures frames from cameras (or generates mock frames) and runs AI inference.
Pushes detections to the backend via WebSocket.
"""

import asyncio
import json
import time
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
                    "racer_id": f"racer_{i+1}",
                    "camera_id": "cam1",
                    "position_x": round(x, 1),
                    "position_y": round(y, 1),
                    "confidence": round(confidence, 3),
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
                    # TODO: Process real frames through AI pipeline
                    # for cam_id, cap in self._captures.items():
                    #     ret, frame = cap.read()
                    #     if ret:
                    #         dets = await self.process_frame(cam_id, frame)
                    #         detections.extend(dets)

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
    runner = StandaloneRunner(use_mock=True, mock_racer_count=4)
    asyncio.run(runner.run())


if __name__ == "__main__":
    main()
