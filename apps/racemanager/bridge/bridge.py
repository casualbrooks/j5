"""HTTP bridge between lap event producers and race manager service.

Modes:
- demo: emits synthetic laps (standalone, no ROS 2 required)
- stdin: reads JSON lap events from stdin (one per line)
- ros2: subscribes to a ROS 2 String topic containing JSON lap events
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import queue
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Optional

import requests
from requests import RequestException

logger = logging.getLogger(__name__)


@dataclass
class LapEvent:
    raceId: str
    carId: str
    sessionLap: int
    lapTimeMs: int
    trackDistanceM: float
    timestamp: datetime
    onTrack: bool
    directionOk: bool
    minLapTimeOk: bool
    source: str = "live"
    replayId: Optional[str] = None

    def as_payload(self) -> dict:
        return {
            "raceId": self.raceId,
            "carId": self.carId,
            "sessionLap": self.sessionLap,
            "lapTimeMs": self.lapTimeMs,
            "trackDistanceM": self.trackDistanceM,
            "timestamp": self.timestamp.isoformat(),
            "source": self.source,
            "replayId": self.replayId,
            "validity": {
                "onTrack": self.onTrack,
                "directionOk": self.directionOk,
                "minLapTimeOk": self.minLapTimeOk,
            },
        }

    @classmethod
    def from_json(cls, raw: str) -> "LapEvent":
        data = json.loads(raw)
        timestamp = data.get("timestamp")
        if isinstance(timestamp, str):
            parsed_time = datetime.fromisoformat(timestamp.replace("Z", "+00:00"))
        else:
            parsed_time = datetime.now(tz=timezone.utc)

        validity = data.get("validity", {})
        return cls(
            raceId=data["raceId"],
            carId=data["carId"],
            sessionLap=int(data["sessionLap"]),
            lapTimeMs=int(data["lapTimeMs"]),
            trackDistanceM=float(data["trackDistanceM"]),
            timestamp=parsed_time,
            onTrack=bool(validity.get("onTrack", True)),
            directionOk=bool(validity.get("directionOk", True)),
            minLapTimeOk=bool(validity.get("minLapTimeOk", True)),
            source=data.get("source", "live"),
            replayId=data.get("replayId"),
        )


class RaceManagerBridge:
    def __init__(self, service_base_url: str) -> None:
        self.service_base_url = service_base_url.rstrip("/")

    def emit_lap(self, lap: LapEvent) -> dict:
        url = f"{self.service_base_url}/ingest/lap"
        payload = lap.as_payload()
        response = requests.post(url, json=payload, timeout=5)
        response.raise_for_status()
        return response.json()


def run_demo(
    bridge: RaceManagerBridge, *, race_id: str, car_id: str, track_m: float
) -> int:
    lap_idx = 1
    while lap_idx <= 3:
        lap = LapEvent(
            raceId=race_id,
            carId=car_id,
            sessionLap=lap_idx,
            lapTimeMs=72000 + (lap_idx * 250),
            trackDistanceM=track_m,
            timestamp=datetime.now(tz=timezone.utc),
            onTrack=True,
            directionOk=True,
            minLapTimeOk=True,
            source="live",
        )
        result = bridge.emit_lap(lap)
        logger.info("emitted demo lap %s -> %s", lap_idx, result)
        lap_idx += 1
        time.sleep(1.0)
    return 0


def run_stdin(bridge: RaceManagerBridge) -> int:
    logger.info("reading lap events from stdin as JSON lines")
    for line in sys.stdin:
        stripped = line.strip()
        if not stripped:
            continue
        lap = LapEvent.from_json(stripped)
        result = bridge.emit_lap(lap)
        logger.info("emitted lap for %s L%s -> %s", lap.carId, lap.sessionLap, result)
    return 0


def run_ros2(bridge: RaceManagerBridge, *, topic: str) -> int:
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
    except ImportError as exc:
        logger.error(
            "ROS 2 dependencies unavailable. Source ROS 2 and install rclpy/std_msgs. Error: %s",
            exc,
        )
        return 2

    class Ros2LapBridge(Node):
        def __init__(self) -> None:
            super().__init__("race_manager_bridge")
            # Keep a strong reference to the subscription. rclpy subscriptions can
            # be garbage-collected if not assigned, which may lead to unstable
            # runtime behavior when messages are published.
            self._subscription = self.create_subscription(
                String, topic, self._on_message, 10
            )
            self.get_logger().info(f"Subscribed to {topic}")

        def _on_message(self, msg: String) -> None:
            try:
                self._pending_messages.put_nowait(msg.data)
            except queue.Full:
                self.get_logger().error(
                    "dropping lap message because processing queue is full"
                )

        def _process_messages(self) -> None:
            while not self._shutdown_event.is_set():
                raw_message = self._pending_messages.get()
                if raw_message is None:
                    return
                try:
                    lap = LapEvent.from_json(raw_message)
                    bridge.emit_lap(lap)
                    self.get_logger().info(
                        f"forwarded lap car={lap.carId} lap={lap.sessionLap}"
                    )
                except Exception as err:
                    self.get_logger().error(f"failed to process lap message: {err}")

        def close(self) -> None:
            self._shutdown_event.set()
            self._pending_messages.put(None)
            self._worker.join(timeout=2)

    rclpy.init()
    node = Ros2LapBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("received Ctrl-C, shutting down ROS 2 bridge")
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Race manager lap bridge")
    parser.add_argument(
        "--service-url",
        default=os.getenv("RACEMANAGER_URL", "http://localhost:4000"),
        help="Race manager API base URL",
    )
    parser.add_argument(
        "--mode",
        choices=["demo", "stdin", "ros2"],
        default="demo",
        help="Bridge input mode",
    )
    parser.add_argument("--topic", default="/race/lap_event", help="ROS 2 topic name")
    parser.add_argument("--race-id", default="demo-race")
    parser.add_argument("--car-id", default="car-01")
    parser.add_argument("--track-distance-m", type=float, default=350.0)
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))
    bridge = RaceManagerBridge(args.service_url)

    try:
        if args.mode == "demo":
            return run_demo(
                bridge,
                race_id=args.race_id,
                car_id=args.car_id,
                track_m=args.track_distance_m,
            )
        if args.mode == "stdin":
            return run_stdin(bridge)
        return run_ros2(bridge, topic=args.topic)
    except RequestException as exc:
        logger.error(
            "Failed to reach race manager at %s: %s. Ensure service is running.",
            args.service_url,
            exc,
        )
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
