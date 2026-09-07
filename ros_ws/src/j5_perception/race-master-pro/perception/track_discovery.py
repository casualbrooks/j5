"""Learn a closed track and travel direction from ordinary tracker centroids."""

from collections import defaultdict, deque
from math import atan2, pi


class TrackDiscovery:
    """Small, model-agnostic trajectory learner suitable for ROS2 or standalone use."""

    def __init__(self, angular_bins: int = 24, history: int = 600):
        self.angular_bins = angular_bins
        self.points = defaultdict(lambda: deque(maxlen=history))

    def observe(
        self,
        object_id: str,
        x: float,
        y: float,
        frame_width: float | None = None,
        frame_height: float | None = None,
    ) -> None:
        """Add a centroid, normalized when frame dimensions are available."""
        normalized_x = float(x) / frame_width if frame_width else float(x)
        normalized_y = float(y) / frame_height if frame_height else float(y)
        self.points[object_id].append((normalized_x, normalized_y))

    def proposal(self) -> dict:
        samples = [point for path in self.points.values() for point in path]
        if len(samples) < self.angular_bins:
            return self._empty("Keep cars moving around the whole circuit.")
        center_x = sum(point[0] for point in samples) / len(samples)
        center_y = sum(point[1] for point in samples) / len(samples)
        bins: dict[int, list[tuple[float, float]]] = defaultdict(list)
        signed_motion = 0.0
        motion_samples = 0
        for path in self.points.values():
            for point in path:
                angle = (atan2(point[1] - center_y, point[0] - center_x) + pi) / (
                    2 * pi
                )
                bins[min(self.angular_bins - 1, int(angle * self.angular_bins))].append(
                    point
                )
            for first, second in zip(path, list(path)[1:]):
                signed_motion += (first[0] - center_x) * (second[1] - first[1]) - (
                    first[1] - center_y
                ) * (second[0] - first[0])
                motion_samples += 1
        coverage = len(bins) / self.angular_bins
        motion_confidence = min(1.0, motion_samples / (self.angular_bins * 4))
        confidence = round(coverage * motion_confidence, 3)
        centerline = []
        for index in sorted(bins):
            bucket = bins[index]
            centerline.append(
                {
                    "x": round(sum(p[0] for p in bucket) / len(bucket), 2),
                    "y": round(sum(p[1] for p in bucket) / len(bucket), 2),
                }
            )
        gate = []
        if len(centerline) > 1:
            first, second = centerline[0], centerline[1]
            dx, dy = second["x"] - first["x"], second["y"] - first["y"]
            length = max(1.0, (dx * dx + dy * dy) ** 0.5)
            average_radius = sum(
                ((point[0] - center_x) ** 2 + (point[1] - center_y) ** 2) ** 0.5
                for point in samples
            ) / len(samples)
            half_gate_width = average_radius * 0.08
            normal_x, normal_y = (
                (-dy / length) * half_gate_width,
                (dx / length) * half_gate_width,
            )
            gate = [
                {
                    "x": round(first["x"] - normal_x, 2),
                    "y": round(first["y"] - normal_y, 2),
                },
                {
                    "x": round(first["x"] + normal_x, 2),
                    "y": round(first["y"] + normal_y, 2),
                },
            ]
        return {
            "confidence": confidence,
            "message": (
                "Track learned; verify the overlay and start the race."
                if confidence >= 0.85
                else "Learning track shape and direction from moving cars."
            ),
            "track_model": {
                "centerline": centerline,
                "direction": "clockwise" if signed_motion > 0 else "counterclockwise",
                "finish_gate": gate,
                "checkpoints": [
                    centerline[index]
                    for index in range(0, len(centerline), max(1, len(centerline) // 4))
                ][:4],
                "sample_count": len(samples),
            },
        }

    def _empty(self, message: str) -> dict:
        return {"confidence": 0.0, "message": message, "track_model": None}
