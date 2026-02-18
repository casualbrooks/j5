"""
Tracking Interface — Abstract base class for multi-object trackers.
Implement this to plug in DeepSORT, ByteTrack, or custom trackers.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional

from perception.ai_models.detection_interface import BoundingBox


@dataclass
class TrackedObject:
    """A tracked object with persistent identity across frames."""

    track_id: int
    bbox: BoundingBox
    velocity: tuple[float, float] = (0.0, 0.0)  # px/frame
    age: int = 0  # frames since first detection
    hits: int = 0  # total successful detections
    time_since_update: int = 0  # frames since last detection


class TrackingInterface(ABC):
    """
    Abstract interface for multi-object trackers.

    Example:
        class MyTracker(TrackingInterface):
            def update(self, detections, frame):
                # Match detections to existing tracks
                # Return list of TrackedObject
                ...
    """

    @abstractmethod
    def update(self, detections: list[BoundingBox], frame=None) -> list[TrackedObject]:
        """
        Update tracker with new detections from current frame.
        Args:
            detections: List of BoundingBox from current frame
            frame: Optional frame image for re-identification
        Returns:
            List of TrackedObject with persistent IDs
        """
        ...

    @abstractmethod
    def reset(self) -> None:
        """Reset all tracks."""
        ...
