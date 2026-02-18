"""
Detection Interface — Abstract base class for all object detectors.
Implement this interface to plug in any detection model (YOLO, SSD, custom CNN, etc.)
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

try:
    import numpy as np
except ImportError:
    np = None  # type: ignore


@dataclass
class BoundingBox:
    """Bounding box with class and confidence."""

    x1: float  # top-left x
    y1: float  # top-left y
    x2: float  # bottom-right x
    y2: float  # bottom-right y
    confidence: float
    class_id: int
    class_name: str

    @property
    def center(self) -> tuple[float, float]:
        return ((self.x1 + self.x2) / 2, (self.y1 + self.y2) / 2)

    @property
    def width(self) -> float:
        return self.x2 - self.x1

    @property
    def height(self) -> float:
        return self.y2 - self.y1


class DetectionInterface(ABC):
    """
    Abstract interface for object detectors.
    To use a custom model, subclass this and implement detect().

    Example:
        class MyDetector(DetectionInterface):
            def load(self, model_path):
                self.model = load_my_model(model_path)

            def detect(self, frame):
                results = self.model.predict(frame)
                return [BoundingBox(...) for r in results]
    """

    @abstractmethod
    def load(self, model_path: str, **kwargs) -> None:
        """Load model weights from disk."""
        ...

    @abstractmethod
    def detect(self, frame) -> list[BoundingBox]:
        """
        Run detection on a single frame.
        Args:
            frame: numpy array (H, W, C) in BGR color space
        Returns:
            List of BoundingBox detections
        """
        ...

    def warmup(self, input_size: tuple[int, int] = (640, 640)) -> None:
        """Optional warmup pass for GPU init."""
        pass
