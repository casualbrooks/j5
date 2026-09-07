import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from perception.track_discovery import TrackDiscovery


def test_discovers_normalized_closed_track_and_markers():
    discovery = TrackDiscovery()
    for _lap in range(4):
        for degree in range(360):
            angle = math.radians(degree)
            discovery.observe(
                "car-1",
                400 + 250 * math.cos(angle),
                200 + 120 * math.sin(angle),
                frame_width=800,
                frame_height=400,
            )

    proposal = discovery.proposal()
    model = proposal["track_model"]

    assert proposal["confidence"] >= 0.85
    assert len(model["centerline"]) == 24
    assert len(model["checkpoints"]) == 4
    assert len(model["finish_gate"]) == 2
    assert all(0 <= point["x"] <= 1 for point in model["centerline"])
    assert all(0 <= point["y"] <= 1 for point in model["centerline"])


def test_requires_enough_observations_before_proposing_track():
    discovery = TrackDiscovery()
    discovery.observe("car-1", 10, 20, frame_width=100, frame_height=100)

    proposal = discovery.proposal()

    assert proposal["confidence"] == 0
    assert proposal["track_model"] is None
