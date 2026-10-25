import cv2
import numpy as np

from app.color_timing import FinishGate, detect_colors


def test_forward_crossings_time_laps_and_reverse_does_not():
    gate = FinishGate(minimum_lap_ms=1000)
    assert gate.observe("red", 0.4, 0.5, 10) is None
    assert gate.observe("red", 0.6, 0.5, 10.1) is None
    assert gate.observe("red", 0.4, 0.5, 10.2) is None
    assert gate.observe("red", 0.6, 0.5, 10.3) is None
    assert gate.observe("red", 0.4, 0.5, 12) is None
    assert gate.observe("red", 0.6, 0.5, 12.1) == 2000


def test_occlusion_and_outside_line_do_not_invent_crossings():
    gate = FinishGate()
    gate.observe("red", 0.4, 0.5, 10)
    assert gate.observe("red", 0.6, 0.5, 11) is None
    assert gate.states["red"].last_crossing is None
    gate.observe("red", 0.4, 0.95, 11.1)
    assert gate.observe("red", 0.6, 0.95, 11.2) is None
    assert gate.states["red"].last_crossing is None


def test_distinct_colors_keep_identity_and_duplicate_blobs_are_rejected():
    image = np.zeros((200, 300, 3), dtype=np.uint8)
    cv2.rectangle(image, (20, 60), (45, 80), (0, 0, 255), -1)
    cv2.rectangle(image, (160, 60), (185, 80), (255, 0, 0), -1)
    racers = [
        {"id": "red", "name": "Red", "color": "#ff0000"},
        {"id": "blue", "name": "Blue", "color": "#0000ff"},
    ]
    assert [d["racer_id"] for d in detect_colors(image, racers)] == ["red", "blue"]
    cv2.rectangle(image, (220, 60), (245, 80), (0, 0, 255), -1)
    assert [d["racer_id"] for d in detect_colors(image, racers)] == ["blue"]


def test_unidentifiable_and_overlapping_colors_do_not_create_false_laps():
    image = np.zeros((100, 120, 3), dtype=np.uint8)
    cv2.rectangle(image, (20, 30), (50, 60), (0, 0, 255), -1)
    racers = [
        {"id": "black", "name": "Black", "color": "#101010"},
        {"id": "red", "name": "Red", "color": "#ff0000"},
        {"id": "other-red", "name": "Other red", "color": "#ed2010"},
    ]
    assert detect_colors(image, racers) == []


def test_finish_region_ignores_same_colored_objects_elsewhere():
    image = np.zeros((200, 300, 3), dtype=np.uint8)
    cv2.rectangle(image, (150, 80), (175, 100), (0, 0, 255), -1)
    cv2.rectangle(image, (20, 20), (45, 40), (0, 0, 255), -1)
    racer = [{"id": "red", "name": "Red", "color": "#ff0000"}]
    detections = detect_colors(image, racer, region=(0.35, 0.3, 0.8, 0.6))
    assert len(detections) == 1
    assert detections[0]["racer_id"] == "red"
    assert detections[0]["box"] == [150, 80, 26, 21]
