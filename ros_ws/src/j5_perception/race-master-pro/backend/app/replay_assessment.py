"""Offline, anonymous finish-line assessment for a marked replay interval."""

import cv2

from app.motion_timing import MotionFinishDetector


def assess_replay(path, start, end, gate, expected_cars):
    """Sample the marked interval without modifying race timing or its ledger."""
    capture = cv2.VideoCapture(str(path))
    try:
        if not capture.isOpened():
            raise ValueError("Could not open replay video")
        fps = capture.get(cv2.CAP_PROP_FPS)
        frame_count = capture.get(cv2.CAP_PROP_FRAME_COUNT)
        if not 1 <= fps <= 240 or frame_count < 1:
            raise ValueError("Replay has no usable frame rate or frames")
        duration = frame_count / fps
        stop = min(end if end is not None else duration, duration)
        if start >= stop:
            raise ValueError("Race markers must enclose part of the video")
        span = stop - start
        sample_length = min(20.0, span / 3) if span > 8 else span
        starts = [start]
        if span > sample_length + 1:
            starts.append(start + (span - sample_length) / 2)
            starts.append(stop - sample_length)
        windows = []
        for window_start in starts:
            detector = MotionFinishDetector(**gate)
            capture.set(cv2.CAP_PROP_POS_FRAMES, round(window_start * fps))
            events = []
            frames = 0
            limit = round((window_start + sample_length) * fps)
            while capture.get(cv2.CAP_PROP_POS_FRAMES) < limit:
                ok, frame = capture.read()
                if not ok:
                    break
                frame_index = capture.get(cv2.CAP_PROP_POS_FRAMES) - 1
                second = frame_index / fps
                frames += 1
                for event in detector.observe(frame, second):
                    key = f"{second:.3f}:{event['y']:.4f}"
                    events.append(
                        {
                            "key": key,
                            "second": round(second, 3),
                            "y": event["y"],
                            "identity": None,
                        }
                    )
            windows.append(
                {
                    "start": round(window_start, 2),
                    "end": round(min(window_start + sample_length, stop), 2),
                    "frames": frames,
                    "candidate_passes": len(events),
                    "events": events,
                }
            )
        return {
            "status": "needs_labels",
            "video": path.name,
            "resolution": [
                int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
                int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            ],
            "fps": round(fps, 2),
            "expected_cars": expected_cars,
            "identified_cars": 0,
            "candidate_passes": sum(window["candidate_passes"] for window in windows),
            "windows": windows,
            "assessment": (
                "Motion crossings are anonymous candidates. This recording has not been "
                "validated for per-car lap counting. Label true crossings and car IDs "
                "in sample clips before using its results for championship scoring."
            ),
        }
    finally:
        capture.release()
