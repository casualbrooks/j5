"""Build an anonymous, motion-scored finish-line preview on an off-Pi computer.

The input video and output JSON stay on the computer running this command. The
preview can be uploaded to /api/quick/review/{race_id}/tracks after inspection;
this script never changes the race's official lap ledger.
"""

import argparse
import hashlib
import json
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np


@dataclass
class Track:
    ident: int
    seen: list = field(default_factory=list)
    emitted: bool = False

    def expected_x(self, frame, scale=1):
        if len(self.seen) < 2:
            return self.seen[-1][1]
        older = self.seen[max(0, len(self.seen) - 4)]
        delta = self.seen[-1][0] - older[0]
        speed = (self.seen[-1][1] - older[1]) / delta if delta else 0
        return self.seen[-1][1] + max(-45 * scale, min(5 * scale, speed)) * (
            frame - self.seen[-1][0]
        )


def file_hash(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def motion_evidence(seen, scale=1):
    frames = np.array([item[0] for item in seen], dtype=float)
    x = np.array([item[1] for item in seen], dtype=float)
    speed = -np.diff(x) / np.maximum(np.diff(frames), 1)
    median = float(np.median(speed))
    variation = float(np.std(speed))
    backward = float(np.mean(speed < 0))
    distance = float(x[0] - x[-1])
    return {
        "consistent": median >= 8 * scale
        and variation <= 0.35 * median + scale
        and backward <= 0.2
        and distance >= 70 * scale,
        "median_px_per_frame": round(median, 2),
        "speed_variation": round(variation, 2),
        "backward_fraction": round(backward, 3),
        "distance_px": round(distance, 1),
    }


def build_preview(video, manifest):
    if file_hash(video) != manifest["video_sha256"]:
        raise ValueError("Video does not match the race manifest")
    capture = cv2.VideoCapture(str(video))
    try:
        if not capture.isOpened():
            raise ValueError("Could not decode the replay video")
        fps = capture.get(cv2.CAP_PROP_FPS)
        width = round(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = round(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if not 1 <= fps <= 240 or width < 100 or height < 100:
            raise ValueError("Replay has no usable frame rate or dimensions")
        gate = manifest["finish_gate"]
        if gate["direction"] != -1:
            raise ValueError(
                "This motion tracker currently requires a leftward finish gate"
            )
        line = round(gate["finish_x"] * width)
        x0, x1 = max(0, line - round(0.206 * width)), min(
            width, line + round(0.216 * width)
        )
        y0 = max(0, round((gate["y_min"] - 0.075) * height))
        y1 = min(height, round((gate["y_max"] + 0.005) * height))
        start = float(manifest["video_start_s"])
        end = float(
            manifest["video_end_s"] or capture.get(cv2.CAP_PROP_FRAME_COUNT) / fps
        )
        samples = []
        for second in np.linspace(start, end, 180):
            capture.set(cv2.CAP_PROP_POS_FRAMES, round(second * fps))
            good, frame = capture.read()
            if good:
                samples.append(frame[y0:y1, x0:x1])
        if len(samples) < 20:
            raise ValueError("Not enough frames to learn the track background")
        background = np.median(np.stack(samples), axis=0).astype(np.uint8)
        first_frame, last_frame = round(start * fps), round(end * fps)
        capture.set(cv2.CAP_PROP_POS_FRAMES, first_frame)
        active, crossings, next_id = [], [], 1
        scale = width / 640
        for frame_index in range(first_frame, last_frame):
            good, frame = capture.read()
            if not good:
                break
            roi = frame[y0:y1, x0:x1]
            difference = np.max(cv2.absdiff(roi, background), axis=2)
            mask = (difference >= 22).astype(np.uint8) * 255
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
            count, _, stats, centroids = cv2.connectedComponentsWithStats(mask)
            candidates = []
            for index in range(1, count):
                x, y, box_width, box_height, area = map(int, stats[index])
                if not (
                    8 * scale * scale <= area <= 500 * scale * scale
                    and 6 * scale <= box_width <= 75 * scale
                    and 2 * scale <= box_height <= 17 * scale
                ):
                    continue
                cx, cy = centroids[index]
                candidates.append((x0 + cx, y0 + cy))
            choices = []
            for track_index, track in enumerate(active):
                delta = frame_index - track.seen[-1][0]
                if delta > 11:
                    continue
                for candidate_index, (x, y) in enumerate(candidates):
                    dx = x - track.seen[-1][1]
                    if dx > 12 * scale or dx < -max(50, 35 * delta) * scale:
                        continue
                    if abs(y - track.seen[-1][2]) > 7 * scale:
                        continue
                    cost = (
                        abs(x - track.expected_x(frame_index, scale))
                        + 3 * abs(y - track.seen[-1][2])
                        + delta * 3
                    )
                    choices.append((cost, track_index, candidate_index))
            used_tracks, used_candidates = set(), set()
            for cost, track_index, candidate_index in sorted(choices):
                if (
                    cost > 70 * scale
                    or track_index in used_tracks
                    or candidate_index in used_candidates
                ):
                    continue
                used_tracks.add(track_index)
                used_candidates.add(candidate_index)
                active[track_index].seen.append(
                    (frame_index, *candidates[candidate_index])
                )
            for candidate_index, point in enumerate(candidates):
                if candidate_index not in used_candidates:
                    active.append(Track(next_id, [(frame_index, *point)]))
                    next_id += 1
            still_active = []
            for track in active:
                seen = track.seen
                before = [point for point in seen if point[1] >= line + 12 * scale]
                after = [point for point in seen if point[1] <= line - 12 * scale]
                if (
                    not track.emitted
                    and before
                    and after
                    and before[0][0] < after[-1][0]
                ):
                    span = max(point[1] for point in seen) - min(
                        point[1] for point in seen
                    )
                    duration = (after[-1][0] - before[0][0]) / fps
                    if len(seen) >= 3 and span >= 35 * scale and duration <= 0.55:
                        pair = min(
                            ((a, b) for a in before for b in after if a[0] < b[0]),
                            key=lambda pair: pair[1][0] - pair[0][0],
                        )
                        left, right = pair
                        fraction = (left[1] - line) / (left[1] - right[1])
                        crossing = (left[0] + fraction * (right[0] - left[0])) / fps
                        crossing_y = left[2] + fraction * (right[2] - left[2])
                        if gate["y_min"] <= crossing_y / height <= gate["y_max"]:
                            crossings.append((track, crossing))
                    track.emitted = True
                if frame_index - seen[-1][0] <= 11:
                    still_active.append(track)
            active = still_active
        events = []
        for track, crossing in crossings:
            motion = motion_evidence(track.seen, scale)
            points = [
                {
                    "second": round(index / fps, 5),
                    "x": round(x / width, 6),
                    "y": round(y / height, 6),
                    "observed": True,
                }
                for index, x, y in track.seen
                if start <= index / fps <= end
            ]
            if (
                len(points) >= 2
                and points[0]["second"] <= crossing <= points[-1]["second"]
            ):
                events.append(
                    {
                        "track_id": track.ident,
                        "crossing_second": round(crossing, 3),
                        "crossing_confidence": 0.96 if motion["consistent"] else 0.35,
                        "motion_evidence": motion,
                        "points": points,
                    }
                )
        events.sort(key=lambda event: event["crossing_second"])
        if len(events) > 1000 or sum(len(event["points"]) for event in events) > 50000:
            raise ValueError("Preview exceeds the Pi's track limit")
        return {
            "race_id": manifest["race_id"],
            "video_sha256": manifest["video_sha256"],
            "source": "offboard-motion-preview-v1",
            "analysis_gate": gate,
            "tracks": events,
        }
    finally:
        capture.release()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text(encoding="utf-8-sig"))
    preview = build_preview(args.video, manifest)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(preview, separators=(",", ":")), encoding="utf-8")
    print(
        f"{len(preview['tracks'])} anonymous crossing tracks; "
        f"{sum(track['motion_evidence']['consistent'] for track in preview['tracks'])} "
        f"pass the motion check; saved {args.output}"
    )


if __name__ == "__main__":
    main()
