"""Pull a Pi replay to a stronger computer and propose temporally reviewed crossings.

Run with --watch for a persistent compute worker. NVIDIA_API_KEY stays on this
computer; the Pi receives only a proposal and never sees the credential.
"""

import argparse
import base64
import hashlib
import json
import os
import sys
import time
import uuid
from pathlib import Path
from urllib.error import HTTPError
from urllib.parse import quote
from urllib.request import Request, urlopen

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "backend"))
from app.motion_timing import MotionFinishDetector  # noqa: E402


def request_json(url, payload=None):
    body = json.dumps(payload).encode() if payload is not None else None
    headers = {"Content-Type": "application/json"} if body else {}
    try:
        with urlopen(Request(url, data=body, headers=headers), timeout=120) as response:
            return json.load(response)
    except HTTPError as exc:
        raise RuntimeError(
            f"{url}: HTTP {exc.code}: {exc.read(500).decode(errors='replace')}"
        ) from exc


def download_video(url, target, expected_hash):
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists() and file_hash(target) == expected_hash:
        return
    partial = target.with_suffix(".partial")
    digest = hashlib.sha256()
    with urlopen(url, timeout=180) as response, partial.open("wb") as stream:
        while chunk := response.read(1024 * 1024):
            stream.write(chunk)
            digest.update(chunk)
    if digest.hexdigest() != expected_hash:
        partial.unlink()
        raise ValueError("Downloaded video does not match Pi manifest")
    partial.replace(target)


def file_hash(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def find_crossings(video, manifest):
    capture = cv2.VideoCapture(str(video))
    try:
        if not capture.isOpened():
            raise ValueError("Could not decode downloaded replay")
        fps = capture.get(cv2.CAP_PROP_FPS)
        if not 1 <= fps <= 240:
            raise ValueError("Replay has no usable frame rate")
        detector = MotionFinishDetector(**manifest["finish_gate"])
        start_frame = round(manifest["video_start_s"] * fps)
        end_frame = round(
            (manifest["video_end_s"] or capture.get(cv2.CAP_PROP_FRAME_COUNT) / fps)
            * fps
        )
        capture.set(cv2.CAP_PROP_POS_FRAMES, start_frame)
        found = []
        for frame_index in range(start_frame, end_frame):
            ok, frame = capture.read()
            if not ok:
                break
            for event in detector.observe(frame, frame_index / fps):
                found.append(
                    {
                        "second": round(frame_index / fps, 3),
                        "y": event["y"],
                        "box": event["box"],
                    }
                )
        return found, fps
    finally:
        capture.release()


def temporal_strip(video, second, gate, fps, start=0, end=None):
    """Seven surrounding frames give the vision model motion and direction."""
    capture = cv2.VideoCapture(str(video))
    try:
        center = round(second * fps)
        frames = []
        evidence = []
        for offset in (-6, -4, -2, 0, 2, 4, 6):
            frame_second = (center + offset) / fps
            if frame_second < start or (end is not None and frame_second > end):
                continue
            capture.set(cv2.CAP_PROP_POS_FRAMES, max(0, center + offset))
            ok, frame = capture.read()
            if not ok:
                continue
            height, width = frame.shape[:2]
            x = round(gate["finish_x"] * width)
            y = round((gate["y_min"] + gate["y_max"]) * height / 2)
            x0, x1 = max(0, x - 145), min(width, x + 145)
            y0, y1 = max(0, y - 65), min(height, y + 65)
            crop = frame[y0:y1, x0:x1]
            if crop.size == 0:
                continue
            crop = cv2.resize(crop, (435, 195))
            cv2.putText(
                crop,
                f"{(center + offset) / fps:.3f}s",
                (8, 27),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
            )
            frames.append(crop)
            evidence.append(round(frame_second, 3))
        if len(frames) < 3:
            raise ValueError("Not enough surrounding frames for visual review")
        import numpy as np

        ok, encoded = cv2.imencode(
            ".jpg", np.vstack(frames), [cv2.IMWRITE_JPEG_QUALITY, 72]
        )
        if not ok:
            raise ValueError("Could not encode temporal strip")
        return encoded.tobytes(), evidence
    finally:
        capture.release()


def classify_strip(jpeg, racers, model):
    """Ask a vision model for an identity only when the sequence supports it."""
    key = os.environ.get("NVIDIA_API_KEY")
    if not key:
        return {
            "racer_id": None,
            "confidence": 0,
            "reason": "NVIDIA_API_KEY unavailable",
        }
    roster = [
        {"id": racer["id"], "name": racer["name"], "color_hint": racer["color"]}
        for racer in racers
    ]
    prompt = (
        "These seven image tiles are consecutive frames of a slot car moving at a finish line. "
        "Use motion across frames, silhouette, highlights and track position; a single pixel color "
        "is insufficient. Racer color hints are only hints, and similar colors can overlap. "
        f"Racers: {json.dumps(roster)}. "
        'Reply with JSON only: {"racer_id": string or null, "confidence": 0..1, '
        '"reason": short string}. Use null whenever the car cannot be uniquely matched to '
        "one racer. Do not invent an identity."
    )
    payload = {
        "model": model,
        "messages": [
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt},
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": "data:image/jpeg;base64,"
                            + base64.b64encode(jpeg).decode()
                        },
                    },
                ],
            }
        ],
        "temperature": 0.2,
        "max_tokens": 350,
        "stream": False,
        "chat_template_kwargs": {"enable_thinking": False},
    }
    with urlopen(
        Request(
            "https://integrate.api.nvidia.com/v1/chat/completions",
            data=json.dumps(payload).encode(),
            headers={
                "Authorization": f"Bearer {key}",
                "Content-Type": "application/json",
            },
        ),
        timeout=120,
    ) as response:
        result = json.load(response)
    answer = (result["choices"][0]["message"].get("content") or "").strip()
    if answer.startswith("```"):
        answer = answer.split("\n", 1)[1].rsplit("```", 1)[0]
    try:
        parsed = json.loads(answer)
    except (TypeError, ValueError):
        return {
            "racer_id": None,
            "confidence": 0,
            "reason": "Vision response was not usable JSON",
        }
    if parsed.get("racer_id") not in {racer["id"] for racer in racers}:
        parsed["racer_id"] = None
    parsed["confidence"] = max(0.0, min(1.0, float(parsed.get("confidence", 0))))
    return parsed


def review_one(pi, race_id, output_dir, model, max_events, submit, detect_only=False):
    uuid.UUID(race_id)
    base = f"{pi.rstrip('/')}/api/quick/review/{quote(race_id)}"
    manifest = request_json(f"{base}/manifest")
    video = output_dir / race_id / manifest["video"]
    download_video(f"{base}/video", video, manifest["video_sha256"])
    events, fps = find_crossings(video, manifest)
    processed = []
    crossings = []
    unassigned = 0
    last_by_racer = {}
    strips = output_dir / race_id / "strips"
    strips.mkdir(parents=True, exist_ok=True)
    for index, event in enumerate(events):
        if index >= max_events:
            unassigned += 1
            continue
        jpeg, evidence = temporal_strip(
            video,
            event["second"],
            manifest["finish_gate"],
            fps,
            manifest["video_start_s"],
            manifest["video_end_s"],
        )
        strip_name = f"{index:04d}.jpg"
        (strips / strip_name).write_bytes(jpeg)
        result = (
            {"racer_id": None, "confidence": 0, "reason": "Anonymous detection run"}
            if detect_only
            else classify_strip(jpeg, manifest["racers"], model)
        )
        processed.append(
            {
                **event,
                "evidence_seconds": evidence,
                "strip_file": f"strips/{strip_name}",
                "vision": result,
            }
        )
        racer_id = result.get("racer_id")
        previous = last_by_racer.get(racer_id)
        plausible_gap = (
            previous is None
            or (event["second"] - previous) * 1000 >= manifest["minimum_lap_ms"]
        )
        if racer_id and result["confidence"] >= 0.9 and plausible_gap:
            crossings.append(
                {
                    "racer_id": racer_id,
                    "second": event["second"],
                    "confidence": result["confidence"],
                    "evidence_seconds": evidence,
                }
            )
            last_by_racer[racer_id] = event["second"]
        else:
            unassigned += 1
    proposal = {
        "race_id": race_id,
        "video_sha256": manifest["video_sha256"],
        "model": "anonymous-motion-only" if detect_only else model,
        "crossings": crossings,
        "unassigned_crossings": unassigned,
        "notes": "Temporal review; unknown identities were withheld.",
    }
    output = output_dir / race_id / "review.json"
    output.write_text(
        json.dumps(
            {"manifest": manifest, "events": processed, "proposal": proposal}, indent=2
        )
    )
    if submit:
        result = request_json(f"{base}/proposal", proposal)
        print(
            f"{race_id}: {len(events)} candidates, {unassigned} unknown; Pi proposal {result['status']}"
        )
    else:
        print(
            f"{race_id}: {len(events)} candidates, {unassigned} unknown; saved {output}"
        )


def import_classifications(pi, race_id, output_dir, classification_file, submit):
    """Turn a NemoClaw or other vision worker's checked clips into a Pi proposal."""
    prepared = json.loads((output_dir / race_id / "review.json").read_text())
    result = json.loads(classification_file.read_text())
    manifest = prepared["manifest"]
    if result["video_sha256"] != manifest["video_sha256"]:
        raise ValueError("Classifications belong to a different video")
    by_index = {int(item["index"]): item for item in result["items"]}
    roster = {racer["id"] for racer in manifest["racers"]}
    crossings = []
    unknown = 0
    previous = {}
    for index, event in enumerate(prepared["events"]):
        item = by_index.get(index, {})
        racer_id = item.get("racer_id")
        confidence = float(item.get("confidence", 0))
        prior = previous.get(racer_id)
        plausible_gap = (
            prior is None
            or (event["second"] - prior) * 1000 >= manifest["minimum_lap_ms"]
        )
        if racer_id in roster and confidence >= 0.9 and plausible_gap:
            crossings.append(
                {
                    "racer_id": racer_id,
                    "second": event["second"],
                    "confidence": confidence,
                    "evidence_seconds": event["evidence_seconds"],
                }
            )
            previous[racer_id] = event["second"]
        else:
            unknown += 1
    proposal = {
        "race_id": race_id,
        "video_sha256": manifest["video_sha256"],
        "model": result.get("model", "offboard-vision"),
        "crossings": crossings,
        "unassigned_crossings": unknown,
        "notes": "Temporal vision review; ambiguous identities were withheld.",
    }
    path = output_dir / race_id / "imported-proposal.json"
    path.write_text(json.dumps(proposal, indent=2))
    if submit:
        url = f"{pi.rstrip('/')}/api/quick/review/{quote(race_id)}/proposal"
        saved = request_json(url, proposal)
        print(
            f"{race_id}: {len(crossings)} assigned; {unknown} unresolved; Pi status {saved['status']}"
        )
    else:
        print(
            f"{race_id}: {len(crossings)} assigned; {unknown} unresolved; saved {path}"
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pi", default="http://192.168.1.134:8080")
    parser.add_argument("--race-id")
    parser.add_argument("--watch", action="store_true")
    parser.add_argument("--submit", action="store_true")
    parser.add_argument("--output-dir", type=Path, default=Path.home() / "j5-reviews")
    parser.add_argument("--model", default="nvidia/nemotron-3.5-super-vl-120b-a12b")
    parser.add_argument("--max-events", type=int, default=10000)
    parser.add_argument("--detect-only", action="store_true")
    parser.add_argument("--import-classifications", type=Path)
    args = parser.parse_args()
    if not args.race_id and not args.watch:
        parser.error("Provide --race-id or --watch")
    if args.import_classifications:
        if not args.race_id:
            parser.error("--import-classifications requires --race-id")
        import_classifications(
            args.pi,
            args.race_id,
            args.output_dir,
            args.import_classifications,
            args.submit,
        )
        return
    if not args.detect_only and not os.environ.get("NVIDIA_API_KEY"):
        parser.error("Set NVIDIA_API_KEY on the compute worker or use --detect-only")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    while True:
        jobs = (
            [{"race_id": args.race_id, "status": "requested"}]
            if args.race_id
            else request_json(f"{args.pi.rstrip('/')}/api/quick/review/jobs")
        )
        for job in jobs:
            if job["status"] != "requested":
                continue
            try:
                review_one(
                    args.pi,
                    job["race_id"],
                    args.output_dir,
                    args.model,
                    args.max_events,
                    args.submit,
                    args.detect_only,
                )
            except Exception as exc:
                print(f"{job['race_id']}: review failed: {exc}", file=sys.stderr)
        if not args.watch:
            break
        time.sleep(20)


if __name__ == "__main__":
    main()
