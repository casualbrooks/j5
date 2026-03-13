#!/usr/bin/env python3
"""Terminal-first preflight checks and race setup wizard for Raspberry Pi deployment."""

from __future__ import annotations

import argparse
import json
import shutil
import socket
import subprocess
import sys
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


@dataclass
class CheckResult:
    name: str
    ok: bool
    details: str


def run_cmd(cmd: list[str], timeout: int = 5) -> tuple[int, str, str]:
    try:
        proc = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
        return proc.returncode, proc.stdout.strip(), proc.stderr.strip()
    except Exception as exc:  # pragma: no cover - defensive
        return 1, "", str(exc)


def check_ros2_on_path() -> CheckResult:
    ros2_path = shutil.which("ros2")
    if ros2_path:
        return CheckResult("ros2 CLI", True, f"Found at {ros2_path}")
    return CheckResult(
        "ros2 CLI",
        False,
        "Not found. Source /opt/ros/iron/setup.bash and ~/alive/j5/ros_ws/install/setup.bash",
    )


def check_tcp(host: str, port: int, timeout: float = 2.5) -> CheckResult:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    try:
        sock.connect((host, port))
    except OSError as exc:
        return CheckResult(f"TCP {host}:{port}", False, str(exc))
    finally:
        sock.close()
    return CheckResult(f"TCP {host}:{port}", True, "Connection successful")


def check_http(url: str, timeout: float = 4.0) -> CheckResult:
    try:
        with urlopen(url, timeout=timeout) as resp:
            status = getattr(resp, "status", 200)
            payload = resp.read(200).decode("utf-8", errors="ignore")
            return CheckResult(
                f"HTTP {url}", 200 <= status < 300, f"status={status} body={payload}"
            )
    except URLError as exc:
        return CheckResult(f"HTTP {url}", False, str(exc))


def list_cameras() -> CheckResult:
    if shutil.which("v4l2-ctl"):
        rc, out, err = run_cmd(["v4l2-ctl", "--list-devices"])
        if rc == 0 and out:
            return CheckResult("Camera discovery", True, out)
        return CheckResult("Camera discovery", False, err or "No output from v4l2-ctl")

    video_nodes = sorted(Path("/dev").glob("video*"))
    if video_nodes:
        return CheckResult(
            "Camera discovery", True, "Found: " + ", ".join(str(v) for v in video_nodes)
        )
    return CheckResult("Camera discovery", False, "No /dev/video* nodes found")


def maybe_capture_frame(output_file: Path) -> CheckResult:
    if shutil.which("ffmpeg"):
        rc, _, err = run_cmd(
            [
                "ffmpeg",
                "-y",
                "-f",
                "video4linux2",
                "-i",
                "/dev/video0",
                "-vframes",
                "1",
                str(output_file),
            ],
            timeout=10,
        )
        if rc == 0 and output_file.exists():
            return CheckResult("Track snapshot", True, f"Saved {output_file}")
        return CheckResult("Track snapshot", False, err or "ffmpeg failed")
    return CheckResult("Track snapshot", False, "ffmpeg not installed; skipped capture")


def prompt(text: str, default: str = "") -> str:
    value = input(f"{text} [{default}]: ").strip()
    return value or default


def http_json(
    method: str, url: str, payload: dict | None = None, timeout: float = 8.0
) -> dict:
    data = None
    headers = {"Accept": "application/json"}
    if payload is not None:
        data = json.dumps(payload).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = Request(url=url, method=method.upper(), data=data, headers=headers)

    try:
        with urlopen(req, timeout=timeout) as resp:
            body = resp.read().decode("utf-8", errors="ignore")
            return json.loads(body) if body else {}
    except HTTPError as exc:
        body = exc.read().decode("utf-8", errors="ignore")
        raise RuntimeError(f"{method} {url} failed: HTTP {exc.code} {body}") from exc
    except URLError as exc:
        raise RuntimeError(f"{method} {url} failed: {exc}") from exc


def build_seed_payload(
    wizard_summary: dict, *, race_name: str, camera_source: str
) -> dict:
    created_at = wizard_summary.get("created_at", datetime.utcnow().isoformat() + "Z")
    year = datetime.utcnow().year
    season = {"name": f"{year} Season", "year": year, "status": "active"}
    championship = {
        "name": f"{wizard_summary['track']['name']} Championship",
        "status": "active",
    }
    track = {
        "name": wizard_summary["track"]["name"],
        "scale": wizard_summary["track"]["scale"],
        "track_distance": wizard_summary["track"]["measured_segment_m"],
        "layout_points": "[]",
        "boundary_polygon": "[]",
    }
    camera = {
        "name": f"{wizard_summary['track']['name']} Camera",
        "source": camera_source,
        "status": "connected",
        "calibration": json.dumps(
            {
                "meters_per_pixel": wizard_summary["track"]["meters_per_pixel"],
                "segment_px": wizard_summary["track"]["segment_px"],
                "measured_segment_m": wizard_summary["track"]["measured_segment_m"],
                "created_at": created_at,
            }
        ),
    }
    event = {
        "name": f"{wizard_summary['track']['name']} Event",
        "event_date": datetime.utcnow().date().isoformat(),
        "round_number": 1,
    }
    race = {
        "type": "main",
        "total_laps": wizard_summary["race"]["laps"],
    }
    racers = [
        {
            "name": racer["name"],
            "number": str(index + 1),
            "vehicle_description": racer["car"],
        }
        for index, racer in enumerate(wizard_summary["race"]["racers"])
    ]
    return {
        "season": season,
        "championship": championship,
        "track": track,
        "camera": camera,
        "event": event,
        "race": race,
        "racers": racers,
        "race_name": race_name,
    }


def seed_backend(base_url: str, payload: dict) -> dict:
    base = base_url.rstrip("/")

    season = http_json("POST", f"{base}/api/seasons", payload["season"])
    championship_payload = {
        **payload["championship"],
        "season_id": season["id"],
    }
    championship = http_json("POST", f"{base}/api/championships", championship_payload)

    track = http_json("POST", f"{base}/api/tracks", payload["track"])

    camera_payload = {
        **payload["camera"],
        "track_id": track["id"],
    }
    camera = http_json("POST", f"{base}/api/cameras", camera_payload)

    event_payload = {
        **payload["event"],
        "championship_id": championship["id"],
        "track_id": track["id"],
    }
    event = http_json("POST", f"{base}/api/events", event_payload)

    race_payload = {
        **payload["race"],
        "event_id": event["id"],
    }
    race = http_json("POST", f"{base}/api/races", race_payload)

    racers = []
    results = []
    for racer_payload in payload["racers"]:
        racer = http_json("POST", f"{base}/api/racers", racer_payload)
        racers.append(racer)
        result = http_json(
            "POST",
            f"{base}/api/results",
            {
                "race_id": race["id"],
                "racer_profile_id": racer["id"],
                "finish_position": 0,
                "total_time": 0,
                "laps_completed": 0,
                "status": "racing",
                "points_earned": 0,
            },
        )
        results.append(result)

    return {
        "season": season,
        "championship": championship,
        "track": track,
        "camera": camera,
        "event": event,
        "race": race,
        "racers": racers,
        "results": results,
    }


def run_wizard() -> dict:
    print("\n=== Race Setup Wizard ===")
    track_name = prompt("Track name", "Main Track")
    scale = prompt("Scale", "1:24")
    measured_m = float(prompt("Known measured segment length (meters)", "5.0"))
    pixels = float(prompt("Same segment length in pixels from snapshot", "250"))
    meters_per_pixel = measured_m / pixels if pixels > 0 else 0

    racer_count = int(prompt("Number of racers", "4"))
    racers = []
    for i in range(racer_count):
        name = prompt(f"Racer {i+1} name", f"Racer {i+1}")
        car = prompt(f"Racer {i+1} car", f"Car {i+1}")
        racers.append({"name": name, "car": car})

    race_laps = int(prompt("Planned laps", "20"))
    pause_policy = {
        "pause_trigger": "Manual race control -> pause endpoint",
        "resume_trigger": "Resume when camera/network healthy",
        "time_correction": "Exclude paused duration from elapsed race-time metrics",
    }

    summary = {
        "created_at": datetime.utcnow().isoformat() + "Z",
        "track": {
            "name": track_name,
            "scale": scale,
            "meters_per_pixel": meters_per_pixel,
            "measured_segment_m": measured_m,
            "segment_px": pixels,
        },
        "race": {"laps": race_laps, "racers": racers},
        "fallback": pause_policy,
    }

    print("\n--- Human-readable Summary ---")
    print(json.dumps(summary, indent=2))
    return summary


def print_results(results: list[CheckResult]) -> None:
    print("\n=== Preflight Results ===")
    for result in results:
        icon = "✅" if result.ok else "❌"
        print(f"{icon} {result.name}: {result.details}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Raspberry Pi race preflight checks")
    parser.add_argument("--backend-host", default="localhost")
    parser.add_argument("--backend-port", type=int, default=8080)
    parser.add_argument("--backend-base-url", default="http://localhost:8080")
    parser.add_argument("--health-url", default="http://localhost:8080/health")
    parser.add_argument("--capture-file", default="track_snapshot.jpg")
    parser.add_argument("--skip-capture", action="store_true")
    parser.add_argument(
        "--wizard", action="store_true", help="Run interactive race setup wizard"
    )
    parser.add_argument(
        "--apply-backend",
        action="store_true",
        help="Create season/championship/track/camera/event/race/racers in backend from wizard answers",
    )
    parser.add_argument(
        "--camera-source",
        default="0",
        help="Camera source value to store in backend camera config",
    )
    parser.add_argument(
        "--start-race",
        action="store_true",
        help="Start the seeded race immediately after creation",
    )
    parser.add_argument(
        "--out", default="preflight_summary.json", help="Output JSON summary file"
    )
    args = parser.parse_args()

    checks = [
        check_ros2_on_path(),
        check_tcp(args.backend_host, args.backend_port),
        check_http(args.health_url),
        list_cameras(),
    ]

    if not args.skip_capture:
        checks.append(maybe_capture_frame(Path(args.capture_file)))

    print_results(checks)

    summary: dict = {
        "timestamp": datetime.utcnow().isoformat() + "Z",
        "checks": [asdict(c) for c in checks],
    }

    if args.wizard:
        summary["wizard"] = run_wizard()

    if args.apply_backend:
        if "wizard" not in summary:
            print("\n❌ --apply-backend requires --wizard so setup data is available.")
            return 2
        try:
            seed_payload = build_seed_payload(
                summary["wizard"],
                race_name="Headless Race",
                camera_source=args.camera_source,
            )
            seeded = seed_backend(args.backend_base_url, seed_payload)
            summary["backend_seed"] = {
                "base_url": args.backend_base_url,
                "created": seeded,
            }
            if args.start_race:
                race_id = seeded["race"]["id"]
                started = http_json(
                    "POST",
                    f"{args.backend_base_url.rstrip('/')}/api/races/{race_id}/start",
                )
                summary["backend_seed"]["started_race"] = started

            print("\n✅ Backend seeded successfully.")
            print(f"   Race ID: {seeded['race']['id']}")
            print(f"   Track ID: {seeded['track']['id']}")
            print(f"   Camera ID: {seeded['camera']['id']}")
        except RuntimeError as exc:
            print(f"\n❌ Backend seed failed: {exc}")
            summary["backend_seed_error"] = str(exc)

    Path(args.out).write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"\nSaved summary to {args.out}")

    return 0 if all(c.ok for c in checks[:4]) else 1


if __name__ == "__main__":
    sys.exit(main())
