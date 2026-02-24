#!/usr/bin/env python3
"""Terminal-first preflight checks and race setup wizard for Raspberry Pi deployment."""

from __future__ import annotations

import argparse
import json
import shutil
import socket
import subprocess
import sys
import time
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path
from urllib.error import URLError
from urllib.request import urlopen


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
    parser.add_argument("--health-url", default="http://localhost:8080/health")
    parser.add_argument("--capture-file", default="track_snapshot.jpg")
    parser.add_argument("--skip-capture", action="store_true")
    parser.add_argument(
        "--wizard", action="store_true", help="Run interactive race setup wizard"
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

    Path(args.out).write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"\nSaved summary to {args.out}")

    return 0 if all(c.ok for c in checks[:4]) else 1


if __name__ == "__main__":
    sys.exit(main())
