#!/usr/bin/env python3
"""Terminal-first preflight checks and race setup wizard for Raspberry Pi deployment."""

from __future__ import annotations

import argparse
import json
import shutil
import socket
import subprocess
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from dataclasses import dataclass, asdict
from datetime import datetime, timezone
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


@dataclass
class CheckResult:
    name: str
    ok: bool
    details: str


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


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
    except (HTTPError, URLError, TimeoutError, OSError) as exc:
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


def camera_source_to_ffmpeg_device(camera_source: str) -> str:
    if camera_source.isdigit():
        return f"/dev/video{camera_source}"
    return camera_source


def maybe_capture_frame(
    output_file: Path, camera_source: str = "/dev/video0"
) -> CheckResult:
    if shutil.which("ffmpeg"):
        rc, _, err = run_cmd(
            [
                "ffmpeg",
                "-y",
                "-f",
                "video4linux2",
                "-i",
                camera_source_to_ffmpeg_device(camera_source),
                "-vframes",
                "1",
                str(output_file),
            ],
            timeout=10,
        )
        if rc == 0 and output_file.exists():
            return CheckResult("Track snapshot", True, f"Saved {output_file}")
        return CheckResult("Track snapshot", False, err or "ffmpeg failed")
    return CheckResult(
        "Track snapshot",
        False,
        "ffmpeg not installed; skipped capture. Install with: sudo apt update && sudo apt install -y ffmpeg",
    )


def is_required_check(check: CheckResult, *, preview_only: bool) -> bool:
    if (
        preview_only
        and check.name == "Track snapshot"
        and "skipped capture" in check.details.lower()
    ):
        return False
    return True


def camera_source_to_cv2_index(camera_source: str):
    if camera_source.isdigit():
        return int(camera_source)
    # Keep explicit device paths (e.g. /dev/video0) as strings so OpenCV
    # opens that exact node instead of remapping to a potentially different
    # numeric camera index.
    return camera_source


def write_frame_to_file(frame, output_file: Path) -> tuple[bool, str]:
    try:
        import cv2
    except ImportError:
        return False, "OpenCV (cv2) not installed; cannot write preview snapshot."

    output_file.parent.mkdir(parents=True, exist_ok=True)
    wrote = cv2.imwrite(str(output_file), frame)
    if not wrote:
        return False, f"Failed to write snapshot to {output_file}"
    return True, f"Saved {output_file}"


def capture_frame_opencv(camera_source: str, output_file: Path) -> tuple[bool, str]:
    try:
        import cv2
    except ImportError:
        return False, "OpenCV (cv2) not installed; cannot capture from browser preview."

    source = camera_source_to_cv2_index(camera_source)
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        return False, f"Unable to open camera source {camera_source}"
    ok, frame = cap.read()
    cap.release()
    if not ok or frame is None:
        return False, "Failed to read frame from camera"
    return write_frame_to_file(frame, output_file)


def run_preview_server(
    *,
    host: str,
    port: int,
    camera_source: str,
    capture_file: Path,
) -> int:
    try:
        import cv2
    except ImportError:
        print(
            "\n❌ Browser preview requires OpenCV. Install with: pip install opencv-python"
        )
        return 2

    boundary = "frame"
    frame_lock = threading.Lock()
    stream_state = {"latest_frame": None, "latest_frame_ts": 0.0, "active_streams": 0}

    class PreviewHandler(BaseHTTPRequestHandler):
        server_version = "pi-preflight-preview/1.0"

        def _set_cors_headers(self) -> None:
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET,POST,OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")

        def _send_html(self) -> None:
            html = f"""<!doctype html>
<html>
  <head>
    <meta charset='utf-8' />
    <title>Track Camera Preview</title>
    <style>
      body {{ font-family: sans-serif; margin: 1rem auto; max-width: 960px; }}
      img {{ width: 100%; border: 1px solid #888; border-radius: 8px; }}
      button {{ padding: 0.6rem 1rem; font-size: 1rem; margin-top: 0.8rem; }}
      code {{ background: #f5f5f5; padding: 0.1rem 0.3rem; border-radius: 4px; }}
    </style>
  </head>
  <body>
    <h1>Track Camera Preview</h1>
    <p>Live stream from <code>{camera_source}</code></p>
    <img src='/stream.mjpg' alt='Camera stream' />
    <form method='post' action='/capture'>
      <button type='submit'>Capture Track Photo</button>
    </form>
    <p>Snapshot path: <code>{capture_file}</code></p>
    <p>Latest snapshot URL: <code>/snapshot.jpg</code></p>
  </body>
</html>
"""
            payload = html.encode("utf-8")
            self.send_response(HTTPStatus.OK)
            self._set_cors_headers()
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self) -> None:  # noqa: N802
            if self.path in ("/", "/index.html"):
                self._send_html()
                return
            if self.path == "/ready":

                def probe_camera_once() -> tuple[bool, str]:
                    source = camera_source_to_cv2_index(camera_source)
                    cap = cv2.VideoCapture(source)
                    if not cap.isOpened():
                        return False, f"unable to open camera source {camera_source}"
                    ok, frame = cap.read()
                    cap.release()
                    if not ok or frame is None:
                        return False, "camera opened but no frame available"
                    with frame_lock:
                        stream_state["latest_frame"] = frame.copy()
                        stream_state["latest_frame_ts"] = time.monotonic()
                    return True, "camera opened and frame acquired"

                with frame_lock:
                    latest_frame = stream_state["latest_frame"]
                    latest_frame_ts = float(stream_state.get("latest_frame_ts") or 0.0)
                    active_streams = stream_state["active_streams"]
                frame_age_s = (
                    time.monotonic() - latest_frame_ts if latest_frame_ts else None
                )

                if (
                    latest_frame is not None
                    and active_streams > 0
                    and frame_age_s is not None
                    and frame_age_s <= 2.0
                ):
                    payload = json.dumps(
                        {
                            "ok": True,
                            "message": f"fresh frame buffered from active preview stream ({frame_age_s:.1f}s old)",
                        }
                    ).encode("utf-8")
                    self.send_response(HTTPStatus.OK)
                elif latest_frame is not None and active_streams > 0:
                    ok, message = probe_camera_once()
                    payload = json.dumps(
                        {
                            "ok": ok,
                            "message": (
                                "active stream has stale frame buffer; " + message
                            ),
                        }
                    ).encode("utf-8")
                    self.send_response(
                        HTTPStatus.OK if ok else HTTPStatus.SERVICE_UNAVAILABLE
                    )
                elif latest_frame is not None:
                    ok, message = probe_camera_once()
                    payload = json.dumps({"ok": ok, "message": message}).encode("utf-8")
                    self.send_response(
                        HTTPStatus.OK if ok else HTTPStatus.SERVICE_UNAVAILABLE
                    )
                elif capture_file.exists():
                    payload = json.dumps(
                        {"ok": True, "message": f"snapshot available at {capture_file}"}
                    ).encode("utf-8")
                    self.send_response(HTTPStatus.OK)
                elif active_streams > 0:
                    payload = json.dumps(
                        {
                            "ok": False,
                            "message": "preview stream active but no frame buffered yet",
                        }
                    ).encode("utf-8")
                    self.send_response(HTTPStatus.SERVICE_UNAVAILABLE)
                else:
                    ok, message = probe_camera_once()
                    payload = json.dumps({"ok": ok, "message": message}).encode("utf-8")
                    self.send_response(
                        HTTPStatus.OK if ok else HTTPStatus.SERVICE_UNAVAILABLE
                    )

                self._set_cors_headers()
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
                return
            if self.path == "/snapshot.jpg":
                if not capture_file.exists():
                    self.send_error(HTTPStatus.NOT_FOUND, "Snapshot not found")
                    return
                payload = capture_file.read_bytes()
                self.send_response(HTTPStatus.OK)
                self._set_cors_headers()
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(payload)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(payload)
                return
            if self.path == "/stream.mjpg":
                self.send_response(HTTPStatus.OK)
                self._set_cors_headers()
                self.send_header(
                    "Content-Type", f"multipart/x-mixed-replace; boundary={boundary}"
                )
                self.send_header("Cache-Control", "no-store")
                self.end_headers()

                source = camera_source_to_cv2_index(camera_source)
                cap = cv2.VideoCapture(source)
                if not cap.isOpened():
                    return
                with frame_lock:
                    stream_state["active_streams"] += 1
                try:
                    consecutive_failures = 0
                    while True:
                        ok, frame = cap.read()
                        if not ok or frame is None:
                            consecutive_failures += 1
                            if consecutive_failures >= 10:
                                break
                            time.sleep(0.1)
                            continue
                        consecutive_failures = 0
                        with frame_lock:
                            stream_state["latest_frame"] = frame.copy()
                            stream_state["latest_frame_ts"] = time.monotonic()
                        ok, encoded = cv2.imencode(".jpg", frame)
                        if not ok:
                            continue
                        data = encoded.tobytes()
                        self.wfile.write(f"--{boundary}\r\n".encode("ascii"))
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(
                            f"Content-Length: {len(data)}\r\n\r\n".encode("ascii")
                        )
                        self.wfile.write(data)
                        self.wfile.write(b"\r\n")
                        time.sleep(0.05)
                except (BrokenPipeError, ConnectionResetError):
                    pass
                finally:
                    with frame_lock:
                        stream_state["active_streams"] -= 1
                    cap.release()
                return

            self.send_error(HTTPStatus.NOT_FOUND, "Not found")

        def do_OPTIONS(self) -> None:  # noqa: N802
            self.send_response(HTTPStatus.NO_CONTENT)
            self._set_cors_headers()
            self.end_headers()

        def do_POST(self) -> None:  # noqa: N802
            if self.path != "/capture":
                self.send_error(HTTPStatus.NOT_FOUND, "Not found")
                return

            with frame_lock:
                latest_frame = stream_state["latest_frame"]
                frame = None if latest_frame is None else latest_frame.copy()

            if frame is not None:
                ok, message = write_frame_to_file(frame, capture_file)
            else:
                with frame_lock:
                    active_streams = stream_state["active_streams"]
                if active_streams > 0:
                    ok = False
                    message = "No frame available yet from active stream; wait for preview video to load and retry."
                else:
                    ok, message = capture_frame_opencv(camera_source, capture_file)

            body = json.dumps({"ok": ok, "message": message}).encode("utf-8")
            self.send_response(
                HTTPStatus.OK if ok else HTTPStatus.INTERNAL_SERVER_ERROR
            )
            self._set_cors_headers()
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, format: str, *args) -> None:  # noqa: A003
            return

    server = ThreadingHTTPServer((host, port), PreviewHandler)
    server.daemon_threads = True

    print("\n=== Camera Browser Preview ===")
    print(f"Open from another device: http://{host}:{port}/")
    print("Press Ctrl+C to stop preview server.")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping preview server...")
    finally:
        server.shutdown()
        server.server_close()

    return 0


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
    created_at = wizard_summary.get("created_at", utc_now_iso())
    year = datetime.now(timezone.utc).year
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
        "event_date": datetime.now(timezone.utc).date().isoformat(),
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
        "created_at": utc_now_iso(),
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


def print_next_steps(summary: dict) -> None:
    print("\n=== What to do next ===")
    print("1. Review the saved wizard/check output in preflight_summary.json.")
    print(
        "2. Use summary['wizard']['track']['meters_per_pixel'] as your baseline camera calibration value."
    )

    backend_seed = summary.get("backend_seed", {})
    created = backend_seed.get("created", {})
    race = created.get("race")
    track = created.get("track")
    camera = created.get("camera")

    if race and track and camera:
        print(
            "3. Keep these IDs for API/UI checks: "
            f"race={race['id']} track={track['id']} camera={camera['id']}."
        )
        print(
            "4. Open the Race Manager UI and confirm racers/laps match the wizard values before starting heats."
        )
    else:
        print(
            "3. Re-run with --apply-backend when ready to create track/camera/race records from wizard answers."
        )
    print(
        "5. If track capture was skipped, install ffmpeg: sudo apt update && sudo apt install -y ffmpeg"
    )
    print(
        "6. Optional browser preview: run with --serve-preview and open the printed URL from another device."
    )


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
    parser.add_argument(
        "--serve-preview",
        action="store_true",
        help="Start browser camera preview server for remote viewing and snapshot trigger",
    )
    parser.add_argument(
        "--preview-only",
        action="store_true",
        help="Skip backend connectivity checks and just run local camera preview diagnostics",
    )
    parser.add_argument(
        "--preview-host",
        default="0.0.0.0",
        help="Host bind for preview server (use 0.0.0.0 for LAN access)",
    )
    parser.add_argument(
        "--preview-port",
        type=int,
        default=8091,
        help="Port for preview server",
    )
    args = parser.parse_args()

    checks = [list_cameras()]

    if not args.preview_only:
        checks = [
            check_ros2_on_path(),
            check_tcp(args.backend_host, args.backend_port),
            check_http(args.health_url),
            *checks,
        ]

    if not args.skip_capture:
        checks.append(maybe_capture_frame(Path(args.capture_file), args.camera_source))

    print_results(checks)

    summary: dict = {
        "timestamp": utc_now_iso(),
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
    print_next_steps(summary)

    if args.serve_preview:
        preview_rc = run_preview_server(
            host=args.preview_host,
            port=args.preview_port,
            camera_source=args.camera_source,
            capture_file=Path(args.capture_file),
        )
        if preview_rc != 0:
            return preview_rc

    return 0 if all(c.ok for c in checks) else 1


if __name__ == "__main__":
    sys.exit(main())
