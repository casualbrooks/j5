# Track Lap Counter (Web Camera, On-Track Perception)

## Overview
Use a single web camera overlooking a closed-loop track to identify multiple cars, count laps per object in real time, and present lap stats locally (display + log file). The system must reject laps where a car is picked up and moved over the finish line; only cars that traverse the track in the correct direction at plausible speed should be credited.

## Goals
- Multi-object lap counting with per-car IDs.
- Robust anti-cheating checks (no teleporting or backwards finishes).
- Stand-alone appliance: edge compute, local display, and on-device logging.
- Real-time feedback (<200 ms latency from crossing to update).

## Hardware & system layout
- **Camera:** USB or CSI web camera mounted overhead, angled to view the full track and finish line. Calibrate intrinsic/extrinsic parameters to recover scale.
- **Edge compute:** Jetson Orin Nano/AGX or x86 mini PC with GPU; runs perception, tracking, and UI stack.
- **Networking:** Optional; defaults to fully offline. Export logs via USB stick or periodic Wi-Fi sync when available.
- **Display/controls:** Small HDMI touch display showing the leaderboard and system state. Physical start/reset button wired to GPIO if desired.

## Software pipeline
1. **Frame ingest:** GStreamer or OpenCV pipeline reading the camera at 30–60 FPS. Apply lens undistortion and perspective transform to a bird's-eye view.
2. **Track mask:** Precompute a binary mask of valid track pixels (semi-automatic segmentation in a setup tool). All valid trajectories must stay inside this mask.
3. **Object detection:** Lightweight detector (e.g., YOLOv8n) fine-tuned on track cars; detect bounding boxes + headings (via keypoints or color patch).
4. **Multi-object tracking:** Tracker such as BYTETrack or DeepSORT maintains stable IDs, produces smoothed center trajectories, and estimates velocity and heading.
5. **Finish-line gate:** Define a line segment in world coordinates. A lap is eligible when a track-confirmed trajectory crosses the gate in the forward direction.

## Anti-cheating & lap validation
Apply the following checks before incrementing a lap counter:
- **Inside track:** All samples between the previous and candidate finish crossings must stay within the track mask; discard any ID that leaves the mask.
- **Minimum lap time:** Compute median lap time per car; require a lap duration > configurable threshold (e.g., 0.6× median, absolute minimum of N seconds) to reject teleports.
- **Forward heading:** At the gate, enforce heading within ±45° of the track direction and velocity pointing forward.
- **Continuous motion:** Require non-zero displacement along the track centerline between gates; jumps in position without intermediate detections invalidate the lap.
- **ID hygiene:** If a tracker re-IDs a car mid-lap, merge only when spatial proximity and appearance embedding match; otherwise, reset lap state to avoid double counting.

## User experience
- Real-time HUD: per-car lap count, last lap time, best lap, current speed, and status badges (e.g., "invalid lap – off track").
- Session control: start/stop/reset buttons in the UI; optional horn/LED at each valid lap.
- Logging: append JSON/CSV with timestamp, car ID, lap number, lap time, and validation flags; rotate files per session.

## Stand-alone deployment checklist
- Bundle the ROS 2/vision stack into a container or systemd service that auto-starts on boot.
- Store calibration (camera matrix, distortion, homography), track mask, and finish gate parameters in a config file per venue.
- Provide a guided setup wizard to redraw the finish gate, collect a fresh mask, and run a quick lap to learn baseline lap time.
- Expose health metrics (FPS, dropped frames, tracking confidence) on the HUD and in logs to spot camera occlusions or poor lighting.

## Testing ideas
- Simulated laps in a video loop to validate minimum-lap-time and direction gates.
- Drop-and-drag test: pick up a car across the finish line; confirm the lap is rejected and a warning appears.
- Off-track excursion: drive through the infield; ensure the tracker marks the lap invalid until the car re-enters and completes a full loop.
- **Prerecorded video verification:** feed an MP4 (captured from the same camera pose) through the identical perception stack. Tag each lap event with `source: "replay"` and a `replayId`, then send it to the race manager. Compare replay vs live laps by timestamp and lap number to confirm the anti-cheat checks and timing math match real-time runs.

## Web race manager integration
- Publish lap events from the perception stack into a ROS 2 topic consumed by a `race_bridge` node; forward only laps that pass track-mask, heading, and minimum-time checks.
- The bridge posts validated laps to the race manager service (see `docs/race_manager.md`), which writes to MongoDB Atlas and pushes websocket updates to the web UI.
- The web UI should render per-car cards: lap count, last/best lap, average speed (track distance / lap time), validity flags, and a delta-to-leader timer.
- For multi-race championships, persist `raceId` and `seasonId` with each lap so standings roll up across events; apply scoring rules server-side and stream updated tables to clients.
- Keep an offline buffer in the bridge when Atlas connectivity drops; replay buffered laps in order to avoid gaps in the leaderboard.
