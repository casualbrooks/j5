# Raspberry Pi 5 Deployment: Gap Analysis + Priority Plan

This document maps your target architecture to what is already implemented in `race-master-pro`, what is close, and what should be finished first for reliable track testing.

## Target Architecture (as requested)

- **Pi = robotics/perception edge node**
  - USB camera ingest
  - local persistence + resilient event buffering
  - real-time event feed to cloud database/API
- **Cloud = source of truth**
  - race/championship schema
  - analytics and historical reporting
- **Web app client (separately hosted)**
  - real-time race dashboard
  - race history + championship leaderboard
  - no heavy CV logic in browser
- **Track setup workflow**
  - camera + network checks
  - track snapshot and scale calibration
  - racer registration with editable confirmation
- **Graceful fallback**
  - pause/resume with reconnect support
  - preserve timing accuracy across feed interruptions

---

## What is already met

1. **Backend API foundation is in place**
   - FastAPI app with REST + WebSocket endpoints, including race state transitions (`start`, `pause`, `finish`).
   - CRUD exists for seasons, championships, tracks, racers, races, laps, camera configs, detections.
2. **Data model supports championship-style statistics**
   - Tables include seasons, championships, events, races, results, lap records, and behavior metrics.
   - Analytics endpoints exist for racer summary, track records, and championship standings.
3. **Frontend dashboard scaffolding exists**
   - React UI includes dashboard, leaderboard, championship view, racer manager, analytics, track canvas, and settings panels.
4. **Perception integration path exists**
   - ROS2 node and standalone runner are present.
   - Standalone runner can stream mock detections over WebSocket for end-to-end flow testing.

---

## What is close (but not production-ready yet)

1. **Pi preflight/setup UX**
   - Existing codebase had no guided terminal workflow for camera/network/db readiness checks.
   - Added `scripts/pi_preflight.py` to provide checks + optional setup wizard.
2. **Track calibration flow**
   - Track fields (`scale`, `track_distance`, `layout_points`) exist in DB.
   - Missing: fully guided calibration pipeline that maps pixels to meters from image annotation.
3. **Cloud DB split architecture**
   - Backend currently defaults to local SQLite.
   - Needs a cloud persistence target (managed Postgres or hosted API) and migration layer.
4. **Resilience logic for interruptions**
   - Race states include pause/resume.
   - Missing: explicit paused-duration accounting in lap/elapsed stats and reconnection replay queue.

---

## Highest-priority finish plan (in order)

### P0 — start testing this week

1. **Use one backend as API boundary (Pi + web both call it)**
   - Keep FastAPI as the shared contract.
   - Run it in cloud or on a reachable host; Pi and web client both connect via API + WS.
2. **Run Pi preflight before every session**
   - Execute `python scripts/pi_preflight.py --wizard` over SSH.
   - Store generated summary JSON with setup parameters for traceability.
3. **Enforce race-control procedure**
   - Operators must use race `pause` on signal loss and `start` to resume.
   - Treat paused windows as excluded timing until auto-correction is implemented.

### P1 — reliability hardening

1. **Add pause-window accounting in backend**
   - Persist `paused_at`, cumulative `paused_duration_ms` per race.
   - Derive effective elapsed race time = `now - start_time - paused_duration`.
2. **Add edge event queue on Pi**
   - Buffer detections/lap events when API unavailable.
   - Flush on reconnect with monotonic timestamps.
3. **Define cloud DB migration**
   - Move from SQLite to managed Postgres for multi-client consistency.

### P2 — competitive feature completeness

1. **Track scanner workflow**
   - Snapshot -> boundary extraction -> minimap geometry persisted to `layout_points`.
2. **Registration/edit loop in UI**
   - Guided confirmation and revision screens for racers/cars/race format before green flag.
3. **Video stream endpoint integration**
   - Publish annotated feed (e.g., RTSP/WebRTC) and embed in browser panel.

---

## New terminal workflow (SSH on Pi)

```bash
cd ~/alive/j5/ros_ws/src/j5_perception/race-master-pro
python3 scripts/pi_preflight.py --backend-host <api-host> --backend-port 8080 --health-url http://<api-host>:8080/health --wizard
```

If `ros2` is missing, source environments:

```bash
source /opt/ros/iron/setup.bash
source ~/alive/j5/ros_ws/install/setup.bash
```

Use this as go/no-go criteria:

- network/API reachable
- camera discovered
- snapshot captured
- setup summary reviewed and accepted

