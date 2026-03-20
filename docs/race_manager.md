# Race Manager Web App + Lap Counter Integration

## Recommended folder layout (apps/racemanager)
Keep the race manager together in `apps/racemanager` so it travels as a single deployable unit alongside the ROS 2 workspace. A starter FastAPI service lives in `apps/racemanager/service` and exposes `/ingest/lap` + `/races/{raceId}/leaderboard` to bridge lap events into MongoDB:

```
apps/
  racemanager/
    ui/           # React/Next.js (or similar) web UI for live races + championships
    service/      # FastAPI service streaming lap events into MongoDB Atlas
    bridge/       # ROS 2/vision bridge: subscribes to lap counter events and forwards them to the service
    README.md     # App-specific setup, env vars, and run scripts
```

- The **web UI** keeps its own `.env` (Atlas connection/websocket URL) under `apps/racemanager/ui`.
- The **service** owns the Mongo schemas, champion standings logic, and websocket fan-out under `apps/racemanager/service`.
- The **bridge** can live under `apps/racemanager/bridge` or as a ROS 2 package (e.g., `ros_ws/src/j5_race_bridge`) that calls the service via HTTP/WebSocket/gRPC.

## Data model in MongoDB Atlas
Use capped collections for fast streams and archival collections for history. These live in the Atlas database named by `RACE_DB_NAME` (see `apps/racemanager/README.md` for environment wiring):

- `laps_live` (capped): `{ raceId, carId, sessionLap, timestamp, lapTimeMs, trackDistanceM, speedKph, validity: { onTrack, directionOk, minLapTimeOk } }`
- `races`: `{ _id, name, trackId, startTime, status, totalLaps, sessionType, participants: [{ carId, driver, team }] }`
- `championships`: `{ _id, name, seasonYear, tracks, scoringRules, standings: [{ carId, points, podiums, wins, bestLapCount }] }`
- `telemetry_archive`: append-only historical laps/events for long-term analytics.

Enable Atlas **change streams** on `laps_live` to push real-time updates to the UI and refresh caches/derived tables.

## Wiring the lap counter into the race manager
1. **Lap counter emitter** (ROS 2 node): publish a normalized lap event message containing car ID, gate crossing time, lap duration, heading validity, and on-track mask verdict.
2. **Bridge subscriber**: listen to the lap counter topic and call the `apps/racemanager/service` ingest endpoint; drop any lap marked invalid by anti-cheat flags.
3. **Service ingestion**:
   - Validate payload, inject `raceId`, compute `trackDistanceM` (from race config), and persist to `laps_live`.
   - Derive speed: `speedKph = (trackDistanceM / lapTimeMs) * 3.6`.
   - Emit websocket notification to subscribed UI clients.
4. **UI subscription**: the web UI opens a websocket to receive `lap`, `race_status`, and `standings` events; update per-car cards and leaderboard in place.

## Championship and per-driver stats (using track distance + lap times)
- **Average speed**: mean of `speedKph` across valid laps; display per lap and session average.
- **Best lap / sector delta**: keep rolling minimum lap time and show gap to best.
- **Consistency score**: standard deviation of lap times (lower is better) and a rolling “consistency %”.
- **Gap to leader**: compute cumulative race time per car; `gap = carTime - leaderTime`.
- **Points + standings**: apply scoring rules in `championships.scoringRules` after each race; update totals and podium counts.
- **Pace trend**: EMA of lap times to surface improvements/declines; store as derived field for quick charting.
- **Flags**: surface invalid laps (off-track, wrong direction, too fast) inline to explain why a lap was rejected.

## Prerecorded video replay for verification
- Allow the bridge to read an MP4 (captured from the same camera pose) and stream frames through the **exact** perception + anti-cheat pipeline used in real time.
- Tag every replayed lap with `source: "replay"`, a `replayId`, and the original video timestamp offset so the service can align it against live laps.
- Store replay laps in Atlas alongside live laps (e.g., `laps_live` with `source`/`replayId` fields) and expose a comparison view in the UI to diff lap counts, timestamps, and validity decisions.
- Use the replay path as a regression harness: the UI can toggle between "live" and "replay" data to validate timing math, heading checks, and minimum-lap-time thresholds without hardware on track.

## Deployment + dev workflow
- Run the bridge inside the ROS 2 workspace (`ros_ws/src/j5_race_bridge`) so it ships with the perception stack.
- Run the service & UI as Docker Compose in `apps/`; include `atlas_uri` and `websocket_public_url` in `.env`.
- For offline venues, buffer laps in the bridge (SQLite/flat file) and replay to the service once Atlas connectivity returns.
- Add `make race-manager` targets to spin up the service/UI locally with seeded fake lap streams for UI development.

## Bringing in `racetrack-master-pro`
If you want to leverage the richer `racetrack-master-pro` starter project, keep it co-located under `apps/racemanager` so the
bridge and ROS workspace can reference it predictably:

1. **Import**: add it as a git submodule or subtree at `apps/racemanager/racetrack-master-pro/`, or copy its `ui`/`server` pieces
   into `apps/racemanager/ui` and `apps/racemanager/service` to reuse the layout above.
2. **Adapt configuration**:
   - Point its Mongo/Atlas config to `RACE_DB_NAME` + Atlas URI used elsewhere in this repo.
   - Align websocket topics and REST endpoints with the ingest contract described in this doc (`/ingest/lap`, `lap`/`leaderboard`
     events) so the bridge can post laps without modification.
   - Map its data model fields to the lap counter payload (`carId`, `raceId`, `lapTimeMs`, `onTrack`, `headingOk`, `minLapTimeOk`,
     `trackDistanceM`, `speedKph`).
3. **Bridge integration**: ensure the ROS-side bridge pushes laps to whichever service binary you run from `racetrack-master-pro`.
   If that project already exposes a websocket, reuse it; otherwise add the ingest endpoint above and emit the same websocket
   events so the existing proof-of-concept UI continues to update.
4. **Shared replay path**: wire prerecorded MP4 ingestion through the same pipeline so both live and replay laps reach Atlas and
   the UI; tag replay laps with `source: "replay"` + `replayId` for comparison views.
5. **Testing**: run the lap counter in simulation/bag replay, feed laps through the bridge into the imported app, and confirm UI,
   standings, and championship rollups match the documented behaviors here.

Keeping `racetrack-master-pro` inside `apps/racemanager` preserves a single interface point to the ROS/perception side and
avoids duplicating bridge logic elsewhere in the workspace.


## Race setup wizard in `race-master-pro`

The `ros_ws/src/j5_perception/race-master-pro` app now includes an operator-first setup wizard in the **Settings** tab, paired with **Computer Vision** tab actions. The wizard enforces ordered checks and surfaces: the current blocking step, the exact shell command to run next, connect/stop buttons, and verification status for each stage.

Flow covered end-to-end:

1. ROS2 prerequisites and package visibility
2. Headless Raspberry Pi connectivity
3. Backend API health
4. MongoDB reachability on the Pi or LAN host
5. Camera preview + track image capture
6. Track markup: start line, finish line, and checkpoints
7. Car identity mapping: camera track IDs -> physical cars -> racers
8. Race initialization data (event/race/racers/laps)
9. Tracking start + lap log monitoring
10. Pause/snapshot/resume controls with state restoration

State is persisted server-side so laps counted, racer metadata, setup context, checkpoint geometry, and car-to-racer assignments can be restored after pause/resume.

### Expanded setup wizard checklist for camera-based race control
Add the following operator tasks to the race setup flow before pressing **Start Tracking**:

1. **Connect to MongoDB on the Pi**
   - Confirm MongoDB is running locally on the Raspberry Pi (`mongodb://localhost:27017`) or on a reachable LAN host.
   - Save the connection in the API service `.env` using `ATLAS_URI`/`MONGO_URI` style wiring, for example `mongodb://race:secret@<pi-ip>:27017/j5_racing?authSource=admin`.
   - Verify connectivity with `mongosh` and then through the backend `/health` endpoint before creating race records.
2. **Capture the track photo from the camera**
   - Use the Computer Vision preview to capture and persist the latest overhead image for the current venue.
   - Store the image path/URL and camera calibration values with the track record so future sessions can be reloaded without repeating calibration from scratch.
3. **Mark the racing geometry on the captured image**
   - Draw the outer/inner track boundary or mask.
   - Mark the **start line** and **finish line** (or a single bidirectional gate if the implementation uses one crossing line with direction validation).
   - Add **checkpoints / sector gates** around the lap so the tracker can verify that each car completed a full loop in order before a lap event is accepted.
4. **Register cars and bind them to racers for this race**
   - Create or reuse racer profiles.
   - Create a per-race car roster with fields such as `carId`, `displayName`, `paintColor`, `numberTag`, and optional appearance embedding metadata from the camera model.
   - Bind each detected tracker identity to a car, and each car to a racer entry in the current race. This mapping should be editable before the green flag and lock once the race starts unless an operator explicitly overrides it.
5. **Start object tracking with validation rules enabled**
   - Require ordered checkpoint traversal, on-track mask validation, forward direction at the finish gate, and minimum lap time checks before incrementing lap count.
   - Emit lap, sector, off-track, re-identification, and finish events into the API so the dashboard stays synchronized.
6. **Drive real-time dashboard updates**
   - Subscribe the dashboard to websocket events (`lap`, `leaderboard`, `race_status`, `car_state`, `checkpoint`, `incident`).
   - Refresh the track map, lap counter, racer cards, and leaderboard as soon as the API persists or validates a new event.

### MongoDB on Raspberry Pi setup steps
When MongoDB is installed on the Pi instead of Atlas, use the local deployment as the source of truth for race state:

1. **Start and verify MongoDB**
   ```bash
   sudo systemctl enable --now mongod
   sudo systemctl status mongod --no-pager
   mongosh --eval 'db.adminCommand({ ping: 1 })'
   ```
2. **Create the race database and application user**
   ```bash
   mongosh <<'EOF'
   use j5_racing
   db.createUser({
     user: 'race_api',
     pwd: 'change-me',
     roles: [{ role: 'readWrite', db: 'j5_racing' }]
   })
   EOF
   ```
3. **Point the race manager service at the Pi database**
   ```env
   ATLAS_URI=mongodb://race_api:change-me@<pi-ip>:27017/j5_racing?authSource=j5_racing
   RACE_DB_NAME=j5_racing
   LAPS_COLLECTION=laps_live
   ```
4. **Seed the core collections before first use**
   - `tracks`: geometry, calibration, mask, start/finish/checkpoints, dashboard overlay metadata.
   - `cars`: physical cars and vision-identification hints.
   - `racers`: people entered in the event.
   - `races`: the active race plus the per-race car/racer assignments.
   - `laps_live`, `telemetry_archive`, and `incidents`: streaming race events.

### Suggested API resources for visualization + CRUD
The API should expose CRUD and visualization payloads that line up with the setup steps above:

- `POST/GET/PUT/DELETE /api/tracks`
  - Include `layout_points`, `mask_polygon`, `start_gate`, `finish_gate`, and `checkpoints`.
- `POST/GET/PUT/DELETE /api/cameras`
  - Store camera source, calibration, snapshot URL, and preview metadata.
- `POST/GET/PUT/DELETE /api/racers`
  - Manage racer profiles used across multiple races.
- `POST/GET/PUT/DELETE /api/cars`
  - Manage physical car metadata and appearance tags used by the tracker.
- `POST/GET/PUT/DELETE /api/races`
  - Persist the race definition, lap target, and `entries: [{ racerId, carId, gridSlot, trackerHint }]`.
- `POST /api/races/{raceId}/tracking/start` and `POST /api/races/{raceId}/tracking/stop`
  - Toggle the perception pipeline for the configured camera.
- `POST /api/races/{raceId}/events`
  - Accept realtime `lap_count`, `checkpoint_crossing`, `off_track`, `finish_crossing`, and `car_assignment_changed` events.
- `GET /api/races/{raceId}/dashboard`
  - Return the aggregated payload needed by the frontend: race clock, leaderboard, track positions, checkpoint status, and recent incidents.

### How dashboard visuals should update
Use the backend as the only writer of authoritative race state, then broadcast deltas to the UI:

1. Perception emits detections and candidate crossings.
2. API validates checkpoint order and lap rules, then writes the accepted state change into MongoDB.
3. API publishes a websocket message with the updated `dashboard` and the incremental event.
4. Frontend store updates:
   - **Track canvas**: move each car marker, highlight crossed checkpoints, and paint the active start/finish gate.
   - **Leaderboard**: update lap count, last lap, best lap, gap to leader, and race position.
   - **Racer cards**: show car color/number, tracker confidence, penalties/incidents, and camera health.
   - **Control state**: disable reconfiguration actions after the race starts unless the operator pauses the session.

That setup gives you a clear race-day sequence: connect services -> capture track -> mark geometry -> assign cars/racers -> start tracking -> validate laps -> broadcast live dashboard updates.
