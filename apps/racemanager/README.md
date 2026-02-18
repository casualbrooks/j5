# Race Manager (apps/racemanager)

This folder holds the race-management web UI, backend service, and ROS bridge that connect the lap counter to MongoDB Atlas and keep live race/championship views updated. A minimal FastAPI service implementation now lives in `service/` and is ready to ingest laps from the bridge.

## Layout
- `ui/`: Web client (e.g., React/Next.js) that subscribes to websocket updates and renders live leaderboards + championship tables.
- `service/`: API + websocket fan-out (FastAPI starter) that ingests validated lap events, writes to Atlas, and computes derived stats.
- `bridge/`: ROS 2/vision-side process that consumes lap counter topics and posts vetted laps to the service.
- `scripts/`: Seeders/loaders for fake races, migrations, and local fixtures.

## Environment
Create per-component `.env` files; the service needs at minimum:

```
ATLAS_URI=mongodb+srv://<user>:<password>@cluster...
RACE_DB_NAME=j5_racing
LAPS_COLLECTION=laps_live
WS_PUBLIC_URL=ws://localhost:4000/ws
HTTP_PORT=4000
# Optional auth
JWT_SECRET=change-me
```

For the FastAPI service, copy `apps/racemanager/service/.env.example` to `.env` in
the same folder and fill in your MongoDB connection string and collection
names. The config loader reads that file automatically on startup.

Quickstart for the service (from the repo root):

1. `cp apps/racemanager/service/.env.example apps/racemanager/service/.env` and
   edit the values
2. `python -m venv apps/racemanager/service/.venv && source apps/racemanager/service/.venv/bin/activate`
3. `pip install -r apps/racemanager/service/requirements.txt`
4. `python -m uvicorn apps.racemanager.service.main:app --reload --env-file apps/racemanager/service/.env`

The UI should point to the same HTTP/websocket endpoints (e.g.,
`NEXT_PUBLIC_API_BASE`, `NEXT_PUBLIC_WS_URL`).

## Data flow (live + replay)
1. **Lap counter node (ROS 2)** publishes validated laps: `{ carId, gateTime, lapTimeMs, headingOk, onTrack, minLapTimeOk }`.
2. **Bridge** (`bridge/`) subscribes to the lap topic, attaches `raceId`/`seasonId`, and posts to `service/ingest` (HTTP or gRPC). Reject laps that fail anti-cheat flags. The bridge can also stream laps detected from prerecorded videos into the same endpoint with `source="replay"`.
3. **Service** persists to Atlas collections (`laps_live`, `races`, `championships`, `telemetry_archive`) and emits websocket events `{ type: "lap", payload: {...} }` to UI clients.
4. **UI** listens for `lap`, `race_status`, and `standings` events to repaint per-car cards and leaderboards in real time.

## Derived stats (track distance + lap times)
The service should enrich each lap with:
- `trackDistanceM`: pulled from race config.
- `speedKph`: `(trackDistanceM / lapTimeMs) * 3.6`.
- `isValid`: `onTrack && headingOk && minLapTimeOk`.

Expose per-car aggregates that the UI can bind to existing proof-of-concept fields:
- Lap count, last lap, best lap, average lap time, and rolling median.
- Average speed per lap and session average.
- Delta to leader: `gapMs = carElapsedMs - leaderElapsedMs`.
- Consistency: standard deviation of recent N lap times + a percentage (e.g., `1 - (stdev/mean)`).
- Championship rollups: accumulate points/podiums/wins/best-lap counts into `championships.standings` after each race.

## Suggested endpoints/events (implemented in FastAPI starter)
- `POST /ingest/lap`: accepts the payload above plus `raceId`, `sessionLap`, and optional `sectorTimes`.
- `GET /races/:raceId/standings`: returns leaderboard with computed gaps and best laps.
- `GET /health` / root: sanity endpoints for probes.
- Websocket events: `lap`, `leaderboard`, `race_status`, `championship` (stubbed; add via FastAPI websockets or a sidecar socket server).

## Local development
- Start Atlas tunnel or use a local Mongo instance with the same schemas.
- **Run the service**: `python -m venv apps/racemanager/service/.venv && source apps/racemanager/service/.venv/bin/activate && pip install -r apps/racemanager/service/requirements.txt && python -m uvicorn apps.racemanager.service.main:app --reload --env-file apps/racemanager/service/.env`.
- **Smoke test ingest**: from another shell run `python apps/racemanager/bridge/bridge.py` to push a demo lap into the service (requires Mongo running).
- **Run the UI**: `cd apps/racemanager/ui && npm install && npm run dev`, pointing to the service URLs above.
- **ROS-side testing without hardware**: feed recorded lap messages through the bridge (bag replay) and confirm the UI updates instantly.
- **Perception regression testing**: point the bridge replay runner to a prerecorded MP4; forward the resulting laps with `source="replay"`/`replayId` so the service/UI can compare them against live runs.

## Pushing the `race-manager` branch to GitHub
If the branch was recreated locally and you need to publish it to your GitHub fork or the upstream origin, do the following from the repo root:

1. Fetch the latest remotes so your local branch has the current refs: `git fetch --all --prune`.
2. Ensure you are on the branch you want to publish (recreate if needed): `git checkout race-manager` (or `git switch race-manager`).
3. If the branch should track `origin/race-manager`, set upstream on first push: `git push -u origin race-manager`.
4. For subsequent pushes just run `git push` and Git will use the tracked upstream.
5. Verify the branch exists on GitHub: `git ls-remote --heads origin race-manager` should show the new ref.

## Using `racetrack-master-pro` with this layout
- **Where to place it**: keep the upstream project under `apps/racemanager/racetrack-master-pro/` (submodule/subtree) or fold its UI/service code into `apps/racemanager/ui` and `apps/racemanager/service` so the bridge path stays unchanged.
- **Endpoints to align**: ensure it exposes `POST /ingest/lap` plus websocket topics `lap`, `leaderboard`, `race_status`, and `championship` so the bridge and UI continue to interoperate.
- **Schema mapping**: map incoming lap fields to the lap counter payload `{ carId, raceId, lapTimeMs, onTrack, headingOk, minLapTimeOk, trackDistanceM, speedKph }` and store them in the Atlas collections noted above.
- **Replay parity**: feed prerecorded MP4s through the same ingestion endpoint with `source: "replay"` and `replayId` to validate against live sessions.

## Standalone + ROS 2 bridge modes

The bridge now supports both non-ROS and ROS 2 execution so you can run on a
single mini PC with only a USB camera and add ROS later.

### Standalone (no ROS 2 required)
1. Start the API service with SQLite (default):
   ```bash
   cp apps/racemanager/service/.env.example apps/racemanager/service/.env
   python -m venv apps/racemanager/service/.venv
   source apps/racemanager/service/.venv/bin/activate
   pip install -r apps/racemanager/service/requirements.txt
   uvicorn apps.racemanager.service.main:app --reload --env-file apps/racemanager/service/.env
   ```
2. In another shell, run a demo lap feed:
   ```bash
   python apps/racemanager/bridge/bridge.py --mode demo --service-url http://localhost:4000
   ```
3. Query leaderboard:
   ```bash
   curl http://localhost:4000/races/demo-race/leaderboard
   ```

### ROS 2 mode (when available)
1. Source ROS 2:
   ```bash
   source /opt/ros/iron/setup.bash
   ```
2. Run bridge subscriber:
   ```bash
   python apps/racemanager/bridge/bridge.py --mode ros2 --topic /race/lap_event
   ```
3. Publish JSON lap event on topic for smoke test:
   ```bash
   ros2 topic pub /race/lap_event std_msgs/msg/String '{data: "{\"raceId\":\"demo-race\",\"carId\":\"car-7\",\"sessionLap\":1,\"lapTimeMs\":70000,\"trackDistanceM\":350.0,\"validity\":{\"onTrack\":true,\"directionOk\":true,\"minLapTimeOk\":true}}"}' --once
   ```

### Websocket updates for GUI
- Connect GUI clients to `ws://<host>:4000/ws`.
- Service broadcasts:
  - `type="lap"` after each ingest
  - `type="leaderboard"` with recalculated standings
