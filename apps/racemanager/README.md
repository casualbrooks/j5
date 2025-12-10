# Race Manager (apps/racemanager)

This folder holds the race-management web UI, backend service, and ROS bridge that connect the lap counter to MongoDB Atlas and keep live race/championship views updated. A minimal FastAPI service implementation now lives in `service/` and is ready to ingest laps from the bridge. Contributions here should target the `race-manager` branch (not `main`) so the core platform stays clean for users who only want the base J5 stack. The PR template includes a branch checklist—tick the `race-manager` box for changes in this folder. If a PR was opened against `main` by accident, edit the PR in GitHub and change the base to `race-manager` before review.

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
HTTP_HOST=0.0.0.0
HTTP_PORT=4000
# Optional auth
JWT_SECRET=change-me
```

The UI should point to the same HTTP/websocket endpoints (e.g., `NEXT_PUBLIC_API_BASE`, `NEXT_PUBLIC_WS_URL`).

## Data flow (live + replay)
1. **Lap counter node (ROS 2)** publishes validated laps: `{ carId, gateTime, lapTimeMs, headingOk, onTrack, minLapTimeOk }`.
2. **Bridge** (`bridge/`) subscribes to the lap topic, attaches `raceId`/`seasonId`, and posts to `service/ingest` (HTTP or gRPC). Reject laps that fail anti-cheat flags. The bridge can also stream laps detected from prerecorded videos into the same endpoint with `source="replay"`.
3. **Service** persists to Atlas collections (`laps_live`, `races`, `championships`, `telemetry_archive`) and emits websocket events `{ type: "lap", payload: {...} }` to UI clients.
4. **UI** listens for `lap`, `race_status`, and `standings` events to repaint per-car cards and leaderboards in real time.

## Derived stats (track distance + lap times)
The service enriches each lap with:
- `trackDistanceM`: pulled from race config.
- `speedKph`: `(trackDistanceM / lapTimeMs) * 3.6`.
- `isValid`: `onTrack && headingOk && minLapTimeOk`.

Per-car aggregates served by the FastAPI starter (filtered to valid laps by default):
- Lap count, last lap, best lap, average lap time, and rolling median.
- Average speed per lap and session average, plus last-lap speed.
- Delta to leader: `gapMs = carElapsedMs - leaderElapsedMs`.
- Consistency: standard deviation of recent N lap times + a percentage (e.g., `1 - (stdev/mean)`).
- Championship rollups: accumulate points/podiums/wins/best-lap counts into `championships.standings` after each race.

## Suggested endpoints/events (implemented in FastAPI starter)
- `POST /ingest/lap`: accepts the payload above plus `raceId`, `sessionLap`, and optional `sectorTimes`.
- `GET /races/:raceId/standings`: returns leaderboard with computed gaps and best laps.
- `GET /races/:raceId/leaderboard?include_replay=0&include_invalid=0`: aggregates lap count, best/average/last lap, last speed, and gap to leader. Replay laps are excluded by default; pass `include_replay=1` to mix them into the standings.
- `GET /health` / root: sanity endpoints for probes.
- Websocket events: `lap`, `leaderboard`, `race_status`, `championship` (stubbed; add via FastAPI websockets or a sidecar socket server).

## Local development
- Start Atlas tunnel or use a local Mongo instance with the same schemas.
- **Run the service**: `cd apps/racemanager/service && python -m venv .venv && source .venv/bin/activate && pip install -r requirements.txt && python -m apps.racemanager.service` (honors `HTTP_HOST`/`HTTP_PORT`).
- **Smoke test ingest**: from another shell run `python apps/racemanager/bridge/bridge.py` to push a demo lap into the service (requires Mongo running).
- **Run the UI**: `cd apps/racemanager/ui && npm install && npm run dev`, pointing to the service URLs above.
- **ROS-side testing without hardware**: feed recorded lap messages through the bridge (bag replay) and confirm the UI updates instantly.
- **Perception regression testing**: point the bridge replay runner to a prerecorded MP4; forward the resulting laps with `source="replay"`/`replayId` so the service/UI can compare them against live runs.

## Using `racetrack-master-pro` with this layout
- **Where to place it**: keep the upstream project under `apps/racemanager/racetrack-master-pro/` (submodule/subtree) or fold its UI/service code into `apps/racemanager/ui` and `apps/racemanager/service` so the bridge path stays unchanged.
- **Endpoints to align**: ensure it exposes `POST /ingest/lap` plus websocket topics `lap`, `leaderboard`, `race_status`, and `championship` so the bridge and UI continue to interoperate.
- **Schema mapping**: map incoming lap fields to the lap counter payload `{ carId, raceId, lapTimeMs, onTrack, headingOk, minLapTimeOk, trackDistanceM, speedKph }` and store them in the Atlas collections noted above.
- **Replay parity**: feed prerecorded MP4s through the same ingestion endpoint with `source: "replay"` and `replayId` to validate against live sessions.
