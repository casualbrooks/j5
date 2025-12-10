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

### Service endpoints
- `POST /ingest/lap`: validate a lap payload, derive speed, and persist with validity flags.
- `GET /races/{raceId}/leaderboard?include_replay=0&include_invalid=0`: aggregates lap count, best/average/last lap, last speed, cumulative time, and gap to leader. Replay laps and invalid laps stay excluded unless the query params are flipped for debugging.

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
- **Gap to leader**: compute cumulative race time per car; `gap = carTime - leaderTime`. Replay laps stay out of the gap unless explicitly requested on the leaderboard endpoint.
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
