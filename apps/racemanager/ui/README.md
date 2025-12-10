# Race Manager UI

Placeholder for the web client. Point your app to the FastAPI service under `apps/racemanager/service`:

- `NEXT_PUBLIC_API_BASE=http://localhost:4000`
- `NEXT_PUBLIC_WS_URL=ws://localhost:4000/ws`

Subscribe to websocket events (`lap`, `leaderboard`, `race_status`, `championship`) and render leaderboards plus championship tables.
