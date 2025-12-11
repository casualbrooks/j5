# Race Manager UI

A minimal Next.js placeholder UI for the race manager FastAPI backend. Configure the client to point at your running API:

- `NEXT_PUBLIC_API_BASE=http://localhost:4000`
- `NEXT_PUBLIC_WS_URL=ws://localhost:4000/ws`

Subscribe to websocket events (`lap`, `leaderboard`, `race_status`, `championship`) and render leaderboards plus championship tables.

## Getting started

```bash
cd apps/racemanager/ui
npm install
npm run dev
```

Then open http://localhost:3000. Environment variables can be provided in a `.env.local` file.
