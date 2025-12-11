# Race Manager UI

A minimal Next.js placeholder UI for the race manager FastAPI backend. Configure the client to point at your running API:

- `NEXT_PUBLIC_API_BASE=http://localhost:4000`
- `NEXT_PUBLIC_WS_URL=ws://localhost:4000/ws`

Subscribe to websocket events (`lap`, `leaderboard`, `race_status`, `championship`) and render leaderboards plus championship tables.

## Getting started

1. Install a Node.js runtime that bundles a recent `npm`. Using [`nvm`](https://github.com/nvm-sh/nvm) with the provided `.nvmrc` keeps you on the tested Node 18.17.x line:

   ```bash
   nvm install
   nvm use
   corepack prepare npm@10.9.2 --activate
   ```

2. Install dependencies and start the dev server:

   ```bash
   cd apps/racemanager/ui
   npm install
   npm run dev
   ```

   If you see `TypeError: Class extends value undefined is not a constructor or null` from a system-wide `npm`, switch to the Corepack-activated version above to avoid distro-packaged `npm` bugs.

Then open http://localhost:3000. Environment variables can be provided in a `.env.local` file.
