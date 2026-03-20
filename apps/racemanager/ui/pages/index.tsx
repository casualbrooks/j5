import Head from 'next/head';
import { useEffect, useMemo, useState } from 'react';
import styles from '../styles/Home.module.css';

const apiBase = process.env.NEXT_PUBLIC_API_BASE ?? 'http://localhost:4000';
const wsUrl = process.env.NEXT_PUBLIC_WS_URL ?? 'ws://localhost:4000/ws';

type Point = { x: number; y: number };
type LineGate = { a: Point; b: Point; label?: string | null };
type Checkpoint = { id: string; name: string; type: string; position: Point; order: number };
type Track = {
  id: string;
  name: string;
  imageUrl?: string | null;
  layoutPoints: Point[];
  maskPolygon: Point[];
  startGate?: LineGate | null;
  finishGate?: LineGate | null;
  checkpoints: Checkpoint[];
};
type Race = { id: string; name: string; totalLaps: number; status: string };
type LeaderboardEntry = {
  carId: string;
  lapCount: number;
  bestLapMs: number;
  avgSpeedKph: number;
  totalTimeMs: number;
  gapToLeaderMs: number;
};
type DashboardEntry = {
  carId: string;
  racerId?: string | null;
  carName: string;
  racerName: string;
  carNumber: string;
  paintColor: string;
  trackerId?: string | null;
  currentLap: number;
  bestLapMs?: number | null;
  avgSpeedKph?: number | null;
  gapToLeaderMs: number;
  totalTimeMs: number;
};
type RaceEvent = {
  id: string;
  type: string;
  carId?: string | null;
  racerId?: string | null;
  checkpointId?: string | null;
  lapNumber?: number | null;
  timestamp: string;
  payload?: Record<string, unknown>;
};
type Dashboard = {
  race: Race;
  track?: Track | null;
  leaderboard: { leaderboard: LeaderboardEntry[] };
  entries: DashboardEntry[];
  recentEvents: RaceEvent[];
};

const formatMs = (value?: number | null) => {
  if (!value) return '—';
  const seconds = value / 1000;
  return `${seconds.toFixed(2)}s`;
};

const formatGap = (value?: number | null) => {
  if (!value) return 'Leader';
  return `+${(value / 1000).toFixed(2)}s`;
};

function useTrackBounds(track?: Track | null) {
  return useMemo(() => {
    const points: Point[] = [];
    if (track?.layoutPoints?.length) points.push(...track.layoutPoints);
    if (track?.maskPolygon?.length) points.push(...track.maskPolygon);
    if (track?.checkpoints?.length) points.push(...track.checkpoints.map((item) => item.position));
    if (track?.startGate) points.push(track.startGate.a, track.startGate.b);
    if (track?.finishGate) points.push(track.finishGate.a, track.finishGate.b);

    if (!points.length) {
      return { minX: 0, minY: 0, width: 100, height: 100 };
    }

    const xs = points.map((point) => point.x);
    const ys = points.map((point) => point.y);
    const minX = Math.min(...xs);
    const minY = Math.min(...ys);
    const maxX = Math.max(...xs);
    const maxY = Math.max(...ys);
    return {
      minX,
      minY,
      width: Math.max(maxX - minX, 1),
      height: Math.max(maxY - minY, 1),
    };
  }, [track]);
}

function TrackPreview({ track, entries }: { track?: Track | null; entries: DashboardEntry[] }) {
  const bounds = useTrackBounds(track);
  const scalePoint = (point: Point) => ({
    x: ((point.x - bounds.minX) / bounds.width) * 100,
    y: ((point.y - bounds.minY) / bounds.height) * 100,
  });

  const polyline = track?.layoutPoints?.length
    ? track.layoutPoints.map((point) => {
        const scaled = scalePoint(point);
        return `${scaled.x},${scaled.y}`;
      }).join(' ')
    : '';

  const polygon = track?.maskPolygon?.length
    ? track.maskPolygon.map((point) => {
        const scaled = scalePoint(point);
        return `${scaled.x},${scaled.y}`;
      }).join(' ')
    : '';

  return (
    <div className={styles.trackCard}>
      <div className={styles.panelHeader}>
        <div>
          <h2>Track & checkpoints</h2>
          <p>Start/finish geometry, checkpoints, and live race entries for the selected race.</p>
        </div>
        {track?.imageUrl ? <a href={track.imageUrl} target="_blank" rel="noreferrer">Track photo</a> : null}
      </div>
      <div className={styles.trackViewport}>
        <svg viewBox="0 0 100 100" className={styles.trackSvg} preserveAspectRatio="none">
          {polygon ? <polygon points={polygon} className={styles.maskPolygon} /> : null}
          {polyline ? <polyline points={polyline} className={styles.layoutPolyline} /> : null}
          {track?.startGate ? (() => {
            const a = scalePoint(track.startGate.a);
            const b = scalePoint(track.startGate.b);
            return <line x1={a.x} y1={a.y} x2={b.x} y2={b.y} className={styles.startGate} />;
          })() : null}
          {track?.finishGate ? (() => {
            const a = scalePoint(track.finishGate.a);
            const b = scalePoint(track.finishGate.b);
            return <line x1={a.x} y1={a.y} x2={b.x} y2={b.y} className={styles.finishGate} />;
          })() : null}
          {track?.checkpoints?.map((checkpoint) => {
            const point = scalePoint(checkpoint.position);
            return (
              <g key={checkpoint.id}>
                <circle cx={point.x} cy={point.y} r="2.2" className={styles.checkpointDot} />
                <text x={point.x + 2.5} y={point.y - 2.5} className={styles.checkpointLabel}>
                  {checkpoint.name}
                </text>
              </g>
            );
          })}
          {entries.map((entry, index) => {
            const checkpoint = track?.checkpoints?.[index % Math.max(track?.checkpoints?.length || 1, 1)];
            const point = checkpoint ? scalePoint(checkpoint.position) : { x: 10 + index * 10, y: 80 - index * 8 };
            return (
              <g key={`${entry.carId}-${entry.racerId || 'na'}`}>
                <circle cx={point.x} cy={point.y} r="3.2" fill={entry.paintColor || '#4ade80'} className={styles.entryDot} />
                <text x={point.x} y={point.y + 8} textAnchor="middle" className={styles.entryLabel}>
                  #{entry.carNumber || entry.carId}
                </text>
              </g>
            );
          })}
        </svg>
        {!track ? <div className={styles.emptyTrack}>Create a track with geometry/checkpoints to render the race map.</div> : null}
      </div>
    </div>
  );
}

export default function Home() {
  const [races, setRaces] = useState<Race[]>([]);
  const [selectedRaceId, setSelectedRaceId] = useState<string>('');
  const [dashboard, setDashboard] = useState<Dashboard | null>(null);
  const [connectionState, setConnectionState] = useState<'connecting' | 'connected' | 'disconnected'>('connecting');

  useEffect(() => {
    let cancelled = false;
    async function loadRaces() {
      const response = await fetch(`${apiBase}/api/races`);
      if (!response.ok || cancelled) return;
      const payload = await response.json() as Race[];
      setRaces(payload);
      if (!selectedRaceId && payload[0]?.id) {
        setSelectedRaceId(payload[0].id);
      }
    }

    void loadRaces();
    return () => {
      cancelled = true;
    };
  }, [selectedRaceId]);

  useEffect(() => {
    if (!selectedRaceId) return;
    let cancelled = false;
    async function loadDashboard() {
      const response = await fetch(`${apiBase}/api/races/${selectedRaceId}/dashboard`);
      if (!response.ok || cancelled) return;
      const payload = await response.json() as Dashboard;
      setDashboard(payload);
    }

    void loadDashboard();
    return () => {
      cancelled = true;
    };
  }, [selectedRaceId]);

  useEffect(() => {
    const socket = new WebSocket(wsUrl);
    setConnectionState('connecting');

    socket.onopen = () => setConnectionState('connected');
    socket.onclose = () => setConnectionState('disconnected');
    socket.onerror = () => setConnectionState('disconnected');
    socket.onmessage = (event) => {
      try {
        const message = JSON.parse(event.data) as { type?: string; payload?: Dashboard & { raceId?: string } };
        if (message.type === 'dashboard' && message.payload?.race?.id === selectedRaceId) {
          setDashboard(message.payload as Dashboard);
        }
        if ((message.type === 'lap' || message.type === 'event' || message.type === 'leaderboard') && selectedRaceId) {
          void fetch(`${apiBase}/api/races/${selectedRaceId}/dashboard`)
            .then((response) => response.ok ? response.json() : null)
            .then((payload) => {
              if (payload) setDashboard(payload as Dashboard);
            });
        }
      } catch {
        // ignore malformed payloads
      }
    };

    const keepAlive = window.setInterval(() => {
      if (socket.readyState === WebSocket.OPEN) {
        socket.send('ping');
      }
    }, 15000);

    return () => {
      window.clearInterval(keepAlive);
      socket.close();
    };
  }, [selectedRaceId]);

  return (
    <div className={styles.container}>
      <Head>
        <title>Race Manager UI</title>
        <meta
          name="description"
          content="Realtime dashboard for race setup, CRUD-backed race data, and websocket lap updates"
        />
      </Head>

      <main className={styles.main}>
        <header className={styles.hero}>
          <div>
            <h1 className={styles.title}>Race Manager Dashboard</h1>
            <p className={styles.description}>
              API: <code>{apiBase}</code> · WebSocket: <code>{wsUrl}</code>
            </p>
          </div>
          <div className={styles.connectionBadge} data-state={connectionState}>
            {connectionState}
          </div>
        </header>

        <section className={styles.toolbar}>
          <label className={styles.selectorLabel}>
            Active race
            <select value={selectedRaceId} onChange={(event) => setSelectedRaceId(event.target.value)}>
              <option value="">Select a race</option>
              {races.map((race) => (
                <option key={race.id} value={race.id}>
                  {race.name} · {race.status} · {race.totalLaps} laps
                </option>
              ))}
            </select>
          </label>
          <div className={styles.toolbarText}>
            Use the API to CRUD tracks/cars/racers/races, then watch lap + checkpoint events refresh this dashboard in real time.
          </div>
        </section>

        {!dashboard ? (
          <section className={styles.emptyState}>
            <h2>No dashboard loaded</h2>
            <p>Create race records through the new CRUD API (`/api/tracks`, `/api/cars`, `/api/racers`, `/api/races`) and select a race to visualize it here.</p>
          </section>
        ) : (
          <div className={styles.grid}>
            <TrackPreview track={dashboard.track} entries={dashboard.entries} />

            <section className={styles.panel}>
              <div className={styles.panelHeader}>
                <div>
                  <h2>{dashboard.race.name}</h2>
                  <p>{dashboard.race.status} · {dashboard.race.totalLaps} total laps</p>
                </div>
              </div>
              <div className={styles.entryList}>
                {dashboard.entries.map((entry) => (
                  <article key={`${entry.carId}-${entry.racerId || 'na'}`} className={styles.entryCard}>
                    <div className={styles.entryIdentity}>
                      <span className={styles.colorSwatch} style={{ backgroundColor: entry.paintColor }} />
                      <div>
                        <strong>{entry.racerName}</strong>
                        <p>{entry.carName} · #{entry.carNumber || entry.carId}</p>
                      </div>
                    </div>
                    <dl className={styles.entryStats}>
                      <div><dt>Lap</dt><dd>{entry.currentLap}</dd></div>
                      <div><dt>Best</dt><dd>{formatMs(entry.bestLapMs)}</dd></div>
                      <div><dt>Gap</dt><dd>{formatGap(entry.gapToLeaderMs)}</dd></div>
                      <div><dt>Avg speed</dt><dd>{entry.avgSpeedKph ? `${entry.avgSpeedKph.toFixed(1)} kph` : '—'}</dd></div>
                    </dl>
                  </article>
                ))}
              </div>
            </section>

            <section className={styles.panel}>
              <div className={styles.panelHeader}>
                <div>
                  <h2>Leaderboard</h2>
                  <p>Validated lap counts and total race time from persisted backend state.</p>
                </div>
              </div>
              <div className={styles.tableWrapper}>
                <table className={styles.table}>
                  <thead>
                    <tr>
                      <th>Car</th>
                      <th>Laps</th>
                      <th>Best</th>
                      <th>Avg speed</th>
                      <th>Gap</th>
                    </tr>
                  </thead>
                  <tbody>
                    {dashboard.leaderboard.leaderboard.map((entry) => (
                      <tr key={entry.carId}>
                        <td>{entry.carId}</td>
                        <td>{entry.lapCount}</td>
                        <td>{formatMs(entry.bestLapMs)}</td>
                        <td>{entry.avgSpeedKph.toFixed(1)} kph</td>
                        <td>{formatGap(entry.gapToLeaderMs)}</td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              </div>
            </section>

            <section className={styles.panel}>
              <div className={styles.panelHeader}>
                <div>
                  <h2>Recent events</h2>
                  <p>Checkpoint, lap-count, incident, and finish events streamed from the API.</p>
                </div>
              </div>
              <div className={styles.eventList}>
                {dashboard.recentEvents.map((event) => (
                  <article key={event.id} className={styles.eventItem}>
                    <div>
                      <strong>{event.type}</strong>
                      <p>{event.carId || '—'} · {event.checkpointId || 'no checkpoint'} · lap {event.lapNumber ?? '—'}</p>
                    </div>
                    <time>{new Date(event.timestamp).toLocaleTimeString()}</time>
                  </article>
                ))}
              </div>
            </section>
          </div>
        )}
      </main>
    </div>
  );
}
