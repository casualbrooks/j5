import Head from 'next/head';
import { MouseEvent, useEffect, useMemo, useState } from 'react';
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
  calibration?: Record<string, unknown>;
};
type Race = { id: string; name: string; totalLaps: number; status: string; trackId?: string | null };
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

type TrackDraft = {
  id?: string;
  name: string;
  imageUrl: string;
  layoutPoints: Point[];
  maskPolygon: Point[];
  checkpoints: Checkpoint[];
  startGate?: LineGate | null;
  finishGate?: LineGate | null;
  calibration: { startPosition?: Point; direction?: string };
};

type PlacementTool =
  | 'layout'
  | 'mask'
  | 'checkpoint'
  | 'start_a'
  | 'start_b'
  | 'finish_a'
  | 'finish_b'
  | 'start_position';

const formatMs = (value?: number | null) => {
  if (!value) return '—';
  const seconds = value / 1000;
  return `${seconds.toFixed(2)}s`;
};

const formatGap = (value?: number | null) => {
  if (!value) return 'Leader';
  return `+${(value / 1000).toFixed(2)}s`;
};

const defaultDraft = (): TrackDraft => ({
  name: 'New track',
  imageUrl: '',
  layoutPoints: [],
  maskPolygon: [],
  checkpoints: [],
  startGate: { a: { x: 10, y: 50 }, b: { x: 20, y: 50 }, label: 'start' },
  finishGate: { a: { x: 80, y: 50 }, b: { x: 90, y: 50 }, label: 'finish' },
  calibration: { direction: 'clockwise' },
});

const asTrackDraft = (track?: Track | null): TrackDraft => {
  if (!track) return defaultDraft();
  return {
    id: track.id,
    name: track.name,
    imageUrl: track.imageUrl ?? '',
    layoutPoints: [...(track.layoutPoints ?? [])],
    maskPolygon: [...(track.maskPolygon ?? [])],
    checkpoints: [...(track.checkpoints ?? [])],
    startGate: track.startGate ?? { a: { x: 10, y: 50 }, b: { x: 20, y: 50 }, label: 'start' },
    finishGate: track.finishGate ?? { a: { x: 80, y: 50 }, b: { x: 90, y: 50 }, label: 'finish' },
    calibration: {
      startPosition: (track.calibration?.startPosition as Point | undefined),
      direction: typeof track.calibration?.direction === 'string' ? track.calibration.direction : 'clockwise',
    },
  };
};

function useTrackBounds(track?: Track | null, draft?: TrackDraft | null) {
  return useMemo(() => {
    const points: Point[] = [];
    const sourceTrack = track ?? undefined;
    if (sourceTrack?.layoutPoints?.length) points.push(...sourceTrack.layoutPoints);
    if (sourceTrack?.maskPolygon?.length) points.push(...sourceTrack.maskPolygon);
    if (sourceTrack?.checkpoints?.length) points.push(...sourceTrack.checkpoints.map((item) => item.position));
    if (sourceTrack?.startGate) points.push(sourceTrack.startGate.a, sourceTrack.startGate.b);
    if (sourceTrack?.finishGate) points.push(sourceTrack.finishGate.a, sourceTrack.finishGate.b);

    if (draft?.layoutPoints?.length) points.push(...draft.layoutPoints);
    if (draft?.maskPolygon?.length) points.push(...draft.maskPolygon);
    if (draft?.checkpoints?.length) points.push(...draft.checkpoints.map((item) => item.position));

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
  }, [track, draft]);
}

function toPercentPoint(raw: Point, bounds: { minX: number; minY: number; width: number; height: number }) {
  return {
    x: ((raw.x - bounds.minX) / bounds.width) * 100,
    y: ((raw.y - bounds.minY) / bounds.height) * 100,
  };
}

function pickCarPositions(track: Track | null | undefined, entries: DashboardEntry[], events: RaceEvent[]) {
  const checkpointById = new Map((track?.checkpoints ?? []).map((checkpoint) => [checkpoint.id, checkpoint.position]));
  const latestByCar = new Map<string, Point>();

  events.forEach((event) => {
    if (!event.carId || latestByCar.has(event.carId)) return;
    const payloadPoint = event.payload && typeof event.payload.x === 'number' && typeof event.payload.y === 'number'
      ? { x: Number(event.payload.x), y: Number(event.payload.y) }
      : null;
    if (payloadPoint) {
      latestByCar.set(event.carId, payloadPoint);
      return;
    }
    if (event.checkpointId && checkpointById.has(event.checkpointId)) {
      latestByCar.set(event.carId, checkpointById.get(event.checkpointId)!);
    }
  });

  return entries.map((entry, index) => {
    const fallbackCheckpoint = track?.checkpoints?.[index % Math.max(track?.checkpoints?.length || 1, 1)]?.position;
    const fallbackLayout = track?.layoutPoints?.[index % Math.max(track?.layoutPoints?.length || 1, 1)];
    const point = latestByCar.get(entry.carId)
      ?? fallbackCheckpoint
      ?? fallbackLayout
      ?? { x: 10 + index * 10, y: 80 - index * 8 };
    return { entry, point };
  });
}

function TrackPreview({ track, entries, events }: { track?: Track | null; entries: DashboardEntry[]; events: RaceEvent[] }) {
  const bounds = useTrackBounds(track);
  const carMarkers = pickCarPositions(track, entries, events);
  const scalePoint = (point: Point) => toPercentPoint(point, bounds);

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
          <h2>Track & live camera overlay</h2>
          <p>Track image, path geometry, checkpoints, and moving car annotations from live events.</p>
        </div>
        {track?.imageUrl ? <a href={track.imageUrl} target="_blank" rel="noreferrer">Track photo</a> : null}
      </div>
      <div className={styles.trackViewport}>
        <svg viewBox="0 0 100 100" className={styles.trackSvg} preserveAspectRatio="none">
          {track?.imageUrl ? <image href={track.imageUrl} x="0" y="0" width="100" height="100" preserveAspectRatio="none" className={styles.trackImage} /> : null}
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
          {carMarkers.map(({ entry, point }) => {
            const scaled = scalePoint(point);
            return (
              <g key={`${entry.carId}-${entry.racerId || 'na'}`}>
                <circle cx={scaled.x} cy={scaled.y} r="3.2" fill={entry.paintColor || '#4ade80'} className={styles.entryDot} />
                <text x={scaled.x} y={scaled.y + 8} textAnchor="middle" className={styles.entryLabel}>
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

function TrackSettings({ tracks, selectedTrackId, setSelectedTrackId, onSaved }: {
  tracks: Track[];
  selectedTrackId: string;
  setSelectedTrackId: (id: string) => void;
  onSaved: (track: Track) => void;
}) {
  const [tool, setTool] = useState<PlacementTool>('layout');
  const [draft, setDraft] = useState<TrackDraft>(defaultDraft());
  const [saveState, setSaveState] = useState<'idle' | 'saving' | 'saved' | 'error'>('idle');

  useEffect(() => {
    const selected = tracks.find((track) => track.id === selectedTrackId);
    setDraft(asTrackDraft(selected));
  }, [selectedTrackId, tracks]);

  const bounds = useTrackBounds(undefined, draft);
  const scalePoint = (point: Point) => toPercentPoint(point, bounds);
  const unscalePoint = (point: Point): Point => ({
    x: bounds.minX + (point.x / 100) * bounds.width,
    y: bounds.minY + (point.y / 100) * bounds.height,
  });

  const onCanvasClick = (event: MouseEvent<SVGSVGElement>) => {
    const rect = event.currentTarget.getBoundingClientRect();
    const clickPercent = {
      x: ((event.clientX - rect.left) / rect.width) * 100,
      y: ((event.clientY - rect.top) / rect.height) * 100,
    };
    const clickRaw = unscalePoint(clickPercent);

    setDraft((current) => {
      if (tool === 'layout') return { ...current, layoutPoints: [...current.layoutPoints, clickRaw] };
      if (tool === 'mask') return { ...current, maskPolygon: [...current.maskPolygon, clickRaw] };
      if (tool === 'checkpoint') {
        const next = current.checkpoints.length + 1;
        return {
          ...current,
          checkpoints: [
            ...current.checkpoints,
            {
              id: `cp-${next}`,
              name: `CP ${next}`,
              type: 'checkpoint',
              order: next,
              position: clickRaw,
            },
          ],
        };
      }
      if (tool === 'start_a') return { ...current, startGate: { ...(current.startGate ?? { a: clickRaw, b: clickRaw }), a: clickRaw } };
      if (tool === 'start_b') return { ...current, startGate: { ...(current.startGate ?? { a: clickRaw, b: clickRaw }), b: clickRaw } };
      if (tool === 'finish_a') return { ...current, finishGate: { ...(current.finishGate ?? { a: clickRaw, b: clickRaw }), a: clickRaw } };
      if (tool === 'finish_b') return { ...current, finishGate: { ...(current.finishGate ?? { a: clickRaw, b: clickRaw }), b: clickRaw } };
      return {
        ...current,
        calibration: {
          ...current.calibration,
          startPosition: clickRaw,
        },
      };
    });
  };

  const saveTrack = async () => {
    setSaveState('saving');
    const payload = {
      name: draft.name,
      imageUrl: draft.imageUrl || null,
      layoutPoints: draft.layoutPoints,
      maskPolygon: draft.maskPolygon,
      startGate: draft.startGate ?? null,
      finishGate: draft.finishGate ?? null,
      checkpoints: draft.checkpoints,
      calibration: draft.calibration,
    };

    const targetId = draft.id || selectedTrackId;
    const url = targetId ? `${apiBase}/api/tracks/${targetId}` : `${apiBase}/api/tracks`;
    const method = targetId ? 'PUT' : 'POST';

    try {
      const response = await fetch(url, {
        method,
        headers: { 'content-type': 'application/json' },
        body: JSON.stringify(payload),
      });
      if (!response.ok) throw new Error('save failed');
      const saved = await response.json() as Track;
      setSaveState('saved');
      setSelectedTrackId(saved.id);
      setDraft(asTrackDraft(saved));
      onSaved(saved);
    } catch {
      setSaveState('error');
    }
  };

  const layout = draft.layoutPoints.map((point) => {
    const scaled = scalePoint(point);
    return `${scaled.x},${scaled.y}`;
  }).join(' ');

  return (
    <section className={styles.panel}>
      <div className={styles.panelHeader}>
        <div>
          <h2>Settings · Track setup wizard</h2>
          <p>Load the camera image, click to trace the spline path, set start/finish gates, and set start direction.</p>
        </div>
      </div>

      <div className={styles.settingsGrid}>
        <div className={styles.formColumn}>
          <label className={styles.selectorLabel}>Track
            <select value={selectedTrackId} onChange={(event) => setSelectedTrackId(event.target.value)}>
              <option value="">New track</option>
              {tracks.map((track) => (
                <option key={track.id} value={track.id}>{track.name}</option>
              ))}
            </select>
          </label>
          <label className={styles.selectorLabel}>Name
            <input value={draft.name} onChange={(event) => setDraft((current) => ({ ...current, name: event.target.value }))} />
          </label>
          <label className={styles.selectorLabel}>Camera image URL
            <input value={draft.imageUrl} onChange={(event) => setDraft((current) => ({ ...current, imageUrl: event.target.value }))} placeholder="https://.../track-camera.jpg" />
          </label>
          <label className={styles.selectorLabel}>Direction
            <select value={draft.calibration.direction || 'clockwise'} onChange={(event) => setDraft((current) => ({ ...current, calibration: { ...current.calibration, direction: event.target.value } }))}>
              <option value="clockwise">Clockwise</option>
              <option value="counter-clockwise">Counter-clockwise</option>
            </select>
          </label>

          <div className={styles.toolRow}>
            {(['layout', 'mask', 'checkpoint', 'start_a', 'start_b', 'finish_a', 'finish_b', 'start_position'] as PlacementTool[]).map((item) => (
              <button key={item} type="button" className={tool === item ? styles.activeTool : styles.toolButton} onClick={() => setTool(item)}>{item.replace('_', ' ')}</button>
            ))}
          </div>

          <div className={styles.toolRow}>
            <button type="button" className={styles.toolButton} onClick={() => setDraft((current) => ({ ...current, layoutPoints: [] }))}>Clear path</button>
            <button type="button" className={styles.toolButton} onClick={() => setDraft((current) => ({ ...current, checkpoints: [] }))}>Clear checkpoints</button>
            <button type="button" className={styles.toolButton} onClick={() => setDraft((current) => ({ ...current, maskPolygon: [] }))}>Clear mask</button>
          </div>

          <button type="button" className={styles.saveButton} onClick={() => void saveTrack()}>
            {saveState === 'saving' ? 'Saving…' : 'Save track setup'}
          </button>
        </div>

        <div className={styles.setupViewport}>
          <svg viewBox="0 0 100 100" preserveAspectRatio="none" onClick={onCanvasClick} className={styles.trackSvg}>
            {draft.imageUrl ? <image href={draft.imageUrl} x="0" y="0" width="100" height="100" preserveAspectRatio="none" className={styles.trackImage} /> : null}
            {layout ? <polyline points={layout} className={styles.layoutPolyline} /> : null}
            {draft.maskPolygon.length >= 3 ? (
              <polygon points={draft.maskPolygon.map((point) => {
                const scaled = scalePoint(point);
                return `${scaled.x},${scaled.y}`;
              }).join(' ')} className={styles.maskPolygon} />
            ) : null}
            {draft.startGate ? <line x1={scalePoint(draft.startGate.a).x} y1={scalePoint(draft.startGate.a).y} x2={scalePoint(draft.startGate.b).x} y2={scalePoint(draft.startGate.b).y} className={styles.startGate} /> : null}
            {draft.finishGate ? <line x1={scalePoint(draft.finishGate.a).x} y1={scalePoint(draft.finishGate.a).y} x2={scalePoint(draft.finishGate.b).x} y2={scalePoint(draft.finishGate.b).y} className={styles.finishGate} /> : null}
            {draft.checkpoints.map((checkpoint) => {
              const point = scalePoint(checkpoint.position);
              return <circle key={checkpoint.id} cx={point.x} cy={point.y} r="2.2" className={styles.checkpointDot} />;
            })}
            {draft.calibration.startPosition ? (
              <circle cx={scalePoint(draft.calibration.startPosition).x} cy={scalePoint(draft.calibration.startPosition).y} r="3" className={styles.startPositionDot} />
            ) : null}
          </svg>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const [tab, setTab] = useState<'dashboard' | 'settings'>('dashboard');
  const [races, setRaces] = useState<Race[]>([]);
  const [tracks, setTracks] = useState<Track[]>([]);
  const [selectedRaceId, setSelectedRaceId] = useState<string>('');
  const [selectedTrackId, setSelectedTrackId] = useState<string>('');
  const [dashboard, setDashboard] = useState<Dashboard | null>(null);
  const [connectionState, setConnectionState] = useState<'connecting' | 'connected' | 'disconnected'>('connecting');

  useEffect(() => {
    let cancelled = false;
    async function loadRaces() {
      const [raceResponse, trackResponse] = await Promise.all([
        fetch(`${apiBase}/api/races`),
        fetch(`${apiBase}/api/tracks`),
      ]);
      if (!raceResponse.ok || !trackResponse.ok || cancelled) return;
      const racePayload = await raceResponse.json() as Race[];
      const trackPayload = await trackResponse.json() as Track[];
      setRaces(racePayload);
      setTracks(trackPayload);
      if (!selectedRaceId && racePayload[0]?.id) setSelectedRaceId(racePayload[0].id);
      if (!selectedTrackId && trackPayload[0]?.id) setSelectedTrackId(trackPayload[0].id);
    }

    void loadRaces();
    return () => {
      cancelled = true;
    };
  }, [selectedRaceId, selectedTrackId]);

  useEffect(() => {
    if (!selectedRaceId) return;
    let cancelled = false;
    async function loadDashboard() {
      const response = await fetch(`${apiBase}/api/races/${selectedRaceId}/dashboard`);
      if (!response.ok || cancelled) return;
      const payload = await response.json() as Dashboard;
      setDashboard(payload);
      if (payload.track?.id) setSelectedTrackId(payload.track.id);
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
      if (socket.readyState === WebSocket.OPEN) socket.send('ping');
    }, 15000);

    return () => {
      window.clearInterval(keepAlive);
      socket.close();
    };
  }, [selectedRaceId]);

  const refreshTracks = async (savedTrack?: Track) => {
    if (savedTrack) {
      setTracks((current) => {
        const without = current.filter((item) => item.id !== savedTrack.id);
        return [savedTrack, ...without];
      });
      return;
    }
    const response = await fetch(`${apiBase}/api/tracks`);
    if (!response.ok) return;
    const payload = await response.json() as Track[];
    setTracks(payload);
  };

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
          <div className={styles.tabRow}>
            <button type="button" className={tab === 'dashboard' ? styles.activeTool : styles.toolButton} onClick={() => setTab('dashboard')}>Dashboard</button>
            <button type="button" className={tab === 'settings' ? styles.activeTool : styles.toolButton} onClick={() => setTab('settings')}>Settings</button>
          </div>
        </section>

        {tab === 'settings' ? (
          <TrackSettings tracks={tracks} selectedTrackId={selectedTrackId} setSelectedTrackId={setSelectedTrackId} onSaved={(track) => void refreshTracks(track)} />
        ) : !dashboard ? (
          <section className={styles.emptyState}>
            <h2>No dashboard loaded</h2>
            <p>Create race records through the new CRUD API (`/api/tracks`, `/api/cars`, `/api/racers`, `/api/races`) and select a race to visualize it here.</p>
          </section>
        ) : (
          <div className={styles.grid}>
            <TrackPreview track={dashboard.track} entries={dashboard.entries} events={dashboard.recentEvents} />

            <section className={styles.panel}>
              <div className={styles.panelHeader}>
                <div>
                  <h2>{dashboard.race.name}</h2>
                  <p>{dashboard.race.status} · {dashboard.race.totalLaps} total laps (lap count pauses while race is paused)</p>
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
