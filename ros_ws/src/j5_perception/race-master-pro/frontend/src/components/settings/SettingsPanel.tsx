import { FormEvent, useEffect, useMemo, useState } from 'react'
import { TRACK_OVERLAY_EVENT, apiFetch, backendBaseUrl, backendWsUrl, configuredApiBaseUrl, getLatestSnapshotUrl, getSelectedTrackId, getTrackCheckpoints, getTrackDirection, getTrackPhotoUrl, parseTrackRecord, setSelectedTrackId, setTrackCheckpoints, setTrackDirection, setTrackPhotoUrl } from '@/lib/utils'
import { useRaceContext } from '@/stores/raceStore'
import TrackCanvas from '@/components/track/TrackCanvas'
import type { Checkpoint, Track, TrackPoint } from '@/types'

interface SetupCheckResult {
    command: string
    ok: boolean
    stdout?: string
    stderr?: string
}

interface SetupStep {
    id: string
    title: string
    description: string
    connected: boolean
    checks: SetupCheckResult[]
    next_command: string
    stop_command: string
    next_step_command?: string
    is_current?: boolean
    last_error?: string | null
    help?: string
}

interface WizardStatus {
    config: Record<string, unknown>
    steps: SetupStep[]
    race_context: Record<string, unknown>
}

const defaultConfig = {
    pi_host: 'pi-host-or-ip.local',
    pi_user: 'pi-user',
    backend_url: 'http://<pi-ip>:8080',
    preview_url: 'http://pi-ip:8091',
    race_name: 'Main Event',
    event_name: 'Weekend Session',
    total_laps: 20,
    racer_names: 'Racer 1, Racer 2',
}

function normalizeBaseUrl(value: string): string {
    const raw = value.trim()
    if (!raw) return ''
    try {
        const parsed = new URL(raw)
        return `${parsed.protocol}//${parsed.host}`
    } catch {
        return raw.replace(/\/$/, '')
    }
}

function pointDistance(a: TrackPoint, b: TrackPoint): number {
    return Math.hypot(a.x - b.x, a.y - b.y)
}

function findNearestSplineIndex(points: TrackPoint[], target: TrackPoint): number {
    if (points.length === 0) return -1
    let bestIndex = 0
    let bestDistance = Number.POSITIVE_INFINITY
    points.forEach((point, index) => {
        const distance = pointDistance(point, target)
        if (distance < bestDistance) {
            bestDistance = distance
            bestIndex = index
        }
    })
    return bestIndex
}

export default function SettingsPanel() {
    const { refreshRaceState } = useRaceContext()
    const [wizard, setWizard] = useState<WizardStatus | null>(null)
    const [busyStepId, setBusyStepId] = useState<string | null>(null)
    const [error, setError] = useState('')
    const [config, setConfig] = useState(defaultConfig)
    const [tracks, setTracks] = useState<Track[]>([])
    const [selectedTrackId, setSelectedTrackIdState] = useState('')
    const [trackName, setTrackName] = useState('Main Track')
    const [trackPhotoUrl, setTrackPhotoUrlState] = useState('')
    const [editorPoints, setEditorPoints] = useState<TrackPoint[]>([])
    const [checkpoints, setCheckpoints] = useState<Checkpoint[]>([])
    const [editTool, setEditTool] = useState<'spline' | 'start' | 'finish' | 'checkpoint'>('spline')
    const [trackDirection, setTrackDirectionState] = useState<'clockwise' | 'counterclockwise'>('clockwise')
    // Compatibility guard: keep legacy symbol available so stale/hot-reloaded closures that
    // still reference `lapDirection` do not crash the whole panel.
    const lapDirection = trackDirection || 'clockwise'
    const [trackPhotoLoadState, setTrackPhotoLoadState] = useState<'idle' | 'loaded' | 'error'>('idle')
    const [markerNotice, setMarkerNotice] = useState('')

    const selectedTrack = useMemo(
        () => tracks.find(track => track.id === selectedTrackId) || null,
        [selectedTrackId, tracks],
    )

    const applyTrackSelection = (track: Track | null) => {
        if (!track) {
            setSelectedTrackIdState('')
            setTrackName('Main Track')
            setTrackPhotoUrlState(getLatestSnapshotUrl())
            setTrackPhotoLoadState('idle')
            setEditorPoints([])
            setCheckpoints([])
            setTrackDirectionState('clockwise')
            return
        }
        setSelectedTrackIdState(track.id)
        setSelectedTrackId(track.id)
        setTrackName(track.name)
        setEditorPoints(track.layout_points)
        setTrackPhotoUrlState(getTrackPhotoUrl(track.id) || getLatestSnapshotUrl())
        setTrackPhotoLoadState('idle')
        setCheckpoints(getTrackCheckpoints(track.id))
        setTrackDirectionState(getTrackDirection(track.id))
    }

    const loadTracks = async () => {
        try {
            const response = await apiFetch('/api/tracks')
            if (!response.ok) throw new Error('Failed to load tracks')
            const payload = await response.json()
            const parsed = Array.isArray(payload)
                ? payload.map(item => parseTrackRecord(item as Record<string, unknown>))
                : []
            setTracks(parsed)
            const persistedTrackId = getSelectedTrackId()
            const nextTrack = parsed.find(track => track.id === persistedTrackId)
                || parsed.find(track => track.id === selectedTrackId)
                || parsed[0]
                || null
            applyTrackSelection(nextTrack)
        } catch (err) {
            setError(err instanceof Error ? err.message : 'Unable to load track list.')
        }
    }

    const loadWizard = async () => {
        try {
            const response = await apiFetch('/api/setup/wizard')
            if (!response.ok) throw new Error('Failed to load wizard status')
            const payload: WizardStatus = await response.json()
            const rawNames = (payload.config.racer_names as string[] | undefined) || []
            setWizard(payload)
            setConfig({
                pi_host: String(payload.config.pi_host ?? defaultConfig.pi_host),
                pi_user: String(payload.config.pi_user ?? defaultConfig.pi_user),
                backend_url: String(payload.config.backend_url ?? defaultConfig.backend_url),
                preview_url: String(payload.config.preview_url ?? defaultConfig.preview_url),
                race_name: String(payload.config.race_name ?? defaultConfig.race_name),
                event_name: String(payload.config.event_name ?? defaultConfig.event_name),
                total_laps: Number(payload.config.total_laps ?? defaultConfig.total_laps),
                racer_names: rawNames.join(', '),
            })
            setError('')
            await refreshRaceState(String(payload.race_context?.race_id || ''))
        } catch (err) {
            setError(err instanceof Error ? err.message : 'Unable to load setup wizard.')
        }
    }

    useEffect(() => {
        void loadWizard()
        void loadTracks()
    }, [])

    useEffect(() => {
        const handleOverlayUpdate = () => {
            const selectedId = getSelectedTrackId() || selectedTrackId
            if (!selectedId) {
                setTrackPhotoUrlState(getLatestSnapshotUrl())
                return
            }
            setTrackPhotoUrlState(getTrackPhotoUrl(selectedId) || getLatestSnapshotUrl())
            setCheckpoints(getTrackCheckpoints(selectedId))
            setTrackDirectionState(getTrackDirection(selectedId))
        }
        window.addEventListener(TRACK_OVERLAY_EVENT, handleOverlayUpdate)
        return () => window.removeEventListener(TRACK_OVERLAY_EVENT, handleOverlayUpdate)
    }, [selectedTrackId])

    const frontendApiUrl = configuredApiBaseUrl() ?? `same-origin /api → fallback ${backendBaseUrl()}`
    const frontendWsUrl = backendWsUrl('organizer')

    const currentStep = useMemo(() => wizard?.steps.find(step => step.is_current), [wizard])

    const runStepAction = async (stepId: string, action: 'verify' | 'connect' | 'stop') => {
        setBusyStepId(stepId)
        try {
            const response = await apiFetch(`/api/setup/wizard/steps/${stepId}/${action}`, { method: 'POST' })
            if (!response.ok) throw new Error(`Failed to ${action} step ${stepId}`)
            const payload = await response.json()
            setWizard(payload.wizard || payload)
            setError('')
            await refreshRaceState(String(payload?.wizard?.race_context?.race_id || payload?.race_context?.race_id || ''))
        } catch (err) {
            setError(err instanceof Error ? err.message : `Unable to ${action} step.`)
        } finally {
            setBusyStepId(null)
        }
    }

    const onSaveConfig = async (event: FormEvent) => {
        event.preventDefault()
        try {
            const response = await apiFetch('/api/setup/wizard/config', {
                method: 'PUT',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    ...config,
                    racer_names: config.racer_names
                        .split(',')
                        .map(name => name.trim())
                        .filter(Boolean),
                }),
            })
            if (!response.ok) throw new Error('Failed to save wizard config')
            const payload = await response.json()
            setWizard(payload)
            setError('')
            await refreshRaceState(String(payload?.race_context?.race_id || ''))
        } catch (err) {
            setError(err instanceof Error ? err.message : 'Unable to save setup configuration.')
        }
    }

    const controlRace = async (action: 'start' | 'pause' | 'snapshot' | 'resume' | 'finish') => {
        const raceId = String(wizard?.race_context?.race_id || '')
        if (!raceId) {
            setError('Initialize race state first before using race controls.')
            return
        }
        const endpointMap: Record<'start' | 'pause' | 'snapshot' | 'resume' | 'finish', string> = {
            start: `/api/races/${raceId}/start`,
            pause: `/api/races/${raceId}/pause`,
            snapshot: `/api/races/${raceId}/snapshot`,
            resume: `/api/races/${raceId}/resume`,
            finish: `/api/races/${raceId}/finish`,
        }
        try {
            const response = await apiFetch(endpointMap[action], { method: 'POST' })
            if (!response.ok) throw new Error(`Failed to ${action} race`)
            await loadWizard()
            await refreshRaceState(raceId)
        } catch (err) {
            setError(err instanceof Error ? err.message : `Unable to ${action} race.`)
        }
    }

    const initializeRace = async () => {
        try {
            const response = await apiFetch('/api/setup/wizard/initialize', { method: 'POST' })
            const payload = await response.json()
            if (!response.ok) {
                const detail = payload?.detail ? String(payload.detail) : 'Failed to initialize race state'
                throw new Error(detail)
            }
            setWizard(payload.wizard)
            setError('')
            await refreshRaceState(String(payload?.wizard?.race_context?.race_id || ''))
        } catch (err) {
            setError(err instanceof Error ? err.message : 'Unable to initialize race.')
        }
    }

    const createTrack = async () => {
        try {
            const response = await apiFetch('/api/tracks', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    name: trackName,
                    scale: '1:24',
                    track_distance: null,
                    layout_points: JSON.stringify(editorPoints),
                    boundary_polygon: JSON.stringify(selectedTrack?.boundary_polygon || []),
                }),
            })
            if (!response.ok) throw new Error('Failed to create track')
            const created = parseTrackRecord(await response.json() as Record<string, unknown>)
            if (trackPhotoUrl) {
                setTrackPhotoUrl(created.id, trackPhotoUrl)
            }
            setTrackCheckpoints(created.id, checkpoints.map((checkpoint, index) => ({
                ...checkpoint,
                track_id: created.id,
                sort_order: index,
            })))
            setTrackDirection(created.id, lapDirection)
            setSelectedTrackId(created.id)
            await loadTracks()
        } catch (err) {
            setError(err instanceof Error ? err.message : 'Unable to create track.')
        }
    }

    const saveTrackSpline = async () => {
        if (!selectedTrackId) {
            await createTrack()
            return
        }
        try {
            const response = await apiFetch(`/api/tracks/${selectedTrackId}`, {
                method: 'PUT',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    name: trackName,
                    scale: selectedTrack?.scale || '1:24',
                    track_distance: selectedTrack?.track_distance || null,
                    layout_points: JSON.stringify(editorPoints),
                    boundary_polygon: JSON.stringify(selectedTrack?.boundary_polygon || []),
                }),
            })
            if (!response.ok) throw new Error('Failed to save track spline')
            if (trackPhotoUrl) {
                setTrackPhotoUrl(selectedTrackId, trackPhotoUrl)
            }
            const start = checkpoints.find(item => item.type === 'start')
            const finish = checkpoints.find(item => item.type === 'finish')
            if (start && finish && pointDistance(start.position, finish.position) > 0.0001) {
                setError('Start and finish must be the same spline point. Re-place one marker so both overlap.')
                return
            }
            const defaultPoint = editorPoints[0]
            const baseStart = start?.position || finish?.position || defaultPoint
            const checkpointPoints = checkpoints.filter(item => item.type === 'checkpoint')
            const normalized = checkpoints
                .filter(item => item.type === 'checkpoint')
                .concat(
                    !start && baseStart ? [{
                        id: crypto.randomUUID(),
                        track_id: selectedTrackId,
                        name: 'START',
                        type: 'start' as const,
                        sort_order: 0,
                        position: baseStart,
                    }] : [],
                    !finish && baseStart ? [{
                        id: crypto.randomUUID(),
                        track_id: selectedTrackId,
                        name: 'FINISH',
                        type: 'finish' as const,
                        sort_order: 0,
                        position: baseStart,
                    }] : [],
                    checkpointPoints.length === 0 && baseStart ? [{
                        id: crypto.randomUUID(),
                        track_id: selectedTrackId,
                        name: 'CP 1',
                        type: 'checkpoint' as const,
                        sort_order: 0,
                        position: baseStart,
                    }] : [],
                )
                .sort((a, b) => {
                    const aIdx = findNearestSplineIndex(editorPoints, a.position)
                    const bIdx = findNearestSplineIndex(editorPoints, b.position)
                    if (lapDirection === 'clockwise') return aIdx - bIdx
                    return bIdx - aIdx
                })
                .map((checkpoint, index) => ({
                    ...checkpoint,
                    track_id: selectedTrackId,
                    sort_order: index,
                }))
            setTrackCheckpoints(selectedTrackId, normalized)
            setTrackDirection(selectedTrackId, lapDirection)
            await loadTracks()
            setError('')
        } catch (err) {
            setError(err instanceof Error ? err.message : 'Unable to save track spline.')
        }
    }

    const useLatestCapture = () => {
        const latest = getLatestSnapshotUrl()
        const previewBase = normalizeBaseUrl(config.preview_url)
        const fallback = previewBase ? `${previewBase}/snapshot.jpg?t=${Date.now()}` : ''
        setTrackPhotoUrlState(latest || fallback)
        setTrackPhotoLoadState('idle')
    }

    const undoSplinePoint = () => {
        setEditorPoints(prev => prev.slice(0, -1))
    }

    const onMarkerPlaced = (checkpoint: Checkpoint) => {
        const nearestIndex = findNearestSplineIndex(editorPoints, checkpoint.position)
        const splineText = nearestIndex >= 0 ? ` snapped to spline point #${nearestIndex + 1}` : ''
        setMarkerNotice(`${checkpoint.type.toUpperCase()} marker placed${splineText}.`)
    }

    return (
        <div className="fade-in space-y-4">
            <h2 className="text-xl font-semibold text-[var(--color-text-primary)]">Settings · Race Setup Wizard</h2>

            <div className="race-card p-4 space-y-3">
                <div>
                    <h3 className="mb-2 text-base font-semibold">Configuration</h3>
                    <p className="text-xs text-[var(--color-text-secondary)]">
                        Save the FastAPI base URL first, then verify Pi reachability plus backend health before clicking <strong>Initialize Race State</strong>. Preview and websocket checks can be completed afterward before live tracking.
                    </p>
                </div>

                <div className="rounded border border-slate-700 bg-slate-950/60 p-3 text-xs text-slate-200">
                    <p><strong>Frontend API target:</strong> <code>{frontendApiUrl}</code></p>
                    <p><strong>Frontend organizer websocket:</strong> <code>{frontendWsUrl}</code></p>
                    <p className="mt-2 text-[var(--color-text-secondary)]">
                        This Vite app uses <code>VITE_API_BASE_URL</code> and <code>VITE_WS_URL</code> when set. Without them, API calls try same-origin <code>/api</code> first and fall back to <code>http://&lt;current-host&gt;:8080</code>; websocket calls stay on same-origin <code>/ws</code> for HTTPS/proxied deployments and fall back to <code>ws://&lt;current-host&gt;:8080/ws</code> for local Vite dev.
                    </p>
                </div>
                <form className="grid gap-3 md:grid-cols-2" onSubmit={onSaveConfig}>
                    <input className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm" value={config.pi_host} onChange={e => setConfig(prev => ({ ...prev, pi_host: e.target.value }))} placeholder="Pi host" />
                    <input className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm" value={config.pi_user} onChange={e => setConfig(prev => ({ ...prev, pi_user: e.target.value }))} placeholder="Pi user" />
                    <input className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm" value={config.backend_url} onChange={e => setConfig(prev => ({ ...prev, backend_url: e.target.value }))} placeholder="Backend URL" />
                    <input className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm" value={config.preview_url} onChange={e => setConfig(prev => ({ ...prev, preview_url: e.target.value }))} placeholder="Preview URL" />
                    <input className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm" value={config.event_name} onChange={e => setConfig(prev => ({ ...prev, event_name: e.target.value }))} placeholder="Event name" />
                    <input className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm" value={config.race_name} onChange={e => setConfig(prev => ({ ...prev, race_name: e.target.value }))} placeholder="Race name" />
                    <input className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm" type="number" min={1} value={config.total_laps} onChange={e => setConfig(prev => ({ ...prev, total_laps: Number(e.target.value) }))} placeholder="Total laps" />
                    <input className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm md:col-span-2" value={config.racer_names} onChange={e => setConfig(prev => ({ ...prev, racer_names: e.target.value }))} placeholder="Racer names (comma separated)" />
                    <div className="md:col-span-2 flex gap-2 flex-wrap">
                        <button className="rounded bg-blue-600 px-4 py-2 text-sm font-semibold text-white hover:bg-blue-500" type="submit">Save Config</button>
                        <button className="rounded bg-emerald-600 px-4 py-2 text-sm font-semibold text-white hover:bg-emerald-500" type="button" onClick={initializeRace}>Initialize Race State</button>
                        <button className="rounded bg-slate-700 px-4 py-2 text-sm font-semibold text-white hover:bg-slate-600" type="button" onClick={loadWizard}>Refresh</button>
                        <button className="rounded bg-indigo-600 px-4 py-2 text-sm font-semibold text-white hover:bg-indigo-500" type="button" onClick={() => controlRace('start')}>Race Start</button>
                        <button className="rounded bg-amber-600 px-4 py-2 text-sm font-semibold text-white hover:bg-amber-500" type="button" onClick={() => controlRace('pause')}>Race Pause</button>
                        <button className="rounded bg-violet-600 px-4 py-2 text-sm font-semibold text-white hover:bg-violet-500" type="button" onClick={() => controlRace('snapshot')}>Save Snapshot</button>
                        <button className="rounded bg-teal-600 px-4 py-2 text-sm font-semibold text-white hover:bg-teal-500" type="button" onClick={() => controlRace('resume')}>Race Resume</button>
                        <button className="rounded bg-rose-700 px-4 py-2 text-sm font-semibold text-white hover:bg-rose-600" type="button" onClick={() => controlRace('finish')}>Race Finish</button>
                    </div>
                </form>
            </div>

            {error ? <p className="text-sm text-red-400">{error}</p> : null}

            <div className="race-card p-4 space-y-3">
                <h3 className="text-base font-semibold">Ordered Connection Checklist</h3>
                {currentStep ? (
                    <div className="space-y-1 text-xs text-[var(--color-text-secondary)]">
                        <p>Current blocking step: <strong>{currentStep.title}</strong> · Next command: <code>{currentStep.next_command}</code></p>
                        <p>Recommended order: 1) save config, 2) verify Pi + backend, 3) initialize race state, 4) bring up preview/websocket before live tracking, 5) capture track photo, 6) draw spline, 7) use Race Start / Pause / Resume / Finish controls.</p>
                    </div>
                ) : <p className="text-xs text-emerald-400">All setup steps are currently marked connected.</p>}

                <div className="space-y-3">
                    {wizard?.steps.map((step, idx) => (
                        <div key={step.id} className={`rounded border p-3 ${step.connected ? 'border-emerald-700 bg-emerald-950/30' : 'border-amber-700 bg-amber-950/30'}`}>
                            <div className="flex flex-wrap items-center justify-between gap-2">
                                <div>
                                    <p className="font-semibold">{idx + 1}. {step.title}</p>
                                    <p className="text-xs text-[var(--color-text-secondary)]">{step.description}</p>
                                </div>
                                <span className={`text-xs font-semibold ${step.connected ? 'text-emerald-400' : 'text-amber-300'}`}>{step.connected ? 'Connected' : 'Not connected'}</span>
                            </div>

                            <div className="mt-2 text-xs space-y-1">
                                <p>Command to continue: <code>{step.next_command}</code></p>
                                {step.next_step_command ? <p>Next step command: <code>{step.next_step_command}</code></p> : null}
                            </div>

                            <div className="mt-3 flex flex-wrap gap-2">
                                <button className="rounded bg-blue-600 px-3 py-1 text-xs font-semibold text-white" onClick={() => runStepAction(step.id, 'verify')} disabled={busyStepId === step.id}>Verify</button>
                                <button className="rounded bg-emerald-600 px-3 py-1 text-xs font-semibold text-white" onClick={() => runStepAction(step.id, 'connect')} disabled={busyStepId === step.id}>Connect / Reconnect</button>
                                <button className="rounded bg-rose-700 px-3 py-1 text-xs font-semibold text-white" onClick={() => runStepAction(step.id, 'stop')} disabled={busyStepId === step.id}>Stop</button>
                            </div>

                            {step.checks.length > 0 ? (
                                <ul className="mt-3 space-y-1 text-xs">
                                    {step.checks.map(check => (
                                        <li key={check.command} className={check.ok ? 'text-emerald-300' : 'text-rose-300'}>
                                            <p>{check.ok ? '✓' : '✗'} {check.command}</p>
                                            {!check.ok && check.stderr ? (
                                                <p className="ml-4 text-[11px] text-rose-200 break-words">{check.stderr}</p>
                                            ) : null}
                                            {check.ok && check.stdout ? (
                                                <p className="ml-4 text-[11px] text-emerald-100 break-words">{check.stdout}</p>
                                            ) : null}
                                        </li>
                                    ))}
                                </ul>
                            ) : null}

                            {step.help ? <p className="mt-2 text-xs text-slate-300">How to unblock: {step.help}</p> : null}
                            {step.last_error ? <p className="mt-2 text-xs text-rose-300">Last error: {step.last_error}</p> : null}
                        </div>
                    ))}
                </div>
            </div>

            <div className="race-card p-4 space-y-3">
                <div className="flex flex-wrap items-center justify-between gap-3">
                    <div>
                        <h3 className="text-base font-semibold">Track spline mapper</h3>
                        <p className="text-xs text-[var(--color-text-secondary)]">
                            Capture a track photo in the Computer Vision tab, then draw a centerline spline over that image. The Dashboard tab will reuse the same photo + spline so racer dots follow the mapped track.
                        </p>
                    </div>
                    <button className="rounded bg-slate-700 px-3 py-2 text-xs font-semibold text-white hover:bg-slate-600" type="button" onClick={loadTracks}>Reload tracks</button>
                </div>

                <div className="grid gap-3 lg:grid-cols-[320px_1fr]">
                    <div className="space-y-3">
                        <label className="block text-sm text-[var(--color-text-secondary)]">
                            Track
                            <select
                                className="mt-1 w-full rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm text-white"
                                value={selectedTrackId}
                                onChange={event => applyTrackSelection(tracks.find(track => track.id === event.target.value) || null)}
                            >
                                <option value="">Create/select a track</option>
                                {tracks.map(track => (
                                    <option key={track.id} value={track.id}>{track.name}</option>
                                ))}
                            </select>
                        </label>

                        <label className="block text-sm text-[var(--color-text-secondary)]">
                            Track name
                            <input
                                className="mt-1 w-full rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm text-white"
                                value={trackName}
                                onChange={event => setTrackName(event.target.value)}
                                placeholder="Main Track"
                            />
                        </label>

                        <label className="block text-sm text-[var(--color-text-secondary)]">
                            Track photo URL
                            <input
                                className="mt-1 w-full rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm text-white"
                                value={trackPhotoUrl}
                                onChange={event => {
                                    setTrackPhotoUrlState(event.target.value)
                                    setTrackPhotoLoadState('idle')
                                }}
                                placeholder="http://pi-ip:8091/snapshot.jpg"
                            />
                        </label>

                        <div className="flex flex-wrap gap-2">
                            <button className="rounded bg-blue-600 px-3 py-2 text-xs font-semibold text-white hover:bg-blue-500" type="button" onClick={useLatestCapture}>Use Latest Capture</button>
                            <button className="rounded bg-indigo-600 px-3 py-2 text-xs font-semibold text-white hover:bg-indigo-500" type="button" onClick={saveTrackSpline}>Save Spline</button>
                            <button className="rounded bg-emerald-600 px-3 py-2 text-xs font-semibold text-white hover:bg-emerald-500" type="button" onClick={createTrack}>New Track</button>
                        </div>

                        <label className="block text-sm text-[var(--color-text-secondary)]">
                            Lap direction
                            <select
                                className="mt-1 w-full rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm text-white"
                                value={trackDirection}
                                onChange={event => setTrackDirectionState(event.target.value as 'clockwise' | 'counterclockwise')}
                            >
                                <option value="clockwise">Clockwise (default)</option>
                                <option value="counterclockwise">Counter-clockwise</option>
                            </select>
                        </label>

                        <div className="flex flex-wrap gap-2">
                            <button className={`rounded px-3 py-2 text-xs font-semibold text-white ${editTool === 'spline' ? 'bg-blue-500' : 'bg-slate-700 hover:bg-slate-600'}`} type="button" onClick={() => setEditTool('spline')}>Spline Tool</button>
                            <button className={`rounded px-3 py-2 text-xs font-semibold text-white ${editTool === 'start' ? 'bg-emerald-500' : 'bg-slate-700 hover:bg-slate-600'}`} type="button" onClick={() => setEditTool('start')}>Mark Start</button>
                            <button className={`rounded px-3 py-2 text-xs font-semibold text-white ${editTool === 'finish' ? 'bg-amber-500' : 'bg-slate-700 hover:bg-slate-600'}`} type="button" onClick={() => setEditTool('finish')}>Mark Finish</button>
                            <button className={`rounded px-3 py-2 text-xs font-semibold text-white ${editTool === 'checkpoint' ? 'bg-violet-500' : 'bg-slate-700 hover:bg-slate-600'}`} type="button" onClick={() => setEditTool('checkpoint')}>Add Checkpoint</button>
                        </div>

                        <div className="flex flex-wrap gap-2">
                            <button className="rounded bg-slate-700 px-3 py-2 text-xs font-semibold text-white hover:bg-slate-600" type="button" onClick={undoSplinePoint} disabled={editorPoints.length === 0}>Undo Point</button>
                            <button className="rounded bg-rose-700 px-3 py-2 text-xs font-semibold text-white hover:bg-rose-600" type="button" onClick={() => setEditorPoints([])} disabled={editorPoints.length === 0}>Clear Spline</button>
                            <button className="rounded bg-rose-700 px-3 py-2 text-xs font-semibold text-white hover:bg-rose-600" type="button" onClick={() => setCheckpoints([])} disabled={checkpoints.length === 0}>Clear Markers</button>
                        </div>

                        <div className="rounded border border-slate-700 bg-slate-950/60 p-3 text-xs text-slate-300 space-y-1">
                            <p><strong>How to use:</strong></p>
                            <p>1. Capture the track photo in the Computer Vision tab.</p>
                            <p>2. Click <strong>Use Latest Capture</strong>.</p>
                            <p>3. Choose <strong>Spline Tool</strong> and click around the lane centerline to place handles.</p>
                            <p>4. Switch tools to mark <strong>Start</strong>, <strong>Finish</strong>, and additional checkpoints.</p>
                            <p>5. Start and finish must be the same point. If unset, both default to spline point 1.</p>
                            <p>6. Save, then verify Dashboard dots and markers line up with the track image.</p>
                            {markerNotice ? <p className="text-emerald-300">{markerNotice}</p> : null}
                            {trackPhotoUrl ? (
                                <p className={trackPhotoLoadState === 'error' ? 'text-rose-300' : 'text-emerald-300'}>
                                    Image status: {trackPhotoLoadState === 'error' ? 'failed to load from URL' : trackPhotoLoadState === 'loaded' ? 'loaded' : 'waiting for load'}
                                </p>
                            ) : null}
                        </div>
                    </div>

                    <div className="rounded border border-slate-700 bg-slate-950/40 p-3">
                        {trackPhotoUrl ? (
                            <img
                                src={trackPhotoUrl}
                                alt="Track snapshot preview"
                                className="mb-3 max-h-52 w-full rounded border border-slate-700 object-contain bg-black/30"
                                onLoad={() => setTrackPhotoLoadState('loaded')}
                                onError={() => setTrackPhotoLoadState('error')}
                            />
                        ) : null}
                        <TrackCanvas
                            racers={[]}
                            layoutPoints={editorPoints}
                            checkpoints={checkpoints}
                            isEditMode
                            editTool={editTool}
                            onTrackUpdate={setEditorPoints}
                            onCheckpointUpdate={setCheckpoints}
                            onMarkerPlaced={onMarkerPlaced}
                            onBackgroundImageError={() => setTrackPhotoLoadState('error')}
                            backgroundImageUrl={trackPhotoUrl || undefined}
                        />
                    </div>
                </div>
            </div>
        </div>
    )
}
