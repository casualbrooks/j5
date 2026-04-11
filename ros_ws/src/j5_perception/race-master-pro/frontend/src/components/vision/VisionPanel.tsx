import { FormEvent, useEffect, useMemo, useState } from 'react'
import { apiFetch, getSelectedTrackId, getTrackRacerAssignments, parseTrackRecord, setLatestSnapshotUrl, setSelectedTrackId, setTrackPhotoUrl } from '@/lib/utils'
import { useRaceContext } from '@/stores/raceStore'

function defaultPreviewBaseUrl(): string {
    if (typeof window === 'undefined') {
        return 'http://localhost:8091'
    }
    return `http://${window.location.hostname}:8091`
}

function normalizePreviewBaseUrl(value: string): string {
    const raw = value.trim()
    if (!raw) return ''
    try {
        const parsed = new URL(raw)
        return `${parsed.protocol}//${parsed.host}`
    } catch {
        return raw.replace(/\/$/, '')
    }
}

export default function VisionPanel() {
    const raceStore = useRaceContext()
    const { liveRace, recentObjectDetections, recentVisionObjects } = raceStore
    const [previewBaseUrl, setPreviewBaseUrl] = useState(defaultPreviewBaseUrl)
    const [previewEnabled, setPreviewEnabled] = useState(false)
    const [statusMessage, setStatusMessage] = useState('')
    const [statusError, setStatusError] = useState(false)
    const [capturing, setCapturing] = useState(false)
    const [raceContext, setRaceContext] = useState<Record<string, unknown>>({})
    const [activeTrackName, setActiveTrackName] = useState('')
    const [statusNowMs, setStatusNowMs] = useState(() => Date.now())

    const normalizedBaseUrl = useMemo(() => normalizePreviewBaseUrl(previewBaseUrl), [previewBaseUrl])
    const streamUrl = `${normalizedBaseUrl}/stream.mjpg`

    const ensureSelectedTrack = async () => {
        const existingTrackId = getSelectedTrackId()
        if (existingTrackId) {
            return existingTrackId
        }
        try {
            const response = await apiFetch('/api/tracks')
            if (!response.ok) return ''
            const payload = await response.json()
            const tracks = Array.isArray(payload)
                ? payload.map(item => parseTrackRecord(item as Record<string, unknown>))
                : []
            const firstTrack = tracks[0]
            if (!firstTrack?.id) return ''
            setSelectedTrackId(firstTrack.id)
            return firstTrack.id
        } catch {
            return ''
        }
    }

    const refreshTrackLabel = async () => {
        const selectedTrackId = await ensureSelectedTrack()
        if (!selectedTrackId) {
            setActiveTrackName('')
            return
        }
        try {
            const response = await apiFetch('/api/tracks')
            if (!response.ok) return
            const payload = await response.json()
            const tracks = Array.isArray(payload)
                ? payload.map(item => parseTrackRecord(item as Record<string, unknown>))
                : []
            const selected = tracks.find(track => track.id === selectedTrackId)
            setActiveTrackName(selected?.name || '')
        } catch {
            // ignore if lookup fails
        }
    }

    const refreshWizardContext = async () => {
        try {
            const response = await apiFetch('/api/setup/wizard')
            if (!response.ok) return
            const payload = await response.json()
            setRaceContext(payload.race_context || {})
        } catch {
            // ignore for panel resilience
        }
    }

    useEffect(() => {
        void refreshWizardContext()
        void refreshTrackLabel()
    }, [])

    useEffect(() => {
        const timer = window.setInterval(() => {
            setStatusNowMs(Date.now())
        }, 1000)
        return () => {
            window.clearInterval(timer)
        }
    }, [])

    const onCapture = async (event: FormEvent) => {
        event.preventDefault()
        setCapturing(true)
        setStatusMessage('')
        setStatusError(false)

        try {
            const response = await fetch(`${normalizedBaseUrl}/capture`, {
                method: 'POST',
            })
            const payload = await response.json()

            if (!response.ok || !payload.ok) {
                throw new Error(payload.message || 'Capture failed')
            }

            setStatusMessage(payload.message || 'Snapshot captured successfully.')
            const snapshotUrl = `${normalizedBaseUrl}/snapshot.jpg?t=${Date.now()}`
            const selectedTrackId = await ensureSelectedTrack()
            setLatestSnapshotUrl(snapshotUrl)
            if (selectedTrackId) {
                setTrackPhotoUrl(selectedTrackId, snapshotUrl)
                setStatusMessage(payload.message || 'Snapshot captured and linked to selected track.')
                setStatusError(false)
            } else {
                setStatusMessage('Snapshot captured, but no track is selected yet. Select/create a track in Settings and use "Use Latest Capture".')
                setStatusError(true)
            }
            const raceId = String(raceContext.race_id || '')
            if (raceId) {
                await apiFetch(`/api/races/${raceId}/logs`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ entry: `${new Date().toISOString()} Track image captured from Vision panel` }),
                })
            }
            await refreshWizardContext()
        } catch (error) {
            const message = error instanceof Error ? error.message : 'Unable to capture snapshot.'
            setStatusMessage(message)
            setStatusError(true)
        } finally {
            setCapturing(false)
        }
    }

    const trackingEnabled = Boolean(raceContext.tracking_enabled)
    const logs = Array.isArray(raceContext.log_stream) ? raceContext.log_stream : []
    const racerAssignments = getSelectedTrackId() ? getTrackRacerAssignments(getSelectedTrackId()) : {}
    const freshVisionObjects = recentVisionObjects.filter((item) => (statusNowMs - item.seenAt) < 4000)
    const hasRecentVisionObjects = freshVisionObjects.length > 0
    const latestVisionSeenAt = recentVisionObjects[0]?.seenAt ?? 0
    const visionFresh = latestVisionSeenAt > 0 && (statusNowMs - latestVisionSeenAt) < 4000

    const toggleTracking = async (start: boolean) => {
        const raceId = String(raceContext.race_id || '')
        if (!raceId) {
            setStatusMessage('Initialize race in Settings before starting object tracking.')
            setStatusError(true)
            return
        }
        const endpoint = start ? `/api/races/${raceId}/tracking/start` : `/api/races/${raceId}/tracking/stop`
        const response = await apiFetch(endpoint, { method: 'POST' })
        if (response.ok) {
            const payload = await response.json() as { warning?: string }
            await refreshWizardContext()
            if (start && payload.warning) {
                setStatusMessage(payload.warning)
                setStatusError(true)
                return
            }
            setStatusMessage(start ? 'Object tracking started.' : 'Object tracking stopped.')
            setStatusError(false)
            return
        }
        setStatusMessage(start ? 'Unable to start object tracking.' : 'Unable to stop object tracking.')
        setStatusError(true)
    }

    return (
        <div className="fade-in space-y-4">
            <h2 className="text-xl font-semibold text-[var(--color-text-primary)]">Computer Vision</h2>

            <div className="race-card p-4 space-y-3">
                <div className="rounded border border-cyan-900/70 bg-cyan-950/30 p-3 text-xs text-cyan-100 space-y-2">
                    <p className="font-semibold text-cyan-200">Tracking bring-up quick start</p>
                    <p>Run these commands on the Pi / perception host before pressing <strong>Start Tracking</strong>:</p>
                    <p><strong>1) Camera preview server</strong></p>
                    <code className="block overflow-x-auto rounded bg-black/40 p-2 text-xs text-emerald-300">
                        python3 scripts/pi_preflight.py --camera-source /dev/video0 --capture-file track_snapshot.jpg --serve-preview --preview-host 0.0.0.0 --preview-port 8091 --preview-fps 15
                    </code>
                    <p><strong>2) Perception node (choose one)</strong></p>
                    <code className="block overflow-x-auto rounded bg-black/40 p-2 text-xs text-emerald-300">
                        python -m perception.standalone.standalone_runner --camera-source /dev/video0 --camera-id cam1
                    </code>
                    <code className="block overflow-x-auto rounded bg-black/40 p-2 text-xs text-emerald-300">
                        ros2 run racetracker_perception perception_node
                    </code>
                    <p>Then in this tab: <strong>Show Live Preview</strong> → <strong>Capture Track Photo</strong> → <strong>Start Tracking</strong>.</p>
                </div>
                <div className="rounded border border-indigo-900/70 bg-indigo-950/30 p-3 text-xs text-indigo-100 space-y-2">
                    <p className="font-semibold text-indigo-200">Computer Vision step-by-step (camera → tracking → lap checks)</p>
                    <ol className="list-decimal space-y-1 pl-4">
                        <li>Start camera preview on the Pi (command above), then click <strong>Show Live Preview</strong> in this tab.</li>
                        <li>Click <strong>Capture Track Photo</strong> so Settings can load the same image for spline + checkpoints.</li>
                        <li>Start perception (<code>standalone_runner</code> or <code>ros2 run racetracker_perception perception_node</code>), then click <strong>Start Tracking</strong>.</li>
                        <li>Confirm live object IDs appear in the preview overlay and in <strong>Recent object detections</strong>.</li>
                        <li>Open <strong>Settings → Racer tracking assignments</strong>, map each racer to an object ID, and save assignments.</li>
                        <li>Drive a test lap crossing the configured finish gate and confirm entries in <strong>Lap / Tracking Logs</strong> before race start.</li>
                    </ol>
                    <p className="text-[11px] text-indigo-200/90">After this passes, return to Settings to initialize/start the race lifecycle controls.</p>
                </div>
                <label className="block text-sm text-[var(--color-text-secondary)]">
                    Preview server URL
                    <input
                        className="mt-1 w-full rounded border border-slate-600 bg-slate-900 px-3 py-2 text-sm text-white"
                        value={previewBaseUrl}
                        onChange={(event) => setPreviewBaseUrl(event.target.value)}
                        placeholder="http://192.168.1.134:8091"
                    />
                </label>
                <p className="text-xs text-[var(--color-text-secondary)]">
                    Capture the track image here after the setup wizard in Settings marks connectivity as connected. Once
                    captured, return here to start object tracking and monitor lap-count logs.
                </p>
                <p className="text-xs text-[var(--color-text-secondary)]">
                    The preview stream is raw camera video. Use the detection table below to map object IDs to racers; dashboard overlay updates when mapped IDs are seen.
                </p>
                <p className="text-xs text-amber-300">
                    Keep the camera stream open in only one viewer at a time. Viewing <code>/stream.mjpg</code> in multiple places can drop the camera connection.
                </p>
                <div className="rounded border border-slate-700 bg-slate-950 p-3">
                    <p className="text-xs font-semibold uppercase tracking-wide text-slate-400">Start camera feed only</p>
                    <p className="mt-1 text-xs text-[var(--color-text-secondary)]">
                        You can bring the camera preview back without re-initializing the race by running this on the Pi:
                    </p>
                    <code className="mt-2 block overflow-x-auto rounded bg-black/40 p-2 text-xs text-emerald-300">
                        python3 scripts/pi_preflight.py --camera-source /dev/video0 --capture-file track_snapshot.jpg --serve-preview --preview-host 0.0.0.0 --preview-port 8091 --preview-fps 15
                    </code>
                    <p className="mt-2 text-xs text-[var(--color-text-secondary)]">
                        Then open the preview URL above, or load the stream again in this tab. Replace <code>/dev/video0</code> if your
                        camera uses a different device.
                    </p>
                </div>
                <div className="rounded border border-slate-700 bg-slate-950/60 p-3 text-xs text-slate-200 space-y-1">
                    <p><strong>Race flow:</strong> 1) Initialize race in Settings, 2) capture track photo here, 3) map spline in Settings, 4) return to Dashboard for overlays, 5) start tracking here, 6) use Race Start/Pause/Resume/Finish in Settings.</p>
                    <p>
                        Active track for auto-linking captures: <strong>{activeTrackName || 'None selected'}</strong>
                    </p>
                </div>
            </div>

            <div className="race-card p-4 space-y-3">
                <div className="flex flex-wrap items-center gap-2">
                    <button
                        type="button"
                        className="rounded bg-slate-700 px-3 py-2 text-xs font-semibold text-white hover:bg-slate-600"
                        onClick={() => setPreviewEnabled(prev => !prev)}
                    >
                        {previewEnabled ? 'Hide Live Preview' : 'Show Live Preview'}
                    </button>
                    <p className="text-xs text-[var(--color-text-secondary)]">Use this only while capturing the track image, then hide it before opening other camera consumers.</p>
                </div>
                <div className="rounded border border-slate-700 bg-slate-950/50 px-3 py-2 text-xs text-slate-200">
                    <p>
                        Websocket: <span className={(raceStore?.connectionStatus ?? 'disconnected') === 'connected' ? 'text-emerald-300' : 'text-amber-300'}>{raceStore?.connectionStatus ?? 'disconnected'}</span>
                        {' · '}
                        Detection stream: <span className={visionFresh ? 'text-emerald-300' : 'text-amber-300'}>{visionFresh ? 'receiving objects' : 'waiting for objects'}</span>
                    </p>
                    <p className="mt-1 text-[11px] text-slate-300">
                        If your ROS2 node is running, this can still show waiting when camera frames are not on the expected topic, no objects are visible yet, or confidence filtering drops detections.
                    </p>
                </div>
                {previewEnabled ? (
                    <div className="relative">
                        <img
                            src={streamUrl}
                            alt="Live camera preview"
                            className="w-full rounded border border-slate-700 bg-black"
                        />
                        <div className="pointer-events-none absolute left-2 top-2 max-w-[65%] space-y-1">
                            {freshVisionObjects.slice(0, 6).map((item) => (
                                <p key={`${item.objectId}-${item.seenAt}`} className="rounded bg-black/70 px-2 py-1 text-[11px] text-emerald-200">
                                    {item.objectId}
                                    {item.position ? ` (${item.position.x.toFixed(1)}, ${item.position.y.toFixed(1)})` : ''}
                                </p>
                            ))}
                            {!hasRecentVisionObjects ? (
                                <p className="rounded bg-black/70 px-2 py-1 text-[11px] text-amber-200">
                                    No objects received yet. If ROS2 perception is already running, verify active camera topic + visible cars in frame.
                                </p>
                            ) : null}
                        </div>
                    </div>
                ) : (
                    <div className="rounded border border-dashed border-slate-700 bg-black/30 p-4 text-xs text-[var(--color-text-secondary)]">
                        Live preview is paused to avoid multiple stream consumers.
                    </div>
                )}

                <form onSubmit={onCapture} className="flex flex-wrap items-center gap-2">
                    <button
                        type="submit"
                        disabled={capturing}
                        className="rounded bg-blue-600 px-4 py-2 text-sm font-semibold text-white hover:bg-blue-500 disabled:cursor-not-allowed disabled:opacity-70"
                    >
                        {capturing ? 'Capturing…' : 'Capture Track Photo'}
                    </button>
                    <button
                        type="button"
                        className="rounded bg-emerald-600 px-4 py-2 text-sm font-semibold text-white hover:bg-emerald-500"
                        onClick={() => toggleTracking(true)}
                    >
                        Start Tracking
                    </button>
                    <button
                        type="button"
                        className="rounded bg-rose-700 px-4 py-2 text-sm font-semibold text-white hover:bg-rose-600"
                        onClick={() => toggleTracking(false)}
                    >
                        Stop Tracking
                    </button>
                </form>

                <p className="text-xs text-[var(--color-text-secondary)]">
                    Tracking status: <span className={trackingEnabled ? 'text-emerald-400' : 'text-amber-300'}>{trackingEnabled ? 'Enabled' : 'Disabled'}</span>
                </p>

                {statusMessage ? (
                    <p className={`text-sm ${statusError ? 'text-red-400' : 'text-emerald-400'}`}>{statusMessage}</p>
                ) : null}
            </div>

            <div className="race-card p-4 space-y-2">
                <h3 className="text-sm font-semibold text-[var(--color-text-primary)]">Recent object detections</h3>
                {recentObjectDetections.length === 0 ? <p className="text-xs text-[var(--color-text-secondary)]">No detections yet. Start tracking and assign object IDs in Settings.</p> : null}
                {recentObjectDetections.length > 0 ? (
                    <div className="max-h-52 overflow-auto rounded border border-slate-700 bg-slate-950 p-2 text-xs text-slate-200 space-y-1">
                        {recentObjectDetections.map(item => {
                            const racer = (liveRace?.racers || []).find(entry => entry.racer_profile_id === item.racerProfileId)
                            const assignedId = racer ? racerAssignments[racer.racer_profile_id] : ''
                            const mapped = Boolean(racer && assignedId && assignedId === item.objectId)
                            return (
                                <p key={`${item.objectId}-${item.seenAt}`}>
                                    <span className={mapped ? 'text-emerald-300' : 'text-amber-300'}>{item.objectId}</span>
                                    {' → '}
                                    <span>{racer?.name || 'unmapped racer'}</span>
                                </p>
                            )
                        })}
                    </div>
                ) : null}
            </div>

            <div className="race-card p-4 space-y-2">
                <h3 className="text-sm font-semibold text-[var(--color-text-primary)]">Lap / Tracking Logs</h3>
                {logs.length === 0 ? <p className="text-xs text-[var(--color-text-secondary)]">No logs yet. Start tracking to populate this feed.</p> : null}
                <div className="max-h-56 overflow-auto rounded border border-slate-700 bg-slate-950 p-2 text-xs text-slate-200">
                    {logs.map((line, index) => (
                        <p key={`${line}-${index}`}>{String(line)}</p>
                    ))}
                </div>
            </div>
        </div>
    )
}
