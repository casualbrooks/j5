import { FormEvent, useEffect, useMemo, useState } from 'react'
import { apiFetch, getSelectedTrackId, getTrackRacerAssignments, parseTrackRecord, setLatestSnapshotUrl, setSelectedTrackId, setTrackPhotoUrl } from '@/lib/utils'
import { useRaceContext } from '@/stores/raceStore'

function defaultPreviewBaseUrl(): string {
    if (typeof window === 'undefined') {
        return 'http://localhost:8091'
    }
    return ''
}


function extractObjectNumber(objectId: string): string {
    const match = objectId.match(/(\d+)(?!.*\d)/)
    return match ? match[1] : objectId
}

function parseNumber(value: unknown): number | null {
    const n = Number(value)
    return Number.isFinite(n) ? n : null
}

function objectIdClass(mapped: boolean): string {
    return mapped ? 'text-emerald-300' : 'text-amber-300'
}

function trackingStatusClass(enabled: boolean): string {
    return enabled ? 'text-emerald-400' : 'text-amber-300'
}

function formatHostForUrl(host: string): string {
    const normalized = host.trim()
    if (!normalized) return ''
    if (normalized.includes(':') && !normalized.startsWith('[') && !normalized.endsWith(']')) {
        return `[${normalized}]`
    }
    return normalized
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

function resolveWizardPreviewUrl(previewUrl: string, piHost?: string): string {
    const raw = previewUrl.trim()
    if (!raw) return ''
    try {
        const parsed = new URL(raw)
        const host = parsed.hostname.toLowerCase().replace(/^\[|\]$/g, '')
        if (host === 'localhost' || host === '127.0.0.1' || host === '::1') {
            const candidatePiHost = String(piHost || '').trim()
            if (candidatePiHost && !['localhost', '127.0.0.1', '::1'].includes(candidatePiHost.toLowerCase())) {
                return `${parsed.protocol}//${formatHostForUrl(candidatePiHost)}:${parsed.port || '8091'}`
            }
            if (typeof window !== 'undefined' && window.location.hostname) {
                return `${window.location.protocol}//${window.location.hostname}:${parsed.port || '8091'}`
            }
        }
        return `${parsed.protocol}//${parsed.host}`
    } catch {
        return raw
    }
}

export default function VisionPanel() {
    const { liveRace, recentObjectDetections, recentVisionObjects, connectionStatus } = useRaceContext()
    const [previewBaseUrl, setPreviewBaseUrl] = useState(defaultPreviewBaseUrl)
    const [previewEnabled, setPreviewEnabled] = useState(false)
    const [statusMessage, setStatusMessage] = useState('')
    const [statusError, setStatusError] = useState(false)
    const [capturing, setCapturing] = useState(false)
    const [raceContext, setRaceContext] = useState<Record<string, unknown>>({})
    const [activeTrackName, setActiveTrackName] = useState('')
    const [statusNowMs, setStatusNowMs] = useState(() => Date.now())

    const normalizedBaseUrl = useMemo(() => normalizePreviewBaseUrl(previewBaseUrl), [previewBaseUrl])
    const streamUrl = normalizedBaseUrl ? `${normalizedBaseUrl}/stream.mjpg` : ''

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
            const previewUrl = resolveWizardPreviewUrl(
                String(payload?.config?.preview_url || ''),
                String(payload?.config?.pi_host || ''),
            )
            if (previewUrl) {
                setPreviewBaseUrl((current) => (current.trim() ? current : previewUrl))
            }
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
            if (!normalizedBaseUrl) {
                throw new Error('Set a valid Preview server URL before capturing a track photo.')
            }
            const response = await fetch(`${normalizedBaseUrl}/capture`, {
                method: 'POST',
            })
            let payload: { ok?: boolean, message?: string } = {}
            try {
                payload = await response.json() as { ok?: boolean, message?: string }
            } catch {
                payload = { ok: response.ok, message: response.ok ? 'Capture request completed.' : 'Capture request failed.' }
            }

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
    const detectionEmptyMessage = trackingEnabled
        ? 'No detections yet. Start tracking and assign object IDs in Settings.'
        : 'Start tracking to show object IDs and annotation events.'
    const trackingStatusText = trackingEnabled ? 'Enabled' : 'Disabled'
    const statusClassName = statusError ? 'text-red-400' : 'text-emerald-400'
    const racerAssignments = getSelectedTrackId() ? getTrackRacerAssignments(getSelectedTrackId()) : {}
    const visibleVisionObjects = trackingEnabled ? recentVisionObjects : []
    const hasRecentVisionObjects = visibleVisionObjects.length > 0
    const drawableVisionObjects = visibleVisionObjects
        .slice(0, 12)
        .map((item) => {
            const x = parseNumber(item.bbox?.x)
            const y = parseNumber(item.bbox?.y)
            const width = parseNumber(item.bbox?.width)
            const height = parseNumber(item.bbox?.height)
            const frameWidth = parseNumber(item.frameSize?.width)
            const frameHeight = parseNumber(item.frameSize?.height)
            if (x == null || y == null || width == null || height == null || frameWidth == null || frameHeight == null) return null
            return {
                item,
                leftPct: (x / frameWidth) * 100,
                topPct: (y / frameHeight) * 100,
                widthPct: (width / frameWidth) * 100,
                heightPct: (height / frameHeight) * 100,
            }
        })
        .filter((entry): entry is {
            item: typeof recentVisionObjects[number]
            leftPct: number
            topPct: number
            widthPct: number
            heightPct: number
        } => entry !== null)
    const hasDrawableVisionObjects = drawableVisionObjects.length > 0
    const latestVisionObject = visibleVisionObjects[0]
    const visionFresh = Boolean(latestVisionObject && statusNowMs - latestVisionObject.seenAt <= 3000)
    const previewContent = (() => {
        if (!previewEnabled) {
            return (
                <div className="rounded border border-dashed border-slate-700 bg-black/30 p-4 text-xs text-[var(--color-text-secondary)]">
                    Live preview is paused to avoid multiple stream consumers.
                </div>
            )
        }
        if (!normalizedBaseUrl) {
            return (
                <div className="rounded border border-dashed border-amber-700 bg-amber-950/20 p-4 text-xs text-amber-200">
                    Set Preview server URL first (for example <code>http://100.90.148.71:8091</code>), then click Show Live Preview.
                </div>
            )
        }
        return (
            <div className="relative">
                    <img
                        src={streamUrl}
                        alt="Live camera preview"
                        className="w-full rounded border border-slate-700 bg-black"
                    />
                    <div className="pointer-events-none absolute inset-0">
                        {drawableVisionObjects.map(({ item, leftPct, topPct, widthPct, heightPct }) => {
                            return (
                                <div
                                    key={`${item.objectId}-${item.seenAt}`}
                                    className="absolute border border-emerald-400/90"
                                    style={{ left: `${leftPct}%`, top: `${topPct}%`, width: `${widthPct}%`, height: `${heightPct}%` }}
                                >
                                    <span className="absolute -top-4 right-0 rounded bg-black/75 px-1.5 py-0.5 text-[10px] font-semibold text-emerald-200">
                                        {extractObjectNumber(item.objectId)}
                                    </span>
                                </div>
                            )
                        })}
                        {hasRecentVisionObjects && !hasDrawableVisionObjects ? (
                            <div className="absolute left-2 top-2 max-w-[65%] space-y-1">
                                {visibleVisionObjects.slice(0, 8).map((item) => (
                                    <p key={`${item.objectId}-${item.seenAt}`} className="rounded bg-black/70 px-2 py-1 text-[11px] text-emerald-200">
                                        {extractObjectNumber(item.objectId)}
                                        {item.cameraTopic ? ` · ${item.cameraTopic}` : ''}
                                    </p>
                                ))}
                            </div>
                        ) : null}
                        {!hasRecentVisionObjects ? (
                            <p className="absolute left-2 top-2 rounded bg-black/70 px-2 py-1 text-[11px] text-amber-200">
                                {trackingEnabled
                                    ? 'No tracking objects yet. Move racers through the frame.'
                                    : 'Tracking overlay is hidden until Start Tracking is pressed.'}
                            </p>
                        ) : null}
                    </div>
            </div>
        )
    })()

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
        let failureMessage = start ? 'Unable to start object tracking.' : 'Unable to stop object tracking.'
        try {
            const payload = await response.json() as { detail?: string | { message?: string } }
            if (typeof payload.detail === 'string' && payload.detail.trim()) {
                failureMessage = payload.detail
            } else if (payload.detail && typeof payload.detail.message === 'string' && payload.detail.message.trim()) {
                failureMessage = payload.detail.message
            }
        } catch {
            // keep default fallback message
        }
        setStatusMessage(failureMessage)
        setStatusError(true)
    }

    try {
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
                    The preview stream shows live object bounding boxes when detection payloads include bbox coordinates. Object labels show only the numeric ID so it is easier to match racers.
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
                        onClick={() => {
                            if (!previewEnabled && !previewBaseUrl.trim()) {
                                void refreshWizardContext()
                            }
                            setPreviewEnabled(prev => !prev)
                        }}
                    >
                        {previewEnabled ? 'Hide Live Preview' : 'Show Live Preview'}
                    </button>
                    <p className="text-xs text-[var(--color-text-secondary)]">Use this only while capturing the track image, then hide it before opening other camera consumers.</p>
                </div>
                <div className="rounded border border-slate-700 bg-slate-950/50 px-3 py-2 text-xs text-slate-200">
                    <p>
                        Websocket: <span className={connectionStatus === 'connected' ? 'text-emerald-300' : 'text-amber-300'}>{connectionStatus}</span>
                        {' · '}
                        Detection stream: <span className={visionFresh ? 'text-emerald-300' : 'text-amber-300'}>{visionFresh ? 'receiving objects' : 'waiting for objects'}</span>
                    </p>
                    <p className="mt-1 text-[11px] text-slate-300">
                        If your ROS2 node is running, this can still show waiting when camera frames are not on the expected topic, no objects are visible yet, or confidence filtering drops detections.
                    </p>
                </div>
                {previewContent}

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
                    Tracking status: <span className={trackingStatusClass(trackingEnabled)}>{trackingStatusText}</span>
                </p>

                {statusMessage && <p className={`text-sm ${statusClassName}`}>{statusMessage}</p>}
            </div>

            <div className="race-card p-4 space-y-2">
                <h3 className="text-sm font-semibold text-[var(--color-text-primary)]">Recent object detections</h3>
                {(!trackingEnabled || recentObjectDetections.length === 0) && <p className="text-xs text-[var(--color-text-secondary)]">{detectionEmptyMessage}</p>}
                {trackingEnabled && recentObjectDetections.length > 0 ? (
                    <div className="max-h-52 overflow-auto rounded border border-slate-700 bg-slate-950 p-2 text-xs text-slate-200 space-y-1">
                        {recentObjectDetections.map(item => {
                            const racer = (liveRace?.racers || []).find(entry => entry.racer_profile_id === item.racerProfileId)
                            const assignedId = racer ? racerAssignments[racer.racer_profile_id] : ''
                            const mapped = Boolean(racer && assignedId && assignedId === item.objectId)
                            return (
                                <p key={`${item.objectId}-${item.seenAt}`}>
                                    <span className={objectIdClass(mapped)}>{item.objectId}</span>
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
                {logs.length === 0 && <p className="text-xs text-[var(--color-text-secondary)]">No logs yet. Start tracking to populate this feed.</p>}
                <div className="max-h-56 overflow-auto rounded border border-slate-700 bg-slate-950 p-2 text-xs text-slate-200">
                    {logs.map((line, index) => (
                        <p key={`${line}-${index}`}>{String(line)}</p>
                    ))}
                </div>
            </div>
            </div>
        )
    } catch (error) {
        const message = error instanceof Error ? error.message : String(error)
        return (
            <div className="fade-in space-y-4">
                <h2 className="text-xl font-semibold text-[var(--color-text-primary)]">Computer Vision</h2>
                <div className="race-card p-4 space-y-2">
                    <p className="text-sm text-red-300">Vision panel rendering failed, but the app recovered.</p>
                    <p className="text-xs text-[var(--color-text-secondary)]">Please refresh this tab and restart the frontend dev server if the error persists.</p>
                    <p className="text-xs text-amber-300 break-all">Error: {message}</p>
                </div>
            </div>
        )
    }
}
