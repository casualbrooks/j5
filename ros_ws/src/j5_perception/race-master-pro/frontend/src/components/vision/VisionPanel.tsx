import { FormEvent, useEffect, useMemo, useState } from 'react'
import { apiFetch, getSelectedTrackId, setLatestSnapshotUrl, setTrackPhotoUrl } from '@/lib/utils'

function defaultPreviewBaseUrl(): string {
    if (typeof window === 'undefined') {
        return 'http://localhost:8091'
    }
    return `http://${window.location.hostname}:8091`
}

export default function VisionPanel() {
    const [previewBaseUrl, setPreviewBaseUrl] = useState(defaultPreviewBaseUrl)
    const [statusMessage, setStatusMessage] = useState('')
    const [statusError, setStatusError] = useState(false)
    const [capturing, setCapturing] = useState(false)
    const [raceContext, setRaceContext] = useState<Record<string, unknown>>({})

    const normalizedBaseUrl = useMemo(() => previewBaseUrl.trim().replace(/\/$/, ''), [previewBaseUrl])
    const streamUrl = `${normalizedBaseUrl}/stream.mjpg`

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
        refreshWizardContext()
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
            const selectedTrackId = getSelectedTrackId()
            setLatestSnapshotUrl(snapshotUrl)
            if (selectedTrackId) {
                setTrackPhotoUrl(selectedTrackId, snapshotUrl)
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
            await refreshWizardContext()
            setStatusMessage(start ? 'Object tracking started.' : 'Object tracking stopped.')
            setStatusError(false)
        }
    }

    return (
        <div className="fade-in space-y-4">
            <h2 className="text-xl font-semibold text-[var(--color-text-primary)]">Computer Vision</h2>

            <div className="race-card p-4 space-y-3">
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
                <div className="rounded border border-slate-700 bg-slate-950 p-3">
                    <p className="text-xs font-semibold uppercase tracking-wide text-slate-400">Start camera feed only</p>
                    <p className="mt-1 text-xs text-[var(--color-text-secondary)]">
                        You can bring the camera preview back without re-initializing the race by running this on the Pi:
                    </p>
                    <code className="mt-2 block overflow-x-auto rounded bg-black/40 p-2 text-xs text-emerald-300">
                        python3 scripts/pi_preflight.py --camera-source /dev/video0 --capture-file track_snapshot.jpg --serve-preview --preview-host 0.0.0.0 --preview-port 8091
                    </code>
                    <p className="mt-2 text-xs text-[var(--color-text-secondary)]">
                        Then open the preview URL above, or load the stream again in this tab. Replace <code>/dev/video0</code> if your
                        camera uses a different device.
                    </p>
                </div>
            </div>

            <div className="race-card p-4 space-y-3">
                <img
                    src={streamUrl}
                    alt="Live camera preview"
                    className="w-full rounded border border-slate-700 bg-black"
                />

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
