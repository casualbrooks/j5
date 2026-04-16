import { useMemo, useState } from 'react'
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

export default function IntegrationCheckPanel() {
    const { connectionStatus, recentVisionObjects, recentObjectDetections, liveRace } = useRaceContext()
    const [previewBaseUrl, setPreviewBaseUrl] = useState(defaultPreviewBaseUrl)
    const normalizedBaseUrl = useMemo(() => normalizePreviewBaseUrl(previewBaseUrl), [previewBaseUrl])
    const lastVisionSeenAt = recentVisionObjects[0]?.seenAt || 0
    const secondsSinceVision = lastVisionSeenAt ? Math.max(0, Math.floor((Date.now() - lastVisionSeenAt) / 1000)) : null
    const raceStatus = liveRace?.status || 'setup'

    return (
        <div className="fade-in space-y-4">
            <h2 className="text-xl font-semibold text-[var(--color-text-primary)]">Perception Integration Check</h2>

            <div className="race-card p-4 space-y-3 text-sm">
                <p className="text-[var(--color-text-secondary)]">
                    Use this page to verify camera preview, ROS2 perception, and websocket updates without running the full race flow.
                </p>

                <div className="rounded border border-slate-700 bg-slate-950/70 p-3 space-y-2 text-xs text-slate-200">
                    <p><strong>1) Camera preview process</strong></p>
                    <code className="block overflow-x-auto rounded bg-black/40 p-2 text-emerald-300">
                        python3 scripts/pi_preflight.py --camera-source /dev/video0 --capture-file track_snapshot.jpg --serve-preview --preview-host 0.0.0.0 --preview-port 8091 --preview-fps 15
                    </code>
                    <p><strong>2) Perception process (choose one)</strong></p>
                    <code className="block overflow-x-auto rounded bg-black/40 p-2 text-emerald-300">
                        python -m perception.standalone.standalone_runner --camera-source /dev/video0 --camera-id cam1
                    </code>
                    <code className="block overflow-x-auto rounded bg-black/40 p-2 text-emerald-300">
                        ros2 run racetracker_perception perception_node
                    </code>
                </div>
            </div>

            <div className="race-card p-4 space-y-3">
                <h3 className="text-sm font-semibold text-[var(--color-text-primary)]">Live module status</h3>
                <ul className="space-y-2 text-sm">
                    <li>
                        UI websocket:{' '}
                        <span className={connectionStatus === 'connected' ? 'text-emerald-300' : 'text-amber-300'}>
                            {connectionStatus}
                        </span>
                    </li>
                    <li>
                        Vision object stream:{' '}
                        <span className={secondsSinceVision !== null && secondsSinceVision <= 3 ? 'text-emerald-300' : 'text-amber-300'}>
                            {secondsSinceVision === null ? 'waiting for objects' : `${secondsSinceVision}s since last object`}
                        </span>
                    </li>
                    <li>
                        Detection events in memory:{' '}
                        <span className={recentObjectDetections.length > 0 ? 'text-emerald-300' : 'text-amber-300'}>
                            {recentObjectDetections.length}
                        </span>
                    </li>
                    <li>
                        Race runtime status:{' '}
                        <span className={raceStatus === 'active' ? 'text-emerald-300' : 'text-slate-300'}>{raceStatus}</span>
                    </li>
                </ul>
            </div>

            <div className="race-card p-4 space-y-2">
                <h3 className="text-sm font-semibold text-[var(--color-text-primary)]">Quick links</h3>
                <label className="block text-sm text-[var(--color-text-secondary)]">
                    Preview server URL
                    <input
                        className="mt-1 w-full rounded border border-slate-600 bg-slate-900 px-3 py-2 text-sm text-white"
                        value={previewBaseUrl}
                        onChange={(event) => setPreviewBaseUrl(event.target.value)}
                        placeholder="http://192.168.1.134:8091"
                    />
                </label>
                <div className="flex flex-wrap gap-2 text-xs">
                    <a
                        className="rounded bg-slate-700 px-3 py-2 font-semibold text-white hover:bg-slate-600"
                        href={`${normalizedBaseUrl}/stream.mjpg`}
                        target="_blank"
                        rel="noreferrer"
                    >
                        Open camera stream
                    </a>
                    <a
                        className="rounded bg-slate-700 px-3 py-2 font-semibold text-white hover:bg-slate-600"
                        href={`${normalizedBaseUrl}/snapshot.jpg`}
                        target="_blank"
                        rel="noreferrer"
                    >
                        Open latest snapshot
                    </a>
                </div>
            </div>
        </div>
    )
}
