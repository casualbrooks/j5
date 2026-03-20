import { useEffect, useMemo, useState } from 'react'
import { apiFetch, formatLapTime } from '@/lib/utils'
import { useRaceContext } from '@/stores/raceStore'
import type { Track } from '@/types'

interface TrackRecordRow {
    id: string
    lap_time: number
    racer_name: string
    racer_number: string
    recorded_at?: string
}

function parseTrack(raw: Record<string, unknown>): Track {
    return {
        id: String(raw.id || ''),
        name: String(raw.name || 'Track'),
        scale: String(raw.scale || '1:24'),
        track_distance: raw.track_distance == null ? null : Number(raw.track_distance),
        layout_points: [],
        boundary_polygon: [],
        created_at: raw.created_at ? String(raw.created_at) : undefined,
    }
}

export default function Analytics() {
    const { liveRace } = useRaceContext()
    const [tracks, setTracks] = useState<Track[]>([])
    const [selectedTrackId, setSelectedTrackId] = useState('')
    const [records, setRecords] = useState<TrackRecordRow[]>([])
    const [status, setStatus] = useState('')

    useEffect(() => {
        const loadTracks = async () => {
            try {
                const response = await apiFetch('/api/tracks')
                if (!response.ok) throw new Error('Failed to load tracks')
                const payload = await response.json()
                const parsed = Array.isArray(payload)
                    ? payload.map(item => parseTrack(item as Record<string, unknown>))
                    : []
                setTracks(parsed)
                if (!selectedTrackId && parsed[0]?.id) {
                    setSelectedTrackId(parsed[0].id)
                }
            } catch (error) {
                setStatus(error instanceof Error ? error.message : 'Unable to load tracks.')
            }
        }
        void loadTracks()
    }, [selectedTrackId])

    useEffect(() => {
        if (!selectedTrackId) return
        const loadRecords = async () => {
            try {
                const response = await apiFetch(`/api/analytics/track/${selectedTrackId}/records`)
                if (!response.ok) throw new Error('Failed to load track records')
                const payload = await response.json()
                setRecords(Array.isArray(payload) ? payload as TrackRecordRow[] : [])
                setStatus('')
            } catch (error) {
                setStatus(error instanceof Error ? error.message : 'Unable to load track records.')
            }
        }
        void loadRecords()
    }, [selectedTrackId])

    const liveStats = useMemo(() => {
        const racers = liveRace?.racers || []
        if (racers.length === 0) return null
        const bestLap = racers
            .map(racer => racer.best_lap_time)
            .filter((value): value is number => value != null)
            .sort((a, b) => a - b)[0] || null
        const averageGap = racers.length > 1
            ? racers.slice(1).reduce((sum, racer) => sum + racer.gap_to_leader, 0) / (racers.length - 1)
            : 0
        return {
            racers: racers.length,
            leader: racers[0]?.name || '—',
            bestLap,
            averageGap,
        }
    }, [liveRace?.racers])

    return (
        <div className="fade-in space-y-4">
            <div>
                <h2 className="text-xl font-semibold text-[var(--color-text-primary)]">Analytics</h2>
                <p className="text-sm text-[var(--color-text-secondary)]">
                    Track records, live race pacing, and simple lag/performance diagnostics pulled from the existing backend analytics endpoints.
                </p>
            </div>

            <div className="grid gap-4 md:grid-cols-4">
                <div className="race-card"><p className="text-xs text-[var(--color-text-muted)]">Live racers</p><p className="mt-2 text-2xl font-semibold">{liveStats?.racers ?? 0}</p></div>
                <div className="race-card"><p className="text-xs text-[var(--color-text-muted)]">Current leader</p><p className="mt-2 text-2xl font-semibold">{liveStats?.leader || '—'}</p></div>
                <div className="race-card"><p className="text-xs text-[var(--color-text-muted)]">Session best lap</p><p className="mt-2 text-2xl font-semibold">{liveStats?.bestLap ? formatLapTime(liveStats.bestLap * 1000) : '—'}</p></div>
                <div className="race-card"><p className="text-xs text-[var(--color-text-muted)]">Avg gap</p><p className="mt-2 text-2xl font-semibold">{liveStats ? `${(liveStats.averageGap / 1000).toFixed(2)}s` : '—'}</p></div>
            </div>

            <div className="race-card space-y-4">
                <div className="flex flex-wrap items-center justify-between gap-3">
                    <div>
                        <h3 className="text-base font-semibold">Track records</h3>
                        <p className="text-xs text-[var(--color-text-secondary)]">Top lap records for the selected track.</p>
                    </div>
                    <select className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm text-white" value={selectedTrackId} onChange={event => setSelectedTrackId(event.target.value)}>
                        <option value="">Select a track</option>
                        {tracks.map(track => <option key={track.id} value={track.id}>{track.name}</option>)}
                    </select>
                </div>

                {status ? <p className="text-sm text-[var(--color-text-secondary)]">{status}</p> : null}

                <div className="overflow-x-auto">
                    <table className="min-w-full text-sm">
                        <thead>
                            <tr className="text-left text-[var(--color-text-muted)]">
                                <th className="pb-2">Pos</th>
                                <th className="pb-2">Racer</th>
                                <th className="pb-2">Number</th>
                                <th className="pb-2">Lap</th>
                                <th className="pb-2">Recorded</th>
                            </tr>
                        </thead>
                        <tbody>
                            {records.map((record, index) => (
                                <tr key={record.id} className="border-t border-white/6">
                                    <td className="py-2">{index + 1}</td>
                                    <td className="py-2">{record.racer_name}</td>
                                    <td className="py-2">#{record.racer_number || '—'}</td>
                                    <td className="py-2 font-semibold">{formatLapTime(record.lap_time * 1000)}</td>
                                    <td className="py-2 text-[var(--color-text-secondary)]">{record.recorded_at ? new Date(record.recorded_at).toLocaleString() : '—'}</td>
                                </tr>
                            ))}
                        </tbody>
                    </table>
                </div>
            </div>
        </div>
    )
}
