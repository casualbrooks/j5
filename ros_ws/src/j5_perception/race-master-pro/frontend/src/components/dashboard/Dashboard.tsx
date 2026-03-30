import { useEffect, useState } from 'react'
import { useRaceContext } from '@/stores/raceStore'
import Leaderboard from './Leaderboard'
import TrackCanvas from '@/components/track/TrackCanvas'
import type { Checkpoint, Track } from '@/types'
import {
    TRACK_OVERLAY_EVENT,
    apiFetch,
    getLatestSnapshotUrl,
    getSelectedTrackId,
    getTrackCheckpoints,
    getTrackPhotoUrl,
    parseTrackRecord,
} from '@/lib/utils'

export default function Dashboard() {
    const { liveRace, showTrack } = useRaceContext()
    const [track, setTrack] = useState<Track | null>(null)
    const [trackPhotoUrl, setTrackPhotoUrl] = useState('')
    const [checkpoints, setCheckpoints] = useState<Checkpoint[]>([])

    useEffect(() => {
        let cancelled = false

        const loadTrack = async () => {
            const selectedTrackId = getSelectedTrackId()
            const listResponse = await apiFetch('/api/tracks')
            if (!listResponse.ok || cancelled) return
            const listPayload = await listResponse.json()
            const tracks = Array.isArray(listPayload)
                ? listPayload.map((item) => parseTrackRecord(item as Record<string, unknown>))
                : []
            const nextTrack = tracks.find((item) => item.id === selectedTrackId) || tracks[0] || null
            setTrack(nextTrack)
            setTrackPhotoUrl(nextTrack ? getTrackPhotoUrl(nextTrack.id) : '')
            setCheckpoints(nextTrack ? getTrackCheckpoints(nextTrack.id) : [])
        }

        void loadTrack()

        const handleOverlayChange = () => {
            void loadTrack()
        }
        window.addEventListener(TRACK_OVERLAY_EVENT, handleOverlayChange)
        return () => {
            cancelled = true
            window.removeEventListener(TRACK_OVERLAY_EVENT, handleOverlayChange)
        }
    }, [])


    if (!liveRace) {
        return (
            <div className="flex flex-col items-center justify-center h-full text-center fade-in">
                <div className="text-6xl mb-4">🏁</div>
                <h2 className="text-2xl font-semibold text-[var(--color-text-primary)] mb-2">
                    No Active Race
                </h2>
                <p className="text-[var(--color-text-muted)] max-w-md">
                    Create a new race from the Settings tab, or select an existing race to begin tracking.
                </p>
            </div>
        )
    }

    return (
        <div className="h-full flex flex-col gap-4 fade-in">
            {showTrack && (
                <div className="race-card" style={{ minHeight: 260, maxHeight: '42%' }}>
                    <div className="mb-3 flex flex-wrap items-center justify-between gap-3">
                        <div>
                            <h3 className="text-sm font-semibold text-[var(--color-text-primary)]">
                                {track?.name || 'Track preview'}
                            </h3>
                            <p className="text-xs text-[var(--color-text-secondary)]">
                                Saved spline path is drawn over the captured track photo. Racer dots follow camera updates when available, and otherwise interpolate along the spline.
                            </p>
                        </div>
                        {track?.layout_points.length ? (
                            <span className="rounded-full bg-blue-500/10 px-3 py-1 text-xs font-semibold text-blue-300">
                                {track.layout_points.length} spline points
                            </span>
                        ) : null}
                    </div>
                    <TrackCanvas
                        racers={liveRace.racers}
                        layoutPoints={track?.layout_points || []}
                        checkpoints={checkpoints}
                        backgroundImageUrl={trackPhotoUrl || undefined}
                    />
                </div>
            )}

            <div className="flex-1 min-h-0">
                <Leaderboard racers={liveRace.racers} totalLaps={liveRace.total_laps} />
            </div>
        </div>
    )
}
