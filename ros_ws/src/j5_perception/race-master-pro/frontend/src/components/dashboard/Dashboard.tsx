import { useRaceContext } from '@/stores/raceStore'
import Leaderboard from './Leaderboard'
import TrackCanvas from '@/components/track/TrackCanvas'

export default function Dashboard() {
    const { liveRace, showTrack } = useRaceContext()

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
            {/* Track Map (toggleable) */}
            {showTrack && (
                <div className="race-card" style={{ minHeight: 200, maxHeight: '35%' }}>
                    <TrackCanvas
                        racers={liveRace.racers}
                        layoutPoints={[]}
                        checkpoints={[]}
                    />
                </div>
            )}

            {/* Leaderboard */}
            <div className="flex-1 min-h-0">
                <Leaderboard racers={liveRace.racers} totalLaps={liveRace.total_laps} />
            </div>
        </div>
    )
}
