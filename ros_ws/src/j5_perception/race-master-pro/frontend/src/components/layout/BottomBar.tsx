import { Play, Pause, Flag } from 'lucide-react'
import type { LiveRaceState } from '@/types'
import type { ConnectionStatus } from '@/stores/raceStore'
import { useRaceContext } from '@/stores/raceStore'
import { formatRaceTime } from '@/lib/utils'
import { useState, useEffect } from 'react'

interface BottomBarProps {
    liveRace: LiveRaceState | null
    connectionStatus: ConnectionStatus
}

export default function BottomBar({ liveRace, connectionStatus }: BottomBarProps) {
    const { sendWsMessage } = useRaceContext()
    const [elapsed, setElapsed] = useState(0)

    // Tick the elapsed timer when race is active
    useEffect(() => {
        if (liveRace?.status !== 'active') {
            setElapsed(liveRace?.elapsed_time ?? 0)
            return
        }
        const interval = setInterval(() => {
            setElapsed(prev => prev + 100)
        }, 100)
        return () => clearInterval(interval)
    }, [liveRace?.status, liveRace?.elapsed_time])

    const handleStart = () => {
        sendWsMessage({ type: 'raceStart', data: { race_id: liveRace?.race_id } })
    }

    const handlePause = () => {
        sendWsMessage({ type: 'racePause', data: { race_id: liveRace?.race_id } })
    }

    const handleFinish = () => {
        sendWsMessage({ type: 'raceFinish', data: { race_id: liveRace?.race_id } })
    }

    const maxLap = liveRace
        ? Math.max(...liveRace.racers.map(r => r.current_lap), 0)
        : 0

    return (
        <footer className="app-bottombar">
            {/* Race Controls */}
            <div className="flex items-center gap-2">
                {liveRace?.status === 'active' ? (
                    <>
                        <button
                            onClick={handlePause}
                            className="flex items-center gap-1 px-3 py-1.5 rounded-md text-xs font-medium bg-[var(--color-accent-orange)] text-black hover:opacity-90 transition-opacity"
                        >
                            <Pause size={12} /> Pause
                        </button>
                        <button
                            onClick={handleFinish}
                            className="flex items-center gap-1 px-3 py-1.5 rounded-md text-xs font-medium bg-[var(--color-accent-green)] text-black hover:opacity-90 transition-opacity"
                        >
                            <Flag size={12} /> Finish
                        </button>
                    </>
                ) : liveRace?.status === 'setup' || liveRace?.status === 'paused' ? (
                    <button
                        onClick={handleStart}
                        className="flex items-center gap-1 px-3 py-1.5 rounded-md text-xs font-medium bg-[var(--color-accent-red)] text-white hover:opacity-90 transition-opacity"
                    >
                        <Play size={12} /> {liveRace.status === 'paused' ? 'Resume' : 'Start Race'}
                    </button>
                ) : (
                    <span className="text-[var(--color-text-muted)] text-xs">No active race</span>
                )}
            </div>

            {/* Spacer */}
            <div className="flex-1" />

            {/* Race Timer */}
            {liveRace && (
                <div className="flex items-center gap-4 text-xs">
                    <div className="font-timing text-[var(--color-text-primary)] text-sm">
                        {formatRaceTime(elapsed)}
                    </div>
                    <div className="text-[var(--color-text-muted)]">
                        Lap {maxLap}/{liveRace.total_laps}
                    </div>
                </div>
            )}

            {/* Spacer */}
            <div className="flex-1" />

            {/* Status Indicators */}
            <div className="flex items-center gap-3 text-xs text-[var(--color-text-muted)]">
                <span>
                    WS: {connectionStatus === 'connected' ? '🟢' : connectionStatus === 'connecting' ? '🟡' : '🔴'}
                </span>
            </div>
        </footer>
    )
}
