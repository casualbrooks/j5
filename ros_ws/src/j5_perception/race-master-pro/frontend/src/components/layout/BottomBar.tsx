import { Play, Pause, Flag } from 'lucide-react'
import type { LiveRaceState } from '@/types'
import type { ConnectionStatus } from '@/stores/raceStore'
import { useRaceContext } from '@/stores/raceStore'
import { formatRaceTime, apiFetch } from '@/lib/utils'
import { useState, useEffect } from 'react'

interface BottomBarProps {
    liveRace: LiveRaceState | null
    connectionStatus: ConnectionStatus
}

export default function BottomBar({ liveRace, connectionStatus }: BottomBarProps) {
    const { refreshRaceState } = useRaceContext()
    const [elapsed, setElapsed] = useState(0)

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

    const controlRace = async (action: 'start' | 'pause' | 'finish' | 'resume') => {
        if (!liveRace?.race_id) return
        const endpointMap = {
            start: `/api/races/${liveRace.race_id}/start`,
            pause: `/api/races/${liveRace.race_id}/pause`,
            finish: `/api/races/${liveRace.race_id}/finish`,
            resume: `/api/races/${liveRace.race_id}/resume`,
        }
        const response = await apiFetch(endpointMap[action], { method: 'POST' })
        if (response.ok) {
            await refreshRaceState(liveRace.race_id)
        }
    }

    const maxLap = liveRace
        ? Math.max(...liveRace.racers.map(r => r.current_lap), 0)
        : 0

    return (
        <footer className="app-bottombar">
            <div className="flex items-center gap-2">
                {liveRace?.status === 'active' ? (
                    <>
                        <button
                            onClick={() => controlRace('pause')}
                            className="flex items-center gap-1 px-3 py-1.5 rounded-md text-xs font-medium bg-[var(--color-accent-orange)] text-black hover:opacity-90 transition-opacity"
                        >
                            <Pause size={12} /> Pause
                        </button>
                        <button
                            onClick={() => controlRace('finish')}
                            className="flex items-center gap-1 px-3 py-1.5 rounded-md text-xs font-medium bg-[var(--color-accent-green)] text-black hover:opacity-90 transition-opacity"
                        >
                            <Flag size={12} /> Finish
                        </button>
                    </>
                ) : liveRace?.status === 'setup' || liveRace?.status === 'paused' ? (
                    <button
                        onClick={() => controlRace(liveRace.status === 'paused' ? 'resume' : 'start')}
                        className="flex items-center gap-1 px-3 py-1.5 rounded-md text-xs font-medium bg-[var(--color-accent-red)] text-white hover:opacity-90 transition-opacity"
                    >
                        <Play size={12} /> {liveRace.status === 'paused' ? 'Resume' : 'Start Race'}
                    </button>
                ) : (
                    <span className="text-[var(--color-text-muted)] text-xs">No active race</span>
                )}
            </div>

            <div className="flex-1" />

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

            <div className="flex-1" />

            <div className="flex items-center gap-3 text-xs text-[var(--color-text-muted)]">
                <span>
                    WS: {connectionStatus === 'connected' ? '🟢' : connectionStatus === 'connecting' ? '🟡' : '🔴'}
                </span>
            </div>
        </footer>
    )
}
