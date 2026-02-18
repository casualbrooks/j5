import { Wifi, WifiOff, Settings, Map } from 'lucide-react'
import type { LiveRaceState } from '@/types'
import type { ConnectionStatus } from '@/stores/raceStore'
import { useRaceContext } from '@/stores/raceStore'

interface TopBarProps {
    connectionStatus: ConnectionStatus
    activeRace: LiveRaceState | null
}

export default function TopBar({ connectionStatus, activeRace }: TopBarProps) {
    const { showTrack, toggleTrack } = useRaceContext()

    return (
        <header className="app-topbar">
            {/* Logo / Brand */}
            <div className="flex items-center gap-2 mr-4">
                <div className="w-7 h-7 rounded-md bg-[var(--color-accent-red)] flex items-center justify-center">
                    <span className="text-white font-bold text-xs">RT</span>
                </div>
                <span className="font-semibold text-sm text-[var(--color-text-primary)] hidden sm:inline">
                    Race Master Pro
                </span>
            </div>

            {/* Connection Status */}
            <div className="flex items-center gap-2 text-xs">
                {connectionStatus === 'connected' ? (
                    <>
                        <span className="status-dot connected" />
                        <Wifi size={14} className="text-[var(--color-status-connected)]" />
                        <span className="text-[var(--color-text-secondary)]">Online</span>
                    </>
                ) : connectionStatus === 'connecting' ? (
                    <>
                        <span className="status-dot warning" />
                        <span className="text-[var(--color-status-warning)]">Connecting…</span>
                    </>
                ) : (
                    <>
                        <span className="status-dot disconnected" />
                        <WifiOff size={14} className="text-[var(--color-status-disconnected)]" />
                        <span className="text-[var(--color-text-muted)]">Offline</span>
                    </>
                )}
            </div>

            {/* Spacer */}
            <div className="flex-1" />

            {/* Active Race */}
            {activeRace && (
                <div className="flex items-center gap-2 text-sm">
                    <span className="text-[var(--color-accent-yellow)] font-semibold">
                        🏁 {activeRace.race_name}
                    </span>
                    <span className="text-[var(--color-text-muted)]">
                        — {activeRace.status === 'active' ? 'LIVE' : activeRace.status.toUpperCase()}
                    </span>
                </div>
            )}

            {/* Spacer */}
            <div className="flex-1" />

            {/* Track Toggle */}
            <button
                onClick={toggleTrack}
                className="flex items-center gap-1 px-2 py-1 rounded-md text-xs transition-colors hover:bg-[var(--color-bg-card)]"
                style={{ color: showTrack ? 'var(--color-accent-blue)' : 'var(--color-text-muted)' }}
                title={showTrack ? 'Hide Track Map' : 'Show Track Map'}
            >
                <Map size={14} />
                <span className="hidden sm:inline">Track</span>
            </button>
        </header>
    )
}
