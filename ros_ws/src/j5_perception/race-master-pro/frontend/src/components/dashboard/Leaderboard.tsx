import { useState } from 'react'
import { ChevronDown, ChevronUp } from 'lucide-react'
import type { LiveRacer } from '@/types'
import { formatLapTime, formatGap, getPositionClass, stringToColor } from '@/lib/utils'

interface LeaderboardProps {
    racers: LiveRacer[]
    totalLaps: number
}

export default function Leaderboard({ racers, totalLaps }: LeaderboardProps) {
    const [expandedId, setExpandedId] = useState<string | null>(null)

    const sorted = [...racers].sort((a, b) => {
        // Sort by position (lower is better)
        if (a.current_position !== b.current_position) return a.current_position - b.current_position
        // Tie-break by total time
        return a.total_time - b.total_time
    })

    return (
        <div className="flex flex-col h-full">
            {/* Header */}
            <div className="leaderboard-row text-[var(--color-text-muted)] text-xs font-medium uppercase tracking-wider mb-1">
                <span>Pos</span>
                <span>Racer</span>
                <span className="text-right">Lap</span>
                <span className="text-right">Last Lap</span>
                <span className="text-right">Best Lap</span>
                <span className="text-right">Gap</span>
            </div>

            {/* Rows */}
            <div className="flex-1 overflow-y-auto space-y-0.5">
                {sorted.map((racer) => {
                    const isExpanded = expandedId === racer.racer_profile_id
                    const color = racer.color || stringToColor(racer.name)

                    return (
                        <div key={racer.racer_profile_id}>
                            {/* Main Row */}
                            <button
                                className={`leaderboard-row w-full text-left ${isExpanded ? 'expanded' : ''}`}
                                onClick={() => setExpandedId(isExpanded ? null : racer.racer_profile_id)}
                            >
                                {/* Position badge */}
                                <div className={`position-badge ${getPositionClass(racer.current_position)}`}>
                                    {racer.current_position}
                                </div>

                                {/* Racer info */}
                                <div className="flex items-center gap-2 min-w-0">
                                    <div
                                        className="w-3 h-3 rounded-full flex-shrink-0"
                                        style={{ backgroundColor: color }}
                                    />
                                    <span className="font-medium truncate text-sm">
                                        {racer.name}
                                    </span>
                                    <span className="text-[var(--color-text-muted)] text-xs flex-shrink-0">
                                        #{racer.number}
                                    </span>
                                    {racer.status === 'dnf' && (
                                        <span className="text-[var(--color-status-dnf)] text-xs font-bold">DNF</span>
                                    )}
                                    {racer.status === 'finished' && (
                                        <span className="text-[var(--color-status-finished)] text-xs font-bold">FIN</span>
                                    )}
                                </div>

                                {/* Lap count */}
                                <div className="text-right text-sm font-timing">
                                    {racer.current_lap}/{totalLaps}
                                </div>

                                {/* Last lap */}
                                <div className="text-right text-sm font-timing">
                                    {formatLapTime(racer.last_lap_time)}
                                </div>

                                {/* Best lap */}
                                <div className="text-right text-sm font-timing text-[var(--color-accent-green)]">
                                    {formatLapTime(racer.best_lap_time)}
                                </div>

                                {/* Gap */}
                                <div className="text-right text-xs text-[var(--color-text-secondary)]">
                                    {formatGap(racer.gap_to_leader)}
                                    {isExpanded ? <ChevronUp size={12} className="inline ml-1" /> : <ChevronDown size={12} className="inline ml-1" />}
                                </div>
                            </button>

                            {/* Expanded Detail */}
                            {isExpanded && (
                                <div className="slide-in-right ml-10 mr-4 mb-2 p-3 rounded-lg bg-[var(--color-bg-surface)] border border-[rgba(255,255,255,0.06)] grid grid-cols-4 gap-3 text-xs">
                                    <div>
                                        <div className="text-[var(--color-text-muted)]">Total Time</div>
                                        <div className="font-timing text-sm">{formatLapTime(racer.total_time)}</div>
                                    </div>
                                    <div>
                                        <div className="text-[var(--color-text-muted)]">Current Lap</div>
                                        <div className="font-timing text-sm">{formatLapTime(racer.current_lap_time)}</div>
                                    </div>
                                    <div>
                                        <div className="text-[var(--color-text-muted)]">Best Lap</div>
                                        <div className="font-timing text-sm text-[var(--color-accent-green)]">
                                            {formatLapTime(racer.best_lap_time)}
                                        </div>
                                    </div>
                                    <div>
                                        <div className="text-[var(--color-text-muted)]">Status</div>
                                        <div className="text-sm font-medium capitalize">{racer.status}</div>
                                    </div>
                                </div>
                            )}
                        </div>
                    )
                })}
            </div>
        </div>
    )
}
