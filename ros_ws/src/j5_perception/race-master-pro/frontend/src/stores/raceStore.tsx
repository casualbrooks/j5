import { createContext, useContext, useState, useCallback, useRef, useEffect, type ReactNode } from 'react'
import type { LiveRaceState, LiveRacer } from '@/types'
import { apiFetch, backendWsUrl, stringToColor } from '@/lib/utils'

export type ConnectionStatus = 'connected' | 'connecting' | 'disconnected'

interface RaceContextValue {
    liveRace: LiveRaceState | null
    setLiveRace: (race: LiveRaceState | null) => void
    updateRacerPosition: (racerProfileId: string, updates: Partial<LiveRacer>) => void
    connectionStatus: ConnectionStatus
    setConnectionStatus: (status: ConnectionStatus) => void
    showTrack: boolean
    toggleTrack: () => void
    wsRef: React.MutableRefObject<WebSocket | null>
    sendWsMessage: (msg: Record<string, unknown>) => void
    refreshRaceState: (raceId?: string | null) => Promise<void>
}

const RaceContext = createContext<RaceContextValue | null>(null)

interface RuntimeStateResponse {
    race?: {
        id?: string
        status?: LiveRaceState['status']
        total_laps?: number
    }
    context?: {
        race_name?: string
        racers?: Array<Record<string, unknown>>
    }
}

interface LapRecordResponse {
    racer_profile_id?: string
    lap_number?: number
    lap_time?: number
}

function buildLiveRace(statePayload: RuntimeStateResponse, activeRaceId: string, laps: LapRecordResponse[]): LiveRaceState {
    const race = statePayload.race || {}
    const context = statePayload.context || {}
    const racers = Array.isArray(context.racers) ? context.racers : []
    const lapStats = new Map<string, { currentLap: number, bestLapTime: number | null, lastLapTime: number | null, totalTime: number }>()

    for (const lap of laps) {
        const racerId = String(lap.racer_profile_id || '')
        if (!racerId) continue
        const lapNumber = Number(lap.lap_number || 0)
        const lapTime = Number(lap.lap_time || 0)
        const current = lapStats.get(racerId) || {
            currentLap: 0,
            bestLapTime: null,
            lastLapTime: null,
            totalTime: 0,
        }

        current.currentLap = Math.max(current.currentLap, lapNumber)
        current.lastLapTime = lapTime || current.lastLapTime
        current.totalTime += lapTime
        current.bestLapTime = current.bestLapTime == null
            ? (lapTime || null)
            : lapTime > 0
                ? Math.min(current.bestLapTime, lapTime)
                : current.bestLapTime
        lapStats.set(racerId, current)
    }

    const liveRacers = racers
        .map((racer): LiveRacer => {
            const racerId = String(racer.id || '')
            const stats = lapStats.get(racerId)
            return {
                racer_profile_id: racerId,
                name: String(racer.name || `Racer ${racerId || '?'}`),
                number: String(racer.number || ''),
                color: stringToColor(String(racer.name || racerId || 'racer')),
                current_position: 0,
                current_lap: stats?.currentLap || 0,
                best_lap_time: stats?.bestLapTime ?? null,
                current_lap_time: 0,
                last_lap_time: stats?.lastLapTime ?? null,
                total_time: stats?.totalTime || 0,
                gap_to_leader: 0,
                status: race.status === 'finished' ? 'finished' : 'racing',
                track_position: null,
            }
        })
        .sort((a, b) => {
            if (b.current_lap !== a.current_lap) return b.current_lap - a.current_lap
            return a.total_time - b.total_time
        })
        .map((racer, index, ordered) => ({
            ...racer,
            current_position: index + 1,
            gap_to_leader: index === 0 ? 0 : Math.max(racer.total_time - ordered[0]!.total_time, 0),
        }))

    return {
        race_id: String(race.id || activeRaceId),
        race_name: String(context.race_name || 'Race Master Pro Session'),
        status: race.status || 'setup',
        total_laps: Number(race.total_laps || 0),
        elapsed_time: 0,
        racers: liveRacers,
    }
}

export function RaceProvider({ children }: { children: ReactNode }) {
    const [liveRace, setLiveRace] = useState<LiveRaceState | null>(null)
    const [connectionStatus, setConnectionStatus] = useState<ConnectionStatus>('disconnected')
    const [showTrack, setShowTrack] = useState(true)
    const wsRef = useRef<WebSocket | null>(null)
    const liveRaceRef = useRef<LiveRaceState | null>(null)

    const toggleTrack = useCallback(() => {
        setShowTrack(prev => !prev)
    }, [])

    const updateRacerPosition = useCallback((racerProfileId: string, updates: Partial<LiveRacer>) => {
        setLiveRace(prev => {
            if (!prev) return prev
            return {
                ...prev,
                racers: prev.racers.map(r =>
                    r.racer_profile_id === racerProfileId ? { ...r, ...updates } : r,
                ),
            }
        })
    }, [])

    const sendWsMessage = useCallback((msg: Record<string, unknown>) => {
        if (wsRef.current?.readyState === WebSocket.OPEN) {
            wsRef.current.send(JSON.stringify(msg))
        }
    }, [])

    const refreshRaceState = useCallback(async (raceId?: string | null) => {
        try {
            let activeRaceId = String(raceId || liveRaceRef.current?.race_id || '')

            if (!activeRaceId) {
                const wizardResponse = await apiFetch('/api/setup/wizard')
                if (!wizardResponse.ok) {
                    setLiveRace(null)
                    return
                }
                const wizardPayload = await wizardResponse.json()
                activeRaceId = String(wizardPayload?.race_context?.race_id || '')
            }

            if (!activeRaceId) {
                setLiveRace(null)
                return
            }

            const [stateResponse, lapsResponse] = await Promise.all([
                apiFetch(`/api/races/${activeRaceId}/state`),
                apiFetch(`/api/laps?race_id=${encodeURIComponent(activeRaceId)}`),
            ])

            if (!stateResponse.ok) {
                setLiveRace(null)
                return
            }

            const statePayload = await stateResponse.json() as RuntimeStateResponse
            const lapsPayload = lapsResponse.ok ? await lapsResponse.json() as LapRecordResponse[] : []
            setLiveRace(buildLiveRace(statePayload, activeRaceId, Array.isArray(lapsPayload) ? lapsPayload : []))
        } catch {
            setLiveRace(null)
        }
    }, [])

    useEffect(() => {
        liveRaceRef.current = liveRace
    }, [liveRace])

    useEffect(() => {
        const wsUrl = backendWsUrl('organizer')

        let reconnectTimer: ReturnType<typeof setTimeout>
        let ws: WebSocket

        function connect() {
            setConnectionStatus('connecting')
            ws = new WebSocket(wsUrl)
            wsRef.current = ws

            ws.onopen = () => {
                setConnectionStatus('connected')
            }

            ws.onmessage = event => {
                try {
                    const msg = JSON.parse(event.data)
                    handleWsMessage(msg)
                } catch {
                    // ignore malformed messages
                }
            }

            ws.onclose = () => {
                setConnectionStatus('disconnected')
                wsRef.current = null
                reconnectTimer = setTimeout(connect, 3000)
            }

            ws.onerror = () => {
                // Let browser manage transition to onclose for failed handshakes.
                // Calling close() here can produce noisy "closed before established" warnings in dev.
            }
        }

        function handleWsMessage(msg: { type: string, data: Record<string, unknown> }) {
            switch (msg.type) {
                case 'raceUpdate':
                    if (msg.data.race) {
                        setLiveRace(msg.data.race as LiveRaceState)
                    }
                    break
                case 'positionUpdate':
                    if (msg.data.racer_profile_id && msg.data) {
                        const x = Number(msg.data.position_x)
                        const y = Number(msg.data.position_y)
                        updateRacerPosition(
                            msg.data.racer_profile_id as string,
                            {
                                ...(msg.data as Partial<LiveRacer>),
                                track_position: Number.isFinite(x) && Number.isFinite(y)
                                    ? { x, y }
                                    : (msg.data as Partial<LiveRacer>).track_position ?? null,
                            },
                        )
                    }
                    break
                case 'visionDetection':
                    if (msg.data.racer_profile_id) {
                        const x = Number(msg.data.position_x)
                        const y = Number(msg.data.position_y)
                        if (Number.isFinite(x) && Number.isFinite(y)) {
                            updateRacerPosition(
                                String(msg.data.racer_profile_id),
                                { track_position: { x, y } },
                            )
                        }
                    }
                    break
                case 'raceStart':
                case 'racePause':
                case 'raceResume':
                case 'raceFinish':
                case 'lapComplete':
                    void refreshRaceState(String(msg.data?.race_id || liveRaceRef.current?.race_id || ''))
                    break
                case 'pong':
                    break
            }
        }

        void refreshRaceState()
        connect()

        const pingInterval = setInterval(() => {
            if (wsRef.current?.readyState === WebSocket.OPEN) {
                wsRef.current.send(JSON.stringify({ type: 'ping' }))
            }
        }, 30000)

        return () => {
            clearTimeout(reconnectTimer)
            clearInterval(pingInterval)
            if (ws?.readyState === WebSocket.OPEN) {
                ws.close()
            }
        }
    }, [refreshRaceState, updateRacerPosition])

    return (
        <RaceContext.Provider value={{
            liveRace,
            setLiveRace,
            updateRacerPosition,
            connectionStatus,
            setConnectionStatus,
            showTrack,
            toggleTrack,
            wsRef,
            sendWsMessage,
            refreshRaceState,
        }}>
            {children}
        </RaceContext.Provider>
    )
}

export function useRaceContext(): RaceContextValue {
    const ctx = useContext(RaceContext)
    if (!ctx) throw new Error('useRaceContext must be used within <RaceProvider>')
    return ctx
}
