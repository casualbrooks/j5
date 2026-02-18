import { createContext, useContext, useState, useCallback, useRef, useEffect, type ReactNode } from 'react'
import type { LiveRaceState, LiveRacer, RaceStatus } from '@/types'

// ── Connection Status ──────────────────────────────────────

export type ConnectionStatus = 'connected' | 'connecting' | 'disconnected'

// ── Context Shape ──────────────────────────────────────────

interface RaceContextValue {
    // Live race state
    liveRace: LiveRaceState | null
    setLiveRace: (race: LiveRaceState | null) => void
    updateRacerPosition: (racerProfileId: string, updates: Partial<LiveRacer>) => void

    // Connection
    connectionStatus: ConnectionStatus
    setConnectionStatus: (status: ConnectionStatus) => void

    // Track visibility
    showTrack: boolean
    toggleTrack: () => void

    // WebSocket
    wsRef: React.MutableRefObject<WebSocket | null>
    sendWsMessage: (msg: Record<string, unknown>) => void
}

const RaceContext = createContext<RaceContextValue | null>(null)

// ── Provider ───────────────────────────────────────────────

export function RaceProvider({ children }: { children: ReactNode }) {
    const [liveRace, setLiveRace] = useState<LiveRaceState | null>(null)
    const [connectionStatus, setConnectionStatus] = useState<ConnectionStatus>('disconnected')
    const [showTrack, setShowTrack] = useState(true)
    const wsRef = useRef<WebSocket | null>(null)

    const toggleTrack = useCallback(() => {
        setShowTrack(prev => !prev)
    }, [])

    const updateRacerPosition = useCallback((racerProfileId: string, updates: Partial<LiveRacer>) => {
        setLiveRace(prev => {
            if (!prev) return prev
            return {
                ...prev,
                racers: prev.racers.map(r =>
                    r.racer_profile_id === racerProfileId ? { ...r, ...updates } : r
                ),
            }
        })
    }, [])

    const sendWsMessage = useCallback((msg: Record<string, unknown>) => {
        if (wsRef.current?.readyState === WebSocket.OPEN) {
            wsRef.current.send(JSON.stringify(msg))
        }
    }, [])

    // Auto-connect WebSocket on mount
    useEffect(() => {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
        const wsUrl = `${protocol}//${window.location.host}/ws?client_type=organizer`

        let reconnectTimer: ReturnType<typeof setTimeout>
        let ws: WebSocket

        function connect() {
            setConnectionStatus('connecting')
            ws = new WebSocket(wsUrl)
            wsRef.current = ws

            ws.onopen = () => {
                setConnectionStatus('connected')
            }

            ws.onmessage = (event) => {
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
                // Reconnect after 3 seconds
                reconnectTimer = setTimeout(connect, 3000)
            }

            ws.onerror = () => {
                ws.close()
            }
        }

        function handleWsMessage(msg: { type: string; data: Record<string, unknown> }) {
            switch (msg.type) {
                case 'raceUpdate':
                    if (msg.data.race) {
                        setLiveRace(msg.data.race as LiveRaceState)
                    }
                    break
                case 'positionUpdate':
                    if (msg.data.racer_profile_id && msg.data) {
                        updateRacerPosition(
                            msg.data.racer_profile_id as string,
                            msg.data as Partial<LiveRacer>,
                        )
                    }
                    break
                case 'pong':
                    // heartbeat received
                    break
            }
        }

        connect()

        // Heartbeat ping every 30s
        const pingInterval = setInterval(() => {
            if (wsRef.current?.readyState === WebSocket.OPEN) {
                wsRef.current.send(JSON.stringify({ type: 'ping' }))
            }
        }, 30000)

        return () => {
            clearTimeout(reconnectTimer)
            clearInterval(pingInterval)
            ws?.close()
        }
    }, [updateRacerPosition])

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
        }}>
            {children}
        </RaceContext.Provider>
    )
}

// ── Hook ───────────────────────────────────────────────────

export function useRaceContext(): RaceContextValue {
    const ctx = useContext(RaceContext)
    if (!ctx) throw new Error('useRaceContext must be used within <RaceProvider>')
    return ctx
}
