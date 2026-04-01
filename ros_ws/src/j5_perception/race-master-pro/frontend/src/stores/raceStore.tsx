import { createContext, useContext, useState, useCallback, useRef, useEffect, type ReactNode } from 'react'
import type { Checkpoint, LiveRaceState, LiveRacer, TrackPoint } from '@/types'
import { apiFetch, backendWsUrl, getSelectedTrackId, getTrackCheckpoints, getTrackRacerAssignments, stringToColor } from '@/lib/utils'

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
    recentObjectDetections: Array<{ objectId: string, racerProfileId: string, seenAt: number }>
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

interface RacerCheckpointState {
    touchedCheckpointIds: Set<string>
    nextCheckpointIndex: number
    insideMarkers: Set<string>
    lastFinishAtMs: number
}

const CHECKPOINT_CAPTURE_RADIUS = 0.035
const FINISH_COOLDOWN_MS = 2000

function pointDistance(a: TrackPoint, b: TrackPoint): number {
    return Math.hypot(a.x - b.x, a.y - b.y)
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
                tracked_object_id: null,
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
    const checkpointStateRef = useRef<Map<string, RacerCheckpointState>>(new Map())
    const [recentObjectDetections, setRecentObjectDetections] = useState<Array<{ objectId: string, racerProfileId: string, seenAt: number }>>([])
    const checkpointsRef = useRef<Checkpoint[]>([])
    const requiredCheckpointIdsRef = useRef<Set<string>>(new Set())

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

    const evaluateCheckpointProgress = useCallback(async (racerProfileId: string, position: TrackPoint) => {
        const race = liveRaceRef.current
        if (!race) return
        const markers = checkpointsRef.current
        if (markers.length === 0) return

        const racer = race.racers.find(item => item.racer_profile_id === racerProfileId)
        if (!racer) return

        const state = checkpointStateRef.current.get(racerProfileId) || {
            touchedCheckpointIds: new Set<string>(),
            nextCheckpointIndex: 0,
            insideMarkers: new Set<string>(),
            lastFinishAtMs: 0,
        }
        const orderedCheckpoints = markers
            .filter(marker => marker.type === 'checkpoint')
            .sort((a, b) => a.sort_order - b.sort_order)

        for (const marker of markers) {
            const isInside = pointDistance(position, marker.position) <= CHECKPOINT_CAPTURE_RADIUS
            const wasInside = state.insideMarkers.has(marker.id)

            if (isInside && !wasInside) {
                state.insideMarkers.add(marker.id)
                if (marker.type === 'checkpoint') {
                    const expected = orderedCheckpoints[state.nextCheckpointIndex]
                    if (expected?.id === marker.id) {
                        state.touchedCheckpointIds.add(marker.id)
                        state.nextCheckpointIndex += 1
                    } else if (orderedCheckpoints[0]?.id === marker.id) {
                        state.touchedCheckpointIds.clear()
                        state.touchedCheckpointIds.add(marker.id)
                        state.nextCheckpointIndex = 1
                    }
                } else if (marker.type === 'finish') {
                    const nowMs = Date.now()
                    if (nowMs - state.lastFinishAtMs > FINISH_COOLDOWN_MS) {
                        const required = requiredCheckpointIdsRef.current
                        const touchedAll = orderedCheckpoints.length === 0
                            || (state.nextCheckpointIndex >= orderedCheckpoints.length
                                && [...required].every(id => state.touchedCheckpointIds.has(id)))
                        if (touchedAll) {
                            const lapNumber = racer.current_lap + 1
                            await apiFetch('/api/laps', {
                                method: 'POST',
                                headers: { 'Content-Type': 'application/json' },
                                body: JSON.stringify({
                                    race_id: race.race_id,
                                    racer_profile_id: racerProfileId,
                                    lap_number: lapNumber,
                                    lap_time: Math.max(racer.current_lap_time, 1),
                                }),
                            })
                            await apiFetch(`/api/races/${race.race_id}/logs`, {
                                method: 'POST',
                                headers: { 'Content-Type': 'application/json' },
                                body: JSON.stringify({ entry: `${new Date().toISOString()} ${racer.name} completed lap ${lapNumber}` }),
                            })
                        } else {
                            await apiFetch(`/api/races/${race.race_id}/logs`, {
                                method: 'POST',
                                headers: { 'Content-Type': 'application/json' },
                                body: JSON.stringify({ entry: `${new Date().toISOString()} ${racer.name} hit finish without all checkpoints (shortcut detected)` }),
                            })
                        }
                        state.touchedCheckpointIds.clear()
                        state.nextCheckpointIndex = 0
                        state.lastFinishAtMs = nowMs
                        void refreshRaceState(race.race_id)
                    }
                }
            } else if (!isInside && wasInside) {
                state.insideMarkers.delete(marker.id)
            }
        }

        checkpointStateRef.current.set(racerProfileId, state)
    }, [refreshRaceState])

    useEffect(() => {
        liveRaceRef.current = liveRace
    }, [liveRace])

    useEffect(() => {
        const hydrateTrackOverlayState = () => {
            const selectedTrackId = getSelectedTrackId()
            const markers = selectedTrackId ? getTrackCheckpoints(selectedTrackId) : []
            checkpointsRef.current = markers
            requiredCheckpointIdsRef.current = new Set(
                markers.filter(marker => marker.type === 'checkpoint').map(marker => marker.id),
            )
        }
        hydrateTrackOverlayState()
        window.addEventListener('j5-track-overlay-updated', hydrateTrackOverlayState)
        return () => window.removeEventListener('j5-track-overlay-updated', hydrateTrackOverlayState)
    }, [])

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
                case 'positionUpdate': {
                    if (msg.data.racer_profile_id && msg.data) {
                        const positionX = Number(msg.data.position_x)
                        const positionY = Number(msg.data.position_y)
                        const nextPosition = Number.isFinite(positionX) && Number.isFinite(positionY)
                            ? { x: positionX, y: positionY }
                            : (msg.data as Partial<LiveRacer>).track_position ?? null
                        updateRacerPosition(
                            msg.data.racer_profile_id as string,
                            {
                                ...(msg.data as Partial<LiveRacer>),
                                track_position: nextPosition,
                            },
                        )
                        if (nextPosition) {
                            void evaluateCheckpointProgress(String(msg.data.racer_profile_id), nextPosition)
                        }
                    }
                    break
                }
                case 'visionDetection':
                    {
                        const selectedTrackId = getSelectedTrackId()
                        const assignments = selectedTrackId ? getTrackRacerAssignments(selectedTrackId) : {}
                        const incomingObjectId = String(
                            msg.data.object_id
                            || msg.data.tracker_id
                            || msg.data.detection_id
                            || msg.data.track_id
                            || '',
                        ).trim()
                        const x = Number(msg.data.position_x)
                        const y = Number(msg.data.position_y)
                        const position = Number.isFinite(x) && Number.isFinite(y) ? { x, y } : null
                        if (incomingObjectId) {
                            setRecentVisionObjects(prev => {
                                const next = [
                                    { objectId: incomingObjectId, seenAt: Date.now(), position },
                                    ...prev.filter(item => item.objectId !== incomingObjectId),
                                ]
                                return next.slice(0, 24)
                            })
                        }
                        let racerId = String(msg.data.racer_profile_id || '').trim()
                        if (incomingObjectId) {
                            const match = Object.entries(assignments).find(([, objectId]) => objectId === incomingObjectId)
                            racerId = match?.[0] || ''
                        }
                        if (!racerId) break
                        if (Object.keys(assignments).length > 0 && !assignments[racerId]) {
                            break
                        }
                        if (incomingObjectId) {
                            setRecentObjectDetections(prev => {
                                const next = [
                                    { objectId: incomingObjectId, racerProfileId: racerId, seenAt: Date.now() },
                                    ...prev.filter(item => !(item.objectId === incomingObjectId && item.racerProfileId === racerId)),
                                ]
                                return next.slice(0, 12)
                            })
                        }
                        const x = Number(msg.data.position_x)
                        const y = Number(msg.data.position_y)
                        if (Number.isFinite(x) && Number.isFinite(y)) {
                            const position = { x, y }
                            updateRacerPosition(racerId, { track_position: position, tracked_object_id: incomingObjectId || null })
                            void evaluateCheckpointProgress(racerId, position)
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
    }, [evaluateCheckpointProgress, refreshRaceState, updateRacerPosition])

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
            recentObjectDetections,
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
