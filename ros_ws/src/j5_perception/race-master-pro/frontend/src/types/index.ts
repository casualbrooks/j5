// ============================================================
// Race Master Pro — Shared TypeScript Interfaces
// All domain types in one place. No duplication across files.
// ============================================================

// ── Season & Championship ──────────────────────────────────

export interface Season {
    id: string
    name: string
    year: number
    status: 'active' | 'completed' | 'upcoming'
    created_at?: string
}

export interface Championship {
    id: string
    season_id: string
    name: string
    points_system: Record<number, number>
    status: 'active' | 'completed' | 'upcoming'
    created_at?: string
}

// ── Track & Checkpoints ────────────────────────────────────

export interface TrackPoint {
    x: number
    y: number
}

export interface Checkpoint {
    id: string
    track_id: string
    name: string
    type: 'start' | 'finish' | 'checkpoint' | 'sector'
    position: TrackPoint
    sort_order: number
}

export interface Track {
    id: string
    name: string
    scale: string
    track_distance: number | null
    layout_points: TrackPoint[]
    boundary_polygon: TrackPoint[]
    created_at?: string
}

// ── Camera ─────────────────────────────────────────────────

export interface CameraConfig {
    id: string
    track_id: string
    name: string
    source: string
    ros_topic: string | null
    calibration: Record<string, unknown> | null
    status: 'connected' | 'disconnected' | 'calibrating' | 'error'
}

// ── Racer Profile ──────────────────────────────────────────

export interface RacerProfile {
    id: string
    name: string
    number: string
    profile_pic: string | null
    vehicle_description: string | null
    created_at?: string
}

export interface BehavioralMetrics {
    racer_profile_id: string
    season_id: string
    consistency_score: number
    avg_lap_variance: number
    improvement_rate: number
    total_races: number
    dnf_count: number
    avg_finish_position: number
    updated_at?: string
}

// ── Race Event, Race, Results ──────────────────────────────

export interface RaceEvent {
    id: string
    championship_id: string
    track_id: string
    name: string
    event_date: string | null
    round_number: number
}

export type RaceType = 'practice' | 'qualifying' | 'main'
export type RaceStatus = 'setup' | 'active' | 'paused' | 'finished' | 'cancelled'
export type RacerStatus = 'racing' | 'finished' | 'dnf'

export interface Race {
    id: string
    event_id: string
    type: RaceType
    status: RaceStatus
    total_laps: number
    start_time: string | null
    end_time: string | null
}

export interface RaceResult {
    id: string
    race_id: string
    racer_profile_id: string
    finish_position: number
    total_time: number
    best_lap_time: number | null
    avg_lap_time: number | null
    laps_completed: number
    status: RacerStatus
    points_earned: number
}

export interface LapRecord {
    id: string
    race_id: string
    racer_profile_id: string
    lap_number: number
    lap_time: number
    sector1_time: number | null
    sector2_time: number | null
    sector3_time: number | null
    recorded_at?: string
}

// ── Vision / Detection ─────────────────────────────────────

export interface Detection {
    id: string
    race_id: string
    camera_id: string
    racer_profile_id: string
    position_x: number
    position_y: number
    confidence: number
    recorded_at?: string
}

// ── Live Race State (in-memory, pushed via WebSocket) ──────

export interface LiveRacer {
    racer_profile_id: string
    name: string
    number: string
    color: string
    current_position: number
    current_lap: number
    best_lap_time: number | null
    current_lap_time: number
    last_lap_time: number | null
    total_time: number
    gap_to_leader: number
    status: RacerStatus
    track_position: TrackPoint | null
    tracked_object_id?: string | null
}

export interface LiveRaceState {
    race_id: string
    race_name: string
    status: RaceStatus
    total_laps: number
    elapsed_time: number
    racers: LiveRacer[]
}

// ── WebSocket Messages ─────────────────────────────────────

export type WSMessageType =
    | 'raceUpdate'
    | 'positionUpdate'
    | 'lapComplete'
    | 'raceStart'
    | 'racePause'
    | 'raceFinish'
    | 'raceResume'
    | 'lapLog'
    | 'racerAdded'
    | 'visionDetection'
    | 'connectionCount'
    | 'ping'
    | 'pong'

export interface WSMessage {
    type: WSMessageType
    data: Record<string, unknown>
    timestamp?: string
}

// ── App Navigation ─────────────────────────────────────────

export type ViewTab =
    | 'dashboard'
    | 'analytics'
    | 'championship'
    | 'racers'
    | 'vision'
    | 'integration'
    | 'settings'

// ── Config ─────────────────────────────────────────────────

export interface AppConfig {
    backend_url: string
    ws_url: string
    ai_model: string
    confidence_threshold: number
    camera_topics: string[]
}
