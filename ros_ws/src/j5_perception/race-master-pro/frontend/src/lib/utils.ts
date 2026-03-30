import { clsx, type ClassValue } from 'clsx'
import { twMerge } from 'tailwind-merge'
import type { Checkpoint, Track, TrackPoint } from '@/types'

const TRACK_SELECTION_KEY = 'race-master-pro.selectedTrackId'
const TRACK_PHOTO_KEY_PREFIX = 'race-master-pro.trackPhoto.'
const LATEST_SNAPSHOT_KEY = 'race-master-pro.latestSnapshotUrl'
export const TRACK_OVERLAY_EVENT = 'j5-track-overlay-updated'

const TRACK_CHECKPOINTS_KEY_PREFIX = 'race-master-pro.trackCheckpoints.'

function sanitizeStoredUrl(url: string): string {
    const value = (url || '').trim()
    if (!value || value.startsWith('blob:')) return ''
    if (value.endsWith('/stream.mjpg')) return `${value.replace(/\/stream\.mjpg$/, '')}/snapshot.jpg`
    try {
        const parsed = new URL(value)
        if (parsed.pathname === '/' || parsed.pathname === '') {
            return `${parsed.origin}/snapshot.jpg`
        }
    } catch {
        // keep raw value when URL parsing fails
    }
    return value
}

export function cn(...inputs: ClassValue[]) {
    return twMerge(clsx(inputs))
}

/**
 * Format milliseconds into a readable lap time string.
 * e.g. 83450 → "1:23.450"
 */
export function formatLapTime(ms: number | null | undefined): string {
    if (ms == null || ms <= 0) return '--:--.---'
    const totalSeconds = ms / 1000
    const minutes = Math.floor(totalSeconds / 60)
    const seconds = totalSeconds % 60
    return `${minutes}:${seconds.toFixed(3).padStart(6, '0')}`
}

/**
 * Format milliseconds into MM:SS.mmm race elapsed time.
 */
export function formatRaceTime(ms: number): string {
    if (ms <= 0) return '0:00.000'
    const totalSeconds = ms / 1000
    const minutes = Math.floor(totalSeconds / 60)
    const seconds = totalSeconds % 60
    return `${minutes}:${seconds.toFixed(3).padStart(6, '0')}`
}

/**
 * Format a gap to leader in seconds.
 * e.g. 1.234 → "+1.234s", 0 → "Leader"
 */
export function formatGap(gapMs: number): string {
    if (gapMs <= 0) return 'Leader'
    return `+${(gapMs / 1000).toFixed(3)}s`
}

/**
 * Get CSS class for a position badge.
 */
export function getPositionClass(position: number): string {
    switch (position) {
        case 1: return 'bg-position-1'
        case 2: return 'bg-position-2'
        case 3: return 'bg-position-3'
        default: return 'bg-[var(--color-bg-surface)] text-[var(--color-text-secondary)]'
    }
}

/**
 * Generate a deterministic color from a string (racer name/id).
 */
export function stringToColor(str: string): string {
    const colors = [
        '#e63946', '#4a7ff7', '#2ecc71', '#f4d03f',
        '#9b59b6', '#e67e22', '#1abc9c', '#e74c3c',
        '#3498db', '#f39c12', '#8e44ad', '#2980b9',
    ]
    let hash = 0
    for (let i = 0; i < str.length; i++) {
        hash = str.charCodeAt(i) + ((hash << 5) - hash)
    }
    return colors[Math.abs(hash) % colors.length]!
}

/**
 * Generate a UUID v4.
 */
export function generateId(): string {
    if (typeof crypto !== 'undefined' && typeof crypto.randomUUID === 'function') {
        return crypto.randomUUID()
    }
    return `id-${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 10)}`
}

export function configuredApiBaseUrl(): string | null {
    const configured = import.meta.env.VITE_API_BASE_URL?.trim()
    return configured ? configured.replace(/\/$/, '') : null
}

export function backendBaseUrl(): string {
    const configured = configuredApiBaseUrl()
    if (configured) {
        return configured
    }
    if (typeof window === 'undefined') {
        return 'http://localhost:8080'
    }
    return `http://${window.location.hostname}:8080`
}

function sameOriginWsBaseUrl(): string {
    return `${window.location.protocol === 'https:' ? 'wss:' : 'ws:'}//${window.location.host}/ws`
}

function directBackendWsBaseUrl(): string {
    return `${window.location.protocol === 'https:' ? 'wss:' : 'ws:'}//${window.location.hostname}:8080/ws`
}

function shouldUseSameOriginWsProxy(): boolean {
    return window.location.protocol === 'https:' || !['5173', '4173'].includes(window.location.port)
}

export function backendWsUrl(clientType = 'organizer'): string {
    const configured = import.meta.env.VITE_WS_URL?.trim()
    const baseUrl = configured
        ? configured.replace(/\/$/, '')
        : shouldUseSameOriginWsProxy()
            ? sameOriginWsBaseUrl()
            : directBackendWsBaseUrl()
    const separator = baseUrl.includes('?') ? '&' : '?'
    return `${baseUrl}${separator}client_type=${encodeURIComponent(clientType)}`
}

export async function apiFetch(path: string, init?: RequestInit): Promise<Response> {
    const normalizedPath = path.startsWith('/') ? path : `/${path}`
    const configuredBaseUrl = configuredApiBaseUrl()

    if (configuredBaseUrl) {
        return fetch(`${configuredBaseUrl}${normalizedPath}`, init)
    }

    const response = await fetch(normalizedPath, init)
    if (response.status !== 404) {
        return response
    }
    return fetch(`${backendBaseUrl()}${normalizedPath}`, init)
}

export function parseTrackPointList(raw: unknown): TrackPoint[] {
    if (Array.isArray(raw)) {
        return raw
            .map((point) => ({
                x: Number((point as TrackPoint)?.x),
                y: Number((point as TrackPoint)?.y),
            }))
            .filter((point) => Number.isFinite(point.x) && Number.isFinite(point.y))
    }
    if (typeof raw !== 'string') return []
    try {
        return parseTrackPointList(JSON.parse(raw))
    } catch {
        return []
    }
}

export function parseTrackRecord(raw: Record<string, unknown>): Track {
    return {
        id: String(raw.id || ''),
        name: String(raw.name || 'Track'),
        scale: String(raw.scale || '1:24'),
        track_distance: raw.track_distance == null ? null : Number(raw.track_distance),
        layout_points: parseTrackPointList(raw.layout_points),
        boundary_polygon: parseTrackPointList(raw.boundary_polygon),
        created_at: raw.created_at ? String(raw.created_at) : undefined,
    }
}

function getStorage(): Storage | null {
    if (typeof window === 'undefined') return null
    return window.localStorage
}

export function setSelectedTrackId(trackId: string): void {
    const storage = getStorage()
    if (!storage) return
    storage.setItem(TRACK_SELECTION_KEY, trackId)
    window.dispatchEvent(new CustomEvent(TRACK_OVERLAY_EVENT))
}

export function getSelectedTrackId(): string {
    const storage = getStorage()
    return storage?.getItem(TRACK_SELECTION_KEY) || ''
}

export function setTrackPhotoUrl(trackId: string, url: string): void {
    const storage = getStorage()
    if (!storage || !trackId) return
    const cleanUrl = sanitizeStoredUrl(url)
    if (!cleanUrl) return
    storage.setItem(`${TRACK_PHOTO_KEY_PREFIX}${trackId}`, cleanUrl)
    storage.setItem(LATEST_SNAPSHOT_KEY, cleanUrl)
    window.dispatchEvent(new CustomEvent(TRACK_OVERLAY_EVENT))
}

export function getTrackPhotoUrl(trackId: string): string {
    const storage = getStorage()
    if (!storage || !trackId) return ''
    return sanitizeStoredUrl(storage.getItem(`${TRACK_PHOTO_KEY_PREFIX}${trackId}`) || '')
}

export function setLatestSnapshotUrl(url: string): void {
    const storage = getStorage()
    if (!storage) return
    const cleanUrl = sanitizeStoredUrl(url)
    if (!cleanUrl) return
    storage.setItem(LATEST_SNAPSHOT_KEY, cleanUrl)
    window.dispatchEvent(new CustomEvent(TRACK_OVERLAY_EVENT))
}

export function getLatestSnapshotUrl(): string {
    const storage = getStorage()
    return sanitizeStoredUrl(storage?.getItem(LATEST_SNAPSHOT_KEY) || '')
}

export function setTrackCheckpoints(trackId: string, checkpoints: Checkpoint[]): void {
    const storage = getStorage()
    if (!storage || !trackId) return
    storage.setItem(`${TRACK_CHECKPOINTS_KEY_PREFIX}${trackId}`, JSON.stringify(checkpoints))
    window.dispatchEvent(new CustomEvent(TRACK_OVERLAY_EVENT))
}

export function getTrackCheckpoints(trackId: string): Checkpoint[] {
    const storage = getStorage()
    if (!storage || !trackId) return []
    const raw = storage.getItem(`${TRACK_CHECKPOINTS_KEY_PREFIX}${trackId}`)
    if (!raw) return []
    try {
        const parsed = JSON.parse(raw)
        if (!Array.isArray(parsed)) return []
        return parsed
            .filter(item => item && typeof item === 'object')
            .map(item => ({
                id: String((item as Checkpoint).id || crypto.randomUUID()),
                track_id: String((item as Checkpoint).track_id || trackId),
                name: String((item as Checkpoint).name || 'Checkpoint'),
                type: ((item as Checkpoint).type || 'checkpoint') as Checkpoint['type'],
                position: {
                    x: Number((item as Checkpoint).position?.x || 0),
                    y: Number((item as Checkpoint).position?.y || 0),
                },
                sort_order: Number((item as Checkpoint).sort_order || 0),
            }))
    } catch {
        return []
    }
}
