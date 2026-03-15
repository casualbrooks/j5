import { clsx, type ClassValue } from 'clsx'
import { twMerge } from 'tailwind-merge'

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
    return crypto.randomUUID()
}

function backendBaseUrl(): string {
    if (typeof window === 'undefined') {
        return 'http://localhost:8080'
    }
    return `http://${window.location.hostname}:8080`
}

export async function apiFetch(path: string, init?: RequestInit): Promise<Response> {
    const normalizedPath = path.startsWith('/') ? path : `/${path}`
    const response = await fetch(normalizedPath, init)
    if (response.status !== 404) {
        return response
    }
    return fetch(`${backendBaseUrl()}${normalizedPath}`, init)
}
