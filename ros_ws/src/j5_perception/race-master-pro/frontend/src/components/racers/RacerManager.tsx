import { FormEvent, useEffect, useMemo, useState } from 'react'
import { apiFetch, stringToColor } from '@/lib/utils'
import { useRaceContext } from '@/stores/raceStore'
import type { RacerProfile } from '@/types'

interface RacerSummary {
    total_races?: number
    avg_position?: number | null
    all_time_best_lap?: number | null
    career_avg_lap?: number | null
    total_points?: number | null
    dnf_count?: number | null
    wins?: number | null
    podiums?: number | null
}

const emptyForm = {
    id: '',
    name: '',
    number: '',
    vehicle_description: '',
}

export default function RacerManager() {
    const { liveRace } = useRaceContext()
    const [racers, setRacers] = useState<RacerProfile[]>([])
    const [summaries, setSummaries] = useState<Record<string, RacerSummary>>({})
    const [form, setForm] = useState(emptyForm)
    const [status, setStatus] = useState('')
    const [loading, setLoading] = useState(true)

    const activeRacerIds = useMemo(
        () => new Set((liveRace?.racers || []).map(racer => racer.racer_profile_id)),
        [liveRace?.racers],
    )

    const loadRacers = async () => {
        setLoading(true)
        try {
            const response = await apiFetch('/api/racers')
            if (!response.ok) throw new Error('Failed to load racers')
            const payload = await response.json() as RacerProfile[]
            setRacers(payload)
            const summaryPairs = await Promise.all(
                payload.map(async racer => {
                    const summaryResponse = await apiFetch(`/api/analytics/racer/${racer.id}/summary`)
                    const summary = summaryResponse.ok ? await summaryResponse.json() as RacerSummary : {}
                    return [racer.id, summary] as const
                }),
            )
            setSummaries(Object.fromEntries(summaryPairs))
            setStatus('')
        } catch (error) {
            setStatus(error instanceof Error ? error.message : 'Unable to load racers.')
        } finally {
            setLoading(false)
        }
    }

    useEffect(() => {
        void loadRacers()
    }, [])

    const submitRacer = async (event: FormEvent) => {
        event.preventDefault()
        const normalizedName = form.name.trim().toLowerCase()
        const normalizedNumber = form.number.trim()
        const duplicate = racers.find(existing => {
            if (form.id && existing.id === form.id) return false
            const sameName = existing.name.trim().toLowerCase() === normalizedName
            const sameNumber = normalizedNumber && existing.number.trim() === normalizedNumber
            return sameName || sameNumber
        })
        if (duplicate) {
            setStatus(`Racer already exists (${duplicate.name} #${duplicate.number || '—'}). Edit the existing profile instead of creating a duplicate.`)
            return
        }
        const payload = {
            name: form.name,
            number: form.number,
            vehicle_description: form.vehicle_description || null,
            profile_pic: null,
        }
        try {
            const response = await apiFetch(form.id ? `/api/racers/${form.id}` : '/api/racers', {
                method: form.id ? 'PUT' : 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(payload),
            })
            if (!response.ok) throw new Error(`Failed to ${form.id ? 'update' : 'create'} racer`)
            setForm(emptyForm)
            setStatus(form.id ? 'Racer updated.' : 'Racer created.')
            await loadRacers()
        } catch (error) {
            setStatus(error instanceof Error ? error.message : 'Unable to save racer.')
        }
    }

    const editRacer = (racer: RacerProfile) => {
        setForm({
            id: racer.id,
            name: racer.name,
            number: racer.number,
            vehicle_description: racer.vehicle_description || '',
        })
    }

    const deleteRacer = async (racerId: string) => {
        try {
            const response = await apiFetch(`/api/racers/${racerId}`, { method: 'DELETE' })
            if (!response.ok) throw new Error('Failed to delete racer')
            if (form.id === racerId) setForm(emptyForm)
            setStatus('Racer deleted.')
            await loadRacers()
        } catch (error) {
            setStatus(error instanceof Error ? error.message : 'Unable to delete racer.')
        }
    }

    return (
        <div className="fade-in space-y-4">
            <div className="flex flex-wrap items-center justify-between gap-3">
                <div>
                    <h2 className="text-xl font-semibold text-[var(--color-text-primary)]">Racers</h2>
                    <p className="text-sm text-[var(--color-text-secondary)]">
                        Create racer profiles, edit car/racer metadata, and compare career summaries alongside the current live race roster.
                    </p>
                </div>
                <button className="rounded bg-slate-700 px-4 py-2 text-sm font-semibold text-white hover:bg-slate-600" onClick={() => void loadRacers()}>
                    Refresh
                </button>
            </div>

            <div className="grid gap-4 xl:grid-cols-[340px_1fr]">
                <form className="race-card space-y-3" onSubmit={submitRacer}>
                    <div>
                        <h3 className="text-base font-semibold">{form.id ? 'Edit racer' : 'Create racer'}</h3>
                        <p className="text-xs text-[var(--color-text-secondary)]">
                            Use this tab to keep racer identities, numbers, and vehicle descriptions aligned with the live race setup.
                        </p>
                    </div>
                    <input className="w-full rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm" value={form.name} onChange={event => setForm(prev => ({ ...prev, name: event.target.value }))} placeholder="Racer name" required />
                    <input className="w-full rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm" value={form.number} onChange={event => setForm(prev => ({ ...prev, number: event.target.value }))} placeholder="Racer number" />
                    <textarea className="min-h-[110px] w-full rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm" value={form.vehicle_description} onChange={event => setForm(prev => ({ ...prev, vehicle_description: event.target.value }))} placeholder="Vehicle / car notes" />
                    <div className="flex flex-wrap gap-2">
                        <button className="rounded bg-blue-600 px-4 py-2 text-sm font-semibold text-white hover:bg-blue-500" type="submit">
                            {form.id ? 'Save Racer' : 'Add Racer'}
                        </button>
                        <button className="rounded bg-slate-700 px-4 py-2 text-sm font-semibold text-white hover:bg-slate-600" type="button" onClick={() => setForm(emptyForm)}>
                            Reset
                        </button>
                    </div>
                    {status ? <p className="text-xs text-[var(--color-text-secondary)]">{status}</p> : null}
                </form>

                <div className="space-y-4">
                    <div className="grid gap-4 md:grid-cols-2 xl:grid-cols-3">
                        {racers.map(racer => {
                            const summary = summaries[racer.id] || {}
                            const isLive = activeRacerIds.has(racer.id)
                            return (
                                <article key={racer.id} className="race-card space-y-3">
                                    <div className="flex items-start justify-between gap-3">
                                        <div className="flex items-center gap-3">
                                            <span
                                                className="inline-flex h-9 w-9 items-center justify-center rounded-full text-sm font-bold text-white"
                                                style={{ backgroundColor: stringToColor(racer.name) }}
                                            >
                                                {racer.number || racer.name.slice(0, 1).toUpperCase()}
                                            </span>
                                            <div>
                                                <h3 className="font-semibold text-[var(--color-text-primary)]">{racer.name}</h3>
                                                <p className="text-xs text-[var(--color-text-secondary)]">
                                                    #{racer.number || '—'} {isLive ? '· Live now' : '· Not on track'}
                                                </p>
                                            </div>
                                        </div>
                                        <div className="flex gap-2">
                                            <button className="rounded bg-slate-700 px-2 py-1 text-xs font-semibold text-white hover:bg-slate-600" onClick={() => editRacer(racer)}>Edit</button>
                                            <button className="rounded bg-rose-700 px-2 py-1 text-xs font-semibold text-white hover:bg-rose-600" onClick={() => void deleteRacer(racer.id)}>Delete</button>
                                        </div>
                                    </div>
                                    <p className="text-sm text-[var(--color-text-secondary)] min-h-[40px]">
                                        {racer.vehicle_description || 'No vehicle description saved yet.'}
                                    </p>
                                    <dl className="grid grid-cols-2 gap-2 text-xs">
                                        <div className="rounded bg-slate-900/70 p-2"><dt className="text-[var(--color-text-muted)]">Points</dt><dd className="font-semibold">{summary.total_points ?? 0}</dd></div>
                                        <div className="rounded bg-slate-900/70 p-2"><dt className="text-[var(--color-text-muted)]">Wins</dt><dd className="font-semibold">{summary.wins ?? 0}</dd></div>
                                        <div className="rounded bg-slate-900/70 p-2"><dt className="text-[var(--color-text-muted)]">Podiums</dt><dd className="font-semibold">{summary.podiums ?? 0}</dd></div>
                                        <div className="rounded bg-slate-900/70 p-2"><dt className="text-[var(--color-text-muted)]">Best lap</dt><dd className="font-semibold">{summary.all_time_best_lap ? `${summary.all_time_best_lap.toFixed(3)}s` : '—'}</dd></div>
                                    </dl>
                                </article>
                            )
                        })}
                    </div>
                    {!loading && racers.length === 0 ? (
                        <div className="race-card p-8 text-center text-[var(--color-text-secondary)]">
                            No racers found yet. Add the first racer using the form on the left.
                        </div>
                    ) : null}
                </div>
            </div>
        </div>
    )
}
