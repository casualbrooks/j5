import { FormEvent, useEffect, useMemo, useState } from 'react'
import { apiFetch } from '@/lib/utils'
import type { Championship, Season } from '@/types'

interface StandingRow {
    racer_id: string
    racer_name: string
    racer_number: string
    total_points: number
    wins: number
    podiums: number
    races_completed: number
}

const seasonDefaults = { name: '', year: new Date().getFullYear(), status: 'active' as Season['status'] }
const championshipDefaults = { season_id: '', name: '', points_system: '{"1":25,"2":18,"3":15}', status: 'active' as Championship['status'] }

function normalizeChampionship(raw: Record<string, unknown>): Championship {
    let pointsSystem: Record<number, number> = {}
    if (typeof raw.points_system === 'string') {
        try {
            pointsSystem = JSON.parse(raw.points_system as string) as Record<number, number>
        } catch {
            pointsSystem = {}
        }
    } else if (raw.points_system && typeof raw.points_system === 'object') {
        pointsSystem = raw.points_system as Record<number, number>
    }

    return {
        id: String(raw.id || ''),
        season_id: String(raw.season_id || ''),
        name: String(raw.name || 'Championship'),
        points_system: pointsSystem,
        status: (raw.status as Championship['status']) || 'active',
        created_at: raw.created_at ? String(raw.created_at) : undefined,
    }
}

export default function ChampionshipView() {
    const [seasons, setSeasons] = useState<Season[]>([])
    const [championships, setChampionships] = useState<Championship[]>([])
    const [selectedChampionshipId, setSelectedChampionshipId] = useState('')
    const [seasonForm, setSeasonForm] = useState(seasonDefaults)
    const [championshipForm, setChampionshipForm] = useState(championshipDefaults)
    const [standings, setStandings] = useState<StandingRow[]>([])
    const [status, setStatus] = useState('')

    const selectedChampionship = useMemo(
        () => championships.find(championship => championship.id === selectedChampionshipId) || null,
        [championships, selectedChampionshipId],
    )

    const loadData = async () => {
        try {
            const [seasonsResponse, championshipsResponse] = await Promise.all([
                apiFetch('/api/seasons'),
                apiFetch('/api/championships'),
            ])
            if (!seasonsResponse.ok || !championshipsResponse.ok) throw new Error('Failed to load championship data')
            const seasonsPayload = await seasonsResponse.json() as Season[]
            const championshipsPayload = await championshipsResponse.json()
            const normalized = Array.isArray(championshipsPayload)
                ? championshipsPayload.map(item => normalizeChampionship(item as Record<string, unknown>))
                : []
            setSeasons(seasonsPayload)
            setChampionships(normalized)
            if (!selectedChampionshipId && normalized[0]?.id) {
                setSelectedChampionshipId(normalized[0].id)
            }
            if (!championshipForm.season_id && seasonsPayload[0]?.id) {
                setChampionshipForm(prev => ({ ...prev, season_id: seasonsPayload[0].id }))
            }
            setStatus('')
        } catch (error) {
            setStatus(error instanceof Error ? error.message : 'Unable to load championships.')
        }
    }

    useEffect(() => {
        void loadData()
    }, [])

    useEffect(() => {
        if (!selectedChampionshipId) {
            setStandings([])
            return
        }
        const loadStandings = async () => {
            try {
                const response = await apiFetch(`/api/analytics/championship/${selectedChampionshipId}/standings`)
                if (!response.ok) throw new Error('Failed to load standings')
                const payload = await response.json()
                setStandings(Array.isArray(payload) ? payload as StandingRow[] : [])
                setStatus('')
            } catch (error) {
                setStatus(error instanceof Error ? error.message : 'Unable to load standings.')
            }
        }
        void loadStandings()
    }, [selectedChampionshipId])

    const submitSeason = async (event: FormEvent) => {
        event.preventDefault()
        try {
            const response = await apiFetch('/api/seasons', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(seasonForm),
            })
            if (!response.ok) throw new Error('Failed to create season')
            setSeasonForm(seasonDefaults)
            setStatus('Season created.')
            await loadData()
        } catch (error) {
            setStatus(error instanceof Error ? error.message : 'Unable to create season.')
        }
    }

    const submitChampionship = async (event: FormEvent) => {
        event.preventDefault()
        try {
            const response = await apiFetch('/api/championships', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(championshipForm),
            })
            if (!response.ok) throw new Error('Failed to create championship')
            setChampionshipForm(prev => ({ ...championshipDefaults, season_id: prev.season_id }))
            setStatus('Championship created.')
            await loadData()
        } catch (error) {
            setStatus(error instanceof Error ? error.message : 'Unable to create championship.')
        }
    }

    return (
        <div className="fade-in space-y-4">
            <div>
                <h2 className="text-xl font-semibold text-[var(--color-text-primary)]">Championship</h2>
                <p className="text-sm text-[var(--color-text-secondary)]">
                    Manage seasons and championships, then review the live points table using the backend standings endpoint.
                </p>
            </div>

            <div className="grid gap-4 xl:grid-cols-[320px_320px_1fr]">
                <form className="race-card space-y-3" onSubmit={submitSeason}>
                    <h3 className="text-base font-semibold">Create season</h3>
                    <input className="w-full rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm" value={seasonForm.name} onChange={event => setSeasonForm(prev => ({ ...prev, name: event.target.value }))} placeholder="Season name" required />
                    <input className="w-full rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm" type="number" value={seasonForm.year} onChange={event => setSeasonForm(prev => ({ ...prev, year: Number(event.target.value) }))} placeholder="Year" required />
                    <button className="rounded bg-blue-600 px-4 py-2 text-sm font-semibold text-white hover:bg-blue-500" type="submit">Add Season</button>
                </form>

                <form className="race-card space-y-3" onSubmit={submitChampionship}>
                    <h3 className="text-base font-semibold">Create championship</h3>
                    <select className="w-full rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm text-white" value={championshipForm.season_id} onChange={event => setChampionshipForm(prev => ({ ...prev, season_id: event.target.value }))} required>
                        <option value="">Select season</option>
                        {seasons.map(season => <option key={season.id} value={season.id}>{season.name} ({season.year})</option>)}
                    </select>
                    <input className="w-full rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm" value={championshipForm.name} onChange={event => setChampionshipForm(prev => ({ ...prev, name: event.target.value }))} placeholder="Championship name" required />
                    <textarea className="min-h-[110px] w-full rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm" value={championshipForm.points_system} onChange={event => setChampionshipForm(prev => ({ ...prev, points_system: event.target.value }))} />
                    <button className="rounded bg-indigo-600 px-4 py-2 text-sm font-semibold text-white hover:bg-indigo-500" type="submit">Add Championship</button>
                </form>

                <div className="race-card space-y-4">
                    <div className="flex flex-wrap items-center justify-between gap-3">
                        <div>
                            <h3 className="text-base font-semibold">Standings</h3>
                            <p className="text-xs text-[var(--color-text-secondary)]">
                                {selectedChampionship ? `${selectedChampionship.name} standings` : 'Select a championship to load standings.'}
                            </p>
                        </div>
                        <select className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm text-white" value={selectedChampionshipId} onChange={event => setSelectedChampionshipId(event.target.value)}>
                            <option value="">Select championship</option>
                            {championships.map(championship => <option key={championship.id} value={championship.id}>{championship.name}</option>)}
                        </select>
                    </div>

                    {status ? <p className="text-sm text-[var(--color-text-secondary)]">{status}</p> : null}

                    <div className="overflow-x-auto">
                        <table className="min-w-full text-sm">
                            <thead>
                                <tr className="text-left text-[var(--color-text-muted)]">
                                    <th className="pb-2">Pos</th>
                                    <th className="pb-2">Racer</th>
                                    <th className="pb-2">Points</th>
                                    <th className="pb-2">Wins</th>
                                    <th className="pb-2">Podiums</th>
                                    <th className="pb-2">Races</th>
                                </tr>
                            </thead>
                            <tbody>
                                {standings.map((row, index) => (
                                    <tr key={row.racer_id} className="border-t border-white/6">
                                        <td className="py-2">{index + 1}</td>
                                        <td className="py-2">{row.racer_name} <span className="text-[var(--color-text-secondary)]">#{row.racer_number || '—'}</span></td>
                                        <td className="py-2 font-semibold">{row.total_points ?? 0}</td>
                                        <td className="py-2">{row.wins ?? 0}</td>
                                        <td className="py-2">{row.podiums ?? 0}</td>
                                        <td className="py-2">{row.races_completed ?? 0}</td>
                                    </tr>
                                ))}
                            </tbody>
                        </table>
                    </div>
                </div>
            </div>
        </div>
    )
}
