import { FormEvent, useEffect, useMemo, useState } from 'react'

interface SetupCheckResult {
    command: string
    ok: boolean
    stdout?: string
    stderr?: string
}

interface SetupStep {
    id: string
    title: string
    description: string
    connected: boolean
    checks: SetupCheckResult[]
    next_command: string
    stop_command: string
    next_step_command?: string
    is_current?: boolean
    last_error?: string | null
}

interface WizardStatus {
    config: Record<string, unknown>
    steps: SetupStep[]
    race_context: Record<string, unknown>
}

const defaultConfig = {
    pi_host: 'pi-host-or-ip.local',
    pi_user: 'pi-user',
    backend_url: 'http://<pi-ip:8080',
    preview_url: 'http://pi-ip:8091',
    race_name: 'Main Event',
    event_name: 'Weekend Session',
    total_laps: 20,
    racer_names: 'Racer 1, Racer 2',
}

export default function SettingsPanel() {
    const [wizard, setWizard] = useState<WizardStatus | null>(null)
    const [busyStepId, setBusyStepId] = useState<string | null>(null)
    const [error, setError] = useState('')
    const [config, setConfig] = useState(defaultConfig)

    const loadWizard = async () => {
        try {
            const response = await fetch('/api/setup/wizard')
            if (!response.ok) throw new Error('Failed to load wizard status')
            const payload: WizardStatus = await response.json()
            const rawNames = (payload.config.racer_names as string[] | undefined) || []
            setWizard(payload)
            setConfig({
                pi_host: String(payload.config.pi_host ?? defaultConfig.pi_host),
                pi_user: String(payload.config.pi_user ?? defaultConfig.pi_user),
                backend_url: String(payload.config.backend_url ?? defaultConfig.backend_url),
                preview_url: String(payload.config.preview_url ?? defaultConfig.preview_url),
                race_name: String(payload.config.race_name ?? defaultConfig.race_name),
                event_name: String(payload.config.event_name ?? defaultConfig.event_name),
                total_laps: Number(payload.config.total_laps ?? defaultConfig.total_laps),
                racer_names: rawNames.join(', '),
            })
            setError('')
        } catch (err) {
            setError(err instanceof Error ? err.message : 'Unable to load setup wizard.')
        }
    }

    useEffect(() => {
        loadWizard()
    }, [])

    const currentStep = useMemo(() => wizard?.steps.find(step => step.is_current), [wizard])

    const runStepAction = async (stepId: string, action: 'verify' | 'connect' | 'stop') => {
        setBusyStepId(stepId)
        try {
            const response = await fetch(`/api/setup/wizard/steps/${stepId}/${action}`, { method: 'POST' })
            if (!response.ok) throw new Error(`Failed to ${action} step ${stepId}`)
            const payload = await response.json()
            setWizard(payload.wizard || payload)
            setError('')
        } catch (err) {
            setError(err instanceof Error ? err.message : `Unable to ${action} step.`)
        } finally {
            setBusyStepId(null)
        }
    }

    const onSaveConfig = async (event: FormEvent) => {
        event.preventDefault()
        try {
            const response = await fetch('/api/setup/wizard/config', {
                method: 'PUT',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    ...config,
                    racer_names: config.racer_names
                        .split(',')
                        .map(name => name.trim())
                        .filter(Boolean),
                }),
            })
            if (!response.ok) throw new Error('Failed to save wizard config')
            const payload = await response.json()
            setWizard(payload)
            setError('')
        } catch (err) {
            setError(err instanceof Error ? err.message : 'Unable to save setup configuration.')
        }
    }

    const controlRace = async (action: 'start' | 'pause' | 'snapshot' | 'resume') => {
        const raceId = String(wizard?.race_context?.race_id || '')
        if (!raceId) {
            setError('Initialize race state first before using race controls.')
            return
        }
        const endpointMap: Record<'start' | 'pause' | 'snapshot' | 'resume', string> = {
            start: `/api/races/${raceId}/start`,
            pause: `/api/races/${raceId}/pause`,
            snapshot: `/api/races/${raceId}/snapshot`,
            resume: `/api/races/${raceId}/resume`,
        }
        try {
            const response = await fetch(endpointMap[action], { method: 'POST' })
            if (!response.ok) throw new Error(`Failed to ${action} race`)
            await loadWizard()
        } catch (err) {
            setError(err instanceof Error ? err.message : `Unable to ${action} race.`)
        }
    }

    const initializeRace = async () => {
        try {
            const response = await fetch('/api/setup/wizard/initialize', { method: 'POST' })
            if (!response.ok) throw new Error('Failed to initialize race state')
            const payload = await response.json()
            setWizard(payload.wizard)
        } catch (err) {
            setError(err instanceof Error ? err.message : 'Unable to initialize race.')
        }
    }

    return (
        <div className="fade-in space-y-4">
            <h2 className="text-xl font-semibold text-[var(--color-text-primary)]">Settings · Race Setup Wizard</h2>

            <div className="race-card p-4">
                <h3 className="mb-2 text-base font-semibold">Configuration</h3>
                <form className="grid gap-3 md:grid-cols-2" onSubmit={onSaveConfig}>
                    <input className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm" value={config.pi_host} onChange={e => setConfig(prev => ({ ...prev, pi_host: e.target.value }))} placeholder="Pi host" />
                    <input className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm" value={config.pi_user} onChange={e => setConfig(prev => ({ ...prev, pi_user: e.target.value }))} placeholder="Pi user" />
                    <input className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm" value={config.backend_url} onChange={e => setConfig(prev => ({ ...prev, backend_url: e.target.value }))} placeholder="Backend URL" />
                    <input className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm" value={config.preview_url} onChange={e => setConfig(prev => ({ ...prev, preview_url: e.target.value }))} placeholder="Preview URL" />
                    <input className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm" value={config.event_name} onChange={e => setConfig(prev => ({ ...prev, event_name: e.target.value }))} placeholder="Event name" />
                    <input className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm" value={config.race_name} onChange={e => setConfig(prev => ({ ...prev, race_name: e.target.value }))} placeholder="Race name" />
                    <input className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm" type="number" min={1} value={config.total_laps} onChange={e => setConfig(prev => ({ ...prev, total_laps: Number(e.target.value) }))} placeholder="Total laps" />
                    <input className="rounded border border-slate-700 bg-slate-900 px-3 py-2 text-sm md:col-span-2" value={config.racer_names} onChange={e => setConfig(prev => ({ ...prev, racer_names: e.target.value }))} placeholder="Racer names (comma separated)" />
                    <div className="md:col-span-2 flex gap-2">
                        <button className="rounded bg-blue-600 px-4 py-2 text-sm font-semibold text-white hover:bg-blue-500" type="submit">Save Config</button>
                        <button className="rounded bg-emerald-600 px-4 py-2 text-sm font-semibold text-white hover:bg-emerald-500" type="button" onClick={initializeRace}>Initialize Race State</button>
                        <button className="rounded bg-slate-700 px-4 py-2 text-sm font-semibold text-white hover:bg-slate-600" type="button" onClick={loadWizard}>Refresh</button>
                        <button className="rounded bg-indigo-600 px-4 py-2 text-sm font-semibold text-white hover:bg-indigo-500" type="button" onClick={() => controlRace('start')}>Race Start</button>
                        <button className="rounded bg-amber-600 px-4 py-2 text-sm font-semibold text-white hover:bg-amber-500" type="button" onClick={() => controlRace('pause')}>Race Pause</button>
                        <button className="rounded bg-violet-600 px-4 py-2 text-sm font-semibold text-white hover:bg-violet-500" type="button" onClick={() => controlRace('snapshot')}>Save Snapshot</button>
                        <button className="rounded bg-teal-600 px-4 py-2 text-sm font-semibold text-white hover:bg-teal-500" type="button" onClick={() => controlRace('resume')}>Race Resume</button>
                    </div>
                </form>
            </div>

            {error ? <p className="text-sm text-red-400">{error}</p> : null}

            <div className="race-card p-4 space-y-3">
                <h3 className="text-base font-semibold">Ordered Connection Checklist</h3>
                {currentStep ? (
                    <p className="text-xs text-[var(--color-text-secondary)]">Current blocking step: <strong>{currentStep.title}</strong> · Next command: <code>{currentStep.next_command}</code></p>
                ) : <p className="text-xs text-emerald-400">All setup steps are currently marked connected.</p>}

                <div className="space-y-3">
                    {wizard?.steps.map((step, idx) => (
                        <div key={step.id} className={`rounded border p-3 ${step.connected ? 'border-emerald-700 bg-emerald-950/30' : 'border-amber-700 bg-amber-950/30'}`}>
                            <div className="flex flex-wrap items-center justify-between gap-2">
                                <div>
                                    <p className="font-semibold">{idx + 1}. {step.title}</p>
                                    <p className="text-xs text-[var(--color-text-secondary)]">{step.description}</p>
                                </div>
                                <span className={`text-xs font-semibold ${step.connected ? 'text-emerald-400' : 'text-amber-300'}`}>{step.connected ? 'Connected' : 'Not connected'}</span>
                            </div>

                            <div className="mt-2 text-xs space-y-1">
                                <p>Command to continue: <code>{step.next_command}</code></p>
                                {step.next_step_command ? <p>Next step command: <code>{step.next_step_command}</code></p> : null}
                            </div>

                            <div className="mt-3 flex flex-wrap gap-2">
                                <button className="rounded bg-blue-600 px-3 py-1 text-xs font-semibold text-white" onClick={() => runStepAction(step.id, 'verify')} disabled={busyStepId === step.id}>Verify</button>
                                <button className="rounded bg-emerald-600 px-3 py-1 text-xs font-semibold text-white" onClick={() => runStepAction(step.id, 'connect')} disabled={busyStepId === step.id}>Connect / Reconnect</button>
                                <button className="rounded bg-rose-700 px-3 py-1 text-xs font-semibold text-white" onClick={() => runStepAction(step.id, 'stop')} disabled={busyStepId === step.id}>Stop</button>
                            </div>

                            {step.checks.length > 0 ? (
                                <ul className="mt-3 space-y-1 text-xs">
                                    {step.checks.map(check => (
                                        <li key={check.command} className={check.ok ? 'text-emerald-300' : 'text-rose-300'}>
                                            {check.ok ? '✓' : '✗'} {check.command}
                                        </li>
                                    ))}
                                </ul>
                            ) : null}

                            {step.last_error ? <p className="mt-2 text-xs text-rose-300">Last error: {step.last_error}</p> : null}
                        </div>
                    ))}
                </div>
            </div>
        </div>
    )
}
