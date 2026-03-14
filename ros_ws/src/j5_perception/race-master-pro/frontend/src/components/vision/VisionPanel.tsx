import { FormEvent, useMemo, useState } from 'react'

function defaultPreviewBaseUrl(): string {
    if (typeof window === 'undefined') {
        return 'http://localhost:8091'
    }
    return `http://${window.location.hostname}:8091`
}

export default function VisionPanel() {
    const [previewBaseUrl, setPreviewBaseUrl] = useState(defaultPreviewBaseUrl)
    const [statusMessage, setStatusMessage] = useState('')
    const [statusError, setStatusError] = useState(false)
    const [capturing, setCapturing] = useState(false)

    const normalizedBaseUrl = useMemo(() => previewBaseUrl.trim().replace(/\/$/, ''), [previewBaseUrl])
    const streamUrl = `${normalizedBaseUrl}/stream.mjpg`

    const onCapture = async (event: FormEvent) => {
        event.preventDefault()
        setCapturing(true)
        setStatusMessage('')
        setStatusError(false)

        try {
            const response = await fetch(`${normalizedBaseUrl}/capture`, {
                method: 'POST',
            })
            const payload = await response.json()

            if (!response.ok || !payload.ok) {
                throw new Error(payload.message || 'Capture failed')
            }

            setStatusMessage(payload.message || 'Snapshot captured successfully.')
        } catch (error) {
            const message = error instanceof Error ? error.message : 'Unable to capture snapshot.'
            setStatusMessage(message)
            setStatusError(true)
        } finally {
            setCapturing(false)
        }
    }

    return (
        <div className="fade-in space-y-4">
            <h2 className="text-xl font-semibold text-[var(--color-text-primary)]">Computer Vision</h2>

            <div className="race-card p-4 space-y-3">
                <label className="block text-sm text-[var(--color-text-secondary)]">
                    Preview server URL
                    <input
                        className="mt-1 w-full rounded border border-slate-600 bg-slate-900 px-3 py-2 text-sm text-white"
                        value={previewBaseUrl}
                        onChange={(event) => setPreviewBaseUrl(event.target.value)}
                        placeholder="http://192.168.1.134:8091"
                    />
                </label>
                <p className="text-xs text-[var(--color-text-secondary)]">
                    Run preflight with <code>--serve-preview --preview-host 0.0.0.0 --preview-port 8091</code> on the Pi,
                    then use this Vision tab to view the live camera and trigger the track photo.
                </p>
            </div>

            <div className="race-card p-4 space-y-3">
                <img
                    src={streamUrl}
                    alt="Live camera preview"
                    className="w-full rounded border border-slate-700 bg-black"
                />

                <form onSubmit={onCapture}>
                    <button
                        type="submit"
                        disabled={capturing}
                        className="rounded bg-blue-600 px-4 py-2 text-sm font-semibold text-white hover:bg-blue-500 disabled:cursor-not-allowed disabled:opacity-70"
                    >
                        {capturing ? 'Capturing…' : 'Capture Track Photo'}
                    </button>
                </form>

                {statusMessage ? (
                    <p className={`text-sm ${statusError ? 'text-red-400' : 'text-emerald-400'}`}>{statusMessage}</p>
                ) : null}
            </div>
        </div>
    )
}
