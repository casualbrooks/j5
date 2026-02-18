export default function SettingsPanel() {
    return (
        <div className="fade-in space-y-4">
            <h2 className="text-xl font-semibold text-[var(--color-text-primary)]">Settings</h2>
            <div className="race-card p-8 text-center">
                <div className="text-4xl mb-3">⚙️</div>
                <p className="text-[var(--color-text-secondary)]">
                    Track setup, race configuration, app settings, and data import/export.
                </p>
            </div>
        </div>
    )
}
