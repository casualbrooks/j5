export default function VisionPanel() {
    return (
        <div className="fade-in space-y-4">
            <h2 className="text-xl font-semibold text-[var(--color-text-primary)]">Computer Vision</h2>
            <div className="race-card p-8 text-center">
                <div className="text-4xl mb-3">📷</div>
                <p className="text-[var(--color-text-secondary)]">
                    Camera configuration, AI model selection, and detection feed preview. Connect cameras to begin.
                </p>
            </div>
        </div>
    )
}
