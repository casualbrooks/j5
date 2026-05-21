import { useEffect, useMemo, useState } from 'react'
import { useRaceContext } from '@/stores/raceStore'
import { apiFetch } from '@/lib/utils'

function defaultPreviewBaseUrl(): string {
    if (typeof window === 'undefined') {
        return 'http://localhost:8091'
    }
    return ''
}

function normalizePreviewBaseUrl(value: string): string {
    const raw = value.trim()
    if (!raw) return ''
    try {
        const parsed = new URL(raw)
        return `${parsed.protocol}//${parsed.host}`
    } catch {
        return raw.replace(/\/$/, '')
    }
}

function parseNumber(value: unknown): number | null {
    const parsed = Number(value)
    return Number.isFinite(parsed) ? parsed : null
}

function extractObjectNumber(objectId: string): string {
    const match = objectId.match(/(\d+)(?!.*\d)/)
    return match ? match[1] : objectId
}

export default function IntegrationCheckPanel() {
    const { connectionStatus, recentVisionObjects, recentObjectDetections, liveRace } = useRaceContext()
    const [previewBaseUrl, setPreviewBaseUrl] = useState(defaultPreviewBaseUrl)
    const [previewEnabled, setPreviewEnabled] = useState(true)
    const normalizedBaseUrl = useMemo(() => normalizePreviewBaseUrl(previewBaseUrl), [previewBaseUrl])
    const streamUrl = normalizedBaseUrl ? `${normalizedBaseUrl}/stream.mjpg` : ''
    const [statusNowMs, setStatusNowMs] = useState(() => Date.now())
    const lastVisionSeenAt = recentVisionObjects[0]?.seenAt || 0
    const secondsSinceVision = lastVisionSeenAt ? Math.max(0, Math.floor((statusNowMs - lastVisionSeenAt) / 1000)) : null
    const lastDetectionSeenAt = recentObjectDetections[0]?.seenAt || 0
    const secondsSinceDetection = lastDetectionSeenAt ? Math.max(0, Math.floor((statusNowMs - lastDetectionSeenAt) / 1000)) : null
    const raceStatus = liveRace?.status || 'setup'
    const hasRecentVisionObjects = recentVisionObjects.length > 0
    const drawableVisionObjects = recentVisionObjects
        .slice(0, 12)
        .map((item) => {
            const x = parseNumber(item.bbox?.x)
            const y = parseNumber(item.bbox?.y)
            const width = parseNumber(item.bbox?.width)
            const height = parseNumber(item.bbox?.height)
            const frameWidth = parseNumber(item.frameSize?.width)
            const frameHeight = parseNumber(item.frameSize?.height)
            if (x == null || y == null || width == null || height == null || frameWidth == null || frameHeight == null) return null
            return {
                item,
                leftPct: (x / frameWidth) * 100,
                topPct: (y / frameHeight) * 100,
                widthPct: (width / frameWidth) * 100,
                heightPct: (height / frameHeight) * 100,
            }
        })
        .filter((entry): entry is {
            item: typeof recentVisionObjects[number]
            leftPct: number
            topPct: number
            widthPct: number
            heightPct: number
        } => entry !== null)
    const hasDrawableVisionObjects = drawableVisionObjects.length > 0
    const visionIsFresh = secondsSinceVision !== null && secondsSinceVision <= 3
    const autoDetectedTopics = Array.from(
        new Set(
            recentVisionObjects
                .map(item => item.cameraTopic)
                .filter(topic => Boolean(topic)),
        ),
    )
    const previewContent = !previewEnabled
        ? (
            <div className="rounded border border-dashed border-slate-700 bg-black/30 p-4 text-xs text-[var(--color-text-secondary)]">
                Embedded camera preview is paused.
            </div>
        )
        : !normalizedBaseUrl
            ? (
                <div className="rounded border border-dashed border-amber-700 bg-amber-950/20 p-4 text-xs text-amber-200">
                    Set Preview server URL first, then show embedded camera.
                </div>
            )
            : (
                <div className="relative">
                    <img
                        src={streamUrl}
                        alt="Embedded camera stream"
                        className="w-full rounded border border-slate-700 bg-black"
                    />
                    <div className="pointer-events-none absolute inset-0">
                        {drawableVisionObjects.map(({ item, leftPct, topPct, widthPct, heightPct }) => (
                            <div
                                key={`${item.objectId}-${item.seenAt}`}
                                className="absolute border border-emerald-400/90"
                                style={{ left: `${leftPct}%`, top: `${topPct}%`, width: `${widthPct}%`, height: `${heightPct}%` }}
                            >
                                <span className="absolute -top-4 right-0 rounded bg-black/75 px-1.5 py-0.5 text-[10px] font-semibold text-emerald-200">
                                    {extractObjectNumber(item.objectId)}
                                </span>
                            </div>
                        ))}
                        {hasRecentVisionObjects && !hasDrawableVisionObjects ? (
                            <div className="absolute left-2 top-2 max-w-[70%] space-y-1">
                                {recentVisionObjects.slice(0, 6).map((item) => (
                                    <p key={`${item.objectId}-${item.seenAt}`} className="rounded bg-black/70 px-2 py-1 text-[11px] text-emerald-200">
                                        {extractObjectNumber(item.objectId)}
                                        {item.position ? ` (${item.position.x.toFixed(1)}, ${item.position.y.toFixed(1)})` : ''}
                                        {item.cameraTopic ? ` · ${item.cameraTopic}` : ''}
                                    </p>
                                ))}
                            </div>
                        ) : null}
                        {!hasRecentVisionObjects ? (
                            <p className="absolute left-2 top-2 rounded bg-black/70 px-2 py-1 text-[11px] text-amber-200">
                                No object IDs yet. Move an object through frame and verify ROS2 topics below.
                            </p>
                        ) : null}
                    </div>
                </div>
            )

    useEffect(() => {
        const timer = window.setInterval(() => {
            setStatusNowMs(Date.now())
        }, 1000)
        return () => {
            window.clearInterval(timer)
        }
    }, [])

    useEffect(() => {
        const loadPreviewUrl = async () => {
            try {
                const response = await apiFetch('/api/setup/wizard')
                if (!response.ok) return
                const payload = await response.json() as { config?: { preview_url?: unknown } }
                const previewUrl = String(payload?.config?.preview_url || '').trim()
                if (previewUrl) {
                    setPreviewBaseUrl(previewUrl)
                }
            } catch {
                // keep fallback URL when setup payload cannot be loaded
            }
        }

        void loadPreviewUrl()
    }, [])

    const topicListCommand = 'ros2 topic list -t | rg -n "Image|CompressedImage|camera|video"'
    const topicHzCommand = 'ros2 topic hz <camera_topic_from_step_1>'
    const topicEchoCommand = 'ros2 topic echo <camera_topic_from_step_1> --once'
    const topicInfoCommand = 'ros2 topic info <camera_topic_from_step_1> -v'
    const nodeListCommand = 'ros2 node list'
    const ros2SourceCommand =
        'if [ -f ~/ros2_kilted/install/setup.bash ]; then source ~/ros2_kilted/install/setup.bash; elif [ -f /opt/ros/iron/setup.bash ]; then source /opt/ros/iron/setup.bash; else echo "Missing ROS underlay setup.bash"; fi; if [ -f ~/j5/ros_ws/install/setup.bash ]; then source ~/j5/ros_ws/install/setup.bash; elif [ -f ~/alive/j5/ros_ws/install/setup.bash ]; then source ~/alive/j5/ros_ws/install/setup.bash; else echo "Missing j5 workspace setup.bash"; fi'
    const perceptionRunCommand =
        'ros2 run racetracker_perception perception_node --ros-args --log-level info -p camera_topics:=[<camera_topic_from_step_1>] -p auto_discover_camera_topics:=true'
    const lapEventPublishCommand =
        "ros2 topic pub /race/lap_event std_msgs/msg/String '{data: \"{\\\"raceId\\\":\\\"demo-race\\\",\\\"carId\\\":\\\"car-7\\\",\\\"sessionLap\\\":1,\\\"lapTimeMs\\\":70000,\\\"trackDistanceM\\\":350.0,\\\"validity\\\":{\\\"onTrack\\\":true,\\\"directionOk\\\":true,\\\"minLapTimeOk\\\":true}}\"}' --once -w 0"

    return (
        <div className="fade-in space-y-4">
            <h2 className="text-xl font-semibold text-[var(--color-text-primary)]">Perception Integration Check</h2>

            <div className="race-card p-4 space-y-3 text-sm">
                <p className="text-[var(--color-text-secondary)]">
                    Use this page to verify camera video, ROS2 perception, and websocket updates without running the full race flow.
                </p>
                <p className="text-xs text-amber-200">
                    Note: the embedded camera image is a raw MJPEG preview. This UI currently overlays object-ID badges, not pixel bounding boxes.
                </p>

                <div className="rounded border border-slate-700 bg-slate-950/70 p-3 space-y-2 text-xs text-slate-200">
                    <p><strong>Run these commands in order (kilted source-build flow)</strong></p>
                    <code className="block overflow-x-auto rounded bg-black/40 p-2 text-emerald-300">
                        {ros2SourceCommand}
                    </code>
                    <p><strong>1) Confirm the camera topic exists and is publishing frames</strong></p>
                    <code className="block overflow-x-auto rounded bg-black/40 p-2 text-emerald-300">
                        {topicListCommand}
                    </code>
                    <p className="text-[11px] text-slate-300">
                        Pick the real topic from step 1 (for example <code>/actual/image/topic</code>) and use it in steps 1, 4, and 5.
                    </p>
                    <code className="block overflow-x-auto rounded bg-black/40 p-2 text-emerald-300">
                        {topicHzCommand}
                    </code>
                    <code className="block overflow-x-auto rounded bg-black/40 p-2 text-emerald-300">
                        {topicEchoCommand}
                    </code>
                    <p><strong>2) Start camera preview (annotated IDs are overlaid in this Integration tab)</strong></p>
                    <code className="block overflow-x-auto rounded bg-black/40 p-2 text-emerald-300">
                        python3 scripts/pi_preflight.py --camera-source /dev/video0 --capture-file track_snapshot.jpg --serve-preview --preview-host 0.0.0.0 --preview-port 8091 --preview-fps 15
                    </code>
                    <p><strong>3) In a second terminal, source workspaces again</strong></p>
                    <code className="block overflow-x-auto rounded bg-black/40 p-2 text-emerald-300">
                        {ros2SourceCommand}
                    </code>
                    <p><strong>4) Start the perception node on the real image topic</strong></p>
                    <code className="block overflow-x-auto rounded bg-black/40 p-2 text-emerald-300">
                        {perceptionRunCommand}
                    </code>
                    <p><strong>5) Verify perception node is alive and subscribed</strong></p>
                    <code className="block overflow-x-auto rounded bg-black/40 p-2 text-emerald-300">
                        {topicInfoCommand}
                    </code>
                    <code className="block overflow-x-auto rounded bg-black/40 p-2 text-emerald-300">
                        {nodeListCommand}
                    </code>
                    <p className="text-[11px] text-amber-200">
                        If <code>ros2 topic hz</code>, <code>ros2 topic echo</code>, or <code>ros2 param</code> crashes with <code>Segmentation fault</code> / <code>double free</code>, open a fresh shell and run only the two <code>source</code> commands shown above.
                    </p>
                    <p className="text-[11px] text-slate-300">
                        Use the same real camera topic in steps 1, 4, and 5 so the perception node subscribes to the stream that is actually publishing.
                    </p>
                    <p><strong>6) Optional bridge smoke test (publish one lap event)</strong></p>
                    <code className="block overflow-x-auto rounded bg-black/40 p-2 text-emerald-300">
                        {lapEventPublishCommand}
                    </code>
                    <p className="text-[11px] text-slate-300">
                        This command only validates the ROS lap-event bridge path. It does <strong>not</strong> create Computer Vision detections by itself.
                        It publishes immediately even if no subscribers are active. If your ROS CLI crashes when using wait flags, retry in a fresh shell with <code>--once</code> only.
                    </p>
                </div>
            </div>

            <div className="race-card p-4 space-y-3">
                <h3 className="text-sm font-semibold text-[var(--color-text-primary)]">Live module status</h3>
                <ul className="space-y-2 text-sm">
                    <li>
                        UI websocket:{' '}
                        <span className={connectionStatus === 'connected' ? 'text-emerald-300' : 'text-amber-300'}>
                            {connectionStatus}
                        </span>
                    </li>
                    <li>
                        Vision object stream:{' '}
                        <span className={secondsSinceVision !== null && secondsSinceVision <= 3 ? 'text-emerald-300' : 'text-amber-300'}>
                            {secondsSinceVision === null ? 'waiting for objects' : `${secondsSinceVision}s since last object`}
                        </span>
                    </li>
                    <li>
                        Auto-detected camera topics:{' '}
                        <span className={autoDetectedTopics.length > 0 ? 'text-emerald-300' : 'text-amber-300'}>
                            {autoDetectedTopics.length > 0 ? autoDetectedTopics.join(', ') : 'none detected yet'}
                        </span>
                    </li>
                    <li>
                        Detection events in memory:{' '}
                        <span className={recentObjectDetections.length > 0 ? 'text-emerald-300' : 'text-amber-300'}>
                            {recentObjectDetections.length}
                        </span>
                    </li>
                    <li>
                        Tracking health:{' '}
                        <span className={visionIsFresh && recentObjectDetections.length > 0 ? 'text-emerald-300' : 'text-amber-300'}>
                            {visionIsFresh && recentObjectDetections.length > 0 ? 'active detections' : 'camera up but no confirmed detections yet'}
                        </span>
                    </li>
                    <li>
                        Race runtime status:{' '}
                        <span className={raceStatus === 'active' ? 'text-emerald-300' : 'text-slate-300'}>{raceStatus}</span>
                    </li>
                </ul>
            </div>

            <div className="race-card p-4 space-y-3">
                <div className="flex flex-wrap items-center justify-between gap-2">
                    <h3 className="text-sm font-semibold text-[var(--color-text-primary)]">Live camera + object activity</h3>
                    <button
                        type="button"
                        className="rounded bg-slate-700 px-3 py-2 text-xs font-semibold text-white hover:bg-slate-600"
                        onClick={() => setPreviewEnabled(prev => !prev)}
                    >
                        {previewEnabled ? 'Hide embedded camera' : 'Show embedded camera'}
                    </button>
                </div>
                <label className="block text-sm text-[var(--color-text-secondary)]">
                    Preview server URL
                    <input
                        className="mt-1 w-full rounded border border-slate-600 bg-slate-900 px-3 py-2 text-sm text-white"
                        value={previewBaseUrl}
                        onChange={(event) => setPreviewBaseUrl(event.target.value)}
                        placeholder="http://192.168.1.134:8091"
                    />
                </label>
                {previewEnabled ? (
                    normalizedBaseUrl ? (
                    <div className="relative">
                        <img
                            src={streamUrl}
                            alt="Embedded camera stream"
                            className="w-full rounded border border-slate-700 bg-black"
                        />
                        <div className="pointer-events-none absolute inset-0">
                            {drawableVisionObjects.map(({ item, leftPct, topPct, widthPct, heightPct }) => (
                                <div
                                    key={`${item.objectId}-${item.seenAt}`}
                                    className="absolute border border-emerald-400/90"
                                    style={{ left: `${leftPct}%`, top: `${topPct}%`, width: `${widthPct}%`, height: `${heightPct}%` }}
                                >
                                    <span className="absolute -top-4 right-0 rounded bg-black/75 px-1.5 py-0.5 text-[10px] font-semibold text-emerald-200">
                                        {extractObjectNumber(item.objectId)}
                                    </span>
                                </div>
                            ))}
                            {hasRecentVisionObjects && !hasDrawableVisionObjects ? (
                                <div className="absolute left-2 top-2 max-w-[70%] space-y-1">
                                    {recentVisionObjects.slice(0, 6).map((item) => (
                                        <p key={`${item.objectId}-${item.seenAt}`} className="rounded bg-black/70 px-2 py-1 text-[11px] text-emerald-200">
                                            {extractObjectNumber(item.objectId)}
                                            {item.position ? ` (${item.position.x.toFixed(1)}, ${item.position.y.toFixed(1)})` : ''}
                                            {item.cameraTopic ? ` · ${item.cameraTopic}` : ''}
                                        </p>
                                    ))}
                                </div>
                            ) : null}
                            {!hasRecentVisionObjects ? (
                                <p className="absolute left-2 top-2 rounded bg-black/70 px-2 py-1 text-[11px] text-amber-200">
                                    No object IDs yet. Move an object through frame and verify ROS2 topics below.
                                </p>
                            ) : null}
                        </div>
                        </div>
                    ) : (
                        <div className="rounded border border-dashed border-amber-700 bg-amber-950/20 p-4 text-xs text-amber-200">
                            Set Preview server URL first, then show embedded camera.
                        </div>
                    )
                ) : (
                    <div className="rounded border border-dashed border-amber-700 bg-amber-950/20 p-4 text-xs text-amber-200">
                        Set Preview server URL first, then show embedded camera.
                    </div>
                ) : (
                    <div className="rounded border border-dashed border-slate-700 bg-black/30 p-4 text-xs text-[var(--color-text-secondary)]">
                        Embedded camera preview is paused.
                    </div>
                )}
                <div className="rounded border border-slate-700 bg-slate-950/60 p-3 text-xs text-slate-200 space-y-1">
                    <p className="font-semibold text-slate-100">If tracking still does not start, check this in order:</p>
                    <p>• Camera stream loads in this panel and updates continuously.</p>
                    <p>• <code>{nodeListCommand}</code> includes <code>/racetracker_perception</code> (or your perception node name).</p>
                    <p>• <code>{topicInfoCommand}</code> shows at least one publisher and one subscriber.</p>
                    <p>• Keep the perception process running in a dedicated terminal and watch logs for frame decode or websocket errors:</p>
                    <code className="block overflow-x-auto rounded bg-black/40 p-2 text-emerald-300">
                        {perceptionRunCommand}
                    </code>
                    <p>• Detection count in this tab increases from 0 and “seconds since last object” stays low.</p>
                </div>
                <div className="rounded border border-slate-700 bg-slate-950/60 p-3 text-xs text-slate-200 space-y-1">
                    <p className="font-semibold text-slate-100">Camera publisher helper script</p>
                    <p>If <code>usb_camera</code> is unavailable, publish camera frames with this script:</p>
                    <code className="block overflow-x-auto rounded bg-black/40 p-2 text-emerald-300">
                        python3 scripts/camera_ros_publisher.py --device /dev/video0 --topic /camera/image_raw --fps 30 --serve-preview --preview-host 0.0.0.0 --preview-port 8091
                    </code>
                    <p>Then set <code>camera_topics</code> on the perception node to the same topic (for example <code>/camera/image_raw</code>).</p>
                </div>
            </div>

            <div className="race-card p-4 space-y-2 text-xs text-slate-200">
                <h3 className="text-sm font-semibold text-[var(--color-text-primary)]">How IDs flow into race tracking</h3>
                <p>1) Object IDs that appear here are the same IDs suggested in the Settings tab assignment fields.</p>
                <p>2) Saved assignments are reused by tracking logic to map object IDs → racer IDs.</p>
                <p>3) During an active race, mapped detections update racer position, trigger checkpoint/finish events, and write lap records/log entries.</p>
            </div>
        </div>
    )
}
