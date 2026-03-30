import { useRef, useEffect, useCallback, useMemo, useState, type PointerEvent as ReactPointerEvent } from 'react'
import type { LiveRacer, TrackPoint, Checkpoint } from '@/types'
import { stringToColor } from '@/lib/utils'

type EditTool = 'spline' | 'start' | 'finish' | 'checkpoint'

interface TrackCanvasProps {
    racers: LiveRacer[]
    layoutPoints: TrackPoint[]
    checkpoints: Checkpoint[]
    isEditMode?: boolean
    editTool?: EditTool
    onTrackUpdate?: (points: TrackPoint[]) => void
    onCheckpointUpdate?: (checkpoints: Checkpoint[]) => void
    onMarkerPlaced?: (checkpoint: Checkpoint) => void
    backgroundImageUrl?: string
}

function sampleSpline(points: TrackPoint[], segments = 18): TrackPoint[] {
    if (points.length < 2) return points
    const closed = points.length > 2
    const result: TrackPoint[] = []
    const total = closed ? points.length : points.length - 1

    for (let i = 0; i < total; i++) {
        const p0 = points[(i - 1 + points.length) % points.length] ?? points[0]!
        const p1 = points[i]!
        const p2 = points[(i + 1) % points.length] ?? points[points.length - 1]!
        const p3 = points[(i + 2) % points.length] ?? points[points.length - 1]!

        for (let step = 0; step < segments; step++) {
            const t = step / segments
            const t2 = t * t
            const t3 = t2 * t
            const x = 0.5 * (
                (2 * p1.x)
                + (-p0.x + p2.x) * t
                + (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * t2
                + (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * t3
            )
            const y = 0.5 * (
                (2 * p1.y)
                + (-p0.y + p2.y) * t
                + (2 * p0.y - 5 * p1.y + 4 * p2.y - p3.y) * t2
                + (-p0.y + 3 * p1.y - 3 * p2.y + p3.y) * t3
            )
            result.push({ x, y })
        }
    }

    result.push(points[closed ? 0 : points.length - 1]!)
    return result
}

function clampNormalized(point: TrackPoint): TrackPoint {
    return {
        x: Math.min(1, Math.max(0, point.x)),
        y: Math.min(1, Math.max(0, point.y)),
    }
}

export default function TrackCanvas({
    racers,
    layoutPoints,
    checkpoints,
    isEditMode = false,
    editTool = 'spline',
    onTrackUpdate,
    onCheckpointUpdate,
    onMarkerPlaced,
    backgroundImageUrl,
}: TrackCanvasProps) {
    const canvasRef = useRef<HTMLCanvasElement>(null)
    const containerRef = useRef<HTMLDivElement>(null)
    const backgroundImageRef = useRef<HTMLImageElement | null>(null)
    const [dragIndex, setDragIndex] = useState<number | null>(null)

    useEffect(() => {
        if (!backgroundImageUrl) {
            backgroundImageRef.current = null
            return
        }
        const loadImage = (useAnonymousCors: boolean) => {
            const image = new Image()
            if (useAnonymousCors) {
                image.crossOrigin = 'anonymous'
            }
            image.src = backgroundImageUrl
            image.onload = () => {
                backgroundImageRef.current = image
            }
            image.onerror = () => {
                if (useAnonymousCors) {
                    loadImage(false)
                    return
                }
                backgroundImageRef.current = null
            }
        }
        loadImage(true)
    }, [backgroundImageUrl])

    const splinePoints = useMemo(() => sampleSpline(layoutPoints), [layoutPoints])

    const pointToCanvas = useCallback((point: TrackPoint, width: number, height: number) => {
        if (point.x >= 0 && point.x <= 1 && point.y >= 0 && point.y <= 1) {
            return { x: point.x * width, y: point.y * height }
        }
        const image = backgroundImageRef.current
        if (image && image.naturalWidth > 0 && image.naturalHeight > 0) {
            return {
                x: (point.x / image.naturalWidth) * width,
                y: (point.y / image.naturalHeight) * height,
            }
        }
        return point
    }, [])

    const canvasToPoint = useCallback((clientX: number, clientY: number) => {
        const canvas = canvasRef.current
        if (!canvas) return null
        const rect = canvas.getBoundingClientRect()
        if (!rect.width || !rect.height) return null
        return clampNormalized({
            x: (clientX - rect.left) / rect.width,
            y: (clientY - rect.top) / rect.height,
        })
    }, [])

    const findNearestIndex = useCallback((clientX: number, clientY: number) => {
        const canvas = canvasRef.current
        if (!canvas) return -1
        const rect = canvas.getBoundingClientRect()
        let bestIndex = -1
        let bestDistance = Number.POSITIVE_INFINITY
        layoutPoints.forEach((point, index) => {
            const screen = pointToCanvas(point, rect.width, rect.height)
            const distance = Math.hypot(screen.x - (clientX - rect.left), screen.y - (clientY - rect.top))
            if (distance < bestDistance) {
                bestDistance = distance
                bestIndex = index
            }
        })
        return bestDistance <= 18 ? bestIndex : -1
    }, [layoutPoints, pointToCanvas])

    const snapToSplinePoint = useCallback((point: TrackPoint): TrackPoint => {
        if (splinePoints.length === 0) return point
        let nearest = splinePoints[0]!
        let bestDistance = Number.POSITIVE_INFINITY
        splinePoints.forEach((candidate) => {
            const distance = Math.hypot(candidate.x - point.x, candidate.y - point.y)
            if (distance < bestDistance) {
                bestDistance = distance
                nearest = candidate
            }
        })
        return nearest
    }, [splinePoints])

    const draw = useCallback(() => {
        const canvas = canvasRef.current
        const container = containerRef.current
        if (!canvas || !container) return

        const ctx = canvas.getContext('2d')
        if (!ctx) return

        const rect = container.getBoundingClientRect()
        canvas.width = rect.width
        canvas.height = rect.height

        ctx.clearRect(0, 0, canvas.width, canvas.height)

        if (backgroundImageRef.current) {
            ctx.drawImage(backgroundImageRef.current, 0, 0, canvas.width, canvas.height)
            ctx.fillStyle = 'rgba(5, 10, 22, 0.28)'
            ctx.fillRect(0, 0, canvas.width, canvas.height)
        } else {
            ctx.fillStyle = 'rgba(255,255,255,0.02)'
            ctx.fillRect(0, 0, canvas.width, canvas.height)
        }

        if (splinePoints.length > 1) {
            ctx.beginPath()
            splinePoints.forEach((point, index) => {
                const screen = pointToCanvas(point, canvas.width, canvas.height)
                if (index === 0) ctx.moveTo(screen.x, screen.y)
                else ctx.lineTo(screen.x, screen.y)
            })
            ctx.closePath()
            ctx.strokeStyle = 'rgba(255,255,255,0.95)'
            ctx.lineWidth = 3
            ctx.shadowColor = 'rgba(74, 127, 247, 0.35)'
            ctx.shadowBlur = 10
            ctx.stroke()
            ctx.shadowBlur = 0
        }

        layoutPoints.forEach((point, index) => {
            const screen = pointToCanvas(point, canvas.width, canvas.height)
            ctx.beginPath()
            ctx.arc(screen.x, screen.y, 5, 0, Math.PI * 2)
            ctx.fillStyle = index === dragIndex ? '#f4d03f' : '#4a7ff7'
            ctx.fill()
            ctx.strokeStyle = '#ffffff'
            ctx.lineWidth = 1.2
            ctx.stroke()
        })

        checkpoints.forEach((checkpoint) => {
            const point = pointToCanvas(checkpoint.position, canvas.width, canvas.height)
            ctx.beginPath()
            ctx.arc(point.x, point.y, 4.5, 0, Math.PI * 2)
            ctx.fillStyle = checkpoint.type === 'finish' ? '#2ecc71' : '#f4d03f'
            ctx.fill()
            ctx.fillStyle = '#ffffff'
            ctx.font = '11px Inter'
            ctx.fillText(checkpoint.name, point.x + 8, point.y - 8)
        })

        const progressPoints = splinePoints.length > 1 ? splinePoints : layoutPoints
        racers.forEach((racer, index) => {
            let position: TrackPoint | null = null
            const direct = racer.track_position
            if (direct) {
                position = direct
            } else if (progressPoints.length > 0) {
                const baselineLap = Math.max(...racers.map((item) => item.current_lap), 1)
                const lapFraction = racer.best_lap_time && racer.best_lap_time > 0
                    ? Math.min(racer.current_lap_time / racer.best_lap_time, 1)
                    : 0
                const predictiveLead = racer.status === 'racing' ? Math.min(0.05, lapFraction * 0.08) : 0
                const progress = ((racer.current_lap + lapFraction + predictiveLead) % (baselineLap + 1)) / Math.max(baselineLap + 1, 1)
                const sampleIndex = Math.min(progressPoints.length - 1, Math.max(0, Math.round(progress * (progressPoints.length - 1))))
                position = progressPoints[sampleIndex]!
            }
            if (!position) return
            const screen = pointToCanvas(position, canvas.width, canvas.height)
            const color = racer.color || stringToColor(racer.name)

            ctx.beginPath()
            ctx.arc(screen.x, screen.y, 11, 0, Math.PI * 2)
            ctx.fillStyle = `${color}30`
            ctx.fill()

            ctx.beginPath()
            ctx.arc(screen.x, screen.y, 6.5, 0, Math.PI * 2)
            ctx.fillStyle = color
            ctx.fill()
            ctx.strokeStyle = '#fff'
            ctx.lineWidth = 1.4
            ctx.stroke()

            ctx.fillStyle = '#fff'
            ctx.font = '10px Inter'
            ctx.textAlign = 'center'
            ctx.fillText(`#${racer.number || index + 1}`, screen.x, screen.y - 12)
        })

        if (isEditMode) {
            ctx.fillStyle = 'rgba(5, 10, 22, 0.55)'
            ctx.fillRect(0, canvas.height - 28, canvas.width, 28)
            ctx.fillStyle = '#cbd5f5'
            ctx.font = '12px Inter'
            ctx.textAlign = 'center'
            const hint = editTool === 'spline'
                ? 'Click to add spline points. Drag blue handles to reshape the path.'
                : editTool === 'checkpoint'
                    ? 'Click to place checkpoints on the track.'
                    : `Click to set the ${editTool.toUpperCase()} marker.`
            ctx.fillText(hint, canvas.width / 2, canvas.height - 10)
        }
    }, [checkpoints, dragIndex, editTool, isEditMode, layoutPoints, pointToCanvas, racers, splinePoints])

    useEffect(() => {
        let animId: number
        const animate = () => {
            draw()
            animId = requestAnimationFrame(animate)
        }
        animId = requestAnimationFrame(animate)
        return () => cancelAnimationFrame(animId)
    }, [draw])

    const handlePointerDown = useCallback((event: ReactPointerEvent<HTMLCanvasElement>) => {
        if (!isEditMode) return

        if (editTool !== 'spline') {
            if (!onCheckpointUpdate) return
            const point = canvasToPoint(event.clientX, event.clientY)
            if (!point) return
            const snappedPoint = snapToSplinePoint(point)
            const nextType = editTool === 'start' ? 'start' : editTool === 'finish' ? 'finish' : 'checkpoint'
            const checkpoint: Checkpoint = {
                id: crypto.randomUUID(),
                track_id: '',
                name: nextType === 'checkpoint' ? `CP ${checkpoints.filter(item => item.type === 'checkpoint').length + 1}` : nextType.toUpperCase(),
                type: nextType,
                sort_order: checkpoints.length,
                position: snappedPoint,
            }
            const withoutSingle = nextType === 'checkpoint' ? checkpoints : checkpoints.filter(item => item.type !== nextType)
            onCheckpointUpdate([...withoutSingle, checkpoint])
            onMarkerPlaced?.(checkpoint)
            return
        }

        if (!onTrackUpdate) return
        const nearestIndex = findNearestIndex(event.clientX, event.clientY)
        if (nearestIndex >= 0) {
            setDragIndex(nearestIndex)
            return
        }
        const point = canvasToPoint(event.clientX, event.clientY)
        if (!point) return
        onTrackUpdate([...layoutPoints, point])
        setDragIndex(layoutPoints.length)
    }, [canvasToPoint, checkpoints, editTool, findNearestIndex, isEditMode, layoutPoints, onCheckpointUpdate, onMarkerPlaced, onTrackUpdate, snapToSplinePoint])

    const handlePointerMove = useCallback((event: ReactPointerEvent<HTMLCanvasElement>) => {
        if (!isEditMode || dragIndex == null || !onTrackUpdate) return
        const point = canvasToPoint(event.clientX, event.clientY)
        if (!point) return
        const next = layoutPoints.map((existing, index) => index === dragIndex ? point : existing)
        onTrackUpdate(next)
    }, [canvasToPoint, dragIndex, isEditMode, layoutPoints, onTrackUpdate])

    const handlePointerUp = useCallback(() => {
        setDragIndex(null)
    }, [])

    return (
        <div ref={containerRef} className="w-full h-full relative" style={{ minHeight: 160 }}>
            <canvas
                ref={canvasRef}
                className="absolute inset-0 w-full h-full cursor-crosshair"
                onPointerDown={handlePointerDown}
                onPointerMove={handlePointerMove}
                onPointerUp={handlePointerUp}
                onPointerLeave={handlePointerUp}
            />
        </div>
    )
}
