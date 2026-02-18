import { useRef, useEffect, useCallback } from 'react'
import type { LiveRacer, TrackPoint, Checkpoint } from '@/types'
import { stringToColor } from '@/lib/utils'

interface TrackCanvasProps {
    racers: LiveRacer[]
    layoutPoints: TrackPoint[]
    checkpoints: Checkpoint[]
    isEditMode?: boolean
    onTrackUpdate?: (points: TrackPoint[]) => void
}

export default function TrackCanvas({
    racers,
    layoutPoints,
    checkpoints,
    isEditMode = false,
}: TrackCanvasProps) {
    const canvasRef = useRef<HTMLCanvasElement>(null)
    const containerRef = useRef<HTMLDivElement>(null)

    const draw = useCallback(() => {
        const canvas = canvasRef.current
        const container = containerRef.current
        if (!canvas || !container) return

        const ctx = canvas.getContext('2d')
        if (!ctx) return

        const rect = container.getBoundingClientRect()
        canvas.width = rect.width
        canvas.height = rect.height

        const cx = rect.width / 2
        const cy = rect.height / 2
        const rx = (rect.width - 80) / 2
        const ry = (rect.height - 40) / 2

        // Clear
        ctx.clearRect(0, 0, canvas.width, canvas.height)

        // Draw track surface (filled ellipse)
        ctx.beginPath()
        ctx.ellipse(cx, cy, rx + 18, ry + 18, 0, 0, Math.PI * 2)
        ctx.fillStyle = 'rgba(255,255,255,0.02)'
        ctx.fill()

        // Outer edge
        ctx.beginPath()
        ctx.ellipse(cx, cy, rx + 18, ry + 18, 0, 0, Math.PI * 2)
        ctx.strokeStyle = 'rgba(255,255,255,0.15)'
        ctx.lineWidth = 1.5
        ctx.stroke()

        // Inner edge
        ctx.beginPath()
        ctx.ellipse(cx, cy, rx - 18, ry - 18, 0, 0, Math.PI * 2)
        ctx.strokeStyle = 'rgba(255,255,255,0.15)'
        ctx.lineWidth = 1.5
        ctx.stroke()

        // Center line (track path)
        ctx.beginPath()
        ctx.ellipse(cx, cy, rx, ry, 0, 0, Math.PI * 2)
        ctx.strokeStyle = 'rgba(255,255,255,0.06)'
        ctx.lineWidth = 1
        ctx.setLineDash([8, 6])
        ctx.stroke()
        ctx.setLineDash([])

        // Start/finish line
        const sfAngle = -Math.PI / 2 // top
        const sfInnerX = cx + (rx - 18) * Math.cos(sfAngle)
        const sfInnerY = cy + (ry - 18) * Math.sin(sfAngle)
        const sfOuterX = cx + (rx + 18) * Math.cos(sfAngle)
        const sfOuterY = cy + (ry + 18) * Math.sin(sfAngle)
        ctx.beginPath()
        ctx.moveTo(sfInnerX, sfInnerY)
        ctx.lineTo(sfOuterX, sfOuterY)
        ctx.strokeStyle = '#2ecc71'
        ctx.lineWidth = 3
        ctx.stroke()

        // Draw checkpoints
        checkpoints.forEach((cp) => {
            if (cp.type === 'start' || cp.type === 'finish') return
            const angle = Math.atan2(cp.position.y - cy, cp.position.x - cx)
            const innerX = cx + (rx - 18) * Math.cos(angle)
            const innerY = cy + (ry - 18) * Math.sin(angle)
            const outerX = cx + (rx + 18) * Math.cos(angle)
            const outerY = cy + (ry + 18) * Math.sin(angle)
            ctx.beginPath()
            ctx.moveTo(innerX, innerY)
            ctx.lineTo(outerX, outerY)
            ctx.strokeStyle = 'rgba(244, 208, 63, 0.5)'
            ctx.lineWidth = 2
            ctx.stroke()
        })

        // Draw racers
        racers.forEach((racer) => {
            // Use track position if available, otherwise calculate from progress
            let x: number, y: number
            if (racer.track_position) {
                x = racer.track_position.x
                y = racer.track_position.y
            } else {
                // Estimate position based on lap progress
                const totalProgress = (racer.current_lap + (racer.current_lap_time / (racer.best_lap_time || 60000))) / (racers.length > 0 ? Math.max(...racers.map(r => r.current_lap), 1) + 1 : 1)
                const angle = -Math.PI / 2 + totalProgress * Math.PI * 2
                x = cx + rx * Math.cos(angle)
                y = cy + ry * Math.sin(angle)
            }

            const color = racer.color || stringToColor(racer.name)

            // Glow
            ctx.beginPath()
            ctx.arc(x, y, 10, 0, Math.PI * 2)
            ctx.fillStyle = color + '30'
            ctx.fill()

            // Dot
            ctx.beginPath()
            ctx.arc(x, y, 6, 0, Math.PI * 2)
            ctx.fillStyle = color
            ctx.fill()
            ctx.strokeStyle = '#fff'
            ctx.lineWidth = 1.5
            ctx.stroke()

            // Number label
            ctx.fillStyle = '#fff'
            ctx.font = '10px Inter'
            ctx.textAlign = 'center'
            ctx.fillText(`#${racer.number}`, x, y - 12)
        })

        // Edit mode indicator
        if (isEditMode) {
            ctx.fillStyle = 'rgba(74, 127, 247, 0.08)'
            ctx.fillRect(0, 0, canvas.width, canvas.height)
            ctx.fillStyle = 'var(--color-accent-blue)'
            ctx.font = '12px Inter'
            ctx.textAlign = 'center'
            ctx.fillText('Click to place checkpoints', cx, canvas.height - 12)
        }
    }, [racers, layoutPoints, checkpoints, isEditMode])

    // Animation loop
    useEffect(() => {
        let animId: number
        const animate = () => {
            draw()
            animId = requestAnimationFrame(animate)
        }
        animId = requestAnimationFrame(animate)
        return () => cancelAnimationFrame(animId)
    }, [draw])

    return (
        <div ref={containerRef} className="w-full h-full relative" style={{ minHeight: 160 }}>
            <canvas
                ref={canvasRef}
                className="absolute inset-0 w-full h-full"
            />
        </div>
    )
}
