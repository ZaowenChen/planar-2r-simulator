import { useEffect, useRef } from 'react'
import { downloadTrajectoryCsv } from '../../export/download'
import type { TrajectoryPreview } from './trajectoryModel'
import { useTrajectoryStore } from './trajectoryStore'

function useTrajectoryClock(preview: TrajectoryPreview | null): void {
  const playing = useTrajectoryStore((state) => state.isPlaying)
  const speed = useTrajectoryStore((state) => state.playbackSpeed)
  const advanceTime = useTrajectoryStore((state) => state.advanceTime)
  const setPlaying = useTrajectoryStore((state) => state.setPlaying)
  const frameRef = useRef<number | null>(null)
  const previousTimestamp = useRef<number | null>(null)
  const continuousTime = useRef(0)

  useEffect(() => {
    if (!playing || preview === null) return
    continuousTime.current = useTrajectoryStore.getState().time
    previousTimestamp.current = null
    const duration = preview.instruction.duration
    const advance = (timestamp: number) => {
      if (previousTimestamp.current !== null) {
        continuousTime.current = Math.min(
          duration,
          continuousTime.current + (timestamp - previousTimestamp.current) / 1000 * speed,
        )
        advanceTime(continuousTime.current)
        if (continuousTime.current >= duration) {
          setPlaying(false)
          return
        }
      }
      previousTimestamp.current = timestamp
      frameRef.current = requestAnimationFrame(advance)
    }
    const pauseWhenHidden = () => {
      if (document.hidden) setPlaying(false)
    }
    document.addEventListener('visibilitychange', pauseWhenHidden)
    frameRef.current = requestAnimationFrame(advance)
    return () => {
      document.removeEventListener('visibilitychange', pauseWhenHidden)
      if (frameRef.current !== null) cancelAnimationFrame(frameRef.current)
      frameRef.current = null
      previousTimestamp.current = null
    }
  }, [advanceTime, playing, preview, setPlaying, speed])

  useEffect(() => () => useTrajectoryStore.getState().setPlaying(false), [])
}

export function TrajectoryPlayback({ preview }: { preview: TrajectoryPreview | null }) {
  const time = useTrajectoryStore((state) => state.time)
  const playing = useTrajectoryStore((state) => state.isPlaying)
  const speed = useTrajectoryStore((state) => state.playbackSpeed)
  const setTime = useTrajectoryStore((state) => state.setTime)
  const setPlaying = useTrajectoryStore((state) => state.setPlaying)
  const setSpeed = useTrajectoryStore((state) => state.setPlaybackSpeed)
  const stepForward = useTrajectoryStore((state) => state.stepForward)
  const resetPlayback = useTrajectoryStore((state) => state.resetPlayback)
  useTrajectoryClock(preview)

  const duration = preview?.instruction.duration ?? 1
  const startPlayback = () => {
    if (preview === null) return
    if (time >= duration) setTime(0)
    setPlaying(true)
  }

  return (
    <div className="trajectory-playback">
      <div className="trajectory-playback__controls">
        <button disabled={preview === null} onClick={playing ? () => setPlaying(false) : startPlayback} type="button">
          {playing ? '暂停' : '试运行'}
        </button>
        <button disabled={preview === null} onClick={stepForward} type="button">单步</button>
        <button disabled={preview === null} onClick={resetPlayback} type="button">复位到 P1</button>
        <label>播放倍率
          <select
            aria-label="轨迹播放倍率"
            onChange={(event) => setSpeed(Number(event.target.value))}
            value={speed}
          >
            <option value="0.25">0.25×</option>
            <option value="0.5">0.5×</option>
            <option value="1">1×</option>
            <option value="2">2×</option>
          </select>
        </label>
        <button
          disabled={preview === null}
          onClick={() => preview !== null && downloadTrajectoryCsv(preview.samples)}
          type="button"
        >导出轨迹 CSV</button>
      </div>
      <label className="trajectory-timeline">轨迹时间
        <input
          aria-label="轨迹时间"
          disabled={preview === null}
          max={duration}
          min="0"
          onChange={(event) => setTime(Number(event.target.value))}
          step={preview === null ? 0.01 : duration / (preview.samples.length - 1)}
          type="range"
          value={Math.min(time, duration)}
        />
        <output data-testid="trajectory-time">{time.toFixed(3)} / {preview === null ? '—' : duration.toFixed(3)} s</output>
      </label>
    </div>
  )
}
