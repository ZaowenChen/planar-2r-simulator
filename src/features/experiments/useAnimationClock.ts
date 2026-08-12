import { useEffect, useRef } from 'react'
import type { SimulationSample } from '../../robotics/integration'
import { useLabStore } from '../../state/labStore'

export function nearestSimulationSample(
  samples: readonly SimulationSample[],
  time: number,
): SimulationSample | undefined {
  if (samples.length === 0) return undefined
  let low = 0
  let high = samples.length - 1
  while (low < high) {
    const middle = Math.floor((low + high) / 2)
    if (samples[middle].time < time) low = middle + 1
    else high = middle
  }
  if (low === 0) return samples[0]
  const before = samples[low - 1]
  const after = samples[low]
  return time - before.time <= after.time - time ? before : after
}

export function useAnimationClock(samples: readonly SimulationSample[]): void {
  const isPlaying = useLabStore((state) => state.experiment.isPlaying)
  const playbackSpeed = useLabStore((state) => state.experiment.playbackSpeed)
  const setPlaying = useLabStore((state) => state.setPlaying)
  const setSimulationTime = useLabStore((state) => state.setSimulationTime)
  const frameRef = useRef<number | null>(null)
  const previousTimestamp = useRef<number | null>(null)
  const continuousTime = useRef(0)

  useEffect(() => {
    if (!isPlaying || samples.length === 0) return
    continuousTime.current = useLabStore.getState().simulationTime
    previousTimestamp.current = null
    const finalTime = samples[samples.length - 1].time

    const advance = (timestamp: number) => {
      if (previousTimestamp.current !== null) {
        continuousTime.current = Math.min(
          finalTime,
          continuousTime.current + (timestamp - previousTimestamp.current) / 1000 * playbackSpeed,
        )
        const sample = nearestSimulationSample(samples, continuousTime.current)
        if (sample !== undefined) setSimulationTime(sample.time)
        if (continuousTime.current >= finalTime) {
          setSimulationTime(finalTime)
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
  }, [isPlaying, playbackSpeed, samples, setPlaying, setSimulationTime])

  useEffect(() => () => useLabStore.getState().setPlaying(false), [])
}
