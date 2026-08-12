import { act, cleanup, render } from '@testing-library/react'
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import type { SimulationSample } from '../../robotics/integration'
import { useLabStore } from '../../state/labStore'
import { useAnimationClock } from './useAnimationClock'

type FrameCallback = (timestamp: number) => void
let frames: FrameCallback[]

const samples: SimulationSample[] = [0, 0.5, 1].map((time) => ({
  time,
  q: [time, 0, 0], qd: [0, 0, 0], qdd: [0, 0, 0], tau: [0, 0, 0],
  kinetic: 0, potential: 0, totalEnergy: 0, jointPower: [0, 0, 0],
}))

function ClockHarness() {
  useAnimationClock(samples)
  return null
}

beforeEach(() => {
  useLabStore.getState().resetLab()
  frames = []
  vi.stubGlobal('requestAnimationFrame', vi.fn((callback: FrameCallback) => {
    frames.push(callback)
    return frames.length
  }))
  vi.stubGlobal('cancelAnimationFrame', vi.fn())
})

afterEach(() => {
  cleanup()
  Object.defineProperty(document, 'hidden', { configurable: true, value: false })
  vi.unstubAllGlobals()
})

function runFrame(timestamp: number): void {
  const frame = frames.shift()
  if (frame === undefined) throw new Error('No animation frame scheduled')
  act(() => frame(timestamp))
}

describe('useAnimationClock', () => {
  it('pauses immediately when the document becomes hidden', () => {
    useLabStore.getState().setPlaying(true)
    render(<ClockHarness />)

    Object.defineProperty(document, 'hidden', { configurable: true, value: true })
    act(() => document.dispatchEvent(new Event('visibilitychange')))

    expect(useLabStore.getState().experiment.isPlaying).toBe(false)
    expect(cancelAnimationFrame).toHaveBeenCalled()
  })

  it('selects the exact final sample and stops at the sample duration', () => {
    useLabStore.getState().setSimulationTime(0.9)
    useLabStore.getState().setPlaying(true)
    render(<ClockHarness />)

    runFrame(1_000)
    runFrame(1_200)

    expect(useLabStore.getState().simulationTime).toBe(1)
    expect(useLabStore.getState().experiment.isPlaying).toBe(false)
    expect(frames).toHaveLength(0)
  })
})
