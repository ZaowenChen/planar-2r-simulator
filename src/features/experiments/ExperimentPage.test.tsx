import { act, cleanup, fireEvent, render, screen } from '@testing-library/react'
import userEvent from '@testing-library/user-event'
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { useLabStore } from '../../state/labStore'
import { ExperimentPage } from './ExperimentPage'

vi.mock('@react-three/fiber', async () => {
  const actual = await vi.importActual<typeof import('@react-three/fiber')>('@react-three/fiber')
  const React = await import('react')
  return {
    ...actual,
    Canvas: ({ children }: React.PropsWithChildren) => {
      const child = React.Children.toArray(children).find(React.isValidElement)
      const sceneModel = child && React.isValidElement<{ sceneModel?: unknown }>(child)
        ? child.props.sceneModel
        : undefined
      return <div data-scene-model={JSON.stringify(sceneModel)} data-testid="experiment-canvas" />
    },
  }
})

vi.mock('react-plotly.js/factory', () => ({
  default: () => ({ onClick }: { onClick?: (event: { points: { x: number }[] }) => void }) => (
    <button onClick={() => onClick?.({ points: [{ x: 0.25 }] })} type="button">曲线采样点 0.25 s</button>
  ),
}))

vi.mock('../../export/download', () => ({
  downloadSimulationCsv: vi.fn(),
  downloadPlotImage: vi.fn(),
}))

type FrameCallback = (timestamp: number) => void
let frames: FrameCallback[]

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
  vi.unstubAllGlobals()
})

function runNextFrame(timestamp: number): void {
  const frame = frames.shift()
  if (frame === undefined) throw new Error('No animation frame was scheduled')
  act(() => frame(timestamp))
}

describe('ExperimentPage', () => {
  it('generates inverse-dynamics torque samples and forward-dynamics state samples', async () => {
    const user = userEvent.setup()
    render(<ExperimentPage />)

    expect(screen.getByRole('tab', { name: '逆动力学' })).toHaveAttribute('aria-selected', 'true')
    expect(Number(screen.getByTestId('sample-count').textContent)).toBeGreaterThan(1)
    expect(screen.getByTestId('current-torque').textContent).toMatch(/N·m/)

    await user.click(screen.getByRole('tab', { name: '正动力学' }))
    await user.selectOptions(screen.getByLabelText('力矩输入类型'), 'sine')
    await user.click(screen.getByRole('button', { name: '生成实验' }))

    expect(useLabStore.getState().experiment.mode).toBe('forward')
    expect(Number(screen.getByTestId('sample-count').textContent)).toBeGreaterThan(1)
    expect(screen.getByTestId('current-state').textContent).toContain('rad')
  })

  it('plays, pauses, single-steps, resets, and clicks a chart through one shared time', async () => {
    const user = userEvent.setup()
    render(<ExperimentPage />)

    await user.click(screen.getByRole('button', { name: '播放' }))
    expect(useLabStore.getState().experiment.isPlaying).toBe(true)
    runNextFrame(1_000)
    runNextFrame(1_100)
    expect(useLabStore.getState().simulationTime).toBeCloseTo(0.1, 8)

    await user.click(screen.getByRole('button', { name: '暂停' }))
    expect(useLabStore.getState().experiment.isPlaying).toBe(false)
    const pausedTime = useLabStore.getState().simulationTime
    await user.click(screen.getByRole('button', { name: '单步' }))
    expect(useLabStore.getState().simulationTime).toBeCloseTo(pausedTime + 0.005, 8)

    await user.click(screen.getAllByRole('button', { name: '曲线采样点 0.25 s' })[0])
    expect(useLabStore.getState().simulationTime).toBe(0.25)
    await user.click(screen.getByRole('button', { name: '重置' }))
    expect(useLabStore.getState().simulationTime).toBe(0)
  })

  it('scrubs the scene and live formulas to the same precomputed sample', () => {
    render(<ExperimentPage />)
    const revisionBefore = Number(screen.getByTestId('experiment-scene').dataset.revision)

    fireEvent.change(screen.getByRole('slider', { name: '仿真时间' }), { target: { value: '0.5' } })

    const scene = screen.getByTestId('experiment-scene')
    const formula = screen.getByTestId('experiment-formula')
    expect(useLabStore.getState().simulationTime).toBe(0.5)
    expect(Number(scene.dataset.revision)).toBeGreaterThan(revisionBefore)
    expect(scene.dataset.revision).toBe(formula.dataset.revision)
    expect(scene.dataset.sampleTime).toBe('0.5')
    expect(formula.dataset.sampleTime).toBe('0.5')
  })

  it('pauses and preserves the exact structured diagnostic for an invalid simulation', () => {
    useLabStore.getState().setExperimentMode('forward')
    useLabStore.getState().setJointVector([Math.PI + 0.1, 0, 0])
    useLabStore.getState().setPlaying(true)

    render(<ExperimentPage />)

    expect(useLabStore.getState().experiment.isPlaying).toBe(false)
    expect(screen.getByRole('alert')).toHaveTextContent(
      '关节 1 超出限位：[-3.141592653589793, 3.141592653589793]，实际值 3.241592653589793',
    )
    expect(screen.getByTestId('sample-count')).toHaveTextContent('0')
  })
})
