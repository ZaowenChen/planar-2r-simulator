import { act, cleanup, fireEvent, render, screen } from '@testing-library/react'
import userEvent from '@testing-library/user-event'
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { useLabStore } from '../../state/labStore'
import { useTrajectoryStore } from '../trajectory/trajectoryStore'
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
      return <div data-scene-model={JSON.stringify(sceneModel)} data-testid="trajectory-canvas" />
    },
  }
})

vi.mock('react-plotly.js/factory', () => ({
  default: () => ({
    onClick,
    layout,
  }: {
    onClick?: (event: { points: { x: number }[] }) => void
    layout?: { title?: { text?: string } }
  }) => (
    <button
      aria-label={layout?.title?.text ?? '轨迹曲线'}
      onClick={() => onClick?.({ points: [{ x: 0.25 }] })}
      type="button"
    >曲线采样点</button>
  ),
}))

vi.mock('../../export/download', () => ({
  downloadTrajectoryCsv: vi.fn(),
  downloadPlotImage: vi.fn(),
}))

type FrameCallback = (timestamp: number) => void
let frames: FrameCallback[]

beforeEach(() => {
  useLabStore.getState().resetLab()
  useTrajectoryStore.getState().reset(useLabStore.getState().jointState.q)
  frames = []
  vi.stubGlobal('requestAnimationFrame', vi.fn((callback: FrameCallback) => {
    frames.push(callback)
    return frames.length
  }))
  vi.stubGlobal('cancelAnimationFrame', vi.fn())
  vi.spyOn(window, 'confirm').mockReturnValue(true)
})

afterEach(() => {
  cleanup()
  vi.restoreAllMocks()
  vi.unstubAllGlobals()
})

function runNextFrame(timestamp: number): void {
  const frame = frames.shift()
  if (frame === undefined) throw new Error('No animation frame was scheduled')
  act(() => frame(timestamp))
}

async function createPreview(profile: 'quintic' | 'trapezoidal' = 'quintic') {
  const user = userEvent.setup()
  await user.click(screen.getByRole('button', { name: '记录当前位置为示教点' }))
  await user.click(screen.getByRole('button', { name: 'J1 正向点动' }))
  await user.click(screen.getByRole('button', { name: '记录当前位置为示教点' }))
  if (profile === 'trapezoidal') {
    await user.selectOptions(screen.getByLabelText('PTP 轨迹类型'), 'trapezoidal')
  }
  await user.click(screen.getByRole('button', { name: '生成轨迹预览' }))
  return user
}

describe('trajectory teaching ExperimentPage', () => {
  it('renders a pure-trajectory teach pendant without dynamics controls or quantities', () => {
    render(<ExperimentPage />)

    expect(screen.getByRole('region', { name: '轨迹三维示教视图' })).toBeInTheDocument()
    expect(screen.getByRole('region', { name: '虚拟示教器' })).toBeInTheDocument()
    expect(screen.getByRole('region', { name: '轨迹数学讲解' })).toBeInTheDocument()
    expect(screen.getByRole('region', { name: '轨迹预览图表' })).toBeInTheDocument()
    expect(screen.getByText('等待 PTP 轨迹')).toBeInTheDocument()
    expect(screen.queryByText(/逆动力学|正动力学|力矩|能量|功率/)).not.toBeInTheDocument()
  })

  it('records copied P1/P2 poses, generates a quintic PTP, and exposes polynomial charts', async () => {
    render(<ExperimentPage />)
    await createPreview()

    expect(screen.getByLabelText('示教点数量')).toHaveTextContent('2 点')
    expect(screen.getByLabelText('P1 名称')).toHaveValue('P1')
    expect(screen.getByLabelText('P2 名称')).toHaveValue('P2')
    expect(screen.getByText('PTP P1 → P2')).toBeInTheDocument()
    expect(screen.getByRole('heading', { name: '五次多项式轨迹' })).toBeInTheDocument()
    expect(screen.getByRole('heading', { name: '五次插值公式' })).toBeInTheDocument()
    expect(screen.getByRole('tab', { name: '位置 q' })).toHaveAttribute('aria-selected', 'true')
    expect(screen.getByRole('button', { name: '归一化插值 s / s′ / s″' })).toBeInTheDocument()
    expect(screen.getByText('峰值速度')).toBeInTheDocument()
  })

  it('updates a committed trajectory immediately when duration or profile changes', async () => {
    render(<ExperimentPage />)
    const user = await createPreview()
    const peakBefore = screen.getByText('峰值速度').parentElement?.textContent

    await user.clear(screen.getByLabelText('PTP 持续时间'))
    await user.type(screen.getByLabelText('PTP 持续时间'), '10')
    expect(screen.getByText('峰值速度').parentElement?.textContent).not.toBe(peakBefore)

    await user.selectOptions(screen.getByLabelText('PTP 轨迹类型'), 'trapezoidal')
    expect(screen.getByRole('heading', { name: '梯形速度轨迹' })).toBeInTheDocument()
    expect(screen.getByLabelText('梯形速度阶段')).toBeInTheDocument()
    expect(screen.getByText(/加速度会从 4\.5 跳到 0/)).toBeInTheDocument()
  })

  it('plays, pauses, steps, resets, and clicks both charts through one shared time', async () => {
    render(<ExperimentPage />)
    const user = await createPreview()

    await user.click(screen.getByRole('button', { name: '试运行' }))
    expect(useTrajectoryStore.getState().isPlaying).toBe(true)
    runNextFrame(1_000)
    runNextFrame(1_100)
    expect(useTrajectoryStore.getState().time).toBeCloseTo(0.1, 6)

    await user.click(screen.getByRole('button', { name: '暂停' }))
    const paused = useTrajectoryStore.getState().time
    await user.click(screen.getByRole('button', { name: '单步' }))
    expect(useTrajectoryStore.getState().time).toBeGreaterThan(paused)

    await user.click(screen.getByRole('button', { name: '归一化插值 s / s′ / s″' }))
    expect(useTrajectoryStore.getState().time).toBeCloseTo(1.25, 8)
    await user.click(screen.getByRole('button', { name: '复位到 P1' }))
    expect(useTrajectoryStore.getState().time).toBe(0)
  })

  it('scrubs the 3D scene, formula and graph cursor to the same analytical time', async () => {
    render(<ExperimentPage />)
    await createPreview()

    fireEvent.change(screen.getByRole('slider', { name: '轨迹时间' }), { target: { value: '2.5' } })

    expect(useTrajectoryStore.getState().time).toBe(2.5)
    expect(screen.getByTestId('trajectory-time')).toHaveTextContent('2.500 / 5.000 s')
    expect(screen.getByText('u = 0.500')).toBeInTheDocument()
    expect(screen.getByText(/J1 30\.5°/)).toBeInTheDocument()
  })

  it('expands the 3D scene without changing the trajectory time', async () => {
    render(<ExperimentPage />)
    const user = await createPreview()
    fireEvent.change(screen.getByRole('slider', { name: '轨迹时间' }), { target: { value: '1.5' } })

    await user.click(screen.getByRole('button', { name: '放大 3D' }))
    expect(document.querySelector('.trajectory-workbench')).toHaveClass('is-scene-expanded')
    expect(useTrajectoryStore.getState().time).toBe(1.5)
    expect(screen.getByRole('button', { name: '退出放大' })).toBeInTheDocument()
  })
})
