import { act, cleanup, render, screen } from '@testing-library/react'
import userEvent from '@testing-library/user-event'
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { useLabStore } from '../../state/labStore'
import { RobotModelPage } from './RobotModelPage'

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
      return <div data-scene-model={JSON.stringify(sceneModel)} data-testid="canvas-boundary" />
    },
  }
})

beforeEach(() => useLabStore.getState().resetLab())
afterEach(cleanup)

describe('RobotModelPage', () => {
  it('renders geometry, environment, frames, and standard D–H notation without source labels', () => {
    render(<RobotModelPage />)

    expect(screen.getByRole('heading', { name: '几何与环境参数' })).toBeInTheDocument()
    expect(screen.getByRole('heading', { name: '坐标系定义' })).toBeInTheDocument()
    expect(screen.getByRole('table', { name: '标准 D–H 参数表' })).toBeInTheDocument()
    expect(screen.getByLabelText('基座高度 d₁')).toHaveAccessibleDescription('mm')
    expect(screen.getByLabelText('基座高度 d₁')).toHaveValue('800')
    expect(screen.getByLabelText('第二连杆长度 l₂')).toHaveAccessibleDescription('mm')
    expect(screen.getByLabelText('第二连杆长度 l₂')).toHaveValue('2000')

    const dhTable = screen.getByRole('table', { name: '标准 D–H 参数表' })
    expect(dhTable).toHaveTextContent('(mm)')
    expect(dhTable).toHaveTextContent('(°)')
    expect(dhTable).toHaveTextContent('800.000')

    const katexText = Array.from(document.querySelectorAll('.katex'))
      .map((node) => node.textContent)
      .join(' ')
    for (const heading of ['θi', 'di', 'ai', 'αi']) {
      expect(katexText.replaceAll('−', '-')).toContain(heading)
    }
    for (const bannedLabel of ['x_target', 'P_B^T', 'T_0^3', 'a_(i-1)']) {
      expect(document.body.textContent).not.toContain(bannedLabel)
    }
  })

  it('commits parameter edits on blur or Enter and preserves the last valid model', async () => {
    const user = userEvent.setup()
    render(<RobotModelPage />)
    const input = screen.getByLabelText('第二连杆长度 l₂')
    const before = useLabStore.getState().calculation.revision

    await user.clear(input)
    await user.type(input, '0')
    expect(useLabStore.getState().calculation.revision).toBe(before)
    await user.tab()

    expect(screen.getByText('连杆长度必须为正数。')).toBeInTheDocument()
    expect(useLabStore.getState().parameters.geometry.l2).toBe(2)
    expect(useLabStore.getState().calculation.revision).toBe(before)

    await user.clear(input)
    await user.type(input, '2400{Enter}')

    expect(screen.queryByText('连杆长度必须为正数。')).not.toBeInTheDocument()
    expect(useLabStore.getState().parameters.geometry.l2).toBe(2.4)
    expect(input).toHaveValue('2400')
    expect(useLabStore.getState().calculation.revision).not.toBe(before)
  })

  it('clears a stale local error when an external valid parameter refreshes the draft', async () => {
    const user = userEvent.setup()
    render(<RobotModelPage />)
    const input = screen.getByLabelText('第二连杆长度 l₂')

    await user.clear(input)
    await user.type(input, '0{Enter}')
    expect(screen.getByText('连杆长度必须为正数。')).toBeInTheDocument()

    act(() => useLabStore.getState().setParameterField('geometry.l2', '2.6'))

    expect(input).toHaveValue('2600')
    expect(screen.queryByText('连杆长度必须为正数。')).not.toBeInTheDocument()
  })
})
