import { cleanup, render, screen, within } from '@testing-library/react'
import userEvent from '@testing-library/user-event'
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { useLabStore } from '../../state/labStore'
import { KinematicsPage } from './KinematicsPage'

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

describe('KinematicsPage', () => {
  it('updates the endpoint, transform, Jacobian, and scene revision from θ₂', async () => {
    const user = userEvent.setup()
    render(<KinematicsPage />)
    const endpoint = screen.getByTestId('endpoint-result')
    const before = endpoint.getAttribute('data-revision')

    await user.clear(screen.getByLabelText('关节角 θ₂'))
    await user.type(screen.getByLabelText('关节角 θ₂'), '30{Enter}')

    expect(endpoint).toHaveTextContent('m')
    const after = endpoint.getAttribute('data-revision')
    expect(after).not.toBe(before)
    expect(screen.getByTestId('transform-result')).toHaveAttribute('data-revision', after)
    expect(screen.getByTestId('jacobian-result')).toHaveAttribute('data-revision', after)
    expect(screen.getByTestId('scene-result')).toHaveAttribute('data-revision', after)
  })

  it('switches degree and radian display while preserving the represented pose', async () => {
    const user = userEvent.setup()
    render(<KinematicsPage />)
    const angle = screen.getByLabelText('关节角 θ₁')
    const qBefore = useLabStore.getState().jointState.q

    expect(angle).toHaveValue('30')
    expect(screen.getAllByText('°').length).toBeGreaterThan(0)
    await user.click(screen.getByRole('radio', { name: '弧度' }))

    expect(angle).toHaveValue('0.5236')
    expect(screen.getAllByText('rad').length).toBeGreaterThan(0)
    expect(useLabStore.getState().jointState.q).toEqual(qBefore)
  })

  it('shows and applies both inverse-kinematics elbow branches', async () => {
    const user = userEvent.setup()
    render(<KinematicsPage />)

    const solutions = screen.getByRole('group', { name: '逆运动学分支' })
    expect(within(solutions).getByRole('radio', { name: /肘下解/ })).toBeInTheDocument()
    expect(within(solutions).getByRole('radio', { name: /肘上解/ })).toBeInTheDocument()

    await user.click(within(solutions).getByRole('radio', { name: /肘上解/ }))
    await user.click(screen.getByRole('button', { name: '应用所选逆解' }))

    expect(useLabStore.getState().jointState.q[2]).toBeLessThanOrEqual(0)
  })

  it('keeps the last valid pose when the desired position is unreachable', async () => {
    const user = userEvent.setup()
    render(<KinematicsPage />)
    const qBefore = useLabStore.getState().jointState.q
    const revisionBefore = useLabStore.getState().calculation.revision

    await user.clear(screen.getByLabelText('期望位置 x'))
    await user.type(screen.getByLabelText('期望位置 x'), '99{Enter}')

    expect(screen.getByRole('alert')).toHaveTextContent('目标位置超出可达工作空间')
    expect(useLabStore.getState().jointState.q).toEqual(qBefore)
    expect(useLabStore.getState().calculation.revision).toBe(revisionBefore)
  })

  it('reports singularity metrics and warns at a straight-arm configuration', async () => {
    const user = userEvent.setup()
    render(<KinematicsPage />)

    for (const label of ['关节角 θ₁', '关节角 θ₂', '关节角 θ₃']) {
      const input = screen.getByLabelText(label)
      await user.clear(input)
      await user.type(input, '0{Enter}')
    }

    expect(screen.getByText('接近奇异位形')).toBeInTheDocument()
    expect(screen.getByText(/最小奇异值/)).toBeInTheDocument()
    expect(screen.getByText(/条件数/)).toBeInTheDocument()
    for (const bannedLabel of ['x_target', 'P_B^T', 'T_0^3', 'a_(i-1)']) {
      expect(document.body.textContent).not.toContain(bannedLabel)
    }
  })

  it('renders dynamic substitutions as KaTeX academic symbols', async () => {
    const user = userEvent.setup()
    render(<KinematicsPage />)
    const endpoint = screen.getByTestId('endpoint-result')

    await user.click(within(endpoint).getByRole('tab', { name: '代入' }))

    const panel = endpoint.querySelector('.formula-card__panel')
    expect(panel?.querySelector('.katex-error')).toBeNull()
    expect(panel?.querySelector('.katex')?.textContent).toContain('q')
  })
})
