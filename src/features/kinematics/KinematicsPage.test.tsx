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
  it('updates the calculation and resets the walkthrough when θ₂ changes', async () => {
    const user = userEvent.setup()
    render(<KinematicsPage />)
    const walkthrough = screen.getByTestId('kinematics-walkthrough')
    const before = walkthrough.getAttribute('data-revision')

    await user.click(screen.getByRole('button', { name: '下一步' }))
    expect(screen.getByTestId('walkthrough-step')).toHaveTextContent('第 2 / 9 步')

    await user.clear(screen.getByLabelText('关节角 θ₂'))
    await user.type(screen.getByLabelText('关节角 θ₂'), '30{Enter}')

    expect(useLabStore.getState().calculation.revision).toBe(Number(before) + 1)
    expect(screen.getByTestId('walkthrough-step')).toHaveTextContent('第 1 / 9 步')
    const after = walkthrough.getAttribute('data-revision')
    expect(after).toBe(String(Number(before) + 1))
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

  it('clears a stale angle error when a unit switch refreshes the draft', async () => {
    const user = userEvent.setup()
    render(<KinematicsPage />)
    const angle = screen.getByLabelText('关节角 θ₁')

    await user.clear(angle)
    await user.type(angle, 'invalid{Enter}')
    expect(screen.getByText('请输入有限角度。')).toBeInTheDocument()

    await user.click(screen.getByRole('radio', { name: '弧度' }))

    expect(angle).toHaveValue('0.5236')
    expect(screen.queryByText('请输入有限角度。')).not.toBeInTheDocument()
  })

  it('shows and applies both inverse-kinematics elbow branches', async () => {
    const user = userEvent.setup()
    render(<KinematicsPage />)

    const solutions = screen.getByRole('group', { name: '逆运动学分支' })
    expect(within(solutions).getByRole('radio', { name: /肘下解/ })).toBeInTheDocument()
    expect(within(solutions).getByRole('radio', { name: /肘上解/ })).toBeInTheDocument()

    await user.click(within(solutions).getByRole('radio', { name: /肘上解/ }))
    const revisionBefore = useLabStore.getState().calculation.revision
    const observedPoses: Array<readonly number[]> = []
    const unsubscribe = useLabStore.subscribe((state) => {
      observedPoses.push(state.jointState.q)
    })
    await user.click(screen.getByRole('button', { name: '应用所选逆解' }))
    unsubscribe()

    expect(useLabStore.getState().jointState.q[2]).toBeLessThanOrEqual(0)
    expect(useLabStore.getState().calculation.revision).toBe(revisionBefore + 1)
    expect(observedPoses).toEqual([useLabStore.getState().jointState.q])
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

    for (let step = 0; step < 6; step += 1) {
      await user.click(screen.getByRole('button', { name: '下一步' }))
    }

    expect(screen.getByText('接近奇异位形')).toBeInTheDocument()
    expect(screen.getByText(/最小奇异值/)).toBeInTheDocument()
    expect(screen.getByText(/条件数/)).toBeInTheDocument()
    for (const bannedLabel of ['x_target', 'P_B^T', 'T_0^3', 'a_(i-1)']) {
      expect(document.body.textContent).not.toContain(bannedLabel)
    }
  })

  it('shows one calculation step at a time and reaches the two inverse solutions in order', async () => {
    const user = userEvent.setup()
    render(<KinematicsPage />)

    expect(screen.getByRole('heading', { name: '角度输入与弧度转换' })).toBeInTheDocument()
    expect(screen.queryByRole('table', { name: '标准 D–H 参数表' })).not.toBeInTheDocument()
    await user.click(screen.getByRole('button', { name: '下一步' }))
    expect(screen.getByRole('table', { name: '标准 D–H 参数表' })).toBeInTheDocument()

    for (let step = 1; step < 8; step += 1) {
      await user.click(screen.getByRole('button', { name: '下一步' }))
    }

    expect(screen.getByRole('heading', { name: '比较肘下解与肘上解' })).toBeInTheDocument()
    expect(screen.getByText('肘下解（+）')).toBeInTheDocument()
    expect(screen.getByText('肘上解（−）')).toBeInTheDocument()
    expect(screen.getByRole('button', { name: '下一步' })).toBeDisabled()
    expect(document.querySelector('.katex-error')).toBeNull()
  })
})
