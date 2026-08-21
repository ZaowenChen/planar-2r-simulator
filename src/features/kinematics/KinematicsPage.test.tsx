import { act, cleanup, render, screen, within } from '@testing-library/react'
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

async function openParameterEditor(user: ReturnType<typeof userEvent.setup>) {
  await user.click(screen.getByRole('button', { name: /修改(?:关节角|目标 \/ 选择逆解|运动状态)/ }))
  expect(screen.getByRole('region', { name: '运动学参数编辑' })).toBeInTheDocument()
}

async function selectMode(
  user: ReturnType<typeof userEvent.setup>,
  name: '正运动学' | '位置逆运动学' | '微分运动学',
) {
  await user.click(screen.getByRole('tab', { name: new RegExp(`^${name}`) }))
}

describe('KinematicsPage', () => {
  it('updates the calculation without resetting the walkthrough when θ₂ changes', async () => {
    const user = userEvent.setup()
    render(<KinematicsPage />)
    const walkthrough = screen.getByTestId('kinematics-walkthrough')
    const before = walkthrough.getAttribute('data-revision')

    await user.click(screen.getByRole('button', { name: '下一步' }))
    expect(screen.getByTestId('walkthrough-step')).toHaveTextContent('第 2 / 6 步')

    await openParameterEditor(user)
    await user.clear(screen.getByLabelText('关节角 θ₂'))
    await user.type(screen.getByLabelText('关节角 θ₂'), '30{Enter}')

    expect(useLabStore.getState().calculation.revision).toBe(Number(before) + 1)
    expect(screen.getByTestId('walkthrough-step')).toHaveTextContent('第 2 / 6 步')
    const after = walkthrough.getAttribute('data-revision')
    expect(after).toBe(String(Number(before) + 1))
    expect(screen.getByTestId('scene-result')).toHaveAttribute('data-revision', after)
  })

  it('keeps parameters collapsed in the summary and uses degrees for edits', async () => {
    const user = userEvent.setup()
    render(<KinematicsPage />)
    expect(screen.getByText(/θ₁ 30\.0°/)).toBeInTheDocument()
    expect(screen.queryByLabelText('关节角 θ₁')).not.toBeInTheDocument()

    await openParameterEditor(user)
    const angle = screen.getByLabelText('关节角 θ₁')
    const qBefore = useLabStore.getState().jointState.q

    expect(screen.queryByLabelText('期望位置 x')).not.toBeInTheDocument()
    expect(angle).toHaveValue('30')
    expect(screen.getAllByText('°').length).toBeGreaterThan(0)
    expect(screen.queryByRole('radio', { name: '弧度' })).not.toBeInTheDocument()
    expect(useLabStore.getState().jointState.q).toEqual(qBefore)
  })

  it('clears a stale angle error when an external pose refreshes the degree draft', async () => {
    const user = userEvent.setup()
    render(<KinematicsPage />)
    await openParameterEditor(user)
    const angle = screen.getByLabelText('关节角 θ₁')

    await user.clear(angle)
    await user.type(angle, 'invalid{Enter}')
    expect(screen.getByText('请输入有限角度。')).toBeInTheDocument()

    act(() => useLabStore.getState().setJoint(0, Math.PI / 4))

    expect(angle).toHaveValue('45')
    expect(screen.queryByText('请输入有限角度。')).not.toBeInTheDocument()
  })

  it('shows desired positions in millimetres and converts edits to internal metres', async () => {
    const user = userEvent.setup()
    render(<KinematicsPage />)
    await selectMode(user, '位置逆运动学')
    await openParameterEditor(user)
    const targetX = screen.getByLabelText('期望位置 x')

    expect(screen.queryByLabelText('关节角 θ₁')).not.toBeInTheDocument()
    expect(targetX).toHaveAccessibleDescription('mm')
    expect(targetX).toHaveValue('2747.099')

    await user.clear(targetX)
    await user.type(targetX, '3000{Enter}')

    expect(useLabStore.getState().desiredPosition[0]).toBe(3)
    expect(targetX).toHaveValue('3000')
  })

  it('shows and applies both inverse-kinematics elbow branches', async () => {
    const user = userEvent.setup()
    render(<KinematicsPage />)
    await selectMode(user, '位置逆运动学')
    await openParameterEditor(user)

    const solutions = screen.getByRole('group', { name: '逆运动学构型' })
    expect(within(solutions).getByRole('radio', { name: /肘下构型/ })).toBeInTheDocument()
    expect(within(solutions).getByRole('radio', { name: /肘上构型/ })).toBeInTheDocument()

    await user.click(within(solutions).getByRole('radio', { name: /肘上构型/ }))
    const revisionBefore = useLabStore.getState().calculation.revision
    const observedPoses: Array<readonly number[]> = []
    const unsubscribe = useLabStore.subscribe((state) => {
      observedPoses.push(state.jointState.q)
    })
    await user.click(screen.getByRole('button', { name: '应用当前预览解' }))
    unsubscribe()

    expect(useLabStore.getState().jointState.q[2]).toBeLessThanOrEqual(0)
    expect(useLabStore.getState().calculation.revision).toBe(revisionBefore + 1)
    expect(observedPoses).toEqual([useLabStore.getState().jointState.q])
  })

  it('keeps the D–H table, four-stage transform, and 3D teaching focus synchronized', async () => {
    const user = userEvent.setup()
    render(<KinematicsPage />)

    expect(screen.getByTestId('walkthrough-step')).toHaveTextContent('第 1 / 6 步')

    await user.click(screen.getByRole('button', { name: '下一步' }))
    expect(screen.getByRole('heading', { name: '把机构写成标准 D–H 参数' })).toBeInTheDocument()

    await user.click(screen.getByRole('button', { name: '沿 x₂ 平移 a₂' }))
    const workspace = screen.getByTestId('kinematics-workspace')
    expect(workspace).toHaveAttribute('data-dh-row', '2')
    expect(workspace).toHaveAttribute('data-dh-operation', 'tx')

    await user.click(screen.getByRole('button', { name: '下一步' }))
    expect(screen.getByRole('heading', { name: '写出三个关节齐次变换' })).toBeInTheDocument()
    expect(screen.getByText('当前子步骤：沿 x₂ 平移 a₂')).toBeInTheDocument()
    expect(screen.getByRole('button', { name: '沿 x₂ 平移 a₂' })).toHaveAttribute('aria-pressed', 'true')

    await user.click(screen.getByRole('button', { name: '下一个子步骤' }))
    expect(workspace).toHaveAttribute('data-dh-operation', 'rx')
    expect(screen.getByText('当前子步骤：绕 x₂ 旋转 α₂')).toBeInTheDocument()
  })

  it('keeps the last valid pose when the desired position is unreachable', async () => {
    const user = userEvent.setup()
    render(<KinematicsPage />)
    const qBefore = useLabStore.getState().jointState.q
    const revisionBefore = useLabStore.getState().calculation.revision

    await selectMode(user, '位置逆运动学')
    await openParameterEditor(user)
    await user.clear(screen.getByLabelText('期望位置 x'))
    await user.type(screen.getByLabelText('期望位置 x'), '99000{Enter}')

    expect(screen.getByRole('alert')).toHaveTextContent('目标位置超出可达工作空间')
    expect(useLabStore.getState().jointState.q).toEqual(qBefore)
    expect(useLabStore.getState().calculation.revision).toBe(revisionBefore)

    await user.click(screen.getByRole('button', { name: '收起参数' }))
    await user.click(screen.getByRole('button', { name: '下一步' }))
    expect(screen.getByText('目标不可达')).toBeInTheDocument()
    expect(screen.getByText(/超出连杆能够构成的三角形范围/)).toBeInTheDocument()
  })

  it('separates a joint-limit result from geometric reachability and keeps the derivation available', async () => {
    const user = userEvent.setup()
    render(<KinematicsPage />)
    act(() => useLabStore.getState().setDesiredPositionVector([0.75, 0, 0.8]))

    await selectMode(user, '位置逆运动学')
    await user.click(screen.getByRole('button', { name: '下一步' }))
    expect(screen.getByText('几何可达，但超出关节限位')).toBeInTheDocument()
    expect(screen.getByText(/关节限位过滤了可应用的候选构型/)).toBeInTheDocument()

    await user.click(screen.getByRole('button', { name: '下一步' }))
    await user.click(screen.getByRole('button', { name: /查看完整计算过程/ }))
    expect(screen.getByTestId('ik-calculation-card-1')).toHaveTextContent('肘角 θ₃')
    expect(screen.getByTestId('ik-calculation-card-4')).toHaveTextContent('肩角 θ₂')

    await user.click(screen.getByRole('button', { name: '下一步' }))
    expect(within(screen.getByRole('group', { name: '肘上肘下构型选择' }))
      .getAllByText('仅教学图示')).toHaveLength(2)
    await user.click(screen.getByRole('button', { name: '下一步' }))
    expect(screen.getByRole('button', { name: '应用逆解' })).toBeDisabled()
  })

  it('reports singularity metrics and warns at a straight-arm configuration', async () => {
    const user = userEvent.setup()
    render(<KinematicsPage />)
    await openParameterEditor(user)

    for (const label of ['关节角 θ₁', '关节角 θ₂', '关节角 θ₃']) {
      const input = screen.getByLabelText(label)
      await user.clear(input)
      await user.type(input, '0{Enter}')
    }

    await selectMode(user, '微分运动学')

    for (let step = 0; step < 3; step += 1) {
      await user.click(screen.getByRole('button', { name: '下一步' }))
    }

    expect(screen.getByTestId('walkthrough-step')).toHaveTextContent('第 4 / 4 步')
    expect(screen.getByText('处于位置奇异位形')).toBeInTheDocument()
    expect(screen.getByText(/最小奇异值/)).toBeInTheDocument()
    expect(screen.getByText(/条件数/)).toBeInTheDocument()
    for (const bannedLabel of ['x_target', 'P_B^T', 'T_0^3', 'a_(i-1)']) {
      expect(document.body.textContent).not.toContain(bannedLabel)
    }
  })

  it('separates forward input, position inverse input, and geometric Jacobian workflows', async () => {
    const user = userEvent.setup()
    render(<KinematicsPage />)

    expect(screen.getByRole('tab', { name: /^正运动学/ })).toHaveAttribute('aria-selected', 'true')
    expect(screen.getByRole('tab', { name: /^位置逆运动学/ })).toHaveAttribute('aria-selected', 'false')
    expect(screen.getByRole('heading', { name: '先识别机构与关节轴' })).toBeInTheDocument()
    expect(screen.getByTestId('walkthrough-method')).toHaveTextContent('正运动学 · D–H 矩阵法')
    expect(screen.getByTestId('walkthrough-step')).toHaveTextContent('第 1 / 6 步')
    expect(screen.getByText('输入关节角 q')).toBeInTheDocument()
    expect(screen.queryByText('输入目标位置')).not.toBeInTheDocument()
    expect(screen.queryByText('当前数值代入')).not.toBeInTheDocument()
    expect(document.body.textContent).not.toContain('rad')
    expect(Array.from(document.querySelectorAll('.katex-html'))
      .map((node) => node.textContent)
      .join('')).not.toContain('\\')
    await user.click(screen.getByRole('button', { name: '下一步' }))
    expect(screen.getByRole('table', { name: '标准 D–H 参数表' })).toBeInTheDocument()
    expect(screen.getByRole('table', { name: '标准 D–H 参数表' })).toHaveTextContent('(mm)')

    for (let step = 0; step < 3; step += 1) {
      await user.click(screen.getByRole('button', { name: '下一步' }))
    }
    expect(screen.getByRole('heading', { name: '从变换矩阵提取末端位置' })).toBeInTheDocument()
    expect(screen.getByTestId('endpoint-result')).toHaveTextContent('mm')

    await user.click(screen.getByRole('button', { name: '下一步' }))
    expect(screen.getByRole('heading', { name: '提取末端姿态' })).toBeInTheDocument()
    expect(screen.getByRole('table', { name: '末端姿态旋转矩阵' })).toBeInTheDocument()
    expect(screen.getByText(/末端连杆仰角/)).toBeInTheDocument()
    expect(screen.getByRole('button', { name: '下一步' })).toBeDisabled()

    await selectMode(user, '位置逆运动学')
    expect(screen.getByRole('heading', { name: '设置目标' })).toBeInTheDocument()
    expect(screen.getByTestId('walkthrough-method')).toHaveTextContent('逆运动学 · 解析几何法')
    expect(screen.getByTestId('walkthrough-step')).toHaveTextContent('第 1 / 5 步')
    expect(screen.getByText((_, element) => (
      element?.tagName === 'DT' && element.textContent?.startsWith('输入目标位置') === true
    ))).toBeInTheDocument()
    expect(screen.queryByText('输入关节角 q')).not.toBeInTheDocument()
    const inverseStepper = screen.getByRole('navigation', { name: '位置逆运动学推导步骤' })
    expect(within(inverseStepper).getAllByRole('button')).toHaveLength(5)
    expect(screen.getByText(/工作平面目标坐标为/)).toBeInTheDocument()

    await user.click(screen.getByRole('button', { name: '下一步' }))
    expect(screen.getByRole('heading', { name: '判断可达' })).toBeInTheDocument()
    expect(screen.getByText('目标可达')).toBeInTheDocument()
    expect(screen.getByText(/两根连杆是否能组成三角形/)).toBeInTheDocument()

    await user.click(screen.getByRole('button', { name: '下一步' }))
    expect(screen.getByRole('heading', { name: '求关节角' })).toBeInTheDocument()
    const calculationDisclosure = screen.getByRole('button', { name: /查看完整计算过程/ })
    expect(calculationDisclosure).toHaveAttribute('aria-expanded', 'false')
    expect(screen.queryByRole('region', { name: '完整计算过程' })).not.toBeInTheDocument()
    await user.click(calculationDisclosure)
    expect(calculationDisclosure).toHaveAttribute('aria-expanded', 'true')
    expect(screen.getByRole('region', { name: '完整计算过程' })).toBeInTheDocument()
    expect(screen.getByTestId('ik-calculation-card-1')).toHaveTextContent('肘角 θ₃')
    expect(screen.getByTestId('ik-calculation-card-2')).toHaveTextContent('方向角 γ')
    expect(screen.getByTestId('ik-calculation-card-3')).toHaveTextContent('补偿角 δ')
    expect(screen.getByTestId('ik-calculation-card-4')).toHaveTextContent('肩角 θ₂')
    expect(screen.getByTestId('ik-calculation-card-1')).toHaveTextContent('数值代入')
    await user.click(calculationDisclosure)
    expect(calculationDisclosure).toHaveAttribute('aria-expanded', 'false')
    expect(screen.queryByRole('region', { name: '完整计算过程' })).not.toBeInTheDocument()

    await user.click(screen.getByRole('button', { name: '下一步' }))
    expect(screen.getByRole('heading', { name: '选择构型' })).toBeInTheDocument()
    const solutionCards = screen.getByRole('group', { name: '肘上肘下构型选择' })
    expect(within(solutionCards).getByRole('button', { name: /肘下构型/ })).toBeInTheDocument()
    const elbowUpCard = within(solutionCards).getByRole('button', { name: /肘上构型/ })
    await user.click(elbowUpCard)
    expect(screen.getByTestId('kinematics-workspace')).toHaveAttribute(
      'data-configuration',
      'conventional:elbow-up',
    )
    expect(elbowUpCard).toHaveAttribute('aria-pressed', 'true')
    expect(screen.getByText(/末端连杆仰角 β/)).toBeInTheDocument()
    const configurationAdvanced = screen.getByText('Advanced · 折叠径向与全部候选')
      .closest('details')
    expect(configurationAdvanced).not.toHaveAttribute('open')

    await user.click(screen.getByRole('button', { name: '下一步' }))
    expect(screen.getByRole('heading', { name: 'FK 回代验证' })).toBeInTheDocument()
    expect(screen.getByTestId('walkthrough-method')).toHaveTextContent('验证 · 正运动学回代')
    expect(screen.getByText('FK 回代完成')).toBeInTheDocument()
    const verificationAdvanced = screen.getByText('Advanced · 完整回代与姿态矩阵')
    expect(verificationAdvanced.closest('details')).not.toHaveAttribute('open')
    await user.click(verificationAdvanced)
    const verificationTable = screen.getByRole('table', { name: '逆解回代比较' })
    expect(verificationTable).toHaveTextContent('肘下')
    expect(verificationTable).toHaveTextContent('肘上')
    expect(verificationTable).toHaveTextContent('Δx / Δy / Δz (mm)')
    expect(verificationTable).toHaveTextContent('ep (mm)')
    const details = screen.getAllByText(/展开完整回代数据/)
    expect(details).toHaveLength(2)
    await user.click(details[0])
    expect(details[0].parentElement).toHaveTextContent('目标位置 pd')
    expect(details[0].parentElement).toHaveTextContent('回代位置 pFK')
    expect(details[0].parentElement).toHaveTextContent('分量误差 Δp')
    expect(screen.getAllByRole('table', { name: /构型末端姿态/ })).toHaveLength(2)
    const applyInverse = screen.getByRole('button', { name: '应用逆解' })
    expect(applyInverse).toBeEnabled()
    expect(screen.queryByRole('button', { name: '下一步' })).not.toBeInTheDocument()
    await user.click(applyInverse)
    expect(screen.getByRole('tab', { name: /^位置逆运动学/ }))
      .toHaveAttribute('aria-selected', 'true')
    expect(screen.getByTestId('walkthrough-step')).toHaveTextContent('第 5 / 5 步')
    expect(useLabStore.getState().jointState.q[2]).toBeLessThan(0)

    await selectMode(user, '微分运动学')
    expect(screen.getByRole('heading', { name: '读取当前关节运动状态' })).toBeInTheDocument()
    expect(screen.getByTestId('walkthrough-method')).toHaveTextContent('微分运动学 · 几何雅可比')
    expect(screen.getByTestId('walkthrough-step')).toHaveTextContent('第 1 / 4 步')
    expect(screen.queryByTestId('jacobian-result')).not.toBeInTheDocument()

    await user.click(screen.getByRole('button', { name: '修改运动状态' }))
    expect(screen.getByLabelText('关节速度 θ̇₂')).toHaveAccessibleDescription('°/s')
    await user.clear(screen.getByLabelText('关节速度 θ̇₂'))
    await user.type(screen.getByLabelText('关节速度 θ̇₂'), '12{Enter}')
    expect(useLabStore.getState().jointState.qd[1]).toBeCloseTo(12 * Math.PI / 180, 12)

    await user.click(screen.getByRole('button', { name: '下一步' }))
    expect(screen.getByRole('heading', { name: '用关节轴逐列构造雅可比' })).toBeInTheDocument()
    expect(screen.getByTestId('jacobian-result')).toBeInTheDocument()
    expect(screen.getByRole('button', { name: '第 1 列' })).toHaveAttribute('aria-pressed', 'true')
    await user.click(screen.getByRole('button', { name: '第 2 列' }))
    expect(screen.getByText('当前展示第 2 列')).toBeInTheDocument()
    await user.click(screen.getByRole('button', { name: '下一步' }))
    expect(screen.getByRole('heading', { name: '计算末端线速度与角速度' })).toBeInTheDocument()
    expect(screen.getByText(/长度经过视觉归一化/)).toBeInTheDocument()
    await user.click(screen.getByRole('button', { name: '下一步' }))
    expect(screen.getByRole('heading', { name: '判断位置奇异性' })).toBeInTheDocument()
    expect(screen.getByRole('button', { name: '下一步' })).toBeDisabled()

    await selectMode(user, '正运动学')
    expect(screen.getByRole('heading', { name: '提取末端姿态' })).toBeInTheDocument()
    expect(screen.getByTestId('walkthrough-step')).toHaveTextContent('第 6 / 6 步')
    expect(document.querySelector('.katex-error')).toBeNull()
  })
})
