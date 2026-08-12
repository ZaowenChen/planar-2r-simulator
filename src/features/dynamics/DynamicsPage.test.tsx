import { cleanup, render, screen, within } from '@testing-library/react'
import userEvent from '@testing-library/user-event'
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { DEFAULT_ROBOT_PARAMETERS } from '../../robotics/defaults'
import { useLabStore } from '../../state/labStore'
import { DynamicsPage } from './DynamicsPage'

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

async function replaceNumber(input: HTMLElement, value: string) {
  const user = userEvent.setup()
  await user.clear(input)
  await user.type(input, `${value}{Enter}`)
}

function cardNamed(title: string): HTMLElement {
  const card = screen.getByRole('heading', { name: title }).closest('article')
  if (card === null) throw new Error(`Missing formula card: ${title}`)
  return card
}

function accessibleMathText(container: HTMLElement): string {
  return Array.from(container.querySelectorAll('.katex-mathml'))
    .map((node) => node.textContent ?? '')
    .join(' ')
    .replaceAll(/\s+/g, '')
}

beforeEach(() => useLabStore.getState().resetLab())
afterEach(cleanup)

describe('DynamicsPage', () => {
  it('changes dynamics without changing forward kinematics when only m₂ changes', async () => {
    render(<DynamicsPage />)
    const endpointBefore = screen.getByTestId('endpoint-result').textContent
    const massBefore = screen.getByTestId('mass-matrix-result').textContent

    await replaceNumber(screen.getByLabelText('连杆 2 质量 m₂'), '4.5')

    expect(screen.getByTestId('endpoint-result')).toHaveTextContent(endpointBefore ?? '')
    expect(screen.getByTestId('mass-matrix-result').textContent).not.toBe(massBefore)
    expect(useLabStore.getState().parameters.links[1].mass).toBe(4.5)
  })

  it('edits all six independent symmetric inertia components and reconstructs the tensor', async () => {
    render(<DynamicsPage />)
    const edits = [
      ['连杆 1 惯性 Iₓₓ', '0.22'],
      ['连杆 1 惯性 Iᵧᵧ', '0.03'],
      ['连杆 1 惯性 Iᶻᶻ', '0.22'],
      ['连杆 1 惯性 Iₓᵧ', '0.001'],
      ['连杆 1 惯性 Iₓᶻ', '0.0015'],
      ['连杆 1 惯性 Iᵧᶻ', '0.0005'],
    ] as const

    for (const [label, value] of edits) {
      await replaceNumber(screen.getByLabelText(label), value)
    }

    expect(useLabStore.getState().parameters.links[0].inertia).toEqual([
      [0.22, 0.001, 0.0015],
      [0.001, 0.03, 0.0005],
      [0.0015, 0.0005, 0.22],
    ])
    expect(screen.getByRole('table', { name: '连杆 1 重构惯性张量' })).toHaveTextContent('0.0015')
    expect(screen.getByTestId('link-1-principal-inertias')).toHaveTextContent('主惯性矩')
  })

  it('keeps the last valid tensor and reports nonphysical principal inertia', async () => {
    render(<DynamicsPage />)
    const validTensor = structuredClone(useLabStore.getState().parameters.links[0].inertia)

    await replaceNumber(screen.getByLabelText('连杆 1 惯性 Iᵧᵧ'), '10')

    expect(screen.getByText(/主惯性矩必须满足三角不等式/)).toBeInTheDocument()
    expect(screen.getByTestId('link-1-principal-inertias')).toHaveTextContent('10.0000')
    expect(useLabStore.getState().parameters.links[0].inertia).toEqual(validTensor)
  })

  it('rejects a center of mass outside the link range and can reset one link', async () => {
    render(<DynamicsPage />)

    await replaceNumber(screen.getByLabelText('连杆 3 质心 cₓ'), '-2')
    expect(screen.getByText(/质心必须位于连杆名义球体范围内/)).toBeInTheDocument()
    expect(useLabStore.getState().parameters.links[2].centerOfMass).toEqual([-0.75, 0, 0])

    await replaceNumber(screen.getByLabelText('连杆 3 质量 m₃'), '2.8')
    await userEvent.click(screen.getByRole('button', { name: '复位连杆 3 刚体参数' }))

    expect(screen.getByLabelText('连杆 3 质心 cₓ')).toHaveValue('-0.75')
    expect(useLabStore.getState().parameters.links[2]).toEqual(DEFAULT_ROBOT_PARAMETERS.links[2])
  })

  it('atomically resets one link while preserving an invalid draft on another link', async () => {
    render(<DynamicsPage />)
    const defaultMassMatrix = screen.getByTestId('mass-matrix-result').textContent

    await replaceNumber(screen.getByLabelText('连杆 2 质量 m₂'), '4.5')
    await replaceNumber(screen.getByLabelText('连杆 1 质心 cₓ'), '2')
    expect(screen.getByText(/质心必须位于连杆名义球体范围内/)).toBeInTheDocument()
    expect(useLabStore.getState().parameters.links[1].mass).toBe(4.5)

    await userEvent.click(screen.getByRole('button', { name: '复位连杆 2 刚体参数' }))

    const state = useLabStore.getState()
    expect(screen.getByLabelText('连杆 1 质心 cₓ')).toHaveValue('2')
    expect(state.rawParameters['links.0.centerOfMass.0']).toBe('2')
    expect(state.fieldIssues['links.0.centerOfMass']).toBeDefined()
    expect(screen.getByLabelText('连杆 2 质量 m₂')).toHaveValue('3')
    expect(state.rawParameters['links.1.mass']).toBe('3')
    expect(state.parameters.links[1]).toEqual(DEFAULT_ROBOT_PARAMETERS.links[1])
    expect(screen.getByTestId('mass-matrix-result')).toHaveTextContent(defaultMassMatrix ?? '')
  })

  it('updates gravity direction, disables friction torque, and resets all teaching parameters', async () => {
    useLabStore.getState().setJointVelocity(0, 1)
    render(<DynamicsPage />)
    const gravityBefore = screen.getByTestId('gravity-result').textContent

    await replaceNumber(screen.getByLabelText('重力方向 gₓ'), '1.25')
    expect(screen.getByTestId('gravity-result').textContent).not.toBe(gravityBefore)
    expect(useLabStore.getState().parameters.gravity).toEqual([1.25, 0, -9.81])

    const frictionToggle = screen.getByRole('checkbox', { name: '启用粘性摩擦' })
    expect(screen.getByTestId('friction-result')).not.toHaveTextContent('[0.0000, 0.0000, 0.0000]')
    await userEvent.click(frictionToggle)
    expect(frictionToggle).not.toBeChecked()
    expect(screen.getByTestId('friction-result')).toHaveTextContent('[0.0000, 0.0000, 0.0000]')

    await userEvent.click(screen.getByRole('button', { name: '复位全部动力学教学参数' }))
    expect(useLabStore.getState().parameters.gravity).toEqual(DEFAULT_ROBOT_PARAMETERS.gravity)
    expect(useLabStore.getState().parameters.frictionEnabled).toBe(true)
  })

  it('shows SI matrix units, every dynamics result card, all three views, and the numerical disclosure', async () => {
    render(<DynamicsPage />)

    expect(screen.getByRole('table', { name: '质量矩阵' })).toHaveTextContent('kg·m²')
    for (const title of [
      '机械臂动力学方程',
      '质量矩阵 M(q)',
      '科氏矩阵 C(q,q̇)',
      '重力项 g(q)',
      '粘性摩擦力矩',
      '动能',
      '势能',
      '总机械能',
      '关节功率',
    ]) {
      expect(screen.getByRole('heading', { name: title })).toBeInTheDocument()
    }

    expect(cardNamed('科氏矩阵 C(q,q̇)')).toHaveTextContent('Christoffel 符号按精确定义构造')
    expect(cardNamed('科氏矩阵 C(q,q̇)')).toHaveTextContent('中心差分，h=10⁻⁵ rad')

    const massDefinition = accessibleMathText(cardNamed('质量矩阵 M(q)'))
    expect(massDefinition).toContain('∑i(miJviTJvi+JωiT0RiciIi0RiTJωi)')

    const massCard = cardNamed('质量矩阵 M(q)')
    expect(within(massCard).getByRole('tabpanel')).toHaveTextContent('M')
    await userEvent.click(within(massCard).getByRole('tab', { name: '代入' }))
    expect(within(massCard).getByRole('tabpanel')).toHaveTextContent('q')
    await userEvent.click(within(massCard).getByRole('tab', { name: '结果' }))
    expect(within(massCard).getByRole('tabpanel')).toHaveTextContent('M')
  })

  it('gives every dynamics result card a glossary and substitutes current computed values', async () => {
    useLabStore.getState().setJointVelocity(0, 0.4)
    useLabStore.getState().setJointVelocity(1, -0.2)
    useLabStore.getState().setJointVelocity(2, 0.3)
    render(<DynamicsPage />)
    const calculation = useLabStore.getState().calculation
    const resultCards = [
      '质量矩阵 M(q)',
      '科氏矩阵 C(q,q̇)',
      '重力项 g(q)',
      '粘性摩擦力矩',
      '动能',
      '势能',
      '总机械能',
      '关节功率',
    ]

    for (const title of resultCards) {
      expect(cardNamed(title).querySelector('.formula-card__symbols')).not.toBeNull()
    }

    const expectations = [
      ['质量矩阵 M(q)', '质量矩阵', 'kg·m²', calculation.dynamics.massMatrix[0][0]],
      ['科氏矩阵 C(q,q̇)', '科氏矩阵', 'kg·m²/s', calculation.dynamics.coriolisMatrix[0][0]],
      ['重力项 g(q)', '重力广义力矩', 'N·m', calculation.dynamics.gravityTorque[0]],
      ['动能', '系统动能', 'J', calculation.energy.kinetic],
      ['关节功率', '各关节机械功率', 'W', calculation.energy.jointPower[0]],
    ] as const

    for (const [title, meaning, unit, currentValue] of expectations) {
      const card = cardNamed(title)
      expect(card).toHaveTextContent(meaning)
      expect(card).toHaveTextContent(unit)
      await userEvent.click(within(card).getByRole('tab', { name: '代入' }))
      expect(within(card).getByRole('tabpanel')).toHaveTextContent(currentValue.toFixed(4))
    }

    const kineticCard = cardNamed('动能')
    await userEvent.click(within(kineticCard).getByRole('tab', { name: '定义' }))
    expect(accessibleMathText(kineticCard)).toContain('K=')
    expect(accessibleMathText(kineticCard)).not.toContain('T=')
  })
})
