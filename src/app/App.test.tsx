import { cleanup, render, screen } from '@testing-library/react'
import userEvent from '@testing-library/user-event'
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { calculationSnapshotCsv } from '../symbols/display'
import { useLabStore } from '../state/labStore'
import { App } from './App'

vi.mock('@react-three/fiber', async () => {
  const actual = await vi.importActual<typeof import('@react-three/fiber')>('@react-three/fiber')
  return { ...actual, Canvas: () => <div data-testid="app-scene" /> }
})

afterEach(cleanup)
beforeEach(() => useLabStore.getState().resetLab())

describe('App', () => {
  it('names the laboratory and exposes its four learning modules', () => {
    render(<App />)
    expect(screen.getByText('空间 3R 机器人学交互实验室')).toBeInTheDocument()
    expect(screen.getByText(/运动学显示 mm \/ °/)).toBeInTheDocument()
    expect(document.body.textContent).not.toContain('内部角度使用弧度')
    for (const label of ['机器人模型', '运动学', '动力学', '动态实验']) {
      expect(screen.getByRole('button', { name: label })).toBeInTheDocument()
    }
  })

  it('visits every module without leaking source-style labels into rendered or exported output', async () => {
    const user = userEvent.setup()
    render(<App />)
    let renderedText = ''

    for (const label of ['机器人模型', '运动学', '动力学', '动态实验']) {
      await user.click(screen.getByRole('button', { name: label }))
      renderedText += document.body.textContent ?? ''
    }

    const csv = calculationSnapshotCsv(useLabStore.getState().calculation)
    for (const bannedLabel of ['x_target', 'P_B^T', 'T_0^3', 'a_(i-1)']) {
      expect(renderedText).not.toContain(bannedLabel)
      expect(csv.split('\n')[0]).not.toContain(bannedLabel)
    }
    expect(csv.split('\n')[0]).toContain('末端位置 x (m)')
  })

  it('marks the selected module and displays the academic three-column workbench', async () => {
    const user = userEvent.setup()
    render(<App />)

    await user.click(screen.getByRole('button', { name: '动力学' }))

    expect(screen.getByRole('button', { name: '动力学' })).toHaveAttribute('aria-current', 'page')
    expect(screen.getByRole('region', { name: '机器人三维视图' })).toBeInTheDocument()
    expect(screen.getByRole('region', { name: '实验控制' })).toBeInTheDocument()
    expect(screen.getByRole('region', { name: '公式与结果' })).toBeInTheDocument()
  })

  it('routes the model and kinematics tabs to their complete learning modules', async () => {
    const user = userEvent.setup()
    render(<App />)

    expect(screen.getByRole('table', { name: '标准 D–H 参数表' })).toBeInTheDocument()
    await user.click(screen.getByRole('button', { name: '运动学' }))

    expect(screen.getByText('D–H 正解 · 几何逆解 · 微分运动学')).toBeInTheDocument()
    expect(screen.getByText(/由关节角计算末端位姿/)).toBeInTheDocument()
    expect(screen.getByText(/θ₂ 25\.0°/)).toBeInTheDocument()
    expect(screen.queryByLabelText('关节角 θ₂')).not.toBeInTheDocument()
    expect(screen.getByRole('tab', { name: /^正运动学/ })).toHaveAttribute('aria-selected', 'true')
    expect(screen.getByRole('tab', { name: /^位置逆运动学/ })).toBeInTheDocument()
    await user.click(screen.getByRole('button', { name: '修改关节角' }))
    expect(screen.getByLabelText('关节角 θ₂')).toBeInTheDocument()
    expect(screen.getByTestId('kinematics-walkthrough')).toBeInTheDocument()
    expect(screen.getByTestId('walkthrough-step')).toHaveTextContent('第 1 / 6 步')
  })
})
