import { render, screen } from '@testing-library/react'
import { describe, expect, it } from 'vitest'
import { App } from './App'

describe('App', () => {
  it('names the laboratory and exposes its four learning modules', () => {
    render(<App />)
    expect(screen.getByRole('heading', { name: '空间 3R 机器人学交互实验室' })).toBeInTheDocument()
    for (const label of ['机器人模型', '运动学', '动力学', '动态实验']) {
      expect(screen.getByRole('button', { name: label })).toBeInTheDocument()
    }
  })
})
