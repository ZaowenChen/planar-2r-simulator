import { cleanup, render, screen } from '@testing-library/react'
import userEvent from '@testing-library/user-event'
import { afterEach, describe, expect, it, vi } from 'vitest'
import type { PlotlyHTMLElement } from 'plotly.js'
import Plotly from 'plotly.js/dist/plotly-basic.js'
import type { SimulationSample } from '../../robotics/integration'
import { TimeSeriesCharts } from './TimeSeriesCharts'

vi.mock('react-plotly.js/factory', async () => {
  const React = await import('react')
  return {
    default: () => ({ layout, onInitialized }: {
      layout: { title?: { text?: string } }
      onInitialized?: (figure: unknown, graph: HTMLElement) => void
    }) => {
      const graph = React.useRef<HTMLDivElement>(null)
      React.useEffect(() => {
        if (graph.current !== null) onInitialized?.({}, graph.current)
      }, [onInitialized])
      return <div data-chart={layout.title?.text} ref={graph} />
    },
  }
})

vi.mock('plotly.js/dist/plotly-basic.js', () => ({
  default: { downloadImage: vi.fn().mockResolvedValue('data:image') },
}))

afterEach(() => {
  cleanup()
  vi.clearAllMocks()
})

const sample: SimulationSample = {
  time: 0,
  q: [0, 0, 0], qd: [0, 0, 0], qdd: [0, 0, 0], tau: [1, 2, 3],
  kinetic: 1, potential: 2, totalEnergy: 3, jointPower: [0, 0, 0],
}

describe('TimeSeriesCharts', () => {
  it('exports every user-visible chart group as SVG and PNG using its own graph and filename', async () => {
    const user = userEvent.setup()
    render(<TimeSeriesCharts onTimeChange={() => undefined} samples={[sample]} time={0} />)

    const targets = [
      ['状态', 'joint-state'],
      ['力矩', 'torque'],
      ['能量', 'energy'],
      ['功率', 'joint-power'],
    ] as const
    for (const [label, filename] of targets) {
      const graph = document.querySelector(`[data-chart][data-export-key="${filename}"]`)
        ?? document.querySelector(`[data-chart="${label === '状态' ? 'q / q̇ / q̈' : label === '力矩' ? 'τ' : label === '能量' ? 'K / V / E' : 'P_i'}"]`)
      expect(graph).not.toBeNull()
      await user.click(screen.getByRole('button', { name: `导出${label} SVG` }))
      expect(Plotly.downloadImage).toHaveBeenLastCalledWith(graph as PlotlyHTMLElement, {
        filename: `3r-dynamics-${filename}`,
        format: 'svg',
        height: 1080,
        width: 1440,
      })
      await user.click(screen.getByRole('button', { name: `导出${label} PNG` }))
      expect(Plotly.downloadImage).toHaveBeenLastCalledWith(graph as PlotlyHTMLElement, {
        filename: `3r-dynamics-${filename}`,
        format: 'png',
        height: 1080,
        width: 1440,
      })
    }
    expect(Plotly.downloadImage).toHaveBeenCalledTimes(8)
  })
})
