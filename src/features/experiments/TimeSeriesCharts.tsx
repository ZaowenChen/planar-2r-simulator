import createPlotlyComponent from 'react-plotly.js/factory'
import type { Data, Layout, PlotMouseEvent, PlotlyHTMLElement } from 'plotly.js'
import Plotly from 'plotly.js/dist/plotly-basic.js'
import { useRef } from 'react'
import type { SimulationSample } from '../../robotics/integration'
import { downloadPlotImage } from '../../export/download'

const Plot = createPlotlyComponent(Plotly)

interface TimeSeriesChartsProps {
  samples: readonly SimulationSample[]
  time: number
  onTimeChange: (time: number) => void
}

const COLORS = ['#167b78', '#bd7718', '#a74736'] as const

function traces(
  samples: readonly SimulationSample[],
  values: (sample: SimulationSample) => readonly number[],
  names: readonly string[],
): Data[] {
  return names.map((name, index) => ({
    type: 'scatter',
    mode: 'lines',
    name,
    x: samples.map((sample) => sample.time),
    y: samples.map((sample) => values(sample)[index]),
    line: { color: COLORS[index % COLORS.length], width: 1.8 },
  }))
}

export function TimeSeriesCharts({ samples, time, onTimeChange }: TimeSeriesChartsProps) {
  const exportGraphs = useRef<Record<string, PlotlyHTMLElement | undefined>>({})
  const groups: readonly {
    title: string
    data: Data[]
    yTitle: string
    exportKey: string
    exportLabel: string
  }[] = [
    {
      title: 'q / q̇ / q̈',
      yTitle: 'SI joint state',
      exportKey: 'joint-state',
      exportLabel: '状态',
      data: [
        ...traces(samples, (sample) => sample.q, ['θ₁', 'θ₂', 'θ₃']),
        ...traces(samples, (sample) => sample.qd, ['ω₁', 'ω₂', 'ω₃']),
        ...traces(samples, (sample) => sample.qdd, ['α₁', 'α₂', 'α₃']),
      ],
    },
    { title: 'τ', yTitle: 'N·m', exportKey: 'torque', exportLabel: '力矩', data: traces(samples, (sample) => sample.tau, ['τ₁', 'τ₂', 'τ₃']) },
    {
      title: 'K / V / E',
      yTitle: 'J',
      exportKey: 'energy',
      exportLabel: '能量',
      data: traces(samples, (sample) => [sample.kinetic, sample.potential, sample.totalEnergy], ['K', 'V', 'E']),
    },
    { title: 'P_i', yTitle: 'W', exportKey: 'joint-power', exportLabel: '功率', data: traces(samples, (sample) => sample.jointPower, ['P₁', 'P₂', 'P₃']) },
  ]
  const handleClick = (event: PlotMouseEvent) => {
    const x = event.points[0]?.x
    if (typeof x === 'number') onTimeChange(x)
  }

  return (
    <div className="time-series">
      <div className="time-series__toolbar">
        <strong>同步时间序列</strong>
      </div>
      <div className="time-series__grid">
        {groups.map((group) => {
          const layout: Partial<Layout> = {
            title: { text: group.title },
            margin: { l: 52, r: 18, t: 42, b: 42 },
            height: 290,
            paper_bgcolor: '#fbf8f0',
            plot_bgcolor: '#fffdf8',
            xaxis: { title: { text: 't (s)' } },
            yaxis: { title: { text: group.yTitle } },
            shapes: [{
              type: 'line',
              x0: time,
              x1: time,
              xref: 'x',
              y0: 0,
              y1: 1,
              yref: 'paper',
              line: { color: '#17283c', dash: 'dash', width: 2 },
            }],
            legend: { orientation: 'h' },
          }
          const exportChart = (format: 'svg' | 'png') => {
            const graph = exportGraphs.current[group.exportKey]
            if (graph !== undefined) {
              void downloadPlotImage(graph, format, `3r-dynamics-${group.exportKey}`)
            }
          }
          return <section className="time-series__chart" key={group.title}>
            <div className="time-series__chart-actions">
              <button aria-label={`导出${group.exportLabel} SVG`} onClick={() => exportChart('svg')} type="button">SVG</button>
              <button aria-label={`导出${group.exportLabel} PNG`} onClick={() => exportChart('png')} type="button">PNG</button>
            </div>
            <Plot
              config={{ displayModeBar: false, responsive: true }}
              data={group.data}
              layout={layout}
              onClick={handleClick}
              onInitialized={(_, graph) => { exportGraphs.current[group.exportKey] = graph as PlotlyHTMLElement }}
              useResizeHandler
            />
          </section>
        })}
      </div>
    </div>
  )
}
