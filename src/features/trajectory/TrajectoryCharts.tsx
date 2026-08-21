import createPlotlyComponent from 'react-plotly.js/factory'
import type { Data, Layout, PlotMouseEvent, PlotlyHTMLElement, Shape } from 'plotly.js'
import Plotly from 'plotly.js/dist/plotly-basic.js'
import { useMemo, useRef, useState } from 'react'
import { downloadPlotImage } from '../../export/download'
import {
  normalizedProfileSample,
  radiansToDegrees,
  type TimedTrajectorySample,
  type TrajectoryPreview,
} from './trajectoryModel'

const Plot = createPlotlyComponent(Plotly)
const COLORS = ['#167b78', '#bd7718', '#a74736'] as const

type JointQuantity = 'q' | 'qd' | 'qdd'

const QUANTITIES: readonly {
  key: JointQuantity
  label: string
  title: string
  unit: string
}[] = [
  { key: 'q', label: '位置 q', title: '关节位置 q(t)', unit: '°' },
  { key: 'qd', label: '速度 q̇', title: '关节速度 q̇(t)', unit: '°/s' },
  { key: 'qdd', label: '加速度 q̈', title: '关节加速度 q̈(t)', unit: '°/s²' },
]

function cursorShape(x: number): Partial<Shape> {
  return {
    type: 'line', x0: x, x1: x, xref: 'x', y0: 0, y1: 1, yref: 'paper',
    line: { color: '#17283c', dash: 'dash', width: 2 },
  }
}

function baseLayout(title: string, xTitle: string, yTitle: string, shapes: Partial<Shape>[]): Partial<Layout> {
  return {
    title: { text: title },
    margin: { l: 58, r: 18, t: 46, b: 48 },
    height: 310,
    paper_bgcolor: '#fbf8f0',
    plot_bgcolor: '#fffdf8',
    xaxis: { title: { text: xTitle } },
    yaxis: { title: { text: yTitle } },
    legend: { orientation: 'h' },
    shapes,
  }
}

export function TrajectoryCharts({
  preview,
  time,
  onTimeChange,
}: {
  preview: TrajectoryPreview
  time: number
  onTimeChange: (time: number) => void
}) {
  const [quantity, setQuantity] = useState<JointQuantity>('q')
  const exportGraphs = useRef<Record<string, PlotlyHTMLElement | undefined>>({})
  const duration = preview.instruction.duration
  const currentQuantity = QUANTITIES.find((item) => item.key === quantity) ?? QUANTITIES[0]
  const normalizedSamples = useMemo(() => Array.from({ length: 201 }, (_, index) => {
    const u = index / 200
    return { u, ...normalizedProfileSample(preview.instruction.profile, u) }
  }), [preview.instruction.profile])

  const normalizedData: Data[] = [
    { name: 's(u)', values: normalizedSamples.map((sample) => sample.position) },
    { name: "s'(u)", values: normalizedSamples.map((sample) => sample.velocity) },
    { name: "s''(u)", values: normalizedSamples.map((sample) => sample.acceleration) },
  ].map((series, index) => ({
    type: 'scatter', mode: 'lines', name: series.name,
    x: normalizedSamples.map((sample) => sample.u), y: series.values,
    line: { color: COLORS[index], width: 2 },
  }))

  const jointData: Data[] = [0, 1, 2].map((index) => ({
    type: 'scatter',
    mode: 'lines',
    name: `J${index + 1}`,
    x: preview.samples.map((sample) => sample.time),
    y: preview.samples.map((sample: TimedTrajectorySample) => radiansToDegrees(sample[quantity][index])),
    line: { color: COLORS[index], width: 2 },
  }))

  const phaseShapes: Partial<Shape>[] = preview.instruction.profile === 'trapezoidal'
    ? [
      { type: 'rect', x0: 0, x1: 1 / 3, y0: 0, y1: 1, xref: 'x', yref: 'paper', fillcolor: '#dcecea', opacity: 0.32, line: { width: 0 } },
      { type: 'rect', x0: 1 / 3, x1: 2 / 3, y0: 0, y1: 1, xref: 'x', yref: 'paper', fillcolor: '#f4e7ca', opacity: 0.28, line: { width: 0 } },
      { type: 'rect', x0: 2 / 3, x1: 1, y0: 0, y1: 1, xref: 'x', yref: 'paper', fillcolor: '#f3ddd8', opacity: 0.25, line: { width: 0 } },
    ]
    : []

  const exportChart = (key: string, format: 'svg' | 'png') => {
    const graph = exportGraphs.current[key]
    if (graph !== undefined) void downloadPlotImage(graph, format, `3r-trajectory-${key}`)
  }
  const handleNormalizedClick = (event: PlotMouseEvent) => {
    const x = event.points[0]?.x
    if (typeof x === 'number') onTimeChange(x * duration)
  }
  const handleJointClick = (event: PlotMouseEvent) => {
    const x = event.points[0]?.x
    if (typeof x === 'number') onTimeChange(x)
  }

  return (
    <div className="trajectory-charts">
      <div className="trajectory-charts__toolbar">
        <div>
          <p className="section-label">步骤 4 · 同步预览</p>
          <h3>多项式与三关节轨迹</h3>
        </div>
        <div aria-label="关节曲线状态量" className="trajectory-chart-tabs" role="tablist">
          {QUANTITIES.map((item) => (
            <button
              aria-selected={quantity === item.key}
              key={item.key}
              onClick={() => setQuantity(item.key)}
              role="tab"
              type="button"
            >{item.label}</button>
          ))}
        </div>
      </div>
      <div className="trajectory-charts__grid">
        <section className="trajectory-chart-card" data-chart="normalized-profile">
          <div className="trajectory-chart-actions">
            <button aria-label="导出归一化曲线 SVG" onClick={() => exportChart('normalized-profile', 'svg')} type="button">SVG</button>
            <button aria-label="导出归一化曲线 PNG" onClick={() => exportChart('normalized-profile', 'png')} type="button">PNG</button>
          </div>
          <Plot
            config={{ displayModeBar: false, responsive: true }}
            data={normalizedData}
            layout={baseLayout('归一化插值 s / s′ / s″', 'u = t / T', '归一化量', [...phaseShapes, cursorShape(time / duration)])}
            onClick={handleNormalizedClick}
            onInitialized={(_, graph) => { exportGraphs.current['normalized-profile'] = graph as PlotlyHTMLElement }}
            useResizeHandler
          />
        </section>
        <section className="trajectory-chart-card" data-chart="joint-trajectory">
          <div className="trajectory-chart-actions">
            <button aria-label="导出关节轨迹 SVG" onClick={() => exportChart(`joint-${quantity}`, 'svg')} type="button">SVG</button>
            <button aria-label="导出关节轨迹 PNG" onClick={() => exportChart(`joint-${quantity}`, 'png')} type="button">PNG</button>
          </div>
          <Plot
            config={{ displayModeBar: false, responsive: true }}
            data={jointData}
            layout={baseLayout(currentQuantity.title, 't (s)', currentQuantity.unit, [cursorShape(time)])}
            onClick={handleJointClick}
            onInitialized={(_, graph) => { exportGraphs.current[`joint-${quantity}`] = graph as PlotlyHTMLElement }}
            useResizeHandler
          />
        </section>
      </div>
    </div>
  )
}
