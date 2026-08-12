import type { PlotlyHTMLElement } from 'plotly.js'
import Plotly from 'plotly.js/dist/plotly-basic.js'
import type { SimulationSample } from '../robotics/integration'
import { simulationSamplesToCsv } from './csv'

function downloadBlob(blob: Blob, filename: string): void {
  const url = URL.createObjectURL(blob)
  const anchor = document.createElement('a')
  anchor.download = filename
  anchor.href = url
  anchor.click()
  URL.revokeObjectURL(url)
}

export function downloadSimulationCsv(
  samples: readonly SimulationSample[],
  filename = '3r-dynamics-simulation.csv',
): void {
  downloadBlob(
    new Blob([simulationSamplesToCsv(samples)], { type: 'text/csv;charset=utf-8' }),
    filename,
  )
}

export function downloadPlotImage(
  graph: PlotlyHTMLElement,
  format: 'svg' | 'png',
  filename = '3r-dynamics-timeseries',
): Promise<string> {
  return Plotly.downloadImage(graph, {
    filename,
    format,
    height: 1080,
    width: 1440,
  })
}
