import type { ReactNode } from 'react'

export interface WorkbenchLayoutProps {
  visual: ReactNode
  controls: ReactNode
  analysis: ReactNode
  timeline?: ReactNode
}

export function WorkbenchLayout({ visual, controls, analysis, timeline }: WorkbenchLayoutProps) {
  return (
    <div className="workbench">
      <section aria-label="机器人三维视图" className="workbench__visual">{visual}</section>
      <section aria-label="实验控制" className="workbench__controls">{controls}</section>
      <section aria-label="公式与结果" className="workbench__analysis">{analysis}</section>
      {timeline !== undefined && (
        <section aria-label="仿真时间曲线" className="workbench__timeline">{timeline}</section>
      )}
    </div>
  )
}
