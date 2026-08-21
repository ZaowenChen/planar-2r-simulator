import { BlockMath, InlineMath } from 'react-katex'
import { FormulaCard } from '../../components/FormulaCard'
import {
  normalizedProfileSample,
  radiansToDegrees,
  type TimedTrajectorySample,
  type TrajectoryPreview,
} from './trajectoryModel'

function vectorDegrees(vector: readonly number[], precision = 2): string {
  return `[${vector.map((value) => radiansToDegrees(value).toFixed(precision)).join(', ')}]`
}

export function PolynomialLesson({
  preview,
  current,
  time,
}: {
  preview: TrajectoryPreview
  current: TimedTrajectorySample
  time: number
}) {
  const duration = preview.instruction.duration
  const u = duration === 0 ? 0 : time / duration
  const normalized = normalizedProfileSample('quintic', u)
  const displacement = preview.instruction.q0.map((value, index) => (
    preview.instruction.qf[index] - value
  ))

  return (
    <div className="trajectory-lesson" data-testid="trajectory-lesson">
      <div className="trajectory-section-heading">
        <div>
          <span>步骤 3 · 数学教学</span>
          <h3>五次多项式轨迹</h3>
        </div>
        <output>u = {u.toFixed(3)}</output>
      </div>
      <p className="trajectory-lesson__purpose">
        六个多项式系数满足起点和终点的位置、速度、加速度六个边界条件。
      </p>
      <ol className="trajectory-lesson__steps">
        <li><b>边界条件</b><span>q 已知，q̇ = 0，q̈ = 0</span></li>
        <li><b>归一化时间</b><span><InlineMath math="u=t/T" /></span></li>
        <li className="is-active"><b>插值多项式</b><span>由 s(u) 映射三个关节</span></li>
      </ol>
      <FormulaCard
        definition={<>
          <BlockMath math={String.raw`s(u)=10u^3-15u^4+6u^5`} />
          <BlockMath math={String.raw`q_i(t)=q_{i,0}+\Delta q_i s(u)`} />
          <BlockMath math={String.raw`\dot q_i=\frac{\Delta q_i}{T}s'(u),\quad \ddot q_i=\frac{\Delta q_i}{T^2}s''(u)`} />
        </>}
        substitution={<>
          <BlockMath math={String.raw`t=${time.toFixed(3)}\,\mathrm{s},\quad T=${duration.toFixed(3)}\,\mathrm{s},\quad u=${u.toFixed(4)}`} />
          <BlockMath math={String.raw`s(u)=${normalized.position.toFixed(5)},\quad s'(u)=${normalized.velocity.toFixed(5)},\quad s''(u)=${normalized.acceleration.toFixed(5)}`} />
          <p>Δq = {vectorDegrees(displacement)}°</p>
        </>}
        result={<>
          <p>q = {vectorDegrees(current.q)}°</p>
          <p>q̇ = {vectorDegrees(current.qd)}°/s</p>
          <p>q̈ = {vectorDegrees(current.qdd)}°/s²</p>
        </>}
        symbols={[
          { symbol: 'u', meaning: '归一化时间', unit: '无量纲' },
          { symbol: 'T', meaning: 'PTP 持续时间', unit: 's' },
          { symbol: '\\Delta q_i', meaning: '第 i 个关节的总位移', unit: 'rad' },
        ]}
        title="五次插值公式"
      />
      <details className="trajectory-derivation">
        <summary>展开完整边界条件与导数</summary>
        <BlockMath math={String.raw`s(0)=0,\ s(1)=1,\ s'(0)=s'(1)=0,\ s''(0)=s''(1)=0`} />
        <BlockMath math={String.raw`s'(u)=30u^2-60u^3+30u^4`} />
        <BlockMath math={String.raw`s''(u)=60u-180u^2+120u^3`} />
        <p>持续时间变为原来的 k 倍时，位置形状不变，速度缩放为 1/k，加速度缩放为 1/k²。</p>
      </details>
    </div>
  )
}

