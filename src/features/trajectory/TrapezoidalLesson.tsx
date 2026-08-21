import { BlockMath } from 'react-katex'
import { FormulaCard } from '../../components/FormulaCard'
import {
  normalizedProfileSample,
  radiansToDegrees,
  type TimedTrajectorySample,
  type TrajectoryPreview,
} from './trajectoryModel'

function degreesText(vector: readonly number[], unit: string): string {
  return `[${vector.map((value) => radiansToDegrees(value).toFixed(2)).join(', ')}] ${unit}`
}

export function TrapezoidalLesson({
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
  const normalized = normalizedProfileSample('trapezoidal', u)
  const phase = u <= 1 / 3 ? '加速段' : u <= 2 / 3 ? '匀速段' : '减速段'

  return (
    <div className="trajectory-lesson" data-testid="trajectory-lesson">
      <div className="trajectory-section-heading">
        <div>
          <span>步骤 3 · 数学教学</span>
          <h3>梯形速度轨迹</h3>
        </div>
        <output>{phase}</output>
      </div>
      <p className="trajectory-lesson__purpose">
        三个关节共享加速、匀速和减速阶段，因此同步抵达目标点。
      </p>
      <div className="trapezoidal-phases" aria-label="梯形速度阶段">
        <span className={phase === '加速段' ? 'is-active' : undefined}>加速 0–T/3</span>
        <span className={phase === '匀速段' ? 'is-active' : undefined}>匀速 T/3–2T/3</span>
        <span className={phase === '减速段' ? 'is-active' : undefined}>减速 2T/3–T</span>
      </div>
      <FormulaCard
        definition={<BlockMath math={String.raw`s(u)=\begin{cases}2.25u^2,&0\le u\le\frac13\\1.5u-0.25,&\frac13<u\le\frac23\\1-2.25(1-u)^2,&\frac23<u\le1\end{cases}`} />}
        substitution={<>
          <BlockMath math={String.raw`u=${u.toFixed(4)},\quad s=${normalized.position.toFixed(5)},\quad s'=${normalized.velocity.toFixed(5)},\quad s''=${normalized.acceleration.toFixed(5)}`} />
          <p>当前处于{phase}。</p>
        </>}
        result={<>
          <p>q = {degreesText(current.q, '°')}</p>
          <p>q̇ = {degreesText(current.qd, '°/s')}</p>
          <p>q̈ = {degreesText(current.qdd, '°/s²')}</p>
        </>}
        symbols={[
          { symbol: 's', meaning: '同步归一化位移', unit: '无量纲' },
          { symbol: "s'", meaning: '归一化速度', unit: '无量纲' },
          { symbol: "s''", meaning: '归一化加速度', unit: '无量纲' },
        ]}
        title="梯形速度分段公式"
      />
      <p className="trajectory-teaching-note">
        位置和速度在分段边界连续；加速度会从 4.5 跳到 0，再跳到 −4.5。这是分段轨迹的数学特征，不是计算错误。
      </p>
    </div>
  )
}
