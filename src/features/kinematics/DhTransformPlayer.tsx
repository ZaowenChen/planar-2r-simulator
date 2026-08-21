import { useEffect, useMemo, useState } from 'react'
import { InlineMath } from 'react-katex'
import { MatrixTable } from '../../components/MatrixTable'
import { decomposeDhTransform } from '../../robotics/transforms'
import type { RobotParameters, Vector3 } from '../../robotics/types'
import { radiansToDegrees, transformInMillimetres } from './presentation'
import type { DhOperation } from './teachingState'

export interface DhTransformPlayerProps {
  operation: DhOperation
  parameters: RobotParameters
  q: Vector3
  row: 0 | 1 | 2
  onOperationChange: (operation: DhOperation) => void
}

const OPERATIONS: readonly DhOperation[] = ['rz', 'tz', 'tx', 'rx']

export function DhTransformPlayer({
  operation,
  parameters,
  q,
  row,
  onOperationChange,
}: DhTransformPlayerProps) {
  const [playing, setPlaying] = useState(false)
  const geometry = parameters.geometry
  const values = [
    { theta: q[0], a: 0, alpha: Math.PI / 2, d: geometry.d1 },
    { theta: q[1], a: geometry.l2, alpha: 0, d: 0 },
    { theta: q[2], a: geometry.l3, alpha: 0, d: 0 },
  ] as const
  const current = values[row]
  const decomposition = useMemo(() => decomposeDhTransform(
    current.theta,
    current.a,
    current.alpha,
    current.d,
  ), [current.a, current.alpha, current.d, current.theta])
  const operationIndex = OPERATIONS.indexOf(operation)
  const subscript = String.fromCharCode(0x2081 + row)
  const previousSubscript = String.fromCharCode(0x2080 + row)
  const descriptions: Record<DhOperation, { label: string; math: string }> = {
    rz: {
      label: `绕 z${previousSubscript} 旋转 θ${subscript}`,
      math: String.raw`R_z(\theta_${row + 1}=${radiansToDegrees(current.theta).toFixed(1)}^\circ)`,
    },
    tz: {
      label: `沿 z${previousSubscript} 平移 d${subscript}`,
      math: String.raw`T_z(d_${row + 1}=${(current.d * 1000).toFixed(1)}\,\mathrm{mm})`,
    },
    tx: {
      label: `沿 x${subscript} 平移 a${subscript}`,
      math: String.raw`T_x(a_${row + 1}=${(current.a * 1000).toFixed(1)}\,\mathrm{mm})`,
    },
    rx: {
      label: `绕 x${subscript} 旋转 α${subscript}`,
      math: String.raw`R_x(\alpha_${row + 1}=${radiansToDegrees(current.alpha).toFixed(1)}^\circ)`,
    },
  }

  useEffect(() => {
    if (!playing) return
    if (window.matchMedia?.('(prefers-reduced-motion: reduce)').matches) {
      setPlaying(false)
      return
    }
    const timer = window.setTimeout(() => {
      if (operationIndex >= OPERATIONS.length - 1) {
        setPlaying(false)
        return
      }
      onOperationChange(OPERATIONS[operationIndex + 1])
    }, 720)
    return () => window.clearTimeout(timer)
  }, [onOperationChange, operationIndex, playing])

  return (
    <section className="dh-transform-player">
      <div className="dh-transform-player__chain" role="group" aria-label={`D–H 第 ${row + 1} 行四段变换`}>
        {OPERATIONS.map((kind) => (
          <button
            aria-label={descriptions[kind].label}
            aria-pressed={operation === kind}
            key={kind}
            onClick={() => {
              setPlaying(false)
              onOperationChange(kind)
            }}
            type="button"
          >
            <InlineMath math={descriptions[kind].math} />
          </button>
        ))}
      </div>
      <div className="dh-transform-player__status" aria-live="polite">
        <strong>当前子步骤：{descriptions[operation].label}</strong>
        <span>{operationIndex + 1} / 4</span>
      </div>
      <MatrixTable
        label={`完成当前子步骤后的累计变换（平移列为 mm）`}
        matrix={transformInMillimetres(decomposition.operations[operationIndex].cumulative)}
        precision={4}
        symbol={String.raw`\mathbf D_${row + 1}^{(${operationIndex + 1})}`}
      />
      <div className="dh-transform-player__actions">
        <button
          disabled={operationIndex === 0}
          onClick={() => onOperationChange(OPERATIONS[Math.max(0, operationIndex - 1)])}
          type="button"
        >
          上一个子步骤
        </button>
        <button onClick={() => setPlaying((value) => !value)} type="button">
          {playing ? '暂停 D–H 演示' : '播放 D–H 演示'}
        </button>
        <button
          disabled={operationIndex === OPERATIONS.length - 1}
          onClick={() => onOperationChange(OPERATIONS[Math.min(OPERATIONS.length - 1, operationIndex + 1)])}
          type="button"
        >
          下一个子步骤
        </button>
        <button
          onClick={() => {
            setPlaying(false)
            onOperationChange('rz')
          }}
          type="button"
        >
          重置当前 D–H 变换
        </button>
      </div>
      <p>这里展示的是坐标系变换的分解，不代表机器人实际按照四段轨迹运动。</p>
    </section>
  )
}
