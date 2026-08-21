import { InlineMath } from 'react-katex'
import type { Matrix4 } from '../../robotics/types'

export type HomogeneousTransformPart = 'rotation' | 'translation'

export interface HomogeneousTransformCardProps {
  matrix: Matrix4
  focusedPart: HomogeneousTransformPart | null
  onFocusPart: (part: HomogeneousTransformPart) => void
}

function format(value: number): string {
  return (Math.abs(value) < 1e-4 ? 0 : value).toFixed(4)
}

export function HomogeneousTransformCard({
  matrix,
  focusedPart,
  onFocusPart,
}: HomogeneousTransformCardProps) {
  return (
    <div className="homogeneous-transform-card">
      <div className="homogeneous-transform-card__controls" role="group" aria-label="齐次变换矩阵分块">
        <button
          aria-pressed={focusedPart === 'rotation'}
          onClick={() => onFocusPart('rotation')}
          type="button"
        >
          聚焦旋转块 R
        </button>
        <button
          aria-pressed={focusedPart === 'translation'}
          onClick={() => onFocusPart('translation')}
          type="button"
        >
          聚焦平移列 p
        </button>
      </div>
      <table aria-label="基座到末端齐次变换分块" className="matrix-table homogeneous-transform-card__matrix">
        <caption><InlineMath math="{}^0\mathbf T_3=[\,\mathbf R\;\mathbf p;\;\mathbf 0^{\mathsf T}\;1\,]" /></caption>
        <tbody>
          {matrix.map((row, rowIndex) => (
            <tr key={rowIndex}>
              {row.map((value, columnIndex) => {
                const part = rowIndex < 3 && columnIndex < 3
                  ? 'rotation'
                  : rowIndex < 3 && columnIndex === 3
                    ? 'translation'
                    : 'homogeneous'
                return (
                  <td
                    className={`${part === 'rotation' ? 'is-rotation' : part === 'translation' ? 'is-translation' : ''}${focusedPart === part ? ' is-focused' : ''}`}
                    key={columnIndex}
                  >
                    {format(value)}
                  </td>
                )
              })}
            </tr>
          ))}
        </tbody>
      </table>
      <dl>
        <div><dt>R</dt><dd>3×3 旋转块，无量纲；决定末端坐标轴方向。</dd></div>
        <div><dt>p</dt><dd>3×1 平移列，单位 mm；决定末端原点位置。</dd></div>
      </dl>
    </div>
  )
}
