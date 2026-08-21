import { InlineMath } from 'react-katex'
import { MatrixTable } from '../../components/MatrixTable'
import type { Vector3 } from '../../robotics/types'
import type { InverseSolutionDerivation } from './derivationModel'

export interface VerificationSummaryProps {
  targetMm: Vector3
  solutions: readonly InverseSolutionDerivation[]
}

function vectorLatex(vector: readonly number[], precision: number): string {
  return String.raw`\begin{bmatrix}${vector.map((value) => (
    Math.abs(value) < 10 ** (-precision) ? '0' : value.toFixed(precision)
  )).join('\\')}\end{bmatrix}`
}

function configurationLabel(detail: InverseSolutionDerivation): string {
  const elbow = detail.solution.branch === 'elbow-down' ? '肘下' : '肘上'
  const radial = detail.solution.radialFamily === 'conventional' ? '常规径向' : '折叠径向'
  return `${elbow} · ${radial}`
}

export function VerificationSummary({ targetMm, solutions }: VerificationSummaryProps) {
  if (solutions.length === 0) {
    return <p>当前没有满足关节限位的解析解可供回代。</p>
  }
  return (
    <div className="verification-summary">
      <div className="verification-summary__table-wrap">
        <table aria-label="逆解回代比较" className="verification-summary__table">
          <thead>
            <tr>
              <th scope="col">构型</th>
              <th scope="col">q<sub>IK</sub> (°)</th>
              <th scope="col">Δx / Δy / Δz (mm)</th>
              <th scope="col">e<sub>p</sub> (mm)</th>
              <th scope="col">β (°)</th>
            </tr>
          </thead>
          <tbody>
            {solutions.map((detail) => (
              <tr key={`${detail.solution.radialFamily}:${detail.solution.branch}`}>
                <th scope="row">{configurationLabel(detail)}</th>
                <td>{detail.qDegrees.map((value) => value.toFixed(2)).join(' / ')}</td>
                <td>{detail.positionResidualMm.map((value) => value.toExponential(2)).join(' / ')}</td>
                <td>{detail.positionErrorMm.toExponential(3)}</td>
                <td>{detail.toolElevationDegrees.toFixed(2)}</td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
      <div className="verification-summary__details">
        {solutions.map((detail) => (
          <details key={`${detail.solution.radialFamily}:${detail.solution.branch}`}>
            <summary>展开完整回代数据 · {configurationLabel(detail)}</summary>
            <dl>
              <div><dt>解析关节解 q<sub>IK</sub></dt><dd><InlineMath math={String.raw`${vectorLatex(detail.qDegrees, 2)}\ ^\circ`} /></dd></div>
              <div><dt>目标位置 p<sub>d</sub></dt><dd><InlineMath math={String.raw`${vectorLatex(targetMm, 6)}\ \mathrm{mm}`} /></dd></div>
              <div><dt>回代位置 p<sub>FK</sub></dt><dd><InlineMath math={String.raw`${vectorLatex(detail.achievedPositionMm, 6)}\ \mathrm{mm}`} /></dd></div>
              <div><dt>分量误差 Δp</dt><dd><InlineMath math={String.raw`${vectorLatex(detail.positionResidualMm, 6)}\ \mathrm{mm}`} /></dd></div>
            </dl>
            <MatrixTable
              label={`${configurationLabel(detail)}末端姿态`}
              matrix={detail.orientation}
              precision={4}
              symbol="{}^0\mathbf R_3"
            />
          </details>
        ))}
      </div>
    </div>
  )
}
