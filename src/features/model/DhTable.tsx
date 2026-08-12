import { InlineMath } from 'react-katex'
import type { RobotParameters } from '../../robotics/types'

export function DhTable({ parameters }: { parameters: RobotParameters }) {
  const rows = [
    ['1', '\\theta_1', parameters.geometry.d1.toFixed(3), '0', '\\pi/2'],
    ['2', '\\theta_2', '0', parameters.geometry.l2.toFixed(3), '0'],
    ['3', '\\theta_3', '0', parameters.geometry.l3.toFixed(3), '0'],
  ] as const

  return (
    <table aria-label="标准 D–H 参数表" className="dh-table">
      <caption>标准 D–H 参数表</caption>
      <thead>
        <tr>
          <th scope="col"><InlineMath math="i" /></th>
          <th scope="col"><InlineMath math="\theta_i" /></th>
          <th scope="col"><InlineMath math="d_i" /> (m)</th>
          <th scope="col"><InlineMath math="a_i" /> (m)</th>
          <th scope="col"><InlineMath math="\alpha_i" /> (rad)</th>
        </tr>
      </thead>
      <tbody>
        {rows.map(([index, theta, d, a, alpha]) => (
          <tr key={index}>
            <th scope="row">{index}</th>
            <td><InlineMath math={theta} /></td>
            <td>{d}</td>
            <td>{a}</td>
            <td><InlineMath math={alpha} /></td>
          </tr>
        ))}
      </tbody>
    </table>
  )
}
