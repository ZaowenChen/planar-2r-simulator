import { InlineMath } from 'react-katex'
import type { ReactNode } from 'react'
import type { RobotParameters } from '../../robotics/types'
import { metresToMillimetres } from '../kinematics/presentation'

export type DhParameter = 'theta' | 'd' | 'a' | 'alpha'

export interface DhTableProps {
  parameters: RobotParameters
  selectedRow?: 0 | 1 | 2
  selectedParameter?: DhParameter
  onRowSelect?: (row: 0 | 1 | 2) => void
  onParameterSelect?: (row: 0 | 1 | 2, parameter: DhParameter) => void
}

const PARAMETER_ACTION: Record<DhParameter, (row: number) => string> = {
  theta: (row) => `绕 z${String.fromCharCode(0x2080 + row)} 旋转 θ${String.fromCharCode(0x2081 + row)}`,
  d: (row) => `沿 z${String.fromCharCode(0x2080 + row)} 平移 d${String.fromCharCode(0x2081 + row)}`,
  a: (row) => `沿 x${String.fromCharCode(0x2081 + row)} 平移 a${String.fromCharCode(0x2081 + row)}`,
  alpha: (row) => `绕 x${String.fromCharCode(0x2081 + row)} 旋转 α${String.fromCharCode(0x2081 + row)}`,
}

export function DhTable({
  parameters,
  selectedRow,
  selectedParameter,
  onRowSelect,
  onParameterSelect,
}: DhTableProps) {
  const rows = [
    ['1', '\\theta_1', metresToMillimetres(parameters.geometry.d1).toFixed(3), '0', '90^\\circ'],
    ['2', '\\theta_2', '0', metresToMillimetres(parameters.geometry.l2).toFixed(3), '0^\\circ'],
    ['3', '\\theta_3', '0', metresToMillimetres(parameters.geometry.l3).toFixed(3), '0^\\circ'],
  ] as const

  return (
    <table aria-label="标准 D–H 参数表" className="dh-table">
      <caption>标准 D–H 参数表</caption>
      <thead>
        <tr>
          <th scope="col"><InlineMath math="i" /></th>
          <th scope="col"><InlineMath math="\theta_i" /></th>
          <th scope="col"><InlineMath math="d_i" /> (mm)</th>
          <th scope="col"><InlineMath math="a_i" /> (mm)</th>
          <th scope="col"><InlineMath math="\alpha_i" /> (°)</th>
        </tr>
      </thead>
      <tbody>
        {rows.map(([index, theta, d, a, alpha], rowIndex) => {
          const row = rowIndex as 0 | 1 | 2
          const cells: readonly [DhParameter, ReactNode][] = [
            ['theta', <InlineMath math={theta} />],
            ['d', d],
            ['a', a],
            ['alpha', <InlineMath math={alpha} />],
          ]
          return (
            <tr className={selectedRow === row ? 'is-selected' : undefined} key={index}>
              <th scope="row">
                {onRowSelect === undefined
                  ? index
                  : (
                    <button
                      aria-label={`选择 D–H 第 ${index} 行`}
                      aria-pressed={selectedRow === row}
                      onClick={() => onRowSelect(row)}
                      type="button"
                    >
                      {index}
                    </button>
                  )}
              </th>
              {cells.map(([parameter, value]) => (
                <td key={parameter}>
                  {onParameterSelect === undefined
                    ? value
                    : (
                      <button
                        aria-label={PARAMETER_ACTION[parameter](row)}
                        aria-pressed={selectedRow === row && selectedParameter === parameter}
                        onClick={() => onParameterSelect(row, parameter)}
                        type="button"
                      >
                        {value}
                      </button>
                    )}
                </td>
              ))}
            </tr>
          )
        })}
      </tbody>
    </table>
  )
}
