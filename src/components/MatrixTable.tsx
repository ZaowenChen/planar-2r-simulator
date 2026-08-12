import { InlineMath } from 'react-katex'

export interface MatrixTableProps {
  matrix: readonly (readonly number[])[]
  label: string
  symbol?: string
  unit?: string
  precision?: number
}

function formatNumber(value: number, precision: number): string {
  if (!Number.isFinite(value)) return '—'
  const rounded = Math.abs(value) < 10 ** (-precision) ? 0 : value
  return rounded.toFixed(precision)
}

export function MatrixTable({
  matrix,
  label,
  symbol,
  unit,
  precision = 3,
}: MatrixTableProps) {
  return (
    <div className="matrix-table-wrap">
      <table className="matrix-table" aria-label={label}>
        <caption>
          {symbol !== undefined && <InlineMath math={symbol} />}
          <span>{label}</span>
          {unit !== undefined && <small>{unit}</small>}
        </caption>
        <tbody>
          {matrix.map((row, rowIndex) => (
            <tr key={rowIndex}>
              {row.map((value, columnIndex) => (
                <td key={columnIndex}>{formatNumber(value, precision)}</td>
              ))}
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  )
}
