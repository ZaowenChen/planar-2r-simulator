import { useId, useState } from 'react'
import type { ReactNode } from 'react'
import { InlineMath } from 'react-katex'

export interface FormulaSymbol {
  symbol: string
  meaning: string
  unit?: string
}

export interface FormulaCardProps {
  title: string
  definition: ReactNode
  substitution: ReactNode
  result: ReactNode
  symbols?: readonly FormulaSymbol[]
}

const VIEWS = [
  { key: 'definition', label: '定义' },
  { key: 'substitution', label: '代入' },
  { key: 'result', label: '结果' },
] as const

type FormulaView = typeof VIEWS[number]['key']

export function FormulaCard({
  title,
  definition,
  substitution,
  result,
  symbols = [],
}: FormulaCardProps) {
  const [view, setView] = useState<FormulaView>('definition')
  const id = useId().replaceAll(':', '')
  const panels: Record<FormulaView, ReactNode> = { definition, substitution, result }
  const selected = VIEWS.find((item) => item.key === view) ?? VIEWS[0]

  return (
    <article className="formula-card">
      <header className="formula-card__header">
        <h3>{title}</h3>
        <div className="formula-card__tabs" role="tablist" aria-label={`${title}公式视图`}>
          {VIEWS.map((item) => (
            <button
              aria-controls={`${id}-${item.key}-panel`}
              aria-selected={view === item.key}
              id={`${id}-${item.key}-tab`}
              key={item.key}
              onClick={() => setView(item.key)}
              role="tab"
              type="button"
            >
              {item.label}
            </button>
          ))}
        </div>
      </header>
      <div
        aria-labelledby={`${id}-${selected.key}-tab`}
        className="formula-card__panel"
        id={`${id}-${selected.key}-panel`}
        role="tabpanel"
      >
        {panels[view]}
      </div>
      {symbols.length > 0 && (
        <dl className="formula-card__symbols">
          {symbols.map(({ symbol, meaning, unit }) => (
            <div key={`${symbol}-${meaning}`}>
              <dt><InlineMath math={symbol} /></dt>
              <dd>{meaning}{unit !== undefined && <span>{unit}</span>}</dd>
            </div>
          ))}
        </dl>
      )}
    </article>
  )
}
