import { useId, useRef, useState } from 'react'
import type { KeyboardEvent, ReactNode } from 'react'
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
  const tabRefs = useRef<Array<HTMLButtonElement | null>>([])
  const id = useId().replaceAll(':', '')
  const panels: Record<FormulaView, ReactNode> = { definition, substitution, result }
  const selected = VIEWS.find((item) => item.key === view) ?? VIEWS[0]

  const activateTab = (index: number) => {
    setView(VIEWS[index].key)
    tabRefs.current[index]?.focus()
  }

  const handleTabKeyDown = (
    event: KeyboardEvent<HTMLButtonElement>,
    index: number,
  ) => {
    let targetIndex: number | undefined
    if (event.key === 'ArrowRight') targetIndex = (index + 1) % VIEWS.length
    if (event.key === 'ArrowLeft') targetIndex = (index - 1 + VIEWS.length) % VIEWS.length
    if (event.key === 'Home') targetIndex = 0
    if (event.key === 'End') targetIndex = VIEWS.length - 1
    if (targetIndex === undefined) return

    event.preventDefault()
    activateTab(targetIndex)
  }

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
              onKeyDown={(event) => handleTabKeyDown(event, VIEWS.indexOf(item))}
              ref={(node) => { tabRefs.current[VIEWS.indexOf(item)] = node }}
              role="tab"
              tabIndex={view === item.key ? 0 : -1}
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
