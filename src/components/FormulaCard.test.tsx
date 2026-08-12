import { cleanup, render, screen } from '@testing-library/react'
import userEvent from '@testing-library/user-event'
import { afterEach, describe, expect, it } from 'vitest'
import { FormulaCard } from './FormulaCard'

afterEach(cleanup)

describe('FormulaCard', () => {
  it('exposes only the selected definition, substitution, or result panel', async () => {
    const user = userEvent.setup()
    render(
      <FormulaCard
        title="末端位置"
        definition="definition-panel"
        substitution="substitution-panel"
        result="result-panel"
        symbols={[{ symbol: 'p', meaning: '末端位置', unit: 'm' }]}
      />,
    )

    expect(screen.getByRole('tabpanel')).toHaveTextContent('definition-panel')
    expect(screen.queryByText('substitution-panel')).not.toBeInTheDocument()
    expect(screen.queryByText('result-panel')).not.toBeInTheDocument()

    await user.click(screen.getByRole('tab', { name: '代入' }))
    expect(screen.getByRole('tabpanel')).toHaveTextContent('substitution-panel')
    expect(screen.queryByText('definition-panel')).not.toBeInTheDocument()

    await user.click(screen.getByRole('tab', { name: '结果' }))
    expect(screen.getByRole('tabpanel')).toHaveTextContent('result-panel')
    expect(screen.getByRole('tab', { name: '结果' })).toHaveAttribute('aria-selected', 'true')
    expect(screen.queryByText('substitution-panel')).not.toBeInTheDocument()
  })

  it('renders the supplied symbol glossary for readers', () => {
    render(
      <FormulaCard
        title="动力学"
        definition="equation"
        substitution="values"
        result="answer"
        symbols={[{ symbol: 'tau', meaning: '关节力矩', unit: 'N·m' }]}
      />,
    )

    expect(screen.getByText('关节力矩')).toBeInTheDocument()
    expect(screen.getByText('N·m')).toBeInTheDocument()
  })

  it('uses roving focus and activates tabs from the keyboard', async () => {
    const user = userEvent.setup()
    render(
      <FormulaCard
        title="末端位置"
        definition="definition-panel"
        substitution="substitution-panel"
        result="result-panel"
      />,
    )
    const [definitionTab, substitutionTab, resultTab] = screen.getAllByRole('tab')

    expect(definitionTab).toHaveAttribute('tabindex', '0')
    expect(substitutionTab).toHaveAttribute('tabindex', '-1')
    expect(resultTab).toHaveAttribute('tabindex', '-1')

    definitionTab.focus()
    await user.keyboard('{ArrowLeft}')
    expect(resultTab).toHaveFocus()
    expect(resultTab).toHaveAttribute('aria-selected', 'true')
    expect(screen.getByRole('tabpanel')).toHaveTextContent('result-panel')
    expect(screen.queryByText('definition-panel')).not.toBeInTheDocument()

    await user.keyboard('{Home}')
    expect(definitionTab).toHaveFocus()
    expect(definitionTab).toHaveAttribute('tabindex', '0')
    expect(screen.getByRole('tabpanel')).toHaveTextContent('definition-panel')

    await user.keyboard('{End}')
    expect(resultTab).toHaveFocus()
    await user.keyboard('{ArrowRight}')
    expect(definitionTab).toHaveFocus()
    expect(screen.getByRole('tabpanel')).toHaveTextContent('definition-panel')
  })
})
