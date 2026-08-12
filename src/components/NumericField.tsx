import { useId } from 'react'
import type { InputHTMLAttributes, KeyboardEvent } from 'react'

export interface NumericFieldProps extends Omit<
  InputHTMLAttributes<HTMLInputElement>,
  'onChange' | 'type' | 'value'
> {
  label: string
  value: string | number
  unit?: string
  error?: string
  onChange: (value: string) => void
  onCommit?: (value: string) => void
}

export function NumericField({
  label,
  value,
  unit,
  error,
  onChange,
  onCommit,
  onBlur,
  onKeyDown,
  ...inputProps
}: NumericFieldProps) {
  const id = useId()
  const errorId = `${id}-error`
  const unitId = `${id}-unit`
  const describedBy = [unit !== undefined ? unitId : undefined, error !== undefined ? errorId : undefined]
    .filter(Boolean)
    .join(' ') || undefined
  const commit = () => onCommit?.(String(value))
  const handleKeyDown = (event: KeyboardEvent<HTMLInputElement>) => {
    onKeyDown?.(event)
    if (!event.defaultPrevented && event.key === 'Enter') {
      commit()
    }
  }

  return (
    <label className="numeric-field" htmlFor={id}>
      <span className="numeric-field__label">{label}</span>
      <span className="numeric-field__control">
        <input
          {...inputProps}
          aria-describedby={describedBy}
          aria-invalid={error !== undefined}
          aria-label={label}
          id={id}
          inputMode="decimal"
          onBlur={(event) => {
            onBlur?.(event)
            commit()
          }}
          onChange={(event) => onChange(event.target.value)}
          onKeyDown={handleKeyDown}
          type="text"
          value={value}
        />
        {unit !== undefined && <span className="numeric-field__unit" id={unitId}>{unit}</span>}
      </span>
      {error !== undefined && <span className="numeric-field__error" id={errorId}>{error}</span>}
    </label>
  )
}
