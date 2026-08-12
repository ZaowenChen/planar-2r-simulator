import { useEffect, useState } from 'react'
import { InlineMath } from 'react-katex'
import { MatrixTable } from '../../components/MatrixTable'
import { NumericField } from '../../components/NumericField'
import { DEFAULT_ROBOT_PARAMETERS } from '../../robotics/defaults'
import { symmetricEigenvalues3 } from '../../robotics/linalg'
import type { Matrix3 } from '../../robotics/types'
import { useLabStore } from '../../state/labStore'

const SUBSCRIPT_DIGITS = ['₁', '₂', '₃'] as const

interface ParameterDraftFieldProps {
  label: string
  path: string
  unit: string
  fallback: number
  constraint?: 'positive' | 'nonnegative' | 'finite'
  mirrorPath?: string
}

function localValidation(
  raw: string,
  constraint: NonNullable<ParameterDraftFieldProps['constraint']>,
): string | undefined {
  const value = Number(raw)
  if (raw.trim() === '' || !Number.isFinite(value)) return '请输入有限数值。'
  if (constraint === 'positive' && value <= 0) return '该参数必须为正数。'
  if (constraint === 'nonnegative' && value < 0) return '该参数不得为负数。'
  return undefined
}

export function ParameterDraftField({
  label,
  path,
  unit,
  fallback,
  constraint = 'finite',
  mirrorPath,
}: ParameterDraftFieldProps) {
  const storedRaw = useLabStore((state) => state.rawParameters[path] ?? String(fallback))
  const setParameterField = useLabStore((state) => state.setParameterField)
  const [draft, setDraft] = useState(storedRaw)
  const [error, setError] = useState<string>()

  useEffect(() => {
    setDraft(storedRaw)
    setError(undefined)
  }, [storedRaw])

  const commit = (raw: string) => {
    const issue = localValidation(raw, constraint)
    setError(issue)
    if (issue !== undefined) return
    setParameterField(path, raw)
    if (mirrorPath !== undefined) setParameterField(mirrorPath, raw)
  }

  return (
    <NumericField
      error={error}
      label={label}
      onChange={(raw) => {
        setDraft(raw)
        if (error !== undefined) setError(undefined)
      }}
      onCommit={commit}
      unit={unit}
      value={draft}
    />
  )
}

function rawNumber(
  rawParameters: Record<string, string>,
  path: string,
  fallback: number,
): number {
  const value = Number(rawParameters[path])
  return Number.isFinite(value) ? value : fallback
}

function inertiaIssueText(code: string): string {
  if (code === 'INERTIA_TRIANGLE') return '主惯性矩必须满足三角不等式。'
  if (code === 'INERTIA_NOT_POSITIVE_DEFINITE') return '所有主惯性矩都必须大于 10⁻⁹ kg·m²。'
  return '惯性张量必须是有限的对称矩阵。'
}

function resetLink(linkIndex: number): void {
  const setParameterField = useLabStore.getState().setParameterField
  const link = DEFAULT_ROBOT_PARAMETERS.links[linkIndex]
  setParameterField(`links.${linkIndex}.mass`, String(link.mass))
  link.centerOfMass.forEach((value, index) => {
    setParameterField(`links.${linkIndex}.centerOfMass.${index}`, String(value))
  })
  link.inertia.forEach((row, rowIndex) => {
    row.forEach((value, columnIndex) => {
      setParameterField(`links.${linkIndex}.inertia.${rowIndex}.${columnIndex}`, String(value))
    })
  })
}

export interface RigidBodyEditorProps {
  linkIndex: 0 | 1 | 2
}

export function RigidBodyEditor({ linkIndex }: RigidBodyEditorProps) {
  const link = useLabStore((state) => state.parameters.links[linkIndex])
  const rawParameters = useLabStore((state) => state.rawParameters)
  const parameterIssues = useLabStore((state) => state.parameterIssues)
  const linkNumber = linkIndex + 1
  const linkSubscript = SUBSCRIPT_DIGITS[linkIndex]
  const centerPath = `links.${linkIndex}.centerOfMass`
  const inertiaPath = `links.${linkIndex}.inertia`
  const draftCenter = link.centerOfMass.map((fallback, index) => (
    rawNumber(rawParameters, `${centerPath}.${index}`, fallback)
  ))
  const component = (row: number, column: number) => rawNumber(
    rawParameters,
    `${inertiaPath}.${row}.${column}`,
    link.inertia[row][column],
  )
  const draftInertia: Matrix3 = [
    [component(0, 0), component(0, 1), component(0, 2)],
    [component(0, 1), component(1, 1), component(1, 2)],
    [component(0, 2), component(1, 2), component(2, 2)],
  ]
  const principalInertias = symmetricEigenvalues3(draftInertia)
  const centerIssue = parameterIssues.find((issue) => issue.path === centerPath)
  const inertiaIssue = parameterIssues.find((issue) => issue.path === inertiaPath)

  return (
    <details className="control-section" open>
      <summary>连杆 {linkNumber} 刚体参数</summary>
      <div className="field-grid">
        <ParameterDraftField
          constraint="positive"
          fallback={link.mass}
          label={`连杆 ${linkNumber} 质量 m${linkSubscript}`}
          path={`links.${linkIndex}.mass`}
          unit="kg"
        />
        {(['x', 'y', 'z'] as const).map((axis, index) => (
          <ParameterDraftField
            fallback={draftCenter[index]}
            key={axis}
            label={`连杆 ${linkNumber} 质心 c${['ₓ', 'ᵧ', 'ᶻ'][index]}`}
            path={`${centerPath}.${index}`}
            unit="m"
          />
        ))}
      </div>
      {centerIssue !== undefined && (
        <p className="inline-alert" role="alert">质心必须位于连杆名义球体范围内。</p>
      )}

      <p className="section-label">六个独立惯性分量</p>
      <div className="field-grid">
        <ParameterDraftField fallback={component(0, 0)} label={`连杆 ${linkNumber} 惯性 Iₓₓ`} path={`${inertiaPath}.0.0`} unit="kg·m²" />
        <ParameterDraftField fallback={component(1, 1)} label={`连杆 ${linkNumber} 惯性 Iᵧᵧ`} path={`${inertiaPath}.1.1`} unit="kg·m²" />
        <ParameterDraftField fallback={component(2, 2)} label={`连杆 ${linkNumber} 惯性 Iᶻᶻ`} path={`${inertiaPath}.2.2`} unit="kg·m²" />
        <ParameterDraftField fallback={component(0, 1)} label={`连杆 ${linkNumber} 惯性 Iₓᵧ`} mirrorPath={`${inertiaPath}.1.0`} path={`${inertiaPath}.0.1`} unit="kg·m²" />
        <ParameterDraftField fallback={component(0, 2)} label={`连杆 ${linkNumber} 惯性 Iₓᶻ`} mirrorPath={`${inertiaPath}.2.0`} path={`${inertiaPath}.0.2`} unit="kg·m²" />
        <ParameterDraftField fallback={component(1, 2)} label={`连杆 ${linkNumber} 惯性 Iᵧᶻ`} mirrorPath={`${inertiaPath}.2.1`} path={`${inertiaPath}.1.2`} unit="kg·m²" />
      </div>

      <MatrixTable
        label={`连杆 ${linkNumber} 重构惯性张量`}
        matrix={draftInertia}
        precision={4}
        symbol={`{}^{${linkNumber}}\\mathbf I_{C_${linkNumber}}`}
        unit="kg·m²"
      />
      <p data-testid={`link-${linkNumber}-principal-inertias`}>
        主惯性矩 <InlineMath math={`\\lambda(\\mathbf I)=[${principalInertias.map((value) => value.toFixed(4)).join(',\\;')}]`} /> kg·m²
      </p>
      {inertiaIssue !== undefined && (
        <p className="inline-alert" role="alert">{inertiaIssueText(inertiaIssue.code)}</p>
      )}
      <button
        aria-label={`复位连杆 ${linkNumber} 刚体参数`}
        onClick={() => resetLink(linkIndex)}
        type="button"
      >
        复位本连杆
      </button>
    </details>
  )
}
