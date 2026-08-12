import { NumericField } from '../../components/NumericField'
import type { ExperimentMode } from '../../state/labStore'

export interface ExperimentDraft {
  duration: string
  stepSize: string
  inverseType: 'quintic' | 'sinusoidal'
  forwardType: 'constant' | 'step' | 'sine' | 'piecewise-constant'
  primary: readonly [string, string, string]
  secondary: readonly [string, string, string]
  frequency: readonly [string, string, string]
  phase: readonly [string, string, string]
  eventTime: string
  pieces: string
}

interface SimulationControlsProps {
  draft: ExperimentDraft
  error?: string
  mode: ExperimentMode
  onDraftChange: (draft: ExperimentDraft) => void
  onGenerate: () => void
  onModeChange: (mode: ExperimentMode) => void
}

const SUBSCRIPTS = ['₁', '₂', '₃'] as const

function updateVector(
  draft: ExperimentDraft,
  key: 'primary' | 'secondary' | 'frequency' | 'phase',
  index: number,
  value: string,
): ExperimentDraft {
  const vector = [...draft[key]] as [string, string, string]
  vector[index] = value
  return { ...draft, [key]: vector }
}

function VectorEditor({
  draft,
  field,
  label,
  onChange,
  unit,
}: {
  draft: ExperimentDraft
  field: 'primary' | 'secondary' | 'frequency' | 'phase'
  label: string
  onChange: (draft: ExperimentDraft) => void
  unit: string
}) {
  return (
    <div className="field-grid">
      {draft[field].map((value, index) => (
        <NumericField
          key={`${field}-${index}`}
          label={`${label}${SUBSCRIPTS[index]}`}
          onChange={(next) => onChange(updateVector(draft, field, index, next))}
          unit={unit}
          value={value}
        />
      ))}
    </div>
  )
}

export function SimulationControls({
  draft,
  error,
  mode,
  onDraftChange,
  onGenerate,
  onModeChange,
}: SimulationControlsProps) {
  return (
    <div className="control-sheet experiment-editor">
      <p className="section-label">实验输入</p>
      <h3>动力学实验编辑器</h3>
      <div aria-label="实验模式" className="experiment-tabs" role="tablist">
        {([['inverse', '逆动力学'], ['forward', '正动力学']] as const).map(([value, label]) => (
          <button
            aria-selected={mode === value}
            key={value}
            onClick={() => onModeChange(value)}
            role="tab"
            type="button"
          >{label}</button>
        ))}
      </div>

      <section className="control-section">
        <NumericField
          label="持续时间"
          onChange={(duration) => onDraftChange({ ...draft, duration })}
          unit="s"
          value={draft.duration}
        />
        <NumericField
          label="积分步长"
          onChange={(stepSize) => onDraftChange({ ...draft, stepSize })}
          unit="s"
          value={draft.stepSize}
        />
      </section>

      {mode === 'inverse' ? (
        <section className="control-section">
          <label>关节轨迹类型
            <select
              aria-label="关节轨迹类型"
              onChange={(event) => onDraftChange({
                ...draft,
                inverseType: event.target.value as ExperimentDraft['inverseType'],
              })}
              value={draft.inverseType}
            >
              <option value="quintic">五次多项式</option>
              <option value="sinusoidal">正弦轨迹</option>
            </select>
          </label>
          <VectorEditor draft={draft} field="primary" label={draft.inverseType === 'quintic' ? '初始角 θ' : '中心角 θ'} onChange={onDraftChange} unit="rad" />
          <VectorEditor draft={draft} field="secondary" label={draft.inverseType === 'quintic' ? '目标角 θ' : '振幅 A'} onChange={onDraftChange} unit="rad" />
          {draft.inverseType === 'sinusoidal' && <>
            <VectorEditor draft={draft} field="frequency" label="频率 f" onChange={onDraftChange} unit="Hz" />
            <VectorEditor draft={draft} field="phase" label="相位 φ" onChange={onDraftChange} unit="rad" />
          </>}
        </section>
      ) : (
        <section className="control-section">
          <label>力矩输入类型
            <select
              aria-label="力矩输入类型"
              onChange={(event) => onDraftChange({
                ...draft,
                forwardType: event.target.value as ExperimentDraft['forwardType'],
              })}
              value={draft.forwardType}
            >
              <option value="constant">常值</option>
              <option value="step">阶跃</option>
              <option value="sine">正弦</option>
              <option value="piecewise-constant">分段常值</option>
            </select>
          </label>
          {draft.forwardType === 'piecewise-constant' ? (
            <label>分段力矩（时间:τ₁,τ₂,τ₃；…）
              <input aria-label="分段力矩" onChange={(event) => onDraftChange({ ...draft, pieces: event.target.value })} value={draft.pieces} />
            </label>
          ) : <>
            <VectorEditor draft={draft} field="primary" label={draft.forwardType === 'sine' ? '偏置 τ' : '力矩 τ'} onChange={onDraftChange} unit="N·m" />
            {(draft.forwardType === 'step' || draft.forwardType === 'sine') && (
              <VectorEditor draft={draft} field="secondary" label={draft.forwardType === 'step' ? '阶跃后 τ' : '振幅 A'} onChange={onDraftChange} unit="N·m" />
            )}
            {draft.forwardType === 'step' && <NumericField label="阶跃时刻" onChange={(eventTime) => onDraftChange({ ...draft, eventTime })} unit="s" value={draft.eventTime} />}
            {draft.forwardType === 'sine' && <>
              <VectorEditor draft={draft} field="frequency" label="频率 f" onChange={onDraftChange} unit="Hz" />
              <VectorEditor draft={draft} field="phase" label="相位 φ" onChange={onDraftChange} unit="rad" />
            </>}
          </>}
        </section>
      )}
      {error !== undefined && <p className="inline-alert" role="alert">{error}</p>}
      <button onClick={onGenerate} type="button">生成实验</button>
      <p className="control-sheet__note">仅在生成实验时提交有效草稿；播放只读取预计算样本。</p>
    </div>
  )
}
