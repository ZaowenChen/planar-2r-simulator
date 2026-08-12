import { useEffect, useState } from 'react'
import { InlineMath } from 'react-katex'
import { NumericField } from '../../components/NumericField'
import { RobotScene } from '../../scene/RobotScene'
import { useLabStore } from '../../state/labStore'
import { WorkbenchLayout } from '../../app/WorkbenchLayout'
import { DhTable } from './DhTable'

interface ParameterFieldProps {
  label: string
  path: string
  unit: string
  value: number
  constraint: 'nonnegative' | 'positive' | 'finite'
}

function validationMessage(constraint: ParameterFieldProps['constraint'], raw: string): string | undefined {
  const value = Number(raw)
  if (raw.trim() === '' || !Number.isFinite(value)) return '请输入有限数值。'
  if (constraint === 'positive' && value <= 0) return '连杆长度或质量必须为正数。'
  if (constraint === 'nonnegative' && value < 0) return '该参数不得为负数。'
  return undefined
}

function ParameterField({ label, path, unit, value, constraint }: ParameterFieldProps) {
  const setParameterField = useLabStore((state) => state.setParameterField)
  const storeIssue = useLabStore((state) => state.fieldIssues[path])
  const [draft, setDraft] = useState(String(value))
  const [localIssue, setLocalIssue] = useState<string>()

  useEffect(() => setDraft(String(value)), [value])

  const commit = (raw: string) => {
    const issue = validationMessage(constraint, raw)
    setLocalIssue(issue)
    if (issue === undefined) setParameterField(path, raw)
  }

  let error = localIssue
  if (storeIssue !== undefined && error === undefined) error = '参数组合不满足机器人模型约束。'
  if (path === 'geometry.l2' && localIssue?.includes('正数')) error = '连杆长度必须为正数。'
  if (path === 'geometry.l3' && localIssue?.includes('正数')) error = '连杆长度必须为正数。'

  return (
    <NumericField
      error={error}
      label={label}
      onChange={(raw) => {
        setDraft(raw)
        if (localIssue !== undefined) setLocalIssue(undefined)
      }}
      onCommit={commit}
      unit={unit}
      value={draft}
    />
  )
}

export function RobotModelPage() {
  const parameters = useLabStore((state) => state.parameters)
  const calculation = useLabStore((state) => state.calculation)
  const jointState = useLabStore((state) => state.jointState)
  const sceneCalculation = {
    forward: calculation.forward,
    jacobian: calculation.jacobian,
    jointState,
    torque: calculation.dynamics.tau,
    gravity: parameters.gravity,
  }

  return (
    <WorkbenchLayout
      visual={<RobotScene calculation={sceneCalculation} />}
      controls={(
        <div className="control-sheet model-controls">
          <p className="section-label">模型参数</p>
          <h3>几何与环境参数</h3>
          <div className="field-grid">
            <ParameterField constraint="nonnegative" label="基座高度 d₁" path="geometry.d1" unit="m" value={parameters.geometry.d1} />
            <ParameterField constraint="positive" label="第二连杆长度 l₂" path="geometry.l2" unit="m" value={parameters.geometry.l2} />
            <ParameterField constraint="positive" label="第三连杆长度 l₃" path="geometry.l3" unit="m" value={parameters.geometry.l3} />
            {parameters.links.map((link, index) => (
              <ParameterField
                constraint="positive"
                key={`mass-${index}`}
                label={`连杆 ${index + 1} 质量 m${index + 1}`}
                path={`links.${index}.mass`}
                unit="kg"
                value={link.mass}
              />
            ))}
            {parameters.gravity.map((value, index) => (
              <ParameterField
                constraint="finite"
                key={`gravity-${index}`}
                label={`重力加速度 ${['gₓ', 'gᵧ', 'g_z'][index]}`}
                path={`gravity.${index}`}
                unit="m/s²"
                value={value}
              />
            ))}
          </div>
        </div>
      )}
      analysis={(
        <div className="analysis-stack model-analysis">
          <section>
            <p className="section-label">建系约定</p>
            <h3>坐标系定义</h3>
            <p>基坐标系 <InlineMath math="\{0\}" /> 的 z 轴竖直向上；各关节轴采用右手定则，末端坐标系 <InlineMath math="\{e\}" /> 固连于第三连杆末端。</p>
            <p>模型为偏航–俯仰–俯仰 3R 串联机构，长度、质量与重力统一采用 SI 制。</p>
          </section>
          <DhTable parameters={parameters} />
        </div>
      )}
    />
  )
}
