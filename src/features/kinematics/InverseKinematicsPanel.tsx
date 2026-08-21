import { useEffect, useMemo, useState } from 'react'
import { NumericField } from '../../components/NumericField'
import { inverseKinematics } from '../../robotics/kinematics'
import type { InverseKinematicsSolution } from '../../robotics/kinematics'
import { useLabStore } from '../../state/labStore'
import {
  metresToMillimetres,
  millimetresToMetres,
  radiansToDegrees,
} from './presentation'
import {
  configurationId,
  type KinematicsConfigurationId,
} from './teachingState'

function branchLabel(solution: InverseKinematicsSolution): string {
  return solution.branch === 'elbow-down' ? '肘下构型' : '肘上构型'
}

function radialFamilyLabel(solution: InverseKinematicsSolution): string {
  return solution.radialFamily === 'conventional' ? '常规径向' : '折叠径向'
}

export interface InverseKinematicsPanelProps {
  activeConfigurationId: KinematicsConfigurationId
  onConfigurationChange: (configuration: KinematicsConfigurationId) => void
}

export function InverseKinematicsPanel({
  activeConfigurationId,
  onConfigurationChange,
}: InverseKinematicsPanelProps) {
  const desiredPosition = useLabStore((state) => state.desiredPosition)
  const parameters = useLabStore((state) => state.parameters)
  const q = useLabStore((state) => state.jointState.q)
  const setDesiredPosition = useLabStore((state) => state.setDesiredPosition)
  const setJointVector = useLabStore((state) => state.setJointVector)
  const displayPosition = (value: number) => Number(
    metresToMillimetres(value).toFixed(3),
  ).toString()
  const [drafts, setDrafts] = useState(() => desiredPosition.map(displayPosition))
  const [issues, setIssues] = useState<Record<number, string>>({})
  const result = useMemo(
    () => inverseKinematics(desiredPosition, parameters, q),
    [desiredPosition, parameters, q],
  )
  const selectedSolution = result.solutions.find((solution) => (
    configurationId(solution) === activeConfigurationId
  ))

  useEffect(() => setDrafts(desiredPosition.map(displayPosition)), [desiredPosition])

  const commitTarget = (index: number, raw: string) => {
    const value = Number(raw)
    if (raw.trim() === '' || !Number.isFinite(value)) {
      setIssues((current) => ({ ...current, [index]: '请输入有限坐标。' }))
      return
    }
    setIssues((current) => {
      const next = { ...current }
      delete next[index]
      return next
    })
    setDesiredPosition(index, millimetresToMetres(value))
  }

  const formatSolution = (solution: InverseKinematicsSolution) => solution.q
    .map((value) => `${radiansToDegrees(value).toFixed(1)}°`)
    .join('，')

  return (
    <section className="control-section ik-panel">
      <h4>期望末端位置</h4>
      <div className="field-grid">
        {desiredPosition.map((_, index) => (
          <NumericField
            error={issues[index]}
            key={index}
            label={`期望位置 ${['x', 'y', 'z'][index]}`}
            onChange={(raw) => setDrafts((current) => current.map((value, draftIndex) => draftIndex === index ? raw : value))}
            onCommit={(raw) => commitTarget(index, raw)}
            unit="mm"
            value={drafts[index]}
          />
        ))}
      </div>

      {(result.status === 'unreachable' || result.status === 'joint-limit') && (
        <div className="inline-alert" role="alert">
          {result.status === 'unreachable'
            ? '目标位置超出可达工作空间，当前关节构型保持不变。'
            : '目标几何可达，但所有逆解均超出关节限位，当前关节构型保持不变。'}
        </div>
      )}
      {result.status === 'axis-singular' && <div className="inline-alert" role="alert">目标位于基座轴线上，基座角采用零值约定。</div>}

      {result.solutions.length > 0 && (
        <fieldset className="ik-branches">
          <legend>逆运动学构型</legend>
          {result.solutions.map((solution) => (
            <label key={configurationId(solution)}>
              <input
                checked={activeConfigurationId === configurationId(solution)}
                name="ik-branch"
                onChange={() => onConfigurationChange(configurationId(solution))}
                type="radio"
              />
              <span>{branchLabel(solution)} · {radialFamilyLabel(solution)}：{formatSolution(solution)}</span>
            </label>
          ))}
        </fieldset>
      )}
      {result.solutions.length > 0 && selectedSolution === undefined && (
        <p className="control-sheet__note">当前三维图正在展示教学构型；该构型不在当前可应用的关节限位解中。</p>
      )}
      <button
        disabled={selectedSolution === undefined}
        onClick={() => selectedSolution && setJointVector(selectedSolution.q)}
        type="button"
      >
        应用当前预览解
      </button>
    </section>
  )
}
