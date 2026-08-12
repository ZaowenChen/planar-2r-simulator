import { useEffect, useMemo, useState } from 'react'
import { NumericField } from '../../components/NumericField'
import { inverseKinematics } from '../../robotics/kinematics'
import type { InverseKinematicsSolution } from '../../robotics/kinematics'
import { useLabStore } from '../../state/labStore'

function branchLabel(solution: InverseKinematicsSolution): string {
  return solution.branch === 'elbow-down' ? '肘下解' : '肘上解'
}

export function InverseKinematicsPanel() {
  const desiredPosition = useLabStore((state) => state.desiredPosition)
  const parameters = useLabStore((state) => state.parameters)
  const q = useLabStore((state) => state.jointState.q)
  const angleUnit = useLabStore((state) => state.angleUnit)
  const setDesiredPosition = useLabStore((state) => state.setDesiredPosition)
  const setJoint = useLabStore((state) => state.setJoint)
  const [drafts, setDrafts] = useState(() => desiredPosition.map(String))
  const [issues, setIssues] = useState<Record<number, string>>({})
  const result = useMemo(
    () => inverseKinematics(desiredPosition, parameters, q),
    [desiredPosition, parameters, q],
  )
  const [selectedBranch, setSelectedBranch] = useState<InverseKinematicsSolution['branch']>('elbow-down')
  const selectedSolution = result.solutions.find((solution) => solution.branch === selectedBranch)
    ?? result.solutions[0]

  useEffect(() => setDrafts(desiredPosition.map(String)), [desiredPosition])

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
    setDesiredPosition(index, value)
  }

  const formatSolution = (solution: InverseKinematicsSolution) => solution.q
    .map((value) => angleUnit === 'degrees'
      ? `${(value * 180 / Math.PI).toFixed(1)}°`
      : `${value.toFixed(3)} rad`)
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
            unit="m"
            value={drafts[index]}
          />
        ))}
      </div>

      {(result.status === 'unreachable' || result.status === 'joint-limit') && (
        <div className="inline-alert" role="alert">
          {result.status === 'unreachable'
            ? '目标位置超出可达工作空间，当前关节姿态保持不变。'
            : '目标几何可达，但所有逆解均超出关节限位，当前关节姿态保持不变。'}
        </div>
      )}
      {result.status === 'axis-singular' && <div className="inline-alert" role="alert">目标位于基座轴线上，基座角采用零值约定。</div>}

      {result.solutions.length > 0 && (
        <fieldset className="ik-branches">
          <legend>逆运动学分支</legend>
          {result.solutions.map((solution) => (
            <label key={solution.branch}>
              <input
                checked={(selectedSolution?.branch ?? selectedBranch) === solution.branch}
                name="ik-branch"
                onChange={() => setSelectedBranch(solution.branch)}
                type="radio"
              />
              <span>{branchLabel(solution)}：{formatSolution(solution)}</span>
            </label>
          ))}
        </fieldset>
      )}
      <button
        disabled={selectedSolution === undefined}
        onClick={() => selectedSolution?.q.forEach((value, index) => setJoint(index, value))}
        type="button"
      >
        应用所选逆解
      </button>
    </section>
  )
}
