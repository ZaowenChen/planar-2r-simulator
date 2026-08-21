import { useEffect, useState } from 'react'
import { NumericField } from '../../components/NumericField'
import { useLabStore } from '../../state/labStore'
import { degreesToRadians, radiansToDegrees } from './presentation'

function displayDegrees(radians: number): string {
  return Number(radiansToDegrees(radians).toFixed(2)).toString()
}

export function JointMotionControls() {
  const q = useLabStore((state) => state.jointState.q)
  const qd = useLabStore((state) => state.jointState.qd)
  const setJoint = useLabStore((state) => state.setJoint)
  const setJointVelocity = useLabStore((state) => state.setJointVelocity)
  const [qDrafts, setQDrafts] = useState(() => q.map(displayDegrees))
  const [qdDrafts, setQdDrafts] = useState(() => qd.map(displayDegrees))
  const [issues, setIssues] = useState<Record<string, string>>({})

  useEffect(() => {
    setQDrafts(q.map(displayDegrees))
    setIssues((current) => Object.fromEntries(
      Object.entries(current).filter(([key]) => key.startsWith('qd-')),
    ))
  }, [q])

  useEffect(() => {
    setQdDrafts(qd.map(displayDegrees))
    setIssues((current) => Object.fromEntries(
      Object.entries(current).filter(([key]) => key.startsWith('q-')),
    ))
  }, [qd])

  const commit = (
    kind: 'q' | 'qd',
    index: number,
    raw: string,
  ) => {
    const value = Number(raw)
    const key = `${kind}-${index}`
    if (raw.trim() === '' || !Number.isFinite(value)) {
      setIssues((current) => ({ ...current, [key]: '请输入有限角度。' }))
      return
    }
    setIssues((current) => {
      const next = { ...current }
      delete next[key]
      return next
    })
    if (kind === 'q') setJoint(index, degreesToRadians(value))
    else setJointVelocity(index, degreesToRadians(value))
  }

  return (
    <section className="control-section joint-motion-controls">
      <div>
        <h4>当前关节角 q</h4>
        <div className="field-grid">
          {q.map((_, index) => (
            <NumericField
              error={issues[`q-${index}`]}
              key={`q-${index}`}
              label={`关节角 θ${['₁', '₂', '₃'][index]}`}
              onChange={(raw) => setQDrafts((current) => current.map((value, draftIndex) => draftIndex === index ? raw : value))}
              onCommit={(raw) => commit('q', index, raw)}
              unit="°"
              value={qDrafts[index]}
            />
          ))}
        </div>
      </div>
      <div>
        <h4>当前关节速度 q̇</h4>
        <div className="field-grid">
          {qd.map((_, index) => (
            <NumericField
              error={issues[`qd-${index}`]}
              key={`qd-${index}`}
              label={`关节速度 θ̇${['₁', '₂', '₃'][index]}`}
              onChange={(raw) => setQdDrafts((current) => current.map((value, draftIndex) => draftIndex === index ? raw : value))}
              onCommit={(raw) => commit('qd', index, raw)}
              unit="°/s"
              value={qdDrafts[index]}
            />
          ))}
        </div>
      </div>
    </section>
  )
}
