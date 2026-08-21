import { useEffect, useState } from 'react'
import { NumericField } from '../../components/NumericField'
import { useLabStore } from '../../state/labStore'
import { degreesToRadians, radiansToDegrees } from './presentation'

function displayAngle(radians: number): string {
  return Number(radiansToDegrees(radians).toFixed(2)).toString()
}

export function JointControls() {
  const q = useLabStore((state) => state.jointState.q)
  const setJoint = useLabStore((state) => state.setJoint)
  const [drafts, setDrafts] = useState(() => q.map(displayAngle))
  const [issues, setIssues] = useState<Record<number, string>>({})

  useEffect(() => {
    setDrafts(q.map(displayAngle))
    setIssues({})
  }, [q])

  const commit = (index: number, raw: string) => {
    const displayedValue = Number(raw)
    if (raw.trim() === '' || !Number.isFinite(displayedValue)) {
      setIssues((current) => ({ ...current, [index]: '请输入有限角度。' }))
      return
    }
    setIssues((current) => {
      const next = { ...current }
      delete next[index]
      return next
    })
    setJoint(index, degreesToRadians(displayedValue))
  }

  return (
    <section className="control-section">
      <div className="field-grid">
        {q.map((_, index) => (
          <NumericField
            error={issues[index]}
            key={index}
            label={`关节角 θ${['₁', '₂', '₃'][index]}`}
            onChange={(raw) => setDrafts((current) => current.map((value, draftIndex) => draftIndex === index ? raw : value))}
            onCommit={(raw) => commit(index, raw)}
            unit="°"
            value={drafts[index]}
          />
        ))}
      </div>
    </section>
  )
}
