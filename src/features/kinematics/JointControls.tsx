import { useEffect, useState } from 'react'
import { NumericField } from '../../components/NumericField'
import { useLabStore } from '../../state/labStore'
import type { AngleUnit } from '../../state/labStore'

const RAD_TO_DEG = 180 / Math.PI

function displayAngle(radians: number, unit: AngleUnit): string {
  const value = unit === 'degrees' ? radians * RAD_TO_DEG : radians
  return Number(value.toFixed(unit === 'degrees' ? 2 : 4)).toString()
}

export function JointControls() {
  const q = useLabStore((state) => state.jointState.q)
  const angleUnit = useLabStore((state) => state.angleUnit)
  const setAngleUnit = useLabStore((state) => state.setAngleUnit)
  const setJoint = useLabStore((state) => state.setJoint)
  const [drafts, setDrafts] = useState(() => q.map((value) => displayAngle(value, angleUnit)))
  const [issues, setIssues] = useState<Record<number, string>>({})

  useEffect(() => {
    setDrafts(q.map((value) => displayAngle(value, angleUnit)))
    setIssues({})
  }, [angleUnit, q])

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
    setJoint(index, angleUnit === 'degrees' ? displayedValue / RAD_TO_DEG : displayedValue)
  }

  return (
    <section className="control-section">
      <div className="unit-switch" role="radiogroup" aria-label="角度显示单位">
        <label><input checked={angleUnit === 'degrees'} name="angle-unit" onChange={() => setAngleUnit('degrees')} type="radio" />角度</label>
        <label><input checked={angleUnit === 'radians'} name="angle-unit" onChange={() => setAngleUnit('radians')} type="radio" />弧度</label>
      </div>
      <div className="field-grid">
        {q.map((_, index) => (
          <NumericField
            error={issues[index]}
            key={index}
            label={`关节角 θ${['₁', '₂', '₃'][index]}`}
            onChange={(raw) => setDrafts((current) => current.map((value, draftIndex) => draftIndex === index ? raw : value))}
            onCommit={(raw) => commit(index, raw)}
            unit={angleUnit === 'degrees' ? '°' : 'rad'}
            value={drafts[index]}
          />
        ))}
      </div>
    </section>
  )
}
