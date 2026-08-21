import { StatusBanner } from '../../components/StatusBanner'
import type { RobotParameters } from '../../robotics/types'
import { radiansToDegrees } from './trajectoryModel'
import { PtpInstructionEditor } from './PtpInstructionEditor'
import { TeachPointList } from './TeachPointList'
import { useTrajectoryStore, type JogStepDegrees } from './trajectoryStore'

const JOINT_LABELS = ['J1 偏航', 'J2 肩部', 'J3 肘部'] as const
const JOG_STEPS: readonly JogStepDegrees[] = [0.1, 1, 5]

export function TeachPendantPanel({
  jointLimits,
}: {
  jointLimits: RobotParameters['jointLimits']
}) {
  const q = useTrajectoryStore((state) => state.currentQ)
  const jogStep = useTrajectoryStore((state) => state.jogStepDegrees)
  const error = useTrajectoryStore((state) => state.error)
  const jogJoint = useTrajectoryStore((state) => state.jogJoint)
  const setJogStep = useTrajectoryStore((state) => state.setJogStep)
  const recordPoint = useTrajectoryStore((state) => state.recordTeachPoint)

  return (
    <div className="teach-pendant">
      <div className="teach-pendant__header">
        <div>
          <p className="section-label">步骤 1 · 手动示教</p>
          <h3>虚拟示教器</h3>
        </div>
        <span>手动模式</span>
      </div>
      <p className="teach-pendant__intro">点动关节到目标姿态，再把当前位置记录为示教点。</p>
      <div className="jog-controls">
        {q.map((angle, index) => (
          <div className="jog-row" key={JOINT_LABELS[index]}>
            <strong>{JOINT_LABELS[index]}</strong>
            <button
              aria-label={`J${index + 1} 负向点动`}
              onClick={() => jogJoint(index, -1, jointLimits)}
              type="button"
            >−</button>
            <button
              aria-label={`J${index + 1} 正向点动`}
              onClick={() => jogJoint(index, 1, jointLimits)}
              type="button"
            >＋</button>
            <output aria-label={`J${index + 1} 当前角度`}>{radiansToDegrees(angle).toFixed(1)}°</output>
          </div>
        ))}
      </div>
      <fieldset className="jog-step-selector">
        <legend>点动步长</legend>
        {JOG_STEPS.map((step) => (
          <button
            aria-pressed={jogStep === step}
            key={step}
            onClick={() => setJogStep(step)}
            type="button"
          >{step}°</button>
        ))}
      </fieldset>
      <button className="record-point-button" onClick={recordPoint} type="button">记录当前位置为示教点</button>
      {error !== undefined && <StatusBanner tone="error" title="示教或轨迹输入需要调整">{error}</StatusBanner>}
      <TeachPointList jointLimits={jointLimits} />
      <PtpInstructionEditor jointLimits={jointLimits} />
    </div>
  )
}
