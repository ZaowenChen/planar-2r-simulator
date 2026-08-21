import { NumericField } from '../../components/NumericField'
import type { RobotParameters } from '../../robotics/types'
import { radiansToDegrees } from './trajectoryModel'
import { useTrajectoryStore } from './trajectoryStore'

function peakText(values: readonly number[], unit: string): string {
  return values.map((value, index) => `J${index + 1} ${radiansToDegrees(value).toFixed(2)} ${unit}`).join(' · ')
}

export function PtpInstructionEditor({
  jointLimits,
}: {
  jointLimits: RobotParameters['jointLimits']
}) {
  const points = useTrajectoryStore((state) => state.teachPoints)
  const draft = useTrajectoryStore((state) => state.draft)
  const preview = useTrajectoryStore((state) => state.preview)
  const setDraft = useTrajectoryStore((state) => state.setDraft)
  const generatePreview = useTrajectoryStore((state) => state.generatePreview)
  const start = points.find((point) => point.id === draft.startPointId)
  const finish = points.find((point) => point.id === draft.endPointId)

  return (
    <section className="ptp-instruction" aria-labelledby="ptp-instruction-title">
      <div className="trajectory-section-heading">
        <div>
          <span>步骤 2</span>
          <h4 id="ptp-instruction-title">建立 PTP 指令</h4>
        </div>
        {start !== undefined && finish !== undefined && (
          <code>PTP {start.name} → {finish.name}</code>
        )}
      </div>
      <div className="ptp-point-pair">
        <label>PTP 起点
          <select
            aria-label="PTP 起点"
            onChange={(event) => setDraft({ startPointId: event.target.value }, jointLimits)}
            value={draft.startPointId}
          >
            <option value="">选择起点</option>
            {points.map((point) => <option key={point.id} value={point.id}>{point.name}</option>)}
          </select>
        </label>
        <span aria-hidden="true">→</span>
        <label>PTP 终点
          <select
            aria-label="PTP 终点"
            onChange={(event) => setDraft({ endPointId: event.target.value }, jointLimits)}
            value={draft.endPointId}
          >
            <option value="">选择终点</option>
            {points.map((point) => <option key={point.id} value={point.id}>{point.name}</option>)}
          </select>
        </label>
      </div>
      <div className="ptp-settings">
        <label>轨迹类型
          <select
            aria-label="PTP 轨迹类型"
            onChange={(event) => setDraft({
              profile: event.target.value as 'quintic' | 'trapezoidal',
            }, jointLimits)}
            value={draft.profile}
          >
            <option value="quintic">五次多项式</option>
            <option value="trapezoidal">梯形速度</option>
          </select>
        </label>
        <NumericField
          label="PTP 持续时间"
          onChange={(durationText) => setDraft({ durationText }, jointLimits)}
          unit="s"
          value={draft.durationText}
        />
      </div>
      <button
        className="trajectory-primary-action"
        onClick={() => generatePreview(jointLimits)}
        type="button"
      >生成轨迹预览</button>
      {preview !== null && (
        <dl className="trajectory-metrics">
          <div><dt>峰值速度</dt><dd>{peakText(preview.metrics.peakVelocity, '°/s')}</dd></div>
          <div><dt>峰值加速度</dt><dd>{peakText(preview.metrics.peakAcceleration, '°/s²')}</dd></div>
          <div><dt>端点误差</dt><dd>{Math.max(...preview.metrics.endError.map(Math.abs)).toExponential(2)} rad</dd></div>
        </dl>
      )}
    </section>
  )
}

