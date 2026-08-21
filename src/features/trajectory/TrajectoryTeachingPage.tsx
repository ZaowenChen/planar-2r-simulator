import { useEffect, useMemo } from 'react'
import { StatusBanner } from '../../components/StatusBanner'
import { geometricJacobian } from '../../robotics/jacobian'
import { forwardKinematics } from '../../robotics/kinematics'
import type { Vector3 } from '../../robotics/types'
import { RobotScene } from '../../scene/RobotScene'
import type { ScenePointModel } from '../../scene/sceneModel'
import { useLabStore } from '../../state/labStore'
import { PolynomialLesson } from './PolynomialLesson'
import { TeachPendantPanel } from './TeachPendantPanel'
import { TrajectoryCharts } from './TrajectoryCharts'
import { TrajectoryPlayback } from './TrajectoryPlayback'
import { TrapezoidalLesson } from './TrapezoidalLesson'
import {
  evaluatePtpInstruction,
  radiansToDegrees,
  type TimedTrajectorySample,
} from './trajectoryModel'
import { useTrajectoryStore } from './trajectoryStore'

const ZERO: Vector3 = [0, 0, 0]

function currentAngleText(q: Vector3): string {
  return q.map((value, index) => `J${index + 1} ${radiansToDegrees(value).toFixed(1)}°`).join(' · ')
}

export function TrajectoryTeachingPage() {
  const parameters = useLabStore((state) => state.parameters)
  const initialJointQ = useLabStore((state) => state.jointState.q)
  const initialized = useTrajectoryStore((state) => state.initialized)
  const initialize = useTrajectoryStore((state) => state.initialize)
  const reset = useTrajectoryStore((state) => state.reset)
  const currentQ = useTrajectoryStore((state) => state.currentQ)
  const points = useTrajectoryStore((state) => state.teachPoints)
  const preview = useTrajectoryStore((state) => state.preview)
  const previewActive = useTrajectoryStore((state) => state.previewActive)
  const time = useTrajectoryStore((state) => state.time)
  const sceneExpanded = useTrajectoryStore((state) => state.sceneExpanded)
  const setSceneExpanded = useTrajectoryStore((state) => state.setSceneExpanded)
  const setTime = useTrajectoryStore((state) => state.setTime)

  useEffect(() => {
    if (!initialized) initialize(initialJointQ)
  }, [initialJointQ, initialize, initialized])

  const previewSample = useMemo(
    () => preview === null ? null : evaluatePtpInstruction(preview.instruction, time),
    [preview, time],
  )
  const displaySample: TimedTrajectorySample = preview !== null && previewActive && previewSample !== null
    ? previewSample
    : { time: 0, q: currentQ, qd: ZERO, qdd: ZERO }
  const forward = useMemo(
    () => forwardKinematics(displaySample.q, parameters),
    [displaySample.q, parameters],
  )
  const calculation = useMemo(() => ({
    forward,
    jacobian: geometricJacobian(displaySample.q, parameters),
    jointState: { q: displaySample.q, qd: displaySample.qd, qdd: displaySample.qdd },
  }), [displaySample, forward, parameters])
  const trail = useMemo(() => preview?.samples.map((sample) => (
    forwardKinematics(sample.q, parameters).endEffectorPosition
  )) ?? [], [parameters, preview])
  const markers: readonly ScenePointModel[] = useMemo(() => points.map((point) => ({
    id: `teach-marker-${point.id}`,
    label: point.name,
    position: forwardKinematics(point.q, parameters).endEffectorPosition,
    color: point.id === preview?.instruction.startPointId
      ? '#58b8b1'
      : point.id === preview?.instruction.endPointId
        ? '#f5c86b'
        : '#dcecea',
  })), [parameters, points, preview?.instruction.endPointId, preview?.instruction.startPointId])

  return (
    <div className={`trajectory-workbench${sceneExpanded ? ' is-scene-expanded' : ''}`}>
      <section aria-label="轨迹三维示教视图" className="trajectory-workbench__scene">
        <div className="trajectory-scene-toolbar">
          <div>
            <p className="section-label">3D 主视图</p>
            <strong>{previewActive && preview !== null ? 'PTP 试运行姿态' : '手动 JOG 姿态'}</strong>
            <output>{currentAngleText(displaySample.q)}</output>
          </div>
          <div>
            <button onClick={() => setSceneExpanded(!sceneExpanded)} type="button">
              {sceneExpanded ? '退出放大' : '放大 3D'}
            </button>
            <button
              onClick={() => {
                if (window.confirm('确定清除全部示教点和轨迹预览吗？')) reset(initialJointQ)
              }}
              type="button"
            >重置示教</button>
          </div>
        </div>
        <RobotScene
          calculation={calculation}
          initialOverlays={{ coordinateFrames: true, grid: true, trail: true }}
          markers={markers}
          trail={trail}
          visibleOverlayControls={['coordinateFrames', 'trail', 'grid']}
        />
        <TrajectoryPlayback preview={preview} />
      </section>

      <section aria-label="虚拟示教器" className="trajectory-workbench__controls">
        <TeachPendantPanel jointLimits={parameters.jointLimits} />
      </section>

      <section aria-label="轨迹数学讲解" className="trajectory-workbench__analysis">
        {preview === null || previewSample === null ? (
          <div className="trajectory-lesson trajectory-lesson--empty">
            <p className="section-label">步骤 3 · 数学教学</p>
            <h3>等待 PTP 轨迹</h3>
            <StatusBanner tone="info" title="先完成示教">
              记录至少两个不同姿态，选择起终点并生成轨迹预览后，这里会展开归一化时间、插值公式和当前数值。
            </StatusBanner>
          </div>
        ) : preview.instruction.profile === 'quintic' ? (
          <PolynomialLesson current={previewSample} preview={preview} time={time} />
        ) : (
          <TrapezoidalLesson current={previewSample} preview={preview} time={time} />
        )}
      </section>

      <section aria-label="轨迹预览图表" className="trajectory-workbench__timeline">
        {preview === null ? (
          <div className="trajectory-chart-placeholder">
            <p className="section-label">步骤 4 · 同步预览</p>
            <h3>生成轨迹后显示多项式与关节曲线</h3>
            <p>曲线将与 3D 姿态、数学代入和时间游标共享同一个时刻。</p>
          </div>
        ) : (
          <TrajectoryCharts preview={preview} time={time} onTimeChange={setTime} />
        )}
      </section>
    </div>
  )
}
