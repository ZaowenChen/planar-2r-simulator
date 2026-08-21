import { useMemo, useReducer, useState } from 'react'
import { RobotScene } from '../../scene/RobotScene'
import { useLabStore } from '../../state/labStore'
import { InverseKinematicsPanel } from './InverseKinematicsPanel'
import { JointControls } from './JointControls'
import { JointMotionControls } from './JointMotionControls'
import { KinematicsWalkthrough } from './KinematicsWalkthrough'
import { buildKinematicsDerivation } from './derivationModel'
import { buildKinematicsScenePresentation } from './KinematicsScenePresentation'
import { KinematicsSceneToolbar } from './KinematicsSceneToolbar'
import { metresToMillimetres, radiansToDegrees } from './presentation'
import {
  INITIAL_KINEMATICS_TEACHING_STATE,
  configurationId,
  kinematicsTeachingReducer,
  type KinematicsMode,
  type KinematicsMobilePane,
} from './teachingState'

const KINEMATICS_MODES: readonly {
  id: KinematicsMode
  label: string
  direction: string
}[] = [
  { id: 'forward', label: '正运动学', direction: 'q → 位置与姿态' },
  { id: 'inverse', label: '位置逆运动学', direction: 'p_d → 关节角' },
  { id: 'jacobian', label: '微分运动学', direction: 'q, q̇ → 末端速度' },
]

export function KinematicsPage() {
  const [parametersExpanded, setParametersExpanded] = useState(false)
  const [teaching, dispatchTeaching] = useReducer(
    kinematicsTeachingReducer,
    INITIAL_KINEMATICS_TEACHING_STATE,
  )
  const calculation = useLabStore((state) => state.calculation)
  const jointState = useLabStore((state) => state.jointState)
  const parameters = useLabStore((state) => state.parameters)
  const desiredPosition = useLabStore((state) => state.desiredPosition)
  const setDesiredPositionVector = useLabStore((state) => state.setDesiredPositionVector)
  const setJointVector = useLabStore((state) => state.setJointVector)
  const revision = String(calculation.revision)
  const sceneCalculation = {
    forward: calculation.forward,
    jacobian: calculation.jacobian,
    jointState,
    torque: calculation.dynamics.tau,
    gravity: parameters.gravity,
  }
  const derivation = useMemo(() => buildKinematicsDerivation(
    jointState.q,
    parameters,
    desiredPosition,
    jointState.qd,
  ), [desiredPosition, jointState.q, jointState.qd, parameters])
  const scenePresentation = useMemo(() => buildKinematicsScenePresentation({
    derivation,
    forward: calculation.forward,
    parameters,
    target: desiredPosition,
    teaching,
  }), [calculation.forward, derivation, desiredPosition, parameters, teaching])
  const paneLabels: readonly [KinematicsMobilePane, string][] = [
    ['analysis', '推导'],
    ['scene', '3D'],
    ['controls', '参数'],
  ]
  const jointSummary = jointState.q
    .map((value, index) => `θ${['₁', '₂', '₃'][index]} ${radiansToDegrees(value).toFixed(1)}°`)
    .join('，')
  const targetSummary = desiredPosition
    .map((value, index) => `${['x', 'y', 'z'][index]} ${metresToMillimetres(value).toFixed(1)}`)
    .join('，')
  const configurationSummary = `${teaching.activeConfigurationId.startsWith('conventional') ? '常规径向' : '折叠径向'} · ${teaching.activeConfigurationId.endsWith('elbow-down') ? '肘下构型' : '肘上构型'}`
  const endpointSummary = calculation.forward.endEffectorPosition
    .map((value, index) => `${['x', 'y', 'z'][index]} ${metresToMillimetres(value).toFixed(1)}`)
    .join('，')
  const inverseCandidateSummary = `${derivation.inverse.candidateDetails.length} 组几何候选，${derivation.inverse.solutionDetails.length} 组当前可应用`
  const jointVelocitySummary = derivation.qdDegreesPerSecond
    .map((value, index) => `θ̇${['₁', '₂', '₃'][index]} ${value.toFixed(1)}°/s`)
    .join('，')
  const linearVelocitySummary = derivation.linearVelocityMillimetresPerSecond
    .map((value, index) => `${['vₓ', 'vᵧ', 'v_z'][index]} ${value.toFixed(1)}`)
    .join('，')
  const angularVelocitySummary = derivation.angularVelocityDegreesPerSecond
    .map((value, index) => `${['ωₓ', 'ωᵧ', 'ω_z'][index]} ${value.toFixed(1)}`)
    .join('，')
  const activeInverseSolution = derivation.inverse.solutionDetails.find((detail) => (
    configurationId(detail.solution) === teaching.activeConfigurationId
  ))

  const setMobilePane = (pane: KinematicsMobilePane) => {
    dispatchTeaching({ type: 'mobile-pane', value: pane })
    setParametersExpanded(pane === 'controls')
  }

  const toggleParameters = () => {
    const nextExpanded = !parametersExpanded
    setParametersExpanded(nextExpanded)
    if (nextExpanded) {
      dispatchTeaching({ type: 'mobile-pane', value: 'controls' })
    } else if (teaching.mobilePane === 'controls') {
      dispatchTeaching({ type: 'mobile-pane', value: 'analysis' })
    }
  }

  const setMode = (mode: KinematicsMode) => {
    dispatchTeaching({ type: 'mode', value: mode })
    dispatchTeaching({ type: 'mobile-pane', value: 'analysis' })
    setParametersExpanded(false)
  }

  const useCurrentPositionAsInverseTarget = () => {
    setDesiredPositionVector(calculation.forward.endEffectorPosition)
    dispatchTeaching({ type: 'step', value: 6 })
    dispatchTeaching({ type: 'mobile-pane', value: 'analysis' })
    setParametersExpanded(false)
  }

  const applyInverseAndOpenForward = () => {
    if (activeInverseSolution === undefined) return
    setJointVector(activeInverseSolution.solution.q)
    dispatchTeaching({ type: 'step', value: 4 })
    dispatchTeaching({ type: 'mobile-pane', value: 'analysis' })
    setParametersExpanded(false)
  }

  return (
    <div
      className="kinematics-workspace"
      data-configuration={teaching.activeConfigurationId}
      data-dh-operation={teaching.dhOperation}
      data-dh-row={teaching.selectedDhRow + 1}
      data-mobile-pane={teaching.mobilePane}
      data-mode={teaching.mode}
      data-symbol-focus={teaching.symbolFocus ?? undefined}
      data-testid="kinematics-workspace"
    >
      <div aria-label="运动学计算模式" className="kinematics-mode-switcher" role="tablist">
        {KINEMATICS_MODES.map((mode) => (
          <button
            aria-selected={teaching.mode === mode.id}
            key={mode.id}
            onClick={() => setMode(mode.id)}
            role="tab"
            type="button"
          >
            <strong>{mode.label}</strong>
            <span>{mode.direction}</span>
          </button>
        ))}
      </div>

      <header className="kinematics-parameter-summary">
        <dl>
          {teaching.mode === 'forward' && (
            <>
              <div><dt>输入关节角 q</dt><dd>{jointSummary}</dd></div>
              <div><dt>输出末端位置 p<sub>e</sub> / mm</dt><dd>{endpointSummary}</dd></div>
              <div><dt>输出末端姿态</dt><dd>β {derivation.toolElevationDegrees.toFixed(1)}° · R<sub>e</sub></dd></div>
            </>
          )}
          {teaching.mode === 'inverse' && (
            <>
              <div><dt>输入目标位置 p<sub>d</sub> / mm</dt><dd>{targetSummary}</dd></div>
              <div><dt>当前预览构型</dt><dd>{configurationSummary}</dd></div>
              <div><dt>位置逆解结果</dt><dd>{inverseCandidateSummary}</dd></div>
            </>
          )}
          {teaching.mode === 'jacobian' && (
            <>
              <div><dt>当前关节角 q</dt><dd>{jointSummary}</dd></div>
              <div><dt>当前关节速度 q̇</dt><dd>{jointVelocitySummary}</dd></div>
              <div><dt>末端线速度 v<sub>e</sub> / mm/s</dt><dd>{linearVelocitySummary}</dd></div>
              <div><dt>末端角速度 ω<sub>e</sub> / °/s</dt><dd>{angularVelocitySummary}</dd></div>
            </>
          )}
        </dl>
        <div className="kinematics-parameter-summary__actions">
          {teaching.mode === 'forward' && (
            <button onClick={useCurrentPositionAsInverseTarget} type="button">
              使用当前 FK 位置作为 IK 目标
            </button>
          )}
          {teaching.mode === 'inverse' && (
            <button
              disabled={activeInverseSolution === undefined}
              onClick={applyInverseAndOpenForward}
              type="button"
            >
              应用当前逆解并进入 FK 验证
            </button>
          )}
          <button
            aria-expanded={parametersExpanded}
            aria-controls="kinematics-parameter-editor"
            onClick={toggleParameters}
            type="button"
          >
            {parametersExpanded
              ? '收起参数'
              : teaching.mode === 'inverse'
                ? '修改目标 / 选择逆解'
                : teaching.mode === 'jacobian' ? '修改运动状态' : '修改关节角'}
          </button>
        </div>
      </header>

      {parametersExpanded && (
        <section
          aria-label="运动学参数编辑"
          className="kinematics-parameter-editor"
          id="kinematics-parameter-editor"
        >
          {teaching.mode === 'inverse' ? (
            <div>
              <p className="section-label">目标与逆解构型</p>
              <InverseKinematicsPanel
                activeConfigurationId={teaching.activeConfigurationId}
                onConfigurationChange={(value) => dispatchTeaching({ type: 'configuration', value })}
              />
            </div>
          ) : teaching.mode === 'jacobian' ? (
            <div>
              <p className="section-label">关节运动状态</p>
              <JointMotionControls />
            </div>
          ) : (
            <div>
              <p className="section-label">关节角</p>
              <JointControls />
            </div>
          )}
        </section>
      )}

      <div aria-label="移动端运动学面板" className="kinematics-workspace__tabs" role="tablist">
        {paneLabels.map(([pane, label]) => (
          <button
            aria-selected={teaching.mobilePane === pane}
            key={pane}
            onClick={() => setMobilePane(pane)}
            role="tab"
            type="button"
          >
            {label}
          </button>
        ))}
      </div>

      <div className="kinematics-stage">
        <section aria-label="机器人三维教学看板" className="kinematics-stage__scene">
          <div data-revision={revision} data-testid="scene-result">
            <KinematicsSceneToolbar
              cameraPreset={teaching.cameraPreset}
              followStepCamera={teaching.followStepCamera}
              frameMode={teaching.frameMode}
              note={scenePresentation.note}
              onCameraPresetChange={(value) => dispatchTeaching({ type: 'camera-preset', value })}
              onFollowStepCameraChange={(value) => dispatchTeaching({ type: 'follow-camera', value })}
              onFrameModeChange={(value) => dispatchTeaching({ type: 'frame-mode', value })}
              onResetCamera={() => dispatchTeaching({ type: 'reset-camera' })}
            />
            <RobotScene
              calculation={sceneCalculation}
              cameraResetRevision={teaching.cameraResetRevision}
              initialOverlays={{
                acceleration: false,
                centerOfMass: false,
                coordinateFrames: false,
                gravity: false,
                grid: true,
                linearVelocity: false,
                torque: false,
                trail: false,
                workspace: false,
              }}
              followPresentationCamera={teaching.followStepCamera}
              onSceneObjectSelect={(id) => {
                const operationMatch = id.match(/^(?:theta|dh-d-|dh-a-|dh-alpha-)([123])$/)
                if (operationMatch !== null) {
                  const row = (Number(operationMatch[1]) - 1) as 0 | 1 | 2
                  const operation = id.startsWith('theta')
                    ? 'rz'
                    : id.startsWith('dh-d-')
                      ? 'tz'
                      : id.startsWith('dh-a-') ? 'tx' : 'rx'
                  dispatchTeaching({ type: 'dh-row', value: row })
                  dispatchTeaching({ type: 'dh-operation', value: operation })
                  return
                }
                const match = id.match(/(?:frame-|link-|dh-(?:axis-(?:source|target)-|joint-|origin-))([123])(?:$|\D)/)
                if (match === null) return
                const number = Number(match[1])
                const row = Math.max(0, Math.min(2, number - 1)) as 0 | 1 | 2
                dispatchTeaching({ type: 'dh-row', value: row })
              }}
              onCameraInteraction={() => dispatchTeaching({ type: 'follow-camera', value: false })}
              presentation={scenePresentation}
              visibleOverlayControls={[]}
            />
          </div>
        </section>
        <section aria-label="运动学公式推导" className="kinematics-stage__analysis">
          <div className="analysis-stack">
            <KinematicsWalkthrough
              activeConfigurationId={teaching.activeConfigurationId}
              dhOperation={teaching.dhOperation}
              onCameraPresetChange={(value) => dispatchTeaching({ type: 'camera-preset', value })}
              onConfigurationChange={(value) => dispatchTeaching({ type: 'configuration', value })}
              onDhOperationChange={(value) => dispatchTeaching({ type: 'dh-operation', value })}
              onDhRowChange={(value) => dispatchTeaching({ type: 'dh-row', value })}
              onStepChange={(value) => dispatchTeaching({ type: 'step', value })}
              onSymbolFocus={(value) => dispatchTeaching({ type: 'symbol-focus', value })}
              onJacobianColumnChange={(value) => dispatchTeaching({ type: 'jacobian-column', value })}
              mode={teaching.mode}
              parameters={parameters}
              q={jointState.q}
              qd={jointState.qd}
              revision={calculation.revision}
              singularity={calculation.singularity}
              selectedDhRow={teaching.selectedDhRow}
              selectedJacobianColumn={teaching.selectedJacobianColumn}
              stepIndex={teaching.stepIndex}
              symbolFocus={teaching.symbolFocus}
              target={desiredPosition}
            />
          </div>
        </section>
      </div>
    </div>
  )
}
