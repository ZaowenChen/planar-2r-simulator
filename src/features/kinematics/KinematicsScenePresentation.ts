import { forwardKinematics } from '../../robotics/kinematics'
import type { ForwardKinematicsResult } from '../../robotics/kinematics'
import { decomposeDhTransform, multiply4 } from '../../robotics/transforms'
import type { Matrix4, RobotParameters, Vector3 } from '../../robotics/types'
import {
  buildRobotGeometry,
  coordinateFrameFromTransform,
} from '../../scene/sceneModel'
import type {
  CoordinateFrameModel,
  SceneArcModel,
  SceneCameraPresetModel,
  SceneDimensionModel,
  ScenePointModel,
  ScenePresentationModel,
  SceneTeachingVectorModel,
} from '../../scene/sceneModel'
import type { KinematicsDerivation, InverseSolutionDerivation } from './derivationModel'
import { configurationId, type KinematicsTeachingState } from './teachingState'

const IDENTITY_4: Matrix4 = [
  [1, 0, 0, 0],
  [0, 1, 0, 0],
  [0, 0, 1, 0],
  [0, 0, 0, 1],
]

const Z_AXIS: Vector3 = [0, 0, 1]

export interface BuildKinematicsScenePresentationInput {
  forward: ForwardKinematicsResult
  derivation: KinematicsDerivation
  parameters: RobotParameters
  target: Vector3
  teaching: KinematicsTeachingState
}

function add(left: Vector3, right: Vector3): Vector3 {
  return [left[0] + right[0], left[1] + right[1], left[2] + right[2]]
}

function scale(vector: Vector3, scalar: number): Vector3 {
  return [vector[0] * scalar, vector[1] * scalar, vector[2] * scalar]
}

function magnitude(vector: Vector3): number {
  return Math.hypot(...vector)
}

function normalize(vector: Vector3): Vector3 {
  const length = magnitude(vector)
  return length < 1e-12 ? [1, 0, 0] : scale(vector, 1 / length)
}

function midpoint(left: Vector3, right: Vector3): Vector3 {
  return scale(add(left, right), 0.5)
}

function transformOrigin(transform: Matrix4): Vector3 {
  return [transform[0][3], transform[1][3], transform[2][3]]
}

function transformAxis(transform: Matrix4, column: 0 | 1 | 2): Vector3 {
  return [transform[0][column], transform[1][column], transform[2][column]]
}

function matricesClose(left: Matrix4, right: Matrix4, tolerance = 1e-10): boolean {
  return left.every((row, rowIndex) => row.every((value, columnIndex) => (
    Math.abs(value - right[rowIndex][columnIndex]) <= tolerance
  )))
}

function millimetres(valueMetres: number, precision = 1): string {
  return `${(valueMetres * 1000).toFixed(precision)} mm`
}

function selectedDetail(
  derivation: KinematicsDerivation,
  teaching: KinematicsTeachingState,
): InverseSolutionDerivation | undefined {
  return derivation.inverse.candidateDetails.find((detail) => (
    configurationId(detail.solution) === teaching.activeConfigurationId
  )) ?? derivation.inverse.candidateDetails[0]
}

function frameTransforms(forward: ForwardKinematicsResult): readonly {
  id: string
  label: string
  originLabel: string
  transform: Matrix4
}[] {
  const entries = [
    { id: 'frame-w0', label: '{W}/{0}', originLabel: 'O₀', transform: IDENTITY_4 },
    { id: 'frame-1', label: '{1}', originLabel: 'O₁', transform: forward.transforms[0] },
    { id: 'frame-2', label: '{2}', originLabel: 'O₂', transform: forward.transforms[1] },
  ]
  if (matricesClose(forward.transforms[2], forward.transforms[3])) {
    return [
      ...entries,
      { id: 'frame-3e', label: '{3}/{e}', originLabel: 'O₃', transform: forward.transforms[2] },
    ]
  }
  return [
    ...entries,
    { id: 'frame-3', label: '{3}', originLabel: 'O₃', transform: forward.transforms[2] },
    { id: 'frame-e', label: '{e}', originLabel: 'Oₑ', transform: forward.transforms[3] },
  ]
}

function buildFrames(
  forward: ForwardKinematicsResult,
  parameters: RobotParameters,
  teaching: KinematicsTeachingState,
): CoordinateFrameModel[] {
  const frameEntries = frameTransforms(forward)
  const finalFrameId = frameEntries.at(-1)?.id ?? 'frame-3e'
  const dhSourceId = teaching.selectedDhRow === 0
    ? 'frame-w0'
    : `frame-${teaching.selectedDhRow}`
  const dhTargetId = teaching.selectedDhRow === 2
    ? frameEntries.find((entry) => entry.id === 'frame-3')?.id ?? finalFrameId
    : `frame-${teaching.selectedDhRow + 1}`
  const activeFrameId = teaching.stepIndex === 1 || teaching.stepIndex === 2
    ? dhTargetId
    : teaching.stepIndex === 5 || teaching.stepIndex === 15
      ? finalFrameId
      : null
  const relatedFrameId = teaching.stepIndex === 1 || teaching.stepIndex === 2
    ? dhSourceId
    : null
  const axisLength = Math.max(0.26, Math.min(0.62, (
    parameters.geometry.l2 + parameters.geometry.l3
  ) * 0.13))
  const manuallyExploring = teaching.frameMode === 'all' || teaching.frameMode === 'chain'

  return frameEntries.map((entry, index) => {
    const active = entry.id === activeFrameId
    const related = entry.id === relatedFrameId
    const overviewMarker = false
    const inDhChain = teaching.stepIndex === 1 || teaching.stepIndex === 2
      ? index <= teaching.selectedDhRow + 1
      : true
    const visible = teaching.frameMode !== 'hidden' && (
      teaching.frameMode === 'all'
      || (teaching.frameMode === 'chain' && inDhChain)
      || (teaching.frameMode === 'current' && (active || related || overviewMarker))
    )
    const activeAxes = teaching.stepIndex === 2
      ? teaching.dhOperation === 'rz' || teaching.dhOperation === 'tz' ? ['z'] as const : ['x'] as const
      : ['x', 'y', 'z'] as const
    return coordinateFrameFromTransform({
      ...entry,
      axisLength: active ? axisLength * 1.35 : axisLength * 0.85,
      visible,
      opacity: active ? 1 : manuallyExploring ? 0.38 : 0.4,
      detail: active ? 'axes' : 'name',
      lineWidth: active ? 4 : 1.5,
      showFrameLabel: !active,
      visibleAxes: overviewMarker || related
        ? []
        : active ? activeAxes : manuallyExploring ? ['x', 'y', 'z'] : [],
    })
  })
}

function buildCamera(
  id: KinematicsTeachingState['cameraPreset'],
  parameters: RobotParameters,
  target: Vector3,
  planeNormal: Vector3,
  endpoint: Vector3,
): SceneCameraPresetModel {
  const reach = Math.max(2.4, parameters.geometry.d1 + parameters.geometry.l2 + parameters.geometry.l3)
  const shoulder: Vector3 = [0, 0, parameters.geometry.d1]
  const planeCenter = midpoint(shoulder, target)
  if (id === 'top') {
    return {
      id,
      position: [planeCenter[0], planeCenter[1], parameters.geometry.d1 + reach * 2.4],
      target: planeCenter,
      up: [0, 1, 0],
    }
  }
  if (id === 'work-plane') {
    return {
      id,
      position: add(
        add(planeCenter, scale(planeNormal, -reach * 1.62)),
        scale(Z_AXIS, reach * 0.1),
      ),
      target: planeCenter,
      up: Z_AXIS,
    }
  }
  if (id === 'tool') {
    return {
      id,
      position: add(
        add(endpoint, scale(planeNormal, -reach * 0.88)),
        scale(Z_AXIS, reach * 0.48),
      ),
      target: endpoint,
      up: Z_AXIS,
    }
  }
  return {
    id: 'overview',
    position: [reach * 1.34, -reach * 1.55, parameters.geometry.d1 + reach * 1.02],
    target: midpoint([0, 0, 0], endpoint),
    up: Z_AXIS,
  }
}

function dimension(
  id: string,
  label: string,
  start: Vector3,
  end: Vector3,
  teaching: KinematicsTeachingState,
  color: string,
  style: SceneDimensionModel['style'] = 'solid',
  labelPosition?: Vector3,
): SceneDimensionModel {
  return {
    id,
    label,
    start,
    end,
    color,
    style,
    emphasized: teaching.symbolFocus === null || teaching.symbolFocus === id,
    labelPosition,
  }
}

function angleArc(
  id: string,
  label: string,
  center: Vector3,
  basisX: Vector3,
  basisY: Vector3,
  radius: number,
  startAngle: number,
  endAngle: number,
  teaching: KinematicsTeachingState,
  color: string,
  labelPosition?: Vector3,
): SceneArcModel {
  return {
    id,
    label,
    center,
    basisX,
    basisY,
    radius,
    startAngle,
    endAngle,
    color,
    emphasized: teaching.symbolFocus === null || teaching.symbolFocus === id,
    labelPosition,
  }
}

export function buildKinematicsScenePresentation({
  forward,
  derivation,
  parameters,
  target,
  teaching,
}: BuildKinematicsScenePresentationInput): ScenePresentationModel {
  const detail = selectedDetail(derivation, teaching)
  const selectedForward = detail === undefined
    ? forward
    : forwardKinematics(detail.solution.q, parameters)
  const useTeachingPose = detail !== undefined && teaching.stepIndex >= 7 && teaching.stepIndex <= 15
  const displayForward = useTeachingPose ? selectedForward : forward
  const shoulder: Vector3 = [0, 0, parameters.geometry.d1]
  const radial = Math.hypot(target[0], target[1])
  const radialDirection: Vector3 = radial < 1e-10
    ? [1, 0, 0]
    : [target[0] / radial, target[1] / radial, 0]
  const planeNormal: Vector3 = [-radialDirection[1], radialDirection[0], 0]
  const radialFoot = add(shoulder, scale(radialDirection, radial))
  const maxReach = parameters.geometry.l2 + parameters.geometry.l3
  const points: ScenePointModel[] = []
  const dimensions: SceneDimensionModel[] = []
  const arcs: SceneArcModel[] = []
  const vectors: SceneTeachingVectorModel[] = []
  const ghostRobots: ScenePresentationModel['ghostRobots'][number][] = []
  const step = teaching.stepIndex

  if (step === 0 || step === 1) {
    const index = teaching.selectedDhRow
    const rowNumber = index + 1
    const sourceOrigin = forward.origins[index]
    const targetOrigin = forward.origins[index + 1]
    const sourceAxis = forward.jointAxes[index]
    const targetAxis = index < 2
      ? forward.jointAxes[index + 1]
      : transformAxis(forward.transforms[2], 2)
    const axisLength = Math.max(0.48, maxReach * 0.18)
    const sourceAxisEnd = add(sourceOrigin, scale(normalize(sourceAxis), axisLength))
    const targetAxisEnd = add(targetOrigin, scale(normalize(targetAxis), axisLength * 0.86))
    points.push(
      { id: `dh-joint-${rowNumber}`, label: step === 0 ? `关节 ${rowNumber}` : undefined, position: sourceOrigin, color: '#f5c86b' },
      { id: `dh-origin-${rowNumber}`, label: undefined, position: targetOrigin, color: '#58d2c9', opacity: 0.72 },
    )
    dimensions.push(
      {
        ...dimension(
          `dh-axis-source-${rowNumber}`,
          `z${String.fromCharCode(0x2080 + index)} · 关节 ${rowNumber} 轴`,
          sourceOrigin,
          sourceAxisEnd,
          teaching,
          '#3b82f6',
        ),
        showLabel: step === 0,
      },
      {
        ...dimension(
          `dh-axis-target-${rowNumber}`,
          index < 2
            ? `z${String.fromCharCode(0x2081 + index)} · 下一关节轴`
            : 'z₃ · 末端约定轴',
          targetOrigin,
          targetAxisEnd,
          teaching,
          '#7aa5d8',
        ),
        showLabel: step === 0,
      },
    )
    if (step === 1) {
      const a = [0, parameters.geometry.l2, parameters.geometry.l3][index]
      const xAxis = transformAxis(forward.transforms[index], 0)
      const commonStart = a === 0 ? targetOrigin : sourceOrigin
      const commonEnd = a === 0
        ? add(commonStart, scale(normalize(xAxis), axisLength * 0.72))
        : targetOrigin
      dimensions.push({
        ...dimension(
          `dh-a-${rowNumber}`,
          `x${String.fromCharCode(0x2081 + index)} 公法线 · a${String.fromCharCode(0x2081 + index)} = ${millimetres(a)}${a === 0 ? '（方向示意）' : ''}`,
          commonStart,
          commonEnd,
          teaching,
          '#ef4444',
        ),
        showLabel: false,
      })
    }
  }

  if (step >= 6 && step <= 14) {
    points.push({
      id: 'target-position',
      label: step === 6 || step === 14 ? 'P_d 目标' : undefined,
      labelOffset: step === 6 || step === 14 ? [0, 0, 0.34] : undefined,
      position: target,
      color: '#f5c86b',
    })
  }
  if (step >= 6 && step <= 12) {
    points.push({
      id: 'shoulder-position',
      label: undefined,
      position: shoulder,
      color: '#e8f0ef',
      opacity: step <= 8 ? 1 : 0.55,
    })
  }

  if (step === 4) {
    const endpoint = forward.endEffectorPosition
    const xFoot: Vector3 = [endpoint[0], 0, 0]
    const xyFoot: Vector3 = [endpoint[0], endpoint[1], 0]
    dimensions.push(
      dimension('x', `x = ${millimetres(endpoint[0])}`, [0, 0, 0], xFoot, teaching, '#ef4444'),
      dimension('y', `y = ${millimetres(endpoint[1])}`, xFoot, xyFoot, teaching, '#22c55e'),
      dimension('z', `z = ${millimetres(endpoint[2])}`, xyFoot, endpoint, teaching, '#3b82f6'),
    )
  }

  if (step === 6 && teaching.symbolFocus === 'theta1') {
    arcs.push(angleArc(
      'theta1',
      `θ₁ = ${derivation.inverse.baseAngleDegrees.toFixed(1)}°`,
      [0, 0, 0],
      [1, 0, 0],
      [0, 1, 0],
      Math.max(0.42, maxReach * 0.14),
      0,
      derivation.inverse.baseAngleDegrees * Math.PI / 180,
      teaching,
      '#22b8b0',
    ))
  } else if (step === 6) {
    dimensions.push(
      dimension('r', `r = ${derivation.inverse.radialMm.toFixed(1)} mm`, shoulder, radialFoot, teaching, '#22b8b0'),
      dimension('h', `h = ${derivation.inverse.verticalMm.toFixed(1)} mm`, radialFoot, target, teaching, '#d8922d', 'dashed'),
    )
  }

  if (step === 7) {
    dimensions.push(
      dimension('r', `r = ${derivation.inverse.radialMm.toFixed(1)} mm`, shoulder, radialFoot, teaching, '#22b8b0'),
      dimension('h', `h = ${derivation.inverse.verticalMm.toFixed(1)} mm`, radialFoot, target, teaching, '#d8922d', 'dashed'),
      dimension('s', `s = ${derivation.inverse.shoulderToTargetMm.toFixed(1)} mm`, shoulder, target, teaching, '#e8f0ef', 'dashed'),
    )
  }

  if (detail !== undefined && step >= 8 && step <= 12) {
    const elbow = selectedForward.origins[2]
    if (step === 8 || step === 9 || step === 11) {
      dimensions.push(
        dimension('l2', `l₂ = ${millimetres(parameters.geometry.l2, 0)}`, shoulder, elbow, teaching, '#87d2cc'),
        dimension('l3', `l₃ = ${millimetres(parameters.geometry.l3, 0)}`, elbow, target, teaching, '#87d2cc'),
      )
    }
    if (step === 8) {
      dimensions.push(dimension(
        's',
        `s = ${derivation.inverse.shoulderToTargetMm.toFixed(1)} mm`,
        shoulder,
        target,
        teaching,
        '#e8f0ef',
        'dashed',
      ))
    }
    const theta2 = detail.solution.q[1]
    const theta3 = detail.solution.q[2]
    const gamma = detail.targetDirectionDegrees * Math.PI / 180
    const radius = Math.max(0.28, maxReach * 0.105)
    if (step === 9 || step === 12) {
      arcs.push(angleArc('theta3', `θ₃ = ${detail.qDegrees[2].toFixed(1)}°`, elbow, radialDirection, Z_AXIS, radius * 0.82, theta2, theta2 + theta3, teaching, '#ef6a54'))
    }
    if (step === 10) {
      arcs.push(angleArc(
        'gamma',
        `γ = ${detail.targetDirectionDegrees.toFixed(1)}°`,
        shoulder,
        radialDirection,
        Z_AXIS,
        radius * 1.45,
        0,
        gamma,
        teaching,
        '#22b8b0',
        undefined,
      ))
    }
    if (step === 11) {
      arcs.push(angleArc(
        'delta',
        `δ = ${detail.triangleCorrectionDegrees.toFixed(1)}°`,
        shoulder,
        radialDirection,
        Z_AXIS,
        radius * 1.92,
        theta2,
        gamma,
        teaching,
        '#d8922d',
        undefined,
      ))
    }
    if (step === 12) {
      arcs.push(angleArc(
        'theta2',
        `θ₂ = ${detail.qDegrees[1].toFixed(1)}°`,
        shoulder,
        radialDirection,
        Z_AXIS,
        radius,
        0,
        theta2,
        teaching,
        '#f4f7f6',
        add(add(shoulder, scale(radialDirection, maxReach * 0.15)), scale(Z_AXIS, -maxReach * 0.09)),
      ))
    }
  }

  if (detail !== undefined && step === 13) {
    const signedRadialEnd = add(
      shoulder,
      scale(radialDirection, detail.solution.radialFamily === 'folded' ? -radial : radial),
    )
    dimensions.push(dimension(
      'r',
      `rₛ = ${detail.solution.radialFamily === 'folded' ? '−' : ''}${derivation.inverse.radialMm.toFixed(1)} mm`,
      shoulder,
      signedRadialEnd,
      teaching,
      '#22b8b0',
    ))
    const alternatives = derivation.inverse.candidateDetails.filter((candidate) => {
      return candidate.solution.branch !== detail.solution.branch
        && candidate.solution.radialFamily === detail.solution.radialFamily
    })
    alternatives.forEach((candidate) => ghostRobots.push(buildRobotGeometry(
      forwardKinematics(candidate.solution.q, parameters),
      {
        id: `ghost-${configurationId(candidate.solution)}`,
        opacity: 0.28,
        showLabel: false,
        style: 'ghost',
      },
    )))
  }

  if (step === 14 && detail !== undefined) {
    const fkPosition = selectedForward.endEffectorPosition
    const residual: Vector3 = [
      fkPosition[0] - target[0],
      fkPosition[1] - target[1],
      fkPosition[2] - target[2],
    ]
    const residualMagnitude = magnitude(residual)
    const minimumVisibleLength = maxReach * 0.1
    const errorVectorWasScaled = residualMagnitude < minimumVisibleLength
    const displayEnd = errorVectorWasScaled
      ? add(target, scale(
          residualMagnitude > 0 ? scale(residual, 1 / residualMagnitude) : planeNormal,
          minimumVisibleLength,
        ))
      : fkPosition
    points.push({
      id: 'fk-position',
      label: 'p_FK 回代',
      labelOffset: [0, 0, -0.42],
      position: fkPosition,
      color: '#58d2c9',
    })
    dimensions.push(dimension(
      'position-error',
      `Δp = ${detail.positionErrorMm.toExponential(2)} mm${errorVectorWasScaled ? '（已放大，仅用于观察）' : ''}`,
      target,
      displayEnd,
      teaching,
      '#ef6a54',
      'solid',
      add(add(target, scale(planeNormal, minimumVisibleLength * 0.85)), scale(Z_AXIS, -0.32)),
    ))
  }

  if (step === 17) {
    const vectorLength = Math.max(0.46, maxReach * 0.16)
    const index = teaching.selectedJacobianColumn
    const column = derivation.jacobianColumns[index]
    const origin = column.origin
    const axisEnd = add(origin, scale(normalize(column.axis), vectorLength))
    const tangentEnd = add(
      forward.endEffectorPosition,
      scale(normalize(column.linear), vectorLength),
    )
    points.push({ id: `jacobian-origin-${index}`, label: `o${index}`, position: origin, color: '#f5c86b' })
    dimensions.push(
      dimension(`jacobian-axis-${index}`, `z${index}`, origin, axisEnd, teaching, '#3b82f6'),
      dimension(`jacobian-offset-${index}`, `pₑ−o${index}`, origin, forward.endEffectorPosition, teaching, '#d8922d', 'dashed'),
      dimension(`jacobian-tangent-${index}`, `J${index + 1} 切向`, forward.endEffectorPosition, tangentEnd, teaching, '#58d2c9'),
    )
  }

  if (step === 16 || step === 18) {
    const endpoint = forward.endEffectorPosition
    vectors.push(
      {
        id: 'end-linear-velocity',
        label: 'vₑ',
        labelOffset: [0, 0, 0.14],
        origin: endpoint,
        vector: derivation.linearVelocityMillimetresPerSecond,
        unit: 'mm/s',
        color: '#38bdf8',
      },
      {
        id: 'end-angular-velocity',
        label: 'ωₑ 轴向',
        labelPositionFactor: 0.5,
        labelOffset: [0, 0, 0.08],
        origin: endpoint,
        vector: derivation.angularVelocityDegreesPerSecond,
        unit: '°/s',
        color: '#f5c86b',
      },
    )
    if (step === 18) {
      const contribution = derivation.velocityContributions[teaching.selectedJacobianColumn]
      vectors.push({
        id: `joint-${teaching.selectedJacobianColumn + 1}-linear-contribution`,
        label: `vₑ(${teaching.selectedJacobianColumn + 1})`,
        labelOffset: [0, 0, -0.18],
        origin: endpoint,
        vector: contribution.linearMillimetresPerSecond,
        unit: 'mm/s',
        color: '#58d2c9',
        opacity: 0.45,
      })
    }
  }

  if (step === 5) {
    arcs.push(angleArc(
      'beta',
      `β = ${derivation.toolElevationDegrees.toFixed(1)}°`,
      forward.endEffectorPosition,
      radialDirection,
      Z_AXIS,
      Math.max(0.3, maxReach * 0.1),
      0,
      derivation.toolElevationDegrees * Math.PI / 180,
      teaching,
      '#d8922d',
    ))
  }

  const workPlane = step >= 6 && step <= 13
    ? {
        origin: midpoint(shoulder, target),
        normal: planeNormal,
        width: maxReach * 1.35,
        height: maxReach * 1.2,
        color: '#58b8b1',
        opacity: 0.11,
      }
    : null

  const note = detail !== undefined && step === 13
    ? `当前为${detail.solution.branch === 'elbow-down' ? '肘下' : '肘上'}构型；虚影用于比较同一径向族的另一肘部姿态。`
    : step === 2
      ? 'D–H 四段操作用于构造相邻坐标系，不代表机器人实际沿四段轨迹运动。'
      : step === 16 || step === 18
        ? '速度箭头长度已归一化，只表示方向；真实量值请查看右侧公式。'
      : '当前世界坐标系 {W} 与基座坐标系 {0} 重合。'

  const primaryRobot = step === 0
    ? buildRobotGeometry(forward, {
        id: 'joint-axis-overview',
        opacity: 0.36,
        showLabel: false,
        style: 'ghost',
      })
    : useTeachingPose
    ? buildRobotGeometry(selectedForward, {
        id: `teaching-${teaching.activeConfigurationId}`,
        label: detail === undefined
          ? '当前构型'
          : `${detail.solution.branch === 'elbow-down' ? '肘下' : '肘上'} · ${detail.solution.radialFamily === 'folded' ? '折叠径向' : '常规径向'}`,
        showLabel: step === 13,
      })
    : null

  let frames = buildFrames(displayForward, parameters, teaching)
  if (step === 2) {
    const rows = [
      { theta: derivation.radians[0], a: 0, alpha: Math.PI / 2, d: parameters.geometry.d1 },
      { theta: derivation.radians[1], a: parameters.geometry.l2, alpha: 0, d: 0 },
      { theta: derivation.radians[2], a: parameters.geometry.l3, alpha: 0, d: 0 },
    ] as const
    const row = rows[teaching.selectedDhRow]
    const operationIndex = ({ rz: 0, tz: 1, tx: 2, rx: 3 } as const)[teaching.dhOperation]
    const previousTransform = teaching.selectedDhRow === 0
      ? IDENTITY_4
      : teaching.selectedDhRow === 1 ? derivation.t01 : derivation.t02
    const decomposition = decomposeDhTransform(row.theta, row.a, row.alpha, row.d)
    const relative = decomposition.operations[operationIndex].cumulative
    const activeTransform = multiply4(previousTransform, relative)
    const activeId = teaching.selectedDhRow === 2
      ? frames.find((frame) => frame.id === 'frame-3')?.id ?? frames.at(-1)?.id ?? 'frame-3e'
      : `frame-${teaching.selectedDhRow + 1}`
    frames = frames.map((frame) => frame.id === activeId
      ? coordinateFrameFromTransform({
          axisLength: frame.axisLength,
          detail: 'axes',
          id: frame.id,
          label: `${frame.label} 构造中`,
          lineWidth: frame.lineWidth,
          opacity: 1,
          originLabel: frame.originLabel,
          showFrameLabel: false,
          transform: activeTransform,
          visible: true,
          visibleAxes: frame.visibleAxes,
        })
      : frame)

    const stageTransforms = decomposition.operations.map((operation) => (
      multiply4(previousTransform, operation.cumulative)
    ))
    const rowNumber = teaching.selectedDhRow + 1
    const operationRadius = Math.max(0.24, maxReach * 0.085)
    if (operationIndex > 0) {
      const sourceFrameId = teaching.selectedDhRow === 0
        ? 'frame-w0'
        : `frame-${teaching.selectedDhRow}`
      frames = frames.map((frame) => frame.id === sourceFrameId
        ? { ...frame, visible: false }
        : frame)
      const previousStageTransform = multiply4(
        previousTransform,
        decomposition.operations[operationIndex - 1].cumulative,
      )
      frames = [
        ...frames,
        coordinateFrameFromTransform({
          axisLength: operationRadius * 1.65,
          detail: 'name',
          id: `dh-stage-previous-${rowNumber}`,
          label: `{${rowNumber}} 上一步`,
          lineWidth: 1.4,
          opacity: 0.24,
          showFrameLabel: true,
          transform: previousStageTransform,
          visible: true,
          visibleAxes: [],
        }),
      ]
    }
    if (teaching.dhOperation === 'rz') {
      if (Math.abs(row.theta) > 1e-10) {
        arcs.push(angleArc(
          `theta${rowNumber}`,
          `θ${String.fromCharCode(0x2080 + rowNumber)} = ${(row.theta * 180 / Math.PI).toFixed(1)}°`,
          transformOrigin(previousTransform),
          transformAxis(previousTransform, 0),
          transformAxis(previousTransform, 1),
          operationRadius,
          0,
          row.theta,
          teaching,
          '#f5c86b',
        ))
      } else {
        const start = transformOrigin(previousTransform)
        dimensions.push(dimension(
          `theta${rowNumber}`,
          `θ${String.fromCharCode(0x2080 + rowNumber)} = 0.0°（绕 z 轴，无转动）`,
          start,
          add(start, scale(transformAxis(previousTransform, 2), operationRadius)),
          teaching,
          '#f5c86b',
        ))
      }
    } else if (teaching.dhOperation === 'tz') {
      const start = transformOrigin(stageTransforms[0])
      const actualEnd = transformOrigin(stageTransforms[1])
      const isZero = magnitude([
        actualEnd[0] - start[0],
        actualEnd[1] - start[1],
        actualEnd[2] - start[2],
      ]) < 1e-10
      dimensions.push(dimension(
        `dh-d-${rowNumber}`,
        `d${String.fromCharCode(0x2080 + rowNumber)} = ${millimetres(row.d)}${isZero ? '（方向示意）' : ''}`,
        start,
        isZero ? add(start, scale(transformAxis(stageTransforms[0], 2), operationRadius)) : actualEnd,
        teaching,
        '#3b82f6',
      ))
    } else if (teaching.dhOperation === 'tx') {
      const start = transformOrigin(stageTransforms[1])
      const actualEnd = transformOrigin(stageTransforms[2])
      const isZero = magnitude([
        actualEnd[0] - start[0],
        actualEnd[1] - start[1],
        actualEnd[2] - start[2],
      ]) < 1e-10
      dimensions.push(dimension(
        `dh-a-${rowNumber}`,
        `a${String.fromCharCode(0x2080 + rowNumber)} = ${millimetres(row.a)}${isZero ? '（方向示意）' : ''}`,
        start,
        isZero ? add(start, scale(transformAxis(stageTransforms[2], 0), operationRadius)) : actualEnd,
        teaching,
        '#ef4444',
      ))
    } else if (Math.abs(row.alpha) > 1e-10) {
      arcs.push(angleArc(
        `dh-alpha-${rowNumber}`,
        `α${String.fromCharCode(0x2080 + rowNumber)} = ${(row.alpha * 180 / Math.PI).toFixed(1)}°`,
        transformOrigin(stageTransforms[2]),
        transformAxis(stageTransforms[2], 1),
        transformAxis(stageTransforms[2], 2),
        operationRadius,
        0,
        row.alpha,
        teaching,
        '#ef4444',
      ))
    } else {
      const start = transformOrigin(stageTransforms[2])
      dimensions.push(dimension(
        `dh-alpha-${rowNumber}`,
        `α${String.fromCharCode(0x2080 + rowNumber)} = 0.0°（绕 x 轴，无转动）`,
        start,
        add(start, scale(transformAxis(stageTransforms[2], 0), operationRadius)),
        teaching,
        '#ef4444',
      ))
    }
  }

  return {
    camera: buildCamera(
      teaching.cameraPreset,
      parameters,
      target,
      planeNormal,
      displayForward.endEffectorPosition,
    ),
    frames,
    points,
    dimensions,
    arcs,
    vectors,
    workPlane,
    primaryRobot,
    ghostRobots,
    hideBaseRobot: primaryRobot !== null,
    note,
  }
}
