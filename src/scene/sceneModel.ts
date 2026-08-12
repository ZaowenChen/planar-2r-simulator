import type { ForwardKinematicsResult } from '../robotics/kinematics'
import type { JointState, Matrix4, Matrix6x3, Vector3 } from '../robotics/types'

export type Quaternion = readonly [number, number, number, number]

export interface SceneOverlayFlags {
  coordinateFrames: boolean
  centerOfMass: boolean
  workspace: boolean
  trail: boolean
  linearVelocity: boolean
  acceleration: boolean
  gravity: boolean
  torque: boolean
  grid: boolean
}

export const DEFAULT_SCENE_OVERLAYS: SceneOverlayFlags = {
  coordinateFrames: true,
  centerOfMass: true,
  workspace: false,
  trail: true,
  linearVelocity: false,
  acceleration: false,
  gravity: false,
  torque: false,
  grid: true,
}

export interface SceneCalculationInput {
  forward: ForwardKinematicsResult
  jointState: JointState
  jacobian: Matrix6x3
  torque: Vector3
  gravity: Vector3
  workspaceSamples?: readonly Vector3[]
  trail?: readonly Vector3[]
  overlays?: Partial<SceneOverlayFlags>
}

export interface JointMarkerModel {
  id: string
  label: string
  position: Vector3
  radius: number
  color: string
}

export interface LinkModel {
  id: string
  start: Vector3
  end: Vector3
  midpoint: Vector3
  quaternion: Quaternion
  length: number
  radius: number
  color: string
}

export interface CoordinateFrameModel {
  id: string
  label: string
  position: Vector3
  quaternion: Quaternion
  axes: { x: Vector3; y: Vector3; z: Vector3 }
  axisLength: number
  visible: boolean
}

export interface CenterOfMassMarkerModel {
  id: string
  label: string
  position: Vector3
  color: string
  visible: boolean
}

export interface VectorOverlayModel {
  id: string
  label: string
  origin: Vector3
  vector: Vector3
  direction: Vector3
  end: Vector3
  shaftMidpoint: Vector3
  quaternion: Quaternion
  displayLength: number
  magnitude: number
  signedMagnitude?: number
  unit: string
  color: string
  normalized: true
  visible: boolean
}

export interface PointCollectionModel {
  points: readonly Vector3[]
  color: string
  opacity: number
  visible: boolean
}

export interface TrailModel {
  points: readonly Vector3[]
  color: string
  visible: boolean
}

export interface SceneModel {
  joints: readonly JointMarkerModel[]
  links: readonly LinkModel[]
  coordinateFrames: readonly CoordinateFrameModel[]
  centerOfMassMarkers: readonly CenterOfMassMarkerModel[]
  workspace: PointCollectionModel
  trail: TrailModel
  vectors: readonly VectorOverlayModel[]
  gridVisible: boolean
}

const ZERO_VECTOR: Vector3 = [0, 0, 0]
const IDENTITY_QUATERNION: Quaternion = [0, 0, 0, 1]
const MAXIMUM_VECTOR_LENGTH = 1.35

function serializableQuaternion(quaternion: Quaternion): Quaternion {
  return quaternion.map((value) => Object.is(value, -0) ? 0 : value) as unknown as Quaternion
}

function vectorBetween(start: Vector3, end: Vector3): Vector3 {
  return [end[0] - start[0], end[1] - start[1], end[2] - start[2]]
}

function midpoint(start: Vector3, end: Vector3): Vector3 {
  return [
    (start[0] + end[0]) / 2,
    (start[1] + end[1]) / 2,
    (start[2] + end[2]) / 2,
  ]
}

function magnitude(vector: Vector3): number {
  return Math.hypot(vector[0], vector[1], vector[2])
}

function scale(vector: Vector3, scalar: number): Vector3 {
  return vector.map((component) => component === 0 ? 0 : component * scalar) as unknown as Vector3
}

function normalized(vector: Vector3): Vector3 {
  const length = magnitude(vector)
  return length === 0 ? ZERO_VECTOR : scale(vector, 1 / length)
}

/** Quaternion rotating Three.js cylinder's local +Y axis onto a segment. */
function quaternionFromYAxis(direction: Vector3): Quaternion {
  const unit = normalized(direction)
  if (unit === ZERO_VECTOR) return IDENTITY_QUATERNION
  if (unit[1] < -1 + 1e-12) return [1, 0, 0, 0]

  const unscaled: Quaternion = [unit[2], 0, -unit[0], 1 + unit[1]]
  const length = Math.hypot(...unscaled)
  return [
    unscaled[0] / length,
    unscaled[1] / length,
    unscaled[2] / length,
    unscaled[3] / length,
  ]
}

function quaternionFromTransform(transform: Matrix4): Quaternion {
  const m00 = transform[0][0]
  const m11 = transform[1][1]
  const m22 = transform[2][2]
  const trace = m00 + m11 + m22

  if (trace > 0) {
    const s = Math.sqrt(trace + 1) * 2
    return [
      (transform[2][1] - transform[1][2]) / s,
      (transform[0][2] - transform[2][0]) / s,
      (transform[1][0] - transform[0][1]) / s,
      s / 4,
    ]
  }
  if (m00 > m11 && m00 > m22) {
    const s = Math.sqrt(1 + m00 - m11 - m22) * 2
    return [
      s / 4,
      (transform[0][1] + transform[1][0]) / s,
      (transform[0][2] + transform[2][0]) / s,
      (transform[2][1] - transform[1][2]) / s,
    ]
  }
  if (m11 > m22) {
    const s = Math.sqrt(1 + m11 - m00 - m22) * 2
    return [
      (transform[0][1] + transform[1][0]) / s,
      s / 4,
      (transform[1][2] + transform[2][1]) / s,
      (transform[0][2] - transform[2][0]) / s,
    ]
  }
  const s = Math.sqrt(1 + m22 - m00 - m11) * 2
  return [
    (transform[0][2] + transform[2][0]) / s,
    (transform[1][2] + transform[2][1]) / s,
    s / 4,
    (transform[1][0] - transform[0][1]) / s,
  ]
}

function axesFromTransform(transform: Matrix4) {
  return {
    x: [transform[0][0], transform[1][0], transform[2][0]] as Vector3,
    y: [transform[0][1], transform[1][1], transform[2][1]] as Vector3,
    z: [transform[0][2], transform[1][2], transform[2][2]] as Vector3,
  }
}

function multiplyJacobian(jacobian: Matrix6x3, vector: Vector3): Vector3 {
  return [0, 1, 2].map((row) => (
    jacobian[row][0] * vector[0]
    + jacobian[row][1] * vector[1]
    + jacobian[row][2] * vector[2]
  )) as unknown as Vector3
}

function average(points: readonly Vector3[]): Vector3 {
  if (points.length === 0) return ZERO_VECTOR
  const total = points.reduce<[number, number, number]>((sum, point) => [
    sum[0] + point[0], sum[1] + point[1], sum[2] + point[2],
  ], [0, 0, 0])
  return scale(total, 1 / points.length)
}

function displayVectorLength(trueMagnitude: number): number {
  if (trueMagnitude === 0) return 0
  return Math.min(MAXIMUM_VECTOR_LENGTH, 0.75 + 0.6 * trueMagnitude / (1 + trueMagnitude))
}

function vectorOverlay(
  input: Omit<
    VectorOverlayModel,
    'direction' | 'end' | 'shaftMidpoint' | 'quaternion' | 'displayLength' | 'magnitude' | 'normalized'
  >,
): VectorOverlayModel {
  const trueMagnitude = magnitude(input.vector)
  const direction = normalized(input.vector)
  const displayLength = displayVectorLength(trueMagnitude)
  const end: Vector3 = [
    input.origin[0] + direction[0] * displayLength,
    input.origin[1] + direction[1] * displayLength,
    input.origin[2] + direction[2] * displayLength,
  ]
  return {
    ...input,
    direction,
    end,
    shaftMidpoint: midpoint(input.origin, end),
    quaternion: serializableQuaternion(quaternionFromYAxis(direction)),
    displayLength,
    magnitude: trueMagnitude,
    normalized: true,
  }
}

function frameTransform(position: Vector3): Matrix4 {
  return [
    [1, 0, 0, position[0]],
    [0, 1, 0, position[1]],
    [0, 0, 1, position[2]],
    [0, 0, 0, 1],
  ]
}

export function buildSceneModel(input: SceneCalculationInput): SceneModel {
  const overlays = { ...DEFAULT_SCENE_OVERLAYS, ...input.overlays }
  const { forward } = input
  const frameTransforms = [
    frameTransform(forward.origins[0]),
    forward.transforms[0],
    forward.transforms[1],
    forward.transforms[2],
    forward.transforms[3],
  ] as const
  const framePositions = [
    forward.origins[0],
    forward.origins[1],
    forward.origins[2],
    forward.origins[3],
    forward.endEffectorPosition,
  ] as const

  const vectors: VectorOverlayModel[] = [
    vectorOverlay({
      id: 'linear-velocity',
      label: 'vₑ',
      origin: forward.endEffectorPosition,
      vector: multiplyJacobian(input.jacobian, input.jointState.qd),
      unit: 'm/s',
      color: '#38bdf8',
      visible: overlays.linearVelocity,
    }),
    vectorOverlay({
      id: 'acceleration',
      label: 'aₑ',
      origin: forward.endEffectorPosition,
      vector: multiplyJacobian(input.jacobian, input.jointState.qdd),
      unit: 'm/s²',
      color: '#f59e0b',
      visible: overlays.acceleration,
    }),
    vectorOverlay({
      id: 'gravity',
      label: 'g',
      origin: average(forward.centerOfMassPositions),
      vector: input.gravity,
      unit: 'm/s²',
      color: '#a78bfa',
      visible: overlays.gravity,
    }),
    ...input.torque.map((signedMagnitude, index) => vectorOverlay({
      id: `torque-${index + 1}`,
      label: `τ${String.fromCharCode(0x2081 + index)}`,
      origin: forward.origins[index],
      vector: scale(forward.jointAxes[index], signedMagnitude),
      signedMagnitude,
      unit: 'N·m',
      color: '#fb7185',
      visible: overlays.torque,
    })),
  ]

  return {
    joints: forward.origins.map((position, index) => ({
      id: `joint-${index}`,
      label: index === 3 ? 'e' : String(index + 1),
      position,
      radius: index === 3 ? 0.105 : 0.13,
      color: index === 3 ? '#f5c86b' : '#e8f0ef',
    })),
    links: forward.origins.slice(0, -1).map((start, index) => {
      const end = forward.origins[index + 1]
      const segment = vectorBetween(start, end)
      return {
        id: `link-${index + 1}`,
        start,
        end,
        midpoint: midpoint(start, end),
        quaternion: serializableQuaternion(quaternionFromYAxis(segment)),
        length: magnitude(segment),
        radius: 0.075,
        color: index === 0 ? '#74a8a5' : '#dcecea',
      }
    }),
    coordinateFrames: frameTransforms.map((transform, index) => ({
      id: `frame-${index === 4 ? 'e' : index}`,
      label: index === 4 ? '{e}' : `{${index}}`,
      position: framePositions[index],
      quaternion: serializableQuaternion(quaternionFromTransform(transform)),
      axes: axesFromTransform(transform),
      axisLength: index === 0 ? 0.52 : 0.38,
      visible: overlays.coordinateFrames,
    })),
    centerOfMassMarkers: forward.centerOfMassPositions.map((position, index) => ({
      id: `center-of-mass-${index + 1}`,
      label: `c${String.fromCharCode(0x2081 + index)}`,
      position,
      color: '#f5c86b',
      visible: overlays.centerOfMass,
    })),
    workspace: {
      points: input.workspaceSamples ?? [],
      color: '#58b8b1',
      opacity: 0.18,
      visible: overlays.workspace,
    },
    trail: {
      points: input.trail ?? [],
      color: '#f5c86b',
      visible: overlays.trail,
    },
    vectors,
    gridVisible: overlays.grid,
  }
}
