export type Vector3 = readonly [number, number, number]

export type Matrix3 = readonly [Vector3, Vector3, Vector3]

export type Matrix6x3 = readonly [
  Vector3,
  Vector3,
  Vector3,
  Vector3,
  Vector3,
  Vector3,
]

export type Matrix4 = readonly [
  readonly [number, number, number, number],
  readonly [number, number, number, number],
  readonly [number, number, number, number],
  readonly [number, number, number, number],
]

export interface LinkParameters {
  mass: number
  centerOfMass: Vector3
  inertia: Matrix3
}

export interface RobotParameters {
  geometry: { d1: number; l2: number; l3: number }
  links: readonly [LinkParameters, LinkParameters, LinkParameters]
  gravity: Vector3
  viscousFriction: Vector3
  frictionEnabled: boolean
  jointLimits: readonly [
    readonly [number, number],
    readonly [number, number],
    readonly [number, number],
  ]
}

export interface JointState {
  q: Vector3
  qd: Vector3
  qdd: Vector3
}

export interface ValidationIssue {
  path: string
  code: string
  message: string
}
