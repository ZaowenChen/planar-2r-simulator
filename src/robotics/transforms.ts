import type { Matrix3, Matrix4, Vector3 } from './types'

const IDENTITY_4: Matrix4 = [
  [1, 0, 0, 0],
  [0, 1, 0, 0],
  [0, 0, 1, 0],
  [0, 0, 0, 1],
]

export type DhTransformOperationKind = 'rz' | 'tz' | 'tx' | 'rx'

export interface DhTransformOperation {
  kind: DhTransformOperationKind
  transform: Matrix4
  cumulative: Matrix4
}

export interface DhTransformDecomposition {
  operations: readonly [
    DhTransformOperation,
    DhTransformOperation,
    DhTransformOperation,
    DhTransformOperation,
  ]
  result: Matrix4
}

export function rotationX(angle: number): Matrix4 {
  const cosine = Math.cos(angle)
  const sine = Math.sin(angle)
  return [
    [1, 0, 0, 0],
    [0, cosine, -sine, 0],
    [0, sine, cosine, 0],
    [0, 0, 0, 1],
  ]
}

export function rotationZ(angle: number): Matrix4 {
  const cosine = Math.cos(angle)
  const sine = Math.sin(angle)
  return [
    [cosine, -sine, 0, 0],
    [sine, cosine, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
  ]
}

export function translation(x: number, y: number, z: number): Matrix4 {
  return [
    [1, 0, 0, x],
    [0, 1, 0, y],
    [0, 0, 1, z],
    [0, 0, 0, 1],
  ]
}

export function multiply4(left: Matrix4, right: Matrix4): Matrix4 {
  const valueAt = (row: number, column: number): number => (
    left[row][0] * right[0][column]
    + left[row][1] * right[1][column]
    + left[row][2] * right[2][column]
    + left[row][3] * right[3][column]
  )

  return [
    [valueAt(0, 0), valueAt(0, 1), valueAt(0, 2), valueAt(0, 3)],
    [valueAt(1, 0), valueAt(1, 1), valueAt(1, 2), valueAt(1, 3)],
    [valueAt(2, 0), valueAt(2, 1), valueAt(2, 2), valueAt(2, 3)],
    [valueAt(3, 0), valueAt(3, 1), valueAt(3, 2), valueAt(3, 3)],
  ]
}

export function transformPoint(transform: Matrix4, point: Vector3): Vector3 {
  return [
    transform[0][0] * point[0] + transform[0][1] * point[1]
      + transform[0][2] * point[2] + transform[0][3],
    transform[1][0] * point[0] + transform[1][1] * point[1]
      + transform[1][2] * point[2] + transform[1][3],
    transform[2][0] * point[0] + transform[2][1] * point[1]
      + transform[2][2] * point[2] + transform[2][3],
  ]
}

export function rotationOf(transform: Matrix4): Matrix3 {
  return [
    [transform[0][0], transform[0][1], transform[0][2]],
    [transform[1][0], transform[1][1], transform[1][2]],
    [transform[2][0], transform[2][1], transform[2][2]],
  ]
}

export function translationOf(transform: Matrix4): Vector3 {
  return [transform[0][3], transform[1][3], transform[2][3]]
}

export function dhTransform(theta: number, a: number, alpha: number, d: number): Matrix4 {
  return decomposeDhTransform(theta, a, alpha, d).result
}

export function decomposeDhTransform(
  theta: number,
  a: number,
  alpha: number,
  d: number,
): DhTransformDecomposition {
  const transforms = [
    { kind: 'rz' as const, transform: rotationZ(theta) },
    { kind: 'tz' as const, transform: translation(0, 0, d) },
    { kind: 'tx' as const, transform: translation(a, 0, 0) },
    { kind: 'rx' as const, transform: rotationX(alpha) },
  ] as const
  let cumulative = IDENTITY_4
  const operations = transforms.map((operation) => {
    cumulative = multiply4(cumulative, operation.transform)
    return { ...operation, cumulative }
  }) as unknown as DhTransformDecomposition['operations']
  return { operations, result: cumulative }
}
