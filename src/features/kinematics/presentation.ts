import type { Matrix4, Matrix6x3 } from '../../robotics/types'

export function metresToMillimetres(_value: number): number {
  return _value * 1000
}

export function millimetresToMetres(value: number): number {
  return value / 1000
}

export function radiansToDegrees(value: number): number {
  return value * 180 / Math.PI
}

export function degreesToRadians(value: number): number {
  return value * Math.PI / 180
}

export function transformInMillimetres(_transform: Matrix4): Matrix4 {
  return _transform.map((row, rowIndex) => row.map((value, columnIndex) => (
    rowIndex < 3 && columnIndex === 3 ? metresToMillimetres(value) : value
  ))) as unknown as Matrix4
}

export function jacobianForMillimetresAndDegrees(
  _jacobian: Matrix6x3,
): Matrix6x3 {
  const linearScale = 1000 * Math.PI / 180
  return _jacobian.map((row, rowIndex) => row.map((value) => (
    rowIndex < 3 ? value * linearScale : value
  ))) as unknown as Matrix6x3
}
