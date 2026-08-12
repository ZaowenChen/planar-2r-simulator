import type { LabCalculation } from '../state/labStore'

export const DISPLAY = {
  endEffectorPosition: String.raw`{}^{0}\mathbf{p}_{e}`,
  desiredPosition: String.raw`{}^{0}\mathbf{p}_{d}`,
  transform03: String.raw`{}^{0}\mathbf{T}_{3}`,
  massMatrix: String.raw`\mathbf{M}(\mathbf{q})`,
  coriolisMatrix: String.raw`\mathbf{C}(\mathbf{q},\dot{\mathbf{q}})`,
  gravityVector: String.raw`\mathbf{g}(\mathbf{q})`,
  jointTorque: String.raw`\boldsymbol{\tau}`,
} as const

const SNAPSHOT_COLUMNS = [
  '末端位置 x (m)',
  '末端位置 y (m)',
  '末端位置 z (m)',
  '关节力矩 1 (N·m)',
  '关节力矩 2 (N·m)',
  '关节力矩 3 (N·m)',
] as const

export function calculationSnapshotCsv(calculation: LabCalculation): string {
  const values = [
    ...calculation.forward.endEffectorPosition,
    ...calculation.dynamics.tau,
  ]
  return `${SNAPSHOT_COLUMNS.join(',')}\n${values.join(',')}`
}
