import type { RobotParameters, Vector3 } from './types'

export type WorkspaceSampleCounts = readonly [number, number, number]

const DEFAULT_WORKSPACE_COUNTS: WorkspaceSampleCounts = [30, 16, 18]
const DEDUPLICATION_SCALE = 1e5

function linspace(minimum: number, maximum: number, count: number): readonly number[] {
  const step = (maximum - minimum) / (count - 1)
  return Array.from({ length: count }, (_, index) => minimum + index * step)
}

function roundedPositionKey(position: Vector3): string {
  return position.map((value) => Math.round(value * DEDUPLICATION_SCALE)).join(',')
}

export function sampleWorkspace(
  parameters: RobotParameters,
  counts: WorkspaceSampleCounts = DEFAULT_WORKSPACE_COUNTS,
): Vector3[] {
  if (
    counts.length !== 3
    || counts.some((count) => !Number.isInteger(count) || count < 2)
  ) {
    throw new RangeError('Workspace counts must be three integers greater than or equal to two')
  }

  const { d1, l2, l3 } = parameters.geometry
  if (![d1, l2, l3].every(Number.isFinite) || d1 < 0 || l2 <= 0 || l3 <= 0) {
    throw new RangeError('Workspace geometry requires finite d1 >= 0 and positive link lengths')
  }

  const jointValues = parameters.jointLimits.map(([minimum, maximum], index) => {
    if (!Number.isFinite(minimum) || !Number.isFinite(maximum) || minimum > maximum) {
      throw new RangeError(`Workspace joint limit ${index + 1} is invalid`)
    }
    return linspace(minimum, maximum, counts[index])
  })
  const positions = new Map<string, Vector3>()

  for (const theta1 of jointValues[0]) {
    for (const theta2 of jointValues[1]) {
      for (const theta3 of jointValues[2]) {
        const radial = l2 * Math.cos(theta2) + l3 * Math.cos(theta2 + theta3)
        const position: Vector3 = [
          radial * Math.cos(theta1),
          radial * Math.sin(theta1),
          d1 + l2 * Math.sin(theta2) + l3 * Math.sin(theta2 + theta3),
        ]
        const key = roundedPositionKey(position)
        if (!positions.has(key)) {
          positions.set(key, position)
        }
      }
    }
  }

  return [...positions.values()]
}
