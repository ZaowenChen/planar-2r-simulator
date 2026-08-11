import { describe, expect, it } from 'vitest'
import { DEFAULT_ROBOT_PARAMETERS } from './defaults'
import { sampleWorkspace } from './workspace'

describe('workspace sampling', () => {
  it('returns the deterministic de-duplicated tensor-grid positions', () => {
    const points = sampleWorkspace(DEFAULT_ROBOT_PARAMETERS, [2, 2, 2])

    expect(points).toHaveLength(4)
    expect(points.every((point) => point.every(Number.isFinite))).toBe(true)
  })

  it('keeps every point inside the analytical maximum reach', () => {
    const { d1, l2, l3 } = DEFAULT_ROBOT_PARAMETERS.geometry
    const points = sampleWorkspace(DEFAULT_ROBOT_PARAMETERS, [6, 4, 5])

    for (const [x, y, z] of points) {
      expect(Math.hypot(x, y, z - d1)).toBeLessThanOrEqual(l2 + l3 + 1e-12)
    }
  })

  it.each([
    [1, 2, 2],
    [2, 2.5, 2],
  ] as const)('rejects invalid tensor-grid counts %j', (...counts) => {
    expect(() => sampleWorkspace(DEFAULT_ROBOT_PARAMETERS, counts)).toThrow(RangeError)
  })
})
