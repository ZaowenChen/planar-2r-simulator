import { describe, expect, it } from 'vitest'
import { DEFAULT_ROBOT_PARAMETERS } from '../../robotics/defaults'
import {
  buildTrajectoryPreview,
  evaluatePtpInstruction,
  normalizedProfileSample,
  samplePtpInstruction,
  type PtpInstruction,
  type TeachPoint,
} from './trajectoryModel'

const points: readonly TeachPoint[] = [
  { id: 'p1', name: 'P1', q: [0, 0.1, -0.2] },
  { id: 'p2', name: 'P2', q: [0.8, -0.4, 0.5] },
]

describe('trajectory teaching model', () => {
  it('builds a 201-point quintic preview with exact analytical metrics', () => {
    const result = buildTrajectoryPreview(points, {
      startPointId: 'p1',
      endPointId: 'p2',
      profile: 'quintic',
      durationText: '2',
    }, DEFAULT_ROBOT_PARAMETERS.jointLimits)

    expect(result.ok).toBe(true)
    if (!result.ok) return
    expect(result.preview.samples).toHaveLength(201)
    expect(result.preview.samples[0].q).toEqual(points[0].q)
    expect(result.preview.samples.at(-1)?.q).toEqual(points[1].q)
    expect(result.preview.metrics.peakVelocity[0]).toBeCloseTo(0.8 * 1.875 / 2, 12)
    expect(result.preview.metrics.peakAcceleration[0]).toBeCloseTo(0.8 * 10 / Math.sqrt(3) / 4, 12)
    expect(result.preview.metrics.endError).toEqual([0, 0, 0])
  })

  it('applies duration scaling to velocity and acceleration without changing normalized position', () => {
    const base: PtpInstruction = {
      startPointId: 'p1', endPointId: 'p2', profile: 'quintic',
      q0: [0, 0, 0], qf: [1, -0.5, 0.25], duration: 2,
    }
    const slower = { ...base, duration: 4 }
    const fastSample = evaluatePtpInstruction(base, 0.8)
    const slowSample = evaluatePtpInstruction(slower, 1.6)

    expect(slowSample.q).toEqual(fastSample.q)
    fastSample.qd.forEach((value, index) => expect(slowSample.qd[index]).toBeCloseTo(value / 2, 12))
    fastSample.qdd.forEach((value, index) => expect(slowSample.qdd[index]).toBeCloseTo(value / 4, 12))
  })

  it('exposes continuous trapezoidal position and velocity with segmented acceleration', () => {
    const before = normalizedProfileSample('trapezoidal', 1 / 3 - 1e-9)
    const boundary = normalizedProfileSample('trapezoidal', 1 / 3)
    const after = normalizedProfileSample('trapezoidal', 1 / 3 + 1e-9)

    expect(before.position).toBeCloseTo(boundary.position, 8)
    expect(after.position).toBeCloseTo(boundary.position, 8)
    expect(before.velocity).toBeCloseTo(boundary.velocity, 8)
    expect(after.velocity).toBeCloseTo(boundary.velocity, 8)
    expect(boundary.acceleration).toBe(4.5)
    expect(after.acceleration).toBe(0)
  })

  it('rejects missing, equal, out-of-limit, and invalid-duration point-to-point tasks', () => {
    const limits = DEFAULT_ROBOT_PARAMETERS.jointLimits
    const draft = { startPointId: 'p1', endPointId: 'p2', profile: 'quintic', durationText: '2' } as const
    expect(buildTrajectoryPreview(points, { ...draft, endPointId: 'missing' }, limits)).toMatchObject({ ok: false })
    expect(buildTrajectoryPreview([points[0], { ...points[1], q: points[0].q }], draft, limits)).toMatchObject({ ok: false })
    expect(buildTrajectoryPreview([points[0], { ...points[1], q: [4, 0, 0] }], draft, limits)).toMatchObject({ ok: false })
    expect(buildTrajectoryPreview(points, { ...draft, durationText: '0.01' }, limits)).toMatchObject({ ok: false })
  })

  it('requires at least two preview samples and includes both endpoints', () => {
    const instruction: PtpInstruction = {
      startPointId: 'p1', endPointId: 'p2', profile: 'trapezoidal',
      q0: [0, 0, 0], qf: [1, 2, 3], duration: 3,
    }
    expect(() => samplePtpInstruction(instruction, 1)).toThrow('至少为 2')
    expect(samplePtpInstruction(instruction, 2).map((sample) => sample.time)).toEqual([0, 3])
  })
})
