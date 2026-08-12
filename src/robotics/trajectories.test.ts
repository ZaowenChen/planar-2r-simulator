import { describe, expect, it } from 'vitest'
import {
  evaluateTorqueProfile,
  quinticTrajectory,
  sinusoidalTrajectory,
} from './trajectories'

describe('joint trajectories', () => {
  it('meets all zero-velocity quintic endpoint constraints', () => {
    const config = {
      q0: [0, 0.2, -0.3],
      qf: [1, -0.4, 0.5],
      duration: 2,
    } as const

    expect(quinticTrajectory(config, 0)).toEqual({
      q: config.q0,
      qd: [0, 0, 0],
      qdd: [0, 0, 0],
    })
    expect(quinticTrajectory(config, 2)).toEqual({
      q: config.qf,
      qd: [0, 0, 0],
      qdd: [0, 0, 0],
    })
  })

  it('clamps quintic evaluation to its experiment interval', () => {
    const config = {
      q0: [0, 0.2, -0.3],
      qf: [1, -0.4, 0.5],
      duration: 2,
    } as const

    expect(quinticTrajectory(config, -1)).toEqual(quinticTrajectory(config, 0))
    expect(quinticTrajectory(config, 3)).toEqual(quinticTrajectory(config, 2))
  })

  it('returns analytical sine position, velocity, and acceleration', () => {
    const sample = sinusoidalTrajectory({
      center: [0.2, -0.1, 0.4],
      amplitude: [1, 2, 0.5],
      frequency: [0.25, 0.5, 1],
      phase: [0, Math.PI / 2, Math.PI],
      duration: 2,
    }, 0)

    expect(sample.q[0]).toBeCloseTo(0.2, 12)
    expect(sample.q[1]).toBeCloseTo(1.9, 12)
    expect(sample.q[2]).toBeCloseTo(0.4, 12)
    expect(sample.qd[0]).toBeCloseTo(Math.PI / 2, 12)
    expect(sample.qd[1]).toBeCloseTo(0, 12)
    expect(sample.qd[2]).toBeCloseTo(-Math.PI, 12)
    expect(sample.qdd[0]).toBeCloseTo(0, 12)
    expect(sample.qdd[1]).toBeCloseTo(-2 * Math.PI ** 2, 12)
    expect(sample.qdd[2]).toBeCloseTo(0, 12)
  })
})

describe('torque profiles', () => {
  it('evaluates constant and step profiles at clamped times', () => {
    expect(evaluateTorqueProfile({
      type: 'constant',
      value: [1, 2, 3],
      duration: 2,
    }, -1)).toEqual([1, 2, 3])

    const step = {
      type: 'step',
      before: [1, 2, 3],
      after: [-1, -2, -3],
      stepTime: 0.5,
      duration: 1,
    } as const
    expect(evaluateTorqueProfile(step, 0.499)).toEqual(step.before)
    expect(evaluateTorqueProfile(step, 0.5)).toEqual(step.after)
    expect(evaluateTorqueProfile(step, 2)).toEqual(step.after)
  })

  it('evaluates sine torque using frequency in hertz', () => {
    expect(evaluateTorqueProfile({
      type: 'sine',
      offset: [1, 2, 3],
      amplitude: [2, 4, 6],
      frequency: [0.5, 0.5, 0.5],
      phase: [0, Math.PI / 2, Math.PI],
      duration: 2,
    }, 0)).toEqual([1, 6, 3.000000000000001])
  })

  it('selects the most recent piecewise-constant segment', () => {
    const profile = {
      type: 'piecewise-constant',
      segments: [
        { time: 0, value: [1, 0, 0] },
        { time: 0.4, value: [0, 2, 0] },
        { time: 0.8, value: [0, 0, 3] },
      ],
      duration: 1,
    } as const

    expect(evaluateTorqueProfile(profile, -1)).toEqual([1, 0, 0])
    expect(evaluateTorqueProfile(profile, 0.4)).toEqual([0, 2, 0])
    expect(evaluateTorqueProfile(profile, 2)).toEqual([0, 0, 3])
  })

  it('selects the latest eligible piecewise segment regardless of input order', () => {
    const profile = {
      type: 'piecewise-constant',
      segments: [
        { time: 0.8, value: [0, 0, 3] },
        { time: 0, value: [1, 0, 0] },
        { time: 0.4, value: [0, 2, 0] },
      ],
      duration: 1,
    } as const

    expect(evaluateTorqueProfile(profile, 0)).toEqual([1, 0, 0])
    expect(evaluateTorqueProfile(profile, 0.6)).toEqual([0, 2, 0])
    expect(evaluateTorqueProfile(profile, 1)).toEqual([0, 0, 3])
  })
})
