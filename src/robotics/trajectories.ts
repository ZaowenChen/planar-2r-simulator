import type { Vector3 } from './types'

export interface TrajectorySample {
  q: Vector3
  qd: Vector3
  qdd: Vector3
}

export interface QuinticTrajectoryConfig {
  type?: 'quintic'
  q0: Vector3
  qf: Vector3
  duration: number
}

export interface SinusoidalTrajectoryConfig {
  type?: 'sinusoidal'
  center: Vector3
  amplitude: Vector3
  frequency: Vector3
  phase: Vector3
  duration: number
}

export type TrajectoryConfig =
  | (QuinticTrajectoryConfig & { type: 'quintic' })
  | (SinusoidalTrajectoryConfig & { type: 'sinusoidal' })

interface TimedProfile {
  duration: number
}

export interface ConstantTorqueProfile extends TimedProfile {
  type: 'constant'
  value: Vector3
}

export interface StepTorqueProfile extends TimedProfile {
  type: 'step'
  before: Vector3
  after: Vector3
  stepTime: number
}

export interface SineTorqueProfile extends TimedProfile {
  type: 'sine'
  offset: Vector3
  amplitude: Vector3
  frequency: Vector3
  phase: Vector3
}

export interface PiecewiseConstantTorqueProfile extends TimedProfile {
  type: 'piecewise-constant'
  segments: readonly {
    time: number
    value: Vector3
  }[]
}

export type TorqueProfile =
  | ConstantTorqueProfile
  | StepTorqueProfile
  | SineTorqueProfile
  | PiecewiseConstantTorqueProfile

export type TorqueEvaluationSide = 'left' | 'right'

function requireDuration(duration: number): void {
  if (!Number.isFinite(duration) || duration <= 0) {
    throw new RangeError('Duration must be finite and positive')
  }
}

function clampedTime(time: number, duration: number): number {
  requireDuration(duration)
  if (!Number.isFinite(time)) {
    throw new RangeError('Evaluation time must be finite')
  }
  return Math.min(duration, Math.max(0, time))
}

function mapVector(
  vector: Vector3,
  transform: (value: number, index: number) => number,
): Vector3 {
  return vector.map(transform) as unknown as Vector3
}

export function quinticTrajectory(
  config: QuinticTrajectoryConfig,
  time: number,
): TrajectorySample {
  const t = clampedTime(time, config.duration)
  if (t === 0) {
    return { q: config.q0, qd: [0, 0, 0], qdd: [0, 0, 0] }
  }
  if (t === config.duration) {
    return { q: config.qf, qd: [0, 0, 0], qdd: [0, 0, 0] }
  }

  const u = t / config.duration
  const u2 = u * u
  const u3 = u2 * u
  const u4 = u3 * u
  const u5 = u4 * u
  const blend = 10 * u3 - 15 * u4 + 6 * u5
  const blendVelocity = (30 * u2 - 60 * u3 + 30 * u4) / config.duration
  const blendAcceleration = (60 * u - 180 * u2 + 120 * u3)
    / (config.duration * config.duration)
  const displacement = mapVector(config.q0, (value, index) => (
    config.qf[index] - value
  ))

  return {
    q: mapVector(config.q0, (value, index) => value + displacement[index] * blend),
    qd: mapVector(displacement, (value) => value * blendVelocity),
    qdd: mapVector(displacement, (value) => value * blendAcceleration),
  }
}

export function sinusoidalTrajectory(
  config: SinusoidalTrajectoryConfig,
  time: number,
): TrajectorySample {
  const t = clampedTime(time, config.duration)
  const angularFrequency = mapVector(config.frequency, (value) => 2 * Math.PI * value)
  const angleAt = (index: number): number => (
    angularFrequency[index] * t + config.phase[index]
  )

  return {
    q: mapVector(config.center, (value, index) => (
      value + config.amplitude[index] * Math.sin(angleAt(index))
    )),
    qd: mapVector(config.center, (_, index) => (
      config.amplitude[index] * angularFrequency[index] * Math.cos(angleAt(index))
    )),
    qdd: mapVector(config.center, (_, index) => (
      -config.amplitude[index]
      * angularFrequency[index] ** 2
      * Math.sin(angleAt(index))
    )),
  }
}

export function evaluateTrajectory(
  config: TrajectoryConfig,
  time: number,
): TrajectorySample {
  return config.type === 'quintic'
    ? quinticTrajectory(config, time)
    : sinusoidalTrajectory(config, time)
}

export function evaluateTorqueProfile(
  profile: TorqueProfile,
  time: number,
  side: TorqueEvaluationSide = 'right',
): Vector3 {
  const t = clampedTime(time, profile.duration)
  switch (profile.type) {
    case 'constant':
      return profile.value
    case 'step':
      return t < profile.stepTime
        || (side === 'left' && t === profile.stepTime)
        ? profile.before
        : profile.after
    case 'sine':
      return mapVector(profile.offset, (value, index) => (
        value + profile.amplitude[index] * Math.sin(
          2 * Math.PI * profile.frequency[index] * t + profile.phase[index],
        )
      ))
    case 'piecewise-constant': {
      if (profile.segments.length === 0) {
        throw new RangeError('Piecewise-constant profile needs at least one segment')
      }
      if (!profile.segments.some(({ time: segmentTime }) => segmentTime === 0)) {
        throw new RangeError('Piecewise-constant profile must have a segment at time zero')
      }
      let selected: PiecewiseConstantTorqueProfile['segments'][number] | undefined
      for (const segment of profile.segments) {
        const eligible = side === 'left' ? segment.time < t : segment.time <= t
        if (eligible && (selected === undefined || segment.time >= selected.time)) {
          selected = segment
        }
      }
      if (selected === undefined) {
        throw new RangeError('Piecewise-constant profile has no value before evaluation time')
      }
      return selected.value
    }
  }
}

export function torqueDiscontinuityTimes(
  profile: TorqueProfile,
): readonly number[] {
  const times = profile.type === 'step'
    ? [profile.stepTime]
    : profile.type === 'piecewise-constant'
      ? profile.segments.map(({ time }) => time)
      : []
  return [...new Set(times)]
    .filter((time) => Number.isFinite(time) && time > 0 && time <= profile.duration)
    .sort((left, right) => left - right)
}
