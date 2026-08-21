import { evaluateTrajectory, type TrajectoryConfig } from '../../robotics/trajectories'
import type { RobotParameters, Vector3 } from '../../robotics/types'

export type PtpProfile = 'quintic' | 'trapezoidal'

export interface TeachPoint {
  id: string
  name: string
  q: Vector3
}

export interface PtpInstructionDraft {
  startPointId: string
  endPointId: string
  profile: PtpProfile
  durationText: string
}

export interface PtpInstruction {
  startPointId: string
  endPointId: string
  q0: Vector3
  qf: Vector3
  profile: PtpProfile
  duration: number
}

export interface TimedTrajectorySample {
  time: number
  q: Vector3
  qd: Vector3
  qdd: Vector3
}

export interface TrajectoryMetrics {
  peakVelocity: Vector3
  peakAcceleration: Vector3
  startError: Vector3
  endError: Vector3
}

export interface TrajectoryPreview {
  instruction: PtpInstruction
  samples: readonly TimedTrajectorySample[]
  metrics: TrajectoryMetrics
}

export interface NormalizedProfileSample {
  position: number
  velocity: number
  acceleration: number
}

export type PreviewResult =
  | { ok: true; preview: TrajectoryPreview }
  | { ok: false; error: string }

export const PREVIEW_SAMPLE_COUNT = 201
export const MIN_TRAJECTORY_DURATION = 0.1
export const MAX_TRAJECTORY_DURATION = 30

export function degreesToRadians(value: number): number {
  return value * Math.PI / 180
}

export function radiansToDegrees(value: number): number {
  return value * 180 / Math.PI
}

export function vectorRadiansToDegrees(vector: Vector3): Vector3 {
  return vector.map(radiansToDegrees) as unknown as Vector3
}

function cloneVector(vector: Vector3): Vector3 {
  return [...vector] as Vector3
}

function vectorDifference(left: Vector3, right: Vector3): Vector3 {
  return left.map((value, index) => value - right[index]) as unknown as Vector3
}

function mapVector(vector: Vector3, transform: (value: number, index: number) => number): Vector3 {
  return vector.map(transform) as unknown as Vector3
}

function trajectoryConfig(instruction: PtpInstruction): TrajectoryConfig {
  return {
    type: instruction.profile,
    q0: instruction.q0,
    qf: instruction.qf,
    duration: instruction.duration,
  }
}

export function normalizedProfileSample(
  profile: PtpProfile,
  normalizedTime: number,
): NormalizedProfileSample {
  const u = Math.min(1, Math.max(0, normalizedTime))
  if (profile === 'quintic') {
    return {
      position: 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5,
      velocity: 30 * u ** 2 - 60 * u ** 3 + 30 * u ** 4,
      acceleration: 60 * u - 180 * u ** 2 + 120 * u ** 3,
    }
  }

  if (u <= 1 / 3) {
    return { position: 2.25 * u ** 2, velocity: 4.5 * u, acceleration: 4.5 }
  }
  if (u <= 2 / 3) {
    return { position: 1.5 * u - 0.25, velocity: 1.5, acceleration: 0 }
  }
  return {
    position: 1 - 2.25 * (1 - u) ** 2,
    velocity: 4.5 * (1 - u),
    acceleration: -4.5,
  }
}

export function evaluatePtpInstruction(
  instruction: PtpInstruction,
  time: number,
): TimedTrajectorySample {
  const clampedTime = Math.min(instruction.duration, Math.max(0, time))
  return {
    time: clampedTime,
    ...evaluateTrajectory(trajectoryConfig(instruction), clampedTime),
  }
}

export function samplePtpInstruction(
  instruction: PtpInstruction,
  sampleCount = PREVIEW_SAMPLE_COUNT,
): readonly TimedTrajectorySample[] {
  if (!Number.isInteger(sampleCount) || sampleCount < 2) {
    throw new RangeError('轨迹采样点数至少为 2。')
  }
  return Array.from({ length: sampleCount }, (_, index) => (
    evaluatePtpInstruction(instruction, instruction.duration * index / (sampleCount - 1))
  ))
}

export function trajectoryMetrics(
  instruction: PtpInstruction,
  samples: readonly TimedTrajectorySample[],
): TrajectoryMetrics {
  const displacement = mapVector(instruction.q0, (value, index) => instruction.qf[index] - value)
  const velocityFactor = instruction.profile === 'quintic' ? 1.875 : 1.5
  const accelerationFactor = instruction.profile === 'quintic' ? 10 / Math.sqrt(3) : 4.5
  const start = samples[0]
  const finish = samples.at(-1)
  if (start === undefined || finish === undefined) {
    throw new RangeError('轨迹预览必须包含起点和终点。')
  }
  return {
    peakVelocity: mapVector(displacement, (value) => Math.abs(value) * velocityFactor / instruction.duration),
    peakAcceleration: mapVector(displacement, (value) => Math.abs(value) * accelerationFactor / instruction.duration ** 2),
    startError: vectorDifference(start.q, instruction.q0),
    endError: vectorDifference(finish.q, instruction.qf),
  }
}

function isFiniteVector(vector: Vector3): boolean {
  return vector.every(Number.isFinite)
}

function withinJointLimits(q: Vector3, jointLimits: RobotParameters['jointLimits']): boolean {
  return q.every((value, index) => value >= jointLimits[index][0] && value <= jointLimits[index][1])
}

function vectorsEqual(left: Vector3, right: Vector3, tolerance = 1e-12): boolean {
  return left.every((value, index) => Math.abs(value - right[index]) <= tolerance)
}

export function buildTrajectoryPreview(
  points: readonly TeachPoint[],
  draft: PtpInstructionDraft,
  jointLimits: RobotParameters['jointLimits'],
): PreviewResult {
  const start = points.find((point) => point.id === draft.startPointId)
  const finish = points.find((point) => point.id === draft.endPointId)
  if (start === undefined || finish === undefined) {
    return { ok: false, error: '请先选择存在的起点和终点示教点。' }
  }
  if (!isFiniteVector(start.q) || !isFiniteVector(finish.q)) {
    return { ok: false, error: '示教点关节角必须是有限数值。' }
  }
  if (!withinJointLimits(start.q, jointLimits) || !withinJointLimits(finish.q, jointLimits)) {
    return { ok: false, error: '示教点超出当前机器人关节限位。' }
  }
  if (vectorsEqual(start.q, finish.q)) {
    return { ok: false, error: '起点与终点相同，没有需要执行的 PTP 位移。' }
  }

  const duration = Number(draft.durationText)
  if (
    draft.durationText.trim() === ''
    || !Number.isFinite(duration)
    || duration < MIN_TRAJECTORY_DURATION
    || duration > MAX_TRAJECTORY_DURATION
  ) {
    return {
      ok: false,
      error: `持续时间必须在 ${MIN_TRAJECTORY_DURATION}–${MAX_TRAJECTORY_DURATION} s 之间。`,
    }
  }
  if (draft.profile !== 'quintic' && draft.profile !== 'trapezoidal') {
    return { ok: false, error: '第一版 PTP 仅支持五次多项式和梯形速度。' }
  }

  const instruction: PtpInstruction = {
    startPointId: start.id,
    endPointId: finish.id,
    q0: cloneVector(start.q),
    qf: cloneVector(finish.q),
    profile: draft.profile,
    duration,
  }

  try {
    const samples = samplePtpInstruction(instruction)
    if (samples.some((sample) => (
      !Number.isFinite(sample.time)
      || !isFiniteVector(sample.q)
      || !isFiniteVector(sample.qd)
      || !isFiniteVector(sample.qdd)
    ))) {
      return { ok: false, error: '轨迹生成产生了非有限数值。' }
    }
    return {
      ok: true,
      preview: {
        instruction,
        samples,
        metrics: trajectoryMetrics(instruction, samples),
      },
    }
  } catch (error) {
    return {
      ok: false,
      error: error instanceof Error ? error.message : '轨迹生成失败。',
    }
  }
}

