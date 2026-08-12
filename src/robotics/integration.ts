import { energy, forwardDynamics, inverseDynamics } from './dynamics'
import {
  evaluateTorqueProfile,
  evaluateTrajectory,
} from './trajectories'
import type {
  TorqueProfile,
  TrajectoryConfig,
  TrajectorySample,
} from './trajectories'
import type { RobotParameters, Vector3 } from './types'

type JointState6 = readonly [number, number, number, number, number, number]

export interface SimulationSample extends TrajectorySample {
  time: number
  tau: Vector3
  kinetic: number
  potential: number
  totalEnergy: number
  jointPower: Vector3
}

export interface InverseDynamicsSimulationConfig {
  trajectory: TrajectoryConfig
  duration: number
  stepSize: number
}

export interface ForwardDynamicsSimulationConfig {
  initialState: { q: Vector3; qd: Vector3 }
  torqueProfile: TorqueProfile
  duration: number
  stepSize: number
}

export interface NonFiniteDiagnostic {
  kind: 'non-finite'
  time: number
  field: 'state' | 'torque' | 'acceleration' | 'sample'
  message: string
}

export interface JointLimitDiagnostic {
  kind: 'joint-limit'
  time: number
  jointIndex: number
  value: number
  minimum: number
  maximum: number
  message: string
}

export type SimulationDiagnostic = NonFiniteDiagnostic | JointLimitDiagnostic

export type SimulationSamples = SimulationSample[] & {
  diagnostic?: SimulationDiagnostic
}

function requireSimulationInterval(duration: number, stepSize: number): void {
  if (!Number.isFinite(duration) || duration < 0) {
    throw new RangeError('Simulation duration must be finite and nonnegative')
  }
  if (!Number.isFinite(stepSize) || stepSize <= 0) {
    throw new RangeError('Integration step size must be finite and positive')
  }
}

function timeGrid(duration: number, stepSize: number): number[] {
  requireSimulationInterval(duration, stepSize)
  const times: number[] = []
  const fullSteps = Math.floor(duration / stepSize)
  for (let index = 0; index <= fullSteps; index += 1) {
    times.push(Number((index * stepSize).toPrecision(15)))
  }
  if (times.length === 0 || times[times.length - 1] < duration) {
    times.push(duration)
  } else {
    times[times.length - 1] = duration
  }
  return times
}

export function integrateScalarRk4(
  initialValue: number,
  initialTime: number,
  finalTime: number,
  stepSize: number,
  derivative: (value: number, time: number) => number,
): number {
  if (!Number.isFinite(initialTime) || !Number.isFinite(finalTime)) {
    throw new RangeError('Integration times must be finite')
  }
  if (finalTime < initialTime) {
    throw new RangeError('Final time must not precede initial time')
  }
  requireSimulationInterval(finalTime - initialTime, stepSize)

  let value = initialValue
  let time = initialTime
  while (time < finalTime) {
    const step = Math.min(stepSize, finalTime - time)
    const k1 = derivative(value, time)
    const k2 = derivative(value + step * k1 / 2, time + step / 2)
    const k3 = derivative(value + step * k2 / 2, time + step / 2)
    const k4 = derivative(value + step * k3, time + step)
    value += step * (k1 + 2 * k2 + 2 * k3 + k4) / 6
    time += step
  }
  return value
}

function isFiniteVector(vector: readonly number[]): boolean {
  return vector.every(Number.isFinite)
}

function stateFromVectors(q: Vector3, qd: Vector3): JointState6 {
  return [q[0], q[1], q[2], qd[0], qd[1], qd[2]]
}

function qFromState(state: JointState6): Vector3 {
  return [state[0], state[1], state[2]]
}

function qdFromState(state: JointState6): Vector3 {
  return [state[3], state[4], state[5]]
}

function addScaledState(
  state: JointState6,
  derivative: JointState6,
  scale: number,
): JointState6 {
  return state.map((value, index) => (
    value + derivative[index] * scale
  )) as unknown as JointState6
}

function stateDerivative(
  state: JointState6,
  time: number,
  profile: TorqueProfile,
  parameters: RobotParameters,
): JointState6 {
  const q = qFromState(state)
  const qd = qdFromState(state)
  const qdd = forwardDynamics(
    q,
    qd,
    evaluateTorqueProfile(profile, time),
    parameters,
  )
  return [qd[0], qd[1], qd[2], qdd[0], qdd[1], qdd[2]]
}

function rk4JointStep(
  state: JointState6,
  time: number,
  stepSize: number,
  profile: TorqueProfile,
  parameters: RobotParameters,
): JointState6 {
  const k1 = stateDerivative(state, time, profile, parameters)
  const k2 = stateDerivative(
    addScaledState(state, k1, stepSize / 2),
    time + stepSize / 2,
    profile,
    parameters,
  )
  const k3 = stateDerivative(
    addScaledState(state, k2, stepSize / 2),
    time + stepSize / 2,
    profile,
    parameters,
  )
  const k4 = stateDerivative(
    addScaledState(state, k3, stepSize),
    time + stepSize,
    profile,
    parameters,
  )
  return state.map((value, index) => (
    value + stepSize * (
      k1[index] + 2 * k2[index] + 2 * k3[index] + k4[index]
    ) / 6
  )) as unknown as JointState6
}

function jointLimitDiagnostic(
  state: JointState6,
  time: number,
  parameters: RobotParameters,
): JointLimitDiagnostic | undefined {
  for (let jointIndex = 0; jointIndex < 3; jointIndex += 1) {
    const value = state[jointIndex]
    const [minimum, maximum] = parameters.jointLimits[jointIndex]
    if (value < minimum || value > maximum) {
      return {
        kind: 'joint-limit',
        time,
        jointIndex,
        value,
        minimum,
        maximum,
        message: `关节 ${jointIndex + 1} 超出限位：[${minimum}, ${maximum}]，实际值 ${value}`,
      }
    }
  }
  return undefined
}

function nonFiniteDiagnostic(
  time: number,
  field: NonFiniteDiagnostic['field'],
): NonFiniteDiagnostic {
  return {
    kind: 'non-finite',
    time,
    field,
    message: `仿真在 ${time} s 的${field}中出现非有限值`,
  }
}

function buildSample(
  time: number,
  state: TrajectorySample,
  tau: Vector3,
  parameters: RobotParameters,
): SimulationSample {
  const energyResult = energy(state, parameters)
  return {
    time,
    ...state,
    tau,
    kinetic: energyResult.kinetic,
    potential: energyResult.potential,
    totalEnergy: energyResult.total,
    jointPower: [
      tau[0] * state.qd[0],
      tau[1] * state.qd[1],
      tau[2] * state.qd[2],
    ],
  }
}

function isFiniteSample(sample: SimulationSample): boolean {
  return Number.isFinite(sample.time)
    && isFiniteVector(sample.q)
    && isFiniteVector(sample.qd)
    && isFiniteVector(sample.qdd)
    && isFiniteVector(sample.tau)
    && Number.isFinite(sample.kinetic)
    && Number.isFinite(sample.potential)
    && Number.isFinite(sample.totalEnergy)
    && isFiniteVector(sample.jointPower)
}

export function simulateInverseDynamics(
  config: InverseDynamicsSimulationConfig,
  parameters: RobotParameters,
): SimulationSamples {
  const samples = [] as SimulationSamples
  for (const time of timeGrid(config.duration, config.stepSize)) {
    const state = evaluateTrajectory(config.trajectory, time)
    const tau = inverseDynamics(state, parameters).tau
    const sample = buildSample(time, state, tau, parameters)
    if (!isFiniteSample(sample)) {
      samples.diagnostic = nonFiniteDiagnostic(time, 'sample')
      break
    }
    const limit = jointLimitDiagnostic(stateFromVectors(state.q, state.qd), time, parameters)
    if (limit !== undefined) {
      samples.diagnostic = limit
      break
    }
    samples.push(sample)
  }
  return samples
}

export function simulateForwardDynamics(
  config: ForwardDynamicsSimulationConfig,
  parameters: RobotParameters,
): SimulationSamples {
  const times = timeGrid(config.duration, config.stepSize)
  const samples = [] as SimulationSamples
  let state = stateFromVectors(config.initialState.q, config.initialState.qd)

  for (let index = 0; index < times.length; index += 1) {
    const time = times[index]
    if (!isFiniteVector(state)) {
      samples.diagnostic = nonFiniteDiagnostic(time, 'state')
      break
    }
    const limit = jointLimitDiagnostic(state, time, parameters)
    if (limit !== undefined) {
      samples.diagnostic = limit
      break
    }
    const tau = evaluateTorqueProfile(config.torqueProfile, time)
    if (!isFiniteVector(tau)) {
      samples.diagnostic = nonFiniteDiagnostic(time, 'torque')
      break
    }
    const q = qFromState(state)
    const qd = qdFromState(state)
    const qdd = forwardDynamics(q, qd, tau, parameters)
    if (!isFiniteVector(qdd)) {
      samples.diagnostic = nonFiniteDiagnostic(time, 'acceleration')
      break
    }
    const sample = buildSample(time, { q, qd, qdd }, tau, parameters)
    if (!isFiniteSample(sample)) {
      samples.diagnostic = nonFiniteDiagnostic(time, 'sample')
      break
    }
    samples.push(sample)

    const nextTime = times[index + 1]
    if (nextTime !== undefined) {
      const nextState = rk4JointStep(
        state,
        time,
        nextTime - time,
        config.torqueProfile,
        parameters,
      )
      if (!isFiniteVector(nextState)) {
        samples.diagnostic = nonFiniteDiagnostic(nextTime, 'state')
        break
      }
      const nextLimit = jointLimitDiagnostic(nextState, nextTime, parameters)
      if (nextLimit !== undefined) {
        samples.diagnostic = nextLimit
        break
      }
      state = nextState
    }
  }
  return samples
}
