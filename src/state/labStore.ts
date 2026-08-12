import { create } from 'zustand'
import { DEFAULT_JOINT_STATE, DEFAULT_ROBOT_PARAMETERS } from '../robotics/defaults'
import { energy, inverseDynamics } from '../robotics/dynamics'
import type { DynamicsResult, EnergyResult } from '../robotics/dynamics'
import { geometricJacobian, singularityMetrics } from '../robotics/jacobian'
import type { SingularityMetrics } from '../robotics/jacobian'
import { forwardKinematics } from '../robotics/kinematics'
import type { ForwardKinematicsResult } from '../robotics/kinematics'
import type {
  JointState,
  Matrix6x3,
  RobotParameters,
  ValidationIssue,
  Vector3,
} from '../robotics/types'
import { validateRobotParameters } from '../robotics/validation'

export type LabModule = 'model' | 'kinematics' | 'dynamics' | 'experiments'
export type ExperimentMode = 'inverse' | 'forward'
export type AngleUnit = 'degrees' | 'radians'

export interface LabCalculation {
  revision: number
  forward: ForwardKinematicsResult & { q: Vector3 }
  jacobian: Matrix6x3
  singularity: SingularityMetrics
  dynamics: DynamicsResult
  energy: EnergyResult
}

export interface ExperimentSettings {
  mode: ExperimentMode
  duration: number
  integrationStep: number
  playbackSpeed: number
  isPlaying: boolean
}

export interface LabStore {
  rawParameters: Record<string, string>
  parameters: RobotParameters
  parameterIssues: readonly ValidationIssue[]
  fieldIssues: Record<string, string>
  jointState: JointState
  desiredPosition: Vector3
  angleUnit: AngleUnit
  activeModule: LabModule
  experiment: ExperimentSettings
  simulationTime: number
  calculation: LabCalculation
  setParameterField: (path: string, rawValue: string) => void
  setFrictionEnabled: (enabled: boolean) => void
  setJoint: (index: number, value: number) => void
  setJointVelocity: (index: number, value: number) => void
  setJointAcceleration: (index: number, value: number) => void
  setDesiredPosition: (index: number, value: number) => void
  setAngleUnit: (unit: AngleUnit) => void
  setActiveModule: (module: LabModule) => void
  setExperimentMode: (mode: ExperimentMode) => void
  setExperimentDuration: (duration: number) => void
  setIntegrationStep: (step: number) => void
  setPlaybackSpeed: (speed: number) => void
  setPlaying: (playing: boolean) => void
  setSimulationTime: (time: number) => void
  resetSimulationTime: () => void
  resetLab: () => void
}

const DEFAULT_SINGULARITY_THRESHOLD = 1e-3
let calculationRevision = 0

function clone<T>(value: T): T {
  return structuredClone(value)
}

export function calculateLabState(
  parameters: RobotParameters,
  jointState: JointState,
): LabCalculation {
  const forwardResult = forwardKinematics(jointState.q, parameters)
  const jacobian = geometricJacobian(jointState.q, parameters)

  return {
    revision: ++calculationRevision,
    forward: { ...forwardResult, q: jointState.q },
    jacobian,
    singularity: singularityMetrics(jacobian, DEFAULT_SINGULARITY_THRESHOLD),
    dynamics: inverseDynamics(jointState, parameters),
    energy: energy(jointState, parameters),
  }
}

function flattenNumericFields(value: unknown, prefix = ''): Record<string, string> {
  if (typeof value === 'number') {
    return { [prefix]: String(value) }
  }
  if (typeof value !== 'object' || value === null) {
    return {}
  }
  return Object.entries(value).reduce<Record<string, string>>((fields, [key, nested]) => {
    const path = prefix === '' ? key : `${prefix}.${key}`
    return Object.assign(fields, flattenNumericFields(nested, path))
  }, {})
}

function setNumericPath(
  parameters: RobotParameters,
  path: string,
  value: number,
): RobotParameters | undefined {
  const candidate = clone(parameters)
  const segments = path.split('.')
  let cursor: unknown = candidate

  for (const segment of segments.slice(0, -1)) {
    if (typeof cursor !== 'object' || cursor === null || !(segment in cursor)) {
      return undefined
    }
    cursor = (cursor as Record<string, unknown>)[segment]
  }

  const finalSegment = segments.at(-1)
  if (
    finalSegment === undefined
    || typeof cursor !== 'object'
    || cursor === null
    || !(finalSegment in cursor)
    || typeof (cursor as Record<string, unknown>)[finalSegment] !== 'number'
  ) {
    return undefined
  }

  ;(cursor as Record<string, unknown>)[finalSegment] = value
  return candidate
}

function replaceVectorValue(vector: Vector3, index: number, value: number): Vector3 {
  if (!Number.isInteger(index) || index < 0 || index > 2) {
    return vector
  }
  const next = [...vector] as [number, number, number]
  next[index] = value
  return next
}

function initialState() {
  const parameters = clone(DEFAULT_ROBOT_PARAMETERS)
  const jointState = clone(DEFAULT_JOINT_STATE)
  const forward = forwardKinematics(jointState.q, parameters)
  return {
    rawParameters: flattenNumericFields(parameters),
    parameters,
    parameterIssues: [] as ValidationIssue[],
    fieldIssues: {} as Record<string, string>,
    jointState,
    desiredPosition: forward.endEffectorPosition,
    angleUnit: 'degrees' as AngleUnit,
    activeModule: 'model' as LabModule,
    experiment: {
      mode: 'inverse' as ExperimentMode,
      duration: 5,
      integrationStep: 0.005,
      playbackSpeed: 1,
      isPlaying: false,
    },
    simulationTime: 0,
    calculation: calculateLabState(parameters, jointState),
  }
}

export const useLabStore = create<LabStore>((set, get) => ({
  ...initialState(),

  setParameterField: (path, rawValue) => {
    const rawParameters = { ...get().rawParameters, [path]: rawValue }
    const numericValue = Number(rawValue)
    if (rawValue.trim() === '' || !Number.isFinite(numericValue)) {
      set({
        rawParameters,
        fieldIssues: { ...get().fieldIssues, [path]: '请输入有限数值。' },
      })
      return
    }

    const candidate = setNumericPath(get().parameters, path, numericValue)
    if (candidate === undefined) {
      set({
        rawParameters,
        fieldIssues: { ...get().fieldIssues, [path]: '未识别的参数字段。' },
      })
      return
    }

    const parameterIssues = validateRobotParameters(candidate)
    if (parameterIssues.length > 0) {
      const fieldIssues = Object.fromEntries(
        parameterIssues.map((issue) => [issue.path, issue.message]),
      )
      set({ rawParameters, parameterIssues, fieldIssues })
      return
    }

    const fieldIssues = { ...get().fieldIssues }
    delete fieldIssues[path]
    set({
      rawParameters,
      parameters: candidate,
      parameterIssues: [],
      fieldIssues,
      calculation: calculateLabState(candidate, get().jointState),
    })
  },

  setFrictionEnabled: (enabled) => {
    const parameters = { ...get().parameters, frictionEnabled: enabled }
    set({
      parameters,
      calculation: calculateLabState(parameters, get().jointState),
    })
  },

  setJoint: (index, value) => {
    if (!Number.isFinite(value)) return
    const jointState = {
      ...get().jointState,
      q: replaceVectorValue(get().jointState.q, index, value),
    }
    if (jointState.q === get().jointState.q) return
    set({
      jointState,
      calculation: calculateLabState(get().parameters, jointState),
    })
  },

  setJointVelocity: (index, value) => {
    if (!Number.isFinite(value)) return
    const jointState = {
      ...get().jointState,
      qd: replaceVectorValue(get().jointState.qd, index, value),
    }
    if (jointState.qd === get().jointState.qd) return
    set({
      jointState,
      calculation: calculateLabState(get().parameters, jointState),
    })
  },

  setJointAcceleration: (index, value) => {
    if (!Number.isFinite(value)) return
    const jointState = {
      ...get().jointState,
      qdd: replaceVectorValue(get().jointState.qdd, index, value),
    }
    if (jointState.qdd === get().jointState.qdd) return
    set({
      jointState,
      calculation: calculateLabState(get().parameters, jointState),
    })
  },

  setDesiredPosition: (index, value) => {
    if (!Number.isFinite(value)) return
    set({ desiredPosition: replaceVectorValue(get().desiredPosition, index, value) })
  },
  setAngleUnit: (angleUnit) => set({ angleUnit }),
  setActiveModule: (activeModule) => set({ activeModule }),
  setExperimentMode: (mode) => set({ experiment: { ...get().experiment, mode } }),
  setExperimentDuration: (duration) => {
    if (!Number.isFinite(duration) || duration <= 0) return
    set({
      experiment: { ...get().experiment, duration },
      simulationTime: Math.min(get().simulationTime, duration),
    })
  },
  setIntegrationStep: (integrationStep) => {
    if (!Number.isFinite(integrationStep) || integrationStep < 0.001 || integrationStep > 0.02) return
    set({ experiment: { ...get().experiment, integrationStep } })
  },
  setPlaybackSpeed: (playbackSpeed) => {
    if (!Number.isFinite(playbackSpeed) || playbackSpeed <= 0) return
    set({ experiment: { ...get().experiment, playbackSpeed } })
  },
  setPlaying: (isPlaying) => set({ experiment: { ...get().experiment, isPlaying } }),
  setSimulationTime: (time) => {
    if (!Number.isFinite(time)) return
    set({ simulationTime: Math.max(0, Math.min(time, get().experiment.duration)) })
  },
  resetSimulationTime: () => set({
    simulationTime: 0,
    experiment: { ...get().experiment, isPlaying: false },
  }),
  resetLab: () => set(initialState()),
}))
