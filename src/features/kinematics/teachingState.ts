import type { InverseKinematicsSolution } from '../../robotics/kinematics'

export type KinematicsConfigurationId =
  | 'conventional:elbow-down'
  | 'conventional:elbow-up'
  | 'folded:elbow-down'
  | 'folded:elbow-up'

export type KinematicsFrameMode = 'hidden' | 'current' | 'chain' | 'all'
export type KinematicsCameraPreset = 'overview' | 'top' | 'work-plane' | 'tool'
export type DhOperation = 'rz' | 'tz' | 'tx' | 'rx'
export type KinematicsSymbol =
  | 'theta1'
  | 'theta2'
  | 'theta3'
  | 'r'
  | 'h'
  | 's'
  | 'l2'
  | 'l3'
  | 'gamma'
  | 'delta'
  | 'beta'

export type KinematicsMobilePane = 'scene' | 'analysis' | 'controls'
export type KinematicsMode = 'forward' | 'inverse' | 'jacobian'

export const KINEMATICS_MODE_STEPS: Record<KinematicsMode, readonly number[]> = {
  forward: [0, 1, 2, 3, 4, 5],
  inverse: [6, 8, 12, 13, 14],
  jacobian: [16, 17, 18, 19],
}

export interface KinematicsTeachingState {
  mode: KinematicsMode
  stepIndex: number
  lastStepByMode: Record<KinematicsMode, number>
  activeConfigurationId: KinematicsConfigurationId
  frameMode: KinematicsFrameMode
  cameraPreset: KinematicsCameraPreset
  followStepCamera: boolean
  cameraResetRevision: number
  selectedDhRow: 0 | 1 | 2
  dhOperation: DhOperation
  symbolFocus: KinematicsSymbol | null
  selectedJacobianColumn: 0 | 1 | 2
  mobilePane: KinematicsMobilePane
}

export type KinematicsTeachingAction =
  | { type: 'mode'; value: KinematicsMode }
  | { type: 'step'; value: number }
  | { type: 'configuration'; value: KinematicsConfigurationId }
  | { type: 'frame-mode'; value: KinematicsFrameMode }
  | { type: 'camera-preset'; value: KinematicsCameraPreset }
  | { type: 'follow-camera'; value: boolean }
  | { type: 'reset-camera' }
  | { type: 'dh-row'; value: 0 | 1 | 2 }
  | { type: 'dh-operation'; value: DhOperation }
  | { type: 'symbol-focus'; value: KinematicsSymbol | null }
  | { type: 'jacobian-column'; value: 0 | 1 | 2 }
  | { type: 'mobile-pane'; value: KinematicsMobilePane }

const STEP_CAMERA_PRESETS: readonly KinematicsCameraPreset[] = [
  'overview',
  'overview',
  'overview',
  'overview',
  'overview',
  'tool',
  'work-plane',
  'work-plane',
  'work-plane',
  'work-plane',
  'work-plane',
  'work-plane',
  'work-plane',
  'overview',
  'overview',
  'tool',
  'overview',
  'overview',
  'tool',
  'overview',
] as const

const STEP_FRAME_MODES: readonly KinematicsFrameMode[] = [
  'current',
  'current',
  'current',
  'current',
  'current',
  'current',
  'current',
  'current',
  'current',
  'current',
  'current',
  'current',
  'current',
  'current',
  'current',
  'current',
  'current',
  'current',
  'current',
  'current',
] as const

const STEP_SYMBOL_FOCUS: readonly (KinematicsSymbol | null)[] = [
  null,
  null,
  null,
  null,
  null,
  'beta',
  'r',
  's',
  's',
  'theta3',
  'gamma',
  'delta',
  'theta2',
  'r',
  null,
  'beta',
  null,
  null,
  null,
  null,
] as const

export const INITIAL_KINEMATICS_TEACHING_STATE: KinematicsTeachingState = {
  mode: 'forward',
  stepIndex: 0,
  lastStepByMode: {
    forward: 0,
    inverse: 6,
    jacobian: 16,
  },
  activeConfigurationId: 'conventional:elbow-down',
  frameMode: STEP_FRAME_MODES[0],
  cameraPreset: STEP_CAMERA_PRESETS[0],
  followStepCamera: true,
  cameraResetRevision: 0,
  selectedDhRow: 0,
  dhOperation: 'rz',
  symbolFocus: STEP_SYMBOL_FOCUS[0],
  selectedJacobianColumn: 0,
  mobilePane: 'analysis',
}

export function configurationId(
  solution: Pick<InverseKinematicsSolution, 'branch' | 'radialFamily'>,
): KinematicsConfigurationId {
  return `${solution.radialFamily}:${solution.branch}` as KinematicsConfigurationId
}

export function configurationBranch(
  id: KinematicsConfigurationId,
): InverseKinematicsSolution['branch'] {
  return id.endsWith('elbow-down') ? 'elbow-down' : 'elbow-up'
}

export function configurationRadialFamily(
  id: KinematicsConfigurationId,
): InverseKinematicsSolution['radialFamily'] {
  return id.startsWith('conventional') ? 'conventional' : 'folded'
}

export function modeForStep(stepIndex: number): KinematicsMode {
  if (stepIndex >= 6 && stepIndex <= 15) return 'inverse'
  if (stepIndex >= 16) return 'jacobian'
  return 'forward'
}

export function kinematicsTeachingReducer(
  state: KinematicsTeachingState,
  action: KinematicsTeachingAction,
): KinematicsTeachingState {
  switch (action.type) {
    case 'mode': {
      const stepIndex = state.lastStepByMode[action.value]
      return {
        ...state,
        mode: action.value,
        stepIndex,
        cameraPreset: state.followStepCamera
          ? STEP_CAMERA_PRESETS[stepIndex]
          : state.cameraPreset,
        frameMode: STEP_FRAME_MODES[stepIndex],
        symbolFocus: STEP_SYMBOL_FOCUS[stepIndex],
      }
    }
    case 'step': {
      const stepIndex = Math.max(0, Math.min(STEP_CAMERA_PRESETS.length - 1, action.value))
      const mode = modeForStep(stepIndex)
      return {
        ...state,
        mode,
        stepIndex,
        lastStepByMode: {
          ...state.lastStepByMode,
          [mode]: stepIndex,
        },
        cameraPreset: state.followStepCamera
          ? STEP_CAMERA_PRESETS[stepIndex]
          : state.cameraPreset,
        frameMode: STEP_FRAME_MODES[stepIndex],
        symbolFocus: STEP_SYMBOL_FOCUS[stepIndex],
      }
    }
    case 'configuration':
      return { ...state, activeConfigurationId: action.value }
    case 'frame-mode':
      return { ...state, frameMode: action.value }
    case 'camera-preset':
      return { ...state, cameraPreset: action.value }
    case 'follow-camera':
      return { ...state, followStepCamera: action.value }
    case 'reset-camera':
      return {
        ...state,
        cameraPreset: STEP_CAMERA_PRESETS[state.stepIndex],
        followStepCamera: true,
        cameraResetRevision: state.cameraResetRevision + 1,
      }
    case 'dh-row':
      return { ...state, selectedDhRow: action.value, dhOperation: 'rz' }
    case 'dh-operation':
      return { ...state, dhOperation: action.value }
    case 'symbol-focus':
      return { ...state, symbolFocus: action.value }
    case 'jacobian-column':
      return { ...state, selectedJacobianColumn: action.value }
    case 'mobile-pane':
      return { ...state, mobilePane: action.value }
  }
}
