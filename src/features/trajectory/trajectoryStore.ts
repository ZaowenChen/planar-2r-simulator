import { create } from 'zustand'
import type { RobotParameters, Vector3 } from '../../robotics/types'
import {
  buildTrajectoryPreview,
  degreesToRadians,
  type PtpInstructionDraft,
  type TeachPoint,
  type TrajectoryPreview,
} from './trajectoryModel'

export type JogStepDegrees = 0.1 | 1 | 5

interface TrajectorySessionState {
  initialized: boolean
  currentQ: Vector3
  jogStepDegrees: JogStepDegrees
  teachPoints: readonly TeachPoint[]
  nextPointNumber: number
  draft: PtpInstructionDraft
  preview: TrajectoryPreview | null
  previewActive: boolean
  error?: string
  time: number
  isPlaying: boolean
  playbackSpeed: number
  sceneExpanded: boolean
  initialize: (q: Vector3) => void
  reset: (q: Vector3) => void
  setJogStep: (step: JogStepDegrees) => void
  jogJoint: (index: number, direction: -1 | 1, limits: RobotParameters['jointLimits']) => void
  recordTeachPoint: () => void
  updateTeachPoint: (id: string, limits: RobotParameters['jointLimits']) => void
  renameTeachPoint: (id: string, name: string) => void
  deleteTeachPoint: (id: string) => void
  setDraft: (
    patch: Partial<PtpInstructionDraft>,
    limits: RobotParameters['jointLimits'],
  ) => void
  generatePreview: (limits: RobotParameters['jointLimits']) => boolean
  setTime: (time: number) => void
  advanceTime: (time: number) => void
  setPlaying: (playing: boolean) => void
  setPlaybackSpeed: (speed: number) => void
  stepForward: () => void
  resetPlayback: () => void
  setSceneExpanded: (expanded: boolean) => void
}

function cloneVector(vector: Vector3): Vector3 {
  return [...vector] as Vector3
}

function initialSession(q: Vector3) {
  return {
    initialized: true,
    currentQ: cloneVector(q),
    jogStepDegrees: 1 as JogStepDegrees,
    teachPoints: [] as readonly TeachPoint[],
    nextPointNumber: 1,
    draft: {
      startPointId: '',
      endPointId: '',
      profile: 'quintic' as const,
      durationText: '5',
    },
    preview: null,
    previewActive: false,
    error: undefined as string | undefined,
    time: 0,
    isPlaying: false,
    playbackSpeed: 1,
    sceneExpanded: false,
  }
}

function refreshPreview(
  state: TrajectorySessionState,
  points: readonly TeachPoint[],
  draft: PtpInstructionDraft,
  limits: RobotParameters['jointLimits'],
): Partial<TrajectorySessionState> {
  if (state.preview === null) return {}
  const result = buildTrajectoryPreview(points, draft, limits)
  return result.ok
    ? { preview: result.preview, error: undefined, time: 0, isPlaying: false, previewActive: true }
    : { error: result.error, time: 0, isPlaying: false }
}

export const useTrajectoryStore = create<TrajectorySessionState>((set, get) => ({
  ...initialSession([0, 0, 0]),
  initialized: false,

  initialize: (q) => {
    if (get().initialized) return
    set(initialSession(q))
  },

  reset: (q) => set(initialSession(q)),

  setJogStep: (jogStepDegrees) => set({ jogStepDegrees }),

  jogJoint: (index, direction, limits) => {
    if (!Number.isInteger(index) || index < 0 || index > 2) return
    const state = get()
    const current = state.currentQ[index]
    const requested = current + direction * degreesToRadians(state.jogStepDegrees)
    const [minimum, maximum] = limits[index]
    const clamped = Math.min(maximum, Math.max(minimum, requested))
    const next = [...state.currentQ] as [number, number, number]
    next[index] = clamped
    set({
      currentQ: next,
      previewActive: false,
      isPlaying: false,
      error: requested === clamped
        ? undefined
        : `关节 ${index + 1} 已到达限位。`,
    })
  },

  recordTeachPoint: () => {
    const state = get()
    const id = `point-${state.nextPointNumber}`
    const point: TeachPoint = {
      id,
      name: `P${state.nextPointNumber}`,
      q: cloneVector(state.currentQ),
    }
    const draft = {
      ...state.draft,
      startPointId: state.draft.startPointId || id,
      endPointId: state.draft.startPointId && !state.draft.endPointId ? id : state.draft.endPointId,
    }
    set({
      teachPoints: [...state.teachPoints, point],
      nextPointNumber: state.nextPointNumber + 1,
      draft,
      error: undefined,
    })
  },

  updateTeachPoint: (id, limits) => {
    const state = get()
    const points = state.teachPoints.map((point) => (
      point.id === id ? { ...point, q: cloneVector(state.currentQ) } : point
    ))
    set({
      teachPoints: points,
      ...refreshPreview(state, points, state.draft, limits),
    })
  },

  renameTeachPoint: (id, rawName) => {
    const name = rawName.trim()
    if (name === '') return
    set((state) => ({
      teachPoints: state.teachPoints.map((point) => point.id === id ? { ...point, name } : point),
    }))
  },

  deleteTeachPoint: (id) => {
    const state = get()
    const referenced = state.draft.startPointId === id || state.draft.endPointId === id
    set({
      teachPoints: state.teachPoints.filter((point) => point.id !== id),
      draft: {
        ...state.draft,
        startPointId: state.draft.startPointId === id ? '' : state.draft.startPointId,
        endPointId: state.draft.endPointId === id ? '' : state.draft.endPointId,
      },
      preview: referenced ? null : state.preview,
      previewActive: referenced ? false : state.previewActive,
      error: referenced ? '被 PTP 指令引用的示教点已删除，请重新选择起点和终点。' : undefined,
      time: referenced ? 0 : state.time,
      isPlaying: false,
    })
  },

  setDraft: (patch, limits) => {
    const state = get()
    const draft = { ...state.draft, ...patch }
    set({
      draft,
      ...refreshPreview(state, state.teachPoints, draft, limits),
    })
  },

  generatePreview: (limits) => {
    const state = get()
    const result = buildTrajectoryPreview(state.teachPoints, state.draft, limits)
    if (!result.ok) {
      set({ error: result.error, isPlaying: false })
      return false
    }
    set({
      preview: result.preview,
      previewActive: true,
      error: undefined,
      time: 0,
      isPlaying: false,
    })
    return true
  },

  setTime: (time) => {
    const preview = get().preview
    if (preview === null || !Number.isFinite(time)) return
    set({
      time: Math.min(preview.instruction.duration, Math.max(0, time)),
      previewActive: true,
      isPlaying: false,
    })
  },

  advanceTime: (time) => {
    const preview = get().preview
    if (preview === null || !Number.isFinite(time)) return
    set({
      time: Math.min(preview.instruction.duration, Math.max(0, time)),
      previewActive: true,
    })
  },

  setPlaying: (isPlaying) => {
    if (get().preview === null) return
    set({ isPlaying, previewActive: true })
  },

  setPlaybackSpeed: (playbackSpeed) => {
    if (!Number.isFinite(playbackSpeed) || playbackSpeed <= 0) return
    set({ playbackSpeed })
  },

  stepForward: () => {
    const state = get()
    if (state.preview === null) return
    const interval = state.preview.instruction.duration / (state.preview.samples.length - 1)
    set({
      time: Math.min(state.preview.instruction.duration, state.time + interval),
      previewActive: true,
      isPlaying: false,
    })
  },

  resetPlayback: () => set((state) => ({
    time: 0,
    previewActive: state.preview !== null,
    isPlaying: false,
  })),

  setSceneExpanded: (sceneExpanded) => set({ sceneExpanded }),
}))
