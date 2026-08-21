import { beforeEach, describe, expect, it } from 'vitest'
import { DEFAULT_ROBOT_PARAMETERS } from '../../robotics/defaults'
import { useTrajectoryStore } from './trajectoryStore'

const limits = DEFAULT_ROBOT_PARAMETERS.jointLimits

beforeEach(() => useTrajectoryStore.getState().reset([0, 0, 0]))

describe('trajectory teaching store', () => {
  it('jogs one joint in degrees and clamps at its configured limit', () => {
    const store = useTrajectoryStore.getState()
    store.setJogStep(5)
    store.jogJoint(1, 1, limits)
    expect(useTrajectoryStore.getState().currentQ[1]).toBeCloseTo(5 * Math.PI / 180, 12)

    useTrajectoryStore.getState().reset([0, limits[1][1] - 0.01, 0])
    useTrajectoryStore.getState().setJogStep(5)
    useTrajectoryStore.getState().jogJoint(1, 1, limits)
    expect(useTrajectoryStore.getState().currentQ[1]).toBe(limits[1][1])
    expect(useTrajectoryStore.getState().error).toContain('限位')
  })

  it('records copied points and automatically selects the first PTP pair', () => {
    useTrajectoryStore.getState().recordTeachPoint()
    useTrajectoryStore.getState().jogJoint(0, 1, limits)
    useTrajectoryStore.getState().recordTeachPoint()
    const state = useTrajectoryStore.getState()

    expect(state.teachPoints.map((point) => point.name)).toEqual(['P1', 'P2'])
    expect(state.teachPoints[0].q).toEqual([0, 0, 0])
    expect(state.teachPoints[1].q[0]).toBeGreaterThan(0)
    expect(state.draft).toMatchObject({ startPointId: 'point-1', endPointId: 'point-2' })
  })

  it('generates a preview and refreshes it immediately after valid draft edits', () => {
    useTrajectoryStore.getState().recordTeachPoint()
    useTrajectoryStore.getState().jogJoint(2, 1, limits)
    useTrajectoryStore.getState().recordTeachPoint()
    expect(useTrajectoryStore.getState().generatePreview(limits)).toBe(true)
    const initialPeak = useTrajectoryStore.getState().preview?.metrics.peakVelocity[2] ?? 0

    useTrajectoryStore.getState().setDraft({ durationText: '10' }, limits)
    expect(useTrajectoryStore.getState().preview?.metrics.peakVelocity[2]).toBeCloseTo(initialPeak / 2, 12)
    expect(useTrajectoryStore.getState().time).toBe(0)
  })

  it('keeps the last valid preview when an edited duration is invalid', () => {
    useTrajectoryStore.getState().recordTeachPoint()
    useTrajectoryStore.getState().jogJoint(0, 1, limits)
    useTrajectoryStore.getState().recordTeachPoint()
    useTrajectoryStore.getState().generatePreview(limits)
    const preview = useTrajectoryStore.getState().preview

    useTrajectoryStore.getState().setDraft({ durationText: 'invalid' }, limits)
    expect(useTrajectoryStore.getState().preview).toBe(preview)
    expect(useTrajectoryStore.getState().error).toContain('持续时间')
  })

  it('deleting a referenced point clears the preview and its draft reference', () => {
    useTrajectoryStore.getState().recordTeachPoint()
    useTrajectoryStore.getState().jogJoint(0, 1, limits)
    useTrajectoryStore.getState().recordTeachPoint()
    useTrajectoryStore.getState().generatePreview(limits)

    useTrajectoryStore.getState().deleteTeachPoint('point-1')
    const state = useTrajectoryStore.getState()
    expect(state.preview).toBeNull()
    expect(state.draft.startPointId).toBe('')
    expect(state.error).toContain('已删除')
  })
})
