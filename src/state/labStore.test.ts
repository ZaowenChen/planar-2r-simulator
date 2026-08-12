import { beforeEach, describe, expect, it } from 'vitest'
import { DEFAULT_ROBOT_PARAMETERS } from '../robotics/defaults'
import { useLabStore } from './labStore'

describe('shared laboratory store', () => {
  beforeEach(() => {
    useLabStore.getState().resetLab()
  })

  it('recomputes one shared result graph when a joint changes', () => {
    const before = useLabStore.getState().calculation.revision

    useLabStore.getState().setJoint(1, 0.25)

    const after = useLabStore.getState()
    expect(after.calculation.revision).toBe(before + 1)
    expect(after.calculation.forward.q[1]).toBe(0.25)
    expect(after.calculation.dynamics.massMatrix).toBeDefined()
    expect(after.calculation.jacobian).toHaveLength(6)
  })

  it('publishes one atomic result graph when the whole joint vector changes', () => {
    const before = useLabStore.getState().calculation.revision
    const snapshots: Array<{ q: readonly number[]; revision: number }> = []
    const unsubscribe = useLabStore.subscribe((state) => {
      snapshots.push({ q: state.jointState.q, revision: state.calculation.revision })
    })

    useLabStore.getState().setJointVector([0.1, 0.2, -0.3])
    unsubscribe()

    expect(snapshots).toEqual([{
      q: [0.1, 0.2, -0.3],
      revision: before + 1,
    }])
  })

  it('does not publish or recalculate an unchanged joint value', () => {
    const before = useLabStore.getState()
    let publications = 0
    const unsubscribe = useLabStore.subscribe(() => {
      publications += 1
    })

    before.setJoint(1, before.jointState.q[1])
    unsubscribe()

    expect(useLabStore.getState().calculation.revision).toBe(before.calculation.revision)
    expect(publications).toBe(0)
  })

  it('preserves the last valid calculation while a raw parameter edit is invalid', () => {
    const before = useLabStore.getState()
    const revision = before.calculation.revision
    const endpoint = before.calculation.forward.endEffectorPosition

    before.setParameterField('geometry.l2', 'not-a-number')

    const invalid = useLabStore.getState()
    expect(invalid.rawParameters['geometry.l2']).toBe('not-a-number')
    expect(invalid.parameters.geometry.l2).toBe(DEFAULT_ROBOT_PARAMETERS.geometry.l2)
    expect(invalid.fieldIssues['geometry.l2']).toMatch(/有限数值/)
    expect(invalid.calculation.revision).toBe(revision)
    expect(invalid.calculation.forward.endEffectorPosition).toEqual(endpoint)

    invalid.setParameterField('geometry.l2', '2.25')

    const valid = useLabStore.getState()
    expect(valid.parameters.geometry.l2).toBe(2.25)
    expect(valid.fieldIssues['geometry.l2']).toBeUndefined()
    expect(valid.calculation.revision).toBe(revision + 1)
  })

  it('retains every raw-field issue and promotes nothing until all edits are valid', () => {
    const initial = useLabStore.getState()
    const revision = initial.calculation.revision

    initial.setParameterField('geometry.l2', 'not-a-number')
    useLabStore.getState().setParameterField('links.1.mass', '-3')

    const multiplyInvalid = useLabStore.getState()
    expect(multiplyInvalid.fieldIssues['geometry.l2']).toMatch(/有限数值/)
    expect(multiplyInvalid.fieldIssues['links.1.mass']).toBeDefined()
    expect(multiplyInvalid.parameters.geometry.l2).toBe(2)
    expect(multiplyInvalid.parameters.links[1].mass).toBe(3)
    expect(multiplyInvalid.calculation.revision).toBe(revision)

    multiplyInvalid.setParameterField('links.1.mass', '5')

    const stillInvalid = useLabStore.getState()
    expect(stillInvalid.fieldIssues['geometry.l2']).toMatch(/有限数值/)
    expect(stillInvalid.fieldIssues['links.1.mass']).toBeUndefined()
    expect(stillInvalid.parameters.links[1].mass).toBe(3)
    expect(stillInvalid.calculation.revision).toBe(revision)

    stillInvalid.setParameterField('geometry.l2', '2.25')

    const promoted = useLabStore.getState()
    expect(promoted.fieldIssues).toEqual({})
    expect(promoted.parameters.geometry.l2).toBe(2.25)
    expect(promoted.parameters.links[1].mass).toBe(5)
    expect(promoted.calculation.revision).toBe(revision + 1)
  })

  it('atomically promotes coupled raw edits that are valid only together', () => {
    const initial = useLabStore.getState()
    const revision = initial.calculation.revision

    initial.setParameterField('geometry.l2', '0.5')

    const invalidLength = useLabStore.getState()
    expect(invalidLength.fieldIssues['links.1.centerOfMass']).toBeDefined()
    expect(invalidLength.parameters.geometry.l2).toBe(2)
    expect(invalidLength.calculation.revision).toBe(revision)

    invalidLength.setParameterField('links.1.centerOfMass.0', '-0.4')

    const promoted = useLabStore.getState()
    expect(promoted.fieldIssues).toEqual({})
    expect(promoted.parameters.geometry.l2).toBe(0.5)
    expect(promoted.parameters.links[1].centerOfMass[0]).toBe(-0.4)
    expect(promoted.calculation.revision).toBe(revision + 1)
  })

  it('owns navigation, experiment settings, and the shared simulation clock', () => {
    const store = useLabStore.getState()

    store.setActiveModule('experiments')
    store.setExperimentMode('forward')
    store.setExperimentDuration(8)
    store.setSimulationTime(2.5)
    store.setPlaying(true)

    const after = useLabStore.getState()
    expect(after.activeModule).toBe('experiments')
    expect(after.experiment).toMatchObject({
      mode: 'forward',
      duration: 8,
      isPlaying: true,
    })
    expect(after.simulationTime).toBe(2.5)
  })
})
