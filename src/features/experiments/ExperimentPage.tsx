import { useEffect, useMemo, useState } from 'react'
import { BlockMath } from 'react-katex'
import { WorkbenchLayout } from '../../app/WorkbenchLayout'
import { FormulaCard } from '../../components/FormulaCard'
import { StatusBanner } from '../../components/StatusBanner'
import { downloadSimulationCsv } from '../../export/download'
import {
  simulateForwardDynamics,
  simulateInverseDynamics,
  type SimulationDiagnostic,
  type SimulationSample,
  type SimulationSamples,
} from '../../robotics/integration'
import type { TorqueProfile, TrajectoryConfig } from '../../robotics/trajectories'
import type { Vector3 } from '../../robotics/types'
import { RobotScene } from '../../scene/RobotScene'
import { useLabStore, type ExperimentMode } from '../../state/labStore'
import { SimulationControls, type ExperimentDraft } from './SimulationControls'
import { TimeSeriesCharts } from './TimeSeriesCharts'
import { nearestSimulationSample, useAnimationClock } from './useAnimationClock'

const INITIAL_DRAFT: ExperimentDraft = {
  duration: '5',
  stepSize: '0.005',
  inverseType: 'quintic',
  forwardType: 'constant',
  primary: ['0', '0', '0'],
  secondary: ['0.8', '0.5', '-0.4'],
  frequency: ['0.25', '0.3', '0.2'],
  phase: ['0', '0', '0'],
  eventTime: '2.5',
  pieces: '0:0,0,0;2.5:2,-1,0.5',
}

function parseVector(values: readonly string[]): Vector3 | undefined {
  const parsed = values.map(Number)
  return parsed.length === 3 && parsed.every(Number.isFinite)
    ? parsed as unknown as Vector3
    : undefined
}

function parsePieces(raw: string, duration: number): TorqueProfile | undefined {
  const segments = raw.split(';').map((piece) => {
    const [rawTime, rawValue] = piece.split(':')
    const value = parseVector(rawValue?.split(',') ?? [])
    const time = Number(rawTime)
    return value !== undefined && Number.isFinite(time) ? { time, value } : undefined
  })
  if (segments.some((segment) => segment === undefined)) return undefined
  return { type: 'piecewise-constant', duration, segments: segments as { time: number; value: Vector3 }[] }
}

function configFromDraft(
  mode: ExperimentMode,
  draft: ExperimentDraft,
): { duration: number; stepSize: number; trajectory?: TrajectoryConfig; profile?: TorqueProfile } | string {
  const duration = Number(draft.duration)
  const stepSize = Number(draft.stepSize)
  const primary = parseVector(draft.primary)
  const secondary = parseVector(draft.secondary)
  const frequency = parseVector(draft.frequency)
  const phase = parseVector(draft.phase)
  if (!Number.isFinite(duration) || duration <= 0 || duration > 30) return '持续时间必须在 0 到 30 s 之间。'
  if (!Number.isFinite(stepSize) || stepSize < 0.001 || stepSize > 0.02) return '积分步长必须在 0.001 到 0.02 s 之间。'
  if (primary === undefined || secondary === undefined || frequency === undefined || phase === undefined) return '所有向量输入必须是有限数值。'

  if (mode === 'inverse') {
    const trajectory: TrajectoryConfig = draft.inverseType === 'quintic'
      ? { type: 'quintic', q0: primary, qf: secondary, duration }
      : { type: 'sinusoidal', center: primary, amplitude: secondary, frequency, phase, duration }
    return { duration, stepSize, trajectory }
  }

  let profile: TorqueProfile | undefined
  switch (draft.forwardType) {
    case 'constant': profile = { type: 'constant', value: primary, duration }; break
    case 'step': {
      const stepTime = Number(draft.eventTime)
      if (!Number.isFinite(stepTime) || stepTime < 0 || stepTime > duration) return '阶跃时刻必须位于仿真时间范围内。'
      profile = { type: 'step', before: primary, after: secondary, stepTime, duration }
      break
    }
    case 'sine': profile = { type: 'sine', offset: primary, amplitude: secondary, frequency, phase, duration }; break
    case 'piecewise-constant': profile = parsePieces(draft.pieces, duration); break
  }
  if (profile === undefined) return '分段力矩格式无效。'
  return { duration, stepSize, profile }
}

function runSimulation(
  mode: ExperimentMode,
  draft: ExperimentDraft,
): { samples: SimulationSamples; duration: number; stepSize: number } | string {
  const config = configFromDraft(mode, draft)
  if (typeof config === 'string') return config
  const { parameters, jointState } = useLabStore.getState()
  try {
    const samples = mode === 'inverse'
      ? simulateInverseDynamics({
        trajectory: config.trajectory as TrajectoryConfig,
        duration: config.duration,
        stepSize: config.stepSize,
      }, parameters)
      : simulateForwardDynamics({
        initialState: { q: jointState.q, qd: jointState.qd },
        torqueProfile: config.profile as TorqueProfile,
        duration: config.duration,
        stepSize: config.stepSize,
      }, parameters)
    return { samples, duration: config.duration, stepSize: config.stepSize }
  } catch (error) {
    return error instanceof Error ? error.message : '仿真输入无效。'
  }
}

function vectorText(vector: readonly number[] | undefined, precision = 4): string {
  return vector === undefined ? '—' : `[${vector.map((value) => value.toFixed(precision)).join(', ')}]`
}

function diagnosticText(diagnostic: SimulationDiagnostic): string {
  return diagnostic.message
}

export function ExperimentPage() {
  const mode = useLabStore((state) => state.experiment.mode)
  const settings = useLabStore((state) => state.experiment)
  const simulationTime = useLabStore((state) => state.simulationTime)
  const calculation = useLabStore((state) => state.calculation)
  const parameters = useLabStore((state) => state.parameters)
  const jointState = useLabStore((state) => state.jointState)
  const [draft, setDraft] = useState<ExperimentDraft>(() => ({
    ...INITIAL_DRAFT,
    duration: String(settings.duration),
    stepSize: String(settings.integrationStep),
    primary: jointState.q.map(String) as unknown as [string, string, string],
  }))
  const [samples, setSamples] = useState<SimulationSamples>(() => {
    const result = runSimulation(mode, {
      ...INITIAL_DRAFT,
      duration: String(settings.duration),
      stepSize: String(settings.integrationStep),
      primary: jointState.q.map(String) as unknown as [string, string, string],
    })
    return typeof result === 'string' ? [] as unknown as SimulationSamples : result.samples
  })
  const [editorError, setEditorError] = useState<string>()
  useAnimationClock(samples)

  const currentSample = useMemo(
    () => nearestSimulationSample(samples, simulationTime),
    [samples, simulationTime],
  )

  useEffect(() => {
    if (samples.diagnostic !== undefined) useLabStore.getState().setPlaying(false)
  }, [samples])

  useEffect(() => {
    if (currentSample === undefined) return
    const store = useLabStore.getState()
    store.setJointVector(currentSample.q)
    currentSample.qd.forEach((value, index) => store.setJointVelocity(index, value))
    currentSample.qdd.forEach((value, index) => store.setJointAcceleration(index, value))
  }, [currentSample])

  const commitSimulation = (
    nextMode: ExperimentMode,
    result: { samples: SimulationSamples; duration: number; stepSize: number },
  ) => {
    const { experiment } = useLabStore.getState()
    useLabStore.setState({
      experiment: {
        ...experiment,
        mode: nextMode,
        duration: result.duration,
        integrationStep: result.stepSize,
        isPlaying: false,
      },
      simulationTime: 0,
    })
    setSamples(result.samples)
    setEditorError(undefined)
  }

  const generate = (nextMode = mode) => {
    const result = runSimulation(nextMode, draft)
    if (typeof result === 'string') {
      setEditorError(result)
      return
    }
    commitSimulation(nextMode, result)
  }

  const changeMode = (nextMode: ExperimentMode) => {
    if (nextMode === mode) return
    const result = runSimulation(nextMode, draft)
    if (typeof result === 'string') setEditorError(result)
    else commitSimulation(nextMode, result)
  }

  const step = () => {
    useLabStore.getState().setPlaying(false)
    const currentIndex = samples.findIndex((sample) => sample.time >= simulationTime)
    const next = samples[Math.min(samples.length - 1, Math.max(0, currentIndex + 1))]
    if (next !== undefined) useLabStore.getState().setSimulationTime(next.time)
  }
  const sceneCalculation = {
    forward: calculation.forward,
    jacobian: calculation.jacobian,
    jointState,
    torque: currentSample?.tau ?? calculation.dynamics.tau,
    gravity: parameters.gravity,
  }
  const sampleTime = currentSample?.time ?? 0
  const revision = String(calculation.revision)

  return (
    <WorkbenchLayout
      visual={<div data-revision={revision} data-sample-time={String(sampleTime)} data-testid="experiment-scene">
        <RobotScene calculation={sceneCalculation} initialOverlays={{ torque: true }} />
      </div>}
      controls={<SimulationControls
        draft={draft}
        error={editorError}
        mode={mode}
        onDraftChange={setDraft}
        onGenerate={() => generate()}
        onModeChange={changeMode}
      />}
      analysis={<div className="analysis-stack experiment-analysis" data-revision={revision} data-sample-time={String(sampleTime)} data-testid="experiment-formula">
        {samples.diagnostic !== undefined && <StatusBanner tone="error" title="仿真已自动暂停">{diagnosticText(samples.diagnostic)}</StatusBanner>}
        <output data-testid="sample-count">{samples.length}</output>
        <output data-testid="current-state">q = {vectorText(currentSample?.q)} rad；q̇ = {vectorText(currentSample?.qd)} rad/s</output>
        <output data-testid="current-torque">τ = {vectorText(currentSample?.tau)} N·m</output>
        <FormulaCard
          definition={<BlockMath math={mode === 'inverse'
            ? String.raw`\boldsymbol{\tau}=\mathbf M\ddot{\mathbf q}+\mathbf C\dot{\mathbf q}+\mathbf g+\boldsymbol{\tau}_f`
            : String.raw`\ddot{\mathbf q}=\mathbf M^{-1}(\boldsymbol{\tau}-\mathbf C\dot{\mathbf q}-\mathbf g-\boldsymbol{\tau}_f)`} />}
          result={<BlockMath math={String.raw`\mathbf q=${vectorText(currentSample?.q)},\quad\boldsymbol{\tau}=${vectorText(currentSample?.tau)}`} />}
          substitution={<BlockMath math={String.raw`t=${sampleTime.toFixed(4)}\;\mathrm{s}`} />}
          title={mode === 'inverse' ? '逆动力学实时结果' : '正动力学实时结果'}
        />
        <div className="playback-controls">
          <button disabled={samples.diagnostic !== undefined} onClick={() => useLabStore.getState().setPlaying(!settings.isPlaying)} type="button">{settings.isPlaying ? '暂停' : '播放'}</button>
          <button onClick={step} type="button">单步</button>
          <button onClick={() => useLabStore.getState().resetSimulationTime()} type="button">重置</button>
          <label>播放速度
            <select aria-label="播放速度" onChange={(event) => useLabStore.getState().setPlaybackSpeed(Number(event.target.value))} value={settings.playbackSpeed}>
              <option value="0.25">0.25×</option><option value="0.5">0.5×</option><option value="1">1×</option><option value="2">2×</option>
            </select>
          </label>
        </div>
        <label className="timeline-slider">仿真时间
          <input
            aria-label="仿真时间"
            max={samples.at(-1)?.time ?? settings.duration}
            min="0"
            onChange={(event) => {
              useLabStore.getState().setPlaying(false)
              const sample = nearestSimulationSample(samples, Number(event.target.value))
              useLabStore.getState().setSimulationTime(sample?.time ?? 0)
            }}
            step={settings.integrationStep}
            type="range"
            value={sampleTime}
          />
          <output data-testid="simulation-time">{sampleTime.toFixed(3)} s</output>
        </label>
        <button onClick={() => downloadSimulationCsv(samples)} type="button">导出 CSV</button>
      </div>}
      timeline={<TimeSeriesCharts samples={samples} time={sampleTime} onTimeChange={(time) => {
        useLabStore.getState().setPlaying(false)
        const sample = nearestSimulationSample(samples, time)
        if (sample !== undefined) useLabStore.getState().setSimulationTime(sample.time)
      }} />}
    />
  )
}
