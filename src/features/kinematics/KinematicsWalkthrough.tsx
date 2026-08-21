import { useEffect, useMemo, useRef, useState } from 'react'
import { BlockMath } from 'react-katex'
import { MatrixTable } from '../../components/MatrixTable'
import { StatusBanner } from '../../components/StatusBanner'
import type { InverseKinematicsSolution } from '../../robotics/kinematics'
import type { SingularityMetrics } from '../../robotics/jacobian'
import type { RobotParameters, Vector3 } from '../../robotics/types'
import { DhTable, type DhParameter } from '../model/DhTable'
import { buildKinematicsDerivation } from './derivationModel'
import { DhTransformPlayer } from './DhTransformPlayer'
import { HomogeneousTransformCard } from './HomogeneousTransformCard'
import { PlanarGeometryDiagram } from './PlanarGeometryDiagram'
import type { GeometryDiagramFocus } from './PlanarGeometryDiagram'
import { metresToMillimetres } from './presentation'
import { VerificationSummary } from './VerificationSummary'
import {
  configurationBranch,
  configurationId,
  KINEMATICS_MODE_STEPS,
  type DhOperation,
  type KinematicsCameraPreset,
  type KinematicsConfigurationId,
  type KinematicsMode,
  type KinematicsSymbol,
} from './teachingState'

interface KinematicsWalkthroughProps {
  q: Vector3
  qd: Vector3
  parameters: RobotParameters
  target: Vector3
  singularity: SingularityMetrics
  revision: number
  mode: KinematicsMode
  stepIndex: number
  onStepChange: (stepIndex: number) => void
  activeConfigurationId: KinematicsConfigurationId
  onConfigurationChange: (configuration: KinematicsConfigurationId) => void
  symbolFocus: KinematicsSymbol | null
  onSymbolFocus: (symbol: KinematicsSymbol | null) => void
  selectedDhRow: 0 | 1 | 2
  dhOperation: DhOperation
  onDhRowChange: (row: 0 | 1 | 2) => void
  onDhOperationChange: (operation: DhOperation) => void
  onCameraPresetChange: (preset: KinematicsCameraPreset) => void
  selectedJacobianColumn: 0 | 1 | 2
  onJacobianColumnChange: (column: 0 | 1 | 2) => void
  canApplyInverse: boolean
  onApplyInverse: () => void
}

const STEPS = [
  { method: '正运动学 · D–H 矩阵法', title: '先识别机构与关节轴' },
  { method: '正运动学 · D–H 矩阵法', title: '把机构写成标准 D–H 参数' },
  { method: '正运动学 · D–H 矩阵法', title: '写出三个关节齐次变换' },
  { method: '正运动学 · D–H 矩阵法', title: '连乘得到基座到末端的变换' },
  { method: '正运动学 · D–H 矩阵法', title: '从变换矩阵提取末端位置' },
  { method: '正运动学 · D–H 矩阵法', title: '提取末端姿态' },
  { method: '逆运动学 · 解析几何法', title: '设置目标' },
  { method: '逆运动学 · 解析几何法', title: '用直角三角形计算肩部到目标距离' },
  { method: '逆运动学 · 解析几何法', title: '判断可达' },
  { method: '逆运动学 · 解析几何法', title: '用余弦定理求肘角' },
  { method: '逆运动学 · 解析几何法', title: '求肩部指向目标的方向角' },
  { method: '逆运动学 · 解析几何法', title: '求连杆三角形补偿角' },
  { method: '逆运动学 · 解析几何法', title: '求关节角' },
  { method: '逆运动学 · 构型比较', title: '选择构型' },
  { method: '验证 · 正运动学回代', title: 'FK 回代验证' },
  { method: '验证 · 正运动学回代', title: '比较每组位置逆解的实际姿态' },
  { method: '微分运动学 · 几何雅可比', title: '读取当前关节运动状态' },
  { method: '微分运动学 · 几何雅可比', title: '用关节轴逐列构造雅可比' },
  { method: '微分运动学 · 速度映射', title: '计算末端线速度与角速度' },
  { method: '微分运动学 · 位置雅可比', title: '判断位置奇异性' },
] as const

const MODE_STEP_LABELS: Record<KinematicsMode, readonly string[]> = {
  forward: ['机构与轴', 'D–H 建系', '相邻变换', '变换连乘', '末端位置', '末端姿态'],
  inverse: ['设置目标', '判断可达', '求关节角', '选择构型', 'FK 回代验证'],
  jacobian: ['运动状态', '逐列构造', '末端速度', '位置奇异性'],
}

const STEP_SYMBOLS: readonly (readonly KinematicsSymbol[])[] = [
  [],
  [],
  ['theta1', 'theta2', 'theta3'],
  [],
  ['theta1'],
  ['theta1', 'beta'],
  ['theta1', 'r', 'h'],
  ['r', 'h', 's'],
  ['s', 'l2', 'l3'],
  ['theta3', 'l2', 'l3'],
  ['gamma'],
  ['delta', 'l2', 'l3'],
  ['theta2', 'theta3', 'gamma', 'delta'],
  ['r'],
  [],
  ['beta'],
  ['theta1', 'theta2', 'theta3'],
  [],
  [],
  [],
] as const

const SYMBOL_LABELS: Record<KinematicsSymbol, string> = {
  theta1: 'θ₁',
  theta2: 'θ₂',
  theta3: 'θ₃',
  r: 'r',
  h: 'h',
  s: 's',
  l2: 'l₂',
  l3: 'l₃',
  gamma: 'γ',
  delta: 'δ',
  beta: 'β',
}

function format(value: number, precision = 4): string {
  if (!Number.isFinite(value)) return value > 0 ? '\\infty' : '—'
  return Math.abs(value) < 10 ** (-precision) ? '0' : value.toFixed(precision)
}

function vectorLatex(vector: readonly number[], precision = 4): string {
  return String.raw`\begin{bmatrix}${vector.map((value) => format(value, precision)).join('\\\\')}\end{bmatrix}`
}

function matrixLatex(matrix: readonly (readonly number[])[], precision = 4): string {
  return String.raw`\begin{bmatrix}${matrix.map((row) => row.map((value) => format(value, precision)).join('&')).join('\\\\')}\end{bmatrix}`
}

function configurationLabel(solution: InverseKinematicsSolution): string {
  return solution.branch === 'elbow-down' ? '肘下构型' : '肘上构型'
}

function radialFamilyLabel(solution: InverseKinematicsSolution): string {
  return solution.radialFamily === 'conventional' ? '常规径向族' : '折叠径向族'
}

function StepSection({ title, children }: React.PropsWithChildren<{ title: string }>) {
  return (
    <section className="walkthrough-section">
      <h4>{title}</h4>
      <div>{children}</div>
    </section>
  )
}

function IkStageField({
  children,
  label,
  tone = 'neutral',
}: React.PropsWithChildren<{
  label: '说明' | '公式' | '数值代入' | '结果'
  tone?: 'neutral' | 'result'
}>) {
  return (
    <section className={`ik-stage-field ik-stage-field--${tone}`}>
      <p className="ik-stage-field__label">{label}</p>
      <div>{children}</div>
    </section>
  )
}

function IkCalculationCard({
  formula,
  index,
  result,
  substitution,
  title,
  value,
}: {
  formula: React.ReactNode
  index: number
  result?: React.ReactNode
  substitution: React.ReactNode
  title: string
  value?: string
}) {
  return (
    <section className="ik-calculation-card" data-testid={`ik-calculation-card-${index}`}>
      <header>
        <span>{String(index).padStart(2, '0')}</span>
        <h4>{title}</h4>
        {value !== undefined && <output>{value}</output>}
      </header>
      <div>
        <p>公式</p>
        {formula}
      </div>
      <div>
        <p>数值代入</p>
        {substitution}
      </div>
      {result !== undefined && (
        <div className="ik-calculation-card__result">
          <p>结果</p>
          {result}
        </div>
      )}
    </section>
  )
}

export function KinematicsWalkthrough({
  q,
  qd,
  parameters,
  target,
  singularity,
  revision,
  mode,
  stepIndex,
  onStepChange,
  activeConfigurationId,
  onConfigurationChange,
  symbolFocus,
  onSymbolFocus,
  selectedDhRow,
  dhOperation,
  onDhRowChange,
  onDhOperationChange,
  onCameraPresetChange,
  selectedJacobianColumn,
  onJacobianColumnChange,
  canApplyInverse,
  onApplyInverse,
}: KinematicsWalkthroughProps) {
  const [calculationExpanded, setCalculationExpanded] = useState(false)
  const calculationDisclosureRef = useRef<HTMLButtonElement>(null)
  const walkthroughBodyRef = useRef<HTMLDivElement>(null)
  const model = useMemo(
    () => buildKinematicsDerivation(q, parameters, target, qd),
    [parameters, q, qd, target],
  )

  useEffect(() => {
    if (mode !== 'inverse' || stepIndex !== 12) setCalculationExpanded(false)
    if (walkthroughBodyRef.current !== null) walkthroughBodyRef.current.scrollTop = 0
  }, [mode, stepIndex])

  useEffect(() => {
    const body = walkthroughBodyRef.current
    const disclosure = calculationDisclosureRef.current
    if (!calculationExpanded || body === null || disclosure === null) return
    const bodyRect = body.getBoundingClientRect()
    const disclosureRect = disclosure.getBoundingClientRect()
    body.scrollTop = Math.max(
      0,
      body.scrollTop + disclosureRect.top - bodyRect.top - 12,
    )
  }, [calculationExpanded])

  const d1Mm = metresToMillimetres(parameters.geometry.d1)
  const l2Mm = metresToMillimetres(parameters.geometry.l2)
  const l3Mm = metresToMillimetres(parameters.geometry.l3)
  const inverseStatus = model.inverse.result.status
  const inverseStatusText = inverseStatus === 'reachable'
    ? '目标可达，可以构成常规径向的肘上、肘下两组解析解。'
    : inverseStatus === 'axis-singular'
      ? '目标位于基座轴线上，基座角 θ₁ 不唯一。'
      : inverseStatus === 'joint-limit'
        ? '几何上可达，但当前关节限位过滤了可应用的候选构型。'
        : '目标距离超出连杆能够构成的三角形范围。'
  const minimumSingularValue = singularity.minimumSingularValue * 1000 * Math.PI / 180
  const displayJacobianScale = 1000 * Math.PI / 180
  const displayManipulability = singularity.yoshikawaManipulability
    * displayJacobianScale ** 3
  const positionSingularityTitle = !Number.isFinite(singularity.conditionNumber)
    ? '处于位置奇异位形'
    : singularity.isSingular ? '接近位置奇异位形' : '远离位置奇异位形'
  const positionSingularityTone = !Number.isFinite(singularity.conditionNumber)
    ? 'error' as const
    : singularity.isSingular ? 'warning' as const : 'success' as const
  const geometryDiagram = (focus: GeometryDiagramFocus) => (
    <PlanarGeometryDiagram
      activeBranch={configurationBranch(activeConfigurationId)}
      branches={model.inverse.conventionalBranches}
      focus={focus}
      onBranchChange={(branch) => onConfigurationChange(`conventional:${branch}`)}
      radialMm={model.inverse.radialMm}
      symbolFocus={symbolFocus}
      verticalMm={model.inverse.verticalMm}
    />
  )
  const dhParameter: Record<DhOperation, DhParameter> = {
    rz: 'theta',
    tz: 'd',
    tx: 'a',
    rx: 'alpha',
  }
  const parameterOperation: Record<DhParameter, DhOperation> = {
    theta: 'rz',
    d: 'tz',
    a: 'tx',
    alpha: 'rx',
  }
  const selectedDisplayTransform = [
    model.displayTransforms.t01,
    model.displayTransforms.t12,
    model.displayTransforms.t23,
  ][selectedDhRow]
  const selectedDhSubstitution = [
    String.raw`\mathbf A_1=R_z(\theta_1)T_z(d_1)T_x(0)R_x(90^\circ)`,
    String.raw`\mathbf A_2=R_z(\theta_2)T_z(0)T_x(l_2)R_x(0^\circ)`,
    String.raw`\mathbf A_3=R_z(\theta_3)T_z(0)T_x(l_3)R_x(0^\circ)`,
  ][selectedDhRow]
  const selectedDhSimplification = [
    String.raw`\boxed{\mathbf A_1=R_z(\theta_1)T_z(d_1)R_x(90^\circ)}`,
    String.raw`\boxed{\mathbf A_2=R_z(\theta_2)T_x(l_2)}`,
    String.raw`\boxed{\mathbf A_3=R_z(\theta_3)T_x(l_3)}`,
  ][selectedDhRow]

  const selectedElbowBranch = configurationBranch(activeConfigurationId)
  const selectedConventionalDetail = model.inverse.conventionalBranches.find((detail) => (
    detail.solution.branch === selectedElbowBranch
  )) ?? model.inverse.conventionalBranches[0]
  const alternateConventionalDetail = model.inverse.conventionalBranches.find((detail) => (
    detail.solution.branch !== selectedConventionalDetail?.solution.branch
  ))
  const selectedCandidateDetail = model.inverse.candidateDetails.find((detail) => (
    configurationId(detail.solution) === activeConfigurationId
  )) ?? selectedConventionalDetail
  const selectedSolutionDetail = model.inverse.solutionDetails.find((detail) => (
    configurationId(detail.solution) === activeConfigurationId
  ))
  const isApplicableSolution = (id: KinematicsConfigurationId) => (
    model.inverse.solutionDetails.some((detail) => configurationId(detail.solution) === id)
  )
  const inverseStatusTone = inverseStatus === 'unreachable'
    ? 'error' as const
    : inverseStatus === 'joint-limit' || inverseStatus === 'axis-singular'
      ? 'warning' as const
      : 'success' as const
  const inverseStatusTitle = inverseStatus === 'unreachable'
    ? '目标不可达'
    : inverseStatus === 'joint-limit'
      ? '几何可达，但超出关节限位'
      : inverseStatus === 'axis-singular'
        ? '目标可达，但基座轴奇异'
        : '目标可达'
  const selectedPositionError = selectedCandidateDetail?.positionErrorMm
  const verificationTone = selectedCandidateDetail === undefined
    ? inverseStatusTone
    : selectedSolutionDetail === undefined || inverseStatus === 'axis-singular'
      ? 'warning' as const
      : 'success' as const
  const verificationTitle = selectedCandidateDetail === undefined
    ? inverseStatusTitle
    : selectedSolutionDetail === undefined
      ? inverseStatus === 'joint-limit'
        ? '几何回代通过，但受关节限位阻止'
        : '几何回代通过，但当前候选不可应用'
      : inverseStatus === 'axis-singular'
        ? 'FK 回代完成，基座角采用约定值'
        : 'FK 回代完成'

  const inverseStageContent = [
    <>
      <IkStageField label="说明">
        <p>先绕基座轴确定方位，再把三维目标化到肩—肘—目标所在的二维工作平面。</p>
      </IkStageField>
      <IkStageField label="公式">
        <BlockMath math={String.raw`r=\sqrt{x^2+y^2},\qquad h=z-d_1,\qquad\theta_1=\operatorname{atan2}(y,x)`} />
      </IkStageField>
      <IkStageField label="数值代入">
        <BlockMath math={String.raw`(x,y,z)=${vectorLatex(model.inverse.targetMm, 1)}\ \mathrm{mm}`} />
        <BlockMath math={String.raw`r=${format(model.inverse.radialMm, 1)}\ \mathrm{mm},\quad h=${format(model.inverse.verticalMm, 1)}\ \mathrm{mm}`} />
      </IkStageField>
      <IkStageField label="结果" tone="result">
        <strong>θ₁ = {format(model.inverse.baseAngleDegrees, 2)}°</strong>
        <p>工作平面目标坐标为（r, h）=（{format(model.inverse.radialMm, 1)}, {format(model.inverse.verticalMm, 1)}）mm。</p>
      </IkStageField>
      <details className="ik-advanced">
        <summary>Advanced · 查看二维工作平面</summary>
        {geometryDiagram('projection')}
      </details>
    </>,
    <>
      <IkStageField label="说明">
        <p>先求肩部到目标的距离 s，再检查两根连杆是否能组成三角形；关节限位在几何可达之后单独判断。</p>
      </IkStageField>
      <IkStageField label="公式">
        <BlockMath math={String.raw`s=\sqrt{r^2+h^2},\qquad |l_2-l_3|\le s\le l_2+l_3`} />
      </IkStageField>
      <IkStageField label="数值代入">
        <BlockMath math={String.raw`s=\sqrt{${format(model.inverse.radialMm, 1)}^2+${format(model.inverse.verticalMm, 1)}^2}=${format(model.inverse.pythagoreanDistanceMm, 1)}\ \mathrm{mm}`} />
        <BlockMath math={String.raw`${format(model.inverse.reachability.minimumMm, 1)}\le ${format(model.inverse.pythagoreanDistanceMm, 1)}\le ${format(model.inverse.reachability.maximumMm, 1)}\ \mathrm{mm}`} />
      </IkStageField>
      <IkStageField label="结果" tone="result">
        <StatusBanner tone={inverseStatusTone} title={inverseStatusTitle}>
          {inverseStatusText}
        </StatusBanner>
      </IkStageField>
      <details className="ik-advanced">
        <summary>Advanced · 查看连杆三角形</summary>
        {geometryDiagram('distance')}
      </details>
    </>,
    <>
      <IkStageField label="说明">
        <p>按 θ₃ → γ → δ → θ₂ 的顺序计算。默认只显示核心链路，需要时再展开每个中间量。</p>
      </IkStageField>
      <IkStageField label="公式">
        <BlockMath math={String.raw`\theta_3=\pm\arccos(D),\qquad\theta_2=\gamma-\operatorname{atan2}(l_3\sin\theta_3,l_2+l_3\cos\theta_3)`} />
      </IkStageField>
      <IkStageField label="数值代入">
        <BlockMath math={String.raw`D=\frac{${format(model.inverse.pythagoreanDistanceMm, 1)}^2-${format(l2Mm, 1)}^2-${format(l3Mm, 1)}^2}{2\times${format(l2Mm, 1)}\times${format(l3Mm, 1)}}=${format(model.inverse.cosineElbow, 4)}`} />
      </IkStageField>
      <IkStageField label="结果" tone="result">
        {model.inverse.conventionalBranches.length > 0 ? (
          <div className="ik-compact-results">
            {model.inverse.conventionalBranches.map((detail) => (
              <span key={detail.solution.branch}>
                <strong>{configurationLabel(detail.solution)}</strong>
                θ₂ = {format(detail.qDegrees[1], 2)}° · θ₃ = {format(detail.qDegrees[2], 2)}°
              </span>
            ))}
          </div>
        ) : (
          <p>当前 D 不在 [−1, 1] 内，因此不存在实数关节角解。</p>
        )}
      </IkStageField>
      <button
        aria-controls="ik-calculation-process"
        aria-expanded={calculationExpanded}
        className="ik-calculation-disclosure"
        onClick={() => setCalculationExpanded((expanded) => !expanded)}
        ref={calculationDisclosureRef}
        type="button"
      >
        <span>
          <strong>{calculationExpanded ? '收起计算过程' : '查看完整计算过程'}</strong>
          <small>{calculationExpanded ? '正在查看 · 4/4' : '4 个子计算'}</small>
        </span>
        <span aria-hidden="true">{calculationExpanded ? '⌃' : '⌄'}</span>
      </button>
      {calculationExpanded && (
        <section aria-label="完整计算过程" className="ik-calculation-process" id="ik-calculation-process">
          <header>
            <h4>计算过程</h4>
            <span>{selectedConventionalDetail === undefined ? '当前目标无实数解' : `${configurationLabel(selectedConventionalDetail.solution)}候选`}</span>
          </header>
          <IkCalculationCard
            formula={<BlockMath math={String.raw`D=\frac{s^2-l_2^2-l_3^2}{2l_2l_3},\qquad\theta_3=\pm\arccos(D)`} />}
            index={1}
            result={selectedConventionalDetail === undefined
              ? <p>|D| &gt; 1，θ₃ 没有实数结果。</p>
              : <BlockMath math={String.raw`\theta_3=${format(selectedConventionalDetail.qDegrees[2], 2)}^\circ,\quad\theta_3'=${format(alternateConventionalDetail?.qDegrees[2] ?? Number.NaN, 2)}^\circ`} />}
            substitution={<BlockMath math={String.raw`D=\frac{${format(model.inverse.pythagoreanDistanceMm, 1)}^2-${format(l2Mm, 1)}^2-${format(l3Mm, 1)}^2}{2\times${format(l2Mm, 1)}\times${format(l3Mm, 1)}}=${format(model.inverse.cosineElbow, 4)}`} />}
            title="肘角 θ₃"
            value={selectedConventionalDetail === undefined ? '无实数解' : `±${format(Math.abs(selectedConventionalDetail.qDegrees[2]), 2)}°`}
          />
          <IkCalculationCard
            formula={<BlockMath math={String.raw`\gamma=\operatorname{atan2}(h,r)`} />}
            index={2}
            result={<p>{selectedConventionalDetail === undefined ? '等待有效肘角。' : `γ = ${format(selectedConventionalDetail.targetDirectionDegrees, 2)}°`}</p>}
            substitution={<BlockMath math={String.raw`\gamma=\operatorname{atan2}(${format(model.inverse.verticalMm, 1)},${format(model.inverse.radialMm, 1)})${selectedConventionalDetail === undefined ? '' : `=${format(selectedConventionalDetail.targetDirectionDegrees, 2)}^\circ`}`} />}
            title="方向角 γ"
            value={selectedConventionalDetail === undefined ? '—' : `${format(selectedConventionalDetail.targetDirectionDegrees, 2)}°`}
          />
          <IkCalculationCard
            formula={<BlockMath math={String.raw`k_1=l_2+l_3\cos\theta_3,\quad k_2=l_3\sin\theta_3,\quad\delta=\operatorname{atan2}(k_2,k_1)`} />}
            index={3}
            result={<p>{selectedConventionalDetail === undefined ? '等待有效肘角。' : `δ = ${format(selectedConventionalDetail.triangleCorrectionDegrees, 2)}°`}</p>}
            substitution={selectedConventionalDetail === undefined
              ? <p>当前没有可代入的 θ₃。</p>
              : <BlockMath math={String.raw`k_1=${format(selectedConventionalDetail.triangleProjectionMm, 1)}\ \mathrm{mm},\quad k_2=${format(selectedConventionalDetail.triangleHeightMm, 1)}\ \mathrm{mm}`} />}
            title="补偿角 δ"
            value={selectedConventionalDetail === undefined ? '—' : `${format(selectedConventionalDetail.triangleCorrectionDegrees, 2)}°`}
          />
          <IkCalculationCard
            formula={<BlockMath math={String.raw`\theta_2=\gamma-\delta`} />}
            index={4}
            result={selectedConventionalDetail === undefined
              ? <p>当前目标没有可用的肩角结果。</p>
              : (
                <>
                  <BlockMath math={String.raw`\mathbf q_{\mathrm{IK}}=${vectorLatex(selectedConventionalDetail.qDegrees, 2)}\ ^\circ`} />
                  {alternateConventionalDetail !== undefined && (
                    <p>另一支候选：θ₂ = {format(alternateConventionalDetail.qDegrees[1], 2)}°，θ₃ = {format(alternateConventionalDetail.qDegrees[2], 2)}°。</p>
                  )}
                </>
              )}
            substitution={selectedConventionalDetail === undefined
              ? <p>等待 γ 与 δ。</p>
              : <BlockMath math={String.raw`\theta_2=${format(selectedConventionalDetail.targetDirectionDegrees, 2)}^\circ-${format(selectedConventionalDetail.triangleCorrectionDegrees, 2)}^\circ=${format(selectedConventionalDetail.qDegrees[1], 2)}^\circ`} />}
            title="肩角 θ₂"
            value={selectedConventionalDetail === undefined ? '—' : `${format(selectedConventionalDetail.qDegrees[1], 2)}°`}
          />
        </section>
      )}
      <details className="ik-advanced">
        <summary>Advanced · 查看角度几何关系</summary>
        {model.inverse.conventionalBranches.length > 0
          ? geometryDiagram('solution')
          : <p>目标不可达时没有可绘制的连杆角度。</p>}
      </details>
    </>,
    <>
      <IkStageField label="说明">
        <p>肘上和肘下到达同一目标位置，但肘点位于肩—目标连线两侧，末端连杆仰角 β 也可能不同。</p>
      </IkStageField>
      <IkStageField label="公式">
        <BlockMath math={String.raw`\mathbf q_{\mathrm{IK}}^{(i)}=\begin{bmatrix}\theta_1&\theta_2&\theta_3\end{bmatrix}^{\mathsf T},\qquad\beta^{(i)}=\theta_2^{(i)}+\theta_3^{(i)}`} />
      </IkStageField>
      <IkStageField label="数值代入">
        {model.inverse.conventionalBranches.length > 0 ? (
          <div aria-label="肘上肘下构型选择" className="ik-solution-grid" role="group">
            {model.inverse.conventionalBranches.map((detail) => {
              const id = configurationId(detail.solution)
              const applicable = isApplicableSolution(id)
              return (
                <button
                  aria-pressed={activeConfigurationId === id}
                  className="ik-solution-card"
                  key={id}
                  onClick={() => onConfigurationChange(id)}
                  type="button"
                >
                  <span className="ik-solution-card__heading">
                    <strong>{configurationLabel(detail.solution)}</strong>
                    <small>{applicable ? '可应用' : '仅教学图示'}</small>
                  </span>
                  <span>θ₂ {format(detail.qDegrees[1], 2)}°</span>
                  <span>θ₃ {format(detail.qDegrees[2], 2)}°</span>
                  <span>β {format(detail.toolElevationDegrees, 2)}°</span>
                  <em>{detail.solution.branch === 'elbow-up' ? '肘部位于 SP 上侧' : '肘部位于 SP 下侧'}</em>
                </button>
              )
            })}
          </div>
        ) : <p>当前目标没有可比较的肘部构型。</p>}
      </IkStageField>
      <IkStageField label="结果" tone="result">
        {selectedCandidateDetail === undefined ? (
          <p>请先返回“设置目标”并选择可达位置。</p>
        ) : (
          <>
            <strong>当前预览：{configurationLabel(selectedCandidateDetail.solution)} · {radialFamilyLabel(selectedCandidateDetail.solution)}</strong>
            <p>β = {format(selectedCandidateDetail.toolElevationDegrees, 2)}°；{selectedSolutionDetail === undefined ? '该候选不能写入当前关节限位。' : '该候选可应用到机器人。'}</p>
          </>
        )}
      </IkStageField>
      <details className="ik-advanced">
        <summary>Advanced · 折叠径向与全部候选</summary>
        <p>折叠径向会让基座反向，再由平面二连杆折叠到目标；它与肘上/肘下是两个独立分类维度。</p>
        <div aria-label="全部解析几何候选" className="configuration-candidates" role="group">
          {model.inverse.candidateDetails.map((detail) => {
            const id = configurationId(detail.solution)
            return (
              <button
                aria-pressed={activeConfigurationId === id}
                key={id}
                onClick={() => onConfigurationChange(id)}
                type="button"
              >
                <strong>{configurationLabel(detail.solution)} · {radialFamilyLabel(detail.solution)}</strong>
                <span>{isApplicableSolution(id) ? '当前可应用' : '仅教学图示'}</span>
              </button>
            )
          })}
        </div>
      </details>
    </>,
    <>
      <IkStageField label="说明">
        <p>把当前逆解写回正运动学，逐项比较目标位置、回代位置与位置残差。</p>
      </IkStageField>
      <IkStageField label="公式">
        <BlockMath math={String.raw`\mathbf q_{\mathrm{IK}}\rightarrow{}^0\mathbf T_{3,\mathrm{FK}}\rightarrow\mathbf p_{\mathrm{FK}},\qquad\Delta\mathbf p=\mathbf p_{\mathrm{FK}}-\mathbf p_d,\quad e_p=\lVert\Delta\mathbf p\rVert_2`} />
      </IkStageField>
      <IkStageField label="数值代入">
        {selectedCandidateDetail === undefined ? <p>当前没有可回代的解析候选。</p> : (
          <>
            <BlockMath math={String.raw`\mathbf p_d=${vectorLatex(model.inverse.targetMm, 2)}\ \mathrm{mm}`} />
            <BlockMath math={String.raw`\mathbf p_{\mathrm{FK}}=${vectorLatex(selectedCandidateDetail.achievedPositionMm, 2)}\ \mathrm{mm}`} />
            <BlockMath math={String.raw`\Delta\mathbf p=${vectorLatex(selectedCandidateDetail.positionResidualMm, 4)}\ \mathrm{mm}`} />
          </>
        )}
      </IkStageField>
      <IkStageField label="结果" tone="result">
        <StatusBanner
          tone={verificationTone}
          title={verificationTitle}
        >
          {selectedPositionError === undefined
            ? inverseStatusText
            : `当前候选的位置误差 eₚ = ${selectedPositionError.toExponential(3)} mm。`}
        </StatusBanner>
      </IkStageField>
      <details className="ik-advanced">
        <summary>Advanced · 完整回代与姿态矩阵</summary>
        {model.inverse.solutionDetails.length > 0 ? (
          <>
            <VerificationSummary
              solutions={model.inverse.solutionDetails}
              targetMm={model.inverse.targetMm}
            />
            <div className="ik-orientation-matrices">
              {model.inverse.solutionDetails.map((detail) => (
                <div key={configurationId(detail.solution)}>
                  <p>{configurationLabel(detail.solution)}：β = {format(detail.toolElevationDegrees, 2)}°</p>
                  <MatrixTable
                    label={`${configurationLabel(detail.solution)}末端姿态`}
                    matrix={detail.orientation}
                    precision={4}
                    symbol="{}^0\mathbf R_3"
                  />
                </div>
              ))}
            </div>
          </>
        ) : <p>当前没有满足关节限位的可应用解。</p>}
      </details>
    </>,
  ]

  const stepContent = [
    <>
      <StepSection title="先找出三个转动关节">
        <BlockMath math={String.raw`\mathbf q=\begin{bmatrix}\theta_1&\theta_2&\theta_3\end{bmatrix}^{\mathsf T}`} />
        <div aria-label="当前关节轴选择" className="walkthrough-column-selector" role="group">
          {([0, 1, 2] as const).map((index) => (
            <button
              aria-pressed={selectedDhRow === index}
              key={index}
              onClick={() => onDhRowChange(index)}
              type="button"
            >
              第 {index + 1} 关节 · z{String.fromCharCode(0x2080 + index)}
            </button>
          ))}
        </div>
        <p>标准 D–H 约定把第 i 个转动关节的轴记作 z<sub>i−1</sub>。当前三维图只突出正在处理的关节轴和下一根相关轴。</p>
      </StepSection>
      <StepSection title="轴编号为什么错开一位">
        <BlockMath math={String.raw`\text{关节 }i\quad\longleftrightarrow\quad\text{绕 }z_{i-1}\text{ 旋转}`} />
        <p>z₀、z₁、z₂ 分别是三个真实关节轴。最后的 z₃ 不对应第 4 个关节，而是根据末端坐标系约定选取。</p>
      </StepSection>
      <StepSection title="本步目标">
        <p>这一步只识别机构、关节位置和转轴，不急着填写 D–H 表。下一步再用相邻两根轴确定公法线、原点和坐标系。</p>
      </StepSection>
    </>,
    <>
      <StepSection title="解析公式"><BlockMath math={String.raw`{}^{i-1}\mathbf T_i=R_z(\theta_i)T_z(d_i)T_x(a_i)R_x(\alpha_i)`} /></StepSection>
      <StepSection title="标准 D–H 建系顺序">
        <ol className="walkthrough-explanation-list">
          <li>令 z<sub>i−1</sub> 沿第 i 个关节轴。</li>
          <li>令 z<sub>i</sub> 沿下一关节轴；末端 z₃ 按末端坐标系约定选取。</li>
          <li>x<sub>i</sub> 沿两根 z 轴的公法线，并由 z<sub>i−1</sub> 指向 z<sub>i</sub>。</li>
          <li>O<sub>i</sub> 位于 x<sub>i</sub> 与 z<sub>i</sub> 的交点，y<sub>i</sub> = z<sub>i</sub> × x<sub>i</sub>。</li>
        </ol>
        <p>若两轴相交，公法线长度为零，x<sub>i</sub> 按机构结构选定；若两轴平行，x<sub>i</sub> 沿它们之间的公法线。</p>
      </StepSection>
      <StepSection title="代入当前参数">
        <DhTable
          onParameterSelect={(row, parameter) => {
            onDhRowChange(row)
            onDhOperationChange(parameterOperation[parameter])
          }}
          onRowSelect={onDhRowChange}
          parameters={parameters}
          selectedParameter={dhParameter[dhOperation]}
          selectedRow={selectedDhRow}
        />
        <BlockMath math={String.raw`d_1=${format(d1Mm)}\ \mathrm{mm},\quad l_2=${format(l2Mm)}\ \mathrm{mm},\quad l_3=${format(l3Mm)}\ \mathrm{mm}`} />
      </StepSection>
      <StepSection title="四个参数的几何意义"><p>θ<sub>i</sub> 绕 z<sub>i−1</sub>，d<sub>i</sub> 沿 z<sub>i−1</sub>，a<sub>i</sub> 沿 x<sub>i</sub>，α<sub>i</sub> 绕 x<sub>i</sub>。点击任一表格单元格会同步三维方向和当前公式。</p></StepSection>
    </>,
    <>
      <StepSection title="从左到右累计右乘">
        <BlockMath math={String.raw`\mathbf A_i=R_z(\theta_i)T_z(d_i)T_x(a_i)R_x(\alpha_i)`} />
        <BlockMath math={String.raw`\mathbf A_i^{(0)}=\mathbf I,\qquad\mathbf A_i^{(k)}=\mathbf A_i^{(k-1)}\mathbf E_k`} />
      </StepSection>
      <StepSection title={`D–H 第 ${selectedDhRow + 1} 行代入与化简`}>
        <BlockMath math={selectedDhSubstitution} />
        <BlockMath math={selectedDhSimplification} />
        <p>零参数仍保留在第一行代入式中；它表示沿对应轴没有位移或绕对应轴没有转动，而不是该参数不存在。</p>
      </StepSection>
      <StepSection title="D–H 四段式变换演示">
        <DhTransformPlayer
          onOperationChange={onDhOperationChange}
          operation={dhOperation}
          parameters={parameters}
          q={q}
          row={selectedDhRow}
        />
      </StepSection>
      <StepSection title="按需展开矩阵">
        <details>
          <summary>展开完整符号矩阵</summary>
          <BlockMath math={String.raw`{}^{i-1}\mathbf T_i=\begin{bmatrix}c_i&-s_ic_{\alpha_i}&s_is_{\alpha_i}&a_ic_i\\s_i&c_ic_{\alpha_i}&-c_is_{\alpha_i}&a_is_i\\0&s_{\alpha_i}&c_{\alpha_i}&d_i\\0&0&0&1\end{bmatrix}`} />
        </details>
        <details>
          <summary>展开当前选中行的数值矩阵</summary>
          <MatrixTable label={`D–H 第 ${selectedDhRow + 1} 行数值变换（平移列为 mm）`} matrix={selectedDisplayTransform} precision={4} symbol={`\\mathbf A_${selectedDhRow + 1}`} />
        </details>
      </StepSection>
    </>,
    <>
      <StepSection title="解析公式"><BlockMath math={String.raw`{}^0\mathbf T_3={}^0\mathbf T_1\,{}^1\mathbf T_2\,{}^2\mathbf T_3`} /></StepSection>
      <StepSection title="代入当前参数"><div className="walkthrough-equation-scroll"><BlockMath math={String.raw`${matrixLatex(model.displayTransforms.t01)}${matrixLatex(model.displayTransforms.t12)}${matrixLatex(model.displayTransforms.t23)}`} /></div></StepSection>
      <StepSection title="矩阵结果">
        <HomogeneousTransformCard
          focusedPart={symbolFocus === 'beta' ? 'rotation' : symbolFocus === 'r' ? 'translation' : null}
          matrix={model.displayTransforms.t03}
          onFocusPart={(part) => {
            onSymbolFocus(part === 'rotation' ? 'beta' : 'r')
            onCameraPresetChange(part === 'rotation' ? 'tool' : 'overview')
          }}
        />
        <BlockMath math={String.raw`{}^0\bar{\mathbf p}_e={}^0\mathbf T_3{}^3\bar{\mathbf p}_e,\qquad{}^3\bar{\mathbf p}_e=\begin{bmatrix}0&0&0&1\end{bmatrix}^{\mathsf T}`} />
        <BlockMath math={String.raw`{}^0\mathbf p_e=\operatorname{trans}({}^0\mathbf T_3)`} />
        <p>横线表示四维齐次坐标。左上 3×3 是姿态，第四列前三项是毫米单位的末端位置，最后一行维持齐次坐标结构。</p>
      </StepSection>
    </>,
    <>
      <StepSection title="解析公式">
        <BlockMath math={String.raw`\rho=l_2\cos\theta_2+l_3\cos(\theta_2+\theta_3)`} />
        <BlockMath math={String.raw`\eta=l_2\sin\theta_2+l_3\sin(\theta_2+\theta_3)`} />
        <BlockMath math={String.raw`{}^0\mathbf p_e=\begin{bmatrix}\cos\theta_1\,\rho\\\sin\theta_1\,\rho\\d_1+\eta\end{bmatrix}`} />
      </StepSection>
      <StepSection title="代入当前参数"><BlockMath math={String.raw`\rho=${format(model.radialReachMm)}\ \mathrm{mm},\qquad\eta=${format(model.verticalReachMm)}\ \mathrm{mm}`} /></StepSection>
      <StepSection title="正运动学位置结果"><div data-revision={revision} data-testid="endpoint-result"><BlockMath math={String.raw`\boxed{{}^0\mathbf p_e=${vectorLatex(model.positionMm)}\ \mathrm{mm}}`} /></div></StepSection>
    </>,
    <>
      <StepSection title="解析公式">
        <BlockMath math={String.raw`\beta=\theta_2+\theta_3,\qquad c_{23}=\cos\beta,\quad s_{23}=\sin\beta`} />
        <BlockMath math={String.raw`{}^0\mathbf R_3=\begin{bmatrix}c_1c_{23}&-c_1s_{23}&s_1\\s_1c_{23}&-s_1s_{23}&-c_1\\s_{23}&c_{23}&0\end{bmatrix}`} />
      </StepSection>
      <StepSection title="代入当前参数"><p>基座方位角 θ₁ = {format(model.azimuthDegrees, 2)}°；末端连杆仰角 β = θ₂ + θ₃ = {format(model.toolElevationDegrees, 2)}°。</p></StepSection>
      <StepSection title="正运动学姿态结果"><MatrixTable label="末端姿态旋转矩阵" matrix={model.orientation} precision={4} symbol="{}^0\mathbf R_3" /><p>位置与旋转矩阵合在一起才是末端位姿。</p></StepSection>
    </>,
    <>
      <StepSection title="解析公式"><BlockMath math={String.raw`r=\sqrt{x^2+y^2},\qquad h=z-d_1,\qquad\theta_1=\operatorname{atan2}(y,x)`} /></StepSection>
      <StepSection title="代入当前参数">
        <BlockMath math={String.raw`(x,y,z)=${vectorLatex(model.inverse.targetMm)}\ \mathrm{mm}`} />
        <BlockMath math={String.raw`r=${format(model.inverse.radialMm)}\ \mathrm{mm},\quad h=${format(model.inverse.verticalMm)}\ \mathrm{mm},\quad\theta_1=${format(model.inverse.baseAngleDegrees, 2)}^\circ`} />
      </StepSection>
      <StepSection title="工作平面几何图">{geometryDiagram('projection')}<p>r 是水平投影，h 是相对肩部高度。绕基座轴求出 θ₁ 后，空间问题就化为肩—肘—目标所在的二维工作平面问题。</p></StepSection>
    </>,
    <>
      <StepSection title="解析公式"><BlockMath math={String.raw`s^2=r^2+h^2,\qquad\boxed{s=\sqrt{r^2+h^2}}`} /></StepSection>
      <StepSection title="代入当前参数"><BlockMath math={String.raw`s=\sqrt{${format(model.inverse.radialMm)}^2+${format(model.inverse.verticalMm)}^2}=${format(model.inverse.pythagoreanDistanceMm)}\ \mathrm{mm}`} /></StepSection>
      <StepSection title="几何解释">{geometryDiagram('distance')}<p>先由直角三角形得到 s；它是肩关节 S 到目标点 P 的直线距离，也是下一步连杆三角形的第三条边。</p></StepSection>
    </>,
    <>
      <StepSection title="解析公式">
        <BlockMath math={String.raw`s_{\min}=|l_2-l_3|,\qquad s_{\max}=l_2+l_3`} />
        <BlockMath math={String.raw`\boxed{s_{\min}\le s\le s_{\max}}`} />
      </StepSection>
      <StepSection title="代入当前参数"><BlockMath math={String.raw`${format(model.inverse.reachability.minimumMm)}\le ${format(model.inverse.pythagoreanDistanceMm)}\le ${format(model.inverse.reachability.maximumMm)}\ \mathrm{mm}`} /></StepSection>
      <StepSection title="可达性判断">
        <StatusBanner tone={model.inverse.reachability.isReachable ? 'success' : 'error'} title={model.inverse.reachability.isReachable ? '目标几何可达' : '目标几何不可达'}>最短可达距离 = {format(model.inverse.reachability.minimumMm)} mm；最远可达距离 = {format(model.inverse.reachability.maximumMm)} mm。</StatusBanner>
        <p>{inverseStatusText}</p>
      </StepSection>
    </>,
    <>
      <StepSection title="解析公式">
        <BlockMath math={String.raw`s^2=l_2^2+l_3^2+2l_2l_3\cos\theta_3`} />
        <BlockMath math={String.raw`D=\cos\theta_3=\frac{s^2-l_2^2-l_3^2}{2l_2l_3}=\frac{r^2+h^2-l_2^2-l_3^2}{2l_2l_3}`} />
        <BlockMath math={String.raw`\boxed{\theta_3^{(\downarrow)}=+\arccos D,\qquad\theta_3^{(\uparrow)}=-\arccos D}`} />
      </StepSection>
      <StepSection title="代入当前参数">
        <BlockMath math={String.raw`D=${format(model.inverse.cosineElbow)}`} />
        {model.inverse.conventionalBranches.map((detail) => <p key={detail.solution.branch}>{configurationLabel(detail.solution)}：θ₃ = {format(detail.qDegrees[2], 2)}°。</p>)}
      </StepSection>
      <StepSection title="几何解释">{geometryDiagram('elbow')}<p>再对连杆三角形使用余弦定理。正、负两个 θ₃ 使肘部落在肩—目标连线的两侧，形成肘下和肘上。</p></StepSection>
    </>,
    <>
      <StepSection title="解析公式"><BlockMath math={String.raw`\boxed{\gamma=\operatorname{atan2}(h,r)}`} /></StepSection>
      <StepSection title="代入当前参数">{model.inverse.conventionalBranches.map((detail) => <p key={detail.solution.branch}>{configurationLabel(detail.solution)}的目标方向角 γ = {format(detail.targetDirectionDegrees, 2)}°。</p>)}</StepSection>
      <StepSection title="几何解释">{geometryDiagram('gamma')}<p>γ 是水平正方向到“肩部—目标连线”SP 的方向角，只由目标位置决定；常规径向的两种肘部构型具有相同 γ。</p></StepSection>
    </>,
    <>
      <StepSection title="解析公式">
        <BlockMath math={String.raw`k_1=l_2+l_3\cos\theta_3,\qquad k_2=l_3\sin\theta_3`} />
        <BlockMath math={String.raw`\boxed{\delta=\operatorname{atan2}(k_2,k_1)=\operatorname{atan2}(l_3\sin\theta_3,l_2+l_3\cos\theta_3)}`} />
      </StepSection>
      <StepSection title="代入当前参数">{model.inverse.conventionalBranches.map((detail) => <div className="walkthrough-solution" key={detail.solution.branch}><strong>{configurationLabel(detail.solution)}</strong><p>k₁ = {format(detail.triangleProjectionMm)} mm，k₂ = {format(detail.triangleHeightMm)} mm；三角形补偿角 δ = {format(detail.triangleCorrectionDegrees, 2)}°。</p></div>)}</StepSection>
      <StepSection title="几何解释">{geometryDiagram('delta')}<p>δ 是第二连杆相对肩—目标连线的偏移角。它随 θ₃ 改变符号，因此两种肘部构型得到不同的肩角。</p></StepSection>
    </>,
    <>
      <StepSection title="解析公式">
        <BlockMath math={String.raw`\boxed{\theta_2=\gamma-\delta}`} />
        <BlockMath math={String.raw`\boxed{\mathbf q_{\mathrm{IK}}=\begin{bmatrix}\theta_1&\theta_2&\theta_3\end{bmatrix}^{\mathsf T}}`} />
      </StepSection>
      <StepSection title="解析解结果">{model.inverse.conventionalBranches.map((detail) => <div className="walkthrough-solution" key={detail.solution.branch}><strong>{configurationLabel(detail.solution)}解析关节解</strong><BlockMath math={String.raw`\theta_2=${format(detail.targetDirectionDegrees, 2)}^\circ-${format(detail.triangleCorrectionDegrees, 2)}^\circ=${format(detail.qDegrees[1], 2)}^\circ`} /><BlockMath math={String.raw`\mathbf q_{\mathrm{IK}}=${vectorLatex(detail.qDegrees, 2)}\ ^\circ`} /></div>)}</StepSection>
      <StepSection title="为什么是相减">{geometryDiagram('solution')}<p>从水平轴转到目标连线需要 γ；第二连杆必须从该连线向肘部一侧退回 δ，所以 θ₂ = γ − δ。图中的三条角弧对应这一个方向关系。</p></StepSection>
    </>,
    <>
      <StepSection title="先完成常规径向"><BlockMath math={String.raw`r_s=r,\qquad\theta_1=\operatorname{atan2}(y,x)`} /><p>前七个逆解步骤先完成最常见的常规径向推导，只比较由 θ₃ 正负产生的肘下和肘上。</p></StepSection>
      <StepSection title="再扩展折叠径向"><BlockMath math={String.raw`r_s=-r,\qquad\theta_1'=\operatorname{wrap}(\theta_1+180^\circ),\qquad\gamma'=\operatorname{atan2}(h,-r)`} /><p>折叠径向不是第三种肘部状态，而是让基座反向后再由平面二连杆折叠到目标。它与肘上/肘下属于两个独立分类维度。</p></StepSection>
      <StepSection title="分类对照">
        <table className="configuration-table"><thead><tr><th>分类</th><th>决定因素</th><th>几何含义</th></tr></thead><tbody><tr><td>肘上 / 肘下</td><td>θ₃ 的正负</td><td>肘部位于肩—目标连线的哪一侧</td></tr><tr><td>常规 / 折叠径向</td><td>rₛ = r 或 −r</td><td>基座朝向目标还是反向折叠到达</td></tr></tbody></table>
        {model.inverse.solutionDetails.map((detail) => <p key={detail.solution.branch}>当前可应用的{configurationLabel(detail.solution)}属于{radialFamilyLabel(detail.solution)}。</p>)}
      </StepSection>
      <StepSection title="三维教学构型">
        <div aria-label="全部解析几何候选" className="configuration-candidates" role="group">
          {model.inverse.candidateDetails.map((detail) => {
            const id = configurationId(detail.solution)
            const applicable = model.inverse.solutionDetails.some((solution) => (
              configurationId(solution.solution) === id
            ))
            return (
              <button
                aria-pressed={activeConfigurationId === id}
                key={id}
                onClick={() => onConfigurationChange(id)}
                type="button"
              >
                <strong>{configurationLabel(detail.solution)} · {radialFamilyLabel(detail.solution)}</strong>
                <span>{applicable ? '当前可应用' : '仅教学图示'}</span>
              </button>
            )
          })}
        </div>
        <p>选择只改变同步教学构型；只有标记为“当前可应用”的解才能通过左侧按钮写入实际关节状态。</p>
      </StepSection>
    </>,
    <>
      <StepSection title="验证公式">
        <BlockMath math={String.raw`{}^0\mathbf T_{3,\mathrm{FK}}^{(i)}=\mathbf A_1(q_1^{(i)})\mathbf A_2(q_2^{(i)})\mathbf A_3(q_3^{(i)})`} />
        <BlockMath math={String.raw`\mathbf p_{\mathrm{FK}}^{(i)}=\operatorname{trans}({}^0\mathbf T_{3,\mathrm{FK}}^{(i)})`} />
        <BlockMath math={String.raw`\Delta\mathbf p^{(i)}=\mathbf p_{\mathrm{FK}}^{(i)}-\mathbf p_d`} />
        <BlockMath math={String.raw`e_p^{(i)}=\|\Delta\mathbf p^{(i)}\|_2`} />
      </StepSection>
      <StepSection title="逐组回代">
        <VerificationSummary
          solutions={model.inverse.solutionDetails}
          targetMm={model.inverse.targetMm}
        />
      </StepSection>
      <StepSection title="验证结论"><p>验证链路是 q<sub>IK</sub> → T<sub>FK</sub> → p<sub>FK</sub> → Δp；这里展示的是三个方向的差值与范数，而不只是最终误差结论。</p></StepSection>
    </>,
    <>
      <StepSection title="姿态回代公式"><BlockMath math={String.raw`\mathbf q_{\mathrm{IK}}^{(i)}\longrightarrow\beta^{(i)}=\theta_2^{(i)}+\theta_3^{(i)}\longrightarrow{}^0\mathbf R_{3,\mathrm{FK}}^{(i)}`} /></StepSection>
      <StepSection title="每组解的实际姿态">
        {model.inverse.solutionDetails.length > 0 ? model.inverse.solutionDetails.map((detail) => <div className="walkthrough-solution" key={detail.solution.branch}><strong>{configurationLabel(detail.solution)}</strong><p>β = θ₂ + θ₃ = {format(detail.qDegrees[1], 2)}° + {format(detail.qDegrees[2], 2)}° = {format(detail.toolElevationDegrees, 2)}°。</p><MatrixTable label={`${configurationLabel(detail.solution)}末端姿态`} matrix={detail.orientation} precision={4} symbol="{}^0\mathbf R_3" /></div>) : <p>{inverseStatusText}</p>}
      </StepSection>
      <StepSection title="位姿含义"><p>两组解析解可以到达同一目标位置，但 β 不同，所以实际姿态也可能不同。这里求解的是目标位置，并展示该位置解自然产生的姿态，不是任意六维目标位姿逆解。</p></StepSection>
    </>,
    <>
      <StepSection title="当前运动状态">
        <BlockMath math={String.raw`\mathbf q=${vectorLatex(model.qDegrees, 2)}\ ^\circ,\qquad\dot{\mathbf q}=${vectorLatex(model.qdDegreesPerSecond, 2)}\ ^\circ/\mathrm s`} />
      </StepSection>
      <StepSection title="核心计算与显示单位">
        <BlockMath math={String.raw`\begin{bmatrix}\mathbf v_e\\\boldsymbol\omega_e\end{bmatrix}=\begin{bmatrix}\mathbf J_v(\mathbf q)\\\mathbf J_\omega(\mathbf q)\end{bmatrix}\dot{\mathbf q}`} />
        <p>核心计算使用 m/rad、rad/s；界面把 J<sub>v</sub> 转换为 mm/°，把 q̇ 和 ω<sub>e</sub> 显示为 °/s。J<sub>ω</sub> 数值不变，因为输入和输出使用相同角度单位。</p>
      </StepSection>
      <StepSection title="当前末端速度">
        <BlockMath math={String.raw`\mathbf v_e=${vectorLatex(model.linearVelocityMillimetresPerSecond, 2)}\ \mathrm{mm/s}`} />
        <BlockMath math={String.raw`\boldsymbol\omega_e=${vectorLatex(model.angularVelocityDegreesPerSecond, 2)}\ ^\circ/\mathrm s`} />
      </StepSection>
    </>,
    <>
      <StepSection title="几何构造公式">
        <BlockMath math={String.raw`\mathbf J_i=\begin{bmatrix}\mathbf z_{i-1}\times({}^0\mathbf p_e-{}^0\mathbf o_{i-1})\\\mathbf z_{i-1}\end{bmatrix}`} />
        <BlockMath math={String.raw`\frac{\partial\mathbf p_{e,\mathrm{mm}}}{\partial\theta_i(^\circ)}=\frac{\pi}{180}\left[\mathbf z_{i-1}\times({}^0\mathbf p_e-{}^0\mathbf o_{i-1})\right]_{\mathrm{mm}}`} />
      </StepSection>
      <StepSection title="逐列代入当前参数">
        <div aria-label="雅可比列选择" className="walkthrough-column-selector" role="group">
          {([0, 1, 2] as const).map((index) => (
            <button
              aria-pressed={selectedJacobianColumn === index}
              key={index}
              onClick={() => onJacobianColumnChange(index)}
              type="button"
            >
              第 {index + 1} 列
            </button>
          ))}
        </div>
        {(() => {
          const column = model.jacobianColumns[selectedJacobianColumn]
          return (
            <div className="walkthrough-column-list">
              <div>
                <strong>当前展示第 {selectedJacobianColumn + 1} 列</strong>
                <BlockMath math={String.raw`\mathbf z_${selectedJacobianColumn}=${vectorLatex(column.axis)},\quad{}^0\mathbf o_${selectedJacobianColumn}=${vectorLatex(column.originMm)}\ \mathrm{mm}`} />
                <BlockMath math={String.raw`{}^0\mathbf p_e-{}^0\mathbf o_${selectedJacobianColumn}=${vectorLatex(column.offsetMm)}\ \mathrm{mm}`} />
                <BlockMath math={String.raw`\frac{\partial\mathbf p_e}{\partial\theta_${selectedJacobianColumn + 1}(^\circ)}=${vectorLatex(column.linearMmPerDegree)}\ \mathrm{mm}/{}^\circ`} />
              </div>
            </div>
          )
        })()}
      </StepSection>
      <StepSection title="雅可比结果">
        <div data-testid="jacobian-result"><MatrixTable label="几何雅可比矩阵（上三行 mm/°，下三行无量纲）" matrix={model.displayJacobian} precision={4} symbol="\mathbf J(\mathbf q)" /></div>
        <p>上三行 J<sub>v</sub> 把 °/s 映射为 mm/s；下三行 J<sub>ω</sub> 把 °/s 映射为 °/s。这里使用轴线叉乘的几何构造，不是数值差分。</p>
      </StepSection>
    </>,
    <>
      <StepSection title="速度映射公式">
        <BlockMath math={String.raw`\boxed{\mathbf v_e=\mathbf J_v(\mathbf q)\dot{\mathbf q}},\qquad\boxed{\boldsymbol\omega_e=\mathbf J_\omega(\mathbf q)\dot{\mathbf q}}`} />
        <BlockMath math={String.raw`\mathbf v_e=\mathbf J_{v,1}\dot\theta_1+\mathbf J_{v,2}\dot\theta_2+\mathbf J_{v,3}\dot\theta_3`} />
      </StepSection>
      <StepSection title="逐关节速度贡献">
        <div className="walkthrough-column-list">
          {model.velocityContributions.map((contribution) => (
            <div key={contribution.jointIndex}>
              <strong>第 {contribution.jointIndex + 1} 关节</strong>
              <BlockMath math={String.raw`\mathbf v_e^{(${contribution.jointIndex + 1})}=${vectorLatex(contribution.linearMillimetresPerSecond, 2)}\ \mathrm{mm/s}`} />
              <BlockMath math={String.raw`\boldsymbol\omega_e^{(${contribution.jointIndex + 1})}=${vectorLatex(contribution.angularDegreesPerSecond, 2)}\ ^\circ/\mathrm s`} />
            </div>
          ))}
        </div>
      </StepSection>
      <StepSection title="末端速度结果">
        <BlockMath math={String.raw`\boxed{\mathbf v_e=${vectorLatex(model.linearVelocityMillimetresPerSecond, 2)}\ \mathrm{mm/s}}`} />
        <BlockMath math={String.raw`\boxed{\boldsymbol\omega_e=${vectorLatex(model.angularVelocityDegreesPerSecond, 2)}\ ^\circ/\mathrm s}`} />
        <p>三维箭头只用于辨识方向，长度经过视觉归一化；以上公式中的数值才是真实量值。</p>
      </StepSection>
    </>,
    <>
      <StepSection title="位置奇异性指标">
        <BlockMath math={String.raw`\sigma_{\min}(\mathbf J_v),\qquad\kappa(\mathbf J_v)=\frac{\sigma_{\max}(\mathbf J_v)}{\sigma_{\min}(\mathbf J_v)}`} />
        <p>这里只判断末端位置可控性的退化，不把完整 6×3 几何雅可比笼统称为奇异。</p>
      </StepSection>
      <StepSection title="当前判断">
        <StatusBanner tone={positionSingularityTone} title={positionSingularityTitle}>
          σ<sub>min</sub>(J<sub>v</sub>) = {minimumSingularValue.toExponential(3)} mm/°；κ(J<sub>v</sub>) = {Number.isFinite(singularity.conditionNumber) ? singularity.conditionNumber.toFixed(3) : '∞'}。
        </StatusBanner>
        <p>条件数为无穷表示数值秩亏；有限但最小奇异值低于教学阈值时标记为“接近”。</p>
      </StepSection>
      <StepSection title="补充指标">
        <details>
          <summary>查看位置可操作度和各向同性</summary>
          <BlockMath math={String.raw`\mu=\sigma_1\sigma_2\sigma_3=${format(displayManipulability)}\ (\mathrm{mm}/{}^\circ)^3`} />
          <BlockMath math={String.raw`\eta=1/\kappa=${format(singularity.inverseConditionNumber, 3)}`} />
        </details>
      </StepSection>
    </>,
  ]

  const currentStep = STEPS[stepIndex]
  const modeSteps = KINEMATICS_MODE_STEPS[mode]
  const localStepIndex = Math.max(0, modeSteps.indexOf(stepIndex))

  return (
    <article
      aria-labelledby="kinematics-walkthrough-title"
      className="kinematics-walkthrough"
      data-configuration={activeConfigurationId}
      data-revision={revision}
      data-symbol-focus={symbolFocus ?? undefined}
      data-testid="kinematics-walkthrough"
    >
      <header className="walkthrough-header">
        <div><p className="walkthrough-method" data-testid="walkthrough-method">{currentStep.method}</p><h3 id="kinematics-walkthrough-title">{currentStep.title}</h3></div>
        <span aria-live="polite" data-testid="walkthrough-step">第 {localStepIndex + 1} / {modeSteps.length} 步</span>
      </header>
      <nav aria-label={`${mode === 'forward' ? '正运动学' : mode === 'inverse' ? '位置逆运动学' : '微分运动学'}推导步骤`} className="walkthrough-local-steps">
        {modeSteps.map((globalStepIndex, index) => (
          <button
            aria-current={globalStepIndex === stepIndex ? 'step' : undefined}
            data-state={globalStepIndex === stepIndex ? 'current' : index < localStepIndex ? 'completed' : 'upcoming'}
            key={globalStepIndex}
            onClick={() => onStepChange(globalStepIndex)}
            type="button"
          >
            <span>{index < localStepIndex ? '✓' : index + 1}</span>
            {MODE_STEP_LABELS[mode][index]}
          </button>
        ))}
      </nav>
      {STEP_SYMBOLS[stepIndex].length > 0 && (
        <div aria-label="本步符号焦点" className="walkthrough-symbols" role="group">
          <span>同步聚焦</span>
          {STEP_SYMBOLS[stepIndex].map((symbol) => (
            <button
              aria-pressed={symbolFocus === symbol}
              key={symbol}
              onClick={() => onSymbolFocus(symbolFocus === symbol ? null : symbol)}
              type="button"
            >
              聚焦 {SYMBOL_LABELS[symbol]}
            </button>
          ))}
        </div>
      )}
      <div className="walkthrough-body" key={stepIndex} ref={walkthroughBodyRef}>
        {mode === 'inverse' ? inverseStageContent[localStepIndex] : stepContent[stepIndex]}
      </div>
      <footer className="walkthrough-actions">
        <button
          disabled={localStepIndex === 0}
          onClick={() => onStepChange(modeSteps[Math.max(0, localStepIndex - 1)])}
          type="button"
        >
          上一步
        </button>
        {mode === 'inverse' && localStepIndex === modeSteps.length - 1 ? (
          <button disabled={!canApplyInverse} onClick={onApplyInverse} type="button">
            应用逆解
          </button>
        ) : (
          <button
            disabled={localStepIndex === modeSteps.length - 1}
            onClick={() => onStepChange(modeSteps[Math.min(modeSteps.length - 1, localStepIndex + 1)])}
            type="button"
          >
            下一步
          </button>
        )}
      </footer>
    </article>
  )
}
