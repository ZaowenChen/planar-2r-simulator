import { useEffect, useMemo, useState } from 'react'
import { BlockMath, InlineMath } from 'react-katex'
import { MatrixTable } from '../../components/MatrixTable'
import { StatusBanner } from '../../components/StatusBanner'
import type { SingularityMetrics } from '../../robotics/jacobian'
import type { Matrix4, RobotParameters, Vector3 } from '../../robotics/types'
import type { AngleUnit } from '../../state/labStore'
import { DhTable } from '../model/DhTable'
import { buildKinematicsDerivation } from './derivationModel'

interface KinematicsWalkthroughProps {
  q: Vector3
  parameters: RobotParameters
  target: Vector3
  angleUnit: AngleUnit
  singularity: SingularityMetrics
  revision: number
}

const STEP_TITLES = [
  '角度输入与弧度转换',
  '读取 D–H 参数',
  '写出三个齐次变换',
  '代入数值得到变换矩阵',
  '连乘得到基座到末端的变换',
  '提取末端位置',
  '逐列计算几何雅可比',
  '用余弦定理求逆运动学',
  '比较肘下解与肘上解',
] as const

function format(value: number, precision = 4): string {
  if (!Number.isFinite(value)) return value > 0 ? '\\infty' : '—'
  return Math.abs(value) < 10 ** (-precision) ? '0' : value.toFixed(precision)
}

function vectorLatex(vector: readonly number[], precision = 4): string {
  return String.raw`\begin{bmatrix}${vector.map((value) => format(value, precision)).join('\\')}\end{bmatrix}`
}

function matrixLatex(matrix: readonly (readonly number[])[], precision = 4): string {
  return String.raw`\begin{bmatrix}${matrix.map((row) => row.map((value) => format(value, precision)).join('&')).join('\\')}\end{bmatrix}`
}

function StepSection({ title, children }: React.PropsWithChildren<{ title: string }>) {
  return (
    <section className="walkthrough-section">
      <h4>{title}</h4>
      <div>{children}</div>
    </section>
  )
}

function TransformTables({
  t01,
  t12,
  t23,
}: {
  t01: Matrix4
  t12: Matrix4
  t23: Matrix4
}) {
  return (
    <div className="walkthrough-matrix-grid">
      <MatrixTable label="关节 1 的数值变换" matrix={t01} precision={4} symbol="{}^0\mathbf T_1" />
      <MatrixTable label="关节 2 的数值变换" matrix={t12} precision={4} symbol="{}^1\mathbf T_2" />
      <MatrixTable label="关节 3 的数值变换" matrix={t23} precision={4} symbol="{}^2\mathbf T_3" />
    </div>
  )
}

export function KinematicsWalkthrough({
  q,
  parameters,
  target,
  angleUnit,
  singularity,
  revision,
}: KinematicsWalkthroughProps) {
  const [stepIndex, setStepIndex] = useState(0)
  const model = useMemo(
    () => buildKinematicsDerivation(q, parameters, target),
    [parameters, q, target],
  )

  useEffect(() => {
    setStepIndex(0)
  }, [angleUnit, parameters, q, target])

  const { d1, l2, l3 } = parameters.geometry
  const inverseStatus = model.inverse.result.status
  const inverseStatusText = inverseStatus === 'reachable'
    ? '目标可达，可以继续计算两组逆解。'
    : inverseStatus === 'axis-singular'
      ? '目标位于基座轴线上，基座角 θ₁ 不唯一。'
      : inverseStatus === 'joint-limit'
        ? '几何上可达，但当前关节限位过滤了候选解。'
        : '余弦值不在 [-1, 1] 内，目标超出工作空间。'

  const stepContent = [
    <>
      <StepSection title="符号公式">
        <BlockMath math={String.raw`\mathbf q=\begin{bmatrix}\theta_1&\theta_2&\theta_3\end{bmatrix}^{\mathsf T},\qquad \theta_{i,\mathrm{rad}}=\theta_{i,\mathrm{deg}}\frac{\pi}{180}`} />
      </StepSection>
      <StepSection title="当前数值代入">
        <BlockMath math={String.raw`\mathbf q_{\mathrm{deg}}=${vectorLatex(model.degrees)}\ ^\circ`} />
        <BlockMath math={String.raw`\mathbf q_{\mathrm{rad}}=${vectorLatex(model.degrees)}\frac{\pi}{180}`} />
      </StepSection>
      <StepSection title="本步结果">
        <BlockMath math={String.raw`\boxed{\mathbf q=${vectorLatex(model.radians)}\ \mathrm{rad}}`} />
        <p>三角函数在程序内部统一使用弧度；界面选择“度”只改变输入与显示方式。</p>
      </StepSection>
    </>,
    <>
      <StepSection title="符号公式">
        <BlockMath math={String.raw`{}^{i-1}\mathbf T_i=R_z(\theta_i)T_z(d_i)T_x(a_i)R_x(\alpha_i)`} />
      </StepSection>
      <StepSection title="当前数值代入">
        <DhTable parameters={parameters} />
      </StepSection>
      <StepSection title="本步结果">
        <BlockMath math={String.raw`d_1=${format(d1)}\ \mathrm m,\qquad l_2=${format(l2)}\ \mathrm m,\qquad l_3=${format(l3)}\ \mathrm m`} />
        <p>D–H 表把每个关节的旋转与连杆尺寸整理为同一种变换规则。</p>
      </StepSection>
    </>,
    <>
      <StepSection title="符号公式">
        <BlockMath math={String.raw`c_i=\cos\theta_i,\quad s_i=\sin\theta_i`} />
        <BlockMath math={String.raw`{}^0\mathbf T_1=\begin{bmatrix}c_1&0&s_1&0\\s_1&0&-c_1&0\\0&1&0&d_1\\0&0&0&1\end{bmatrix}`} />
        <BlockMath math={String.raw`{}^1\mathbf T_2=\begin{bmatrix}c_2&-s_2&0&l_2c_2\\s_2&c_2&0&l_2s_2\\0&0&1&0\\0&0&0&1\end{bmatrix}`} />
        <BlockMath math={String.raw`{}^2\mathbf T_3=\begin{bmatrix}c_3&-s_3&0&l_3c_3\\s_3&c_3&0&l_3s_3\\0&0&1&0\\0&0&0&1\end{bmatrix}`} />
      </StepSection>
      <StepSection title="当前数值代入">
        <BlockMath math={String.raw`(\theta_1,\theta_2,\theta_3)=${format(q[0])},\ ${format(q[1])},\ ${format(q[2])}\ \mathrm{rad}`} />
      </StepSection>
      <StepSection title="本步结果">
        <p>每个矩阵的左上角 <InlineMath math="3\times3" /> 部分表示姿态，最后一列表示位置；这就是同时携带姿态和位置的“齐次变换”。</p>
      </StepSection>
    </>,
    <>
      <StepSection title="符号公式">
        <BlockMath math={String.raw`c_i=\cos\theta_i,\quad s_i=\sin\theta_i\quad\Longrightarrow\quad{}^{i-1}\mathbf T_i`} />
      </StepSection>
      <StepSection title="当前数值代入">
        <BlockMath math={String.raw`(c_1,s_1)=(${format(Math.cos(q[0]))},${format(Math.sin(q[0]))}),\quad(c_2,s_2)=(${format(Math.cos(q[1]))},${format(Math.sin(q[1]))}),\quad(c_3,s_3)=(${format(Math.cos(q[2]))},${format(Math.sin(q[2]))})`} />
      </StepSection>
      <StepSection title="本步结果">
        <TransformTables t01={model.t01} t12={model.t12} t23={model.t23} />
      </StepSection>
    </>,
    <>
      <StepSection title="符号公式">
        <BlockMath math={String.raw`{}^0\mathbf T_3={}^0\mathbf T_1\,{}^1\mathbf T_2\,{}^2\mathbf T_3`} />
      </StepSection>
      <StepSection title="当前数值代入">
        <div className="walkthrough-equation-scroll">
          <BlockMath math={String.raw`${matrixLatex(model.t01)}${matrixLatex(model.t12)}${matrixLatex(model.t23)}`} />
        </div>
      </StepSection>
      <StepSection title="本步结果">
        <MatrixTable label="前两个变换的乘积" matrix={model.t02} precision={4} symbol="{}^0\mathbf T_2" />
        <div data-testid="transform-result">
          <MatrixTable label="基座到末端的齐次变换" matrix={model.t03} precision={4} symbol="{}^0\mathbf T_3" />
        </div>
      </StepSection>
    </>,
    <>
      <StepSection title="符号公式">
        <BlockMath math={String.raw`\rho=l_2\cos\theta_2+l_3\cos(\theta_2+\theta_3),\qquad \eta=l_2\sin\theta_2+l_3\sin(\theta_2+\theta_3)`} />
        <BlockMath math={String.raw`{}^0\mathbf p_e=\begin{bmatrix}\cos\theta_1\,\rho\\\sin\theta_1\,\rho\\d_1+\eta\end{bmatrix}`} />
      </StepSection>
      <StepSection title="当前数值代入">
        <BlockMath math={String.raw`\rho=${format(l2)}\cos(${format(q[1])})+${format(l3)}\cos(${format(q[1] + q[2])})=${format(model.radialReach)}\ \mathrm m`} />
        <BlockMath math={String.raw`\eta=${format(l2)}\sin(${format(q[1])})+${format(l3)}\sin(${format(q[1] + q[2])})=${format(model.verticalReach)}\ \mathrm m`} />
      </StepSection>
      <StepSection title="本步结果">
        <div data-revision={revision} data-testid="endpoint-result">
          <BlockMath math={String.raw`\boxed{{}^0\mathbf p_e=${vectorLatex(model.position)}\ \mathrm m}`} />
        </div>
        <p>最终矩阵第四列的前三个数，就是末端相对基座的 <InlineMath math="x,y,z" /> 坐标。</p>
      </StepSection>
    </>,
    <>
      <StepSection title="符号公式">
        <BlockMath math={String.raw`\mathbf J_i=\begin{bmatrix}\mathbf z_{i-1}\times({}^0\mathbf p_e-{}^0\mathbf o_{i-1})\\\mathbf z_{i-1}\end{bmatrix},\qquad \dot{\mathbf x}=\mathbf J(\mathbf q)\dot{\mathbf q}`} />
      </StepSection>
      <StepSection title="当前数值代入">
        <div className="walkthrough-column-list">
          {model.jacobianColumns.map((column, index) => (
            <div key={index}>
              <strong>第 {index + 1} 列</strong>
              <BlockMath math={String.raw`\mathbf z_${index}=${vectorLatex(column.axis)},\quad{}^0\mathbf o_${index}=${vectorLatex(column.origin)}`} />
              <BlockMath math={String.raw`{}^0\mathbf p_e-{}^0\mathbf o_${index}=${vectorLatex(column.offset)}`} />
              <BlockMath math={String.raw`\mathbf z_${index}\times({}^0\mathbf p_e-{}^0\mathbf o_${index})=${vectorLatex(column.linear)}`} />
            </div>
          ))}
        </div>
      </StepSection>
      <StepSection title="本步结果">
        <div data-testid="jacobian-result">
          <MatrixTable label="几何雅可比矩阵" matrix={model.jacobian} precision={4} symbol="\mathbf J(\mathbf q)" />
        </div>
        <StatusBanner
          tone={singularity.isSingular ? 'warning' : 'success'}
          title={singularity.isSingular ? '接近奇异位形' : '远离奇异位形'}
        >
          最小奇异值 σ<sub>min</sub> = {singularity.minimumSingularValue.toExponential(3)}；条件数 κ = {Number.isFinite(singularity.conditionNumber) ? singularity.conditionNumber.toFixed(3) : '∞'}
        </StatusBanner>
        <p>雅可比把关节角速度映射为末端线速度与角速度，也用来判断机械臂是否接近奇异位形。</p>
      </StepSection>
    </>,
    <>
      <StepSection title="符号公式">
        <BlockMath math={String.raw`r=\sqrt{x^2+y^2},\qquad z'=z-d_1`} />
        <BlockMath math={String.raw`D=\cos\theta_3=\frac{r^2+z'^2-l_2^2-l_3^2}{2l_2l_3}`} />
      </StepSection>
      <StepSection title="当前数值代入">
        <BlockMath math={String.raw`(x,y,z)=(${format(target[0])},${format(target[1])},${format(target[2])})\ \mathrm m`} />
        <BlockMath math={String.raw`r=\sqrt{${format(target[0])}^2+${format(target[1])}^2}=${format(model.inverse.radial)}\ \mathrm m,\quad z'=${format(target[2])}-${format(d1)}=${format(model.inverse.vertical)}\ \mathrm m`} />
        <BlockMath math={String.raw`D=\frac{${format(model.inverse.radial)}^2+${format(model.inverse.vertical)}^2-${format(l2)}^2-${format(l3)}^2}{2(${format(l2)})(${format(l3)})}=${format(model.inverse.cosineElbow)}`} />
      </StepSection>
      <StepSection title="本步结果">
        <p>{inverseStatusText}</p>
      </StepSection>
    </>,
    <>
      <StepSection title="符号公式">
        <BlockMath math={String.raw`\theta_3=\pm\arccos D`} />
        <BlockMath math={String.raw`\theta_2=\operatorname{atan2}(z',r)-\operatorname{atan2}(l_3\sin\theta_3,l_2+l_3\cos\theta_3),\qquad \theta_1=\operatorname{atan2}(y,x)`} />
      </StepSection>
      <StepSection title="当前数值代入">
        {model.inverse.result.solutions.length > 0 ? model.inverse.result.solutions.map((solution) => (
          <div className="walkthrough-solution" key={solution.branch}>
            <strong>{solution.branch === 'elbow-down' ? '肘下解（+）' : '肘上解（−）'}</strong>
            <BlockMath math={String.raw`\mathbf q=${vectorLatex(solution.q)}\ \mathrm{rad}=${vectorLatex(solution.q.map((angle) => angle * 180 / Math.PI))}\ ^\circ`} />
          </div>
        )) : <p>{inverseStatusText}</p>}
      </StepSection>
      <StepSection title="本步结果">
        <p>余弦相同的角度可以取正、负两个符号，所以机械臂的肘部可以位于目标连线的两侧，形成“肘下”和“肘上”两种姿态。两组关节角经过正运动学都会到达同一目标位置。</p>
      </StepSection>
    </>,
  ]

  return (
    <article
      aria-labelledby="kinematics-walkthrough-title"
      className="kinematics-walkthrough"
      data-revision={revision}
      data-testid="kinematics-walkthrough"
    >
      <header className="walkthrough-header">
        <div>
          <p className="section-label">计算过程</p>
          <h3 id="kinematics-walkthrough-title">{STEP_TITLES[stepIndex]}</h3>
        </div>
        <span aria-live="polite" data-testid="walkthrough-step">第 {stepIndex + 1} / {STEP_TITLES.length} 步</span>
      </header>

      <div className="walkthrough-body" key={stepIndex}>
        {stepContent[stepIndex]}
      </div>

      <footer className="walkthrough-actions">
        <button
          disabled={stepIndex === 0}
          onClick={() => setStepIndex((current) => Math.max(0, current - 1))}
          type="button"
        >
          上一步
        </button>
        <button
          disabled={stepIndex === STEP_TITLES.length - 1}
          onClick={() => setStepIndex((current) => Math.min(STEP_TITLES.length - 1, current + 1))}
          type="button"
        >
          下一步
        </button>
      </footer>
    </article>
  )
}
