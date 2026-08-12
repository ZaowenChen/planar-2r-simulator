import { BlockMath, InlineMath } from 'react-katex'
import { WorkbenchLayout } from '../../app/WorkbenchLayout'
import { FormulaCard } from '../../components/FormulaCard'
import { MatrixTable } from '../../components/MatrixTable'
import { DEFAULT_ROBOT_PARAMETERS } from '../../robotics/defaults'
import { RobotScene } from '../../scene/RobotScene'
import { useLabStore } from '../../state/labStore'
import { DISPLAY } from '../../symbols/display'
import { ParameterDraftField, RigidBodyEditor } from './RigidBodyEditor'

const AXIS_SUBSCRIPTS = ['ₓ', 'ᵧ', 'ᶻ'] as const
const JOINT_SUBSCRIPTS = ['₁', '₂', '₃'] as const

function vectorText(vector: readonly number[], precision = 4): string {
  return `[${vector.map((value) => value.toFixed(precision)).join(', ')}]`
}

function vectorLatex(vector: readonly number[], precision = 4): string {
  return `\\begin{bmatrix}${vector.map((value) => value.toFixed(precision)).join('\\cr ')}\\end{bmatrix}`
}

function matrixLatex(matrix: readonly (readonly number[])[], precision = 4): string {
  return `\\begin{bmatrix}${matrix.map((row) => row.map((value) => value.toFixed(precision)).join('&')).join('\\cr ')}\\end{bmatrix}`
}

function scalarText(value: number, precision = 4): string {
  return Number.isFinite(value) ? value.toFixed(precision) : '—'
}

function resetDynamicsParameters(): void {
  const { setFrictionEnabled, setParameterFieldsAtomically } = useLabStore.getState()
  const fields: Record<string, string> = {}
  DEFAULT_ROBOT_PARAMETERS.links.forEach((link, linkIndex) => {
    fields[`links.${linkIndex}.mass`] = String(link.mass)
    link.centerOfMass.forEach((value, componentIndex) => {
      fields[`links.${linkIndex}.centerOfMass.${componentIndex}`] = String(value)
    })
    link.inertia.forEach((row, rowIndex) => {
      row.forEach((value, columnIndex) => {
        fields[`links.${linkIndex}.inertia.${rowIndex}.${columnIndex}`] = String(value)
      })
    })
  })
  DEFAULT_ROBOT_PARAMETERS.gravity.forEach((value, index) => {
    fields[`gravity.${index}`] = String(value)
  })
  DEFAULT_ROBOT_PARAMETERS.viscousFriction.forEach((value, index) => {
    fields[`viscousFriction.${index}`] = String(value)
  })
  setParameterFieldsAtomically(fields)
  setFrictionEnabled(DEFAULT_ROBOT_PARAMETERS.frictionEnabled)
}

function EnvironmentEditor() {
  const parameters = useLabStore((state) => state.parameters)
  const setFrictionEnabled = useLabStore((state) => state.setFrictionEnabled)

  return (
    <section className="control-section" aria-labelledby="environment-editor-title">
      <h4 id="environment-editor-title">重力与关节摩擦</h4>
      <div className="field-grid">
        {parameters.gravity.map((value, index) => (
          <ParameterDraftField
            fallback={value}
            key={`gravity-${index}`}
            label={`重力方向 g${AXIS_SUBSCRIPTS[index]}`}
            path={`gravity.${index}`}
            unit="m/s²"
          />
        ))}
        {parameters.viscousFriction.map((value, index) => (
          <ParameterDraftField
            constraint="nonnegative"
            fallback={value}
            key={`friction-${index}`}
            label={`关节 ${index + 1} 粘性摩擦 b${JOINT_SUBSCRIPTS[index]}`}
            path={`viscousFriction.${index}`}
            unit="N·m·s/rad"
          />
        ))}
      </div>
      <label>
        <input
          checked={parameters.frictionEnabled}
          onChange={(event) => setFrictionEnabled(event.target.checked)}
          type="checkbox"
        />
        启用粘性摩擦
      </label>
    </section>
  )
}

export function DynamicsPage() {
  const calculation = useLabStore((state) => state.calculation)
  const jointState = useLabStore((state) => state.jointState)
  const parameters = useLabStore((state) => state.parameters)
  const dynamics = calculation.dynamics
  const energy = calculation.energy
  const sceneCalculation = {
    forward: calculation.forward,
    jacobian: calculation.jacobian,
    jointState,
    torque: dynamics.tau,
    gravity: parameters.gravity,
  }

  return (
    <WorkbenchLayout
      visual={(
        <div>
          <output data-testid="endpoint-result">
            <InlineMath math={DISPLAY.endEffectorPosition} /> = {vectorText(calculation.forward.endEffectorPosition)} m
          </output>
          <RobotScene
            calculation={sceneCalculation}
            initialOverlays={{ centerOfMass: true, gravity: true, torque: true }}
          />
        </div>
      )}
      controls={(
        <div className="control-sheet dynamics-controls">
          <p className="section-label">刚体与环境参数</p>
          <h3>可编辑动力学模型</h3>
          <p className="control-sheet__note">非法草稿不会进入计算；公式结果始终使用最近一次通过物理约束验证的参数。</p>
          <RigidBodyEditor linkIndex={0} />
          <RigidBodyEditor linkIndex={1} />
          <RigidBodyEditor linkIndex={2} />
          <EnvironmentEditor />
          <button
            aria-label="复位全部动力学教学参数"
            onClick={resetDynamicsParameters}
            type="button"
          >
            复位全部动力学教学参数
          </button>
        </div>
      )}
      analysis={(
        <div className="analysis-stack dynamics-analysis">
          <FormulaCard
            definition={<BlockMath math="\boldsymbol{\tau}=\mathbf M(\mathbf q)\ddot{\mathbf q}+\mathbf C(\mathbf q,\dot{\mathbf q})\dot{\mathbf q}+\mathbf g(\mathbf q)+\boldsymbol{\tau}_f" />}
            result={<BlockMath math={`\\boldsymbol{\\tau}=${vectorLatex(dynamics.tau)}\\;\\mathrm{N\\,m}`} />}
            substitution={<BlockMath math={`\\mathbf q=${vectorLatex(jointState.q)},\\quad\\dot{\\mathbf q}=${vectorLatex(jointState.qd)},\\quad\\ddot{\\mathbf q}=${vectorLatex(jointState.qdd)}`} />}
            symbols={[
              { symbol: '\\boldsymbol{\\tau}', meaning: '关节驱动力矩', unit: 'N·m' },
              { symbol: '\\mathbf q', meaning: '关节位置', unit: 'rad' },
            ]}
            title="机械臂动力学方程"
          />

          <section data-testid="mass-matrix-result">
            <FormulaCard
              definition={<BlockMath math="\mathbf M(\mathbf q)=\sum_i\left(m_i\mathbf J_{v_i}^{\mathsf T}\mathbf J_{v_i}+\mathbf J_{\omega_i}^{\mathsf T}{}^0\mathbf R_i{}^{c_i}\mathbf I_i{}^0\mathbf R_i^{\mathsf T}\mathbf J_{\omega_i}\right)" />}
              result={<BlockMath math={`\\mathbf M(\\mathbf q)=${matrixLatex(dynamics.massMatrix)}`} />}
              substitution={<BlockMath math={`\\mathbf M(\\mathbf q=${vectorLatex(jointState.q)})=${matrixLatex(dynamics.massMatrix)}\\;\\mathrm{kg\\,m^2}`} />}
              symbols={[
                { symbol: '\\mathbf M', meaning: '质量矩阵', unit: 'kg·m²' },
                { symbol: '\\mathbf J_{v_i}', meaning: '第 i 连杆质心线速度雅可比', unit: 'm' },
                { symbol: '\\mathbf J_{\\omega_i}', meaning: '第 i 连杆角速度雅可比' },
                { symbol: '{}^{c_i}\\mathbf I_i', meaning: '质心坐标系惯性张量', unit: 'kg·m²' },
              ]}
              title="质量矩阵 M(q)"
            />
            <MatrixTable label="质量矩阵" matrix={dynamics.massMatrix} precision={4} symbol={DISPLAY.massMatrix} unit="kg·m²" />
          </section>

          <FormulaCard
            definition={(
              <>
                <BlockMath math="C_{ij}=\sum_k\Gamma_{ijk}\dot q_k,\qquad\Gamma_{ijk}=\frac12\left(\frac{\partial M_{ij}}{\partial q_k}+\frac{\partial M_{ik}}{\partial q_j}-\frac{\partial M_{jk}}{\partial q_i}\right)" />
                <p>Christoffel 符号按精确定义构造；质量矩阵偏导采用中心差分，h=10⁻⁵ rad。</p>
              </>
            )}
            result={<BlockMath math={`\\mathbf C(\\mathbf q,\\dot{\\mathbf q})=${matrixLatex(dynamics.coriolisMatrix)}`} />}
            substitution={<BlockMath math={`\\mathbf C(${vectorLatex(jointState.q)},${vectorLatex(jointState.qd)})=${matrixLatex(dynamics.coriolisMatrix)}\\;\\mathrm{kg\\,m^2/s}`} />}
            symbols={[
              { symbol: '\\mathbf C', meaning: '科氏矩阵', unit: 'kg·m²/s' },
              { symbol: '\\Gamma_{ijk}', meaning: '第一类 Christoffel 符号', unit: 'kg·m²' },
              { symbol: '\\dot{\\mathbf q}', meaning: '关节角速度', unit: 'rad/s' },
            ]}
            title="科氏矩阵 C(q,q̇)"
          />

          <section data-testid="gravity-result">
            <output><InlineMath math="\mathbf g(\mathbf q)" /> = {vectorText(dynamics.gravityTorque)} N·m</output>
            <FormulaCard
              definition={<BlockMath math="\mathbf g(\mathbf q)=\frac{\partial V(\mathbf q)}{\partial\mathbf q}" />}
              result={<BlockMath math={`\\mathbf g(\\mathbf q)=${vectorLatex(dynamics.gravityTorque)}\\;\\mathrm{N\\,m}`} />}
              substitution={<BlockMath math={`\\mathbf g(${vectorLatex(jointState.q)})=${vectorLatex(dynamics.gravityTorque)}\\;\\mathrm{N\\,m}`} />}
              symbols={[
                { symbol: '\\mathbf g(\\mathbf q)', meaning: '重力广义力矩', unit: 'N·m' },
                { symbol: 'V', meaning: '重力势能', unit: 'J' },
              ]}
              title="重力项 g(q)"
            />
          </section>

          <section data-testid="friction-result">
            <output><InlineMath math="\boldsymbol{\tau}_f" /> = {vectorText(dynamics.frictionTorque)} N·m</output>
            <FormulaCard
              definition={<BlockMath math="\boldsymbol{\tau}_f=\mathbf B\dot{\mathbf q},\qquad\mathbf B=\operatorname{diag}(b_1,b_2,b_3)" />}
              result={<BlockMath math={`\\boldsymbol{\\tau}_f=${vectorLatex(dynamics.frictionTorque)}\\;\\mathrm{N\\,m}`} />}
              substitution={<BlockMath math={`\\operatorname{diag}(${parameters.viscousFriction.map((value) => value.toFixed(4)).join(',')})${vectorLatex(jointState.qd)}=${vectorLatex(dynamics.frictionTorque)}\\;\\mathrm{N\\,m}`} />}
              symbols={[
                { symbol: '\\boldsymbol{\\tau}_f', meaning: '粘性摩擦力矩', unit: 'N·m' },
                { symbol: '\\mathbf B', meaning: '粘性摩擦系数矩阵', unit: 'N·m·s/rad' },
              ]}
              title="粘性摩擦力矩"
            />
          </section>

          <FormulaCard
            definition={<BlockMath math="K=\frac12\dot{\mathbf q}^{\mathsf T}\mathbf M(\mathbf q)\dot{\mathbf q}" />}
            result={<BlockMath math={`K=${scalarText(energy.kinetic)}\\;\\mathrm J`} />}
            substitution={<BlockMath math={`K=\\frac12${vectorLatex(jointState.qd)}^{\\mathsf T}${matrixLatex(dynamics.massMatrix)}${vectorLatex(jointState.qd)}=${scalarText(energy.kinetic)}\\;\\mathrm J`} />}
            symbols={[
              { symbol: 'K', meaning: '系统动能', unit: 'J' },
              { symbol: '\\dot{\\mathbf q}', meaning: '关节角速度', unit: 'rad/s' },
            ]}
            title="动能"
          />
          <FormulaCard
            definition={<BlockMath math="V=-\sum_i m_i{}^0\mathbf g^{\mathsf T}{}^0\mathbf p_{C_i}" />}
            result={<BlockMath math={`V=${scalarText(energy.potential)}\\;\\mathrm J`} />}
            substitution={<BlockMath math={`V(\\mathbf q=${vectorLatex(jointState.q)})=${scalarText(energy.potential)}\\;\\mathrm J`} />}
            symbols={[
              { symbol: 'V', meaning: '系统重力势能', unit: 'J' },
              { symbol: '{}^0\\mathbf p_{C_i}', meaning: '第 i 连杆质心位置', unit: 'm' },
              { symbol: '{}^0\\mathbf g', meaning: '重力加速度', unit: 'm/s²' },
            ]}
            title="势能"
          />
          <FormulaCard
            definition={<BlockMath math="E=K+V" />}
            result={<BlockMath math={`E=${scalarText(energy.total)}\\;\\mathrm J`} />}
            substitution={<BlockMath math={`E=${scalarText(energy.kinetic)}+${scalarText(energy.potential)}=${scalarText(energy.total)}\\;\\mathrm J`} />}
            symbols={[
              { symbol: 'E', meaning: '总机械能', unit: 'J' },
              { symbol: 'K', meaning: '系统动能', unit: 'J' },
              { symbol: 'V', meaning: '系统重力势能', unit: 'J' },
            ]}
            title="总机械能"
          />
          <FormulaCard
            definition={<BlockMath math="\mathbf P=\boldsymbol{\tau}\odot\dot{\mathbf q}" />}
            result={<BlockMath math={`\\mathbf P=${vectorLatex(energy.jointPower)}\\;\\mathrm W`} />}
            substitution={<BlockMath math={`\\mathbf P=${vectorLatex(dynamics.tau)}\\odot${vectorLatex(jointState.qd)}=${vectorLatex(energy.jointPower)}\\;\\mathrm W`} />}
            symbols={[
              { symbol: '\\mathbf P', meaning: '各关节机械功率', unit: 'W' },
              { symbol: '\\boldsymbol{\\tau}', meaning: '关节驱动力矩', unit: 'N·m' },
              { symbol: '\\odot', meaning: '逐元素乘法' },
            ]}
            title="关节功率"
          />
        </div>
      )}
    />
  )
}
