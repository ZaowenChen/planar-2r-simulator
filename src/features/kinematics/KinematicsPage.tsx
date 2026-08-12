import { BlockMath, InlineMath } from 'react-katex'
import { WorkbenchLayout } from '../../app/WorkbenchLayout'
import { FormulaCard } from '../../components/FormulaCard'
import { MatrixTable } from '../../components/MatrixTable'
import { StatusBanner } from '../../components/StatusBanner'
import { RobotScene } from '../../scene/RobotScene'
import { useLabStore } from '../../state/labStore'
import { DISPLAY } from '../../symbols/display'
import { InverseKinematicsPanel } from './InverseKinematicsPanel'
import { JointControls } from './JointControls'

function vectorText(vector: readonly number[], precision = 3): string {
  return `[${vector.map((value) => value.toFixed(precision)).join(', ')}]`
}

function matrixLatex(matrix: readonly (readonly number[])[][] | readonly (readonly number[])[]): string {
  const rows = matrix as readonly (readonly number[])[]
  return `\\begin{bmatrix}${rows.map((row) => row.map((value) => value.toFixed(3)).join('&')).join('\\\\')}\\end{bmatrix}`
}

export function KinematicsPage() {
  const calculation = useLabStore((state) => state.calculation)
  const jointState = useLabStore((state) => state.jointState)
  const parameters = useLabStore((state) => state.parameters)
  const revision = String(calculation.revision)
  const transform = calculation.forward.transforms[2]
  const sceneCalculation = {
    forward: calculation.forward,
    jacobian: calculation.jacobian,
    jointState,
    torque: calculation.dynamics.tau,
    gravity: parameters.gravity,
  }
  const conditionNumber = Number.isFinite(calculation.singularity.conditionNumber)
    ? calculation.singularity.conditionNumber.toFixed(3)
    : '∞'

  return (
    <WorkbenchLayout
      visual={(
        <div data-revision={revision} data-testid="scene-result">
          <RobotScene calculation={sceneCalculation} />
        </div>
      )}
      controls={(
        <div className="control-sheet kinematics-controls">
          <p className="section-label">关节与目标</p>
          <h3>运动学输入</h3>
          <JointControls />
          <InverseKinematicsPanel />
        </div>
      )}
      analysis={(
        <div className="analysis-stack">
          <section data-revision={revision} data-testid="endpoint-result">
            <output><InlineMath math={DISPLAY.endEffectorPosition} /> = {vectorText(calculation.forward.endEffectorPosition)} m</output>
            <FormulaCard
              definition={<BlockMath math="{}^{0}\mathbf p_e={}^0\mathbf T_3\begin{bmatrix}0&0&0&1\end{bmatrix}^{\mathsf T}" />}
              result={<BlockMath math={`${DISPLAY.endEffectorPosition}=${vectorText(calculation.forward.endEffectorPosition)}`} />}
              substitution={<BlockMath math={`\\mathbf q=${vectorText(jointState.q)}`} />}
              title="正运动学末端位置"
            />
          </section>

          <section data-revision={revision} data-testid="transform-result">
            <FormulaCard
              definition={<BlockMath math="{}^0\mathbf T_3={}^0\mathbf T_1{}^1\mathbf T_2{}^2\mathbf T_3" />}
              result={<BlockMath math={`${DISPLAY.transform03}=${matrixLatex(transform)}`} />}
              substitution={<BlockMath math={`\\mathbf q=${vectorText(jointState.q)}`} />}
              title="齐次变换"
            />
            <MatrixTable label="基座到末端的齐次变换" matrix={transform} precision={3} symbol={DISPLAY.transform03} />
          </section>

          <section data-revision={revision} data-testid="jacobian-result">
            <FormulaCard
              definition={<BlockMath math="\dot{\mathbf x}=\mathbf J(\mathbf q)\dot{\mathbf q}" />}
              result={<BlockMath math={`\\mathbf J=${matrixLatex(calculation.jacobian)}`} />}
              substitution={<BlockMath math={`\\mathbf q=${vectorText(jointState.q)}`} />}
              title="几何雅可比"
            />
            <MatrixTable label="几何雅可比矩阵" matrix={calculation.jacobian} precision={3} symbol="\mathbf J(\mathbf q)" />
          </section>

          <StatusBanner
            tone={calculation.singularity.isSingular ? 'warning' : 'success'}
            title={calculation.singularity.isSingular ? '接近奇异位形' : '远离奇异位形'}
          >
            最小奇异值 σ<sub>min</sub> = {calculation.singularity.minimumSingularValue.toExponential(3)}；条件数 κ = {conditionNumber}
          </StatusBanner>
        </div>
      )}
    />
  )
}
