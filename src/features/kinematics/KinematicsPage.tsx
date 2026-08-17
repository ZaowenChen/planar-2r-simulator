import { WorkbenchLayout } from '../../app/WorkbenchLayout'
import { RobotScene } from '../../scene/RobotScene'
import { useLabStore } from '../../state/labStore'
import { InverseKinematicsPanel } from './InverseKinematicsPanel'
import { JointControls } from './JointControls'
import { KinematicsWalkthrough } from './KinematicsWalkthrough'

export function KinematicsPage() {
  const calculation = useLabStore((state) => state.calculation)
  const jointState = useLabStore((state) => state.jointState)
  const parameters = useLabStore((state) => state.parameters)
  const desiredPosition = useLabStore((state) => state.desiredPosition)
  const angleUnit = useLabStore((state) => state.angleUnit)
  const revision = String(calculation.revision)
  const sceneCalculation = {
    forward: calculation.forward,
    jacobian: calculation.jacobian,
    jointState,
    torque: calculation.dynamics.tau,
    gravity: parameters.gravity,
  }
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
          <KinematicsWalkthrough
            angleUnit={angleUnit}
            parameters={parameters}
            q={jointState.q}
            revision={calculation.revision}
            singularity={calculation.singularity}
            target={desiredPosition}
          />
        </div>
      )}
    />
  )
}
