# Interactive 3R Robotics Lab Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the current Matplotlib demonstration with a public, browser-based Chinese 3R robotics laboratory whose 3D pose, academic formulas, kinematics, dynamics, trajectories, and time histories stay synchronized.

**Architecture:** Keep every robotics calculation in framework-independent TypeScript modules and expose one typed result graph to the React UI. A Zustand store owns parameters, pose, experiment configuration, and simulation time; React Three Fiber, KaTeX, and Plotly render that shared state without recalculating the model independently.

**Tech Stack:** React 19, TypeScript 5.9, Vite 6, Three.js/React Three Fiber, KaTeX, Plotly, Zustand, ml-matrix, Vitest, React Testing Library, Playwright, GitHub Actions, GitHub Pages.

## Global Constraints

- Model only the existing spatial yaw–pitch–pitch 3R arm; do not add URDF import or another robot.
- Use Chinese explanatory copy and international LaTeX notation for every user-visible mathematical symbol.
- Never render source-style labels such as `x_target`, `P_B^T`, `T_0^3`, or `a_(i-1)` in the UI or exported column names.
- Use SI units and radians internally; degree display is an explicit presentation option only.
- Implement both inverse dynamics (trajectory to torque) and forward dynamics (torque to state).
- Keep geometry, mass, center of mass, inertia, gravity, and viscous friction editable and validated.
- Keep the application static and backend-free so it can run on GitHub Pages.
- Preserve the Python 2R and 3R programs under `legacy/` as references, not as the primary entry point.
- Use Node.js 20.15 or newer with Vite 6.4.x; do not upgrade the project to Vite 8 while the workspace runtime remains Node 20.15.
- Follow test-driven development for every calculation or behavior change.

---

## File Structure

```text
.
├── .github/workflows/ci-pages.yml
├── docs/
│   ├── mathematics.md
│   ├── symbols.md
│   └── superpowers/{plans,specs}/...
├── e2e/robotics-lab.spec.ts
├── legacy/
│   ├── artifacts/{planar_2r_preview.png,robot_3r_3d_preview.png}
│   └── python/{planar_2r_simulator.py,robot_3r_3d_simulator.py,test_planar_2r_simulator.py,test_robot_3r_3d_simulator.py}
├── src/
│   ├── app/{App.tsx,Navigation.tsx,WorkbenchLayout.tsx,app.css}
│   ├── components/{FormulaCard.tsx,MatrixTable.tsx,NumericField.tsx,StatusBanner.tsx}
│   ├── export/{csv.ts,download.ts}
│   ├── features/dynamics/{DynamicsPage.tsx,RigidBodyEditor.tsx}
│   ├── features/experiments/{ExperimentPage.tsx,SimulationControls.tsx,TimeSeriesCharts.tsx,useAnimationClock.ts}
│   ├── features/kinematics/{InverseKinematicsPanel.tsx,KinematicsPage.tsx,JointControls.tsx}
│   ├── features/model/{DhTable.tsx,RobotModelPage.tsx}
│   ├── robotics/{defaults.ts,dynamics.ts,integration.ts,jacobian.ts,kinematics.ts,linalg.ts,trajectories.ts,transforms.ts,types.ts,validation.ts,workspace.ts}
│   ├── scene/{CoordinateFrame.tsx,RobotScene.tsx,SceneOverlays.tsx,sceneModel.ts}
│   ├── state/labStore.ts
│   ├── symbols/display.ts
│   ├── test/setup.ts
│   └── main.tsx
├── package.json
├── playwright.config.ts
├── tsconfig.json
├── vite.config.ts
└── vitest.config.ts
```

---

### Task 1: Establish the Web Baseline and Preserve the Python Reference

**Files:**
- Create: `package.json`
- Create: `index.html`
- Create: `tsconfig.json`
- Create: `vite.config.ts`
- Create: `vitest.config.ts`
- Create: `src/main.tsx`
- Create: `src/app/App.tsx`
- Create: `src/test/setup.ts`
- Create: `src/app/App.test.tsx`
- Create: `legacy/__init__.py`
- Create: `legacy/python/__init__.py`
- Move: `planar_2r_simulator.py` → `legacy/python/planar_2r_simulator.py`
- Move: `test_planar_2r_simulator.py` → `legacy/python/test_planar_2r_simulator.py`
- Import from branch and move: `robot_3r_3d_simulator.py`, `test_robot_3r_3d_simulator.py`, `artifacts/robot_3r_3d_preview.png`

**Interfaces:**
- Produces: `npm run dev`, `npm test`, `npm run typecheck`, and `npm run build` as stable project commands.
- Produces: a React root with the heading `空间 3R 机器人学交互实验室`.

- [ ] **Step 1: Integrate the completed 3R reference history without publishing it yet**

Run:

```bash
git merge --no-commit --no-ff codex/robot-3r-3d-simulator
mkdir -p legacy/python legacy/artifacts
git mv planar_2r_simulator.py legacy/python/planar_2r_simulator.py
git mv test_planar_2r_simulator.py legacy/python/test_planar_2r_simulator.py
git mv robot_3r_3d_simulator.py legacy/python/robot_3r_3d_simulator.py
git mv test_robot_3r_3d_simulator.py legacy/python/test_robot_3r_3d_simulator.py
git mv artifacts/planar_2r_preview.png legacy/artifacts/planar_2r_preview.png
git mv artifacts/robot_3r_3d_preview.png legacy/artifacts/robot_3r_3d_preview.png
```

Update the two Python test modules so their imports use `legacy.python.planar_2r_simulator` and `legacy.python.robot_3r_3d_simulator`. Add empty `legacy/__init__.py` and `legacy/python/__init__.py` with `apply_patch`.

- [ ] **Step 2: Write the failing React smoke test**

```tsx
// src/app/App.test.tsx
import { render, screen } from '@testing-library/react'
import { describe, expect, it } from 'vitest'
import { App } from './App'

describe('App', () => {
  it('names the laboratory and exposes its four learning modules', () => {
    render(<App />)
    expect(screen.getByRole('heading', { name: '空间 3R 机器人学交互实验室' })).toBeInTheDocument()
    for (const label of ['机器人模型', '运动学', '动力学', '动态实验']) {
      expect(screen.getByRole('button', { name: label })).toBeInTheDocument()
    }
  })
})
```

- [ ] **Step 3: Add pinned, Node-compatible tooling**

Create `package.json` with this command surface and dependency set:

```json
{
  "name": "interactive-3r-robotics-lab",
  "private": true,
  "version": "1.0.0",
  "type": "module",
  "engines": { "node": ">=20.15.0" },
  "scripts": {
    "dev": "vite",
    "build": "tsc -b && vite build",
    "preview": "vite preview",
    "test": "vitest run",
    "test:watch": "vitest",
    "typecheck": "tsc -b --pretty false",
    "e2e": "playwright test"
  },
  "dependencies": {
    "@react-three/drei": "^10.7.8",
    "@react-three/fiber": "^9.7.0",
    "katex": "^0.18.0",
    "ml-matrix": "^6.15.0",
    "plotly.js": "^3.7.0",
    "react": "^19.2.0",
    "react-dom": "^19.2.0",
    "react-katex": "^3.1.0",
    "react-plotly.js": "^4.1.0",
    "three": "^0.185.0",
    "zustand": "^5.0.14"
  },
  "devDependencies": {
    "@playwright/test": "^1.62.1",
    "@testing-library/jest-dom": "^6.9.1",
    "@testing-library/react": "^16.3.2",
    "@testing-library/user-event": "^14.6.1",
    "@types/katex": "^0.16.7",
    "@types/node": "^20.19.0",
    "@types/plotly.js": "^3.0.13",
    "@types/react": "^19.2.0",
    "@types/react-dom": "^19.2.0",
    "@types/react-plotly.js": "^2.6.4",
    "@types/three": "^0.185.0",
    "@vitejs/plugin-react": "^4.7.0",
    "jsdom": "^26.1.0",
    "typescript": "^5.9.3",
    "vite": "^6.4.3",
    "vitest": "^3.2.4"
  }
}
```

Run `npm install` and commit the generated `package-lock.json`.

- [ ] **Step 4: Implement the minimum accessible shell**

```tsx
// src/app/App.tsx
const modules = ['机器人模型', '运动学', '动力学', '动态实验'] as const

export function App() {
  return (
    <main>
      <header><p>ROBOTICS LAB · 3R</p><h1>空间 3R 机器人学交互实验室</h1></header>
      <nav aria-label="学习模块">
        {modules.map((label) => <button key={label} type="button">{label}</button>)}
      </nav>
    </main>
  )
}
```

Configure Vitest for `jsdom` and load `@testing-library/jest-dom/vitest` from `src/test/setup.ts`.

- [ ] **Step 5: Verify the baseline**

Run:

```bash
npm test -- src/app/App.test.tsx
npm run typecheck
npm run build
python3 -m unittest discover -s legacy/python -p 'test_*.py'
```

Expected: the React smoke test, typecheck, production build, and both legacy Python suites pass.

- [ ] **Step 6: Commit**

```bash
git add package.json package-lock.json index.html tsconfig.json vite.config.ts vitest.config.ts src legacy docs
git commit -m "chore: establish 3r robotics web lab"
```

---

### Task 2: Define Scientific Types, Linear Algebra, Defaults, and Validation

**Files:**
- Create: `src/robotics/types.ts`
- Create: `src/robotics/linalg.ts`
- Create: `src/robotics/defaults.ts`
- Create: `src/robotics/validation.ts`
- Test: `src/robotics/linalg.test.ts`
- Test: `src/robotics/validation.test.ts`

**Interfaces:**
- Produces: `Vector3`, `Matrix3`, `Matrix4`, `RobotParameters`, `JointState`, `ValidationIssue`.
- Produces: `DEFAULT_ROBOT_PARAMETERS: RobotParameters`.
- Produces: `DEFAULT_JOINT_STATE: JointState` with \(\mathbf q=[30^\circ,25^\circ,-50^\circ]^{\mathsf T}\) converted to radians and zero velocity/acceleration.
- Produces: `validateRobotParameters(parameters): ValidationIssue[]`.
- Produces: small-matrix operations used by every later robotics module.

- [ ] **Step 1: Write failing type-level and numerical behavior tests**

```ts
// src/robotics/validation.test.ts
import { describe, expect, it } from 'vitest'
import { DEFAULT_ROBOT_PARAMETERS } from './defaults'
import { validateRobotParameters } from './validation'

describe('robot parameter validation', () => {
  it('accepts the documented teaching parameters', () => {
    expect(validateRobotParameters(DEFAULT_ROBOT_PARAMETERS)).toEqual([])
  })

  it('identifies a nonphysical inertia tensor by field and link', () => {
    const invalid = structuredClone(DEFAULT_ROBOT_PARAMETERS)
    invalid.links[1].inertia = [[0.01, 0, 0], [0, 5, 0], [0, 0, 0.01]]
    expect(validateRobotParameters(invalid)).toContainEqual(expect.objectContaining({
      path: 'links.1.inertia', code: 'INERTIA_TRIANGLE'
    }))
  })

  it('rejects a center of mass outside its nominal link sphere', () => {
    const invalid = structuredClone(DEFAULT_ROBOT_PARAMETERS)
    invalid.links[2].centerOfMass = [-2, 0, 0]
    expect(validateRobotParameters(invalid)).toContainEqual(expect.objectContaining({
      path: 'links.2.centerOfMass', code: 'CENTER_OF_MASS_RANGE'
    }))
  })
})
```

Add `linalg.test.ts` cases for identity multiplication, transpose, cross product, determinant, inverse, symmetric eigenvalues, and `nearlyEqual`.

- [ ] **Step 2: Run the tests and confirm missing-module failures**

Run: `npm test -- src/robotics/linalg.test.ts src/robotics/validation.test.ts`

Expected: FAIL because the robotics type, default, algebra, and validation modules do not exist.

- [ ] **Step 3: Implement stable tuple types and defaults**

```ts
// src/robotics/types.ts
export type Vector3 = readonly [number, number, number]
export type Matrix3 = readonly [Vector3, Vector3, Vector3]
export type Matrix4 = readonly [
  readonly [number, number, number, number],
  readonly [number, number, number, number],
  readonly [number, number, number, number],
  readonly [number, number, number, number]
]

export interface LinkParameters {
  mass: number
  centerOfMass: Vector3
  inertia: Matrix3
}

export interface RobotParameters {
  geometry: { d1: number; l2: number; l3: number }
  links: readonly [LinkParameters, LinkParameters, LinkParameters]
  gravity: Vector3
  viscousFriction: Vector3
  frictionEnabled: boolean
  jointLimits: readonly [readonly [number, number], readonly [number, number], readonly [number, number]]
}

export interface JointState { q: Vector3; qd: Vector3; qdd: Vector3 }
export interface ValidationIssue { path: string; code: string; message: string }
```

Encode the exact geometry, masses, centers of mass, inertias, gravity, friction, and joint limits from the approved specification in `defaults.ts`. Freeze the exported default object in development to catch accidental mutation.

- [ ] **Step 4: Implement focused linear-algebra wrappers**

Use immutable arrays at public boundaries and `ml-matrix` only inside `linalg.ts`. Implement `add3`, `subtract3`, `scale3`, `dot3`, `cross3`, `norm3`, `matMul`, `matVec`, `transpose`, `determinant3`, `inverse3`, `symmetricEigenvalues3`, `frobeniusNorm`, and `nearlyEqual`. Throw descriptive errors for dimension or singularity failures.

- [ ] **Step 5: Implement exact validation rules**

Validate finite positive geometry and masses, finite nonnegative base height and friction, center-of-mass norm bounds, symmetry to `1e-10`, minimum inertia eigenvalue above `1e-9`, and principal-inertia triangle inequalities to `1e-9`. Return every field issue in one pass.

- [ ] **Step 6: Verify and commit**

Run:

```bash
npm test -- src/robotics/linalg.test.ts src/robotics/validation.test.ts
npm run typecheck
git add src/robotics
git commit -m "feat: define validated 3r robot parameters"
```

Expected: all new tests and the project typecheck pass.

---

### Task 3: Implement D–H Transforms and Forward Kinematics

**Files:**
- Create: `src/robotics/transforms.ts`
- Create: `src/robotics/kinematics.ts`
- Test: `src/robotics/transforms.test.ts`
- Test: `src/robotics/kinematics.test.ts`

**Interfaces:**
- Produces: `dhTransform(theta, a, alpha, d): Matrix4`.
- Produces: `forwardKinematics(q, parameters): ForwardKinematicsResult`.
- Produces: transforms, origins, base-frame joint axes, end-effector position, and link center-of-mass positions.

- [ ] **Step 1: Write failing hand-derived transform and pose tests**

```ts
it('reproduces the general nonzero D–H reference transform', () => {
  expectMatrixClose(dhTransform(Math.PI / 2, 2, Math.PI / 2, 3), [
    [0, 0, 1, 0], [1, 0, 0, 2], [0, 1, 0, 3], [0, 0, 0, 1]
  ], 1e-12)
})

it('places the straight arm at the analytical endpoint', () => {
  const result = forwardKinematics([0, 0, 0], DEFAULT_ROBOT_PARAMETERS)
  expectVectorClose(result.endEffectorPosition, [3.5, 0, 0.8], 1e-12)
  expect(result.transforms).toHaveLength(4)
  expect(result.origins).toEqual(expect.arrayContaining([[0, 0, 0], [0, 0, 0.8]]))
})
```

Also compare 12 deterministic poses against the existing Python implementation by storing their expected endpoint values directly in the test table.

- [ ] **Step 2: Verify the tests fail for missing exports**

Run: `npm test -- src/robotics/transforms.test.ts src/robotics/kinematics.test.ts`

Expected: FAIL because `dhTransform` and `forwardKinematics` are not implemented.

- [ ] **Step 3: Implement transformations without React or Three.js dependencies**

Implement `rotationX`, `rotationZ`, `translation`, `multiply4`, `transformPoint`, `rotationOf`, and `translationOf`. Clean values with magnitude below `1e-14` to zero only when formatting, never during calculation.

- [ ] **Step 4: Implement the fixed 3R transform chain**

```ts
export interface ForwardKinematicsResult {
  transforms: readonly [Matrix4, Matrix4, Matrix4, Matrix4]
  origins: readonly [Vector3, Vector3, Vector3, Vector3]
  jointAxes: readonly [Vector3, Vector3, Vector3]
  centerOfMassPositions: readonly [Vector3, Vector3, Vector3]
  endEffectorPosition: Vector3
}
```

Build `T01`, `T02`, `T03`, and `T0e`; derive each revolute axis from the preceding frame's Z column and each center of mass by transforming its local vector with the link frame.

- [ ] **Step 5: Verify invariants and commit**

Run:

```bash
npm test -- src/robotics/transforms.test.ts src/robotics/kinematics.test.ts
npm run typecheck
git add src/robotics
git commit -m "feat: add 3r forward kinematics core"
```

Expected: pose references pass; every rotation block satisfies `RᵀR = I` and `det(R) = 1` within `1e-10`.

---

### Task 4: Add Closed-Form IK, Jacobians, Singularity Metrics, and Workspace

**Files:**
- Create: `src/robotics/jacobian.ts`
- Create: `src/robotics/workspace.ts`
- Modify: `src/robotics/types.ts`
- Modify: `src/robotics/kinematics.ts`
- Test: `src/robotics/inverseKinematics.test.ts`
- Test: `src/robotics/jacobian.test.ts`
- Test: `src/robotics/workspace.test.ts`

**Interfaces:**
- Produces: `inverseKinematics(target, parameters, referenceQ?): InverseKinematicsResult`.
- Produces: `geometricJacobian(q, parameters): Matrix6x3`.
- Produces: `singularityMetrics(jacobian, threshold): SingularityMetrics`.
- Produces: `sampleWorkspace(parameters, counts): Vector3[]`.

- [ ] **Step 1: Write failing round-trip, finite-difference, and singularity tests**

```ts
it.each([
  [1.8, 0.8, 1.4], [2.2, -0.7, 0.2], [0.5, 0.5, 3.0]
])('returns every valid position branch for target %j', (...target) => {
  const result = inverseKinematics(target as Vector3, DEFAULT_ROBOT_PARAMETERS)
  for (const solution of result.solutions) {
    expectVectorClose(
      forwardKinematics(solution.q, DEFAULT_ROBOT_PARAMETERS).endEffectorPosition,
      target as Vector3,
      1e-8
    )
  }
})

it('matches translational Jacobian columns to position finite differences', () => {
  const q: Vector3 = [0.4, -0.2, 0.7]
  const jacobian = geometricJacobian(q, DEFAULT_ROBOT_PARAMETERS)
  expectJacobianToMatchFiniteDifference(jacobian, q, 1e-6, 2e-6)
})
```

Test unreachable, boundary, base-axis singular, joint-limit, straight-arm singular, and a workspace sample count with finite coordinates.

- [ ] **Step 2: Run and observe missing-feature failures**

Run: `npm test -- src/robotics/inverseKinematics.test.ts src/robotics/jacobian.test.ts src/robotics/workspace.test.ts`

- [ ] **Step 3: Port the proven two-branch IK behavior**

Use the approved equations for `r`, `z′`, and `D`; clamp only floating-point excursions within `1e-10`; preserve both radial families so folded poses can be selected relative to `referenceQ`. Return explicit statuses `reachable`, `unreachable`, `axis-singular`, and `joint-limit` instead of throwing for normal user targets.

Add `Matrix6x3` to `types.ts` as a six-row tuple of `Vector3` rows so the Jacobian interface is defined before any consumer imports it.

- [ ] **Step 4: Build the geometric Jacobian from frame geometry**

For revolute joint `j`, use `Jv_j = z_{j-1} × (p_e - o_{j-1})` and `Jω_j = z_{j-1}`. Compute SVD with `ml-matrix`; return sorted singular values, minimum singular value, condition number, and the configured singular flag.

- [ ] **Step 5: Add deterministic workspace sampling**

Sample the three joint ranges on a configurable tensor grid, defaulting to `30 × 16 × 18`; de-duplicate positions rounded to `1e-5` only for rendering payload size.

- [ ] **Step 6: Verify and commit**

```bash
npm test -- src/robotics/inverseKinematics.test.ts src/robotics/jacobian.test.ts src/robotics/workspace.test.ts
npm run typecheck
git add src/robotics
git commit -m "feat: add inverse and velocity kinematics"
```

---

### Task 5: Implement Full Rigid-Body Dynamics

**Files:**
- Create: `src/robotics/dynamics.ts`
- Test: `src/robotics/dynamics.test.ts`

**Interfaces:**
- Produces: `massMatrix(q, parameters): Matrix3`.
- Produces: `coriolisMatrix(q, qd, parameters, derivativeStep?): Matrix3`.
- Produces: `gravityVector(q, parameters): Vector3`.
- Produces: `energy(state, parameters): EnergyResult`.
- Produces: `inverseDynamics(state, parameters): DynamicsResult`.
- Produces: `forwardDynamics(q, qd, tau, parameters): Vector3`.

- [ ] **Step 1: Write failing physical-invariant tests**

```ts
it('builds a symmetric positive-definite mass matrix', () => {
  const matrix = massMatrix([0.3, -0.4, 0.8], DEFAULT_ROBOT_PARAMETERS)
  expectMatrixClose(matrix, transpose(matrix), 1e-10)
  expect(Math.min(...symmetricEigenvalues3(matrix))).toBeGreaterThan(1e-9)
})

it('satisfies the skew-symmetry identity', () => {
  const q: Vector3 = [0.2, -0.3, 0.6]
  const qd: Vector3 = [0.4, -0.2, 0.3]
  const mDot = directionalMassDerivative(q, qd, DEFAULT_ROBOT_PARAMETERS, 1e-5)
  const c = coriolisMatrix(q, qd, DEFAULT_ROBOT_PARAMETERS)
  expect(frobeniusNorm(addMatrices(subtractMatrices(mDot, scaleMatrix(c, 2)), transpose(subtractMatrices(mDot, scaleMatrix(c, 2)))))).toBeLessThan(1e-6)
})

it('round-trips acceleration through inverse and forward dynamics', () => {
  const state = { q: [0.3, -0.2, 0.5], qd: [0.1, 0.2, -0.1], qdd: [0.4, -0.3, 0.2] } as JointState
  const tau = inverseDynamics(state, DEFAULT_ROBOT_PARAMETERS).tau
  expectVectorClose(forwardDynamics(state.q, state.qd, tau, DEFAULT_ROBOT_PARAMETERS), state.qdd, 1e-7)
})
```

Also test zero-gravity behavior, disabled friction, potential-energy gradient, positive kinetic energy, and exact per-joint power `tau_i * qd_i`.

- [ ] **Step 2: Confirm the dynamics test fails before implementation**

Run: `npm test -- src/robotics/dynamics.test.ts`

- [ ] **Step 3: Implement center-of-mass Jacobians and the mass matrix**

For link `i`, zero every Jacobian column with joint index greater than `i`; for active revolute columns use the same cross-product rule as the end-effector Jacobian. Sum translational and rotated-inertia terms exactly as specified.

- [ ] **Step 4: Implement potential, gravity, and Christoffel evaluation**

Compute `V = -Σ m_i gᵀp_ci` and `g(q) = -Σ m_i Jv_iᵀg`. Evaluate all three partial mass derivatives with central differences at `1e-5 rad`, then assemble `C_ij = Σ c_ijk qd_k`. Export the derivative helper only for invariant tests.

- [ ] **Step 5: Implement inverse and forward dynamics with diagnostics**

```ts
export interface DynamicsResult {
  tau: Vector3
  massMatrix: Matrix3
  coriolisMatrix: Matrix3
  coriolisTorque: Vector3
  gravityTorque: Vector3
  frictionTorque: Vector3
  conditionNumber: number
}
```

Before solving forward dynamics, reject `λmin(M) ≤ 1e-9` or `cond(M) > 1e10` with a `DynamicsError` containing a Chinese diagnostic key and numerical value.

- [ ] **Step 6: Verify and commit**

```bash
npm test -- src/robotics/dynamics.test.ts
npm run typecheck
git add src/robotics/dynamics.ts src/robotics/dynamics.test.ts
git commit -m "feat: implement complete 3r rigid body dynamics"
```

---

### Task 6: Add Trajectories, Torque Profiles, and Time Integration

**Files:**
- Create: `src/robotics/trajectories.ts`
- Create: `src/robotics/integration.ts`
- Test: `src/robotics/trajectories.test.ts`
- Test: `src/robotics/integration.test.ts`

**Interfaces:**
- Produces: `quinticTrajectory(config, time): TrajectorySample`.
- Produces: `sinusoidalTrajectory(config, time): TrajectorySample`.
- Produces: `evaluateTorqueProfile(profile, time): Vector3`.
- Produces: `simulateInverseDynamics(config, parameters): SimulationSample[]`.
- Produces: `simulateForwardDynamics(config, parameters): SimulationSample[]`.

```ts
export interface TrajectorySample { q: Vector3; qd: Vector3; qdd: Vector3 }
export interface SimulationSample extends TrajectorySample {
  time: number
  tau: Vector3
  kinetic: number
  potential: number
  totalEnergy: number
  jointPower: Vector3
}
```

- [ ] **Step 1: Write failing endpoint, profile, convergence, and consistency tests**

```ts
it('meets all zero-velocity quintic endpoint constraints', () => {
  const config = { q0: [0, 0.2, -0.3], qf: [1, -0.4, 0.5], duration: 2 } as const
  expect(quinticTrajectory(config, 0)).toMatchObject({ q: config.q0, qd: [0, 0, 0], qdd: [0, 0, 0] })
  expect(quinticTrajectory(config, 2)).toMatchObject({ q: config.qf, qd: [0, 0, 0], qdd: [0, 0, 0] })
})

it('shows fourth-order RK convergence on y′ = y', () => {
  const errors = [0.2, 0.1, 0.05].map((h) => Math.abs(integrateScalarRk4(1, 0, 1, h, (y) => y) - Math.E))
  expect(errors[0] / errors[1]).toBeGreaterThan(12)
  expect(errors[1] / errors[2]).toBeGreaterThan(12)
})
```

Add a two-second unforced, frictionless energy test at `0.001 s` requiring relative drift below `0.005`, and a test that inverse-dynamics torque reproduces the prescribed acceleration through `forwardDynamics` at every tenth sample.

- [ ] **Step 2: Run and confirm expected failures**

Run: `npm test -- src/robotics/trajectories.test.ts src/robotics/integration.test.ts`

- [ ] **Step 3: Implement deterministic trajectory and torque evaluators**

Use the normalized quintic blend `s(u)=10u³−15u⁴+6u⁵` with analytical first and second derivatives. Implement sine trajectories and constant, step, sine, and piecewise-constant torque profiles as discriminated unions; clamp evaluation time to the configured experiment interval.

- [ ] **Step 4: Implement RK4 over the six-dimensional joint state**

Represent the integrated state as `[q1,q2,q3,qd1,qd2,qd3]`; evaluate forward dynamics at all four RK stages. Stop and return a structured diagnostic before appending any non-finite sample or a sample outside configured joint limits.

- [ ] **Step 5: Implement both simulation pipelines**

Each `SimulationSample` contains `time`, `q`, `qd`, `qdd`, `tau`, `kinetic`, `potential`, `totalEnergy`, and `jointPower`. Inverse dynamics samples the chosen trajectory; forward dynamics integrates the chosen torque profile. Include the exact final time even when duration is not divisible by step size.

- [ ] **Step 6: Verify and commit**

```bash
npm test -- src/robotics/trajectories.test.ts src/robotics/integration.test.ts
npm run typecheck
git add src/robotics
git commit -m "feat: add trajectory and dynamics simulations"
```

---

### Task 7: Build the Shared Store, Academic Shell, and Formula Components

**Files:**
- Create: `src/state/labStore.ts`
- Create: `src/symbols/display.ts`
- Create: `src/app/Navigation.tsx`
- Create: `src/app/WorkbenchLayout.tsx`
- Create: `src/app/app.css`
- Create: `src/components/FormulaCard.tsx`
- Create: `src/components/MatrixTable.tsx`
- Create: `src/components/NumericField.tsx`
- Create: `src/components/StatusBanner.tsx`
- Modify: `src/app/App.tsx`
- Test: `src/state/labStore.test.ts`
- Test: `src/components/FormulaCard.test.tsx`
- Test: `src/app/App.test.tsx`

**Interfaces:**
- Produces: `useLabStore` with parameter, pose, navigation, experiment, and simulation-time actions.
- Produces: `calculateLabState(parameters, jointState): LabCalculation`, the only composition function that invokes kinematics, Jacobian, and instantaneous dynamics together.
- Produces: `FormulaCard({title, definition, substitution, result, symbols})`.
- Produces: consistent responsive workbench layout and design tokens.

- [ ] **Step 1: Write failing store synchronization and formula-tab tests**

```ts
it('recomputes one shared result graph when a joint changes', () => {
  const before = useLabStore.getState().calculation.revision
  useLabStore.getState().setJoint(1, 0.25)
  const after = useLabStore.getState()
  expect(after.calculation.revision).toBe(before + 1)
  expect(after.calculation.forward.q[1]).toBe(0.25)
  expect(after.calculation.dynamics.massMatrix).toBeDefined()
})
```

Render `FormulaCard`, switch among “定义”“代入”“结果”, and assert that only the selected panel is exposed to assistive technology.

- [ ] **Step 2: Run and observe the missing store/component failures**

Run: `npm test -- src/state/labStore.test.ts src/components/FormulaCard.test.tsx src/app/App.test.tsx`

- [ ] **Step 3: Implement the store as the single calculation owner**

Store raw editable fields separately from the last valid `RobotParameters`. On successful validation, call one `calculateLabState(parameters, jointState)` function that returns forward kinematics, Jacobian metrics, and dynamics. Invalid edits preserve the last valid calculation and expose field issues.

- [ ] **Step 4: Centralize visible symbols**

```ts
export const DISPLAY = {
  endEffectorPosition: String.raw`{}^{0}\mathbf{p}_{e}`,
  desiredPosition: String.raw`{}^{0}\mathbf{p}_{d}`,
  transform03: String.raw`{}^{0}\mathbf{T}_{3}`,
  massMatrix: String.raw`\mathbf{M}(\mathbf{q})`,
  coriolisMatrix: String.raw`\mathbf{C}(\mathbf{q},\dot{\mathbf{q}})`,
  gravityVector: String.raw`\mathbf{g}(\mathbf{q})`,
  jointTorque: String.raw`\boldsymbol{\tau}`,
} as const
```

Use these constants in formula, table, control, and export components. Add rendered integration tests that visit every module and generate one CSV, then fail if the visible document or exported column names contain any of the four banned source-style labels.

- [ ] **Step 5: Implement the visual system and shell**

Use a warm off-white canvas, navy ink, teal analytical accents, amber warnings, fine ruled borders, tabular numerals, and a Chinese serif face for section titles. Keep controls accessible at 44 px minimum height and provide visible focus styles. The top navigation must remain usable at 1024 px; below that width, stack the three workbench columns for reading.

- [ ] **Step 6: Verify and commit**

```bash
npm test -- src/state/labStore.test.ts src/components/FormulaCard.test.tsx src/app/App.test.tsx
npm run typecheck
git add src/app src/components src/state src/symbols
git commit -m "feat: add synchronized academic lab shell"
```

---

### Task 8: Render the Interactive 3D Robot and Scientific Overlays

**Files:**
- Create: `src/scene/sceneModel.ts`
- Create: `src/scene/CoordinateFrame.tsx`
- Create: `src/scene/SceneOverlays.tsx`
- Create: `src/scene/RobotScene.tsx`
- Test: `src/scene/sceneModel.test.ts`
- Test: `src/scene/RobotScene.test.tsx`

**Interfaces:**
- Consumes: `ForwardKinematicsResult`, joint state, workspace samples, and overlay flags.
- Produces: `buildSceneModel(calculation): SceneModel` for deterministic visual geometry.
- Produces: `<RobotScene />` with orbit, zoom, reset, and overlay toggles.

- [ ] **Step 1: Write failing scene-model tests**

Verify that four joint markers match FK origins, each link midpoint and length match adjacent origins, coordinate frames use transform rotation columns, center-of-mass markers use the dynamics positions, and a torque arrow uses the corresponding joint axis and signed magnitude.

- [ ] **Step 2: Run and confirm missing scene-model failures**

Run: `npm test -- src/scene/sceneModel.test.ts src/scene/RobotScene.test.tsx`

- [ ] **Step 3: Implement a pure scene view model**

The view model must contain no Three.js objects. Return serializable positions, quaternions, lengths, colors, labels, and visibility flags so numerical scene correctness is unit-testable without WebGL.

- [ ] **Step 4: Implement the React Three Fiber scene**

Render cylinders between joint origins, spheres at joints, compact RGB coordinate axes, labeled center-of-mass markers, a low-opacity workspace point cloud, end-effector trail, grid, and shadow-free studio lighting. Use `OrbitControls` with damping and a reset button that restores the documented camera.

In `RobotScene.test.tsx`, mock the React Three Fiber `Canvas` boundary and assert the serialized scene model passed to child renderers; reserve real WebGL rendering for Playwright Chromium.

- [ ] **Step 5: Add optional scientific vector overlays**

Use separate toggles for linear velocity, acceleration, gravity, and joint torque. Normalize arrow length for readability while displaying the true magnitude and unit in an adjacent HTML legend; do not imply screen length is a one-to-one physical scale.

- [ ] **Step 6: Verify and commit**

```bash
npm test -- src/scene/sceneModel.test.ts src/scene/RobotScene.test.tsx
npm run typecheck
git add src/scene
git commit -m "feat: render interactive 3r robot scene"
```

---

### Task 9: Deliver the Robot Model and Kinematics Learning Modules

**Files:**
- Create: `src/features/model/DhTable.tsx`
- Create: `src/features/model/RobotModelPage.tsx`
- Create: `src/features/kinematics/JointControls.tsx`
- Create: `src/features/kinematics/InverseKinematicsPanel.tsx`
- Create: `src/features/kinematics/KinematicsPage.tsx`
- Modify: `src/app/App.tsx`
- Test: `src/features/model/RobotModelPage.test.tsx`
- Test: `src/features/kinematics/KinematicsPage.test.tsx`

**Interfaces:**
- Consumes: the shared store and `RobotScene`.
- Produces: complete model/FK/IK/Jacobian interactions using standard notation.

- [ ] **Step 1: Write failing user-flow tests**

```tsx
it('updates the endpoint, transform, Jacobian, and scene revision from θ₂', async () => {
  render(<KinematicsPage />)
  const before = screen.getByTestId('endpoint-result').getAttribute('data-revision')
  await userEvent.clear(screen.getByLabelText('关节角 θ₂'))
  await userEvent.type(screen.getByLabelText('关节角 θ₂'), '30')
  expect(screen.getByTestId('endpoint-result')).toHaveTextContent('m')
  const after = screen.getByTestId('endpoint-result').getAttribute('data-revision')
  expect(after).not.toBe(before)
  expect(screen.getByTestId('transform-result')).toHaveAttribute('data-revision', after)
  expect(screen.getByTestId('jacobian-result')).toHaveAttribute('data-revision', after)
})
```

Test D–H headings via rendered KaTeX, elbow-up/down selection, unreachable target preservation, degree/radian display switching, singular warning, and absence of banned source labels in `document.body.textContent`.

- [ ] **Step 2: Run and confirm feature tests fail**

Run: `npm test -- src/features/model/RobotModelPage.test.tsx src/features/kinematics/KinematicsPage.test.tsx`

- [ ] **Step 3: Implement the robot model module**

Show geometry, environment, frame definitions, standard D–H table, and the 3D scene. Every editable number uses `NumericField` with a visible SI unit and an inline validation message; commit the value on blur or Enter so partially typed numbers do not invalidate the shared calculation.

- [ ] **Step 4: Implement FK, IK, Jacobian, and singularity panels**

Use joint controls for FK and a three-coordinate desired-position editor for IK. Preserve the last valid pose on unreachable input, show both valid branches, report minimum singular value and condition number, and connect all formula cards to the same revision id as the scene.

- [ ] **Step 5: Verify and commit**

```bash
npm test -- src/features/model/RobotModelPage.test.tsx src/features/kinematics/KinematicsPage.test.tsx
npm run typecheck
git add src/features/model src/features/kinematics src/app/App.tsx
git commit -m "feat: add academic kinematics modules"
```

---

### Task 10: Deliver the Editable Dynamics Module

**Files:**
- Create: `src/features/dynamics/RigidBodyEditor.tsx`
- Create: `src/features/dynamics/DynamicsPage.tsx`
- Modify: `src/app/App.tsx`
- Test: `src/features/dynamics/DynamicsPage.test.tsx`

**Interfaces:**
- Consumes: `RobotParameters`, validated field edits, `DynamicsResult`, and energy result.
- Produces: editable mass/center/inertia/friction/gravity controls and formula-result cards.

- [ ] **Step 1: Write failing parameter-isolation and validation tests**

```tsx
it('changes dynamics without changing forward kinematics when only m₂ changes', async () => {
  render(<DynamicsPage />)
  const endpointBefore = screen.getByTestId('endpoint-result').textContent
  const massBefore = screen.getByTestId('mass-matrix-result').textContent
  await replaceNumber(screen.getByLabelText('连杆 2 质量 m₂'), '4.5')
  expect(screen.getByTestId('endpoint-result')).toHaveTextContent(endpointBefore ?? '')
  expect(screen.getByTestId('mass-matrix-result').textContent).not.toBe(massBefore)
})
```

Test all six inertia components, nonphysical principal inertia, center-of-mass range, friction enable/disable, gravity direction, matrix units, and the three formula-card views.

- [ ] **Step 2: Run and confirm failures**

Run: `npm test -- src/features/dynamics/DynamicsPage.test.tsx`

- [ ] **Step 3: Implement the structured rigid-body editor**

Use one collapsible section per link. Present mass, three center-of-mass coordinates, and six symmetric inertia components. Show the reconstructed inertia matrix and its principal moments next to validation. Add explicit reset buttons per link and for all teaching parameters.

- [ ] **Step 4: Implement dynamics and energy explanations**

Render cards for the manipulator equation, `M(q)`, `C(q,q̇)`, `g(q)`, friction torque, kinetic energy, potential energy, total energy, and joint power. State in the Coriolis definition card that Christoffel symbols are exact while mass derivatives are evaluated by central difference with `h=10⁻⁵ rad`.

- [ ] **Step 5: Verify and commit**

```bash
npm test -- src/features/dynamics/DynamicsPage.test.tsx
npm run typecheck
git add src/features/dynamics src/app/App.tsx
git commit -m "feat: add editable rigid body dynamics lesson"
```

---

### Task 11: Deliver Inverse/Forward Dynamics Experiments, Charts, and Export

**Files:**
- Create: `src/features/experiments/useAnimationClock.ts`
- Create: `src/features/experiments/SimulationControls.tsx`
- Create: `src/features/experiments/TimeSeriesCharts.tsx`
- Create: `src/features/experiments/ExperimentPage.tsx`
- Create: `src/export/csv.ts`
- Create: `src/export/download.ts`
- Modify: `src/app/App.tsx`
- Test: `src/features/experiments/ExperimentPage.test.tsx`
- Test: `src/export/csv.test.ts`

**Interfaces:**
- Consumes: both simulation pipelines and the shared simulation time.
- Produces: synchronized playback, scrub, charts, CSV, and Plotly SVG/PNG export.

- [ ] **Step 1: Write failing mode, playback, synchronization, and CSV tests**

Test that inverse mode maps a quintic trajectory to nonempty torque samples; forward mode maps a sine torque to state samples; play/pause/step/reset mutate one shared time; scrubbing updates scene and formula revision; invalid simulation pauses with the exact diagnostic; CSV contains `time_s`, `theta_1_rad`, `omega_1_rad_s`, `tau_1_N_m`, `kinetic_J`, `potential_J`, and `total_energy_J`.

- [ ] **Step 2: Run and confirm failures**

Run: `npm test -- src/features/experiments/ExperimentPage.test.tsx src/export/csv.test.ts`

- [ ] **Step 3: Implement the single animation clock**

Use `requestAnimationFrame` only to advance display time. Convert wall-clock delta by the selected playback speed, select the nearest precomputed simulation sample, and stop exactly at duration. Pause on component unmount and when the document becomes hidden.

- [ ] **Step 4: Implement both experiment editors**

Inverse dynamics offers quintic and sinusoidal joint trajectories. Forward dynamics offers constant, step, sinusoidal, and piecewise-constant torque. Expose duration and integration step within the specified bounds; regenerate samples only after a committed valid edit.

- [ ] **Step 5: Implement synchronized charts and export**

Create Plotly groups for `q/q̇/q̈`, `τ`, `K/V/E`, and `P_i`, all with the same vertical time cursor. A Plotly click updates the shared time. Build CSV entirely from simulation samples with an explicit UTF-8 BOM for Chinese spreadsheet compatibility. Use Plotly's `downloadImage` for SVG and PNG.

- [ ] **Step 6: Verify and commit**

```bash
npm test -- src/features/experiments/ExperimentPage.test.tsx src/export/csv.test.ts
npm run typecheck
git add src/features/experiments src/export src/app/App.tsx
git commit -m "feat: add synchronized dynamics experiments"
```

---

### Task 12: Add Browser Acceptance Tests, Documentation, CI, and Pages

**Files:**
- Create: `playwright.config.ts`
- Create: `e2e/robotics-lab.spec.ts`
- Create: `docs/mathematics.md`
- Create: `docs/symbols.md`
- Create: `.github/workflows/ci-pages.yml`
- Create or replace: `README.md`
- Modify: `vite.config.ts`
- Modify: `.gitignore`

**Interfaces:**
- Produces: reproducible local and CI verification.
- Produces: a Pages build rooted at `/planar-2r-simulator/` in GitHub Actions and `/` locally.
- Produces: Chinese user/developer documentation and anonymous demo entry.

- [ ] **Step 1: Write the failing Playwright acceptance flow**

```ts
test('completes a kinematics and inverse-dynamics learning flow', async ({ page }) => {
  await page.goto('/')
  await expect(page.getByRole('heading', { name: '空间 3R 机器人学交互实验室' })).toBeVisible()
  await page.getByRole('button', { name: '运动学' }).click()
  await page.getByLabel('关节角 θ₂').fill('35')
  await expect(page.getByTestId('endpoint-result')).toContainText('m')
  await page.getByRole('button', { name: '动态实验' }).click()
  await page.getByRole('tab', { name: '逆动力学' }).click()
  await page.getByRole('button', { name: '生成实验' }).click()
  await page.getByRole('button', { name: '播放' }).click()
  await expect(page.getByTestId('simulation-time')).not.toHaveText('0.000 s')
})
```

Add one screenshot test at 1440 × 1000 and one 1024 px layout test. Install Chromium with `npx playwright install chromium` and confirm the first run exposes any unfinished integration.

- [ ] **Step 2: Configure local and Pages base paths**

```ts
// vite.config.ts
export default defineConfig({
  base: process.env.GITHUB_ACTIONS ? '/planar-2r-simulator/' : '/',
  plugins: [react()],
})
```

Configure Playwright's web server as `npm run dev -- --host 127.0.0.1` and base URL `http://127.0.0.1:5173`.

- [ ] **Step 3: Write source-backed Chinese documentation**

`README.md` must cover the learning modules, screenshot, Pages link, formulas at a glance, install/run/test commands, browser support, numerical-method disclosure, legacy folder, license, and citation suggestion. `docs/mathematics.md` mirrors the implemented D–H, IK, Jacobian, mass, Christoffel, gravity, energy, RK4, and trajectory equations. `docs/symbols.md` maps every displayed symbol to Chinese meaning and SI unit.

- [ ] **Step 4: Add one CI and Pages workflow**

The workflow triggers on pull requests and pushes to `main`; it checks out, sets up Node 20.19, runs `npm ci`, unit tests, typecheck, build, installs Chromium, and runs Playwright. On `main` only, upload `dist` with `actions/upload-pages-artifact` and deploy with `actions/deploy-pages` using `pages: write` and `id-token: write` permissions.

- [ ] **Step 5: Run the complete local acceptance suite**

```bash
npm test
npm run typecheck
npm run build
npm run e2e
python3 -m unittest discover -s legacy/python -p 'test_*.py'
git diff --check
```

Expected: all unit, component, E2E, legacy Python, type, build, and whitespace checks pass.

- [ ] **Step 6: Visually inspect the production UI**

Run `npm run dev -- --host 127.0.0.1`, open the app at 1440 × 1000, and inspect all four modules. Confirm no clipped formulas, unreadable matrices, overlapping controls, hidden units, misleading vector scales, or source-style labels. Save the approved screenshot as `docs/robotics-lab-preview.png`.

- [ ] **Step 7: Commit**

```bash
git add README.md docs .github playwright.config.ts e2e vite.config.ts .gitignore
git commit -m "docs: add verified public robotics lab release"
```

---

### Task 13: Security-Check, Publish, Make Public, and Verify Pages

**Files:**
- No source files should change unless the release verification finds a defect.

**Interfaces:**
- Consumes: a fully verified branch integrated into `main`.
- Produces: a public GitHub repository and anonymously accessible GitHub Pages site.

- [ ] **Step 1: Re-run the completion gate immediately before publishing**

Run:

```bash
npm test
npm run typecheck
npm run build
npm run e2e
python3 -m unittest discover -s legacy/python -p 'test_*.py'
git status --short
```

Expected: every command passes and the worktree is clean.

- [ ] **Step 2: Inspect tracked history for accidental secrets before public visibility**

Run:

```bash
git grep -nEI '(api[_-]?key|client[_-]?secret|access[_-]?token|private[_-]?key|password)[[:space:]]*[:=]'
git log --all --format='%H %ae' | head -50
```

Expected: no credential assignment, private key, non-public account secret, or unexpected author email is present. Documentation references to generic words such as “token” are reviewed manually rather than blindly removed.

- [ ] **Step 3: Integrate the implementation branch into local `main` using the finishing-development-branch workflow**

Choose the workflow's local merge option, update `main`, and repeat the full verification there. Do not force-push and do not rewrite the existing Python history.

- [ ] **Step 4: Push the verified default branch**

Run:

```bash
git push origin main
```

Expected: `origin/main` points to the verified release commit.

- [ ] **Step 5: Change the existing repository visibility to public**

Run:

```bash
gh repo edit ZaowenChen/planar-2r-simulator --visibility public --accept-visibility-change-consequences
gh repo view ZaowenChen/planar-2r-simulator --json visibility,url,defaultBranchRef
```

Expected: visibility is `PUBLIC` and the default branch is `main`.

- [ ] **Step 6: Enable workflow-based Pages and watch the deployment**

First inspect Pages with `gh api repos/ZaowenChen/planar-2r-simulator/pages`. If it returns not found, create it with:

```bash
gh api --method POST repos/ZaowenChen/planar-2r-simulator/pages -f build_type=workflow
```

Then run:

```bash
robotics_run_id="$(gh run list --repo ZaowenChen/planar-2r-simulator --workflow ci-pages.yml --limit 1 --json databaseId --jq '.[0].databaseId')"
gh run watch "$robotics_run_id" --repo ZaowenChen/planar-2r-simulator --exit-status
```

Expected: CI and deployment complete successfully.

- [ ] **Step 7: Verify anonymous public access**

Run:

```bash
curl -I https://github.com/ZaowenChen/planar-2r-simulator
curl -I https://zaowenchen.github.io/planar-2r-simulator/
```

Expected: both return a successful public response. Open the Pages URL in a signed-out browser context and complete the Playwright learning flow once more against the deployed URL.
