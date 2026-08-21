# Kinematics 3D Teaching Linkage Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the kinematics walkthrough drive the 3D robot, D–H frames, planar geometry, and forward-check visuals as one synchronized teaching experience.

**Architecture:** `KinematicsPage` owns one controlled teaching state and passes it to the walkthrough, IK controls, and an opt-in kinematics presentation model for `RobotScene`. Domain calculations remain in the existing metre/radian robotics core; pure presentation builders derive frame visibility, camera presets, annotations, ghost poses, and mm/degree labels without changing dynamics or experiment behavior.

**Tech Stack:** TypeScript 5.9, React 19, Zustand 5, React Three Fiber 9, Drei 10, Three.js 0.185, KaTeX, Vitest, Testing Library, Playwright.

## Global Constraints

- Preserve the current uncommitted user work; do not reset, discard, or silently overwrite it. Confirm a clean baseline or an approved checkpoint before executing commit steps.
- Modify only the kinematics module and opt-in shared scene capabilities. Do not change dynamics or dynamic-experiment business behavior.
- The robotics core continues to use metres and radians; user-facing kinematics lengths use mm and angles use degrees.
- Use the existing standard D–H convention and existing forward/inverse kinematics calculations as the source of truth.
- D–H substep animation explains coordinate-frame construction; it must not imply that the robot physically follows four separate motions.
- Teaching animations are local presentation state and must not mutate the Zustand joint state unless the user explicitly presses “应用所选逆解”.
- All kinematics-only `RobotScene` behavior must be optional; callers in model, dynamics, and experiments retain current defaults.
- Respect `prefers-reduced-motion`, provide text/line-style alternatives to color, and prevent page-level horizontal overflow.
- This plan defers the coordinate-conversion laboratory, ideal/limited reachability volumes, direct KaTeX-character clicking, and Jacobian small-perturbation animation.

## File Responsibility Map

- `src/features/kinematics/teachingState.ts`: controlled teaching-state types, defaults, reducer, configuration identifiers, and step-to-chapter mapping.
- `src/features/kinematics/KinematicsPage.tsx`: owns teaching state and wires controls, walkthrough, layout, and 3D presentation together.
- `src/features/kinematics/KinematicsWalkthrough.tsx`: renders controlled steps, chapter navigation, focus chips, and step actions.
- `src/features/kinematics/KinematicsScenePresentation.ts`: pure mapping from derivation + teaching state to the generic `ScenePresentationModel`.
- `src/features/kinematics/KinematicsSceneToolbar.tsx`: camera preset, follow-camera, frame-mode, and “回到本步视角” controls.
- `src/features/kinematics/DhTransformPlayer.tsx`: D–H row/substep controls and explanatory copy.
- `src/features/kinematics/VerificationSummary.tsx`: compact branch comparison for FK back-substitution.
- `src/features/kinematics/InverseKinematicsPanel.tsx`: applies the controlled teaching configuration when it is an applicable IK solution.
- `src/features/kinematics/PlanarGeometryDiagram.tsx`: uses the same controlled configuration and symbol focus as the 3D scene.
- `src/features/model/DhTable.tsx`: remains reusable, with optional row/cell interaction props.
- `src/robotics/transforms.ts`: exposes tested D–H decomposition using existing transform primitives.
- `src/robotics/kinematics.ts`: exposes all analytic geometric candidates so teaching and application do not duplicate IK formulas.
- `src/scene/sceneModel.ts`: generic optional scene-presentation primitives; no knowledge of walkthrough step numbers.
- `src/scene/RobotScene.tsx`: controlled camera preset and optional teaching-presentation input.
- `src/scene/KinematicsTeachingOverlays.tsx`: renders typed work planes, dimensions, arcs, target/check points, and ghost chains.
- `src/scene/CoordinateFrame.tsx`: frame emphasis, merged labels, axis labels, and accessible presentation.
- `src/app/app.css` and `src/scene/robotScene.css`: scoped focus layout, responsive panes, and visual states.

---

### Task 1: Controlled teaching state and kinematics-only layout

**Files:**
- Create: `src/features/kinematics/teachingState.ts`
- Modify: `src/features/kinematics/KinematicsPage.tsx`
- Modify: `src/features/kinematics/KinematicsWalkthrough.tsx`
- Modify: `src/features/kinematics/InverseKinematicsPanel.tsx`
- Modify: `src/features/kinematics/KinematicsPage.test.tsx`
- Modify: `src/app/app.css`

**Interfaces:**
- Produces: `KinematicsTeachingState`, `KinematicsTeachingAction`, `kinematicsTeachingReducer`, `configurationId(solution)`.
- `KinematicsWalkthrough` consumes controlled `stepIndex`, `onStepChange`, `activeConfigurationId`, and `onConfigurationChange` props.
- `InverseKinematicsPanel` consumes the same configuration ID and reports user selection through `onConfigurationChange`.

- [ ] **Step 1: Add a failing preservation and synchronization test**

```tsx
it('preserves the current step when parameters change and shares the IK configuration', async () => {
  const user = userEvent.setup()
  render(<KinematicsPage />)

  await user.click(screen.getByRole('button', { name: '下一步' }))
  await user.clear(screen.getByLabelText('关节角 θ₂'))
  await user.type(screen.getByLabelText('关节角 θ₂'), '30{Enter}')
  expect(screen.getByTestId('walkthrough-step')).toHaveTextContent('第 2 / 17 步')

  await user.click(screen.getByRole('radio', { name: /肘上构型/ }))
  expect(screen.getByTestId('kinematics-workspace'))
    .toHaveAttribute('data-configuration', 'conventional:elbow-up')
})
```

- [ ] **Step 2: Run the focused page test and verify the old reset expectation fails**

Run: `npx vitest run src/features/kinematics/KinematicsPage.test.tsx`

Expected: FAIL because `KinematicsWalkthrough` still resets to step 1 and configuration state is split between components.

- [ ] **Step 3: Add the controlled state model**

```ts
export type KinematicsConfigurationId =
  | 'conventional:elbow-down'
  | 'conventional:elbow-up'
  | 'folded:elbow-down'
  | 'folded:elbow-up'

export type KinematicsFrameMode = 'hidden' | 'current' | 'chain' | 'all'
export type KinematicsCameraPreset = 'overview' | 'top' | 'work-plane' | 'tool'
export type DhOperation = 'rz' | 'tz' | 'tx' | 'rx'
export type KinematicsSymbol = 'theta1' | 'theta2' | 'theta3' | 'r' | 'h' | 's' | 'l2' | 'l3' | 'gamma' | 'delta' | 'beta'

export interface KinematicsTeachingState {
  stepIndex: number
  activeConfigurationId: KinematicsConfigurationId
  frameMode: KinematicsFrameMode
  cameraPreset: KinematicsCameraPreset
  followStepCamera: boolean
  selectedDhRow: 0 | 1 | 2
  dhOperation: DhOperation
  symbolFocus: KinematicsSymbol | null
  focusedLayout: boolean
  mobilePane: 'scene' | 'analysis' | 'controls'
}
```

Implement reducer actions for step, configuration, frame mode, camera, D–H row/operation, symbol focus, focus layout, and mobile pane. Step changes set only step-scoped defaults; parameter and calculation revisions do not dispatch a reset.

- [ ] **Step 4: Lift state into `KinematicsPage` and make the walkthrough controlled**

```tsx
const [teaching, dispatchTeaching] = useReducer(
  kinematicsTeachingReducer,
  INITIAL_KINEMATICS_TEACHING_STATE,
)

<KinematicsWalkthrough
  activeConfigurationId={teaching.activeConfigurationId}
  onConfigurationChange={(value) => dispatchTeaching({ type: 'configuration', value })}
  onStepChange={(value) => dispatchTeaching({ type: 'step', value })}
  stepIndex={teaching.stepIndex}
/>
```

Remove the effect that calls `setStepIndex(0)` and `setDiagramBranch('elbow-down')`. Add a kinematics-only workspace wrapper with desktop focus mode and mobile `3D / 推导 / 参数` controls; do not change `WorkbenchLayout` behavior for other modules.

- [ ] **Step 5: Add clickable chapter navigation and pass the shared configuration to the IK panel**

Use these chapter ranges: method `0`, D–H FK `1–5`, geometric IK `6–13`, verification `14–15`, Jacobian `16`. Keep previous/next buttons and expose current chapter with `aria-current="step"`.

- [ ] **Step 6: Run the focused test and commit the state foundation**

Run: `npx vitest run src/features/kinematics/KinematicsPage.test.tsx`

Expected: PASS, including the new step-preservation assertion.

```bash
git add src/features/kinematics/teachingState.ts src/features/kinematics/KinematicsPage.tsx src/features/kinematics/KinematicsWalkthrough.tsx src/features/kinematics/InverseKinematicsPanel.tsx src/features/kinematics/KinematicsPage.test.tsx src/app/app.css
git commit -m "feat: control kinematics teaching state"
```

### Task 2: Single-source D–H decomposition and analytic IK candidates

**Files:**
- Modify: `src/robotics/transforms.ts`
- Modify: `src/robotics/transforms.test.ts`
- Modify: `src/robotics/kinematics.ts`
- Modify: `src/robotics/inverseKinematics.test.ts`
- Modify: `src/features/kinematics/derivationModel.ts`
- Modify: `src/features/kinematics/derivationModel.test.ts`

**Interfaces:**
- Produces: `decomposeDhTransform(theta, a, alpha, d): DhTransformDecomposition`.
- Produces: `inverseKinematicsCandidates(target, parameters): readonly InverseKinematicsCandidate[]`.
- `inverseKinematics` preserves its current public result and selects applicable candidates from the new candidate list.

- [ ] **Step 1: Add failing transform and candidate tests**

```ts
it('recomposes the four standard D-H operations exactly', () => {
  const parts = decomposeDhTransform(0.4, 2, Math.PI / 2, 0.8)
  expect(parts.result).toEqual(dhTransform(0.4, 2, Math.PI / 2, 0.8))
  expect(parts.operations.map((operation) => operation.kind))
    .toEqual(['rz', 'tz', 'tx', 'rx'])
})

it('enumerates both elbow branches in both radial families', () => {
  const candidates = inverseKinematicsCandidates([2.7, 1.5, 1], DEFAULT_ROBOT_PARAMETERS)
  expect(candidates.map((candidate) => `${candidate.radialFamily}:${candidate.branch}`)).toEqual([
    'conventional:elbow-down',
    'conventional:elbow-up',
    'folded:elbow-down',
    'folded:elbow-up',
  ])
})
```

- [ ] **Step 2: Run the focused math tests and verify missing exports fail**

Run: `npx vitest run src/robotics/transforms.test.ts src/robotics/inverseKinematics.test.ts src/features/kinematics/derivationModel.test.ts`

Expected: FAIL because neither new function exists.

- [ ] **Step 3: Implement D–H decomposition with existing primitives**

```ts
export function decomposeDhTransform(
  theta: number,
  a: number,
  alpha: number,
  d: number,
): DhTransformDecomposition {
  const operations = [
    { kind: 'rz' as const, transform: rotationZ(theta) },
    { kind: 'tz' as const, transform: translation(0, 0, d) },
    { kind: 'tx' as const, transform: translation(a, 0, 0) },
    { kind: 'rx' as const, transform: rotationX(alpha) },
  ]
  return {
    operations,
    result: operations.reduce((value, operation) => multiply4(value, operation.transform), IDENTITY_4),
  }
}
```

Export or locally define an immutable `IDENTITY_4` in `transforms.ts`; keep all values in metres/radians.

- [ ] **Step 4: Extract candidate enumeration from the existing IK formula**

Define `InverseKinematicsCandidate` as an `InverseKinematicsSolution` plus `withinJointLimits: boolean`. Enumerate conventional then folded families, and elbow-down then elbow-up branches using the existing `atan2` and cosine-law expressions. Refactor `inverseKinematics` to select from those candidates without changing current status codes, joint-limit behavior, or reference-pose preference.

- [ ] **Step 5: Replace duplicated conventional-branch math in the derivation model**

Build `conventionalBranches` and `solutionDetails` from `inverseKinematicsCandidates`; keep all current mm/degree and FK residual fields. Do not reimplement `theta1`, `theta2`, or `theta3` in `derivationModel.ts`.

- [ ] **Step 6: Run the focused suites and commit the mathematical foundation**

Run: `npx vitest run src/robotics/transforms.test.ts src/robotics/inverseKinematics.test.ts src/features/kinematics/derivationModel.test.ts`

Expected: PASS with existing IK expectations unchanged.

```bash
git add src/robotics/transforms.ts src/robotics/transforms.test.ts src/robotics/kinematics.ts src/robotics/inverseKinematics.test.ts src/features/kinematics/derivationModel.ts src/features/kinematics/derivationModel.test.ts
git commit -m "refactor: expose kinematics teaching calculations"
```

### Task 3: Coordinate frames, scene presentation, and teaching cameras

**Files:**
- Create: `src/features/kinematics/KinematicsScenePresentation.ts`
- Create: `src/features/kinematics/KinematicsScenePresentation.test.ts`
- Create: `src/features/kinematics/KinematicsSceneToolbar.tsx`
- Modify: `src/features/kinematics/KinematicsPage.tsx`
- Create: `src/scene/KinematicsTeachingOverlays.tsx`
- Modify: `src/scene/sceneModel.ts`
- Modify: `src/scene/sceneModel.test.ts`
- Modify: `src/scene/CoordinateFrame.tsx`
- Modify: `src/scene/RobotScene.tsx`
- Modify: `src/scene/RobotScene.test.tsx`
- Modify: `src/scene/SceneOverlays.tsx`
- Modify: `src/scene/robotScene.css`

**Interfaces:**
- Produces: `buildKinematicsScenePresentation(input): ScenePresentationModel`.
- `RobotScene` accepts optional `presentation?: ScenePresentationModel`, `onCameraInteraction?: () => void`, `onSceneObjectSelect?: (id: string) => void`, and `visibleOverlayControls?: readonly (keyof SceneOverlayFlags)[]`.
- `ScenePresentationModel` is declared in `src/scene/sceneModel.ts`, so shared scene code never imports from a feature directory. The kinematics builder fills camera, frame emphasis, target/check points, work-plane/dimension/arc primitives, ghost poses, and explanatory note without React state or walkthrough rendering.

- [ ] **Step 1: Add failing scene-presentation tests**

```ts
it('merges coincident world/base and frame-3/tool labels', () => {
  const presentation = buildKinematicsScenePresentation(makeInput({ stepIndex: 1 }))
  expect(presentation.frames.map((frame) => frame.label)).toEqual([
    '{W}/{0}', '{1}', '{2}', '{3}/{e}',
  ])
  expect(presentation.note).toContain('世界坐标系 {W} 与基座坐标系 {0} 重合')
})

it('uses the work-plane camera for geometric projection', () => {
  const presentation = buildKinematicsScenePresentation(makeInput({ stepIndex: 6 }))
  expect(presentation.camera.id).toBe('work-plane')
})
```

- [ ] **Step 2: Run scene tests and verify the presentation API is missing**

Run: `npx vitest run src/features/kinematics/KinematicsScenePresentation.test.ts src/scene/sceneModel.test.ts src/scene/RobotScene.test.tsx`

Expected: FAIL because the presentation builder and optional scene prop do not exist.

- [ ] **Step 3: Define generic typed presentation primitives**

```ts
export interface SceneCameraPresetModel {
  id: 'overview' | 'top' | 'work-plane' | 'tool'
  position: Vector3
  target: Vector3
  up: Vector3
}

export interface ScenePresentationModel {
  camera: SceneCameraPresetModel
  frames: readonly CoordinateFrameModel[]
  points: readonly { id: string; label: string; position: Vector3; color: string }[]
  dimensions: readonly { id: string; label: string; start: Vector3; end: Vector3; style: 'solid' | 'dashed' }[]
  arcs: readonly { id: string; label: string; center: Vector3; axis: Vector3; start: Vector3; angle: number; color: string }[]
  workPlane: { origin: Vector3; normal: Vector3; width: number; height: number } | null
  ghostPoses: readonly { id: string; forward: ForwardKinematicsResult; opacity: number; label: string }[]
  note: string
}
```

Keep the base `SceneModel` unchanged when no presentation is supplied. Generate camera distance from `d1 + l2 + l3`, not a fixed robot size.

For the kinematics page, pass controlled defaults that hide center of mass, workspace, trail, velocity, acceleration, gravity, and torque. Show only grid and teaching-frame controls; retain the original overlay defaults and full control list for every other `RobotScene` caller.

- [ ] **Step 4: Upgrade coordinate-frame rendering**

Render red x, green y, blue z axes with arrowheads, origin label, frame label, and emphasis opacity. Merge `{W}` with `{0}` and `{3}` with `{e}` only when their transforms are equal within tolerance. Default teaching mode shows the current adjacent pair; chain mode shows `{W}/{0} → {1} → {2} → {3}/{e}`; hidden and all modes behave literally. Labels remain camera-facing; distant frames show only the frame name and the active adjacent pair shows axis labels.

- [ ] **Step 5: Implement one-shot camera following**

Render the four preset buttons, follow-camera switch, frame-mode control, and “回到本步视角” in `KinematicsSceneToolbar`. On a step/preset change, interpolate camera position and OrbitControls target once. `OrbitControls.onStart` suspends automatic movement until the next explicit step change or “回到本步视角”. If `matchMedia('(prefers-reduced-motion: reduce)')` matches, apply the preset immediately.

- [ ] **Step 6: Run scene tests and commit the opt-in scene foundation**

Run: `npx vitest run src/features/kinematics/KinematicsScenePresentation.test.ts src/scene/sceneModel.test.ts src/scene/RobotScene.test.tsx`

Expected: PASS; existing dynamics/experiment `RobotScene` calls require no new props.

```bash
git add src/features/kinematics/KinematicsScenePresentation.ts src/features/kinematics/KinematicsScenePresentation.test.ts src/features/kinematics/KinematicsSceneToolbar.tsx src/features/kinematics/KinematicsPage.tsx src/scene/KinematicsTeachingOverlays.tsx src/scene/sceneModel.ts src/scene/sceneModel.test.ts src/scene/CoordinateFrame.tsx src/scene/RobotScene.tsx src/scene/RobotScene.test.tsx src/scene/SceneOverlays.tsx src/scene/robotScene.css
git commit -m "feat: add kinematics teaching scene"
```

### Task 4: D–H table, transform player, and FK steps 1–6

**Files:**
- Create: `src/features/kinematics/DhTransformPlayer.tsx`
- Create: `src/features/kinematics/DhTransformPlayer.test.tsx`
- Create: `src/features/kinematics/HomogeneousTransformCard.tsx`
- Modify: `src/features/model/DhTable.tsx`
- Modify: `src/features/kinematics/KinematicsPage.tsx`
- Modify: `src/features/kinematics/KinematicsWalkthrough.tsx`
- Modify: `src/features/kinematics/KinematicsPage.test.tsx`
- Modify: `src/features/kinematics/KinematicsScenePresentation.ts`
- Modify: `src/app/app.css`

**Interfaces:**
- `DhTable` accepts optional `selectedRow`, `selectedParameter`, `onRowSelect`, and `onParameterSelect` props; without them it remains a static table on the model page.
- `DhTransformPlayer` consumes one decomposition, displayed mm/degree values, controlled operation, and `onOperationChange`.
- `HomogeneousTransformCard` displays rotation block `R` and translation column `p` with `onFocusPart('rotation' | 'translation')`.

- [ ] **Step 1: Add failing interaction tests**

```tsx
it('links a D-H row and operation to the teaching scene', async () => {
  const user = userEvent.setup()
  render(<KinematicsPage />)
  await user.click(screen.getByRole('button', { name: 'D–H 正运动学' }))
  await user.click(screen.getByRole('button', { name: '选择 D–H 第 2 行' }))
  await user.click(screen.getByRole('button', { name: '沿 x₂ 平移 a₂' }))
  expect(screen.getByTestId('kinematics-workspace')).toHaveAttribute('data-dh-row', '2')
  expect(screen.getByTestId('kinematics-workspace')).toHaveAttribute('data-dh-operation', 'tx')
})
```

- [ ] **Step 2: Run the player/page tests and verify controls are absent**

Run: `npx vitest run src/features/kinematics/DhTransformPlayer.test.tsx src/features/kinematics/KinematicsPage.test.tsx`

Expected: FAIL because D–H rows and operations are not interactive.

- [ ] **Step 3: Make the D–H table optionally interactive**

Use real `<button type="button">` elements inside row headers and parameter cells rather than attaching click handlers to `<tr>`. Expose exact labels such as “选择 D–H 第 2 行” and “沿 x₂ 平移 a₂”. Selecting a cell also selects its row.

- [ ] **Step 4: Implement the four-operation player**

Render `上一步`, `下一步`, `播放/暂停`, and `重置当前 D–H 变换`. Playback advances `rz → tz → tx → rx` once and stops. The displayed chain for row 1 is `Rz(θ₁) → Tz(d₁) → Tx(a₁) → Rx(α₁)`, with current values converted to mm/degree.

- [ ] **Step 5: Connect FK steps to the 3D presentation**

Map steps 1–6 to a simplified robot/work-plane overview, full frame chain, adjacent-frame D–H operation, transform-chain highlight, endpoint x/y/z projection, and tool-orientation camera with θ₁ and β. Add the required explanatory sentence that the four operations are coordinate-transform decomposition rather than physical robot trajectory. When `onSceneObjectSelect` reports `frame-1`, `frame-2`, `frame-3`, `link-1`, `link-2`, or `link-3`, map it back to the corresponding D–H row; scene selection remains disabled for callers that omit the callback.

- [ ] **Step 6: Add the split homogeneous-transform card**

Highlight `R` and `p` as separate blocks. Focusing `R` requests the tool camera and endpoint axes; focusing `p` requests endpoint origin and x/y/z projection lines. Mark `R` as dimensionless and `p` as mm.

- [ ] **Step 7: Run focused tests and commit the FK teaching flow**

Run: `npx vitest run src/features/kinematics/DhTransformPlayer.test.tsx src/features/kinematics/KinematicsPage.test.tsx`

Expected: PASS with no KaTeX errors.

```bash
git add src/features/kinematics/DhTransformPlayer.tsx src/features/kinematics/DhTransformPlayer.test.tsx src/features/kinematics/HomogeneousTransformCard.tsx src/features/model/DhTable.tsx src/features/kinematics/KinematicsPage.tsx src/features/kinematics/KinematicsWalkthrough.tsx src/features/kinematics/KinematicsPage.test.tsx src/features/kinematics/KinematicsScenePresentation.ts src/app/app.css
git commit -m "feat: link D-H derivation to 3D scene"
```

### Task 5: Geometric IK steps 7–14 and synchronized branch focus

**Files:**
- Modify: `src/features/kinematics/KinematicsScenePresentation.ts`
- Modify: `src/features/kinematics/KinematicsScenePresentation.test.ts`
- Modify: `src/features/kinematics/KinematicsWalkthrough.tsx`
- Modify: `src/features/kinematics/PlanarGeometryDiagram.tsx`
- Modify: `src/features/kinematics/InverseKinematicsPanel.tsx`
- Modify: `src/features/kinematics/KinematicsPage.test.tsx`
- Modify: `src/scene/KinematicsTeachingOverlays.tsx`
- Modify: `src/app/app.css`

**Interfaces:**
- Formula focus chips, the planar diagram, and the 3D presentation consume one `symbolFocus` value.
- The selected `KinematicsConfigurationId` drives the planar active branch and 3D ghost/current branch.
- An active teaching candidate not returned by `inverseKinematics().solutions` is labeled “仅教学图示” and cannot be applied.

- [ ] **Step 1: Add failing projection and branch tests**

```tsx
it('synchronizes r focus and elbow configuration across formula, 2D, and 3D', async () => {
  const user = userEvent.setup()
  render(<KinematicsPage />)
  await user.click(screen.getByRole('button', { name: '几何逆运动学' }))
  await user.click(screen.getByRole('button', { name: '聚焦 r' }))
  expect(screen.getByRole('img', { name: /解析几何工作平面/ })).toHaveAttribute('data-focus-symbol', 'r')
  expect(screen.getByTestId('kinematics-workspace')).toHaveAttribute('data-symbol-focus', 'r')

  await user.click(screen.getByRole('button', { name: '显示肘上构型' }))
  expect(screen.getByTestId('kinematics-workspace'))
    .toHaveAttribute('data-configuration', 'conventional:elbow-up')
})
```

- [ ] **Step 2: Run the focused tests and verify the 3D linkage is absent**

Run: `npx vitest run src/features/kinematics/KinematicsScenePresentation.test.ts src/features/kinematics/KinematicsPage.test.tsx`

Expected: FAIL because the current planar controls do not update the 3D scene.

- [ ] **Step 3: Render target, work plane, and r/h/s dimensions**

Step 7 shows `P_d`, the plane through the base z-axis and target, the top-view `θ₁` arc, and r/h dimensions before transitioning to the work-plane camera. Step 8 adds shoulder-to-target line s. Step 9 shows the numeric ideal interval `|l₂-l₃| ≤ s ≤ l₂+l₃`; defer volume rendering.

- [ ] **Step 4: Render elbow branches and angular derivation**

Step 10 shows the selected branch as a solid robot and the other elbow branch as a labeled translucent ghost. Steps 11–13 show γ, δ, and θ₂ arcs with the same solid/dashed/color vocabulary as `PlanarGeometryDiagram`; step 13 displays all three arcs together for `θ₂=γ−δ`.

- [ ] **Step 5: Render folded-radial teaching candidates**

Step 14 uses the candidate with matching elbow branch and `radialFamily='folded'`, displays `rₛ=-r`, and labels radial family separately from elbow branch. Do not replace the applied robot pose until the user presses the apply button on an applicable solution.

- [ ] **Step 6: Add explicit focus chips and application status**

Use button labels such as `聚焦 r`, `聚焦 γ`, and `聚焦 θ₂`. Give each focused symbol a matching formula class, planar `data-focus-symbol`, and 3D annotation ID. When the selected candidate is not in the applicable result, display “当前为教学图示，不能应用到关节限位内的机器人”.

- [ ] **Step 7: Run focused tests and commit the geometric linkage**

Run: `npx vitest run src/features/kinematics/KinematicsScenePresentation.test.ts src/features/kinematics/KinematicsPage.test.tsx`

Expected: PASS for projection, focus, branch, and folded-family assertions.

```bash
git add src/features/kinematics/KinematicsScenePresentation.ts src/features/kinematics/KinematicsScenePresentation.test.ts src/features/kinematics/KinematicsWalkthrough.tsx src/features/kinematics/PlanarGeometryDiagram.tsx src/features/kinematics/InverseKinematicsPanel.tsx src/features/kinematics/KinematicsPage.test.tsx src/scene/KinematicsTeachingOverlays.tsx src/app/app.css
git commit -m "feat: synchronize geometric IK teaching views"
```

### Task 6: FK verification, pose comparison, and Jacobian vectors

**Files:**
- Create: `src/features/kinematics/VerificationSummary.tsx`
- Create: `src/features/kinematics/VerificationSummary.test.tsx`
- Modify: `src/features/kinematics/KinematicsScenePresentation.ts`
- Modify: `src/features/kinematics/KinematicsScenePresentation.test.ts`
- Modify: `src/features/kinematics/KinematicsWalkthrough.tsx`
- Modify: `src/features/kinematics/KinematicsPage.test.tsx`
- Modify: `src/scene/KinematicsTeachingOverlays.tsx`

**Interfaces:**
- `VerificationSummary` consumes the existing `InverseSolutionDerivation[]` and renders one comparison row per solution with expandable details.
- Step 15 presentation exposes `p_d`, `p_FK`, and display-scaled `Δp` without modifying global joint state.
- Step 16 presentation exposes two ghost poses with distinct tool frames and β labels.
- Step 17 presentation reuses `jacobianColumns` for axis, offset, and tangential direction vectors.

- [ ] **Step 1: Add failing verification and scene tests**

```tsx
it('summarizes each FK back-check before expandable vectors', () => {
  render(<VerificationSummary targetMm={[1, 2, 3]} solutions={makeSolutions()} />)
  expect(screen.getByRole('table', { name: '逆解回代比较' })).toBeInTheDocument()
  expect(screen.getAllByRole('button', { name: '展开完整回代数据' })).toHaveLength(2)
})

it('builds verification markers without changing the application pose', () => {
  const presentation = buildKinematicsScenePresentation(makeInput({ stepIndex: 14 }))
  expect(presentation.points.map((point) => point.id)).toContain('target-position')
  expect(presentation.points.map((point) => point.id)).toContain('fk-position')
})
```

- [ ] **Step 2: Run focused tests and verify summary/markers are missing**

Run: `npx vitest run src/features/kinematics/VerificationSummary.test.tsx src/features/kinematics/KinematicsScenePresentation.test.ts src/features/kinematics/KinematicsPage.test.tsx`

Expected: FAIL because the summary and verification scene states do not exist.

- [ ] **Step 3: Replace dense verification cards with a compact comparison**

Default columns are configuration, `q_IK`, `Δx/Δy/Δz`, `e_p`, and `β`. Each row has a `<details>` section for full `p_d`, `p_FK`, residual vector, and orientation matrix. Keep all displayed positions/errors in mm and angles in degrees.

- [ ] **Step 4: Add local FK-check scene animation**

Display the sequence `q_IK → T_FK → p_FK → Δp` using local presentation progress. Render target and FK points simultaneously. If `e_p < 1 mm`, scale only the drawn error arrow to a visible minimum and show `误差向量已放大 ×N，仅用于观察`; retain the true value in the label.

- [ ] **Step 5: Add pose comparison and geometric Jacobian overlays**

Step 16 overlays the two applicable solution chains and tool frames at the common endpoint, labeled with β and configuration. Step 17 renders each existing `zᵢ₋₁`, `pₑ-oᵢ₋₁`, and `zᵢ₋₁×(pₑ-oᵢ₋₁)` direction with textual labels; do not add the deferred perturbation button.

- [ ] **Step 6: Run focused tests and commit verification teaching**

Run: `npx vitest run src/features/kinematics/VerificationSummary.test.tsx src/features/kinematics/KinematicsScenePresentation.test.ts src/features/kinematics/KinematicsPage.test.tsx`

Expected: PASS with the Zustand joint state unchanged during presentation animation.

```bash
git add src/features/kinematics/VerificationSummary.tsx src/features/kinematics/VerificationSummary.test.tsx src/features/kinematics/KinematicsScenePresentation.ts src/features/kinematics/KinematicsScenePresentation.test.ts src/features/kinematics/KinematicsWalkthrough.tsx src/features/kinematics/KinematicsPage.test.tsx src/scene/KinematicsTeachingOverlays.tsx
git commit -m "feat: visualize IK forward verification"
```

### Task 7: Responsive, accessible, and regression-complete delivery

**Files:**
- Modify: `src/app/app.css`
- Modify: `src/scene/robotScene.css`
- Modify: `src/app/App.tsx`
- Modify: `src/app/App.test.tsx`
- Modify: `e2e/robotics-lab.spec.ts`
- Modify: `README.md`
- Modify: `docs/mathematics.md`
- Modify: `docs/symbols.md`
- Create: `public/favicon.svg`

**Interfaces:**
- Kinematics focus layout and responsive panes are scoped under `.kinematics-workspace`; other workbenches keep their current grid.
- Browser tests cover 1440×1000, 1024×900, and 390×844.

- [ ] **Step 1: Add failing responsive and workflow E2E assertions**

```ts
test('keeps the teaching step while editing and switches mobile panes', async ({ page }) => {
  await page.setViewportSize({ width: 390, height: 844 })
  await page.goto('/')
  await page.getByRole('button', { name: '运动学' }).click()
  await page.getByRole('button', { name: 'D–H 正运动学' }).click()
  await page.getByLabel('关节角 θ₂').fill('30')
  await page.getByLabel('关节角 θ₂').press('Enter')
  await expect(page.getByTestId('walkthrough-step')).toContainText('第 2 / 17 步')
  await page.getByRole('tab', { name: '3D' }).click()
  await expect(page.getByRole('region', { name: '机器人三维视图' })).toBeVisible()
  expect(await page.evaluate(() => document.documentElement.scrollWidth - document.documentElement.clientWidth)).toBeLessThanOrEqual(1)
})
```

- [ ] **Step 2: Run the focused E2E test and verify responsive controls are incomplete**

Run: `npx playwright test e2e/robotics-lab.spec.ts --grep "teaching step"`

Expected: FAIL until mobile pane roles and preservation behavior are implemented.

- [ ] **Step 3: Finish responsive and accessibility styling**

At desktop, focus mode expands scene + analysis and collapses controls. At 1024px and below, show one selected pane instead of stacking all three. Default mobile pane is `推导`; keep chapter and step controls reachable. Add formula overflow gradients/text, larger geometry labels, line-style differences for ghost branches, aria-labels for cameras/frames/D–H operations, and reduced-motion CSS. On phone widths, make all four top-level learning modules discoverable with a compact layout or an explicit horizontal-scroll affordance.

- [ ] **Step 4: Replace internal status copy and add favicon**

For kinematics, replace “结果图修订 N” with user-facing calculation validity/update copy while leaving singularity warnings intact. Add `public/favicon.svg` and reference it from `index.html` if Vite does not discover it automatically.

- [ ] **Step 5: Update documentation**

Document the frame convention `{W}/{0} → {1} → {2} → {3}/{e}`, D–H decomposition disclaimer, controlled teaching branches, 3D/2D responsibility split, mm/degree presentation boundary, and deferred enhancements.

- [ ] **Step 6: Run complete verification**

Run in order:

```bash
npm test
npm run typecheck
npm run build
npm run e2e
git diff --check
```

Expected: all commands exit 0, no new browser console errors, and no page-level horizontal overflow at the three target widths.

- [ ] **Step 7: Perform visual QA and commit delivery polish**

Capture and inspect steps 2, 3, 7, 10, 13, 15, 16, and 17 at 1440×1000; inspect steps 7 and 15 at 1024×900 and 390×844. Confirm labels remain legible, camera presets frame the configured robot, manual camera control is not immediately overridden, and dynamics/experiments retain their prior scene controls.

```bash
git add src/app/app.css src/scene/robotScene.css src/app/App.tsx src/app/App.test.tsx e2e/robotics-lab.spec.ts README.md docs/mathematics.md docs/symbols.md public/favicon.svg index.html
git commit -m "test: complete kinematics teaching experience"
```

## Deferred Follow-Up Plan

Create a separate plan only after the synchronized core experience is accepted. That plan may cover point-coordinate conversion (`{}^0p={}^0T_i{}^ip`), direction-vector conversion (`{}^0v={}^0R_i{}^iv`), ideal and joint-limited reachability volumes, direct KaTeX symbol interaction, and Jacobian small-perturbation animation.
