# Kinematics Millimetres, Geometry, and Pose Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the kinematics teaching UI millimetre- and degree-only, add a complete geometric IK derivation, and display the achieved end-effector orientation.

**Architecture:** Preserve the existing metre/radian domain model and add explicit presentation conversion helpers at component/derivation boundaries. Extend the derivation model with display transforms, orientation, natural attitude parameters, branch-specific geometric quantities, and forward-check errors so React only renders prepared values.

**Tech Stack:** TypeScript 5.9, React 19, Zustand, KaTeX, Vitest, Testing Library, Playwright.

## Global Constraints

- Core robotics and dynamics remain in metres and radians.
- User-facing kinematics angles use degrees only and lengths use millimetres.
- A homogeneous transform scales only its translation column for millimetre display.
- Pose means end-effector position plus orientation; elbow-up/down means configuration.
- Do not add arbitrary desired 6D pose IK to this 3R mechanism.

---

### Task 1: Presentation conversions and derivation data

**Files:**
- Create: `src/features/kinematics/presentation.ts`
- Modify: `src/features/kinematics/derivationModel.ts`
- Test: `src/features/kinematics/derivationModel.test.ts`

**Interfaces:**
- Produces: `metresToMillimetres(value)`, `millimetresToMetres(value)`, `radiansToDegrees(value)`, `degreesToRadians(value)`, `transformInMillimetres(transform)`, and `jacobianForMillimetresAndDegrees(jacobian)`.
- Extends: `KinematicsDerivation` with `qDegrees`, `displayTransforms`, `orientation`, `azimuthDegrees`, `toolElevationDegrees`, and per-solution geometric checks.

- [ ] Write tests with hand-derived values: `1.25 m → 1250 mm`, a transform only scales `[0..2][3]`, the default pose produces `β=-25°`, and every IK solution has sub-micrometre forward-check error.
- [ ] Run `npm test -- --run src/features/kinematics/derivationModel.test.ts` and confirm the new assertions fail because the APIs do not exist.
- [ ] Implement the conversion helpers and derivation fields without changing domain values.
- [ ] Re-run the focused test and confirm it passes.

### Task 2: Degree-only and millimetre inputs

**Files:**
- Modify: `src/features/kinematics/JointControls.tsx`
- Modify: `src/features/kinematics/InverseKinematicsPanel.tsx`
- Modify: `src/features/model/RobotModelPage.tsx`
- Modify: `src/features/model/DhTable.tsx`
- Test: `src/features/kinematics/KinematicsPage.test.tsx`
- Test: `src/features/model/RobotModelPage.test.tsx`

**Interfaces:**
- Joint inputs convert degrees to internal radians on commit.
- Target and geometry inputs convert displayed millimetres to internal metres on commit.
- D–H rows receive internal parameters and render `d/a` in millimetres and `α` in degrees.

- [ ] Add component tests asserting the radian switch is absent, `θ₁=30°`, the initial target is rendered in millimetres, a `3000 mm` edit stores `3 m`, and geometry fields round-trip millimetres.
- [ ] Run the two focused component suites and confirm failures arise from current metre/radian UI behavior.
- [ ] Implement minimal boundary conversion and degree-only rendering.
- [ ] Re-run the focused suites and confirm they pass.

### Task 3: Geometric derivation and pose walkthrough

**Files:**
- Modify: `src/features/kinematics/KinematicsPage.tsx`
- Modify: `src/features/kinematics/KinematicsWalkthrough.tsx`
- Modify: `src/features/kinematics/derivationModel.ts`
- Test: `src/features/kinematics/KinematicsPage.test.tsx`

**Interfaces:**
- Walkthrough consumes prepared millimetre/degree data only.
- Geometry solution records expose `signedRadialMm`, `targetDirectionDegrees`, `triangleCorrectionDegrees`, `qDegrees`, `positionErrorMm`, and `orientation`.

- [ ] Add page assertions for projection geometry, `γ−δ`, folded-family explanation, `⁰R₃`, `β=θ₂+θ₃`, degree-only branch results, and millimetre pose output.
- [ ] Run the page suite and confirm those sections are missing.
- [ ] Replace the radian-conversion step, expand the analytic geometry steps, add pose output, and rename elbow branches as configurations.
- [ ] Re-run the page suite and confirm it passes with no KaTeX errors.

### Task 4: Documentation and full verification

**Files:**
- Modify: `README.md`
- Modify: `docs/mathematics.md`
- Modify: `docs/symbols.md`
- Modify: `e2e/robotics-lab.spec.ts` only if user-visible assertions require adjustment.

**Interfaces:**
- Documentation distinguishes display units from internal domain units and records the orientation formula and geometric IK construction.

- [ ] Update human documentation for millimetre/degree presentation, internal SI/radian computation, natural attitude parameters, and geometric branch definitions.
- [ ] Run `npm test` and fix only regressions caused by this feature.
- [ ] Run `npm run typecheck` and `npm run build`.
- [ ] Run `npm run e2e` and confirm the real browser workflow passes.
- [ ] Review `git diff --check` and `git diff` against every design requirement.
