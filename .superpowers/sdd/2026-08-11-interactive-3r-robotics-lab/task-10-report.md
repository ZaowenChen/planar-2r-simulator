# Task 10 Report: Editable Dynamics Module

## Outcome

Implemented the dynamics learning page and connected it to the application shell. The page provides collapsible per-link rigid-body editors for mass, center of mass, and the six independent components of a symmetric inertia tensor. It reconstructs each full tensor, displays principal moments, preserves the last validated model for calculations, and supports per-link and all-parameter reset actions.

Added gravity, viscous-friction, and friction-enable controls plus academic definition/substitution/result cards for the manipulator equation, mass matrix, Coriolis matrix, gravity, friction torque, kinetic energy, potential energy, total energy, and joint power. The Coriolis card explicitly states that Christoffel symbols follow the exact definition while mass-matrix derivatives use central differences with `h=10⁻⁵ rad`.

## TDD Evidence

- RED: `npm test -- src/features/dynamics/DynamicsPage.test.tsx` failed because `DynamicsPage` did not exist.
- GREEN: the focused dynamics suite passes all 6 tests.
- Parameter isolation verifies that changing only `m₂` changes dynamics while the forward-kinematics endpoint remains unchanged.
- Coverage includes all six inertia entries, symmetric reconstruction, nonphysical principal inertia, COM bounds, gravity direction, friction toggle, SI matrix units, resets, and FormulaCard views.

## Verification

- `npm test -- src/features/dynamics/DynamicsPage.test.tsx`: 6 passed.
- `npm run typecheck`: passed.
- `npm test`: 18 files and 133 tests passed.
- `git diff --check`: passed.

## Self-review

- Invalid physical combinations remain visible as drafts but never replace the validated calculation parameters.
- Off-diagonal inertia edits update their mirrored entries before a validated tensor is accepted.
- Dynamic-only parameter edits do not alter geometry or the forward-kinematics endpoint.
- No KaTeX warnings or malformed formula output remain in the focused test run.

## Concern

The installed npm 11.5.2 emits an environment warning because Node 20.15.0 is below npm's preferred Node 20.17.0 minimum. It did not affect tests or typechecking.

## Fix Round 1

- Added an atomic multi-field parameter transaction. Resetting one link now updates its raw controls and last-valid calculation parameters together even when another link retains an invalid draft; the unrelated draft and its diagnostic remain visible.
- Used the same transaction for symmetric off-diagonal inertia pairs and the all-dynamics reset.
- Corrected the mass-matrix definition so the translational and world-rotated rotational inertia contributions are both grouped inside the link sum.
- Added Chinese symbol meanings, physical SI units, and evaluated current-value substitutions to the eight dynamics result cards.
- Standardized kinetic energy notation on `K` throughout the kinetic and total-energy cards.
- TDD RED: the focused suite failed on the stale validated reset target, incomplete rendered mass formula, missing glossaries/current substitutions, and `T` kinetic notation.
- Verification: focused dynamics tests 8/8 passed; TypeScript passed; full suite 18 files and 135/135 tests passed; `git diff --check` passed.
