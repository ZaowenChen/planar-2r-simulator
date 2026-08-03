# Interactive 3D 3R Kinematics Simulator Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a directly runnable educational 3D 3R robot simulator that unifies standard D-H forward kinematics, spatial pose transformations, closed-form multi-solution inverse kinematics, and workspace visualization.

**Architecture:** Put all numerical operations in pure functions returning small dataclasses, and isolate Matplotlib state in `Robot3DSimulator`. The GUI owns reusable 3D lines, widget state, pose/formula panels, and a guarded FK/IK event flow; numerical functions remain independently testable.

**Tech Stack:** Python standard library (`dataclasses`, `unittest`), NumPy, Matplotlib (`mplot3d`, `pyplot`, `widgets`, `table`).

## Global Constraints

- The runnable 3D simulator is one file named `robot_3r_3d_simulator.py`.
- Runtime dependencies are limited to NumPy and Matplotlib.
- Geometry is yaw–pitch–pitch with `d1=0.8`, `L2=2.0`, and `L3=1.5`.
- Initial FK angles are `theta1=30°`, `theta2=25°`, and `theta3=-50°`.
- Joint slider ranges are `theta1 [-180°,180°]`, `theta2 [-90°,90°]`, and `theta3 [-150°,150°]`.
- IK target ranges are `X/Y [-3.5,3.5]` and `Z [-2.7,4.3]`.
- The standard D-H chain is `T03 = T01 @ T12 @ T23` with `alpha0=+pi/2`.
- RPY uses the ZYX convention `R = Rz(yaw) @ Ry(pitch) @ Rx(roll)`.
- Local coordinate axes are X red, Y green, and Z blue at `{B}`, `{1}`, `{2}`, and `{W}/{T}`.
- `{W}` and `{T}` coincide because `T_W^T = I`.
- Geometrically reachable IK solutions outside configured slider joint limits are reported and do not replace the last valid GUI pose.

## File Structure

- `robot_3r_3d_simulator.py`: dataclasses, pure FK/IK/pose/workspace functions, 3D GUI class, and `main()`.
- `test_robot_3r_3d_simulator.py`: standard-library numerical and headless GUI tests; selects Matplotlib `Agg` before importing the simulator.
- `artifacts/robot_3r_3d_preview.png`: generated visual-verification image.

---

### Task 1: Standard D-H Model and 3D Forward Kinematics

**Files:**
- Create: `robot_3r_3d_simulator.py`
- Create: `test_robot_3r_3d_simulator.py`

**Interfaces:**
- Consumes: NumPy trigonometry and `@` matrix multiplication.
- Produces: `DHParameters`, `ForwardKinematicsResult3D`, `dh_transform(theta, a, alpha, d)`, and `forward_kinematics(theta1, theta2, theta3, d1=0.8, l2=2.0, l3=1.5)`.

- [ ] **Step 1: Write a failing module-availability test**

```python
import importlib.util
import unittest


class TestModuleAvailability(unittest.TestCase):
    def test_3d_simulator_module_can_be_imported(self):
        spec = importlib.util.find_spec("robot_3r_3d_simulator")
        self.assertIsNotNone(spec, "robot_3r_3d_simulator.py does not exist")
```

- [ ] **Step 2: Run the import test and verify RED**

Run: `python3 -m unittest test_robot_3r_3d_simulator.TestModuleAvailability -v`

Expected: FAIL because `find_spec("robot_3r_3d_simulator")` returns `None`.

- [ ] **Step 3: Create the minimal importable module**

```python
"""Interactive 3D 3R robot kinematics and pose-visualization simulator."""
```

- [ ] **Step 4: Run the import test and verify GREEN**

Run: `python3 -m unittest test_robot_3r_3d_simulator.TestModuleAvailability -v`

Expected: one passing test.

- [ ] **Step 5: Add failing D-H and FK tests**

After selecting `matplotlib.use("Agg")`, import the module and define `require_attr()` to turn missing APIs into assertion failures. Add these independent hand-derived behaviors:

```python
def test_general_standard_dh_transform(self):
    dh_transform = self.require_attr("dh_transform")
    actual = dh_transform(np.pi / 2, 2.0, np.pi / 2, 3.0)
    expected = np.array([
        [0.0, 0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0, 2.0],
        [0.0, 1.0, 0.0, 3.0],
        [0.0, 0.0, 0.0, 1.0],
    ])
    np.testing.assert_allclose(actual, expected, atol=1e-12)

def test_fk_chain_and_analytic_endpoint_agree(self):
    forward_kinematics = self.require_attr("forward_kinematics")
    theta1, theta2, theta3 = np.deg2rad([35.0, 20.0, -55.0])
    result = forward_kinematics(theta1, theta2, theta3)
    radial = 2.0 * np.cos(theta2) + 1.5 * np.cos(theta2 + theta3)
    expected = np.array([
        radial * np.cos(theta1),
        radial * np.sin(theta1),
        0.8 + 2.0 * np.sin(theta2) + 1.5 * np.sin(theta2 + theta3),
    ])
    np.testing.assert_allclose(result.origins[3], expected, atol=1e-12)
    np.testing.assert_allclose(
        result.t03, result.t01 @ result.t12 @ result.t23, atol=1e-12
    )
    self.assertEqual(result.origins.shape, (4, 3))

def test_fk_exposes_complete_dh_table(self):
    dh_parameters = self.require_attr("DHParameters")
    forward_kinematics = self.require_attr("forward_kinematics")
    result = forward_kinematics(0.1, 0.2, 0.3)
    self.assertEqual(result.dh_parameters, (
        dh_parameters(0.1, 0.0, np.pi / 2, 0.8),
        dh_parameters(0.2, 2.0, 0.0, 0.0),
        dh_parameters(0.3, 1.5, 0.0, 0.0),
    ))

def test_invalid_geometry_or_angles_are_rejected(self):
    forward_kinematics = self.require_attr("forward_kinematics")
    invalid_calls = [
        (0.0, 0.0, 0.0, -0.1, 2.0, 1.5),
        (0.0, 0.0, 0.0, 0.8, 0.0, 1.5),
        (0.0, 0.0, 0.0, 0.8, 2.0, np.inf),
        (np.nan, 0.0, 0.0, 0.8, 2.0, 1.5),
    ]
    for args in invalid_calls:
        with self.subTest(args=args), self.assertRaises(ValueError):
            forward_kinematics(*args)
```

- [ ] **Step 6: Run the numerical tests and verify RED**

Run: `python3 -m unittest test_robot_3r_3d_simulator.TestForwardKinematics -v`

Expected: FAIL because `dh_transform` and `forward_kinematics` are missing.

- [ ] **Step 7: Implement D-H records and pure FK**

Use these exact records and transform chain:

```python
@dataclass(frozen=True)
class DHParameters:
    theta: float
    a: float
    alpha: float
    d: float


@dataclass(frozen=True)
class ForwardKinematicsResult3D:
    t01: np.ndarray
    t12: np.ndarray
    t23: np.ndarray
    t02: np.ndarray
    t03: np.ndarray
    origins: np.ndarray
    dh_parameters: tuple


def dh_transform(theta, a, alpha, d):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=float)
```

Validate finite angles and geometry. Build rows `(theta1,0,pi/2,d1)`, `(theta2,l2,0,0)`, and `(theta3,l3,0,0)`. Compute `t02=t01@t12`, `t03=t02@t23`, and stack origins `[zeros(3), t01[:3,3], t02[:3,3], t03[:3,3]]`.

- [ ] **Step 8: Run all current tests and verify GREEN**

Run: `python3 -m unittest -v`

Expected: five tests pass without warnings.

- [ ] **Step 9: Commit the D-H and FK core**

```bash
git add robot_3r_3d_simulator.py test_robot_3r_3d_simulator.py
git commit -m "feat: add 3D 3R forward kinematics"
```

---

### Task 2: Spatial Transforms, RPY, Closed-Form IK, and Workspace

**Files:**
- Modify: `robot_3r_3d_simulator.py`
- Modify: `test_robot_3r_3d_simulator.py`

**Interfaces:**
- Consumes: Task 1's `forward_kinematics`.
- Produces: `UnreachableTargetError`, `IKSolutions(elbow_down, elbow_up, axis_singular)`, `invert_transform(transform)`, `rotation_to_rpy(rotation)`, `inverse_kinematics(target, d1=0.8, l2=2.0, l3=1.5, tolerance=1e-9)`, and `sample_workspace(d1=0.8, l2=2.0, l3=1.5, counts=(30,16,18))`.

- [ ] **Step 1: Write failing inverse-transform and RPY tests**

```python
def test_fast_inverse_matches_general_inverse(self):
    fk = self.require_attr("forward_kinematics")
    invert = self.require_attr("invert_transform")
    transform = fk(*np.deg2rad([40.0, -25.0, 70.0])).t03
    actual = invert(transform)
    np.testing.assert_allclose(actual, np.linalg.inv(transform), atol=1e-12)
    np.testing.assert_allclose(transform @ actual, np.eye(4), atol=1e-12)

def test_zyx_rpy_reconstructs_regular_and_gimbal_lock_rotations(self):
    rotation_to_rpy = self.require_attr("rotation_to_rpy")
    for source in [(0.3, -0.4, 0.8), (0.7, np.pi / 2, -0.2)]:
        with self.subTest(source=source):
            rotation = literal_zyx_rotation(*source)
            roll, pitch, yaw = rotation_to_rpy(rotation)
            reconstructed = literal_zyx_rotation(roll, pitch, yaw)
            np.testing.assert_allclose(reconstructed, rotation, atol=1e-10)
```

In the test file, implement `literal_zyx_rotation(roll,pitch,yaw)` directly from the three literal axis-rotation matrices; do not call production helpers:

```python
def literal_zyx_rotation(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx
```

- [ ] **Step 2: Write failing IK and workspace tests**

```python
def test_both_ik_branches_reproduce_one_target(self):
    ik = self.require_attr("inverse_kinematics")
    fk = self.require_attr("forward_kinematics")
    target = np.array([2.5, 0.0, 0.8])
    solutions = ik(target)
    self.assertGreater(solutions.elbow_down[2], 0.0)
    self.assertLess(solutions.elbow_up[2], 0.0)
    for angles in (solutions.elbow_down, solutions.elbow_up):
        np.testing.assert_allclose(fk(*angles).origins[3], target, atol=1e-10)

def test_ik_reports_unreachable_boundary_and_axis_singularity(self):
    ik = self.require_attr("inverse_kinematics")
    unreachable = self.require_attr("UnreachableTargetError")
    with self.assertRaises(unreachable):
        ik(np.array([4.0, 0.0, 0.8]))
    boundary = ik(np.array([3.5, 0.0, 0.8]))
    np.testing.assert_allclose(boundary.elbow_down, boundary.elbow_up, atol=1e-12)
    singular = ik(np.array([0.0, 0.0, 3.8]))
    self.assertTrue(singular.axis_singular)
    self.assertEqual(singular.elbow_down[0], 0.0)

def test_workspace_sampler_is_finite_and_has_requested_size(self):
    sample_workspace = self.require_attr("sample_workspace")
    points = sample_workspace(counts=(12, 6, 8))
    self.assertEqual(points.shape, (12 * 6 * 8, 3))
    self.assertTrue(np.isfinite(points).all())
    radial_distance = np.linalg.norm(points - np.array([0.0, 0.0, 0.8]), axis=1)
    self.assertLessEqual(radial_distance.max(), 3.5 + 1e-12)
```

- [ ] **Step 3: Run Task 2 tests and verify RED**

Run: `python3 -m unittest test_robot_3r_3d_simulator.TestSpatialTransforms test_robot_3r_3d_simulator.TestInverseKinematics -v`

Expected: FAIL with missing `invert_transform`, `rotation_to_rpy`, or `inverse_kinematics` assertions.

- [ ] **Step 4: Implement transform inverse and ZYX RPY extraction**

For `invert_transform`, validate shape `(4,4)`, finite elements, last row `[0,0,0,1]`, and `R.T @ R≈I`; return identity with `R.T` and translation `-R.T @ p`.

For `rotation_to_rpy`, compute:

```python
horizontal = np.hypot(rotation[0, 0], rotation[1, 0])
pitch = np.arctan2(-rotation[2, 0], horizontal)
if horizontal > 1e-10:
    roll = np.arctan2(rotation[2, 1], rotation[2, 2])
    yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
elif pitch > 0.0:
    roll = np.arctan2(rotation[0, 1], rotation[0, 2])
    yaw = 0.0
else:
    roll = np.arctan2(-rotation[0, 1], -rotation[0, 2])
    yaw = 0.0
```

- [ ] **Step 5: Implement closed-form IK and workspace sampling**

Use:

```python
@dataclass(frozen=True)
class IKSolutions:
    elbow_down: np.ndarray
    elbow_up: np.ndarray
    axis_singular: bool


class UnreachableTargetError(ValueError):
    pass
```

Validate target shape `(3,)`, finite geometry, and finite tolerance. Compute `D`; reject values outside `[-1-tolerance,1+tolerance]`, clamp round-off, and construct `theta3=±arccos(D)` plus the corresponding `theta2`. Set `theta1=0` when radial distance is below `1e-10`.

In `sample_workspace`, use `np.meshgrid` over the global joint ranges, evaluate the analytic FK position equations in vectorized form, and return flattened `(N,3)` points.

- [ ] **Step 6: Run the full test file and verify GREEN**

Run: `python3 -m unittest -v`

Expected: all numerical tests pass.

- [ ] **Step 7: Commit spatial math and IK**

```bash
git add robot_3r_3d_simulator.py test_robot_3r_3d_simulator.py
git commit -m "feat: add 3D pose transforms and closed-form IK"
```

---

### Task 3: Static 3D Scene, Pose Panel, Formula Panel, and Controls

**Files:**
- Modify: `robot_3r_3d_simulator.py`
- Modify: `test_robot_3r_3d_simulator.py`

**Interfaces:**
- Consumes: all Task 1 and 2 numerical APIs.
- Produces: `Robot3DSimulator(d1=0.8, l2=2.0, l3=1.5, theta1_deg=30.0, theta2_deg=25.0, theta3_deg=-50.0)` with `fig`, `ax3d`, `pose_ax`, `math_ax`, `arm_line`, `position_vector`, `workspace_scatter`, `frame_artists`, `frame_labels`, `transform_labels`, `pose_text`, `dh_table`, `joint_sliders`, `target_sliders`, `mode_button`, `solution_radio`, and `status_text`.

- [ ] **Step 1: Write failing static-GUI tests**

```python
def test_gui_has_3d_scene_frames_workspace_and_panels(self):
    simulator_cls = self.require_attr("Robot3DSimulator")
    simulator = simulator_cls()
    self.assertEqual(len(simulator.arm_line.get_xdata()), 4)
    self.assertEqual(len(simulator.frame_artists), 12)
    self.assertEqual(len(simulator.frame_labels), 4)
    self.assertEqual(len(simulator.transform_labels), 3)
    x, y, z = simulator.workspace_scatter._offsets3d
    self.assertGreater(len(x), 7_000)
    self.assertEqual(len(x), len(y))
    self.assertEqual(len(y), len(z))
    self.assertIn("T_0^3", simulator.pose_text.get_text())
    self.assertIn("T^{-1}", simulator.formula_text.get_text())

def test_gui_controls_have_required_defaults_and_ranges(self):
    simulator_cls = self.require_attr("Robot3DSimulator")
    simulator = simulator_cls()
    expected_joint_ranges = [(-180.0, 180.0), (-90.0, 90.0), (-150.0, 150.0)]
    for slider, expected in zip(simulator.joint_sliders, expected_joint_ranges):
        self.assertEqual((slider.valmin, slider.valmax), expected)
    expected_target_ranges = [(-3.5, 3.5), (-3.5, 3.5), (-2.7, 4.3)]
    for slider, expected in zip(simulator.target_sliders, expected_target_ranges):
        self.assertEqual((slider.valmin, slider.valmax), expected)
    self.assertEqual(simulator.mode, "FK")
    self.assertEqual(simulator.mode_button.label.get_text(), "Mode: FK")
    self.assertEqual(
        [label.get_text() for label in simulator.solution_radio.labels],
        ["Solution 1: Elbow Down", "Solution 2: Elbow Up"],
    )
```

- [ ] **Step 2: Run the static GUI tests and verify RED**

Run: `python3 -m unittest test_robot_3r_3d_simulator.TestRobot3DGUI -v`

Expected: FAIL because `Robot3DSimulator` is missing.

- [ ] **Step 3: Implement figure layout and static scene**

Create a `16×10` figure. Use a two-column GridSpec with the 3D axes spanning both left rows, the pose panel at upper right, the formula/D-H panel at lower right, and reserve the bottom 30% for widgets.

```python
self.fig = plt.figure(figsize=(16.0, 10.0))
grid = self.fig.add_gridspec(
    2, 2, left=0.04, right=0.98, bottom=0.31, top=0.94,
    width_ratios=(1.65, 1.0), height_ratios=(1.12, 1.0),
    wspace=0.18, hspace=0.18,
)
self.ax3d = self.fig.add_subplot(grid[:, 0], projection="3d")
self.pose_ax = self.fig.add_subplot(grid[0, 1])
self.math_ax = self.fig.add_subplot(grid[1, 1])
self.pose_ax.set_axis_off()
self.math_ax.set_axis_off()
```

Plot the workspace once with `scatter(..., s=2, c="#8FD3E8", alpha=0.055, depthshade=False)`. Plot the arm as a thick dark-blue `Line3D` with four circular markers and a dashed base-to-tool `position_vector`. Set equal limits around the shoulder and `set_box_aspect((1,1,1))`.

Implement `_draw_coordinate_frames(result)` by removing prior frame artists and creating 12 quivers from the rotation columns of identity, `T01`, `T02`, and `T03`. Create labels `{B}`, `{1}`, `{2}`, `{W}/{T}` plus `T01/T12/T23` labels at link midpoints.

- [ ] **Step 4: Implement pose/formula panels and controls**

Create the dynamic D-H table with columns `i, a_(i-1), alpha_(i-1), d_i, theta_i` and three data rows. Add a static mathtext formula block containing:

```text
T_(i-1)^i = Rz(theta_i) Tz(d_i) Tx(a_(i-1)) Rx(alpha_(i-1))
T_0^3 = T_0^1 T_1^2 T_2^3
T^-1 = [R^T, -R^T P; 0, 1]
D = (r^2 + z'^2 - L2^2 - L3^2) / (2 L2 L3)
theta3 = +/- acos(D)
```

Create three joint sliders on the lower-left, three target sliders in the lower-center, a mode button, a solution RadioButtons group, and a figure-level status text. Initialize target values from the default FK endpoint. Dim target axes in FK mode.

- [ ] **Step 5: Implement initial visual update without callbacks**

Implement `_update_visuals(result, status, status_color)` to update the arm, position vector, frames, labels, pose panel, D-H angle cells, and status. Format matrices to three decimals after replacing values smaller than `5e-13` with zero. Include position, R03, T03, RPY in degrees, T30, and `||T03 @ T30 - I||_F`.

Call `_update_visuals` once at the end of construction, but do not register widget callbacks until Task 4.

- [ ] **Step 6: Run the full test file and verify GREEN**

Run: `python3 -m unittest -v`

Expected: all numerical and static-GUI tests pass.

- [ ] **Step 7: Commit the static educational GUI**

```bash
git add robot_3r_3d_simulator.py test_robot_3r_3d_simulator.py
git commit -m "feat: add 3D robot pose and formula visualization"
```

---

### Task 4: FK/IK Mode Interaction, Multi-Solution Switching, and Visual Verification

**Files:**
- Modify: `robot_3r_3d_simulator.py`
- Modify: `test_robot_3r_3d_simulator.py`
- Generate: `artifacts/robot_3r_3d_preview.png`

**Interfaces:**
- Consumes: all previous tasks.
- Produces: `_on_joint_change`, `_on_target_change`, `_on_solution_change`, `_toggle_mode`, `_set_mode`, `_solution_within_limits`, `show()`, and `main()`.

- [ ] **Step 1: Write failing FK/IK interaction tests**

```python
def test_fk_sliders_update_endpoint_and_dynamic_dh_table(self):
    simulator_cls = self.require_attr("Robot3DSimulator")
    simulator = simulator_cls()
    for slider, value in zip(simulator.joint_sliders, (0.0, 0.0, 0.0)):
        slider.set_val(value)
    np.testing.assert_allclose(
        simulator.current_result.origins[3], [3.5, 0.0, 0.8], atol=1e-12
    )
    self.assertIn("0.0 deg", simulator.dh_table[(1, 4)].get_text().get_text())

def test_switch_to_ik_preserves_pose_and_syncs_target(self):
    simulator_cls = self.require_attr("Robot3DSimulator")
    simulator = simulator_cls()
    endpoint_before = simulator.current_result.origins[3].copy()
    simulator._toggle_mode(None)
    self.assertEqual(simulator.mode, "IK")
    np.testing.assert_allclose(
        [slider.val for slider in simulator.target_sliders], endpoint_before, atol=1e-10
    )
    np.testing.assert_allclose(simulator.current_result.origins[3], endpoint_before, atol=1e-10)

def test_ik_target_and_radio_switch_between_two_valid_branches(self):
    simulator_cls = self.require_attr("Robot3DSimulator")
    simulator = simulator_cls()
    simulator._toggle_mode(None)
    for slider, value in zip(simulator.target_sliders, (2.5, 0.0, 0.8)):
        slider.set_val(value)
    endpoint_down = simulator.current_result.origins[3].copy()
    elbow_down = simulator.current_result.origins[2].copy()
    simulator.solution_radio.set_active(1)
    endpoint_up = simulator.current_result.origins[3].copy()
    elbow_up = simulator.current_result.origins[2].copy()
    np.testing.assert_allclose(endpoint_down, [2.5, 0.0, 0.8], atol=1e-9)
    np.testing.assert_allclose(endpoint_up, [2.5, 0.0, 0.8], atol=1e-9)
    self.assertFalse(np.allclose(elbow_down, elbow_up))

def test_unreachable_ik_target_preserves_last_valid_pose(self):
    simulator_cls = self.require_attr("Robot3DSimulator")
    simulator = simulator_cls()
    simulator._toggle_mode(None)
    last_valid = simulator.current_result.origins.copy()
    simulator.target_sliders[0].set_val(3.5)
    simulator.target_sliders[1].set_val(3.5)
    self.assertIn("unreachable", simulator.status_text.get_text().lower())
    np.testing.assert_allclose(simulator.current_result.origins, last_valid, atol=1e-12)
```

- [ ] **Step 2: Run interaction tests and verify RED**

Run: `python3 -m unittest test_robot_3r_3d_simulator.TestRobot3DInteraction -v`

Expected: FAIL because widgets are not connected to mode and update callbacks.

- [ ] **Step 3: Implement guarded callbacks and mode state**

Register all slider, button, and RadioButtons callbacks. Each callback returns immediately when `_updating_controls` is true or its mode is inactive.

```python
for slider in self.joint_sliders:
    slider.on_changed(self._on_joint_change)
for slider in self.target_sliders:
    slider.on_changed(self._on_target_change)
self.mode_button.on_clicked(self._toggle_mode)
self.solution_radio.on_clicked(self._on_solution_change)

def _on_joint_change(self, _value):
    if self._updating_controls or self.mode != "FK":
        return
    angles = np.deg2rad([slider.val for slider in self.joint_sliders])
    result = forward_kinematics(*angles, self.d1, self.l2, self.l3)
    self.current_angles = angles
    self._update_visuals(result, "FK mode", "#1F4E79")

def _on_target_change(self, _value):
    if self._updating_controls or self.mode != "IK":
        return
    self._apply_selected_ik_solution()
```

In `_toggle_mode`:

1. When entering IK, derive both IK solutions for the current endpoint.
2. Select the solution with the smallest wrapped angular distance to the current angles.
3. Under `_updating_controls=True`, synchronize target sliders and the RadioButtons selection.
4. Enable target sliders, dim joint sliders, set `mode="IK"`, and update without jumping.
5. When returning to FK, synchronize joint sliders to current angles, reactivate them, and dim target controls.

In `_on_target_change` and `_on_solution_change`, catch `UnreachableTargetError`. Before accepting a solution, convert it to degrees and require theta ranges to fit all three configured joint limits; otherwise display `Target requires angles outside configured joint limits` and preserve the last pose.

- [ ] **Step 4: Add executable entry point and render test**

```python
def test_headless_3d_figure_can_render(self):
    simulator_cls = self.require_attr("Robot3DSimulator")
    simulator = simulator_cls()
    output = io.BytesIO()
    simulator.fig.savefig(output, format="png", dpi=100)
    self.assertGreater(len(output.getvalue()), 50_000)
```

Add:

```python
def main():
    simulator = Robot3DSimulator()
    simulator.show()


if __name__ == "__main__":
    main()
```

- [ ] **Step 5: Run complete unit, syntax, and render verification**

Run:

```bash
python3 -m unittest -v
python3 -m py_compile robot_3r_3d_simulator.py test_robot_3r_3d_simulator.py
mkdir -p artifacts
MPLBACKEND=Agg python3 -c 'from robot_3r_3d_simulator import Robot3DSimulator; s=Robot3DSimulator(); s.fig.savefig("artifacts/robot_3r_3d_preview.png", dpi=140)'
```

Expected: all tests pass, compilation exits 0, and the preview PNG is nonempty.

- [ ] **Step 6: Inspect the preview**

Confirm the image contains an unclipped 3D arm, workspace cloud, four red/green/blue frames, `{B}/{1}/{2}/{W}/{T}` labels, transform labels, pose matrices, RPY, inverse check, formulas, dynamic D-H table, six sliders, mode button, solution radio group, and status text.

- [ ] **Step 7: Commit the completed simulator**

```bash
git add robot_3r_3d_simulator.py test_robot_3r_3d_simulator.py artifacts/robot_3r_3d_preview.png
git commit -m "feat: complete interactive 3D 3R simulator"
```
