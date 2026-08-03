# Planar 2R Kinematics Simulator Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a directly runnable interactive planar 2R forward-kinematics simulator with standard D-H transforms, live sliders, local coordinate frames, workspace boundaries, and numerical pose output.

**Architecture:** Keep the numerical model in pure functions returning immutable dataclasses, and isolate all Matplotlib state in `Planar2RSimulator`. The GUI owns reusable artists and updates them in place whenever either angle slider changes.

**Tech Stack:** Python standard library (`dataclasses`, `unittest`), NumPy, Matplotlib (`pyplot`, `patches.Circle`, `widgets.Slider`).

## Global Constraints

- The runnable simulator is a single file named `planar_2r_simulator.py`.
- Runtime dependencies are limited to NumPy and Matplotlib.
- Default lengths are exactly `L1 = 2.0` and `L2 = 1.5`.
- Initial angles are exactly `theta1 = 30°` and `theta2 = 45°`.
- Both slider ranges are exactly `[-180°, 180°]`.
- Standard D-H transforms use radians internally and `T02 = T01 @ T12`.
- Local X axes are red and local Y axes are green at `O0`, `O1`, and `O2`.
- The workspace shows radii `L1 + L2` and `abs(L1 - L2)`.

## File Structure

- `planar_2r_simulator.py`: D-H dataclasses, pure kinematics functions, Matplotlib GUI class, and executable `main()` entry point.
- `test_planar_2r_simulator.py`: standard-library unit and headless GUI tests; sets the Matplotlib backend to `Agg` before importing the simulator.
- `artifacts/planar_2r_preview.png`: generated verification preview; not required by the runtime.

---

### Task 1: Pure D-H and Forward-Kinematics Core

**Files:**
- Create: `planar_2r_simulator.py`
- Create: `test_planar_2r_simulator.py`

**Interfaces:**
- Consumes: NumPy scalar trigonometry and matrix multiplication.
- Produces: `DHParameters(theta: float, a: float, alpha: float, d: float)`, `ForwardKinematicsResult(t01, t12, t02, origins, dh_parameters)`, `dh_transform(theta, a, alpha, d) -> np.ndarray`, and `forward_kinematics(theta1, theta2, l1, l2) -> ForwardKinematicsResult`.

- [ ] **Step 1: Write a failing import test**

```python
import importlib
import unittest


class TestModuleAvailability(unittest.TestCase):
    def test_simulator_module_can_be_imported(self):
        spec = importlib.util.find_spec("planar_2r_simulator")
        self.assertIsNotNone(spec, "planar_2r_simulator.py does not exist")
```

- [ ] **Step 2: Run the import test and verify RED**

Run: `python -m unittest test_planar_2r_simulator.TestModuleAvailability -v`

Expected: FAIL because `find_spec("planar_2r_simulator")` returns `None`.

- [ ] **Step 3: Create the minimal importable module**

```python
"""Interactive planar 2R robot forward-kinematics simulator."""
```

- [ ] **Step 4: Run the import test and verify GREEN**

Run: `python -m unittest test_planar_2r_simulator.TestModuleAvailability -v`

Expected: one passing test.

- [ ] **Step 5: Add failing numerical behavior tests**

Set `matplotlib.use("Agg")` before importing the module. Replace the initial test module with the following imports and helper, then add the numerical test methods to `TestKinematics`. The helper makes a missing API a test failure rather than an attribute error.

```python
import importlib
import unittest

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

simulator_module = importlib.import_module("planar_2r_simulator")


class SimulatorTestCase(unittest.TestCase):
    def require_attr(self, name):
        self.assertTrue(
            hasattr(simulator_module, name),
            "planar_2r_simulator is missing {!r}".format(name),
        )
        return getattr(simulator_module, name)

    def tearDown(self):
        plt.close("all")


class TestKinematics(SimulatorTestCase):
def test_zero_angle_dh_transform(self):
    dh_transform = self.require_attr("dh_transform")
    actual = dh_transform(0.0, 2.0, 0.0, 0.0)
    expected = np.array([[1., 0., 0., 2.],
                         [0., 1., 0., 0.],
                         [0., 0., 1., 0.],
                         [0., 0., 0., 1.]])
    np.testing.assert_allclose(actual, expected, atol=1e-12)

def test_reference_forward_kinematics_poses(self):
    forward_kinematics = self.require_attr("forward_kinematics")
    cases = [
        ((0.0, 0.0), (3.5, 0.0)),
        ((np.pi / 2.0, 0.0), (0.0, 3.5)),
        ((0.0, np.pi), (0.5, 0.0)),
    ]
    for angles, expected_o2 in cases:
        with self.subTest(angles=angles):
            result = forward_kinematics(*angles, 2.0, 1.5)
            np.testing.assert_allclose(result.origins[2], expected_o2, atol=1e-12)
            np.testing.assert_allclose(result.t02, result.t01 @ result.t12, atol=1e-12)

def test_nonpositive_or_nonfinite_link_lengths_are_rejected(self):
    forward_kinematics = self.require_attr("forward_kinematics")
    for lengths in [(0.0, 1.5), (-1.0, 1.5), (2.0, np.inf), (np.nan, 1.5)]:
        with self.subTest(lengths=lengths):
            with self.assertRaisesRegex(ValueError, "finite positive"):
                forward_kinematics(0.0, 0.0, *lengths)
```

- [ ] **Step 6: Run numerical tests and verify RED**

Run: `python -m unittest test_planar_2r_simulator.TestKinematics -v`

Expected: FAIL with an assertion that `dh_transform` is missing.

- [ ] **Step 7: Implement immutable parameter/result records and pure functions**

Use the exact standard D-H matrix:

```python
@dataclass(frozen=True)
class DHParameters:
    theta: float
    a: float
    alpha: float = 0.0
    d: float = 0.0


@dataclass(frozen=True)
class ForwardKinematicsResult:
    t01: np.ndarray
    t12: np.ndarray
    t02: np.ndarray
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

In `forward_kinematics`, reject any `l1` or `l2` for which `not np.isfinite(length) or length <= 0.0`. Build `DHParameters(theta1, l1)` and `DHParameters(theta2, l2)`, compute `t01`, `t12`, `t02`, and return origins `[[0, 0], t01[:2, 3], t02[:2, 3]]` as a `(3, 2)` float array.

- [ ] **Step 8: Run the full test file and verify GREEN**

Run: `python -m unittest -v`

Expected: all import and kinematics tests pass without warnings.

- [ ] **Step 9: Commit the numerical core**

```bash
git add planar_2r_simulator.py test_planar_2r_simulator.py
git commit -m "feat: add planar 2R kinematics core"
```

---

### Task 2: Static Simulator Layout and Visual Elements

**Files:**
- Modify: `planar_2r_simulator.py`
- Modify: `test_planar_2r_simulator.py`

**Interfaces:**
- Consumes: `forward_kinematics(theta1, theta2, l1, l2)` from Task 1.
- Produces: `Planar2RSimulator(l1=2.0, l2=1.5, theta1_deg=30.0, theta2_deg=45.0)`, with `fig`, `ax`, `info_ax`, `arm_line`, `workspace_circles`, `frame_lines`, `frame_labels`, `position_text`, `matrix_text`, `theta1_slider`, and `theta2_slider` attributes.

- [ ] **Step 1: Write failing static-GUI tests**

```python
class TestSimulatorGUI(SimulatorTestCase):
def test_gui_contains_required_visual_elements(self):
    simulator_cls = self.require_attr("Planar2RSimulator")
    simulator = simulator_cls()
    self.assertEqual(simulator.arm_line.get_marker(), "o")
    self.assertEqual(len(simulator.workspace_circles), 2)
    self.assertAlmostEqual(simulator.workspace_circles[0].radius, 3.5)
    self.assertAlmostEqual(simulator.workspace_circles[1].radius, 0.5)
    self.assertEqual(len(simulator.frame_lines), 3)
    for x_line, y_line in simulator.frame_lines:
        self.assertEqual(x_line.get_color(), "red")
        self.assertEqual(y_line.get_color(), "green")
    self.assertEqual(simulator.theta1_slider.valmin, -180.0)
    self.assertEqual(simulator.theta1_slider.valmax, 180.0)
    self.assertEqual(simulator.theta2_slider.valmin, -180.0)
    self.assertEqual(simulator.theta2_slider.valmax, 180.0)
```

- [ ] **Step 2: Run the GUI test and verify RED**

Run: `python -m unittest test_planar_2r_simulator.TestSimulatorGUI.test_gui_contains_required_visual_elements -v`

Expected: FAIL because `Planar2RSimulator` is missing.

- [ ] **Step 3: Implement the GUI constructor and static drawing**

Create a `13×7.5` inch figure with a left plot and right information panel, reserving the bottom for sliders. Set an equal aspect ratio and axis limits to `±1.15 * (l1 + l2)`. Add:

- two centered `Circle` patches with radii `l1 + l2` and `abs(l1 - l2)`;
- one blue arm line with circular nodes;
- three pairs of coordinate-frame lines, with red X and green Y;
- `O0`, `O1`, `O2` labels;
- two monospace text objects in `info_ax`;
- two `Slider` objects with the required limits and initial values.

Use a private `_set_frame_artist(index, origin, rotation)` helper. Its endpoint equations are `origin + frame_axis_length * rotation[:, 0]` and `origin + frame_axis_length * rotation[:, 1]`, where `frame_axis_length = 0.16 * (l1 + l2)`.

- [ ] **Step 4: Run the full test file and verify GREEN**

Run: `python -m unittest -v`

Expected: all tests pass.

- [ ] **Step 5: Commit the static GUI**

```bash
git add planar_2r_simulator.py test_planar_2r_simulator.py
git commit -m "feat: add planar 2R simulator layout"
```

---

### Task 3: Live Updates, Executable Entry Point, and Visual Verification

**Files:**
- Modify: `planar_2r_simulator.py`
- Modify: `test_planar_2r_simulator.py`
- Generate: `artifacts/planar_2r_preview.png`

**Interfaces:**
- Consumes: all interfaces from Tasks 1 and 2.
- Produces: slider callback `_update(_value=None)`, `show()`, and `main()`.

- [ ] **Step 1: Write failing interaction and render tests**

```python
class TestSimulatorGUI(SimulatorTestCase):
def test_sliders_update_arm_pose_and_information(self):
    simulator_cls = self.require_attr("Planar2RSimulator")
    simulator = simulator_cls()
    simulator.theta1_slider.set_val(0.0)
    simulator.theta2_slider.set_val(0.0)
    np.testing.assert_allclose(simulator.arm_line.get_xdata(), [0.0, 2.0, 3.5], atol=1e-12)
    np.testing.assert_allclose(simulator.arm_line.get_ydata(), [0.0, 0.0, 0.0], atol=1e-12)
    self.assertIn("x = 3.500", simulator.position_text.get_text())
    self.assertIn("T_0^2", simulator.matrix_text.get_text())

def test_headless_figure_can_be_rendered(self):
    simulator_cls = self.require_attr("Planar2RSimulator")
    simulator = simulator_cls()
    output = io.BytesIO()
    simulator.fig.savefig(output, format="png", dpi=100)
    self.assertGreater(len(output.getvalue()), 10_000)
```

- [ ] **Step 2: Run the interaction tests and verify RED**

Run: `python -m unittest test_planar_2r_simulator.TestSimulatorGUI -v`

Expected: FAIL because slider changes do not yet update the existing artists and text.

- [ ] **Step 3: Implement live updates**

Register both sliders with `on_changed(self._update)`. In `_update`:

1. Convert both slider degrees with `np.deg2rad`.
2. Call `forward_kinematics` and store the result as `self.current_result`.
3. Set arm x/y data from `result.origins`.
4. Update frames using identity, `result.t01[:2, :2]`, and `result.t02[:2, :2]`.
5. Move all origin labels.
6. Format the endpoint with three decimals.
7. Format `T02` using `np.array2string(..., precision=3, suppress_small=True, floatmode="fixed")`, after replacing values with absolute magnitude below `5e-13` by zero.
8. Call `self.fig.canvas.draw_idle()`.

Call `_update()` once at the end of construction so the initial view and information match the default angles.

- [ ] **Step 4: Add the executable entry point**

```python
def main():
    simulator = Planar2RSimulator()
    simulator.show()


if __name__ == "__main__":
    main()
```

- [ ] **Step 5: Run unit, syntax, and headless-render verification**

Run:

```bash
python -m unittest -v
python -m py_compile planar_2r_simulator.py test_planar_2r_simulator.py
mkdir -p artifacts
MPLBACKEND=Agg python -c 'from planar_2r_simulator import Planar2RSimulator; s=Planar2RSimulator(); s.fig.savefig("artifacts/planar_2r_preview.png", dpi=140)'
```

Expected: all tests pass; compilation exits with status 0; preview PNG exists and is nonempty.

- [ ] **Step 6: Inspect the preview image**

Confirm the preview visibly contains the full two-link arm, three red/green coordinate frames, inner and outer workspace circles, two sliders, endpoint values, and a legible `4×4` matrix without clipping.

- [ ] **Step 7: Commit the finished simulator**

```bash
git add planar_2r_simulator.py test_planar_2r_simulator.py artifacts/planar_2r_preview.png
git commit -m "feat: complete interactive planar 2R simulator"
```
