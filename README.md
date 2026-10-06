# CableRobot

A standalone Python foundation for cable-driven multibody systems. A cable is a
route through points on arbitrary body-attached frames. The same model supports a
floating spatial platform, an articulated serial mechanism, and their hybrids.

Milestone 1 uses massless, straight, tension-only cable segments. Python owns robot
topology and semantics. NumPy is the working numerical backend; MATLAB is an optional
adapter boundary.

Milestone 1 is **COMPLETED — ACCEPTED FOUNDATION**; see its
[permanent milestone record](docs/milestones/milestone_01_foundation.md).
Milestone 2, **Pose-Dependent Tension Allocation and Gravity Compensation**, is
**IMPLEMENTED — AWAITING REVIEW**. See the [formulation and decisions](docs/tension_allocation.md)
and the temporary `.review/review_summary.md` handoff. No later milestone has begun.
The separately authorized [performance study](docs/performance_results.md) is also
**IMPLEMENTED — AWAITING REVIEW**; it preserves Milestone 2's pending review package.
Read [contribution instructions](CONTRIBUTING.md) and the required
[development workflow](docs/development_workflow.md) before changing it.

## Install and run

Python 3.11 or newer is required.

```sh
python -m venv .venv
# Windows: .venv\Scripts\activate
# macOS/Linux: source .venv/bin/activate
python -m pip install -e ".[dev]"
python -m pytest
python examples/spatial_cdpr.py --output examples/output/cdpr.png
python examples/serial_robot.py --output examples/output/serial.png
python -m examples.animate_robots --duration 5 --fps 20 --format gif
python -m examples.equilibrium_allocation --output-dir examples/output/equilibrium
python -m benchmarks.control_pipeline --output-dir examples/output/performance/study
```

Omit `--output` to open an interactive Matplotlib figure. For headless environments,
set `MPLBACKEND=Agg` before running examples.

The animation example renders spatial, serial and hybrid robots individually and
side by side. It uses prescribed kinematic trajectories. Optional MP4 output requires
FFmpeg: use `--format both --ffmpeg /path/to/ffmpeg` to export GIF and MP4. All route
points move with their declared bodies. Blue cables touch the outer frame; orange
cables have every attachment and guide on the moving robot.

## Small public API

```python
from cablerobot import gravity_generalized_force, solve_equilibrium_tensions
from cablerobot.examples import spatial_cdpr, serial_mechanism, serial_routed_mechanism, hybrid_mechanism
from cablerobot.visualization import plot_robot

robot, state = spatial_cdpr()
lengths = robot.cable_lengths(state)
J_l = robot.cable_jacobian(state)
B = robot.cable_force_matrix(state)  # B = -J_l.T
gravity = gravity_generalized_force(robot, state)
allocation = solve_equilibrium_tensions(robot, state, reference_tension=20.0)
if allocation.feasible:
    plot_robot(robot, state, tensions=allocation.tensions)
```

For programmatic construction use `CableRobot`, `Body`, `Joint`, `Frame`, `Cable`,
`CableRoute`, and `Attachment`. Every body automatically has a same-named frame.
Root bodies are fixed in world; use a floating joint to represent a free rigid body.
Coordinates follow joint insertion order. Robot models can round-trip through
versioned JSON with `cablerobot.io.save_robot` and `load_robot`.

## Capabilities and limits

- Tree/forest multibody transforms with fixed, revolute, prismatic and floating joints.
- Arbitrary frame-to-frame cable routes, lengths and generalized cable Jacobians.
- Generalized gravity at offset centers of mass, equilibrium residuals and bounded tension allocation.
- Local length-to-configuration, selected-frame position and full-pose nonlinear solvers.
- Selected-body cable wrench matrices, selected-frame twist Jacobians, force-set vertices, rank and basic manipulability.
- Validated forward/inverse dynamics contracts for externally supplied numerical mass and bias terms, plus static gravity special cases.
- Matplotlib rendering of bodies, joints, frames, routes, cable labels and tensions.
- Sampled periodic kinematic trajectories and synchronized spatial/serial/hybrid GIF or MP4 animations.
- NumPy backend and lazy MATLAB availability detection with explicit stub errors.

Cable and point Jacobians now use generic analytic derivatives for the supported
straight, body-fixed routes and fixed/revolute/prismatic/floating tree joints.
Centralized finite differences remain an explicit validation/reference path:
`robot.cable_jacobian(state, method="finite_difference")`; a supplied low-level
`step` retains numerical differentiation. See the [derivation/compatibility decisions](docs/performance_study.md).
Local kinematic solvers require a reasonable initial guess and do not resolve global
ambiguity. Degenerate zero-length segments have no differentiable cable direction.
The foundation `allocate_tensions`/`solve_tension_allocation` remain unchanged.
For at most eight cables, this baseline exhaustively checks box active sets; larger
problems use projected least squares and can report `numerically_unresolved`.
Infeasibility requires a separating certificate; a stalled iteration is not proof
that equilibrium is impossible.
The reference allocator prioritizes feasibility, with no guarantee of a preferred
minimum-norm or energy-optimal tension distribution. See the documented tolerances.
It is a **bounded equilibrium / reference numerical allocator**: "reference" means
a numerical baseline for validation, not tracking a reference tension vector.
The separate Milestone 2 `allocate_reference_tensions`/`solve_equilibrium_tensions`
minimize `0.5 ||W(t - t_ref)||^2` subject to FULL generalized equilibrium and bounds.
Scalar or per-cable references and positive diagonal weights are supported. Results
include objective/dual diagnostics, active bounds and lower/upper reserves.
The NumPy dual solver checks equilibrium and numerical optimality; inconclusive
solves remain unresolved, with no `tension_command`. This is static model allocation,
not real-time, friction compensation or experimental proof of uniform operator feel.
The example compares equal 20 N with equilibrium allocation at individual poses and
along an 81-pose translational sweep, including a deliberately infeasible bounds case.

`analyze_wrench_workspace` currently analyzes **one configuration**, returning the
bounded generalized-force set and optionally a feasibility result. Full workspace
sampling, wrench closure and stiffness analysis are planned interfaces.

Full dynamics, dynamic simulation, cable sag/mass/friction/contact/slack/elastic dynamics,
motors, flexible links, closed kinematic loops and hardware communication are deferred.

## Documentation

- [Conventions](docs/conventions.md)
- [Architecture and milestone boundaries](docs/architecture.md)
- [Numerical backends and MATLAB boundary](docs/backends.md)
- [Pose-dependent tension allocation](docs/tension_allocation.md)
- [Computational performance study and measured results](docs/performance_results.md)
- [Validation report](docs/validation.md)
- [Serial/hybrid cable routing and animations](docs/routing_animation.md)
- [Review correction decisions](docs/review_corrections.md)
- [Milestone record index](docs/milestones/README.md)

Repository layout follows the specification: `model`, `kinematics`, `statics`,
`allocation`, `dynamics`, `analysis`, `simulation`, `backends`, `visualization`, `io`,
plus `tests`, `examples`, and `docs`. Shared transform math is in `model/transforms.py`;
reference constructors are in `cablerobot/examples.py` so examples and tests use the
same models. No existing RobotModel code is copied.
