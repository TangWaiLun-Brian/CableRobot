# CableRobot

A standalone Python foundation for cable-driven multibody systems. A cable is a
route through points on arbitrary body-attached frames. The same model supports a
floating spatial platform, an articulated serial mechanism, and their hybrids.

Milestone 1 uses massless, straight, tension-only cable segments. Python owns robot
topology and semantics. NumPy is the working numerical backend; MATLAB is an optional
adapter boundary.

Status: **COMPLETED — ACCEPTED FOUNDATION**. The foundation and routed-animation
milestone was accepted after its documentation amendment. See the
[permanent milestone record](docs/milestones/milestone_01_foundation.md).
No next milestone has begun.
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
from cablerobot import gravity_generalized_force, solve_tension_allocation
from cablerobot.examples import spatial_cdpr, serial_mechanism, serial_routed_mechanism, hybrid_mechanism
from cablerobot.visualization import plot_robot

robot, state = spatial_cdpr()
lengths = robot.cable_lengths(state)
J_l = robot.cable_jacobian(state)
B = robot.cable_force_matrix(state)  # B = -J_l.T
gravity = gravity_generalized_force(robot, state)
allocation = solve_tension_allocation(robot, state)
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

Jacobians use centralized numerical differentiation in this reference release.
Local kinematic solvers require a reasonable initial guess and do not resolve global
ambiguity. Degenerate zero-length segments have no differentiable cable direction.
For at most eight cables, allocation exhaustively checks box active sets; larger
problems use projected least squares and can report `numerically_unresolved`.
Infeasibility requires a separating certificate; a stalled iteration is not proof
that equilibrium is impossible.
The reference allocator prioritizes feasibility, with no guarantee of a preferred
minimum-norm or energy-optimal tension distribution. See the documented tolerances.
It is a **bounded equilibrium / reference numerical allocator**: "reference" means
a numerical baseline for validation, not tracking a reference tension vector.
Neither the public allocator nor its backend problem exposes a configurable
`t_ref` objective such as minimizing `||t - t_ref||_2^2`; that remains future work.

`analyze_wrench_workspace` currently analyzes **one configuration**, returning the
bounded generalized-force set and optionally a feasibility result. Full workspace
sampling, wrench closure and stiffness analysis are planned interfaces.

Full dynamics, dynamic simulation, cable sag/mass/friction/contact/slack/elastic dynamics,
motors, flexible links, closed kinematic loops and hardware communication are deferred.

## Documentation

- [Conventions](docs/conventions.md)
- [Architecture and milestone boundaries](docs/architecture.md)
- [Numerical backends and MATLAB boundary](docs/backends.md)
- [Validation report](docs/validation.md)
- [Serial/hybrid cable routing and animations](docs/routing_animation.md)
- [Review correction decisions](docs/review_corrections.md)
- [Milestone record index](docs/milestones/README.md)

Repository layout follows the specification: `model`, `kinematics`, `statics`,
`allocation`, `dynamics`, `analysis`, `simulation`, `backends`, `visualization`, `io`,
plus `tests`, `examples`, and `docs`. Shared transform math is in `model/transforms.py`;
reference constructors are in `cablerobot/examples.py` so examples and tests use the
same models. No existing RobotModel code is copied.
