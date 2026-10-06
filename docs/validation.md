# Milestone 1 validation report

This document preserves historical validation evidence. The accepted foundation
and routed-animation milestone, current closure results and Git boundary are
recorded in [milestone_01_foundation.md](milestones/milestone_01_foundation.md).
Historical counts below are not the current full-suite count.
Milestone 2 allocation validation is documented in [tension_allocation.md](tension_allocation.md)
and its temporary review package. This page's earlier results remain historical.

Validated on 2026-09-30. The standalone repository is delivered at
`C:\Users\thisi\OneDrive\Desktop\CableRobot`.

## Result

- Source suite: **67 passed in 2.08 seconds**.
- Installed-wheel suite: **67 passed in 2.02 seconds**, invoked from outside the source tree with importlib test loading. The imported package path was the local virtual environment's `Lib/site-packages/cablerobot/__init__.py`.
- Wheel build and installation succeeded using `pip install --no-deps --no-build-isolation .` in a repository-local virtual environment.
- Both examples executed successfully using the installed package and Matplotlib's Agg backend. Their rendered figures were inspected.
- Environment: Python 3.12.13, NumPy 2.1.0, Matplotlib 3.9.2, pytest 7.4.4. The test environment inherited these existing dependencies; no system packages were changed and no dependency download was needed.
- Baseline execution required no MATLAB engine.

## Example outputs

The eight-cable CDPR has six generalized coordinates and an 8-by-6 cable Jacobian
of rank six. At its reference pose, all cable lengths are approximately 2.298956 m.
Its 2 kg platform has an offset COM, yielding gravity generalized force
`[0, 0, -19.62, 0.3924, 0.5886, 0]` at zero rotation vector.

Gravity-compensation tensions in cable insertion order are approximately
`[1.0000, 11.6772, 5.7611, 6.9160, 1.0000, 11.6772, 1.2506, 11.4266] N`.
The equilibrium residual norm is `9.488510541505395e-15`. Adding a 10,000 N downward
load correctly produces an infeasible bounded-tension result.

The serial example uses two revolute links and four cables: upper/lower base-to-distal,
interlink, and a routed base-to-link-1-to-distal cable. Reference lengths are
`[2.252701, 2.199354, 1.233630, 2.299719] m` at `q=[0.25, -0.5] rad`.
For cable tensions `[5, 3, 2, 4] N`, generalized cable force is approximately
`[-2.892586, -3.656620] N m`.

## Repository tree

```text
CableRobot/
|-- .gitignore
|-- LICENSE
|-- README.md
|-- pyproject.toml
|-- cablerobot/
|   |-- __init__.py
|   |-- examples.py
|   |-- model/
|   |   |-- __init__.py
|   |   |-- body.py, cable.py, frame.py, joint.py
|   |   `-- robot.py, routing.py, state.py, transforms.py
|   |-- kinematics/
|   |   |-- __init__.py
|   |   `-- bodies.py, cables.py, forward.py, inverse.py, jacobians.py
|   |-- statics/
|   |   |-- __init__.py
|   |   `-- equilibrium.py, gravity.py, tension.py
|   |-- allocation/
|   |   `-- __init__.py, tension.py
|   |-- dynamics/
|   |   `-- __init__.py, equations.py, forward.py, inverse.py
|   |-- analysis/
|   |   |-- __init__.py
|   |   `-- manipulability.py, singularity.py, stiffness.py, twist.py, workspace.py, wrench.py
|   |-- simulation/
|   |   `-- __init__.py, simulator.py, state.py
|   |-- backends/
|   |   `-- __init__.py, base.py, matlab_backend.py, numpy_backend.py
|   |-- visualization/
|   |   `-- __init__.py, robot.py, workspace.py, wrench.py
|   `-- io/
|       `-- __init__.py, config.py
|-- tests/
|   |-- conftest.py
|   `-- test_dynamics_contract.py, test_geometry.py, test_solvers_backends_io.py,
|       test_statics.py, test_visualization.py
|-- examples/
|   `-- spatial_cdpr.py, serial_robot.py
`-- docs/
    `-- architecture.md, backends.md, conventions.md, validation.md
```

Git is initialized with branch name `main`. No remote, commit, or hardware connection
is configured. Build caches and the test virtual environment are excluded from the
Desktop source copy.

## Mathematics and public abstractions

`T_A_B` maps B coordinates into A. World is right-handed, units are SI, and default
gravity is `[0, 0, -9.81]`. Generalized coordinates follow joint insertion order;
floating coordinates are `[x, y, z, rx, ry, rz]` with exponential-map rotation vectors.
Wrench/twist order is linear then angular, expressed in world axes about an explicit
selected frame origin. Rotation-coordinate forces are not generally Cartesian moments.

`J_l=dl/dq`, `l_dot=J_l @ qd`, and **`B=-J_l.T`**. Positive cable tension pulls each
segment endpoint toward the other. Virtual work therefore gives
`tau_cable @ qd = -tensions @ l_dot`. Static equilibrium includes applied gravity
and any additional external generalized load. COM offsets are retained.

Core abstractions are CableRobot, Body, Joint, Frame, Attachment, CableRoute, Cable,
RobotState and RobotParameters. The same core supports tree/forest articulated
systems, free bodies through floating joints, and arbitrary cable routes. Versioned
JSON preserves topology, route points, bounds and selected analysis frames.

## Implemented numerical capabilities and tests

Body/frame transforms, cable lengths, generalized Jacobians, selected-body wrenches,
selected-frame twists, gravity, equilibrium residuals and bounded tension allocation
are implemented. Local inverse solvers cover measured lengths, selected-frame position
and full pose. Allocation distinguishes feasible, infeasible, numerically unresolved
and numerical linear-algebra failure. Geometry and allocation are separate.

Physics tests include transform composition, joint-axis conventions, known and routed
lengths, analytic and independent five-point Jacobian checks, virtual work, route
reversal, aggregate body-wrench consistency at nonzero rotations, prismatic and
revolute gravity with offset COM, tension bounds, feasible/overloaded equilibrium,
and deliberately unresolved large allocation. Solvers, serialization, backend
contracts, unavailable/installed-stub MATLAB cases and both visualizers are covered.

Validated forward/inverse dynamics interfaces accept a numerical DynamicsProvider.
A one-axis mass/gravity test checks `M qdd+h=B t+tau_ext`. The built-in general
multibody dynamics implementation remains deferred; the static required-force case
is implemented and default dynamic calls fail explicitly.

Python prepares backend-neutral numerical TensionProblem arrays. NumPy solves them
and returns Python result dataclasses. MATLAB is a lazy optional adapter stub with
explicit unavailable/not-implemented errors. MATLAB array layout, indexing, units,
error handling and reproducibility requirements are documented in `docs/backends.md`.

## Deliberately deferred and design choices

Full multibody dynamics, time integration, closed-loop constraints, sag, cable mass,
pulley mechanics/friction/wrapping/contact, slack, elastic cable dynamics, motor and
flexible-link models, heavy workspace/closure/stiffness/performance algorithms and
hardware communication are deferred. Optional cable stiffness is metadata only.

The suggested subsystem layout is retained, with centralized transform helpers and
reusable example constructors. Reference Jacobians use centralized finite differences
rather than specialized geometry paths. The dependency baseline has no SciPy.
Small bounded allocation exhaustively searches active sets for at most eight cables;
larger problems use projected least squares with explicit convergence uncertainty.
This is a bounded-equilibrium/reference numerical allocator: "reference" denotes
a validation baseline, not a configurable `t_ref` objective. Reference/pretension
tracking is not a foundation capability.
Kinematic solves are local and require reasonable initial guesses. The workspace
entry point currently reports one configuration, not a full workspace grid.

Detailed conventions and these limits are documented before any next milestone.

## Serial/hybrid routing animation extension

The updated repository passes **87 tests in 5.64 seconds** directly from its Desktop
location. It now includes a three-link serial example and a floating-platform/arm
hybrid, each with outer-frame routes through moving link guides and routes entirely
on the moving robot. Tests verify shared-motion length invariance, articulation
response, intermediate guide forces and full rendered polylines. Periodic prescribed
kinematic trajectories support five-second GIF and MP4 animations for all three
topologies and a synchronized comparison. Full tension-driven dynamic simulation
remains deferred. See `routing_animation.md` for the model details and reproduction.
