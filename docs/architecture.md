# Architecture and first milestone

Review status: **IMPLEMENTED — AWAITING REVIEW**. See
[review correction decisions](review_corrections.md) for the rationale and
compatibility consequences of the current hardening, recorded before its code
changes. Model topology, state ordering and force signs are unchanged.

`CableRobot` owns bodies, joints, frames, routes, physical parameters and coordinate
ordering. Topology is a directed tree or forest with one parent per non-root body.
It is validated before body kinematics. Closed loops need future constraint equations;
hybrid systems with a tree of rigid bodies and arbitrary crossing cables are supported.

Deterministic maps share one geometry implementation:

```text
RobotState.q -> body/frame transforms -> world route points -> cable lengths
                                                     -> numerical cable Jacobian
                                                         -> B = -J_l.T
```

Nonlinear solvers consume these maps. Statics uses COM point Jacobians and B.
The allocator consumes numerical arrays only and does not own any geometry.
Python converts an analysis request to `TensionProblem`, a selected backend computes
a `TensionAllocationResult`, and Python presents or plots the result.

## Implemented abstractions

`Body` stores mass, local COM, inertia and a display scale. `Joint` stores parent,
child, axis, two constant transforms and a coordinate block. `Frame` stores a body
and local rigid transform. `Attachment` stores a frame-local point. `CableRoute`
holds at least two attachments. `Cable` stores its route, unilateral bounds and an
optional stiffness value reserved for later elastic mechanics. `RobotParameters`
holds gravity and numerical step size. `RobotState` holds q and qd.

Serialization uses standard-library JSON with `schema_version=1`. Arrays are lists,
joint order is preserved, and infinite upper tension is the string `"inf"` to avoid
nonstandard JSON. Selected analysis frames are part of the model. Model description
files contain no executable code. Backend data never contains MATLAB objects.

## Dynamics and analysis boundaries

Future full multibody dynamics follows
`M(q) @ qdd + h(q, qd) = B(q) @ tensions + tau_ext`.
Here h includes gravity as its negative applied force along with velocity-dependent
terms. `static_generalized_force_required = -tau_gravity` is implemented.
`DynamicsProvider` supplies numerical mass and bias terms to validated forward and
inverse interfaces. With a provider, `inverse_dynamics` returns `M @ qdd + h` and
`forward_dynamics` solves the mass-matrix equation. A one-axis mass/gravity reference
test validates the equations. Built-in general multibody mass matrices and velocity
bias are deferred; default calls raise explicit deferred-feature errors. Allocation
remains a separate step, and dynamic integration is deferred.

Available generalized-force vertices are implemented for bounded tensions and a
small cable count. Selected-body wrench and selected-frame twist Jacobians support
arbitrary frames. Singularity rank and basic generalized manipulability are small
reference diagnostics. Full velocity polytopes, wrench closure/feasible workspace
grids, Cartesian/generalized stiffness and dynamic-performance algorithms remain
planned. `analyze_wrench_workspace` is a pointwise reference interface, not a complete
workspace grid algorithm.

## Choices relative to the bootstrap specification

The suggested subsystem layout is retained. `model/transforms.py` centralizes SE(3)
utilities and `cablerobot/examples.py` holds reusable model constructors. Analytic
joint Jacobians and automatic differentiation are deferred in favor of a single
topology-independent numerical reference, validated against physical invariants.
No SciPy dependency is introduced. Exhaustive active sets are capped at eight cables
to keep the reference allocator small and predictable; larger cases explicitly
expose convergence uncertainty through result status.

The next milestone should be chosen after reviewing the public model, signs, tests,
allocation behavior and optional backend boundary. Hardware communication is outside
this foundation.

## Routing animation extension

`simulation/kinematic.py` now supplies sampled periodic q/qd trajectories without
integrating dynamics. `visualization/animation.py` renders the common route geometry
for one or multiple synchronized robots. Dedicated three-link serial and hybrid
constructors demonstrate both fixed-frame-to-robot routing and routing entirely on
moving links. The hybrid carries that arm on a floating eight-cable platform.
See `routing_animation.md` for tested invariants and the simulation boundary.
