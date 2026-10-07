# Architecture and milestone boundaries

Review status: **COMPLETED — ACCEPTED FOUNDATION**. See the
[accepted milestone record](milestones/milestone_01_foundation.md) and
[review correction decisions](review_corrections.md) for the rationale and
compatibility consequences of the current hardening, recorded before its code
changes. Model topology, state ordering and force signs are unchanged.

Milestone 2 is **COMPLETED — ACCEPTED** on 2026-10-07, with no required code amendment.
See its [permanent record](milestones/milestone_02_tension_allocation.md).
Its additive allocation API and
numerical decisions were [documented before implementation](tension_allocation.md).
It reuses the foundation geometry, Jacobians, B and gravity without modification.
That statement describes its original reviewed revision `d5b733d`. The subsequently
authorized [computational study](performance_study.md), also accepted on 2026-10-07,
preserves model semantics but makes analytic derivatives the default and adds
validated call-local geometry reuse. Its [measured results](performance_results.md)
remain a distinct computational validation, not hardware acceptance. Both are
closed at the same integrated Git boundary; see the
[study record](milestones/performance_study_01_pipeline.md).

`CableRobot` owns bodies, joints, frames, routes, physical parameters and coordinate
ordering. Topology is a directed tree or forest with one parent per non-root body.
It is validated before body kinematics. Closed loops need future constraint equations;
hybrid systems with a tree of rigid bodies and arbitrary crossing cables are supported.

Deterministic maps share one geometry implementation:

```text
RobotState.q -> body/frame transforms -> world route points -> cable lengths
                                                     -> cable Jacobian
                                                         -> B = -J_l.T
```

Cable/point Jacobians default to analytic derivatives; centralized finite differences
remain selectable references. Nonlinear solvers consume these maps. Statics uses
COM point Jacobians and B.
The allocator consumes numerical arrays only and does not own any geometry.
It is a bounded static-equilibrium/reference numerical implementation. Small
problems enumerate active sets and compare residual norm, then tension norm,
using the existing internal criteria; larger problems use projected least squares.
That foundation allocator has no user-configurable reference-tension objective.
It remains available unchanged for regression and validation.
Python converts an analysis request to `TensionProblem`, a selected backend computes
a `TensionAllocationResult`, and Python presents or plots the result.

Milestone 2 adds `allocation/reference.py`: a strictly convex reference-tension QP
on numerical arrays, and a thin robot-facing wrapper that forms the full target.
It returns `EquilibriumTensionResult`, an additive richer result type. The NumPy
dual solver uses free-column SVD and directional searches; the old allocator supplies
fallback witnesses/separating certificates, never unverified secondary optima.
Existing `AnalysisBackend`/`TensionProblem` contracts remain unchanged. The new API
identifies its NumPy solver explicitly; no MATLAB QP or silently selected backend
is implied. No new dependency, topology change, hardware loop, dynamics, or
continuity regularizer is introduced.

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
utilities and `cablerobot/examples.py` holds reusable model constructors. The
foundation initially used a single topology-independent numerical reference rather
than analytic derivatives or automatic differentiation, validating physical invariants.
The later performance study retains this centralized finite-difference path as an
explicit reference while adding generic analytic cable/point derivatives. No SciPy
dependency is introduced. Exhaustive active sets are capped at eight cables
to keep the reference allocator small and predictable; larger cases explicitly
expose convergence uncertainty through result status.

Milestone 2 and the first computational study are accepted. The explicitly
authorized [best-effort allocation / FK warm-start study](best_effort_allocation.md)
is IMPLEMENTED — AWAITING REVIEW, not an accepted next milestone. Separate
`allocation/best_effort.py` and `allocation/scaling.py` own additive result/status
and numerical scaling contracts. The robot wrapper forms canonical B/gravity/bounds,
uses the accepted exact/reference QP whenever equilibrium may exist, and otherwise
performs certified residual minimization followed by reference selection at the
unique force image. A bounded feasible witness avoids unnecessary primary screening;
original-space separation permits skipping a known-impossible exact QP. Neither
screen candidate is itself a command. Projected LS and small-system exhaustive
fallback are reused, not reimplemented. Old allocators, backend contracts, model,
derivatives and FK remain unchanged. FK prediction lives only in a developer
benchmark; no new production tracking API/cache is introduced. Hardware
communication remains outside the current implementation.

## Routing animation extension

`simulation/kinematic.py` now supplies sampled periodic q/qd trajectories without
integrating dynamics. `visualization/animation.py` renders the common route geometry
for one or multiple synchronized robots. Dedicated three-link serial and hybrid
constructors demonstrate both fixed-frame-to-robot routing and routing entirely on
moving links. The hybrid carries that arm on a floating eight-cable platform.
See `routing_animation.md` for tested invariants and the simulation boundary.
