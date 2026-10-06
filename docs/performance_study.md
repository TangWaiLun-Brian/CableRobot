# FK + equilibrium pipeline performance study

Scope authorized on 2026-10-06: computational study of the implemented pipeline,
not a new physical-model milestone or acceptance of Milestone 2. Starting revision
`d5b733d204bf382db68f44415469ed7f01ff962d` is implemented, awaiting review. Its
existing `.review/` and exported review ZIP will be preserved. This study's separate
handoff will be `.review/performance_study/` to avoid overwriting pending review.

## Plan recorded before optimization

First add a reproducible benchmark with high-resolution, unprofiled frame timings
and separate cProfile evidence. Compare the same interpreter, machine, workloads,
thread settings, tolerances and warm-start policy before/after. Record all statuses,
pose/length error, equilibrium residual, objective, timing tails and evaluation counts.
No timing thresholds in correctness tests, and no weakening solver tolerances.

Workload A knows q and only calls equilibrium allocation. Workload B generates
synthetic lengths, solves local FK using the last successful state, then allocates.
Report both virtual-sensor-inclusive latency and measured-length->FK->tension latency;
the latter excludes synthetic length generation, NOT actual encoder overhead.
The first FK guess is the first true state, matching the provided simulation sample;
this does not benchmark global recovery or cold hardware startup.

Use the sample's 0.2 kg platform, unequal [0.15,0.10,0.08] m attachment half-extents,
1 m anchor half-extents, 1–100 N bounds, reference 10 N, 0.3 m circular translation
with z from -0.3 to 0.3 at 50 Hz/10 s (500 frames). The original Downloads script is
reference data only: do not execute its plotting/main block, modify it or add hardware.
Also reuse spatial_cdpr() with offset COM and a small continuous translated/mildly
rotated 500-frame trajectory. Component snapshots include static, small translation
and translated+rotated states; check feasibility/rank rather than hiding singularity.

Measure lengths, Jacobian, gravity, pure numerical reference QP (precomputed arrays),
robot-facing allocation, FK and complete pipelines. Profiled cumulative FK/Jacobian/
allocation times overlap hierarchically; do not add them as disjoint fractions.
Count FK length/Jacobian function evaluations using a separate profile pass, keeping
instrumentation out of headline timings. Effective Hz means reciprocal mean software
computation latency, not a paced or hard-real-time control-rate measurement.

Only after evidence identifies a dominant cost, document its mathematical/API change
before implementation. If finite differences/repeated geometry dominate, retain a
numerical reference and introduce generic analytic derivatives for supported joints
and straight body-fixed routed segments, validated at translated/rotated/random
spatial, serial, hybrid and offset-joint configurations. Preserve J_l=dl/dq,
B=-J_l.T, COM gravity, complete equilibrium and both tension solver algorithms.
Prefer call-local transform reuse over persistent caches on mutable models. Avoid
changing languages, backends, optimization objectives or physical models.

Target: 20 ms/frame (50 Hz) with practical margin for communication/filtering/safety/
logging/OS jitter. Report mean/median/p95/max, overruns and limitations honestly;
ordinary Python tests do not establish hard-real-time behavior. MATLAB stays a stub,
not a per-frame migration. Final targeted/full tests and actual benchmark evidence
must precede an IMPLEMENTED — AWAITING REVIEW handoff, with selected canonical files.
Current status: **IMPLEMENTED — AWAITING REVIEW**. See `performance_results.md` for
actual before/after timings, correctness comparisons, profiler counts and limitations.

## Decision recorded after baseline profiling, before core changes

Practical sample (500 frames, NumPy threads explicitly 1): mode A averages
43.70935 ms; measured-length->FK->tension averages 139.51636 ms (p95 148.74953 ms).
All 500 frames are feasible/converged. Separate 30-frame cProfile cumulative shares:
FK 66.28%, Jacobian construction 85.22%, allocation wrapper 31.66%, pure QP 0.68%.
These are nested, NOT additive. This confirms numerical Jacobians/repeated geometry
as the dominant path, not the reference-tension optimizer or Python language itself.

Previous design: each cable endpoint recomputes/validates all body transforms; cable
Jacobians call lengths 2*dof+1 times, and COM derivatives are also finite differences.
Local FK separately evaluates residual/Jacobian and repeats accepted trial geometry.

New design: keep the body-transform equation and model ownership. A private call-local
kinematic evaluation reuses all body/frame transforms and propagates body-origin
linear/angular derivatives through the tree. A point derivative is
`J_point = J_origin - skew(p_point-p_origin) J_angular`. Roots have zero derivatives.
For a revolute joint, the world joint axis supplies angular derivative and its cross
product with the lever arm supplies linear derivative; a prismatic joint supplies
the world joint axis in translation. Fixed joints inherit transported parent motion.
Floating translation columns are the pre-motion joint rotation. Floating angular
columns are `R_world_prejoint * J_left(phi)`, NOT identity or coordinate rates
interpreted as angular velocity. Child post-motion offsets use the moving joint
origin, including floating translation, as the angular lever origin.

For K=skew(phi), theta=||phi||:
`J_left(phi) = I + (1-cos(theta))/theta^2 K + (theta-sin(theta))/theta^3 K^2`.
Use series coefficients near zero to avoid cancellation. This differentiates the
existing `R=Exp(phi)` convention; it changes neither q nor physical force signs.

For each routed segment d=p_next-p_previous, unit u=d/||d||:
`d length/dq = u.T (J_next-J_previous)`; sum all segment contributions, including
body-fixed intermediate guides. Equal segment tensions still imply `B=-J_l.T`.
Zero-length segments retain the existing undefined-Jacobian rejection. Support all
existing fixed/revolute/prismatic/floating tree/forest models; do not claim closed-loop,
contact/pulley/tangency support. Gravity reuses the same COM point derivatives.

Default cable/point Jacobians become analytic. Retain the central-difference path:
explicit `step` still selects it; add keyword method='finite_difference' or 'analytic'.
The CableRobot cable-Jacobian convenience method gets an additive method selector.
FK gets an additive jacobian_method selector, preserving warm starts, damping,
backtracking, residual/iteration tolerances and statuses. A per-FK-call geometry
cache may reuse combined lengths/Jacobian at identical q (including accepted trials),
but no persistent cache or skipped validation across public calls on mutable models.
Gravity gets the same explicit reference selector. Neither allocator algorithm,
objective, status/certificate contract, backend nor JSON/model schema changes.

Alternatives: only accelerate one CDPR's wrench map would violate generic routing;
larger finite-difference steps/looser FK tolerances would trade correctness for speed;
persistent model caches risk stale topology/attachment/state data; rewriting in
MATLAB adds boundary overhead and lacks bottleneck evidence. No such changes.

Compatibility: existing valid calls/shapes/units/signs remain; default floating-point
values improve from finite-difference approximations, so exact historical bit patterns
are not promised. Users explicitly supplying a step retain numerical differentiation.
Explicit analytic method plus a finite-difference step is invalid, rather than ignored.
New method selectors and all supported-tree analytic derivatives require external
review before acceptance. No hard-real-time guarantee follows from this change.

Tests required: many seeded spatial/serial/hybrid/offset-joint analytic-vs-independent
five-point derivatives, nonzero/near-zero/near-pi rotvec, rotated joint frames/postoffsets,
multiple roots, arbitrary/reversed/on-robot routes, virtual work, gravity/reference
equivalence, FK recovery/singularity/failure and reference-objective/status parity.
Keep every existing test and tolerance, and reprofile after the change before doing
any further optimization.

## Implemented compatibility and execution notes

The selected FK optimization memoizes only the length residual at the last q within
one solve. It preserves existing public cable-length/Jacobian callbacks; no fused
private FK geometry path or persistent cache was necessary to meet the numerical
budget. Analytic Jacobian construction naturally computes segment lengths along
with derivatives; profiler counts include those coupled evaluations rather than
misrepresenting a reduced number of public calls as all geometry computations.

The sample's Downloads file remains unchanged (SHA256
`DE6CFA36C9703BF7A8FB8575AE435DF11D76B7DB6FEAB49B17A7D79A4DEB2B09`).
It was inspected, not executed or copied wholesale. The benchmark adapts its geometry,
sampling and previous-success warm-start pattern. No sample workflow changes are
needed to benefit from the optimized package.

The base Anaconda interpreter could not import cablerobot when checked from Downloads
(`ModuleNotFoundError`), although source-root benchmark imports work. Running an
external script requires an installation in the SAME interpreter/environment used to
execute it. For example, from the repository root in the intended environment:

```powershell
python -m pip install -e . --no-deps --no-build-isolation
```

This assumes the declared build/runtime dependencies are already installed. No global
installation or system dependency modification was performed by the study. Dedicated
installed-wheel verification is separate from the user's execution environment.
